#!/usr/bin/env python3
"""Move the xArm to the camera pose (Pose B, docs/robot/arm_pose.md) through MoveIt, and check it got there.

Runs on the robot, inside the container, next to move_group (over Wi-Fi the laptop often cannot see it):

    sudo docker cp arm_to_pose.py silvanus:/tmp/arm_to_pose.py
    sudo docker exec -it silvanus bash -c "source /opt/ros/jazzy/setup.bash && source /opt/robot/setup.bash && python3 /tmp/arm_to_pose.py"

1. reads the arm joints from /joint_states and asks move_group whether that state touches the robot model.
   If it does (2026-10-27: the folded arm had the gripper 2-3 mm into link1/link_base in the model, and
   move_group refuses to plan from a colliding state), it looks for the smallest move of up to 20 deg on
   joint5/joint3/joint2 that ends clear, with the overlap shrinking at every step. That move goes straight
   to the arm's trajectory controller at 1 deg/s.
2. asks move_group for a plan to Pose B at 5 % speed. move_group checks the whole path against the
   chassis, payload box, lidar and gripper models, the same check as Plan in RViz
3. prints each step and moves only after you type y (Ctrl-C stops the arm)
4. checks every joint against Pose B, and the camera against where it was in Pose B recordings

--plan-only works out both steps and prints them without moving.
"""

import argparse
import itertools
import math
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"joint{i}" for i in range(1, 7)]
POSE_B = [-0.002082, 0.816467, -2.330569, -0.007465, 1.580136, 0.000002]   # docs/robot/arm_pose.md
LINK6_B = (0.870, 0.000, 0.870)        # base_footprint -> link6 in Pose B, move_group's model on 2026-10-27
CAMERA_B = (0.938, 0.015, 0.846)       # base_footprint -> camera_depth_optical_frame, tf2_echo 2026-10-01 and
                                       # again 2026-10-27 (the 2026-10-08 bags have it 6 cm lower)
ERRORS = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}
ESCAPE_JOINTS = [[4], [2], [1], [2, 4], [1, 4], [1, 2]]          # wrist first: smallest swing
ESCAPE_DEG = [-20, -15, -10, -5, 5, 10, 15, 20]


def deg(a):
    return "  ".join(f"{math.degrees(v):7.1f}" for v in a)


def escape_candidates():
    """Joint offsets (rad), smallest first: by largest single change, then total change."""
    out = []
    for joints in ESCAPE_JOINTS:
        for mags in itertools.product(ESCAPE_DEG, repeat=len(joints)):
            d = np.zeros(6)
            d[joints] = np.radians(mags)
            out.append((max(map(abs, mags)), sum(map(abs, mags)), d))
    out.sort(key=lambda c: (c[0], c[1]))
    return [c[2] for c in out]


class Arm(Node):
    def __init__(self, group):
        super().__init__("arm_to_pose")
        self.group, self.js = group, None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.plan_client = ActionClient(self, MoveGroup, "/move_action")
        self.exec_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.direct = ActionClient(self, FollowJointTrajectory, "/xarm6_traj_controller/follow_joint_trajectory")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.tf = Buffer()
        TransformListener(self.tf, self)

    def on_joints(self, m):
        if all(j in m.name for j in JOINTS):          # the base publishes wheel joints on the same topic
            self.js = np.array([m.position[m.name.index(j)] for j in JOINTS])

    def wait_for(self, cond, timeout, what):
        t0 = time.time()
        while not cond():
            if time.time() - t0 > timeout:
                sys.exit(f"No {what} after {timeout:.0f} s. Is the arm stack (--arm) running?")
            rclpy.spin_once(self, timeout_sec=0.1)

    def done(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            sys.exit("move_group did not answer in time")
        return future.result()

    def settle(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def allow_start_tolerance(self):
        """Without this move_group refuses every move after the first (docs/robot/robot_session_guide.md)."""
        cli = self.create_client(SetParameters, "/move_group/set_parameters")
        if not cli.wait_for_service(timeout_sec=3.0):
            return print("  (could not set allowed_start_tolerance; a second move may be refused)")
        req = SetParameters.Request(parameters=[
            Parameter("trajectory_execution.allowed_start_tolerance", value=0.1).to_parameter_msg()])
        self.done(cli.call_async(req), 5.0)

    def state(self, q):
        return RobotState(joint_state=JointState(name=JOINTS, position=[float(v) for v in q]))

    def contacts(self, q):
        """(clear, summed overlap in m, touching pairs) for these joints in move_group's model."""
        res = self.done(self.validity.call_async(GetStateValidity.Request(robot_state=self.state(q),
                                                                          group_name=self.group)), 5.0)
        pairs = sorted({f"{c.contact_body_1}/{c.contact_body_2}" for c in res.contacts})
        return res.valid, sum(c.depth for c in res.contacts), pairs

    def find_escape(self, q):
        """Smallest offset that ends clear, with the overlap never growing on the way."""
        for d in escape_candidates():
            if not self.contacts(q + d)[0]:
                continue
            depth = [self.contacts(q + s * d)[1] for s in np.linspace(0, 0.9, 10)]
            if all(b <= a + 1e-4 for a, b in zip(depth, depth[1:])):
                return d, depth
        return None, None

    def flange(self, positions):
        """link6 in base_footprint for these joint values, from move_group's own model."""
        if not self.fk.service_is_ready():
            return None
        req = GetPositionFK.Request(fk_link_names=["link6"], robot_state=self.state(positions))
        req.header.frame_id = "base_footprint"
        res = self.done(self.fk.call_async(req), 5.0)
        if res.error_code.val != MoveItErrorCodes.SUCCESS or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def plan(self, target, speed, start=None):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = self.group
        r.num_planning_attempts = 5
        r.allowed_planning_time = 10.0
        r.max_velocity_scaling_factor = speed
        r.max_acceleration_scaling_factor = speed
        if start is None:
            r.start_state.is_diff = True
        else:
            r.start_state = self.state(start)
        r.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(p), tolerance_above=0.005, tolerance_below=0.005, weight=1.0)
            for j, p in zip(JOINTS, target)])]
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = self.done(self.plan_client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            sys.exit("move_group refused the planning request")
        res = self.done(handle.get_result_async(), 60.0).result
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            sys.exit(f"No safe plan: {ERRORS.get(res.error_code.val, res.error_code.val)}. Nothing moved.")
        return res.planned_trajectory

    def follow(self, handle, target, seconds):
        """Wait for a running goal, printing progress; Ctrl-C cancels it, which stops the arm."""
        result = handle.get_result_async()
        t0, shown = time.time(), 0.0
        try:
            while not result.done():
                rclpy.spin_once(self, timeout_sec=0.2)
                if time.time() - shown > 2.0 and self.js is not None:
                    shown = time.time()
                    print(f"  moving, {time.time() - t0:4.0f} of {seconds:.0f} s: furthest joint still "
                          f"{np.degrees(np.max(np.abs(self.js - target))):5.1f} deg from the target")
        except KeyboardInterrupt:
            print("\nStopping the arm ...")
            self.done(handle.cancel_goal_async(), 5.0)
            sys.exit("Stopped. The arm is somewhere between the two poses.")
        return result.result().result

    def move_direct(self, target, seconds):
        """Small move straight to the arm's controller, used only to get clear of a contact."""
        start = self.js.copy()
        jt = JointTrajectory(joint_names=JOINTS)
        for k, s in enumerate(np.linspace(0, 1, 11)[1:], 1):
            t = seconds * k / 10
            jt.points.append(JointTrajectoryPoint(positions=[float(v) for v in start + s * (target - start)],
                                                  time_from_start=Duration(sec=int(t), nanosec=int(t % 1 * 1e9))))
        if not self.direct.wait_for_server(timeout_sec=5.0):
            sys.exit("No /xarm6_traj_controller/follow_joint_trajectory. Nothing moved.")
        handle = self.done(self.direct.send_goal_async(FollowJointTrajectory.Goal(trajectory=jt)), 10.0)
        if not handle.accepted:
            sys.exit("The arm controller refused the move. Nothing moved.")
        res = self.follow(handle, target, seconds)
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            sys.exit(f"The arm controller stopped with error {res.error_code}: {res.error_string}")

    def execute(self, trajectory, target, seconds):
        handle = self.done(self.exec_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)), 10.0)
        if not handle.accepted:
            sys.exit("move_group refused to execute. Nothing moved.")
        res = self.follow(handle, target, seconds)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            sys.exit(f"Execution ended with {ERRORS.get(res.error_code.val, res.error_code.val)}")


def ask(question):
    if input(f"\n{question} [y/N] ").strip().lower() != "y":
        sys.exit("Not moved.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--speed", type=float, default=0.05, help="velocity and acceleration scaling (default 0.05)")
    ap.add_argument("--group", default="xarm6")
    ap.add_argument("--plan-only", action="store_true")
    a = ap.parse_args()
    target = np.array(POSE_B)

    rclpy.init()
    arm = Arm(a.group)
    print("Waiting for the arm joints and move_group ...")
    arm.wait_for(lambda: arm.js is not None, 15, "/joint_states with joint1-joint6")
    if not arm.plan_client.wait_for_server(timeout_sec=15.0):
        sys.exit("No /move_action. move_group is not running.")
    arm.exec_client.wait_for_server(timeout_sec=5.0)
    arm.fk.wait_for_service(timeout_sec=5.0)
    if not arm.validity.wait_for_service(timeout_sec=5.0):
        sys.exit("No /check_state_validity from move_group.")
    arm.allow_start_tolerance()

    print(f"\n          {'  '.join(f'{j:>7s}' for j in JOINTS)}")
    print(f"now       {deg(arm.js)}")
    print(f"Pose B    {deg(target)}   (degrees)")
    if np.max(np.abs(arm.js - target)) < math.radians(1.0):
        print("\nThe arm is already in Pose B.")
    else:
        start = arm.js.copy()
        clear, overlap, pairs = arm.contacts(start)
        escape = None
        if not clear:
            print(f"\nIn the planner's model the arm touches the robot now ({overlap * 1000:.0f} mm overlap summed over "
                  f"{len(pairs)} pairs: {', '.join(pairs)}). It will not plan from there.")
            d, depth = arm.find_escape(start)
            if d is None:
                sys.exit("No small move (up to 20 deg) gets it clear. Nothing moved; move the arm in RViz.")
            escape = start + d
            moved = [f"{j} {math.degrees(v):+.0f} deg" for j, v in zip(JOINTS, d) if v]
            esc_s = max(5.0, math.degrees(np.max(np.abs(d))))
            print(f"Step 1: {', '.join(moved)} in {esc_s:.0f} s, straight to the arm controller. The overlap shrinks at "
                  f"every step ({' > '.join(f'{x * 1000:.0f}' for x in depth[::3])} mm) and is gone at the end.")
        planned = arm.plan(target, a.speed, start=escape)
        traj = planned.joint_trajectory
        order = [traj.joint_names.index(j) for j in JOINTS]
        path = np.array([[pt.positions[i] for i in order] for pt in traj.points])
        end = traj.points[-1].time_from_start
        move_s = end.sec + end.nanosec * 1e-9
        print(f"{'Step 2: ' if escape is not None else ''}plan to Pose B, collision-checked by move_group, "
              f"{len(path)} waypoints, {move_s:.0f} s at {a.speed * 100:.0f} % speed")
        if np.degrees(np.max(np.abs(path[-1] - target))) > 1.0:
            sys.exit(f"  The plan ends at {deg(path[-1])}, not at Pose B. Not moving.")
        heights = [f for f in (arm.flange(q) for q in path[:: max(1, len(path) // 15)]) if f is not None]
        last = arm.flange(path[-1])
        if last is not None:              # information: the model's flange against the Pose B recording
            print(f"  arm flange ends at ({last[0]:.2f}, {last[1]:.2f}, {last[2]:.2f}) m; in the Pose B recording "
                  f"it was ({LINK6_B[0]:.2f}, {LINK6_B[1]:.2f}, {LINK6_B[2]:.2f}): "
                  f"{np.linalg.norm(last - LINK6_B) * 100:.0f} cm apart")
        if heights:
            print(f"  lowest flange height on the way: {min(f[2] for f in heights):.2f} m above the floor")
        if a.plan_only:
            return
        if escape is not None:
            ask("Hand on the arm emergency stop. Step 1, move the wrist clear now?")
            arm.move_direct(escape, esc_s)
            arm.settle(1.0)
            if not arm.contacts(arm.js)[0]:
                sys.exit("Still touching in the model after step 1. Stopped here.")
            planned = arm.plan(target, a.speed)              # from where the arm really is now
        ask("Hand on the arm emergency stop. Move the arm to Pose B now?")
        arm.execute(planned, target, move_s)

    # check: joints, then the camera itself
    arm.settle(1.5)
    err = np.degrees(np.abs(arm.js - target))
    print(f"\nnow       {deg(arm.js)}")
    print(f"error     {'  '.join(f'{e:7.1f}' for e in err)}   (degrees)")
    ok = err.max() < 1.0
    try:
        p = arm.tf.lookup_transform("base_footprint", "camera_depth_optical_frame", Time()).transform.translation
        cam_off = math.dist((p.x, p.y, p.z), CAMERA_B)
        print(f"camera    {p.x:.2f} m ahead, {p.y:+.2f} m left, {p.z:.2f} m up; Pose B recording: "
              f"{CAMERA_B[0]:.2f}, {CAMERA_B[1]:+.2f}, {CAMERA_B[2]:.2f} ({cam_off * 100:.0f} cm apart)")
        if cam_off > 0.05:
            print("          more than 5 cm: the joints match, so the robot model or camera mount has changed")
    except TransformException:
        print("camera    no transform base_footprint -> camera_depth_optical_frame; judged on the joints only")
    print("\nOK: the arm is in Pose B." if ok else "\nNOT in Pose B: see the errors above.")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
