#!/usr/bin/env python3
"""Print what the speed governor decides: each change between free, slow and stop, and felt or not per hazard.

    python3 navigation/governor_watch.py                 # on the robot network
    python3 navigation/governor_watch.py --ros-args -p use_sim_time:=true    # with a replayed bag
"""
import json
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Watch(Node):
    def __init__(self):
        super().__init__("governor_watch")
        self.mode = None
        self.create_subscription(String, "/governor/state", self.on_state, 10)
        self.create_subscription(String, "/governor/felt", self.on_felt, 10)
        print("waiting for /governor/state ...", flush=True)

    def stamp(self):
        return f"{self.get_clock().now().nanoseconds * 1e-9 % 1000:7.1f} s"

    def on_state(self, msg):
        d = json.loads(msg.data)
        if d["mode"] != self.mode:
            self.mode = d["mode"]
            asked = "" if d["command"] is None else f"asked {d['command']:.2f}, "
            print(f"{self.stamp()}  {d['mode'].upper():5s} {asked}sent {d['out']:.2f} m/s  {d['reason']}", flush=True)

    def on_felt(self, msg):
        d = json.loads(msg.data)
        verdict = "FELT, class C" if d["felt"] else "not felt, class A"
        print(f"{self.stamp()}  {d['id']} {d['type']} {d['height_mm']} mm: jolt {d['imu_peak']} m/s^2 -> {verdict}", flush=True)


def main():
    rclpy.init()
    node = Watch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
