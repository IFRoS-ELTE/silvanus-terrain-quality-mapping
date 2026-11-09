from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'use_sim_time': ('false', bool),
        'accel_topic': ('/filter/free_acceleration', str),
        'odom_topic': ('/odom', str),
        'rate_hz': ('4.0', float),
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, (default, _) in args.items()]
        + [Node(
            package='terrain_imu', executable='imu_roughness_node', name='imu_roughness_node', output='screen',
            parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                         for name, (_, kind) in args.items()}],
        )]
    )
