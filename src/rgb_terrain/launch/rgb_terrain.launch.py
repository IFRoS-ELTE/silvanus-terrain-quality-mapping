from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'use_sim_time': ('false', bool),
        'image_topic': ('/camera/camera/color/image_raw', str),
        'rate_hz': ('2.0', float),
        'min_confidence': ('0.5', float),
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, (default, _) in args.items()]
        + [Node(
            package='rgb_terrain', executable='terrain_node', name='rgb_terrain_node', output='screen',
            parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                         for name, (_, kind) in args.items()}],
        )]
    )
