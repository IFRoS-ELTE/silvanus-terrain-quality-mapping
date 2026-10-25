from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('crack_detection'))
    defaults = {
        'use_sim_time': ('false', bool),
        'image_topic': ('/camera/camera/color/image_raw', str),
        'model_path': (str(share / 'models/crack_unet16_v1.onnx'), str),
        'config_path': ('', str),
        'color_robot_mask': (str(share / 'models/robot_mask_color.png'), str),
        'robot_mask': (str(share / 'models/robot_mask_depth.png'), str),
        'publish_cloud': ('true', bool),
        'publish_markers': ('true', bool),
        'target_frame': ('', str),
        'max_rate_hz': ('5.0', float),
        'threshold': ('-1.0', float),
        'min_area': ('150.0', float),
        'min_length': ('1', int),
        'num_threads': ('2', int),
    }
    arguments = [DeclareLaunchArgument(name, default_value=default)
                 for name, (default, _) in defaults.items()]
    arguments.append(DeclareLaunchArgument('run_rviz', default_value='false'))
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, (_, kind) in defaults.items()}
    return LaunchDescription(arguments + [
        Node(
            package='crack_detection', executable='crack_bbox_node',
            name='crack_bbox_node', output='screen', parameters=[parameters],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('run_rviz')),
            arguments=['-d', str(share / 'rviz/crack_detection.rviz')],
            parameters=[{'use_sim_time': parameters['use_sim_time']}],
        ),
    ])
