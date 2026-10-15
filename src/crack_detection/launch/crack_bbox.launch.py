from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crack_detection',
            executable='crack_bbox_node',
            name='crack_bbox_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/color/image_raw',
                'debug_topic': '/crack_detection/debug_image',
                'detections_topic': '/crack_detection/detections',
                'dark_threshold': 80,
                'min_area': 120.0,
                'max_area': 12000.0,
                'min_width': 12,
                'min_height': 4,
                'roi_y_min_ratio': 0.30,
                'edge_margin_ratio': 0.04,
                'morph_kernel_size': 5,
            }],
        ),
    ])
