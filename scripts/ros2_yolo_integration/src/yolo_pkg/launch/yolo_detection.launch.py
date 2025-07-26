# src/yolo_pkg/launch/yolo_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_pkg',
            executable='yolo_detection_node',
            name='yolo_detection',
            output='screen',
        )
    ])
