from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_webcam',
            namespace='vigitia',
            executable='simple_webcam_node',
            name='webcam',
        ),
    ])
