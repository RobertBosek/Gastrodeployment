from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

DEBUG_MODE = True

# TODO: launchfile in src oder in einem package
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logitech_brio_node',
            namespace='vigitia',
            executable='logitech_brio_node',
            name='camera_frames',
            # remappings=[
            #     ('/vigitia/brio_rgb_full', '/vigitia/brio_rgb_full_testlaunch'),
            # ]
            parameters=[
                {'DEBUG_MODE': DEBUG_MODE,
                 'FLIP_IMAGE': False}
            ],
            # parameters=[
            #     {'window_origin.x': 3002}
            # ],
        ),
#        Node(
#            package='simple_webcam',
#            namespace='vigitia',
#            executable='simple_webcam_node',
#            name='camera_frames',
#            remappings=[
#                ('/vigitia/brio_rgb_full', '/vigitia/brio_rgb_full_testlaunch'),
#            ]
#            # parameters=[
#            #     {'window_origin.x': 3002}
#            # ],
#        ),
        Node(
            package='fiducials_detector_node',
            namespace='vigitia',
            executable='fiducials_detector_node',
            name='aruco_detector',
            parameters=[
                {'DEBUG_MODE': DEBUG_MODE}
            ],
        ),
        Node(
            package='cube_tracker_node',
            namespace='vigitia',
            executable='cube_tracker_node',
            name='cube_tracker',
            parameters=[
                {'DEBUG_MODE': DEBUG_MODE}
            ],
        ),
        Node(
            package='surface_extraction_node',
            namespace='vigitia',
            executable='surface_extraction_node',
            name='table_frames',
            parameters=[
                {'DEBUG_MODE': DEBUG_MODE}
            ],
            # remappings=[
            #     ('/vigitia/brio_rgb_full', '/vigitia/brio_rgb_full_testlaunch'),
            # ]
        ),
        Node(
            package='sensor_switch_node',
            namespace='vigitia',
            executable='sensor_switch_node',
            name='sensor_switch',
        ),
    ])
