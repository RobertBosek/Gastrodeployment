from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    '''
    azure_kinect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('azure_kinect_ros_driver'), 'launch'),
            '/driver.launch.py']), launch_arguments={'color_enabled': 'true',
                'color_resolution': '1080P',
                'fps': '15',
                'recording_loop_enabled': 'true'}.items()  # Change launch arguments in a specific file
    )
    '''

    return LaunchDescription([
        #azure_kinect,
        Node(
            package='logitech_brio_node',
            namespace='vigitia',
            executable='logitech_brio_node',
            name='camera_frames',
            # parameters=[
            #     {'window_origin.x': 3002}
            # ],
        ),
        Node(
            package='surface_extraction_node',
            namespace='vigitia',
            executable='surface_extraction_node',
            name='table_frames'
        ),
        Node(
            package='fiducials_detector_node',
            namespace='vigitia',
            executable='fiducials_detector_node',
            name='aruco_detector',
        ),
        Node(
            package='cube_tracker_node',
            namespace='vigitia',
            executable='cube_tracker_node',
            name='cube_tracker',
        )
    ])
