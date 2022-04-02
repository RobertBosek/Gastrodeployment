from .CubeTrackerService import CubeDetector

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from vigitia_interfaces.msg import ArucoList, ArucoMarker

from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge

DEBUG_MODE = True

class ArucoSubscriber(Node):

    def __init__(self):
        super().__init__('aruco_subscriber')

        # Set Parameters
        topic_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='detected aruco markers')
        self.declare_parameter('aruco_topic_parameter', '/aruco_list', topic_parameter)

        queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Length of the queue')
        self.declare_parameter('queue_length', 10, queue_length)

        # Init VIGITIA Service
        self.cube_detector = CubeDetector()

        # Create the subscriber. This subscriber will receive an Image
        # from the /rgb/image_raw topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            ArucoList,  # Datentyp
            self.get_parameter("aruco_topic_parameter").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback,
            self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.publisher = self.create_publisher(msg_type=ArucoMarker,
                                               topic='/cube_pose',
                                               qos_profile=10)

    def listener_callback(self, data):
        """
         Callback function.
         """
        detected_markers = []

        for aruco_marker in data.aruco_markers:
            detected_marker = {'id': aruco_marker.id,
                               'angle': aruco_marker.angle,
                               'corners': [[aruco_marker.x0, aruco_marker.y0],
                                           [aruco_marker.x1, aruco_marker.y1],
                                           [aruco_marker.x2, aruco_marker.y2],
                                           [aruco_marker.x3, aruco_marker.y3]],
                               'centroid': [aruco_marker.x_centroid, aruco_marker.y_centroid]}
            detected_markers.append(detected_marker)

        markers = self.cube_detector.preselect_markers(detected_markers)
        #cube geometrics are structured just like aruco marker information
        cube_geoms = self.cube_detector.get_cube_spatial_info(markers)

        if cube_geoms:
            cube_pose = ArucoMarker()
            cube_pose.id = int(cube_geoms['id'])
            cube_pose.angle = int(cube_geoms['angle'])

            cube_pose.x0 = float(cube_geoms['corners'][0][0])
            cube_pose.y0 = float(cube_geoms['corners'][0][1])
            cube_pose.x1 = float(cube_geoms['corners'][1][0])
            cube_pose.y1 = float(cube_geoms['corners'][1][1])
            cube_pose.x2 = float(cube_geoms['corners'][2][0])
            cube_pose.y2 = float(cube_geoms['corners'][2][1])
            cube_pose.x3 = float(cube_geoms['corners'][3][0])
            cube_pose.y3 = float(cube_geoms['corners'][3][1])

            cube_pose.x_centroid = float(cube_geoms['centroid'][0])
            cube_pose.y_centroid = float(cube_geoms['centroid'][1])

            print(cube_pose)
            self.publisher.publish(cube_pose)

            # Display image
            if DEBUG_MODE:
                print("[Cube Tracker Node]: Publishing Cube Pose", cube_geoms)



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    aruco_subscriber = ArucoSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(aruco_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()