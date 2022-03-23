import sys
import numpy as np
import cv2
import cv2.aruco as aruco

from .FiducialsDetectorService import FiducialsDetectorService

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from aruco_interfaces.msg import ArucoList, ArucoMarker

from cv_bridge import CvBridge

DEBUG_MODE = True


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('tablearea_subscriber')

        self.cv_bridge = CvBridge()

        # Set Parameters
        topic_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Camera type')
        self.declare_parameter('topic_parameter', '/vigitia/rgb_table', topic_parameter)

        queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Length of the queue')
        self.declare_parameter('queue_length', 10, queue_length)

        # Init VIGITIA Service
        self.fiducials_detector = FiducialsDetectorService()

        # Create the subscriber. This subscriber will receive an Image
        # from the /rgb/image_raw topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,  # Datentyp
            self.get_parameter("topic_parameter").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback,
            self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.publisher = self.create_publisher(msg_type=ArucoList,
                                               topic='/aruco_list',
                                               qos_profile=10)

    def listener_callback(self, data):
        """
         Callback function.
         """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)
        frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2GRAY)

        # Call the VIGITIA service
        aruco_markers = self.fiducials_detector.detect_fiducials(frame_gray)

        aruco_list = self.convert_to_aruco_interface_msg(aruco_markers, data.header)

        if len(aruco_markers) > 0:
            self.publisher.publish(aruco_list)

            # Display image
            if DEBUG_MODE:
                print("[Fiducials Detector Node]: publishing ", len(aruco_markers), " detected markers")
                marker_detection = current_frame.copy()
                for marker in aruco_markers:
                    aruco.drawDetectedMarkers(marker_detection, [np.array([marker['corners']], dtype=np.float32)], np.array([marker['id']], dtype=np.int))
                cv2.imshow('markers_detected', marker_detection)

        cv2.waitKey(1)

    def convert_to_aruco_interface_msg(self, marker_list, header=None):
        aruco_list = ArucoList()
        if header is not None:
            aruco_list.header = header

        for marker in marker_list:
            aruco_marker = ArucoMarker()
            aruco_marker.id = int(marker['id'])
            aruco_marker.angle = marker['angle']

            aruco_marker.x0 = float(marker['corners'][0][0])
            aruco_marker.y0 = float(marker['corners'][0][1])
            aruco_marker.x1 = float(marker['corners'][1][0])
            aruco_marker.y1 = float(marker['corners'][1][1])
            aruco_marker.x2 = float(marker['corners'][2][0])
            aruco_marker.y2 = float(marker['corners'][2][1])
            aruco_marker.x3 = float(marker['corners'][3][0])
            aruco_marker.y3 = float(marker['corners'][3][1])

            aruco_marker.x_centroid = float(marker['centroid'][0])
            aruco_marker.y_centroid = float(marker['centroid'][1])

            aruco_list.aruco_markers.append(aruco_marker)

        return aruco_list


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
