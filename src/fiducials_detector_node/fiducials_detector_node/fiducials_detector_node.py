import sys
import numpy as np
import cv2
import cv2.aruco as aruco

from .FiducialsDetectorService import FiducialsDetectorService

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from vigitia_interfaces.msg import ArucoList, ArucoMarker

from cv_bridge import CvBridge

DEBUG_MODE = True


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('tablearea_subscriber')

        param_desc_debug = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='debug mode bool')
        param_desc_rgb_table = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of subscribing topic')
        param_desc_queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='length of the queue')
        param_desc_aruco_list = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of publishing topic')

        self.declare_parameter('DEBUG_MODE', True, param_desc_debug)
        self.declare_parameter('topic_rgb_table', '/vigitia/rgb_table', param_desc_rgb_table)
        self.declare_parameter('queue_length', 10, param_desc_queue_length)
        self.declare_parameter('aruco_list', '/vigitia/aruco_list', param_desc_aruco_list)

        self.create_subscription(msg_type=Image,
                                 topic=self.get_parameter("topic_rgb_table").get_parameter_value().string_value,
                                 callback=self.listener_callback,
                                 qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.publisher = self.create_publisher(msg_type=ArucoList,
                                               topic=self.get_parameter("aruco_list").get_parameter_value().string_value,
                                               qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.cv_bridge = CvBridge()

        self.fiducials_detector = FiducialsDetectorService()

        self.get_logger().info('initialized')

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

        if len(aruco_markers) > 0:
            aruco_list = self.convert_to_aruco_interface_msg(aruco_markers, data.header)
            self.publisher.publish(aruco_list)

            # Display image
            if self.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                self.get_logger().info('detection rate: %s' % self.fiducials_detector.marker_detection_rate)
                marker_detection = current_frame.copy()
                for marker in aruco_markers:
                    aruco.drawDetectedMarkers(marker_detection, [np.array([marker['corners']], dtype=np.float32)], np.array([marker['id']], dtype=np.int))
                prev = cv2.resize(marker_detection, (1280, 720))
                cv2.imshow('markers_detected', prev)

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
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
