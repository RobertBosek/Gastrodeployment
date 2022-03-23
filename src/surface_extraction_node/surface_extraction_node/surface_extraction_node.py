#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes

from sensor_msgs.msg import Image  # Image is the message type
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from cv_bridge import CvBridge

from .table_extraction_service import TableExtractionService

DEBUG_MODE = True


class TableExtractorNode(Node):

    def __init__(self):
        super().__init__('brio_subscriber_tablearea_publisher')

        self.cv_bridge = CvBridge()
        self.table_extraction_service = TableExtractionService()

        rgb_full_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Camera type')
        self.declare_parameter('topic_rgb_full', '/vigitia/brio_rgb_full', rgb_full_parameter)

        queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Length of the queue')
        self.declare_parameter('queue_length', 10, queue_length)

        self.subscription = self.create_subscription(
            Image,  # Datentyp
            self.get_parameter("topic_rgb_full").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback,
            self.get_parameter("queue_length").get_parameter_value().integer_value)

        rgb_table_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Camera type')
        self.declare_parameter('topic_rgb_table', '/vigitia/rgb_table', rgb_table_parameter)

        self.publisher_rgb_table = self.create_publisher(msg_type=Image,
                                                         topic=self.get_parameter("topic_rgb_table").get_parameter_value().string_value,
                                                         qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

    def listener_callback(self, data):
        """
         Callback function.
         """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)

        if DEBUG_MODE:
            cv2.imshow("ros msg", current_frame)

        color_image_table = self.table_extraction_service.extract_table_area(current_frame, self.get_parameter("topic_rgb_full").get_parameter_value().string_value)

        self.publisher_rgb_table.publish(self.cv_bridge.cv2_to_imgmsg(color_image_table, "passthrough"))

        if DEBUG_MODE:
            print("[Surface Extractor Node]: Mapping Table area")
            cv2.imshow("table_area", color_image_table)

        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    table_extractor_node = TableExtractorNode()

    # Spin the node so the callback function is called.
    rclpy.spin(table_extractor_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    table_extractor_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()