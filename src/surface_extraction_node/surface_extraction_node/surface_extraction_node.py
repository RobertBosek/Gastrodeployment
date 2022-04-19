#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes

from sensor_msgs.msg import Image  # Image is the message type
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from cv_bridge import CvBridge

from .table_extraction_service import TableExtractionService


class TableExtractorNode(Node):

    def __init__(self):
        super().__init__('brio_subscriber_tablearea_publisher')

        param_desc_debug = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='debug mode bool')
        param_desc_rgb_full = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of subscribing topic')
        param_desc_queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='length of the queue')
        param_desc_rgb_table = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of publishing topic')

        self.declare_parameter('DEBUG_MODE', True, param_desc_debug)
        self.declare_parameter('topic_rgb_full', '/vigitia/brio_rgb_full', param_desc_rgb_full)
        self.declare_parameter('queue_length', 10, param_desc_queue_length)
        self.declare_parameter('topic_rgb_table', '/vigitia/rgb_table', param_desc_rgb_table)

        self.cv_bridge = CvBridge()

        self.create_subscription(msg_type=Image,
                                 topic=self.get_parameter("topic_rgb_full").get_parameter_value().string_value,
                                 callback=self.listener_callback,
                                 qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)
        self.publisher = self.create_publisher(msg_type=Image,
                                               topic=self.get_parameter("topic_rgb_table").get_parameter_value().string_value,
                                               qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.cv_bridge = CvBridge()

        # TODO: reloadable config file
        self.table_extraction_service = TableExtractionService()

    def listener_callback(self, data):
        """
         Callback function.
         """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)

        if self.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
            prev = cv2.resize(current_frame, (1280, 720))
            cv2.imshow("ros msg", prev)

        color_image_table = self.table_extraction_service.extract_table_area(current_frame, self.get_parameter("topic_rgb_full").get_parameter_value().string_value)

        self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(color_image_table, "passthrough"))

        if self.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
            print("[Surface Extractor Node]: Mapping Table area")
            prev = cv2.resize(color_image_table, (1280, 720))
            cv2.imshow("table_area", prev)

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