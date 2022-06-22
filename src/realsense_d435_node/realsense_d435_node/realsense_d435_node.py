#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import cv2

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

from sensor_msgs.msg import Image
from vigitia_interfaces.srv import SensorToggle
from vigitia_interfaces.srv import SensorToggle as VIGITIAMovement

from cv_bridge import CvBridge

from .realsense_d435 import RealsenseD435Camera

FLIP_IMAGE = False  # Necessary if the camera is upside down (very ressource intensive!)


class VIGITIARealsenseFramesNode(Node):

    def __init__(self):
        super().__init__('brio_frames_publisher')

        param_desc_debug = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='debug mode bool')
        param_desc_img_flip = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='flip image bool')
        param_desc_ir_full = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of ir publishing topic')
        param_desc_rgb_full = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of rgb publishing topic')
        param_desc_movement_toggle = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of movement toggle service')
        param_desc_sensor_toggle = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of sensor toggle service')
        param_desc_queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='length of the queue')


        self.declare_parameter('DEBUG_MODE', True, param_desc_debug)
        self.declare_parameter('FLIP_IMAGE', False, param_desc_img_flip)
        self.declare_parameter('topic_rgb', '/vigitia/realsense_rgb_full', param_desc_ir_full)
        self.declare_parameter('topic_ir', '/vigitia/realsense_ir_full', param_desc_ir_full)
        self.declare_parameter('srv_movement_toggle', '/vigitia/srv/movement_toggle', param_desc_movement_toggle)
        self.declare_parameter('srv_sensor_toggle', '/vigitia/srv/sensor_toggle', param_desc_sensor_toggle)
        self.declare_parameter('queue_length', 10, param_desc_queue_length)

        self.cv_bridge = CvBridge()

        self.publisher_rgb = self.create_publisher(msg_type=Image,
                                                   topic=self.get_parameter("topic_rgb").get_parameter_value().string_value,
                                                   qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.publisher_ir = self.create_publisher(msg_type=Image,
                                                  topic=self.get_parameter("topic_ir").get_parameter_value().string_value,
                                                  qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.create_service(srv_type=VIGITIAMovement,
                            srv_name=self.get_parameter("srv_movement_toggle").get_parameter_value().string_value,
                            callback=self.movement_toggle_callback)

        self.create_service(srv_type=SensorToggle,
                            srv_name=self.get_parameter("srv_sensor_toggle").get_parameter_value().string_value,
                            callback=self.sensor_toggle_callback)

        self.realsense_d435_camera = RealsenseD435Camera(self)
        self.realsense_d435_camera.init_video_capture()

        self.get_logger().info('initialized, waiting for action to activate camera...')

        self.realsense_d435_camera.start()

    def movement_toggle_callback(self, request, response):
        if not request.active:
            self.get_logger().info('Incoming request: no person at table, stop frames')
            self.logitech_brio_camera.stop()
        response.status = 'successful'
        return response

    def sensor_toggle_callback(self, request, response):
        self.get_logger().info('Incoming request: %s frames' % ('start' if request.active else 'end'))
        if request.active:
            self.logitech_brio_camera.start()
        else:
            self.logitech_brio_camera.stop()
        response.status = 'successful'
        return response


def main(args=None):
    rclpy.init(args=args)
    vigitia_realsense_frames_node = VIGITIARealsenseFramesNode()
    rclpy.spin(vigitia_realsense_frames_node)
    vigitia_realsense_frames_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
