#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vigitia_interfaces.srv import SensorToggle
from vigitia_interfaces.srv import SensorToggle as VIGITIAMovement

from cv_bridge import CvBridge

from .Webcam import GenericWebcam


DEBUG_MODE = True


class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam_frames_node')

        self.publisher = self.create_publisher(msg_type=Image, topic='/vigitia/brio_rgb_full', qos_profile=10)
        self.ble_srv = self.create_service(SensorToggle, '/vigitia/toggle_camera', self.toggle_camera_callback)
        self.move_srv = self.create_service(VIGITIAMovement, '/vigitia/person_near', self.toggle_movement_callback)

        self.cv_bridge = CvBridge()

        self.camera = GenericWebcam(self)
        self.camera.init_video_capture()
        self.get_logger().info('initialized, waiting for action to activate camera...')

        # self.camera.start()

    def toggle_movement_callback(self, request, response):
        if not request.active:
            self.get_logger().info('Incoming request: no person at table stop frames')
            self.camera.stop()
        response.status = 'successful'
        return response


    def toggle_camera_callback(self, request, response):
        self.get_logger().info('Incoming request: %s frames' % ('start' if request.active else 'end'))
        if request.active:
            self.camera.start()
        else:
            self.camera.stop()
        response.status = 'successful'
        return response

def main(args=None):
    rclpy.init(args=args)
    vigitia_frames_node = WebcamNode()
    rclpy.spin(vigitia_frames_node)
    vigitia_frames_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()