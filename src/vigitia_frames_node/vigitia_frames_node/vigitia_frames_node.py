#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .realsense_d435 import RealsenseD435Camera
from .logitech_brio import LogitechBrio
from .table_extraction_service import TableExtractionService

DEBUG_MODE = False

FLIP_IMAGE = False  # Necessary if the camera is upside down (very ressource intensive!)

NEED_COLOR_IMAGE_FULL = False
NEED_IR_IMAGE_FULL = False
NEED_COLOR_IMAGE_ADDITIONAL_FULL = True

NEED_COLOR_IMAGE_TABLE = True
NEED_IR_IMAGE_TABLE = True
NEED_COLOR_IMAGE_ADDITIONAL_TABLE = True

NEED_DEPTH_IMAGE_FULL = False
NEED_DEPTH_IMAGE_TABLE = False

NEED_HSV_IMAGE_TABLE = False
NEED_FOREGROUND_MASK = False


class VIGITIAFramesNode(Node):

    def __init__(self):
        super().__init__('vigitia_frames_node')

        self.init_publisher()

        self.cv_bridge = CvBridge()

        self.table_extraction_service = TableExtractionService()

        #self.realsense_d435_camera = RealsenseD435Camera()
        #self.realsense_d435_camera.init_video_capture()
        #self.realsense_d435_camera.start()

        self.logitech_brio_camera = LogitechBrio()
        self.logitech_brio_camera.init_video_capture()
        self.logitech_brio_camera.start()

        self.loop()

    def init_publisher(self):
        if NEED_COLOR_IMAGE_FULL:
            self.publisher_rgb_full = self.create_publisher(msg_type=Image, topic='/vigitia/rgb_full', qos_profile=10)
        if NEED_IR_IMAGE_FULL:
            self.publisher_ir_full = self.create_publisher(msg_type=Image, topic='/vigitia/ir_full', qos_profile=10)
        if NEED_COLOR_IMAGE_ADDITIONAL_FULL:
            self.publisher_rgb_additional_full = self.create_publisher(msg_type=Image,
                                                                       topic='/vigitia/rgb_additional_full', qos_profile=10)

        self.publisher_rgb_table = self.create_publisher(msg_type=Image, topic='/vigitia/rgb_table', qos_profile=10)
        self.publisher_ir_table = self.create_publisher(msg_type=Image, topic='/vigitia/ir_table', qos_profile=10)
        self.publisher_rgb_additional_table = self.create_publisher(msg_type=Image,
                                                                    topic='/vigitia/rgb_additional_table',
                                                                    qos_profile=10)

    # The main application loop. Code parts for fps counter from
    # https://stackoverflow.com/questions/43761004/fps-how-to-divide-count-by-time-function-to-determine-fps
    def loop(self):
        # Variables for fps counter
        start_time = 0
        counter = 0

        while True:
            #color_image, depth_image, left_ir_image = self.realsense_d435_camera.get_frames()  # Get frames from cameras
            color_image_additional = self.logitech_brio_camera.get_frames()

            #if color_image is None and color_image_additional is None:
            if color_image_additional is None:
                continue

            # if color_image is not None:
            #     self.realsense_d435_camera.confirm_received_frames()

            if color_image_additional is not None:
                self.logitech_brio_camera.confirm_received_frames()

            # Pre-process camera frames
            color_image_table, left_ir_image_table, depth_image_table, foreground_mask, hsv_image_table, \
                color_image_additional_table = self.preprocess_camera_frames(None, None, None,
                                                                             color_image_additional)

            self.publish_camera_frames(None, color_image_table, None, left_ir_image_table,
                                       depth_image_table, foreground_mask, hsv_image_table, color_image_additional,
                                       color_image_additional_table)

            if DEBUG_MODE:
                if color_image_additional is not None:
                    cv2.imshow('color additional', color_image_additional)
                # cv2.imshow('ir left', left_ir_image)
                # cv2.imshow('ir left table', left_ir_image_table)
                # cv2.imshow('color table', color_image_table)
                # cv2.imshow('zoom', color_image_zoom)
                # cv2.imshow('Binary Mask of the foreground', foreground_mask)

            # FPS Counter
            counter += 1
            if (time.time() - start_time) > 1:  # displays the frame rate every 1 second
                print("[VIGITIA Frames Node]: FPS: ", round(counter / (time.time() - start_time), 1))
                counter = 0
                start_time = time.time()

            if DEBUG_MODE:
                key = cv2.waitKey(1)
                # Press s to save the current image
                if key & 0xFF == ord('s'):
                    try:
                        cv2.imwrite('screenshot.png', color_image_additional)
                    except Exception as e:
                        print(e)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    # break
                    sys.exit(0)

    def publish_camera_frames(self, color_image, color_image_table, left_ir_image, left_ir_image_table,
                              depth_image_table, foreground_mask, hsv_image_table, color_image_additional,
                              color_image_additional_table):

        if NEED_COLOR_IMAGE_FULL and color_image is not None:
            self.publisher_rgb_full.publish(self.cv_bridge.cv2_to_imgmsg(color_image, "passthrough"))
        if color_image_table is not None:
            self.publisher_rgb_table.publish(self.cv_bridge.cv2_to_imgmsg(color_image_table, "passthrough"))
        if NEED_IR_IMAGE_FULL and left_ir_image is not None:
            self.publisher_ir_full.publish(self.cv_bridge.cv2_to_imgmsg(left_ir_image, "passthrough"))
        if NEED_IR_IMAGE_TABLE and left_ir_image_table is not None:
            self.publisher_ir_table.publish(self.cv_bridge.cv2_to_imgmsg(left_ir_image_table, "passthrough"))
        if NEED_COLOR_IMAGE_ADDITIONAL_FULL and color_image_additional is not None:
            self.publisher_rgb_additional_full.publish(self.cv_bridge.cv2_to_imgmsg(color_image_additional,
                                                                                    "passthrough"))
        if color_image_additional_table is not None:
            self.publisher_rgb_additional_table.publish(self.cv_bridge.cv2_to_imgmsg(color_image_additional_table,
                                                                                     "passthrough"))

    # TODO: Run these steps in parallel
    def preprocess_camera_frames(self, color_image, depth_image, left_ir_image, color_image_additional):
        color_image_table = None
        hsv_table_image = None
        left_ir_image_table = None
        depth_image_table = None
        foreground_mask = None
        color_image_additional_table = None

        if NEED_COLOR_IMAGE_TABLE and color_image is not None:
            color_image_table = self.table_extraction_service.extract_table_area(color_image, '/vigitia/rgb_full')
        if NEED_HSV_IMAGE_TABLE and color_image_table is not None:
            hsv_table_image = cv2.cvtColor(color_image_table, cv2.COLOR_BGR2HSV)
        if NEED_IR_IMAGE_TABLE and left_ir_image is not None:
            left_ir_image_table = self.table_extraction_service.extract_table_area(left_ir_image, '/vigitia/ir_full')
        if NEED_DEPTH_IMAGE_FULL and depth_image is not None:
            depth_image_table = self.table_extraction_service.extract_table_area(depth_image, '/vigitia/depth_full')

        if color_image_additional is not None:
            color_image_additional_table = self.table_extraction_service.extract_table_area(color_image_additional,
                                                                                            '/vigitia/rgb_additional_full')

        # Some SensorProcessingServices need the foreground mask. Request it here and pass it over to them.
        # if NEED_FOREGROUND_MASK:
        #     foreground_mask = self.foreground_mask_extraction_service.get_foreground_mask_otsu(color_image_table)

        # TODO: This needs a lot of resources and drops the fps. Try to avoid it
        # TODO: Check if left and right hand label is correct in mediapipe tracking after flip
        # if FLIP_IMAGE:
        #     color_image = cv2.flip(color_image, -1)
        #     color_image_table = cv2.flip(color_image_table, -1)
        #     left_ir_image = cv2.flip(left_ir_image, -1)
        #     left_ir_image_table = cv2.flip(left_ir_image_table, -1)
        #     depth_image = cv2.flip(depth_image, -1)
        #     depth_image_table = cv2.flip(depth_image_table, -1)

        return color_image_table, left_ir_image_table, depth_image_table, foreground_mask, hsv_table_image, color_image_additional_table


def main(args=None):
    rclpy.init(args=args)
    vigitia_frames_node = VIGITIAFramesNode()
    rclpy.spin(vigitia_frames_node)
    vigitia_frames_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
