#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .logitech_brio import LogitechBrio

DEBUG_MODE = True

FLIP_IMAGE = False  # Necessary if the camera is upside down (very ressource intensive!)


class VIGITIABrioFramesNode(Node):

    def __init__(self):
        super().__init__('brio_frames_publisher')

        self.publisher_rgb_full = self.create_publisher(msg_type=Image, topic='/vigitia/brio_rgb_full', qos_profile=10)

        self.cv_bridge = CvBridge()

        self.logitech_brio_camera = LogitechBrio()
        self.logitech_brio_camera.init_video_capture()
        self.logitech_brio_camera.start()

        self.loop()

    # The main application loop. Code parts for fps counter from
    # https://stackoverflow.com/questions/43761004/fps-how-to-divide-count-by-time-function-to-determine-fps
    def loop(self):
        # Variables for fps counter
        start_time = 0
        counter = 0

        while True:
            #color_image, depth_image, left_ir_image = self.realsense_d435_camera.get_frames()  # Get frames from cameras
            color_image = self.logitech_brio_camera.get_frames()

            #if color_image is None and color_image_additional is None:
            if color_image is None:
                continue

            if color_image is not None:
                self.logitech_brio_camera.confirm_received_frames()

            self.publisher_rgb_full.publish(self.cv_bridge.cv2_to_imgmsg(color_image, "passthrough"))

            if DEBUG_MODE:
                if color_image is not None:
                    cv2.imshow('color brio', color_image)

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
                        cv2.imwrite('screenshot.png', color_image)
                    except Exception as e:
                        print(e)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    # break
                    sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    vigitia_brio_frames_node = VIGITIABrioFramesNode()
    rclpy.spin(vigitia_brio_frames_node)
    vigitia_brio_frames_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
