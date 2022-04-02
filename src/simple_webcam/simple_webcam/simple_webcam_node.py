#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .Webcam import GenericWebcam


DEBUG_MODE = True


class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam_frames_node')

        self.publisher = self.create_publisher(msg_type=Image, topic='/vigitia/brio_rgb_full', qos_profile=10)

        self.cv_bridge = CvBridge()

        self.camera = GenericWebcam()
        self.camera.init_video_capture()
        self.camera.start()

        # Start the main application loop
        self.loop()


    # The main application loop. Code parts for fps counter from
    # https://stackoverflow.com/questions/43761004/fps-how-to-divide-count-by-time-function-to-determine-fps
    def loop(self):
        # Variables for fps counter
        start_time = 0
        counter = 0

        while True:
            # Get latest frame from camera
            color_image = self.camera.get_last_frame()

            # Only continue if needed frames are available
            if color_image is not None:
                self.publisher.publish(self.cv_bridge.cv2_to_imgmsg(color_image, "passthrough"))

                # Update FPS Counter
                counter += 1
                if (time.time() - start_time) > 1:  # displays the frame rate every 1 second
                    print("FPS: ", round(counter / (time.time() - start_time), 1))
                    counter = 0
                    start_time = time.time()

            if DEBUG_MODE:
                if color_image is not None:
                    cv2.imshow('color full frame', color_image)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break


def main(args=None):
    rclpy.init(args=args)
    vigitia_frames_node = WebcamNode()
    print('hallo')
    rclpy.spin(vigitia_frames_node)
    vigitia_frames_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()