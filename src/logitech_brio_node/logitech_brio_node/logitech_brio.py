#!/usr/bin/env
# coding: utf-8

# Code parts for asynchronous video capture taken from
# http://blog.blitzblit.com/2017/12/24/asynchronous-video-capture-in-python-with-opencv/

import cv2
import threading
import time

CAMERA_ID = 1
RES_X = 4096
RES_Y = 2160
FPS = 30


class LogitechBrio:

    frame = None
    counter = 0
    start_time = time.time()
    thread = None

    def __init__(self, node=None):
        self.node = node
        self.started = False
        self.read_lock = threading.Lock()

    def init_video_capture(self, camera_id=CAMERA_ID, resolution_x=RES_X, resolution_y=RES_Y, fps=FPS):
        self.capture = cv2.VideoCapture()
        self.capture.open(camera_id)

        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.capture.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution_x)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution_y)
        self.capture.set(cv2.CAP_PROP_FPS, fps)

    def start(self):
        if self.started:  # Prevent the thread from starting it again if it is already running
            print('Already running')
            return None
        else:
            self.started = True
            self.thread = threading.Thread(target=self.__update, args=())
            # thread.daemon = True
            self.thread.start()
            return self

    def __update(self):
        while self.started:
            # Get the newest frame from the camera
            ret, frame = self.capture.read()

            # If a new frame is available, store it in the corresponding variable
            if frame is not None:
                self.counter += 1
                '''
                if frame.shape[0] < RES_Y or frame.shape[1] < RES_X:
                    print('WARNING: Output image resolution for additional camera is smaller then expected!')
                '''
                with self.read_lock:
                    self.frame = frame

                    if self.node is not None:
                        self.node.publisher.publish(self.node.cv_bridge.cv2_to_imgmsg(self.frame, "passthrough"))
                        if (time.time() - self.start_time) > 1:  # displays the frame rate every 1 second
                            self.node.get_logger().info(
                                "FPS: %s" % round(self.counter / (time.time() - self.start_time), 1))
                            self.counter = 0
                            self.start_time = time.time()
                    else:
                        if (time.time() - self.start_time) > 1:  # displays the frame rate every 1 second
                            print("FPS: %s" % round(self.counter / (time.time() - self.start_time), 1))
                            self.counter = 0
                            self.start_time = time.time()

    # Call this method from the outside to get the latest stored frame
    def get_last_frame(self):
        with self.read_lock:
            return self.frame

    @staticmethod
    # Request the current resolution of the camera
    def get_resolution():
        return RES_X, RES_Y

    # Stop the thread
    def stop(self):
        if self.started:
            self.started = False
            self.thread.join()

    # Release the camera if the script is stopped
    def __exit__(self, exec_type, exc_value, traceback):
        self.capture.release()


if __name__ == '__main__':
    camera = LogitechBrio()
    camera.init_video_capture()
    camera.start()
