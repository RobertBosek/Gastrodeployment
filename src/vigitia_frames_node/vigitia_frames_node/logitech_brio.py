#!/usr/bin/env
# coding: utf-8

# Code parts for asynchronous video capture taken from
# http://blog.blitzblit.com/2017/12/24/asynchronous-video-capture-in-python-with-opencv/

import cv2
import threading

CAMERA_ID = 0
RES_X = 4096
RES_Y = 2160
FPS = 5


class LogitechBrio:

    frame = None

    def __init__(self, ):
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
        if self.started:
            print('Already running')
            return None
        else:
            self.started = True
            self.thread = threading.Thread(target=self.update, args=())
            # thread.daemon = True
            self.thread.start()
            return self

    def update(self):

        while self.started:
            ret, frame = self.capture.read()

            if frame is not None:
                if frame.shape[0] < RES_Y or frame.shape[1] < RES_X:
                    print('WARNING: Output image resolution for additional camera is smaller then expected!')
                with self.read_lock:
                    self.frame = frame

    def get_frames(self):
        with self.read_lock:
            return self.frame

    # Just a test
    def confirm_received_frames(self):
        with self.read_lock:
            self.frame = None

    def get_resolution(self):
        return RES_X, RES_Y

    def stop(self):
        self.started = False
        self.thread.join()

    def __exit__(self, exec_type, exc_value, traceback):
        self.capture.release()


if __name__ == '__main__':
    camera = LogitechBrio()
    camera.init_video_capture()
    camera.start()
