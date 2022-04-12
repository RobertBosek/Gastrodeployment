#!/usr/bin/env
# coding: utf-8

# Code parts for asynchronous video capture taken from
# http://blog.blitzblit.com/2017/12/24/asynchronous-video-capture-in-python-with-opencv/

import cv2
import threading
import time

# Default values for the camera settings

CAMERA_ID = 0  # The ID of the camera connected to your PC/laptop. Default 0
RES_X = 1280#4096 #1280  # Horizontal resolution in pixel
RES_Y = 720#2160 #720 # Vertical resolution in pixel
FPS = 30 # setting FPS does only work with cameraspecific values, see also: https://stackoverflow.com/questions/52068277/change-frame-rate-in-opencv-3-4-2
EXPOSURE_TIME = 40
params_dic = d = {0: "cv.CAP_PROP_POS_MSEC",
                  1: "cv.CAP_PROP_POS_FRAMES",
                  2: "cv.CAP_PROP_POS_AVI_RATIO",
                  3: "cv.CAP_PROP_FRAME_WIDTH",
                  4: "cv.CAP_PROP_FRAME_HEIGHT",
                  5: "cv.CAP_PROP_FPS",
                  6: "cv.CAP_PROP_FOURCC",
                  7: "cv.CAP_PROP_FRAME_COUNT",
                  8: "cv.CAP_PROP_FORMAT",
                  9: "cv.CAP_PROP_MODE",
                  10: "cv.CAP_PROP_BRIGHTNESS",
                  11: "cv.CAP_PROP_CONTRAST",
                  12: "cv.CAP_PROP_SATURATION",
                  13: "cv.CAP_PROP_HUE",
                  14: "cv.CAP_PROP_GAIN",
                  15: "cv.CAP_PROP_EXPOSURE",
                  16: "cv.CAP_PROP_CONVERT_RGB",
                  17: "cv.CAP_PROP_WHITE_BALANCE_BLUE_U",
                  18: "cv.CAP_PROP_RECTIFICATION",
                  19: "cv.CAP_PROP_MONOCHROME",
                  20: "cv.CAP_PROP_SHARPNESS",
                  21: "cv.CAP_PROP_AUTO_EXPOSURE",
                  22: "cv.CAP_PROP_GAMMA",
                  23: "cv.CAP_PROP_TEMPERATURE",
                  24: "cv.CAP_PROP_TRIGGER",
                  25: "cv.CAP_PROP_TRIGGER_DELAY",
                  26: "cv.CAP_PROP_WHITE_BALANCE_RED_V",
                  27: "cv.CAP_PROP_ZOOM",
                  28: "cv.CAP_PROP_FOCUS",
                  29: "cv.CAP_PROP_GUID",
                  30: "cv.CAP_PROP_ISO_SPEED",
                  32: "cv.CAP_PROP_BACKLIGHT",
                  33: "cv.CAP_PROP_PAN",
                  34: "cv.CAP_PROP_TILT",
                  35: "cv.CAP_PROP_ROLL",
                  36: "cv.CAP_PROP_IRIS",
                  37: "cv.CAP_PROP_SETTINGS",
                  38: "cv.CAP_PROP_BUFFERSIZE",
                  39: "cv.CAP_PROP_AUTOFOCUS",
                  40: "cv.CAP_PROP_SAR_NUM",
                  41: "cv.CAP_PROP_SAR_DEN",
                  42: "cv.CAP_PROP_BACKEND",
                  43: "cv.CAP_PROP_CHANNEL",
                  44: "cv.CAP_PROP_AUTO_WB",
                  45: "cv.CAP_PROP_WB_TEMPERATURE",
                  46: "cv.CAP_PROP_CODEC_PIXEL_FORMAT",
                  47: "cv.CAP_PROP_BITRATE",
                  48: "cv.CAP_PROP_ORIENTATION_META",
                  49: "cv.CAP_PROP_ORIENTATION_AUTO",
                  53: "cv.CAP_PROP_OPEN_TIMEOUT_MSEC",
                  54: "cv.CAP_PROP_READ_TIMEOUT_MSEC"}


# This class allows asynchronous capturing of frames from a generic webcam (like the one built into your laptop or
# connected to a PC via USB)
class GenericWebcam:

    # The newest frame will be stored in this variable
    frame = None
    counter = 0
    start_time = time.time()
    thread = None

    def __init__(self, node=None):
        self.node = node
        self.started = False
        self.read_lock = threading.Lock()

    # If no settings are provided as arguments, the default values will be used
    def init_video_capture(self, camera_id=CAMERA_ID, resolution_x=RES_X, resolution_y=RES_Y, fps=FPS):

        self.capture = cv2.VideoCapture(camera_id)

        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        self.capture.set(cv2.CAP_PROP_FOURCC, fourcc)

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, resolution_x)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution_y)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        '''
        for k in params_dic.keys():
            print(params_dic[k], self.capture.get(k))
        #print("auto_fps: ", self.capture.get(cv2.CAP_PROP_FPS))
        #print("auto_exp: ", self.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE))
        #print("autofocus: ", self.capture.get(cv2.CAP_PROP_AUTOFOCUS))

        #self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

        #self.capture.set(cv2.CAP_PROP_FPS, fps)
        #self.capture.set(cv2.CAP_PROP_EXPOSURE, 0)

        print(self.capture.get(cv2.cv2.CAP_PROP_FPS))
        #print(self.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE))
        #print(self.capture.get(cv2.CAP_PROP_EXPOSURE))

        '''

    # Start a thread for async video capturing
    def start(self):
        if self.started:  # Prevent the thread from starting it again if it is already running
            print('Already running')
            return None
        else:
            self.started = True
            self.thread = threading.Thread(target=self.__update, args=())
            self.thread.start()
            return self

    def __update(self):
        while self.started:
            # Get the newest frame from the camera
            ret, frame = self.capture.read()

            # If a new frame is available, store it in the corresponding variable
            if frame is not None:
                self.counter += 1
                with self.read_lock:
                    self.frame = frame

                    if self.node is not None:
                        self.node.publisher.publish(self.node.cv_bridge.cv2_to_imgmsg(self.frame, "passthrough"))
                        if (time.time() - self.start_time) > 1:  # displays the frame rate every 1 second
                            self.node.get_logger().info("FPS: %s" % round(self.counter / (time.time() - self.start_time), 1))
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



# Uncomment the following lines if you just want to test the camera by itself

# webcam = GenericWebcam()
# webcam.init_video_capture()
# webcam.start()
#
# while True:
#     frame = webcam.get_last_frame()
#     if frame is not None:
#         cv2.imshow('webcam', frame)
#
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
#
# cv2.destroyAllWindows()


## cam properties helper
