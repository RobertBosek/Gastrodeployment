import sys
import numpy as np
import cv2

import threading

#from .DeggingerEventService import DeggingerEventService as EventService
#from .DeggingerMenuService import MealMenuService, DrinkMenuService

SOME_GLOBALS = None
data_dir = "C:/Gastrodeployment/deployment_data/"
data_dir = "/home/vigitia/Desktop/Gastrodeployment/deployment_data/"


video_filename = "demo.mov"
window_name = "app prev"
interframe_wait_ms = 30

info_filename = 'daten.png'
instruction_filename = 'events.jpg'

app_id = {0: 'info',
          1: 'menu',
          2: 'drinks',
          3: 'coffee',
          4: 'events'}



class AppController:

    pose = None
    mode = [False, False, False]

    def __init__(self, node):
        self.node = node
        self.started = False

        self.read_lock = threading.Lock()

        self.info_screen = cv2.imread(data_dir + info_filename)
        self.info_screen = cv2.resize(self.info_screen, (600, 600))
        self.icon = cv2.imread(data_dir + instruction_filename)

    def reset_to_defaults(self):
        self.pose = None
        self.mode = [False, False, False]

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
            print(self.mode)
            #TODO: check conflict /w app nodes
            cv2.destroyAllWindows()
            # Get the newest frame from the camera
            if self.mode == [False, False, False]:
                self.show_demo_video()
            elif self.mode == [True, False, False]:
                self.show_info_screen()
            elif self.mode == [True, True, False]:
                self.show_icon_screen()

    def show_icon_screen(self):
        # show the image, provide window name first
        '''
        while self.mode == [True, False, False]:
            cv2.imshow(window_name, self.data_holder[6])
            if cv2.waitKey(interframe_wait_ms) & 0x7F == ord('q'):
                print("Exit requested.")
                break
        '''
        cv2.imshow(window_name, self.icon)
        while self.mode == [True, True, False]:
            cv2.waitKey(16)

    def show_info_screen(self):
        # show the image, provide window name first
        '''
        while self.mode == [True, False, False]:
            cv2.imshow(window_name, self.data_holder[6])
            if cv2.waitKey(interframe_wait_ms) & 0x7F == ord('q'):
                print("Exit requested.")
                break
        '''
        cv2.imshow(window_name, self.info_screen)
        while self.mode == [True, False, False]:
            cv2.waitKey(16)

    # from https://stackoverflow.com/questions/49949639/fullscreen-a-video-on-opencv
    def show_demo_video(self):
        cap = cv2.VideoCapture(data_dir + video_filename)
        if not cap.isOpened():
            print("Error: Could not open video.")
            exit()

        #cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
        #cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        while self.mode == [False, False, False]:
            ret, frame = cap.read()
            if not ret:
                print("Reached end of video, exiting.")
                break

            cv2.imshow(window_name, frame)
            if cv2.waitKey(interframe_wait_ms) & 0x7F == ord('q'):
                print("Exit requested.")
                break

        cap.release()

    def stop(self):
        if self.started:
            self.started = False
            self.thread.join()

    def manage_input(self):
        print(self.pose)


def main():
    AppController()
    sys.exit()


if __name__ == '__main__':
    main()
