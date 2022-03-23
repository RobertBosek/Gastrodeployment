import sys
import time

import cv2
import cv2.aruco as aruco

import numpy as np

from FiducialsDetectionService import FiducialsDetectionService
from CubeDetectorService import CubeDetector
from Webcam import GenericWebcam

from scipy import ndimage
import skimage.exposure




DEBUG_MODE = False

# equal to framerate?
N_FRAMES_DETECTION_RATIO = 60


app_id = {0: 'info',
          4: 'menu',
          2: 'drinks',
          3: 'coffee',
          4: 'events'}

app_x_y = (1,1)
data_dir = "C:/Gastrodeployment/deployment_data/"
data_dir = "/home/vigitia/Desktop/Gastrodeployment-main/deployment_data/"

class DeploymentControl:

    def __init__(self):
        # Init Aruco marker detector
        print("CUBE CONTROL MODE:")
        print("cubeside detection with auco")
        self.markers_detected = [False] * N_FRAMES_DETECTION_RATIO
        self.aruco_marker_detector = FiducialsDetectionService()
        self.cube_detector = CubeDetector()

        # Init webcam and start the asynchronous image capturing
        self.camera = GenericWebcam()
        self.camera.init_video_capture(fps=N_FRAMES_DETECTION_RATIO)
        self.camera.start()
        self.camera.get_last_frame()

        self.data_holder = {}
        self.init_data()

        # Start the main application loop
        self.loop()

    def init_data(self):
        daten = cv2.imread(data_dir + 'daten.png')
        daten = cv2.resize(daten, (600, 600))
        self.data_holder[6] = daten
        for key in app_id.keys():
            print(key)
            icon = cv2.imread(data_dir + 'icon-' + app_id[key] + '.png')
            dir = data_dir + app_id[key] + '.jpg'
            print(dir)
            prev = cv2.imread(data_dir + app_id[key] + '.jpg')
            prev = cv2.resize(prev, (600,600))
            self.data_holder[key] = [icon, prev]

    def loop(self):
        # Variables for fps counter
        start_time = 0
        counter = 0

        while True:
            # Get latest frame from camera
            color_image = self.camera.get_last_frame()

            # Only continue if needed frames are available
            if color_image is not None:

                prev = cv2.resize(color_image, (1280, 720))
                cv2.imshow('preview', prev)


                detected = self.detect_cubeside(color_image)
                self.markers_detected = self.markers_detected[1:N_FRAMES_DETECTION_RATIO]
                self.markers_detected.append(detected)

                # Update FPS Counter
                counter += 1
                if (time.time() - start_time) > 1:  # displays the frame rate every 1 second
                    print("FPS: ", round(counter / (time.time() - start_time), 1))
                    print("detection rate:", sum(self.markers_detected)/N_FRAMES_DETECTION_RATIO)
                    counter = 0
                    start_time = time.time()

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    def detect_cubeside(self, color_image):
        # Use the color image from the webcam to detect aruco markers

        # aruco_marker_detector only returns markers that are relevant for cubeside detection
        aruco_marker = self.aruco_marker_detector.detect_fiducials(color_image)

        # only a maximum of 4 markers and with ids of one specific side should be present, TODO: check this requirement
        markers = sorted(aruco_marker, key=lambda found_markers: found_markers['id'])

        if len(markers) in range(1, 5):
            cen, cubeside = self.cube_detector.substitute_markers(markers)

            if DEBUG_MODE:
                preview = color_image.copy()
                if cen is None:
                    cv2.circle(preview, np.array(cubeside['centroid'], dtype=int), 5, (255, 0, 255), -20)
                else:
                    cv2.circle(preview, np.array(cen, dtype=int), 5, (255, 0, 255), -20)
                #print(cubeside['corners'])
                #print(cubeside['sidenumber'])
                aruco.drawDetectedMarkers(preview, [np.array([cubeside['corners']], dtype=np.float32)], np.array([cubeside['sidenumber']], dtype=int))
                #print(markers)
                for marker in markers:
                    aruco.drawDetectedMarkers(preview, [np.array([marker['corners']], dtype=np.float32)], np.array([marker['id']], dtype=int))

                helper = int(abs((abs(cubeside['corners'][0][0]) - abs(cubeside['corners'][1][0]))))
                w = helper*2
                h = helper*3


                #cv2.rectangle(preview, (int(1.1), int(1.1)), (int(50.5),int(50.5)), (0,0,0), -1)
                cv2.rectangle(preview, (int(cubeside['centroid'][0]+helper), int(cubeside['centroid'][1]+helper)), (int(w), int(h)), (255, 255, 255), -1)
                #aruco.drawDetectedMarkers(preview, [np.array([centroids], dtype=np.float32)], np.array([id], dtype=np.int))
                prev = cv2.resize(preview, (1280, 720))
                cv2.imshow('marker', prev)

            if cubeside['sidenumber'] != 1 and cubeside['sidenumber'] != 5:
                self.handle_application(cubeside, color_image)

            return True
        else:
            prev = color_image.copy()
            daten = self.data_holder[6]
            x_offset = app_x_y[1]
            y_offset = app_x_y[0]

            x_end = x_offset + daten.shape[1]
            y_end = y_offset + daten.shape[0]

            prev[y_offset:y_end, x_offset:x_end] = daten
            cv2.imshow('prev', prev)
            return False


    def handle_application(self, cubeside, color_image):
        imgs = self.data_holder[cubeside['sidenumber']]

        icon = imgs[0]
        img = imgs[1]

        prev = color_image.copy()

        #handle application inserts
        x_offset = app_x_y[1]
        y_offset = app_x_y[0]

        x_end = x_offset + img.shape[1]
        y_end = y_offset + img.shape[0]

        prev[y_offset:y_end, x_offset:x_end] = img

        print(np.array(cubeside['corners'], dtype=int))
        cv2.fillPoly(prev, pts=[np.array(cubeside['corners'], dtype=int)], color=(0, 0, 0))
        '''
        image_center = tuple(np.array(icon.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, cubeside['angle'], 1.0)
        #icon = cv2.warpAffine(icon, rot_mat, icon.shape[1::-1], flags=cv2.INTER_LINEAR)
        #cv2.imshow('rotationscipy', rotated)
        #cv2.imshow('rotation', result)
        #cv2.imshow('icon', icon)

        # rotation angle in degree
        icon = ndimage.rotate(icon, -cubeside['angle'], cval=255)
        '''
        '''
        height, width = (icon.shape[0], icon.shape[1])
        size = (width, height)
        center = (width / 2, height / 2)

        dst_mat = np.zeros((height, width, 4), np.uint8)

        angle = cubeside['angle']

        scale = 0.5

        rotation_matrix = cv2.getRotationMatrix2D(center, angle, scale)

        img_dst = cv2.warpAffine(icon, rotation_matrix, size, dst_mat,
                                 flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_TRANSPARENT)



        sl = img_dst.shape[1]/2
        x_off = int(cubeside['centroid'][0]-sl)
        y_off = int(cubeside['centroid'][1]-sl)
        x_e = x_off + img_dst.shape[1]
        y_e = y_off + img_dst.shape[0]

        prev[y_off:y_e, x_off:x_e] = img_dst
        cv2.imshow('detection', prev)
        '''

        #handle icon inserts
        #https://learnopencv.com/homography-examples-using-opencv-python-c/#viewSource
        # Read source image.
        im_src = icon
        # Four corners of the book in source image
        h, w = icon.shape[0], icon.shape[1]
        pts_src = np.array([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]], dtype=int)

        # Read destination image.
        im_dst = prev
        # Four corners of the book in destination image.
        pts_dst = cubeside['corners']

        # Calculate Homography
        hh, status = cv2.findHomography(pts_src, pts_dst)

        # Warp source image to destination based on homography
        im_out = cv2.warpPerspective(im_src, hh, (im_dst.shape[1], im_dst.shape[0]))

        prev[np.where(im_out == 255)] = im_out[np.where(im_out == 255)]
        # Display images
        cv2.imshow("Source Image", im_src)
        cv2.imshow("Destination Image", im_dst)
        cv2.imshow("Warped Source Image", im_out)

        cv2.imshow("prev", prev)


def main():
    DeploymentControl()
    sys.exit()


if __name__ == '__main__':
    main()
