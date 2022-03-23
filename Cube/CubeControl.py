import sys
import time

import cv2
import cv2.aruco as aruco

import numpy as np

import detection
from ArucoMarkerDetector import ArucoMarkerDetector
from FiducialsDetectionService import FiducialsDetectionService
from CubeDetectorService import CubeDetector
from Webcam import GenericWebcam

# enter detection mode here:
# 0 for aruco
# 1 for basic seed marker
# 2 for icon seed marker
# 3 for aruco cube side
MODE = 3

DEBUG_MODE = True
# equal to framerate?
N_FRAMES_DETECTION_RATIO = 100


class CubeControl:
    frame_id = 0

    def __init__(self):
        # Init Aruco marker detector
        print("CUBE CONTROL MODE:")
        if MODE == 0:
            print("single aruco detector")
            self.aruco_marker_detector = ArucoMarkerDetector()
        elif MODE == 1:
            print("basic seed marker detection")
            self.marker_list = self.get_predefined_markernames(list(range(1, 7)), "data/square{}.png")
            #print(self.marker_list)
        elif MODE == 2:
            print("icon seed marker detection")
            self.marker_list = self.get_predefined_markernames(list(range(0, 4)), "data/side0{}.png")
            #print(self.marker_list)
        elif MODE == 3:
            print("cubeside detection with auco")
            self.markers_detected = [False] * N_FRAMES_DETECTION_RATIO
            self.aruco_marker_detector = FiducialsDetectionService()
            self.cube_detector = CubeDetector()

        # Init webcam and start the asynchronous image capturing
        self.camera = GenericWebcam()
        self.camera.init_video_capture(fps=N_FRAMES_DETECTION_RATIO)
        self.camera.start()
        self.camera.get_last_frame()
        #self.camera.adjust_params(fps=30, exposure=1)

        # Start the main application loop
        self.loop()

    def get_predefined_markernames(self, ids, file_base):
        markers = []
        for m in ids:
            im = cv2.imread(file_base.format(m), cv2.IMREAD_UNCHANGED)
            marker = detection.process(im, None, None)
            markers.append(marker[0].get_lhds_name())
        return markers

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
                #prev = cv2.resize(color_image.copy(), (1280, 720))
                #cv2.imshow('preview', prev)

                self.frame_id += 1

                if MODE == 0: # aruco marker
                    self.detect_aruco(color_image)

                elif MODE == 1 or MODE == 2: # basic/icon seed marker
                    self.detect_seedmarker(color_image)

                elif MODE == 3: # for aruco cube side
                    detected = self.detect_cubeside(color_image)
                    self.markers_detected = self.markers_detected[1:N_FRAMES_DETECTION_RATIO]
                    self.markers_detected.append(detected)

                # Update FPS Counter
                counter += 1
                if (time.time() - start_time) > 1:  # displays the frame rate every 1 second
                    print("FPS: ", round(counter / (time.time() - start_time), 1))
                    if MODE == 3:
                        print("detection rate:", sum(self.markers_detected)/N_FRAMES_DETECTION_RATIO)
                    counter = 0
                    start_time = time.time()

                    print("params")
                    print("fps: ", self.camera.capture.get(cv2.CAP_PROP_FPS))
                    print("auto_exp: ", self.camera.capture.get(cv2.CAP_PROP_AUTO_EXPOSURE))
                    print("exposure: ", self.camera.capture.get(cv2.CAP_PROP_EXPOSURE))
                    print("focus: ", self.camera.capture.get(cv2.CAP_PROP_FOCUS))

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
                aruco.drawDetectedMarkers(preview, [np.array([cubeside['corners']], dtype=np.float32)], np.array([cubeside['sidenumber']], dtype=np.int))
                #print(markers)
                for marker in markers:
                    aruco.drawDetectedMarkers(preview, [np.array([marker['corners']], dtype=np.float32)], np.array([marker['id']], dtype=np.int))
                #aruco.drawDetectedMarkers(preview, [np.array([centroids], dtype=np.float32)], np.array([id], dtype=np.int))
                prev = cv2.resize(preview, (1280, 720))
                #cv2.imshow('marker', prev)
            return True
        else:
            return False

        #if len(aruco_marker) > 0:
        #    print(aruco_marker)


    def get_cube_geometrics(self, found_markers, color_image):
        print("markers")
        print(found_markers)
        markers = sorted(found_markers, key=lambda found_markers: found_markers['id'])
        print(markers)
        angles = []
        centroids = []
        ids = []
        for m in markers:
            angles.append(m['angle'])
            centroids.append([m['centroid'][0], m['centroid'][1]])
            ids.append(m['id']%4)
        if len(markers) == 4:
            if len(angles) == len(set(angles)):
                print("wrong")
                return()
            else:
                copy_angle = angles
                eq = False
                while(len(copy_angle) > 1 or eq == False):
                    ca = copy_angle.pop(0)
                    for a in copy_angle:
                        if a == ca:
                            eq = True
                            break
            if(eq):
                angle = ca

            cen = self.aruco_marker_detector.centroid(centroids)

            radiae = np.array(centroids) - np.array(cen)

            rad = np.sqrt((sum([x[0]**2 for x in radiae])/4) + (sum([y[1]**2 for y in radiae])/4)) / 0.7
            id = ids[0]
            print("here")
            print(cen)
            print(rad)
            print(centroids)
            print(id)
            print(self.get_corners_rad(angle, rad, cen))
            print(self.get_corners_scale(centroids, cen))
            '''
            if DEBUG_MODE:
                preview = color_image.copy()
                cv2.circle(preview, np.array(cen, dtype=int), 5, (255, 0, 255), -20)
                aruco.drawDetectedMarkers(preview, [np.array([self.get_corners_rad(angle, rad, cen)],dtype=np.float32)], np.array([id], dtype=np.int))

                aruco.drawDetectedMarkers(preview, [np.array([self.get_corners_scale(centroids, cen)],dtype=np.float32)], np.array([id], dtype=np.int))
                aruco.drawDetectedMarkers(preview, [np.array([centroids],dtype=np.float32)], np.array([id], dtype=np.int))
                prev = cv2.resize(preview, (720, 1280))
                #cv2.imshow('marker', prev)
            '''

        elif len(markers) == 3:
            pass
            c_corner = markers[0]
            cube_angle = None
            for m in markers:
                if m['angle'] == c_corner['angle']:
                    cube_angle

        elif len(markers) == 2:
            print(len(markers))
        else:
            print("only 2 markers detected")


        return(centroids)


    def get_corners_rad(self, phi, r, centroid):
        print(phi, r, centroid)
        cul = np.array([r*np.cos((phi+225)* np.pi / 180), r*np.sin((phi+225)* np.pi / 180)]) + np.array(centroid)
        cur = np.array([r*np.cos((phi+315)* np.pi / 180), r*np.sin((phi+315)* np.pi / 180)]) + np.array(centroid)
        clr = np.array([r*np.cos((phi+45)* np.pi / 180), r*np.sin((phi+45)* np.pi / 180)]) + np.array(centroid)
        cll = np.array([r*np.cos((phi+135)* np.pi / 180), r*np.sin((phi+135)* np.pi / 180)]) + np.array(centroid)
        return np.array([cul,cur, clr, cll], dtype=np.float32)


    def get_corners_scale(self, centroids, cen):
        corners = ((np.array(centroids) - np.array(cen))/0.7) + np.array(cen)
        return corners


    def turn_90(self, phi, r, centroid):
        turn = np.array([r*np.cos((90)* np.pi / 180), r*np.sin((90)* np.pi / 180)])


    def detect_aruco(self, color_image):
        # Use the color image from the webcam to detect aruco markers
        aruco_marker = self.aruco_marker_detector.detect_aruco_marker(color_image)

        if len(aruco_marker) > 0:
            print(aruco_marker)

        ids = [marker["id"] for marker in aruco_marker]
        if len(ids) > 0:
            print(ids)
        for idx in ids:
            if idx in list(range(0, 7)):
                print(True)


    def detect_seedmarker(self, color_image):
        marker = detection.process(color_image.copy(), None, None)
        #print(marker)
        if len(marker) > 0:
            for m in marker:
                if m.get_lhds_name() in self.marker_list:
                    print(m.get_lhds_name())
                    print(self.marker_list.index(m.get_lhds_name()))





def main():
    CubeControl()
    sys.exit()


if __name__ == '__main__':
    main()
