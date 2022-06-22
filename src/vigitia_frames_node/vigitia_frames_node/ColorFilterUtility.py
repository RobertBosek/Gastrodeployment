
import sys
import cv2
import numpy as np

from realsense_d435 import RealsenseD435Camera
from logitech_brio import LogitechBrio
from table_extraction_service import TableExtractionService

WINDOW_NAME = 'HSV Color filter utility'

PREVIEW_RESOLUTION = (1280, 720)  # (620, 360)


# Based on: https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
class ColorFilterUtility:
    """ ColorFilterUtility

    """

    low_H = 0
    low_S = 0
    low_V = 0
    high_H = 360 // 2
    high_S = 255
    high_V = 255

    low_H_name = 'Low HUE'
    low_S_name = 'Low SATURATION'
    low_V_name = 'Low VALUE'
    high_H_name = 'High HUE'
    high_S_name = 'High SATURATION'
    high_V_name = 'High VALUE'

    def __init__(self):

        self.camera_realsense = RealsenseD435Camera()
        self.camera_realsense.init_video_capture()
        self.camera_realsense.start()

        self.camera_brio = LogitechBrio()
        self.camera_brio.init_video_capture()
        self.camera_brio.start()

        self.table_extraction_service = TableExtractionService()

        cv2.namedWindow(WINDOW_NAME)
        cv2.createTrackbar(self.low_H_name, WINDOW_NAME, self.low_H, self.high_H, self.on_low_H_thresh_trackbar)
        cv2.createTrackbar(self.high_H_name, WINDOW_NAME, self.high_H, self.high_H, self.on_high_H_thresh_trackbar)
        cv2.createTrackbar(self.low_S_name, WINDOW_NAME, self.low_S, self.high_S, self.on_low_S_thresh_trackbar)
        cv2.createTrackbar(self.high_S_name, WINDOW_NAME, self.high_S, self.high_S, self.on_high_S_thresh_trackbar)
        cv2.createTrackbar(self.low_V_name, WINDOW_NAME, self.low_V, self.high_V, self.on_low_V_thresh_trackbar)
        cv2.createTrackbar(self.high_V_name, WINDOW_NAME, self.high_V, self.high_V, self.on_high_V_thresh_trackbar)

        self.loop()

    def loop(self):
        while True:
            color_image, depth_image, left_ir_image = self.camera_realsense.get_frames()  # Get frames from cameras
            color_image_additional = self.camera_brio.get_frames()  # Get frames from cameras

            # Only continue if needed frames are available
            if color_image is not None and color_image_additional is not None:

                
                # Pre-process camera frames
                color_image_table = self.table_extraction_service.extract_table_area(color_image, '/vigitia/rgb_full')
                color_image_additional_table = self.table_extraction_service.\
                    extract_table_area(color_image_additional, '/vigitia/rgb_additional_full')

                if color_image_table is not None and color_image_additional_table is not None:

                    hsv_frame_realsense = cv2.cvtColor(color_image_table, cv2.COLOR_BGR2HSV)
                    threshold_frame_realsense = cv2.inRange(hsv_frame_realsense, (self.low_H, self.low_S, self.low_V),
                                                            (self.high_H, self.high_S, self.high_V))

                    result_realsense = cv2.bitwise_and(color_image_table, color_image_table,
                                                       mask=threshold_frame_realsense)

                    hsv_frame_brio = cv2.cvtColor(color_image_additional_table, cv2.COLOR_BGR2HSV)
                    threshold_frame_brio = cv2.inRange(hsv_frame_brio,
                                                            (self.low_H, self.low_S, self.low_V),
                                                            (self.high_H, self.high_S, self.high_V))

                    result_brio = cv2.bitwise_and(color_image_additional_table, color_image_additional_table,
                                                       mask=threshold_frame_brio)

                    combined_frames_realsense = cv2.hconcat([cv2.resize(result_realsense, PREVIEW_RESOLUTION,
                                                                        interpolation=cv2.INTER_AREA),
                                                             cv2.resize(cv2.cvtColor(threshold_frame_realsense,
                                                                                     cv2.COLOR_GRAY2BGR),
                                                                        PREVIEW_RESOLUTION,
                                                                        interpolation=cv2.INTER_AREA)])

                    combined_frames_brio = cv2.hconcat(
                        [cv2.resize(result_brio, PREVIEW_RESOLUTION, interpolation=cv2.INTER_AREA),
                         cv2.resize(cv2.cvtColor(threshold_frame_brio, cv2.COLOR_GRAY2BGR), PREVIEW_RESOLUTION,
                                    interpolation=cv2.INTER_AREA)])

                    combined_frames = cv2.vconcat(([combined_frames_realsense, combined_frames_brio]))

                    cv2.imshow(WINDOW_NAME, combined_frames)

                    print('')
                    print('LOWER = np.array([{}, {}, {}])'.format(self.low_H, self.low_S, self.low_V))
                    print('UPPER = np.array([{}, {}, {}])'.format(self.high_H, self.high_S, self.high_V))

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                self.save_calibration_results()
                cv2.destroyAllWindows()
                break

    def save_calibration_results(self):
        print('saving calibration results')
        # Save the camera matrix and the distortion coefficients to given path/file.
        cv_file = cv2.FileStorage('../../../yellow_paper_calibration.yml', cv2.FILE_STORAGE_WRITE)
        cv_file.write('lower', np.array([self.low_H, self.low_S, self.low_V]))
        cv_file.write('upper', np.array([self.high_H, self.high_S, self.high_V]))
        cv_file.release()

    def on_low_H_thresh_trackbar(self, val):
        self.low_H = val
        self.low_H = min(self.high_H - 1, self.low_H)
        cv2.setTrackbarPos(self.low_H_name, WINDOW_NAME, self.low_H)

    def on_high_H_thresh_trackbar(self, val):
        self.high_H = val
        self.high_H = max(self.high_H, self.low_H + 1)
        cv2.setTrackbarPos(self.high_H_name, WINDOW_NAME, self.high_H)

    def on_low_S_thresh_trackbar(self, val):
        self.low_S = val
        self.low_S = min(self.high_S - 1, self.low_S)
        cv2.setTrackbarPos(self.low_S_name, WINDOW_NAME, self.low_S)

    def on_high_S_thresh_trackbar(self, val):
        self.high_S = val
        self.high_S = max(self.high_S, self.low_S + 1)
        cv2.setTrackbarPos(self.high_S_name, WINDOW_NAME, self.high_S)

    def on_low_V_thresh_trackbar(self, val):
        self.low_V = val
        self.low_V = min(self.high_V - 1, self.low_V)
        cv2.setTrackbarPos(self.low_V_name, WINDOW_NAME, self.low_V)

    def on_high_V_thresh_trackbar(self, val):
        self.high_V = val
        self.high_V = max(self.high_V, self.low_V + 1)
        cv2.setTrackbarPos(self.high_V_name, WINDOW_NAME, self.high_V)


def main():
    ColorFilterUtility()
    sys.exit()


if __name__ == '__main__':
    main()
