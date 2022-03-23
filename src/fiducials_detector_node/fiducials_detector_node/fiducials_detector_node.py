import sys
import numpy as np
import cv2

from .FiducialsDetectorService import FiducialsDetectorService

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from aruco_interfaces.msg import ArucoList, ArucoMarker

from cv_bridge import CvBridge


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('tablearea_subscriber')

        self.cv_bridge = CvBridge()

        # Set Parameters
        topic_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Camera type')
        self.declare_parameter('topic_parameter', '/vigitia/rgb_table', topic_parameter)

        queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Length of the queue')
        self.declare_parameter('queue_length', 10, queue_length)

        # Init VIGITIA Service
        self.fiducials_detector = FiducialsDetectorService()

        # Create the subscriber. This subscriber will receive an Image
        # from the /rgb/image_raw topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,  # Datentyp
            self.get_parameter("topic_parameter").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback,
            self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.publisher = self.create_publisher(msg_type=ArucoList,
                                               topic='/aruco_list',
                                               qos_profile=10)

    def listener_callback(self, data):
        """
         Callback function.
         """
        # Display the message on the console
        # self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)
        frame_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGRA2GRAY)

        # Call the VIGITIA service
        aruco_markers = self.fiducials_detector.detect_fiducials(frame_gray)

        aruco_list = ArucoList()
        aruco_list.header = data.header

        for marker in aruco_markers:
            aruco_marker = ArucoMarker()
            aruco_marker.id = int(marker['id'])
            aruco_marker.angle = marker['angle']

            aruco_marker.x0 = float(marker['corners'][0][0])
            aruco_marker.y0 = float(marker['corners'][0][1])
            aruco_marker.x1 = float(marker['corners'][1][0])
            aruco_marker.y1 = float(marker['corners'][1][1])
            aruco_marker.x2 = float(marker['corners'][2][0])
            aruco_marker.y2 = float(marker['corners'][2][1])
            aruco_marker.x3 = float(marker['corners'][3][0])
            aruco_marker.y3 = float(marker['corners'][3][1])

            aruco_marker.x_centroid = float(marker['centroid'][0])
            aruco_marker.y_centroid = float(marker['centroid'][1])

            aruco_list.aruco_markers.append(aruco_marker)

        if len(aruco_markers) > 0:
            print(aruco_markers[0])
            self.publisher.publish(aruco_list)

        # Display image
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)

    # # convert a ROS Image message to an OpenCV BGRA image
    # def imgmsg_to_cv2(self, img_msg):
    #     dtype = np.dtype("uint8")  # Hardcode to 8 bits
    #     dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    #     # Since OpenCV works with bgr natively, we don't need to reorder the channels.
    #     image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 4), dtype=dtype, buffer=img_msg.data)
    #     # If the byte order is different between the message and the system.
    #     if img_msg.is_bigendian == (sys.byteorder == 'little'):
    #         image_opencv = image_opencv.byteswap().newbyteorder()
    #
    #     return image_opencv
    #
    # def cv2_to_imgmsg(self, cv_image):
    #     img_msg = Image()
    #     img_msg.height = cv_image.shape[0]
    #     img_msg.width = cv_image.shape[1]
    #     img_msg.encoding = "bgr8"
    #     img_msg.is_bigendian = 0
    #     img_msg.data = cv_image.tostring()
    #     img_msg.step = len(
    #         img_msg.data) // img_msg.height  # That double line is actually integer division, not a comment
    #     return img_msg


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
