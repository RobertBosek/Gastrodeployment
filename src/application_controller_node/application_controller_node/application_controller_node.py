import cv2
import cv2.aruco as aruco
import numpy as np

from .ApplicationController import AppController

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from aruco_interfaces.msg import ArucoMarker

from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge


DEBUG_MODE = True

class ArucoSubscriber(Node):

    frame = None

    def __init__(self):
        super().__init__('aruco_subscriber')

        self.cv_bridge = CvBridge()

        # Set Parameters
        topic_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='detected aruco markers')
        self.declare_parameter('aruco_topic_parameter', '/aruco_list', topic_parameter)

        queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Length of the queue')
        self.declare_parameter('queue_length', 10, queue_length)

        # Init VIGITIA Service
        self.app_control = AppController()

        # Create the subscriber. This subscriber will receive an Image
        # from the /rgb/image_raw topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            ArucoMarker,  # Datentyp
            self.get_parameter("aruco_topic_parameter").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback_cubepose,
            self.get_parameter("queue_length").get_parameter_value().integer_value)

        topic_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Camera type')
        self.declare_parameter('topic_parameter', '/vigitia/rgb_table', topic_parameter)

        self.subscription = self.create_subscription(
            Image,  # Datentyp
            self.get_parameter("topic_parameter").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback_camera,
            self.get_parameter("queue_length").get_parameter_value().integer_value)



        #self.publisher = self.create_publisher(#msg_type=ArucoMarker,
                                               #topic='/cube_pose',
                                               #qos_profile=10)

    def listener_callback_camera(self, data):
        # Convert ROS Image message to OpenCV image
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)
        self.frame = current_frame

        cv2.waitKey(1)


    def listener_callback_cubepose(self, data):
        cubeside = data

        cube_pose = {'id': cubeside.id,
                    'angle': cubeside.angle,
                    'corners': [[cubeside.x0, cubeside.y0],
                                [cubeside.x1, cubeside.y1],
                                [cubeside.x2, cubeside.y2],
                                [cubeside.x3, cubeside.y3]],
                    'centroid': [cubeside.x_centroid, cubeside.y_centroid]}
        print(cube_pose)
        self.app_control.manage_input(cube_pose)

        # Display image
        if DEBUG_MODE:
            print("[Application Controller Node]: Received cube pose: ", cube_pose)
            if self.frame is not None:
                cube_detection = self.frame.copy()
                aruco.drawDetectedMarkers(cube_detection, [np.array([cube_pose['corners']], dtype=np.float32)],
                                              np.array([cube_pose['id']], dtype=np.int))
                cv2.imshow('cube_detection', cube_detection)



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    aruco_subscriber = ArucoSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(aruco_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()