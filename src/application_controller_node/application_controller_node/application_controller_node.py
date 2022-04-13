import cv2
import cv2.aruco as aruco
import numpy as np

from .ApplicationController import AppController

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from vigitia_interfaces.msg import ArucoMarker
from vigitia_interfaces.srv import SensorToggle
from vigitia_interfaces.srv import SensorToggle as VIGITIAMovement

from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge


DEBUG_MODE = True

class AppControllerNode(Node):

    frame = None

    def __init__(self):
        super().__init__('app_controller_node')

        self.cv_bridge = CvBridge()

        # Set Parameters
        topic_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='detected cube pose')
        self.declare_parameter('cube_pose_topic_parameter', '/cube_pose', topic_parameter)

        queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='Length of the queue')
        self.declare_parameter('queue_length', 10, queue_length)

        # Init VIGITIA Service
        self.app_control = AppController(self)

        # Create the subscriber. This subscriber will receive an Image
        # from the /rgb/image_raw topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            ArucoMarker,  # Datentyp
            self.get_parameter("cube_pose_topic_parameter").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback_cubepose,
            self.get_parameter("queue_length").get_parameter_value().integer_value)

        topic_parameter = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Camera type')
        self.declare_parameter('topic_parameter', '/vigitia/rgb_table', topic_parameter)

        self.subscription = self.create_subscription(
            Image,  # Datentyp
            self.get_parameter("topic_parameter").get_parameter_value().string_value,  # Name des Topics
            self.listener_callback_camera,
            self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.move_srv = self.create_service(VIGITIAMovement, '/vigitia/movement_toggle', self.movement_toggle_callback)
        self.ble_srv = self.create_service(SensorToggle, '/vigitia/sensor_toggle', self.sensor_toggle_callback)
        self.app_control.start()

    def sensor_toggle_callback(self, request, response):
        self.get_logger().info('Incoming request: %s handling cube poses' % ('start' if request.active else 'end'))

        if request.active:
            self.app_control.mode[1] = True
            if self.app_control.pose is not None:
                self.app_control.manage_input()
        else:
            self.app_control.mode[1] = False
            self.app_control.mode[2] = False

        response.status = 'successful'
        return response

    def movement_toggle_callback(self, request, response):
        self.get_logger().info('Incoming request: person %s' % ('approached' if request.active else 'left'))

        if request.active:
            self.app_control.mode[0] = True
        else:
            self.app_control.reset_to_defaults()
        response.status = 'successful'
        return response

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
        self.app_control.pose = cube_pose
        self.app_control.mode[2] = True

        # publish_cube_pose_to_topic()

        # Display image
        if DEBUG_MODE:
            print("[Application Controller Node]: Received cube pose: ", cube_pose)
            if self.frame is not None:
                cube_detection = self.frame.copy()
                aruco.drawDetectedMarkers(cube_detection, [np.array([cube_pose['corners']], dtype=np.float32)],
                                              np.array([cube_pose['id']], dtype=np.int))
                prev = cv2.resize(cube_detection, (1280, 720))
                cv2.imshow('cube_detection', prev)



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    app_controller_node = AppControllerNode()

    # Spin the node so the callback function is called.
    rclpy.spin(app_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    app_controller_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
