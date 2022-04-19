from .CubeTrackerService import CubeDetector

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from vigitia_interfaces.msg import ArucoList, ArucoMarker


class CubeTrackerNode(Node):

    def __init__(self):
        super().__init__('aruco_subscriber')

        param_desc_debug = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='debug mode bool')
        param_desc_aruco_list = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of subscribing topic')
        param_desc_queue_length = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description='length of the queue')
        param_desc_cube_pose = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='name of publishing topic')

        self.declare_parameter('DEBUG_MODE', True, param_desc_debug)
        self.declare_parameter('topic_aruco_list', '/vigitia/aruco_list', param_desc_aruco_list)
        self.declare_parameter('queue_length', 10, param_desc_queue_length)
        self.declare_parameter('topic_cube_pose', '/vigitia/cube_pose', param_desc_cube_pose)

        self.create_subscription(msg_type=ArucoList,
                                 topic=self.get_parameter("topic_aruco_list").get_parameter_value().string_value,
                                 callback=self.listener_callback,
                                 qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

        self.publisher= self.create_publisher(msg_type=ArucoMarker,
                                              topic=self.get_parameter("topic_cube_pose").get_parameter_value().string_value,
                                              qos_profile=self.get_parameter("queue_length").get_parameter_value().integer_value)

        # Init VIGITIA Service
        self.cube_detector = CubeDetector(self)

        self.get_logger().info('initialized')


    def listener_callback(self, data):
        """
         Callback function.
         """
        detected_markers = []

        for aruco_marker in data.aruco_markers:
            detected_marker = {'id': aruco_marker.id,
                               'angle': aruco_marker.angle,
                               'corners': [[aruco_marker.x0, aruco_marker.y0],
                                           [aruco_marker.x1, aruco_marker.y1],
                                           [aruco_marker.x2, aruco_marker.y2],
                                           [aruco_marker.x3, aruco_marker.y3]],
                               'centroid': [aruco_marker.x_centroid, aruco_marker.y_centroid]}
            detected_markers.append(detected_marker)

        markers = self.cube_detector.preselect_markers(detected_markers)
        # cube geometrics are structured just like aruco marker information
        cube_geoms = self.cube_detector.get_cube_spatial_info(markers)

        if cube_geoms:
            cube_pose = ArucoMarker()
            cube_pose.id = int(cube_geoms['id'])
            cube_pose.angle = int(cube_geoms['angle'])

            cube_pose.x0 = float(cube_geoms['corners'][0][0])
            cube_pose.y0 = float(cube_geoms['corners'][0][1])
            cube_pose.x1 = float(cube_geoms['corners'][1][0])
            cube_pose.y1 = float(cube_geoms['corners'][1][1])
            cube_pose.x2 = float(cube_geoms['corners'][2][0])
            cube_pose.y2 = float(cube_geoms['corners'][2][1])
            cube_pose.x3 = float(cube_geoms['corners'][3][0])
            cube_pose.y3 = float(cube_geoms['corners'][3][1])

            cube_pose.x_centroid = float(cube_geoms['centroid'][0])
            cube_pose.y_centroid = float(cube_geoms['centroid'][1])

            print(cube_pose)
            self.publisher.publish(cube_pose)

            if self.get_parameter("DEBUG_MODE").get_parameter_value().bool_value:
                self.get_logger().info("Publishing Cube Pose %s" % str(cube_geoms))


def main(args=None):
    rclpy.init(args=args)
    cube_tracker_node = CubeTrackerNode()
    rclpy.spin(cube_tracker_node)
    cube_tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()