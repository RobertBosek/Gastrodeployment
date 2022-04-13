import sys

from vigitia_interfaces.srv import SensorToggle

#TODO: define Movement service and update topic name
#from vigitia_interfaces.srv import VIGITIAMovement
from vigitia_interfaces.srv import SensorToggle as VIGITIAMovement
import rclpy
from rclpy.node import Node

from .sensor_switch import SensorSwitch


class SensorSwitchNode(Node):

    def __init__(self):
        super().__init__('sensor_switch_node')
        self.cli = self.create_client(SensorToggle, '/vigitia/sensor_toggle')
        self.srv = self.create_service(VIGITIAMovement, '/vigitia/movement_toggle', self.movement_toggle_callback)
        self.sensor_switch = SensorSwitch(self)

        #TODO: remove loop
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service for /vigitia/toggle_camera not available, waiting...')
        self.req = SensorToggle.Request()

        self.get_logger().info('initialized, waiting for action to start scanning...')

    def movement_toggle_callback(self, request, response):
        self.get_logger().info('Incoming request: %s scanning for cube switch poition' % ('start' if request.active else 'end'))

        if request.active:
            self.sensor_switch.start()
        else:
            self.sensor_switch.stop()
        response.status = 'successful'
        return response

    def send_request(self, status):
        self.req.active = status
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    sensor_switch_node = SensorSwitchNode()

    rclpy.spin(sensor_switch_node)
    '''
    while rclpy.ok():
        rclpy.spin_once(sensor_switch_node)
        if sensor_switch_node.future.done():
            try:
                response = sensor_switch_node.future.result()
            except Exception as e:
                sensor_switch_node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                sensor_switch_node.get_logger().info(
                    'Result of request: sent %s ; received %s' %
                    (sensor_switch_node.req.active, response.status))
            break
    '''
    sensor_switch_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
