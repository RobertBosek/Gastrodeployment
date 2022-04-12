import sys

from vigitia_interfaces.srv import BLEToggle

#TODO: define Movement service and update topic name
#from vigitia_interfaces.srv import VIGITIAMovement
from vigitia_interfaces.srv import BLEToggle as VIGITIAMovement
import rclpy
from rclpy.node import Node

from .ble_switch import BLESwitch


class BLESwitchNode(Node):

    def __init__(self):
        super().__init__('ble_switch_node')
        self.cli = self.create_client(BLEToggle, '/vigitia/toggle_camera')
        self.srv = self.create_service(VIGITIAMovement, '/vigitia/person_near', self.toggle_ble_callback)
        self.BLE_switch = BLESwitch(self)

        #TODO: remove loop
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service for /vigitia/toggle_camera not available, waiting...')
        self.req = BLEToggle.Request()

    def toggle_ble_callback(self, request, response):
        self.get_logger().info('Incoming request: %s scanning for cube switch poition' % ('start' if request.active else 'end'))

        if request.active:
            self.BLE_switch.start()
        else:
            self.BLE_switch.stop()
        response.status = 'successful'

        return response

    def send_request(self, status):
        self.req.active = status
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    ble_switch_node = BLESwitchNode()

    rclpy.spin(ble_switch_node)
    '''
    while rclpy.ok():
        rclpy.spin_once(ble_switch_node)
        if ble_switch_node.future.done():
            try:
                response = ble_switch_node.future.result()
            except Exception as e:
                ble_switch_node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                ble_switch_node.get_logger().info(
                    'Result of request: sent %s ; received %s' %
                    (ble_switch_node.req.active, response.status))
            break
    '''
    ble_switch_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
