import sys

from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node


'''
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
'''


class BLESensorNode(Node):

    def __init__(self):
        super().__init__('cube_activation_signal')
        self.cli = self.create_client(SetBool, '/ble_signal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()
        print(self.req)
        #init ble_sensor()
        #send request when cube activated/deactivated
        #self.send_request()

    def send_request(self):
        self.req.data = True
        print('sending req')
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    minimal_client = BLESensorNode()
    minimal_client.send_request()
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
