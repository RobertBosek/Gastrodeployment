from vigitia_interfaces.srv import BLEToggle

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(BLEToggle, 'add_two_ints', self.add_two_ints_callback)
        c = 0
        while c < 1000000:
            c+=1
            print(c)

    def add_two_ints_callback(self, request, response):
        response.status = str(request.active)
        self.get_logger().info('Incoming request\nactive: %s' % (request.active))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()