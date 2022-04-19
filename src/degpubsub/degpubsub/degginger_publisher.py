from .DeggingerEventService import DeggingerEventService
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# TODO: use different msg type as soon as design layout of "events-app" is set
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.event_service = DeggingerEventService('https://www.regensburg.de/degginger/programm')

        self.publisher_ = self.create_publisher(String, '/degginer_events', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = str(self.event_service.get_events())
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
