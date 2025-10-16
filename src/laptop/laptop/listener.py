import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.input_listener = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.input_listener

    def listener_callback(self, msg):
        self.get_logger().info(f'[{msg.linear.x}, {msg.angular.z}]')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Listener()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()