import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
from .include.controller import Controller
from .include.keyboard import Keyboard

class Input(Node):
    def __init__(self, input_type='keyboard'):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = None
        self.input_device = None
        self.drive = 0.0
        self.steer = 0.0

        if input_type == 'controller':
            self.timer = self.create_timer(timer_period, self.controller_callback)
            self.input_device = Controller()
        else:
            self.timer = self.create_timer(timer_period, self.keyboard_callback)
            self.input_device = Keyboard()

    def controller_callback(self):
        msg = Twist()

        msg.linear.x = 0.0  # Velocity in x (m/s)
        msg.angular.z = 0.0  # Angle of rotation in z (rad/s)

        axis, buttons = self.input_device.read_input()

        self.drive = round(axis[1] * -1, 2)
        self.steer = round(axis[3] * -1, 2) # Invert to match ROS standard (left is positive)
        msg.linear.x = self.drive if abs(self.drive) >= 0.05 else 0.0
        msg.angular.z = self.steer if abs(self.steer) >= 0.05 else 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'[{msg.linear.x}, {msg.angular.z}]')

    def keyboard_callback(self):
        msg = Twist()

        keys = self.input_device.read_input()
        up = keys[pygame.K_w]
        down = keys[pygame.K_s]
        left = keys[pygame.K_a]
        right = keys[pygame.K_d]
        shift = keys[pygame.K_LSHIFT]

        if shift:
            self.drive = 0.0
            self.steer = 0.0
        else:
            self.drive = float((up - down))
            self.steer = float((left - right)) # Invert steering for correct left/right mapping

        msg.linear.x = self.drive  # Velocity in x (m/s)
        msg.angular.z = self.steer  # Angle of rotation in z (rad/s)

        self.publisher_.publish(msg)
        self.get_logger().info(f'[{msg.linear.x}, {msg.angular.z}]')

    def quit(self):
        msg = Twist()
        msg.linear.x = 0.0  # Velocity in x (m/s)
        msg.angular.z = 0.0  # Angle of rotation in z (rad/s)

        self.publisher_.publish(msg)
        self.get_logger().info(f'[{msg.linear.x}, {msg.angular.z}]')
        self.input_device.close()


def main(args=None):
    try:
        rclpy.init(args=args)

        input_type = 'keyboard'
        if len(sys.argv) >= 2 and sys.argv[1] in ['keyboard', 'controller']:
            input_type = sys.argv[1]

        input_node = Input(input_type)

        rclpy.spin(input_node)

    finally:
        input_node.quit()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
