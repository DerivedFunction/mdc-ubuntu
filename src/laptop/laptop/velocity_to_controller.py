#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class VelocityToController(Node):
    def __init__(self):
        super().__init__('velocity_to_controller')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('joystick_scale', 1.0)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.joystick_scale = self.get_parameter('joystick_scale').value

        # Subscribe to Nav2's velocity commands
        self.velocity_subscription = self.create_subscription(
            Twist,
            '/nav2/cmd_vel',
            self.velocity_callback,
            10)

        # Publisher for controller-like outputs
        self.controller_publisher = self.create_publisher(
            Float32MultiArray,
            '/laptop/controller_output',
            10)

    def velocity_callback(self, msg):
        # Convert velocity commands to controller-like outputs
        # Assuming a simple differential drive model:
        # Left analog stick Y axis controls linear velocity
        # Right analog stick X axis controls angular velocity
        
        # Normalize velocities to [-1, 1] range like a joystick
        linear_y = (msg.linear.x / self.max_linear_speed) * self.joystick_scale
        angular_x = (msg.angular.z / self.max_angular_speed) * self.joystick_scale
        
        # Clamp values to [-1, 1]
        linear_y = max(min(linear_y, 1.0), -1.0)
        angular_x = max(min(angular_x, 1.0), -1.0)
        
        # Create controller output message
        # Using the same format as your controller.py
        # Assuming we need an array of axis values
        controller_msg = Float32MultiArray()
        
        # Create an array of zeros for all axes
        axes = [0.0] * 6  # Assuming 6 axes like a typical gamepad
        
        # Set the relevant axes
        # Typically axis 1 is left stick Y and axis 2 is right stick X
        axes[1] = -linear_y  # Negative because forward is typically negative on gamepads
        axes[2] = angular_x
        
        controller_msg.data = axes
        
        # Publish the controller-like output
        self.controller_publisher.publish(controller_msg)


def main(args=None):
    rclpy.init(args=args)
    
    velocity_to_controller = VelocityToController()
    
    try:
        rclpy.spin(velocity_to_controller)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_to_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
