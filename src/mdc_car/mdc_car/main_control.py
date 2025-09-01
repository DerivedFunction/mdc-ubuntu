import math
import rclpy
from rclpy.node import Node
import contextlib
from geometry_msgs.msg import Twist
from .include.maestro import Maestro
from typing import Iterator

# Maestro configuration
_MAESTRO_PATH = "/dev/ttyACM0"
_MAESTRO_BAUD = 9600
_CHANNEL_STEER = 1
_CHANNEL_DRIVE = 0

# Servo PWM values (in microseconds)
_NEUTRAL_DRIVE_VALUE = 6000
_BACKWARD_DRIVE_MIN = 5640
_FORWARD_DRIVE_MIN = 6200
_NEUTRAL_STEER_VALUE = 5800
_STEER_LEFT = 4200
_STEER_RIGHT = 7600
_STEER_LEFT_RANGE = _NEUTRAL_STEER_VALUE - _STEER_LEFT  # 1600
_STEER_RIGHT_RANGE = _STEER_RIGHT - _NEUTRAL_STEER_VALUE  # 1800

# Physical constants
_WHEELBASE_LENGTH = 0.3556  # Wheelbase in meters

# Get user input with validation
while True:
    try:
        LIMIT = float(input("Enter top speed (m/s): "))  # Changed to float for finer control
        if LIMIT <= 0:
            print("Speed must be positive.")
            continue
        break
    except ValueError:
        print("Please enter a valid number.")

while True:
    try:
        ACCEL = int(input("Enter acceleration (PWM units, 0-255): "))
        if ACCEL < 0 or ACCEL > 255:  # Typical Maestro speed range
            print("Acceleration must be between 0 and 255.")
            continue
        break
    except ValueError:
        print("Please enter a valid integer.")

# Linear equations: 
# Speed (m/s) = M * abs(drive - _NEUTRAL_DRIVE_VALUE) + B
_M_FORWARD = 0.002306997333  # Slope for forward motion
_B_FORWARD = -0.4474072417   # Intercept for forward motion
_TOP_FORWARD_DRIVE = int((LIMIT - _B_FORWARD) / _M_FORWARD + _NEUTRAL_DRIVE_VALUE)

_M_BACKWARD = 0.001061176966  # Slope for backward motion
_B_BACKWARD = -0.3531598483   # Intercept for backward motion
_TOP_BACKWARD_DRIVE = int(_NEUTRAL_DRIVE_VALUE - (LIMIT - _B_BACKWARD) / _M_BACKWARD)

# Avg Angle = M * abs(steer - _NEUTRAL_STEER_VALUE) + B
_M_RIGHT =  0.01269313305
_B_RIGHT = -0.3433476395

_M_LEFT = 0.0169201995
_B_LEFT = -4.554862843

class MainControl(Node):
    def __init__(self, mdev):
        super().__init__('main_control')
        self.mdev = mdev
        self.subscription = self.create_subscription(
            Twist,
            'input_node',
            self.listener_callback,
            10)  # Queue size of 10

    def listener_callback(self, msg):
        # Calculate drive value based on linear.x (forward/backward velocity)
        if msg.linear.x > 0:
            point = _FORWARD_DRIVE_MIN
            range_val = _TOP_FORWARD_DRIVE - _FORWARD_DRIVE_MIN
        elif msg.linear.x < 0:
            point = _BACKWARD_DRIVE_MIN
            range_val = _BACKWARD_DRIVE_MIN - _TOP_BACKWARD_DRIVE
        else:
            point = _NEUTRAL_DRIVE_VALUE
            range_val = 0

        drive = int(msg.linear.x * range_val + point)
        drive = max(_TOP_BACKWARD_DRIVE, min(_TOP_FORWARD_DRIVE, drive))  # Clamp drive value
        drive_offset = abs(drive - _NEUTRAL_DRIVE_VALUE)
        # Calculate steer value based on angular.z (steering angle)
        if msg.angular.z > 0:
            steer_range = _STEER_RIGHT_RANGE
        elif msg.angular.z < 0:
            steer_range = _STEER_LEFT_RANGE
        else:
            steer_range = 0
        steer = int(msg.angular.z * steer_range + _NEUTRAL_STEER_VALUE)
        steer = max(_STEER_LEFT, min(_STEER_RIGHT, steer))  # Clamp steer value
        steer_offset = abs(steer - _NEUTRAL_STEER_VALUE)
        # Calculate steering angle in degrees
        if steer < _NEUTRAL_STEER_VALUE:
            steer_angle = -(_M_LEFT * steer_offset + _B_LEFT)
        elif steer > _NEUTRAL_STEER_VALUE:
            steer_angle = _M_RIGHT * steer_offset + _B_RIGHT
        else:
            steer_angle = 0.0

        # Calculate speed in m/s
        if drive >= _FORWARD_DRIVE_MIN:  # Forward
            speed = _M_FORWARD * drive_offset + _B_FORWARD
        elif drive <= _BACKWARD_DRIVE_MIN:  # Reverse
            speed = _M_BACKWARD * drive_offset + _B_BACKWARD
        else:  # Deadband
            speed = 0.0

        radius = calculate_turning_radius(speed, steer_angle)

        # Send commands to Maestro
        try:
            self.mdev.set_target(_CHANNEL_DRIVE, drive)
            self.mdev.set_target(_CHANNEL_STEER, steer)
            self.get_logger().info(
                f'[Drive: {drive} | Speed: {speed:.2f} m/s | Steer: {steer_angle:.2f}° | Radius: {radius:.2f} m]'
            )
        except Exception as e:
            self.get_logger().error(f"Failed to set Maestro targets: {e}")

@contextlib.contextmanager
def make_maestro(*args, **kwargs) -> Iterator[Maestro]:
    try:
        m = Maestro(*args, **kwargs)
        yield m
    except Exception as e:
        print(f"Failed to initialize Maestro: {e}")
        raise
    finally:
        m.close()

def main(args=None):
    rclpy.init(args=args)
    with contextlib.ExitStack() as es:
        mdev = es.enter_context(make_maestro(_MAESTRO_PATH, _MAESTRO_BAUD))
        
        # Initialize steering
        mdev.set_target(_CHANNEL_STEER, _NEUTRAL_STEER_VALUE)
        mdev.set_speed(_CHANNEL_STEER, 0)  # No speed limit
        mdev.set_acceleration(_CHANNEL_STEER, 0)  # Instant movement

        # Initialize drive
        mdev.set_target(_CHANNEL_DRIVE, _NEUTRAL_DRIVE_VALUE)
        mdev.set_speed(_CHANNEL_DRIVE, ACCEL)  # User-defined acceleration
        mdev.set_acceleration(_CHANNEL_DRIVE, 0)  # Instant start

        main_control = MainControl(mdev)
        try:
            rclpy.spin(main_control)
        except KeyboardInterrupt:
            print("Shutting down gracefully...")

        main_control.destroy_node()
    rclpy.shutdown()

def calculate_turning_radius(speed, avg_angle_deg, mu=1.0):
    """
    Calculate the turning radius of an RC car, considering speed dynamically.
    
    Parameters:
    - speed (float): Speed in m/s.
    - avg_angle_deg (float): Average steering angle in degrees.
    - mu (float): Friction coefficient (default 1.0).
    
    Returns:
    - float: Turning radius in meters, or infinity for straight line.
    """
    angle_rad = avg_angle_deg * math.pi / 180
    
    if abs(avg_angle_deg) < 0.01 or speed == 0.0:
        return float('inf')  # Straight line
    
    # Static turning radius
    static_radius = _WHEELBASE_LENGTH / math.tan(max(min(angle_rad, math.pi/2 - 0.01), -math.pi/2 + 0.01))
    static_radius = abs(static_radius)
    
    # Required lateral acceleration
    lateral_acc = (speed ** 2) / static_radius
    
    # Maximum grip (g = 9.81 m/s^2)
    max_acc = mu * 9.81
    
    # Adjust radius if exceeding grip
    if lateral_acc > max_acc:
        return (speed ** 2) / max_acc
    return static_radius

if __name__ == '__main__':
    main()