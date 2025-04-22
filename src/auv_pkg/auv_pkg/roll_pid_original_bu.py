#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32MultiArray
from auv_custom_interfaces.msg import ServoMovementCommand
import time
import math

class WingPitchPIDNode(Node):
    def __init__(self):
        super().__init__('wing_roll_pid_node')
        
        # Parameters for PID control
        self.kp_roll = 0.5   # Proportional gain for roll
        self.ki_roll = 0.0   # Integral gain for roll
        self.kd_roll = 0.05  # Derivative gain for roll

        # State variables for PID control
        self.integral_roll = 0.0
        self.previous_error_roll = 0.0
        self.previous_time = self.get_clock().now()

        # Switch state
        self.declare_parameter('activate_on_start', True)
        self.control_active = self.get_parameter('activate_on_start').get_parameter_value().bool_value

        # Current wing servo angles
        self.current_left_wing_angle = 90.0  # Assuming center position is 90 degrees
        self.current_right_wing_angle = 90.0  # Assuming center position is 90 degrees

        # Latest roll value
        self.roll = 0.0
        # Target roll value (default is 0 for leveling, will maintain the last received value)
        self.target_roll = 0.0

        # Subscribers
        self.target_roll_subscriber_ = self.create_subscription(Float32MultiArray, 'pid_target_roll_angle', self.target_roll_callback, 10)
        self.euler_subscriber_ = self.create_subscription(Vector3, 'imu/euler', self.euler_callback, 10)
        self.switch_subscriber_ = self.create_subscription(Bool, 'wing_pitch_pid_switch', self.switch_callback, 10)
        self.servo_angles_subscriber_ = self.create_subscription(Float32MultiArray, 'current_servo_angles', self.servo_angles_callback, 10)

        # Publisher for wing servo commands
        self.wing_servo_publisher_ = self.create_publisher(ServoMovementCommand, 'servo_driver_commands', 10)

        # Timer for PID updates
        self.timer = self.create_timer(0.1, self.update_pid)  # 10Hz update rate

        if self.control_active:
            self.get_logger().info('Wing Pitch PID control activated on start (debug mode)')

    def target_roll_callback(self, msg):
        # Update the target roll value based on the incoming message
        if len(msg.data) > 0:
            self.target_roll = msg.data[0]
            self.get_logger().info(f'Target roll updated to: {self.target_roll:.2f} degrees. Maintaining this target until a new command is received.')

    def switch_callback(self, msg):
        # Update the control active state based on the switch message
        self.control_active = msg.data
        if self.control_active:
            self.get_logger().info('Wing Pitch PID control activated')
        else:
            self.get_logger().info('Wing Pitch PID control deactivated')

    def euler_callback(self, msg):
        # Store the latest roll value
        self.roll = msg.x  # Roll angle

    def servo_angles_callback(self, msg):
        # Update the current wing servo angles from the message
        self.current_left_wing_angle = msg.data[1]  # Wing pitch left servo angle
        self.current_right_wing_angle = msg.data[3]  # Wing pitch right servo angle

    def update_pid(self):
        if not self.control_active:
            return

        # Calculate the time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.previous_time = current_time

        # Calculate roll PID control
        roll_error = self.target_roll - self.roll  # Desired roll is set by the target_roll value
        self.integral_roll += roll_error * dt
        derivative_roll = (roll_error - self.previous_error_roll) / dt if dt > 0 else 0.0
        self.previous_error_roll = roll_error

        roll_correction = (self.kp_roll * roll_error) + (self.ki_roll * self.integral_roll) + (self.kd_roll * derivative_roll)

        # Calculate the target angles for left and right wing pitch servos
        left_wing_angle = 90.0 - roll_correction
        right_wing_angle = 90.0 + roll_correction

        # Limit the wing servo angles to avoid over-actuation
        left_wing_angle = max(60.0, min(120.0, left_wing_angle))  # Limit to 90 +/- 30 degrees
        right_wing_angle = max(60.0, min(120.0, right_wing_angle))  # Limit to 90 +/- 30 degrees

        # Create and publish the ServoMovementCommand message
        servo_command_msg = ServoMovementCommand()
        servo_command_msg.header.stamp = self.get_clock().now().to_msg()
        servo_command_msg.servo_numbers = [1, 3]  # Wing pitch left and right servos
        servo_command_msg.target_angles = [left_wing_angle, right_wing_angle]
        servo_command_msg.durations = [0.1, 0.1]  # Duration to reach target angle in seconds
        servo_command_msg.easing_algorithms = ['linear', 'linear']  # Using linear easing for both
        servo_command_msg.easing_in_factors = [0.0, 0.0]
        servo_command_msg.easing_out_factors = [0.0, 0.0]
        servo_command_msg.movement_type = 'roll_correction'
        servo_command_msg.deadline = (self.get_clock().now() + rclpy.duration.Duration(seconds=1)).to_msg()
        servo_command_msg.operational_mode = 'active_maneuvering'
        servo_command_msg.priority = 0  # Normal priority

        self.wing_servo_publisher_.publish(servo_command_msg)
        self.get_logger().info(f'Publishing Wing Servo Command - Left: {left_wing_angle:.2f}, Right: {right_wing_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    wing_pitch_pid_node = WingPitchPIDNode()
    rclpy.spin(wing_pitch_pid_node)

    # Clean up
    wing_pitch_pid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
