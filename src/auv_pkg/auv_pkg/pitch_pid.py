#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from auv_custom_interfaces.msg import ServoMovementCommand

class TailPitchRollController(Node):
    def __init__(self):
        super().__init__('tail_pitch_roll_controller')

        # PID Coefficients for Pitch
        self.kp_pitch = 1.5
        self.ki_pitch = 0.03
        self.kd_pitch = 0.6

        # PID Coefficients for Roll
        self.kp_roll = 2.0  # Increased proportional gain for roll
        self.ki_roll = 0.05  # Increased integral gain for stronger adjustments
        self.kd_roll = 0.8  # Increased derivative gain for faster response

        # Overcompensation factor for pitch
        self.overcompensation_factor_pitch = 2.0

        # PID state for Pitch
        self.integral_error_pitch = 0.0
        self.previous_error_pitch = 0.0

        # PID state for Roll
        self.integral_error_roll = 0.0
        self.previous_error_roll = 0.0

        # Anti-windup limits
        self.integral_limit_pitch = 15.0
        self.integral_limit_roll = 20.0  # Increased limit for roll

        # Correction clamp limits
        self.correction_limit_pitch = 30.0
        self.correction_limit_roll = 25.0  # Increased limit for roll corrections

        # Damping factors
        self.damping_factor_pitch = 0.6
        self.damping_factor_roll = 0.5  # Reduced damping for quicker response to roll

        # Previous corrections
        self.previous_correction_pitch = 0.0
        self.previous_correction_roll = 0.0

        # Target and current values
        self.target_pitch = 0.0
        self.current_pitch = 0.0
        self.target_roll = 0.0
        self.current_roll = 0.0

        # Servo limits
        self.min_angle = 40.0
        self.max_angle = 140.0

        # Smoothing factor for IMU noise
        self.alpha = 0.8

        # Publisher
        self.servo_publisher = self.create_publisher(ServoMovementCommand, 'tail_commands', 10)

        # Subscribers
        self.create_subscription(Float32, 'target_pitch', self.target_pitch_callback, 10)
        self.create_subscription(Float32, 'target_roll', self.target_roll_callback, 10)
        self.create_subscription(Vector3, 'imu/euler', self.imu_callback, 10)

        # Timer for PID updates
        self.timer = self.create_timer(0.033, self.update_pid)  # Update at ~30 Hz

        self.get_logger().info('Tail Pitch and Roll Controller Node Initialized')

    def target_pitch_callback(self, msg):
        self.target_pitch = msg.data

    def target_roll_callback(self, msg):
        self.target_roll = msg.data

    def imu_callback(self, msg):
        # Exponential smoothing filter to reduce IMU noise
        self.current_pitch = self.alpha * self.current_pitch + (1 - self.alpha) * msg.y
        self.current_roll = self.alpha * self.current_roll + (1 - self.alpha) * msg.x

    def update_pid(self):
        # Time calculations
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds / 1e9)

        if dt <= 0:
            return

        # Pitch PID
        error_pitch = self.target_pitch - self.current_pitch
        if abs(error_pitch) < 10.0:  # Overcompensation threshold for pitch
            error_pitch *= self.overcompensation_factor_pitch

        proportional_pitch = self.kp_pitch * error_pitch
        self.integral_error_pitch += error_pitch * dt
        self.integral_error_pitch = max(-self.integral_limit_pitch, min(self.integral_limit_pitch, self.integral_error_pitch))
        integral_pitch = self.ki_pitch * self.integral_error_pitch
        derivative_pitch = self.kd_pitch * (error_pitch - self.previous_error_pitch) / dt
        self.previous_error_pitch = error_pitch

        correction_pitch = proportional_pitch + integral_pitch + derivative_pitch
        correction_pitch = self.damping_factor_pitch * self.previous_correction_pitch + (1 - self.damping_factor_pitch) * correction_pitch
        correction_pitch = max(-self.correction_limit_pitch, min(self.correction_limit_pitch, correction_pitch))
        self.previous_correction_pitch = correction_pitch

        # Roll PID
        error_roll = self.target_roll - self.current_roll

        proportional_roll = self.kp_roll * error_roll
        self.integral_error_roll += error_roll * dt
        self.integral_error_roll = max(-self.integral_limit_roll, min(self.integral_limit_roll, self.integral_error_roll))
        integral_roll = self.ki_roll * self.integral_error_roll
        derivative_roll = self.kd_roll * (error_roll - self.previous_error_roll) / dt
        self.previous_error_roll = error_roll

        correction_roll = proportional_roll + integral_roll + derivative_roll
        correction_roll = self.damping_factor_roll * self.previous_correction_roll + (1 - self.damping_factor_roll) * correction_roll
        correction_roll = max(-self.correction_limit_roll, min(self.correction_limit_roll, correction_roll))
        self.previous_correction_roll = correction_roll

        # Combine corrections for servos with reversed roll logic for left servo
        left_tail_angle = 90.0 + (correction_pitch / self.correction_limit_pitch) * (90.0 - self.min_angle)+ correction_roll  #change the + or + on both lines to correct the respective angle direction of response
        right_tail_angle = 90.0 - (correction_pitch / self.correction_limit_pitch) * (self.max_angle - 90.0) + correction_roll

        # Clamp servo angles
        left_tail_angle = max(self.min_angle, min(self.max_angle, left_tail_angle))
        right_tail_angle = max(self.min_angle, min(self.max_angle, right_tail_angle))

        # Publish servo commands
        command = ServoMovementCommand()
        command.servo_numbers = [4, 5]
        command.target_angles = [left_tail_angle, right_tail_angle]
        command.durations = [0.033, 0.033]
        self.servo_publisher.publish(command)

        # Logging
        self.get_logger().info(f'Pitch Error: {error_pitch:.2f}, Roll Error: {error_roll:.2f}')
        self.get_logger().info(f'Left Tail Angle: {left_tail_angle:.2f}, Right Tail Angle: {right_tail_angle:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TailPitchRollController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
