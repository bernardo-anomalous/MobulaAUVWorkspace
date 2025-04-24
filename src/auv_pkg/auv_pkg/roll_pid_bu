#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand

class WingRollController(Node):
    def __init__(self):
        super().__init__('wing_roll_controller')

        # PID Coefficients for Roll
        self.kp_roll = 2.0  # Proportional gain
        self.ki_roll = 0.05  # Integral gain
        self.kd_roll = 0.8  # Derivative gain

        # PID state for Roll
        self.integral_error_roll = 0.0
        self.previous_error_roll = 0.0

        # Anti-windup limit for roll integral
        self.integral_limit_roll = 20.0

        # Correction clamp limit for roll
        self.correction_limit_roll = 25.0

        # Damping factor for smooth roll corrections
        self.damping_factor_roll = 0.5
        self.previous_correction_roll = 0.0

        # Target and current values
        self.target_roll = 0.0
        self.current_roll = 0.0

        # Servo limits
        self.min_angle = 0.0
        self.max_angle = 60.0

        # Activation flag
        self.pid_active = True
        self.last_pid_active_state = True

        # Publisher
        self.servo_publisher = self.create_publisher(ServoMovementCommand, 'servo_driver_commands', 10)

        # Subscribers
        self.create_subscription(Float32, 'target_roll', self.target_roll_callback, 10)
        self.create_subscription(Vector3, 'imu/euler', self.imu_callback, 10)
        self.create_subscription(Bool, 'wing_pid_active', self.pid_activation_callback, 10)

        # Timer for PID updates
        self.timer = self.create_timer(0.033, self.update_pid)  # Update at ~30 Hz

        self.get_logger().info('Wing Roll Controller Node Initialized')

    def target_roll_callback(self, msg):
        self.target_roll = msg.data

    def imu_callback(self, msg):
        # Extract current roll from IMU data
        self.current_roll = msg.x  # Assuming roll is in the x field of the IMU message

    def pid_activation_callback(self, msg):
        self.pid_active = msg.data

    def update_pid(self):
        if not self.pid_active:
            if self.last_pid_active_state:
                self.get_logger().info("Wing PID controllers deactivated.")
                self.last_pid_active_state = False
            return
        else:
            if not self.last_pid_active_state:
                self.get_logger().info("Wing PID controllers activated.")
                self.last_pid_active_state = True

        # Time calculations
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds / 1e9)

        if dt <= 0:
            return

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

        # Servo angle adjustments based on roll correction
        left_wing_angle = 30.0 + (correction_roll / self.correction_limit_roll) * 30.0  # Center at 30.0
        right_wing_angle = 30.0 + (correction_roll / self.correction_limit_roll) * 30.0

        # Clamp servo angles
        left_wing_angle = max(self.min_angle, min(self.max_angle, left_wing_angle))
        right_wing_angle = max(self.min_angle, min(self.max_angle, right_wing_angle))

        # Publish servo commands
        command = ServoMovementCommand()
        command.servo_numbers = [1, 3]  # Assuming servos 1 and 2 control the wings
        command.target_angles = [left_wing_angle, right_wing_angle]
        command.durations = [0.033, 0.033]
        self.servo_publisher.publish(command)

        

def main(args=None):
    rclpy.init(args=args)
    node = WingRollController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
