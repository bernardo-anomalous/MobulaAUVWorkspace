#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand

class WingRollController(Node):
    def __init__(self):
        super().__init__('Roll_PID')

        # PID Coefficients for Roll
        self.kp_roll = 2.0  # Proportional gain
        self.ki_roll = 0.05  # Integral gain
        # Slightly smaller derivative gain helps minimize oscillations
        self.kd_roll = 0.01 #deduce to smooth out the reaction

        # PID state for Roll
        self.integral_error_roll = 0.0
        self.previous_error_roll = 0.0

        # Anti-windup limit for roll integral
        self.integral_limit_roll = 20.0

        # Correction clamp limit for roll
        self.correction_limit_roll = 25.0

        # Damping factor for smooth roll corrections (increase for less jitter)
        self.damping_factor_roll = 0.8
        self.previous_correction_roll = 0.0

        # Target and current values
        self.target_roll = 0.0
        self.current_roll = 0.0

        # Servo limits
        self.min_angle = 90.0
        self.max_angle = 180.0

        # Smoothing factor for IMU noise (tune between 0.5 and 0.9)
        self.alpha = 0.8

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
        
        self.last_imu_time = self.get_clock().now()
        self.imu_timeout_sec = 1.0  # Time window to consider IMU data stale
        self.recovery_factor = 0.0  # Starts at zero, ramps up to 1.0
        self.recovery_rate = 0.05   # Speed of ramp-up (tune this value)
        self.imu_data_valid = False

        # Timestamp for PID update calculations
        self.last_update_time = self.get_clock().now()


        self.get_logger().info('Wing Roll Controller Node Initialized')

    def target_roll_callback(self, msg):
        self.target_roll = msg.data

    def imu_callback(self, msg):
        # Exponential smoothing to reduce IMU noise
        self.current_roll = self.alpha * self.current_roll + (1 - self.alpha) * msg.x
        self.last_imu_time = self.get_clock().now()
        self.imu_data_valid = True

    def pid_activation_callback(self, msg):
        was_active = self.pid_active
        self.pid_active = msg.data
        if self.pid_active and not was_active:
            # Reset recovery ramp when PID control is re-enabled
            self.recovery_factor = 0.0

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

        now = self.get_clock().now()
        time_since_last_imu = (now.nanoseconds - self.last_imu_time.nanoseconds) / 1e9

        # Check IMU data freshness
        if time_since_last_imu > self.imu_timeout_sec:
            self.imu_data_valid = False
            self.recovery_factor = 0.0  # Reset ramp-up
            return  # Skip PID calculation if no valid data

        if not self.imu_data_valid:
            return  # Extra safety check, though this line is optional

        # Ramp-up recovery factor after IMU comes back
        if self.recovery_factor < 1.0:
            self.recovery_factor += self.recovery_rate
            self.recovery_factor = min(self.recovery_factor, 1.0)

        # Time calculations
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        if dt <= 0:
            return

        # Roll PID calculations
        error_roll = -(self.target_roll - self.current_roll)
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

        # Apply recovery ramp factor here for smooth re-entry
        correction_roll *= self.recovery_factor

        # Servo range config
        self.min_angle = 90.0
        self.max_angle = 180.0
        center_angle = 135.0
        max_correction = 50.0  # max deviation from center in either direction

        # Scaled correction applied around center
        left_wing_angle = center_angle + (correction_roll / self.correction_limit_roll) * max_correction
        right_wing_angle = center_angle + (correction_roll / self.correction_limit_roll) * max_correction

        # Clamp servo angles
        left_wing_angle = max(self.min_angle, min(self.max_angle, left_wing_angle))
        right_wing_angle = max(self.min_angle, min(self.max_angle, right_wing_angle))

        # Publish servo commands
        command = ServoMovementCommand()
        command.servo_numbers = [1, 3]
        command.target_angles = [left_wing_angle, right_wing_angle]
        command.durations = [1.0, 1.0]
        command.movement_type = "pid_control"
        self.servo_publisher.publish(command)


        

def main(args=None):
    rclpy.init(args=args)
    node = WingRollController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
