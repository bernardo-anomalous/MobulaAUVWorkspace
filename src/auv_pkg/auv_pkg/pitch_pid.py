#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand

class TailPitchRollController(Node):
    def __init__(self):
        super().__init__('Pitch_PID')
        



        # PID Coefficients for Pitch
        self.kp_pitch = 1.5
        self.ki_pitch = 0.03
        # Slightly smaller derivative gain to reduce jitter
        self.kd_pitch = 0.01

        # PID Coefficients for Roll
        self.kp_roll = 2.0  # Increased proportional gain for roll
        self.ki_roll = 0.05  # Increased integral gain for stronger adjustments
        self.kd_roll = 0.01  # Slightly reduced derivative gain for smoother roll

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
        # Increased damping factors help smooth out servo oscillations
        self.damping_factor_pitch = 0.8
        self.damping_factor_roll = 0.8

        # Previous corrections
        self.previous_correction_pitch = 0.0
        self.previous_correction_roll = 0.0

        # Activation flag
        self.pid_active = True
        self.last_pid_active_state = True

        # Target and current values
        self.target_pitch = 0.0
        self.current_pitch = 0.0
        self.target_roll = 0.0
        self.current_roll = 0.0

        # Servo limits // replace here after running calibration 
        self.servo_limits = {
            4: {"min": 30.0, "max": 150.0},
            5: {"min": 30.0, "max": 150.0},
        }#calibration seems to show mid point for servo 4 is 70, and for servo 5 is 105, there might be a gear ration changining the ranges in reality


        # Smoothing factor for IMU noise (typical range 0.5-0.9)
        self.alpha = 0.8

        # Publisher
        self.servo_publisher = self.create_publisher(ServoMovementCommand, 'tail_commands', 10)

        # Subscribers
        self.create_subscription(Float32, 'target_pitch', self.target_pitch_callback, 10)
        self.create_subscription(Float32, 'target_roll', self.target_roll_callback, 10)
        self.create_subscription(Vector3, 'imu/euler', self.imu_callback, 10)
        self.create_subscription(Bool, 'tail_pid_active', self.pid_activation_callback, 10)

        # Timer for PID updates
        self.timer = self.create_timer(0.02, self.update_pid)  # Update at ~30 Hz
        
        self.last_imu_time = self.get_clock().now()
        self.imu_timeout_sec = 1.0  # IMU considered stale after 1 second
        self.recovery_factor = 0.0   # Starts at zero, ramps up to 1.0
        self.recovery_rate = 0.05    # Ramp speed for recovery
        self.imu_data_valid = False

        # Timestamp for PID update calculations
        self.last_update_time = self.get_clock().now()
        
                # Pitch scaling factors for asymmetric tail response
        self.left_pitch_scale = (90.0 - self.servo_limits[4]["min"])
        self.right_pitch_scale = (self.servo_limits[5]["max"] - 90.0)
        # Scale for right tail servo

        # Optional gain factors in case mechanical linkages are not symmetric
        self.left_tail_gain = 1.0   # Adjust this if the left tail needs more/less authority
        self.right_tail_gain = 1.0  # Adjust this if the right tail needs more/less authority


        self.get_logger().info('Tail Pitch and Roll Controller Node Initialized')

    def target_pitch_callback(self, msg):
        self.target_pitch = msg.data

    def target_roll_callback(self, msg):
        self.target_roll = msg.data

    def imu_callback(self, msg):
        # Exponential smoothing to reduce IMU noise
        self.current_pitch = self.alpha * self.current_pitch + (1 - self.alpha) * msg.y
        self.current_roll = self.alpha * self.current_roll + (1 - self.alpha) * msg.x
        self.last_imu_time = self.get_clock().now()
        self.imu_data_valid = True

    def pid_activation_callback(self, msg):
        was_active = self.pid_active
        self.pid_active = msg.data
        if self.pid_active and not was_active:
            # Reset recovery ramp when PID control is re-enabled
            self.recovery_factor = 0.0
            self.integral_error_pitch = 0.0
            self.previous_error_pitch = 0.0
            self.previous_correction_pitch = 0.0
            self.integral_error_roll = 0.0
            self.previous_error_roll = 0.0
            self.previous_correction_roll = 0.0

    def update_pid(self):
        if not self.pid_active:
            if self.last_pid_active_state:
                self.get_logger().info("Tail PID controllers deactivated.")
                self.last_pid_active_state = False
            return
        else:
            if not self.last_pid_active_state:
                self.get_logger().info("Tail PID controllers activated.")
                self.last_pid_active_state = True

        now = self.get_clock().now()
        time_since_last_imu = (now.nanoseconds - self.last_imu_time.nanoseconds) / 1e9

        # Check IMU data freshness
        if time_since_last_imu > self.imu_timeout_sec:
            self.imu_data_valid = False
            self.recovery_factor = 0.0  # Reset ramp-up
            return  # Skip PID calculation if no valid data

        if not self.imu_data_valid:
            return  # Extra safety check

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

        # === Pitch PID ===
        error_pitch = self.target_pitch - self.current_pitch
        if abs(error_pitch) < 10.0:  # Overcompensation zone for pitch
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

        # === Roll PID (tail differential contribution) ===
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

        # === Apply recovery factor for smooth ramp-up ===
        correction_pitch *= self.recovery_factor
        correction_roll *= self.recovery_factor

        # === Proper pitch scaling with gains and asymmetric factors ===
        pitch_component_left  = self.left_tail_gain  * (correction_pitch / self.correction_limit_pitch) * self.left_pitch_scale
        pitch_component_right = self.right_tail_gain * (correction_pitch / self.correction_limit_pitch) * self.right_pitch_scale

        # === Roll scaling adjusted to match servo ranges ===
        roll_scale_left  = (90.0 - self.servo_limits[4]["min"])#adjusting with new mid points
        roll_scale_right = (self.servo_limits[5]["max"] - 90.0)


        # === Apply scaled roll correction ===
        left_tail_angle  = 90.0 + pitch_component_left  + (correction_roll * roll_scale_left / self.correction_limit_roll)
        right_tail_angle = 90.0 - pitch_component_right + (correction_roll * roll_scale_right / self.correction_limit_roll)

        # === Clamp servo angles ===
        left_tail_angle  = max(self.servo_limits[4]["min"], min(self.servo_limits[4]["max"], left_tail_angle))
        right_tail_angle = max(self.servo_limits[5]["min"], min(self.servo_limits[5]["max"], right_tail_angle))

        # === Publish the servo commands ===
        command = ServoMovementCommand()
        command.servo_numbers = [4, 5]
        command.target_angles = [left_tail_angle, right_tail_angle]
        command.durations = [1.0, 1.0]
        command.movement_type = "pid_control"
        self.servo_publisher.publish(command)




        # Logging
        #self.get_logger().info(f'Pitch Error: {error_pitch:.2f}, Roll Error: {error_roll:.2f}')
        #self.get_logger().info(f'Left Tail Angle: {left_tail_angle:.2f}, Right Tail Angle: {right_tail_angle:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TailPitchRollController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
