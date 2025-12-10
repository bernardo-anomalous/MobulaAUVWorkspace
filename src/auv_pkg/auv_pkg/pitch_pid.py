#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand

class TailPitchRollController(Node):
    def __init__(self):
        super().__init__('Pitch_PID')

        # === 1. TUNING PARAMETERS ===
        # Pitch Gains
        # Slightly increased kp to 2.0 (was 1.5) to maintain authority 
        # now that we removed the "overcompensation" hack.
        self.kp_pitch = 2.0 
        self.ki_pitch = 0.03
        self.kd_pitch = 0.01

        # Roll Gains
        self.kp_roll = 2.0  
        self.ki_roll = 0.05  
        self.kd_roll = 0.01  

        # === 2. BIOMIMETIC EFFICIENCY ===
        # Leaky Integrator: 0.98 means "forget 2% of the error every cycle"
        self.integrator_leak_pitch = 0.98
        self.integrator_leak_roll = 0.95

        # Output Smoothing (Low Pass Filter):
        # 0.7 = Fluid/Organic motion. 
        # 0.0 = Robotic/Twitchy.
        self.output_smoothing_factor = 0.7
        
        # Store previous servo states for smoothing
        self.prev_servo_left = 90.0
        self.prev_servo_right = 90.0

        # PID state
        self.integral_error_pitch = 0.0
        self.previous_error_pitch = 0.0
        self.integral_error_roll = 0.0
        self.previous_error_roll = 0.0

        # Limits
        self.integral_limit_pitch = 15.0
        self.integral_limit_roll = 20.0
        self.correction_limit_pitch = 30.0
        self.correction_limit_roll = 25.0

        # Internal Damping (Kept as is)
        self.damping_factor_pitch = 0.8
        self.damping_factor_roll = 0.8
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

        # Servo limits
        self.servo_limits = {
            4: {"min": 30.0, "max": 150.0},
            5: {"min": 30.0, "max": 150.0},
        }

        # Smoothing factor for IMU noise
        self.alpha = 0.8

        # Publisher
        self.servo_publisher = self.create_publisher(ServoMovementCommand, 'tail_commands', 10)

        # Subscribers
        self.create_subscription(Float32, 'target_pitch', self.target_pitch_callback, 10)
        self.create_subscription(Float32, 'target_roll', self.target_roll_callback, 10)
        self.create_subscription(Vector3, 'imu/euler', self.imu_callback, 10)
        self.create_subscription(Bool, 'tail_pid_active', self.pid_activation_callback, 10)

        # === 3. TIMING FIX ===
        # Timer set to 0.02 (50Hz) to match standard servo updates
        self.timer = self.create_timer(0.02, self.update_pid) 
        
        self.last_imu_time = self.get_clock().now()
        self.imu_timeout_sec = 1.0
        self.recovery_factor = 0.0 
        self.recovery_rate = 0.05 
        self.imu_data_valid = False
        self.last_update_time = self.get_clock().now()
        
        # Scaling factors
        self.left_pitch_scale = (90.0 - self.servo_limits[4]["min"])
        self.right_pitch_scale = (self.servo_limits[5]["max"] - 90.0)
        self.left_tail_gain = 1.0
        self.right_tail_gain = 1.0

        self.get_logger().info('Biomimetic Tail Controller (Optimized) Initialized')

    def target_pitch_callback(self, msg): self.target_pitch = msg.data
    def target_roll_callback(self, msg): self.target_roll = msg.data
    
    def imu_callback(self, msg):
        self.current_pitch = self.alpha * self.current_pitch + (1 - self.alpha) * msg.y
        self.current_roll = self.alpha * self.current_roll + (1 - self.alpha) * msg.x
        self.last_imu_time = self.get_clock().now()
        self.imu_data_valid = True

    def pid_activation_callback(self, msg):
        was_active = self.pid_active
        self.pid_active = msg.data
        if self.pid_active and not was_active:
            self.recovery_factor = 0.0
            self.integral_error_pitch = 0.0
            self.previous_error_pitch = 0.0
            self.integral_error_roll = 0.0
            self.previous_error_roll = 0.0

    def update_pid(self):
        # Safety / Activation Checks
        if not self.pid_active:
            if self.last_pid_active_state:
                self.get_logger().info("Tail PID deactivated.")
                self.last_pid_active_state = False
            return
        else:
            if not self.last_pid_active_state:
                self.get_logger().info("Tail PID activated.")
                self.last_pid_active_state = True

        now = self.get_clock().now()
        time_since_last_imu = (now.nanoseconds - self.last_imu_time.nanoseconds) / 1e9

        if time_since_last_imu > self.imu_timeout_sec:
            self.imu_data_valid = False
            self.recovery_factor = 0.0
            return

        if not self.imu_data_valid: return

        if self.recovery_factor < 1.0:
            self.recovery_factor += self.recovery_rate
            self.recovery_factor = min(self.recovery_factor, 1.0)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time
        if dt <= 0: return

        # ==========================
        # === PITCH PID (Biomimetic) ===
        # ==========================
        error_pitch = self.current_pitch - self.target_pitch
        
        # Deadband: Relax if error is tiny (< 1.0 deg)
        if abs(error_pitch) < 1.0: error_pitch = 0.0

        proportional_pitch = self.kp_pitch * error_pitch
        
        # Leaky Integrator
        self.integral_error_pitch = (self.integral_error_pitch * self.integrator_leak_pitch) + (error_pitch * dt)
        self.integral_error_pitch = max(-self.integral_limit_pitch, min(self.integral_limit_pitch, self.integral_error_pitch))
        
        integral_pitch = self.ki_pitch * self.integral_error_pitch
        derivative_pitch = self.kd_pitch * (error_pitch - self.previous_error_pitch) / dt
        self.previous_error_pitch = error_pitch

        correction_pitch = proportional_pitch + integral_pitch + derivative_pitch
        correction_pitch = self.damping_factor_pitch * self.previous_correction_pitch + (1 - self.damping_factor_pitch) * correction_pitch
        correction_pitch = max(-self.correction_limit_pitch, min(self.correction_limit_pitch, correction_pitch))
        self.previous_correction_pitch = correction_pitch

        # ==========================
        # === ROLL PID (Biomimetic) ===
        # ==========================
        error_roll = self.target_roll - self.current_roll
        
        # Deadband for roll
        if abs(error_roll) < 1.0: error_roll = 0.0

        proportional_roll = self.kp_roll * error_roll
        
        # Leaky Integrator
        self.integral_error_roll = (self.integral_error_roll * self.integrator_leak_roll) + (error_roll * dt)
        self.integral_error_roll = max(-self.integral_limit_roll, min(self.integral_limit_roll, self.integral_error_roll))
        
        integral_roll = self.ki_roll * self.integral_error_roll
        derivative_roll = self.kd_roll * (error_roll - self.previous_error_roll) / dt
        self.previous_error_roll = error_roll

        correction_roll = proportional_roll + integral_roll + derivative_roll
        correction_roll = self.damping_factor_roll * self.previous_correction_roll + (1 - self.damping_factor_roll) * correction_roll
        correction_roll = max(-self.correction_limit_roll, min(self.correction_limit_roll, correction_roll))
        self.previous_correction_roll = correction_roll

        # === Apply recovery factor ===
        correction_pitch *= self.recovery_factor
        correction_roll *= self.recovery_factor

        # === Scaling & Mixer ===
        pitch_component_left  = self.left_tail_gain  * (correction_pitch / self.correction_limit_pitch) * self.left_pitch_scale
        pitch_component_right = self.right_tail_gain * (correction_pitch / self.correction_limit_pitch) * self.right_pitch_scale

        roll_scale_left  = (90.0 - self.servo_limits[4]["min"])
        roll_scale_right = (self.servo_limits[5]["max"] - 90.0)

        # Raw calculated angles (Existing Mixer Logic Preserved)
        raw_left = 90.0 + pitch_component_left + (correction_roll * roll_scale_left / self.correction_limit_roll)
        raw_right = 90.0 - pitch_component_right + (correction_roll * roll_scale_right / self.correction_limit_roll)

        # ==========================
        # === OUTPUT SMOOTHING (LPF) ===
        # ==========================
        final_left = (self.output_smoothing_factor * self.prev_servo_left) + ((1.0 - self.output_smoothing_factor) * raw_left)
        final_right = (self.output_smoothing_factor * self.prev_servo_right) + ((1.0 - self.output_smoothing_factor) * raw_right)

        # Update History
        self.prev_servo_left = final_left
        self.prev_servo_right = final_right

        # Clamp
        final_left = max(self.servo_limits[4]["min"], min(self.servo_limits[4]["max"], final_left))
        final_right = max(self.servo_limits[5]["min"], min(self.servo_limits[5]["max"], final_right))

        # === Publish ===
        command = ServoMovementCommand()
        command.servo_numbers = [4, 5]
        command.target_angles = [final_left, final_right]
        
        # CRITICAL FIX: Duration set to 0.02 to match loop rate (50Hz)
        # This ensures immediate reaction, while the LPF above handles the smoothness.
        command.durations = [0.02, 0.02] 
        
        command.movement_type = "pid_control"
        self.servo_publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = TailPitchRollController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
