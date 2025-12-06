#!/usr/bin/env python3

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand, HoldRequest

class WingRollController(Node):
    def __init__(self):
        super().__init__('Roll_PID')

        # === 1. TUNING PARAMETERS ===
        self.kp_roll = 2.0  
        self.ki_roll = 0.05 
        self.kd_roll = 0.01 

        # === 2. BIOMIMETIC EFFICIENCY ===
        # Leaky Integrator: 0.98 means "forget 2% of the error every cycle"
        # This prevents the wings from fighting a lopsided battery forever.
        self.integrator_leak = 0.98

        # Output Smoothing (Low Pass Filter):
        # 0.7 means "70% previous position, 30% new command".
        # This eats the high-frequency jitter and makes the wings "flow".
        self.output_smoothing_factor = 0.7
        self.prev_servo_left = 135.0 
        self.prev_servo_right = 135.0

        # PID state
        self.integral_error_roll = 0.0
        self.previous_error_roll = 0.0

        # Limits
        self.integral_limit_roll = 20.0
        self.correction_limit_roll = 25.0
        
        # Internal Damping (We keep this as is)
        self.damping_factor_roll = 0.8
        self.previous_correction_roll = 0.0

        # Standard Init
        self.target_roll = 0.0
        self.current_roll = 0.0
        self.min_angle = 90.0
        self.max_angle = 180.0
        self.alpha = 0.8 

        # Activation flags
        self.pid_enabled = True
        self.hold_active = False
        self.pid_active = True
        self.last_pid_active_state = True
        self.local_hold_requested = False
        self.last_roll_error_deg = 0.0

        # Hold thresholds
        self.declare_parameter('hold_threshold_deg', 6.0)
        self.declare_parameter('release_threshold_deg', 3.0)
        self.hold_threshold_deg = max(0.0, float(self.get_parameter('hold_threshold_deg').value))
        self.release_threshold_deg = max(0.0, float(self.get_parameter('release_threshold_deg').value))

        # QoS
        hold_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.hold_state_publisher = self.create_publisher(HoldRequest, 'wing_hold_request', hold_qos)
        self.create_subscription(HoldRequest, 'wing_hold_request', self.hold_request_callback, hold_qos)
        self._last_published_hold_state = None

        self.servo_publisher = self.create_publisher(ServoMovementCommand, 'servo_driver_commands', 10)

        # Subscribers
        self.create_subscription(Float32, 'target_roll', self.target_roll_callback, 10)
        self.create_subscription(Vector3, 'imu/euler', self.imu_callback, 10)
        self.create_subscription(Bool, 'wing_pid_active', self.pid_activation_callback, 10)
        
        self.publish_hold_state()

        # === 3. CRITICAL TIMING FIX ===
        # Update rate matches Duration (30Hz = 0.033s)
        self.timer = self.create_timer(0.033, self.update_pid) 
        
        self.last_imu_time = self.get_clock().now()
        self.imu_timeout_sec = 1.0 
        self.recovery_factor = 0.0 
        self.recovery_rate = 0.05 
        self.imu_data_valid = False
        self.last_update_time = self.get_clock().now()

        self.get_logger().info('Biomimetic Wing Roll Controller (Optimized) Initialized')

    # === CALLBACKS ===
    def target_roll_callback(self, msg): self.target_roll = msg.data
    
    def imu_callback(self, msg):
        self.current_roll = self.alpha * self.current_roll + (1 - self.alpha) * msg.x
        self.last_imu_time = self.get_clock().now()
        self.imu_data_valid = True

    def pid_activation_callback(self, msg):
        self.pid_enabled = msg.data
        self.update_pid_activation_state()

    # ... [Hold Request Logic] ...
    def hold_request_callback(self, msg):
        if msg.hold == self.hold_active:
            if not msg.hold: self.local_hold_requested = False
            return
        self.hold_active = msg.hold
        if self.hold_active:
            self.local_hold_requested = False
        else:
            self.local_hold_requested = False
        self.update_pid_activation_state()

    def publish_hold_state(self):
        if self._last_published_hold_state == self.hold_active: return
        msg = HoldRequest()
        msg.hold = self.hold_active
        self.hold_state_publisher.publish(msg)
        self._last_published_hold_state = self.hold_active

    def update_pid_activation_state(self):
        hold_blocks_pid = self.hold_active and not self.local_hold_requested
        new_state = self.pid_enabled and not hold_blocks_pid
        if new_state and not self.pid_active:
            self.recovery_factor = 0.0
            self.integral_error_roll = 0.0
            self.previous_error_roll = 0.0
            self.previous_correction_roll = 0.0
        self.pid_active = new_state
        self.publish_hold_state()

    def _set_hold_state_from_local(self, hold_active: bool, error_deg: Optional[float] = None):
        if hold_active:
            if self.local_hold_requested or self.hold_active: return
            self.local_hold_requested = True
            self.hold_active = True
            self.update_pid_activation_state()
            return
        if not self.local_hold_requested: return
        self.local_hold_requested = False
        if self.hold_active:
            self.hold_active = False
            self.update_pid_activation_state()

    def _evaluate_hold_thresholds(self, error_deg: float):
        if not self.local_hold_requested and error_deg >= self.hold_threshold_deg:
            self._set_hold_state_from_local(True, error_deg)
            return
        if self.local_hold_requested and error_deg <= self.release_threshold_deg:
            self._set_hold_state_from_local(False, error_deg)

    def update_pid(self):
        # Time / Safety Checks
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

        absolute_error_roll = abs(self.target_roll - self.current_roll)
        self.last_roll_error_deg = absolute_error_roll
        self._evaluate_hold_thresholds(absolute_error_roll)

        pid_engaged = self.pid_active or self.local_hold_requested
        if not pid_engaged:
            if self.last_pid_active_state: self.last_pid_active_state = False
            return
        if not self.last_pid_active_state: self.last_pid_active_state = True

        # === PID CALCULATION ===
        error_roll = -(self.target_roll - self.current_roll)
        
        # Deadband: Ignore < 1.0 degree errors to save battery
        if abs(error_roll) < 1.0: error_roll = 0.0

        proportional_roll = self.kp_roll * error_roll
        
        # Leaky Integrator Logic
        self.integral_error_roll = (self.integral_error_roll * self.integrator_leak) + (error_roll * dt)
        self.integral_error_roll = max(-self.integral_limit_roll, min(self.integral_limit_roll, self.integral_error_roll))
        integral_roll = self.ki_roll * self.integral_error_roll
        
        derivative_roll = self.kd_roll * (error_roll - self.previous_error_roll) / dt
        self.previous_error_roll = error_roll

        correction_roll = proportional_roll + integral_roll + derivative_roll
        
        # Internal Damping
        correction_roll = self.damping_factor_roll * self.previous_correction_roll + (1 - self.damping_factor_roll) * correction_roll
        correction_roll = max(-self.correction_limit_roll, min(self.correction_limit_roll, correction_roll))
        self.previous_correction_roll = correction_roll

        correction_roll *= self.recovery_factor

        # === SERVO MAPPING ===
        self.min_angle = 90.0
        self.max_angle = 180.0
        center_angle = 135.0
        max_correction = 50.0 

        # Mixer logic preserved as verified by you
        raw_left = center_angle + (correction_roll / self.correction_limit_roll) * max_correction
        raw_right = center_angle + (correction_roll / self.correction_limit_roll) * max_correction

        # === OUTPUT SMOOTHING (LPF) ===
        # This makes the servo movement "organic"
        final_left = (self.output_smoothing_factor * self.prev_servo_left) + ((1.0 - self.output_smoothing_factor) * raw_left)
        final_right = (self.output_smoothing_factor * self.prev_servo_right) + ((1.0 - self.output_smoothing_factor) * raw_right)
        
        # Store for next cycle
        self.prev_servo_left = final_left
        self.prev_servo_right = final_right

        final_left = max(self.min_angle, min(self.max_angle, final_left))
        final_right = max(self.min_angle, min(self.max_angle, final_right))

        command = ServoMovementCommand()
        command.servo_numbers = [1, 3]
        command.target_angles = [final_left, final_right]
        
        # FIXED: Duration matches the loop speed (0.033s). 
        # The servo will now move instantly, relying on our LPF code for smoothness.
        command.durations = [0.033, 0.033] 
        
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
