#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray, String
from auv_custom_interfaces.msg import ServoMovementCommand, HoldRequest
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from adafruit_motor.servo import Servo
from adafruit_pca9685 import PCA9685
import board
import busio
import os
import sys
import time
from collections import deque
import threading
import math


class ServoDriverNode(LifecycleNode):
    def __init__(self):
        super().__init__('servo_driver_node')
        self.get_logger().info('Servo Driver Node initialized.')

        #Params
        self.declare_parameter('glide_position', [90.0, 135.0, 90.0, 135.0, 90.0, 90.0])
        self.declare_parameter('update_rate_hz', 60.0) 
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('debug_logging', False)
        self.declare_parameter('stubborn', True)

        # --- NEW PARAMETERS FOR INRUSH PROTECTION ---
        # Delay (seconds) between waking each servo to prevent Brownouts
        self.declare_parameter('startup_stagger_delay', 0.25) 
        # Max speed (degrees per second) to smooth out jerky movements
        self.declare_parameter('max_slew_rate_deg_s', 120.0) 
        # ---------------------------------------------

        self.simulation_mode = False
        self.debug_logging = False
        self.i2c = None
        self.pca = None
        self.servos = {}
        self.current_angles = []
        self.angle_tolerance_deg = 0.5  # Reduced slightly for smoother control
        self.last_target_angles = [90.0] * 6
        self.stubborn = bool(self.get_parameter('stubborn').value)

        self.angles_publisher = None
        self.command_subscriber = None
        self.tail_command_subscriber = None
        self.heartbeat_timer = None
        self.refresh_timer = None
        self.busy_status_publisher = self.create_publisher(String, 'servo_driver_status', 10)
        self.last_published_status = None
        self.executing_movement = False
        self.current_movement_type = ''
        self.hold_active = False
        self.hold_state_subscription = None
        self._last_base_status_message = None
        self._publish_busy_status("starting up")

        # Lifecycle state tracking
        self.lifecycle_state_publisher = self.create_publisher(String, 'servo_driver/lifecycle_state', 10)
        self.last_lifecycle_state = 'unconfigured'
        self.lifecycle_state_timer = self.create_timer(2.0, self._publish_lifecycle_state)

        # Failure tracking
        self.failure_timestamps = deque()
        self.max_failures = 3
        self.failure_window_seconds = 30  
        self.target_lock = threading.Lock()
        self.reset_attempts = 0
        self.max_reset_attempts = 3

        # Track lifecycle restoration after a restart
        self._restoration_target = os.environ.pop('SERVO_DRIVER_TARGET_STATE', None)
        self._restoration_attempted = False
        self._restoration_timer = self.create_timer(1.0, self._attempt_restore_lifecycle_state)

    def restart_process(self):
        if not self.stubborn:
            self.get_logger().error("Stubborn mode disabled; not restarting.")
            return
        self.get_logger().error("Servo Driver Node restarting itself now...")
        os.environ['SERVO_DRIVER_TARGET_STATE'] = self.last_lifecycle_state
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def reset_i2c_bus(self) -> bool:
        self.get_logger().error("Attempting soft I2C reset...")
        try:
            if self.pca:
                try:
                    self.pca.deinit()
                except Exception as e:
                    self.get_logger().error(f"Error deinitializing PCA9685: {e}")
            if self.i2c:
                try:
                    self.i2c.deinit()
                except Exception as e:
                    self.get_logger().error(f"Error deinitializing I2C: {e}")
            time.sleep(0.1)

            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c)
            self.pca.frequency = 60
            time.sleep(0.5)
            self.servos = {
                0: Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500),
                1: Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500),
                2: Servo(self.pca.channels[2], min_pulse=500, max_pulse=2500),
                3: Servo(self.pca.channels[3], min_pulse=500, max_pulse=2500),
                4: Servo(self.pca.channels[4], min_pulse=500, max_pulse=2500),
                5: Servo(self.pca.channels[5], min_pulse=500, max_pulse=2500),
            }
            self.move_servos_to_glide_position()
            self.get_logger().error("Soft reset successful.")
            return True
        except Exception as e:
            self.get_logger().error(f"Soft reset failed: {e}")
            return False

    def record_failure_and_check_restart(self):
        if not self.stubborn:
            return
        current_time = time.time()
        self.failure_timestamps.append(current_time)
        while self.failure_timestamps and (current_time - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()

        if len(self.failure_timestamps) >= self.max_failures:
            self.get_logger().error(
                f"Exceeded {self.max_failures} failures within {self.failure_window_seconds} seconds. Attempting soft reset..."
            )
            if self.reset_attempts < self.max_reset_attempts and self.reset_i2c_bus():
                self.failure_timestamps.clear()
                self.reset_attempts += 1
            else:
                self.get_logger().fatal("Soft reset failed or limit reached. Restarting process...")
                time.sleep(2)
                self.restart_process()

    def _stop_timers(self):
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        if self.refresh_timer:
            self.refresh_timer.cancel()
            self.refresh_timer = None

    def _stop_pwm(self):
        if not self.simulation_mode and self.pca:
            try:
                for channel in self.pca.channels:
                    channel.duty_cycle = 0
                self.get_logger().info(
                    'PWM signals stopped. Servos should be limp (if supported by model).'
                )
            except Exception as e:
                self.get_logger().error(f'Error while setting duty cycles to 0: {e}')

    def _release_hardware(self):
        self._stop_pwm()
        if not self.simulation_mode and self.pca:
            try:
                self.pca.deinit()
            except Exception as e:
                self.get_logger().error(f'Error deinitializing PCA9685: {e}')
            self.pca = None
        if self.i2c:
            try:
                self.i2c.deinit()
            except Exception:
                pass
            self.i2c = None

    def start_refresh_loop(self):
        update_rate_hz = self.get_parameter('update_rate_hz').value
        if update_rate_hz <= 0:
            update_rate_hz = 60.0
        
        self.refresh_timer = self.create_timer(1.0 / update_rate_hz, self.refresh_servo_positions)
        self.get_logger().info(f"Started refresh loop at {update_rate_hz} Hz.")

    def refresh_servo_positions(self):
        """
        Main hardware loop with Slew Rate Limiting.
        Prevent brownouts during operation by clamping sudden moves.
        """
        try:
            # 1. Setup Slew Limiting Math
            update_rate = self.get_parameter('update_rate_hz').value
            max_rate_deg_s = self.get_parameter('max_slew_rate_deg_s').value
            
            # Max degrees per tick (e.g., 120 deg/s / 60Hz = 2.0 degrees max per tick)
            if update_rate > 0:
                max_step_per_tick = max_rate_deg_s / update_rate
            else:
                max_step_per_tick = 2.0 # Fallback safety

            # Snapshot targets
            targets_snapshot = []
            with self.target_lock:
                targets_snapshot = list(enumerate(self.last_target_angles))

            # Hardware I/O
            for servo_number, target_angle in targets_snapshot:
                
                # Validation
                if target_angle is None or math.isnan(target_angle):
                    continue
                if servo_number not in self.servos and not self.simulation_mode:
                    continue

                # Determine last known angle (Current State)
                current_angle = None
                if servo_number < len(self.current_angles):
                    current_angle = self.current_angles[servo_number]

                # --- SLEW RATE LIMITING LOGIC ---
                command_angle = target_angle

                if current_angle is not None:
                    diff = target_angle - current_angle
                    
                    # If the requested jump is huge, clamp it
                    if abs(diff) > max_step_per_tick:
                        # Determine direction (+1 or -1)
                        sign = 1.0 if diff > 0 else -1.0
                        # Calculate limited step
                        command_angle = current_angle + (max_step_per_tick * sign)
                    
                    # Check Deadband using the NEW calculated command_angle
                    # (If we are already close enough, don't spam the I2C bus)
                    if abs(current_angle - command_angle) < self.angle_tolerance_deg:
                        continue 
                # --------------------------------

                if not self.simulation_mode:
                    try:
                        self.servos[servo_number].angle = command_angle
                    except ValueError as ve:
                        self.get_logger().warn(f"Servo {servo_number} Value Error: {ve}")
                        continue
                    except OSError as e:
                        # I2C IO Error
                        raise e 

                # Update internal tracking to what we ACTUALLY sent
                if servo_number < len(self.current_angles):
                    self.current_angles[servo_number] = command_angle
                else:
                    self.current_angles.extend(
                        [0.0] * (servo_number + 1 - len(self.current_angles))
                    )
                    self.current_angles[servo_number] = command_angle

            # Publish State
            self._publish_current_servo_angles()

        except Exception as e:
            self.get_logger().error(f"I2C error while refreshing servo positions: {e}")
            if "Remote I/O" in str(e) or "No device" in str(e) or "[Errno 121]" in str(e):
                self.record_failure_and_check_restart()

    def move_servos_to_glide_position(self):
        """
        Moves servos to glide position with STAGGERED STARTUP.
        Prioritizes big servos (0 and 2) first to handle inrush current.
        """
        glide_position = self.get_parameter('glide_position').value
        stagger_delay = self.get_parameter('startup_stagger_delay').value

        if not self.simulation_mode:
            self.get_logger().info(f"Initializing servos. Priority: 0 & 2. Stagger: {stagger_delay}s")
            
            # --- DEFINE PRIORITY ORDER ---
            # 1. Heavy Lifters (Servo 0 and 2)
            # 2. The Rest (1, 3, 4, 5...)
            priority_indices = [0, 2]
            remaining_indices = [i for i in range(len(glide_position)) if i not in priority_indices]
            
            full_startup_order = priority_indices + remaining_indices
            # -----------------------------

            for i in full_startup_order:
                # Safety check if index exists in config
                if i < len(glide_position) and i in self.servos:
                    target_angle = glide_position[i]
                    try:
                        self.servos[i].angle = target_angle
                        
                        # Wait for this servo's inrush spike to settle
                        if stagger_delay > 0:
                            time.sleep(stagger_delay)
                            
                    except Exception as e:
                         self.get_logger().warn(f"Failed to set glide for servo {i}: {e}")

        # Update internal tracking
        with self.target_lock:
            self.last_target_angles = glide_position.copy()
        self.current_angles = glide_position.copy()

        # Immediately publish angles
        self._publish_current_servo_angles()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Servo Driver Node...')

        # === Publishers First ===
        self.angles_publisher = self.create_publisher(Float32MultiArray, 'current_servo_angles', 10)
        self.heartbeat_publisher = self.create_publisher(String, 'servo_driver/heartbeat', 10)
        self.busy_status_publisher = self.create_publisher(String, 'servo_driver_status', 10)
        self._publish_busy_status("configuring")

        try:
            # === Load Parameters ===
            self.simulation_mode = self.get_parameter('simulation_mode').value
            self.debug_logging = self.get_parameter('debug_logging').value

            if self.simulation_mode:
                self.get_logger().info('Simulation mode enabled. No hardware will be initialized.')
            else:
                self.get_logger().info('Initializing hardware...')
                self.i2c = busio.I2C(board.SCL, board.SDA)
                self.pca = PCA9685(self.i2c)
                self.pca.frequency = 60
                time.sleep(1.0) 
                self.servos = {
                    0: Servo(self.pca.channels[0], min_pulse=500, max_pulse=2500),
                    1: Servo(self.pca.channels[1], min_pulse=500, max_pulse=2500),
                    2: Servo(self.pca.channels[2], min_pulse=500, max_pulse=2500),
                    3: Servo(self.pca.channels[3], min_pulse=500, max_pulse=2500),
                    4: Servo(self.pca.channels[4], min_pulse=500, max_pulse=2500),
                    5: Servo(self.pca.channels[5], min_pulse=500, max_pulse=2500),
                }

            # === Move Servos to Glide Position (With Stagger) ===
            self.move_servos_to_glide_position()

            # === Subscribers ===
            self.command_subscriber = self.create_subscription(
                ServoMovementCommand, 'servo_driver_commands', self._servo_command_callback, 10)
            self.tail_command_subscriber = self.create_subscription(
                ServoMovementCommand, 'tail_commands', self._servo_command_callback, 10)

            hold_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.hold_state_subscription = self.create_subscription(
                HoldRequest,
                'wing_hold_request',
                self._hold_state_callback,
                hold_qos,
            )

            self.get_logger().info('Configuration successful.')
            self.last_lifecycle_state = 'inactive'
            self._publish_lifecycle_state()
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to configure hardware: {e}')
            if "Remote I/O" in str(e) or "No device" in str(e):
                self.record_failure_and_check_restart()
            self.last_lifecycle_state = 'unconfigured'
            self._publish_lifecycle_state()
            return TransitionCallbackReturn.FAILURE


    def _hold_state_callback(self, msg: HoldRequest):
        previous_hold_state = self.hold_active
        self.hold_active = bool(msg.hold)
        if self.debug_logging:
            self.get_logger().info(
                f"Received wing hold state update: {'active' if self.hold_active else 'released'}"
            )

        if self.hold_active and not previous_hold_state:
            self.executing_movement = False
            self.current_movement_type = ''
            self._freeze_targets_at_current_positions()

        if self._last_base_status_message is not None:
            self._publish_busy_status(self._last_base_status_message, force=True)

    def _freeze_targets_at_current_positions(self) -> None:
        with self.target_lock:
            if not self.last_target_angles:
                return
            for idx, angle in enumerate(self.current_angles):
                if idx < len(self.last_target_angles):
                    self.last_target_angles[idx] = angle

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Servo Driver Node...')
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat) 
        self.start_refresh_loop()
        self._publish_busy_status("activated")
        self.last_lifecycle_state = 'active'
        self._publish_lifecycle_state()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating Servo Driver Node... Attempting to limp servos.')
        self._stop_timers()
        self._stop_pwm()
        self.last_lifecycle_state = 'inactive'
        self._publish_lifecycle_state()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up Servo Driver Node...')
        self._stop_timers()
        self._release_hardware()
        self.last_lifecycle_state = 'unconfigured'
        self._publish_lifecycle_state()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down Servo Driver Node...')
        self._stop_timers()
        self._release_hardware()
        self.last_lifecycle_state = 'shutdown'
        self._publish_lifecycle_state()
        return TransitionCallbackReturn.SUCCESS

    def _attempt_restore_lifecycle_state(self):
        if self._restoration_attempted:
            return
        target_state = self._restoration_target
        if not self.stubborn or not target_state:
            self._restoration_attempted = True
            return

        self._restoration_attempted = True
        try:
            if target_state in ('inactive', 'active'):
                config_result = self.on_configure(None)
                if config_result != TransitionCallbackReturn.SUCCESS:
                    self.get_logger().error(
                        f"Failed to restore configure state ({config_result}); staying unconfigured."
                    )
                    return
                if target_state == 'active':
                    activate_result = self.on_activate(None)
                    if activate_result != TransitionCallbackReturn.SUCCESS:
                        self.get_logger().error(
                            f"Failed to restore activation ({activate_result}); staying inactive."
                        )
                        return
            self.get_logger().info(
                f"Restored lifecycle state to '{self.last_lifecycle_state}' after restart (target was {target_state})."
            )
        except Exception as e:
            self.get_logger().error(f"Error while restoring lifecycle state: {e}")
            self.record_failure_and_check_restart()

    def _servo_command_callback(self, msg: ServoMovementCommand):
        if self.debug_logging:
            self.get_logger().info(f'Received command: {msg}')

        try:
            movement_type_raw = msg.movement_type or ""
            movement_type = movement_type_raw.lower()
            is_pid_control = 'pid_control' in movement_type
            is_end_message = 'end' in movement_type

            if is_end_message:
                self.executing_movement = False
                self.current_movement_type = ''
                self._publish_busy_status("nominal: roll and pitch pid engaged")
            elif is_pid_control:
                if not self.executing_movement:
                    self._publish_busy_status("nominal: roll and pitch pid engaged")
            else:
                self.executing_movement = True
                self.current_movement_type = movement_type
                self._publish_busy_status(f"busy: executing {msg.movement_type}")

            if self.hold_active and not is_pid_control:
                if self.debug_logging:
                    self.get_logger().info(
                        "Wing hold active; ignoring non-PID command with movement_type=%s",
                        movement_type_raw or 'unknown',
                    )
                return

            with self.target_lock:
                for i, servo_number in enumerate(msg.servo_numbers):
                    if 0 <= servo_number < len(self.last_target_angles):
                        target_angle = msg.target_angles[i]
                        if target_angle is None or math.isnan(target_angle):
                            continue
                        self.last_target_angles[servo_number] = target_angle
            
        except Exception as e:
            self.get_logger().error(f'Failed to process command: {e}')
            if "Remote I/O" in str(e) or "No device" in str(e):
                self.record_failure_and_check_restart()

    def _publish_current_servo_angles(self):
        angles_msg = Float32MultiArray()
        with self.target_lock:
            angles_msg.data = list(self.current_angles)
        self.angles_publisher.publish(angles_msg)

    def _publish_heartbeat(self):
        heartbeat_msg = String()
        heartbeat_msg.data = 'alive'
        self.heartbeat_publisher.publish(heartbeat_msg)

    def _publish_lifecycle_state(self):
        state_msg = String()
        state_msg.data = self.last_lifecycle_state
        self.lifecycle_state_publisher.publish(state_msg)

    def _publish_busy_status(self, status_message: str, *, force: bool = False):
        self._last_base_status_message = status_message
        enriched_message = self._compose_status_message(status_message)
        if force or enriched_message != self.last_published_status:
            status_msg = String()
            status_msg.data = enriched_message
            self.busy_status_publisher.publish(status_msg)
            if self.debug_logging: 
                self.get_logger().info(f"[ServoDriver Status] Published: {status_msg.data}")
            self.last_published_status = enriched_message

    def _compose_status_message(self, base_message: str) -> str:
        hold_descriptor = 'wing hold active' if self.hold_active else 'wing hold released'
        return f"{base_message} | {hold_descriptor}"

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        if node.stubborn:
            node.restart_process()
        else:
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
