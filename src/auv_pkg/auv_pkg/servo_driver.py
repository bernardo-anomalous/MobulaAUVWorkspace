#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray, String
from auv_custom_interfaces.msg import ServoMovementCommand
from adafruit_motor.servo import Servo
from adafruit_pca9685 import PCA9685
import board
import os
import sys
import time
from collections import deque
import threading


class ServoDriverNode(LifecycleNode):
    def __init__(self):
        super().__init__('servo_driver_node')
        self.get_logger().info('Servo Driver Node initialized.')

        # Parameters
        self.declare_parameter('glide_position', [60.0, 90.0, 60.0, 90.0, 90.0, 90.0])
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('debug_logging', False)

        self.simulation_mode = False
        self.debug_logging = False
        self.pca = None
        self.servos = {}
        self.current_angles = []
        self.last_target_angles = [90.0] * 6

        self.angles_publisher = None
        self.command_subscriber = None
        self.tail_command_subscriber = None
        self.heartbeat_timer = None
        self.refresh_timer = None
        self.busy_status_publisher = self.create_publisher(String, 'servo_driver_status', 10)
        self.last_published_status = None
        self.executing_movement = False

        # Failure tracking
        self.failure_timestamps = deque()
        self.max_failures = 3
        self.failure_window_seconds = 30  # Restart if 3 failures occur within 30 seconds
        self.target_lock = threading.Lock()


    def restart_process(self):
        self.get_logger().error("Servo Driver Node restarting itself now...")
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def record_failure_and_check_restart(self):
        current_time = time.time()
        self.failure_timestamps.append(current_time)
        while self.failure_timestamps and (current_time - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()

        if len(self.failure_timestamps) >= self.max_failures:
            self.get_logger().fatal(
                f"Exceeded {self.max_failures} failures within {self.failure_window_seconds} seconds. Restarting process..."
            )
            time.sleep(2)
            self.restart_process()

    def start_refresh_loop(self):
        update_rate_hz = self.get_parameter('update_rate_hz').value
        self.refresh_timer = self.create_timer(1.0 / update_rate_hz, self.refresh_servo_positions)
        self.get_logger().info(f"Started refresh loop at {update_rate_hz} Hz.")

    def refresh_servo_positions(self):
        if not self.simulation_mode:
            try:
                with self.target_lock:
                    target_angles_copy = self.last_target_angles.copy()

                for servo_number, target_angle in enumerate(target_angles_copy):
                    if servo_number in self.servos:
                        self.servos[servo_number].angle = target_angle

            except Exception as e:
                self.get_logger().error(f"I2C error while refreshing servo positions: {e}")
                if "Remote I/O" in str(e) or "No device" in str(e):
                    self.record_failure_and_check_restart()
                    
    def move_servos_to_glide_position(self):
        glide_position = self.get_parameter('glide_position').value

        if not self.simulation_mode:
            self.get_logger().info(f"Moving servos to glide position: {glide_position}")
            for i, angle in enumerate(glide_position):
                if i in self.servos:
                    self.servos[i].angle = angle

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

        try:
            # === Load Parameters ===
            self.simulation_mode = self.get_parameter('simulation_mode').value
            self.debug_logging = self.get_parameter('debug_logging').value

            if self.simulation_mode:
                self.get_logger().info('Simulation mode enabled. No hardware will be initialized.')
            else:
                self.get_logger().info('Initializing hardware...')
                i2c = board.I2C()
                self.pca = PCA9685(i2c)
                self.pca.frequency = 100
                self.servos = {
                    0: Servo(self.pca.channels[0], min_pulse=400, max_pulse=2500),
                    1: Servo(self.pca.channels[1], min_pulse=400, max_pulse=2500),
                    2: Servo(self.pca.channels[2], min_pulse=400, max_pulse=2500),
                    3: Servo(self.pca.channels[3], min_pulse=400, max_pulse=2500),
                    4: Servo(self.pca.channels[4], min_pulse=500, max_pulse=2500),
                    5: Servo(self.pca.channels[5], min_pulse=500, max_pulse=2500),
                }

            # === Move Servos to Glide Position ===
            self.move_servos_to_glide_position()

            # === Subscribers ===
            self.command_subscriber = self.create_subscription(
                ServoMovementCommand, 'servo_driver_commands', self._servo_command_callback, 10)
            self.tail_command_subscriber = self.create_subscription(
                ServoMovementCommand, 'tail_commands', self._servo_command_callback, 10)

            self.get_logger().info('Configuration successful.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to configure hardware: {e}')
            if "Remote I/O" in str(e) or "No device" in str(e):
                self.record_failure_and_check_restart()
            return TransitionCallbackReturn.FAILURE


    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Servo Driver Node...')
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)  # 1 Hz heartbeat

        self.start_refresh_loop()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating Servo Driver Node... Attempting to limp servos.')
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        if self.refresh_timer:
            self.refresh_timer.cancel()
            self.refresh_timer = None

        if not self.simulation_mode and self.pca:
            try:
                for channel in self.pca.channels:
                    channel.duty_cycle = 0
                self.get_logger().info('PWM signals stopped. Servos should be limp (if supported by model).')
            except Exception as e:
                self.get_logger().error(f'Error while setting duty cycles to 0: {e}')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up Servo Driver Node...')
        if not self.simulation_mode and self.pca:
            self.pca.deinit()
            self.pca = None
        return TransitionCallbackReturn.SUCCESS

    def _servo_command_callback(self, msg: ServoMovementCommand):
        if self.debug_logging:
            self.get_logger().info(f'Received command: {msg}')

        try:
            movement_type = msg.movement_type.lower() if msg.movement_type else ""

            if 'end' in movement_type:
                self.executing_movement = False
                self._publish_busy_status("nominal: roll and pitch pid engaged")
            elif movement_type != 'pid_control':
                if not self.executing_movement:
                    self.executing_movement = True
                    self._publish_busy_status(f"busy: executing {msg.movement_type}")
            else:
                if not self.executing_movement:
                    self._publish_busy_status("nominal: roll and pitch pid engaged")

            # === NORMAL SERVO EXECUTION ===
            with self.target_lock:
                for i, servo_number in enumerate(msg.servo_numbers):
                    if 0 <= servo_number < len(self.last_target_angles):
                        self.last_target_angles[servo_number] = msg.target_angles[i]


            self._publish_current_servo_angles()

        except Exception as e:
            self.get_logger().error(f'Failed to execute command: {e}')
            if "Remote I/O" in str(e) or "No device" in str(e):
                self.record_failure_and_check_restart()

    def _publish_current_servo_angles(self):
        angles_msg = Float32MultiArray()
        with self.target_lock:
            angles_msg.data = self.last_target_angles.copy()

        self.angles_publisher.publish(angles_msg)

    def _publish_heartbeat(self):
        heartbeat_msg = String()
        heartbeat_msg.data = 'alive'
        self.heartbeat_publisher.publish(heartbeat_msg)

    def _publish_busy_status(self, status_message: str):
        if status_message != self.last_published_status:
            status_msg = String()
            status_msg.data = status_message
            self.busy_status_publisher.publish(status_msg)
            self.get_logger().info(f"[ServoDriver Status] Published: {status_msg.data}")
            self.last_published_status = status_message

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
