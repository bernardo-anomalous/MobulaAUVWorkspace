#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray, String
from auv_custom_interfaces.msg import ServoMovementCommand
from adafruit_pca9685 import PCA9685
import board
import os
import sys
import time
import threading
from collections import deque

class ServoDriverNode(LifecycleNode):
    def __init__(self):
        super().__init__('servo_driver_node')
        self.get_logger().info('Servo Driver Node initialized.')

        # Declare Parameters
        self.declare_parameter('glide_position', [60.0, 90.0, 60.0, 90.0, 90.0, 90.0])
        self.declare_parameter('update_rate_hz', 100.0)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('debug_logging', False)

        self.simulation_mode = False
        self.debug_logging = False
        self.pca = None

        self.current_angles = []
        self.last_target_angles = [90.0] * 6
        self.target_lock = threading.Lock()

        self.failure_timestamps = deque()
        self.max_failures = 3
        self.failure_window_seconds = 30

        self.angles_publisher = None
        self.heartbeat_publisher = None
        self.busy_status_publisher = None
        self.last_published_status = None

        self.command_subscriber = None
        self.tail_command_subscriber = None
        self.heartbeat_timer = None
        self.refresh_timer = None
        self.executing_movement = False

    def angle_to_pwm(self, angle, min_pulse=500, max_pulse=2500):
        pulse_length = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
        duty_cycle = int((pulse_length / 20000.0) * 4096)  # assuming 20ms = 50Hz cycle
        return max(0, min(duty_cycle, 4095))

    def restart_process(self):
        self.get_logger().fatal("Servo Driver restarting...")
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def record_failure_and_check_restart(self):
        now = time.time()
        self.failure_timestamps.append(now)
        while self.failure_timestamps and (now - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()
        if len(self.failure_timestamps) >= self.max_failures:
            self.restart_process()

    def start_refresh_loop(self):
        hz = self.get_parameter('update_rate_hz').value
        self.refresh_timer = self.create_timer(1.0 / hz, self.refresh_servo_positions)
        self.get_logger().info(f'Refresh loop running at {hz:.1f} Hz')

    def refresh_servo_positions(self):
        if self.simulation_mode or not self.pca:
            return
        try:
            with self.target_lock:
                angles = self.last_target_angles.copy()

            for i, angle in enumerate(angles):
                min_pulse = 400 if i < 4 else 500
                max_pulse = 2500
                pwm = self.angle_to_pwm(angle, min_pulse, max_pulse)
                self.pca.channels[i].duty_cycle = pwm

        except Exception as e:
            self.get_logger().error(f'I2C Refresh Error: {e}')
            if 'Remote I/O' in str(e) or 'No device' in str(e):
                self.record_failure_and_check_restart()

    def _servo_command_callback(self, msg: ServoMovementCommand):
        try:
            movement_type = msg.movement_type.lower() if msg.movement_type else ""
            if 'end' in movement_type:
                self.executing_movement = False
                self._publish_busy_status("nominal: roll and pitch pid engaged")
            elif movement_type != 'pid_control':
                if not self.executing_movement:
                    self.executing_movement = True
                    self._publish_busy_status(f"busy: executing {movement_type}")
            else:
                if not self.executing_movement:
                    self._publish_busy_status("nominal: roll and pitch pid engaged")

            with self.target_lock:
                for i, servo_number in enumerate(msg.servo_numbers):
                    if 0 <= servo_number < len(self.last_target_angles):
                        self.last_target_angles[servo_number] = msg.target_angles[i]

            self._publish_current_servo_angles()

        except Exception as e:
            self.get_logger().error(f"Command execution failed: {e}")
            if "Remote I/O" in str(e) or "No device" in str(e):
                self.record_failure_and_check_restart()

    def _publish_current_servo_angles(self):
        msg = Float32MultiArray(data=self.last_target_angles)
        self.angles_publisher.publish(msg)

    def _publish_busy_status(self, status: str):
        if status != self.last_published_status:
            msg = String(data=status)
            self.busy_status_publisher.publish(msg)
            self.last_published_status = status

    def _publish_heartbeat(self):
        self.heartbeat_publisher.publish(String(data="alive"))

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.debug_logging = self.get_parameter('debug_logging').value

        self.heartbeat_publisher = self.create_publisher(String, 'servo_driver/heartbeat', 10)
        self.angles_publisher = self.create_publisher(Float32MultiArray, 'current_servo_angles', 10)
        self.busy_status_publisher = self.create_publisher(String, 'servo_driver_status', 10)

        self.command_subscriber = self.create_subscription(
            ServoMovementCommand, 'servo_driver_commands', self._servo_command_callback, 10)
        self.tail_command_subscriber = self.create_subscription(
            ServoMovementCommand, 'tail_commands', self._servo_command_callback, 10)

        try:
            if not self.simulation_mode:
                i2c = board.I2C()
                self.pca = PCA9685(i2c)
                self.pca.frequency = 60
                for i, angle in enumerate(self.get_parameter('glide_position').value):
                    min_pulse = 400 if i < 4 else 500
                    max_pulse = 2500
                    self.pca.channels[i].duty_cycle = self.angle_to_pwm(angle, min_pulse, max_pulse)

            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Hardware config failed: {e}')
            if "Remote I/O" in str(e) or "No device" in str(e):
                self.record_failure_and_check_restart()
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating node...')
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self.start_refresh_loop()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
        if self.refresh_timer:
            self.refresh_timer.cancel()
        if not self.simulation_mode and self.pca:
            for ch in self.pca.channels:
                ch.duty_cycle = 0
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        if not self.simulation_mode and self.pca:
            self.pca.deinit()
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
