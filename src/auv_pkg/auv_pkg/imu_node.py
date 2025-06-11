#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import math
import time
import os
import sys
from collections import deque

from .utils import quaternion_to_euler

class IMUNode(Node):
    def __init__(self):
        super().__init__('optimized_imu_node')

        self.heading_offset = -30.0  # Adjust after calibration
        self.sensor_ready = False

        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'imu/heading', 10)
        self.health_status_publisher = self.create_publisher(String, 'imu/health_status', 10)

        self.last_health_status = None
        self.last_failure_reason = None

        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10 Hz
        self.retry_timer = self.create_timer(5.0, self._retry_init_if_needed)

        self.i2c = None
        self.bno = None

        self.failure_timestamps = deque()
        self.max_failures = 3
        self.failure_window_seconds = 30

        self.CRITICAL_ERRORS = [
            "No device", "Remote I/O", "Input/output error",
            "Unprocessable Batch bytes", "Was not able to enable feature"
        ]

        self.reset_attempts = 0
        self.max_reset_attempts = 3

        self.initialize_sensor()

    def restart_process(self):
        self.publish_health_status("IMU RESTARTING")
        self.get_logger().error("IMU node restarting itself now...")
        time.sleep(0.2)
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def reset_i2c_bus(self) -> bool:
        self.get_logger().error("Attempting soft I2C reset...")
        try:
            if self.i2c:
                try:
                    self.i2c.deinit()
                except Exception as e:
                    self.get_logger().error(f"Error deinitializing I2C: {e}")
            time.sleep(0.1)
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c, address=0x4B)
            time.sleep(0.2)
            try:
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                time.sleep(0.1)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                time.sleep(0.1)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            except Exception as fe:
                self.get_logger().error(f"Critical error enabling feature: {fe}")
                return False
            self.sensor_ready = True
            self.get_logger().error("Soft reset successful.")
            return True
        except Exception as e:
            self.get_logger().error(f"Soft reset failed: {e}")
            self.sensor_ready = False
            return False

    def record_failure_and_check_restart(self):
        current_time = time.time()
        self.failure_timestamps.append(current_time)

        while self.failure_timestamps and (current_time - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()

        failure_count = len(self.failure_timestamps)

        if failure_count >= self.max_failures:
            self.get_logger().error(
                f"Exceeded {self.max_failures} failures within "
                f"{self.failure_window_seconds} seconds.")
            if self.reset_attempts < self.max_reset_attempts and self.reset_i2c_bus():
                self.reset_attempts += 1
                self.get_logger().info(
                    f"Soft reset attempt {self.reset_attempts}/"
                    f"{self.max_reset_attempts} successful")
                self.publish_health_status("IMU SOFT RESET")
                self.failure_timestamps.clear()
            else:
                self.get_logger().fatal(
                    f"Restart after {self.reset_attempts} soft reset attempts")
                self.publish_health_status(
                    f"IMU RESTARTING | Reason: {self.last_failure_reason}")
                time.sleep(2)
                self.restart_process()
        else:
            self.publish_health_status(f"IMU UNSTABLE ({failure_count} failures in last {self.failure_window_seconds} sec)")

    def publish_health_status(self, status):
        message_text = status
        if "UNSTABLE" in status and self.last_failure_reason:
            message_text += f" | Last error: {self.last_failure_reason}"
        elif "RESTARTING" in status and self.last_failure_reason:
            message_text += f" | Reason: {self.last_failure_reason}"

        if message_text != self.last_health_status:
            status_msg = String()
            status_msg.data = message_text
            self.health_status_publisher.publish(status_msg)
            self.get_logger().info(f"[IMU Health] {message_text}")
            self.last_health_status = message_text

    def _retry_init_if_needed(self):
        if self.sensor_ready:
            return
        self.record_failure_and_check_restart()

    def initialize_sensor(self):
        self.get_logger().info("Initializing BNO08X sensor...")

        try:
            if self.i2c:
                self.i2c.deinit()
                time.sleep(0.1)

            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c, address=0x4B)

            # Try enabling features one by one
            time.sleep(0.2)
            try:
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                time.sleep(0.1)
                self.bno.enable_feature(BNO_REPORT_GYROSCOPE)#if error to enable feature, comment here and run script again, then uncomment and run again.
                time.sleep(0.1)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                time.sleep(0.1)
            except Exception as feature_e:
                self.get_logger().error(
                    f"Critical error enabling feature: {feature_e}")
                self.last_failure_reason = str(feature_e)
                self.get_logger().error(
                    "Invoking failure handler: will attempt soft reset before any restart")
                self.record_failure_and_check_restart()
                return

            time.sleep(0.5)

            self.sensor_ready = True
            self.failure_timestamps.clear()
            self.publish_health_status("IMU OK")
            self.get_logger().info("Sensor initialized and ready.")

        except Exception as e:
            self.sensor_ready = False
            self.last_failure_reason = str(e)
            self.get_logger().error(f"Error initializing sensor: {e}")

            if any(keyword in self.last_failure_reason for keyword in self.CRITICAL_ERRORS):
                self.record_failure_and_check_restart()


    def publish_imu_data(self):
        if not self.sensor_ready:
            return

        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            accel_x, accel_y, accel_z = self.bno.acceleration
            gyro_x, gyro_y, gyro_z = self.bno.gyro

            # === Data Validation ===
            if not self.validate_imu_data(quat_i, quat_j, quat_k, quat_real, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
                self.get_logger().warn("Invalid IMU data detected. Skipping publish.")
                self.publish_health_status("IMU HICCUP | Invalid sensor data detected")
                return

            # === Publish valid data ===
            imu_msg = Imu()
            imu_msg.orientation.x = quat_i
            imu_msg.orientation.y = quat_j
            imu_msg.orientation.z = quat_k
            imu_msg.orientation.w = quat_real
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            imu_msg.orientation_covariance[0] = -1
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1

            self.imu_publisher_.publish(imu_msg)

            roll, pitch, yaw = quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            rpy_msg = Vector3()
            rpy_msg.x = roll
            rpy_msg.y = pitch
            rpy_msg.z = yaw
            self.rpy_publisher_.publish(rpy_msg)

            heading_degrees = (-yaw + self.heading_offset + 360) % 360
            heading_msg = String()
            heading_msg.data = f'Heading: {self.yaw_to_cardinal(heading_degrees)}, {heading_degrees:.2f} degrees'
            self.heading_publisher_.publish(heading_msg)

            # === Recovery Health Reporting ===
            if self.last_health_status != "IMU OK":
                self.publish_health_status("IMU OK")

        except Exception as e:
            self.last_failure_reason = str(e)
            self.get_logger().error(f"Sensor read error: {e}")

            if any(keyword in self.last_failure_reason for keyword in self.CRITICAL_ERRORS):
                self.get_logger().warn(f"Critical failure detected: {e}")
                self.record_failure_and_check_restart()
            else:
                self.publish_health_status(f"IMU HICCUP | Last error: {self.last_failure_reason}")



    def yaw_to_cardinal(self, heading_degrees):
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]
    
    def validate_imu_data(self, qi, qj, qk, qr, ax, ay, az, gx, gy, gz):
        # Validate quaternion: check near unit length
        quat_norm = math.sqrt(qi**2 + qj**2 + qk**2 + qr**2)
        if abs(quat_norm - 1.0) > 0.2:  # Allow small error
            return False

        # Validate acceleration: magnitude reasonable (Earth gravity ~9.8 m/s²)
        accel_magnitude = math.sqrt(ax**2 + ay**2 + az**2)
        if accel_magnitude < 5.0 or accel_magnitude > 20.0:  # Accept 5-20 m/s² range
            return False

        # Validate gyro: unrealistic spin rates (e.g., > 2000 deg/s) should not happen
        gyro_threshold = math.radians(2000)  # ~35 rad/s
        if any(abs(g) > gyro_threshold for g in [gx, gy, gz]):
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
