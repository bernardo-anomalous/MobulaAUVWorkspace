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

        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10 Hz update

        self.i2c = None
        self.bno = None

        # Failure tracking with time window
        self.failure_timestamps = deque()
        self.max_failures = 3
        self.failure_window_seconds = 30  # Restart if 3 failures occur within 30 seconds

        self.initialize_sensor()

    def restart_process(self):
        self.publish_health_status("IMU RESTARTING")
        self.get_logger().error("IMU node restarting itself now...")
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def record_failure_and_check_restart(self):
        current_time = time.time()
        self.failure_timestamps.append(current_time)

        while self.failure_timestamps and (current_time - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()

        failure_count = len(self.failure_timestamps)

        if failure_count >= self.max_failures:
            self.publish_health_status(f"IMU RESTARTING | Reason: {self.last_failure_reason}")
            self.get_logger().fatal(
                f"Exceeded {self.max_failures} failures within {self.failure_window_seconds} seconds. Restarting process..."
            )
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

    def initialize_sensor(self):
        self.get_logger().info("Initializing BNO08X sensor...")

        try:
            if self.i2c:
                self.i2c.deinit()
                time.sleep(0.1)

            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(self.i2c, address=0x4B)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            time.sleep(0.5)

            self.sensor_ready = True
            self.failure_timestamps.clear()
            self.publish_health_status("IMU OK")
            self.get_logger().info("Sensor initialized and ready.")

        except Exception as e:
            self.sensor_ready = False
            self.get_logger().error(f"Error initializing sensor: {e}")
            self.last_failure_reason = f"{e}"
            if "No device" in str(e) or "Remote I/O" in str(e):
                self.record_failure_and_check_restart()

    def reset_sensor(self):
        self.get_logger().warn("Attempting to reset the BNO08X sensor...")
        self.sensor_ready = False

        try:
            if self.i2c:
                self.i2c.deinit()
                time.sleep(0.1)

            self.initialize_sensor()

        except Exception as e:
            self.get_logger().error(f"Sensor reset failed: {e}")
            self.last_failure_reason = f"{e}"
            if "No device" in str(e) or "Remote I/O" in str(e):
                self.record_failure_and_check_restart()

    def publish_imu_data(self):
        if not self.sensor_ready:
            return

        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            accel_x, accel_y, accel_z = self.bno.acceleration
            gyro_x, gyro_y, gyro_z = self.bno.gyro

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

            roll, pitch, yaw = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            rpy_msg = Vector3()
            rpy_msg.x = roll
            rpy_msg.y = pitch
            rpy_msg.z = yaw
            self.rpy_publisher_.publish(rpy_msg)

            heading_degrees = (-yaw + self.heading_offset + 360) % 360
            heading_msg = String()
            heading_msg.data = f'Heading: {self.yaw_to_cardinal(heading_degrees)}, {heading_degrees:.2f} degrees'
            self.heading_publisher_.publish(heading_msg)

            # On successful read, clear failures and report OK
            if self.failure_timestamps:
                self.failure_timestamps.clear()
                self.publish_health_status("IMU OK")

        except Exception as e:
            self.get_logger().error(f"Sensor read error: {e}")
            self.last_failure_reason = f"{e}"
            error_str = str(e)
            critical_errors = ["Remote I/O", "No device", "Input/output error", "Unprocessable Batch bytes"]
            if any(err in error_str for err in critical_errors) or error_str in ['123', '0']:
                self.get_logger().warn(f"Critical failure detected: {e}")
                self.record_failure_and_check_restart()
            else:
                self.get_logger().info(f"Non-critical sensor hiccup: {e}")

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)
        pitch = -math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def yaw_to_cardinal(self, heading_degrees):
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]

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
