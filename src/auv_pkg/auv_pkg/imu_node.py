#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_STABILITY_CLASSIFIER
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class IMUNode(Node):
    def __init__(self):
        super().__init__('optimized_imu_node')

        # === Configurable Offsets ===
        self.heading_offset = -30.0  # Degrees (adjust after calibration)
        self.roll_offset = 0.0
        self.pitch_offset = 0.0

        # === Sensor Startup Mode ===
        self.sensor_ready_mode = 'alive'  # 'alive' or 'stable'
        self.sensor_ready = False

        # === Initialize Sensor ===
        self.initialize_sensor()

        # === ROS Publishers ===
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)

        # === Preallocate Messages ===
        self.imu_msg = Imu()
        self.rpy_msg = Vector3()

        # === Timer at 5 Hz ===
        self.timer = self.create_timer(0.2, self.publish_imu_data)

    def initialize_sensor(self):
        self.get_logger().info("Initializing BNO08X sensor...")
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c, address=0x4B)
            features = [
                BNO_REPORT_ACCELEROMETER,
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_ROTATION_VECTOR,
                BNO_REPORT_STABILITY_CLASSIFIER
            ]
            for feature in features:
                for attempt in range(5):
                    try:
                        self.bno.enable_feature(feature)
                        break
                    except Exception as e:
                        self.get_logger().warn(f"Attempt {attempt+1} to enable feature {feature} failed: {e}")
                else:
                    raise RuntimeError(f"Could not enable feature {feature} after 5 attempts.")
            self.sensor_ready = False
        except Exception as e:
            self.get_logger().error(f"Error initializing sensor: {e}")

    def check_sensor_ready(self):
        try:
            status = self.bno.stability_classification
        except (RuntimeError, KeyError, Exception) as e:
            self.get_logger().warn(f"Sensor not ready: {e}")
            return False

        if status and status != "Unknown":
            return self.sensor_ready_mode == 'alive' or status in ["Stable", "Stationary", "On Table", "In motion"]
        return False

    def reset_sensor(self):
        self.get_logger().info("Resetting BNO08X sensor...")
        self.initialize_sensor()

    def publish_imu_data(self):
        if not self.sensor_ready:
            if self.check_sensor_ready():
                self.sensor_ready = True
                self.get_logger().info(f"Sensor ready (mode: {self.sensor_ready_mode}).")
            else:
                return  # Skip publishing if not ready

        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            accel_x, accel_y, accel_z = self.bno.acceleration
            gyro_x, gyro_y, gyro_z = self.bno.gyro

            # === Update IMU Message ===
            imu_msg = self.imu_msg
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
            imu_msg.orientation_covariance[0] = -1  # Unknown covariance
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1
            self.imu_publisher_.publish(imu_msg)

            # === Euler Conversion and RPY Publishing ===
            roll, pitch, yaw = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
            self.rpy_msg.x = roll + self.roll_offset
            self.rpy_msg.y = pitch + self.pitch_offset
            self.rpy_msg.z = yaw
            self.rpy_publisher_.publish(self.rpy_msg)

        except Exception as e:
            self.get_logger().error(f"Sensor error: {e}")
            self.reset_sensor()

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, 1.0), -1.0)
        pitch = -math.asin(t2)  # Negative because pitch direction is reversed

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
