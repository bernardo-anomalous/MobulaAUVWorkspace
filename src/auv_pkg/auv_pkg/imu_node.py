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

class IMUNode(Node):
    def __init__(self):
        super().__init__('optimized_imu_node')

        # Heading offset for correction
        self.heading_offset = -30  # Adjust based on observed offset

        # Initialize I2C connection
        self.initialize_sensor()

        # Publishers
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'imu/heading', 10)

        # Publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_imu_data)

    def initialize_sensor(self):
        """Initialize the BNO08X sensor."""
        self.get_logger().info("Initializing BNO08X sensor...")
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.bno = BNO08X_I2C(i2c, address=0x4B)

            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

            # Allow time for the sensor to stabilize
            time.sleep(0.5)
            self.get_logger().info("Sensor initialization complete.")
        except Exception as e:
            self.get_logger().error(f"Error initializing sensor: {e}")

    def reset_sensor(self):
        """Fully reset the BNO08X sensor to recover from errors."""
        self.get_logger().info("Resetting BNO08X sensor...")
        try:
            self.initialize_sensor()
        except Exception as e:
            self.get_logger().error(f"Error resetting sensor: {e}")

    def publish_imu_data(self):
        try:
            # Read sensor data
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            accel_x, accel_y, accel_z = self.bno.acceleration
            gyro_x, gyro_y, gyro_z = self.bno.gyro
        except KeyError as e:
            self.get_logger().error(f"Unexpected report ID: {e}")
            self.reset_sensor()
            return
        except Exception as e:
            self.get_logger().error(f"Sensor error: {e}")
            self.reset_sensor()
            return

        # Publish IMU data
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

        imu_msg.orientation_covariance[0] = -1  # Unknown covariance
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.linear_acceleration_covariance[0] = -1

        self.imu_publisher_.publish(imu_msg)

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        roll, pitch, yaw = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)

        # Publish RPY
        rpy_msg = Vector3()
        rpy_msg.x = roll
        rpy_msg.y = pitch
        rpy_msg.z = yaw
        self.rpy_publisher_.publish(rpy_msg)

        # Publish heading
        heading_degrees = (-yaw + self.heading_offset + 360) % 360  # Normalize to 0-360 degrees, with offset
        cardinal_direction = self.yaw_to_cardinal(heading_degrees)
        heading_msg = String()
        heading_msg.data = f'Heading: {cardinal_direction}, {heading_degrees:.2f} degrees'
        self.heading_publisher_.publish(heading_msg)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to roll, pitch, yaw."""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = -math.asin(t2)  # Reverse pitch direction

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        # Convert radians to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return roll, pitch, yaw

    def yaw_to_cardinal(self, heading_degrees):
        """Convert yaw angle to cardinal direction."""
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)

    # Cleanup
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
