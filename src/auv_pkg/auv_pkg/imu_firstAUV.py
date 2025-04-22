#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ROTATION_VECTOR
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3
import math
import time

class IMUNode(Node):
    def __init__(self):
        super().__init__('bno080_imu_node')
        
        # Set up I2C connection on default I2C Bus 1
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=0x4B)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                
        # Publisher for IMU data
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.euler_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'heading', 10)
        self.heading_value_publisher_ = self.create_publisher(Float32, 'heading_value', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10Hz

    def publish_imu_data(self):
        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
            accel_x, accel_y, accel_z = self.bno.acceleration
        except KeyError as e:
            self.get_logger().error(f'Unknown sensor report ID: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Unexpected error when reading sensor data: {e}')
            return

        quat = type('', (), {})()  # Create a simple object to hold quaternion data
        quat.x = quat_i
        quat.y = quat_j
        quat.z = quat_k
        quat.w = quat_real
        if quat is not None:
            imu_msg = Imu()

            # Set orientation in quaternion (x, y, z, w)
            imu_msg.orientation.x = quat.x
            imu_msg.orientation.y = quat.y
            imu_msg.orientation.z = quat.z
            imu_msg.orientation.w = quat.w

            # Since the BNO080 has an integrated fusion algorithm, the covariance can be set to 0 for now.
            imu_msg.orientation_covariance[0] = -1  # -1 indicates that covariance is unknown

            # Publish the IMU message
            self.imu_publisher_.publish(imu_msg)
            self.get_logger().info('Publishing IMU data')

            # Calculate roll, pitch, and yaw from quaternion
            roll, pitch, yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

            # Publish roll, pitch, and yaw to the imu/euler topic
            euler_msg = Vector3()
            euler_msg.x = roll
            euler_msg.y = pitch
            euler_msg.z = yaw
            self.euler_publisher_.publish(euler_msg)
            self.get_logger().info(f'Publishing Euler angles - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')

            # Calculate cardinal direction from yaw
            cardinal_direction = self.yaw_to_cardinal(yaw)

            # Calculate heading in degrees
            heading_degrees = (yaw + 360) % 360  # Normalize yaw to 0-360 degrees

            # Publish combined heading message
            heading_msg = String()
            heading_msg.data = f'Heading: {cardinal_direction}, {heading_degrees:.2f} degrees'
            self.heading_publisher_.publish(heading_msg)
            self.get_logger().info(heading_msg.data)

            # Publish heading value in degrees to the heading_value topic
            heading_value_msg = Float32()
            heading_value_msg.data = heading_degrees
            self.heading_value_publisher_.publish(heading_value_msg)
            self.get_logger().info(f'Publishing Heading Value - {heading_degrees:.2f} degrees')

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to roll, pitch, yaw
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        # Convert radians to degrees
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        return roll, pitch, yaw

    def yaw_to_cardinal(self, yaw):
        # Convert yaw angle to cardinal direction
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(yaw / 45.0) % 8
        return directions[index]

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)

    # Clean up
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
