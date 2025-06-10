#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE, BNO_REPORT_MAGNETOMETER, BNO_REPORT_ROTATION_VECTOR
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
import math

from .utils import quaternion_to_euler

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
        self.rpy_publisher_ = self.create_publisher(String, 'imu_euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'heading', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # Publish at 10Hz

    def publish_imu_data(self):
        try:
            quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        except KeyError as e:
            self.get_logger().error(f'Unknown sensor report ID: {e}')
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
            roll, pitch, yaw = quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

            # Publish roll, pitch, and yaw as a combined message
            self.get_logger().info(f'Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')

            # Publish roll, pitch, and yaw to the imu_euler topic
            rpy_msg = String()
            rpy_msg.data = f'Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}'
            self.rpy_publisher_.publish(rpy_msg)

            # Calculate cardinal direction from yaw
            cardinal_direction = self.yaw_to_cardinal(yaw)

            # Calculate heading in degrees
            heading_degrees = (yaw + 360) % 360  # Normalize yaw to 0-360 degrees

            # Publish combined heading message
            heading_msg = String()
            heading_msg.data = f'Heading: {cardinal_direction}, {heading_degrees:.2f} degrees'
            self.heading_publisher_.publish(heading_msg)
            self.get_logger().info(heading_msg.data)


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
