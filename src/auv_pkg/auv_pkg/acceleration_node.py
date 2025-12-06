#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class AccelerationAnalysisNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')

        # Parameters
        self.declare_parameter('imu_topic', 'imu/data')
        self.declare_parameter('publish_topic', 'acceleration/body_linear')
        self.declare_parameter('gravity_m_s2', 9.80665)
        
        # Helper: Rotation adjustment if sensor is mounted at an angle
        # [Roll, Pitch, Yaw] in degrees
        self.declare_parameter('mount_rpy_deg', [0.0, 0.0, 0.0])

        imu_topic = self.get_parameter('imu_topic').value
        pub_topic = self.get_parameter('publish_topic').value
        self.gravity = self.get_parameter('gravity_m_s2').value
        mount_deg = self.get_parameter('mount_rpy_deg').value
        
        # Pre-compute mounting rotation matrix (Numpy)
        self.mount_rot_matrix = self.euler_to_matrix(
            np.radians(mount_deg[0]), 
            np.radians(mount_deg[1]), 
            np.radians(mount_deg[2])
        )

        # QoS - Best Effort is usually fine for high-rate sensor data
        self.subscription = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.publisher = self.create_publisher(Vector3, pub_topic, 10)

        self.get_logger().info('Acceleration Node Initialized (Numpy Optimized)')
        self.get_logger().info(f'Output: Body Frame Linear Acceleration (Gravity Removed)')

    def imu_callback(self, msg: Imu):
        try:
            # 1. Extract Raw Data
            # Note: BNO08x "Accelerometer" report INCLUDES gravity.
            raw_accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            # Quaternion [x, y, z, w]
            q = np.array([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ])

            # 2. Apply Physical Mounting Rotation (if any) to the acceleration vector
            # If the sensor is tilted 90deg, the "Down" vector isn't Z.
            accel_body = self.mount_rot_matrix @ raw_accel

            # 3. Calculate Gravity Vector in Body Frame
            # Instead of rotating Accel to World and subtracting [0,0,9.8],
            # we rotate the World Gravity Vector [0,0,9.8] into the Body Frame
            # and subtract it from the measurement. 
            # This keeps the result in BODY FRAME (Surge/Sway/Heave).
            
            # Rotate World Z [0,0,1] into Body Frame using Inverse Quaternion
            gravity_in_body = self.rotate_vector_inverse(np.array([0.0, 0.0, self.gravity]), q)

            # 4. Remove Gravity
            linear_accel_body = accel_body - gravity_in_body

            # 5. Publish
            out_msg = Vector3()
            out_msg.x = linear_accel_body[0] # Surge (Forward/Back)
            out_msg.y = linear_accel_body[1] # Sway (Left/Right)
            out_msg.z = linear_accel_body[2] # Heave (Up/Down)
            
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Math Error: {e}")

    # --- Pure Numpy Math Helpers (No Scipy needed) ---

    def euler_to_matrix(self, roll, pitch, yaw):
        # Pre-compute Rotation Matrix for mounting offset
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def rotate_vector_inverse(self, v, q):
        # Rotates vector v by the INVERSE of quaternion q (World -> Body)
        # q = [x, y, z, w]
        x, y, z, w = q
        # Conjugate of unit quaternion is just [-x, -y, -z, w]
        q_inv = np.array([-x, -y, -z, w])
        return self.rotate_vector(v, q_inv)

    def rotate_vector(self, v, q):
        # Standard Quaternion Rotation formula: v' = v + 2 * r x (s * v + r x v) / m
        # where q = (r, s)
        r = q[:3]
        s = q[3]
        return v + 2 * np.cross(r, s * v + np.cross(r, v))

def main(args=None):
    rclpy.init(args=args)
    node = AccelerationAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
