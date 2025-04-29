import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Duration




class AccelerationAnalysisNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')

        # Parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_topic', '/acceleration/processed')
        self.declare_parameter('gravity_m_s2', 9.80665)
        self.declare_parameter('mount_orientation_rpy_deg', [0.0, 0.0, 0.0])

        # Get parameters
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.gravity = self.get_parameter('gravity_m_s2').get_parameter_value().double_value
        rpy_deg = self.get_parameter('mount_orientation_rpy_deg').get_parameter_value().double_array_value
        self.mount_rotation = R.from_euler('xyz', rpy_deg, degrees=True)

        # Subscribers and Publishers
        self.subscription = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.publisher = self.create_publisher(Vector3, publish_topic, 10)

        # Throttling parameters
        self.last_publish_time = self.get_clock().now()
        

        self.get_logger().info('Acceleration Analysis Node Initialized.')
        self.publish_interval = rclpy.duration.Duration(nanoseconds=int(1e9 * 0.1))  #10hz

    def imu_callback(self, msg: Imu):
        try:
            now = self.get_clock().now()
            if (now.nanoseconds - self.last_publish_time.nanoseconds) < self.publish_interval.nanoseconds:
                return
  # Skip this message to maintain publish rate

            # Extract acceleration and orientation
            accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

            quat = [
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z
            ]

            accel_corrected = self.mount_rotation.apply(accel)
            rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            accel_world = rotation.apply(accel_corrected)
            gravity_vector = np.array([0, 0, self.gravity])
            accel_net = accel_world - gravity_vector

            processed_msg = Vector3()
            processed_msg.x = accel_net[0]
            processed_msg.y = accel_net[1]
            processed_msg.z = accel_net[2]

            self.publisher.publish(processed_msg)
            self.last_publish_time = now  # Update time of last publish

        except Exception as e:
            self.get_logger().error(f"Error processing IMU data: {e}")


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
