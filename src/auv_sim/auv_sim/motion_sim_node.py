#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (rad) into a quaternion tuple (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def heading_to_cardinal(heading_deg: float) -> str:
    """Map a heading in degrees to a coarse cardinal direction string."""
    headings = [
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
    ]
    idx = int((heading_deg % 360.0) / 22.5 + 0.5) % len(headings)
    return headings[idx]


class MotionSimNode(Node):
    """Publish smooth, synthetic IMU pose and heading topics for GUI testing."""

    def __init__(self):
        super().__init__('motion_sim')

        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('roll_amplitude_deg', 15.0)
        self.declare_parameter('pitch_amplitude_deg', 10.0)
        self.declare_parameter('yaw_rate_dps', 15.0)
        self.declare_parameter('roll_period_sec', 30.0)
        self.declare_parameter('pitch_period_sec', 24.0)
        self.declare_parameter('roll_min_deg', float('nan'))
        self.declare_parameter('roll_max_deg', float('nan'))
        self.declare_parameter('pitch_min_deg', float('nan'))
        self.declare_parameter('pitch_max_deg', float('nan'))
        self.declare_parameter('yaw_min_deg', float('nan'))
        self.declare_parameter('yaw_max_deg', float('nan'))

        rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.roll_amp = math.radians(float(self.get_parameter('roll_amplitude_deg').value))
        self.pitch_amp = math.radians(float(self.get_parameter('pitch_amplitude_deg').value))
        self.yaw_rate = math.radians(float(self.get_parameter('yaw_rate_dps').value))
        self.roll_period = max(1.0, float(self.get_parameter('roll_period_sec').value))
        self.pitch_period = max(1.0, float(self.get_parameter('pitch_period_sec').value))
        self.roll_bounds = self._bounds_from_params('roll_min_deg', 'roll_max_deg')
        self.pitch_bounds = self._bounds_from_params('pitch_min_deg', 'pitch_max_deg')
        self.yaw_bounds = self._bounds_from_params('yaw_min_deg', 'yaw_max_deg')

        period = 1.0 / max(0.1, rate_hz)
        self.start_time = self.get_clock().now()
        self.last_update = self.start_time
        self.yaw_value = math.radians(self._initial_yaw_deg())
        self.yaw_direction = 1.0

        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.euler_pub = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_pub = self.create_publisher(String, 'imu/heading', 10)
        self.health_pub = self.create_publisher(String, 'imu/health_status', health_qos)
        self.mode_pub = self.create_publisher(String, 'imu/mode', 1)

        self.create_timer(period, self._publish)

        # Prime static status so late subscribers see something immediately.
        self._publish_status_once()

    def _bounds_from_params(self, min_name: str, max_name: str):
        min_deg = float(self.get_parameter(min_name).value)
        max_deg = float(self.get_parameter(max_name).value)
        if math.isnan(min_deg) or math.isnan(max_deg) or min_deg >= max_deg:
            return None
        return math.radians(min_deg), math.radians(max_deg)

    def _initial_yaw_deg(self) -> float:
        if self.yaw_bounds:
            return math.degrees(self.yaw_bounds[0])
        return 0.0

    def _elapsed(self) -> float:
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds / 1e9

    def _publish(self):
        t = self._elapsed()
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_update).nanoseconds / 1e9)
        self.last_update = now

        roll = self.roll_amp * math.sin((2.0 * math.pi / self.roll_period) * t)
        pitch = self.pitch_amp * math.sin((2.0 * math.pi / self.pitch_period) * t)
        roll = self._clamp_if_needed(roll, self.roll_bounds)
        pitch = self._clamp_if_needed(pitch, self.pitch_bounds)

        if self.yaw_bounds:
            self.yaw_value += self.yaw_direction * self.yaw_rate * dt
            lower, upper = self.yaw_bounds
            if self.yaw_value > upper:
                self.yaw_value = upper
                self.yaw_direction *= -1.0
            elif self.yaw_value < lower:
                self.yaw_value = lower
                self.yaw_direction *= -1.0
            yaw = self.yaw_value
        else:
            yaw = (self.yaw_rate * t) % (2.0 * math.pi)

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        imu_msg = Imu()
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.orientation_covariance[0] = 0.0025  # ~3 deg std-dev^2
        imu_msg.orientation_covariance[4] = 0.0025
        imu_msg.orientation_covariance[8] = 0.0025
        self.imu_pub.publish(imu_msg)

        rpy_deg = Vector3()
        rpy_deg.x = math.degrees(roll)
        rpy_deg.y = math.degrees(pitch)
        rpy_deg.z = math.degrees(yaw)
        self.euler_pub.publish(rpy_deg)

        heading_deg = (math.degrees(yaw) + 360.0) % 360.0
        heading_msg = String()
        heading_msg.data = (
            f"Heading: {heading_to_cardinal(heading_deg)}, {heading_deg:.2f} degrees"
        )
        self.heading_pub.publish(heading_msg)

        # Refresh status periodically so GUI widgets stay current.
        self._publish_status_once()

    def _clamp_if_needed(self, value: float, bounds):
        if not bounds:
            return value
        lower, upper = bounds
        return max(lower, min(upper, value))

    def _publish_status_once(self):
        health_msg = String()
        health_msg.data = 'IMU SIMULATION'
        self.health_pub.publish(health_msg)

        mode_msg = String()
        mode_msg.data = 'SIMULATED'
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
