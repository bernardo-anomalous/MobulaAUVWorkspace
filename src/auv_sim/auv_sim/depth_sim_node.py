#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import SetParametersResult
from std_msgs.msg import Float32


class DepthSimNode(Node):
    """Publish slowly varying water/air depth readings."""

    def __init__(self):
        super().__init__('depth_sim')

        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('mean_depth_m', 1.2)
        self.declare_parameter('depth_amplitude_m', 0.6)
        self.declare_parameter('oscillation_period_sec', 40.0)
        self.declare_parameter('enable_depth_target', False)
        self.declare_parameter('target_depth_m', 4.0)
        self.declare_parameter('target_rate_m_per_s', 0.15)

        rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.mean_depth = float(self.get_parameter('mean_depth_m').value)
        self.depth_amp = abs(float(self.get_parameter('depth_amplitude_m').value))
        self.period = max(1.0, float(self.get_parameter('oscillation_period_sec').value))
        self.target_enabled = bool(self.get_parameter('enable_depth_target').value)
        self.target_depth = float(self.get_parameter('target_depth_m').value)
        self.target_rate = max(0.0, float(self.get_parameter('target_rate_m_per_s').value))

        self.publisher_ = self.create_publisher(Float32, '/depth', 10)
        self.air_publisher_ = self.create_publisher(Float32, '/depth_air', 10)

        self.start_time = self.get_clock().now()
        self.last_update = self.start_time
        self.current_depth = self.mean_depth
        self.add_on_set_parameters_callback(self._on_parameter_update)
        self.create_timer(1.0 / max(0.1, rate_hz), self._publish)

    def _elapsed(self) -> float:
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds / 1e9

    def _publish(self):
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_update).nanoseconds / 1e9)
        self.last_update = now

        if self.target_enabled:
            delta = self.target_depth - self.current_depth
            step = math.copysign(min(abs(delta), self.target_rate * dt), delta) if self.target_rate > 0 else delta
            self.current_depth += step
        else:
            t = self._elapsed()
            self.current_depth = self.mean_depth + self.depth_amp * math.sin((2.0 * math.pi / self.period) * t)

        water_msg = Float32()
        water_msg.data = float(self.current_depth)
        self.publisher_.publish(water_msg)

        air_msg = Float32()
        # Use a mirrored curve to resemble altitude when the vehicle is out of water.
        air_msg.data = float(-self.current_depth * 0.8)
        self.air_publisher_.publish(air_msg)

    def _on_parameter_update(self, params):
        for param in params:
            if param.name == 'enable_depth_target' and param.type_ == param.Type.BOOL:
                self.target_enabled = bool(param.value)
            elif param.name == 'target_depth_m' and param.type_ in (param.Type.DOUBLE, param.Type.INTEGER):
                self.target_depth = float(param.value)
            elif param.name == 'target_rate_m_per_s' and param.type_ in (param.Type.DOUBLE, param.Type.INTEGER):
                self.target_rate = max(0.0, float(param.value))
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = DepthSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
