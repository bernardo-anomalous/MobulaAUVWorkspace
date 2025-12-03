#!/usr/bin/env python3

import math
import threading
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32MultiArray, String
from auv_custom_interfaces.msg import ServoMovementCommand, HoldRequest


class ServoSimNode(Node):
    """Simulate lifecycle, heartbeat, and servo positions expected by the GUI."""

    def __init__(self):
        super().__init__('servo_sim')

        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('glide_position', [90.0, 135.0, 90.0, 135.0, 90.0, 90.0])

        rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.period = 1.0 / max(1.0, rate_hz)
        self.num_servos = 6
        glide_position = self._sanitize_angles(
            self.get_parameter('glide_position').value, fallback=[90.0] * self.num_servos
        )

        self.target_lock = threading.Lock()
        self.current_angles: List[float] = glide_position.copy()
        self.targets: Dict[int, Dict[str, float]] = {
            i: {
                'start': glide_position[i],
                'target': glide_position[i],
                'start_time': 0.0,
                'duration': 0.0,
            }
            for i in range(self.num_servos)
        }
        self.hold_active = False
        self._start_time = self.get_clock().now()
        self._movement_type = 'idle'

        hold_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.angles_pub = self.create_publisher(Float32MultiArray, 'current_servo_angles', 10)
        self.heartbeat_pub = self.create_publisher(String, 'servo_driver/heartbeat', 10)
        self.status_pub = self.create_publisher(String, 'servo_driver_status', 10)
        self.lifecycle_pub = self.create_publisher(String, 'servo_driver/lifecycle_state', 10)

        self.command_sub = self.create_subscription(
            ServoMovementCommand, 'servo_driver_commands', self._command_callback, 10
        )
        self.tail_command_sub = self.create_subscription(
            ServoMovementCommand, 'tail_commands', self._command_callback, 10
        )
        self.hold_sub = self.create_subscription(HoldRequest, 'wing_hold_request', self._hold_callback, hold_qos)

        self.create_timer(self.period, self._tick)
        self.create_timer(1.5, self._publish_heartbeat)
        self.create_timer(2.0, self._publish_lifecycle_state)

        self._publish_status('starting up')
        self._publish_angles()
        self._publish_heartbeat()
        self._publish_lifecycle_state()

    def _sanitize_angles(self, values, fallback: List[float]) -> List[float]:
        cleaned = list(fallback)
        try:
            for idx in range(min(len(values), len(fallback))):
                cleaned[idx] = float(values[idx])
        except Exception:
            pass
        return cleaned

    def _seconds_since_start(self) -> float:
        now = self.get_clock().now()
        return (now - self._start_time).nanoseconds / 1e9

    def _publish_lifecycle_state(self):
        msg = String()
        msg.data = 'active'
        self.lifecycle_pub.publish(msg)

    def _publish_heartbeat(self):
        msg = String()
        msg.data = f"heartbeat @ {self.get_clock().now().to_msg().sec}"
        self.heartbeat_pub.publish(msg)

    def _publish_status(self, text: str, force: bool = False):
        msg = String()
        msg.data = text
        if force or msg.data != getattr(self, '_last_status', None):
            self.status_pub.publish(msg)
            self._last_status = msg.data

    def _command_callback(self, msg: ServoMovementCommand):
        if self.hold_active:
            self._publish_status('hold active; ignoring command', force=True)
            return

        with self.target_lock:
            for idx, servo in enumerate(msg.servo_numbers):
                if servo >= self.num_servos:
                    continue
                target = msg.target_angles[idx] if idx < len(msg.target_angles) else math.nan
                duration = msg.durations[idx] if idx < len(msg.durations) else 0.0
                if math.isnan(target):
                    continue
                duration = max(0.0, float(duration))
                self.targets[servo] = {
                    'start': self.current_angles[servo],
                    'target': float(target),
                    'start_time': self._seconds_since_start(),
                    'duration': duration,
                }
                self._movement_type = msg.movement_type or 'commanded'
                self._publish_status(f"moving: {self._movement_type}")

    def _hold_callback(self, msg: HoldRequest):
        self.hold_active = bool(msg.hold)
        if self.hold_active:
            self._publish_status('hold engaged', force=True)
        else:
            self._publish_status('hold released', force=True)

    def _tick(self):
        now = self._seconds_since_start()
        updated = False

        with self.target_lock:
            for idx in range(self.num_servos):
                target = self.targets[idx]
                duration = target['duration']
                if duration <= 0.0:
                    self.current_angles[idx] = target['target']
                    continue

                t = max(0.0, min(1.0, (now - target['start']) / duration))
                eased = t * t * (3 - 2 * t)  # smoothstep
                new_angle = target['start'] + (target['target'] - target['start']) * eased
                if abs(new_angle - self.current_angles[idx]) > 1e-3:
                    self.current_angles[idx] = new_angle
                    updated = True

        if updated:
            self._publish_angles()

    def _publish_angles(self):
        msg = Float32MultiArray()
        msg.data = list(self.current_angles)
        self.angles_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
