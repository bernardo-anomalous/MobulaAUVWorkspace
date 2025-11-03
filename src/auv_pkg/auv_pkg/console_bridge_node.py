"""Console bridge node for formatting ROS log messages."""
from __future__ import annotations

from typing import Final

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from rcl_interfaces.msg import Log
from std_msgs.msg import String


_LEVEL_MAP: Final = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}


class ConsoleBridgeNode(Node):
    """Subscribe to /rosout, reformat entries, and republish as plain text."""

    def __init__(self) -> None:
        super().__init__("console_bridge_node")
        publisher_qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription_qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._publisher = self.create_publisher(
            String, "console_bridge/log", publisher_qos
        )
        self.create_subscription(
            Log, "/rosout", self._handle_log, subscription_qos
        )

    def _handle_log(self, msg: Log) -> None:
        level = _LEVEL_MAP.get(msg.level, f"LEVEL_{msg.level}")
        timestamp = msg.stamp.sec + msg.stamp.nanosec / 1_000_000_000
        formatted = (
            f"[{msg.name}] [{level}] [{timestamp:.9f}] {msg.msg}".rstrip()
        )
        self._publisher.publish(String(data=formatted))


def main() -> None:
    rclpy.init()
    node = ConsoleBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - convenience entrypoint
    main()
