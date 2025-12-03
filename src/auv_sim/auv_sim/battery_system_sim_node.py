#!/usr/bin/env python3

import json
import math
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float32, String
from auv_custom_interfaces.msg import SystemHealth


BATTERY_PROFILES = {
    'standard': {
        'base_voltage_compute': 12.3,
        'base_voltage_actuation': 12.1,
        'discharge_rate_v_per_hr': 0.15,
        'capacity_mAh': 5300.0,
        'idle_current_compute': 0.8,
        'idle_current_actuation': 1.4,
    },
    'high_capacity': {
        'base_voltage_compute': 12.5,
        'base_voltage_actuation': 12.3,
        'discharge_rate_v_per_hr': 0.1,
        'capacity_mAh': 7000.0,
        'idle_current_compute': 0.75,
        'idle_current_actuation': 1.3,
    },
    'aged': {
        'base_voltage_compute': 12.0,
        'base_voltage_actuation': 11.8,
        'discharge_rate_v_per_hr': 0.2,
        'capacity_mAh': 4200.0,
        'idle_current_compute': 0.85,
        'idle_current_actuation': 1.5,
    },
}


@dataclass
class PackState:
    name: str
    base_voltage: float
    discharge_rate_v_per_hr: float
    idle_current: float
    capacity_mAh: float


class BatterySystemSimNode(Node):
    """Lightweight simulator for battery and system-health topics."""

    def __init__(self):
        super().__init__('battery_system_sim')

        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('battery_type', 'standard')
        self.declare_parameter('discharge_rate_v_per_hr', 0.15)
        self.declare_parameter('capacity_mAh', BATTERY_PROFILES['standard']['capacity_mAh'])
        self.declare_parameter('base_voltage_compute', BATTERY_PROFILES['standard']['base_voltage_compute'])
        self.declare_parameter('base_voltage_actuation', BATTERY_PROFILES['standard']['base_voltage_actuation'])

        rate_hz = float(self.get_parameter('update_rate_hz').value)
        battery_type = str(self.get_parameter('battery_type').value).lower()
        profile = BATTERY_PROFILES.get(battery_type, BATTERY_PROFILES['standard'])

        drop_rate_param = float(self.get_parameter('discharge_rate_v_per_hr').value)
        drop_rate = max(0.0, drop_rate_param if drop_rate_param >= 0.0 else profile['discharge_rate_v_per_hr'])

        base_compute_param = float(self.get_parameter('base_voltage_compute').value)
        base_actuation_param = float(self.get_parameter('base_voltage_actuation').value)
        base_compute = base_compute_param if base_compute_param > 0 else profile['base_voltage_compute']
        base_actuation = base_actuation_param if base_actuation_param > 0 else profile['base_voltage_actuation']

        capacity_param = float(self.get_parameter('capacity_mAh').value)
        capacity_mAh = capacity_param if capacity_param > 0 else profile['capacity_mAh']

        self.packs: Dict[str, PackState] = {
            'compute': PackState('compute', base_compute, drop_rate, profile['idle_current_compute'], capacity_mAh),
            'actuation': PackState('actuation', base_actuation, drop_rate, profile['idle_current_actuation'], capacity_mAh),
        }

        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pack_publishers = {}
        for pack in self.packs.values():
            base = f"/battery/{pack.name}"
            self.pack_publishers[pack.name] = {
                'summary': self.create_publisher(String, f"{base}/summary", qos_transient),
                'present': self.create_publisher(Bool, f"{base}/present", qos_transient),
                'voltage': self.create_publisher(Float32, f"{base}/voltage_V", 10),
                'current': self.create_publisher(Float32, f"{base}/current_A", 10),
                'power': self.create_publisher(Float32, f"{base}/power_W", 10),
                'soc': self.create_publisher(Float32, f"{base}/soc_pct", 10),
                'used_mAh': self.create_publisher(Float32, f"{base}/used_mAh", 10),
                'used_Wh': self.create_publisher(Float32, f"{base}/used_Wh", 10),
                'eta_min': self.create_publisher(Float32, f"{base}/eta_min", 10),
                'estimation': self.create_publisher(String, f"{base}/behavior_based_estimation", 10),
                'voltage_state': self.create_publisher(String, f"{base}/voltage_state", qos_transient),
                'current_monitor': self.create_publisher(String, f"{base}/current_monitor", qos_transient),
                'current_critical': self.create_publisher(String, f"{base}/current_draw_critical", qos_transient),
            }

        qos_persistent = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.summary_pub = self.create_publisher(String, '/system/summary', 10)
        self.health_pub = self.create_publisher(SystemHealth, '/system/health', 10)
        self.events_pub = self.create_publisher(String, '/system/events', 10)
        self.prev_shutdown_pub = self.create_publisher(String, '/system/previous_shutdown', qos_persistent)

        self.start_time = self.get_clock().now()
        self.create_timer(1.0 / max(0.1, rate_hz), self._publish)

        # Publish initial presence states
        self._publish_prev_shutdown()
        self._publish_events("System simulator started.")
        self._publish_system_summary("System OK (simulated)")

    def _elapsed_hours(self) -> float:
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds / 3.6e12

    def _publish(self):
        elapsed_hours = self._elapsed_hours()
        self._publish_packs(elapsed_hours)
        self._publish_system_health(elapsed_hours)

    def _publish_packs(self, elapsed_hours: float):
        for pack in self.packs.values():
            voltage = max(9.5, pack.base_voltage - pack.discharge_rate_v_per_hr * elapsed_hours)
            # Add a gentle ripple to keep plots active.
            voltage += 0.1 * math.sin(elapsed_hours * 2.0 * math.pi)

            current = pack.idle_current + 0.4 * math.sin(elapsed_hours * 4.0 * math.pi)
            power = voltage * current

            estimated_used_mAh = min(
                pack.capacity_mAh,
                max(0.0, elapsed_hours * max(pack.idle_current, abs(current)) * 1000.0),
            )
            soc = max(0.0, 100.0 * (1.0 - estimated_used_mAh / pack.capacity_mAh))
            used_mAh = estimated_used_mAh
            used_Wh = used_mAh / 1000.0 * (voltage / 3.0)
            eta_min = 60.0 * soc / max(1.0, current)  # rough minutes remaining

            pubs = self.pack_publishers[pack.name]
            pubs['present'].publish(Bool(data=True))
            pubs['voltage'].publish(Float32(data=float(voltage)))
            pubs['current'].publish(Float32(data=float(current)))
            pubs['power'].publish(Float32(data=float(power)))
            pubs['soc'].publish(Float32(data=float(soc)))
            pubs['used_mAh'].publish(Float32(data=float(used_mAh)))
            pubs['used_Wh'].publish(Float32(data=float(used_Wh)))
            pubs['eta_min'].publish(Float32(data=float(eta_min)))

            estimate = String()
            estimate.data = (
                f"{pack.name}: avgP~{power:.2f}W ETA~{eta_min/60.0:.1f}h "
                f"(~{eta_min:.0f}min)"
            )
            pubs['estimation'].publish(estimate)

            state_label = 'normal'
            if voltage < 10.5:
                state_label = 'warning'
            if voltage < 10.0:
                state_label = 'critical'

            pubs['voltage_state'].publish(String(data=state_label))
            monitor_payload = json.dumps({'pack': pack.name, 'current_A': current})
            pubs['current_monitor'].publish(String(data=monitor_payload))
            if current > 7.0:
                pubs['current_critical'].publish(String(data=monitor_payload))

            summary = String()
            summary.data = (
                f"{pack.name}: ETA=~{eta_min/60.0:.1f}h | V={voltage:.2f}V "
                f"I={current:.2f}A P~{power:.2f}W SoC={soc:.1f}%"
            )
            pubs['summary'].publish(summary)

    def _publish_system_health(self, elapsed_hours: float):
        health = SystemHealth()
        health.undervoltage_detected = False
        health.undervoltage_recent = False
        health.undervoltage_history = False
        health.overvoltage_detected = False
        health.overvoltage_recent = False
        health.overvoltage_history = False
        health.throttled = False
        health.throttled_recent = False
        health.throttled_history = False
        health.frequency_capped = False
        health.frequency_capped_recent = False
        health.frequency_capped_history = False
        health.cpu_temp_c = 48.0 + 2.0 * math.sin(elapsed_hours * 6.0)
        health.gpu_temp_c = 47.0 + 2.0 * math.sin(elapsed_hours * 6.0 + 0.5)
        health.overtemp = False
        health.overtemp_recent = False
        health.soft_temp_limit = False
        health.cpu_load_total_percent = 12.0 + 5.0 * math.sin(elapsed_hours * 8.0)
        health.cpu_load_per_core = [health.cpu_load_total_percent] * 4
        health.cpu_freq_mhz = 1500.0
        health.uptime_sec = elapsed_hours * 3600.0
        health.mem_used_mb = 820.0
        health.mem_total_mb = 3900.0
        health.swap_used_mb = 0.0
        health.swap_total_mb = 0.0
        health.disk_used_percent = 23.0
        health.disk_free_gb = 24.0
        health.disk_total_gb = 32.0
        health.primary_interface = 'wlan0'
        health.tx_rate_kbps = 30.0
        health.rx_rate_kbps = 28.0
        health.tx_errors = 0
        health.rx_errors = 0
        health.link_up = True
        health.os_release = 'simulated'
        health.kernel_version = 'simulated'
        health.firmware_version = 'simulated'
        health.last_boot_reason = 'simulator'
        health.startup_status = 'OK'
        health.max_cpu_temp_c = max(health.cpu_temp_c, 48.0)
        health.undervoltage_events = 0
        health.overvoltage_events = 0
        health.throttled_events = 0
        health.overtemp_events = 0
        health.healthy = True
        health.status_summary = 'Nominal (simulated)'

        self.health_pub.publish(health)
        self._publish_system_summary(health.status_summary)

    def _publish_events(self, text: str):
        msg = String()
        msg.data = text
        self.events_pub.publish(msg)

    def _publish_prev_shutdown(self):
        msg = String()
        msg.data = 'No previous shutdown (simulated)'
        self.prev_shutdown_pub.publish(msg)

    def _publish_system_summary(self, text: str):
        msg = String()
        msg.data = text
        self.summary_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatterySystemSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
