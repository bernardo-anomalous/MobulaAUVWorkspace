#!/usr/bin/env python3
"""System monitor node for Mobula AUV."""

from __future__ import annotations

import datetime as _datetime
import importlib
import math
import os
import pathlib
import re
import shutil
import signal
import subprocess
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile

from std_msgs.msg import String

psutil_spec = importlib.util.find_spec('psutil')
if psutil_spec is not None:  # pragma: no cover - optional dependency in CI
    psutil = importlib.import_module('psutil')  # type: ignore
else:  # pragma: no cover - graceful fallback on systems without psutil
    psutil = None

from auv_custom_interfaces.msg import SystemHealth


class SystemMonitorNode(Node):
    """Monitor system health and publish structured telemetry."""

    _THROTTLE_REGEX = re.compile(r"0x([0-9a-fA-F]+)")

    def __init__(self) -> None:
        super().__init__('system_monitor_node')

        self.fast_poll_sec = self.declare_parameter('fast_poll_sec', 1.0).value
        self.slow_poll_sec = self.declare_parameter('slow_poll_sec', 10.0).value
        self.event_window_sec = self.declare_parameter('event_window_sec', 300.0).value
        self.max_log_files = int(self.declare_parameter('max_log_files', 3).value)
        self.include_prev_boot = bool(self.declare_parameter('include_prev_boot', True).value)
        default_log_dir = os.path.expanduser('~/MobulaAUVWorkspace/src/auv_pkg/auv_pkg/system_monitor_logs')
        self.log_dir = pathlib.Path(self.declare_parameter('log_dir', default_log_dir).value)
        self.summary_topic = self.declare_parameter('summary_topic', '/system/summary').value
        self.events_topic = self.declare_parameter('events_topic', '/system/events').value
        self.default_mem_total_mb = float(self.declare_parameter('default_mem_total_mb', 4096.0).value)
        self.default_swap_total_mb = float(self.declare_parameter('default_swap_total_mb', 2048.0).value)
        self.default_disk_total_gb = float(self.declare_parameter('default_disk_total_gb', 64.0).value)

        self.log_dir.mkdir(parents=True, exist_ok=True)

        qos_persistent = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.summary_pub = self.create_publisher(String, self.summary_topic, 10)
        self.health_pub = self.create_publisher(SystemHealth, '/system/health', 10)
        self.events_pub = self.create_publisher(String, self.events_topic, 10)
        self.prev_shutdown_pub = self.create_publisher(String, '/system/previous_shutdown', qos_profile=qos_persistent)

        self.session_start_ts = _datetime.datetime.now()
        self.session_log_entries: List[str] = []
        self.active_warnings: set[str] = set()
        neg_inf = float('-inf')
        self.last_trigger_times: Dict[str, float] = {
            'undervoltage': neg_inf,
            'overvoltage': neg_inf,
            'throttled': neg_inf,
            'overtemp': neg_inf,
            'frequency_capped': neg_inf,
        }
        self.prev_states: Dict[str, bool] = {key: False for key in self.last_trigger_times}
        self.event_counters = {
            'undervoltage_events': 0,
            'overvoltage_events': 0,
            'throttled_events': 0,
            'overtemp_events': 0,
        }

        self.max_cpu_temp_c = 0.0
        self.current_uptime_sec = 0.0
        self.overvoltage_supported = False
        self.warned_missing_vcgencmd = False
        self.warned_missing_journalctl = False
        self.psutil_warning_logged = False

        self.cached_metrics: Dict[str, object] = {
            'cpu_load_total_percent': 0.0,
            'cpu_load_per_core': [],
            'cpu_freq_mhz': 0.0,
            'mem_used_mb': 0.0,
            'mem_total_mb': self.default_mem_total_mb,
            'swap_used_mb': 0.0,
            'swap_total_mb': self.default_swap_total_mb,
            'disk_used_percent': 0.0,
            'disk_free_gb': 0.0,
            'disk_total_gb': self.default_disk_total_gb,
            'primary_interface': 'unknown',
            'tx_rate_kbps': 0.0,
            'rx_rate_kbps': 0.0,
            'tx_errors': 0,
            'rx_errors': 0,
            'link_up': False,
            'os_release': self._read_os_release(),
            'kernel_version': self._read_kernel_version(),
            'firmware_version': self._read_firmware_version(),
            'last_boot_reason': 'unknown',
            'startup_status': 'running',
        }

        self.previous_shutdown_message = 'previous_shutdown: NOMINAL'
        self.previous_shutdown_reason = 'unknown'

        self.latest_fast_metrics: Dict[str, object] = {
            'cpu_temp_c': float('nan'),
            'gpu_temp_c': float('nan'),
            'undervoltage_detected': False,
            'undervoltage_recent': False,
            'undervoltage_history': False,
            'overvoltage_detected': False,
            'overvoltage_recent': False,
            'overvoltage_history': False,
            'throttled': False,
            'throttled_recent': False,
            'throttled_history': False,
            'frequency_capped': False,
            'frequency_capped_recent': False,
            'frequency_capped_history': False,
            'overtemp': False,
            'overtemp_recent': False,
            'soft_temp_limit': False,
            'healthy': True,
            'state': 'NOMINAL',
            'status_summary': 'initialising',
        }

        self.primary_interface = 'unknown'
        self._prev_net_sample: Optional[Tuple[str, float, int, int]] = None
        self.shutdown_initiated = False

        if psutil is not None:
            try:
                psutil.cpu_percent(interval=None)
            except Exception:  # pragma: no cover - psutil edge case
                pass

        if self.include_prev_boot:
            self._analyze_previous_boot()
        else:
            self.previous_shutdown_reason = 'nominal (analysis disabled)'
            try:
                self.prev_shutdown_pub.publish(String(data=self.previous_shutdown_message))
            except Exception:
                pass
            self._log_previous_shutdown_summary()

        self.fast_timer = self.create_timer(self.fast_poll_sec, self._fast_timer_cb)
        self.slow_timer = self.create_timer(self.slow_poll_sec, self._slow_timer_cb)

        signal.signal(signal.SIGINT, self._handle_sigint)

    # ------------------------------------------------------------------
    # Helper methods for system information

    def _handle_sigint(self, signum, frame) -> None:  # pragma: no cover - signal handling
        del signum, frame
        self.get_logger().info('SIGINT received, preparing shutdown...')
        rclpy.shutdown()

    def _read_os_release(self) -> str:
        path = pathlib.Path('/etc/os-release')
        if not path.exists():
            return 'unknown'
        try:
            data = path.read_text().splitlines()
            info = {}
            for line in data:
                if '=' in line:
                    key, value = line.split('=', 1)
                    info[key.strip()] = value.strip().strip('"')
            name = info.get('PRETTY_NAME') or info.get('NAME')
            if name:
                return name
        except Exception:
            pass
        return 'unknown'

    def _read_kernel_version(self) -> str:
        try:
            return os.uname().release
        except Exception:
            return 'unknown'

    def _read_firmware_version(self) -> str:
        if shutil.which('vcgencmd') is None:
            return 'unknown'
        try:
            result = subprocess.run(['vcgencmd', 'version'], capture_output=True, text=True, check=False)
            if result.stdout:
                return result.stdout.splitlines()[0].strip()
        except Exception:
            pass
        return 'unknown'

    def _read_throttled_flags(self) -> Tuple[Optional[int], Dict[str, bool]]:
        if shutil.which('vcgencmd') is None:
            if not self.warned_missing_vcgencmd:
                self.get_logger().info('Note: vcgencmd not found; skipping Pi firmware power flags.')
                self.warned_missing_vcgencmd = True
            return None, {
                'undervoltage': False,
                'undervoltage_history': False,
                'throttled': False,
                'throttled_history': False,
                'frequency_capped': False,
                'frequency_capped_history': False,
                'soft_temp_limit': False,
            }
        try:
            result = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True, text=True, check=False)
            output = result.stdout.strip()
            match = self._THROTTLE_REGEX.search(output)
            if not match:
                return None, {
                    'undervoltage': False,
                    'undervoltage_history': False,
                    'throttled': False,
                    'throttled_history': False,
                    'frequency_capped': False,
                    'frequency_capped_history': False,
                    'soft_temp_limit': False,
                }
            value = int(match.group(1), 16)
            flags = {
                'undervoltage': bool(value & (1 << 0)),
                'undervoltage_history': bool(value & (1 << 16)),
                'frequency_capped': bool(value & (1 << 1)),
                'frequency_capped_history': bool(value & (1 << 17)),
                'throttled': bool(value & (1 << 2)),
                'throttled_history': bool(value & (1 << 18)),
                'soft_temp_limit': bool(value & (1 << 3) or value & (1 << 19)),
            }
            return value, flags
        except Exception:
            return None, {
                'undervoltage': False,
                'undervoltage_history': False,
                'throttled': False,
                'throttled_history': False,
                'frequency_capped': False,
                'frequency_capped_history': False,
                'soft_temp_limit': False,
            }

    def _read_cpu_temp(self) -> Optional[float]:
        thermal_zone = pathlib.Path('/sys/class/thermal/thermal_zone0/temp')
        try:
            if thermal_zone.exists():
                raw = thermal_zone.read_text().strip()
                return float(raw) / 1000.0
        except Exception:
            pass
        if shutil.which('vcgencmd') is None:
            return None
        try:
            result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True, check=False)
            if result.stdout:
                match = re.search(r"([0-9]+\.[0-9]+)", result.stdout)
                if match:
                    return float(match.group(1))
        except Exception:
            pass
        return None

    def _read_gpu_temp(self) -> Optional[float]:
        if shutil.which('vcgencmd') is None:
            return None
        try:
            result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True, check=False)
            if result.stdout:
                match = re.search(r"([0-9]+\.[0-9]+)", result.stdout)
                if match:
                    return float(match.group(1))
        except Exception:
            pass
        return None

    def _read_uptime(self) -> float:
        try:
            with open('/proc/uptime', 'r', encoding='utf-8') as handle:
                line = handle.readline()
                if line:
                    return float(line.split()[0])
        except Exception:
            pass
        return 0.0

    def _determine_primary_interface(self) -> str:
        if psutil is None:
            return 'unknown'
        try:
            stats = psutil.net_if_stats()
            for name, info in stats.items():
                if name == 'lo':
                    continue
                if info.isup:
                    return name
            if 'eth0' in stats:
                return 'eth0'
            if stats:
                return next(iter(stats.keys()))
        except Exception:
            pass
        return 'unknown'

    def append_log_entry(self, text: str) -> None:
        timestamp = _datetime.datetime.now().strftime('%H:%M:%S')
        entry = f'[{timestamp}] {text}'
        self.session_log_entries.append(entry)

    def maybe_warn(self, condition: bool, message: str) -> None:
        if condition and message not in self.active_warnings:
            self.get_logger().warn(message)
            try:
                self.events_pub.publish(String(data=message))
            except Exception:
                pass
            self.active_warnings.add(message)
            self.append_log_entry(f'⚠ {message}')

    def _log_previous_shutdown_summary(self) -> None:
        message = self.previous_shutdown_message
        if not message:
            return

        reason = (self.previous_shutdown_reason or '').lower()
        if 'nominal' in reason and '⚠' not in message:
            self.get_logger().info(f'Previous shutdown findings: {message}')
        else:
            self.get_logger().warn(f'Previous shutdown findings: {message}')

    def _analyze_previous_boot(self) -> None:
        issues: List[str] = []
        log_lines: List[str] = []

        journal_available = shutil.which('journalctl') is not None
        if journal_available:
            try:
                result = subprocess.run(
                    ['journalctl', '-b', '-1', '-p', '3..4', '--no-pager'],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                if result.stdout:
                    log_lines.extend(result.stdout.splitlines())
                lowered = [line.lower() for line in log_lines]
                if any('under-voltage' in line or 'undervoltage' in line for line in lowered):
                    issues.append('Undervoltage detected during previous boot')
                if any('thermal' in line or 'overheat' in line or 'temperature' in line for line in lowered):
                    issues.append('Thermal event recorded before shutdown')
                if any('throttl' in line for line in lowered):
                    issues.append('Throttling recorded before shutdown')
                if any('panic' in line for line in lowered):
                    issues.append('Kernel panic reported in previous session')
            except Exception:
                log_lines.append('journalctl query failed during previous boot analysis.')
        else:
            if not self.warned_missing_journalctl:
                self.get_logger().info('Note: journalctl not available; previous-boot analysis skipped.')
                self.warned_missing_journalctl = True

        _, throttle_flags = self._read_throttled_flags()
        if throttle_flags['undervoltage_history'] and 'Undervoltage detected during previous boot' not in issues:
            issues.append('Undervoltage detected during previous boot')
        if throttle_flags['throttled_history'] and 'Throttling recorded before shutdown' not in issues:
            issues.append('Throttling recorded before shutdown')
        if throttle_flags['soft_temp_limit'] and 'Thermal event recorded before shutdown' not in issues:
            issues.append('Thermal event recorded before shutdown')

        status_message = 'previous_shutdown: NOMINAL'
        if issues:
            status_message = 'previous_shutdown: ⚠ ' + '; '.join(issues)
            for issue in issues:
                self.maybe_warn(True, issue)
            self.previous_shutdown_reason = '; '.join(issues)
        else:
            self.previous_shutdown_reason = 'nominal'

        self.cached_metrics['last_boot_reason'] = self.previous_shutdown_reason

        self.previous_shutdown_message = status_message
        try:
            self.prev_shutdown_pub.publish(String(data=status_message))
        except Exception:
            pass

        self._log_previous_shutdown_summary()

        previous_log = self.log_dir / 'previous_boot.log'
        try:
            with previous_log.open('w', encoding='utf-8') as handle:
                handle.write(f'Previous boot analysis generated {self.session_start_ts.isoformat()}\n')
                handle.write(f'Status: {status_message}\n')
                handle.write('--- journalctl excerpts ---\n')
                for line in log_lines:
                    handle.write(line + '\n')
        except Exception as exc:
            self.get_logger().error(f'Failed to write previous boot log: {exc}')

    # ------------------------------------------------------------------
    # Timer callbacks

    def _fast_timer_cb(self) -> None:
        try:
            now = time.time()
            _, throttle_flags = self._read_throttled_flags()
            cpu_temp = self._read_cpu_temp()
            gpu_temp = self._read_gpu_temp()
            uptime_sec = self._read_uptime()
            self.current_uptime_sec = uptime_sec

            if cpu_temp is not None and not math.isnan(cpu_temp):
                self.max_cpu_temp_c = max(self.max_cpu_temp_c, cpu_temp)

            overtemp = cpu_temp is not None and cpu_temp >= 85.0
            soft_temp_limit = bool(throttle_flags['soft_temp_limit']) or (cpu_temp is not None and cpu_temp >= 80.0)

            states = {
                'undervoltage': throttle_flags['undervoltage'],
                'overvoltage': False,
                'throttled': throttle_flags['throttled'],
                'overtemp': overtemp,
                'frequency_capped': throttle_flags['frequency_capped'],
            }

            for key, active in states.items():
                if active:
                    self.last_trigger_times[key] = now
                    if not self.prev_states.get(key, False):
                        counter_key = f'{key}_events'
                        if counter_key in self.event_counters:
                            self.event_counters[counter_key] += 1
                self.prev_states[key] = active

            if throttle_flags['undervoltage']:
                self.maybe_warn(True, 'Undervoltage detected (check power input)')
            if states['overvoltage']:
                self.maybe_warn(True, 'Overvoltage detected on power rail')
            if throttle_flags['throttled'] and soft_temp_limit:
                temp_text = 'unknown'
                if cpu_temp is not None:
                    temp_text = f'{cpu_temp:.1f}°C'
                self.maybe_warn(True, f'Thermal throttling active at {temp_text}')
            if overtemp:
                temp_val = cpu_temp if cpu_temp is not None else float('nan')
                self.maybe_warn(True, f'CPU temperature critical: {temp_val:.1f}°C')
            if self.event_counters['undervoltage_events'] >= 3:
                self.maybe_warn(True, 'Repeated undervoltage events observed (>=3)')

            undervoltage_recent = (now - self.last_trigger_times['undervoltage']) < self.event_window_sec
            overvoltage_recent = (now - self.last_trigger_times['overvoltage']) < self.event_window_sec
            throttled_recent = (now - self.last_trigger_times['throttled']) < self.event_window_sec
            frequency_recent = (now - self.last_trigger_times['frequency_capped']) < self.event_window_sec
            overtemp_recent = (now - self.last_trigger_times['overtemp']) < self.event_window_sec

            state, reason = self._determine_state(
                throttle_flags['undervoltage'],
                overtemp,
                throttle_flags['throttled'],
                undervoltage_recent,
                throttled_recent,
                overtemp_recent,
            )

            cpu_temp_display = 'n/a' if cpu_temp is None else f'{cpu_temp:.1f}°C'
            cpu_load = self.cached_metrics.get('cpu_load_total_percent', 0.0)
            cpu_freq = self.cached_metrics.get('cpu_freq_mhz', 0.0)
            mem_used = self.cached_metrics.get('mem_used_mb', 0.0)
            mem_total = self.cached_metrics.get('mem_total_mb', self.default_mem_total_mb)
            link_up = self.cached_metrics.get('link_up', False)
            primary_iface = self.cached_metrics.get('primary_interface', 'unknown')

            uptime_fmt = self._format_uptime(uptime_sec)
            uv_segment = f"UV={'TRUE' if throttle_flags['undervoltage'] else 'FALSE'}(5m={'TRUE' if undervoltage_recent else 'FALSE'})"
            ov_segment = ''
            if self.overvoltage_supported or self.latest_fast_metrics['overvoltage_history']:
                ov_segment = f" | OV={'TRUE' if states['overvoltage'] else 'FALSE'}"
                ov_segment += f"(5m={'TRUE' if overvoltage_recent else 'FALSE'})"
            throt_segment = f"Throt={'TRUE' if throttle_flags['throttled'] else 'FALSE'}(hist={'TRUE' if throttle_flags['throttled_history'] else 'FALSE'})"
            cpu_segment = f"CPU={cpu_load:.0f}%@{cpu_freq/1000.0:.2f}GHz" if cpu_freq else f"CPU={cpu_load:.0f}%@n/a"
            mem_segment = f"Mem={mem_used:.0f}/{mem_total:.0f}MB"
            link_segment = f"Link={'UP' if link_up else 'DOWN'}"
            state_segment = f"state={state}" + (f"({reason})" if reason else '')

            summary = f"sys: T={cpu_temp_display} | {uv_segment}{ov_segment} | {throt_segment} | {cpu_segment} | {mem_segment} | Up={uptime_fmt} | {link_segment} | {state_segment}"
            self.summary_pub.publish(String(data=summary))

            self.latest_fast_metrics.update({
                'cpu_temp_c': cpu_temp if cpu_temp is not None else float('nan'),
                'gpu_temp_c': gpu_temp if gpu_temp is not None else float('nan'),
                'undervoltage_detected': throttle_flags['undervoltage'],
                'undervoltage_recent': undervoltage_recent,
                'undervoltage_history': throttle_flags['undervoltage_history'],
                'overvoltage_detected': states['overvoltage'],
                'overvoltage_recent': overvoltage_recent,
                'overvoltage_history': self.event_counters['overvoltage_events'] > 0,
                'throttled': throttle_flags['throttled'],
                'throttled_recent': throttled_recent,
                'throttled_history': throttle_flags['throttled_history'],
                'frequency_capped': throttle_flags['frequency_capped'],
                'frequency_capped_recent': frequency_recent,
                'frequency_capped_history': throttle_flags['frequency_capped_history'],
                'overtemp': overtemp,
                'overtemp_recent': overtemp_recent,
                'soft_temp_limit': soft_temp_limit,
                'healthy': state == 'NOMINAL',
                'state': state,
                'status_summary': summary,
            })
        except Exception as exc:
            self.get_logger().error(f'Fast loop error: {exc}')

    def _determine_state(
        self,
        undervoltage: bool,
        overtemp: bool,
        throttled: bool,
        undervoltage_recent: bool,
        throttled_recent: bool,
        overtemp_recent: bool,
    ) -> Tuple[str, str]:
        reasons_fault: List[str] = []
        reasons_caution: List[str] = []
        if undervoltage:
            reasons_fault.append('Undervoltage')
        if overtemp:
            reasons_fault.append('Overtemp')
        if throttled:
            reasons_fault.append('Throttled')
        if undervoltage_recent and not undervoltage:
            reasons_caution.append('Undervoltage recent')
        if overtemp_recent and not overtemp:
            reasons_caution.append('Overtemp recent')
        if throttled_recent and not throttled:
            reasons_caution.append('Throttled recent')

        if reasons_fault:
            return 'FAULT', '+'.join(reasons_fault)
        if reasons_caution:
            return 'CAUTION', '+'.join(reasons_caution)
        return 'NOMINAL', ''

    def _slow_timer_cb(self) -> None:
        try:
            now = time.time()
            cpu_load_total = 0.0
            cpu_load_per_core: List[float] = []
            cpu_freq_mhz = 0.0
            mem_used_mb = 0.0
            mem_total_mb = self.default_mem_total_mb
            swap_used_mb = 0.0
            swap_total_mb = self.default_swap_total_mb
            disk_used_percent = 0.0
            disk_free_gb = 0.0
            disk_total_gb = self.default_disk_total_gb
            primary_interface = self.primary_interface
            tx_rate_kbps = 0.0
            rx_rate_kbps = 0.0
            tx_errors = 0
            rx_errors = 0
            link_up = False

            if psutil is None:
                if not self.psutil_warning_logged:
                    self.get_logger().warn('psutil not available; CPU/memory/disk metrics limited.')
                    self.psutil_warning_logged = True
            else:
                try:
                    per_core = psutil.cpu_percent(interval=None, percpu=True)
                    cpu_load_per_core = [float(val) for val in per_core]
                    if cpu_load_per_core:
                        cpu_load_total = float(sum(cpu_load_per_core) / len(cpu_load_per_core))
                except Exception:
                    cpu_load_total = 0.0
                    cpu_load_per_core = []
                try:
                    freq = psutil.cpu_freq()
                    if freq:
                        cpu_freq_mhz = float(freq.current)
                except Exception:
                    cpu_freq_mhz = 0.0
                try:
                    mem = psutil.virtual_memory()
                    mem_used_mb = float(mem.used) / (1024 * 1024)
                    mem_total_mb = float(mem.total) / (1024 * 1024)
                except Exception:
                    mem_used_mb = 0.0
                try:
                    swap = psutil.swap_memory()
                    swap_used_mb = float(swap.used) / (1024 * 1024)
                    swap_total_mb = float(swap.total) / (1024 * 1024)
                except Exception:
                    swap_used_mb = 0.0
                try:
                    disk = psutil.disk_usage('/')
                    disk_used_percent = float(disk.percent)
                    disk_total_gb = float(disk.total) / (1024 ** 3)
                    disk_free_gb = float(disk.free) / (1024 ** 3)
                except Exception:
                    disk_used_percent = 0.0
                try:
                    primary_interface = self._determine_primary_interface()
                    net_counters = psutil.net_io_counters(pernic=True)
                    iface = primary_interface if primary_interface in net_counters else 'lo'
                    counters = net_counters.get(iface)
                    stats = psutil.net_if_stats().get(iface)
                    if counters:
                        if self._prev_net_sample and self._prev_net_sample[0] == iface:
                            prev_iface, prev_time, prev_tx, prev_rx = self._prev_net_sample
                            dt = now - prev_time
                            if dt > 0:
                                tx_rate_kbps = ((counters.bytes_sent - prev_tx) * 8.0) / (1000.0 * dt)
                                rx_rate_kbps = ((counters.bytes_recv - prev_rx) * 8.0) / (1000.0 * dt)
                        self._prev_net_sample = (iface, now, counters.bytes_sent, counters.bytes_recv)
                        tx_errors = int(counters.errout)
                        rx_errors = int(counters.errin)
                    link_up = bool(stats.isup) if stats else False
                except Exception:
                    link_up = False

            self.primary_interface = primary_interface
            self.cached_metrics.update({
                'cpu_load_total_percent': cpu_load_total,
                'cpu_load_per_core': cpu_load_per_core,
                'cpu_freq_mhz': cpu_freq_mhz,
                'mem_used_mb': mem_used_mb,
                'mem_total_mb': mem_total_mb,
                'swap_used_mb': swap_used_mb,
                'swap_total_mb': swap_total_mb,
                'disk_used_percent': disk_used_percent,
                'disk_free_gb': disk_free_gb,
                'disk_total_gb': disk_total_gb,
                'primary_interface': primary_interface,
                'tx_rate_kbps': tx_rate_kbps,
                'rx_rate_kbps': rx_rate_kbps,
                'tx_errors': tx_errors,
                'rx_errors': rx_errors,
                'link_up': link_up,
            })

            health_msg = SystemHealth()
            health_msg.stamp = self.get_clock().now().to_msg()
            health_msg.undervoltage_detected = bool(self.latest_fast_metrics['undervoltage_detected'])
            health_msg.undervoltage_recent = bool(self.latest_fast_metrics['undervoltage_recent'])
            health_msg.undervoltage_history = bool(self.latest_fast_metrics['undervoltage_history'])
            health_msg.overvoltage_detected = bool(self.latest_fast_metrics['overvoltage_detected'])
            health_msg.overvoltage_recent = bool(self.latest_fast_metrics['overvoltage_recent'])
            health_msg.overvoltage_history = bool(self.latest_fast_metrics['overvoltage_history'])
            health_msg.throttled = bool(self.latest_fast_metrics['throttled'])
            health_msg.throttled_recent = bool(self.latest_fast_metrics['throttled_recent'])
            health_msg.throttled_history = bool(self.latest_fast_metrics['throttled_history'])
            health_msg.frequency_capped = bool(self.latest_fast_metrics['frequency_capped'])
            health_msg.frequency_capped_recent = bool(self.latest_fast_metrics['frequency_capped_recent'])
            health_msg.frequency_capped_history = bool(self.latest_fast_metrics['frequency_capped_history'])
            health_msg.cpu_temp_c = float(self.latest_fast_metrics['cpu_temp_c'])
            health_msg.gpu_temp_c = float(self.latest_fast_metrics['gpu_temp_c'])
            health_msg.overtemp = bool(self.latest_fast_metrics['overtemp'])
            health_msg.overtemp_recent = bool(self.latest_fast_metrics['overtemp_recent'])
            health_msg.soft_temp_limit = bool(self.latest_fast_metrics['soft_temp_limit'])
            health_msg.cpu_load_total_percent = float(self.cached_metrics['cpu_load_total_percent'])
            health_msg.cpu_load_per_core = [float(val) for val in self.cached_metrics['cpu_load_per_core']]
            health_msg.cpu_freq_mhz = float(self.cached_metrics['cpu_freq_mhz'])
            health_msg.uptime_sec = float(self.current_uptime_sec)
            health_msg.mem_used_mb = float(self.cached_metrics['mem_used_mb'])
            health_msg.mem_total_mb = float(self.cached_metrics['mem_total_mb'])
            health_msg.swap_used_mb = float(self.cached_metrics['swap_used_mb'])
            health_msg.swap_total_mb = float(self.cached_metrics['swap_total_mb'])
            health_msg.disk_used_percent = float(self.cached_metrics['disk_used_percent'])
            health_msg.disk_free_gb = float(self.cached_metrics['disk_free_gb'])
            health_msg.disk_total_gb = float(self.cached_metrics['disk_total_gb'])
            health_msg.primary_interface = str(self.cached_metrics['primary_interface'])
            health_msg.tx_rate_kbps = float(self.cached_metrics['tx_rate_kbps'])
            health_msg.rx_rate_kbps = float(self.cached_metrics['rx_rate_kbps'])
            health_msg.tx_errors = int(self.cached_metrics['tx_errors'])
            health_msg.rx_errors = int(self.cached_metrics['rx_errors'])
            health_msg.link_up = bool(self.cached_metrics['link_up'])
            health_msg.os_release = str(self.cached_metrics['os_release'])
            health_msg.kernel_version = str(self.cached_metrics['kernel_version'])
            health_msg.firmware_version = str(self.cached_metrics['firmware_version'])
            health_msg.last_boot_reason = self.previous_shutdown_reason
            health_msg.startup_status = str(self.cached_metrics['startup_status'])
            health_msg.max_cpu_temp_c = float(self.max_cpu_temp_c)
            health_msg.undervoltage_events = int(self.event_counters['undervoltage_events'])
            health_msg.overvoltage_events = int(self.event_counters['overvoltage_events'])
            health_msg.throttled_events = int(self.event_counters['throttled_events'])
            health_msg.overtemp_events = int(self.event_counters['overtemp_events'])
            health_msg.healthy = bool(self.latest_fast_metrics['healthy'])
            health_msg.status_summary = str(self.latest_fast_metrics['status_summary'])

            self.health_pub.publish(health_msg)
        except Exception as exc:
            self.get_logger().error(f'Slow loop error: {exc}')

    def _format_uptime(self, uptime_sec: float) -> str:
        hours = int(uptime_sec) // 3600
        minutes = (int(uptime_sec) % 3600) // 60
        return f"{hours:02d}h{minutes:02d}m"

    # ------------------------------------------------------------------
    # Shutdown handling

    def shutdown(self) -> None:
        if self.shutdown_initiated:
            return
        self.shutdown_initiated = True
        try:
            self._write_session_log()
        except Exception as exc:
            self.get_logger().error(f'Failed to write session log: {exc}')

    def _write_session_log(self) -> None:
        end_ts = _datetime.datetime.now()
        duration = end_ts - self.session_start_ts
        log_path = self.log_dir / f"session_{self.session_start_ts.strftime('%Y%m%d_%H%M%S')}.log"
        try:
            with log_path.open('w', encoding='utf-8') as handle:
                handle.write(f'Session start: {self.session_start_ts.isoformat()}\n')
                handle.write(f'Session end: {end_ts.isoformat()}\n')
                handle.write(f'Uptime: {self._format_uptime(self.current_uptime_sec)}\n')
                handle.write(f'Session duration: {duration}\n')
                handle.write(f'Max CPU temperature: {self.max_cpu_temp_c:.1f}°C\n')
                handle.write('Event counters:\n')
                for key, value in self.event_counters.items():
                    handle.write(f'  {key}: {value}\n')
                handle.write('Warnings:\n')
                if self.session_log_entries:
                    for entry in self.session_log_entries:
                        handle.write(f'  {entry}\n')
                else:
                    handle.write('  (none)\n')
        except Exception as exc:
            self.get_logger().error(f'Error writing session log: {exc}')
            return

        try:
            self._rotate_logs()
        except Exception as exc:
            self.get_logger().error(f'Error rotating logs: {exc}')

    def _rotate_logs(self) -> None:
        session_logs = sorted(self.log_dir.glob('session_*.log'), key=lambda p: p.stat().st_mtime, reverse=True)
        for index, path in enumerate(session_logs):
            if index >= self.max_log_files:
                try:
                    path.unlink()
                except Exception:
                    pass


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
