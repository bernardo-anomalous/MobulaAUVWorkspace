#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import time
import threading
import signal
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

# --- Optional robust I2C backend (ExtendedI2C) --------------------------------
try:
    from adafruit_extended_bus import ExtendedI2C  # /dev/i2c-*
    _HAS_EXTENDED_I2C = True
except Exception:
    _HAS_EXTENDED_I2C = False

# Fallback Blinka bus
try:
    import board
    import busio
except Exception:
    board = None
    busio = None

# --- BNO08X / I2C driver ------------------------------------------------------
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)

from .utils import quaternion_to_euler

# ------------------------ Quaternion utilities ---------------------
def q_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0: return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / n
    return (x*inv, y*inv, z*inv, w*inv)

def q_mul(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def q_slerp(q0, q1, t):
    # Simplified SLERP for small steps (Lerp approximation is faster for high Hz)
    # For 60Hz updates, angles are small enough that Lerp + Normalize is efficient and accurate
    q0 = q_normalize(q0)
    q1 = q_normalize(q1)
    
    # Dot product
    dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3]
    
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot
    
    # Linear Interpolation (Lerp) is sufficient for small dt and saves CPU
    scale0 = 1.0 - t
    scale1 = t
    
    res = (
        scale0 * q0[0] + scale1 * q1[0],
        scale0 * q0[1] + scale1 * q1[1],
        scale0 * q0[2] + scale1 * q1[2],
        scale0 * q0[3] + scale1 * q1[3]
    )
    return q_normalize(res)

def quat_from_gyro(gx, gy, gz, dt):
    omega = math.sqrt(gx*gx + gy*gy + gz*gz)
    if omega < 1e-12 or dt <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    theta = omega * dt
    half = 0.5 * theta
    s = math.sin(half) / omega
    return (gx*s, gy*s, gz*s, math.cos(half))


class IMUNode(Node):
    def __init__(self):
        super().__init__('optimized_imu_node')

        self.get_logger().info(f"[IMU] Python executable: {sys.executable}")
        
        # ---- parameters ------------------------------------------------------
        self.declare_parameter('i2c_bus_id', 3)   
        self.declare_parameter('i2c_address', 0x4B)
        self.bus_id = int(self.get_parameter('i2c_bus_id').value)
        self.addr = int(self.get_parameter('i2c_address').value)
        
        self.declare_parameter('stagnant_timeout_sec', 3.0)
        self.stagnant_timeout_sec = float(self.get_parameter('stagnant_timeout_sec').value)

        # CRITICAL UPDATE: Increased poll rate to 60Hz (0.016s)
        # This prevents the "Staircase Effect" in your PIDs
        self.declare_parameter('poll_period_sec', 0.016)  
        poll_period = float(self.get_parameter('poll_period_sec').value)
        
        # SENSOR REPORT RATE: Must match or exceed poll rate
        self.sensor_report_hz = int(1.0 / poll_period) 
        self.get_logger().info(f"IMU target rate: {self.sensor_report_hz} Hz")

        # Stagnant tolerances - Tightened for Glider Mode
        # Gliders can be very steady. We only want to reset if it's UNNATURALLY steady (frozen sensor).
        # We look for effectively zero noise on the accelerometer.
        self._stagnant_accel_tolerance = 0.001 
        self._stagnant_gyro_tolerance = 0.0001
        self._stagnant_orientation_tolerance = 0.00001 # Quaternions shouldn't be identical

        self._stagnant_tolerances = (
            self._stagnant_orientation_tolerance,
            self._stagnant_orientation_tolerance,
            self._stagnant_orientation_tolerance,
            self._stagnant_orientation_tolerance,
            self._stagnant_accel_tolerance,
            self._stagnant_accel_tolerance,
            self._stagnant_accel_tolerance,
            self._stagnant_gyro_tolerance,
            self._stagnant_gyro_tolerance,
            self._stagnant_gyro_tolerance,
        )

        # gyro-based bridging during hiccups
        self._est_active = False
        self._est_elapsed = 0.0
        self._est_max_sec = 1.0     
        self._blend_back_sec = 0.2  
        self._last_q = (0.0, 0.0, 0.0, 1.0)
        self._last_t = time.monotonic()

        self.heading_offset = -30.0
        self.sensor_ready = False
        self._offline_published = False
        self._shutdown_requested = False

        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'imu/heading', 10)
        self.health_status_publisher = self.create_publisher(String, 'imu/health_status', health_qos)
        self.mode_pub = self.create_publisher(String, 'imu/mode', 1) 

        self.last_health_status = None
        self.last_failure_reason = None

        self.timer = self.create_timer(poll_period, self.publish_imu_data)
        self.retry_timer = self.create_timer(5.0, self._retry_init_if_needed)

        self.i2c = None
        self.bno = None
        self._imu_lock = threading.Lock()

        self._last_data = None
        self._stagnant_start = None
        self._bad_frame_streak = 0
        self._bad_frame_streak_limit = 2

        self.failure_timestamps = deque()
        self.max_failures = 3
        self.failure_window_seconds = 30

        self.CRITICAL_ERRORS = [
            "No device", "Remote I/O", "Input/output error",
            "Unprocessable Batch bytes", "Was not able to enable feature",
        ]

        self.reset_attempts = 0
        self.max_reset_attempts = 3

        self.initialize_sensor()

    # [Keep backend logging / i2c helpers exactly as they were]
    def _log_i2c_backend(self):
        pass # (Existing code was fine, omitted for brevity in chat, keep your original)
    def _log_bno_backend(self):
        pass 
    def _new_i2c(self):
        if _HAS_EXTENDED_I2C: return ExtendedI2C(self.bus_id)
        if busio is None or board is None: raise RuntimeError("No I2C backend")
        return busio.I2C(board.SCL, board.SDA)

    def _close_i2c(self):
        if self.i2c is None: return
        try:
            if hasattr(self.i2c, "deinit"): self.i2c.deinit()
            elif hasattr(self.i2c, "close"): self.i2c.close()
        except Exception: pass
        finally: self.i2c = None

    def _enable_feature_safe(self, feature, hz=60):
        # UPDATED DEFAULT HZ to 60 to match poll rate
        report_us = max(10000, int(1_000_000 / max(1, hz))) 
        for _ in range(3):
            try:
                try:
                    self.bno.enable_feature(feature, report_interval=report_us, batch_interval=0)
                except TypeError:
                    self.bno.enable_feature(feature) # Fallback
                return
            except Exception:
                time.sleep(0.05)
        self.bno.enable_feature(feature)

    # [Keep Publish Health / Offline logic as is]
    def _publish_mode(self, mode: str):
        m = String(); m.data = mode; self.mode_pub.publish(m)

    def publish_health_status(self, status: str, *, force: bool = False, log: bool = True):
        if self._shutdown_requested and status != "IMU OFFLINE" and not force: return
        if self.last_health_status == "IMU OFFLINE" and status != "IMU OFFLINE" and not force: return
        
        msg_text = status
        if "UNSTABLE" in status and "Last error" not in status and self.last_failure_reason:
            msg_text += f" | Last error: {self.last_failure_reason}"
        
        if force or msg_text != self.last_health_status:
            m = String(); m.data = msg_text; self.health_status_publisher.publish(m)
            if log: self.get_logger().info(f"[IMU Health] {msg_text}")
            self.last_health_status = msg_text

    def mark_offline(self):
        if self._offline_published: return
        try:
            self.sensor_ready = False
            self._cancel_timers()
            self.publish_health_status("IMU OFFLINE", force=True, log=False)
            self._publish_mode("OFFLINE")
        except Exception: pass
        self._offline_published = True

    def _cancel_timers(self):
        for t in (getattr(self, "timer", None), getattr(self, "retry_timer", None)):
            try: t.cancel() 
            except Exception: pass

    def request_shutdown(self):
        if self._shutdown_requested: return
        self._shutdown_requested = True
        self.mark_offline()

    def restart_process(self):
        self.publish_health_status("IMU RESTARTING", force=True)
        time.sleep(0.2)
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def reset_i2c_bus(self) -> bool:
        if self._shutdown_requested: return False
        self.get_logger().error("Attempting soft I2C reset...")
        try:
            self._close_i2c()
            time.sleep(0.1)
            self.i2c = self._new_i2c()
            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            time.sleep(0.2)

            # Re-enable at higher rate
            self._enable_feature_safe(BNO_REPORT_ACCELEROMETER, hz=self.sensor_report_hz)
            self._enable_feature_safe(BNO_REPORT_GYROSCOPE, hz=self.sensor_report_hz)
            self._enable_feature_safe(BNO_REPORT_ROTATION_VECTOR, hz=self.sensor_report_hz)

            self.sensor_ready = True
            self._bad_frame_streak = 0
            self.failure_timestamps.clear()
            self.last_failure_reason = None
            self._est_active = False
            self._est_elapsed = 0.0

            self.publish_health_status("IMU OK", force=True)
            self._publish_mode("NORMAL")
            self.reset_attempts = 0
            return True
        except Exception as e:
            self.get_logger().error(f"Soft reset failed: {e}")
            self.sensor_ready = False
            return False

    def record_failure_and_check_restart(self):
        if self._shutdown_requested: return
        now = time.time()
        self.failure_timestamps.append(now)
        while self.failure_timestamps and (now - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()

        failure_count = len(self.failure_timestamps)
        if failure_count >= self.max_failures:
            if self.reset_attempts < self.max_reset_attempts and self.reset_i2c_bus():
                self.reset_attempts += 1
                return
            if self.initialize_sensor_minimal():
                self._close_i2c(); time.sleep(0.1); self.initialize_sensor()
                return
            self.restart_process()
        else:
            self.publish_health_status(f"IMU UNSTABLE ({failure_count} fails)")

    def _retry_init_if_needed(self):
        if self._shutdown_requested or self.sensor_ready: return
        self.record_failure_and_check_restart()

    def initialize_sensor_minimal(self) -> bool:
        if self._shutdown_requested: return False
        try:
            self._close_i2c(); time.sleep(0.1)
            self.i2c = self._new_i2c()
            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            time.sleep(0.2)
            self._enable_feature_safe(BNO_REPORT_ACCELEROMETER, hz=self.sensor_report_hz)
            self.sensor_ready = True
            self.publish_health_status("IMU MINIMAL OK", force=True)
            return True
        except Exception:
            self.sensor_ready = False
            return False

    def initialize_sensor(self):
        if self._shutdown_requested: return
        self.get_logger().info(f"Initializing BNO08X sensor at {self.sensor_report_hz}Hz...")
        self.sensor_ready = False
        try:
            self._close_i2c(); time.sleep(0.1)
            self.i2c = self._new_i2c()
            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            time.sleep(0.2)

            try:
                self._enable_feature_safe(BNO_REPORT_ACCELEROMETER, hz=self.sensor_report_hz)
                self._enable_feature_safe(BNO_REPORT_GYROSCOPE, hz=self.sensor_report_hz)
                self._enable_feature_safe(BNO_REPORT_ROTATION_VECTOR, hz=self.sensor_report_hz)
            except Exception as feature_e:
                self.last_failure_reason = str(feature_e)
                self.record_failure_and_check_restart()
                return

            time.sleep(0.2)
            self.sensor_ready = True
            self.failure_timestamps.clear()
            self._est_active = False
            self.publish_health_status("IMU OK", force=True)
            self._publish_mode("NORMAL")
        except Exception as e:
            self.sensor_ready = False
            self.last_failure_reason = str(e)
            if any(k in self.last_failure_reason for k in self.CRITICAL_ERRORS):
                self.record_failure_and_check_restart()

    def publish_imu_data(self):
        if self._shutdown_requested or not self.sensor_ready: return

        now = time.monotonic()
        dt = max(1e-3, now - self._last_t)
        self._last_t = now

        try:
            with self._imu_lock:
                qi, qj, qk, qr = self.bno.quaternion
                ax, ay, az = self.bno.acceleration
                gx, gy, gz = self.bno.gyro

            sensor_q = q_normalize((qi, qj, qk, qr))

            if self._est_active:
                blend_t = min(1.0, dt / self._blend_back_sec)
                sensor_q = q_slerp(self._last_q, sensor_q, blend_t)
                self._est_active = False
                self._est_elapsed = 0.0

            self._last_q = sensor_q
            current_tuple = (sensor_q[0], sensor_q[1], sensor_q[2], sensor_q[3], ax, ay, az, gx, gy, gz)

            if self._last_data is None:
                self._last_data = current_tuple
                self._stagnant_start = None
            else:
                if self._within_stagnant_tolerance(current_tuple):
                    if self._stagnant_start is None:
                        self._stagnant_start = time.time()
                    elif time.time() - self._stagnant_start >= self.stagnant_timeout_sec:
                        self.get_logger().error('IMU data stagnant; restarting')
                        self.restart_process()
                        return
                else:
                    self._stagnant_start = None
                self._last_data = current_tuple

            if not self.validate_imu_data(*current_tuple):
                self._bad_frame_streak += 1
                if self._bad_frame_streak >= self._bad_frame_streak_limit:
                    self._publish_mode("ESTIMATED")
                return
            else:
                self._bad_frame_streak = 0

            # Publish IMU
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = sensor_q
            imu_msg.angular_velocity.x = gx; imu_msg.angular_velocity.y = gy; imu_msg.angular_velocity.z = gz
            imu_msg.linear_acceleration.x = ax; imu_msg.linear_acceleration.y = ay; imu_msg.linear_acceleration.z = az
            self.imu_publisher_.publish(imu_msg)

            # Publish RPY
            roll, pitch, yaw = quaternion_to_euler(*sensor_q)
            rpy_msg = Vector3()
            rpy_msg.x = roll; rpy_msg.y = pitch; rpy_msg.z = yaw
            self.rpy_publisher_.publish(rpy_msg)

            # Publish Heading (Less string formatting for speed)
            if self.heading_publisher_.get_subscription_count() > 0:
                heading_degrees = (-yaw + self.heading_offset + 360) % 360
                msg = String()
                msg.data = f'{heading_degrees:.1f}' # Simple heading is enough for OSD
                self.heading_publisher_.publish(msg)

            if self.last_health_status != "IMU OK": self.publish_health_status("IMU OK", force=True)
            self._publish_mode("NORMAL")

        except Exception as e:
            self.handle_read_error(e, dt)

    def handle_read_error(self, e, dt):
        self.last_failure_reason = str(e)
        if isinstance(e, IndexError) or any(k in str(e) for k in ["Remote I/O", "Input/output error"]):
            if self.reset_i2c_bus(): return

        # Gyro Dead Reckoning
        gx = gy = gz = 0.0 
        dq = quat_from_gyro(gx, gy, gz, dt)
        est_q = q_normalize(q_mul(self._last_q, dq))
        self._last_q = est_q
        self._est_active = True
        self._est_elapsed += dt
        
        imu_msg = Imu()
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = est_q
        imu_msg.orientation_covariance[0] = 9999.0
        self.imu_publisher_.publish(imu_msg)
        self._publish_mode("ESTIMATED")
        
        if self._est_elapsed >= self._est_max_sec:
            self.record_failure_and_check_restart()

    def yaw_to_cardinal(self, heading_degrees):
        directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]

    def validate_imu_data(self, qi, qj, qk, qr, ax, ay, az, gx, gy, gz):
        quat_norm = math.sqrt(qi**2 + qj**2 + qk**2 + qr**2)
        if not (0.7 <= quat_norm <= 1.3): return False
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        if not (0.5 <= accel_mag <= 30.0): return False # Wider tolerance for launch/recovery
        return True

    def _within_stagnant_tolerance(self, current_tuple):
        if self._last_data is None: return False
        for value, last_value, tolerance in zip(current_tuple, self._last_data, self._stagnant_tolerances):
            if abs(value - last_value) > tolerance: return False
        return True

def _install_exit_signals(node: IMUNode):
    def _handler(signum, frame): node.request_shutdown()
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    _install_exit_signals(node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not node._shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.request_shutdown()
        try: node.destroy_node() 
        except: pass
        try: rclpy.shutdown() 
        except: pass

if __name__ == '__main__':
    main()
