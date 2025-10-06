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

# --- your local util ----------------------------------------------------------
from .utils import quaternion_to_euler


# ------------------------ Quaternion utilities (no numpy) ---------------------
def q_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
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

def q_dot(q1, q2):
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return x1*x2 + y1*y2 + z1*z2 + w1*w2

def q_slerp(q0, q1, t):
    # t in [0,1]
    q0 = q_normalize(q0)
    q1 = q_normalize(q1)
    dot = q_dot(q0, q1)
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot
    if dot > 0.9995:
        # Linear fallback
        x = q0[0] + t*(q1[0]-q0[0])
        y = q0[1] + t*(q1[1]-q0[1])
        z = q0[2] + t*(q1[2]-q0[2])
        w = q0[3] + t*(q1[3]-q0[3])
        return q_normalize((x,y,z,w))
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    sin0 = math.sin(theta0)
    s0 = math.sin((1.0 - t) * theta0) / sin0
    s1 = math.sin(t * theta0) / sin0
    return (
        s0*q0[0] + s1*q1[0],
        s0*q0[1] + s1*q1[1],
        s0*q0[2] + s1*q1[2],
        s0*q0[3] + s1*q1[3],
    )

def quat_from_gyro(gx, gy, gz, dt):
    # Map body rates [rad/s] over dt to small-angle quaternion increment
    # Returns (x,y,z,w)
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

        # ---- env banner ------------------------------------------------------
        self.get_logger().info(f"[IMU] Python executable: {sys.executable}")
        self.get_logger().info(f"[IMU] ExtendedI2C available: {_HAS_EXTENDED_I2C}")

        # ---- parameters ------------------------------------------------------
        # where the IMU lives now
        self.declare_parameter('i2c_bus_id', 0)   # you moved IMU to bus 0
        self.declare_parameter('i2c_address', 0x4B)
        self.bus_id = int(self.get_parameter('i2c_bus_id').value)
        self.addr = int(self.get_parameter('i2c_address').value)
        self.get_logger().info(f"[IMU] Target I2C bus: {self.bus_id}, address: 0x{self.addr:02X}")

        self.declare_parameter('stagnant_timeout_sec', 3.0)
        self.stagnant_timeout_sec = float(self.get_parameter('stagnant_timeout_sec').value)

        self.declare_parameter('poll_period_sec', 0.1)  # times per second to poll data
        poll_period = float(self.get_parameter('poll_period_sec').value)
        if poll_period <= 0:
            self.get_logger().warn(f"Invalid poll_period_sec={poll_period}; falling back to 0.1s")
            poll_period = 0.1
        self.get_logger().info(f"IMU polling period set to {poll_period:.3f}s (~{1.0 / poll_period:.2f} Hz)")

        # tolerances for "stagnant" detection
        self.declare_parameter('stagnant_orientation_tolerance', 1e-4)
        self.declare_parameter('stagnant_accel_tolerance', 5e-3)
        self.declare_parameter('stagnant_gyro_tolerance', 1e-3)
        self._stagnant_orientation_tolerance = float(self.get_parameter('stagnant_orientation_tolerance').value)
        self._stagnant_accel_tolerance = float(self.get_parameter('stagnant_accel_tolerance').value)
        self._stagnant_gyro_tolerance = float(self.get_parameter('stagnant_gyro_tolerance').value)
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
        self._est_max_sec = 1.0     # max duration to bridge with gyro integration
        self._blend_back_sec = 0.2  # smooth return to sensor data
        self._last_q = (0.0, 0.0, 0.0, 1.0)
        self._last_t = time.monotonic()

        # ---- config/state ----------------------------------------------------
        self.heading_offset = -30.0
        self.sensor_ready = False

        self._offline_published = False
        self._shutdown_requested = False

        # Health publisher QoS: Transient Local so late subscribers get the last state
        health_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # publishers
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'imu/heading', 10)
        self.health_status_publisher = self.create_publisher(String, 'imu/health_status', health_qos)
        self.mode_pub = self.create_publisher(String, 'imu/mode', 1)  # NORMAL | ESTIMATED | OFFLINE

        self.last_health_status = None
        self.last_failure_reason = None

        # timers
        self.timer = self.create_timer(poll_period, self.publish_imu_data)
        self.retry_timer = self.create_timer(5.0, self._retry_init_if_needed)

        # state
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
            "No device",
            "Remote I/O",
            "Input/output error",
            "Unprocessable Batch bytes",
            "Was not able to enable feature",
        ]

        self.reset_attempts = 0
        self.max_reset_attempts = 3

        # bring the sensor up
        self.initialize_sensor()

    # -------------------------------------------------------------------------
    # backend logging helpers
    # -------------------------------------------------------------------------
    def _log_i2c_backend(self):
        try:
            if _HAS_EXTENDED_I2C and isinstance(self.i2c, ExtendedI2C):
                import adafruit_extended_bus as _mod
                self.get_logger().info(f"[IMU] I2C backend: ExtendedI2C (from {_mod.__file__})")
                return
        except Exception:
            pass
        try:
            import adafruit_blinka.microcontroller.generic_linux.i2c as _mod
            self.get_logger().info(f"[IMU] I2C backend: Blinka busio (from {_mod.__file__})")
        except Exception:
            self.get_logger().info("[IMU] I2C backend: unknown")

    def _log_bno_backend(self):
        try:
            import adafruit_bno08x.i2c as _bno_mod
            self.get_logger().info(f"[IMU] BNO08X backend: BNO08X_I2C (from {_bno_mod.__file__})")
        except Exception:
            self.get_logger().info("[IMU] BNO08X backend: unknown")

    # -------------------------------------------------------------------------
    # I2C helpers
    # -------------------------------------------------------------------------
    def _new_i2c(self):
        # Respect selected bus
        if _HAS_EXTENDED_I2C:
            return ExtendedI2C(self.bus_id)  # e.g. /dev/i2c-0 or /dev/i2c-1
        if busio is None or board is None:
            raise RuntimeError("No I2C backend available (need adafruit_extended_bus or busio+board).")
        # Blinka doesn't expose "bus index" easily; default to board.SCL/SDA (usually bus 1)
        return busio.I2C(board.SCL, board.SDA)

    def _close_i2c(self):
        if self.i2c is None:
            return
        try:
            if hasattr(self.i2c, "deinit"):
                self.i2c.deinit()
            elif hasattr(self.i2c, "close"):
                self.i2c.close()
        except Exception as e:
            self.get_logger().error(f"Error closing I2C: {e}")
        finally:
            self.i2c = None

    # -------------------------------------------------------------------------
    # feature enable
    # -------------------------------------------------------------------------
    def _enable_feature_safe(self, feature, hz=25):
        report_us = max(10000, int(1_000_000 / max(1, hz)))  # >=10ms
        for _ in range(3):
            try:
                try:
                    self.bno.enable_feature(feature, report_interval=report_us, batch_interval=0)
                except TypeError:
                    try:
                        self.bno.enable_feature(feature, report_interval=report_us)
                    except TypeError:
                        self.bno.enable_feature(feature)
                self.get_logger().info(f"[IMU] enable_feature ok (feature=0x{feature:x})")
                return
            except Exception:
                time.sleep(0.05)
        # last attempt, raise if fails
        self.bno.enable_feature(feature)

    # -------------------------------------------------------------------------
    # health + mode publishing
    # -------------------------------------------------------------------------
    def _publish_mode(self, mode: str):
        m = String(); m.data = mode
        self.mode_pub.publish(m)

    def publish_health_status(self, status: str, *, force: bool = False, log: bool = True):
        """
        Publish a health status.
        - force=True: publish even if unchanged.
        - log=False: suppress rosout logging (useful during shutdown).
        """
        if self._shutdown_requested:
            # During teardown we only want OFFLINE; block others unless forced.
            if status != "IMU OFFLINE" and not force:
                return

        # Never overwrite OFFLINE unless explicitly forced
        if self.last_health_status == "IMU OFFLINE" and status != "IMU OFFLINE" and not force:
            return

        msg_text = status
        if "UNSTABLE" in status and "Last error" not in status and self.last_failure_reason:
            msg_text += f" | Last error: {self.last_failure_reason}"
        elif "RESTARTING" in status and "Reason:" not in status and self.last_failure_reason:
            msg_text += f" | Reason: {self.last_failure_reason}"

        if force or msg_text != self.last_health_status:
            m = String()
            m.data = msg_text
            self.health_status_publisher.publish(m)
            if log:
                self.get_logger().info(f"[IMU Health] {msg_text}")
            self.last_health_status = msg_text

    def mark_offline(self):
        """Publish IMU OFFLINE once and cancel timers."""
        if self._offline_published:
            return
        try:
            self.sensor_ready = False
            self._cancel_timers()
            self.publish_health_status("IMU OFFLINE", force=True, log=False)
            self._publish_mode("OFFLINE")
            time.sleep(0.05)  # let DDS ship it
        except Exception:
            pass
        self._offline_published = True

    def _cancel_timers(self):
        for t in (getattr(self, "timer", None), getattr(self, "retry_timer", None)):
            try:
                if t:
                    t.cancel()
            except Exception:
                pass

    def request_shutdown(self):
        """Idempotent: set flag, publish OFFLINE, stop timers."""
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.mark_offline()

    # -------------------------------------------------------------------------
    # process control
    # -------------------------------------------------------------------------
    def restart_process(self):
        self.publish_health_status("IMU RESTARTING", force=True)
        self.get_logger().error("IMU node restarting itself now...")
        time.sleep(0.2)
        python = sys.executable
        os.execv(python, [python] + sys.argv)

    def reset_i2c_bus(self) -> bool:
        if self._shutdown_requested:
            return False
        self.get_logger().error("Attempting soft I2C reset...")
        try:
            self._close_i2c()
            time.sleep(0.1)
            self.i2c = self._new_i2c()
            self._log_i2c_backend()
            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            self._log_bno_backend()
            time.sleep(0.2)

            self._enable_feature_safe(BNO_REPORT_ACCELEROMETER, hz=25)
            self._enable_feature_safe(BNO_REPORT_GYROSCOPE, hz=25)
            self._enable_feature_safe(BNO_REPORT_ROTATION_VECTOR, hz=25)

            self.sensor_ready = True
            self._bad_frame_streak = 0
            self.failure_timestamps.clear()
            self.last_failure_reason = None
            self._est_active = False
            self._est_elapsed = 0.0

            self.get_logger().error("Soft reset successful.")
            self.publish_health_status("IMU OK", force=True)   # ensure OK after successful reset
            self._publish_mode("NORMAL")
            self.reset_attempts = 0
            return True
        except Exception as e:
            self.get_logger().error(f"Soft reset failed: {e}")
            self.sensor_ready = False
            return False

    def record_failure_and_check_restart(self):
        if self._shutdown_requested:
            return
        now = time.time()
        self.failure_timestamps.append(now)
        while self.failure_timestamps and (now - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()

        failure_count = len(self.failure_timestamps)
        if failure_count >= self.max_failures:
            self.get_logger().error(
                f"Exceeded {self.max_failures} failures within {self.failure_window_seconds} seconds."
            )
            if self.reset_attempts < self.max_reset_attempts and self.reset_i2c_bus():
                self.reset_attempts += 1
                self.get_logger().info(f"Soft reset attempt {self.reset_attempts}/{self.max_reset_attempts} successful")
                self.publish_health_status("IMU SOFT RESET", force=True)
                self._publish_mode("NORMAL")
                return
            self.get_logger().fatal(f"Restart after {self.reset_attempts} soft reset attempts")
            if self.initialize_sensor_minimal():
                self._close_i2c()
                time.sleep(0.1)
                self.initialize_sensor()
                return
            self.publish_health_status("IMU RESTARTING", force=True)
            time.sleep(2)
            self.restart_process()
        else:
            self.publish_health_status(
                f"IMU UNSTABLE ({failure_count} failures in last {self.failure_window_seconds} sec)"
            )

    def _retry_init_if_needed(self):
        if self._shutdown_requested or self.sensor_ready:
            return
        self.record_failure_and_check_restart()

    # -------------------------------------------------------------------------
    # init paths
    # -------------------------------------------------------------------------
    def initialize_sensor_minimal(self) -> bool:
        if self._shutdown_requested:
            return False
        self.get_logger().info("Attempting minimal BNO08X initialization...")
        try:
            self._close_i2c()
            time.sleep(0.1)
            self.i2c = self._new_i2c()
            self._log_i2c_backend()
            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            self._log_bno_backend()
            time.sleep(0.2)

            self._enable_feature_safe(BNO_REPORT_ACCELEROMETER, hz=25)

            self.sensor_ready = True
            self.failure_timestamps.clear()
            self._bad_frame_streak = 0
            self.last_failure_reason = None
            self._est_active = False
            self._est_elapsed = 0.0
            self.publish_health_status("IMU MINIMAL OK", force=True)
            self._publish_mode("NORMAL")
            self.get_logger().info("Minimal initialization successful.")
            self.reset_attempts = 0
            return True
        except Exception as e:
            self.sensor_ready = False
            self.last_failure_reason = str(e)
            self.get_logger().error(f"Minimal init failed: {e}")
            return False

    def initialize_sensor(self):
        if self._shutdown_requested:
            return
        self.get_logger().info("Initializing BNO08X sensor...")
        self.sensor_ready = False
        try:
            self._close_i2c()
            time.sleep(0.1)
            self.i2c = self._new_i2c()
            self._log_i2c_backend()
            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            self._log_bno_backend()
            time.sleep(0.2)

            try:
                self._enable_feature_safe(BNO_REPORT_ACCELEROMETER, hz=25)
                self._enable_feature_safe(BNO_REPORT_GYROSCOPE, hz=25)
                self._enable_feature_safe(BNO_REPORT_ROTATION_VECTOR, hz=25)
            except Exception as feature_e:
                self.get_logger().error(f"Critical error enabling feature: {feature_e}")
                self.last_failure_reason = str(feature_e)
                self.get_logger().error("Invoking failure handler: soft reset then restart if needed")
                self.sensor_ready = False
                self.record_failure_and_check_restart()
                return

            time.sleep(0.2)

            self.sensor_ready = True
            self.failure_timestamps.clear()
            self._bad_frame_streak = 0
            self.last_failure_reason = None
            self._est_active = False
            self._est_elapsed = 0.0
            self.publish_health_status("IMU OK", force=True)
            self._publish_mode("NORMAL")
            self.get_logger().info("Sensor initialized and ready.")
            self.reset_attempts = 0
        except Exception as e:
            self.sensor_ready = False
            self.last_failure_reason = str(e)
            self.get_logger().error(f"Error initializing sensor: {e}")
            if any(k in self.last_failure_reason for k in self.CRITICAL_ERRORS):
                self.record_failure_and_check_restart()

    # -------------------------------------------------------------------------
    # main loop
    # -------------------------------------------------------------------------
    def publish_imu_data(self):
        if self._shutdown_requested or not self.sensor_ready:
            return

        now = time.monotonic()
        dt = max(1e-3, now - self._last_t)
        self._last_t = now

        try:
            with self._imu_lock:
                qi, qj, qk, qr = self.bno.quaternion
                ax, ay, az = self.bno.acceleration
                gx, gy, gz = self.bno.gyro

            sensor_q = q_normalize((qi, qj, qk, qr))

            # If we were estimating, blend back smoothly for one step
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
                        self.get_logger().error('IMU data unchanged beyond timeout; restarting')
                        self.publish_health_status('IMU STAGNANT', force=True)
                        self.restart_process()
                        return
                else:
                    self._stagnant_start = None
                self._last_data = current_tuple

            # validation with small streak to avoid flapping
            if not self.validate_imu_data(*current_tuple):
                self._bad_frame_streak += 1
                if self._bad_frame_streak >= self._bad_frame_streak_limit:
                    self.publish_health_status("IMU HICCUP | Invalid sensor data detected", force=True)
                    self._publish_mode("ESTIMATED")  # signal degraded
                return
            else:
                self._bad_frame_streak = 0

            # Publish IMU (NORMAL)
            imu_msg = Imu()
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = sensor_q
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.orientation_covariance[0] = -1
            imu_msg.angular_velocity_covariance[0] = -1
            imu_msg.linear_acceleration_covariance[0] = -1
            self.imu_publisher_.publish(imu_msg)

            roll, pitch, yaw = quaternion_to_euler(*sensor_q)
            rpy_msg = Vector3()
            rpy_msg.x = roll
            rpy_msg.y = pitch
            rpy_msg.z = yaw
            self.rpy_publisher_.publish(rpy_msg)

            heading_degrees = (-yaw + self.heading_offset + 360) % 360
            heading_msg = String()
            heading_msg.data = f'Heading: {self.yaw_to_cardinal(heading_degrees)}, {heading_degrees:.2f} degrees'
            self.heading_publisher_.publish(heading_msg)

            if self.last_health_status != "IMU OK":
                self.publish_health_status("IMU OK", force=True)
            self._publish_mode("NORMAL")

        except Exception as e:
            self.last_failure_reason = str(e)
            self.get_logger().error(f"Sensor read error: {e}")

            # Fast-path resyncs for framing/short-read style errors
            if (isinstance(e, IndexError)
                or "Unprocessable Batch bytes" in self.last_failure_reason
                or "Input/output error" in self.last_failure_reason
                or "Remote I/O" in self.last_failure_reason
                or self.last_failure_reason.strip() in {"0","120","123","125","129","133","254","255"}):
                if self.reset_i2c_bus():
                    return  # reset already published IMU OK

            # If reset didn't happen, bridge with gyro for a short window
            gx = gy = gz = 0.0  # default if we never got rates
            try:
                # If the exception happened *after* reading gyro, reuse last available gx/gy/gz
                # (They may still be in local scope if the error occurred later.)
                _ = (gx, gy, gz)  # no-op, placeholder for clarity
            except Exception:
                pass

            dq = quat_from_gyro(gx, gy, gz, dt)
            est_q = q_normalize(q_mul(self._last_q, dq))
            self._last_q = est_q
            self._est_active = True
            self._est_elapsed += dt

            # publish estimated pose (degraded)
            imu_msg = Imu()
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = est_q
            # Keep angular velocity from rates if available (we used zeros above)
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            # Conservative accel during estimate
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0
            # Mark covariance to indicate "estimated"
            imu_msg.orientation_covariance[0] = 9999.0
            self.imu_publisher_.publish(imu_msg)
            self._publish_mode("ESTIMATED")
            # Keep health topic informative
            self.publish_health_status(f"IMU HICCUP | Last error: {self.last_failure_reason}", force=True)

            # If we've been estimating too long, let the failure machinery run
            if self._est_elapsed >= self._est_max_sec:
                if any(keyword in self.last_failure_reason for keyword in self.CRITICAL_ERRORS):
                    self.record_failure_and_check_restart()

    # -------------------------------------------------------------------------
    # helpers
    # -------------------------------------------------------------------------
    def yaw_to_cardinal(self, heading_degrees):
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]

    def validate_imu_data(self, qi, qj, qk, qr, ax, ay, az, gx, gy, gz):
        # quaternion magnitude ~ 1
        quat_norm = math.sqrt(qi**2 + qj**2 + qk**2 + qr**2)
        if not (0.7 <= quat_norm <= 1.3):
            return False
        # accel approx gravity magnitude (wide tolerance; vehicle can thrust)
        accel_mag = math.sqrt(ax**2 + ay**2 + az**2)
        if not (3.0 <= accel_mag <= 25.0):
            return False
        # gyro sanity
        gyro_threshold = math.radians(2000)  # ~35 rad/s
        if any(abs(g) > gyro_threshold for g in (gx, gy, gz)):
            return False
        return True

    def _within_stagnant_tolerance(self, current_tuple):
        if self._last_data is None:
            return False
        for value, last_value, tolerance in zip(current_tuple, self._last_data, self._stagnant_tolerances):
            if abs(value - last_value) > tolerance:
                return False
        return True


# ---- signal wiring ------------------------------------------------------------
def _install_exit_signals(node: IMUNode):
    """Ensure OFFLINE fires and our manual loop exits on SIGINT/SIGTERM."""
    def _handler(signum, frame):
        node.request_shutdown()   # publish OFFLINE + stop timers (do not shutdown rclpy yet)
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    _install_exit_signals(node)

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        # Manual spin: exits when Ctrl-C sets node._shutdown_requested = True
        while not node._shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        # Guarantee OFFLINE once, then shut middleware down.
        node.request_shutdown()
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
