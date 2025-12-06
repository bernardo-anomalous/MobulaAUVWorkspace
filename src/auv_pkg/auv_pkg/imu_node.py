#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import time
import threading
import signal
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float32  # Added Float32 for the topics

# --- IMPORT YOUR UTILS ---
from .utils import quaternion_to_euler

# --- Hardware Imports ---
try:
    from adafruit_extended_bus import ExtendedI2C
    _HAS_EXTENDED_I2C = True
except Exception:
    _HAS_EXTENDED_I2C = False

try:
    import board
    import busio
except Exception:
    pass

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)

# --- Math Utilities ---
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

def quat_from_gyro(gx, gy, gz, dt):
    omega = math.sqrt(gx*gx + gy*gy + gz*gz)
    if omega < 1e-12 or dt <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    theta = omega * dt
    half = 0.5 * theta
    s = math.sin(half) / omega
    return (gx*s, gy*s, gz*s, math.cos(half))


class RobustImmortalIMU(Node):
    def __init__(self):
        super().__init__('imu_node')

        # --- Parameters ---
        self.declare_parameter('i2c_bus_id', 3)   
        self.declare_parameter('i2c_address', 0x4B)
        self.bus_id = int(self.get_parameter('i2c_bus_id').value)
        self.addr = int(self.get_parameter('i2c_address').value)
        
        self.declare_parameter('publish_rate_hz', 60.0)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        
        # Default starting offset (Calibrated value)
        self.declare_parameter('mounting_offset_deg', 224.5) 
        self.mounting_offset_deg = float(self.get_parameter('mounting_offset_deg').value)
        
        # --- Publishers ---
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'imu/heading', 10)
        
        health_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_pub = self.create_publisher(String, 'imu/health_status', health_qos)

        # --- Subscribers (Dynamic Adjustment) ---
        self.create_subscription(Float32, 'imu/nudge_heading', self.cb_nudge_heading, 10)
        self.create_subscription(Float32, 'imu/set_heading_offset', self.cb_set_heading, 10)

        # --- State ---
        self.lock = threading.Lock()
        self.running = True
        self.sensor_ready = False
        
        # Data
        self.last_hw_q = (0.0, 0.0, 0.0, 1.0)
        self.last_hw_gyro = (0.0, 0.0, 0.0)
        self.last_hw_accel = (0.0, 0.0, 0.0)
        self.last_hw_time = time.monotonic()
        self.new_hw_data_available = False

        # Error Tracking
        self.reset_count = 0
        self.CRITICAL_ERRORS = [
            "No device", "Remote I/O", "Input/output error",
            "Unprocessable Batch bytes", "Errno 6", "255"
        ]

        self.i2c = None
        self.bno = None

        # --- Threads ---
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_cycle)
        
        self.get_logger().info(f"Robust IMU Node Started. Offset: {self.mounting_offset_deg}")

    # --- DYNAMIC OFFSET CALLBACKS ---
    def cb_nudge_heading(self, msg):
        """Adds/Subtracts from current offset. Example: msg.data = 5.0 adds 5 degrees"""
        with self.lock:
            old_offset = self.mounting_offset_deg
            self.mounting_offset_deg = (self.mounting_offset_deg + msg.data) % 360.0
            
        self.get_logger().info(
            f"Heading Nudged: {msg.data:+.1f} deg. (Old: {old_offset:.1f} -> New: {self.mounting_offset_deg:.1f})"
        )

    def cb_set_heading(self, msg):
        """Sets the offset directly. Example: msg.data = 90.0 sets offset to 90"""
        with self.lock:
            self.mounting_offset_deg = msg.data % 360.0
        self.get_logger().info(f"Heading Offset Set To: {self.mounting_offset_deg:.1f}")

    # --- HARDWARE RECOVERY ---
    def _teardown_i2c(self):
        try:
            if self.i2c:
                try: self.i2c.deinit()
                except: pass
                try: self.i2c.close()
                except: pass
        except: pass
        self.i2c = None
        self.bno = None

    def _init_i2c_hardware(self):
        try:
            if _HAS_EXTENDED_I2C:
                self.i2c = ExtendedI2C(self.bus_id)
            else:
                self.i2c = busio.I2C(board.SCL, board.SDA)

            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            
            # Enable Features (No Args to prevent TypeError)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Hardware Init Failed: {e}")
            return False

    def perform_hard_reset(self):
        with self.lock:
            self.sensor_ready = False
        
        self.reset_count += 1
        self.health_pub.publish(String(data=f"RESETTING ({self.reset_count})"))
        self.get_logger().warn(f"--- PERFORMING HARD RESET #{self.reset_count} ---")
        
        self._teardown_i2c()
        time.sleep(0.5)

        if self._init_i2c_hardware():
            with self.lock:
                self.sensor_ready = True
            self.get_logger().info(">>> SENSOR RECOVERED <<<")
            self.health_pub.publish(String(data="IMU OK"))
            return True
        else:
            self.get_logger().error("Reset Attempt Failed.")
            return False

    # --- LOOPS ---
    def read_loop(self):
        while self.running and not self.sensor_ready:
            if self.perform_hard_reset(): break
            time.sleep(1.0)

        while self.running and rclpy.ok():
            try:
                # BLOCKING READ
                qi, qj, qk, qr = self.bno.quaternion
                ax, ay, az = self.bno.acceleration
                gx, gy, gz = self.bno.gyro

                now = time.monotonic()
                
                with self.lock:
                    self.last_hw_q = q_normalize((qi, qj, qk, qr))
                    self.last_hw_accel = (ax, ay, az)
                    self.last_hw_gyro = (gx, gy, gz)
                    self.last_hw_time = now
                    self.new_hw_data_available = True
                    self.sensor_ready = True

                time.sleep(0.01)

            except Exception as e:
                error_str = str(e)
                # Filter noise vs Critical
                is_critical = any(c in error_str for c in self.CRITICAL_ERRORS) or isinstance(e, OSError)
                
                if is_critical:
                    self.get_logger().error(f"CRITICAL BUS FAILURE: {error_str}")
                    self.health_pub.publish(String(data=f"CRITICAL: {error_str}"))
                    while self.running and rclpy.ok():
                        if self.perform_hard_reset(): break
                        time.sleep(2.0)
                else:
                    self.get_logger().warn(f"Minor Glitch: {error_str}")
                    time.sleep(0.05) 

    def publish_cycle(self):
        now = time.monotonic()
        
        with self.lock:
            hw_q = self.last_hw_q
            hw_gyro = self.last_hw_gyro
            hw_accel = self.last_hw_accel
            last_time = self.last_hw_time
            is_ready = self.sensor_ready
            is_fresh = self.new_hw_data_available
            current_offset = self.mounting_offset_deg # Capture safely
            if is_fresh: self.new_hw_data_available = False

        dt = now - last_time
        
        if dt > 2.0:
            status = f"STALE ({dt:.1f}s)" if is_ready else "RECOVERING"
            self.health_pub.publish(String(data=status))
            hw_gyro = (0.0, 0.0, 0.0)

        dq = quat_from_gyro(hw_gyro[0], hw_gyro[1], hw_gyro[2], dt)
        predicted_q = q_normalize(q_mul(hw_q, dq))
        
        self.publish_ros_msgs(predicted_q, hw_gyro, hw_accel, current_offset)

    def yaw_to_cardinal(self, heading_degrees):
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]

    def publish_ros_msgs(self, q, gyro, accel, offset):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        self.imu_publisher_.publish(msg)

        # Euler (Using your UTILS)
        roll, pitch, raw_yaw = quaternion_to_euler(*q)
        
        vec = Vector3()
        vec.x = roll
        vec.y = pitch
        vec.z = raw_yaw
        self.rpy_publisher_.publish(vec)
        
        # Heading (Calibrated with Dynamic Offset)
        heading_degrees = (-raw_yaw + offset + 360) % 360
        cardinal = self.yaw_to_cardinal(heading_degrees)
        
        heading_str = f'Heading: {cardinal}, {heading_degrees:.2f} degrees'
        self.heading_publisher_.publish(String(data=heading_str))

def main(args=None):
    rclpy.init(args=args)
    node = RobustImmortalIMU()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
