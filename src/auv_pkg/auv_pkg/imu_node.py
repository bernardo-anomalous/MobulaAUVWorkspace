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
from std_msgs.msg import String, Float32

# --- YOUR UTILS ---
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


class RobustThreadedIMU(Node):
    def __init__(self):
        super().__init__('imu_node')

        # --- Parameters ---
        self.declare_parameter('i2c_bus_id', 3)   
        self.declare_parameter('i2c_address', 0x4B)
        self.bus_id = int(self.get_parameter('i2c_bus_id').value)
        self.addr = int(self.get_parameter('i2c_address').value)
        
        self.declare_parameter('publish_rate_hz', 60.0)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        
        # Default Heading Offset (Calibrated)
        self.declare_parameter('mounting_offset_deg', 254.5) 
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
        self.reset_count = 0
        
        # Critical Errors that trigger a BUS RESET
        self.CRITICAL_ERRORS = [
            "No device", "Remote I/O", "Input/output error",
            "Unprocessable Batch bytes", "Errno 6", "255"
        ]
        
        self.last_hw_q = (0.0, 0.0, 0.0, 1.0)
        self.last_hw_gyro = (0.0, 0.0, 0.0)
        self.last_hw_accel = (0.0, 0.0, 0.0)
        self.last_hw_time = time.monotonic()
        self.new_hw_data_available = False

        self.i2c = None
        self.bno = None

        # --- Threads ---
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_cycle)
        
        self.get_logger().info(f"Robust IMU Node v2.0 Started. Offset: {self.mounting_offset_deg}")

    # --- TOPIC CALLBACKS ---
    def cb_nudge_heading(self, msg):
        with self.lock:
            self.mounting_offset_deg = (self.mounting_offset_deg + msg.data) % 360.0
        self.get_logger().info(f"Offset Nudged to: {self.mounting_offset_deg:.1f}")

    def cb_set_heading(self, msg):
        with self.lock:
            self.mounting_offset_deg = msg.data % 360.0
        self.get_logger().info(f"Offset Set to: {self.mounting_offset_deg:.1f}")

    # --- HARDWARE RECOVERY LOGIC (From your old Robust Node) ---
    def _rebuild_hardware(self):
        """Destroys and recreates the I2C connection."""
        self.get_logger().warn(f"Performing I2C Bus Reset #{self.reset_count + 1}...")
        
        # 1. Teardown
        try:
            if self.i2c:
                if hasattr(self.i2c, 'deinit'): self.i2c.deinit()
                elif hasattr(self.i2c, 'close'): self.i2c.close()
        except: pass
        
        self.i2c = None
        self.bno = None
        time.sleep(0.5) # Physics wait

        # 2. Rebuild
        try:
            if _HAS_EXTENDED_I2C:
                self.i2c = ExtendedI2C(self.bus_id)
            else:
                self.i2c = busio.I2C(board.SCL, board.SDA)

            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            
            # Enable features (No Args!)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Rebuild Failed: {e}")
            return False

    def read_loop(self):
        """
        The Miner Thread. It handles the dirty work of I2C.
        """
        # Initial Boot
        while self.running and not self.sensor_ready:
            if self._rebuild_hardware():
                self.sensor_ready = True
                break
            time.sleep(1.0)

        while self.running and rclpy.ok():
            try:
                # --- READ ATTEMPT ---
                # This blocks. If I2C dies, this throws an Exception.
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

                # Yield CPU
                time.sleep(0.01)

            except Exception as e:
                # --- ERROR HANDLING ---
                error_str = str(e)
                
                # Check for "Bus Killers"
                is_critical = any(c in error_str for c in self.CRITICAL_ERRORS) or isinstance(e, OSError)
                
                if is_critical:
                    self.get_logger().error(f"CRITICAL I2C FAILURE: {error_str}")
                    self.health_pub.publish(String(data=f"CRITICAL: {error_str}"))
                    
                    with self.lock: self.sensor_ready = False
                    
                    # RETRY LOOP
                    while self.running and rclpy.ok():
                        self.reset_count += 1
                        if self._rebuild_hardware():
                            self.get_logger().info(">>> SENSOR RECOVERED <<<")
                            with self.lock: self.sensor_ready = True
                            self.health_pub.publish(String(data="IMU OK"))
                            break # Back to main loop
                        time.sleep(2.0)
                else:
                    # Minor glitch (CRC check failed, etc)
                    self.get_logger().warn(f"Minor Glitch: {error_str}")
                    time.sleep(0.05)

    def publish_cycle(self):
        """
        The Broadcast Thread. Runs at 60Hz. 
        Uses prediction if the Read Thread is busy resetting the bus.
        """
        now = time.monotonic()
        
        with self.lock:
            hw_q = self.last_hw_q
            hw_gyro = self.last_hw_gyro
            hw_accel = self.last_hw_accel
            last_time = self.last_hw_time
            is_ready = self.sensor_ready
            is_fresh = self.new_hw_data_available
            current_offset = self.mounting_offset_deg
            if is_fresh: self.new_hw_data_available = False

        dt = now - last_time
        
        # If dead for > 2s, we are in trouble
        if dt > 2.0:
            status = "RECOVERING" if not is_ready else "STALE"
            self.health_pub.publish(String(data=status))
            # Stop spinning the virtual model if we have no data
            hw_gyro = (0.0, 0.0, 0.0)

        # --- PREDICTION (Gyro Integration) ---
        dq = quat_from_gyro(hw_gyro[0], hw_gyro[1], hw_gyro[2], dt)
        predicted_q = q_normalize(q_mul(hw_q, dq))
        
        self.publish_ros_msgs(predicted_q, hw_gyro, hw_accel, current_offset)

    def yaw_to_cardinal(self, heading_degrees):
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]

    def publish_ros_msgs(self, q, gyro, accel, offset):
        # 1. IMU
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        self.imu_publisher_.publish(msg)

        # 2. Euler (Matches your Utils)
        roll, pitch, raw_yaw = quaternion_to_euler(*q)
        
        vec = Vector3()
        vec.x = roll
        vec.y = pitch
        vec.z = raw_yaw
        self.rpy_publisher_.publish(vec)
        
        # 3. Heading (Matches your GUI)
        heading_degrees = (-raw_yaw + offset + 360) % 360
        cardinal = self.yaw_to_cardinal(heading_degrees)
        
        heading_str = f'Heading: {cardinal}, {heading_degrees:.2f} degrees'
        self.heading_publisher_.publish(String(data=heading_str))

def main(args=None):
    rclpy.init(args=args)
    node = RobustThreadedIMU()
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
