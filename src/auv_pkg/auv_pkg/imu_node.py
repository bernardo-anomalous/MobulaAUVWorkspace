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
from std_msgs.msg import String

# --- IMPORT YOUR UTILS (Crucial for correct heading) ---
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

# --- Math Utilities (Local for Prediction only) ---
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


class ImmortalIMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # --- Parameters ---
        self.declare_parameter('i2c_bus_id', 3)   
        self.declare_parameter('i2c_address', 0x4B)
        self.bus_id = int(self.get_parameter('i2c_bus_id').value)
        self.addr = int(self.get_parameter('i2c_address').value)
        
        self.declare_parameter('publish_rate_hz', 60.0)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        
        # This matches your old logic (-30 offset)
        self.declare_parameter('mounting_offset_deg', 224.5) # Use the calibrated value we found
        self.mounting_offset_deg = float(self.get_parameter('mounting_offset_deg').value)
        
        # --- Publishers ---
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'imu/heading', 10)
        
        # QoS for health (latched)
        health_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_pub = self.create_publisher(String, 'imu/health_status', health_qos)

        # --- State ---
        self.lock = threading.Lock()
        self.running = True
        self.sensor_ready = False
        
        # The "Truth"
        self.last_hw_q = (0.0, 0.0, 0.0, 1.0)
        self.last_hw_gyro = (0.0, 0.0, 0.0)
        self.last_hw_accel = (0.0, 0.0, 0.0)
        self.last_hw_time = time.monotonic()
        self.new_hw_data_available = False

        # Hardware Objects
        self.i2c = None
        self.bno = None

        # --- Threads ---
        # 1. Background Reader (The "Miner" - Digs for data, Handles collapses)
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        # 2. Fast Publisher (The "Broadcast" - Sends smooth data to PIDs)
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_cycle)
        
        self.get_logger().info(f"Immortal IMU Node started. Pub: {self.publish_rate}Hz")

    def hard_reset_i2c(self):
        """
        The Nuclear Option: Destroys the I2C object and recreates it.
        This fixes [Errno 6] No such device.
        """
        with self.lock:
            self.sensor_ready = False
        
        self.get_logger().warn("Performing Hard I2C Reset...")
        
        # 1. Teardown
        try:
            if self.i2c:
                try: self.i2c.deinit()
                except: pass
                try: self.i2c.close()
                except: pass
        except: pass
        
        self.i2c = None
        self.bno = None
        time.sleep(0.5) # Let the bus breathe

        # 2. Rebuild
        try:
            if _HAS_EXTENDED_I2C:
                self.i2c = ExtendedI2C(self.bus_id)
            else:
                self.i2c = busio.I2C(board.SCL, board.SDA)

            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            
            # Enable Features (No args, fixes TypeError)
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            
            with self.lock:
                self.sensor_ready = True
            
            self.get_logger().info("I2C Bus Recovered & Sensor Re-enabled.")
            self.health_pub.publish(String(data="IMU OK"))
            return True

        except Exception as e:
            self.get_logger().error(f"Reset Failed: {e}")
            return False

    def read_loop(self):
        """
        The Supervisor Thread. It tries to read. If it fails, it fixes the bus.
        """
        # Initial Setup
        while self.running and not self.sensor_ready:
            if self.hard_reset_i2c(): break
            time.sleep(1.0)

        while self.running and rclpy.ok():
            try:
                # --- READ ATTEMPT ---
                # This blocks on I2C. If the bus dies, this throws an exception.
                qi, qj, qk, qr = self.bno.quaternion
                ax, ay, az = self.bno.acceleration
                gx, gy, gz = self.bno.gyro

                now = time.monotonic()
                
                # Update State safely
                with self.lock:
                    self.last_hw_q = q_normalize((qi, qj, qk, qr))
                    self.last_hw_accel = (ax, ay, az)
                    self.last_hw_gyro = (gx, gy, gz)
                    self.last_hw_time = now
                    self.new_hw_data_available = True
                    self.sensor_ready = True

                # Tiny sleep to yield CPU
                time.sleep(0.01)

            except Exception as e:
                # --- FAILURE DETECTED ---
                error_str = str(e)
                self.get_logger().warn(f"Read Error: {error_str}")
                self.health_pub.publish(String(data=f"IMU ERROR: {error_str}"))
                
                # Check for critical errors that require a Reset
                critical = ["Errno 6", "No such device", "Remote I/O", "255", "Unprocessable"]
                
                if any(c in error_str for c in critical) or isinstance(e, OSError):
                    self.get_logger().error("Critical Bus Failure detected. Initiating Recovery...")
                    
                    # Loop until we fix it or shutting down
                    while self.running and rclpy.ok():
                        if self.hard_reset_i2c():
                            break # We are back!
                        time.sleep(1.0) # Wait before retry

    def publish_cycle(self):
        """
        Runs at 60Hz. Even during a reset, this keeps publishing (using prediction/fallback).
        """
        now = time.monotonic()
        
        with self.lock:
            hw_q = self.last_hw_q
            hw_gyro = self.last_hw_gyro
            hw_accel = self.last_hw_accel
            last_time = self.last_hw_time
            is_ready = self.sensor_ready
            is_fresh = self.new_hw_data_available
            if is_fresh: self.new_hw_data_available = False

        # Calculate time delta
        dt = now - last_time
        
        # If the sensor has been dead for > 2 seconds, stop predicting (safety)
        if dt > 2.0:
            if not is_ready:
                self.health_pub.publish(String(data="IMU RECOVERING"))
            else:
                self.health_pub.publish(String(data="IMU TIMEOUT"))
            # We still publish the last known quaternion to keep nodes from crashing,
            # but we zero the gyro so the robot stops "spinning" in its head.
            hw_gyro = (0.0, 0.0, 0.0)

        # PREDICTION LOGIC (Gyro Integration)
        # This keeps the motion smooth even if I2C is stuttering or resetting
        dq = quat_from_gyro(hw_gyro[0], hw_gyro[1], hw_gyro[2], dt)
        predicted_q = q_normalize(q_mul(hw_q, dq))
        
        self.publish_ros_msgs(predicted_q, hw_gyro, hw_accel)

    def yaw_to_cardinal(self, heading_degrees):
        directions = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West']
        index = round(heading_degrees / 45.0) % 8
        return directions[index]

    def publish_ros_msgs(self, q, gyro, accel):
        # 1. IMU Msg
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyro
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = accel
        self.imu_publisher_.publish(msg)

        # 2. Euler Msg (USING YOUR UTILS)
        roll, pitch, raw_yaw = quaternion_to_euler(*q)
        
        vec = Vector3()
        vec.x = roll
        vec.y = pitch
        vec.z = raw_yaw
        self.rpy_publisher_.publish(vec)
        
        # 3. Heading Msg (Calibrated)
        heading_degrees = (-raw_yaw + self.mounting_offset_deg + 360) % 360
        cardinal = self.yaw_to_cardinal(heading_degrees)
        
        # Exact format for GUI
        heading_str = f'Heading: {cardinal}, {heading_degrees:.2f} degrees'
        
        self.heading_publisher_.publish(String(data=heading_str))

def main(args=None):
    rclpy.init(args=args)
    node = ImmortalIMUNode()
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
