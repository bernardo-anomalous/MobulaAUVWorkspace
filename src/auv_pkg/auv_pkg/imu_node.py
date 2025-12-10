#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger  # Standard Service for resets

# --- UTILS FALLBACK ---
try:
    from .utils import quaternion_to_euler
except ImportError:
    def quaternion_to_euler(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

# --- Hardware Imports ---
try:
    from adafruit_extended_bus import ExtendedI2C
    _HAS_EXTENDED_I2C = True
except ImportError:
    _HAS_EXTENDED_I2C = False

try:
    import board
    import busio
except ImportError:
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

def yaw_quaternion(angle_rad: float):
    half = 0.5 * angle_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))

def rotate_vector_z(vector, angle_rad: float):
    x, y, z = vector
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return (c * x - s * y, s * x + c * y, z)


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

        self.declare_parameter('mounting_offset_deg', 0)
        self.mounting_offset_deg = float(self.get_parameter('mounting_offset_deg').value)

        self.declare_parameter('mounting_yaw_deg', 0.0)
        self.mounting_yaw_deg = float(self.get_parameter('mounting_yaw_deg').value)
        
        # --- Publishers ---
        self.imu_publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.rpy_publisher_ = self.create_publisher(Vector3, 'imu/euler', 10)
        self.heading_publisher_ = self.create_publisher(String, 'imu/heading', 10)
        
        health_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.health_pub = self.create_publisher(String, 'imu/health_status', health_qos)

        # --- Subscribers ---
        self.create_subscription(Float32, 'imu/nudge_heading', self.cb_nudge_heading, 10)
        self.create_subscription(Float32, 'imu/set_heading_offset', self.cb_set_heading, 10)
        
        # --- Service ---
        self.create_service(Trigger, 'imu/trigger_reset', self.cb_trigger_reset_service)

        # --- State ---
        self.lock = threading.Lock()
        self.running = True
        self.sensor_ready = False
        self.reset_count = 0
        self.forced_reset_pending = False  # Flag to signal read_thread
        
        self.CRITICAL_ERRORS = [
            "No device", "Remote I/O", "Input/output error",
            "Unprocessable Batch bytes", "Errno 6", "255"
        ]
        
        self.last_hw_q = (0.0, 0.0, 0.0, 1.0)
        self.last_hw_gyro = (0.0, 0.0, 0.0)
        self.last_hw_accel = (0.0, 0.0, 0.0)
        self.last_hw_time = time.monotonic()
        self.new_hw_data_available = False

        self.last_health_status = None

        self.i2c = None
        self.bno = None

        # --- Threads ---
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_cycle)
        
        self.get_logger().info(
            f"Robust IMU Node v2.2 Started. Service: /imu/trigger_reset"
        )

    # --- TOPIC CALLBACKS ---
    def cb_nudge_heading(self, msg):
        with self.lock:
            self.mounting_offset_deg = (self.mounting_offset_deg + msg.data) % 360.0
        self.get_logger().info(f"Offset Nudged to: {self.mounting_offset_deg:.1f}")

    def cb_set_heading(self, msg):
        with self.lock:
            self.mounting_offset_deg = msg.data % 360.0
        self.get_logger().info(f"Offset Set to: {self.mounting_offset_deg:.1f}")

    # --- SERVICE CALLBACK ---
    def cb_trigger_reset_service(self, request, response):
        """
        Trigger a manual reset via ROS Service.
        """
        self.get_logger().warn(">>> MANUAL RESET REQUESTED VIA SERVICE <<<")
        
        # Signal the read_thread to throw an exception and restart
        self.forced_reset_pending = True
        
        # Wait briefly to confirm the thread picked it up (max 1s)
        # We don't want to block the executor too long
        wait_start = time.monotonic()
        ack = False
        while time.monotonic() - wait_start < 1.0:
            if not self.forced_reset_pending: # Flag gets cleared when read_loop acts
                ack = True
                break
            time.sleep(0.05)
            
        if ack:
            response.success = True
            response.message = "IMU Reset Initiated"
        else:
            # If the loop is stuck, we still return true because the flag is set,
            # so it WILL happen eventually.
            response.success = True
            response.message = "IMU Reset Queued (Loop busy)"
            
        return response

    # --- HARDWARE RECOVERY LOGIC ---
    def _rebuild_hardware(self):
        """Destroys and recreates the I2C connection with a 'Hail Mary' soft reset."""
        self._publish_health("RESETTING")
        self.get_logger().warn(f"Performing I2C Bus Reset #{self.reset_count + 1}...")
        
        # 1. Teardown
        try:
            if self.i2c:
                if hasattr(self.i2c, 'deinit'): self.i2c.deinit()
                elif hasattr(self.i2c, 'close'): self.i2c.close()
        except: pass
        
        self.i2c = None
        self.bno = None
        time.sleep(0.5)

        # 2. Rebuild Bus & Hail Mary Reset
        try:
            if _HAS_EXTENDED_I2C:
                self.i2c = ExtendedI2C(self.bus_id)
            else:
                self.i2c = busio.I2C(board.SCL, board.SDA)

            # --- SHTP SOFT RESET ATTEMPT ---
            try:
                # Packet: [LenLSB, LenMSB, Channel(1=Exe), Seq, Cmd(1=Reset)]
                # 0x05 bytes total, Ch 1, Seq 0, Cmd 1
                reset_pkt = bytearray([0x05, 0x00, 0x01, 0x00, 0x01]) 
                
                while not self.i2c.try_lock():
                    pass
                try:
                    self.i2c.writeto(self.addr, reset_pkt)
                    self.get_logger().info(">>> Sent SHTP Soft Reset Command <<<")
                finally:
                    self.i2c.unlock()
                
                time.sleep(0.5) # Wait for reboot
            except Exception as e:
                self.get_logger().warn(f"Soft Reset Command Failed (Sensor might be dead): {e}")
            # -------------------------------

            # 3. Initialize Sensor
            self.bno = BNO08X_I2C(self.i2c, address=self.addr)
            
            # Enable features
            self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Rebuild Failed: {e}")
            # Cleanup
            try: 
                if self.i2c: self.i2c.deinit()
            except: pass
            self.i2c = None
            return False

    def _publish_health(self, status):
        if status != self.last_health_status:
            self.health_pub.publish(String(data=status))
            self.last_health_status = status

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
                # --- CHECK MANUAL TRIGGER ---
                if self.forced_reset_pending:
                    self.forced_reset_pending = False
                    raise RuntimeError("Manual Reset Triggered")

                # --- READ ATTEMPT ---
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
                
                is_manual = "Manual Reset" in error_str
                is_critical = any(c in error_str for c in self.CRITICAL_ERRORS) \
                              or isinstance(e, OSError) \
                              or is_manual
                
                if is_critical:
                    log_func = self.get_logger().warn if is_manual else self.get_logger().error
                    log_func(f"CRITICAL FAILURE: {error_str}")
                    
                    with self.lock: self.sensor_ready = False
                    
                    # RETRY LOOP
                    while self.running and rclpy.ok():
                        self.reset_count += 1
                        if self._rebuild_hardware():
                            self.get_logger().info(">>> SENSOR RECOVERED <<<")
                            with self.lock: self.sensor_ready = True
                            self._publish_health("IMU OK")
                            break 
                        time.sleep(2.0)
                else:
                    self.get_logger().warn(f"Minor Glitch: {error_str}")
                    time.sleep(0.05)

    def publish_cycle(self):
        """
        The Broadcast Thread. Runs at 60Hz. 
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
            mounting_yaw_deg = self.mounting_yaw_deg
            if is_fresh: self.new_hw_data_available = False

        dt = now - last_time
        
        if dt > 2.0:
            status = "RECOVERING" if not is_ready else "STALE"
            self._publish_health(status)
            hw_gyro = (0.0, 0.0, 0.0)
        else:
            status = "IMU OK" if is_ready else "RECOVERING"
            self._publish_health(status)

        # --- PREDICTION ---
        dq = quat_from_gyro(hw_gyro[0], hw_gyro[1], hw_gyro[2], dt)
        predicted_q = q_normalize(q_mul(hw_q, dq))

        yaw_rad = math.radians(mounting_yaw_deg)
        mounting_correction_q = yaw_quaternion(-yaw_rad)

        corrected_q = q_normalize(q_mul(predicted_q, mounting_correction_q))
        corrected_gyro = rotate_vector_z(hw_gyro, -yaw_rad)
        corrected_accel = rotate_vector_z(hw_accel, -yaw_rad)

        self.publish_ros_msgs(corrected_q, corrected_gyro, corrected_accel, current_offset)

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

        # 2. Euler
        roll, pitch, raw_yaw = quaternion_to_euler(*q)
        vec = Vector3()
        vec.x = roll
        vec.y = pitch
        vec.z = raw_yaw
        self.rpy_publisher_.publish(vec)
        
        # 3. Heading
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
