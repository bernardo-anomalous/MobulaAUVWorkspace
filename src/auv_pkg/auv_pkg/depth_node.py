#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import ms5837
import time
from collections import deque
import os
import sys

class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor_node')

        self.declare_parameter('stubborn', True)
        self.stubborn = bool(self.get_parameter('stubborn').value)

        self.failure_timestamps = deque()
        self.max_failures = 3
        self.failure_window_seconds = 20
        self.reset_attempts = 0
        self.max_reset_attempts = 3
        self._shutdown_requested = False

        self.sensor = None
        self.timer = None
        self.retry_timer = self.create_timer(5.0, self._retry_init_if_needed)

        # Publishers
        self.publisher_ = self.create_publisher(Float32, '/depth', 10)
        self.air_publisher_ = self.create_publisher(Float32, '/depth_air', 10)

        # Moving average buffers
        self.water_buffer = deque(maxlen=5)
        self.air_buffer = deque(maxlen=5)

        # Deadband setting (meters)
        self.deadband_threshold = 0.05  # ±5 cm

        # Attempt initialization immediately
        self.initialize_sensor()

    def calibrate_surface_pressure(self, duration=5):
        self.get_logger().info(f"🟢 Calibrating surface pressure for {duration} seconds. Keep the AUV floating at the surface...")
        readings = []
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.sensor.read():
                pressure = self.sensor.pressure(ms5837.UNITS_mbar)
                readings.append(pressure)
            time.sleep(0.1)

        if readings:
            avg_pressure = sum(readings) / len(readings)
            self.get_logger().info(f"✅ Surface pressure calibration complete: {avg_pressure:.2f} mbar")
            return avg_pressure
        else:
            self.get_logger().error("❌ Failed to read sensor during calibration!")
            raise RuntimeError("Calibration failed")

    def calculate_air_reference(self):
        DENSITY_AIR = 1.225          # kg/m³
        GRAVITY = 9.80665            # m/s²
        KNOWN_TESTING_ALTITUDE = 12.0  # meters (hardcoded)

        if not self.sensor.read():
            self.get_logger().error("❌ Failed to read sensor during air reference calculation!")
            raise RuntimeError("Sensor read failed")

        initial_pressure = self.sensor.pressure(ms5837.UNITS_mbar)
        delta_pressure = (DENSITY_AIR * GRAVITY * KNOWN_TESTING_ALTITUDE) / 100
        reference_sea_level_pressure = initial_pressure + delta_pressure

        self.get_logger().info(
            f"✅ Calculated sea-level reference pressure for air mode: {reference_sea_level_pressure:.2f} mbar "
            f"(initial {initial_pressure:.2f} mbar, ΔP {delta_pressure:.2f} mbar at {KNOWN_TESTING_ALTITUDE} m)"
        )

        return reference_sea_level_pressure

    def apply_deadband(self, value):
        if abs(value) < self.deadband_threshold:
            return 0.0
        return value

    def publish_depth(self):
        if not self.sensor:
            self.get_logger().warn("⚠️ Sensor not ready; skipping depth publish.")
            return

        try:
            if self.sensor.read():
                pressure_mbar = self.sensor.pressure(ms5837.UNITS_mbar)

                # Raw depth calculations
                # Convert mbar -> Pa by multiplying by 100 before dividing by rho*g
                depth_water = (
                    (pressure_mbar - self.surface_pressure) * 100
                ) / (1025 * 9.80665)
                depth_air = (
                    (self.air_reference_pressure - pressure_mbar) * 100
                ) / (1.225 * 9.80665)

                # Add to buffers
                self.water_buffer.append(depth_water)
                self.air_buffer.append(depth_air)

                # Compute moving averages
                filtered_depth_water = sum(self.water_buffer) / len(self.water_buffer)
                filtered_depth_air = sum(self.air_buffer) / len(self.air_buffer)

                # Apply deadband logic
                filtered_depth_water = self.apply_deadband(filtered_depth_water)
                filtered_depth_air = self.apply_deadband(filtered_depth_air)

                # Publish seawater depth (filtered)
                depth_msg = Float32()
                depth_msg.data = filtered_depth_water
                self.publisher_.publish(depth_msg)

                # Publish air-based depth (filtered)
                air_depth_msg = Float32()
                air_depth_msg.data = filtered_depth_air
                self.air_publisher_.publish(air_depth_msg)
            else:
                self.get_logger().warn("⚠️ Sensor read failed!")
                self._record_failure_and_maybe_recover("sensor read returned False")
        except Exception as e:
            self.get_logger().error(f"❌ Exception while publishing depth: {e}")
            self._record_failure_and_maybe_recover(str(e))

    def initialize_sensor(self):
        try:
            self.sensor = ms5837.MS5837_30BA(bus=1)
            if not self.sensor.init():
                raise RuntimeError("Sensor initialization failed")

            self.get_logger().info("✅ Sensor initialized successfully.")

            # Calibration
            self.surface_pressure = self.calibrate_surface_pressure()
            self.air_reference_pressure = self.calculate_air_reference()

            # Timer for 2 Hz publishing
            if self.timer is None:
                self.timer = self.create_timer(0.5, self.publish_depth)

            self.failure_timestamps.clear()
            self.reset_attempts = 0
            return True
        except Exception as e:
            self.sensor = None
            self.get_logger().error(f"❌ Sensor initialization/calibration failed: {e}")
            if not self.stubborn:
                raise
            self._record_failure_and_maybe_recover(str(e))
            return False

    def _retry_init_if_needed(self):
        if self.sensor or self._shutdown_requested:
            return
        self.initialize_sensor()

    def _record_failure_and_maybe_recover(self, reason: str):
        if not self.stubborn or self._shutdown_requested:
            return
        current_time = time.time()
        self.failure_timestamps.append(current_time)
        while self.failure_timestamps and (current_time - self.failure_timestamps[0] > self.failure_window_seconds):
            self.failure_timestamps.popleft()

        if len(self.failure_timestamps) >= self.max_failures:
            if self.reset_attempts < self.max_reset_attempts:
                self.reset_attempts += 1
                self.get_logger().error(
                    f"Depth sensor unstable ({reason}); attempting reinitialization {self.reset_attempts}/{self.max_reset_attempts}."
                )
                self.initialize_sensor()
            else:
                self.get_logger().fatal("Depth sensor unrecoverable; restarting node.")
                self.restart_process()

    def restart_process(self):
        if not self.stubborn:
            self.get_logger().error("Stubborn mode disabled; not restarting process.")
            return
        self._shutdown_requested = True
        self.get_logger().fatal("Restarting depth sensor node now...")
        python = sys.executable
        os.execv(python, [python] + sys.argv)

def main(args=None):
    rclpy.init(args=args)
    node = DepthSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in depth node: {e}")
        if node.stubborn:
            node.restart_process()
        else:
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
