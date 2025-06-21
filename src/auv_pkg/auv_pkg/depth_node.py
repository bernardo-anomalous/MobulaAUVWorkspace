#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import ms5837
import time
from collections import deque

class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor_node')

        # Publishers
        self.publisher_ = self.create_publisher(Float32, '/depth', 10)
        self.air_publisher_ = self.create_publisher(Float32, '/depth_air', 10)

        # Sensor setup
        self.sensor = ms5837.MS5837_30BA(bus=0)
        if not self.sensor.init():
            self.get_logger().error("❌ Sensor could not be initialized! Check wiring and power.")
            raise RuntimeError("Sensor initialization failed")

        self.get_logger().info("✅ Sensor initialized successfully.")

        # Calibration
        self.surface_pressure = self.calibrate_surface_pressure()
        self.air_reference_pressure = self.calculate_air_reference()

        # Moving average buffers
        self.water_buffer = deque(maxlen=5)
        self.air_buffer = deque(maxlen=5)

        # Deadband setting (meters)
        self.deadband_threshold = 0.005  # ±5 cm

        # Timer for 2 Hz publishing
        self.timer = self.create_timer(0.5, self.publish_depth)

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
        reference_sea_level_pressure = initial_pressure + (DENSITY_AIR * GRAVITY * KNOWN_TESTING_ALTITUDE)

        self.get_logger().info(
            f"✅ Calculated sea-level reference pressure for air mode: {reference_sea_level_pressure:.2f} mbar "
            f"(initial reading {initial_pressure:.2f} mbar at {KNOWN_TESTING_ALTITUDE} m altitude)"
        )

        return reference_sea_level_pressure

    def apply_deadband(self, value):
        if abs(value) < self.deadband_threshold:
            return 0.0
        return value

    def publish_depth(self):
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

def main(args=None):
    rclpy.init(args=args)
    node = DepthSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
