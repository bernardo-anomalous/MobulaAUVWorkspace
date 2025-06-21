#!/usr/bin/env python3

import ms5837
import time
import sys

# Constants
DENSITY_SEAWATER = 1025  # kg/m³
GRAVITY = 9.80665        # m/s²
SEA_LEVEL_PRESSURE = 1013.25  # Standard sea level pressure (mbar)
FOURTH_FLOOR_PRESSURE = 1005.7  # Actual measured pressure at your 4th floor (mbar)

# Command-line mode selection
if len(sys.argv) != 2 or sys.argv[1] not in ["testing", "running"]:
    print("Usage: python3 pressure_sensor_test.py [testing|running]")
    exit(1)

mode = sys.argv[1]
surface_pressure = FOURTH_FLOOR_PRESSURE if mode == "testing" else SEA_LEVEL_PRESSURE
print(f"✅ Mode: {mode} | Surface pressure reference: {surface_pressure:.2f} mbar")

# Initialize sensor on I2C bus 0
sensor = ms5837.MS5837_30BA(bus=0)

if not sensor.init():
    print("❌ Sensor could not be initialized! Check wiring and power.")
    exit(1)

print("✅ Sensor initialized successfully! Starting readings...")

try:
    while True:
        if sensor.read():
            pressure_mbar = sensor.pressure(ms5837.UNITS_mbar)
            temperature_c = sensor.temperature(ms5837.UNITS_Centigrade)

            # Depth calculation relative to selected surface pressure
            # Convert mbar -> Pa by multiplying by 100 before dividing by rho*g
            depth_m = (
                (pressure_mbar - surface_pressure) * 100
            ) / (DENSITY_SEAWATER * GRAVITY)

            print(f"Pressure: {pressure_mbar:.2f} mbar | Temp: {temperature_c:.2f} °C | Depth: {depth_m:.3f} m")
        else:
            print("⚠️ Sensor read failed!")

        time.sleep(1)

except KeyboardInterrupt:
    print("\nStopped by user.")
