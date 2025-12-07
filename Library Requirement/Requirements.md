# Library Requirements

## Suspected native (expected with Ubuntu + ROS 2)
- **console_bridge_node**: Uses `rclpy`, `rcl_interfaces.msg.Log`, and `std_msgs.msg.String` which are provided by a standard ROS 2 installation.【F:src/auv_pkg/auv_pkg/console_bridge_node.py†L6-L56】
- **servo_interpolation**: Depends on `rclpy`, standard message types, and `numpy`, all typically available via base ROS 2/Python installs.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L1-L60】
- **pitch_pid / roll_pid**: Use `rclpy`, `geometry_msgs`, `std_msgs`, and custom messages from `auv_custom_interfaces`; these are bundled in the workspace or standard ROS 2 packages.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L1-L18】【F:src/auv_pkg/auv_pkg/roll_pid.py†L1-L9】
- **acceleration_node**: Relies on `rclpy`, `sensor_msgs`, `geometry_msgs`, and `numpy`, which are standard dependencies in many ROS 2 Python environments.【F:src/auv_pkg/auv_pkg/acceleration_node.py†L1-L36】
- **system_monitor_node**: Uses ROS 2 core libraries and optionally `psutil`; the node degrades gracefully if `psutil` is absent, but it is commonly available via system packages.【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L6-L30】

## Required installation (extra hardware/third‑party libraries)
- **imu_node**: Requires Adafruit IMU stack (`adafruit_bno08x`, `adafruit_extended_bus`, `board`, `busio`) for BNO08X over I2C in addition to ROS 2 messaging.【F:src/auv_pkg/auv_pkg/imu_node.py†L16-L41】
- **servo_driver**: Needs Adafruit PCA9685 and servo control libraries plus I2C helpers (`adafruit_pca9685`, `adafruit_motor.servo`, `board`, `busio`).【F:src/auv_pkg/auv_pkg/servo_driver.py†L3-L24】
- **depth_sensor_node**: Depends on the `ms5837` pressure sensor library for depth readings over I2C.【F:src/auv_pkg/auv_pkg/depth_node.py†L3-L10】
- **battery_node**: Uses Adafruit INA260 power sensor support and I2C helpers (`adafruit_ina260`, `adafruit_extended_bus`, `board`, `busio`).【F:src/auv_pkg/auv_pkg/battery_node.py†L8-L31】
