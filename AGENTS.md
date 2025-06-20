# Mobula AUV Developer Guidelines

This repository contains the ROS 2 packages for the Mobula AUV. The vehicle runs a Raspberry Pi 4 and therefore has limited CPU power, memory and energy. All code should favour efficiency and low resource usage.

## Repository Overview
- `auv_custom_interfaces` – ROS 2 message definitions.
- `auv_pkg` – Python nodes controlling sensors and actuators.
- `mobula_bringup` – launch files used to start the system.

The main nodes are documented in `README.md` and include:
`imu_node`, `servo_driver`, `servo_interpolation`, `pitch_pid`, `roll_pid`, `depth_node`, `acceleration_node`, `keyboard_control`, and `camera_control`. Data flows from sensors through the controllers to the servos.

## Development Notes
- Target ROS 2 Jazzy and follow standard best practices.
- Optimise for real‑time behaviour; avoid unnecessary allocations and excessive logging on the Raspberry Pi.
- Use asynchronous APIs where possible to keep loops responsive.
- The workspace imports hardware libraries such as `adafruit_pca9685` and `ms5837`. They are not bundled here, so many scripts will not run in environments without the hardware. This is expected.
- When updating code, try running `pytest -q` to execute style linters. These tests may fail if dependencies for the linters are missing.

## Edge Operation Considerations
- Keep CPU usage low to conserve battery life underwater.
- Memory is limited; prefer lightweight data structures and release resources promptly.
- Be mindful of I/O bandwidth on I2C and other buses to avoid contention with sensors.

