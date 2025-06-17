# Mobula AUV ROS2 Workspace

This workspace contains the ROS2 nodes and custom interfaces used to control the **Mobula** autonomous underwater vehicle (AUV).  The main nodes are located in the `auv_pkg` package and are launched together via `mobula_bringup`.

```
src/
 ├── auv_custom_interfaces/   # custom ROS2 messages
 ├── auv_pkg/                 # Python nodes controlling the AUV
 └── mobula_bringup/          # launch files
```

The following sections describe each script, their ROS2 interfaces, and parameters that influence AUV behaviour.

## Custom Message

`auv_custom_interfaces/msg/ServoMovementCommand.msg` defines the command format used by several nodes to send servo positions.  Key fields are:

```
std_msgs/Header header
uint8[] servo_numbers      # indices of affected servos
float32[] target_angles    # desired angles in degrees
float32[] durations        # time to reach each angle
string[] easing_algorithms # interpolation mode (e.g. 'linear')
float32[] easing_in_factors
float32[] easing_out_factors
string movement_type       # semantic tag for the movement
builtin_interfaces/Time deadline
string operational_mode
uint8 priority
```

Source: `auv_custom_interfaces/msg/ServoMovementCommand.msg` lines 1‑11【F:src/auv_custom_interfaces/msg/ServoMovementCommand.msg†L1-L11】.

## Launch Overview

`mobula_bringup/launch/mobula.launch.xml` starts the primary nodes in order:

1. `imu_node` – publishes IMU orientation and health.
2. `servo_driver` – lifecycle node commanding the servos.
3. `servo_interpolation` – converts higher level movement commands to servo commands.
4. `pitch_pid` – tail pitch/roll PID controller.
5. `roll_pid` – wing roll PID controller.
6. `depth_sensor` – reads the MS5837 depth sensor.
7. `acceleration_node` – converts IMU data to world‑frame acceleration.

Source lines【F:src/mobula_bringup/launch/mobula.launch.xml†L3-L13】.

### Node Interaction Diagram

```
                +-------------+      publishes imu/euler      +--------------+
                |   imu_node  | ----------------------------> |  pitch_pid    |
                |             |                               +--------------+
                |             |      publishes imu/euler      +--------------+
                |             | ----------------------------> |  roll_pid     |
                +-------------+                               +--------------+
                       |                                          |
                       | publishes imu/data                       |
                       v                                          v
                +-------------+           +--------------+    publishes
                |acceleration |           | depth_sensor |   /depth,
                |    _node    |           +--------------+   /depth_air
                +-------------+
                       |
                       v
       +-----------------------------+
       |     servo_interpolation     |<------+ keyboard_control / camera_control
       +-----------------------------+
                       |
                       v
                +-------------+
                | servo_driver|<---------------- pitch_pid / roll_pid
                +-------------+
                       |
                       v
              publishes current_servo_angles
```

This diagram shows the main data flow for a typical mission. Auxiliary nodes such as `keyboard_control` can inject commands or manage the `servo_driver` lifecycle.

## Node Descriptions

### `imu_node.py`
Publishes IMU orientation and health. Important parameters and behaviour:
* Maintains a `heading_offset` used when translating yaw to compass directions (default `-30.0`)【F:src/auv_pkg/auv_pkg/imu_node.py†L24-L24】.
* Publishes `Imu` messages on `imu/data`, Euler angles on `imu/euler`, textual heading on `imu/heading`, and status on `imu/health_status`【F:src/auv_pkg/auv_pkg/imu_node.py†L27-L31】.
* Periodically attempts sensor reinitialization if read failures occur.

### `servo_driver.py`
Lifecycle node controlling the PCA9685 servo board.
* Declares parameters such as `glide_position`, `update_rate_hz`, `simulation_mode`, and `debug_logging`【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L32】.
* Uses a refresh loop running at `update_rate_hz` to write servo positions to the PCA9685 board.  Only writes when the requested angle changes by more than `angle_tolerance_deg` (1° by default) to reduce I2C traffic【F:src/auv_pkg/auv_pkg/servo_driver.py†L28-L39】【F:src/auv_pkg/auv_pkg/servo_driver.py†L115-L134】.
* On configuration it initializes hardware and subscribes to `servo_driver_commands` and `tail_commands`【F:src/auv_pkg/auv_pkg/servo_driver.py†L200-L213】.
* Publishes servo angles on `current_servo_angles`, a heartbeat on `servo_driver/heartbeat`, busy status on `servo_driver_status`, and lifecycle state on `servo_driver/lifecycle_state`【F:src/auv_pkg/auv_pkg/servo_driver.py†L179-L184】【F:src/auv_pkg/auv_pkg/servo_driver.py†L50-L52】.
* Adafruit `Servo` objects expect commands roughly in the 0–180° range.
* The node can restart itself if repeated I2C errors are detected.【F:src/auv_pkg/auv_pkg/servo_driver.py†L73-L118】

### `servo_interpolation.py`
Interpolates complex movements into small servo steps.
* Parameters `interpolation_density`, `update_rate_hz`, and `cross_fade_factor` control resolution and blending【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L15-L21】.
* Generates servo commands at `update_rate_hz` (70 Hz by default). This determines how often new angles are sent to `servo_driver`.
* Subscribes to `servo_interpolation_commands` and publishes resulting `ServoMovementCommand` messages to `servo_driver_commands`【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L24-L32】.
* Maintains the last known servo angles so movements can cross‑fade smoothly.

### `pitch_pid.py`
Tail pitch and roll PID controller.
* Uses PID coefficients (`kp_pitch`, `ki_pitch`, `kd_pitch`, etc.) and damping to stabilize attitude.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L13-L23】【F:src/auv_pkg/auv_pkg/pitch_pid.py†L37-L46】
* Servo limits stored in `servo_limits` clamp servo 4 to 30–150° and servo 5 to 60–180°【F:src/auv_pkg/auv_pkg/pitch_pid.py†L61-L65】.
* Exponential smoothing of IMU input uses factor `alpha` (default `0.8`)【F:src/auv_pkg/auv_pkg/pitch_pid.py†L69-L69】.
* Publishes commands to `tail_commands` and subscribes to `target_pitch`, `target_roll`, and `imu/euler`【F:src/auv_pkg/auv_pkg/pitch_pid.py†L72-L81】.

### `roll_pid.py`
Wing roll PID controller.
* Gains `kp_roll`, `ki_roll`, `kd_roll` and damping factor tune roll response【F:src/auv_pkg/auv_pkg/roll_pid.py†L9-L23】.
* Uses smoothing factor `alpha` to filter IMU noise【F:src/auv_pkg/auv_pkg/roll_pid.py†L41-L43】.
* Subscribes to `target_roll`, `imu/euler`, and a `wing_pid_active` flag to enable/disable control【F:src/auv_pkg/auv_pkg/roll_pid.py†L51-L54】.
* Servo output is clamped with `min_angle` 90° and `max_angle` 180°.
* Sends servo commands for the wing servos on `servo_driver_commands`【F:src/auv_pkg/auv_pkg/roll_pid.py†L49-L49】.

### `depth_node.py`
Reads the MS5837 pressure sensor and outputs filtered depth measurements.
* Calibrates surface pressure at startup and sets a deadband threshold of ±5 cm to prevent jitter【F:src/auv_pkg/auv_pkg/depth_node.py†L27-L37】【F:src/auv_pkg/auv_pkg/depth_node.py†L33-L35】.
* Publishes seawater depth on `/depth` and an air‑reference depth on `/depth_air` every 0.5 s【F:src/auv_pkg/auv_pkg/depth_node.py†L15-L17】【F:src/auv_pkg/auv_pkg/depth_node.py†L38-L41】.

### `acceleration_node.py`
Computes world‑frame acceleration from IMU data.
* Parameters `imu_topic`, `publish_topic`, `gravity_m_s2` and `mount_orientation_rpy_deg` define the input and transformation【F:src/auv_pkg/auv_pkg/acceleration_node.py†L14-L26】.
* Publishes processed acceleration on `/acceleration/processed` at 10 Hz【F:src/auv_pkg/auv_pkg/acceleration_node.py†L31-L39】.

### `keyboard_control.py` & `keyboard_control_swim.py`
Provide manual control using the keyboard.
* Publish target pitch and roll values on `target_pitch` and `target_roll`【F:src/auv_pkg/auv_pkg/keyboard_control.py†L18-L21】.
* Use a lifecycle service client to command state transitions of `servo_driver`【F:src/auv_pkg/auv_pkg/keyboard_control.py†L22-L30】.
* The `swim` variant can send canned `ServoMovementCommand` sequences and enable/disable the wing PID via `wing_pid_active`【F:src/auv_pkg/auv_pkg/keyboard_control_swim.py†L18-L21】【F:src/auv_pkg/auv_pkg/keyboard_control_swim.py†L20-L21】.

### `camera_control.py`
Optional node mapping arm/hand poses to servo commands using OpenCV and MediaPipe.
* Publishes `ServoMovementCommand` messages to `servo_driver_commands`【F:src/auv_pkg/auv_pkg/camera_control.py†L14-L17】.
* Subscribes to `current_servo_angles` to start from the latest positions【F:src/auv_pkg/auv_pkg/camera_control.py†L16-L17】.
* Main servos are clipped to 0–120° while pitch servos are limited to 45–135°.

### Utility and Test Scripts
* `servo_test.py` provides a CLI to exercise servos for calibration; angles are clamped to 0–270°.
* `pressure_sensor_test.py` reads the pressure sensor outside of ROS.
* `fix_imu_node.py` and other backups serve as experimental versions of main nodes.

## Modifying Behaviour

Many scripts expose constants or parameters that change the AUV response:
* **PID gains and limits** – adjust `kp_*`, `ki_*`, `kd_*`, `integral_limit_*`, `correction_limit_*` inside `pitch_pid.py` and `roll_pid.py`.
* **Smoothing factors** – parameters such as `alpha` in the PID nodes control IMU filtering.
* **Servo ranges** – update `servo_limits` in `pitch_pid.py` or the default `glide_position` parameter of `servo_driver.py`.
* **Interpolation rate** – tune `update_rate_hz` and `interpolation_density` for the `servo_interpolation` node.
* **I2C write frequency** – `servo_driver` writes to the PCA9685 board at `update_rate_hz` (30 Hz by default) but only sends updates when the change exceeds `angle_tolerance_deg`.
* **Depth deadband** – modify `deadband_threshold` in `depth_node.py` to change how small pressure variations are ignored.

Parameter files or launch arguments can override these values to suit specific hardware.

---
This README provides a map of how the ROS2 nodes collaborate to control the AUV. Use it as a starting point when adjusting parameters or extending the system.
