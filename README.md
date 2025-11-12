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
float32[] target_angles    # desired angles in degrees; NaN keeps the current position
float32[] durations        # time to reach each angle
string[] easing_algorithms # interpolation mode (e.g. 'linear')
float32[] easing_in_factors
float32[] easing_out_factors
string movement_type       # semantic tag for the movement
builtin_interfaces/Time deadline
string operational_mode
uint8 priority
```
If a value in `target_angles` is `NaN`, that servo keeps its current position.

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
8. `battery_node` – monitors pack voltage, current, and state of charge.
9. `system_monitor_node` – publishes CPU, thermal, and power throttling status.
10. `console_bridge_node` – rebroadcasts `/rosout` in a plain‑text topic.

Source lines【F:src/mobula_bringup/launch/mobula.launch.xml†L1-L16】.

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

### 'Servo Numbers, Location, Function and range for actuation'

* View Frame: The relative description for the servos uses numbers from 0 to 5 at the moment. And when referring to left and right, its from the perspective as one rides a car, meaning, as if we were inside the auv looking forward. With this in mind, the servo relative positions are the following:

* Servo Number 0: Location: Left. Main servo that controls the flapping motion of the left wing. Angle range: 0 to 180, where 0 is uppermost angle, and 180 the lowermost angle of the flapping motion, and 90 is the gliding position or neutral position, stretched outwards to the sides.

* Servo Number 1: Location Left. Pitch servo, that controls the attack angle of the left wing, rotating the distal two thirds section of the wing, lifting or lowering the trailing edge of the wing's section. Adjusting the angle of attack affects the magnitud of the thrust produced by the flapping motion, and when gliding, via differential actuation controls roll of the auv. Angle range: 90 to 180, with 135 being the center. For this servo, moving the servo to angle 180 lowers the trailing edge of the wing's control surface to its lower limit, and 90 lifts the trailing edge of the control surface to its upper limit. Therefore, if the left wing was swinging upwards and we wanted to create forward thrust, we would lower the trailing edge anywhere between 135 (neutral position) to 180 (lowermost position for the trailing edge). The closer the angle is to 135 the more thrust the wing will produce, but the energy requirements for the swing motion on the main servos will increase. Likewise, the further the angle of attack is from 135 towards on of its lower or uppermost limits, the less thrust the swinging motion will produce, but the energy requirements for the main servos to swing the wings will be reduced. If we wanted to generate roll to the right via actuation of the pitch servo of the left wing, we would also lower the trailing edge of the left wing accordingly, assuming we are moving forward.

* Servo Number 2: Location: Right. Main servo that controls the flapping motion of the right wing. Angle range: 0 to 180, where 0 is lowermost angle, and 180 the uppermost angle of the flapping motion, and 90 is the gliding position or neutral position, stretched outwards to the sides.

* Servo Number 3: Location Right. Pitch servo, that controls the attack angle of the right wing, rotating the distal two thirds section of the wing, lifting or lowering the trailing edge of the wing's section. Adjusting the angle of attack affects the magnitud of the thrust produced by the flapping motion, and when gliding, via differential actuation controls roll of the auv. Angle range: 90 to 180, with 135 being the center. For this servo, moving the servo to angle 90 lowers the trailing edge of the wing's control surface to its lower limit, and 180 lifts the trailing edge of the control surface to its upper limit. Therefore, if the right wing was swinging upwards and we wanted to create forward thrust, we would lower the trailing edge anywhere between 135 (neutral position) to 90 (lowermost position for the trailing edge) The closer the angle is to 135 the more thrust the wing will produce, but the energy requirements for the swing motion on the main servos will increase. Likewise, the further the angle of attack is from 135 towards on of its lower or uppermost limits, the less thrust the swinging motion will produce, but the energy requirements for the main servos to swing the wings will be reduced. If we wanted to generate roll to the right via actuation of the pitch servo of the left wing, we would also lower the trailing edge of the left wing accordingly, assuming we are moving forward.

* Servo Number 4: Location Right. The tail of the auv is composed of two control surfaces, one for the right side of the tail, and one for the left side of the tail. These control surfaces stretch outwards and backwards from the main body of the auv. This arrangement allows to control pitch of the auv and help inducing roll via differential actuation. Angle range: 30 to 150. For this servo, 30 is the uppermost angle (lifting the trailing edge of the control surface), and 150 is the lowermost angle, lowering the trailing edge of the control surface. Meaning, that in order to increase the pitch via this servo, we will lift the trailing edge accordingly (towards 30). In order to reduce the pitch and bring the nose of the auv down, we will lower the trailing edge of the control surface (towards 150). If i wanted to unduce a roll to the right with this servo, we lift the trailing edge of the control surface.

* Servo Number 5: Location Left. The tail of the auv is composed of two control surfaces, one for the right side of the tail, and one for the left side of the tail. These control surfaces stretch outwards and backwards from the main body of the auv. This arrangement allows to control pitch of the auv and help inducing roll via differential actuation. Angle range: 30 to 150. For this servo, 150 is the uppermost angle (lifting the trailing edge of the control surface), and 30 is the lowermost angle, lowering the trailing edge of the control surface. Meaning, that in order to increase the pitch via this servo, we will lift the trailing edge accordingly (towards 150). In order to reduce the pitch and bring the nose of the auv down, we will lower the trailing edge of the control surface (towards 30). If i wanted to unduce a roll to the right with this servo, we lower the trailing edge of the control surface.

All servos are 9Imod 55kg 270 degree servos. Operating at 7.4 volts. Power consumption during stress movements is around 3 amps for the main servos ( servos 0 and 2) all other servos demand much less current during operation. Different gear ratios on the servo assemblies determine the range for each of the servos. They are operated via interfacing with a PCA9685 board on I2C with the Adafruit Servo Library which limits expected degrees from 0 to 180, but it still maps to the full 270 degree range of the servo.

## Node Descriptions

### `imu_node.py`
Publishes IMU orientation and health. Important parameters and behaviour:
* Maintains a `heading_offset` used when translating yaw to compass directions (default `-30.0`)【F:src/auv_pkg/auv_pkg/imu_node.py†L24-L24】.
* Publishes `Imu` messages on `imu/data`, Euler angles on `imu/euler`, textual heading on `imu/heading`, and status on `imu/health_status`【F:src/auv_pkg/auv_pkg/imu_node.py†L27-L31】.
* Periodically attempts sensor reinitialization if read failures occur.
* After repeated failures the node performs a minimal initialization
  (accelerometer only) before restarting.
* If sensor values remain unchanged for `stagnant_timeout_sec` seconds
  (default `3.0`), the node restarts itself to recover from bus hangs.

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
* Parameters `interpolation_density`, `update_rate_hz`, `cross_fade_factor`, and `interrupt_transition_duration` tune resolution and how aggressively new commands blend in【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L13-L36】.
* Subscribes to `servo_interpolation_commands` and the latest `current_servo_angles`, then publishes interpolated `ServoMovementCommand` messages on `servo_driver_commands`【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L28-L52】.
* Publishes `HoldRequest` messages on `wing_hold_request` so the wing controller can request a pause, and ignores new commands while a hold is active【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L37-L47】【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L59-L79】.
* Generates servo commands at `update_rate_hz` (70 Hz by default); when a command ends the final angles are maintained instead of forcing a return to glide position【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L82-L143】.

### `pitch_pid.py`
Tail pitch and roll PID controller.
* Uses PID coefficients (`kp_pitch`, `ki_pitch`, `kd_pitch`, etc.) and damping to stabilize attitude.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L13-L23】【F:src/auv_pkg/auv_pkg/pitch_pid.py†L37-L46】
* Servo limits stored in `servo_limits` clamp servo 4 to 30–150° and servo 5 to 30–150°【F:src/auv_pkg/auv_pkg/pitch_pid.py†L61-L65】.
* Exponential smoothing of IMU input uses factor `alpha` (default `0.8`)【F:src/auv_pkg/auv_pkg/pitch_pid.py†L69-L69】.
* Publishes commands to `tail_commands` and subscribes to `target_pitch`, `target_roll`, `imu/euler`, and a `tail_pid_active` flag to enable/disable control【F:src/auv_pkg/auv_pkg/pitch_pid.py†L74-L83】.
* After calibrating, midpoints seem to be slightly different for each tail servo, it might be due to gear reduction introduced in the assembly. The midpoints are 70 for servo 4, and 105 for servo 5.

### `roll_pid.py`
Wing roll PID controller.
* Gains `kp_roll`, `ki_roll`, `kd_roll` and damping factor tune roll response【F:src/auv_pkg/auv_pkg/roll_pid.py†L9-L23】.
* Uses smoothing factor `alpha` to filter IMU noise【F:src/auv_pkg/auv_pkg/roll_pid.py†L41-L43】.
* Subscribes to `target_roll`, `imu/euler`, and a `wing_pid_active` flag to enable/disable control【F:src/auv_pkg/auv_pkg/roll_pid.py†L51-L54】.
* Servo output is clamped with `min_angle` 90° and `max_angle` 180°.
* Sends servo commands for the wing servos on `servo_driver_commands`【F:src/auv_pkg/auv_pkg/roll_pid.py†L49-L49】.
* Neutral attack angle at 135 degrees.

### `depth_node.py`
Reads the MS5837 pressure sensor and outputs filtered depth measurements.
* Calibrates surface pressure and an air reference at startup, then filters readings with a five-sample moving average and a ±5 cm deadband to prevent jitter【F:src/auv_pkg/auv_pkg/depth_node.py†L12-L67】【F:src/auv_pkg/auv_pkg/depth_node.py†L74-L104】.
* Publishes seawater depth on `/depth` and an air-reference depth on `/depth_air` every 0.5 s (2 Hz)【F:src/auv_pkg/auv_pkg/depth_node.py†L12-L15】【F:src/auv_pkg/auv_pkg/depth_node.py†L92-L105】.

### `acceleration_node.py`
Computes world‑frame acceleration from IMU data.
* Parameters `imu_topic`, `publish_topic`, `gravity_m_s2`, and `mount_orientation_rpy_deg` define the input stream and the rotation applied to the IMU frame【F:src/auv_pkg/auv_pkg/acceleration_node.py†L9-L25】.
* Uses SciPy to rotate acceleration into world coordinates, subtract gravity, and publish `/acceleration/processed` at 10 Hz【F:src/auv_pkg/auv_pkg/acceleration_node.py†L1-L46】.

### `keyboard_control.py` & `keyboard_control_swim.py`
Provide manual control using the keyboard.
* Publish target pitch and roll values on `target_pitch` and `target_roll`【F:src/auv_pkg/auv_pkg/keyboard_control.py†L18-L21】.
* Use a lifecycle service client to command state transitions of `servo_driver`【F:src/auv_pkg/auv_pkg/keyboard_control.py†L22-L30】.
* The `swim` variant can send canned `ServoMovementCommand` sequences and enable/disable the wing PID via `wing_pid_active`【F:src/auv_pkg/auv_pkg/keyboard_control_swim.py†L18-L21】【F:src/auv_pkg/auv_pkg/keyboard_control_swim.py†L20-L21】.

### `battery_node.py`
Monitors up to two INA260 power monitors.
* Publishes voltage, current, power, state of charge, usage counters, and estimated runtime for each pack under topics such as `/battery/compute/voltage_V`, `/battery/compute/soc_pct`, and `/battery/compute/behavior_based_estimation`【F:src/auv_pkg/auv_pkg/battery_node.py†L82-L156】.
* Tracks high-current events, logs them, and emits JSON payloads on `/battery/<pack>/current_monitor` and `/battery/<pack>/current_draw_critical` when thresholds are crossed【F:src/auv_pkg/auv_pkg/battery_node.py†L157-L259】.
* Parameters cover I²C bus selection, pack capacities, series cell counts, sampling rate, and the behaviour window used for runtime estimates【F:src/auv_pkg/auv_pkg/battery_node.py†L263-L338】.

### `system_monitor_node.py`
Reports compute health and throttling status.
* Publishes structured data on `/system/health` (`SystemHealth`), human-readable summaries on `/system/summary`, and detailed events on `/system/events`【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L43-L112】.
* Tracks CPU temperature, throttling flags, network throughput, disk usage, and previous shutdown reasons while writing session logs to disk【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L113-L209】.

### `console_bridge_node.py`
Formats ROS log messages for lightweight clients.
* Subscribes to `/rosout`, reformats entries with timestamp and severity, and republishes them on `console_bridge/log` using a transient-local QoS so late subscribers get the latest logs【F:src/auv_pkg/auv_pkg/console_bridge_node.py†L1-L41】.

### `camera_control.py`
Optional node mapping arm/hand poses to servo commands using OpenCV and MediaPipe.
* Publishes `ServoMovementCommand` messages to `servo_driver_commands`【F:src/auv_pkg/auv_pkg/camera_control.py†L14-L17】.
* Subscribes to `current_servo_angles` to start from the latest positions【F:src/auv_pkg/auv_pkg/camera_control.py†L16-L17】.
* Main servos are clipped to 0–120° while pitch servos are limited to 45–135°.



### Utility and Test Scripts
* `servo_test.py` provides a CLI to exercise servos for calibration; angles are clamped to 0–180°.
* `pressure_sensor_test.py` reads the pressure sensor outside of ROS.
* `fix_imu_node.py` was an experimental fallback. Its logic is now
  integrated into `imu_node`.

## Modifying Behaviour

Many scripts expose constants or parameters that change the AUV response:
* **PID gains and limits** – adjust `kp_*`, `ki_*`, `kd_*`, `integral_limit_*`, `correction_limit_*` inside `pitch_pid.py` and `roll_pid.py`.
* **Smoothing factors** – parameters such as `alpha` in the PID nodes control IMU filtering.
* **Servo ranges** – update `servo_limits` in `pitch_pid.py` or the default `glide_position` parameter of `servo_driver.py`.
* **Interpolation rate** – tune `update_rate_hz` and `interpolation_density` for the `servo_interpolation` node.
* **I2C write frequency** – `servo_driver` writes to the PCA9685 board at `update_rate_hz` (30 Hz by default) but only sends updates when the change exceeds `angle_tolerance_deg`.
* **Depth deadband** – default is ±5 cm. Modify `deadband_threshold` in `depth_node.py` to change how small pressure variations are ignored.

Parameter files or launch arguments can override these values to suit specific hardware.

---
This README provides a map of how the ROS2 nodes collaborate to control the AUV. Use it as a starting point when adjusting parameters or extending the system.
