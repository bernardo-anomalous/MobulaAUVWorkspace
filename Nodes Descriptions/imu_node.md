# IMU Node (`imu_node.py`)

This node wraps the BNO08X IMU in a dual-threaded ROS 2 node that keeps
orientation data flowing even if the I²C bus hiccups. It publishes raw
quaternions, derived Euler angles, a human-friendly heading string, and a
health status feed so downstream controllers know whether to trust the
measurements.【F:src/auv_pkg/auv_pkg/imu_node.py†L71-L129】

## Purpose and data flow
- Collects rotation, gyro, and accelerometer readings from the BNO08X over
  I²C, then republishes them on standard IMU topics at `publish_rate_hz`
  (60 Hz by default).【F:src/auv_pkg/auv_pkg/imu_node.py†L36-L91】
- The publish thread fuses gyro-based prediction with the last hardware
  quaternion so the AUV keeps getting data during short bus resets. It also
  publishes a `health_status` string (`IMU OK`, `RECOVERING`, `STALE`) that
  other nodes can gate on.【F:src/auv_pkg/auv_pkg/imu_node.py†L92-L127】【F:src/auv_pkg/auv_pkg/imu_node.py†L204-L233】
- Roll/pitch/yaw are derived from quaternions via `utils.quaternion_to_euler`
  and published on `imu/euler`; a friendly cardinal heading goes to
  `imu/heading`. Downstream PID nodes (pitch/roll) consume these topics for
  stabilisation.【F:src/auv_pkg/auv_pkg/imu_node.py†L88-L107】【F:src/auv_pkg/auv_pkg/imu_node.py†L237-L260】

## Parameters
| Name | Default | Effect |
| --- | --- | --- |
| `i2c_bus_id` | `3` | Which Linux I²C bus the BNO08X sits on.【F:src/auv_pkg/auv_pkg/imu_node.py†L75-L83】 |
| `i2c_address` | `0x4B` | BNO08X address; adjust if solder bridges differ.【F:src/auv_pkg/auv_pkg/imu_node.py†L75-L83】 |
| `publish_rate_hz` | `60.0` | Timer rate for publishing IMU data and health.【F:src/auv_pkg/auv_pkg/imu_node.py†L81-L83】【F:src/auv_pkg/auv_pkg/imu_node.py†L123-L129】 |
| `mounting_offset_deg` | `250` | Heading offset applied to yaw before emitting `imu/heading` so magnetic north matches the hull orientation.【F:src/auv_pkg/auv_pkg/imu_node.py†L84-L107】【F:src/auv_pkg/auv_pkg/imu_node.py†L240-L260】 |

Subscribers allow live tweaks to this offset via `imu/nudge_heading` and
`imu/set_heading_offset`, which is handy when aligning the AUV to a compass
rose on deck.【F:src/auv_pkg/auv_pkg/imu_node.py†L96-L140】

## Topics
- **Publishes**
  - `imu/data` (`sensor_msgs/Imu`): orientation quaternion, angular velocity,
    and linear acceleration direct from hardware or predicted when recovering.
  - `imu/euler` (`geometry_msgs/Vector3`): roll, pitch, yaw radians derived
    from the quaternion.【F:src/auv_pkg/auv_pkg/imu_node.py†L88-L107】【F:src/auv_pkg/auv_pkg/imu_node.py†L229-L260】
  - `imu/heading` (`std_msgs/String`): cardinal direction plus degrees after
    applying `mounting_offset_deg`.【F:src/auv_pkg/auv_pkg/imu_node.py†L237-L260】
  - `imu/health_status` (`std_msgs/String`, transient-local): lifecycle /
    recovery state for watchdogs.【F:src/auv_pkg/auv_pkg/imu_node.py†L92-L127】【F:src/auv_pkg/auv_pkg/imu_node.py†L204-L233】
- **Subscribes**
  - `imu/nudge_heading`, `imu/set_heading_offset` (`std_msgs/Float32`): tweak
    heading offset on the fly.【F:src/auv_pkg/auv_pkg/imu_node.py†L96-L140】

## Reliability tactics
- Separate read thread constantly polls the IMU; critical I²C errors trigger a
  full bus rebuild loop until the sensor comes back online.【F:src/auv_pkg/auv_pkg/imu_node.py†L142-L209】
- Publish thread flags stale data after 2 s without fresh hardware samples and
  stops integrating gyro drift until the bus recovers.【F:src/auv_pkg/auv_pkg/imu_node.py†L200-L233】

## Interaction with other nodes
- `servo_interpolation` and PID controllers depend on timely `imu/euler`
  updates; if `health_status` is not `IMU OK` they should pause or degrade
  gracefully.
- `acceleration_node` uses `imu/data` to remove gravity; mismatched
  `mounting_offset_deg` can propagate into downstream navigation estimates.

Use this node as the primary inertial reference; if you see repeated
`RECOVERING` statuses, inspect the I²C bus, cable strain, or BNO08X power.
