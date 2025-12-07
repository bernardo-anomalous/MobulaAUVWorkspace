# Acceleration Node (`acceleration_node.py`)

Transforms IMU linear acceleration into the vehicle body frame with gravity
removed. This gives controllers surge/sway/heave estimates that align with the
vehicle, regardless of sensor mounting angle.【F:src/auv_pkg/auv_pkg/acceleration_node.py†L1-L46】

## Pipeline
- Subscribes to `imu_topic` (default `imu/data`) for `sensor_msgs/Imu` messages.
  Each callback extracts the accelerometer vector and orientation quaternion.
  【F:src/auv_pkg/auv_pkg/acceleration_node.py†L9-L40】
- Applies the configured mounting rotation then rotates a gravity vector into
  the body frame via the inverse quaternion. Subtracting that gravity estimate
  yields linear acceleration in body axes.【F:src/auv_pkg/auv_pkg/acceleration_node.py†L9-L70】
- Publishes the result as a `geometry_msgs/Vector3` on `publish_topic`
  (default `acceleration/body_linear`).【F:src/auv_pkg/auv_pkg/acceleration_node.py†L9-L46】

## Parameters
| Name | Default | Effect |
| --- | --- | --- |
| `imu_topic` | `imu/data` | Source IMU stream.【F:src/auv_pkg/auv_pkg/acceleration_node.py†L11-L25】 |
| `publish_topic` | `acceleration/body_linear` | Output topic for processed acceleration.【F:src/auv_pkg/auv_pkg/acceleration_node.py†L11-L25】 |
| `gravity_m_s2` | `9.80665` | Gravity magnitude subtracted from measurements.【F:src/auv_pkg/auv_pkg/acceleration_node.py†L11-L32】 |
| `mount_rpy_deg` | `[0,0,0]` | Roll/pitch/yaw offsets describing how the IMU is mounted relative to the hull.【F:src/auv_pkg/auv_pkg/acceleration_node.py†L13-L33】 |

## Interactions
- Consumes `imu_node` output; large heading offsets there will affect gravity
  removal accuracy here.
- Downstream planners can integrate the published vectors to estimate surge/sway
  motion or fuse with depth data for state estimation.
