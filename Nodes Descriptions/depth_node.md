# Depth Sensor Node (`depth_node.py`)

Reads the MS5837-30BA pressure sensor, calibrates to ambient pressure, and
publishes both water and air-referenced depths for the control stack. Designed
to keep retrying on transient I²C hiccups and optionally restart itself if the
sensor stays unhealthy.【F:src/auv_pkg/auv_pkg/depth_node.py†L12-L69】【F:src/auv_pkg/auv_pkg/depth_node.py†L92-L146】

## Operation
- On startup the node initialises the sensor, performs a 5 s surface-pressure
  average, and computes an air reference using a fixed test altitude. Those
  references seed two moving-average filters that smooth the live readings.
  【F:src/auv_pkg/auv_pkg/depth_node.py†L12-L69】【F:src/auv_pkg/auv_pkg/depth_node.py†L102-L129】
- A timer publishes `/depth` (seawater) and `/depth_air` at 2 Hz. Each value is
  filtered with a five-sample moving average and a ±5 cm deadband to avoid PID
  chatter near the surface.【F:src/auv_pkg/auv_pkg/depth_node.py†L12-L69】【F:src/auv_pkg/auv_pkg/depth_node.py†L74-L105】

## Parameters and recovery
- `stubborn` (default `True`) enables automatic retries and eventual process
  restart after repeated failures; when disabled the node raises exceptions
  instead.【F:src/auv_pkg/auv_pkg/depth_node.py†L12-L30】【F:src/auv_pkg/auv_pkg/depth_node.py†L130-L185】
- Failure counters are tracked over a 20 s window; after three failures the node
  reinitialises the sensor, and after three failed resets it restarts itself to
  clear bus hangs.【F:src/auv_pkg/auv_pkg/depth_node.py†L16-L30】【F:src/auv_pkg/auv_pkg/depth_node.py†L130-L185】

## Topics
- **Publishes**
  - `/depth` (`std_msgs/Float32`): filtered depth in seawater, metres.
  - `/depth_air` (`std_msgs/Float32`): filtered depth computed against an air
    reference, useful for pressure leak checks.【F:src/auv_pkg/auv_pkg/depth_node.py†L12-L69】【F:src/auv_pkg/auv_pkg/depth_node.py†L92-L146】

## Interactions
- PID controllers for pitch/roll and `servo_interpolation` consume `/depth` to
  manage buoyancy or ground-clearance behaviours; expect a 0.5 s update period
  and plan deadbands accordingly.
- System monitors can watch for frequent restart logs as a sign of water ingress
  or I²C wiring issues.
