# Battery Monitoring Node (`auv_pkg.battery_node`)

This node reads one or two INA260 sensors, tracks consumption for each pack, and
publishes detailed telemetry so other nodes (or dashboards) can make decisions.
It is launched automatically by `mobula.launch.xml` but can also run on its own:

```bash
ros2 run auv_pkg battery_node
```

## Hardware assumptions

- INA260 modules with the current path wired from **V+ (battery)** → **V- (load)**.
- Default I²C bus: `/dev/i2c-1` (configurable via the `i2c_bus_id` parameter).【F:src/auv_pkg/auv_pkg/battery_node.py†L100-L109】【F:src/auv_pkg/auv_pkg/battery_node.py†L263-L289】
- Default addresses assume solder bridges on A0/A1:
  - Compute pack: `0x45` (A0 + A1 bridged).
  - Actuation pack: `0x44` (A1 bridged, A0 open).【F:src/auv_pkg/auv_pkg/battery_node.py†L263-L277】

If only one pack is present the other channel stays offline and the node keeps
retrying in the background.【F:src/auv_pkg/auv_pkg/battery_node.py†L129-L155】

## Published topics

For each detected pack (`compute` and `actuation`) the node creates the
following topics under `/battery/<pack>/…` (standard QoS unless noted):

| Topic suffix | Type | Notes |
| --- | --- | --- |
| `present` | `std_msgs/Bool` | Latched (`TRANSIENT_LOCAL`) indicator that the sensor is online.【F:src/auv_pkg/auv_pkg/battery_node.py†L100-L118】 |
| `summary` | `std_msgs/String` | Human-readable line with voltage, current, SoC, usage, ETA.【F:src/auv_pkg/auv_pkg/battery_node.py†L174-L210】 |
| `voltage_V`, `current_A`, `power_W` | `std_msgs/Float32` | Raw telemetry converted to SI units.【F:src/auv_pkg/auv_pkg/battery_node.py†L144-L173】 |
| `soc_pct` | `std_msgs/Float32` | Blend of coulomb counting and OCV estimate.【F:src/auv_pkg/auv_pkg/battery_node.py†L136-L168】 |
| `used_mAh`, `used_Wh` | `std_msgs/Float32` | Integrated consumption counters.【F:src/auv_pkg/auv_pkg/battery_node.py†L132-L168】 |
| `eta_min` | `std_msgs/Float32` | Remaining runtime in minutes (if power draw > 0).【F:src/auv_pkg/auv_pkg/battery_node.py†L168-L210】 |
| `behavior_based_estimation` | `std_msgs/String` | Average power and ETA over the rolling behaviour window.【F:src/auv_pkg/auv_pkg/battery_node.py†L123-L210】 |
| `voltage_state` | `std_msgs/String` | Latched state (`NOMINAL`, `LOW`, `CRITICAL`) with thresholds per cell.【F:src/auv_pkg/auv_pkg/battery_node.py†L82-L138】【F:src/auv_pkg/auv_pkg/battery_node.py†L210-L233】 |
| `current_monitor` | `std_msgs/String` | JSON payload with current draw details when thresholds are crossed.【F:src/auv_pkg/auv_pkg/battery_node.py†L139-L209】【F:src/auv_pkg/auv_pkg/battery_node.py†L210-L259】 |
| `current_draw_critical` | `std_msgs/String` | Latched version of `current_monitor` emitted only for critical events.【F:src/auv_pkg/auv_pkg/battery_node.py†L210-L259】 |

All strings are UTF-8 and suitable for dashboards or logging.

## Runtime parameters

Parameters can be changed at startup or with `ros2 param set` while the node is
running. The on-set callback applies changes immediately and logs which values
were updated.【F:src/auv_pkg/auv_pkg/battery_node.py†L306-L338】

| Parameter | Default | Purpose |
| --- | --- | --- |
| `i2c_bus_id` | `1` | Linux I²C bus used to talk to the INA260s.【F:src/auv_pkg/auv_pkg/battery_node.py†L263-L271】 |
| `compute_addr`, `actuation_addr` | `0x45`, `0x44` | Sensor addresses (int or hex string).【F:src/auv_pkg/auv_pkg/battery_node.py†L263-L279】 |
| `compute_capacity_mAh`, `actuation_capacity_mAh` | `5300.0` | Nominal pack capacity for coulomb counting.【F:src/auv_pkg/auv_pkg/battery_node.py†L269-L289】 |
| `compute_series_cells`, `actuation_series_cells` | `3` | Number of series cells for voltage classification.【F:src/auv_pkg/auv_pkg/battery_node.py†L269-L289】 |
| `poll_hz` | `5.0` | Sampling rate for each sensor.【F:src/auv_pkg/auv_pkg/battery_node.py†L289-L297】 |
| `behavior_window_sec` | `300` | Duration of the rolling window used for average power/ETA.【F:src/auv_pkg/auv_pkg/battery_node.py†L104-L123】【F:src/auv_pkg/auv_pkg/battery_node.py†L263-L338】 |

## Current monitoring and safety thresholds

Each pack tracks the highest observed current and publishes structured events
when draw exceeds the configurable thresholds embedded in the code (6 A
important, 8 A critical, warnings when near the 10 A fuse). These messages are
JSON strings containing severity, current, timestamp, and fuse information so a
human operator can react quickly.【F:src/auv_pkg/auv_pkg/battery_node.py†L118-L209】

## Persistence

Integrated consumption counters are stored at
`~/.local/share/auv_battery_node.json` every 30 seconds and again during
shutdown. The file stores `used_mAh` and `used_Wh` for each pack and includes a
version number so future changes can invalidate stale data safely.【F:src/auv_pkg/auv_pkg/battery_node.py†L40-L69】【F:src/auv_pkg/auv_pkg/battery_node.py†L318-L338】

## Behaviour-based runtime estimate

A per-pack deque stores the last `behavior_window_sec` seconds of power data.
The average power is used to compute a remaining minutes estimate (converted
from nominal pack Wh). The summary string and the dedicated
`behavior_based_estimation` topic both include this ETA, which is suppressed if
power draw is nearly zero.【F:src/auv_pkg/auv_pkg/battery_node.py†L104-L210】

## Recovery and shutdown

- If a sensor read fails the node logs a warning, marks the pack as offline, and
  retries on subsequent timer ticks without blocking the rest of the system.【F:src/auv_pkg/auv_pkg/battery_node.py†L123-L155】【F:src/auv_pkg/auv_pkg/battery_node.py†L210-L259】
- On shutdown the node persists counters one last time and destroys the ROS
  node cleanly.【F:src/auv_pkg/auv_pkg/battery_node.py†L340-L382】

Use these details when wiring the hardware, configuring launch files, or
building dashboards that visualise the pack status.
