# Battery Node (`battery_node.py`)

Monitors two INA260 sensors (compute and actuation packs) over I²C, estimating
voltage, current, power, state of charge, and behaviour-based runtime for each
pack while persisting counters between runs.【F:src/auv_pkg/auv_pkg/battery_node.py†L90-L485】

## Sampling and estimation
- Each `Pack` manages its own I²C handle, retries sensor bring-up, and exposes
  multiple publishers (voltage, current, power, SoC, usage, ETA, state) under
  `/battery/<name>/…`.【F:src/auv_pkg/auv_pkg/battery_node.py†L96-L180】【F:src/auv_pkg/auv_pkg/battery_node.py†L263-L300】
- Samples convert INA260 readings to SI units, blend coulomb counting with an
  OCV-based SoC estimate, compute moving-average power for ETA, and summarise
  status text for dashboards.【F:src/auv_pkg/auv_pkg/battery_node.py†L185-L295】
- Current thresholds trigger warning/critical logs plus JSON payloads on
  monitoring topics, recording peaks and fuse proximity for safety checks.【F:src/auv_pkg/auv_pkg/battery_node.py†L341-L407】

## Node parameters and timers
- Configurable I²C bus, sensor addresses, capacities, series-cell counts,
  poll rate, and behaviour averaging window are declared on startup.【F:src/auv_pkg/auv_pkg/battery_node.py†L418-L469】
- A periodic sampler ticks each pack, while a 30 s timer persists usage counters
  to `~/.local/share/auv_battery_node.json` for continuity across reboots.【F:src/auv_pkg/auv_pkg/battery_node.py†L456-L485】
- Parameter updates adjust capacities, cell counts, and behaviour windows at
  runtime, logging any changes applied.【F:src/auv_pkg/auv_pkg/battery_node.py†L486-L507】
