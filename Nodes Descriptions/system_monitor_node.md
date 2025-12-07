# System Monitor Node (`system_monitor_node.py`)

Collects Raspberry Pi system health data, logs startup diagnostics, and
publishes both human-readable summaries and structured `SystemHealth` messages
for the rest of the stack.【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L33-L210】【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L624-L914】

## Monitoring loop
- Declares polling rates, log directory, and topic names, prepares publishers for
  summaries, events, health telemetry, and previous-shutdown info, and records
  startup progress with durable QoS.【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L45-L210】
- Fast timer reads firmware throttle flags, CPU/GPU temperatures, uptime, and
  recent power/thermal events to produce a textual summary and update the
  fast-metrics cache.【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L624-L733】【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L677-L730】
- Slow timer uses `psutil` (when available) to gather CPU, memory, disk, and
  network stats, caching results for summaries and filling a `SystemHealth`
  message that is published each cycle.【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L780-L914】

## Event handling and shutdown
- Converts throttling or over-temperature conditions into warnings and event
  messages, tracking counters and maximum CPU temperature for session logs.【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L624-L733】【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L923-L969】
- On shutdown, writes a session log, rotates old logs, and republishes previous
  boot analysis so operators can diagnose resets or brownouts.【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L600-L618】【F:src/auv_pkg/auv_pkg/system_monitor_node.py†L923-L969】
