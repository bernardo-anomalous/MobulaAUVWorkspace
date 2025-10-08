— Design & Usage

This document explains how the battery monitoring node is structured, how it works, and how to configure and operate it on your AUV. The node reads one or two INA260 power monitors over I²C, estimates state-of-charge (SoC) for LiPo packs, and publishes health/telemetry topics for downstream consumers.

1) What the node does (at a glance)

Reads voltage (V), current (A), and computes power (W) from one or two INA260s.

Publishes raw telemetry, a human-readable summary, diagnostic status, and a behavior-based runtime estimate.

Tracks consumption (mAh/Wh) with proper unit handling and numerically stable integration.

Estimates SoC using a LiPo-appropriate open-circuit-voltage (OCV) curve with sag compensation.

Survives faults: non-blocking startup, retries if a sensor is missing, no spin lockups.

Persists counters to disk periodically and on clean shutdown (Ctrl-C).

Runtime reconfigurable parameters (warn thresholds, capacities, behavior window, etc.).

2) Hardware assumptions & wiring

INA260 modules (each integrates a 2 mΩ shunt, measures both bus voltage and shunt current).

Addresses on your boards:

Board A: A1 bridged, A0 bridged → address 0x45 (compute battery).

Board B: A1 bridged, A0 open → address 0x41 (actuation battery).

Current path: route pack positive through the INA260 V+ → V- in the correct direction.

I²C: keep SDA/SCL short and clean; typical bus = /dev/i2c-1 on Pi. Pull-ups may already exist on the HAT or breakout; avoid double/too-strong pull-ups.

⚠️ Polarity matters: V+ toward battery, V- toward load. Reversed direction will flip current sign.

3) ROS 2 interfaces
Topics (publishers)

/battery/compute/raw (std_msgs/Float32MultiArray or struct-like msg)

[voltage_V, current_A, power_W] at 5 Hz.

/battery/actuation/raw (same schema as above; only if the second INA260 is present).

/battery/compute/summary (std_msgs/String)

Human-readable snapshot: compute: V=12.26V I=0.53A P~6.7W SoC=76.2% used=342mAh/5300mAh.

/battery/actuation/summary (std_msgs/String) (if present).

/battery/diagnostics (diagnostic_msgs/DiagnosticArray)

Error rates, stale flags, UV/OCP flags, last exception, presence booleans.

/battery/behavior_based_estimation (std_msgs/String)

Rolling 5-minute behavior window: average current/power, predicted remaining time.

QoS: Telemetry uses reliable, volatile. For summary & diagnostics, default reliable. (You can tune to your system’s DDS profile.)

4) Parameters (runtime adjustable)
Name	Type	Default	Description
i2c_bus_id	int	1	Linux I²C bus number.
compute_addr	string/int	"0x45"	INA260 address for compute battery.
actuation_addr	string/int	"0x41"	INA260 address for actuation battery (optional).
rate_hz	float	5.0	Publish/read rate.
pack_type	string	"lipo_3s"	Pack chemistry/profile (lipo_3s assumed here).
capacity_mAh_compute	float	5300.0	Capacity of compute pack.
capacity_mAh_actuation	float	5300.0	Capacity of actuation pack.
warn_soc_percent	float	25.0	Warn when estimated SoC drops below this threshold.
crit_soc_percent	float	15.0	Critical threshold.
behavior_window_sec	float	300.0	Rolling window for behavior-based estimate.
sag_compensation	float	0.10	Fractional voltage sag correction used in OCV → SoC mapping.
persist_path	string	~/.local/share/auv/battery_state.json	Where mAh/Wh counters are saved.
persist_interval_sec	float	30.0	Periodic flush interval.

Runtime updates: on_set_parameters_callback lets you change warn thresholds, capacities, window size, etc., without restarting.

Example YAML override

battery_node:
  ros__parameters:
    i2c_bus_id: 1
    compute_addr: 0x45
    actuation_addr: 0x41
    rate_hz: 5.0
    capacity_mAh_compute: 5300.0
    capacity_mAh_actuation: 5300.0
    warn_soc_percent: 25.0
    crit_soc_percent: 15.0
    behavior_window_sec: 300.0
    persist_path: "/home/ubuntu/.local/share/auv/battery_state.json"

5) How the node calculates things
5.1 Unit handling (the “huge A” bug)

The INA260 library returns voltage in volts and current in amperes (not mA) or milli-units depending on the driver. The node normalizes to V and A right after each read.

We integrate current in A over time to get mAh:

ΔmAh = current_A * (Δt_seconds / 3600.0)

Power in W is simply V * A, and energy in Wh integrates similarly.

5.2 SoC (LiPo-aware)

SoC is estimated via a 3-cell LiPo OCV curve. The mapping applies:

Sag compensation: V_adj = V_meas + sag_compensation * (I * R_equiv) (we use a small, bounded heuristic for R_equiv).

OCV→SoC lookup: higher voltage at the top (12.6 V) drops quickly to ~12.0, lingers around mid-range (~11.6 V), then declines fast below ~11.1 V (3.7 V, 3.5 V, 3.3 V per cell landmarks). The curve is clamped to [0, 100%].

The SoC is an estimate; behavior-based output complements it.

5.3 Behavior-based estimation (rolling window)

We keep a timed deque of the last behavior_window_sec worth of samples.

Report average current & power and predict runtime (remaining_mAh / avg_current_A), if current > a small floor.

Publishes a formatted line on /battery/behavior_based_estimation.

6) Fault tolerance & lifecycle

Non-blocking startup: If a sensor is missing on boot (e.g., only compute battery attached), that channel is marked present=false. The node keeps running and retries in the background.

Diagnostics always on: Error counters, stale flags, last exception published even when sensors flap.

Persistence: used_mAh/Wh are periodically written to persist_path and again in the shutdown handler (Ctrl-C).

Clean shutdown: We use a standard SingleThreadedExecutor().spin() inside try/except KeyboardInterrupt and persist state in finally. Ctrl-C in ros2 run/launch terminates cleanly.

7) How to run
Build

Add the console script in setup.py:

'console_scripts': [
  # ...other nodes...
  'battery_node = auv_pkg.battery_node:main',
],


Then:

colcon build --symlink-install
source install/setup.bash

Run (defaults)
ros2 run auv_pkg battery_node

Run with parameters
ros2 run auv_pkg battery_node --ros-args \
  -p compute_addr:=0x45 \
  -p actuation_addr:=0x41 \
  -p capacity_mAh_compute:=5300 \
  -p capacity_mAh_actuation:=5300 \
  -p rate_hz:=5.0

8) Example outputs

Summary

compute:   V=12.26V I=0.53A P~6.7W  SoC=76.2% used=342mAh/5300mAh
actuation: V=11.98V I=1.20A P~14.4W SoC=61.5% used=840mAh/5300mAh


Behavior-based

window=300s avg_I=0.62A avg_P=7.5W est_runtime=3h12m (compute)
window=300s avg_I=1.15A avg_P=13.4W est_runtime=1h58m (actuation)


Diagnostics (humanized)

present: compute=true, actuation=true

read_errors_last_min: 0 / 0

stale: false

last_exception: —

uv_warn: false, crit_warn: false

9) Practical guardrails (implemented)

Non-blocking startup: missing INA260s do not block node init.

Runtime parameter updates: warn thresholds, capacities, and behavior window via on_set_parameters_callback.

Diagnostics always on: error rate, stale status, OCP/UV flags are published regardless of flaps.

Persistence: counters flushed every persist_interval_sec and on shutdown; boot tolerates missing/old file.

10) Troubleshooting

Insane current values (hundreds of A)
Likely a units interpretation issue or address mismatch. Confirm addresses (0x45, 0x44) and ensure the node prints sane units (A, not mA). This node normalizes to A—if you still see nonsense, check wiring and I²C integrity.

One pack not showing
The corresponding INA260 is probably not detected. Verify pull-ups, address solder bridges, and i2c_bus_id. The node will keep retrying; see /battery/diagnostics.

No Ctrl-C shutdown
Ensure you’re running the updated version that uses ex.spin() inside try/except KeyboardInterrupt (no custom SIGINT handlers). The node logs "[BatteryNode] Shutting down..." on exit and persists counters.

SoC feels “off” under sustained load
Increase sag_compensation slightly (e.g., 0.12–0.15) or rely more on the behavior-based estimate for “right-now” decisions, especially during heavy actuation.

11) Notes & future work

Add optional temperature input (from a separate sensor) to improve sag compensation.

Persist a per-pack learned capacity over time (coulomb counting over full cycles).

Optionally publish sensor_msgs/BatteryState for broader ecosystem compatibility.

Status: Deployed and tested at 5 Hz, dual-pack configuration (0x45 compute, 0x41 actuation). Correct unit handling verified; Ctrl-C shutdown confirmed.