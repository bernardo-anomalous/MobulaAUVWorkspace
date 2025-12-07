# Servo Interpolation Node (`servo_interpolation.py`)

Generates smooth servo trajectories from high-level commands. It blends segments
with easing functions, cross-fades interruptions, and feeds the servo driver at a
controlled rate so motion stays fluid even when commands arrive irregularly.
【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L9-L178】

## Workflow
- Subscribes to `servo_interpolation_commands` for `ServoMovementCommand`
  messages. Each command contains servo IDs, per-segment target angles, segment
  durations, easing choices, and metadata such as `movement_type` and priority.
  【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L43-L176】
- Uses the latest `current_servo_angles` feedback from the driver as a starting
  pose, unless an interpolation is already running (feed-forward mode).【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L55-L97】
- Builds per-step trajectories at `update_rate_hz` (default 30 Hz) with optional
  interrupt transitions and cross-fading to avoid jerks when a new command
  pre-empts the current one.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L14-L178】
- Publishes each step as a `ServoMovementCommand` to `servo_driver_commands`;
  also mirrors hold state on `wing_hold_request` using transient-local QoS so
  late-joining nodes respect freezes.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L23-L71】

## Parameters
| Name | Default | Role |
| --- | --- | --- |
| `interpolation_density` | `10` | Steps per segment, controls smoothness.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L14-L27】 |
| `update_rate_hz` | `30` | Publish rate for interpolated steps; intentionally below the driver’s 60 Hz refresh to prevent frame drops.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L14-L27】 |
| `cross_fade_factor` | `0.5` | Percentage of steps blended with the previous command during handover.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L14-L27】【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L150-L174】 |
| `interrupt_transition_duration` | `0.8` | Duration (s) of the smoothing bridge applied when a new command interrupts the old one.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L14-L27】【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L132-L149】 |
| `interrupt_transition_easing` | `cubic` | Easing algorithm for the interrupt bridge.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L14-L27】【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L132-L149】 |

## Interactions
- Requires `current_servo_angles` feedback from `servo_driver` to seed the first
  trajectory; if unavailable, commands are ignored until a baseline arrives.
- PID controllers and teleop nodes can set `movement_type` (e.g., `pid_control`)
  so downstream logic recognises stabilisation moves.
- The hold channel keeps the driver and interpolator in sync: activating hold
  prevents new trajectories until the flag is cleared, protecting against
  thruster conflicts during safety stops.【F:src/auv_pkg/auv_pkg/servo_interpolation.py†L23-L71】
