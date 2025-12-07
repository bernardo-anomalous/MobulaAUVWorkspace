# Servo Driver Node (`servo_driver.py`)

Lifecycle node that owns the PCA9685 board and drives all six servos. It is the
execution endpoint for interpolated trajectories and PID corrections, exposing
its own lifecycle to let bringup scripts stage hardware safely.【F:README.md†L115-L120】

## Purpose and runtime loop
- Configures hardware (unless `simulation_mode` is true) when entering the
  *configured* state, moves servos to the glide pose with staggered tail
  startup, and subscribes to command topics.【F:src/auv_pkg/auv_pkg/servo_driver.py†L200-L241】【F:src/auv_pkg/auv_pkg/servo_driver.py†L175-L213】
- Refresh timer writes servo angles at `update_rate_hz` (default 60 Hz) and
  republishes the latest angles on `current_servo_angles`. Deadbands keep I²C
  traffic low by skipping tiny movements.【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L39】【F:src/auv_pkg/auv_pkg/servo_driver.py†L92-L134】
- Heartbeat and lifecycle publishers advertise status for monitors; failure
  counters trigger soft I²C resets or self-restart when repeated errors occur.
  【F:src/auv_pkg/auv_pkg/servo_driver.py†L14-L20】【F:src/auv_pkg/auv_pkg/servo_driver.py†L40-L89】

## Parameters
Key parameters declared during construction:
| Name | Default | Notes |
| --- | --- | --- |
| `glide_position` | `[90,135,90,135,90,90]` | Safe neutral pose applied at startup.【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L31】【F:src/auv_pkg/auv_pkg/servo_driver.py†L137-L174】 |
| `update_rate_hz` | `60.0` | Rate for the refresh timer and angle publisher.【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L32】【F:src/auv_pkg/auv_pkg/servo_driver.py†L86-L134】 |
| `simulation_mode` | `False` | Skip hardware init; still publishes lifecycle events.【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L32】【F:src/auv_pkg/auv_pkg/servo_driver.py†L200-L218】 |
| `debug_logging` | `False` | Verbose command logging.【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L32】【F:src/auv_pkg/auv_pkg/servo_driver.py†L258-L283】 |
| `stubborn` | `True` | Enables self-recovery and restart on repeated I²C failures.【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L39】【F:src/auv_pkg/auv_pkg/servo_driver.py†L40-L89】 |
| `heavy_servo_startup_delay` | `1.5` | Staggers heavy wing servos 0 and 2 during glide setup.【F:src/auv_pkg/auv_pkg/servo_driver.py†L23-L39】【F:src/auv_pkg/auv_pkg/servo_driver.py†L137-L174】 |

## Topics
- **Publishes**: `current_servo_angles` (`Float32MultiArray`),
  `servo_driver/heartbeat` (`String`), `servo_driver_status` (`String`), and
  `servo_driver/lifecycle_state` (`String`) to advertise activity and readiness.
  【F:src/auv_pkg/auv_pkg/servo_driver.py†L175-L220】
- **Subscribes**: `servo_driver_commands` and `tail_commands`
  (`ServoMovementCommand`) for main motion input; `wing_hold_request`
  (`HoldRequest`, transient-local) to respect hold mode and suppress non-PID
  moves.【F:src/auv_pkg/auv_pkg/servo_driver.py†L200-L241】【F:src/auv_pkg/auv_pkg/servo_driver.py†L213-L238】

## Lifecycle behaviour
- **configure**: initialise hardware (unless simulated), move to glide pose,
  and start subscriptions.【F:src/auv_pkg/auv_pkg/servo_driver.py†L200-L241】
- **activate**: start heartbeat and refresh timers, mark state `active`.【F:src/auv_pkg/auv_pkg/servo_driver.py†L241-L260】
- **deactivate/cleanup/shutdown**: stop timers, drop PWM duty cycles to zero,
  and release the PCA9685/I²C handles to leave the servos limp.【F:src/auv_pkg/auv_pkg/servo_driver.py†L61-L89】【F:src/auv_pkg/auv_pkg/servo_driver.py†L260-L313】

## Interactions
- `servo_interpolation` feeds this node via `servo_driver_commands`; PID nodes
  often tag `movement_type` with `pid_control` so the driver applies tighter
  deadbands during stabilisation.【F:src/auv_pkg/auv_pkg/servo_driver.py†L86-L134】【F:src/auv_pkg/auv_pkg/servo_driver.py†L258-L317】
- When the driver reports busy states, higher-level planners can defer new
  trajectories or use the hold channel to freeze wings during safety events.
