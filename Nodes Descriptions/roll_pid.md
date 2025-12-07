# Roll PID Node (`roll_pid.py`)

Wing controller that stabilises roll using servos 1 and 3, blending PID control
with hold logic so wings can lock when large errors are detected. Runs at 30 Hz
with smoothing and recovery ramps to keep movements fluid.【F:src/auv_pkg/auv_pkg/roll_pid.py†L11-L278】

## Control loop
- Listens to `target_roll`, `imu/euler`, and `wing_pid_active` plus a transient
  `wing_hold_request` channel to coordinate hold state between nodes.【F:src/auv_pkg/auv_pkg/roll_pid.py†L70-L188】
- Filters target roll changes to suppress sudden jumps, enforces IMU freshness,
  and gradually increases authority after timeouts.【F:src/auv_pkg/auv_pkg/roll_pid.py†L32-L210】
- Computes roll error with deadband, leaky integration, damping, and smoothing
  before clamping to servo limits and issuing commands at a 33 ms cadence.【F:src/auv_pkg/auv_pkg/roll_pid.py†L20-L278】

## Outputs and hold behaviour
- Publishes `ServoMovementCommand` on `servo_driver_commands` with `pid_control`
  movement type and 33 ms durations tuned to the loop rate.【F:src/auv_pkg/auv_pkg/roll_pid.py†L81-L278】
- Automatically requests or releases holds when roll error crosses configured
  thresholds, republishing the hold state for other components to respect the
  locked-wing condition.【F:src/auv_pkg/auv_pkg/roll_pid.py†L64-L188】
