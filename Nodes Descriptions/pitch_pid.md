# Pitch PID Node (`pitch_pid.py`)

Tail controller that blends pitch and roll corrections into smooth commands for
servos 4 and 5 at 50 Hz, with leaky integrators, recovery ramping, and low-pass
output smoothing to keep biomimetic motion.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L9-L240】

## Control loop
- Subscribes to `target_pitch`, `target_roll`, `imu/euler`, and `tail_pid_active`
  to track desired attitudes and allow external arming/disarming.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L77-L144】
- Rejects stale IMU data, gradually increases authority after recovery, and runs
  the PID at 50 Hz to match servo timing.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L86-L154】【F:src/auv_pkg/auv_pkg/pitch_pid.py†L202-L240】
- Applies deadbands, leaky integrators, and damping on both axes before mixing
  corrections into left/right tails; clamps to configured servo limits and
  smooths each update with an exponential filter.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L156-L240】

## Outputs
- Publishes `ServoMovementCommand` on `tail_commands` with a 20 ms duration and
  `pid_control` movement type so the driver can apply tight deadbands.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L77-L240】
- Uses servo limit parameters (30–150 deg) and scaling gains to preserve centre
  positions while allowing combined pitch/roll authority.【F:src/auv_pkg/auv_pkg/pitch_pid.py†L68-L215】
