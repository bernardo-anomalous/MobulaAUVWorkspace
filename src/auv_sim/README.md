# auv_sim

A lightweight simulator for GUI development that mirrors the AUV's telemetry interfaces.

## Custom message dependency
These simulator nodes rely on the existing `auv_custom_interfaces` package for message
contracts such as `ServoMovementCommand`, `HoldRequest`, and `SystemHealth`. Build and
source that package alongside `auv_sim` on the portable machine; otherwise the simulator
nodes will fail to import their message types and won't match the GUI's expected topics.

## Included nodes
- `motion_sim`: publishes smooth roll/pitch/yaw/heading on the same IMU topics as the real vehicle. Adjustable
  bounds let you constrain roll/pitch/yaw ranges to exercise GUI triggers.
- `depth_sim`: publishes slow depth changes on `/depth` and `/depth_air`. A target-depth mode can be toggled to
  climb or dive toward a specified value at a configurable rate.
- `servo_sim`: mirrors the servo lifecycle heartbeat/status and consumes `ServoMovementCommand`
  and `HoldRequest` messages to update servo angles.
- `battery_system_sim`: emits battery pack telemetry and system health summaries using the
  existing `SystemHealth` message. Battery type, capacity, and discharge rate parameters let you
  simulate faster or slower voltage drop and state-of-charge decay.

Launch all simulators together with `ros2 launch auv_sim simulation.launch.py`, or run the
entry points individually via `ros2 run auv_sim <node>`.

## Portable usage on another machine
You can copy only `auv_sim` plus the existing `auv_custom_interfaces` package into a clean
ROS 2 workspace on your laptop:

1. Place both packages under `src/` in your portable workspace.
2. Build just these packages: `colcon build --packages-select auv_custom_interfaces auv_sim`.
3. Source the workspace: `source install/setup.bash`.
4. Run the simulators with `ros2 launch auv_sim simulation.launch.py` or individual
   `ros2 run` commands.

The nodes are pure Python and depend only on standard ROS 2 runtime libraries, so no
hardware-specific system packages are required.
