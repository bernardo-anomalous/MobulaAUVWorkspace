# MobulaAUVWorkspace Agent Guidelines

This repository contains ROS 2 Jazzy packages intended to run on a Raspberry Pi 4
(4 GB) installed in a biomimetic Autonomous Underwater Vehicle (AUV).  The AUV
operates without communication for extended periods, so reliability and crash
recovery are paramount.  These guidelines describe the expectations for agents
working on this repository.

## Key Principles

1. **Reliability on edge hardware** – avoid heavy dependencies and minimize CPU
   usage.  Nodes must handle I²C errors gracefully and restart if failures
   persist (see `servo_driver.py` and `imu_node.py`).
2. **ROS 2 lifecycle** – nodes such as the servo driver use lifecycle
   transitions.  Maintain this structure and publish heartbeats or status topics
   to monitor health.
3. **Consistent servo mapping** – servo numbers follow the conventions used in
   `ServoMovementCommand.msg`.  New motion logic must preserve these mappings so
   that kinematics remain predictable.
4. **Resilience** – when sensors fail repeatedly, nodes restart themselves.
   Follow the patterns in `imu_node.py` and `servo_driver.py` when adding new
   hardware drivers.
5. **Edge resource awareness** – avoid verbose logging or unnecessary memory
   allocations.  Use numpy and math libraries carefully as the Raspberry Pi has
   limited resources.
6. **Thrust generation** – thrust comes from coordinated wing and tail servo
   movements.  See `servo_interpolation.py`, `pitch_pid.py`, and `roll_pid.py`
   for examples of generating thrust cycles and stabilizing the vehicle.

## Development Style

- **Python style** follows `flake8` and `pep257`.  Tests are provided under
  `src/auv_pkg/test/`.  Run `colcon test` or `pytest` before committing.
- Each node provides a `main(args=None)` entry point and is declared in
  `setup.py`.  Follow this pattern for any new nodes.
- Use descriptive log messages and catch hardware exceptions.  Publish health
  status on a topic when appropriate.
- Keep dependencies minimal.  Update `pip_requirements.txt` and
  `apt_package_list.txt` with `./update_package_lists.sh` when new packages are
  required.

## Building and Running

1. Source ROS 2 Jazzy: `source /opt/ros/jazzy/setup.bash`.
2. Build the workspace from the repository root:

   ```bash
   colcon build
   ```

3. Source the overlay and launch the vehicle:

   ```bash
   source install/setup.bash
   ros2 launch mobula_bringup mobula.launch.xml
   ```

This launch file starts the IMU first, then the servo driver, interpolation
node, PID controllers, depth sensor, and acceleration node.

## Testing

Execute the style tests using:

```bash
colcon test --packages-select auv_pkg
```

or run `pytest` within `src/auv_pkg/test/` directly.  Ensure all tests pass
before submitting changes.

## Hardware Notes

- I²C sensors are on bus 0 by default.  The MS5837 depth sensor and BNO08x IMU
  require stable power.  Nodes should check initialization repeatedly on start.
- Servos are controlled through a PCA9685 running at 60 Hz.  There is currently
  no position feedback; motions rely on timing and the PID nodes to maintain
  orientation.
- The AUV kinematics mimic a manta ray: thrust is produced by synchronized wing
  strokes and tail adjustments.  Use the `ServoMovementCommand` message when
  sending servo actions to keep compatibility with existing nodes.

## Updating Package Lists

Run `./update_package_lists.sh` after adding system or Python dependencies.  The
script regenerates `pip_requirements.txt` and `apt_package_list.txt` and stages
them for commit.

---
Agents must follow these guidelines to keep the AUV software reliable and ready
for deployment on constrained edge hardware.
