# Mission Manager Node (`mission_manager_node.py`)

Starts and supervises the ROS 2 mission launch so the vehicle is immediately ready for GUI control, exposing system-level services for mission lifecycle and power actions.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L11-L183】

## Goals and responsibilities
- Bring up `mobula.launch.xml` as a child process, using `start_new_session` so mission nodes run in their own process group that can be cleanly stopped.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L15-L147】
- Provide `/system/start_mission`, `/system/stop_mission`, `/system/is_running`, `/system/shutdown`, and `/system/reboot` services for the GUI to control mission state and initiate power actions.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L21-L183】
- Guard against double-starts and report the current running status to external clients.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L125-L167】

## Startup pathway
- A systemd unit launches the node on boot so it is awake before the GUI connects. Ensure the operator account has sudo/`visudo` permissions for `systemctl` actions and to install/enable the service.
- Recommended service file:
  ```ini
  [Unit]
  Description=Mobula AUV Mission Manager
  After=network-online.target
  Wants=network-online.target

  [Service]
  Type=simple
  User=mobula

  # Ensure correct HOME for Python and ROS tooling
  Environment="HOME=/home/mobula"
  # Force Python to flush logs immediately to journald
  Environment="PYTHONUNBUFFERED=1"

  # IMPORTANT: use a writable working directory so lgpio can create .lgd-nfy* pipes
  WorkingDirectory=/home/mobula/Documents/MobulaRos2

  # Start the mission manager node using the same sequence as your shell
  ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source /home/mobula/Documents/MobulaRos2/install/setup.bash && ros2 run auv_pkg mission_manager_node'

  # Restart Policy: if it crashes, wait 5s and restart it automatically
  Restart=on-failure
  RestartSec=5

  [Install]
  WantedBy=multi-user.target
  ```
- Install the unit (e.g., `/etc/systemd/system/mobula-mission-manager.service`), run `systemctl daemon-reload`, enable it, and verify `systemctl status` succeeds on boot.

## Mission launch behavior
- On `/system/start_mission`, spawns `ros2 launch mobula_bringup mobula.launch.xml` and records the PID for later teardown.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L15-L147】 The launch file staggers node startup with timers to avoid hammering the I2C bus during bring-up (e.g., IMU at 1.5 s, servo and PID controllers between 3–4.5 s, sensors and monitors up to 6.5 s).【F:src/mobula_bringup/launch/mobula.launch.xml†L2-L48】
- `/system/stop_mission` walks the process tree via `psutil`, sends SIGTERM/SIGKILL to any stubborn children, and finally cleans up the process group to prevent orphaned I2C users.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L32-L111】 Use this before powering off to keep the bus unclogged.
- `/system/is_running` reports whether the tracked launch process is alive.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L163-L167】 GUI flows can poll this to disable duplicate start requests.
- `/system/shutdown` and `/system/reboot` stop any running mission before issuing `systemctl poweroff` or `systemctl reboot` through sudo. Validate sudoers entries so these calls are not blocked at runtime.【F:src/auv_pkg/auv_pkg/mission_manager_node.py†L113-L183】

## Expected outcomes and troubleshooting
- After boot, `systemctl status mobula-mission-manager` should show the node running and logs should include "Mission Manager Online". If it is inactive, check sudoers rights for `systemctl` and that ROS environment paths in `ExecStart` match the deployed workspace.
- If mission start fails, confirm the launch file exists and that staggered timers are keeping the I2C bus responsive. Excessive delays or missing nodes indicate either dependency issues or that a prior mission instance was not fully terminated—use `/system/stop_mission` and recheck.
- If shutdown/reboot calls do nothing, verify the service account can invoke `sudo systemctl` without a password prompt and that the unit file is installed under the correct path.
