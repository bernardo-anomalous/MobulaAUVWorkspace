#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import subprocess
import signal
import os


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        # --- CONFIGURATION ---
        self.launch_cmd = ["ros2", "launch", "mobula_bringup", "mobula.launch.xml"]

        # --- STATE ---
        self.mission_process = None

        # --- SERVICES ---
        self.create_service(Trigger, 'system/start_mission', self.cb_start)
        self.create_service(Trigger, 'system/stop_mission', self.cb_stop)
        self.create_service(SetBool, 'system/is_running', self.cb_status)
        self.create_service(Trigger, 'system/shutdown', self.cb_shutdown)
        self.create_service(Trigger, 'system/reboot', self.cb_reboot)

        self.get_logger().info("Mission Manager Online. Ready for GUI commands.")

    # --- internal helpers ---

    def _stop_mission_process(self) -> bool:
        """Stop the running mission process, if any. Returns True if something was stopped."""
        if self.mission_process is None:
            return False

        # If process already exited, just clear state
        if self.mission_process.poll() is not None:
            self.mission_process = None
            return False

        self.get_logger().warn("Stopping Mobula Mission...")

        try:
            # Kill the entire process group (launch + nodes)
            os.killpg(os.getpgid(self.mission_process.pid), signal.SIGTERM)

            # Wait up to 5 seconds for graceful shutdown
            try:
                self.mission_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Process refused TERM, forcing KILL.")
                os.killpg(os.getpgid(self.mission_process.pid), signal.SIGKILL)
                self.mission_process.wait()
        except Exception as e:
            self.get_logger().warn(f"Error during shutdown: {e}")

        self.mission_process = None
        return True

    def _systemctl_action(self, action: str) -> bool:
        """
        Fire-and-forget systemctl action (poweroff or reboot).
        Returns True if the subprocess started successfully.
        """
        try:
            self.get_logger().warn(f"Initiating system {action} via systemctl...")
            subprocess.Popen(["sudo", "systemctl", action])
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to initiate {action}: {e}")
            return False

    # --- service callbacks ---

    def cb_start(self, request, response):
        # Prevent double-launching
        if self.mission_process is not None and self.mission_process.poll() is None:
            response.success = False
            response.message = "Mission is already running!"
            return response

        try:
            self.get_logger().info(f"Launching: {' '.join(self.launch_cmd)}")

            self.mission_process = subprocess.Popen(
                self.launch_cmd,
                start_new_session=True,
                env=os.environ.copy(),  # Pass current ROS environment to child
            )

            response.success = True
            response.message = "Mobula Mission Launched"
        except Exception as e:
            self.get_logger().error(f"Failed to launch: {e}")
            response.success = False
            response.message = str(e)

        return response

    def cb_stop(self, request, response):
        stopped = self._stop_mission_process()

        if not stopped:
            response.success = False
            response.message = "No mission running."
        else:
            response.success = True
            response.message = "Mission Stopped"

        return response

    def cb_status(self, request, response):
        is_running = (self.mission_process is not None) and (self.mission_process.poll() is None)
        response.success = is_running
        response.message = "Running" if is_running else "Stopped"
        return response

    def cb_shutdown(self, request, response):
        self.get_logger().warn("Shutdown requested via /system/shutdown")

        # Stop mission if running
        self._stop_mission_process()

        ok = self._systemctl_action("poweroff")
        response.success = ok
        response.message = "System shutdown initiated." if ok else "Failed to initiate shutdown."
        return response

    def cb_reboot(self, request, response):
        self.get_logger().warn("Reboot requested via /system/reboot")

        # Stop mission if running
        self._stop_mission_process()

        ok = self._systemctl_action("reboot")
        response.success = ok
        response.message = "System reboot initiated." if ok else "Failed to initiate reboot."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure we don't leave zombie launches if the manager dies
        if node.mission_process:
            try:
                os.killpg(os.getpgid(node.mission_process.pid), signal.SIGTERM)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
