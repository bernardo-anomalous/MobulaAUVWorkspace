#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import subprocess
import signal
import os
import psutil  # NEW


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
        """Stop the running mission process and all its children. Returns True if anything was stopped."""
        if self.mission_process is None:
            return False

        # If already gone, clear and return
        if self.mission_process.poll() is not None:
            self.mission_process = None
            return False

        pid = self.mission_process.pid
        self.get_logger().warn(f"Stopping Mobula Mission (PID {pid})...")

        # Try psutil-based tree kill first
        try:
            parent = psutil.Process(pid)
            children = parent.children(recursive=True)
            self.get_logger().info(
                f"Found {len(children)} child processes under mission PID {pid}; terminating all."
            )

            # Send SIGTERM to children
            for proc in children:
                try:
                    self.get_logger().debug(f"Terminating child PID {proc.pid} ({proc.name()})")
                    proc.terminate()
                except Exception as e:
                    self.get_logger().warn(f"Error terminating child PID {proc.pid}: {e}")

            # Terminate parent
            try:
                self.get_logger().debug(f"Terminating mission parent PID {parent.pid} ({parent.name()})")
                parent.terminate()
            except Exception as e:
                self.get_logger().warn(f"Error terminating mission parent PID {parent.pid}: {e}")

            # Wait up to 5 seconds
            procs = [parent] + children
            gone, alive = psutil.wait_procs(procs, timeout=5.0)

            if alive:
                self.get_logger().warn(
                    f"{len(alive)} processes still alive after SIGTERM; sending SIGKILL."
                )
                for proc in alive:
                    try:
                        self.get_logger().debug(f"Killing PID {proc.pid} ({proc.name()})")
                        proc.kill()
                    except Exception as e:
                        self.get_logger().warn(f"Error killing PID {proc.pid}: {e}")
                psutil.wait_procs(alive, timeout=5.0)
        except psutil.NoSuchProcess:
            self.get_logger().warn("Mission parent process disappeared before we could terminate it.")
        except Exception as e:
            self.get_logger().warn(f"psutil-based termination failed: {e}")

        # Belt & suspenders: also try killpg on the original process group
        try:
            pgid = os.getpgid(pid)
            self.get_logger().info(f"Sending SIGTERM to process group {pgid}.")
            os.killpg(pgid, signal.SIGTERM)
        except Exception as e:
            self.get_logger().debug(f"killpg(SIGTERM) failed or not needed: {e}")

        try:
            self.mission_process.wait(timeout=2.0)
        except Exception:
            pass

        # Final KILL on group if absolutely necessary
        try:
            pgid = os.getpgid(pid)
            self.get_logger().info(f"Sending SIGKILL to process group {pgid}.")
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            # If pgid no longer exists, that's fine.
            pass

        self.mission_process = None
        return True

    def _systemctl_action(self, action: str) -> bool:
        """Fire-and-forget systemctl action (poweroff or reboot)."""
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
                env=os.environ.copy(),
            )

            self.get_logger().info(f"Mission launch started with PID {self.mission_process.pid}")
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
        self._stop_mission_process()
        ok = self._systemctl_action("poweroff")
        response.success = ok
        response.message = "System shutdown initiated." if ok else "Failed to initiate shutdown."
        return response

    def cb_reboot(self, request, response):
        self.get_logger().warn("Reboot requested via /system/reboot")
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
        if node.mission_process:
            try:
                os.killpg(os.getpgid(node.mission_process.pid), signal.SIGTERM)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
