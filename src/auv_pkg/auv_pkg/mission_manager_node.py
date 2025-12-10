#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
import subprocess
import signal
import os
import time

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        # --- CONFIGURATION ---
        # The command to launch your main system
        # Ensure 'mobula_bringup' and 'mobula.launch.xml' are correct
        self.launch_cmd = ["ros2", "launch", "mobula_bringup", "mobula.launch.xml"]
        
        # --- STATE ---
        self.mission_process = None
        
        # --- SERVICES ---
        # 1. Start Mission
        self.create_service(Trigger, 'system/start_mission', self.cb_start)
        # 2. Stop Mission
        self.create_service(Trigger, 'system/stop_mission', self.cb_stop)
        # 3. Check Status (Optional)
        self.create_service(SetBool, 'system/is_running', self.cb_status)

        self.get_logger().info("Mission Manager Online. Ready for GUI commands.")

    def cb_start(self, request, response):
        # Prevent double-launching
        if self.mission_process is not None:
            if self.mission_process.poll() is None:
                response.success = False
                response.message = "Mission is already running!"
                return response
        
        try:
            self.get_logger().info(f"Launching: {' '.join(self.launch_cmd)}")
            
            # --- LAUNCH PROCESS ---
            # We use start_new_session=True so we can kill the whole group later.
            # We do NOT use stdout=PIPE, so logs flow directly to journalctl.
            self.mission_process = subprocess.Popen(
                self.launch_cmd, 
                start_new_session=True,
                env=os.environ.copy() # Pass current ROS environment to child
            )
            
            response.success = True
            response.message = "Mobula Mission Launched"
        except Exception as e:
            self.get_logger().error(f"Failed to launch: {e}")
            response.success = False
            response.message = str(e)
            
        return response

    def cb_stop(self, request, response):
        if self.mission_process is None:
            response.success = False
            response.message = "No mission running."
            return response

        self.get_logger().warn("Stopping Mobula Mission...")
        
        # Kill the entire process group (XML launch + spawned nodes)
        try:
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
        response.success = True
        response.message = "Mission Stopped"
        return response

    def cb_status(self, request, response):
        # Allow GUI to poll if we are active
        is_running = (self.mission_process is not None) and (self.mission_process.poll() is None)
        response.success = is_running
        response.message = "Running" if is_running else "Stopped"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure we don't leave zombie robots if the manager dies
        if node.mission_process:
             try:
                os.killpg(os.getpgid(node.mission_process.pid), signal.SIGTERM)
             except: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()