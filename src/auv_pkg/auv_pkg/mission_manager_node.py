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
        # UPDATED: Pointing to your specific package and XML launch file
        self.launch_cmd = ["ros2", "launch", "mobula_bringup", "mobula.launch.xml"]
        
        # --- STATE ---
        self.mission_process = None
        
        # --- SERVICES ---
        self.create_service(Trigger, 'system/start_mission', self.cb_start)
        self.create_service(Trigger, 'system/stop_mission', self.cb_stop)
        self.create_service(SetBool, 'system/is_running', self.cb_status)

        self.get_logger().info("Mission Manager Online. Ready for GUI commands.")

    def cb_start(self, request, response):
        if self.mission_process is not None:
            if self.mission_process.poll() is None:
                response.success = False
                response.message = "Mission is already running!"
                return response
        
        try:
            self.get_logger().info(f"Launching: {' '.join(self.launch_cmd)}")
            
            # Start the launch file in a new session (Process Group)
            self.mission_process = subprocess.Popen(
                self.launch_cmd, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                start_new_session=True,
                env=os.environ.copy() # Pass current environment (ROS paths) to child
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
            self.mission_process.wait(timeout=5.0)
        except Exception as e:
            self.get_logger().warn(f"Forced Kill required: {e}")
            try:
                os.killpg(os.getpgid(self.mission_process.pid), signal.SIGKILL)
            except: pass

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
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()