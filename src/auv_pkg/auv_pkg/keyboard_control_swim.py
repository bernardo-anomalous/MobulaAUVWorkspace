import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from auv_custom_interfaces.msg import ServoMovementCommand
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import sys
import select
import termios
import tty

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.get_logger().info('Initializing Keyboard Control Node')
        
        # Publishers
        self.target_roll_publisher = self.create_publisher(Float32, 'target_roll', 10)
        self.target_pitch_publisher = self.create_publisher(Float32, 'target_pitch', 10)
        self.canned_publisher = self.create_publisher(ServoMovementCommand, 'servo_interpolation_commands', 10)
        self.roll_pid_bool_publisher = self.create_publisher(Bool, 'wing_pid_active', 10)
        
        # Lifecycle service client
        self.lifecycle_client = self.create_client(ChangeState, 'servo_driver_node/change_state')
        while not self.lifecycle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifecycle service...')
            
        # Initialize terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        # Initialize target pitch and roll
        self.target_pitch = 0.0  # Neutral
        self.target_roll = 0.0   # Neutral

    def call_lifecycle_service(self, transition_id):
        """Send a lifecycle state change request."""
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.lifecycle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f'Lifecycle transition {transition_id} succeeded.')
        else:
            self.get_logger().error(f'Lifecycle transition {transition_id} failed or no response.')

    def publish_target_pitch(self):
        """Publish the current target pitch."""
        msg = Float32()
        msg.data = self.target_pitch
        self.target_pitch_publisher.publish(msg)
        self.get_logger().info(f'Published target pitch: {self.target_pitch}')

    def publish_target_roll(self):
        """Publish the current target roll."""
        msg = Float32()
        msg.data = self.target_roll
        self.target_roll_publisher.publish(msg)
        self.get_logger().info(f'Published target roll: {self.target_roll}')
        
    def publish_canned_message(self):
        """Publish a predefined canned servo movement command."""
        canned_commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [60.0, 50.0, 60.0, 110.0,
                              0.0, 50.0, 120.0, 110.0,
                              0.0, 90.0, 120.0, 90.0,
                              0.0, 130.0, 120.0, 30.0,
                              120.0, 110.0, 0.0, 50.0,
                              120.0, 90.0, 0.0, 90.0,
                              120.0, 50.0, 0.0, 110.0,
                              60.0, 50.0, 60.0, 110.0,
                              60.0, 90.0, 60.0, 90.0],
            'durations': [0.2, 2.0, 0.1, 0.1,
                          2.5, 0.01, 0.1, 2.0,
                          2.0],
            'easing_algorithms': ['exponential', 'cubic', 'cubic', 'exponential', 
                                  'cubic', 'cubic', 'exponential', 'cubic', 'exponential'],
            'easing_in_factors': [0.0] * 9,
            'easing_out_factors': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1],
            'movement_type': 'thrust_unit',
            'deadline': (self.get_clock().now() + rclpy.duration.Duration(seconds=5)).to_msg(),
            'operational_mode': 'energy_efficient',
            'priority': 0
        }

        msg = ServoMovementCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.servo_numbers = canned_commands['servo_numbers']
        msg.target_angles = canned_commands['target_angles']
        msg.durations = canned_commands['durations']
        msg.easing_algorithms = canned_commands['easing_algorithms']
        msg.easing_in_factors = canned_commands['easing_in_factors']
        msg.easing_out_factors = canned_commands['easing_out_factors']
        msg.movement_type = canned_commands['movement_type']
        msg.deadline = canned_commands['deadline']
        msg.operational_mode = canned_commands['operational_mode']
        msg.priority = canned_commands['priority']
        
        self.canned_publisher.publish(msg)
        self.get_logger().info("Published canned message.")
        
    def activate_roll_pid(self):
        """Publish True to activate the roll PID node."""
        msg = Bool()
        msg.data = True
        self.roll_pid_bool_publisher.publish(msg)
        self.get_logger().info("Activated roll PID.")

    def deactivate_roll_pid(self):
        """Publish False to deactivate the roll PID node."""
        msg = Bool()
        msg.data = False
        self.roll_pid_bool_publisher.publish(msg)
        self.get_logger().info("Deactivated roll PID.")
        
    def run(self):
        self.get_logger().info('Controls: "l" increase roll, "j" decrease roll, "i" decrease pitch, "k" increase pitch, "e" for canned message, "w" to deactivate roll PID, "r" to activate roll PID, "1-4" for lifecycle transitions.')
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                    key = sys.stdin.read(1)
                    if key == 'l':  # Increase roll
                        self.target_roll += 5
                        self.publish_target_roll()
                    elif key == 'j':  # Decrease roll
                        self.target_roll -= 5
                        self.publish_target_roll()
                    elif key == 'i':  # Decrease pitch
                        self.target_pitch -= 5
                        self.publish_target_pitch()
                    elif key == 'k':  # Increase pitch
                        self.target_pitch += 5
                        self.publish_target_pitch()
                    elif key == 'e':  # Send canned message
                        self.publish_canned_message()
                    elif key == 'w':  # Deactivate roll PID
                        self.deactivate_roll_pid()
                    elif key == 'r':  # Activate roll PID
                        self.activate_roll_pid()
                    elif key == '1':
                        self.call_lifecycle_service(Transition.TRANSITION_CONFIGURE)
                    elif key == '2':
                        self.call_lifecycle_service(Transition.TRANSITION_ACTIVATE)
                    elif key == '3':
                        self.call_lifecycle_service(Transition.TRANSITION_DEACTIVATE)
                    elif key == '4':
                        self.call_lifecycle_service(Transition.TRANSITION_CLEANUP)
                    elif key == '\x03':  # Ctrl+C to quit
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
