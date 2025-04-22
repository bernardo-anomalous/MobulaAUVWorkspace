import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from auv_custom_interfaces.msg import ServoMovementCommand
from lifecycle_msgs.srv import ChangeState
import sys
import select
import termios
import tty

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.get_logger().info('Initializing Keyboard Control Node')

        # Publishers
        self.manual_publisher = self.create_publisher(ServoMovementCommand, 'servo_driver_commands', 10)
        self.canned_publisher = self.create_publisher(ServoMovementCommand, 'servo_interpolation_commands', 10)

        # Lifecycle service client
        self.lifecycle_client = self.create_client(ChangeState, 'servo_driver_node/change_state')

        # Subscriber to current servo angles
        self.current_angles_subscriber = self.create_subscription(
            Float32MultiArray, 'current_servo_angles', self.current_angles_callback, 10
        )

        # Wait for lifecycle service
        while not self.lifecycle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for lifecycle service...')

        # Initialize terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Servo angle storage
        self.current_angles = [90.0] * 6
        self.last_angles = [90.0] * 6  # Initialize with defaults until updated by `current_servo_angles`

    def current_angles_callback(self, msg):
        """Update the current servo angles."""
        self.current_angles = msg.data
        if self.last_angles == [90.0] * 6:
            self.last_angles = msg.data

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

    def publish_roll_command(self, roll_adjustment):
        """Adjust roll by modifying the pitch servos."""
        left_pitch = self.last_angles[1] + roll_adjustment
        right_pitch = self.last_angles[3] + roll_adjustment

        left_pitch = max(0, min(120, left_pitch))
        right_pitch = max(0, min(120, right_pitch))

        msg = ServoMovementCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.servo_numbers = [1, 3]
        msg.target_angles = [left_pitch, right_pitch]
        msg.durations = [1.0, 1.0]
        self.manual_publisher.publish(msg)

        self.last_angles[1] = left_pitch
        self.last_angles[3] = right_pitch

    def publish_pitch_command(self, pitch_adjustment):
        """Adjust pitch by modifying the tail servos."""
        tail_4 = self.last_angles[4] - pitch_adjustment
        tail_5 = self.last_angles[5] + pitch_adjustment

        tail_4 = max(0, min(120, tail_4))
        tail_5 = max(0, min(120, tail_5))

        msg = ServoMovementCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.servo_numbers = [4, 5]
        msg.target_angles = [tail_4, tail_5]
        msg.durations = [1.0, 1.0]
        self.manual_publisher.publish(msg)

        self.last_angles[4] = tail_4
        self.last_angles[5] = tail_5

    def publish_canned_message(self):
        """Publish a predefined canned movement message."""
        canned_commands = {
            'servo_numbers': [0, 1, 2, 3],
            'target_angles': [
                60.0, 50.0, 60.0, 110.0,
                0.0, 50.0, 120.0, 110.0,
                0.0, 90.0, 120.0, 90.0,
                0.0, 130.0, 120.0, 30.0,
                120.0, 110.0, 0.0, 50.0,
                120.0, 90.0, 0.0, 90.0,
                120.0, 50.0, 0.0, 110.0,
                60.0, 50.0, 60.0, 110.0,
                60.0, 90.0, 60.0, 90.0
            ],
            'durations': [
                0.2, 2.0, 0.1, 0.1,
                2.5, 0.01, 0.1, 2.0,
                2.0
            ],
            'easing_algorithms': [
                'exponential', 'cubic', 'cubic', 'exponential', 
                'cubic', 'cubic', 'exponential', 'cubic', 'exponential'
            ],
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
        self.get_logger().info(f'Published canned message.')

    def run(self):
        self.get_logger().info('Controls: "l"/"j" for roll, "i"/"k" for pitch, "c" for canned message.')

        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                    key = sys.stdin.read(1)
                    if key == 'c':
                        self.publish_canned_message()
                    elif key == 'l':
                        self.publish_roll_command(-10)
                    elif key == 'j':
                        self.publish_roll_command(10)
                    elif key == 'i':
                        self.publish_pitch_command(10)
                    elif key == 'k':
                        self.publish_pitch_command(-10)
                    elif key == '\x03':  # Ctrl+C
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
