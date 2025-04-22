import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
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

        # Lifecycle service client
        self.lifecycle_client = self.create_client(ChangeState, 'servo_driver_node/change_state')

        # Wait for lifecycle service
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

    def run(self):
        self.get_logger().info('Controls: "l" increase roll, "j" decrease roll, "i" decrease pitch, "k" increase pitch, "1-4" for lifecycle transitions.')

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
