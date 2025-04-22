import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray, String
from auv_custom_interfaces.msg import ServoMovementCommand
from adafruit_motor.servo import Servo
from adafruit_pca9685 import PCA9685
import board

class ServoDriverNode(LifecycleNode):
    def __init__(self):
        super().__init__('servo_driver_node')
        self.get_logger().info('Servo Driver Node initialized.')

        # Declare ROS2 parameters for configuring the node
        self.declare_parameter('glide_position', [90.0, 90.0, 90.0, 90.0, 90.0, 90.0])  # Default glide position for all servos
        self.declare_parameter('update_rate_hz', 10.0)  # Update rate for publishing servo positions
        self.declare_parameter('simulation_mode', False)  # Simulation mode toggle

        # Initialize hardware and software resources
        self.simulation_mode = False
        self.pca = None  # PCA9685 controller instance
        self.servos = {}  # Dictionary to map servo IDs to Servo objects
        self.current_angles = []  # List to store current angles of the servos

        # Publishers and subscribers will be initialized during configuration
        self.angles_publisher = None
        self.command_subscriber = None
        self.tail_command_subscriber = None  # Dedicated subscription for tail commands

        # Timer for periodic tasks like heartbeats
        self.heartbeat_timer = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Servo Driver Node...')
        try:
            # Check simulation mode
            self.simulation_mode = self.get_parameter('simulation_mode').value
            if self.simulation_mode:
                self.get_logger().info('Simulation mode enabled. No hardware will be initialized.')
            else:
                self.get_logger().info('Initializing hardware...')
                # Initialize I2C communication and PCA9685 board
                i2c = board.I2C()
                self.pca = PCA9685(i2c)
                self.pca.frequency = 60  # Set PWM frequency for servos

                # Initialize servos with default glide positions
                self.servos = {
                    0: Servo(self.pca.channels[0], min_pulse=400, max_pulse=2500),
                    1: Servo(self.pca.channels[1], min_pulse=400, max_pulse=2500),
                    2: Servo(self.pca.channels[2], min_pulse=400, max_pulse=2500),
                    3: Servo(self.pca.channels[3], min_pulse=400, max_pulse=2500),
                    4: Servo(self.pca.channels[4], min_pulse=500, max_pulse=2500),  # Left tail
                    5: Servo(self.pca.channels[5], min_pulse=500, max_pulse=2500),  # Right tail
                }

                # Set default positions (glide position)
                glide_position = self.get_parameter('glide_position').value
                for i, angle in enumerate(glide_position):
                    self.servos[i].angle = angle

            # Initialize current angles
            self.current_angles = [90.0] * 6
            glide_position = self.get_parameter('glide_position').value
            for i, angle in enumerate(glide_position):
                self.current_angles[i] = angle

            # Create a publisher for current servo angles
            self.angles_publisher = self.create_publisher(Float32MultiArray, 'current_servo_angles', 10)

            # Create subscribers for movement commands
            self.command_subscriber = self.create_subscription(
                ServoMovementCommand,
                'servo_driver_commands',
                self.command_callback,
                10
            )

            self.tail_command_subscriber = self.create_subscription(
                ServoMovementCommand,
                'tail_commands',
                self.tail_command_callback,
                10
            )

            self.get_logger().info('Configuration successful.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            # Log any errors encountered during configuration
            self.get_logger().error(f'Failed to configure hardware: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Servo Driver Node...')
        # Start a timer to periodically publish heartbeat messages
        self.heartbeat_timer = self.create_timer(1.0 / self.get_parameter('update_rate_hz').value, self.publish_heartbeat)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating Servo Driver Node...')
        # Stop the heartbeat timer if it is running
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up Servo Driver Node...')
        # Deinitialize the PCA9685 controller to release resources if not in simulation mode
        if not self.simulation_mode and self.pca:
            self.pca.deinit()
            self.pca = None
        return TransitionCallbackReturn.SUCCESS

    def command_callback(self, msg: ServoMovementCommand):
        self.get_logger().info(f'Received wing command: {msg}')
        try:
            # Iterate through the received servo numbers and update their positions
            for i, servo_number in enumerate(msg.servo_numbers):
                if servo_number < len(self.current_angles):
                    target_angle = msg.target_angles[i]
                    if not self.simulation_mode and servo_number in self.servos:
                        self.servos[servo_number].angle = target_angle
                    self.current_angles[servo_number] = target_angle

            # Publish updated angles after executing the command
            self.publish_current_servo_angles()
        except Exception as e:
            # Log errors encountered during command execution
            self.get_logger().error(f'Failed to execute wing command: {e}')

    def tail_command_callback(self, msg: ServoMovementCommand):
        self.get_logger().info(f'Received tail command: {msg}')
        try:
            # Iterate through the received servo numbers and update their positions
            for i, servo_number in enumerate(msg.servo_numbers):
                if servo_number < len(self.current_angles):
                    target_angle = msg.target_angles[i]
                    if not self.simulation_mode and servo_number in self.servos:
                        self.servos[servo_number].angle = target_angle
                    self.current_angles[servo_number] = target_angle

            # Publish updated angles after executing the command
            self.publish_current_servo_angles()
        except Exception as e:
            # Log errors encountered during command execution
            self.get_logger().error(f'Failed to execute tail command: {e}')

    def publish_current_servo_angles(self):
        # Create and populate the message with current servo angles
        angles_msg = Float32MultiArray()
        angles_msg.data = self.current_angles
        self.angles_publisher.publish(angles_msg)

    def publish_heartbeat(self):
        # Log a simple heartbeat message to indicate the node is active
        self.get_logger().info('Servo Driver Node is alive.')


def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():  # Ensure shutdown is only called if the context is active
            rclpy.shutdown()  # Shut down ROS2 safely


if __name__ == '__main__':
    main()
