import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from std_msgs.msg import Float32MultiArray, String
from auv_custom_interfaces.msg import ServoMovementCommand
from adafruit_motor.servo import Servo
from adafruit_pca9685 import PCA9685
import board

class ServoDriverNode(LifecycleNode):
    def __init__(self):
        super().__init__('servo_driver_node')
        self.get_logger().info('Servo Driver Node initialized.')

        # Parameters
        self.declare_parameter('glide_position', [60.0, 90.0, 60.0, 90.0, 90.0, 90.0])
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('debug_logging', False)  # New parameter to toggle detailed logging

        self.simulation_mode = False
        self.debug_logging = False
        self.pca = None
        self.servos = {}
        self.current_angles = []

        self.angles_publisher = None
        self.command_subscriber = None
        self.tail_command_subscriber = None
        self.heartbeat_timer = None
        self.busy_status_publisher = self.create_publisher(String, 'servo_driver_status', 10)
        self.last_published_status = None  # Track last status string
        self.last_published_status = None
        self.executing_movement = False  # Flag for whether we're in a movement sequence




    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Servo Driver Node...')
        try:
            self.simulation_mode = self.get_parameter('simulation_mode').value
            self.debug_logging = self.get_parameter('debug_logging').value

            if self.simulation_mode:
                self.get_logger().info('Simulation mode enabled. No hardware will be initialized.')
            else:
                self.get_logger().info('Initializing hardware...')
                i2c = board.I2C()
                self.pca = PCA9685(i2c)
                self.pca.frequency = 60
                self.servos = {
                    0: Servo(self.pca.channels[0], min_pulse=400, max_pulse=2500),
                    1: Servo(self.pca.channels[1], min_pulse=400, max_pulse=2500),
                    2: Servo(self.pca.channels[2], min_pulse=400, max_pulse=2500),
                    3: Servo(self.pca.channels[3], min_pulse=400, max_pulse=2500),
                    4: Servo(self.pca.channels[4], min_pulse=500, max_pulse=2500),
                    5: Servo(self.pca.channels[5], min_pulse=500, max_pulse=2500),
                }
                glide_position = self.get_parameter('glide_position').value
                for i, angle in enumerate(glide_position):
                    self.servos[i].angle = angle

            self.current_angles = self.get_parameter('glide_position').value

            self.angles_publisher = self.create_publisher(Float32MultiArray, 'current_servo_angles', 10)
            self.command_subscriber = self.create_subscription(
                ServoMovementCommand, 'servo_driver_commands', self._servo_command_callback, 10
            )
            self.tail_command_subscriber = self.create_subscription(
                ServoMovementCommand, 'tail_commands', self._servo_command_callback, 10
            )

            self.get_logger().info('Configuration successful.')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to configure hardware: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Servo Driver Node...')
        self.heartbeat_timer = self.create_timer(
            1.0 / self.get_parameter('update_rate_hz').value, self._publish_heartbeat
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating Servo Driver Node... Attempting to limp servos.')
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        if not self.simulation_mode and self.pca:
            try:
                for channel in self.pca.channels:
                    channel.duty_cycle = 0
                self.get_logger().info('PWM signals stopped. Servos should be limp (if supported by model).')
            except Exception as e:
                self.get_logger().error(f'Error while setting duty cycles to 0: {e}')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up Servo Driver Node...')
        if not self.simulation_mode and self.pca:
            self.pca.deinit()
            self.pca = None
        return TransitionCallbackReturn.SUCCESS

    def _servo_command_callback(self, msg: ServoMovementCommand):
        if self.debug_logging:
            self.get_logger().info(f'Received command: {msg}')

        try:
            movement_type = msg.movement_type.lower() if msg.movement_type else ""

            if 'end' in movement_type:
                self.executing_movement = False
                self._publish_busy_status("nominal: roll and pitch pid engaged")
            elif movement_type != 'pid_control':
                if not self.executing_movement:
                    # Only announce busy when we start a new sequence
                    self.executing_movement = True
                    self._publish_busy_status(f"busy: executing {msg.movement_type}")
                # Stay busy — do not publish again if already in executing state
            else:
                if not self.executing_movement:
                    self._publish_busy_status("nominal: roll and pitch pid engaged")

            # === NORMAL SERVO EXECUTION ===
            if not self.simulation_mode:
                for i, servo_number in enumerate(msg.servo_numbers):
                    if servo_number < len(self.current_angles) and servo_number in self.servos:
                        self.servos[servo_number].angle = msg.target_angles[i]
                    self.current_angles[servo_number] = msg.target_angles[i]
            else:
                for i, servo_number in enumerate(msg.servo_numbers):
                    self.current_angles[servo_number] = msg.target_angles[i]

            self._publish_current_servo_angles()

        except Exception as e:
            self.get_logger().error(f'Failed to execute command: {e}')




    def _publish_current_servo_angles(self):
        angles_msg = Float32MultiArray()
        angles_msg.data = self.current_angles
        self.angles_publisher.publish(angles_msg)

    def _publish_heartbeat(self):
        heartbeat_msg = String()
        heartbeat_msg.data = 'alive'
        self.angles_publisher.publish(heartbeat_msg)  # Optionally use a separate heartbeat topic
        
    def _publish_busy_status(self, status_message: str):
        if status_message != self.last_published_status:
            status_msg = String()
            status_msg.data = status_message
            self.busy_status_publisher.publish(status_msg)
            self.get_logger().info(f"[ServoDriver Status] Published: {status_msg.data}")
            self.last_published_status = status_message  # Update last status





def main(args=None):
    rclpy.init(args=args)
    node = ServoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
