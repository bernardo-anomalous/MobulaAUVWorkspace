import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from auv_custom_interfaces.msg import ServoMovementCommand, HoldRequest
import numpy as np
import math

class ServoInterpolationNodeV3(Node):
    def __init__(self):
        super().__init__('servo_interpolation_node')
        self.get_logger().info('Initializing Servo Interpolation Node V3')

        # === Parameters ===
        self.declare_parameter('interpolation_density', 10)
        self.declare_parameter('update_rate_hz', 70)
        self.declare_parameter('cross_fade_factor', 0.5)
        self.declare_parameter('interrupt_transition_duration', 0.8)
        self.declare_parameter('interrupt_transition_easing', 'cubic')

        self.interpolation_density = self.get_parameter('interpolation_density').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.cross_fade_factor = self.get_parameter('cross_fade_factor').value
        self.interrupt_transition_duration = self.get_parameter('interrupt_transition_duration').value
        self.interrupt_transition_easing = self.get_parameter('interrupt_transition_easing').value

        # === Publishers and Subscribers ===
        self.publisher = self.create_publisher(ServoMovementCommand, 'servo_driver_commands', 10)

        hold_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.hold_state_publisher = self.create_publisher(HoldRequest, 'wing_hold_request', hold_qos)
        self.create_subscription(HoldRequest, 'wing_hold_request', self.hold_state_callback, hold_qos)
        self.hold_active = False
        self._last_published_hold_state = None

        self.command_subscriber = self.create_subscription(
            ServoMovementCommand,
            'servo_interpolation_commands',
            self.command_callback,
            10)

        self.current_angles_subscriber = self.create_subscription(
            Float32MultiArray,
            'current_servo_angles',
            self.current_angles_callback,
            10)

        # === Timer and State ===
        self.timer = None
        self.steps = []
        self.current_step_index = 0
        # Track all known servo angles; the driver supports six servos
        self.current_angles = np.zeros(6)
        self.movement_type = ''
        self.initial_angles_set = False
        self.last_command_servo_numbers = list(range(6))
        self.operational_mode = 'energy_efficient'
        self.priority = 0
        self.deadline = rclpy.time.Time()
        self.paused = False

        self.publish_hold_state()

    def current_angles_callback(self, msg):
        # Resize the internal array to match the incoming data so we can
        # accommodate all available servos.
        self.current_angles = np.array(msg.data)
        self.initial_angles_set = True
        # self.get_logger().info(f'Received current servo angles: {self.current_angles}')  # Commented out for optimization

    def command_callback(self, msg):
        # self.get_logger().info(f'Received command: servo_numbers={msg.servo_numbers}, target_angles={msg.target_angles}, durations={msg.durations}, movement_type={msg.movement_type}, operational_mode={msg.operational_mode}')  # Commented out for optimization

        if not self.initial_angles_set:
            # self.get_logger().warn('Current servo angles not yet received. Ignoring command.')  # Commented out for optimization
            return

        if self.hold_active:
            self.get_logger().info('Hold request active; queuing new servo interpolation without executing until resume.')

        # Cancel any in-progress interpolation so new instructions take effect
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.steps = []
        self.current_step_index = 0

        servo_numbers = msg.servo_numbers
        self.last_command_servo_numbers = servo_numbers
        target_angles = msg.target_angles
        durations = msg.durations
        easing_algorithms = msg.easing_algorithms
        easing_in_factors = msg.easing_in_factors
        easing_out_factors = msg.easing_out_factors
        self.movement_type = msg.movement_type
        self.operational_mode = msg.operational_mode or 'energy_efficient'
        self.priority = msg.priority
        self.deadline = msg.deadline

        start_angles = self.current_angles[servo_numbers]
        start_index = 0
        if self.interrupt_transition_duration > 0 and len(target_angles) >= len(servo_numbers):
            first_target = np.array(target_angles[:len(servo_numbers)])
            num_steps = max(int(self.interrupt_transition_duration * self.update_rate_hz), 2)
            transition_steps = self.calculate_interpolation(
                start_angles,
                first_target,
                self.interrupt_transition_duration,
                num_steps,
                self.interrupt_transition_easing,
                0.0,
                0.0,
                servo_numbers
            )
            self.steps.extend(transition_steps)
            start_angles = first_target
            start_index = 1

        for i in range(start_index, len(durations)):
            end_angles = np.array(target_angles[i * len(servo_numbers):(i + 1) * len(servo_numbers)])
            easing_algorithm = easing_algorithms[i] if i < len(easing_algorithms) else 'linear'
            easing_in_factor = easing_in_factors[i] if i < len(easing_in_factors) else 0.0
            easing_out_factor = easing_out_factors[i] if i < len(easing_out_factors) else 0.0

            num_steps = max(int(durations[i] * self.update_rate_hz), 2)
            steps = self.calculate_interpolation(start_angles, end_angles, durations[i], num_steps, easing_algorithm, easing_in_factor, easing_out_factor, servo_numbers)

            cross_fade_steps = min(int(self.cross_fade_factor * num_steps), len(steps), len(self.steps))
            if len(self.steps) > 0 and cross_fade_steps > 0:
                for j in range(cross_fade_steps):
                    blend_factor = (j + 1) / cross_fade_steps
                    if self.steps[-cross_fade_steps + j]['servo_numbers'] == servo_numbers:
                        self.steps[-cross_fade_steps + j]['angles'] = (
                            (1 - blend_factor) * np.array(self.steps[-cross_fade_steps + j]['angles']) +
                            blend_factor * steps[j]['angles']
                        )

            self.steps.extend(steps[cross_fade_steps:])
            start_angles = end_angles


        # The previous implementation performed an additional blending step for
        # movements tagged as "thrust_unit". It gradually returned servos in odd
        # positions to the neutral 90° angle. This behaviour produced unwanted
        # jumps at the end of canned messages so it has been removed. Servos now
        # maintain the final target angles provided in the command.

        if self.timer is None and not self.paused:
            self.timer = self.create_timer(1 / self.update_rate_hz, self.publish_next_step)


    def calculate_interpolation(self, start_angles, end_angles, duration, num_steps, easing_algorithm, easing_in_factor, easing_out_factor, servo_numbers):
        steps = []
        for step in range(num_steps):
            t = step / (num_steps - 1)
            t = self.apply_easing_factors(t, easing_in_factor, easing_out_factor)

            if easing_algorithm == 'cubic':
                t = self.cubic_ease_in_out(t)
            elif easing_algorithm == 'exponential':
                t = self.exponential_ease_out(t)

            intermediate_angles = start_angles + (end_angles - start_angles) * t
            steps.append({'angles': intermediate_angles, 'duration': duration / num_steps, 'servo_numbers': servo_numbers})

        return steps

    def apply_easing_factors(self, t, easing_in_factor, easing_out_factor):
        """Smoothly ease progress near the start and end of a segment."""
        if easing_in_factor > 0 and t < easing_in_factor:
            scaled = t / easing_in_factor
            return scaled * scaled * easing_in_factor
        if easing_out_factor > 0 and t > 1 - easing_out_factor:
            scaled = (1 - t) / easing_out_factor
            return 1 - scaled * scaled * easing_out_factor
        return t

    def cubic_ease_in_out(self, t):
        return 4 * t**3 if t < 0.5 else 1 - ((-2 * t + 2) ** 3) / 2

    def exponential_ease_out(self, t):
        return 1 - math.pow(2, -10 * t) if t != 1 else 1

    def publish_next_step(self):
        if self.paused:
            return
        if self.current_step_index >= len(self.steps):
            if self.timer:
                self.timer.cancel()
            self.timer = None
            self.steps = []
            self.current_step_index = 0
            self.publish_end_command()
            return

        step = self.steps[self.current_step_index]

        command = ServoMovementCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.servo_numbers = list(step.get('servo_numbers', range(len(step['angles']))))
        command.target_angles = step['angles'].tolist()
        command.durations = [step['duration']] * len(step['angles'])
        command.easing_algorithms = ['linear'] * len(step['angles'])
        command.easing_in_factors = [0.0] * len(step['angles'])
        command.easing_out_factors = [0.0] * len(step['angles'])
        command.movement_type = self.movement_type
        command.deadline = self.deadline
        command.operational_mode = self.operational_mode
        command.priority = self.priority

        self.publisher.publish(command)
        self.current_step_index += 1
        # Update tracked angles for the addressed servos only
        self.current_angles[command.servo_numbers] = step['angles']

    def hold_state_callback(self, msg):
        if msg.hold == self.hold_active:
            return
        self.hold_active = msg.hold
        if self.hold_active:
            self.get_logger().info('Servo interpolation hold requested; pausing command execution.')
            self.paused = True
            if self.timer:
                self.timer.cancel()
            self.timer = None
        else:
            self.get_logger().info('Servo interpolation hold released; resuming queued command execution.')
            # Resume the existing interpolation sequence so queued steps complete.
            self.paused = False
            if self.current_step_index < len(self.steps) and self.timer is None:
                self.timer = self.create_timer(1 / self.update_rate_hz, self.publish_next_step)
        self.publish_hold_state()

    def publish_hold_state(self):
        if self._last_published_hold_state == self.hold_active:
            return
        msg = HoldRequest()
        msg.hold = self.hold_active
        self.hold_state_publisher.publish(msg)
        self._last_published_hold_state = self.hold_active

    def publish_end_command(self):
        command = ServoMovementCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.servo_numbers = list(range(len(self.current_angles)))
        # Fill the final message with NaN for all servos so the driver keeps the
        # previous angles but still receives the "end" flag via movement_type.
        command.target_angles = [float('nan')] * len(self.current_angles)
        command.durations = [0.1] * len(self.current_angles)
        command.easing_algorithms = ['linear'] * len(self.current_angles)
        command.easing_in_factors = [0.0] * len(self.current_angles)
        command.easing_out_factors = [0.0] * len(self.current_angles)
        command.movement_type = f'{self.movement_type} + end'
        command.deadline = self.deadline
        command.operational_mode = self.operational_mode
        command.priority = self.priority

        self.publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = ServoInterpolationNodeV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
