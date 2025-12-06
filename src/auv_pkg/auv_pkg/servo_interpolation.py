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
        self.get_logger().info('Initializing Servo Interpolation Node V3 (Optimized)')

        # === Parameters ===
        self.declare_parameter('interpolation_density', 10)
        # CHANGED: Default lowered to 30Hz to ensure Driver (60Hz) is always faster.
        # This prevents "frame dropping" and reduces CPU load significantly.
        self.declare_parameter('update_rate_hz', 30)
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
        self.get_logger().info(f"Interpolation running at {self.update_rate_hz} Hz")

    def current_angles_callback(self, msg):
        """
        Updates the internal knowledge of where the servos are.
        CRITICAL OPTIMIZATION: We only accept updates when we are NOT currently
        interpolating. This prevents the 'delayed reality' of the hardware from
        corrupting the 'perfect math' of the trajectory generator.
        """
        # If the timer is running, we trust our own math (Feed-Forward).
        # If the timer is stopped, we sync with reality (Feedback).
        if self.timer is None and not self.paused:
            # Resize logic to handle dynamic array sizes safely
            incoming_data = np.array(msg.data)
            if len(incoming_data) > len(self.current_angles):
                # Expand internal array if driver reports more servos
                new_angles = np.zeros(len(incoming_data))
                new_angles[:len(self.current_angles)] = self.current_angles
                self.current_angles = new_angles
            
            # Update only indices that exist in incoming message
            limit = min(len(self.current_angles), len(incoming_data))
            self.current_angles[:limit] = incoming_data[:limit]
            
            self.initial_angles_set = True

    def command_callback(self, msg):
        if not self.initial_angles_set:
            # We need at least one update from the driver to know where to start
            return

        if self.hold_active:
            self.get_logger().warn(
                'Hold request active; ignoring new servo interpolation command.'
            )
            return

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

        # Because we ignore driver feedback during motion, this start point
        # is now mathematically consistent with the end of the previous move.
        start_angles = self.current_angles[servo_numbers]
        start_index = 0

        # Handle Interrupt/Transition smoothing
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

        # Process standard segments
        for i in range(start_index, len(durations)):
            end_angles = np.array(target_angles[i * len(servo_numbers):(i + 1) * len(servo_numbers)])
            easing_algorithm = easing_algorithms[i] if i < len(easing_algorithms) else 'linear'
            easing_in_factor = easing_in_factors[i] if i < len(easing_in_factors) else 0.0
            easing_out_factor = easing_out_factors[i] if i < len(easing_out_factors) else 0.0

            num_steps = max(int(durations[i] * self.update_rate_hz), 2)
            steps = self.calculate_interpolation(start_angles, end_angles, durations[i], num_steps, easing_algorithm, easing_in_factor, easing_out_factor, servo_numbers)

            # Cross-fading logic
            cross_fade_steps = min(int(self.cross_fade_factor * num_steps), len(steps), len(self.steps))
            if len(self.steps) > 0 and cross_fade_steps > 0:
                for j in range(cross_fade_steps):
                    blend_factor = (j + 1) / cross_fade_steps
                    # Only blend if the servo mapping matches
                    prev_step_idx = -cross_fade_steps + j
                    if self.steps[prev_step_idx]['servo_numbers'] == servo_numbers:
                        self.steps[prev_step_idx]['angles'] = (
                            (1 - blend_factor) * np.array(self.steps[prev_step_idx]['angles']) +
                            blend_factor * steps[j]['angles']
                        )

            self.steps.extend(steps[cross_fade_steps:])
            start_angles = end_angles

        if self.timer is None and not self.paused:
            self.timer = self.create_timer(1 / self.update_rate_hz, self.publish_next_step)

    def calculate_interpolation(self, start_angles, end_angles, duration, num_steps, easing_algorithm, easing_in_factor, easing_out_factor, servo_numbers):
        steps = []
        # Pre-calculate delta to save CPU inside loop
        delta = end_angles - start_angles
        
        for step in range(num_steps):
            t = step / (num_steps - 1)
            # Apply Easing
            t = self.apply_easing_factors(t, easing_in_factor, easing_out_factor)
            
            if easing_algorithm == 'cubic':
                t = self.cubic_ease_in_out(t)
            elif easing_algorithm == 'exponential':
                t = self.exponential_ease_out(t)

            intermediate_angles = start_angles + (delta * t)
            steps.append({
                'angles': intermediate_angles, 
                'duration': duration / num_steps, 
                'servo_numbers': servo_numbers
            })

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
        # Optimization: We don't need to send easing data for single steps
        command.durations = [step['duration']] * len(step['angles'])
        command.movement_type = self.movement_type
        command.deadline = self.deadline
        command.operational_mode = self.operational_mode
        command.priority = self.priority

        self.publisher.publish(command)
        self.current_step_index += 1
        
        # Update tracked angles mathematically (Feed-Forward)
        # This keeps the continuity perfect for the NEXT step.
        self.current_angles[command.servo_numbers] = step['angles']

    def hold_state_callback(self, msg):
        if msg.hold == self.hold_active:
            return
        self.hold_active = msg.hold
        if self.hold_active:
            self.get_logger().info('Interpolation hold requested; pausing.')
            self.paused = True
            was_running = self.timer is not None or len(self.steps) > 0
            if self.timer:
                self.timer.cancel()
            self.timer = None
            self.steps = []
            self.current_step_index = 0
            if was_running:
                self.publish_end_command()
        else:
            self.get_logger().info('Interpolation hold released.')
            self.paused = False
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
        # Send end command to all known servos to be safe
        command.servo_numbers = list(range(len(self.current_angles)))
        
        # Fill with NaN so driver maintains position
        command.target_angles = [float('nan')] * len(self.current_angles)
        command.durations = [0.1] * len(self.current_angles)
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
