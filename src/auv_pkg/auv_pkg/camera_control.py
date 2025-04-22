import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from auv_custom_interfaces.msg import ServoMovementCommand  # Replace with your actual custom message
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time

class AUVHandPoseControlNode(Node):
    def __init__(self):
        super().__init__('auv_hand_pose_control_node')
        self.publisher_ = self.create_publisher(ServoMovementCommand, 'servo_driver_commands', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'current_servo_angles', self.current_servo_angles_callback, 10)
        self.current_servo_angles = [90.0] * 6  # Default initial angles, assuming neutral position
        self.target_servo_angles = [90.0] * 4  # Initialize target angles
        self.initial_left_shoulder = None
        self.initial_right_shoulder = None
        self.calibrated = False
        
        # Initialize OpenCV and MediaPipe Pose and Hands
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('/dev/video14')  # Assuming webcam index 0
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, model_complexity=1, smooth_landmarks=True, enable_segmentation=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        
        # Start processing loop
        self.timer = self.create_timer(0.1, self.process_frame)  # Run at 10 Hz

    def current_servo_angles_callback(self, msg):
        # Update current servo angles
        self.current_servo_angles = msg.data[:4]  # Get only the relevant servos

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame from camera')
            return
        
        # Convert frame to RGB for MediaPipe
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(frame_rgb)
        hand_results = self.hands.process(frame_rgb)

        if pose_results.pose_landmarks:
            landmarks = pose_results.pose_landmarks.landmark

            if not self.calibrated:
                self.calibrate_position(landmarks, frame)
            else:
                self.handle_pose(landmarks)

            # Draw pose landmarks on the frame
            mp.solutions.drawing_utils.draw_landmarks(frame, pose_results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

        if hand_results.multi_hand_landmarks:
            for hand_landmarks in hand_results.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                self.handle_hand_pose(hand_landmarks, hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x)

            # Display the servo angles on the frame
            cv2.putText(frame, f'Left Main: {self.target_servo_angles[0]:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f'Left Pitch: {self.target_servo_angles[1]:.2f}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f'Right Main: {self.target_servo_angles[2]:.2f}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f'Right Pitch: {self.target_servo_angles[3]:.2f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow('AUV Hand Pose Control', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()

    def calibrate_position(self, landmarks, frame):
        # Countdown for calibration
        cv2.putText(frame, 'Calibrating... Please hold your arms in gliding position', (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        for i in range(5, 0, -1):
            cv2.putText(frame, f'Calibration in: {i}', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow('AUV Hand Pose Control', frame)
            cv2.waitKey(1000)  # Wait for 1 second
        
        # Set the initial positions as the gliding position reference
        self.initial_left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        self.initial_right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        self.calibrated = True
        self.get_logger().info('Calibration complete. Arms gliding position set.')

    def handle_pose(self, landmarks):
        # Extract shoulder and wrist positions
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]

        # Calculate angles for main servos (arms raised/lowered)
        left_arm_angle = self.calculate_arm_angle(self.initial_left_shoulder, left_shoulder, left_wrist)
        right_arm_angle = self.calculate_arm_angle(self.initial_right_shoulder, right_shoulder, right_wrist)

        # Map arm angles to servo target angles (opposite directions)
        left_main_target = np.clip(60 - left_arm_angle, 0, 120)  # Left arm up decreases angle
        right_main_target = np.clip(60 + right_arm_angle, 0, 120)  # Right arm up increases angle

        # Update target servo angles for main servos
        self.target_servo_angles[0] = left_main_target
        self.target_servo_angles[2] = right_main_target

    def handle_hand_pose(self, hand_landmarks, wrist_x_position):
        # Extract landmarks for pitch control
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        thumb_cmc = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_CMC]
        middle_finger_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

        # Calculate the vector from thumb to middle finger
        thumb_to_middle = np.array([middle_finger_mcp.x - thumb_cmc.x, middle_finger_mcp.y - thumb_cmc.y, middle_finger_mcp.z - thumb_cmc.z])
        wrist_to_thumb = np.array([thumb_cmc.x - wrist.x, thumb_cmc.y - wrist.y, thumb_cmc.z - wrist.z])

        # Calculate the angle between the thumb and middle finger
        dot_product = np.dot(thumb_to_middle, wrist_to_thumb)
        magnitude_product = np.linalg.norm(thumb_to_middle) * np.linalg.norm(wrist_to_thumb)
        angle_radians = np.arccos(np.clip(dot_product / (magnitude_product + 1e-6), -1.0, 1.0))  # Add small value to avoid division by zero and clip for safety
        angle_degrees = np.degrees(angle_radians)

        # Set the neutral pitch angle to 90 degrees (thumb and middle finger close to each other)
        neutral_pitch_angle = 90.0

        # Determine which hand is being tracked based on the wrist's x position
        if wrist_x_position > 0.5:  # Assuming left hand
            # Left hand logic
            if thumb_cmc.y < wrist.y:  # Thumb pointing up
                left_pitch_angle = neutral_pitch_angle - angle_degrees  # Decrease left pitch
            else:  # Thumb pointing down
                left_pitch_angle = neutral_pitch_angle + angle_degrees  # Increase left pitch

            # Clip pitch angles to stay within bounds (45 to 135 degrees)
            left_pitch_angle = np.clip(left_pitch_angle, 45, 135)

            # Gradual adjustment for pitch angles to achieve smooth transition
            self.target_servo_angles[1] = 0.7 * self.target_servo_angles[1] + 0.3 * left_pitch_angle

        elif wrist_x_position < 0.5:  # Assuming right hand
            # Right hand logic
            if thumb_cmc.y < wrist.y:  # Thumb pointing up
                right_pitch_angle = neutral_pitch_angle - angle_degrees  # Decrease right pitch
            else:  # Thumb pointing down
                right_pitch_angle = neutral_pitch_angle + angle_degrees  # Increase right pitch

            # Clip pitch angles to stay within bounds (45 to 135 degrees)
            right_pitch_angle = np.clip(right_pitch_angle, 45, 135)

            # Gradual adjustment for pitch angles to achieve smooth transition
            self.target_servo_angles[3] = 0.7 * self.target_servo_angles[3] + 0.3 * right_pitch_angle

        # Create and publish servo movement command
        command = ServoMovementCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.servo_numbers = [0, 1, 2, 3]
        command.target_angles = self.target_servo_angles
        command.durations = [0.5, 0.5, 0.5, 0.5]  # Example duration
        command.easing_algorithms = ['linear', 'linear', 'linear', 'linear']
        command.movement_type = 'manual_control'
        command.deadline = self.get_clock().now().to_msg()  # No specific deadline
        command.operational_mode = 'active_maneuvering'
        command.priority = 0

        self.publisher_.publish(command)

    def calculate_arm_angle(self, initial_shoulder, shoulder, wrist):
        # Calculate the angle of the arm relative to the initial gliding position
        delta_y = wrist.y - initial_shoulder.y
        return -delta_y * 180  # Simplified mapping for demonstration


def main(args=None):
    rclpy.init(args=args)
    node = AUVHandPoseControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
