import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class EnhancedTrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        
        # ROS2 publishers and subscribers setup (same as before)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.sign_publisher = self.create_publisher(String, '/detected_sign', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.line_follower_restart_publisher = self.create_publisher(
            String, 
            '/line_follower_restart', 
            10
        )
        
        self.bridge = CvBridge()
        
        # Initialize SIFT detector
        self.sift = cv2.SIFT_create()
        
        # FLANN matcher parameters
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        # Load reference signs and compute their features
        self.sign_features = self.load_sign_features()
        
        # State management
        self.is_blocked = False  # Controls whether detection is allowed
        self.stop_timer = None
        self.cooldown_timer = None
        
    def load_sign_features(self):
        """Load reference signs and compute their SIFT features."""
        sign_features = {}
        sign_paths = {
            'STOP': '/home/david/Desktop/autobot_facultate/my_ros2_ws/src/turtlebot3_autobot/turtlebot3_autobot/images/stop_sign_bun.jpeg',
            'PARKING': '/home/david/Desktop/autobot_facultate/my_ros2_ws/src/turtlebot3_autobot/turtlebot3_autobot/images/parking_photo.jpeg'
        }
        
        for sign_type, path in sign_paths.items():
            try:
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    self.get_logger().error(f"Failed to load {sign_type} sign template!")
                    continue
                    
                keypoints, descriptors = self.sift.detectAndCompute(img, None)
                sign_features[sign_type] = {
                    'image': img,
                    'keypoints': keypoints,
                    'descriptors': descriptors
                }
                self.get_logger().info(f"Loaded {sign_type} sign template successfully")
                
            except Exception as e:
                self.get_logger().error(f"Error loading {sign_type} sign: {str(e)}")
                
        return sign_features

    def detect_sign(self, frame):
        """Detect traffic signs in the frame using SIFT feature matching."""
        # Your existing detect_sign implementation (unchanged)
        # ... (keep your current implementation)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_keypoints, frame_descriptors = self.sift.detectAndCompute(gray, None)
        
        if frame_descriptors is None:
            return None
            
        best_matches = 0
        detected_sign = None
        
        for sign_type, features in self.sign_features.items():
            matches = self.flann.knnMatch(
                features['descriptors'],
                frame_descriptors,
                k=2
            )
            
            good_matches = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
            
            if len(good_matches) > 10:
                src_pts = np.float32([
                    features['keypoints'][m.queryIdx].pt 
                    for m in good_matches
                ]).reshape(-1, 1, 2)
                dst_pts = np.float32([
                    frame_keypoints[m.trainIdx].pt 
                    for m in good_matches
                ]).reshape(-1, 1, 2)
                
                H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                
                if H is not None:
                    inliers = np.sum(mask)
                    if inliers > best_matches:
                        best_matches = inliers
                        detected_sign = sign_type
                        
                        h, w = features['image'].shape
                        pts = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
                        dst = cv2.perspectiveTransform(pts, H)
                        frame = cv2.polylines(frame, [np.int32(dst)], True, (0, 255, 0), 3)
                        
                        frame = cv2.drawMatches(
                            features['image'], 
                            features['keypoints'],
                            frame,
                            frame_keypoints,
                            good_matches,
                            None,
                            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                        )
        
        return detected_sign

    def image_callback(self, msg):
        # Skip processing if detection is blocked
        if self.is_blocked:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
            return
            
        detected_sign = self.detect_sign(cv_image)
        
        if detected_sign and not self.is_blocked:
            self.get_logger().info(f"{detected_sign} sign detected!")
            self.sign_publisher.publish(String(data=detected_sign))
            
            # Block further detections immediately
            self.is_blocked = True
            
            if detected_sign == "STOP":
                self.handle_stop_sign()
            elif detected_sign == "PARKING":
                self.handle_parking_sign()
        
        cv2.imshow('Traffic Sign Detection', cv_image)
        cv2.waitKey(1)

    def start_countdown(self):
        """Start the 5-second countdown timer"""
        self.remaining_time = 5
        self.stop_timer = self.create_timer(1.0, self.countdown_callback)

    def countdown_callback(self):
        """Handle the countdown"""
        self.get_logger().info(f"Countdown: {self.remaining_time}")
        self.remaining_time -= 1
        
        if self.remaining_time < 0:
            self.stop_timer.cancel()
            # Resume movement
            move_twist = Twist()
            move_twist.linear.x = 0.2
            move_twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(move_twist)
            
            # Restart line follower
            restart_msg = String()
            restart_msg.data = "RESTART"
            self.line_follower_restart_publisher.publish(restart_msg)
            
            self.get_logger().info("Robot resumed movement after STOP")
            
            # Start 10-second cooldown
            self.cooldown_timer = self.create_timer(10.0, self.end_cooldown)

    def handle_stop_sign(self):
        """Handle STOP sign detection."""
        # Stop the robot
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_twist)
        self.get_logger().info("Robot stopped at STOP sign")
        
        # Start the 5-second countdown
        self.start_countdown()

    def end_cooldown(self):
        """End the cooldown period and allow new detections"""
        self.is_blocked = False
        self.cooldown_timer.cancel()
        self.get_logger().info("Cooldown ended, ready for new detections")
    

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedTrafficSignDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()