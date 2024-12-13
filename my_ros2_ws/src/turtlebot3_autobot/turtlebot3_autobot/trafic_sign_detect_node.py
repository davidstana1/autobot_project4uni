import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.sign_publisher = self.create_publisher(String, '/detected_sign', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # New publisher to restart line follower
        self.line_follower_restart_publisher = self.create_publisher(String, '/line_follower_restart', 10)

        self.bridge = CvBridge()

        # Enhanced template loading with multiple scales and rotations
        self.stop_templates = self.load_stop_templates()

        # Detection state to prevent multiple detections
        self.last_detection_time = 0
        self.detection_cooldown = 5.0  # 5 seconds between detections

        # Redimensionare imagini și șabloane pentru performanță
        self.resized_templates = self.resize_templates(self.stop_templates)

    def load_stop_templates(self):
        """Load multiple stop sign templates with different scales and slight rotations."""
        base_path = '/home/david/Desktop/autobot_facultate/my_ros2_ws/src/turtlebot3_autobot/turtlebot3_autobot/images/stop_sign_bun.jpeg'
        try:
            base_template = cv2.imread(base_path, cv2.IMREAD_GRAYSCALE)
            if base_template is None:
                self.get_logger().error("Failed to load base STOP template!")
                return []

            templates = []
            scales = [0.5, 0.75, 1.0, 1.25, 1.5]
            rotations = [0, -15, 15]

            for scale in scales:
                for angle in rotations:
                    # Resize template
                    resized = cv2.resize(base_template, None, fx=scale, fy=scale)
                    
                    # Rotate template
                    rows, cols = resized.shape
                    rotation_matrix = cv2.getRotationMatrix2D((cols/2, rows/2), angle, 1)
                    rotated = cv2.warpAffine(resized, rotation_matrix, (cols, rows))
                    
                    templates.append(rotated)

            return templates
        except Exception as e:
            self.get_logger().error(f"Error loading templates: {str(e)}")
            return []

    def resize_templates(self, templates):
        """Resize all templates for faster matching."""
        resized_templates = []
        for template in templates:
            # Redimensionează șablonul la dimensiunea dorită pentru matching rapid
            resized_templates.append(cv2.resize(template, (64, 64)))  # Ajustează dimensiunea după necesități
        return resized_templates

    def image_callback(self, msg):
        # Check if enough time has passed since last detection
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_detection_time < self.detection_cooldown:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
            return

        # Conversie la imagine grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Normalizează contrastul
        gray_image = cv2.equalizeHist(gray_image)

        # Detectare semn STOP
        detected = self.detect_stop_sign(gray_image, cv_image)

        if detected:
            self.get_logger().info("STOP sign detected!")
            self.sign_publisher.publish(String(data="STOP"))
            self.last_detection_time = current_time
            
            # Utilizează thread sau timer pentru oprire și pornire
            self.stop_and_resume_robot()

        # Afișare imagine (optional, poate fi eliminată pentru performanță)
        cv2.imshow('Traffic Sign Detection', cv_image)
        cv2.waitKey(1)

    def detect_stop_sign(self, gray_image, cv_image):
        """Detectează semnul STOP folosind template matching pe șabloane redimensionate."""
        if not self.resized_templates:
            self.get_logger().error("No resized STOP templates loaded!")
            return False

        best_match = 0
        best_location = None
        best_template = None

        for template in self.resized_templates:
            # Template matching
            result = cv2.matchTemplate(gray_image, template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            if max_val > best_match:
                best_match = max_val
                best_location = max_loc
                best_template = template

        threshold = 0.4
        if best_match >= threshold and best_location and best_template is not None:
            top_left = best_location
            h, w = best_template.shape
            bottom_right = (top_left[0] + w, top_left[1] + h)

            # Desenează un dreptunghi în jurul semnului detectat
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
            
            self.get_logger().info(f"Best match: {best_match}")
            return True

        return False

    def stop_and_resume_robot(self):
        """Oprește robotul pentru 2 secunde și reia mișcarea."""
        # Trimite comanda pentru oprire
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_twist)
        self.get_logger().info("Robot stopped.")

        # Așteaptă 2 secunde
        # Folosește un timer sau alt mecanism pentru a evita blocarea
        time.sleep(5)

        # Reia mișcarea robotului
        move_twist = Twist()
        move_twist.linear.x = 0.2  # Viteză liniară constantă
        move_twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(move_twist)
        
        # Publică semnal de restart pentru line follower
        restart_msg = String()
        restart_msg.data = "RESTART"
        self.line_follower_restart_publisher.publish(restart_msg)
        
        self.get_logger().info("Robot resumed movement. Line follower restarted.")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
