import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw',  # camera topic
            self.image_callback, 
            10
        )
        
        self.bridge = CvBridge()
        
        #PID parameters
        self.Kp = 0.003
        self.Kd = 0.0005
        
        self.previous_error = 0
        self.MAX_LINEAR_SPEED = 0.1
        self.MAX_ANGULAR_SPEED = 0.4

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def process_image(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Detect edges
        edges = cv2.Canny(blur, 30, 100)
        
        # Create ROI mask (lower part of image)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([
            [(width*0.1, height),
             (width*0.1, height*0.6),
             (width*0.9, height*0.6),
             (width*0.9, height)]
        ], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)
        
        # Detect lines
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=30,
            maxLineGap=10
        )
        
        self.follow_lines(lines, width, height)

    def follow_lines(self, lines, width, height):
        twist = Twist()
        
        if lines is not None and len(lines) > 0:
            # Detect line angles to handle curves
            left_lines = []
            right_lines = []
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Calculate slope and angle
                if x2 - x1 != 0:
                    slope = (y2 - y1) / (x2 - x1)
                    angle = np.arctan(slope)
                    
                    # Categorize lines
                    if slope < -0.1:
                        left_lines.append((x1, y1, x2, y2, angle))
                    elif slope > 0.1:
                        right_lines.append((x1, y1, x2, y2, angle))
            
            # Advanced line tracking for curves
            if left_lines and right_lines:
                # Weighted average of line midpoints and angles
                left_midpoints = [(x1 + x2)/2 for x1, y1, x2, y2, _ in left_lines]
                right_midpoints = [(x1 + x2)/2 for x1, y1, x2, y2, _ in right_lines]
                
                # Compute center and curve adaptation
                left_center = np.mean(left_midpoints)
                right_center = np.mean(right_midpoints)
                road_center = (left_center + right_center) / 2
                
                # Error calculation with curve consideration
                error = road_center - (width / 2)
                
                # PD control with curve adaptation
                angular_z = self.Kp * error + self.Kd * (error - self.previous_error)
                self.previous_error = error
                
                # Limit angular speed
                angular_z = max(min(angular_z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)
                
                # Adaptive speed based on curve sharpness
                linear_speed = self.MAX_LINEAR_SPEED * max(0.5, 1 - abs(angular_z))
                
                # Set movement
                twist.linear.x = linear_speed
                twist.angular.z = -angular_z
                
                self.get_logger().info(f"Curve tracking: Center={road_center:.2f}, Error={error:.2f}")
            
            elif left_lines or right_lines:
                # Searching behavior for missing line
                twist.linear.x = self.MAX_LINEAR_SPEED * 0.5
                twist.angular.z = 0.2
            else:
                # No lines
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # No lines at all
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()