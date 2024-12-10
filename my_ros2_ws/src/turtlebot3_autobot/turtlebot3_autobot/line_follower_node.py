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
        
       

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')
        canny = cv2.Canny(cv_image, 100, 200)
        self.freespace(canny, cv_image)
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)

    def freespace(self, canny_frame, img):
        height, width = canny_frame.shape
        
        DreaptaLim = width // 2
        StangaLim = width // 2
 
        mask = np.zeros((height, width), dtype=np.uint8)
        contour = []
 
        # Set right limit
        for i in range(width // 2, width-1):
            if canny_frame[height - 10, i]:
                DreaptaLim = i
                break
 
        # Set left limit
        for i in range(width // 2):
            if canny_frame[height - 10, width // 2 - i]:
                StangaLim = width // 2 - i
                break
 
        # Adjust limits
        if StangaLim == width // 2:
            StangaLim = 1
        if DreaptaLim == width // 2:
            DreaptaLim = width
        contour.append((StangaLim, height - 10))
        cv2.circle(img,( StangaLim,height-10),5,(255),-1)
        cv2.circle(img,( DreaptaLim, height-10),5,(255),-1)
 
        for j in range(StangaLim, DreaptaLim - 1, 10):
            for i in range(height - 10, 9, -1):
                if canny_frame[i, j]:
                    cv2.line(img, (j, height - 10), (j, i), (255), 2)
                    contour.append((j, i))
                    break
                if i == 10:
                    contour.append((j, i))
                    cv2.line(img, (j, height - 10), (j, i), (255), 2)
        contour.append((DreaptaLim, height - 10))
        contours = [np.array(contour)]
        cv2.drawContours(mask, contours, 0, (255), cv2.FILLED)
        cv2.imshow("mask", mask)
        M = cv2.moments(contours[0])
        if M["m00"] != 0:
            centroid_x = int(M["m10"] / M["m00"])
            centroid_y = int(M["m01"] / M["m00"])
        else:
            centroid_x, centroid_y = 0, 0
 
        cv2.arrowedLine(img,(width//2, height-10),(centroid_x,centroid_y),(60,90,255),4)

        angle = np.arctan2(centroid_x -width//2, height-10 - centroid_y)
        print(angle)
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = - angle * 0.5
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()