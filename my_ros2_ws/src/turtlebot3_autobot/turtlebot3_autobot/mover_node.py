import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBot3Mover(Node):
    def __init__(self):
        super().__init__('turtlebot3_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, speed=0.2, duration=2.0):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.get_logger().info(f'Moving forward at speed {speed} for {duration} seconds.')
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info('Stopping the robot.')
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Mover()
    try:
        node.move_forward(speed=0.2, duration=2.0)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
