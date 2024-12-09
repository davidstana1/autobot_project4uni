import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/david/Desktop/autobot_facultate/my_ros2_ws/install/turtlebot3_autobot'
