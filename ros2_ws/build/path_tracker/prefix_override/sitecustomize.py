import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yashverma/turtlebot3_path_tracking/ros2_ws/install/path_tracker'
