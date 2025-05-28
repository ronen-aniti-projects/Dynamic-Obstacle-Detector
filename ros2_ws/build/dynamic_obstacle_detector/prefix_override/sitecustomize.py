import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ronen/Documents/Optical-Flow-Node/ros2_ws/install/dynamic_obstacle_detector'
