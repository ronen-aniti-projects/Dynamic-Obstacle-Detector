import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ronen/Optical_Flow_Node/install/dynamic_obstacle_detector'
