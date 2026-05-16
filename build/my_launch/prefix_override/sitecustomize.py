import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sunrise/lidar_locate_ws/lidar_locate_ws/src/install/my_launch'
