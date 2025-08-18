import sys
if sys.prefix == '/home/user1/ROS2_Workspace/ros2_ws/.venv':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user1/ROS2_Workspace/ros2_ws/src/frontier/install/frontier'
