#!/bin/bash

# Frustum Visualization Test Script
# This script demonstrates how to test frustum generation and visualization

echo "========================================="
echo "FRONTIER Frustum Visualization Test"
echo "========================================="

# Get the script's directory and navigate to workspace relative to it
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

# Navigate to ROS2 workspace
cd "${WORKSPACE_ROOT}"

# Activate virtual environment
source .venv/bin/activate

# Source ROS2 setup
source install/setup.bash

echo ""
echo "Starting frustum visualizer in manual mode..."
echo "Frustum will be generated for:"
echo "  - Camera: camera_1"
echo "  - Bounding box: center=(320, 240), size=100x100"
echo "  - Near/Far distance: 1.0m / 30.0m"
echo ""

# Run the frustum visualizer with manual bbox
echo "To visualize in RViz2, open another terminal and run:"
echo "  rviz2 -d src/frontier/config/frustum_test.rviz"
echo ""
echo "Or launch with RViz directly:"
echo "  ros2 launch frontier frustum_test_launch.py launch_rviz:=true"
echo ""

# Launch with different options based on argument
if [ "$1" == "rviz" ]; then
    echo "Launching with RViz..."
    ros2 launch frontier frustum_test_launch.py launch_rviz:=true
elif [ "$1" == "yolo" ]; then
    echo "Launching in YOLO detection mode..."
    ros2 launch frontier frustum_test_launch.py use_manual_bbox:=false
elif [ "$1" == "custom" ]; then
    echo "Launching with custom bbox parameters..."
    ros2 launch frontier frustum_test_launch.py \
        manual_bbox_center_x:=400 \
        manual_bbox_center_y:=300 \
        manual_bbox_width:=150 \
        manual_bbox_height:=200 \
        near_distance:=2.0 \
        far_distance:=50.0
else
    echo "Launching in default manual mode..."
    ros2 launch frontier frustum_test_launch.py
fi