# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Package Overview

FRONTIER (Frustum-based Refined Object Networking & Tracking via Intersection Engine for Ranging-data) is a ROS2 Python package that performs 3D spatial matching between 2D camera detections (YOLO) and 3D LiDAR detections using view frustum intersection techniques.

## Architecture

### Core Modules

1. **FrustumGenerator** (`frustum_generator.py`): Generates 3D view frustums from 2D bounding boxes using camera calibration
2. **IntersectionEngine** (`intersection_engine.py`): Computes intersections between 3D frustums and 3D bounding boxes
3. **CalibrationLoader** (`calibration_loader.py`): Loads multi-camera intrinsic/extrinsic calibration parameters
4. **Visualization** (`visualization.py`): Creates RViz markers for debugging frustums and matching results
5. **FrontierNode** (`frontier_node.py`): Main ROS2 node orchestrating the fusion pipeline

### Input Topics
- `/camera_1/yolo_detections` - 2D YOLO detections from camera 1
- `/camera_2/yolo_detections` - 2D YOLO detections from camera 2
- `/cone_detection/sorted_cones_time` - 3D LiDAR detections
- `/usb_cam/camera_info` - Camera calibration info
- `tf2` transforms for coordinate frame conversions

### Output Topics
- `/frontier/fused_detections` - Matched 3D objects (vision_msgs/Detection3DArray)
- `/frontier/visualization` - RViz markers for debugging

## Development Commands

### Build Package
```bash
# Always activate virtual environment first (from workspace root)
source .venv/bin/activate
# Or if running from elsewhere:
# source ${ROS2_WS}/.venv/bin/activate
cd ${ROS2_WS}  # Navigate to your ROS2 workspace
colcon build --packages-select frontier
```

### Run Tests
```bash
# Unit tests
colcon test --packages-select frontier
colcon test-result --verbose

# Linting
python3 -m flake8 frontier/
python3 -m pep257 frontier/
```

### Launch Nodes

```bash
# Main frontier node
ros2 launch frontier frontier_launch.py

# Frustum visualization test (manual mode)
ros2 launch frontier frustum_test_launch.py

# With RViz
ros2 launch frontier frustum_test_launch.py launch_rviz:=true

# With custom bbox parameters
ros2 launch frontier frustum_test_launch.py \
    manual_bbox_center_x:=400 \
    manual_bbox_center_y:=300 \
    near_distance:=2.0 \
    far_distance:=50.0

# YOLO detection mode
ros2 launch frontier frustum_test_launch.py \
    use_manual_bbox:=false \
    camera_id:=camera_1
```

### Test Scripts
```bash
# Test frustum visualization
bash scripts/test_frustum_viz.sh

# Test calibrated frustum
python3 scripts/test_calibrated_frustum.py

# Test frustum intersection
python3 scripts/test_frustum_intersection.py
```

## Configuration Files

- `config/frontier_config.yaml` - Main node configuration
- `config/multi_camera_intrinsic_calibration.yaml` - Camera intrinsic parameters (K matrix, distortion)
- `config/multi_camera_extrinsic_calibration.yaml` - Camera-to-LiDAR transforms (rotation, translation)
- `config/frustum_test.rviz` - RViz configuration for visualization

## Key Implementation Details

### Frustum Generation
- Uses pinhole camera model with intrinsic matrix K
- Converts 2D bbox corners to 3D rays
- Creates 6-plane frustum (4 sides + near/far planes)
- Plane equations stored as ax + by + cz + d = 0

### Intersection Calculation
- Point-in-frustum test for 3D bbox vertices
- IoU-based scoring for association
- Hungarian algorithm for optimal matching

### Multi-Camera Support
- Simultaneous processing of camera_1 and camera_2
- Independent calibration per camera
- Unified fusion output

## Development Workflow

1. **Python PoC First**: Prototype algorithms in Python for rapid iteration
2. **Test with Manual Mode**: Use frustum_test_launch.py to verify calibration
3. **Integrate with Live Data**: Connect to actual YOLO/LiDAR topics
4. **Performance Optimization**: Profile and optimize critical paths
5. **C++ Migration**: Port performance-critical sections to C++ if needed

## Important Notes

- Always source the virtual environment before building/running
- Calibration files must match actual camera setup
- Use timeout for ros2 launch commands during testing: `timeout 10 ros2 launch ...`
- Check tf2 transforms are being published for coordinate conversions
- Monitor `/frontier/visualization` in RViz for debugging

## Dependencies

- ROS2 packages: rclpy, tf2_ros, sensor_msgs, vision_msgs, visualization_msgs
- Python libs: numpy, scipy, opencv-python, pyyaml
- Build type: ament_python