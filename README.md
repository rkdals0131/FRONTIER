# FRONTIER - Frustum-based 3D Object Fusion for ROS2

**F**rustum-based **R**efined **O**bject **N**etworking & **T**racking via **I**ntersection **E**ngine for **R**anging-data

## 🎯 Overview

FRONTIER is a sophisticated ROS2 package that performs real-time 3D spatial matching between 2D camera detections (YOLO) and 3D LiDAR detections using view frustum intersection techniques. The system supports multi-camera fusion with precise extrinsic calibration, enabling accurate object tracking across different sensor modalities.

### Key Features

- **Multi-Camera Support**: Simultaneous processing of multiple cameras (camera_1, camera_2)
- **Real-time Performance**: 20Hz visualization update rate with optimized processing
- **Precise Calibration**: Extrinsic/intrinsic calibration with actual physical camera positions
- **Advanced Visualization**: 3D frustums with both wireframe and semi-transparent filled surfaces
- **Flexible Matching**: Hungarian algorithm for optimal frustum-to-box association
- **Production-Ready**: Unified pipeline for both testing and production

## 🏗️ Architecture

### System Overview

```
┌─────────────┐   ┌─────────────┐        ┌──────────────┐
│  Camera 1   │   │  Camera 2   │        │    LiDAR     │
│    YOLO     │   │    YOLO     │        │  3D Boxes    │
└─────┬───────┘   └──────┬──────┘        └──────┬───────┘
      │                  │                       │
      │ 2D Detections    │ 2D Detections       │ 3D Detections
      │                  │                       │
      └──────────┬───────┴───────────────────────┘
                 │
         ┌───────▼────────┐
         │   FRONTIER     │
         │     Node       │
         └───────┬────────┘
                 │
      ┌──────────┴──────────┐
      │                     │
┌─────▼──────┐    ┌─────────▼────────┐
│   Fused    │    │  Visualization   │
│ Detections │    │    (RViz2)       │
└────────────┘    └──────────────────┘
```

### Core Components

1. **FrustumGenerator** (`frustum_generator.py`)
   - Generates 3D view frustums from 2D bounding boxes
   - Uses pinhole camera model with intrinsic matrix K
   - Transforms frustums to LiDAR coordinate system
   - **Special Feature**: Frustum apex precisely matches actual camera lens center position

2. **IntersectionEngine** (`intersection_engine.py`)
   - Computes IoU between 3D frustums and 3D bounding boxes
   - Multiple methods: point-in-frustum, voxel-based, sampling
   - Optimized for real-time performance

3. **CalibrationLoader** (`calibration_loader.py`)
   - Loads multi-camera intrinsic/extrinsic parameters
   - Handles coordinate transformations (camera ↔ LiDAR)
   - Computes actual camera positions in 3D space

4. **FrontierVisualizer** (`visualization.py`)
   - Creates RViz2 markers for debugging
   - Camera-specific colors (camera_1: yellow, camera_2: cyan)
   - Wireframe + semi-transparent filled frustums
   - Configurable update rate (10/20/30 Hz)

## 🚀 Installation

### Prerequisites

```bash
# ROS2 Humble or later
sudo apt install ros-humble-desktop

# Python dependencies
pip install numpy scipy opencv-python pyyaml
```

### Build

```bash
# Activate virtual environment
source ${ROS2_WS}/.venv/bin/activate  # Or source .venv/bin/activate from workspace root

# Navigate to workspace
cd ${ROS2_WS}  # Or navigate to your ROS2 workspace directory

# Build package
colcon build --packages-select frontier

# Source the workspace
source install/setup.bash
```

## 📡 Topics

### Input Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera_1/detections` | `yolo_msgs/DetectionArray` | YOLO 2D detections from camera 1 |
| `/camera_2/detections` | `yolo_msgs/DetectionArray` | YOLO 2D detections from camera 2 |
| `/cone/lidar/box` | `vision_msgs/Detection3DArray` | LiDAR 3D bounding boxes |

### Output Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/frontier/fused_detections` | `vision_msgs/Detection3DArray` | Matched 3D objects with IoU scores |
| `/frontier/visualization` | `visualization_msgs/MarkerArray` | RViz2 visualization markers |

## 🎮 Usage

### Launch Main Node

```bash
# Standard launch
ros2 launch frontier frontier_launch.py

# With debug logging
ros2 launch frontier frontier_launch.py log_level:=debug
```

### Test/Debug Mode

```bash
# Launch with visualization
ros2 launch frontier frustum_test_launch.py

# With RViz2
ros2 launch frontier frustum_test_launch.py launch_rviz:=true
```

## ⚙️ Configuration

### Main Configuration (`frontier_config.yaml`)

```yaml
frontier:
  # Frustum parameters
  frustum:
    near_distance: 1.0    # meters
    far_distance: 15.0    # meters
    
  # Matching parameters
  matching:
    iou_threshold: 0.3    # Minimum IoU for valid match
    method: "hungarian"   # Optimal assignment algorithm
    
  # Visualization settings
  visualization:
    publish_markers: true
    publish_rate_hz: 20.0  # Smooth visualization at 20Hz
    marker_lifetime_sec: 0.1
    
  # Camera configurations
  cameras:
    - id: "camera_1"
      detections_topic: "/camera_1/detections"
    - id: "camera_2"
      detections_topic: "/camera_2/detections"
```

## 🔬 Technical Details

### Coordinate System Transformations

The system handles multiple coordinate transformations:

1. **Pixel → Camera Ray**: Using intrinsic matrix K⁻¹
2. **Camera → LiDAR**: Using extrinsic calibration matrix
3. **os_lidar → os_sensor**: 180° rotation correction

### Frustum Generation Process

```python
# 1. Unproject 2D bbox corners to 3D rays
ray_cam = K_inv @ [u, v, 1]ᵀ

# 2. Transform to LiDAR frame
ray_lidar = R_lidar_cam @ ray_cam

# 3. Calculate frustum corners
near_point = camera_center + ray * near_distance
far_point = camera_center + ray * far_distance
```

### Camera Position Calculation

The frustum apex (camera center) is computed from extrinsic calibration:

```python
T_lidar_cam = np.linalg.inv(extrinsic_matrix)
camera_position = T_lidar_cam[:3, 3]
```

This ensures the frustum apex **exactly matches the physical camera lens position** in 3D space.

### Visualization Features

- **Wireframe Edges**: LINE_LIST markers for frustum structure
- **Filled Surfaces**: TRIANGLE_LIST markers with semi-transparency
- **Camera-Specific Colors**:
  - Camera 1: Yellow (matched) / Orange (unmatched)
  - Camera 2: Cyan (matched) / Light Blue (unmatched)
- **Dynamic Update**: Configurable 10/20/30 Hz refresh rate

## 📊 Performance Optimization

### Real-time Optimizations

- **Message Throttling**: Maximum 100 YOLO / 200 LiDAR detections
- **Distance Gating**: Skip far/near objects for efficiency
- **Memory Management**: Immediate reference release after processing
- **UDP-style QoS**: Best-effort reliability for low latency
- **Computation Limits**: Maximum 500 IoU calculations per frame

### Benchmarks

| Metric | Value |
|--------|-------|
| Processing Rate | 20 Hz |
| Max Frustums/Frame | 20 |
| Max 3D Boxes | 50 |
| Typical Latency | < 50ms |
| Memory Usage | < 100MB |

## 🧪 Testing

### Unit Tests

```bash
colcon test --packages-select frontier
colcon test-result --verbose
```

### Visualization Test

```bash
# Test with manual bounding box
ros2 launch frontier frustum_test_launch.py \
    use_manual_bbox:=true \
    manual_bbox_center_x:=400 \
    manual_bbox_center_y:=300
```

## 🔧 Troubleshooting

### Common Issues

1. **No Visualization**
   - Check if YOLO detections are being received
   - Verify LiDAR topic is publishing
   - Ensure RViz2 is subscribed to `/frontier/visualization`

2. **Poor Matching**
   - Verify calibration files are correct
   - Check time synchronization (< 100ms difference)
   - Adjust IoU threshold in config

3. **Performance Issues**
   - Reduce `max_frustums` and `max_boxes` in config
   - Lower visualization rate to 10Hz
   - Enable distance gating

## 📚 Advanced Features

### Adaptive Frustum (Experimental)

The system includes an adaptive frustum feature that adjusts near/far distances based on detection size:

```yaml
adaptive_frustum:
  enabled: true
  object_height_m: 0.70
  object_width_m: 0.30
```

### Multi-Stage Matching

For complex scenarios, the system supports:
- Distance-based gating with different IoU thresholds
- FOV-based filtering
- Confidence-weighted matching

## 🤝 Contributing

Contributions are welcome! Please follow these guidelines:

1. Create feature branches from `main`
2. Write unit tests for new functionality
3. Update documentation and comments
4. Run linting before commits

## 📄 License

This project is part of a ROS2 workspace.

## 🙏 Acknowledgments

- YOLO for 2D object detection
- ROS2 community for the framework
- OpenCV for camera calibration utilities

---

**Note**: This package uses a unified pipeline for both testing and production. The same core logic (`frontier_node.py`) handles all processing, with debug visualization always available when enabled in configuration.