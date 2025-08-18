#!/usr/bin/env python3
"""
Frustum Visualizer Node
YOLO detection으로부터 frustum을 생성하고 RViz에 시각화만 수행
테스트 및 캘리브레이션 검증용
"""

import os
import yaml
import numpy as np
import time
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time

from yolo_msgs.msg import DetectionArray, BoundingBox2D
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration

from .calibration_loader import CalibrationLoader, CameraCalibration
from .frustum_generator import FrustumGenerator, Frustum
from .visualization import FrontierVisualizer
from .adaptive_frustum import AdaptiveFrustumEstimator, AdaptiveFrustumConfig


class FrustumVisualizerNode(Node):
    """Frustum 시각화 전용 노드"""
    
    def __init__(self):
        super().__init__('frustum_visualizer_node')
        
        # Parameters
        self.declare_parameter('config_file', 
            '/home/user1/ROS2_Workspace/ros2_ws/src/frontier/config/frontier_config.yaml')
        self.declare_parameter('near_distance', 0.5)
        self.declare_parameter('far_distance', 30.0)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('use_manual_bbox', False)
        self.declare_parameter('manual_bbox_center_x', 320.0)
        self.declare_parameter('manual_bbox_center_y', 240.0)
        self.declare_parameter('manual_bbox_width', 100.0)
        self.declare_parameter('manual_bbox_height', 100.0)
        
        # Safety parameters
        self.declare_parameter('max_frustums_per_frame', 5)  # 프레임당 최대 frustum 수
        self.declare_parameter('min_process_interval', 0.1)  # 최소 처리 간격 (초)
        self.declare_parameter('drop_old_messages', True)  # 오래된 메시지 드롭
        self.declare_parameter('message_age_threshold', 0.5)  # 메시지 나이 임계값 (초)
        
        # Get parameters
        self.config_file = self.get_parameter('config_file').value
        self.near_distance = self.get_parameter('near_distance').value
        self.far_distance = self.get_parameter('far_distance').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_manual_bbox = self.get_parameter('use_manual_bbox').value
        
        # Get safety parameters
        self.max_frustums_per_frame = self.get_parameter('max_frustums_per_frame').value
        self.min_process_interval = self.get_parameter('min_process_interval').value
        self.drop_old_messages = self.get_parameter('drop_old_messages').value
        self.message_age_threshold = self.get_parameter('message_age_threshold').value
        
        # use_sim_time은 ROS2가 자동 관리하는 파라미터
        try:
            self.use_sim_time = self.get_parameter('use_sim_time').value
        except:
            self.use_sim_time = False
        
        # Processing control
        self.processing_lock = threading.Lock()
        self.is_processing = False
        self.last_process_time = 0.0
        self.dropped_messages = 0
        
        # Load configuration
        self.load_config()
        
        # Load calibrations for all cameras
        self.load_calibrations()
        
        # Initialize components
        self.frustum_generators = {}
        self.adaptive_estimators = {}
        for cam_id in self.calibrations.keys():
            self.frustum_generators[cam_id] = FrustumGenerator(self.calibrations[cam_id])
        # Adaptive estimator per camera (optional via config)
        adaptive_cfg = (self.config or {}).get('adaptive_frustum', {})
        self.adaptive_enabled = bool(adaptive_cfg.get('enabled', False))
        for cam_id, calib in self.calibrations.items():
            try:
                cfg = AdaptiveFrustumConfig(
                    enabled=self.adaptive_enabled,
                    object_height_m=adaptive_cfg.get('object_height_m', 0.70),
                    object_width_m=adaptive_cfg.get('object_width_m', 0.30),
                    near_min_m=adaptive_cfg.get('near_min_m', self.near_distance),
                    far_max_m=adaptive_cfg.get('far_max_m', self.far_distance),
                    margin_min=adaptive_cfg.get('margin_min', 0.15),
                    margin_max=adaptive_cfg.get('margin_max', 0.80),
                    area_thresholds=tuple(adaptive_cfg.get('area_thresholds', [0.10, 0.05, 0.02]))
                )
                self.adaptive_estimators[cam_id] = AdaptiveFrustumEstimator(calib, cfg)
            except Exception as e:
                self.get_logger().warn(f"Failed to init adaptive estimator for {cam_id}: {e}")

        self.visualizer = FrontierVisualizer(frame_id='os_sensor')  # LiDAR frame
        
        # Storage for latest messages
        self.latest_yolo_detections = {}  # camera_id -> DetectionArray
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/frustum_visualizer/markers',
            10
        )
        
        self.get_logger().info(f"Publishing frustum markers to: /frustum_visualizer/markers")
        
        # QoS for subscribers - 안전한 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # 최신 메시지만 유지
        )
        
        if self.use_manual_bbox:
            # Manual mode - create timer for periodic publishing
            self.get_logger().info("Using manual bounding box mode for all cameras")
            self.timer = self.create_timer(
                1.0 / self.publish_rate, 
                self.manual_bbox_callback
            )
        else:
            # Subscribe to YOLO detections for ALL cameras
            self.yolo_subs = {}
            if 'cameras' in self.config:
                for camera in self.config['cameras']:
                    cam_id = camera.get('id')
                    yolo_topic = camera.get('detections_topic')
                    
                    if cam_id and yolo_topic:
                        # Create subscription for this camera
                        self.yolo_subs[cam_id] = self.create_subscription(
                            DetectionArray,
                            yolo_topic,
                            lambda msg, cid=cam_id: self.yolo_callback(msg, cid),
                            qos
                        )
                        self.get_logger().info(f"Subscribing to {cam_id} detections on: {yolo_topic}")
            
            if not self.yolo_subs:
                self.get_logger().error("No camera subscriptions created!")
        
        # Timer for processing all cameras
        self.process_timer = self.create_timer(0.1, self.process_all_detections)  # 10Hz
        
        # Statistics - per camera
        self.camera_stats = {}
        for cam_id in self.calibrations.keys():
            self.camera_stats[cam_id] = {
                'frame_count': 0,
                'total_frustums': 0,
                'detections_received': 0
            }
        
        # 마커 청소를 위한 타이머 (30초마다 DELETEALL 전송)
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_markers)
        self.last_cleanup_time = 0
        
        cameras_list = ', '.join(self.calibrations.keys())
        self.get_logger().info(
            f"Frustum Visualizer initialized for cameras: {cameras_list}\n"
            f"  Near distance: {self.near_distance}m\n"
            f"  Far distance: {self.far_distance}m"
        )
    
    def load_config(self):
        """Load configuration from YAML file"""
        if not os.path.exists(self.config_file):
            self.get_logger().warn(f"Config file not found: {self.config_file}")
            self.get_logger().info("Using default configuration")
            return
        
        with open(self.config_file, 'r') as f:
            full_config = yaml.safe_load(f)
            self.config = full_config.get('frontier', {})
        
        # Override parameters from config if not set via launch
        if 'frustum' in self.config:
            if self.near_distance == 0.5:  # Default value
                self.near_distance = self.config['frustum'].get('near_distance', 0.5)
            if self.far_distance == 30.0:  # Default value
                self.far_distance = self.config['frustum'].get('far_distance', 30.0)
    
    def load_calibrations(self):
        """Load camera calibrations for all cameras"""
        config_folder = os.path.dirname(self.config_file)
        loader = CalibrationLoader(config_folder)
        
        # Load calibration files
        intrinsic_file = 'multi_camera_intrinsic_calibration.yaml'
        extrinsic_file = 'multi_camera_extrinsic_calibration.yaml'
        
        try:
            self.calibrations = loader.load_calibrations(intrinsic_file, extrinsic_file)
            
            for cam_id, calibration in self.calibrations.items():
                self.get_logger().info(
                    f"Loaded calibration for {cam_id}\n"
                    f"  Image size: {calibration.image_width}x{calibration.image_height}\n"
                    f"  Camera position in LiDAR: {calibration.get_camera_position_in_lidar()}"
                )
            
        except Exception as e:
            self.get_logger().error(f"Failed to load calibrations: {e}")
            raise
    
    def manual_bbox_callback(self):
        """Create frustums from manual bounding box parameters for all cameras"""
        # Get manual bbox parameters - using yolo_msgs structure
        from yolo_msgs.msg import Pose2D, Point2D, Vector2
        
        bbox = BoundingBox2D()
        bbox.center = Pose2D()
        bbox.center.position = Point2D()
        bbox.center.position.x = self.get_parameter('manual_bbox_center_x').value
        bbox.center.position.y = self.get_parameter('manual_bbox_center_y').value
        bbox.center.theta = 0.0
        bbox.size = Vector2()
        bbox.size.x = self.get_parameter('manual_bbox_width').value
        bbox.size.y = self.get_parameter('manual_bbox_height').value
        
        # Generate frustums for ALL cameras with the same bbox
        all_frustums = []
        for cam_id, generator in self.frustum_generators.items():
            if self.adaptive_enabled and cam_id in self.adaptive_estimators:
                near_d, far_d = self.adaptive_estimators[cam_id].compute_near_far(bbox)
            else:
                near_d, far_d = self.near_distance, self.far_distance
            frustum = generator.generate_frustum(
                bbox,
                float(near_d),
                float(far_d)
            )
            all_frustums.append((cam_id, frustum, bbox))
        
        # Visualize all frustums
        self.visualize_all_cameras(all_frustums)
        
        if sum(self.camera_stats[cam]['frame_count'] for cam in self.camera_stats) % 10 == 0:
            self.get_logger().info(
                f"Manual frustums for all cameras: center=({bbox.center.position.x:.0f}, "
                f"{bbox.center.position.y:.0f}), size=({bbox.size.x:.0f}x{bbox.size.y:.0f})"
            )
    
    def yolo_callback(self, msg: DetectionArray, camera_id: str):
        """Callback for YOLO detections with safety mechanisms"""
        self.get_logger().debug(f"[{camera_id}] YOLO callback triggered with {len(msg.detections)} detections")
        
        if not msg.detections:
            return
        
        # Store detections for this camera
        self.latest_yolo_detections[camera_id] = msg
        
        # Update camera statistics
        if camera_id in self.camera_stats:
            self.camera_stats[camera_id]['detections_received'] += 1
        
        self.get_logger().info(
            f"[{camera_id}] Received {len(msg.detections)} detections "
            f"(Total: {self.camera_stats[camera_id]['detections_received']})"
        )
        
    def process_all_detections(self):
        """Process detections from all cameras"""
        if not self.latest_yolo_detections:
            return
        
        # Check if processing is in progress
        if self.is_processing:
            self.dropped_messages += 1
            if self.dropped_messages % 10 == 0:
                self.get_logger().warn(f"Dropped {self.dropped_messages} messages due to processing lag")
            return
        
        # Check minimum processing interval
        current_time = time.time()
        if current_time - self.last_process_time < self.min_process_interval:
            return
        
        # Set processing flag
        with self.processing_lock:
            if self.is_processing:
                return
            self.is_processing = True
        
        try:
            # Process each camera
            all_camera_frustums = []
            
            for cam_id, msg in self.latest_yolo_detections.items():
                if cam_id not in self.frustum_generators:
                    continue
                
                # Check message age if needed
                if self.drop_old_messages and not self.use_sim_time:
                    current_time = self.get_clock().now()
                    msg_time = Time.from_msg(msg.header.stamp)
                    age = (current_time - msg_time).nanoseconds / 1e9
                    
                    if abs(age) > self.message_age_threshold:
                        self.get_logger().debug(f"[{cam_id}] Dropping old message (age: {age:.2f}s)")
                        continue
                
                # Limit number of detections to process
                detections_to_process = msg.detections[:self.max_frustums_per_frame]
                
                if len(msg.detections) > self.max_frustums_per_frame:
                    self.get_logger().debug(
                        f"[{cam_id}] Limiting detections from {len(msg.detections)} to {self.max_frustums_per_frame}"
                    )
                
                # Generate frustums for this camera
                calibration = self.calibrations[cam_id]
                generator = self.frustum_generators[cam_id]
                
                for detection in detections_to_process:
                    bbox = detection.bbox
                    
                    # Skip invalid bboxes
                    if bbox.size.x <= 0 or bbox.size.y <= 0:
                        continue
                    
                    # Check if bbox is within image bounds
                    if (bbox.center.position.x < 0 or 
                        bbox.center.position.x >= calibration.image_width or
                        bbox.center.position.y < 0 or 
                        bbox.center.position.y >= calibration.image_height):
                        continue
                    
                    try:
                        # Generate frustum (adaptive near/far if enabled)
                        if self.adaptive_enabled and cam_id in self.adaptive_estimators:
                            near_d, far_d = self.adaptive_estimators[cam_id].compute_near_far(bbox)
                        else:
                            near_d, far_d = self.near_distance, self.far_distance
                        frustum = generator.generate_frustum(
                            bbox,
                            float(near_d),
                            float(far_d)
                        )
                        all_camera_frustums.append((cam_id, frustum, bbox))
                        
                    except Exception as e:
                        self.get_logger().warn(f"[{cam_id}] Failed to generate frustum: {e}")
                
                # Update statistics for this camera
                if cam_id in self.camera_stats:
                    self.camera_stats[cam_id]['frame_count'] += 1
                    self.camera_stats[cam_id]['total_frustums'] += len(detections_to_process)
            
            # Visualize all frustums from all cameras
            if all_camera_frustums:
                self.get_logger().debug(f"Generated {len(all_camera_frustums)} total frustums from all cameras")
                self.visualize_all_cameras(all_camera_frustums)
                
                # Log statistics periodically
                total_frames = sum(stats['frame_count'] for stats in self.camera_stats.values())
                if total_frames % 10 == 0:
                    for cam_id, stats in self.camera_stats.items():
                        if stats['frame_count'] > 0:
                            avg_frustums = stats['total_frustums'] / stats['frame_count']
                            self.get_logger().info(
                                f"[{cam_id}] Stats: {stats['frame_count']} frames, "
                                f"{stats['total_frustums']} total frustums, "
                                f"{avg_frustums:.1f} avg frustums/frame"
                            )
            
            # Clear processed messages
            self.latest_yolo_detections.clear()
            
            # Update last process time
            self.last_process_time = time.time()
            
        finally:
            # Always release processing flag
            with self.processing_lock:
                self.is_processing = False
    
    def visualize_all_cameras(self, all_camera_frustums: List[Tuple]):
        """Create and publish visualization markers for all cameras"""
        marker_array = MarkerArray()
        
        # Reset visualizer ID counter
        self.visualizer.reset_id_counter()
        
        # Camera-specific colors
        camera_colors = {
            'camera_1': (1.0, 1.0, 0.0, 0.4),  # Yellow
            'camera_2': (0.0, 1.0, 1.0, 0.4),  # Cyan
        }
        
        # Create markers for each frustum from each camera
        for i, (cam_id, frustum, bbox) in enumerate(all_camera_frustums):
            # Use camera-specific color
            color = camera_colors.get(cam_id, (0.5, 0.5, 0.5, 0.4))  # Default gray
            
            # Create frustum wireframe
            frustum_marker = self.visualizer.create_frustum_marker(
                frustum,
                'os_sensor',
                color,
                f"frustum_{cam_id}_{i}"
            )
            marker_array.markers.append(frustum_marker)
            
            # Add camera center marker with camera-specific color
            camera_marker = self.create_camera_marker(frustum.camera_center, cam_id, i)
            marker_array.markers.append(camera_marker)
            
            # Add text label with camera ID and bbox info
            text_pos = np.mean(frustum.corners[:4], axis=0)  # Near plane center
            text_marker = self.create_info_text(
                cam_id, bbox, text_pos, i
            )
            marker_array.markers.append(text_marker)
            
            # Add coordinate axes at frustum origin
            axes_markers = self.create_axes_markers(frustum.camera_center, cam_id, i)
            marker_array.markers.extend(axes_markers)
        
        # Set timestamp
        now = self.get_clock().now().to_msg()
        for marker in marker_array.markers:
            marker.header.stamp = now
        
        # Publish
        self.marker_pub.publish(marker_array)
    
    def create_camera_marker(self, position: np.ndarray, camera_id: str, index: int) -> Marker:
        """Create a sphere marker for camera position"""
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.ns = f"camera_{camera_id}_{index}"
        marker.id = self.visualizer._get_next_id()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Camera-specific colors
        if camera_id == 'camera_1':
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # Yellow
        elif camera_id == 'camera_2':
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.8)  # Cyan
        else:
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)  # Orange (default)
        
        marker.lifetime = Duration(sec=0, nanosec=100000000)  # 0.1초
        
        return marker
    
    def create_info_text(self, camera_id: str, bbox: BoundingBox2D, position: np.ndarray, index: int) -> Marker:
        """Create text marker with camera ID and bbox information"""
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.ns = f"info_{camera_id}_{index}"
        marker.id = self.visualizer._get_next_id()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + 0.3
        
        marker.scale.z = 0.2
        
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Text with camera ID and bbox info
        marker.text = (
            f"{camera_id}\n"
            f"2D: ({bbox.center.position.x:.0f}, {bbox.center.position.y:.0f})\n"
            f"Size: {bbox.size.x:.0f}x{bbox.size.y:.0f}"
        )
        
        marker.lifetime = Duration(sec=0, nanosec=100000000)  # 0.1초
        
        return marker
    
    def create_axes_markers(self, origin: np.ndarray, camera_id: str, index: int) -> List[Marker]:
        """Create coordinate axes markers at origin"""
        markers = []
        
        # Axes specifications: (direction, color)
        axes = [
            ([1, 0, 0], [1.0, 0.0, 0.0, 0.8]),  # X - Red
            ([0, 1, 0], [0.0, 1.0, 0.0, 0.8]),  # Y - Green
            ([0, 0, 1], [0.0, 0.0, 1.0, 0.8]),  # Z - Blue
        ]
        
        for i, (direction, color) in enumerate(axes):
            marker = Marker()
            marker.header.frame_id = 'os_sensor'
            marker.ns = f"axes_{camera_id}_{index}"
            marker.id = self.visualizer._get_next_id()
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Start point
            start = Point()
            start.x = origin[0]
            start.y = origin[1]
            start.z = origin[2]
            
            # End point
            end = Point()
            end.x = origin[0] + direction[0] * 0.5
            end.y = origin[1] + direction[1] * 0.5
            end.z = origin[2] + direction[2] * 0.5
            
            marker.points = [start, end]
            
            marker.scale.x = 0.02  # Shaft diameter
            marker.scale.y = 0.04  # Head diameter
            
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            
            marker.lifetime = Duration(sec=0, nanosec=100000000)  # 0.1초
            
            markers.append(marker)
        
        return markers
    
    def cleanup_markers(self):
        """주기적으로 DELETEALL 마커를 전송하여 RViz의 잔류 마커 정리"""
        # DELETEALL 마커 생성
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = 'os_sensor'
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = ""
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        
        marker_array.markers.append(delete_marker)
        
        # 퍼블리시
        self.marker_pub.publish(marker_array)
        
        # 로그는 디버그 레벨로만
        self.get_logger().debug("Sent DELETEALL marker for cleanup")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FrustumVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()