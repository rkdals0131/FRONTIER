#!/usr/bin/env python3
"""
FRONTIER ROS2 Node
Frustum-based 3D object matching between camera and LiDAR detections
"""

import os
import yaml
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from scipy.optimize import linear_sum_assignment
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3DArray, BoundingBox3D
from yolo_msgs.msg import DetectionArray, BoundingBox2D
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion, Pose, PoseArray, Point
from custom_interface.msg import TrackedCone, TrackedConeArray
from std_msgs.msg import Header

import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .calibration_loader import CalibrationLoader, CameraCalibration
from .frustum_generator import FrustumGenerator, Frustum
from .intersection_engine import IntersectionEngine
from .visualization import FrontierVisualizer
from .adaptive_frustum import AdaptiveFrustumEstimator, AdaptiveFrustumConfig


class FrontierNode(Node):
    """FRONTIER main ROS2 node"""
    
    def __init__(self):
        super().__init__('frontier_node')
        
        # Load configuration
        self.load_config()
        
        # Load calibrations
        self.load_calibrations()
        
        # Initialize components
        self.initialize_components()
        
        # Setup ROS2 interfaces
        self.setup_ros_interfaces()
        
        self.get_logger().info("FRONTIER node initialized successfully")
    
    def load_config(self):
        """Load configuration from YAML file"""
        # Get config file path from parameter or use package-relative default
        package_share_dir = get_package_share_directory('frontier')
        default_config_path = Path(package_share_dir) / 'config' / 'frontier_config.yaml'
        
        config_file = self.declare_parameter('config_file', str(default_config_path)).value
        
        if not os.path.exists(config_file):
            self.get_logger().error(f"Config file not found: {config_file}")
            raise FileNotFoundError(f"Config file not found: {config_file}")
        
        with open(config_file, 'r') as f:
            full_config = yaml.safe_load(f)
            self.config = full_config['frontier']
        
        self.get_logger().info(f"Loaded config from: {config_file}")
        
        # Extract key parameters
        self.iou_threshold = self.config['matching']['iou_threshold']
        self.max_distance = self.config['matching']['max_distance']
        self.near_distance = self.config['frustum']['near_distance']
        self.far_distance = self.config['frustum']['far_distance']
        self.iou_method = self.config['iou']['method']
        self.apply_distortion = self.config['calibration']['apply_distortion_correction']
        # Adaptive frustum (optional)
        self.adaptive_settings = self.config.get('adaptive_frustum', {}) or {}
        self.adaptive_enabled = bool(self.adaptive_settings.get('enabled', False))
    
    def load_calibrations(self):
        """Load camera calibration parameters"""
        config_folder = os.path.dirname(
            self.get_parameter('config_file').value
        )
        
        loader = CalibrationLoader(config_folder)
        
        # Load calibration files
        intrinsic_file = self.config['calibration']['intrinsic_file']
        extrinsic_file = self.config['calibration']['extrinsic_file']
        
        self.calibrations = loader.load_calibrations(intrinsic_file, extrinsic_file)
        
        # Create frustum generators for each camera
        self.frustum_generators = {}
        self.adaptive_estimators = {}
        for cam_id in self.calibrations.keys():
            self.frustum_generators[cam_id] = FrustumGenerator(self.calibrations[cam_id])
            self.get_logger().info(f"Loaded calibration for {cam_id}")

            # Initialize adaptive estimator per camera if enabled
            try:
                cfg = AdaptiveFrustumConfig(
                    enabled=self.adaptive_enabled,
                    object_height_m=self.adaptive_settings.get('object_height_m', 0.70),
                    object_width_m=self.adaptive_settings.get('object_width_m', 0.30),
                    near_min_m=self.adaptive_settings.get('near_min_m', self.near_distance),
                    far_max_m=self.adaptive_settings.get('far_max_m', self.far_distance),
                    margin_min=self.adaptive_settings.get('margin_min', 0.15),
                    margin_max=self.adaptive_settings.get('margin_max', 0.80),
                    area_thresholds=tuple(self.adaptive_settings.get('area_thresholds', [0.10, 0.05, 0.02]))
                )
                self.adaptive_estimators[cam_id] = AdaptiveFrustumEstimator(self.calibrations[cam_id], cfg)
            except Exception as e:
                self.get_logger().warn(f"Failed to init adaptive estimator for {cam_id}: {e}")
    
    def initialize_components(self):
        """Initialize processing components"""
        # Intersection engine (configurable sampling)
        iou_cfg = self.config.get('iou', {})
        self.intersection_engine = IntersectionEngine(
            method=self.iou_method,
            samples_per_axis=int(iou_cfg.get('samples_per_axis', 5)),
            voxel_size=float(iou_cfg.get('voxel_size', 0.2)),
            n_samples=int(iou_cfg.get('n_samples', 1000))
        )
        
        # Visualizer with configurable marker lifetime
        marker_lifetime = self.config.get('visualization', {}).get('marker_lifetime_sec', 0.1)
        self.visualizer = FrontierVisualizer(
            frame_id=self.config['frames']['lidar_frame'],
            marker_lifetime=marker_lifetime
        )
        
        # Storage for latest messages
        self.latest_yolo_detections = {}  # camera_id -> DetectionArray
        self.latest_lidar_detections = None  # BoundingBox3DArray
        
        # Statistics - per camera
        self.camera_stats = {}
        for cam_id in self.calibrations.keys():
            self.camera_stats[cam_id] = {
                'detections_received': 0,
                'matches': 0,
                'frustums_generated': 0,
                'last_received': None
            }
        self.total_matches = 0
        self.total_processed = 0
    
    def setup_ros_interfaces(self):
        """Setup ROS2 publishers and subscribers"""
        # QoS profile - UDP 스타일 빠른 전송 (실시간성 중시)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # UDP 방식
            history=HistoryPolicy.KEEP_LAST,
            depth=3  # 큐 크기 최소화로 메모리 절약
        )
        
        # Publishers - depth를 1로 줄여 메모리 절약
        self.fused_pub = self.create_publisher(
            Detection3DArray,
            self.config['fused_detections_topic'],
            1  # 버퍼 최소화
        )
        # Fused centers as TrackedConeArray for UKF
        self.fused_cones_pub = self.create_publisher(
            TrackedConeArray,
            self.config.get('fused_centers_topic', '/cone/fused'),
            10
        )
        
        if self.config['visualization']['publish_markers']:
            self.viz_pub = self.create_publisher(
                MarkerArray,
                self.config['visualization_topic'],
                1  # 버퍼 최소화
            )
        
        # Subscribers for each camera
        self.yolo_subs = {}
        for cam_config in self.config['cameras']:
            cam_id = cam_config['id']
            
            # YOLO detection subscriber
            self.yolo_subs[cam_id] = self.create_subscription(
                DetectionArray,
                cam_config['detections_topic'],
                lambda msg, cid=cam_id: self.yolo_callback(msg, cid),
                qos
            )
        
        # LiDAR detection subscriber
        self.lidar_sub = self.create_subscription(
            BoundingBox3DArray,
            self.config['lidar_detections_topic'],
            self.lidar_callback,
            qos
        )
        
        # Timer for processing - config에서 설정 가능 (기본 20Hz)
        viz_rate = self.config.get('visualization', {}).get('publish_rate_hz', 20.0)
        timer_period = 1.0 / viz_rate
        self.process_timer = self.create_timer(timer_period, self.process_detections)
        self.get_logger().info(f"Visualization timer set to {viz_rate}Hz (period: {timer_period:.3f}s)")
    
    def yolo_callback(self, msg: DetectionArray, camera_id: str):
        """Callback for YOLO 2D detections"""
        # 메모리 관리 - Python GC가 자동 처리
        
        # 탐지 개수 제한 체크 (fault 방지)
        if len(msg.detections) > 100:
            self.get_logger().warn(f"Too many detections ({len(msg.detections)}) from {camera_id}, limiting to 100")
            msg.detections = msg.detections[:100]
        
        self.latest_yolo_detections[camera_id] = msg
        
        # Update camera statistics
        if camera_id in self.camera_stats:
            self.camera_stats[camera_id]['detections_received'] += 1
            self.camera_stats[camera_id]['last_received'] = self.get_clock().now()
        
        # Enhanced logging for camera distinction
        self.get_logger().info(
            f"[{camera_id}] Received {len(msg.detections)} YOLO detections "
            f"(Total: {self.camera_stats[camera_id]['detections_received']})"
        )
        
        if self.config['debug']['log_level'] == 'debug':
            self.get_logger().debug(
                f"[{camera_id}] Detection details: {len(msg.detections)} objects"
            )
    
    def lidar_callback(self, msg: BoundingBox3DArray):
        """Callback for LiDAR 3D detections"""
        # 메모리 관리 - Python GC가 자동 처리
        
        # 탐지 개수 제한 체크 (fault 방지)
        if len(msg.boxes) > 200:
            self.get_logger().warn(f"Too many LiDAR detections ({len(msg.boxes)}), limiting to 200")
            msg.boxes = msg.boxes[:200]
        
        self.latest_lidar_detections = msg
        
        if self.config['debug']['log_level'] == 'debug':
            self.get_logger().debug(
                f"Received {len(msg.boxes)} LiDAR detections"
            )
    
    def process_detections(self):
        """Main processing function - match 2D and 3D detections"""
        # Check if we have YOLO data (LiDAR is optional for visualization)
        if not self.latest_yolo_detections:
            return
        
        # Process each camera
        all_matches = []
        all_frustums = []  # Store all frustums for visualization
        cameras_processed = []
        
        for cam_id, yolo_msg in self.latest_yolo_detections.items():
            if cam_id not in self.frustum_generators:
                self.get_logger().warn(f"No frustum generator for {cam_id}")
                continue
            
            # Check time synchronization only if we have LiDAR data
            if self.latest_lidar_detections is not None:
                time_diff = abs(
                    yolo_msg.header.stamp.sec - self.latest_lidar_detections.header.stamp.sec
                ) + abs(
                    yolo_msg.header.stamp.nanosec - self.latest_lidar_detections.header.stamp.nanosec
                ) * 1e-9
                
                if time_diff > self.config['sync']['slop_seconds']:
                    self.get_logger().debug(f"[{cam_id}] Time sync failed: {time_diff:.3f}s > {self.config['sync']['slop_seconds']}s")
                    continue
            
            # Generate frustums from YOLO detections
            frustums = self.generate_frustums(yolo_msg, cam_id)
            
            if frustums:
                self.camera_stats[cam_id]['frustums_generated'] += len(frustums)
                self.get_logger().info(
                    f"[{cam_id}] Generated {len(frustums)} frustums from {len(yolo_msg.detections)} detections"
                )
                
                # Store all frustums with camera ID and YOLO detection for visualization
                for i, frustum in enumerate(frustums):
                    all_frustums.append((cam_id, frustum, yolo_msg.detections[i] if i < len(yolo_msg.detections) else None))
            
            # Perform matching only if we have LiDAR data
            if self.latest_lidar_detections is not None:
                matches = self.match_frustums_to_boxes(
                    frustums,
                    self.latest_lidar_detections.boxes,
                    yolo_msg.detections,
                    cam_id
                )
                
                if matches:
                    self.camera_stats[cam_id]['matches'] += len(matches)
                    self.get_logger().info(
                        f"[{cam_id}] Found {len(matches)} matches with LiDAR boxes"
                    )
                
                all_matches.extend(matches)
            else:
                self.get_logger().debug(f"[{cam_id}] No LiDAR data for matching")
            
            cameras_processed.append(cam_id)
        
        # Log which cameras were processed
        if cameras_processed:
            self.get_logger().info(f"Processed cameras: {', '.join(cameras_processed)}")
        
        # Publish results and visualization
        if all_matches:
            self.publish_results(all_matches)
            
            # Update statistics
            self.total_matches += len(all_matches)
            self.total_processed += 1
            
            if self.config['debug']['print_matches']:
                # Print overall and per-camera statistics
                cam_summary = ", ".join(
                    f"{cam}: {self.camera_stats[cam]['matches']}"
                    for cam in self.camera_stats.keys()
                )
                self.get_logger().info(
                    f"Found {len(all_matches)} matches "
                    f"(Total: {self.total_matches}/{self.total_processed}) "
                    f"[{cam_summary}]"
                )
        else:
            # Debug: log when no matches found
            if all_frustums and self.latest_lidar_detections:
                self.get_logger().warn(
                    f"No matches found! Frustums: {len(all_frustums)}, "
                    f"LiDAR boxes: {len(self.latest_lidar_detections.boxes) if self.latest_lidar_detections else 0}"
                )
        
        # Always publish visualization if we have frustums (matched or not)
        if all_frustums and self.config['visualization']['publish_markers'] and hasattr(self, 'viz_pub') and self.viz_pub:
            self.publish_all_frustum_visualization(all_frustums, all_matches)
        
        # 메모리 최적화 - 처리 완료 후 즉시 참조 해제
        self.latest_lidar_detections = None
        self.latest_yolo_detections.clear()
    
    def generate_frustums(self, yolo_msg: DetectionArray, camera_id: str) -> List[Frustum]:
        """Generate 3D frustums from 2D YOLO detections"""
        generator = self.frustum_generators[camera_id]
        frustums = []
        
        # 처리할 detection 개수 제한 (실시간성 보장)
        max_detections = min(len(yolo_msg.detections), 
                           self.config['optimization']['max_frustums'])
        
        for i, detection in enumerate(yolo_msg.detections[:max_detections]):
            # Use yolo_msgs bbox directly - no conversion needed
            bbox = detection.bbox
            
            # Skip if bbox is invalid or too small/large (fault 방지)
            if (bbox.size.x <= 5 or bbox.size.y <= 5 or
                bbox.size.x > 1000 or bbox.size.y > 1000):
                continue
            
            # Skip if bbox is out of image bounds
            if (bbox.center.position.x < 0 or 
                bbox.center.position.y < 0):
                continue
            
            # Generate frustum
            try:
                if self.adaptive_enabled and camera_id in self.adaptive_estimators:
                    near_d, far_d = self.adaptive_estimators[camera_id].compute_near_far(bbox)
                else:
                    near_d, far_d = self.near_distance, self.far_distance
                frustum = generator.generate_frustum(
                    bbox,
                    float(near_d),
                    float(far_d)
                )
                frustums.append(frustum)
            except Exception as e:
                if i < 3:  # 처음 3개만 경고 출력 (로그 스팸 방지)
                    self.get_logger().warn(f"Failed to generate frustum: {e}")
        
        return frustums
    
    def match_frustums_to_boxes(self, 
                                frustums: List[Frustum],
                                lidar_boxes: List[BoundingBox3D],
                                yolo_detections: List,
                                camera_id: str) -> List[Tuple]:
        """Match frustums to 3D boxes using Hungarian algorithm"""
        if not frustums or not lidar_boxes:
            return []
        
        # Build cost matrix (1 - IoU)
        n_frustums = len(frustums)
        n_boxes = len(lidar_boxes)
        
        # Apply limits for performance
        if n_frustums > self.config['optimization']['max_frustums']:
            frustums = frustums[:self.config['optimization']['max_frustums']]
            n_frustums = len(frustums)
        
        if n_boxes > self.config['optimization']['max_boxes']:
            lidar_boxes = lidar_boxes[:self.config['optimization']['max_boxes']]
            n_boxes = len(lidar_boxes)
        
        # Debug: Log sample frustum and box coordinates
        if n_frustums > 0 and n_boxes > 0:
            # Log first frustum corners
            frustum_corners = frustums[0].corners
            frustum_min = np.min(frustum_corners, axis=0)
            frustum_max = np.max(frustum_corners, axis=0)
            
            # Log first LiDAR box
            box = lidar_boxes[0]
            box_center = np.array([
                box.center.position.x,
                box.center.position.y,
                box.center.position.z
            ])
            box_size = np.array([box.size.x, box.size.y, box.size.z])
            box_min = box_center - box_size/2
            box_max = box_center + box_size/2
            
            # Always log coordinate comparison for debugging
            self.get_logger().warn(
                f"[{camera_id}] COORDINATE DEBUG:\n"
                f"  Frustum[0] Z range: [{frustum_min[2]:.3f}, {frustum_max[2]:.3f}]\n"
                f"  LiDAR Box[0] Z range: [{box_min[2]:.3f}, {box_max[2]:.3f}]\n"
                f"  Frustum XY: [{frustum_min[0]:.1f}, {frustum_min[1]:.1f}] to [{frustum_max[0]:.1f}, {frustum_max[1]:.1f}]\n"
                f"  Box XY: [{box_min[0]:.1f}, {box_min[1]:.1f}] to [{box_max[0]:.1f}, {box_max[1]:.1f}]"
            )
        
        # Compute IoU matrix with early termination
        iou_matrix = np.zeros((n_frustums, n_boxes))
        
        # 계산 횟수 추적 (실시간성 보장)
        max_computations = 500  # 최대 계산 횟수 제한
        computation_count = 0
        
        for i, frustum in enumerate(frustums):
            for j, bbox_3d in enumerate(lidar_boxes):
                # 계산 횟수 제한 체크
                if computation_count >= max_computations:
                    self.get_logger().debug(f"Reached max computation limit ({max_computations})")
                    break
                
                # Distance gating
                if self.config['optimization']['use_distance_gating']:
                    box_center = np.array([
                        bbox_3d.center.position.x,
                        bbox_3d.center.position.y,
                        bbox_3d.center.position.z
                    ])
                    distance = np.linalg.norm(box_center)
                    
                    if distance > self.max_distance or distance < 0.5:  # 너무 가깝거나 먼 것 제외
                        iou_matrix[i, j] = 0.0
                        continue
                
                # Compute IoU
                try:
                    iou = self.intersection_engine.compute_iou_3d(frustum, bbox_3d)
                    iou_matrix[i, j] = iou
                    computation_count += 1
                except Exception as e:
                    iou_matrix[i, j] = 0.0
                    if computation_count < 3:
                        self.get_logger().warn(f"IoU computation failed: {e}")
        
        # Cost matrix (1 - IoU)
        cost_matrix = 1.0 - iou_matrix
        
        # Hungarian algorithm
        if self.config['matching']['method'] == 'hungarian':
            row_indices, col_indices = linear_sum_assignment(cost_matrix)
        else:
            # Greedy matching as fallback
            row_indices, col_indices = self.greedy_matching(cost_matrix)
        
        # Filter matches by IoU threshold
        matches = []
        # Debug: log max IoU values
        max_iou = np.max(iou_matrix) if iou_matrix.size > 0 else 0.0
        if max_iou > 0:
            self.get_logger().info(f"[{camera_id}] Max IoU: {max_iou:.4f}, Threshold: {self.iou_threshold}")
        else:
            self.get_logger().warn(f"[{camera_id}] All IoU values are 0! Check coordinate system alignment")
            
        for i, j in zip(row_indices, col_indices):
            if iou_matrix[i, j] >= self.iou_threshold:
                matches.append((
                    i,  # frustum index
                    j,  # box index
                    iou_matrix[i, j],  # IoU value
                    camera_id,
                    yolo_detections[i],  # Original YOLO detection
                    lidar_boxes[j]   # Original LiDAR box
                ))
        
        return matches
    
    def greedy_matching(self, cost_matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Simple greedy matching algorithm"""
        n_rows, n_cols = cost_matrix.shape
        row_indices = []
        col_indices = []
        used_cols = set()
        
        # Sort by cost
        sorted_indices = np.unravel_index(np.argsort(cost_matrix, axis=None), cost_matrix.shape)
        
        for i, j in zip(sorted_indices[0], sorted_indices[1]):
            if i not in row_indices and j not in used_cols:
                row_indices.append(i)
                col_indices.append(j)
                used_cols.add(j)
                
                if len(row_indices) >= min(n_rows, n_cols):
                    break
        
        return np.array(row_indices), np.array(col_indices)
    
    def publish_results(self, matches: List[Tuple]):
        """Publish matched detections and visualization"""
        # Create output message
        output_msg = Detection3DArray()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.header.frame_id = self.config['frames']['lidar_frame']
        
        # Add matched detections
        cones_msg = TrackedConeArray()
        cones_msg.header = output_msg.header
        for match in matches:
            _, _, iou, camera_id, yolo_det, lidar_box = match
            
            # Create enhanced detection with both 2D and 3D info (late fusion)
            fused_detection = Detection3D()
            fused_detection.header = output_msg.header
            fused_detection.bbox = lidar_box  # BoundingBox3D is directly the bbox
            
            # Create a simple result with IoU score as confidence
            from vision_msgs.msg import ObjectHypothesisWithPose
            result = ObjectHypothesisWithPose()
            result.score = iou
            # Try to get class from YOLO detection
            if yolo_det and hasattr(yolo_det, 'class_name'):
                result.hypothesis.class_id = yolo_det.class_name
            elif yolo_det and hasattr(yolo_det, 'class_id'):
                result.hypothesis.class_id = str(yolo_det.class_id)
            fused_detection.results = [result]
            
            # Best-effort: store camera id if message has an appropriate field
            try:
                if hasattr(fused_detection, 'id'):
                    # Some message variants have a string id
                    fused_detection.id = str(camera_id)
                elif hasattr(fused_detection, 'tracking_id'):
                    fused_detection.tracking_id = str(camera_id)
            except Exception:
                pass
            
            output_msg.detections.append(fused_detection)

            # Append TrackedCone for UKF
            cone = TrackedCone()
            cone.track_id = 0  # tracker can overwrite; we just fill position/color
            cone.position = Point(
                x=lidar_box.center.position.x,
                y=lidar_box.center.position.y,
                z=lidar_box.center.position.z
            )
            # Try to infer color from YOLO detection class_name
            try:
                raw_label = yolo_det.class_name if hasattr(yolo_det, 'class_name') else ""
            except Exception:
                raw_label = ""
            lbl = raw_label.lower()
            if "blue" in lbl:
                cone.color = "Blue cone"
            elif "yellow" in lbl:
                cone.color = "Yellow cone"
            elif "red" in lbl:
                cone.color = "Red cone"
            elif raw_label:
                # Fallback: use raw string
                cone.color = raw_label
            else:
                cone.color = "Unknown"
            cones_msg.cones.append(cone)
        
        # Publish fused detections
        self.fused_pub.publish(output_msg)
        # Publish fused cones
        if cones_msg.cones:
            self.fused_cones_pub.publish(cones_msg)
        
        # Publish visualization if enabled
        if self.config['visualization']['publish_markers'] and self.viz_pub:
            self.publish_visualization(matches)
    
    def publish_all_frustum_visualization(self, all_frustums: List[Tuple], matches: List[Tuple]):
        """Publish visualization markers for all frustums (matched and unmatched)"""
        # Create marker array
        marker_array = MarkerArray()
        
        # Reset visualizer ID counter
        self.visualizer.reset_id_counter()
        
        # Debug mode에서는 더 많이 시각화
        if self.config['debug']['log_level'] == 'debug':
            max_viz_frustums = min(len(all_frustums), 30)  # Debug: 최대 30개
        else:
            max_viz_frustums = min(len(all_frustums), 15)  # Normal: 최대 15개
        
        # Cone class-based colors (RGB values matching actual cone colors)
        cone_class_colors = {
            # Blue cone - pure blue
            'blue_cone': [0.0, 0.0, 1.0],
            'blue': [0.0, 0.0, 1.0],
            
            # Yellow cone - pure yellow  
            'yellow_cone': [1.0, 1.0, 0.0],
            'yellow': [1.0, 1.0, 0.0],
            
            # Red cone - pure red
            'red_cone': [1.0, 0.0, 0.0],
            'red': [1.0, 0.0, 0.0],
            
            # Unknown/default - gray
            'unknown': [0.5, 0.5, 0.5],
        }
        
        # Create a set of matched frustum indices for quick lookup
        matched_frustum_info = {}
        for match in matches:
            frustum_idx, box_idx, iou, camera_id, yolo_det, lidar_det = match
            key = (camera_id, frustum_idx)
            matched_frustum_info[key] = (iou, lidar_det)
        
        # Visualize all frustums
        for idx, (cam_id, frustum, yolo_det) in enumerate(all_frustums[:max_viz_frustums]):
            # Check if this frustum is matched
            is_matched = False
            iou = 0.0
            lidar_det = None
            
            # Find if this frustum has a match
            for i, (cam, f, y) in enumerate(all_frustums[:len(all_frustums)]):
                if cam == cam_id and i in [m[0] for m in matches if m[3] == cam_id]:
                    # This frustum is matched
                    for m in matches:
                        if m[3] == cam_id and m[0] == i:
                            is_matched = True
                            iou = m[2]
                            lidar_det = m[5]
                            break
                    if is_matched:
                        break
            
            # Get cone class color based on YOLO detection class_name
            base_color = cone_class_colors.get('unknown')  # default gray
            detected_class = 'unknown'
            if yolo_det and hasattr(yolo_det, 'class_name'):
                class_name = yolo_det.class_name.lower()
                detected_class = class_name
                # Try to find matching color for class
                for key in cone_class_colors:
                    if key in class_name or class_name in key:
                        base_color = cone_class_colors[key]
                        break
                
                # Log cone class detection (only first few to avoid spam)
                if idx < 3 and self.config['debug']['log_level'] == 'debug':
                    self.get_logger().debug(
                        f"[{cam_id}] Frustum {idx}: class='{class_name}' -> color={base_color}"
                    )
            
            # Adjust transparency based on match status
            if is_matched:
                # Matched: more opaque
                frustum_color = base_color + [0.6]
            else:
                # Unmatched: more transparent
                frustum_color = base_color + [0.3]
            
            # Create frustum wireframe marker
            frustum_marker = self.visualizer.create_frustum_marker(
                frustum,
                self.config['frames']['lidar_frame'],
                tuple(frustum_color),
                f"frustum_{cam_id}_{idx}"
            )
            marker_array.markers.append(frustum_marker)
            
            # Create frustum filled (semi-transparent) marker
            frustum_filled_marker = self.visualizer.create_frustum_filled_marker(
                frustum,
                self.config['frames']['lidar_frame'],
                tuple(frustum_color),
                f"frustum_filled_{cam_id}_{idx}"
            )
            marker_array.markers.append(frustum_filled_marker)
            
            # If matched, add box and connection line
            if is_matched and lidar_det:
                # Create box marker
                box_marker = self.visualizer.create_bbox_marker(
                    lidar_det,  # BoundingBox3D is directly the bbox
                    self.config['frames']['lidar_frame'],
                    tuple(self.config['visualization']['matched_box_color']),
                    f"box_{cam_id}_{idx}"
                )
                marker_array.markers.append(box_marker)
                
                # Create match line
                frustum_center = np.mean(frustum.corners, axis=0)
                box_center = np.array([
                    lidar_det.center.position.x,
                    lidar_det.center.position.y,
                    lidar_det.center.position.z
                ])
                
                line_marker = self.visualizer.create_match_line(
                    frustum_center,
                    box_center,
                    self.config['frames']['lidar_frame'],
                    f"match_line_{cam_id}_{idx}"
                )
                marker_array.markers.append(line_marker)
                
                # Create IoU text with camera ID and cone class
                text_position = (frustum_center + box_center) / 2.0
                # Include cone class in text if available
                text_content = f"{cam_id}\n{detected_class}\nIoU: {iou:.2f}"
                text_marker = self.visualizer.create_text_marker(
                    text_content,
                    text_position,
                    self.config['frames']['lidar_frame'],
                    self.config['visualization']['text_scale'],
                    f"iou_text_{cam_id}_{idx}"
                )
                marker_array.markers.append(text_marker)
        
        # Set timestamp for all markers
        now = self.get_clock().now().to_msg()
        for marker in marker_array.markers:
            marker.header.stamp = now
        
        # Publish
        self.viz_pub.publish(marker_array)
        self.get_logger().debug(
            f"Published {len(marker_array.markers)} visualization markers "
            f"({len(all_frustums)} frustums, {len(matches)} matches)"
        )
    
    def publish_visualization(self, matches: List[Tuple]):
        """Legacy visualization for matched objects only (kept for compatibility)"""
        # This method is now replaced by publish_all_frustum_visualization
        # but kept here in case it's called from elsewhere
        pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FrontierNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()