#!/usr/bin/env python3
"""
FRONTIER ROS2 Node
Frustum-based 3D object matching between camera and LiDAR detections
"""

import os
import yaml
import numpy as np
from typing import Dict, List, Tuple, Optional
from scipy.optimize import linear_sum_assignment

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D
from yolo_msgs.msg import DetectionArray, BoundingBox2D
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .calibration_loader import CalibrationLoader, CameraCalibration
from .frustum_generator import FrustumGenerator, Frustum
from .intersection_engine import IntersectionEngine
from .visualization import FrontierVisualizer


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
        # Get config file path from parameter or default
        config_file = self.declare_parameter('config_file', 
            '/home/user1/ROS2_Workspace/ros2_ws/src/frontier/config/frontier_config.yaml').value
        
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
        for cam_id in self.calibrations.keys():
            self.frustum_generators[cam_id] = FrustumGenerator(self.calibrations[cam_id])
            self.get_logger().info(f"Loaded calibration for {cam_id}")
    
    def initialize_components(self):
        """Initialize processing components"""
        # Intersection engine
        self.intersection_engine = IntersectionEngine(method=self.iou_method)
        
        # Visualizer
        self.visualizer = FrontierVisualizer(frame_id=self.config['frames']['lidar_frame'])
        
        # Storage for latest messages
        self.latest_yolo_detections = {}  # camera_id -> DetectionArray
        self.latest_lidar_detections = None  # Detection3DArray
        
        # Statistics
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
            Detection3DArray,
            self.config['lidar_detections_topic'],
            self.lidar_callback,
            qos
        )
        
        # Timer for processing - 10Hz로 감소하여 CPU 부하 절감
        self.process_timer = self.create_timer(0.1, self.process_detections)  # 10Hz
    
    def yolo_callback(self, msg: DetectionArray, camera_id: str):
        """Callback for YOLO 2D detections"""
        # 메모리 관리 - Python GC가 자동 처리
        
        # 탐지 개수 제한 체크 (fault 방지)
        if len(msg.detections) > 100:
            self.get_logger().warn(f"Too many detections ({len(msg.detections)}) from {camera_id}, limiting to 100")
            msg.detections = msg.detections[:100]
        
        self.latest_yolo_detections[camera_id] = msg
        
        if self.config['debug']['log_level'] == 'debug':
            self.get_logger().debug(
                f"Received {len(msg.detections)} YOLO detections from {camera_id}"
            )
    
    def lidar_callback(self, msg: Detection3DArray):
        """Callback for LiDAR 3D detections"""
        # 메모리 관리 - Python GC가 자동 처리
        
        # 탐지 개수 제한 체크 (fault 방지)
        if len(msg.detections) > 200:
            self.get_logger().warn(f"Too many LiDAR detections ({len(msg.detections)}), limiting to 200")
            msg.detections = msg.detections[:200]
        
        self.latest_lidar_detections = msg
        
        if self.config['debug']['log_level'] == 'debug':
            self.get_logger().debug(
                f"Received {len(msg.detections)} LiDAR detections"
            )
    
    def process_detections(self):
        """Main processing function - match 2D and 3D detections"""
        # Check if we have data
        if self.latest_lidar_detections is None:
            return
        
        if not self.latest_yolo_detections:
            return
        
        # Process each camera
        all_matches = []
        
        for cam_id, yolo_msg in self.latest_yolo_detections.items():
            if cam_id not in self.frustum_generators:
                continue
            
            # Check time synchronization
            time_diff = abs(
                yolo_msg.header.stamp.sec - self.latest_lidar_detections.header.stamp.sec
            ) + abs(
                yolo_msg.header.stamp.nanosec - self.latest_lidar_detections.header.stamp.nanosec
            ) * 1e-9
            
            if time_diff > self.config['sync']['slop_seconds']:
                continue
            
            # Generate frustums from YOLO detections
            frustums = self.generate_frustums(yolo_msg, cam_id)
            
            # Perform matching
            matches = self.match_frustums_to_boxes(
                frustums,
                self.latest_lidar_detections.detections,
                yolo_msg.detections,
                cam_id
            )
            
            all_matches.extend(matches)
        
        # Publish results
        if all_matches:
            self.publish_results(all_matches)
            
            # Update statistics
            self.total_matches += len(all_matches)
            self.total_processed += 1
            
            if self.config['debug']['print_matches']:
                self.get_logger().info(
                    f"Found {len(all_matches)} matches "
                    f"(Total: {self.total_matches}/{self.total_processed})"
                )
        
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
                frustum = generator.generate_frustum(
                    bbox,
                    self.near_distance,
                    self.far_distance
                )
                frustums.append(frustum)
            except Exception as e:
                if i < 3:  # 처음 3개만 경고 출력 (로그 스팸 방지)
                    self.get_logger().warn(f"Failed to generate frustum: {e}")
        
        return frustums
    
    def match_frustums_to_boxes(self, 
                                frustums: List[Frustum],
                                lidar_detections: List[Detection3D],
                                yolo_detections: List,
                                camera_id: str) -> List[Tuple]:
        """Match frustums to 3D boxes using Hungarian algorithm"""
        if not frustums or not lidar_detections:
            return []
        
        # Build cost matrix (1 - IoU)
        n_frustums = len(frustums)
        n_boxes = len(lidar_detections)
        
        # Apply limits for performance
        if n_frustums > self.config['optimization']['max_frustums']:
            frustums = frustums[:self.config['optimization']['max_frustums']]
            n_frustums = len(frustums)
        
        if n_boxes > self.config['optimization']['max_boxes']:
            lidar_detections = lidar_detections[:self.config['optimization']['max_boxes']]
            n_boxes = len(lidar_detections)
        
        # Compute IoU matrix with early termination
        iou_matrix = np.zeros((n_frustums, n_boxes))
        
        # 계산 횟수 추적 (실시간성 보장)
        max_computations = 500  # 최대 계산 횟수 제한
        computation_count = 0
        
        for i, frustum in enumerate(frustums):
            for j, detection_3d in enumerate(lidar_detections):
                # 계산 횟수 제한 체크
                if computation_count >= max_computations:
                    self.get_logger().debug(f"Reached max computation limit ({max_computations})")
                    break
                
                # Distance gating
                if self.config['optimization']['use_distance_gating']:
                    box_center = np.array([
                        detection_3d.bbox.center.position.x,
                        detection_3d.bbox.center.position.y,
                        detection_3d.bbox.center.position.z
                    ])
                    distance = np.linalg.norm(box_center)
                    
                    if distance > self.max_distance or distance < 0.5:  # 너무 가깝거나 먼 것 제외
                        iou_matrix[i, j] = 0.0
                        continue
                
                # Compute IoU
                try:
                    iou = self.intersection_engine.compute_iou_3d(frustum, detection_3d.bbox)
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
        for i, j in zip(row_indices, col_indices):
            if iou_matrix[i, j] >= self.iou_threshold:
                matches.append((
                    i,  # frustum index
                    j,  # box index
                    iou_matrix[i, j],  # IoU value
                    camera_id,
                    yolo_detections[i],  # Original YOLO detection
                    lidar_detections[j]   # Original LiDAR detection
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
        for match in matches:
            _, _, iou, camera_id, yolo_det, lidar_det = match
            
            # Create enhanced detection with both 2D and 3D info
            fused_detection = Detection3D()
            fused_detection.header = output_msg.header
            fused_detection.bbox = lidar_det.bbox
            fused_detection.results = lidar_det.results
            
            # Add IoU score as confidence
            if fused_detection.results:
                fused_detection.results[0].score = iou
            
            # Store camera ID in tracking ID (temporary solution)
            fused_detection.tracking_id = camera_id
            
            output_msg.detections.append(fused_detection)
        
        # Publish fused detections
        self.fused_pub.publish(output_msg)
        
        # Publish visualization if enabled
        if self.config['visualization']['publish_markers'] and self.viz_pub:
            self.publish_visualization(matches)
    
    def publish_visualization(self, matches: List[Tuple]):
        """Publish visualization markers"""
        # Create marker array
        marker_array = MarkerArray()
        
        # Reset visualizer ID counter
        self.visualizer.reset_id_counter()
        
        # 매치 개수 제한 (시각화 성능)
        max_viz_matches = min(len(matches), 10)  # 최대 10개만 시각화
        
        # Add markers for each match
        for i, match in enumerate(matches[:max_viz_matches]):
            frustum_idx, box_idx, iou, camera_id, yolo_det, lidar_det = match
            
            # Skip visualization for low IoU matches
            if iou < 0.2:  # 낮은 IoU는 시각화 생략
                continue
            
            try:
                # Get frustum (재사용 대신 필요시에만 생성)
                generator = self.frustum_generators[camera_id]
                frustum = generator.generate_frustum(
                    yolo_det.bbox,
                    self.near_distance,
                    self.far_distance
                )
            
                # Create frustum marker
                frustum_marker = self.visualizer.create_frustum_marker(
                    frustum,
                    self.config['frames']['lidar_frame'],
                    tuple(self.config['visualization']['matched_frustum_color']),
                    f"frustum_{camera_id}_{frustum_idx}"
                )
                marker_array.markers.append(frustum_marker)
                
                # Create box marker
                box_marker = self.visualizer.create_bbox_marker(
                    lidar_det.bbox,
                    self.config['frames']['lidar_frame'],
                    tuple(self.config['visualization']['matched_box_color']),
                    f"box_{box_idx}"
                )
                marker_array.markers.append(box_marker)
                
                # Create match line
                frustum_center = np.mean(frustum.corners, axis=0)
                box_center = np.array([
                    lidar_det.bbox.center.position.x,
                    lidar_det.bbox.center.position.y,
                    lidar_det.bbox.center.position.z
                ])
                
                line_marker = self.visualizer.create_match_line(
                    frustum_center,
                    box_center,
                    self.config['frames']['lidar_frame'],
                    f"match_line_{frustum_idx}_{box_idx}"
                )
                marker_array.markers.append(line_marker)
                
                # Create IoU text
                text_position = (frustum_center + box_center) / 2.0
                text_marker = self.visualizer.create_text_marker(
                    f"IoU: {iou:.2f}",
                    text_position,
                    self.config['frames']['lidar_frame'],
                    self.config['visualization']['text_scale'],
                    f"iou_text_{frustum_idx}_{box_idx}"
                )
                marker_array.markers.append(text_marker)
            
            except Exception as e:
                # 시각화 실패 시 계속 진행
                if i < 3:  # 처음 3개만 경고
                    self.get_logger().warn(f"Visualization failed for match {i}: {e}")
                continue
        
        # Set timestamp for all markers
        now = self.get_clock().now().to_msg()
        for marker in marker_array.markers:
            marker.header.stamp = now
        
        # Publish
        self.viz_pub.publish(marker_array)


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