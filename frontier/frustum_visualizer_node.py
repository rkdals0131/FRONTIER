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


class FrustumVisualizerNode(Node):
    """Frustum 시각화 전용 노드"""
    
    def __init__(self):
        super().__init__('frustum_visualizer_node')
        
        # Parameters
        self.declare_parameter('config_file', 
            '/home/user1/ROS2_Workspace/ros2_ws/src/frontier/config/frontier_config.yaml')
        self.declare_parameter('camera_id', 'camera_1')
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
        self.camera_id = self.get_parameter('camera_id').value
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
        
        # Load calibration
        self.load_calibration()
        
        # Initialize components
        self.frustum_generator = FrustumGenerator(self.calibration)
        self.visualizer = FrontierVisualizer(frame_id='os_sensor')  # LiDAR frame
        
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
            self.get_logger().info("Using manual bounding box mode")
            self.timer = self.create_timer(
                1.0 / self.publish_rate, 
                self.manual_bbox_callback
            )
        else:
            # Subscribe to YOLO detections - use topic from config
            # Find the camera config in the loaded configuration
            yolo_topic = None
            if 'cameras' in self.config:
                for camera in self.config['cameras']:
                    if camera.get('id') == self.camera_id:
                        yolo_topic = camera.get('detections_topic')
                        break
            
            # Fallback to default pattern if not found in config
            if not yolo_topic:
                yolo_topic = f'/{self.camera_id}/detections'
                self.get_logger().warn(
                    f"Camera {self.camera_id} not found in config, using default topic: {yolo_topic}"
                )
            
            self.yolo_sub = self.create_subscription(
                DetectionArray,
                yolo_topic,
                self.yolo_callback,
                qos
            )
            self.get_logger().info(f"Subscribing to YOLO detections on: {yolo_topic}")
        
        # Statistics
        self.frame_count = 0
        self.total_frustums = 0
        
        # 마커 청소를 위한 타이머 (30초마다 DELETEALL 전송)
        self.cleanup_timer = self.create_timer(30.0, self.cleanup_markers)
        self.last_cleanup_time = 0
        
        self.get_logger().info(
            f"Frustum Visualizer initialized for {self.camera_id}\n"
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
    
    def load_calibration(self):
        """Load camera calibration"""
        config_folder = os.path.dirname(self.config_file)
        loader = CalibrationLoader(config_folder)
        
        # Load calibration files
        intrinsic_file = 'multi_camera_intrinsic_calibration.yaml'
        extrinsic_file = 'multi_camera_extrinsic_calibration.yaml'
        
        try:
            calibrations = loader.load_calibrations(intrinsic_file, extrinsic_file)
            
            if self.camera_id not in calibrations:
                available = ', '.join(calibrations.keys())
                self.get_logger().error(
                    f"Camera '{self.camera_id}' not found in calibration. "
                    f"Available cameras: {available}"
                )
                raise ValueError(f"Camera {self.camera_id} not found")
            
            self.calibration = calibrations[self.camera_id]
            self.get_logger().info(
                f"Loaded calibration for {self.camera_id}\n"
                f"  Image size: {self.calibration.image_width}x{self.calibration.image_height}\n"
                f"  Camera position in LiDAR: {self.calibration.get_camera_position_in_lidar()}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            raise
    
    def manual_bbox_callback(self):
        """Create frustum from manual bounding box parameters"""
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
        
        # Generate frustum
        frustum = self.frustum_generator.generate_frustum(
            bbox,
            self.near_distance,
            self.far_distance
        )
        
        # Visualize
        self.visualize_frustums([frustum], [bbox])
        
        self.frame_count += 1
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f"Manual frustum: center=({bbox.center.position.x:.0f}, "
                f"{bbox.center.position.y:.0f}), size=({bbox.size.x:.0f}x{bbox.size.y:.0f})"
            )
    
    def yolo_callback(self, msg: DetectionArray):
        """Callback for YOLO detections with safety mechanisms"""
        self.get_logger().debug(f"YOLO callback triggered with {len(msg.detections)} detections")
        
        if not msg.detections:
            return
        
        # Check if processing is in progress
        if self.is_processing:
            self.dropped_messages += 1
            if self.dropped_messages % 10 == 0:
                self.get_logger().warn(f"Dropped {self.dropped_messages} messages due to processing lag")
            return
        
        # Check message age - use_sim_time이 false일 때만 체크
        if self.drop_old_messages and not self.use_sim_time:
            current_time = self.get_clock().now()
            msg_time = Time.from_msg(msg.header.stamp)
            age = (current_time - msg_time).nanoseconds / 1e9
            
            if abs(age) > self.message_age_threshold:
                self.get_logger().debug(f"Dropping old message (age: {age:.2f}s)")
                self.dropped_messages += 1
                return
        elif self.use_sim_time:
            # 시뮬레이션 시간 사용 시 메시지 age 체크 생략
            self.get_logger().debug("Using sim_time, skipping message age check")
        
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
            # Limit number of detections to process
            detections_to_process = msg.detections[:self.max_frustums_per_frame]
            
            if len(msg.detections) > self.max_frustums_per_frame:
                self.get_logger().debug(
                    f"Limiting detections from {len(msg.detections)} to {self.max_frustums_per_frame}"
                )
            
            # Generate frustums for limited detections
            frustums = []
            bboxes = []
            
            for detection in detections_to_process:
                # Use yolo_msgs bbox directly - no conversion needed
                bbox = detection.bbox
                
                # Skip invalid bboxes
                if bbox.size.x <= 0 or bbox.size.y <= 0:
                    continue
                
                # Check if bbox is within image bounds
                if (bbox.center.position.x < 0 or 
                    bbox.center.position.x >= self.calibration.image_width or
                    bbox.center.position.y < 0 or 
                    bbox.center.position.y >= self.calibration.image_height):
                    continue
                
                try:
                    # Generate frustum
                    frustum = self.frustum_generator.generate_frustum(
                        bbox,
                        self.near_distance,
                        self.far_distance
                    )
                    frustums.append(frustum)
                    bboxes.append(bbox)
                    
                except Exception as e:
                    self.get_logger().warn(f"Failed to generate frustum: {e}")
            
            # Visualize all frustums
            if frustums:
                self.get_logger().debug(f"Generated {len(frustums)} frustums, visualizing...")
                self.visualize_frustums(frustums, bboxes)
                self.total_frustums += len(frustums)
                self.frame_count += 1
                
                # Log statistics periodically
                if self.frame_count % 10 == 0:
                    avg_frustums = self.total_frustums / self.frame_count
                    self.get_logger().info(
                        f"Stats: {self.frame_count} frames, "
                        f"{self.total_frustums} total frustums, "
                        f"{avg_frustums:.1f} avg frustums/frame, "
                        f"dropped: {self.dropped_messages}"
                    )
            else:
                self.get_logger().debug("No frustums generated from detections")
            
            # Update last process time
            self.last_process_time = time.time()
            
        finally:
            # Always release processing flag
            with self.processing_lock:
                self.is_processing = False
    
    def visualize_frustums(self, frustums: List[Frustum], bboxes: List[BoundingBox2D]):
        """Create and publish visualization markers with safety limits"""
        marker_array = MarkerArray()
        
        # Reset visualizer ID counter
        self.visualizer.reset_id_counter()
        
        # Limit number of frustums to visualize
        frustums_to_viz = frustums[:self.max_frustums_per_frame]
        bboxes_to_viz = bboxes[:self.max_frustums_per_frame]
        
        # Create markers for each frustum
        for i, (frustum, bbox) in enumerate(zip(frustums_to_viz, bboxes_to_viz)):
            # Frustum color based on index
            colors = [
                (1.0, 0.0, 0.0, 0.4),  # Red
                (0.0, 1.0, 0.0, 0.4),  # Green
                (0.0, 0.0, 1.0, 0.4),  # Blue
                (1.0, 1.0, 0.0, 0.4),  # Yellow
                (1.0, 0.0, 1.0, 0.4),  # Magenta
                (0.0, 1.0, 1.0, 0.4),  # Cyan
            ]
            color = colors[i % len(colors)]
            
            # Create frustum wireframe
            frustum_marker = self.visualizer.create_frustum_marker(
                frustum,
                'os_sensor',
                color,
                f"frustum_{i}"
            )
            marker_array.markers.append(frustum_marker)
            
            # Add camera center marker
            camera_marker = self.create_camera_marker(frustum.camera_center, i)
            marker_array.markers.append(camera_marker)
            
            # Add text label with bbox info
            text_pos = np.mean(frustum.corners[:4], axis=0)  # Near plane center
            text_marker = self.create_info_text(
                bbox, text_pos, i
            )
            marker_array.markers.append(text_marker)
            
            # Add coordinate axes at frustum origin
            axes_markers = self.create_axes_markers(frustum.camera_center, i)
            marker_array.markers.extend(axes_markers)
        
        # Set timestamp
        now = self.get_clock().now().to_msg()
        for marker in marker_array.markers:
            marker.header.stamp = now
        
        # Publish
        self.marker_pub.publish(marker_array)
    
    def create_camera_marker(self, position: np.ndarray, index: int) -> Marker:
        """Create a sphere marker for camera position"""
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.ns = f"camera_{index}"
        marker.id = self.visualizer._get_next_id()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)  # Orange
        
        marker.lifetime = Duration(sec=0, nanosec=100000000)  # 0.1초
        
        return marker
    
    def create_info_text(self, bbox: BoundingBox2D, position: np.ndarray, index: int) -> Marker:
        """Create text marker with bbox information"""
        marker = Marker()
        marker.header.frame_id = 'os_sensor'
        marker.ns = f"info_{index}"
        marker.id = self.visualizer._get_next_id()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + 0.3
        
        marker.scale.z = 0.2
        
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        
        # Text with bbox info (yolo_msgs structure)
        marker.text = (
            f"Frustum {index}\n"
            f"2D: ({bbox.center.position.x:.0f}, {bbox.center.position.y:.0f})\n"
            f"Size: {bbox.size.x:.0f}x{bbox.size.y:.0f}"
        )
        
        marker.lifetime = Duration(sec=0, nanosec=100000000)  # 0.1초
        
        return marker
    
    def create_axes_markers(self, origin: np.ndarray, index: int) -> List[Marker]:
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
            marker.ns = f"axes_{index}"
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