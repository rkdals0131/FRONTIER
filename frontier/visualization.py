#!/usr/bin/env python3
import numpy as np
from typing import List, Tuple, Optional
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from vision_msgs.msg import BoundingBox3D
from builtin_interfaces.msg import Duration
from .frustum_generator import Frustum
from .intersection_engine import AABB


class FrontierVisualizer:
    """RViz2를 위한 FRONTIER 시각화 도구"""
    
    def __init__(self, frame_id: str = "base_link", marker_lifetime: float = 0.1):
        """
        Args:
            frame_id: 기본 프레임 ID
            marker_lifetime: 마커 유지 시간 (초)
        """
        self.frame_id = frame_id
        self.marker_id_counter = 0
        self.marker_lifetime = marker_lifetime
        
        # 기본 색상 정의
        self.colors = {
            'frustum': (0.0, 1.0, 0.0, 0.3),      # 녹색 반투명
            'lidar_box': (0.0, 0.0, 1.0, 0.5),    # 파란색 반투명
            'matched_frustum': (1.0, 1.0, 0.0, 0.5),  # 노란색
            'matched_box': (1.0, 0.5, 0.0, 0.7),      # 주황색
            'match_line': (1.0, 1.0, 0.0, 0.8),       # 노란색
            'text': (1.0, 1.0, 1.0, 1.0)              # 흰색
        }
    
    def create_frustum_marker(self, 
                            frustum: Frustum,
                            frame_id: Optional[str] = None,
                            color: Optional[Tuple[float, float, float, float]] = None,
                            namespace: str = "frustum") -> Marker:
        """
        Frustum을 선 리스트 마커로 생성
        
        Args:
            frustum: View Frustum 객체
            frame_id: 프레임 ID (None이면 기본값 사용)
            color: RGBA 색상 (None이면 기본 frustum 색상 사용)
            namespace: 마커 네임스페이스
            
        Returns:
            Frustum 와이어프레임 마커
        """
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.header.stamp.sec = 0  # 타임스탬프는 노드에서 설정
        marker.header.stamp.nanosec = 0
        marker.ns = namespace
        marker.id = self._get_next_id()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # 스케일 설정 (선 두께)
        marker.scale.x = 0.02  # 2cm 두께
        
        # 색상 설정
        if color:
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        else:
            c = self.colors['frustum']
            marker.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
        
        # Frustum 엣지 추가
        corners = frustum.corners
        
        # Near 평면 엣지 (0-1-2-3-0)
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Near 평면
            (4, 5), (5, 6), (6, 7), (7, 4),  # Far 평면
            (0, 4), (1, 5), (2, 6), (3, 7)   # 연결선
        ]
        
        for i, j in edges:
            p1 = Point(x=corners[i][0], y=corners[i][1], z=corners[i][2])
            p2 = Point(x=corners[j][0], y=corners[j][1], z=corners[j][2])
            marker.points.append(p1)
            marker.points.append(p2)
        
        # 카메라 중심에서 near 평면 코너로의 선 추가
        camera_pt = Point(x=frustum.camera_center[0], 
                         y=frustum.camera_center[1], 
                         z=frustum.camera_center[2])
        for i in range(4):
            marker.points.append(camera_pt)
            marker.points.append(Point(x=corners[i][0], y=corners[i][1], z=corners[i][2]))
        
        # Convert seconds to nanoseconds for Duration
        lifetime_ns = int(self.marker_lifetime * 1e9)
        marker.lifetime = Duration(sec=int(self.marker_lifetime), nanosec=lifetime_ns % int(1e9))
        
        return marker
    
    def create_frustum_filled_marker(self, 
                                   frustum: Frustum,
                                   frame_id: Optional[str] = None,
                                   color: Optional[Tuple[float, float, float, float]] = None,
                                   namespace: str = "frustum_filled") -> Marker:
        """
        Frustum을 반투명 면으로 채운 마커 생성
        
        Args:
            frustum: View Frustum 객체
            frame_id: 프레임 ID (None이면 기본값 사용)
            color: RGBA 색상 (None이면 기본 frustum 색상 사용)
            namespace: 마커 네임스페이스
            
        Returns:
            Frustum 채워진 면 마커
        """
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = namespace
        marker.id = self._get_next_id()
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # 스케일 설정 (TRIANGLE_LIST는 scale 사용 안 함)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # 색상 설정 (더 투명하게)
        if color:
            # 면은 와이어프레임보다 더 투명하게
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3] * 0.3)
        else:
            c = self.colors['frustum']
            marker.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3] * 0.3)
        
        # Frustum 코너 가져오기
        corners = frustum.corners
        
        # 6개 면을 삼각형으로 정의
        # 각 면은 2개의 삼각형으로 구성
        faces = [
            # Near 평면 (0-1-2-3)
            [0, 1, 2], [0, 2, 3],
            # Far 평면 (4-5-6-7)
            [4, 6, 5], [4, 7, 6],
            # Top 면 (2-3-7-6)
            [2, 6, 7], [2, 7, 3],
            # Bottom 면 (0-1-5-4)
            [0, 4, 5], [0, 5, 1],
            # Left 면 (0-3-7-4)
            [0, 3, 7], [0, 7, 4],
            # Right 면 (1-2-6-5)
            [1, 5, 6], [1, 6, 2]
        ]
        
        # 삼각형 점들 추가
        for face in faces:
            for idx in face:
                marker.points.append(Point(
                    x=corners[idx][0],
                    y=corners[idx][1],
                    z=corners[idx][2]
                ))
        
        # Convert seconds to nanoseconds for Duration
        lifetime_ns = int(self.marker_lifetime * 1e9)
        marker.lifetime = Duration(sec=int(self.marker_lifetime), nanosec=lifetime_ns % int(1e9))
        
        return marker
    
    def create_bbox_marker(self, 
                          bbox: BoundingBox3D,
                          frame_id: Optional[str] = None,
                          color: Optional[Tuple[float, float, float, float]] = None,
                          namespace: str = "bbox") -> Marker:
        """
        3D 바운딩 박스를 큐브 마커로 생성
        
        Args:
            bbox: 3D 바운딩 박스
            frame_id: 프레임 ID
            color: RGBA 색상
            namespace: 마커 네임스페이스
            
        Returns:
            3D 박스 마커
        """
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.header.stamp.sec = 0  # 타임스탬프는 노드에서 설정
        marker.header.stamp.nanosec = 0
        marker.ns = namespace
        marker.id = self._get_next_id()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 위치 설정
        marker.pose.position.x = bbox.center.position.x
        marker.pose.position.y = bbox.center.position.y
        marker.pose.position.z = bbox.center.position.z
        marker.pose.orientation = bbox.center.orientation
        
        # 크기 설정
        marker.scale.x = bbox.size.x
        marker.scale.y = bbox.size.y
        marker.scale.z = bbox.size.z
        
        # 색상 설정
        if color:
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        else:
            c = self.colors['lidar_box']
            marker.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
        
        # Convert seconds to nanoseconds for Duration
        lifetime_ns = int(self.marker_lifetime * 1e9)
        marker.lifetime = Duration(sec=int(self.marker_lifetime), nanosec=lifetime_ns % int(1e9))
        
        return marker
    
    def create_match_line(self, 
                         frustum_center: np.ndarray,
                         bbox_center: np.ndarray,
                         frame_id: Optional[str] = None,
                         namespace: str = "match_lines") -> Marker:
        """
        매칭된 frustum과 box를 연결하는 선 생성
        
        Args:
            frustum_center: Frustum 중심점
            bbox_center: Box 중심점
            frame_id: 프레임 ID
            namespace: 마커 네임스페이스
            
        Returns:
            연결선 마커
        """
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.header.stamp.sec = 0  # 타임스탬프는 노드에서 설정
        marker.header.stamp.nanosec = 0
        marker.ns = namespace
        marker.id = self._get_next_id()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 선 두께
        marker.scale.x = 0.03
        
        # 색상
        c = self.colors['match_line']
        marker.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
        
        # 점 추가
        p1 = Point(x=frustum_center[0], y=frustum_center[1], z=frustum_center[2])
        p2 = Point(x=bbox_center[0], y=bbox_center[1], z=bbox_center[2])
        marker.points.append(p1)
        marker.points.append(p2)
        
        # Convert seconds to nanoseconds for Duration
        lifetime_ns = int(self.marker_lifetime * 1e9)
        marker.lifetime = Duration(sec=int(self.marker_lifetime), nanosec=lifetime_ns % int(1e9))
        
        return marker
    
    def create_text_marker(self, 
                          text: str,
                          position: np.ndarray,
                          frame_id: Optional[str] = None,
                          scale: float = 0.2,
                          namespace: str = "text") -> Marker:
        """
        텍스트 마커 생성 (IoU 값 표시 등)
        
        Args:
            text: 표시할 텍스트
            position: 텍스트 위치
            frame_id: 프레임 ID
            scale: 텍스트 크기
            namespace: 마커 네임스페이스
            
        Returns:
            텍스트 마커
        """
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.header.stamp.sec = 0  # 타임스탬프는 노드에서 설정
        marker.header.stamp.nanosec = 0
        marker.ns = namespace
        marker.id = self._get_next_id()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 위치
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + 0.5  # 위쪽에 표시
        
        # 크기
        marker.scale.z = scale
        
        # 색상
        c = self.colors['text']
        marker.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
        
        # 텍스트
        marker.text = text
        
        # Convert seconds to nanoseconds for Duration
        lifetime_ns = int(self.marker_lifetime * 1e9)
        marker.lifetime = Duration(sec=int(self.marker_lifetime), nanosec=lifetime_ns % int(1e9))
        
        return marker
    
    def create_full_visualization(self,
                                 frustums: List[Frustum],
                                 boxes: List[BoundingBox3D],
                                 matches: List[Tuple[int, int, float]],
                                 frame_id: Optional[str] = None) -> MarkerArray:
        """
        전체 시각화 마커 배열 생성
        
        Args:
            frustums: Frustum 리스트
            boxes: 3D 박스 리스트
            matches: 매칭 결과 [(frustum_idx, box_idx, iou), ...]
            frame_id: 프레임 ID
            
        Returns:
            전체 시각화를 위한 MarkerArray
        """
        marker_array = MarkerArray()
        
        # 매칭된 인덱스 추적
        matched_frustums = set()
        matched_boxes = set()
        for f_idx, b_idx, _ in matches:
            matched_frustums.add(f_idx)
            matched_boxes.add(b_idx)
        
        # Frustum 시각화
        for i, frustum in enumerate(frustums):
            if i in matched_frustums:
                color = self.colors['matched_frustum']
            else:
                color = self.colors['frustum']
            
            marker = self.create_frustum_marker(
                frustum, frame_id, color, f"frustum_{i}"
            )
            marker_array.markers.append(marker)
        
        # Box 시각화
        for i, box in enumerate(boxes):
            if i in matched_boxes:
                color = self.colors['matched_box']
            else:
                color = self.colors['lidar_box']
            
            marker = self.create_bbox_marker(
                box, frame_id, color, f"box_{i}"
            )
            marker_array.markers.append(marker)
        
        # 매칭 선 및 IoU 텍스트
        for f_idx, b_idx, iou in matches:
            # Frustum과 Box 중심 계산
            frustum_center = np.mean(frustums[f_idx].corners, axis=0)
            box_center = np.array([
                boxes[b_idx].center.position.x,
                boxes[b_idx].center.position.y,
                boxes[b_idx].center.position.z
            ])
            
            # 연결선
            line_marker = self.create_match_line(
                frustum_center, box_center, frame_id, f"match_line_{f_idx}_{b_idx}"
            )
            marker_array.markers.append(line_marker)
            
            # IoU 텍스트
            text_position = (frustum_center + box_center) / 2.0
            text_marker = self.create_text_marker(
                f"IoU: {iou:.2f}",
                text_position,
                frame_id,
                0.15,
                f"iou_text_{f_idx}_{b_idx}"
            )
            marker_array.markers.append(text_marker)
        
        return marker_array
    
    def create_aabb_marker(self,
                          aabb: AABB,
                          frame_id: Optional[str] = None,
                          color: Optional[Tuple[float, float, float, float]] = None,
                          namespace: str = "aabb") -> Marker:
        """
        AABB를 와이어프레임 박스로 시각화
        
        Args:
            aabb: Axis-Aligned Bounding Box
            frame_id: 프레임 ID
            color: RGBA 색상
            namespace: 마커 네임스페이스
            
        Returns:
            AABB 와이어프레임 마커
        """
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.header.stamp.sec = 0  # 타임스탬프는 노드에서 설정
        marker.header.stamp.nanosec = 0
        marker.ns = namespace
        marker.id = self._get_next_id()
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        # 선 두께
        marker.scale.x = 0.02
        
        # 색상
        if color:
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        else:
            c = self.colors['lidar_box']
            marker.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
        
        # AABB 코너 가져오기
        corners = aabb.get_corners()
        
        # 박스 엣지 정의
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # 바닥면
            (4, 5), (5, 6), (6, 7), (7, 4),  # 윗면
            (0, 4), (1, 5), (2, 6), (3, 7)   # 수직 엣지
        ]
        
        for i, j in edges:
            p1 = Point(x=corners[i][0], y=corners[i][1], z=corners[i][2])
            p2 = Point(x=corners[j][0], y=corners[j][1], z=corners[j][2])
            marker.points.append(p1)
            marker.points.append(p2)
        
        # Convert seconds to nanoseconds for Duration
        lifetime_ns = int(self.marker_lifetime * 1e9)
        marker.lifetime = Duration(sec=int(self.marker_lifetime), nanosec=lifetime_ns % int(1e9))
        
        return marker
    
    def clear_all_markers(self, frame_id: Optional[str] = None) -> MarkerArray:
        """
        모든 마커 제거를 위한 MarkerArray 생성
        
        Args:
            frame_id: 프레임 ID
            
        Returns:
            DELETE_ALL 액션을 가진 MarkerArray
        """
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = frame_id or self.frame_id
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.action = Marker.DELETEALL
        
        marker_array.markers.append(marker)
        
        return marker_array
    
    def _get_next_id(self) -> int:
        """다음 마커 ID 반환"""
        self.marker_id_counter += 1
        return self.marker_id_counter
    
    def reset_id_counter(self):
        """ID 카운터 리셋"""
        self.marker_id_counter = 0