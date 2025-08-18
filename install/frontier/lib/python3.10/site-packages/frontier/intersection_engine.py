#!/usr/bin/env python3
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from vision_msgs.msg import BoundingBox3D
from .frustum_generator import Frustum, Plane


@dataclass
class AABB:
    """Axis-Aligned Bounding Box"""
    min_point: np.ndarray
    max_point: np.ndarray
    
    def get_center(self) -> np.ndarray:
        """박스 중심점 반환"""
        return (self.min_point + self.max_point) / 2.0
    
    def get_size(self) -> np.ndarray:
        """박스 크기 반환"""
        return self.max_point - self.min_point
    
    def get_volume(self) -> float:
        """박스 부피 반환"""
        size = self.get_size()
        return size[0] * size[1] * size[2]
    
    def get_corners(self) -> np.ndarray:
        """8개 코너 포인트 반환"""
        x_min, y_min, z_min = self.min_point
        x_max, y_max, z_max = self.max_point
        
        corners = np.array([
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_max, y_max, z_min],
            [x_min, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_max, y_max, z_max],
            [x_min, y_max, z_max]
        ])
        return corners
    
    def contains_point(self, point: np.ndarray) -> bool:
        """점이 박스 내부에 있는지 확인"""
        return np.all(point >= self.min_point) and np.all(point <= self.max_point)


class IntersectionEngine:
    """3D Frustum과 3D Bounding Box 간의 교차 계산 엔진"""
    
    def __init__(self,
                 method: str = 'point_in_frustum',
                 seed: Optional[int] = None,
                 samples_per_axis: int = 5,
                 voxel_size: float = 0.2,
                 n_samples: int = 1000):
        """
        Args:
            method: 교차 계산 방법 ('point_in_frustum', 'voxel', 'sampling')
            seed: 난수 생성기 시드 (재현 가능한 결과를 위해)
            samples_per_axis: point_in_frustum 방식에서 축당 샘플 수
            voxel_size: voxel 방식에서 복셀 크기 (m)
            n_samples: sampling 방식에서 무작위 샘플 개수
        """
        self.method = method
        self._rng = np.random.default_rng(seed)  # 재현 가능한 난수 생성
        self.samples_per_axis = max(2, int(samples_per_axis))
        self.voxel_size = float(voxel_size)
        self.n_samples = int(n_samples)
        
    def compute_iou_3d(self, frustum: Frustum, bbox_3d: BoundingBox3D) -> float:
        """
        3D IoU (Intersection over Union) 계산
        
        Args:
            frustum: 3D View Frustum
            bbox_3d: 3D Bounding Box (ROS 메시지)
            
        Returns:
            IoU 값 (0.0 ~ 1.0)
        """
        # BoundingBox3D를 AABB로 변환
        aabb = self._bbox3d_to_aabb(bbox_3d)
        
        if self.method == 'point_in_frustum':
            return self._point_in_frustum_iou(frustum, aabb)
        elif self.method == 'voxel':
            return self._voxel_based_iou(frustum, aabb)
        elif self.method == 'sampling':
            return self._sample_based_iou(frustum, aabb)
        else:
            raise ValueError(f"Unknown method: {self.method}")
    
    def point_in_frustum(self, point: np.ndarray, frustum: Frustum) -> bool:
        """
        점이 frustum 내부에 있는지 검사
        
        Args:
            point: 3D 점
            frustum: View Frustum
            
        Returns:
            내부에 있으면 True
        """
        return frustum.contains_point(point)
    
    def aabb_frustum_overlap(self, aabb: AABB, frustum: Frustum) -> bool:
        """
        AABB와 Frustum이 겹치는지 빠른 검사
        
        Args:
            aabb: Axis-Aligned Bounding Box
            frustum: View Frustum
            
        Returns:
            겹치면 True
        """
        # 간단한 검사: AABB의 코너 중 하나라도 frustum 내부에 있는지
        corners = aabb.get_corners()
        for corner in corners:
            if frustum.contains_point(corner):
                return True
        
        # Frustum의 코너 중 하나라도 AABB 내부에 있는지
        for corner in frustum.corners:
            if aabb.contains_point(corner):
                return True
        
        # SAT (Separating Axis Theorem) 기반 정밀 검사도 가능하지만
        # PoC에서는 위의 간단한 검사만 수행
        return False
    
    def _point_in_frustum_iou(self, frustum: Frustum, aabb: AABB) -> float:
        """
        Point-in-Frustum 방식으로 IoU 근사
        
        박스 내부의 점들을 샘플링하여 frustum 내부에 있는 비율을 계산
        """
        # 빠른 겹침 검사
        if not self.aabb_frustum_overlap(aabb, frustum):
            return 0.0
        
        # 박스 내부 점 샘플링 (configurable)
        samples = self._generate_aabb_samples(aabb, self.samples_per_axis)
        
        # Frustum 내부 점 개수 계산
        inside_count = sum(1 for point in samples if frustum.contains_point(point))
        
        # 근사 IoU 계산
        # 주의: 이것은 정확한 IoU가 아니라 근사값
        overlap_ratio = inside_count / len(samples)
        
        # 보정 계수 적용 (실험적으로 조정 필요)
        # 박스가 frustum 내부에 완전히 포함되면 1.0에 가깝게
        # 부분적으로 겹치면 실제 IoU에 근사하도록
        if overlap_ratio > 0.8:
            return overlap_ratio
        else:
            return overlap_ratio * 0.7  # 부분 겹침에 대한 보정
    
    def _voxel_based_iou(self, frustum: Frustum, aabb: AABB, voxel_size: float = None) -> float:
        """
        복셀 기반 IoU 계산
        
        공간을 복셀로 나누고 각 복셀이 frustum과 aabb에 포함되는지 검사
        """
        # 빠른 겹침 검사
        if not self.aabb_frustum_overlap(aabb, frustum):
            return 0.0
        
        # 관심 영역 계산 (AABB와 frustum 바운딩 박스의 교집합)
        roi_min, roi_max = self._compute_roi(aabb, frustum)
        
        # 복셀 그리드 생성
        voxels = self._generate_voxel_grid(roi_min, roi_max, self.voxel_size if voxel_size is None else voxel_size)
        
        # 각 복셀에 대해 포함 여부 검사
        aabb_voxels = set()
        frustum_voxels = set()
        
        for i, voxel_center in enumerate(voxels):
            if aabb.contains_point(voxel_center):
                aabb_voxels.add(i)
            if frustum.contains_point(voxel_center):
                frustum_voxels.add(i)
        
        # IoU 계산
        intersection = len(aabb_voxels & frustum_voxels)
        union = len(aabb_voxels | frustum_voxels)
        
        if union == 0:
            return 0.0
        
        return intersection / union
    
    def _sample_based_iou(self, frustum: Frustum, aabb: AABB, n_samples: Optional[int] = None) -> float:
        """
        몬테카를로 샘플링 기반 IoU 계산
        
        무작위 점을 생성하여 IoU 추정
        """
        # 빠른 겹침 검사
        if not self.aabb_frustum_overlap(aabb, frustum):
            return 0.0
        
        # 관심 영역 계산
        roi_min, roi_max = self._compute_roi(aabb, frustum)
        
        # 하드캡 적용 - 너무 많은 샘플 방지
        MAX_SAMPLES = 5000
        n = self.n_samples if n_samples is None else n_samples
        n_samples = min(n, MAX_SAMPLES)
        
        # 무작위 점 생성 (시드된 난수 생성기 사용)
        samples = self._rng.uniform(roi_min, roi_max, (n_samples, 3))
        
        # 각 점에 대해 포함 여부 검사
        in_aabb = 0
        in_frustum = 0
        in_both = 0
        
        for point in samples:
            is_in_aabb = aabb.contains_point(point)
            is_in_frustum = frustum.contains_point(point)
            
            if is_in_aabb:
                in_aabb += 1
            if is_in_frustum:
                in_frustum += 1
            if is_in_aabb and is_in_frustum:
                in_both += 1
        
        # IoU 계산
        union = in_aabb + in_frustum - in_both
        if union == 0:
            return 0.0
        
        return in_both / union
    
    def _bbox3d_to_aabb(self, bbox_3d: BoundingBox3D) -> AABB:
        """
        ROS BoundingBox3D 메시지를 AABB로 변환
        
        Args:
            bbox_3d: ROS BoundingBox3D 메시지
            
        Returns:
            AABB 객체
        """
        center = np.array([
            bbox_3d.center.position.x,
            bbox_3d.center.position.y,
            bbox_3d.center.position.z
        ])
        
        half_size = np.array([
            bbox_3d.size.x / 2.0,
            bbox_3d.size.y / 2.0,
            bbox_3d.size.z / 2.0
        ])
        
        # 회전은 일단 무시 (AABB 가정)
        # TODO: Oriented Bounding Box 지원 추가
        
        min_point = center - half_size
        max_point = center + half_size
        
        return AABB(min_point, max_point)
    
    def _generate_aabb_samples(self, aabb: AABB, n_samples_per_axis: int) -> List[np.ndarray]:
        """
        AABB 내부에 균등하게 분포된 샘플 점 생성
        
        Args:
            aabb: Axis-Aligned Bounding Box
            n_samples_per_axis: 각 축당 샘플 개수
            
        Returns:
            샘플 점들의 리스트
        """
        size = aabb.get_size()
        min_pt = aabb.min_point
        
        samples = []
        for i in range(n_samples_per_axis):
            for j in range(n_samples_per_axis):
                for k in range(n_samples_per_axis):
                    x = min_pt[0] + (i + 0.5) * size[0] / n_samples_per_axis
                    y = min_pt[1] + (j + 0.5) * size[1] / n_samples_per_axis
                    z = min_pt[2] + (k + 0.5) * size[2] / n_samples_per_axis
                    samples.append(np.array([x, y, z]))
        
        return samples
    
    def _compute_roi(self, aabb: AABB, frustum: Frustum) -> Tuple[np.ndarray, np.ndarray]:
        """
        관심 영역 (Region of Interest) 계산
        
        AABB와 frustum의 바운딩 박스 교집합
        """
        # Frustum의 바운딩 박스 계산
        frustum_min = np.min(frustum.corners, axis=0)
        frustum_max = np.max(frustum.corners, axis=0)
        
        # 교집합 계산
        roi_min = np.maximum(aabb.min_point, frustum_min)
        roi_max = np.minimum(aabb.max_point, frustum_max)
        
        # 유효성 검사 (교집합이 없는 경우)
        if np.any(roi_min > roi_max):
            # 교집합이 없으면 작은 영역 반환
            center = (aabb.get_center() + frustum.camera_center) / 2.0
            roi_min = center - 0.1
            roi_max = center + 0.1
        
        return roi_min, roi_max
    
    def _generate_voxel_grid(self, min_point: np.ndarray, 
                           max_point: np.ndarray, 
                           voxel_size: float) -> np.ndarray:
        """
        복셀 그리드 생성 (NumPy 벡터화 버전)
        
        Args:
            min_point: 그리드 최소 좌표
            max_point: 그리드 최대 좌표
            voxel_size: 복셀 크기
            
        Returns:
            복셀 중심점들의 NumPy 배열 (N x 3)
        """
        # 각 축별 복셀 개수 계산
        size = max_point - min_point
        n_voxels = np.ceil(size / voxel_size).astype(int)
        
        # 하드캡 적용 - 너무 많은 복셀 생성 방지
        MAX_VOXELS_PER_AXIS = 50  # 축당 최대 50개
        n_voxels = np.minimum(n_voxels, MAX_VOXELS_PER_AXIS)
        
        # 벡터화된 그리드 생성 - 메모리 효율적
        if n_voxels[0] * n_voxels[1] * n_voxels[2] > 0:
            xs = np.linspace(min_point[0] + 0.5*voxel_size, 
                           max_point[0] - 0.5*voxel_size, 
                           n_voxels[0])
            ys = np.linspace(min_point[1] + 0.5*voxel_size, 
                           max_point[1] - 0.5*voxel_size, 
                           n_voxels[1])
            zs = np.linspace(min_point[2] + 0.5*voxel_size, 
                           max_point[2] - 0.5*voxel_size, 
                           n_voxels[2])
            
            # meshgrid로 효율적인 3D 그리드 생성
            grid = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), axis=-1)
            return grid.reshape(-1, 3)
        else:
            return np.array([])
    
    def compute_batch_iou(self, frustums: List[Frustum], 
                         boxes: List[BoundingBox3D]) -> np.ndarray:
        """
        여러 frustum과 box 쌍에 대한 IoU 행렬 계산
        
        Args:
            frustums: Frustum 리스트
            boxes: BoundingBox3D 리스트
            
        Returns:
            IoU 행렬 (N x M)
        """
        n_frustums = len(frustums)
        n_boxes = len(boxes)
        iou_matrix = np.zeros((n_frustums, n_boxes))
        
        for i, frustum in enumerate(frustums):
            for j, box in enumerate(boxes):
                iou_matrix[i, j] = self.compute_iou_3d(frustum, box)
        
        return iou_matrix