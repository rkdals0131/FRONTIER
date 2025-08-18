#!/usr/bin/env python3
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from sensor_msgs.msg import CameraInfo
from yolo_msgs.msg import BoundingBox2D
from .calibration_loader import CameraCalibration


@dataclass
class Plane:
    """3D 평면을 나타내는 클래스 (ax + by + cz + d = 0)"""
    a: float
    b: float
    c: float
    d: float
    
    def distance_to_point(self, point: np.ndarray) -> float:
        """점과 평면 사이의 거리 계산"""
        return abs(self.a * point[0] + self.b * point[1] + self.c * point[2] + self.d) / \
               np.sqrt(self.a**2 + self.b**2 + self.c**2)
    
    def is_point_inside(self, point: np.ndarray) -> bool:
        """점이 평면의 음의 방향(내부)에 있는지 확인"""
        return (self.a * point[0] + self.b * point[1] + self.c * point[2] + self.d) <= 0


@dataclass
class Frustum:
    """3D View Frustum을 나타내는 클래스 (라이다 좌표계)"""
    planes: List[Plane]  # 6개 평면 (near, far, top, bottom, left, right)
    corners: np.ndarray  # 8개 코너 포인트 (near 4개, far 4개) - 라이다 좌표계
    camera_center: np.ndarray  # 카메라 중심 위치 - 라이다 좌표계
    near_distance: float
    far_distance: float
    
    def contains_point(self, point: np.ndarray) -> bool:
        """점이 frustum 내부에 있는지 확인 (라이다 좌표계)"""
        # 가장 제한적인 평면(near/far)부터 체크하여 조기 종료
        # Near plane (index 0)
        if not self.planes[0].is_point_inside(point):
            return False
        # Far plane (index 1)  
        if not self.planes[1].is_point_inside(point):
            return False
        # 나머지 평면들
        for plane in self.planes[2:]:
            if not plane.is_point_inside(point):
                return False
        return True
    
    def get_volume(self) -> float:
        """Frustum의 부피 계산 (절두체 공식)"""
        # 절두체(frustum) 부피 = h/3 * (A1 + A2 + sqrt(A1*A2))
        # A1: near 평면 면적, A2: far 평면 면적, h: 높이
        near_area = self._compute_quad_area(self.corners[:4])
        far_area = self._compute_quad_area(self.corners[4:])
        
        # 높이는 near와 far 중심 간 거리
        near_center = np.mean(self.corners[:4], axis=0)
        far_center = np.mean(self.corners[4:], axis=0)
        height = np.linalg.norm(far_center - near_center)
        
        # 절두체 부피 공식
        volume = height / 3.0 * (near_area + far_area + np.sqrt(near_area * far_area))
        return volume
    
    def _compute_quad_area(self, corners: np.ndarray) -> float:
        """4개 코너로 이루어진 사각형의 면적 계산 (삼각형 분할)"""
        # 두 삼각형으로 분할하여 면적 계산
        # 삼각형 1: corners[0], corners[1], corners[2]
        # 삼각형 2: corners[0], corners[2], corners[3]
        
        # 첫 번째 삼각형 면적
        v1 = corners[1] - corners[0]
        v2 = corners[2] - corners[0]
        area1 = 0.5 * np.linalg.norm(np.cross(v1, v2))
        
        # 두 번째 삼각형 면적
        v3 = corners[2] - corners[0]
        v4 = corners[3] - corners[0]
        area2 = 0.5 * np.linalg.norm(np.cross(v3, v4))
        
        return area1 + area2


class FrustumGenerator:
    """캘리브레이션 파라미터와 2D 바운딩 박스로부터 3D frustum을 생성하는 클래스"""
    
    def __init__(self, calibration: Optional[CameraCalibration] = None):
        """
        Args:
            calibration: 카메라 캘리브레이션 객체
        """
        self.calibration = calibration
        if calibration:
            self.camera_center_lidar = calibration.get_camera_position_in_lidar()
        else:
            self.camera_center_lidar = np.array([0.0, 0.0, 0.0])
        
    def set_calibration(self, calibration: CameraCalibration):
        """캘리브레이션 정보 업데이트"""
        self.calibration = calibration
        self.camera_center_lidar = calibration.get_camera_position_in_lidar()
        
    def generate_frustum(self, 
                        bbox_2d: BoundingBox2D,
                        near_dist: float = 0.5,
                        far_dist: float = 100.0) -> Frustum:
        """
        2D 바운딩 박스로부터 3D frustum 생성 (센서 좌표계로 변환)
        
        Args:
            bbox_2d: 2D 바운딩 박스 (center.position.x/y, size_x, size_y)
            near_dist: Near 평면까지의 거리 (카메라 좌표계 기준)
            far_dist: Far 평면까지의 거리 (카메라 좌표계 기준)
            
        Returns:
            생성된 Frustum 객체 (os_sensor 좌표계)
        """
        if self.calibration is None:
            raise ValueError("캘리브레이션 정보가 설정되지 않았습니다.")
        
        # 2D 바운딩 박스의 4개 코너 계산
        corners_2d = self._get_bbox_corners(bbox_2d)
        
        # 각 코너를 라이다 좌표계의 3D 광선으로 변환
        rays_lidar = []
        for corner in corners_2d:
            try:
                # 픽셀 → 카메라 좌표계 광선
                ray_cam = self.calibration.unproject_2d_to_ray(corner[0], corner[1])
                # 카메라 좌표계 → 라이다 좌표계 광선
                ray_lidar = self.calibration.transform_ray_to_lidar(ray_cam)
                
                # NaN/Inf 체크 (fault 방지)
                if np.any(np.isnan(ray_lidar)) or np.any(np.isinf(ray_lidar)):
                    ray_lidar = np.array([0.0, 0.0, 1.0])  # 기본 방향
                    
                rays_lidar.append(ray_lidar)
            except Exception as e:
                # 실패 시 기본 광선 사용
                rays_lidar.append(np.array([0.0, 0.0, 1.0]))
        
        # Near와 Far 평면에서의 3D 코너 계산 (라이다 좌표계)
        corners_3d_lidar = []
        for ray in rays_lidar:
            # Near 평면 교점
            near_point = self.camera_center_lidar + ray * near_dist
            corners_3d_lidar.append(near_point)
        for ray in rays_lidar:
            # Far 평면 교점
            far_point = self.camera_center_lidar + ray * far_dist
            corners_3d_lidar.append(far_point)
        
        # os_lidar → os_sensor 좌표계 변환
        # os_sensor는 전방을 보고, os_lidar는 후방을 보므로 180도 회전
        corners_3d_sensor = []
        for corner_lidar in corners_3d_lidar:
            corner_sensor = self.calibration.transform_to_sensor_frame(corner_lidar)
            corners_3d_sensor.append(corner_sensor)
        
        corners_3d_sensor = np.array(corners_3d_sensor)
        
        # 카메라 중심도 센서 좌표계로 변환
        camera_center_sensor = self.calibration.get_camera_position_in_sensor()
        
        # 광선도 센서 좌표계로 변환 (방향 벡터이므로 회전만 적용)
        rays_sensor = []
        for ray_lidar in rays_lidar:
            # 방향 벡터는 X, Y 반전만 적용 (Z축은 그대로)
            ray_sensor = np.array([-ray_lidar[0], -ray_lidar[1], ray_lidar[2]])
            ray_sensor = ray_sensor / np.linalg.norm(ray_sensor)
            rays_sensor.append(ray_sensor)
        
        # 6개 평면 계산 (센서 좌표계)
        planes = self.compute_frustum_planes(corners_3d_sensor, rays_sensor, camera_center_sensor)
        
        return Frustum(
            planes=planes,
            corners=corners_3d_sensor,
            camera_center=camera_center_sensor,
            near_distance=near_dist,
            far_distance=far_dist
        )
    
    def compute_frustum_planes(self, 
                              corners: np.ndarray,
                              rays: List[np.ndarray],
                              camera_center: np.ndarray = None) -> List[Plane]:
        """
        Frustum을 구성하는 6개 평면 계산 (센서 좌표계)
        
        Args:
            corners: 8개의 3D 코너 포인트 (near 4개, far 4개) - 센서 좌표계
            rays: 4개의 광선 벡터 - 센서 좌표계
            camera_center: 카메라 중심 위치 - 센서 좌표계 (None이면 self.camera_center_lidar 사용)
            
        Returns:
            6개 평면의 리스트 [near, far, top, bottom, left, right]
        """
        planes = []
        
        # 카메라 중심 설정
        if camera_center is None:
            camera_center = self.camera_center_lidar
        
        # Frustum 내부 참조점 (near와 far 중간)
        frustum_center = np.mean(corners, axis=0)
        
        # Near 평면: near 코너들로 정의 (법선이 카메라 반대 방향)
        near_plane = self._plane_from_points(corners[0], corners[1], corners[2], 
                                            reference_point=camera_center)
        planes.append(near_plane)
        
        # Far 평면: far 코너들로 정의 (법선이 카메라 방향)
        far_center = np.mean(corners[4:], axis=0)
        far_plane = self._plane_from_points(corners[4], corners[5], corners[6],
                                           reference_point=far_center + rays[0] * 10)  # frustum 밖의 점
        planes.append(far_plane)
        
        # 측면 평면들 (top, bottom, left, right)
        # 각 평면의 법선이 frustum 내부를 향하도록 설정
        
        # Top 평면: camera_center, near_top_left, near_top_right
        bottom_center = (corners[2] + corners[3] + corners[6] + corners[7]) / 4
        planes.append(self._plane_from_points(
            camera_center, corners[0], corners[1],
            reference_point=bottom_center  # 아래쪽 중심을 참조점으로
        ))
        
        # Bottom 평면: camera_center, near_bottom_right, near_bottom_left
        top_center = (corners[0] + corners[1] + corners[4] + corners[5]) / 4
        planes.append(self._plane_from_points(
            camera_center, corners[2], corners[3],
            reference_point=top_center  # 위쪽 중심을 참조점으로
        ))
        
        # Left 평면: camera_center, near_bottom_left, near_top_left
        right_center = (corners[1] + corners[2] + corners[5] + corners[6]) / 4
        planes.append(self._plane_from_points(
            camera_center, corners[3], corners[0],
            reference_point=right_center  # 오른쪽 중심을 참조점으로
        ))
        
        # Right 평면: camera_center, near_top_right, near_bottom_right
        left_center = (corners[0] + corners[3] + corners[4] + corners[7]) / 4
        planes.append(self._plane_from_points(
            camera_center, corners[1], corners[2],
            reference_point=left_center  # 왼쪽 중심을 참조점으로
        ))
        
        return planes
    
    def _get_bbox_corners(self, bbox: BoundingBox2D) -> List[Tuple[float, float]]:
        """
        2D 바운딩 박스의 4개 코너 좌표 계산
        
        Args:
            bbox: 2D 바운딩 박스 (yolo_msgs)
            
        Returns:
            4개 코너의 (u, v) 좌표 리스트 [top-left, top-right, bottom-right, bottom-left]
        """
        center_x = bbox.center.position.x
        center_y = bbox.center.position.y
        half_width = bbox.size.x / 2.0
        half_height = bbox.size.y / 2.0
        
        corners = [
            (center_x - half_width, center_y - half_height),  # top-left
            (center_x + half_width, center_y - half_height),  # top-right
            (center_x + half_width, center_y + half_height),  # bottom-right
            (center_x - half_width, center_y + half_height),  # bottom-left
        ]
        
        # 이미지 경계 내로 클리핑 (fault 방지)
        if self.calibration:
            # 이미지 크기 체크
            if not hasattr(self.calibration, 'image_width') or self.calibration.image_width <= 0:
                self.calibration.image_width = 640  # 기본값
            if not hasattr(self.calibration, 'image_height') or self.calibration.image_height <= 0:
                self.calibration.image_height = 480  # 기본값
                
            corners = [(max(0, min(self.calibration.image_width - 1, u)),
                       max(0, min(self.calibration.image_height - 1, v))) for u, v in corners]
        
        return corners
    
    def _plane_from_points(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, 
                          reference_point: np.ndarray = None) -> Plane:
        """
        3개의 점으로부터 평면 방정식 계산
        법선이 reference_point 반대 방향을 향하도록 표준화
        
        Args:
            p1, p2, p3: 평면을 정의하는 3개 점
            reference_point: 평면의 '외부'를 정의하는 참조점 (None이면 frustum 중심 사용)
            
        Returns:
            평면 객체 (법선이 내부를 향함)
        """
        # 두 벡터 계산
        v1 = p2 - p1
        v2 = p3 - p1
        
        # 외적으로 법선 벡터 계산
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)
        
        # 평면 방정식: ax + by + cz + d = 0
        a, b, c = normal
        d = -np.dot(normal, p1)
        
        # 법선 방향 검증 및 수정
        if reference_point is not None:
            # reference_point가 평면의 양의 방향에 있으면 법선 뒤집기
            test_value = a * reference_point[0] + b * reference_point[1] + c * reference_point[2] + d
            if test_value > 0:
                a, b, c, d = -a, -b, -c, -d
        
        return Plane(a, b, c, d)
    
    def create_test_frustum(self) -> Frustum:
        """테스트용 기본 frustum 생성"""
        from .calibration_loader import CalibrationLoader
        
        # 실제 캘리브레이션 파일 로드
        loader = CalibrationLoader()
        calibrations = loader.load_calibrations(
            "multi_camera_intrinsic_calibration.yaml",
            "multi_camera_extrinsic_calibration.yaml"
        )
        
        # camera_1 사용
        if 'camera_1' in calibrations:
            self.set_calibration(calibrations['camera_1'])
        else:
            raise ValueError("camera_1 캘리브레이션을 찾을 수 없습니다.")
        
        # 테스트 바운딩 박스 (이미지 중앙)
        test_bbox = BoundingBox2D()
        test_bbox.center.position.x = 320
        test_bbox.center.position.y = 180
        test_bbox.size_x = 100
        test_bbox.size_y = 100
        
        return self.generate_frustum(test_bbox, near_dist=1.0, far_dist=50.0)