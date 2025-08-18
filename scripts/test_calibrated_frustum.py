#!/usr/bin/env python3
"""
캘리브레이션 기반 Frustum 생성 및 교차 테스트
실제 캘리브레이션 파라미터를 사용한 정확한 frustum 생성
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from frontier.calibration_loader import CalibrationLoader, CameraCalibration
from frontier.frustum_generator import FrustumGenerator, Frustum
from frontier.intersection_engine import IntersectionEngine, AABB

from vision_msgs.msg import BoundingBox2D, BoundingBox3D


def create_test_bbox_2d(center_x, center_y, width, height):
    """테스트용 2D 바운딩 박스 생성"""
    bbox = BoundingBox2D()
    bbox.center.position.x = center_x
    bbox.center.position.y = center_y
    bbox.size_x = width
    bbox.size_y = height
    return bbox


def create_test_bbox_3d(x, y, z, size_x, size_y, size_z):
    """테스트용 3D 바운딩 박스 생성 (라이다 좌표계)"""
    bbox = BoundingBox3D()
    bbox.center.position.x = x
    bbox.center.position.y = y
    bbox.center.position.z = z
    bbox.center.orientation.x = 0
    bbox.center.orientation.y = 0
    bbox.center.orientation.z = 0
    bbox.center.orientation.w = 1
    bbox.size.x = size_x
    bbox.size.y = size_y
    bbox.size.z = size_z
    return bbox


def visualize_frustum_3d(frustum: Frustum, aabbs: list = None, title: str = "Frustum Visualization"):
    """3D frustum 시각화"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Frustum 그리기
    frustum_corners = frustum.corners
    
    # Frustum 엣지 그리기
    edges = [
        # Near 평면
        [0, 1], [1, 2], [2, 3], [3, 0],
        # Far 평면
        [4, 5], [5, 6], [6, 7], [7, 4],
        # 연결선
        [0, 4], [1, 5], [2, 6], [3, 7]
    ]
    
    for edge in edges:
        points = frustum_corners[edge]
        ax.plot3D(*points.T, 'g-', linewidth=2)
    
    # 카메라 위치에서 near 코너로의 선
    camera_pos = frustum.camera_center
    for i in range(4):
        ax.plot3D([camera_pos[0], frustum_corners[i][0]],
                 [camera_pos[1], frustum_corners[i][1]],
                 [camera_pos[2], frustum_corners[i][2]],
                 'g--', alpha=0.5)
    
    # 카메라 위치 표시
    ax.scatter([camera_pos[0]], [camera_pos[1]], [camera_pos[2]], 
              color='red', s=200, marker='o', label='Camera')
    
    # AABB 그리기
    if aabbs:
        for i, aabb in enumerate(aabbs):
            box_corners = aabb.get_corners()
            
            # Box 엣지
            box_edges = [
                [0, 1], [1, 2], [2, 3], [3, 0],
                [4, 5], [5, 6], [6, 7], [7, 4],
                [0, 4], [1, 5], [2, 6], [3, 7]
            ]
            
            for edge in box_edges:
                points = box_corners[edge]
                ax.plot3D(*points.T, 'b-', linewidth=1, alpha=0.7)
            
            # Box 중심 표시
            center = aabb.get_center()
            ax.scatter([center[0]], [center[1]], [center[2]], 
                      color='blue', s=50, alpha=0.7)
    
    # 좌표축 설정
    ax.set_xlabel('X (전방)', fontsize=12)
    ax.set_ylabel('Y (좌측)', fontsize=12)
    ax.set_zlabel('Z (상방)', fontsize=12)
    ax.set_title(title, fontsize=14)
    
    # 뷰 각도 설정 (라이다 관점)
    ax.view_init(elev=20, azim=45)
    
    # 축 범위 자동 조정
    all_points = [frustum_corners, camera_pos.reshape(1, -1)]
    if aabbs:
        for aabb in aabbs:
            all_points.append(aabb.get_corners())
    
    all_points = np.vstack(all_points)
    min_vals = np.min(all_points, axis=0)
    max_vals = np.max(all_points, axis=0)
    
    # 균등한 스케일 설정
    max_range = np.max(max_vals - min_vals)
    mid_vals = (max_vals + min_vals) / 2
    
    ax.set_xlim([mid_vals[0] - max_range/2, mid_vals[0] + max_range/2])
    ax.set_ylim([mid_vals[1] - max_range/2, mid_vals[1] + max_range/2])
    ax.set_zlim([mid_vals[2] - max_range/2, mid_vals[2] + max_range/2])
    
    ax.legend()
    plt.tight_layout()
    plt.show()


def test_calibration_loading():
    """캘리브레이션 로딩 테스트"""
    print("=" * 60)
    print("캘리브레이션 파일 로딩 테스트")
    print("=" * 60)
    
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    
    for cam_id, calib in calibrations.items():
        print(f"\n{cam_id}:")
        print(f"  - 이미지 크기: {calib.image_width}x{calib.image_height}")
        print(f"  - 카메라 행렬 (fx, fy, cx, cy):")
        print(f"    fx={calib.camera_matrix[0,0]:.2f}, fy={calib.camera_matrix[1,1]:.2f}")
        print(f"    cx={calib.camera_matrix[0,2]:.2f}, cy={calib.camera_matrix[1,2]:.2f}")
        print(f"  - 왜곡 계수: {calib.dist_coeffs.shape}")
        print(f"  - 라이다 좌표계 카메라 위치: {calib.get_camera_position_in_lidar()}")
    
    return calibrations


def test_projection():
    """투영 테스트"""
    print("\n" + "=" * 60)
    print("3D → 2D 투영 테스트")
    print("=" * 60)
    
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    calib = calibrations['camera_1']
    
    # 테스트 3D 점들 (라이다 좌표계)
    test_points = [
        np.array([5, 0, 0]),     # 전방 5m
        np.array([10, -2, 0]),    # 전방 10m, 우측 2m
        np.array([15, 2, -1]),    # 전방 15m, 좌측 2m, 아래 1m
        np.array([3, 0, 1]),      # 전방 3m, 위 1m
    ]
    
    print("3D 점 (라이다) → 2D 픽셀:")
    for point_3d in test_points:
        point_2d = calib.project_3d_to_2d(point_3d)
        print(f"  {point_3d} → ({point_2d[0]:.1f}, {point_2d[1]:.1f})")


def test_frustum_generation():
    """Frustum 생성 테스트"""
    print("\n" + "=" * 60)
    print("캘리브레이션 기반 Frustum 생성 테스트")
    print("=" * 60)
    
    # 캘리브레이션 로드
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    
    # Camera 1 사용
    calib = calibrations['camera_1']
    generator = FrustumGenerator(calib)
    
    # 다양한 위치의 바운딩 박스 테스트
    test_cases = [
        (320, 180, 100, 100, "중앙"),
        (100, 100, 80, 80, "좌상단"),
        (540, 280, 80, 80, "우하단"),
        (320, 50, 150, 50, "상단 와이드"),
    ]
    
    frustums = []
    for x, y, w, h, desc in test_cases:
        bbox_2d = create_test_bbox_2d(x, y, w, h)
        frustum = generator.generate_frustum(bbox_2d, near_dist=2.0, far_dist=20.0)
        frustums.append(frustum)
        
        print(f"\n{desc} 바운딩 박스 ({x}, {y}, {w}x{h}):")
        print(f"  - 카메라 위치: {frustum.camera_center}")
        print(f"  - Near 코너 중심: {np.mean(frustum.corners[:4], axis=0)}")
        print(f"  - Far 코너 중심: {np.mean(frustum.corners[4:], axis=0)}")
    
    # 첫 번째 frustum 시각화
    visualize_frustum_3d(frustums[0], title="중앙 Frustum (라이다 좌표계)")
    
    return frustums


def test_frustum_box_intersection():
    """Frustum-Box 교차 테스트"""
    print("\n" + "=" * 60)
    print("Frustum-Box 교차 계산 테스트")
    print("=" * 60)
    
    # Frustum 생성
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    calib = calibrations['camera_1']
    generator = FrustumGenerator(calib)
    
    # 중앙 바운딩 박스로 frustum 생성
    bbox_2d = create_test_bbox_2d(320, 180, 120, 120)
    frustum = generator.generate_frustum(bbox_2d, near_dist=3.0, far_dist=25.0)
    
    # 테스트용 3D 박스들 (라이다 좌표계)
    test_boxes = [
        (10, 0, 0, 2, 2, 2, "정면 10m"),
        (15, -3, 0, 2, 2, 2, "우측 15m"),
        (8, 2, 0, 1.5, 1.5, 1.5, "좌측 8m"),
        (5, 0, -1, 1, 1, 1, "아래 5m"),
        (20, 0, 0, 3, 3, 3, "정면 20m (큰 박스)"),
        (30, 0, 0, 2, 2, 2, "정면 30m (frustum 밖)"),
    ]
    
    # 교차 계산
    engine = IntersectionEngine(method='point_in_frustum')
    aabbs = []
    
    print("\nIoU 계산 결과:")
    print("-" * 40)
    
    for x, y, z, sx, sy, sz, desc in test_boxes:
        bbox_3d = create_test_bbox_3d(x, y, z, sx, sy, sz)
        iou = engine.compute_iou_3d(frustum, bbox_3d)
        
        # AABB 생성 (시각화용)
        aabb = AABB(
            np.array([x - sx/2, y - sy/2, z - sz/2]),
            np.array([x + sx/2, y + sy/2, z + sz/2])
        )
        aabbs.append(aabb)
        
        print(f"{desc:20s}: IoU = {iou:.4f}")
    
    # 시각화
    visualize_frustum_3d(frustum, aabbs[:3], 
                        title="Frustum-Box 교차 테스트 (라이다 좌표계)")


def test_multi_camera():
    """멀티 카메라 테스트"""
    print("\n" + "=" * 60)
    print("멀티 카메라 Frustum 생성 테스트")
    print("=" * 60)
    
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    
    # 동일한 2D 바운딩 박스
    bbox_2d = create_test_bbox_2d(320, 180, 100, 100)
    
    frustums = {}
    for cam_id, calib in calibrations.items():
        generator = FrustumGenerator(calib)
        frustum = generator.generate_frustum(bbox_2d, near_dist=2.0, far_dist=15.0)
        frustums[cam_id] = frustum
        
        print(f"\n{cam_id}:")
        print(f"  - 카메라 위치: {frustum.camera_center}")
        print(f"  - Frustum 부피 (근사): {frustum.get_volume():.2f} m³")
    
    # 두 카메라의 frustum 비교 시각화
    if len(frustums) >= 2:
        fig = plt.figure(figsize=(16, 8))
        
        for idx, (cam_id, frustum) in enumerate(frustums.items(), 1):
            ax = fig.add_subplot(1, 2, idx, projection='3d')
            
            # Frustum 엣지 그리기
            corners = frustum.corners
            edges = [
                [0, 1], [1, 2], [2, 3], [3, 0],
                [4, 5], [5, 6], [6, 7], [7, 4],
                [0, 4], [1, 5], [2, 6], [3, 7]
            ]
            
            for edge in edges:
                points = corners[edge]
                ax.plot3D(*points.T, 'g-', linewidth=2)
            
            # 카메라 위치
            cam_pos = frustum.camera_center
            ax.scatter([cam_pos[0]], [cam_pos[1]], [cam_pos[2]], 
                      color='red', s=200, marker='o')
            
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'{cam_id} Frustum')
            ax.view_init(elev=20, azim=45)
        
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    print("\n")
    print("*" * 70)
    print("*" + " " * 68 + "*")
    print("*" + "  FRONTIER 캘리브레이션 기반 Frustum 테스트".center(66) + "  *")
    print("*" + " " * 68 + "*")
    print("*" * 70)
    
    # 각 테스트 실행
    calibrations = test_calibration_loading()
    test_projection()
    frustums = test_frustum_generation()
    test_frustum_box_intersection()
    test_multi_camera()
    
    print("\n" + "=" * 60)
    print("모든 테스트 완료!")
    print("=" * 60)