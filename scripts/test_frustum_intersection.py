#!/usr/bin/env python3
"""
FRONTIER Frustum-Box 교차 테스트 스크립트
Frustum 생성, 교차 계산, 시각화를 통합 테스트
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from frontier.calibration_loader import CalibrationLoader
from frontier.frustum_generator import FrustumGenerator, Frustum
from frontier.intersection_engine import IntersectionEngine, AABB
from frontier.visualization import FrontierVisualizer

from vision_msgs.msg import BoundingBox2D, BoundingBox3D
from geometry_msgs.msg import Pose, Quaternion, Point as ROSPoint, Vector3


def create_test_bbox_2d(center_x, center_y, width, height):
    """테스트용 2D 바운딩 박스 생성"""
    bbox = BoundingBox2D()
    bbox.center.position.x = center_x
    bbox.center.position.y = center_y
    bbox.size_x = width
    bbox.size_y = height
    return bbox


def create_test_bbox_3d(x, y, z, size_x, size_y, size_z):
    """테스트용 3D 바운딩 박스 생성"""
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


def plot_frustum_and_box(frustum: Frustum, aabb: AABB, iou: float):
    """Matplotlib로 Frustum과 Box 시각화"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Frustum 그리기
    frustum_corners = frustum.corners
    
    # Frustum 면 정의
    frustum_faces = [
        [frustum_corners[0], frustum_corners[1], frustum_corners[2], frustum_corners[3]],  # Near
        [frustum_corners[4], frustum_corners[5], frustum_corners[6], frustum_corners[7]],  # Far
        [frustum_corners[0], frustum_corners[1], frustum_corners[5], frustum_corners[4]],  # Top
        [frustum_corners[2], frustum_corners[3], frustum_corners[7], frustum_corners[6]],  # Bottom
        [frustum_corners[0], frustum_corners[3], frustum_corners[7], frustum_corners[4]],  # Left
        [frustum_corners[1], frustum_corners[2], frustum_corners[6], frustum_corners[5]],  # Right
    ]
    
    # Frustum 폴리곤 그리기
    frustum_collection = Poly3DCollection(frustum_faces, alpha=0.2, facecolor='green', edgecolor='darkgreen')
    ax.add_collection3d(frustum_collection)
    
    # AABB 그리기
    box_corners = aabb.get_corners()
    
    # Box 면 정의
    box_faces = [
        [box_corners[0], box_corners[1], box_corners[2], box_corners[3]],  # Bottom
        [box_corners[4], box_corners[5], box_corners[6], box_corners[7]],  # Top
        [box_corners[0], box_corners[1], box_corners[5], box_corners[4]],  # Front
        [box_corners[2], box_corners[3], box_corners[7], box_corners[6]],  # Back
        [box_corners[0], box_corners[3], box_corners[7], box_corners[4]],  # Left
        [box_corners[1], box_corners[2], box_corners[6], box_corners[5]],  # Right
    ]
    
    # Box 폴리곤 그리기
    box_collection = Poly3DCollection(box_faces, alpha=0.3, facecolor='blue', edgecolor='darkblue')
    ax.add_collection3d(box_collection)
    
    # 카메라 위치 표시
    ax.scatter([frustum.camera_center[0]], [frustum.camera_center[1]], [frustum.camera_center[2]], 
              color='red', s=100, marker='o', label='Camera')
    
    # 축 범위 설정
    all_points = np.vstack([frustum_corners, box_corners, frustum.camera_center.reshape(1, -1)])
    min_vals = np.min(all_points, axis=0)
    max_vals = np.max(all_points, axis=0)
    
    ax.set_xlim([min_vals[0] - 1, max_vals[0] + 1])
    ax.set_ylim([min_vals[1] - 1, max_vals[1] + 1])
    ax.set_zlim([min_vals[2] - 1, max_vals[2] + 1])
    
    # 라벨
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Frustum-Box Intersection Visualization\nIoU: {iou:.3f}')
    
    ax.legend()
    plt.show()


def test_basic_intersection():
    """기본 교차 테스트"""
    print("=" * 50)
    print("기본 Frustum-Box 교차 테스트")
    print("=" * 50)
    
    # 1. Frustum 생성
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    calib = calibrations['camera_1']  # camera_1 사용
    generator = FrustumGenerator(calib)
    
    # 이미지 중앙에 100x100 바운딩 박스
    bbox_2d = create_test_bbox_2d(320, 180, 100, 100)
    frustum = generator.generate_frustum(bbox_2d, near_dist=1.0, far_dist=10.0)
    
    print(f"Frustum 생성 완료:")
    print(f"  - Near distance: {frustum.near_distance}")
    print(f"  - Far distance: {frustum.far_distance}")
    print(f"  - 평면 개수: {len(frustum.planes)}")
    
    # 2. 3D Box 생성
    # Case 1: Frustum 내부에 완전히 포함된 박스
    bbox_3d_inside = create_test_bbox_3d(0, 0, 5, 1, 1, 1)
    
    # Case 2: Frustum과 부분적으로 겹치는 박스
    bbox_3d_partial = create_test_bbox_3d(2, 0, 5, 2, 2, 2)
    
    # Case 3: Frustum 외부의 박스
    bbox_3d_outside = create_test_bbox_3d(10, 10, 5, 1, 1, 1)
    
    # 3. 교차 계산
    engine = IntersectionEngine(method='point_in_frustum')
    
    iou_inside = engine.compute_iou_3d(frustum, bbox_3d_inside)
    iou_partial = engine.compute_iou_3d(frustum, bbox_3d_partial)
    iou_outside = engine.compute_iou_3d(frustum, bbox_3d_outside)
    
    print(f"\nIoU 계산 결과:")
    print(f"  - 내부 박스: {iou_inside:.3f}")
    print(f"  - 부분 겹침: {iou_partial:.3f}")
    print(f"  - 외부 박스: {iou_outside:.3f}")
    
    # 4. 시각화
    aabb_inside = AABB(
        np.array([bbox_3d_inside.center.position.x - bbox_3d_inside.size.x/2,
                 bbox_3d_inside.center.position.y - bbox_3d_inside.size.y/2,
                 bbox_3d_inside.center.position.z - bbox_3d_inside.size.z/2]),
        np.array([bbox_3d_inside.center.position.x + bbox_3d_inside.size.x/2,
                 bbox_3d_inside.center.position.y + bbox_3d_inside.size.y/2,
                 bbox_3d_inside.center.position.z + bbox_3d_inside.size.z/2])
    )
    
    plot_frustum_and_box(frustum, aabb_inside, iou_inside)


def test_different_methods():
    """다양한 IoU 계산 방법 비교"""
    print("\n" + "=" * 50)
    print("IoU 계산 방법 비교 테스트")
    print("=" * 50)
    
    # Frustum 생성
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    calib = calibrations['camera_1']
    generator = FrustumGenerator(calib)
    bbox_2d = create_test_bbox_2d(320, 180, 150, 150)
    frustum = generator.generate_frustum(bbox_2d, near_dist=2.0, far_dist=15.0)
    
    # 3D Box 생성
    bbox_3d = create_test_bbox_3d(0, 0, 7, 2, 2, 2)
    
    # 다양한 방법으로 IoU 계산
    methods = ['point_in_frustum', 'voxel', 'sampling']
    
    for method in methods:
        engine = IntersectionEngine(method=method)
        iou = engine.compute_iou_3d(frustum, bbox_3d)
        print(f"  - {method:20s}: IoU = {iou:.4f}")


def test_distance_effect():
    """거리에 따른 IoU 변화 테스트"""
    print("\n" + "=" * 50)
    print("거리별 IoU 변화 테스트")
    print("=" * 50)
    
    # Frustum 생성
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    calib = calibrations['camera_1']
    generator = FrustumGenerator(calib)
    bbox_2d = create_test_bbox_2d(320, 180, 100, 100)
    frustum = generator.generate_frustum(bbox_2d, near_dist=1.0, far_dist=30.0)
    
    # 교차 엔진
    engine = IntersectionEngine(method='point_in_frustum')
    
    # 다양한 거리에서 박스 생성 및 IoU 계산
    distances = [3, 5, 10, 15, 20, 25]
    ious = []
    
    print(f"거리\tIoU")
    print("-" * 20)
    
    for dist in distances:
        bbox_3d = create_test_bbox_3d(0, 0, dist, 1, 1, 1)
        iou = engine.compute_iou_3d(frustum, bbox_3d)
        ious.append(iou)
        print(f"{dist}m\t{iou:.4f}")
    
    # 그래프 그리기
    plt.figure(figsize=(10, 6))
    plt.plot(distances, ious, 'b-o', linewidth=2, markersize=8)
    plt.xlabel('Distance (m)', fontsize=12)
    plt.ylabel('IoU', fontsize=12)
    plt.title('IoU vs Distance for Fixed-Size Box', fontsize=14)
    plt.grid(True, alpha=0.3)
    plt.show()


def test_batch_processing():
    """배치 처리 성능 테스트"""
    print("\n" + "=" * 50)
    print("배치 처리 성능 테스트")
    print("=" * 50)
    
    import time
    
    # 캘리브레이션 로드
    loader = CalibrationLoader()
    calibrations = loader.load_calibrations()
    calib = calibrations['camera_1']
    generator = FrustumGenerator(calib)
    
    # 여러 Frustum 생성
    n_frustums = 5
    n_boxes = 8
    
    frustums = []
    for i in range(n_frustums):
        # 이미지의 다양한 위치에 바운딩 박스 (640x360 이미지)
        x = 160 + i * 80
        y = 100 + i * 30
        bbox_2d = create_test_bbox_2d(x, y, 80, 80)
        frustum = generator.generate_frustum(bbox_2d, near_dist=1.0, far_dist=20.0)
        frustums.append(frustum)
    
    # 여러 3D Box 생성
    boxes = []
    for i in range(n_boxes):
        x = (i - 4) * 2
        y = (i - 4) * 1
        z = 5 + i * 2
        bbox_3d = create_test_bbox_3d(x, y, z, 1.5, 1.5, 1.5)
        boxes.append(bbox_3d)
    
    # 배치 IoU 계산
    engine = IntersectionEngine(method='point_in_frustum')
    
    start_time = time.time()
    iou_matrix = engine.compute_batch_iou(frustums, boxes)
    elapsed_time = time.time() - start_time
    
    print(f"배치 크기: {n_frustums} x {n_boxes}")
    print(f"처리 시간: {elapsed_time*1000:.2f} ms")
    print(f"평균 시간/쌍: {elapsed_time*1000/(n_frustums*n_boxes):.3f} ms")
    
    # IoU 행렬 출력
    print(f"\nIoU 행렬:")
    print("Box →", end="")
    for j in range(n_boxes):
        print(f"\t{j}", end="")
    print("\nFrust↓")
    
    for i in range(n_frustums):
        print(f"{i}", end="")
        for j in range(n_boxes):
            print(f"\t{iou_matrix[i,j]:.2f}", end="")
        print()
    
    # 최적 매칭 찾기 (간단한 greedy)
    print(f"\n최적 매칭 (Greedy):")
    used_boxes = set()
    matches = []
    
    for i in range(n_frustums):
        best_j = -1
        best_iou = 0
        for j in range(n_boxes):
            if j not in used_boxes and iou_matrix[i,j] > best_iou:
                best_j = j
                best_iou = iou_matrix[i,j]
        
        if best_j >= 0 and best_iou > 0.1:  # 임계값
            matches.append((i, best_j, best_iou))
            used_boxes.add(best_j)
    
    for f_idx, b_idx, iou in matches:
        print(f"  Frustum {f_idx} ↔ Box {b_idx}: IoU = {iou:.3f}")


if __name__ == "__main__":
    print("\n")
    print("*" * 60)
    print("*" + " " * 58 + "*")
    print("*" + "  FRONTIER Frustum-Box Intersection Test Suite".center(56) + "  *")
    print("*" + " " * 58 + "*")
    print("*" * 60)
    
    # 각 테스트 실행
    test_basic_intersection()
    test_different_methods()
    test_distance_effect()
    test_batch_processing()
    
    print("\n" + "=" * 50)
    print("모든 테스트 완료!")
    print("=" * 50)