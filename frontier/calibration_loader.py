#!/usr/bin/env python3
"""
캘리브레이션 파라미터 로더
카메라 intrinsic/extrinsic 파라미터를 YAML 파일에서 로드
"""

import os
import yaml
import numpy as np
from typing import Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class CameraCalibration:
    """카메라 캘리브레이션 데이터"""
    camera_id: str
    camera_matrix: np.ndarray  # 3x3 K matrix
    dist_coeffs: np.ndarray    # 1x5 distortion coefficients
    extrinsic_matrix: np.ndarray  # 4x4 transform matrix (lidar to camera)
    image_width: int
    image_height: int
    
    def get_projection_matrix(self) -> np.ndarray:
        """
        투영 행렬 P = K[R|t] 계산
        extrinsic_matrix가 라이다→카메라 변환이므로 직접 사용
        """
        # extrinsic_matrix에서 R과 t 추출
        R = self.extrinsic_matrix[:3, :3]
        t = self.extrinsic_matrix[:3, 3]
        
        # [R|t] 생성 (3x4)
        Rt = np.hstack([R, t.reshape(3, 1)])
        
        # P = K * [R|t]
        P = self.camera_matrix @ Rt
        
        return P
    
    def project_3d_to_2d(self, point_3d: np.ndarray) -> np.ndarray:
        """
        3D 점(라이다 좌표계)을 2D 이미지 좌표로 투영
        
        Args:
            point_3d: 3D 점 [x, y, z] (라이다 좌표계)
            
        Returns:
            2D 점 [u, v] (이미지 좌표)
        """
        # 동차 좌표로 변환
        point_homo = np.append(point_3d, 1.0)
        
        # 카메라 좌표계로 변환
        point_cam = self.extrinsic_matrix @ point_homo
        
        # 카메라 앞에 있는지 확인
        if point_cam[2] <= 0:
            return np.array([-1, -1])  # 카메라 뒤의 점
        
        # 이미지 평면에 투영
        point_img_homo = self.camera_matrix @ point_cam[:3]
        u = point_img_homo[0] / point_img_homo[2]
        v = point_img_homo[1] / point_img_homo[2]
        
        return np.array([u, v])
    
    def unproject_2d_to_ray(self, u: float, v: float, apply_distortion: bool = True) -> np.ndarray:
        """
        2D 이미지 좌표를 3D 광선으로 역투영 (카메라 좌표계)
        왜곡 보정을 포함한 정확한 역투영
        
        Args:
            u, v: 이미지 좌표 (왜곡된 이미지의 픽셀 좌표)
            apply_distortion: 왜곡 보정 적용 여부
            
        Returns:
            정규화된 3D 광선 벡터 (카메라 좌표계)
        """
        if apply_distortion and self.dist_coeffs is not None:
            # OpenCV 스타일 왜곡 보정
            # 픽셀 좌표를 정규화된 이미지 좌표로 변환
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            
            # 정규화된 좌표
            x_normalized = (u - cx) / fx
            y_normalized = (v - cy) / fy
            
            # 반복적 왜곡 보정 (Newton-Raphson)
            x_corrected = x_normalized
            y_corrected = y_normalized
            
            k1, k2, p1, p2, k3 = self.dist_coeffs[0, :5] if self.dist_coeffs.shape[1] >= 5 else \
                                (*self.dist_coeffs[0], 0, 0, 0)[:5]
            
            # 5회 반복으로 보정 (대부분 수렴)
            for _ in range(5):
                r2 = x_corrected**2 + y_corrected**2
                radial_distortion = 1 + k1*r2 + k2*r2**2 + k3*r2**3
                
                # 접선 왜곡
                dx = 2*p1*x_corrected*y_corrected + p2*(r2 + 2*x_corrected**2)
                dy = p1*(r2 + 2*y_corrected**2) + 2*p2*x_corrected*y_corrected
                
                x_corrected = (x_normalized - dx) / radial_distortion
                y_corrected = (y_normalized - dy) / radial_distortion
            
            # 보정된 좌표로 광선 생성
            ray_cam = np.array([x_corrected, y_corrected, 1.0])
        else:
            # 왜곡 보정 없이 기본 역투영
            K_inv = np.linalg.inv(self.camera_matrix)
            pixel_homo = np.array([u, v, 1.0])
            ray_cam = K_inv @ pixel_homo
        
        # 정규화
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        
        return ray_cam
    
    def transform_ray_to_lidar(self, ray_cam: np.ndarray) -> np.ndarray:
        """
        카메라 좌표계의 광선을 라이다 좌표계로 변환
        
        Args:
            ray_cam: 3D 광선 벡터 (카메라 좌표계)
            
        Returns:
            3D 광선 벡터 (라이다 좌표계)
        """
        # extrinsic_matrix의 역변환으로 카메라→라이다
        T_lidar_cam = np.linalg.inv(self.extrinsic_matrix)
        
        # 회전 부분만 적용 (방향 벡터이므로)
        R_lidar_cam = T_lidar_cam[:3, :3]
        ray_lidar = R_lidar_cam @ ray_cam
        
        return ray_lidar / np.linalg.norm(ray_lidar)
    
    def get_camera_position_in_lidar(self) -> np.ndarray:
        """
        라이다 좌표계에서의 카메라 위치 반환
        
        Returns:
            카메라 위치 [x, y, z] (라이다 좌표계)
        """
        # extrinsic_matrix의 역변환으로 카메라 원점 위치 계산
        T_lidar_cam = np.linalg.inv(self.extrinsic_matrix)
        camera_pos = T_lidar_cam[:3, 3]
        
        return camera_pos
    
    def transform_to_sensor_frame(self, point_lidar: np.ndarray) -> np.ndarray:
        """
        os_lidar 좌표계의 점을 os_sensor 좌표계로 변환
        
        os_sensor: 전방을 보는 좌표계 (카메라와 유사)
        os_lidar: 후방을 보는 좌표계 (180도 회전)
        
        Args:
            point_lidar: 3D 점 (os_lidar 좌표계)
            
        Returns:
            3D 점 (os_sensor 좌표계)
        """
        # os_lidar -> os_sensor 변환 행렬
        # X, Y축 반전 (180도 회전) + Z축 오프셋
        T_sensor_to_lidar = np.array([
            [-1,  0,  0,  0],
            [ 0, -1,  0,  0],
            [ 0,  0,  1, -0.038195],  # Z축 오프셋
            [ 0,  0,  0,  1]
        ], dtype=np.float64)
        
        # 역변환 (lidar -> sensor)
        T_lidar_to_sensor = np.linalg.inv(T_sensor_to_lidar)
        
        # 4D 동차 좌표로 변환
        if point_lidar.shape[-1] == 3:
            point_homo = np.append(point_lidar, 1.0)
        else:
            point_homo = point_lidar
        
        # 변환 적용
        point_sensor = T_lidar_to_sensor @ point_homo
        
        return point_sensor[:3]
    
    def get_camera_position_in_sensor(self) -> np.ndarray:
        """
        센서 좌표계(os_sensor)에서의 카메라 위치 반환
        
        Returns:
            카메라 위치 [x, y, z] (os_sensor 좌표계)
        """
        camera_pos_lidar = self.get_camera_position_in_lidar()
        return self.transform_to_sensor_frame(camera_pos_lidar)


class CalibrationLoader:
    """캘리브레이션 파일 로더"""
    
    def __init__(self, config_folder: str = None):
        """
        Args:
            config_folder: 설정 파일이 있는 폴더 경로
        """
        if config_folder is None:
            # 기본 경로 사용 - ament_index를 통해 패키지 경로 찾기
            from ament_index_python.packages import get_package_share_directory
            frontier_share = get_package_share_directory('frontier')
            self.config_folder = os.path.join(frontier_share, 'config')
        else:
            self.config_folder = config_folder
    
    def load_calibrations(self, 
                         intrinsic_file: str = "multi_camera_intrinsic_calibration.yaml",
                         extrinsic_file: str = "multi_camera_extrinsic_calibration.yaml") -> Dict[str, CameraCalibration]:
        """
        모든 카메라의 캘리브레이션 데이터 로드
        
        Args:
            intrinsic_file: intrinsic 캘리브레이션 파일명
            extrinsic_file: extrinsic 캘리브레이션 파일명
            
        Returns:
            카메라 ID를 키로 하는 CameraCalibration 딕셔너리
        """
        intrinsic_path = os.path.join(self.config_folder, intrinsic_file)
        extrinsic_path = os.path.join(self.config_folder, extrinsic_file)
        
        # Intrinsic 파라미터 로드
        intrinsics = self._load_intrinsic_params(intrinsic_path)
        
        # Extrinsic 파라미터 로드
        extrinsics = self._load_extrinsic_params(extrinsic_path)
        
        # 통합된 캘리브레이션 객체 생성
        calibrations = {}
        
        for camera_id in intrinsics.keys():
            if camera_id not in extrinsics:
                import logging
                logging.warning(f"Extrinsic parameters not found for {camera_id}.")
                continue
            
            K, D, width, height = intrinsics[camera_id]
            T = extrinsics[camera_id]
            
            calibrations[camera_id] = CameraCalibration(
                camera_id=camera_id,
                camera_matrix=K,
                dist_coeffs=D,
                extrinsic_matrix=T,
                image_width=width,
                image_height=height
            )
        
        return calibrations
    
    def _load_intrinsic_params(self, yaml_path: str) -> Dict[str, Tuple[np.ndarray, np.ndarray, int, int]]:
        """
        Intrinsic 파라미터 로드
        
        Returns:
            {camera_id: (camera_matrix, dist_coeffs, width, height)}
        """
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Intrinsic 캘리브레이션 파일이 없습니다: {yaml_path}")
        
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        intrinsics = {}
        
        for camera_id, cam_data in data.items():
            # 카메라 행렬 추출
            cam_matrix_data = cam_data['camera_matrix']['data']
            # 중첩 리스트를 평탄화
            if isinstance(cam_matrix_data[0], list):
                cam_matrix_flat = [item for sublist in cam_matrix_data for item in sublist]
            else:
                cam_matrix_flat = cam_matrix_data
            
            K = np.array(cam_matrix_flat, dtype=np.float64).reshape(3, 3)
            
            # 왜곡 계수 추출
            dist_data = cam_data['distortion_coefficients']['data']
            D = np.array(dist_data, dtype=np.float64).reshape(1, -1)
            
            # 이미지 크기
            width = cam_data['image_size']['width']
            height = cam_data['image_size']['height']
            
            intrinsics[camera_id] = (K, D, width, height)
        
        return intrinsics
    
    def _load_extrinsic_params(self, yaml_path: str) -> Dict[str, np.ndarray]:
        """
        Extrinsic 파라미터 로드
        
        Returns:
            {camera_id: extrinsic_matrix}
        """
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Extrinsic 캘리브레이션 파일이 없습니다: {yaml_path}")
        
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        
        extrinsics = {}
        
        for camera_id, cam_data in data.items():
            # Extrinsic 행렬 추출
            matrix_data = cam_data['extrinsic_matrix']
            
            # 중첩 리스트를 평탄화
            if isinstance(matrix_data[0], list):
                matrix_flat = [item for sublist in matrix_data for item in sublist]
            else:
                matrix_flat = matrix_data
            
            T = np.array(matrix_flat, dtype=np.float64).reshape(4, 4)
            extrinsics[camera_id] = T
        
        return extrinsics
    
    def load_frontier_config(self, config_file: str = "multi_hungarian_config.yaml") -> dict:
        """
        FRONTIER 설정 파일 로드
        
        Args:
            config_file: 설정 파일명
            
        Returns:
            설정 딕셔너리
        """
        config_path = os.path.join(self.config_folder, config_file)
        
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"설정 파일이 없습니다: {config_path}")
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # config_folder 경로를 frontier 경로로 수정
        if 'hungarian_association' in config and 'calibration' in config['hungarian_association']:
            config['hungarian_association']['calibration']['config_folder'] = self.config_folder
        
        return config