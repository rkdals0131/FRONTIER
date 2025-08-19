# FRONTIER Frustum Visualization Test Guide

## 개요
Frustum 생성 및 투영 행렬이 올바르게 구성되었는지 확인하기 위한 테스트 도구입니다.
YOLO detection 또는 수동 바운딩 박스를 사용하여 frustum beam을 RViz에 시각화합니다.

## 주요 기능
- **Manual Mode**: 수동으로 설정한 2D 바운딩 박스로 frustum 생성
- **YOLO Mode**: 실제 YOLO detection으로부터 frustum 생성
- **실시간 시각화**: RViz2에서 3D frustum beam 표시
- **캘리브레이션 검증**: 카메라 intrinsic/extrinsic 파라미터 검증

## 실행 방법

### 1. 기본 실행 (Manual Mode)
```bash
# 가상환경 활성화 (workspace root에서)
source .venv/bin/activate
source install/setup.bash

# Manual mode로 실행 (기본값)
ros2 launch frontier frustum_test_launch.py
```

### 2. RViz와 함께 실행
```bash
ros2 launch frontier frustum_test_launch.py launch_rviz:=true
```

### 3. 커스텀 바운딩 박스 파라미터
```bash
ros2 launch frontier frustum_test_launch.py \
    manual_bbox_center_x:=400 \
    manual_bbox_center_y:=300 \
    manual_bbox_width:=150 \
    manual_bbox_height:=200 \
    near_distance:=2.0 \
    far_distance:=50.0
```

### 4. YOLO Detection Mode
```bash
# YOLO detection을 사용한 frustum 생성
ros2 launch frontier frustum_test_launch.py \
    use_manual_bbox:=false \
    camera_id:=camera_1
```

### 5. 테스트 스크립트 사용
```bash
# ROS2 workspace로 이동
cd ${ROS2_WS}  # 또는 실제 workspace 경로

# 기본 실행
./src/frontier/scripts/test_frustum_viz.sh

# RViz와 함께
./src/frontier/scripts/test_frustum_viz.sh rviz

# YOLO mode
./src/frontier/scripts/test_frustum_viz.sh yolo

# 커스텀 파라미터
./src/frontier/scripts/test_frustum_viz.sh custom
```

## Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `camera_id` | `camera_1` | 사용할 카메라 ID (`camera_1` 또는 `camera_2`) |
| `near_distance` | `1.0` | Near 평면 거리 (meters) |
| `far_distance` | `30.0` | Far 평면 거리 (meters) |
| `use_manual_bbox` | `true` | Manual bbox 사용 여부 |
| `manual_bbox_center_x` | `320.0` | Manual bbox 중심 X (pixels) |
| `manual_bbox_center_y` | `240.0` | Manual bbox 중심 Y (pixels) |
| `manual_bbox_width` | `100.0` | Manual bbox 너비 (pixels) |
| `manual_bbox_height` | `100.0` | Manual bbox 높이 (pixels) |
| `publish_rate` | `10.0` | 퍼블리시 레이트 (Hz) |
| `log_level` | `info` | 로그 레벨 |
| `launch_rviz` | `false` | RViz2 실행 여부 |

## RViz 시각화

### 토픽
- **Markers**: `/frustum_visualizer/markers`
  - Frustum wireframe (색상으로 구분)
  - 카메라 위치 (주황색 구)
  - 좌표축 (X:빨강, Y:초록, Z:파랑)
  - 바운딩 박스 정보 텍스트

### RViz 설정
1. Fixed Frame: `os_sensor` (LiDAR 좌표계)
2. MarkerArray 토픽: `/frustum_visualizer/markers`
3. 설정 파일: `src/frontier/config/frustum_test.rviz`

## 시각화 요소

### Frustum Beam
- **와이어프레임**: Near/Far 평면과 연결선
- **카메라 중심**: 주황색 구로 표시
- **색상**: 각 frustum마다 다른 색상 (빨강, 초록, 파랑, 노랑, 마젠타, 시안)

### 정보 표시
- **텍스트 라벨**: 2D 바운딩 박스 좌표와 크기
- **좌표축**: 카메라 위치에서의 X, Y, Z 축

## 검증 포인트

### 1. Frustum 방향
- Frustum이 카메라에서 올바른 방향으로 뻗어나가는지 확인
- Near 평면이 카메라 가까이, Far 평면이 멀리 있는지 확인

### 2. Frustum 크기
- 바운딩 박스 크기에 비례하여 frustum 크기가 변하는지 확인
- Near에서 Far로 갈수록 frustum이 확장되는지 확인

### 3. 카메라 위치
- 카메라 위치가 캘리브레이션 파라미터와 일치하는지 확인
- camera_1과 camera_2의 위치가 다른지 확인

### 4. 좌표 변환
- Frustum이 LiDAR 좌표계에서 올바르게 표시되는지 확인
- 이미지 좌표 → 카메라 좌표 → LiDAR 좌표 변환이 정확한지 검증

## 문제 해결

### Frustum이 표시되지 않음
1. 토픽 확인: `ros2 topic echo /frustum_visualizer/markers`
2. 노드 상태: `ros2 node list`
3. 캘리브레이션 파일 존재 여부 확인

### Frustum 방향이 이상함
1. Extrinsic 캘리브레이션 파라미터 확인
2. 좌표계 변환 행렬 검증
3. Near/Far 거리 설정 확인

### Manual Mode에서 frustum이 고정됨
1. `publish_rate` 파라미터 확인
2. 타이머 콜백이 실행되는지 로그 확인

## 예상 결과

### 정상 동작 시
1. RViz에 frustum wireframe이 표시됨
2. 카메라 위치에서 시작하여 far distance까지 확장되는 절두체 형태
3. 바운딩 박스 크기/위치 변경 시 frustum이 즉시 업데이트
4. 각 frustum이 다른 색상으로 구분되어 표시

### 캘리브레이션 검증
- Frustum이 예상한 방향과 크기로 생성되면 캘리브레이션이 올바름
- 왜곡 보정이 적용되어 정확한 3D 투영이 이루어짐

## 다음 단계

Frustum 생성이 올바르게 작동하면:
1. LiDAR 3D detection과의 융합 테스트
2. IoU 계산 검증
3. Hungarian 알고리즘 매칭 테스트
4. 전체 FRONTIER 시스템 통합 테스트