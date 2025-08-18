# FRONTIER Python PoC 구현 아키텍처

## 1. 프로젝트 개요

FRONTIER (Frustum-based Refined Object Networking & Tracking via Intersection Engine for Ranging-data)의 Python Proof-of-Concept 구현으로, 2D YOLO 탐지와 3D LiDAR 데이터의 정확한 매칭을 위한 3D 공간 교차 테스트 시스템입니다.

## 2. 패키지 구조

```
frontier/
├── src/
│   └── frontier/
│       ├── __init__.py
│       ├── frustum_generator.py      # Frustum 생성 모듈
│       ├── intersection_engine.py    # 교차 계산 엔진
│       ├── association_matcher.py    # 헝가리안 매칭 모듈
│       ├── visualization.py          # 시각화 유틸리티
│       └── frontier_node.py          # ROS2 노드 메인
├── scripts/
│   ├── test_frustum.py              # Frustum 생성 테스트
│   ├── test_intersection.py         # 교차 계산 테스트
│   └── benchmark.py                 # 성능 벤치마크
├── config/
│   └── frontier_config.yaml         # 노드 설정 파일
└── launch/
    └── frontier_poc.launch.py       # 런치 파일
```

## 3. 핵심 모듈 상세 설계

### 3.1 Frustum Generator 모듈

#### 클래스: `FrustumGenerator`

**목적**: 카메라 내부 파라미터와 2D 바운딩 박스로부터 3D 뷰 프러스텀 생성

**주요 메서드**:
```python
class FrustumGenerator:
    def __init__(self, camera_info: CameraInfo):
        """카메라 정보로 초기화"""
        
    def generate_frustum(self, bbox_2d: BoundingBox2D, 
                        near_dist: float = 0.5, 
                        far_dist: float = 100.0) -> Frustum:
        """2D 바운딩 박스에서 3D frustum 생성"""
        
    def pixel_to_ray(self, u: float, v: float) -> np.ndarray:
        """픽셀 좌표를 3D 광선 벡터로 변환"""
        
    def compute_frustum_planes(self, corners_3d: np.ndarray) -> List[Plane]:
        """프러스텀을 구성하는 6개 평면 계산"""
```

**구현 세부사항**:
- `image_geometry.PinholeCameraModel` 활용
- 2D 박스의 4개 코너 → 3D 광선 변환
- Near/Far 평면과 4개 측면 평면 정의
- 평면 방정식: `ax + by + cz + d = 0` 형태로 저장

### 3.2 Intersection Engine 모듈

#### 클래스: `IntersectionEngine`

**목적**: 3D Frustum과 3D Bounding Box 간의 교차 계산

**주요 메서드**:
```python
class IntersectionEngine:
    def __init__(self, method: str = 'point_in_frustum'):
        """교차 계산 방법 설정"""
        
    def compute_iou_3d(self, frustum: Frustum, 
                       bbox_3d: BoundingBox3D) -> float:
        """3D IoU 계산"""
        
    def point_in_frustum(self, point: np.ndarray, 
                        frustum: Frustum) -> bool:
        """포인트가 frustum 내부에 있는지 검사"""
        
    def aabb_frustum_overlap(self, aabb: AABB, 
                            frustum: Frustum) -> float:
        """AABB-Frustum 정확한 교차 부피 계산"""
        
    def sample_based_iou(self, frustum: Frustum, 
                        bbox_3d: BoundingBox3D, 
                        n_samples: int = 1000) -> float:
        """샘플링 기반 근사 IoU"""
```

**구현 전략**:
1. **Phase 1**: Point-in-Frustum 테스트
   - 박스 내 포인트 샘플링
   - 각 포인트의 frustum 포함 여부 검사
   - 포함 비율로 IoU 근사

2. **Phase 2**: Voxel 기반 IoU
   - 공간을 0.1m 그리드로 복셀화
   - 각 복셀의 frustum/box 포함 여부 검사
   - 교집합 복셀 수 / 합집합 복셀 수

3. **Phase 3**: 정확한 기하학적 교차
   - Sutherland-Hodgman 알고리즘 활용
   - 다면체 클리핑으로 정확한 교차 부피 계산

### 3.3 Association Matcher 모듈

#### 클래스: `AssociationMatcher`

**목적**: 최적의 frustum-box 매칭 수행

**주요 메서드**:
```python
class AssociationMatcher:
    def __init__(self, iou_threshold: float = 0.3):
        """매칭 임계값 설정"""
        
    def build_cost_matrix(self, frustums: List[Frustum], 
                         boxes: List[BoundingBox3D],
                         engine: IntersectionEngine) -> np.ndarray:
        """비용 행렬 생성 (1.0 - IoU)"""
        
    def hungarian_matching(self, cost_matrix: np.ndarray) -> List[Tuple[int, int]]:
        """헝가리안 알고리즘으로 최적 매칭"""
        
    def greedy_matching(self, cost_matrix: np.ndarray) -> List[Tuple[int, int]]:
        """Greedy 매칭 (fallback)"""
        
    def filter_matches(self, matches: List[Tuple[int, int]], 
                      cost_matrix: np.ndarray) -> List[Tuple[int, int]]:
        """임계값 기반 매칭 필터링"""
```

**최적화 전략**:
- 거리 기반 사전 필터링 (50m 이상 제외)
- 시야각 기반 게이팅 (FOV 외부 제외)
- 타임아웃 설정 (20ms 초과 시 greedy fallback)

### 3.4 Visualization 모듈

#### 클래스: `FrontierVisualizer`

**목적**: RViz2를 위한 시각화 마커 생성

**주요 메서드**:
```python
class FrontierVisualizer:
    def create_frustum_marker(self, frustum: Frustum, 
                             frame_id: str,
                             color: Tuple[float, float, float, float]) -> Marker:
        """Frustum을 선 리스트 마커로 생성"""
        
    def create_bbox_marker(self, bbox: BoundingBox3D,
                          frame_id: str,
                          color: Tuple[float, float, float, float]) -> Marker:
        """3D 박스를 큐브 마커로 생성"""
        
    def create_match_line(self, frustum_center: np.ndarray,
                         bbox_center: np.ndarray,
                         frame_id: str) -> Marker:
        """매칭된 쌍을 연결하는 선 생성"""
        
    def create_text_marker(self, text: str, 
                          position: np.ndarray,
                          frame_id: str) -> Marker:
        """IoU 값 등 텍스트 표시"""
```

**시각화 요소**:
- Frustum: 와이어프레임 (녹색)
- LiDAR Box: 실선 큐브 (파란색)
- 매칭 선: 연결선 (노란색)
- IoU 텍스트: 각 매칭 위 표시

## 4. ROS2 노드 구현

### 클래스: `FrontierNode`

**주요 콜백**:
```python
def camera_info_callback(self, msg: CameraInfo):
    """카메라 정보 업데이트"""
    
def yolo_callback(self, msg: Detection2DArray):
    """YOLO 탐지 수신 및 frustum 생성"""
    
def lidar_callback(self, msg: Detection3DArray):
    """LiDAR 탐지 수신"""
    
def sync_and_match(self):
    """시간 동기화 및 매칭 수행"""
    
def publish_results(self, matches: List[Match]):
    """매칭 결과 발행"""
```

**시간 동기화**:
- `message_filters.ApproximateTimeSynchronizer` 사용
- 허용 오차: 50ms
- 버퍼 크기: 10 메시지

## 5. 테스트 계획

### 5.1 단위 테스트

**test_frustum.py**:
- Frustum 평면 방정식 정확도
- 픽셀-광선 변환 정확도
- 다양한 바운딩 박스 크기 테스트

**test_intersection.py**:
- Point-in-Frustum 정확도
- IoU 계산 정확도 (ground truth 대비)
- 에지 케이스 (완전 포함, 부분 교차, 분리)

### 5.2 통합 테스트

**시나리오**:
1. 단일 객체 매칭 (1 YOLO, 1 LiDAR)
2. 다중 객체 매칭 (N YOLO, M LiDAR)
3. 오클루전 처리 (부분 가림)
4. 원거리 객체 (> 30m)
5. 밀집 환경 (10+ 객체)

### 5.3 성능 벤치마크

**측정 항목**:
- Frustum 생성 시간: < 1ms/frustum
- IoU 계산 시간: < 0.5ms/pair
- 헝가리안 매칭: < 10ms (20x20 행렬)
- 전체 파이프라인: < 20ms/frame

## 6. 설정 파라미터

**frontier_config.yaml**:
```yaml
frontier:
  ros__parameters:
    # 입력 토픽
    camera_info_topic: "/usb_cam/camera_info"
    yolo_topic: "/yolo/detections"
    lidar_topic: "/cone_detection/sorted_cones_time"
    
    # 출력 토픽
    fused_topic: "/frontier/fused_detections"
    viz_topic: "/frontier/visualization"
    
    # Frustum 파라미터
    frustum:
      near_distance: 0.5
      far_distance: 100.0
      
    # 매칭 파라미터
    matching:
      iou_threshold: 0.3
      max_distance: 50.0
      timeout_ms: 20
      method: "hungarian"  # "hungarian" or "greedy"
      
    # IoU 계산 방법
    iou:
      method: "point_in_frustum"  # "point_in_frustum", "voxel", "exact"
      n_samples: 1000
      voxel_size: 0.1
      
    # 시간 동기화
    sync:
      queue_size: 10
      slop_seconds: 0.05
```

## 7. 의존성

**Python 패키지**:
- `numpy`: 행렬 연산
- `scipy`: 헝가리안 알고리즘 (`linear_sum_assignment`)
- `open3d`: 포인트 클라우드 처리 (선택적)
- `shapely`: 2D 기하 연산 (선택적)
- `trimesh`: 3D 메시 연산 (Phase 3)

**ROS2 패키지**:
- `rclpy`: ROS2 Python 클라이언트
- `vision_msgs`: Detection 메시지 타입
- `sensor_msgs`: CameraInfo
- `visualization_msgs`: RViz 마커
- `tf2_ros`: 좌표계 변환
- `image_geometry`: 카메라 모델

## 8. 개발 로드맵

### Week 1: 기초 구현
- [ ] FrustumGenerator 클래스 구현
- [ ] Point-in-Frustum 테스트 구현
- [ ] 기본 시각화 구현
- [ ] 단위 테스트 작성

### Week 2: 매칭 시스템
- [ ] IntersectionEngine 완성
- [ ] AssociationMatcher 구현
- [ ] ROS2 노드 통합
- [ ] 시간 동기화 구현

### Week 3: 최적화
- [ ] Voxel 기반 IoU 구현
- [ ] 성능 프로파일링
- [ ] 거리 기반 최적화
- [ ] 멀티스레딩 적용

### Week 4: 검증
- [ ] 통합 테스트 수행
- [ ] calico 대비 성능 측정
- [ ] 파라미터 튜닝
- [ ] 문서화 완성

## 9. 성공 지표

**정확도**:
- Precision > 0.95
- Recall > 0.90
- F1-Score > 0.92

**성능**:
- 평균 처리 시간 < 15ms
- 최대 처리 시간 < 30ms
- 처리율 > 20 Hz

**안정성**:
- 메모리 누수 없음
- 24시간 연속 실행 가능
- 에러 복구 메커니즘

## 10. 참고 자료

- [Frustum PointNets](https://arxiv.org/abs/1711.08488)
- [3D IoU Calculation](https://github.com/AlienCat-K/3D-IoU-Python)
- [Hungarian Algorithm](https://en.wikipedia.org/wiki/Hungarian_algorithm)
- [ROS2 Message Filters](https://github.com/ros2/message_filters)