# Frustum Visualizer 안전 개선 사항

## 문제점
- 높은 빈도의 YOLO detection 토픽으로 인한 CPU/메모리 과부하
- 컴퓨터 멈춤 현상 발생

## 해결 방안

### 1. QoS 설정 개선
```python
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1  # 최신 메시지만 유지
)
```
- Queue depth를 1로 설정하여 메시지 백로그 방지
- VOLATILE durability로 메모리 사용 최소화

### 2. 처리 제한 (Throttling)
- **max_frustums_per_frame**: 프레임당 최대 frustum 수 제한 (기본값: 5)
- **min_process_interval**: 최소 처리 간격 (기본값: 0.1초)
- 처리 중 플래그로 중복 처리 방지

### 3. 메시지 드롭 메커니즘
- **drop_old_messages**: 오래된 메시지 자동 드롭 (기본값: true)
- **message_age_threshold**: 메시지 나이 임계값 (기본값: 0.5초)
- 처리 지연 시 메시지 드롭 통계 표시

### 4. 동시성 제어
```python
self.processing_lock = threading.Lock()
self.is_processing = False
```
- 스레드 안전 처리 플래그
- 동시 처리 방지

## 사용 방법

### 안전한 실행 예제

```bash
# 낮은 부하 설정
ros2 launch frontier frustum_test_launch.py \
    max_frustums_per_frame:=3 \
    min_process_interval:=0.2 \
    publish_rate:=5.0

# YOLO mode에서 안전 설정
ros2 launch frontier frustum_test_launch.py \
    use_manual_bbox:=false \
    max_frustums_per_frame:=2 \
    min_process_interval:=0.5 \
    drop_old_messages:=true \
    message_age_threshold:=0.3
```

## 새로운 파라미터

| 파라미터 | 기본값 | 설명 | 권장 범위 |
|---------|--------|------|-----------|
| `max_frustums_per_frame` | `5` | 프레임당 최대 frustum 수 | 1-10 |
| `min_process_interval` | `0.1` | 최소 처리 간격 (초) | 0.05-1.0 |
| `drop_old_messages` | `true` | 오래된 메시지 드롭 여부 | true/false |
| `message_age_threshold` | `0.5` | 메시지 나이 임계값 (초) | 0.1-2.0 |

## 성능 튜닝 가이드

### 낮은 성능 시스템
```bash
ros2 launch frontier frustum_test_launch.py \
    max_frustums_per_frame:=2 \
    min_process_interval:=0.5 \
    publish_rate:=2.0
```

### 중간 성능 시스템
```bash
ros2 launch frontier frustum_test_launch.py \
    max_frustums_per_frame:=5 \
    min_process_interval:=0.1 \
    publish_rate:=10.0
```

### 높은 성능 시스템
```bash
ros2 launch frontier frustum_test_launch.py \
    max_frustums_per_frame:=10 \
    min_process_interval:=0.05 \
    publish_rate:=20.0
```

## 모니터링

노드는 다음 정보를 주기적으로 출력:
- 처리된 프레임 수
- 생성된 총 frustum 수
- 평균 frustum/프레임
- 드롭된 메시지 수

```
[INFO] Stats: 100 frames, 450 total frustums, 4.5 avg frustums/frame, dropped: 23
```

## 문제 해결

### 여전히 시스템이 느림
1. `max_frustums_per_frame` 감소 (예: 2-3)
2. `min_process_interval` 증가 (예: 0.3-0.5)
3. `publish_rate` 감소 (예: 2-5Hz)

### 시각화가 끊김
1. `drop_old_messages`를 `false`로 설정
2. `message_age_threshold` 증가
3. QoS depth를 2-3으로 증가 (코드 수정 필요)

### 메시지 드롭이 너무 많음
1. YOLO detection 빈도 확인
2. `min_process_interval` 감소
3. `max_frustums_per_frame` 증가 (시스템 성능 고려)

## 개선 효과
- ✅ CPU/메모리 과부하 방지
- ✅ 시스템 응답성 유지
- ✅ 안정적인 실시간 처리
- ✅ 설정 가능한 성능 파라미터
- ✅ 드롭된 메시지 통계 제공