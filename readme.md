## FRONTIER 설정 가이드 (한국어)

이 문서는 카메라 2D 바운딩박스만으로 생성되는 프러스텀(Frustum)의 길이(near/far)를 적응형으로 추정하는 파라미터를 아주 자세히 설명합니다. 특히 단안(모노큘러) 기반의 규칙(rule)으로 근·원거리를 정하는 `adaptive_frustum` 설정을 중심으로 다룹니다.

---

## 어디에 설정하나요?
- 설정 파일: `frontier/config/frontier_config.yaml`
- 주요 블록:
  - `frontier.frustum`: 기본(고정) near/far
  - `frontier.adaptive_frustum`: 적응형(동적) near/far
- 노드: `frontier/frontier_node.py`, `frontier/frustum_visualizer_node.py`에서 설정을 읽어 적용합니다.

---

## 프러스텀 생성 원리 (요약)
- 카메라 내부 파라미터(초점거리 `fx, fy`)와 객체의 대략적인 실제 크기(높이/폭)를 알고 있을 때, 핀홀 모델로 대강의 깊이 Z를 추정합니다.
  - 높이 기반: `Z_h ≈ fy * H_real / h_pixels`
  - 폭 기반: `Z_w ≈ fx * W_real / w_pixels`
  - 보수적으로 더 “가까운” 값 사용: `Z_est = min(Z_h, Z_w)` (단, `near_min_m`보다 작아지지 않게 클램프)
- 불확실성에 따라 대칭 마진을 주어 밴드(near/far)를 만듭니다: `near = Z_est - band`, `far = Z_est + band` (경계는 `near_min_m`, `far_max_m`로 클램프).
- 마진은 바운딩박스 면적 비율이 작을수록(멀수록) 커집니다. 모드는 `coarse`(권장)와 `fine` 중 선택합니다.

---

## 핵심 파라미터 상세 (adaptive_frustum)
```yaml
frontier:
  adaptive_frustum:
    enabled: true            # 적응형 프러스텀 사용 여부
    mode: "coarse"             # coarse | fine

    # 대상 물체의 실제 크기(미터)
    object_height_m: 0.70    # 예: 높이 70cm
    object_width_m: 0.30     # 예: 밑변 30cm

    # 프러스텀 거리 경계(절대 한계)
    near_min_m: 0.5          # 이보다 더 가깝게는 잡지 않음
    far_max_m: 80.0          # 이보다 더 멀게는 잡지 않음 (원거리 확장)

    # fine 모드용(연속형) 파라미터 (mode=fine일 때만 사용)
    margin_min: 0.20         # bbox가 매우 클 때(가까움)에 적용되는 최소 마진 계수
    margin_max: 1.20         # bbox가 매우 작을 때(멀음)에 적용되는 최대 마진 계수
    area_thresholds: [0.10, 0.05, 0.02]  # 이미지 대비 bbox 면적 비율 경계(2~5단계 스무딩 기준)

    # coarse 모드용(3단계) 파라미터 (mode=coarse일 때만 사용)
    coarse_thresholds: [0.06, 0.02]      # 큰/중간/작은 bbox 경계 (면적 비율)
    coarse_margins: [0.50, 0.90, 1.40]   # 큰/중/작 순서의 마진 계수
```

### 1) enabled
- `true`면 적응형 near/far를 사용합니다. `false`면 `frontier.frustum.near_distance/far_distance`의 고정값을 사용합니다.

### 2) mode: coarse vs fine
- `coarse`(권장): 3개의 굵은 단계로 나누어 마진을 부여합니다.
  - `coarse_thresholds: [t_large, t_medium]`
    - `area_ratio ≥ t_large` → 큰 bbox (가까움) → `coarse_margins[0]` 적용
    - `t_medium ≤ area_ratio < t_large` → 중간 bbox → `coarse_margins[1]` 적용
    - `area_ratio < t_medium` → 작은 bbox (멀음) → `coarse_margins[2]` 적용
  - 단계별 마진이 명확하여 동작이 “더 코스(coarse)”하고 멀리도 잘 뻗습니다.
- `fine`: 2~5단계 스무딩과 연속형 마진 보간(smoothstep)으로 좀 더 섬세하게 밴드를 조절합니다.
  - `margin_min`~`margin_max` 사이에서 `area_thresholds`를 기준으로 스무딩.
  - 너무 섬세하다고 느껴지면 `coarse` 모드로 바꾸세요.

### 3) object_height_m, object_width_m
- 대상 물체의 대략적인 실제 크기(미터). 예: 밑변 30cm, 높이 70cm → `0.30`, `0.70`.
- 해상도/카메라 FOV에 따라 추정 깊이가 달라지므로 현장 데이터로 미세 보정 권장.

### 4) near_min_m, far_max_m
- 최종 `near`/`far`는 이 범위를 넘지 않도록 강제합니다.
- far가 너무 짧으면 탐지 실패가 늘 수 있으니 `far_max_m`을 여유 있게 키우세요(예: 80m).

### 5) margin 파라미터 (밴드 폭 조절)
- 마진은 `band = Z_est * margin * tier_scale`로 계산됩니다.
  - `margin`: 모드에 따라 `coarse_margins` 또는 `margin_min/max`에서 결정
  - `tier_scale`: 단계 수를 2~5로 두고 5에 가까울수록 더 넓게(보수적으로) 확장
- 결과적으로 `near = max(near_min_m, Z_est - band)`, `far = min(far_max_m, Z_est + band)`
- 너무 촘촘하면 마진을 키우고(값↑), 과하게 넓으면 줄이세요(값↓).

---

## bbox 면적 비율(area_ratio) 계산 개념
- `area_ratio = (bbox_width * bbox_height) / (image_width * image_height)`
- 카메라 캘리브레이션에서 `image_width/height`, `fx/fy`를 가져옵니다.
- bbox가 작을수록(멀수록) area_ratio가 작아지고 마진을 더 크게 적용합니다.

---

## 추천값/튜닝 가이드
- 멀리까지 커버가 필요하면:
  - `far_max_m`를 충분히 키우기(예: 80~100m)
  - `coarse_margins`의 작은 bbox 구간 값을 더 키우기(예: 1.6)
- 너무 코스하면:
  - `coarse_margins`를 전반적으로 낮추거나, `fine` 모드로 전환
- 너무 세밀하면:
  - `mode: "coarse"` 유지, `coarse_thresholds` 간격을 넓혀 단계를 더 단순화
- 타깃 크기가 다르면:
  - `object_height_m`, `object_width_m`를 실제 물체 크기에 맞춤

---

## 예제 1: 코스(coarse) 모드 (권장)
```yaml
frontier:
  frustum:
    near_distance: 1.0
    far_distance: 15.0

  adaptive_frustum:
    enabled: true
    mode: "coarse"
    object_height_m: 0.70
    object_width_m: 0.30
    near_min_m: 0.5
    far_max_m: 80.0
    coarse_thresholds: [0.06, 0.02]
    coarse_margins: [0.50, 0.90, 1.40]
```

## 예제 2: 파인(fine) 모드
```yaml
frontier:
  adaptive_frustum:
    enabled: true
    mode: "fine"
    object_height_m: 0.70
    object_width_m: 0.30
    near_min_m: 0.5
    far_max_m: 80.0
    margin_min: 0.20
    margin_max: 1.20
    area_thresholds: [0.10, 0.05, 0.02]
```

---

## 런치/사용
- 시각화 테스트:
  - `ros2 launch frontier frustum_test_launch.py` (RViz 포함)
  - 로그 레벨 조정: `ros2 launch frontier frustum_test_launch.py log_level:=debug`
- 메인 노드:
  - `ros2 launch frontier frontier_launch.py`
- 파라미터 변경 후에는 노드를 재시작하세요.

---

## 검증 팁
- RViz에서 프러스텀 길이가 bbox 크기에 따라 덜컥덜컥(코스) 혹은 부드럽게(파인) 변하는지 확인
- 너무 짧으면 `far_max_m`/마진↑, 너무 길면 마진↓ 또는 `coarse_margins` 보수적으로
- 실제 라이다 박스와 IoU를 보며 적절한 밴드인지 체감 조정

---

## 자주 묻는 질문
- Q: 모드만 바꿔도 될까요? → 네, `mode`를 `coarse`로 두는 걸 권장합니다. 파인이 필요할 때만 `fine`으로.
- Q: 해상도가 다르면? → 캘리브레이션에서 `image_width/height`, `fx/fy`를 읽어 쓰므로 그대로 동작합니다. 다만 `object_*_m`은 환경에 맞게 보정하세요.
- Q: 너무 세밀/거칠어요. → `coarse_thresholds`/`coarse_margins`(coarse) 또는 `margin_*`, `area_thresholds`(fine)로 즉각 조정 가능합니다.
