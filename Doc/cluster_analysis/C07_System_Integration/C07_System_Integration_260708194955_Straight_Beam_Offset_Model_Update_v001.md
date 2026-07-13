# C07 Straight Beam Offset Model Update v001

| 항목 | 내용 |
|---|---|
| 문서 종류 | 광학 시뮬레이터 모델 정정 기록 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-08 19:49:55 KST |
| Cluster | C07 System Integration / Optical Timing Simulator |
| 관련 HTML | `Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260708130837_Polygon_Mirror_Optical_Timing_Simulator_v001.html` |

## 1. 사용자 요청

입사 레이저가 회전하는 미러면 중심축을 따라가도록 조준되는 표현을 제거하고, 레이저에는 직진성만 부여한다. 또한 입사 레이저의 위치를 다각 미러 중심 기준 위/아래로 평행 이동했을 때, 회전 중인 다각 미러면과의 입사/반사가 동기화되어 변화되도록 한다.

## 2. 정정된 모델

| 항목 | 정정 전 | 정정 후 |
|---|---|---|
| 입사 ray | fixed source -> current face center 동적 조준 | 180도 고정 source에서 0도 방향 직진 |
| hit point | face center 또는 조준 face | 직진 ray와 회전 polygon의 첫 교점 |
| 위치 조정 | face normal/Q2 조준 기준 | polygon 중심 기준 `입사 Y offset` 평행 이동 |
| laser gate | active window AND Q2 gate | active window AND first-hit |
| 반사 계산 | 조준 face 중심 기준 | 실제 first-hit edge 기준 |

## 3. 기하광학 의미

정정 후 모델은 다음 수식을 따른다.

```text
source = fixed 180 deg position
incident_ray(t) = source + t x (1, 0)
incident_ray_y = polygon_center_y - y_offset
hit = nearest intersection(incident_ray, rotating_polygon_edges)
reflected = incident - 2 x dot(incident, normal) x normal
```

HTML 내부에서는 `reflectFromEdge()`가 edge tangent 기준으로 같은 반사 법칙을 계산한다.

```text
reflected_angle = 2 x tangent_angle - incident_angle
```

## 4. UI 반영

| UI | 의미 |
|---|---|
| `직진 입사 beam` | ON이면 진행각 0도 직진 ray 사용 |
| `입사 Y offset` | 다면미러 중심 기준 위/아래 평행 이동. 양수는 화면 위쪽 |
| `입사 진행각` | 직진 beam ON에서는 0도로 고정, OFF일 때만 수동 입력 |

## 5. 검증 기준

| 검증 | 기대 |
|---|---|
| JS syntax | PASS |
| Y offset 변경 | sourceY와 ray line이 평행 이동 |
| mirror rotation | first-hit edge와 reflected angle이 회전에 따라 변화 |
| rest window | guide만 표시, trail 미누적 |
| active window | first-hit가 있으면 incident/reflected ray 및 trail 표시 |

## 6. 판단

이번 정정이 현재 기하광학 이해에 더 맞다. 다면미러가 90도 화각을 만드는 핵심은 `face center 조준`이 아니라 `회전 미러면의 접선/법선 각도 변화`이며, 입사 beam은 직진성을 유지한 상태에서 회전 polygon과의 교점이 변해야 한다.
