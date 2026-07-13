# C07 Timing / Optical Coupling Review v001

| 항목 | 내용 |
|---|---|
| 문서 종류 | PRF / 모터 / 화각 / 분해능 연계 검토 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-08 19:03:40 KST |
| Cluster | C07 System Integration / Optical Timing Simulator |
| 관련 HTML | `Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260708130837_Polygon_Mirror_Optical_Timing_Simulator_v001.html` |

## 1. 결론

시뮬레이터의 계산표에서는 `PRF 주기`, `모터 회전 주기`, `화각`, `분해능`이 연계되어 있었다. 다만 광학 화면에서 실제 ray/trail이 `face active window`와 완전히 gate되지 않아 기하광학적으로 이해하기 어려운 상태였다.

이번 보완으로 실제 laser ON 조건을 다음과 같이 정정했다.

```text
laser_on = face_active_window AND incident_Q2_gate
```

따라서 rest window에서는 레이저가 꺼지고, active window에서만 shot index/PRF/반사 ray가 의미를 가진다.

## 2. 연계 수식

### 2.1 모터 회전 주기

현재 모델은 1회 360도 회전이 1 frame을 만든다고 둔다.

```text
frame_period = 1 / frame_rate
mirror_rpm   = frame_rate x 60
```

예: 20 Hz이면 1 frame은 50 ms이고, mirror speed는 1200 rpm이다.

### 2.2 한 면의 기계각

```text
face_mech_angle = 360 deg / facet_count
```

4각이면 한 면은 90도 기계각을 담당한다.

### 2.3 광학 FoV와 미러 기계각

평면 반사에서는 미러가 `theta`만큼 회전하면 반사광은 `2 x theta`만큼 변한다.

```text
optical_scan_angle = 2 x mirror_mech_angle
active_mech_angle  = horizontal_FoV / 2
```

따라서 수평 광학 FoV가 90도이면 필요한 미러 기계각은 45도다.

### 2.4 active/rest 시간

```text
active_time_per_face = frame_period x active_mech_angle / 360
rest_time_per_face   = frame_period x (face_mech_angle - active_mech_angle) / 360
```

4각, 20 Hz, H_FoV 90도이면:

```text
frame_period = 50 ms
face_mech_angle = 90 deg
active_mech_angle = 45 deg
active_time = 50 ms x 45 / 360 = 6.25 ms
rest_time   = 50 ms x 45 / 360 = 6.25 ms
```

### 2.5 분해능과 shot 수

```text
shots_per_face = ceil(horizontal_FoV / horizontal_resolution)
```

예: 90도 / 0.2도 = 450 shots/face.

### 2.6 PRF와 레이저 주기

```text
active_PRF    = shots_per_face / active_time_per_face
laser_period  = active_time_per_face / shots_per_face
```

예:

```text
active_PRF   = 450 / 6.25 ms = 72 kHz
laser_period = 6.25 ms / 450 = 13.8889 us
```

## 3. 기하광학 해석

현재 시뮬레이터의 반사 계산은 다음 관계를 쓴다.

```text
reflected_angle = 2 x mirror_tangent_angle - incident_angle
```

단, 사용자가 정정한 라이다 구조 기준으로 입사 광원 시작점은 180도 위치에 고정된다. `고정 광원->면 중심` 모드에서는:

```text
source point = absolute 180 deg fixed
target point = current face center
incident_angle = angle(source point -> target point)
normal_relative_source_angle = angle(face normal -> source point)
laser_on only if 90 deg <= normal_relative_source_angle <= 180 deg
```

즉, 기하광학 화면에서 레이저가 보이려면 두 조건이 동시에 만족되어야 한다.

```text
1. 현재 미러 phase가 active window 안에 있음
2. 고정 광원이 현재 face normal 기준 Q2에 있음
```

## 4. 이번 HTML 반영

| 항목 | 반영 내용 |
|---|---|
| 실제 laser gate | `scan.active && incidentQ2Allowed` |
| rest window | incident guide만 dashed line으로 표시, reflected ray/trail 미누적 |
| active/Q2 ON | incident arrow, reflected ray, trail 표시 |
| Optical diagnostics | `Timing coupling`, `PRF coupling`, `Current shot`, `Laser gate` 행 추가 |

## 5. 현재 한계

HTML 애니메이션의 재생 속도는 화면 표시용이다. 즉, 브라우저 animation speed가 실제 20 Hz/1200 rpm을 시간적으로 정확히 재생하는 것은 아니다. 그러나 표시된 phase, active/rest 판정, shot index, PRF 계산은 같은 수식으로 연결된다.

또한 `고정 광원->면 중심` 모드는 보조 steering optics가 있다고 보는 이상화 모델이다. 보조 steering이 없다면 실제 물리 모드는 `fixed source + fixed/manual ray + polygon first-hit`이 되어야 한다.

## 6. 기하광학 정정 (2026-07-08 19:20:09 KST)

섹션 3의 모델이 구현 수준에서 기하학적으로 틀려 있었고, 다음과 같이 정정했다.

| 결함 | 증상 | 정정 |
|---|---|---|
| 조준 face 선택 | timing faceIdx를 polygon edge index로 직접 사용해, 180도 광원 반대편(외향 법선 22~67도) face의 뒷면을 조준. active window 동안 반사각이 221~305도로 계산됨 | 고정 광원을 target sector 중심(90도)으로 반사시키는 법선 방위각 135도에 가장 가까운 face를 선택 |
| 법선 부호 | `tangent + 90`은 CCW winding polygon에서 내향 법선. 이 값 기준의 Q2 판정은 사실상 뒷면 조사를 통과시키는 조건이었음 | face 중점 방위각과 비교해 외향 법선을 판별. Q2 gate는 내향 법선 기준 unsigned 각도 90~180도로 재정의(= 전면 조사와 등가) |
| 차폐 미검사 | 조준 ray가 다른 face를 먼저 통과해도 그대로 반사 계산 | 첫 교점 edge가 조준 face와 다르면 laser gate OFF |
| 면수 의존 offset | polygon 첫 vertex offset 고정 45도는 4각에서만 우연히 정렬. 5각은 active window와 반사 sector가 18도 어긋남 | offset을 `135 - 360/면수`로 일반화 |

정정 후 실측: 4각 trail 34.8~129.8도(span 95.0도), 5각 trail 32.7~128.9도(span 96.2도). span이 H FoV 90도보다 약 5~6도 큰 것과 34~45도 구간의 orange fringe는 유한 거리 광원이 face center를 조준하는 이상화 모델의 시차이며, 광원 거리를 늘리면 45~135도에 수렴한다.

수식 정정: 섹션 3의 `laser_on only if 90 deg <= normal_relative_source_angle <= 180 deg`에서 `normal_relative_source_angle`은 **내향 법선 기준 unsigned 각도**로 해석해야 하며, 이는 외향 법선 기준 입사각 90도 미만(전면 조사)과 동일 조건이다.
