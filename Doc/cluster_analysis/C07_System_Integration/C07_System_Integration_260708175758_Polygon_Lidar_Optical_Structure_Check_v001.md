# C07 Polygon LiDAR Optical Structure Check v001

| 항목 | 내용 |
|---|---|
| 문서 종류 | 다면미러 라이다 광학 구조 재검토 기록 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-08 17:57:58 KST |
| Cluster | C07 System Integration / Optical Timing Simulator |
| 관련 HTML | `Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260708130837_Polygon_Mirror_Optical_Timing_Simulator_v001.html` |
| 사용자 지적 | 입사 레이저 발사 시작점은 움직이면 안 되며, 180도 위치에 고정되어야 함 |

## 1. 결론

사용자 지적이 맞다. 다면미러 라이다/레이저 스캐너 구조에서 `laser emitter/collimator`는 고정된 기계 위치에 두고, 회전 다면미러 또는 스캐닝 미러가 출력 방향을 만든다. 따라서 직전 시뮬레이터의 `광원 자체를 face normal 기준 Q2 위치로 이동`시키는 모델은 폐기해야 한다.

이번 시뮬레이터 기준은 다음으로 정정한다.

| 항목 | 정정 기준 |
|---|---|
| 입사 광원 시작점 | 절대 180도 위치에 고정 |
| face center 지향 | 광원이 이동하는 것이 아니라, 고정 광원에서 현재 face center로 향하는 ray를 계산 |
| Q1/Q2 조건 | face normal 기준 고정 광원의 상대각이 Q2(90~180도)에 있을 때만 laser ON |
| Q1 조건 | laser OFF. 화면에는 dashed guide만 표시 가능 |
| 반사 계산 | laser ON일 때만 reflected ray/trail 누적 |

## 2. 근거 검토

### 2.1 레이저 스캐너 일반 구조

레이저 스캐닝 구조는 광원 자체를 scan 위치마다 이동시키기보다, 움직이는 mirror가 beam을 steer하는 구조가 기본이다. `Laser scanning` 참고 문헌은 moving mirror, polygon mirror, galvanometer mirror를 beam steering 요소로 설명한다. 이 설명은 Gerald F. Marshall의 *Handbook of Optical and Laser Scanning*을 대표 참고문헌으로 든다.

Source: https://en.wikipedia.org/wiki/Laser_scanning

이 근거에서 시뮬레이터에 반영할 핵심은 다음이다.

```text
source/emitter fixed
scanner mirror moves
beam direction changes by reflection law
```

### 2.2 LiDAR scanner/optics 구조

LiDAR scanner/optics 설명에서는 azimuth/elevation scan 방법으로 dual oscillating mirrors, polygon mirror 조합, dual-axis scanner 등을 언급한다. 또한 return collection에는 hole mirror나 beam splitter가 쓰일 수 있다고 설명한다.

Source: https://en.wikipedia.org/wiki/Lidar

이 근거에서 중요한 점은 scanner optics가 beam 방향과 수신 경로를 구성한다는 것이며, 광원 위치를 polygon face에 맞춰 움직이는 구조가 기본 모델은 아니라는 점이다.

### 2.3 최신 beam scanner 연구의 공통 구조

Microcantilever-integrated photonic circuit 연구는 laser beam scanning이 display, microscopy, 3D mapping 등에 핵심이며, scanner가 단일 light beam을 steer하는 구조임을 보인다. 이 논문은 polygon mirror 자체는 아니지만, `고정된 광원/광도파로 또는 emitter + beam steering element`라는 구조적 관점과 일치한다.

Source: https://arxiv.org/abs/2207.12374

## 3. 우리 시뮬레이터에 대한 판단

직전 구현은 다음 오류가 있었다.

```text
face normal 기준 Q2 위치로 sourceX/sourceY 자체를 이동
-> 실제 라이다 광학계의 고정 emitter 조건 위반
```

정정된 구현은 다음 구조로 바꿨다.

```text
source point = absolute 180 deg fixed
target point = current face center
incident propagation = vector(source -> target)
source-normal-relative angle = angle(face normal -> source point)
laser ON only if 90 deg <= relative angle <= 180 deg
```

## 4. 물리 해석

입사 광원이 180도에 고정되어 있고 face center를 지향하려면, 실제 하드웨어에서는 다음 중 하나가 필요하다.

| 구현 방식 | 설명 |
|---|---|
| 고정 collimator + 정렬된 polygon geometry | 특정 active window에서 고정 beam이 원하는 facet 영역을 통과하도록 기계 정렬 |
| 고정 emitter + 보조 steering optics | 광원 위치는 고정하되 내부 보조 미러/렌즈로 현재 face center를 지향 |
| fixed beam only | 보조 steering이 없으면 face center를 항상 맞출 수 없고, 회전 중인 facet의 실제 첫 교점이 hit point가 됨 |

따라서 시뮬레이터는 다음 두 모드를 명확히 구분해야 한다.

| Mode | 의미 |
|---|---|
| Manual fixed ray | 고정 광원 + 수동 진행각. polygon 첫 교점을 실제 hit으로 사용 |
| Fixed source to face center | 고정 광원 180도 + 현재 face center 지향. Q2 조건일 때만 laser ON |

## 5. HTML 반영 사항

| 항목 | 반영 |
|---|---|
| 광원 방위각 | `incidentBearingDeg = 180`으로 계산상 강제 |
| UI | `입사 광원 고정 방위각` 입력은 180도로 표시하고 비활성화 |
| 중심 조준 | `고정 광원->면 중심` ON이면 진행각은 `source -> face center` 벡터로 동적 계산 |
| Q2 gate | face normal 기준 고정 광원의 상대각이 90~180도면 ON, 아니면 OFF |
| OFF 표시 | Q2가 아니면 incident guide를 dashed gray로 표시하고 reflected ray/trail은 누적하지 않음 |

## 6. 추적 결론

이번 수정으로 시뮬레이터는 `움직이는 광원`이 아니라 `고정 광원 + 스캐닝/반사 광학계` 모델이 되었다. 이는 라이다 다면미러 구조의 일반 해석과 맞다. 다음 검토는 실제 하드웨어에서 `고정 180도 광원 -> face center`를 만들 보조 steering optics가 존재하는지 여부를 결정하는 것이다. 보조 steering이 없다면 시뮬레이터의 기본 물리 모드는 `Manual fixed ray / polygon first-hit`가 되어야 한다.
