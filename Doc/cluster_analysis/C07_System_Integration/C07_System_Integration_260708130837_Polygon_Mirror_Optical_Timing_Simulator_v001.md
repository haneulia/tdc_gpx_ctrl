# C07 Polygon Mirror Optical Timing Simulator v001

| 항목 | 내용 |
|---|---|
| 문서 종류 | HTML 시뮬레이터 생성 기록 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-08 13:08:37 KST |
| 수정 시간 | 2026-07-08 19:49:55 KST |
| Cluster | C07 System Integration / Release Readiness |
| 절대 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| HTML 산출물 | `Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260708130837_Polygon_Mirror_Optical_Timing_Simulator_v001.html` |

## 1. 목적

사용자와 논의한 4각/5각 다면 미러, 반사법칙, TDC-GPX 4-chip 구조, VDMA DDR 적재, face 단위 Ethernet batch 전송 개념을 한 화면에서 조정할 수 있는 독립 HTML 시뮬레이터를 생성했다.

## 2. 반영된 운용 모델

| 항목 | 반영 방식 |
|---|---|
| 다면 미러 | 1각/2각/4각/5각 선택 |
| 평면 미러 모델 | 1각은 단면 평면 미러, 2각은 양면 평면 미러로 표시 |
| 반사법칙 | 광학 수평 FoV `H`를 만들기 위한 미러 기계각은 `H/2` |
| Face active window | `frame_period x (H/2) / 360` |
| Face rest window | `frame_period x ((360/N) - H/2) / 360` |
| 수평 shot 수 | `ceil(horizontal_fov / horizontal_resolution)` |
| 수직 분해능 | `vertical_fov / APD_array_count` |
| ToF | `2 x distance / c` |
| TDC read | `read_ns x APD_count x echoes / parallel_TDC_pair` |
| VDMA DDR | point payload, AXIS width/clock, DDR effective bandwidth로 rough estimate |
| Ethernet | face payload를 rest window 안에 batch 전송 가능한지 판단 |
| 시각화 | 회전 다면 미러, 현재 반사 기준면, reflected laser fan, target plane point map, zoom/hover point 정보 |
| 표현 기준 | polygon body의 실제 현재 facet edge가 입사 레이저를 받는 반사면이며, 파란 굵은 선은 그 edge와 동기화 |
| 면 분류 | 현재 active face, rest/active 상태, shot index, APD lane, 면별 담당 수직 FoV 표시 |
| 동기화 | 회전 phase와 입사/반사 레이저, 현재 목표면 APD column point를 같은 시간축으로 갱신 |
| 입사/반사 광학 표시 | 입사 레이저는 좌측 9시 방향에서 현재 반사점으로 들어오고, 반사 레이저는 같은 반사점에서 `출사각 = 2 x 반사 기준면 각도`로 표시 |
| 반사 좌표계 정정 | 화면 기준 0도=오른쪽, 90도=위쪽, 180도=왼쪽. 입사 광원 위치는 180도, 광선 진행 벡터는 0도이며, 4각/90도 face/45도 active 기계각 기준 active 반사 범위는 45~135도 |
| Active/Rest window 정정 | 4각/90도 face에서 active window는 face phase 22.5~67.5도, rest는 앞/뒤 22.5도씩 총 45도. ring과 laser fan을 이 기준으로 동기화 |
| Face별 sector 표시 | 각 face마다 기계 회전 phase 기준 `rest lead / active reflection / rest tail` 구간을 ring에 반복 표시. 4각 기본값은 F0 0~90도, F1 90~180도, F2 180~270도, F3 270~360도 phase sector |
| 360도 연속 회전 표시 | 다면 미러가 face마다 재생성되어 보이지 않도록 실제 current facet edge와 polygon을 동기화하고, ring phase marker로 현재 360도 회전 위치를 표시 |
| 최종 반사 방출 sector 정정 | 입사 레이저의 광원 위치는 180도이나 광선 진행 벡터는 0도. 반사면 접선각 `22.5~67.5도`에서 반사각은 `45~135도`이며, 4개 face 모두 이 동일 방출 sector를 반복 |
| 상태 색상 제거 | active/rest 색상 arc와 파란색/갈색 굵은 facet 강조를 제거. 미러 body는 중립색, 입사/반사 광선만 기능 색으로 표시 |
| 실제 교점 반사 모델 | 중심점 반사를 폐기하고, 180도 입사선이 회전 중인 polygon 외곽면과 처음 만나는 교점을 계산. 중심에서 해당 교점까지의 거리 `center_to_hit`를 회전각별로 표시 |
| 미러 회전 방향 | 기본값은 화면 기준 시계방향. `반시계 방향 회전` 체크박스로 반시계방향 표시를 선택 가능 |
| 법선 기준 각도 표시 | 법선 방향은 실제 hit edge의 접선각에서 계산하되, dashed normal axis는 반사면의 `face center`를 기준점으로 고정 표시 |
| 정보 표시 위치 | 캔버스 좌측 상단의 상태/각도 정보를 좌측 하단으로 이동해 광학 경로와 겹침을 줄임 |
| 반사 궤적 잔상 | 유효 반사 ray를 누적해 흐린 초록색 trail로 표시하고, 잔상에 누적된 `min_angle..max_angle`의 차이를 `Trail scanned FoV`로 표시 |
| Encoder target fire | 5000 CPR 기본값과 x1/x2/x4 decode mode를 선택해 `counts/rev`, `deg/count`, target reflected FoV count span을 계산 |
| 물리 trail / 제어 target 분리 | 초록색 trail은 실제 polygon 외곽면 첫 교점 기준의 물리 반사 FoV이고, 보라색 dashed overlay는 encoder count로 예약하려는 목표 발사 FoV |
| FoV 판정 방식 | target FoV는 `90 - H_FoV/2 .. 90 + H_FoV/2`를 기준으로 count 양자화 결과를 표시하며, 기본 90도 설정에서는 `45..135 deg`가 됨 |
| 물리 trail 차이 해석 | 실제 물리 trail이 target FoV보다 작으면 광학 구조/정렬/발사 gating 또는 mirror radius/입사 위치 모델을 별도 검토해야 함 |
| 입사 광원/진행각 분리 | 입사 광원 위치 `incident source bearing`과 실제 광선 진행 방향 `incident propagation`을 독립 입력으로 분리. 기본값은 광원 180도, 진행 0도 |
| 고정 광원/직진 입사 | 입사 광원 시작점은 절대 180도 위치에 고정. `직진 입사 beam` ON이면 ray는 0도 방향으로 직진하며 face center를 추적하지 않음 |
| 일반 ray 교점 모델 | 수평 입사선 전용 계산을 제거하고, 입력된 고정 진행각 ray와 polygon edge segment의 가장 가까운 교점을 찾는 방식으로 확장 |
| 반사광 표시 정책 | 물리 반사광은 target sector 밖이어도 항상 표시하며, target 내부는 초록색, target 밖은 주황색으로 구분 |
| 반사 법칙 표시 | 하단 `Optical diagnostics` 테이블에 `out = 2 x tangent - incident` 수식을 표시해 입사 진행각 변화가 반사각에 반영되는지 직접 확인 가능 |
| 입사 ray 정책 | `직진 입사 beam` ON에서는 실제 hit point를 회전 polygon과 직진 ray의 첫 교점으로 설정. `입사 Y offset`으로 중심 기준 위/아래 평행 이동을 적용 |
| PRF/모터/화각/분해능 연계 | `frameRate -> framePeriod`, `facet count -> face sector`, `H_FoV/2 -> active 기계각`, `ceil(H_FoV/H_res) -> shots/face`, `activeTime/shots -> laser period/active PRF`로 계산 |
| 광학 gate 연계 | 실제 incident/reflected ray와 trail은 `face active window AND first-hit`이 참일 때만 ON. rest window에서는 guide만 표시 |
| Face-center 검증 표시 | 캔버스에 현재 source 쪽 `face center` 점을 표시하고, 고정 ray의 실제 hit과 현재 face midpoint의 차이를 `Face-center aim error`로 표시 |
| Optical diagnostics 테이블 | 캔버스 좌측 하단의 상태 텍스트를 제거하고, mirror canvas 하단의 별도 표에서 교점/법선/반사각/encoder target 정보를 확인 |
| 조준 face 선택 정정 | `고정 광원→면 중심` 모드에서 timing faceIdx를 polygon edge index로 직접 쓰던 방식을 폐기. 고정 광원(180도)을 target sector 중심(90도)으로 반사시키는 법선 방위각 135도에 가장 가까운 실제 face를 조준 대상으로 선택 |
| 법선 방향 정정 | `tangent + 90`이 CCW winding polygon에서 내향 법선이 되던 문제를 정정. face 중점의 방위각과 비교해 외향 법선을 선택하고, Q2 gate는 내향 법선 기준 unsigned 각도 90~180도(= 전면 조사)로 판정 |
| 차폐 검사 | 조준 ray를 polygon 전체 edge와 교차 검사해 첫 교점이 조준 face가 아니면 laser gate OFF. 뒷면 반사/몸체 관통 표현이 원천 차단됨 |
| 면수 독립 phase offset | polygon 첫 vertex offset을 고정 45도에서 `135 - 360/면수`로 정정. 4각은 기존과 동일하고, 5각도 active window 동안 반사 fan이 45~135도 sector에 정렬됨 |
| 평면미러(1각/2각) 정렬 정정 | 미러 선분 각도를 `rotationDeg` 그대로 쓰던 것을 `rotationDeg + 45 - 180/면수`로 정정. mid-active phase에서 접선각이 45도가 되어 1각(기존 315~45도 방출), 2각(기존 135~225도 방출) 모두 45~135도 sector에 정렬. mod-180 선분 성질로 CW/CCW 공통 수식 |
| 평면미러 차폐 판정 | 2-vertex 선분은 edge 0/1이 같은 물리 선분이므로 first-hit edge index 불일치를 차폐로 오판하지 않도록 예외 처리 |
| 직진 beam / 평행 이동 모델 | 기본 광학 모델을 `직진 입사 beam`으로 재정정. 입사 ray는 180도 고정 광원에서 0도 방향으로 직진하며, `입사 Y offset`으로 다면미러 중심 기준 위/아래 평행 이동. 반사는 회전 polygon과 만나는 첫 교점에서 계산 |
| Ring phase 점 제거 | ring 위의 회전 phase 점이 폴리곤 회전 방향(기본 CW)과 무관하게 반시계로 돌아 혼동을 유발하여 제거. 현재 회전 위상은 진단표 `Mirror rotation phase` 숫자로만 표시 |

## 3. 기본값

기본값은 직전 C07 운용 논의의 대표 조건으로 설정했다.

| 항목 | 기본값 |
|---|---:|
| facets | 4 |
| frame rate | 20 Hz |
| incident source bearing | 180 deg |
| incident propagation | 0 deg |
| incident fixed source bearing | 180 deg |
| straight incident beam | ON |
| incident Y offset | 0 px |
| encoder CPR / decode | 5000 CPR / x4 |
| encoder target fire overlay | ON |
| horizontal FoV | 90 deg |
| horizontal resolution | 0.2 deg |
| vertical FoV | 20 deg |
| APD array | 16 ch |
| distance | 300 m |
| echo count | 7 |
| AXI4-Stream width | 128 bit |
| stream clock | 150 MHz |

이 기본값의 핵심 검산:

```text
active time/face = 6.25 ms
shots/face       = 450
active PRF       = 72 kHz
laser period     = 13.8889 us
```

## 4. 검증

```text
node syntax check: PASS
basic calculation check: active PRF 72kHz, laser period 13.8889us
flat mirror extension check:
  1각 rest 43.75ms
  2각 rest 18.75ms
  4각 rest  6.25ms
  5각 rest  3.75ms
encoder target check:
  5000 CPR x4 = 20000 counts/rev
  deg/count    = 0.018 deg/count
  H_FoV 90 deg target = 45..135 deg
  target counts = 1250..3750, span 2500 counts
independent incident ray check:
  incident source bearing is fixed at 180 deg
  straight beam ON: propagation is fixed 0 deg
  straight beam ON: Y offset shifts the incident line up/down from polygon center
  straight beam ON: hit point is the first polygon edge intersected by the ray
display policy check:
  physical reflection is always drawn
  target sector accepted ray = green
  target sector outside ray = orange
optical diagnostics table check:
  canvas lower-left text block removed
  bottom Optical diagnostics table contains timing coupling, PRF coupling, current shot, laser gate, hit edge, face-center error, normal anchor, edge tangent/normal, reflection law, normal-based angle, target sector status, and encoder target FoV
geometric optics correction check (2026-07-08 19:20:09 KST):
  before fix: active window 동안 반사각 221..305 deg (target 45..135 deg 밖), 조준 face는 back-lit
  after fix (node sweep, 0.05 deg step, full revolution):
    4각 CW/CCW reflected 34.0..129.8 deg, span 95.8 deg, back-lit fired 0, occluded 0
    5각 CW/CCW reflected 32.3..128.9 deg, span 96.5 deg, back-lit fired 0, occluded 0
  after fix (browser live check):
    4각 Trail scanned FoV 95.00 deg (34.78..129.8 deg), Laser gate ON = active AND Q2
    5각 Trail scanned FoV 96.24 deg (32.65..128.9 deg)
  잔여 34..45 deg 구간의 orange fringe는 유한 거리 광원이 face center를 조준하는
  이상화 모델의 시차(parallax)로, 광원 거리가 멀수록 45..135 deg에 수렴
flat mirror correction check (2026-07-08 19:36:29 KST):
  before fix (node sweep): 1각 reflected 315..45 deg (0도 중심), 2각 135..225 deg (180도 중심) -- target 밖
  after fix  (node sweep): 1각/2각 CW/CCW 모두 reflected 45.0..135.0 deg, 불연속 없음 -- PASS
  browser spot check: 2각 phase 0 reflected 270.0 deg = 2x(45-90),
    1각 phase 0 tangent 225.0/normal 135.0/reflected 90.00 deg = 수정 수식 예측과 일치
  평면미러는 조준점이 미러 중심이라 시차가 없어 45..135 deg 정확히 일치
straight beam offset correction check (2026-07-08 19:49:55 KST):
  JS syntax: PASS
  incident source: fixed 180 deg
  incident propagation: fixed 0 deg when straight beam ON
  Y offset: sourceY = polygon_center_y - offset
  hit model: first intersection between straight ray and rotating polygon edge
  laser gate: face active window AND first-hit
  Q2/face-center steering: removed from current physical model
straight beam behavior verification (2026-07-08 19:58 KST, node sweep 0.05 deg + browser):
  physical property: 평행 이동은 반사 "방향"을 바꾸지 않음(반사법칙은 면 접선각만 사용).
    offset이 바꾸는 것은 (1) 어느 face에 맞는지의 전환 phase, (2) 교점 위치, (3) 유한 미러 이탈 여부
  4각 offset +36px(+0.5R): active window 동안 단일 face 유지, reflected 45.0..135.0 deg 연속 -- 이상 정렬
  4각 offset 0px: face 전환이 active window 중앙(phase 45)에 걸려 reflected 90..135 + 225..270 두 fan으로 분할
  4각 offset -36px: reflected 225..315 deg (아래 방향)
  5각 offset +36px: reflected 48..135 deg 위주(이상 offset은 약 +41px = apothem x sin45)
  1각/2각 offset 0px: reflected 45.0..135.0 deg 정확 일치. offset +36px에서는 미러 구경(aperture) 이탈로
    coverage 1각 75% / 2각 91%로 감소 -- 유한 미러 크기의 실제 물리
  browser spot check (4각 phase 0): offset 0/+36/-36 모두 reflected 180.0 deg 동일(평행이동 방향 불변성),
    center-to-hit 0.707R/0.866R/0.866R로 교점만 이동, offset 200px는 Ray intersection none
```

## 5. 수정 이력

| 시간 | 내용 |
|---|---|
| 2026-07-08 13:08:37 KST | 4각/5각 다면 미러 HTML 시뮬레이터 v001 생성 |
| 2026-07-08 13:20:03 KST | 사용자 확인에 따라 1각 단면 평면 미러, 2각 양면 평면 미러 선택 및 시각화 지원 추가 |
| 2026-07-08 13:29:23 KST | 사용자 요청에 따라 면 분류표, 면별 담당 수직 화각, 현재 active face/shot/APD lane readout, 회전 phase에 동기화된 입사/반사 레이저 및 현재 목표면 APD column 표시 추가 |
| 2026-07-08 13:38:05 KST | 파란 굵은 선이 polygon edge처럼 이동하던 표현을 현재 반사 기준면으로 정정하고, 좌측 9시 입사 레이저와 동일 반사점 기준 reflected laser arrow로 광학 표시를 보완 |
| 2026-07-08 13:43:17 KST | 사용자 지적에 따라 반사 레이저 좌표계를 정정. 9시 입사 조건에서 반사 레이저가 90~180도 영역으로 나가도록 기본 반사 중심 135도와 y-up 각도계를 적용 |
| 2026-07-08 13:51:59 KST | 사용자 지적에 따라 미러 실제 facet edge와 입사/반사 레이저 경로를 동기화. Active window를 face phase 중앙 22.5~67.5도로 이동. 이 시점의 112.5~157.5 반사 범위 해석은 이후 14:29:29 KST에 45~135도로 재정정 |
| 2026-07-08 14:03:39 KST | 사용자 지적에 따라 반사면/휴식면을 별도 단일 표현으로 두지 않고 각 face별 sector에 rest/active/rest를 반복 표시. 다면미러는 current facet edge와 polygon을 동기화하고 phase marker를 추가해 360도 연속 회전이 보이도록 보완 |
| 2026-07-08 14:29:29 KST | 사용자 지적에 따라 입사 위치 180도와 진행 벡터 0도를 분리해 반사식을 재정의. 반사면 접선각 `22.5/45/67.5도`가 반사각 `45/90/135도`가 되도록 수정하고, 모든 face의 출력 laser fan을 45~135도 sector로 고정 |
| 2026-07-08 14:44:15 KST | 사용자 지적에 따라 active/rest 색상과 파란색/갈색 굵은 facet 강조를 제거. 중심 반사 대신 입사선과 다면미러 외곽면의 실제 첫 교점을 구하고, 그 교점에서만 반사 벡터를 계산하도록 물리 렌더러를 추가 |
| 2026-07-08 15:03:38 KST | 사용자 요청에 따라 다면미러 시각화 회전 방향을 시계방향에서 반시계방향으로 변경. 타이밍/거리 계산식은 변경 없음 |
| 2026-07-08 15:06:43 KST | 사용자 요청에 따라 기본 회전 방향을 다시 시계방향으로 되돌리고, `반시계 방향 회전` 체크박스를 추가해 방향을 선택 가능하도록 변경 |
| 2026-07-08 15:10:13 KST | 사용자 요청에 따라 실제 hit edge의 법선 기준 입사각/반사각을 표시하도록 보완하고, 캔버스 좌측 상단 정보 표시를 좌측 하단으로 이동 |
| 2026-07-08 15:15:01 KST | 사용자 요청에 따라 반사 궤적 잔상을 추가하고, 잔상으로 누적된 반사 vector 각도의 최소/최대 차이를 `Trail scanned FoV`로 표시해 목표 수평 광학 FoV와 비교 가능하도록 보완 |
| 2026-07-08 15:34:21 KST | 5000 CPR optical disk와 x1/x2/x4 encoder decode mode를 추가하고, 목표 FoV `90 - H_FoV/2 .. 90 + H_FoV/2`를 encoder count로 양자화한 보라색 target fire overlay 및 count/shot 계산을 HTML에 반영 |
| 2026-07-08 16:01:49 KST | 입사 광원 방위각 입력을 추가하고, 고정 180도 수평 입사 전용 교점/반사 계산을 임의 방위각 ray와 polygon edge 교점 기반 반사 계산으로 확장 |
| 2026-07-08 16:07:14 KST | target sector 밖 반사광도 반사 법칙에 따른 물리 반사로 항상 표시하도록 수정. target 내부는 초록색, target 밖은 주황색으로 구분하고 정보 패널에 `out = 2 x tangent - incident` 수식을 표시 |
| 2026-07-08 16:36:43 KST | 입사 ray가 polygon 중심이 아니라 현재 source 쪽을 바라보는 반사면 edge midpoint를 조준하도록 수정. 캔버스에 `face center` 표시와 `Face-center aim error` 검증 값을 추가 |
| 2026-07-08 16:52:05 KST | 입사 레이저가 회전 중인 면을 따라 움직이지 않도록 정정. 이 시점의 기준 active-center face midpoint 1회 정렬 방식은 이후 17:10:20 KST에 독립 진행각 방식으로 재정정 |
| 2026-07-08 17:10:20 KST | 사용자 지적에 따라 입사 광원 위치와 입사 진행각을 독립 입력으로 분리. 입사 ray가 face center를 자동 조준/추적하지 않도록 하고, 기존 좌측 하단 진단 정보를 맨 하단 `Optical diagnostics` 표로 이동 |
| 2026-07-08 17:31:49 KST | 사용자 지적에 따라 입사 광원 방위각 조절 시 ray가 다면미러를 향하지 않는 문제를 수정. 이 시점에는 `다면미러 중심 조준`으로 진행각을 `광원 방위각 + 180도`로 자동 계산했으나, 이후 17:39:53 KST에 `면 중심 Q2 조준`으로 재정정 |
| 2026-07-08 17:36:07 KST | 사용자 지적에 따라 법선 dashed line이 실제 hit point를 따라 이동하던 표현을 수정. 법선 방향은 동일 edge 접선에서 계산하되 표시 기준점은 반사면 `face center`로 고정하고, 진단 테이블에 `Normal anchor` 항목을 추가 |
| 2026-07-08 17:39:53 KST | 사용자 요청에 따라 입사 광원을 face normal 기준 2사분면에 배치하고 1사분면 광원 배치를 사용하지 않도록 변경. `면 중심 Q2 조준` 모드에서는 ray가 현재 face center를 직접 지향하며, 기본 source offset은 normal + 135도로 설정 |
| 2026-07-08 17:57:58 KST | 사용자 지적에 따라 직전 `광원 이동 Q2` 모델을 폐기. 라이다 다면미러 구조 검토 기준에 맞춰 입사 광원 시작점을 절대 180도에 고정하고, `고정 광원->현재 face center` 동적 ray로 변경. face normal 기준 고정 광원이 Q2에 있을 때만 laser ON |
| 2026-07-08 19:03:40 KST | 사용자 질문에 따라 PRF 주기, 모터 회전 주기, 화각, 분해능의 연계성을 재검토. 계산표의 연계식은 유지하되, 광학 렌더링의 실제 laser ON 조건을 `face active window AND Q2 gate`로 보완하고, 하단 진단표에 `Timing coupling`, `PRF coupling`, `Current shot`, `Laser gate` 항목을 추가 |
| 2026-07-08 19:20:09 KST | 사용자 지적(기하광학이 이해되지 않음)에 따라 4가지 기하 결함을 정정. (1) `고정 광원→면 중심` 모드가 timing faceIdx를 edge index로 직접 써서 반대편 face 뒷면에 반사하던 것을 법선 135도 기준 실제 대면 face 선택으로 정정, (2) `tangent+90` 내향 법선을 외향 법선 판별로 정정하고 Q2 gate를 unsigned 각도 기반 전면 조사 판정으로 재정의, (3) 조준 ray의 첫 교점이 조준 face인지 차폐 검사 추가, (4) polygon 첫 vertex offset을 `135 - 360/면수`로 일반화해 5각에서도 반사 fan이 45~135도 sector에 정렬되도록 수정. node 수치 스윕과 브라우저 실측으로 4각/5각, CW/CCW 모두 검증 |
| 2026-07-08 19:36:29 KST | 사용자 질문(1각/2각 평면미러 검토 여부)에 따라 평면미러 분기의 동일 계열 정렬 결함을 정정. 미러 선분 각도를 `rotationDeg + 45 - 180/면수`로 수정해 1각(기존 315~45도 방출)과 2각(기존 135~225도 방출)이 모두 45~135도 sector에 정렬되도록 하고, 2-vertex 선분의 first-hit edge index 동률을 차폐로 오판하지 않도록 예외 처리. node 스윕(CW/CCW 45.0~135.0도)과 브라우저 스팟 체크로 검증 |
| 2026-07-08 19:49:55 KST | 사용자 요청에 따라 `face center 지향/추적` 모델을 기본 물리 모델에서 제거하고, 입사 레이저에 직진성만 부여. `입사 Y offset`으로 다면미러 중심 기준 위/아래 평행 이동을 추가하고, 회전 중인 다각 미러면과의 첫 교점에서 입사/반사각이 동기화되어 변하도록 수정. laser gate는 `face active window AND first-hit`로 재정정 |
| 2026-07-08 19:58:00 KST | 직진 beam/평행 이동 모델의 동작 검증을 수행. node 스윕과 브라우저 실측으로 평행이동의 반사 방향 불변성, offset에 따른 face 전환 phase 이동(4각 offset 0에서 fan 분할, +36px에서 45~135도 단일 fan), 유한 미러 구경 이탈을 확인하고 검증 수치를 4절에 기록 |
| 2026-07-08 20:07:39 KST | 사용자 요청에 따라 ring 위의 회전 phase 점을 제거(폴리곤이 시계방향으로 돌 때도 점은 반시계로 돌던 표시 불일치). 캔버스 픽셀 검사로 제거 확인. 직진 빔 상/하 평행이동 반영 여부도 브라우저 실측으로 재확인: offset 0에서 좌측 face 교점 0.707R, ±60px에서는 phase 0 플랫탑 자세의 폴리곤 최고 높이(50.9px)를 벗어나 교점 없음이며 회전 중에는 재교차(정상 물리) |

## 6. Lineage

| 이전 문서/결정 | 이번 반영 |
|---|---|
| `C07_System_Integration_260708125148_100m_20Hz_4Facet_Mirror_PRF_Budget_v002.md` | 4각 다면 미러, 광학 90도 = 기계각 45도, face rest Ethernet batch 개념을 UI 계산 모델로 구현 |
| `C07_System_Integration_260708125917_300m_7Echo_4Chip_VDMA_DDR_Budget_v001.md` | 300m/7echo/4-chip/VDMA DDR 가능성 판단을 기본값과 timing budget 계산에 반영 |
| 사용자 요청 2026-07-08 | 4각/5각 선택, 광학 시뮬레이터, 회전 미러, 휴식기 표시, 거리/echo 기반 frame rate 계산, 레이저 포인트 맵과 확대/간격 확인 기능을 HTML로 구현 |
