# Checkpoint K1-3: RTL/HTML 운용 매트릭스 정합성

## 1. 목적

K1-3은 처리 조건을 바꾸었을 때 RTL에서 실제로 관측되는 시간과 데이터 구조가
HTML 운용 계산기의 결과와 같은 계약을 따르는지 자동 비교한다. 이 단계의 핵심은
단순한 화면 계산이 아니라 다음 세 계층을 하나의 Golden 계약으로 묶는 것이다.

1. `tdc_gpx_lidar_ctrl_v2_top` 전체 경로에서 측정한 Shot 수집 시간;
2. Rise/Fall topology와 AXIS 폭에 따라 생성되는 HSIZE, VSIZE, STRIDE와 Beat 수;
3. RPM, 인접한 레이저 발사 후보점 사이의 요청 광학각
   (`OPTICAL_SHOT_INTERVAL_UDEG`), 레이저 목표 왕복시간
   (2R/c, `TARGET_RANGE_WINDOW_5NS`)과 Runtime Return 수를 조합한 운용 판정.

J9/J10은 이미 확정된 한 데이터 형식의 DDR Word와 PS/Ethernet Byte를 비교한다.
K1-3은 그 형식을 사용하는 운용 조건 전체가 시간 예산 안에 들어오는지를 비교한다.
따라서 두 검증은 서로 대체하지 않는다.

## 2. 검증 자산

| 자산 | 역할 |
|---|---|
| `tb_tdc_gpx_lidar_ctrl_v2_k05.vhd` | 실제 Top에서 레이저 목표 왕복시간 (`TARGET_RANGE_WINDOW_5NS`), Return 수, Processing/TDC 클럭 관계와 첫 Shot Line 완료시간 측정 |
| `tb_lidar_gpx_axis_output_subsystem.vhd` | slope topology, STOP 수, Return 수와 32/64/128-bit 출력 geometry 측정 |
| `run_v2_k13_operating_matrix.ps1` | RTL을 한 번 컴파일하고 40개 profile을 실행하여 구조화된 측정값 보관 |
| `verify_v2_k13_operating_matrix.mjs` | RTL 측정값, 체크인된 Golden JSON, HTML 실행 모델을 자동 비교 |
| `v2_k13_operating_matrix_golden.json` | 검토가 끝난 계산식과 허용 범위를 고정한 단일 Golden Vector |
| `C08_HDL_HTML_Alignment_260808_V2_Operating_Matrix_Simulator_v027.html` | 사용자가 조건을 바꾸며 PASS/CHECK와 계산 순서를 확인하는 운용 도구 |

## 3. 시간 기준과 사건 정의

모든 시간 이름은 사건의 시작점과 끝점을 함께 표현한다.

| 측정값 | 시작 사건 | 끝 사건 | 단위 |
|---|---|---|---|
| `shot_to_start_clks` | Shot 요청 승인 | 동기화된 물리 `fire_done`을 승인하고 `start_tdc`를 발생시키는 측정 시작 기준시점 (T0) | Processing clock |
| `start_to_stop_clks` | 측정 시작 기준시점 (T0) | 레이저 목표 왕복시간 (2R/c) 대기 종료인 `stop_tdc` | Processing clock |
| `stop_to_line_clks` | STOP_TDC | 첫 Shot Line의 마지막 Beat가 실제 handshake된 시점 | Processing clock |
| `shot_to_line_clks` | Shot 요청 승인 | 첫 Shot Line의 마지막 Beat handshake | Processing clock |

`stop_to_line_clks`는 AXIS `TVALID`만 올라온 시점이 아니라 `TVALID=1`과
`TREADY=1`이 동시에 성립한 마지막 Beat를 기준으로 한다. 전체 측정은
Processing→TDC Shot CDC와 TDC→Processing 결과 CDC의 독립 경계를 통과한다.
각 경계가 최대 1 Processing clock의 위상차를 만들 수 있으므로 Golden은 nominal
baseline의 `-2..+2 Processing clocks` 범위를 허용한다. 운용 예산에는 보수적인
`+2 Processing clocks`를 한 번만 더하며 topology 배율에 중복 적용하지 않는다.

외부 TDC-GPX 기준 클럭은 40 MHz이고 Tref는 25 ns이다. Runtime 레이저 목표
왕복시간 (`TARGET_RANGE_WINDOW_5NS`) 입력은 5 ns ticks이지만 실제
`Reg7.MTimer`는 25 ns 단위이므로 다음처럼 올림 양자화된다.

```text
requested_target_5ns_ticks
    -> ceil(requested / 5)
    -> Reg7.MTimer count
    -> effective_target_5ns_ticks = Reg7.MTimer count * 5
```

예를 들어 288 ticks 요청은 290 ticks, 668 ticks 요청은 670 ticks으로 실제 적용된다.

## 4. 실행 매트릭스

### 4.1 전체 Top 수집 시간: 22개 profile

- Processing/TDC: `150/200 MHz`, `200/150 MHz`
- Runtime Return: `1..7`
- AXIS 폭: `32/64/128-bit`
- 레이저 목표 왕복시간 (`TARGET_RANGE_WINDOW_5NS`): `288`, `668`, `1335`개의 5 ns ticks
- 기준 topology: 2개 Rise chip, chip당 STOP 8개, AXIS backpressure 없음

전체 조합을 무작정 곱하지 않고, Return sweep에 폭과 거리의 경계값을 교차하는
pairwise/boundary 방식으로 구성했다. 각 축의 독립 효과와 최악 조건을 확인하면서
회귀시간을 약 4분 수준으로 제한하기 위한 선택이다.

### 4.2 출력 topology: 18개 profile

- Processing: `150`, `200 MHz`
- AXIS 폭: `32/64/128-bit`
- topology:
  - 2개 Rise + 2개 Fall 전용 chip
  - 1개 chip의 Rise/Fall 동시 사용
  - 4개 chip 모두 Rise/Fall 동시 사용
- chip당 STOP 8개, Return 7개

이 매트릭스는 출력 슬롯 수와 데이터 geometry를 정확히 검증한다. 전체 Top의
시간 baseline은 현재 2개 Rise chip 경로에서 측정했으므로 다른 topology의 시간은
HTML에서 `MODEL`로 표시한다.

## 5. RTL 측정 결과

### 5.1 Runtime Return에 따른 GPX drain과 Cell 생성 시간

STOP_TDC부터 첫 Rise Shot Line의 마지막 Beat handshake까지 걸린 Processing clock이다.

| Runtime Return | 150/200 MHz | 200/150 MHz |
|---:|---:|---:|
| 1 | 306 | 343 |
| 2 | 402 | 557 |
| 3 | 515 | 786 |
| 4 | 623 | 999 |
| 5 | 757 | 1229 |
| 6 | 877 | 1442 |
| 7 | 1013 | 1671 |

TDC clock이 Processing clock보다 느린 `200/150 MHz`에서 Return 증가 비용이 더 크다.
이는 출력 폭보다 GPX IFIFO drain과 Hit/Cell 생성 경로가 현재 완료시간을 지배한다는
뜻이다. 이전 기준값보다 모든 조합이 정확히 2 Processing clocks 증가한 이유는
Cell dispatch와 Shot-Line dispatch에 각각 한 단계의 등록 경계를 추가했기 때문이다.
이 지연은 Return 수와 AXIS 폭에 따라 늘어나지 않으며 정상상태 처리율은
1 Cell/clock 및 1 Word/clock으로 유지된다.

### 5.2 AXIS 폭 결과

Return 7, 16 Cell 슬롯의 한 Shot Line과 Footer를 합친 결과다.

| AXIS 폭 | Frame Beat 수 | HSIZE | STRIDE | STOP->Line 시간 변화 |
|---:|---:|---:|---:|---|
| 32-bit | 168 | 336 B | 336 B | 기준 |
| 64-bit | 84 | 336 B | 336 B | 없음 |
| 128-bit | 42 | 336 B | 336 B | 없음 |

폭을 2배로 늘릴 때 Beat 수는 정확히 절반이 된다. 그러나 zero-backpressure 조건의
첫 Line 완료시간은 같았다. 따라서 폭 증가는 AXIS/VDMA 대역폭과 backpressure 여유를
늘리지만, 현재 baseline의 GPX drain 자체를 빠르게 만들지는 않는다.

### 5.3 topology별 출력 geometry

| topology | Shot당 Cell 슬롯 | HSIZE/STRIDE | 32/64/128-bit Beat 수 |
|---|---:|---:|---:|
| 2 Rise + 2 Fall 전용 chip | slope당 16 | 336 B | 168 / 84 / 42 |
| 1 chip Rise/Fall 동시 | slope당 8 | 176 B | 88 / 44 / 22 |
| 4 chip 모두 Rise/Fall 동시 | slope당 32 | 656 B | 328 / 164 / 82 |

한 chip에서 Rise와 Fall을 동시에 수집하는 경우와 최대 4개 chip 모두에서 양 edge를
수집하는 경우까지 RTL geometry가 동일한 함수 계약을 따르는 것을 확인했다.

## 6. HTML 운용 판정식

```text
decoded_states_per_revolution = CPR * decode_multiplier
optical_degrees_per_state     = 720 / decoded_states_per_revolution
shot_interval_states          = ceil(requested_optical_angle /
                                     optical_degrees_per_state)
candidate_interval_time       = revolution_period *
                                shot_interval_states /
                                decoded_states_per_revolution
```

다면미러 반사 때문에 광학각은 기계 회전각의 2배이다. 따라서 광학각/state 계산에
720도가 사용되며, 모터 자체의 기계 회전은 여전히 360도이다.

필수 Shot 시간은 다음 순서로 계산한다.

```text
physical encoder sample -> fire latency
+ fire_done response allowance
+ Reg7.MTimer effective target round-trip window
+ max(re-arm margin, measured GPX drain/Cell/Line completion)
```

요청 광학각으로 결정된 후보점 간 시간이 이 필수 시간보다 짧으면 `CHECK`, 같거나
길면 `PASS`이다. `CHECK`는 프로그램 오류가 아니라 해당 RPM/거리/Return/각분해능
조합으로 다음 Shot을 안전하게 보장할 수 없다는 운용 판정이다.

기본 `CPR=3600`, x4 decode에서는 광학각/state가 0.05도이다. 따라서 0.18도나
0.17도를 요청해도 4 state인 0.20도로 올림 양자화된다.

## 7. 자동 비교 판정

검증기는 다음을 모두 만족해야 PASS를 출력한다.

1. HTML 안의 실행 가능한 계산 모델과 체크인된 Golden JSON이 동일하다.
2. 22개 전체 Top profile의 레이저 목표 왕복시간 양자화와
   HSIZE/VSIZE/STRIDE/Beat 수가 같다.
3. RTL의 STOP->Line 및 Shot->Line 측정값이 Golden의 두 비동기 경계 위상 범위
   `-2..+2 Processing clocks` 안에 있다.
4. 18개 topology profile의 슬롯 수와 출력 geometry가 같다.
5. 운용 예제에 의도된 `PASS`와 `CHECK`가 모두 존재한다.

Top TB의 가상 모터는 한 Face를 처리하는 시간보다 의도적으로 빠르게 회전한다.
따라서 첫 번째 완성 Frame은 Beat, Line, SOF, Footer commit 및 실제 데이터 포함을
정확히 비교하고, 그 뒤 겹쳐 들어온 Face가 출력되면 완전한 결측 Frame만 허용한다.
부분 Frame, 부분 Footer 또는 첫 Frame 데이터의 중복은 허용하지 않는다.

최악의 Golden 예제는 Processing/TDC `200/150 MHz`, Return 7, 레이저 목표 편도
거리 (R) 1 km, 1200 RPM, 인접한 레이저 발사 후보점 사이의 요청 광학각 0.2도이며
`-526 Processing clocks`로 `CHECK`이다. 이는 느린 TDC drain을 숨기지 않고
운용자가 RPM, Return, 레이저 목표 편도 거리 또는 요청 광학각을 다시 선택해야 함을
표시한다.

## 8. Sign-off 범위와 남은 항목

K1-3에서 닫힌 범위:

- 두 routine 비동기 clock 관계;
- Runtime Return 1..7;
- AXIS 32/64/128-bit;
- 레이저 목표 왕복시간의 5 ns 입력과 25 ns GPX 양자화;
- 전용 slope, 한 chip 양 edge, 4 chip 양 edge 출력 geometry;
- RTL 측정값과 HTML Golden의 자동 비교.

K1-3에서 아직 닫지 않은 범위:

- AXIS/VDMA 장기 backpressure가 있는 운용시간;
- 다른 slope topology의 전체 Top drain 시간 실측;
- 실제 AXI VDMA, Zynq HP port, DDR cache ownership과 PS DMA API;
- PCB의 GPX 40 MHz Tref, LVDS STOP, laser/fire_done 물리 지연;
- 실제 Ethernet 전송률과 Viewer 종단 비교.

K1-4는 routine/extreme/sync CDC, bounded RTL backpressure, DDR/PS/HTML 및
package/XGUI/OOC 재검사를 완료했다. 다른 slope topology의 전체 Top drain 시간은
측정 baseline과 슬롯 배율을 사용하는 보수 모델로 관리한다. 실제 장기 VDMA
backpressure, DDR cache와 PCB 측정은 Stage L0 보드 통합 범위로 유지한다.

## 9. 실행 방법

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k13_operating_matrix.ps1
```

전체 실행 스크립트의 최종 표식은 다음과 같다.

```text
LIDAR_V2_K13_OPERATING_MATRIX_PASS
```

내부 비교기를 독립 실행하면
`LIDAR_V2_K13_RTL_HTML_OPERATING_MATRIX_PASS`를 출력한다. 전체 스크립트는 이
비교가 성공한 뒤에만 통합 표식을 출력하며, 상세 결과는 archive의
`comparison.json`에 남긴다.

실행 중 대용량 wave를 보관하지 않는다. Sign-off archive에는 profile 목록,
구조화된 RTL 측정값, 비교 결과, metric 요약과 source manifest만 남긴다.
