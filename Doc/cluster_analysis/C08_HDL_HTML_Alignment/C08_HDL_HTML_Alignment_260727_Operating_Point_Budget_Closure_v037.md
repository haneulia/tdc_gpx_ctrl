# C08 Operating Point Budget Closure

## 1. 정정 사항

이 문서는 기존 `Return 7 PASS` 표현을 운용 조건이 명시된 판정으로 교체한다. Return 저장 용량과 GPX drain만 통과했다고 해서 임의의 탐지거리, 모터 속도, 수평 각분해능에서 Return 7을 처리할 수 있는 것은 아니다.

이번 RTL 판정은 다음 조건에 한해서 유효하다.

| 항목 | 검증 조건 |
|---|---:|
| 운용 모터 속도 | 1,200 RPM |
| 수평 광학 각분해능 | 0.2 deg, 경계 확인 0.18 deg |
| 설정 최대 탐지거리 | 1,000 m |
| Echo 자극 거리 | 950 m |
| Return | 1..7 |
| APD / slope | 16채널, rising + falling |
| 엔코더 경로 | external encoder -> motor_decoder -> laser_ctrl |
| AXIS / TDC clock | 150 / 200 MHz |
| 출력 폭 | 32, 64, 128 bit |
| VDMA backpressure | 없음, `tready=1` |

대응 HTML은 [C08-S24 운용점 시간 예산 시뮬레이터](C08_HDL_HTML_Alignment_260727_Operating_Point_Budget_Simulator_v024.html)이다.

## 2. 포인트 간격

다면 미러의 광학 편향각은 기계 회전각의 2배이므로 포인트 간격은 다음과 같다.

```text
revolution_period_us = 60,000,000 / motor_RPM
point_interval_us = revolution_period_us * horizontal_optical_resolution_deg / 720
```

1,200 RPM과 0.2 deg에서는 다음과 같다.

```text
revolution_period = 50,000 us
point_interval = 50,000 * 0.2 / 720
               = 13.888889 us
```

긴 실제 회전 주기를 그대로 시뮬레이션하지 않고 `200 us/rev + 50 deg`를 사용했다. 두 조건은 모두 포인트 간격이 13.888889 us이므로 Shot 단위 시간 검증에 대해서만 등가이다. 전체 Face의 실제 column 수나 장시간 Ethernet 처리량이 등가라는 뜻은 아니다.

## 3. RTL 계측 시간 예산

Return 7, 32-bit 대표 결과의 시간 순서는 다음과 같다.

| 순서 | 구간 | 계측값 | 시간 |
|---:|---|---:|---:|
| 1 | 최소 레이저 포인트 간격 | 2,083 AXIS clk | 13.886667 us |
| 2 | `fire_pulse` -> `start_tdc` | 9 AXIS clk | 0.060000 us |
| 3 | `start_tdc`/Shot -> `stop_tdc` | 1,010 AXIS clk | 6.733333 us |
| 4 | `stop_tdc` -> 최종 Rise/Fall VDMA `TLAST` | 848 AXIS clk | 5.653333 us |
| 5 | `fire_pulse` -> 최종 VDMA `TLAST` | 1,867 AXIS clk | 12.446667 us |
| 6 | 최종 포인트 여유 | 216 AXIS clk | 1.440000 us |

산식은 다음과 같이 닫힌다.

```text
fire_to_output = fire_to_start + range_wait + post_range_processing
               = 9 + 1,010 + 848
               = 1,867 clocks

point_margin = measured_minimum_point_interval - fire_to_output
             = 2,083 - 1,867
             = 216 clocks
```

`max_range_5ns_ticks=1335`는 6.675 us이다. 계측된 1,010 AXIS clock에는 최대거리 창과 레이저/TDC 경계 제어 및 clock 양자화가 포함된다. 실제 Echo 자극이 950 m에서 발생해도 캡처 창은 설정 최대거리 1,000 m까지 유지하므로 50 m 차이를 후단 처리 여유로 재사용하지 않는다.

설정 자극의 delay count와 계측 지연도 구분한다. 현재 계측 계약은 샘플링 edge 기준 `fire_pulse -> start_tdc = 9 clocks`이며, 이전 문서의 정적 8-clock 표기는 실제 계측값이 아니므로 Sign-off 계산에 사용하지 않는다.

## 4. Return 및 출력 폭 회귀

모든 행은 동일한 1,200 RPM, 0.2 deg, 최대거리 1,000 m 조건이다.

| Return | 폭 | Raw 28-bit words/Shot | fire->VDMA TLAST | 포인트 여유 | 판정 |
|---:|---:|---:|---:|---:|:---:|
| 1 | 32 | 32 | 1,219 clk | 864 clk | PASS |
| 2 | 32 | 64 | 1,327 clk | 756 clk | PASS |
| 3 | 32 | 96 | 1,435 clk | 648 clk | PASS |
| 4 | 32 | 128 | 1,543 clk | 540 clk | PASS |
| 5 | 32 | 160 | 1,651 clk | 432 clk | PASS |
| 6 | 32 | 192 | 1,759 clk | 324 clk | PASS |
| 7 | 32 | 224 | 1,867 clk | 216 clk | PASS |
| 7 | 64 | 224 | 1,843 clk | 240 clk | PASS |
| 7 | 128 | 224 | 1,835 clk | 248 clk | PASS |

Return 하나가 늘 때 32-bit 경로의 완료시간은 108 AXIS clock씩 증가한다. 출력 폭을 32 -> 64 -> 128 bit로 키우면 Return 7 완료시간은 1,867 -> 1,843 -> 1,835 clock으로 감소한다. 폭 증가로 전송 시간이 늘어나는 역전 현상은 이번 RTL 계측에서 나타나지 않았다.

## 5. 경계 운용점

1,200 RPM, 0.18 deg에서는 포인트 간격이 정확히 12.5 us, 1,875 AXIS clock이다. 같은 최대거리와 Return 7 처리에 1,867 clock이 필요하여 8 clock, 53.333 ns만 남았다.

| 항목 | 0.2 deg | 0.18 deg 경계 |
|---|---:|---:|
| 최소 계측 포인트 간격 | 2,083 clk | 1,875 clk |
| fire->VDMA TLAST | 1,867 clk | 1,867 clk |
| 여유 | 216 clk / 1.440 us | 8 clk / 0.053 us |
| 포인트 간격 대비 여유율 | 10.37% | 0.43% |
| RTL 결과 | PASS | PASS, 경계 경고 |

추가 reserve가 전혀 없다는 가정에서 계산되는 이론 경계는 다음과 같다.

```text
1,200 RPM에서 최소 광학 각분해능 = 0.179232 deg
0.2 deg에서 최대 모터 속도       = 약 1,339.05 RPM
```

0.18 deg 결과는 경계 검출 기능을 입증하기 위한 값이지 권장 운용점이 아니다. 제품 Sign-off에는 허용할 최소 시간 또는 비율 reserve를 별도로 확정해야 한다.

## 6. PASS의 정확한 의미

이번 `조건부 PASS`가 확인하는 범위는 다음과 같다.

1. 16 APD 채널에서 발생한 Return 1..7의 외부 GPX I-Mode 28-bit word를 읽는다.
2. 하위 Hit[16:0] 거리정보가 rising/falling 양 경로에서 보존된다.
3. 최대거리 대기와 GPX drain, cell/AXIS 포맷, VDMA `TLAST`까지가 다음 포인트 전에 끝난다.
4. 출력 폭이 커질수록 VDMA 완료시간이 줄거나 같아지는 방향을 유지한다.

다음 항목은 이번 PASS에 포함되지 않는다.

- VDMA `tready` stall 또는 DDR 장기 backpressure
- 실제 전체 Face column 수를 연속 처리하는 장시간 회귀
- Face rest 구간의 DDR batch와 Ethernet MTU Repack 지속 처리량
- 실제 보드의 GPX bus timing 및 post-implementation timing
- synthetic Shot을 사용하는 internal encoder smoke 경로의 `fire_pulse` 지연 계약
- 0.18 deg 경계에 적용할 제품용 engineering reserve 기준

따라서 `Return 7 PASS`는 단독 문장이 아니라, 최소한 `1,200 RPM / 0.2 deg / max 1,000 m / AXIS 150 MHz / TDC 200 MHz / no backpressure`와 함께 기록해야 한다.

## 7. 재현 자료

```powershell
powershell.exe -ExecutionPolicy Bypass -File system_integration/scripts/run_return_feasibility_matrix.ps1 -Stamp 260727_operating_point_matrix1
node system_integration/scripts/check_c08_v024_html.mjs
```

주요 결과는 다음 경로에 있다.

- `sim_results/vivado_xsim/sessions/260727_operating_point_matrix1_return_feasibility_matrix/return_feasibility_summary.json`
- `sim_results/vivado_xsim/sessions/260727_operating_point_final1_system_integration_smoke/rtl_result.json`
- `sim_results/vivado_xsim/sessions/260727_point_budget_boundary1_system_integration_smoke/rtl_result.json`
- `C08_HDL_HTML_Alignment_260727_Operating_Point_Budget_Simulator_v024.html`

자동 HTML/RTL 계약 검사의 최종 marker는 `C08_V024_HTML_SELF_TEST_PASS`이다.
