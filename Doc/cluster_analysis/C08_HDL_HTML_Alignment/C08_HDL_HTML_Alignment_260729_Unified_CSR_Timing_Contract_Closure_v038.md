# C08 Unified CSR and Timing Contract Closure

## 1. 목적

이 문서는 `motor_decoder`, `laser_ctrl`, `echo_receiver`, `tdc_gpx_top`의 통합 CSR 경로와 Shot 처리 시간 계약을 RTL 실측값 및 C08 HTML 계산값으로 교차 검증한 결과를 기록한다.

대응 HTML은 [C08-S25 통합 CSR 타이밍 계약 시뮬레이터](C08_HDL_HTML_Alignment_260729_Unified_CSR_Timing_Contract_Simulator_v025.html)이다.

## 2. 이번에 확정한 시간 소유권

| 순서 | 값 | 소유 모듈 | 의미 |
|---:|---|---|---|
| 1 | Laser T0 response | `laser_ctrl` | `fire_pulse` 후 실제 `fire_done`이 돌아오기까지의 모델 또는 실측 지연 |
| 2 | `FIRE_DONE_TIMEOUT` | `laser_ctrl` | 명령 후 `fire_done` 미수신을 검출하는 watchdog 상한 |
| 3 | target round-trip | `echo_receiver` | 목표 거리의 왕복시간을 STOP 자극으로 만드는 지연 |
| 4 | range wait | `laser_ctrl` | `fire_done` 이후 설정 최대거리까지 다음 Shot을 막는 시간 |
| 5 | TDC drain/output | `tdc_gpx_top` | GPX read, Cell 조립, AXIS/VDMA `TLAST` 완료 시간 |

이전 통합 TB는 target round-trip 값을 Laser T0 지연에도 써서 두 지연을 직렬로 더했다. 그 결과 계획 간격 761 clock에 대해 Shot이 약 1,521 clock마다 수락되고 `schedule_overrun=1`이 발생했다.

수정 후 계약은 다음과 같다.

```text
Laser simulation T0 response = 8 x 5 ns = 40 ns
FIRE_DONE_TIMEOUT             = 16 x 5 ns = 80 ns
Echo target delay             = target-distance round trip

T0 response < FIRE_DONE_TIMEOUT <= maximum-range round trip
```

`fire_done` watchdog은 실제 응답시간의 상한이고, Echo target delay는 광학 왕복시간이다. 두 값은 같은 물리량이 아니며 서로 대체하지 않는다.

## 3. 포인트 예산 판정

마진은 실제로 수락된 Shot 사이 간격이 아니라 사용자가 요청한 각도 격자의 계획 간격을 기준으로 판정한다.

```text
point_budget_margin
    = planned_shot_interval
    - worst_fire_to_output
```

처리가 느려 Shot을 건너뛴 경우 측정 간격은 커질 수 있다. 이를 마진 계산에 사용하면 실패한 스케줄이 오히려 PASS로 보이는 위양성이 생기므로 금지한다.

Worst re-arm 계산은 다음 항목을 포함한다.

```text
worst_rearm
    = FIRE_DONE_TIMEOUT
    + maximum-range wait
    + configured re-arm guard
    + scheduler boundary overhead
```

## 4. 로컬 CSR과 통합 CSR 동등성

두 제어 모드는 동일한 scenario에서 비교했다.

| 비교 정책 | 항목 수 | 기준 |
|---|---:|---|
| 정확히 동일 | 78 | 설정, 상태, I-Mode, VDMA geometry, Shot 수, 오류 상태 |
| 최대 1 clock 허용 | 6 | Rise/Fall 완료 시각, fire-to-output, 직접 파생된 마진 |

1 clock 허용은 로컬 AXI-Lite bank와 통합 CSR adapter의 commit 출력 register 경계 차이에만 적용한다. 데이터 개수, Hit 값, 오류 상태에는 허용 오차가 없다.

| AXIS / TDC | Local | Unified | 판정 |
|---|---:|---:|:---:|
| 150 / 200 MHz | 78 exact + 6 timing | 78 exact + 6 timing | PASS |
| 200 / 200 MHz | 78 exact + 6 timing | 78 exact + 6 timing | PASS |

통합 설정 순서는 `reset -> register stage -> epoch commit -> status 확인`으로 고정했다. 여러 레지스터를 갱신하는 동안 부분 설정이 실행 경로에 노출되지 않는다.

## 5. 기준 회귀 결과

| 제어 | AXIS/TDC MHz | 계획/측정 최소 간격 | T0 | Timeout | Worst re-arm | Fire-to-output | 마진 | Overrun |
|---|---:|---:|---:|---:|---:|---:|---:|:---:|
| Local | 150/200 | 761/760 clk | 6 clk | 12 clk | 523 clk | 583 clk | 178 clk | 0 |
| Unified | 150/200 | 761/760 clk | 6 clk | 12 clk | 523 clk | 582 clk | 179 clk | 0 |
| Local | 200/200 | 1,014/1,014 clk | 8 clk | 16 clk | 696 clk | 762 clk | 252 clk | 0 |
| Unified | 200/200 | 1,014/1,014 clk | 8 clk | 16 clk | 696 clk | 761 clk | 253 clk | 0 |

모든 결과에서 `point_budget_pass=1`, `schedule_overrun=0`, Rise/Fall `TLAST` 수와 수락 Shot 수가 일치했다.

## 6. 출력 폭 회귀

통합 CSR 모드에서 150/200 MHz와 200/200 MHz를 각각 검증했다.

| `g_OUTPUT_WIDTH` | Rise beats | Fall beats | 150/200 | 200/200 |
|---:|---:|---:|:---:|:---:|
| 32 bit | 288 | 288 | PASS | PASS |
| 64 bit | 144 | 144 | PASS | PASS |
| 128 bit | 72 | 72 | PASS | PASS |

폭이 두 배가 될 때 beat 수가 절반으로 줄었다. canonical Cell/line byte 수는 변하지 않고, VDMA 전달 beat 수만 폭에 맞게 감소한다.

## 7. I-Mode와 HTML 대조

v025 자동 검사기는 다음 10개 `rtl_result.json`을 모두 읽는다.

- Local/Unified 150/200 MHz 기준 결과 2개
- Local/Unified 200/200 MHz 기준 결과 2개
- Unified 150/200 MHz의 32/64/128-bit 결과 3개
- Unified 200/200 MHz의 32/64/128-bit 결과 3개

모든 결과에서 다음 항목이 PASS다.

1. Raw GPX D[27:0] I-Mode word 수와 field 검사 수
2. D[16:0] Hit의 Rise/Fall VDMA 경로 보존
3. T0, watchdog, range, re-arm 및 point budget 계산
4. VDMA HSIZE, VSIZE, beat 수와 출력 폭 관계

`synthetic_single`은 활성 chip마다 STOP0의 1 Return만 주입한다. `physical_multi`는 설정된 STOP과 Return 전체를 주입한다. HTML은 이 자극 범위를 설계의 최대 APD/Return 처리 용량과 구분해서 비교한다.

최종 자동 검사 marker는 다음과 같다.

```text
C08_V025_HTML_SELF_TEST_PASS
```

## 8. 시뮬레이션 저장공간 보호

이전 장시간 `run all`로 약 230 GiB의 XSim WDB가 생성된 사례를 반영했다. 통합 회귀 runner는 다음 안전장치를 갖는다.

- 기본 실행 제한시간 600초
- 기본 WDB 상한 512 MiB
- 제한 초과 시 XSim process tree 종료
- 회귀 종료 시 생성 WDB 제거

이번 회귀 후 workspace WDB 잔존 수는 0이다.

## 9. Sign-off 범위와 잔여 항목

이번 단계에서 다음 항목은 sign-off 가능하다.

- Local/Unified CSR 기능 동등성
- 150 MHz 및 200 MHz AXIS 처리 clock
- 200 MHz TDC clock
- 32/64/128-bit 출력 폭
- synthetic single-return 통합 데이터 경로
- 외부 GPX I-Mode D[27:0] 읽기와 Hit[16:0] 보존
- 요청 각도 격자 기준 Shot 시간 예산

다음 항목은 시스템 제품 sign-off에 별도 증거가 필요하다.

- 실제 parent Block Design의 합성, 구현 및 timing closure
- 실제 GPX/레이저/광수신 보드에서 측정한 T0와 bus timing
- 장기 AXIS/VDMA backpressure와 DDR/Ethernet 전체 Face 부하
- 실제 16 APD 채널과 Return 1..7을 동시에 구동한 장시간 회귀
- 제품 운용점별 최소 engineering reserve 정책

따라서 현재 판정은 통합 RTL 및 HTML 계약 sign-off이며, 보드와 전체 데이터 경로의 제품 sign-off를 대신하지 않는다.

## 10. 재현 명령

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File system_integration/scripts/compare_control_mode_results.ps1 `
  -LocalResult <local rtl_result.json> `
  -UnifiedResult <unified rtl_result.json>

node system_integration/scripts/check_c08_v025_html.mjs
```
