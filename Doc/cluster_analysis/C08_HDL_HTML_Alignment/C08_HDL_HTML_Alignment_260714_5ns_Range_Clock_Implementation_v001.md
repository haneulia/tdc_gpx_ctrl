# C08 5 ns Range Clock Implementation v001

- Date: 2026-07-14
- Scope: `TDC_GPX_TOP`, `max_range_5ns_ticks`, AXIS/TDC clock-domain timing
- Simulator: `C08_HDL_HTML_Alignment_260714_5ns_Range_Clock_Simulator_v004.html`
- Previous finding: v001 review P0 `max_range_clks` clock-domain unit mixing

## 1. 결론

`max_range`는 클럭별 CSR을 늘리지 않고 하나의 물리 시간 값으로 관리하는 것이 맞다. CSR 단위를 고정 5 ns로 확정하고, TDC와 AXIS 소비 지점에서만 각 로컬 클럭 count로 올림 변환했다.

또한 `g_AXIS_CLK_MHZ <= g_TDC_CLK_MHZ`를 HDL 계약으로 적용했다. 이 조건에서는 여러 도메인을 통과하는 처리량과 end-to-end 시간 마진은 더 느린 AXIS 도메인 기준으로 판단하는 것이 맞다. 단, TDC 내부 range/drain watchdog은 TDC-local count를, AXIS 내부 orphan/cell watchdog은 AXIS-local count를 사용해야 한다. 모든 watchdog을 AXIS count 하나로 통일하는 것은 맞지 않다.

`g_AXIS_CLK_MHZ`와 `g_TDC_CLK_MHZ`는 클럭 생성기가 아니라 산술과 검증에 사용하는 elaboration-time metadata이다. 실제 MMCM/PLL 출력 및 XDC 주기와 generic이 다르면 HDL 계산이 틀려지므로 반드시 같은 값으로 관리해야 한다.

## 2. 확정 CSR 계약

Pipeline CSR #2의 `CTL1 (0x04) RANGE_COLS`를 다음과 같이 사용한다.

| Bits | 이름 | 계약 |
|---|---|---|
| `[15:0]` | `max_range_5ns_ticks` | 항상 200 MHz 기준, 1 tick = 5 ns |
| `[31:16]` | `cols_per_face` | face당 accepted shot 수 |

Software 계산식은 다음으로 고정한다.

```text
round_trip_time = 2 * max_distance / c
max_range_5ns_ticks = ceil(round_trip_time / 5 ns)
```

- 실제 AXIS/TDC 주파수와 무관하게 CSR에는 항상 5 ns tick을 쓴다.
- `0`은 기존 계약과 같이 range/window check disabled이다.
- 300 m 예시는 `ceil(2.001385 us / 5 ns) = 401`이다.
- 로컬 클럭 count를 Software가 미리 계산해서 CSR에 쓰면 안 된다.

## 3. HDL 도메인 적용

| 소비 경로 | 입력 경계 | 사용 값 | snapshot |
|---|---|---|---|
| `chip_ctrl` / `chip_run` | config CDC 후 TDC domain | `max_range_tdc_clks` | run/shot 경계 |
| `stop_cfg_decode` | AXIS domain | `max_range_axis_clks` | `shot_start` |
| `cell_pipe` / `cell_builder` | face-snapshotted config | `max_range_axis_clks` | buffer/drop/output 경계 |

변환식은 공통 package 함수 하나로 통일했다.

```text
local_clks = ceil(max_range_5ns_ticks * local_clk_mhz / 200)
```

각 경로는 snapshot 시점이 서로 다르다. `stop_cfg_decode`는 live config를 shot 경계에서 잡고, `cell_builder`는 face config와 정렬된 값을 buffer 경계에서 잡는다. 이 때문에 변환 호출 지점은 분리하되 함수와 CSR 변수는 하나만 유지했다. snapshot 의미를 합치기 위해 별도 runtime 변수를 늘리지는 않았다.

## 4. 지원 클럭과 산술

지원 값은 `50/100/125/150/200 MHz`로 제한했다. 401 tick 예시는 다음과 같다.

| Local clock | Local count | 표현 시간 |
|---:|---:|---:|
| 50 MHz | 101 | 2.020000 us |
| 100 MHz | 201 | 2.010000 us |
| 125 MHz | 251 | 2.008000 us |
| 150 MHz | 301 | 2.006667 us |
| 200 MHz | 401 | 2.005000 us |

모든 경우 올림을 사용하므로 표현 window가 CSR 요청 시간보다 짧아지지 않는다. 구현은 generic별 shift/add case이며 일반 runtime multiplier/divider를 사용하지 않는다.

- 50 MHz: `(N + 3) >> 2`
- 100 MHz: `(N + 1) >> 1`
- 125 MHz: `(5N + 7) >> 3`
- 150 MHz: `(3N + 3) >> 2`
- 200 MHz: `N`

CSR가 16-bit이고 모든 지원 클럭이 200 MHz 이하이므로 변환 결과도 원본 tick 이하이며 16-bit를 넘지 않는다. 기존 margin generic은 로컬 clock cycle 단위로 유지했다. 따라서 느린 클럭에서는 물리 margin이 길어지는 보수적 동작이다.

## 5. Clock 계약 판정

HDL elaboration assertion은 다음을 검사한다.

1. AXIS/TDC 주파수가 지원 집합에 포함되는가.
2. `g_AXIS_CLK_MHZ <= g_TDC_CLK_MHZ`인가.
3. `g_STREAM_CLK_MODE="SYNC"`이면 두 클럭이 동일한가.
4. output width와 watchdog margin 등 기존 구조 계약을 만족하는가.

AXIS가 TDC보다 빠른 조합은 계산 자체가 불가능해서가 아니라 제품의 신호처리/CDC 마진 계약에 어긋나므로 failure로 처리한다. 시간 제약과 처리량 budget은 AXIS 기준으로 닫되, TDC bus timing과 TDC-local watchdog은 계속 TDC 기준으로 검증한다.

서로 다른 클럭을 쓸 때는 raw-event 경계에 `g_STREAM_CLK_MODE="ASYNC"`를 사용해야 한다. `SYNC` bypass는 AXIS와 TDC가 실제로 같은 clock source일 때만 허용한다.

## 6. C08 v004 판정

기본값은 AXIS 150 MHz / TDC 200 MHz / 300 m이다.

| 항목 | 결과 |
|---|---:|
| CSR | `401 x 5 ns` |
| TDC local | 401 clocks |
| AXIS local | 301 clocks |
| Range clock contract | PASS |
| HDL integrity verdict | `SIM PASS / DOMAIN COUNTS SPLIT` |

HTML은 지원 클럭을 select로 제한하고, AXIS 200 MHz / TDC 150 MHz처럼 `AXIS > TDC`인 조합을 `CHECK`로 표시한다. 판단 근거 표에도 clock contract와 range CSR 변환을 별도 행으로 추가했다.

기본 C08의 shot critical path는 PASS이지만 전체 20 Hz 목표는 별도 Ethernet budget 때문에 CHECK이다. 현재 800 Mbps 조건에서 face 전송 9.648 ms가 rest window 6.250 ms보다 길다. 이것은 `max_range_5ns_ticks` 수정과 독립된 시스템 대역폭 문제이다.

## 7. 검증과 잔여 게이트

적용 검증:

- 5 ns tick 변환 unit matrix: 0/267/401/65535 x 50/100/125/150/200 MHz
- top integration: 32/64/128-bit common-clock regression
- top integration: AXIS 150 MHz / TDC 200 MHz asynchronous CDC regression
- 64-bit bounded backpressure regression
- C07 4-chip target: 64/128-bit packet regression
- HTML: 기본값, 5개 변환 count, invalid clock ordering, desktop/mobile overflow 및 browser console

검증 아카이브:

- C06 final: `sim_results/vivado_xsim/sessions/260714_range5ns_final_c06_v002_regression`
- C07 4-chip: `sim_results/vivado_xsim/sessions/260714_range5ns_c07_c07_v002_4chip_target`

HDL logic 기준 P0 단위 혼용은 닫힌다. 최종 bitstream 무결화에는 다음 구현 게이트가 남는다.

- 실제 clock wizard/MMCM 설정과 두 generic 값의 일치 확인
- XDC `create_clock`/generated clock 및 AXIS-TDC CDC timing report 확인
- 50/100/125/150/200 MHz 제품별 synthesis/implementation timing closure
- Ethernet/VDMA 실측 처리율로 C08 20 Hz system budget 재판정

고정 물리 margin이 제품 요구사항으로 추가되지 않는 한 margin용 CSR이나 주파수별 변수를 더 만들지 않는다. 현재 구조가 변수 수, 산술량, snapshot 무결성 사이의 합리적인 최소안이다.
