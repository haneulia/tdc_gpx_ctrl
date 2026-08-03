# TDC-GPX 극단 ASYNC 및 150 MHz SYNC 검증

## 1. 목적과 판정

`g_STREAM_CLK_MODE`의 양쪽 구조를 다음 순서로 검증했다.

1. AXIS 50 MHz / TDC 200 MHz, `ASYNC` 4:1 극단 조합
2. AXIS 200 MHz / TDC 50 MHz, `ASYNC` 1:4 극단 조합
3. AXIS 150 MHz / TDC 150 MHz, 동일 clock net `SYNC`

세 조합 모두 Return 7 기능 회귀, raw 28-bit I-Mode 비교, 17-bit Hit 보존,
Rising/Falling line 종료, 상태 레지스터, OOC 합성 및 post-route timing을
통과했다. 따라서 **stream clock mode 자체는 이 세 대표점에서 sign-off 가능**하다.
단, 임의 RPM/각분해능/Face 경계의 레이저 스케줄 가능 여부는 별도의 운용점
검증이며, 7절의 경계 조건을 함께 적용해야 한다.

## 2. RTL 계약 보완

### 2.1 ASYNC raw FIFO reset

XPM asynchronous FIFO는 reset이 양쪽 clock에서 충분히 길어야 한다. 기존의
16 TDC-clock 고정값은 AXIS 50/TDC 200 MHz에서 80 ns, 즉 4 AXIS clocks에
불과했다. 다음 elaboration 식으로 변경했다.

```text
reset_tdc_clks = max(16, ceil(5 * TDC_MHz / AXIS_MHz) + 1)
```

| AXIS/TDC | 계산 reset | 물리 시간 | AXIS 환산 |
|---|---:|---:|---:|
| 50/200 MHz | 21 TDC clocks | 105 ns | 5.25 clocks |
| 200/50 MHz | 16 TDC clocks | 320 ns | 64 clocks |

주파수 generic으로 합성 시 상수화되므로 runtime 변수와 연산기는 추가되지 않는다.
지원 주파수 범위에서 최대값은 21이며 기존 5-bit reset counter에 들어간다.

### 2.2 SYNC 계약

`SYNC`는 raw CDC FIFO를 제거하고 TDC raw valid/data/ready를 AXIS 경로에 직접
연결한다. 따라서 `g_AXIS_CLK_MHZ = g_TDC_CLK_MHZ`만 같게 쓰는 것으로는
충분하지 않다. 부모 설계에서 `i_axis_aclk`와 `i_tdc_clk`에 **동일한 물리
clock net**을 연결해야 한다. 독립된 두 150 MHz FCLK는 `ASYNC`로 설정한다.

## 3. 기능 회귀 결과

| 순서 | Mode | AXIS/TDC | 폭 | Shot/START/STOP | Rise/Fall TLAST | raw bus 비교 | point margin | 결과 |
|---:|---|---:|---:|---:|---:|---:|---:|---|
| 1 | ASYNC | 50/200 MHz | 128 | 3/3/3 | 3/3 | 672 | 1,858 AXIS clocks = 37.160 us | PASS |
| 2 | ASYNC | 200/50 MHz | 32 | 3/3/3 | 3/3 | 672 | 5,830 AXIS clocks = 29.150 us | PASS |
| 3 | SYNC | 150/150 MHz | 64 | 9/9/9 | 9/9 | 2,016 | 159 AXIS clocks = 1.060 us | PASS |

세 결과 모두 `STAT5=0x00000001`, `STAT6=0xF0000000`,
`STAT7=0x00000000`, `pipeline_abort=0`, `cfg_rejected=0`이다. 16 APD 채널,
전용 Rising `0011`/Falling `1100`, STOP당 Return 7을 사용했다.

SYNC testbench는 두 개의 독립 oscillator를 같은 값으로 설정하지 않고
`tdc_clk <= axis_clk`로 같은 signal을 사용했다.

## 4. 합성 구조 증명

합성 DCP에서 `*.gen_raw_async.u_raw_cdc` root만 정확히 세었다.

| Mode | 예상 | 실측 | 판정 |
|---|---:|---:|---|
| 50/200 ASYNC | 4 | 4 | PASS |
| 200/50 ASYNC | 4 | 4 | PASS |
| 150/150 SYNC | 0 | 0 | PASS |

검사는 `scripts/check_raw_stream_checkpoint.tcl`과
`scripts/run_ooc_signoff.tcl`에 반영했다. XPM 내부 descendant를 함께 세는
광범위 문자열 검색은 사용하지 않는다.

## 5. Post-route 결과

대상 소자는 `xc7z020clg484-2`, 전략은 `TIMING_EXPLORE`이다.

| Mode | AXIS/TDC | 폭 | WNS | WHS | route error |
|---|---:|---:|---:|---:|---:|
| ASYNC | 50/200 MHz | 128 | +0.250 ns | +0.071 ns | 0 |
| ASYNC | 200/50 MHz | 32 | +0.366 ns | +0.058 ns | 0 |
| SYNC | 150/150 MHz | 64 | +0.425 ns | +0.064 ns | 0 |

OOC의 `HD.CLK_SRC` 관련 methodology 경고는 부모 clock root가 없는 OOC 특성상
남으며, 실제 parent 구현에서 clock source와 XDC를 연결해 닫아야 한다.

## 6. IP 패키지와 XGUI

- standalone TDC-GPX package revision을 9로 올렸다.
- XGUI는 ASYNC에서 50/200 및 200/50 MHz를 모두 허용한다.
- SYNC는 같은 MHz만 허용하고, 도움말에 동일 물리 clock net 계약을 표시한다.
- package static/source-copy/BD local-unified CSR visibility 검사를 통과했다.
- 통합 IP package revision 19에도 `g_STREAM_CLK_MODE`를 노출했다.
- 통합 packaged OOC의 SYNC 150/150 MHz는 Echo on/off와 32/64/128-bit 전부
  `raw_cdc=0`, `blackbox=0`, `LUTAR-1=0`으로 통과했다.

## 7. 별도 발견: Face 경계 스케줄 충돌

초기 150/150 MHz 진단에서 0.48 deg 및 0.72 deg profile은 Face 전환 직후
`schedule_overrun`을 발생시켰다. 같은 profile을 150/150 ASYNC로 바꿔도 같은
시점에 재현되어 SYNC bypass나 CDC 문제는 아니다.

첫 관측은 약 439.726 us이며 직전 `shot_start` 뒤 약 73 ns 시점에
`fire_busy=1`, Laser FSM state 2, decoded count 24였다. Face 시작이 angular grid를
재정렬하는 순간 이전 Face의 shot이 아직 진행 중이면 새 Face의 첫 유효 grid
point를 건너뛰는 현재 정책에 해당한다.

따라서 다음 두 판정을 분리한다.

- Clock/CDC 구조: 본 문서의 세 조합에서 PASS
- 운용점: RPM, 각분해능, range/re-arm 시간뿐 아니라 Face 경계 첫 point 정책까지
  포함해 `schedule_overrun=0`을 확인해야 PASS

이번 SYNC sign-off는 1200 RPM, 0.2 deg, 최대거리 1000 m profile에서
`schedule_overrun=0`과 양의 159-clock margin을 확인했다. Face 경계에서 첫 point를
반드시 보존해야 한다면 Laser scheduler에 경계 예약 또는 명시적 drop 정책을
추가하는 별도 변경이 필요하다.

## 8. 산출물과 재현성

- 기능 결과: `sim_results/vivado_xsim/sessions/260803_extreme_a50_t200_async_fix_system_integration_smoke`
- 기능 결과: `sim_results/vivado_xsim/sessions/260803_extreme_a200_t50_async_system_integration_smoke`
- 기능 결과: `sim_results/vivado_xsim/sessions/260803140138_system_integration_smoke`
- 구현 결과: `signoff_results/sessions/260803_extreme_a50_t200_async_w128_a50_t200_p1111_r0011_f1100_impl`
- 구현 결과: `signoff_results/sessions/260803_extreme_a200_t50_async_w32_a200_t50_p1111_r0011_f1100_impl`
- 구현 결과: `signoff_results/sessions/260803_sync_same_net_150_w64_a150_t150_p1111_r0011_f1100_impl`

세 구현 session은 수정 중 RTL을 검증하기 위해 `AllowDirty`로 생성됐다. 각
`session.properties`와 기능 `rtl_result.json`에 당시 Git HEAD와 dirty 상태가
기록되어 있다. 본 변경 commit 뒤 동일 matrix를 실행하면 clean-session 증거로
승격할 수 있다.

Vivado 실행 중 사용자 Tcl Store 손상 경고가 발생하지만 설치 Tcl Store를 직접
로드하는 workaround를 사용했고, 기능·합성·route 판정과는 무관하다.
