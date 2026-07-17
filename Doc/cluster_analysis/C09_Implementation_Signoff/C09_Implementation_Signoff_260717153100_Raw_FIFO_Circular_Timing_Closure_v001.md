# C09 Raw FIFO Circular Timing Closure

## 1. 판정

`tdc_gpx_chip_ctrl`의 raw FIFO backpressure 결함과 TDC 200 MHz 합성 timing blocker를 같은 구조 변경으로 닫았다.

- 체크포인트 커밋: `e01a55963afc726c478d3846d8eab2ea60aa6d5c`
- 기능 판정: PASS
- OOC 합성 timing 판정: PASS
- 이 단계의 상태: **raw FIFO 최적화 closure 완료**
- 전체 시스템 sign-off 상태: post-route 및 clock/width matrix가 남아 있으므로 아직 최종 완료가 아니다.

## 2. 수정 전 결함

수정 전 FIFO는 8개 entry의 shift-register 배열이었다.

1. dequeue마다 모든 entry를 한 칸씩 이동했다.
2. enqueue마다 free entry를 전체 검색했다.
3. FIFO full에서 control beat가 오면 가장 오래된 data를 검색하고 tail을 다시 이동했다.
4. data는 free slot이 4개 이상일 때만 받았지만, source busy는 8개 slot이 모두 찼을 때만 올라갔다.

따라서 occupancy 5에서 data admission은 이미 막혔는데 source backpressure는 걸리지 않았다. 새 `[16c]` 시나리오로 80 TDC-clock 동안 `tready=0`을 주입한 수정 전 결과는 다음과 같다.

| 항목 | 수정 전 결과 |
|---|---:|
| 예상 data | 56 words |
| 실제 data | 51 words |
| drop | 5 words |
| 최초 drop 조건 | `free=3` |
| sticky | `raw_drop=1` |

즉, 기존 `[16b]`의 30-cycle bounded stall PASS는 reserve 임계값을 넘지 않는 정상 구간만 증명했다.

## 3. 확정한 FIFO 계약

FIFO depth는 새로운 CSR/generic 없이 8로 고정했다. 한 shot에서 발생 가능한 control beat는 IFIFO1-done과 final drain-done 두 개다.

| 용도 | Slot 수 | 의미 |
|---|---:|---|
| 정상 backlog | 4 | source busy가 올라가기 전 data 저장 |
| backpressure reaction | 2 | registered busy와 response skid에 이미 들어온 data 흡수 |
| control reserve | 2 | IFIFO1-done + final drain-done 전용 |
| 합계 | 8 | FIFO 전체 depth |

구현 규칙은 다음과 같다.

- `occupancy >= 4`이고 sink가 stall이면 registered source busy를 assert한다.
- data는 occupancy 6 미만에서만 enqueue한다.
- control은 occupancy 8 미만에서 enqueue한다.
- dequeue를 먼저 반영한 뒤 enqueue하므로 full 상태에서도 같은 cycle pop+push가 가능하다.
- final control, IFIFO1 control, data 순으로 우선순위를 두고 정상 동작에서는 세 source가 상호 배타적임을 assertion으로 검사한다.
- payload memory는 reset하지 않고 read pointer, write pointer, count만 reset한다.
- 4개 chip 모두 `8 x 40` distributed RAM(`RAM32M x 7`, chip당 LUTRAM 24)으로 추론됐다.

이 구조는 기존의 free scan, 전체 shift, data eviction, tail compaction을 제거한다. 데이터 또는 control drop은 정상 congestion 정책이 아니라 protocol-credit 위반 sticky로만 남긴다.

## 4. 기능 검증

최종 회귀 세션:

`sim_results/vivado_xsim/sessions/260717152500_raw_fifo_ring_regression`

| 검증 | 결과 |
|---|---|
| 수정 전 `[16c]` 재현 | 51/56 data, 5 drop으로 FAIL 재현 |
| 수정 후 `[16c]` count | 56/56 data PASS |
| 수정 후 `[16c]` 순서 | IFIFO1 payload -> IFIFO1-done -> IFIFO2 payload -> final PASS |
| 수정 후 sticky | raw/data/control drop 모두 0 |
| chip_ctrl 전체 TB | `ALL TESTS PASSED` |
| output width | 32/64/128-bit top integration PASS |
| clock split | AXIS 150 MHz / TDC 200 MHz PASS |
| output backpressure | 64-bit bounded backpressure PASS |
| CDC response/lane/status | 모두 PASS |

Latency 영향은 의도와 일치한다.

| 조건 | 수정 전 | 수정 후 | 판정 |
|---|---:|---:|---|
| no backpressure output done | 428 clk | 428 clk | 변화 없음 |
| bounded stall output done | 428 clk | 433 clk | source를 5 clk 먼저/더 오래 throttle한 무손실 비용 |
| output II min/max | 1/15 clk | 1/15 clk | throughput 계약 유지 |

## 5. Clean OOC 합성 결과

최종 clean 세션:

`signoff_results/sessions/260717152600_raw_fifo_ring_clean_w32_a150_t200_dedicated_2x2_synth`

세션 조건:

- `git_head=e01a55963afc726c478d3846d8eab2ea60aa6d5c`
- `git_state=clean`
- `g_OUTPUT_WIDTH=32`
- `g_AXIS_CLK_MHZ=150`
- `g_TDC_CLK_MHZ=200`
- `g_SLOPE_CHIP_MODE=DEDICATED_2X2`
- `g_STREAM_CLK_MODE=ASYNC`

### 5.1 Timing

| Clock/항목 | 수정 전 | 수정 후 | 변화 |
|---|---:|---:|---:|
| AXI WNS | +5.957 ns | +5.957 ns | 동일 |
| AXIS WNS | +1.074 ns | +1.074 ns | 동일 |
| TDC WNS | -1.765 ns | **+0.453 ns** | +2.218 ns |
| TDC TNS | -444.109 ns | **0.000 ns** | closure |
| TDC failing endpoints | 762 | **0** | closure |
| 전체 timing 판정 | FAIL | **PASS** | closure |

`TIMING-16 Large setup violation`도 196건에서 0건으로 감소했다.

### 5.2 자원

| 자원 | 수정 전 | 수정 후 | 변화 |
|---|---:|---:|---:|
| Total LUT | 28,471 | 25,492 | -2,979 (-10.5%) |
| Logic LUT | 27,783 | 24,708 | -3,075 |
| LUTRAM | 688 | 784 | +96 (4 chip x 24) |
| FF | 43,821 | 42,838 | -983 (-2.2%) |
| Control sets | 1,939 | 1,939 | 동일 |

4개 `chip_ctrl` 합산 변화가 전체 감소량과 거의 일치한다.

- LUT: 7,002 -> 4,008 (-2,994)
- FF: 5,989 -> 5,005 (-984)

### 5.3 CDC와 methodology

| 검사 | 결과 |
|---|---|
| CDC-3 | 195, 기존과 동일 |
| CDC-6 | 24, 기존과 동일 |
| CDC-15 | 1,683, 기존과 동일 |
| CDC-4 / CDC-10 | 0 |
| `unconstrained_internal_endpoints` | 0 |
| Methodology | `TIMING-18=799`, `ULMTCS-1=1`만 잔존 |
| OOC DRC | 기존과 동일한 board-level `NSTD-1`, `UCIO-1` |

`TIMING-18`, `NSTD-1`, `UCIO-1`은 parent design의 I/O delay, pin, IOSTANDARD 제약 범위이며 OOC 내부 timing closure와 분리 관리한다.

## 6. 다음 단계

raw FIFO는 더 이상 200 MHz blocker가 아니다. 다음 최적화 우선순위는 다음과 같다.

1. `DEDICATED_2X2`에서 정적으로 사용하지 않는 wrong-slope `cell_builder`를 generate 단계에서 제거한다.
2. 32/64/128-bit와 50/100/125/150/200 MHz 허용 조합의 OOC matrix를 실행한다.
3. 대표 worst-case를 post-route까지 구현해 실제 배치배선 WNS와 route DRC를 확인한다.

현재 결과는 복잡도 감소 방향이 기능, timing, 자원을 동시에 개선했음을 입증한다. 다만 전체 sign-off 선언은 위 세 단계가 끝난 뒤로 유지한다.
