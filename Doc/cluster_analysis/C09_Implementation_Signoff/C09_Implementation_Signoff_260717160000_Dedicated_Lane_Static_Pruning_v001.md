# C09 Dedicated Lane Static Pruning

## 1. 목적과 판정

`DEDICATED_2X2`에서 물리적으로 사용할 수 없는 4개의 wrong-slope
`tdc_gpx_cell_builder`를 elaboration 단계에서 제거했다.

- 구현 커밋: `60769122bcc31d04ec0bf7c56521448c983479d5`
- 기능 회귀: PASS
- dedicated clean OOC 합성: PASS
- shared clean OOC 합성: PASS
- 판정: **topology 선택에 따른 정적 lane pruning 완료**

이 변경은 CSR, VDMA geometry, cell/beat 형식, face header 형식을 변경하지
않는다. 기존 top generic `g_SLOPE_CHIP_MODE`만 `tdc_gpx_cell_pipe`까지
전달하므로 런타임 변수도 추가하지 않는다.

## 2. 변경 계약

### 2.1 Compile-time topology

| Mode | Rise builder | Fall builder | 합계 |
|---|---|---|---:|
| `DEDICATED_2X2` | chip 0, 1 | chip 2, 3 | 4 |
| `SHARED_DUAL_EDGE` | chip 0, 1, 2, 3 | chip 0, 1, 2, 3 | 8 |

`tdc_gpx_cell_pipe`의 direct-instantiation 기본값은 기존 unit TB 호환을 위해
`SHARED_DUAL_EDGE`로 유지한다. 실제 top은 자신의
`g_SLOPE_CHIP_MODE` 값을 명시적으로 전달한다.

### 2.2 Runtime mask와 진단 보존

정적 topology mask와 face-snapshot runtime mask를 AND한 effective mask를
다음 항목에 공통 적용한다.

1. per-lane `shot_start`
2. drain control beat 전달
3. hit beat destination ready 판정
4. masked-slope consume-and-drop sticky

따라서 active chip 선택은 기존처럼 face 단위로 동작한다. 반대로
`DEDICATED_2X2`에서 runtime mask를 실수로 `1111`로 주어도 제거된 builder는
되살아나지 않는다. 해당 slope의 물리 hit가 들어오면 stream을 막지 않고
소비한 뒤 기존 `masked_slope_drop` sticky를 세운다.

정적으로 제거된 lane의 stream/status 출력은 0, input ready는 1로 묶었다.
downstream 배열의 4-chip 인터페이스는 유지되므로 다른 cluster의 구조 변경은
필요하지 않다.

## 3. 기능 검증

최종 회귀 session:

`sim_results/vivado_xsim/sessions/260717_lane_prune_v1_c06_v002_regression`

| 검증 항목 | 결과 |
|---|---|
| Shared lane TB | 8개 lane 모두 slice 출력 PASS |
| Dedicated lane TB | 올바른 4개 lane만 출력 PASS |
| Dedicated + runtime mask `1111` | 제거 lane이 되살아나지 않음 PASS |
| Wrong-slope hit | consume/drop + sticky PASS |
| Sticky lifecycle | abort 보존, explicit clear PASS |
| Width 32/64/128-bit top integration | PASS |
| AXIS 150 MHz / TDC 200 MHz | PASS |
| 64-bit bounded backpressure | PASS |
| STAT7 masked-slope 노출 | exact compare PASS |
| range tick, register response CDC, status aggregate | PASS |

총 13개 simulation log에서 마지막 PASS marker를 확인했고
failure/error marker는 0개였다.

## 4. Clean OOC 합성

### 4.1 Sessions

| 용도 | Session |
|---|---|
| 변경 전 dedicated baseline | `260717152600_raw_fifo_ring_clean_w32_a150_t200_dedicated_2x2_synth` |
| 변경 후 dedicated | `260717154500_lane_prune_clean_w32_a150_t200_dedicated_2x2_synth` |
| 변경 후 shared | `260717155000_lane_prune_shared_w32_a150_t200_shared_dual_edge_synth` |

세 session의 공통 조건은 output 32-bit, AXIS 150 MHz, TDC 200 MHz,
ASYNC stream이다. 변경 후 두 session은 모두 `git_state=clean`이고
`git_head=60769122bcc31d04ec0bf7c56521448c983479d5`이다.

### 4.2 Hierarchy 증거

| 항목 | 변경 전 dedicated | 변경 후 dedicated | 변경 후 shared |
|---|---:|---:|---:|
| 합성된 cell builder | 8 | **4** | 8 |
| `u_cell_pipe` LUT | 13,532 | **6,897** | 13,532 |
| `u_cell_pipe` FF | 20,962 | **10,567** | 20,962 |

변경 후 dedicated hierarchy에는 다음 4개만 존재한다.

- `gen_chip[0].gen_rise_builder.u_cell_bld_rise`
- `gen_chip[1].gen_rise_builder.u_cell_bld_rise`
- `gen_chip[2].gen_fall_builder.u_cell_bld_fall`
- `gen_chip[3].gen_fall_builder.u_cell_bld_fall`

shared hierarchy에는 chip별 rise/fall builder가 모두 존재한다.

### 4.3 전체 자원

| 자원 | 변경 전 dedicated | 변경 후 dedicated | 변화 |
|---|---:|---:|---:|
| Total LUT | 25,492 | **18,857** | -6,635 (-26.0%) |
| Logic LUT | 24,708 | **18,073** | -6,635 (-26.9%) |
| LUTRAM | 784 | 784 | 0 |
| FF | 42,838 | **32,443** | -10,395 (-24.3%) |
| Control sets | 1,939 | **1,371** | -568 (-29.3%) |

shared mode는 Total LUT 25,489, FF 42,838, control sets 1,939로
변경 전 8-builder 구조와 사실상 동일하다.

## 5. Timing, CDC, methodology

### 5.1 Dedicated timing

| Clock | WNS | TNS | Failing endpoints |
|---|---:|---:|---:|
| AXI 100 MHz | +5.957 ns | 0.000 ns | 0 |
| AXIS 150 MHz | +1.074 ns | 0.000 ns | 0 |
| TDC 200 MHz | +0.453 ns | 0.000 ns | 0 |

shared mode도 같은 WNS/TNS로 모든 timing constraint를 만족했다. critical
path가 제거 대상 builder 밖에 있어 WNS 자체는 변하지 않았지만 endpoint 수와
logic 규모는 감소했다.

### 5.2 CDC와 constraint 상태

clock-to-clock `post_synth_cdc_data.rpt`는 변경 전/후 및 두 topology에서
모두 같다.

| CDC rule | Count |
|---|---:|
| CDC-3 | 195 |
| CDC-6 | 24 |
| CDC-15 | 1,683 |
| CDC-4 / CDC-10 | 0 |

- `no_clock=0`
- `constant_clock=0`
- `unconstrained_internal_endpoints=0`
- Methodology: `TIMING-18=799`, `ULMTCS-1=1`
- OOC DRC: 기존과 같은 board-level `NSTD-1`, `UCIO-1`

`TIMING-18`, `NSTD-1`, `UCIO-1`은 parent design의 I/O delay, pin,
IOSTANDARD 제약 범위이며 OOC 내부 timing closure와 분리 관리한다.

## 6. 결론과 다음 단계

이번 변경은 기능 경로를 재배치하지 않고 존재 불가능한 상태 머신과 저장소를
제거했다. dedicated 기본 구성에서 LUT 약 26%, FF 약 24%, control set 약
29%를 줄였고, shared dual-edge 기능과 기존 진단 계약을 보존했다.

다음 sign-off 단계는 다음 순서로 진행한다.

1. 32/64/128-bit와 허용 clock 조합의 대표 경계 matrix OOC 합성
2. worst-case 조합의 post-route timing/DRC
3. board top에서 I/O delay, pin, IOSTANDARD를 포함한 최종 외부 제약 확인

