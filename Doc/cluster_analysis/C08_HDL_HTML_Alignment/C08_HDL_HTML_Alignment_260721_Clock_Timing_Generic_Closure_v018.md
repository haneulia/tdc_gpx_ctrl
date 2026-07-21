# C08 clock / timing generic closure

- 작성일: 2026-07-21
- 대상: `tdc_gpx_top`
- FPGA 기준: `xc7z020clg484-2`, PS FCLK 사용
- 판정: **모듈 범위 RTL 기능 및 OOC 합성 closure PASS**
- HTML: [C08 Clock/Timing Generic Contract Simulator v016](C08_HDL_HTML_Alignment_260721_Clock_Timing_Generic_Contract_Simulator_v016.html)
- 사용 설명서: [tdc_gpx_top 신호처리 상세 사용 설명서](../../tdc_gpx_top_신호처리_상세_사용_설명서.md)

## 1. 확정한 clock / 시간 계약

| 구분 | 확정 내용 |
|---|---|
| 지원 clock | TDC와 AXIS는 50/100/125/150/200 MHz 중에서 선택하며 `g_AXIS_CLK_MHZ <= g_TDC_CLK_MHZ`여야 한다. |
| clock generic 의미 | `g_TDC_CLK_MHZ`, `g_AXIS_CLK_MHZ`는 실제 clock 생성기가 아니라 elaboration 계산용 metadata이다. 실제 PS FCLK/XDC와 반드시 일치시킨다. |
| 고정 물리 시간 | power-up, recovery, ALU pulse, bus read/idle, drain, stop margin, debounce, Cell margin은 top의 `*_TIME_NS` generic으로 관리한다. |
| clock 수 변환 | `ceil(time_ns * clock_MHz / 1000)`을 elaboration에서 한 번 계산한다. runtime divider나 multiplier는 생성하지 않는다. |
| EF guard | GPX 최소 11.8 ns와 2-FF synchronizer를 반영해 `ceil(11.8 ns / TDC period) + 2 clocks`로 계산한다. |
| 거리 CSR | `max_range_5ns_ticks`는 200 MHz 기준 5 ns tick이며 TDC와 AXIS 각 도메인의 clock 수로 별도 변환한다. |
| 스캔 CSR | `max_scan_5ns_ticks`도 5 ns tick이며 AXIS clock 수로 변환한다. 0은 programmable timeout만 끄며 hard cap은 유지된다. |
| VDMA geometry | Face snapshot 뒤 3 AXIS clocks에 안정된다: 입력 포착, payload block 계산, header 합산 및 HSIZE/VSIZE 출력 순서이다. |

고정 시간 generic의 변환은 모두 정수 올림이므로 요청한 시간보다 짧아지지 않는다. 예를 들어 `g_BUS_IDLE_STABLE_TIME_NS=20480`은 TDC 50/100/125/150/200 MHz에서 각각 1024/2048/2560/3072/4096 clocks가 된다.

## 2. Generic 계층 검토 결과

사용자가 제품이나 보드 구성에서 정해야 하는 build policy는 모두 top까지 올리고, instance identity나 내부 구조에서만 결정되는 값은 해당 하위 모듈에 남겼다.

| 분류 | 처리 |
|---|---|
| chip/slope/build 한계, clock, 물리 시간, stream 폭과 mode | `tdc_gpx_top` public generic으로 노출 |
| parent 관리 | `tdc_gpx_parent_core`가 top의 25개 generic을 같은 이름과 타입으로 노출하고 direct map |
| 하위 전파 | top에서 도메인별 clock 수를 만든 뒤 `config_ctrl -> chip_ctrl -> chip_run/bus_phy/stop_cfg/err_handler`, `cell_pipe -> cell_builder`로 전달 |
| 내부 전용 | `g_CHIP_ID`, `g_SLOPE_VALUE`, FIFO width/depth, CDC stage처럼 instance identity 또는 상위값에서 파생되는 generic은 중복 노출하지 않음 |

`verify_parent_generic_parity.ps1`가 top 선언, parent 선언, parent generic map을 독립적으로 파싱한다. 최종 결과는 **top 25 / parent 25 / same-name direct map 25**이며 누락, 개명, 추가, 간접 매핑은 실행 실패로 처리한다.

## 3. Git 기준점

| Commit | 내용 |
|---|---|
| `1a5f1f9` | 물리 시간 generic, 5 ns scan CSR, top-to-child 및 parent generic 전파 |
| `3036cd7` | 200 MHz geometry 경로 폭 축소, 3단 pipeline, Face ID control path 최적화 |

## 4. RTL 회귀

- 실행: `scripts/run_c06_v002_regression.ps1 -Stamp 260721_timing200_fix -NoArchiveOnExit`
- Vivado/xsim: 2025.2.1
- 종료 코드: 0
- 실행 시간: 약 333 s

다음 항목을 함께 통과했다.

- 32/64/128-bit output과 canonical line packing
- range/scan 5 ns tick의 TDC/AXIS clock 변환
- 50/100/125/150/200 MHz 계산 계약
- 150 MHz AXIS / 200 MHz TDC 비동기 구성
- runtime Fall OFF, 3-chip 2R+1F, 1-chip dual-edge, Rise-only build
- Shot 경계와 bounded output backpressure에서 beat/TLAST 보존
- masked-slope status와 sticky lifecycle

## 5. OOC 합성 timing

모든 값은 post-synthesis timing이며 FPGA는 `xc7z020clg484-2`, output은 32-bit, topology는 `P1111/R0011/F1100`이다.

| Profile | AXI WNS | AXIS WNS | TDC WNS | 판정 |
|---|---:|---:|---:|---|
| 50/50 MHz SYNC, 최종 HEAD | +5.957 ns | +15.191 ns | +15.404 ns | PASS |
| 150/200 MHz ASYNC, 최종 HEAD | +5.957 ns | +1.858 ns | +0.800 ns | PASS |
| 200/200 MHz SYNC, 수정 전 | +5.957 ns | -0.399 ns | +0.820 ns | FAIL, 24 endpoints |
| 200/200 MHz SYNC, 수정 후 | +5.957 ns | +0.205 ns | +0.820 ns | PASS |

200 MHz 실패의 주 경로는 geometry slot 수에서 VDMA HSIZE로 이어지는 넓은 산술 경로였다. slot/block 신호를 실제 최대 범위에 맞게 6/7비트로 제한하고 일반 곱셈·나눗셈을 shift/add로 바꾸며 geometry를 3단으로 분리했다. 별도의 Face ID clock-enable 경로는 idle start 요청 시점에 run-local ID를 초기화하도록 정리했다. 수정 후 동일 200/200 MHz profile에서 failing endpoint가 24개에서 0개가 됐다.

200 MHz AXIS의 +0.205 ns는 OOC 합성 기준으로 통과하지만 여유가 크지 않다. 실제 parent가 생기면 반드시 placement/routing을 포함한 timing을 다시 확인해야 한다.

OOC session:

- `260721234500_timing_generic_final50_w32_a50_t50_p1111_r0011_f1100_synth`
- `260721233000_timing_generic_final_w32_a150_t200_p1111_r0011_f1100_synth`
- `260721231000_timing_generic_200_sync_w32_a200_t200_p1111_r0011_f1100_synth`
- `260721232000_timing_generic_200_sync_fix1_w32_a200_t200_p1111_r0011_f1100_synth`

## 6. Parent reference 검증

- session: `260721_final_timing_generic_ps_fclk_parent_ref`
- FPGA/FCLK: `xc7z020clg484-2`, 100/150/200 MHz
- generic parity: 25/25/25
- block-design contract: 58개 exact check PASS
- 판정: `PARENT_REF_VALIDATE_PASS`

현재 실제 parent 구현 프로젝트는 없다. 이 결과는 새 board-independent reference의 BD 생성, port/generic/clock/address 계약 검증이며 implementation이나 실제 DDR/VDMA 경로의 timing sign-off는 아니다.

## 7. C08 HTML 검증

C08-S16은 물리 시간 generic, TDC/AXIS clock, 5 ns scan tick을 직접 바꿀 수 있고 각 도메인의 변환 clock 수와 실제 시간을 시간순 표로 표시한다.

- 기본 200/150 MHz: TDC power-up 48, bus idle 4096, drain 256 clocks; AXIS stop margin 32, Cell quarantine 512 clocks
- TDC 125 / AXIS 100 MHz, scan 1000 ticks: scan timeout 500 AXIS clocks = 5 us
- scan tick 0: programmable timeout disabled, hard cap 65535 AXIS clocks 유지
- clock self-test와 slope self-test PASS
- Chrome 1920x1080 및 390x844에서 overflow, label 겹침, JavaScript error 없음

## 8. 최종 판정과 sign-off 경계

다음 범위는 closure 가능하다.

1. top public generic과 parent generic의 단일 관리 경로
2. 하위 모듈 timing generic의 도메인별 전파
3. 고정 물리 시간과 runtime 5 ns CSR의 구분
4. 지원 clock 조합의 기능 회귀와 50/150/200 MHz 대표 OOC 합성
5. C08 계산 모델과 RTL 시간 계약의 일치

다음은 실제 parent 구현이 생긴 뒤 별도 sign-off가 필요하다.

1. PS FCLK가 연결된 전체 design의 implementation/post-route timing
2. GPX I/O pin, input/output delay, board waveform
3. VDMA/DDR 장기 backpressure와 실제 memory arbitration
4. Ethernet repack과 1440-byte payload의 software/network 실측

따라서 현 기준은 **`tdc_gpx_top` 모듈 기능 및 OOC 합성 sign-off PASS**이고, **보드/시스템 최종 sign-off는 아직 대상 프로젝트가 없어 미실시**이다.
