# TDC-GPX LiDAR Controller V3 HLS Migration

V3는 V2의 검증된 운용 계약을 유지하면서, 측정 이후의 데이터 처리 경로만
Vitis HLS 2025.2로 단계적으로 교체한다. V2의 기능 ABI와 Golden 결과는 비교
기준으로 고정한다. V3가 공유하는 V2 제어·물리 RTL에 결함 또는 타이밍 문제가
발견되면 재현 TB를 먼저 보강한 뒤 V2/V3에 함께 적용한다.

## 구현 경계

RTL에 유지하는 항목:

- AXI4-Lite CSR, Shadow/Active, COMMIT, IRQ
- Motor decoder, Face tracker, Shot scheduler, Laser executor
- 물리 `fire_done`에서 측정 시작 기준시점 (T0)을 만드는 저지연 경로
- Echo LVDS-to-STOP 경로와 TDC-GPX bus PHY/IFIFO Drain
- 모든 CDC, XPM FIFO, Reset 동기화
- VDMA profile 안전 적용과 최종 32/64-bit AXI4-Stream packer

HLS로 단계 전환하는 항목:

- GPX I-Mode Raw28 해석과 Hit17 생성
- Return 1~7 Cell 수집과 Runtime 직렬화(전시) Return 필터
- Rise/Fall Cell 정렬, PACKED17, Shot/Hole/Footer 32-bit Word 생성

## 디렉터리

| 경로 | 역할 |
|---|---|
| `docs/` | 마이그레이션 계약과 체크포인트 결과 |
| `hls/common/include/` | H0 공통 상한·Bit field 도구와 H1~H4 의미 기반 ABI Header |
| `hls/gpx_hit_decoder/` | 첫 변환 대상 Raw28-to-Hit17 컴포넌트 |
| `hls/gpx_cell_collector/` | Hit17-to-Cell 수집·필터·오류 분류 컴포넌트 |
| `hls/gpx_frame_assembler/` | Cell-to-Frame Rise/Fall 정렬·gap·Face 종료 컴포넌트 |
| `hls/gpx_lane_word_formatter/` | Shot Metadata, PACKED17 Cell, Hole, Face Footer Word 생성 |
| `rtl/bridges/` | HLS와 V2 record 경계를 연결하는 Adapter |
| `rtl/top/` | H1~H4와 유지 RTL packer를 연결한 H5 Top, GPX bus/CDC의 H6-A 데이터 경계, V2 제어 계층과 결합한 H6-B1 통합 Top |
| `tb/` | V2/HLS 차등 및 구현 Timing Harness |
| `scripts/` | Vitis HLS 재현 실행기 |

생성 프로젝트와 로그는 저장소 루트의 `.work/` 아래에만 만들며 Git에 넣지
않는다. HLS 소스, 실행 Tcl, 테스트와 요약 결과만 형상관리한다.

## 고정 도구와 대상

- Vitis HLS 2025.2
- Vivado 2025.2 계열
- Device: `xc7z020clg484-2`
- 제품 Processing clock 검증점: 150 MHz 및 200 MHz
- H3 내부 HLS 스케줄 목표: 250 MHz 상당(4 ns), 200 MHz 실배선 여유 확보용
- 제품 AXI4-Stream 폭: 합성 시 32 또는 64 bit
- 데이터 ABI: PACKED17

## 첫 실행

```powershell
./system_integration/v3/scripts/run_v3_hls_hit_decoder.ps1 -Step all
```

`all`은 C simulation, C synthesis, RTL co-simulation 순서로 실행한다.

## 현재 체크포인트

| 단계 | 상태 | 판정 |
|---|---|---|
| H0 공통 계약/실행 환경 | 완료 | ABI 3.2 공통 Header, C++/VHDL 단일 Bit 계약, Vitis HLS/Vivado 2025.2.1 재현 확인 |
| H1 Raw28-to-Hit17 | 완료 | CSim, C/RTL co-sim, V2 차동 회귀, 150/200 MHz 배치·배선 PASS |
| H2 Hit-to-Cell | 완료 | CSim, C/RTL co-sim, V2 차등, Abort, 150/200 MHz 배치·배선 PASS |
| H3 Cell-to-Frame | 완료 | 5 topology C/RTL, V2 차등 10개, Abort, 150/200 MHz 배치·배선 PASS |
| H4 Frame-to-Word | 완료 | 5 profile C/RTL, V2 최종 Beat 직접 비교 4개, 32/64-bit 150/200 MHz 배치·배선 PASS |
| H5 혼합 RTL/HLS Top | 완료 | H1~H4와 유지 RTL packer 통합, V2 종단 차등 5개(Rise-only 포함), abort/backpressure/idle, 150/200 MHz x 32/64-bit OOC PASS |
| H6-A Parent 데이터 경계 | 완료 | GPX bus/EF 전체 Drain, async FIFO, H1~H4, V2 종단 차분 5개, 150/200·200/150 MHz OOC PASS |
| H6-B1 통합 제어·데이터 Top | 완료 | V2 CSR/Shadow/Active/COMMIT/IRQ와 V3 HLS 데이터 경로 통합, 4 Chip 기능 회귀, formatter 진단, 두 제품 clock OOC PASS |
| H6-B2 Runtime VDMA/DDR/PS | 보드 독립 범위 완료 | 7→3→7 Return VDMA ACK 원자성, 32/64-bit DDR Word Golden, Cortex-A9 PS cache 소유권 및 H-Line/Ethernet 바이트 비교 PASS |
| H6-B3A PS 보드 투입 준비 | 완료 | V2 CSR ABI 2.7, Lane별 적용·ACK, 처리-idle gate, 1-Face IRQ/3중 버퍼, Cortex-A9 BSP와 Vivado VDMA 계약 PASS |
| H6-B3B Parent 구현 | 보드 독립 범위 완료 | 4 Chip 32/64-bit BD/XCI 재개방, 32-bit 배치·배선·Bitstream/XSA, 64-bit 전체 합성 PASS |
| H6-B4 실물 보드 증거 | 다음 단계 | 실제 GPX/VDMA/DDR/cache API/Ethernet, 레이저 및 선택적 Echo LVDS 계측 |

H0~H4 Header의 역할, 전체 Bit Map, 생산자·소비자와 ABI 수정 규칙은
[`docs/V3_H0_H4_HEADER_CONTRACT_KO.md`](docs/V3_H0_H4_HEADER_CONTRACT_KO.md)에
기록한다. H0~H4 C++를 실제 함수와 상태 순서로 읽기 위한 통합 해설은
[`docs/V3_HLS_CODE_READING_GUIDE_KO.md`](docs/V3_HLS_CODE_READING_GUIDE_KO.md)에
기록한다. 검증된 H1~H4 Vitis HLS workspace와 32/64-bit Parent Vivado
Sign-off 결과를 GUI에서 여는 절차는
[`docs/V3_GUI_PROJECT_GUIDE_KO.md`](docs/V3_GUI_PROJECT_GUIDE_KO.md)에 기록한다.
Vivado Package IP GUI에서 V3의 File Groups, Generic, Interface와 XGUI를 유지보수하는
절차와 canonical 원본 소유권은
[`docs/V3_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md`](docs/V3_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md)에
기록한다.
H1의 계약, 시험 행렬과 수치는
[`docs/V3_H1_GPX_HIT_DECODER_SIGNOFF_KO.md`](docs/V3_H1_GPX_HIT_DECODER_SIGNOFF_KO.md)에
기록한다. H2의 계약과 검증 결과는
[`docs/V3_H2_GPX_CELL_COLLECTOR_SIGNOFF_KO.md`](docs/V3_H2_GPX_CELL_COLLECTOR_SIGNOFF_KO.md)에
기록하고, 테스트별 목적·수행 순서·유지보수 규칙은
[`docs/V3_H2_TESTBENCH_GUIDE_KO.md`](docs/V3_H2_TESTBENCH_GUIDE_KO.md)에
분리해 둔다. H3의 역할·Bit 계약·타이밍 결과는
[`docs/V3_H3_GPX_FRAME_ASSEMBLER_SIGNOFF_KO.md`](docs/V3_H3_GPX_FRAME_ASSEMBLER_SIGNOFF_KO.md),
테스트 소유권과 유지보수 규칙은
[`docs/V3_H3_TESTBENCH_GUIDE_KO.md`](docs/V3_H3_TESTBENCH_GUIDE_KO.md)에
기록한다. 각 단계 PASS는 해당 경계만 닫았다는 뜻이며 V3 통합 IP 또는 Parent
프로젝트 전체 Sign-off를 뜻하지 않는다.

H4의 Word/Metadata/Footer 역할과 결과는
[`docs/V3_H4_GPX_LANE_WORD_FORMATTER_SIGNOFF_KO.md`](docs/V3_H4_GPX_LANE_WORD_FORMATTER_SIGNOFF_KO.md),
테스트 소유권과 유지보수 규칙은
[`docs/V3_H4_TESTBENCH_GUIDE_KO.md`](docs/V3_H4_TESTBENCH_GUIDE_KO.md)에 기록한다.

H5의 통합 경계, 데이터 흐름, V2 종단 비교와 구현 결과는
[`docs/V3_H5_MIXED_TOP_SIGNOFF_KO.md`](docs/V3_H5_MIXED_TOP_SIGNOFF_KO.md),
테스트 수행 순서와 변경 영향은
[`docs/V3_H5_TESTBENCH_GUIDE_KO.md`](docs/V3_H5_TESTBENCH_GUIDE_KO.md)에 기록한다.

H6-A의 물리 GPX Drain~AXI 데이터 경계, 64-bit H4 내부 계약,
CDC와 배치·배선 결과는
[`docs/V3_H6_PARENT_DATA_SIGNOFF_KO.md`](docs/V3_H6_PARENT_DATA_SIGNOFF_KO.md),
테스트별 소유 범위와 필수 회귀는
[`docs/V3_H6_TESTBENCH_GUIDE_KO.md`](docs/V3_H6_TESTBENCH_GUIDE_KO.md)에 기록한다.

H6-B1의 통합 CSR/COMMIT/IRQ와 HLS 데이터 경계, 유휴 안전 계약, formatter 진단,
4 Chip 기능 회귀와 OOC 구현 결과는
[`docs/V3_H6B_INTEGRATED_TOP_CHECKPOINT_KO.md`](docs/V3_H6B_INTEGRATED_TOP_CHECKPOINT_KO.md)에
기록한다. H6-B1 PASS는 실제 DDR/PS 또는 Parent 보드 Sign-off를 뜻하지 않는다.

H6-B2의 Runtime 직렬화(전시) Return 변경, VDMA ACK 원자성, DDR Word Golden,
PS cache 소유권 및 Ethernet payload 비교는
[`docs/V3_H6B2_RUNTIME_VDMA_DDR_PS_SIGNOFF_KO.md`](docs/V3_H6B2_RUNTIME_VDMA_DDR_PS_SIGNOFF_KO.md)에
기록한다. H6-B2 PASS는 보드 없이 가능한 계약의 종료이며 실제 DMA/cache API와
물리 Ethernet은 H6-B4 보드 증거로 남는다.

H6-B3A는 [`docs/V3_H6B3A_PS_BOARD_PREFLIGHT_KO.md`](docs/V3_H6B3A_PS_BOARD_PREFLIGHT_KO.md)에
기록한다. V3는 검증된 V2 CSR ABI 2.7을 공유하고, V3 전용 PS 계층은 CTL23/24의
새 sequence로 Processing/Echo/TDC-GPX/AXIS idle을 확인한 뒤에만 VDMA를 정지한다.
Rise/Fall Lane별 실제 VDMA 적용 뒤 개별 ACK, ACK 해제 지연 중복 방지, 미처리 DDR
Frame 소유권과 실패 복구도 담당한다. S2MM-only AXI VDMA, Lane당 3개 Frame Store,
Face마다 1회 완료 IRQ를 초기화 단계에서 강제한다.

H6-B3B의 4-Chip Parent 구성, 32/64-bit VDMA XCI 계약, 핀/IOB/CDC/DRC 및
배치·배선 결과는
[`docs/V3_H6B3B_PARENT_IMPLEMENTATION_CHECKPOINT_KO.md`](docs/V3_H6B3B_PARENT_IMPLEMENTATION_CHECKPOINT_KO.md)에
기록한다. 생성·GUI·구현 절차는
[`parent_l0/README_KO.md`](parent_l0/README_KO.md)를 따른다. 실제 보드 증거는
H6-B4로 남는다.

## H1 재현 명령

```powershell
# CSim + CSynth + 네 가지 topology C/RTL co-sim
./system_integration/v3/scripts/run_v3_hls_hit_decoder.ps1 -Step all

# V2 RTL과 V3 HLS 혼합 시뮬레이션: 네 topology × 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_hit_decoder_diff.ps1

# xc7z020clg484-2 실제 OOC 합성·배치·배선: 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_hit_decoder_impl.ps1 -SkipHlsSynthesis
```

## H2 재현 명령

```powershell
# CSim + CSynth + 네 가지 topology C/RTL co-sim
./system_integration/v3/scripts/run_v3_hls_cell_collector.ps1 -Step all

# V2 RTL과 V3 HLS 혼합 시뮬레이션: 네 topology x 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_cell_collector_diff.ps1

# xc7z020clg484-2 실제 OOC 합성·배치·배선: 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_cell_collector_impl.ps1 -SkipHlsSynthesis
```

## H3 재현 명령

```powershell
# CSim + CSynth + 다섯 topology C/RTL co-sim
./system_integration/v3/scripts/run_v3_hls_frame_assembler.ps1 -Step all

# V2 RTL과 V3 HLS 혼합 시뮬레이션: 다섯 topology x 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_frame_assembler_diff.ps1 -SkipHlsSynthesis

# xc7z020clg484-2 실제 OOC 합성·배치·배선: 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_frame_assembler_impl.ps1 -SkipHlsSynthesis
```

## H4 재현 명령

```powershell
# CSim + CSynth + 다섯 Profile C/RTL CoSim
./system_integration/v3/scripts/run_v3_hls_lane_word_formatter.ps1 -Step all

# V2 전체 Lane pipeline과 최종 32/64-bit AXI Beat 직접 비교
./system_integration/v3/scripts/run_v3_gpx_lane_word_formatter_diff.ps1 -SkipHlsSynthesis

# Rise/Fall 동시 32/64-bit, 150/200 MHz OOC 배치·배선
./system_integration/v3/scripts/run_v3_gpx_lane_word_formatter_impl.ps1 -SkipHlsSynthesis
```

## H5 재현 명령

```powershell
# H1~H4 HLS와 유지 RTL packer를 연결한 Top의 V2 종단 차등
./system_integration/v3/scripts/run_v3_hls_mixed_top_diff.ps1 -SkipHlsSynthesis

# 4 Chip 최대 구성, 150/200 MHz x 32/64-bit OOC 배치·배선
./system_integration/v3/scripts/run_v3_hls_mixed_top_impl.ps1 -SkipHlsSynthesis
```

## H6-A 재현 명령

```powershell
# skid/sync FIFO stale-ready 공용 경계
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_shared_stream_boundary_diff.ps1

# 외부 GPX bus부터 최종 AXI까지 V2/V3 종단 차분 5개
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6_parent_data_diff.ps1 `
  -SkipHlsSynthesis

# 제품 두 교차 clock 조합 OOC 배치·배선
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6_parent_data_impl.ps1 `
  -SkipHlsSynthesis -ImplementationStrategy timing_explore
```

## H6-B1 재현 명령

```powershell
# 4 Chip 통합 CSR/COMMIT/Reg7/IFIFO/HLS/AXI 기능 회귀
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b_integrated_top_diff.ps1 `
  -SkipHlsSynthesis

# H4 formatter fault의 0x1A/0x27/GPX_DATA IRQ 분류 시험
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b_formatter_status.ps1

# 4 Chip 최대 물리 구성, 두 제품 clock/폭 OOC 배치·배선
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b_integrated_top_impl.ps1 `
  -SkipHlsSynthesis -ImplementationStrategy timing_explore
```

## H6-B2 재현 명령

```powershell
# 통합 Top에서 7→3→7 Return과 Rise/Fall VDMA ACK 원자성
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b_integrated_top_diff.ps1 `
  -SkipHlsSynthesis

# V3 HLS/AXIS가 만든 32/64-bit DDR 영상을 HTML/V2 Golden과 Word 비교
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b2_ddr_golden.ps1 `
  -SkipHlsSynthesis

# DDR 회귀부터 Cortex-A9 컴파일, cache 소유권, H-Line/Ethernet까지 연속 검증
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b2_ps_hline.ps1 `
  -SkipHlsSynthesis
```
