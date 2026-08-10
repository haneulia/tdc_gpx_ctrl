# TDC-GPX LiDAR Controller V3 HLS Migration

V3는 V2의 검증된 운용 계약을 유지하면서, 측정 이후의 데이터 처리 경로만
Vitis HLS 2025.2로 단계적으로 교체한다. V2 RTL은 비교 기준이며 직접 수정하지
않는다.

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
| `rtl/top/` | H1~H4와 유지 RTL packer를 연결한 H5 Processing-domain Top |
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
| H6 Parent 통합 Sign-off | 다음 단계 | 상위 async FIFO 점유율, CDC/CSR/VDMA/DDR, 4-Chip Parent timing/DRC/bitstream |

H0~H4 Header의 역할, 전체 Bit Map, 생산자·소비자와 ABI 수정 규칙은
[`docs/V3_H0_H4_HEADER_CONTRACT_KO.md`](docs/V3_H0_H4_HEADER_CONTRACT_KO.md)에
기록한다. H1의 계약, 시험 행렬과 수치는
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
