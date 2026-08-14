# V3 Vivado 및 Vitis HLS GUI 프로젝트 가이드

## 1. 목적

이 문서는 검증된 `tdc_gpx_lidar_ctrl_v3`를 GUI에서 다시 열어 다음 작업을
안전하게 수행하는 절차를 고정한다.

- 4-Chip 32-bit 및 64-bit Parent Block Design 확인
- 각 폭의 최종 배치·배선 Sign-off checkpoint와 timing 확인
- H1~H4 HLS 원본 C++ 확인 및 수정
- C simulation, C synthesis, C/RTL co-simulation 재검증

한 번에 여는 명령은 다음과 같다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/open_v3_gui_projects.ps1
```

이 명령은 Vivado 창 2개와 Vitis Unified IDE 창 1개를 연다.
Vivado의 임시 GUI session 파일은 `.work/v3_gui_sessions/W32`와 `W64`로
분리되므로 두 Profile을 동시에 열어도 journal과 임시 파일이 충돌하지 않는다.

## 2. Vivado 프로젝트와 Sign-off 증거

| Profile | 편집 가능한 Project | 최종 물리 결과 |
|---|---|---|
| 32-bit | `.work/v3_parent_l0/w32/project_4_lidar_v3_l0.xpr` | `.work/v3_parent_signoff/260813_200mhz_adaptive_reset_w32_impl1/post_route.dcp` |
| 64-bit | `.work/v3_parent_l0/w64/project_4_lidar_v3_l0.xpr` | `.work/v3_parent_signoff/260813_200mhz_adaptive_reset_w64_impl1/post_route.dcp` |

두 Profile은 모두 `xc7z020clg484-2`, TDC-GPX 4 Chip, Processing/AXIS
150 MHz, TDC-GPX bus/IFIFO acquisition 200 MHz를 사용한다. 최종 AXI4-Stream
출력 폭만 합성 시 32-bit 또는 64-bit로 다르다.

일반 `.xpr`에는 재사용 가능한 Block Design과 IP 설정이 있고, 최종 Sign-off는
project run database가 아니라 별도 `post_route.dcp`로 보존되어 있다. 실행기는
각 Vivado 창에서 다음 두 대상을 함께 연다.

1. `design_1_lidar_ctrl_v3.bd`: 연결과 Generic을 확인하는 편집 가능한 설계
2. `post_route.dcp`: 실제 배치·배선, Timing path와 Device 배치를 확인하는 읽기용 결과

GUI의 `Reports`에서 `SIGNOFF_W32_TIMING` 또는 `SIGNOFF_W64_TIMING`을 선택할 수
있다. 판정의 원본 증거는 checkpoint 옆의 `post_route_timing_summary.rpt`이며,
GUI report는 같은 checkpoint에서 다시 만든 열람용 report다.

개별 실행은 다음과 같다.

```powershell
./system_integration/v3/scripts/open_v3_gui_projects.ps1 -Target Vivado32
./system_integration/v3/scripts/open_v3_gui_projects.ps1 -Target Vivado64
```

Block Design을 수정했다고 기존 `post_route.dcp`가 갱신되는 것은 아니다. 수정 후에는
Parent 재생성·합성·배치배선 Sign-off 절차를 다시 수행하고 새로운 증거 폴더를 만들어야
한다.

## 3. Vitis HLS workspace

Vitis Unified IDE는 저장소의 `.work`를 workspace로 열며 다음 네 Component를
표시한다.

| 단계 | Vitis Component | Canonical 원본 |
|---|---|---|
| H1 | `v3_hls_hit_decoder_component` | `system_integration/v3/hls/gpx_hit_decoder/` |
| H2 | `v3_hls_cell_collector_component` | `system_integration/v3/hls/gpx_cell_collector/` |
| H3 | `v3_hls_frame_assembler_component` | `system_integration/v3/hls/gpx_frame_assembler/` |
| H4 | `v3_hls_lane_word_formatter_component` | `system_integration/v3/hls/gpx_lane_word_formatter/` |

각 `.work/*_component/vitis-comp.json`은 위 Canonical `hls_config.cfg`를 직접
참조한다. 따라서 GUI의 Sources에서 C++와 Testbench를 열어 수정하면 저장소의
형상관리 대상 원본이 수정된다. 반대로 `.work/*_component/hls/` 아래 생성 RTL,
log 및 report는 도구가 다시 만드는 산출물이므로 직접 수정하지 않는다.

HLS GUI만 여는 명령은 다음과 같다.

```powershell
./system_integration/v3/scripts/open_v3_gui_projects.ps1 -Target Hls
```

## 4. HLS 수정 후 검증 순서

GUI에서 한 Component를 수정한 경우 최소 순서는 다음과 같다.

1. C Simulation: 알고리즘과 Testbench 기대값 확인
2. C Synthesis: Interface, latency, initiation interval 및 자원 확인
3. C/RTL Co-simulation: 생성 RTL이 C 결과와 같은지 확인
4. 해당 단계의 V2/V3 차등 회귀 실행
5. 150 MHz와 200 MHz 구현 회귀 실행
6. H5/H6 통합 경계를 변경했다면 상위 통합 회귀까지 실행

H1~H4 전체 검증 프로젝트를 명령행에서 재생성하는 명령은 다음과 같다.

```powershell
./system_integration/v3/scripts/run_v3_hls_hit_decoder.ps1 -Step all
./system_integration/v3/scripts/run_v3_hls_cell_collector.ps1 -Step all
./system_integration/v3/scripts/run_v3_hls_frame_assembler.ps1 -Step all
./system_integration/v3/scripts/run_v3_hls_lane_word_formatter.ps1 -Step all
```

`.work`가 삭제되어 HLS Component가 없을 때는 다음 명령이 누락된 Component를
`-Step all`로 재검증한 뒤 GUI를 연다.

```powershell
./system_integration/v3/scripts/open_v3_gui_projects.ps1 `
  -Target Hls -RebuildMissingHls
```

GUI에서 보이는 마지막 초록색 결과만으로 Sign-off를 갱신하지 않는다. 실행 스크립트의
Profile sweep, V2/V3 차등 비교와 실제 FPGA 배치·배선 결과까지 모두 통과해야 한다.

## 5. 실행 전 상태 점검

창을 열지 않고 필요한 Project, checkpoint, Bitstream, XSA 및 HLS 검증 결과의
존재와 원본 연결만 확인할 수 있다.

```powershell
./system_integration/v3/scripts/open_v3_gui_projects.ps1 -ValidateOnly
```

정상 결과는 다음 표식을 모두 포함한다.

```text
LIDAR_V3_VIVADO_GUI_PROFILE_PASS profile=W32
LIDAR_V3_VIVADO_GUI_PROFILE_PASS profile=W64
LIDAR_V3_HLS_GUI_COMPONENT_PASS component=...
LIDAR_V3_GUI_PROJECTS_VALIDATE_PASS target=All
```

## 6. 변경 책임

| 변경 대상 | 수정 위치 | 재검증 핵심 |
|---|---|---|
| HLS 알고리즘 | `system_integration/v3/hls/*/src` | CSim, CSynth, CoSim, 차등, 구현 |
| HLS ABI Header | `system_integration/v3/hls/common/include` | H1~H6 전체와 VHDL Bit 계약 |
| HLS Testbench | `system_integration/v3/hls/*/tb` | 정상·오류·경계 Profile 기대값 |
| Parent Block Design | `system_integration/v3/parent_l0`의 생성 Tcl | 32/64-bit BD 재생성 및 Sign-off |
| 생성 RTL/report | `.work/*` | 직접 수정 금지, 실행기로 재생성 |

HLS 코드의 기능과 데이터 흐름은
[`V3_HLS_CODE_READING_GUIDE_KO.md`](V3_HLS_CODE_READING_GUIDE_KO.md), Header의
Bit 계약은 [`V3_H0_H4_HEADER_CONTRACT_KO.md`](V3_H0_H4_HEADER_CONTRACT_KO.md)를
함께 참조한다.
