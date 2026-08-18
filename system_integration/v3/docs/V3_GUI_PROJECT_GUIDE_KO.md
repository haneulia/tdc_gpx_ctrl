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

| Profile | 구조 참조 Project | 최종 물리 결과 |
|---|---|---|
| 32-bit | `.work/v3_parent_l0/w32/project_4_lidar_v3_l0.xpr` | `.work/v3_parent_signoff/260813_200mhz_adaptive_reset_w32_impl1/post_route.dcp` |
| 64-bit | `.work/v3_parent_l0/w64/project_4_lidar_v3_l0.xpr` | `.work/v3_parent_signoff/260813_200mhz_adaptive_reset_w64_impl1/post_route.dcp` |

두 Profile은 모두 `xc7z020clg484-2`, TDC-GPX 4 Chip, Processing/AXIS
150 MHz, TDC-GPX bus/IFIFO acquisition 200 MHz를 사용한다. 최종 AXI4-Stream
출력 폭만 합성 시 32-bit 또는 64-bit로 다르다.

일반 `.xpr`에는 Block Design과 IP 설정이 있고, 최종 Sign-off는 project run
database가 아니라 별도 `post_route.dcp`로 보존되어 있다. 실행기는 Sign-off
증거를 보호하기 위해 `.xpr`를 **읽기 전용**으로 열고 다음 두 대상을 함께 표시한다.

1. `design_1_lidar_ctrl_v3.bd`: 연결과 Generic을 확인하는 읽기 전용 구조
2. `post_route.dcp`: 실제 배치·배선, Timing path와 Device 배치를 확인하는 읽기용 결과

GUI의 `Reports`에서 `SIGNOFF_W32_TIMING` 또는 `SIGNOFF_W64_TIMING`을 선택할 수
있다. 판정의 원본 증거는 checkpoint 옆의 `post_route_timing_summary.rpt`이며,
GUI report는 같은 checkpoint에서 다시 만든 열람용 report다.

개별 실행은 다음과 같다.

```powershell
./system_integration/v3/scripts/open_v3_gui_projects.ps1 -Target Vivado32
./system_integration/v3/scripts/open_v3_gui_projects.ps1 -Target Vivado64
```

이 창에서는 `Upgrade IP`, `Reset Output Products`, `Generate Output Products` 또는
Block Design 저장을 수행하지 않는다. 기존 `post_route.dcp`가 열린 상태에서 IP
생성물을 Reset하면 Sign-off 증거와 무관한 `.gen` 폴더만 부분 삭제될 수 있다.

### 2.1 최신 IP 수정 및 재검증용 Parent

IP XGUI를 확인하거나 Parent Block Design을 수정할 때는 Sign-off 열람 Project를
Upgrade하지 않고, 최신 packaged IP로 별도 writable Parent를 생성한다. 다음 명령은
32-bit Parent를 생성하고 IP lock 및 Upgrade 필요 여부를 검사한 뒤 Vivado를 연다.
실행기는 Windows 260자 경로 제한을 피하기 위해 짧은 `.work\v3gui` 물리 경로를
사용하고, Vivado에는 `V:`~`Z:` 중 비어 있는 가상 드라이브 경로를 전달한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/open_v3_custom_ip_gui.ps1 `
  -OutputWidth 32 -Recreate
```

64-bit는 `-OutputWidth 64`를 사용한다. 최초 생성 후 같은 프로젝트를 다시 열 때는
`-Recreate`를 생략할 수 있다. IP package가 변경되어 stale 또는 locked 판정이 나면
해당 writable 창을 닫고 `-Recreate`로 다시 생성한다.

새 Project는 현재 `system_integration/v3/ip_repo`를 사용하므로 생성 직후 별도
`Upgrade IP`가 필요하지 않다. Block Design의 `tdc_gpx_lidar_ctrl_v3_0`을 더블클릭하면
쓰기 가능한 `Customize IP` 화면을 확인할 수 있다. GUI에서 확인한 구조 변경을 제품에
남기려면 `.work`의 BD만 저장하지 말고 `system_integration/v3/parent_l0`의 생성 Tcl에도
같은 변경을 반영해야 한다.

가상 드라이브는 Vivado에서 IP Output Products, Synthesis, Implementation을 수행하는
동안 유지해야 한다. 관련 Vivado 창을 모두 닫은 뒤 실행 로그에 표시된 드라이브가 더
필요하지 않으면 다음처럼 해제한다.

```powershell
subst V: /D
```

실제 할당 문자는 실행 로그의 `short_map=` 값을 사용한다. 프로젝트를 긴
`C:\Projects\...` 물리 경로로 직접 다시 열면 AXI Interconnect 자동 생성 경로가
260자를 넘을 수 있으므로, 항상 위 실행기를 사용한다.

### 2.2 `Target 'Simulation' failed to remove` 오류

다음 오류는 HLS 또는 RTL 기능 오류가 아니다.

```text
IP Reset error: Target 'Simulation' failed to remove previously generated files
Unable to reset IP before upgrading
```

Sign-off 열람 창에서 `Upgrade IP`를 실행하면 Vivado가 기존 Simulation 생성물을
삭제하려 하지만, 같은 창에서 열린 BD/checkpoint 또는 다른 Vivado가 파일을 참조해
삭제가 막힐 수 있다. 이 경우 다음 순서를 따른다.

2026-08-18 사례는 Project XCI와 현재 packaged IP가 모두 VLNV `3.0`, revision
`1`이었지만, IP에 HLS 해설 문서를 추가하면서 file-group checksum이 변경되어
Vivado가 Upgrade 대상으로 표시한 경우다. RTL 포트나 Generic 변경이 아니므로 기존
Sign-off Project에 metadata Upgrade를 적용하지 않는다.

1. Sign-off 창에서는 Upgrade를 취소하고 저장하지 않는다.
2. 다른 Vivado와 실행 중인 simulation이 해당 Project를 사용하지 않는지 확인한다.
3. 수정이 필요하면 2.1의 별도 개발 Project를 최신 IP로 재생성한다.
4. 개발 Project에서 합성·배치배선 회귀를 완료한 뒤 새 Sign-off 증거를 만든다.

생성 중간 파일인 `.gen/.../sim`이 부분 삭제되어도 원본 `.bd`, `.xci`와 별도
`post_route.dcp`, Bitstream, XSA가 변경되지 않았다면 기존 물리 Sign-off 판정은
유지된다. 다만 그 Project의 simulation 생성물은 완전하지 않으므로 개발·simulation
용도로 계속 사용하지 않는다.

### 2.3 `[Common 17-680] Path length exceeds 260-Byte` 오류

이 오류는 RTL 합성 오류가 아니라 Vivado가 AXI Interconnect와 Protocol Converter의
긴 자동 생성 이름을 Project 경로 뒤에 붙이면서 Windows 260자 제한을 넘은 것이다.
2026-08-18 사례에서 기존 `.work\v3_parent_editable\w32`의 실패 XCI는 264자였다.

`open_v3_custom_ip_gui.ps1`은 다음 두 보호를 함께 적용한다.

1. 물리 Project를 짧은 `.work\v3gui\w32` 또는 `w64`에 생성한다.
2. Vivado는 이 폴더를 가리키는 `V:`~`Z:` 가상 드라이브로 Project를 연다.

32-bit 생성물 전체를 확인한 결과 물리 경로는 최대 307자였지만, Vivado가 사용하는
가상 드라이브 경로는 최대 256자였고 260자를 넘는 파일은 없었다. 따라서 파일 탐색기나
최근 Project 목록에서 긴 물리 `.xpr`을 직접 열지 않는다. 오류가 발생한 기존 Project는
닫고 2.1 명령에 `-Recreate`를 붙여 다시 생성한다. Output Products 생성이 완료된 뒤
같은 `V:/w32` 또는 해당 실행에서 배정된 드라이브의 Project에서 Synthesis와
Implementation을 수행한다.

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
