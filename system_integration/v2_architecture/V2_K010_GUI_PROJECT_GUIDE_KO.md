# TDC-GPX LiDAR Controller v2 GUI 검증 프로젝트 가이드

## 1. 목적

이 프로젝트는 K0-10 자동 검증과 동일한 packaged IP 설정을 Vivado GUI에서
직접 확인하기 위한 **독립 IP 검증 프로젝트**다. 실제 parent Block Design,
Zynq PS, AXI VDMA, HP port, DDR, 보드 핀은 포함하지 않는다.

2026-08-07 기준 세 Block Design validation, IP별 OOC 합성, black box 0,
latch 0 및 기존 프로젝트 재검증을 완료했다.

## 2. 프로젝트 위치

기본 생성 위치:

`C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/.work/tdc_gpx_lidar_ctrl_v2_gui/tdc_gpx_lidar_ctrl_v2_gui.xpr`

`.work` 아래의 Vivado 산출물은 Git에 넣지 않는다. 프로젝트를 동일하게 다시
만드는 Tcl/PowerShell과 이 가이드만 Git으로 관리한다.

## 3. 포함된 Block Design

| Block Design | Processing/TDC | CDC | AXIS 폭 | Echo Receiver |
|---|---|---|---:|---|
| `bd_async32_tdc_faster` | 150/200 MHz | ASYNC | 32 bit | 활성 |
| `bd_async128_proc_faster` | 200/150 MHz | ASYNC | 128 bit | 비활성 |
| `bd_sync64_equal` | 150/150 MHz | SYNC | 64 bit | 활성 |

각 Block Design에는 v2 IP 한 개만 배치하며 모든 활성 interface와 physical pin을
외부 port로 만든다. 따라서 GUI에서 IP의 실제 port enablement, AXIS 폭,
clock metadata와 XGUI 설정을 바로 확인할 수 있다.

## 4. GUI 확인 순서

1. 위 `.xpr` 파일을 Vivado 2025.2.1에서 연다.
2. **Flow Navigator > IP Integrator > Open Block Design**에서 세 BD를 차례로 연다.
3. `u_lidar_ctrl`을 더블 클릭하여 Customize IP 값을 확인한다.
4. `bd_async128_proc_faster`에서 Echo Receiver가 비활성이고 LVDS/STOP 관련 port가
   보이지 않는지 확인한다.
5. 32/64/128-bit profile별 Rise/Fall AXIS interface 폭을 확인한다.
6. **Design Runs**에서 아래 세 run이 `synth_design Complete!`인지 확인한다.
7. run을 우클릭하고 **Open Synthesized Design**을 선택해 hierarchy와 utilization을
   확인한다.
8. 프로젝트 루트의 `GUI_CHECK_GUIDE_KO.md`도 동일한 확인 절차를 담고 있다.

합성 Run:

- `bd_async32_tdc_faster_u_lidar_ctrl_0_synth_1`;
- `bd_async128_proc_faster_u_lidar_ctrl_0_synth_1`;
- `bd_sync64_equal_u_lidar_ctrl_0_synth_1`.

## 5. 결과 파일

프로젝트 내부 `gui_signoff_reports` 폴더에 다음 파일이 생성된다.

- `GUI_PROJECT_SUMMARY.txt`;
- profile별 `utilization_*.rpt`.

PASS marker는 다음과 같다.

`LIDAR_V2_K010_GUI_PROJECT_SIGNOFF_PASS`

최종 독립 재검증 세션:

`signoff_results/sessions/260807_211239_k010_gui_verify`

합성 후 최상위 이용량은 다음과 같다. 이 값은 IP 단독 OOC 결과이며 parent의
PS/VDMA/IOBUF와 배치배선 이용량은 포함하지 않는다.

| Block Design | LUT | FF | LUTRAM | RAMB36 | RAMB18 |
|---|---:|---:|---:|---:|---:|
| `bd_async32_tdc_faster` | 16,006 | 29,227 | 450 | 5 | 1 |
| `bd_async128_proc_faster` | 15,119 | 27,605 | 450 | 5 | 1 |
| `bd_sync64_equal` | 15,451 | 28,954 | 448 | 0 | 0 |

## 6. 재생성

다음 명령을 실행한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k010_gui_project.ps1
```

- 기본 프로젝트가 없으면 세 BD를 생성하고 합성한다.
- 완성된 기본 프로젝트가 있으면 합성을 반복하지 않고 BD 설정, Run 상태,
  black box/latch 및 Echo 비활성 port를 다시 검증한다.
- 기존 프로젝트는 자동 삭제하지 않는다.

다른 위치에 새로 만들 때는 다음처럼 지정한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k010_gui_project.ps1 `
  -ProjectDir .work/tdc_gpx_lidar_ctrl_v2_gui_rebuild
```

## 7. Sign-off 경계

이 프로젝트에서 확인되는 것은 packaged v2 IP, XGUI, Block Design validation과
독립 OOC 합성이다. 다음 항목은 확인되지 않는다.

- 실제 Zynq PS FCLK/reset 연결;
- AXI VDMA와 HP port;
- DDR frame buffer 및 cache coherency;
- Ethernet 송신과 Viewer 수신;
- LVDS/GPX 보드 핀, IOSTANDARD 및 실제 parent timing.

이 항목들은 K1 이후 Stage L0 parent/보드 Sign-off에서 닫는다.
