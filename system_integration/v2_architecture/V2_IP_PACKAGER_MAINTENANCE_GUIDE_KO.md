# TDC-GPX LiDAR Controller V2 IP Packager 유지보수 가이드

## 1. 관리 대상

이 문서는 `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0`을 Vivado GUI에서 확인하고
유지보수하는 절차를 설명한다. V2 package는 88개 생산 RTL을 자체 포함하며 child
`.xci` 또는 외부 `../HDL` 참조를 사용하지 않는다.

관리 대상은 다음과 같다.

- VLNV, Display Name과 Zynq-7000 호환성
- Synthesis, Simulation, XGUI, Product Guide File Group
- 20개 합성 Generic과 32/64/128-bit PACKED17 출력 선택
- AXI4-Lite CSR, AXI4-Stream, clock/reset과 TDC-GPX 물리 포트
- XGUI 네 Page와 Echo Receiver Optional port 표시 조건
- Review and Package의 IP 무결성 DRC

IP-XACT 저장과 XGUI 생성 순서는
`V2_IP_XACT_XGUI_REGENERATION_RULES_KO.md`를 강제 규칙으로 따른다.

## 2. 원본과 생성물

| 구분 | 형상관리 원본 | 비고 |
|---|---|---|
| RTL 목록 | `v2/ip_package/v2_ip_package_manifest.tcl` | 88개, 순서 포함 |
| XGUI | `v2/ip_package/tdc_gpx_lidar_ctrl_v2_xgui.tcl` | Vivado-native 형식 |
| IP-XACT 속성 | `v2/scripts/package_v2_ip.tcl` | 목록·범위·interface 포함 |
| 배포 package | `v2/ip_repo/tdc_gpx_lidar_ctrl_v2_2_0` | 스크립트 생성물 |
| GUI 편집 project | `.work/v2_ip_packager` | 삭제 후 재생성 가능 |
| Customize preview project | `.work/v2_customize_ip` | 삭제 후 재생성 가능 |

`ip_repo/src`를 직접 수정한 내용은 재패키징 때 사라진다. 기능 수정은 canonical RTL,
XGUI 수정은 canonical XGUI, IP-XACT 수정은 package 스크립트에 반영한다.

## 3. Package IP GUI

package와 편집 project를 새로 만들고 자동 검사만 수행하려면 다음을 실행한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v2/scripts/open_v2_ip_packager_gui.ps1 `
  -RefreshPackage -Recreate -ValidateOnly
```

실제 Package IP GUI를 열려면 `-ValidateOnly`를 제외한다.

```powershell
./system_integration/v2/scripts/open_v2_ip_packager_gui.ps1 -Recreate
```

GUI가 열리기 전에 package checker와 Packager project checker가 실행된다. 실패한
package를 GUI로 열어 수동으로 고치는 흐름은 허용하지 않는다.

## 4. 실제 Customize IP GUI

Block Design 사용자가 보는 실제 화면은 별도 실행기로 연다.

```powershell
./system_integration/v2/scripts/open_v2_customize_ip_gui.ps1 -Recreate
```

이 창에서 확인할 항목은 다음과 같다.

- CSR, Processing, TDC bus clock: 50/100/125/150/200 MHz
- Stream clock mode: ASYNC 또는 같은 물리 clock을 쓰는 SYNC
- TDC-GPX chip 수: 1~4
- STOP 수: chip당 1~8
- 합성 최대 Return 수: STOP당 1~7
- Rising/Falling capability mask와 한 chip의 양 edge 지원
- PACKED17 출력 폭: 32/64/128 bit
- Mirror face 수: 1~5
- Echo Receiver와 Echo simulation enablement
- OEN 배선 모드와 여섯 종류의 물리 시간 Generic

지원 목록과 숫자 범위는 XGUI Tcl이 아니라 `component.xml`의 IP-XACT 속성으로
관리된다. 교차 Generic 오류는 elaboration의 `fn_validate_build_config`가 최종
차단하고 테스트벤치가 회귀 검증한다.

## 5. File Group과 Interface 점검

Synthesis와 Simulation에는 같은 88개 self-contained RTL이 있어야 한다. 다음은
오류다.

- `../HDL` 또는 절대 경로
- `.xci` child IP
- 88개와 다른 source 수
- top 이외 RTL의 VHDL-2008 type 누락
- XGUI 파일이 한 개가 아니거나 이름이 다른 상태

주요 interface는 다음과 같다.

- `s_axi_csr`: 통합 32-control/32-status AXI4-Lite CSR
- `m_axis_rise`, `m_axis_fall`: PACKED17 거리 데이터
- `m_axis_monitor`: 모터·레이저 관측 stream
- `s_axi_csr_aclk`, `proc_aclk`, `i_tdc_clk`: 독립 clock 계약
- `io_tdc_d`, `o_tdc_csn`, `o_tdc_oen` 등: 외부 TDC-GPX 물리 bus

Echo Receiver 비활성 시 포트를 entity에서 제거하지 않는다. IP-XACT enablement가
Block Design 표시만 제어하고, RTL generate가 기능을 제거한다.

## 6. Windows 경로와 단축 드라이브

실행기는 Vivado 자동 생성 경로가 Windows 260자 제한을 넘지 않도록
`tdc_gpx_ctrl`을 `P:`~`U:` 중 빈 드라이브에 임시 연결한다. GUI가 열린 동안 해당
연결을 유지한다. 모든 관련 Vivado 창을 닫은 뒤 출력된 문자에 맞춰 해제한다.

```powershell
subst P: /D
```

검증 전용 실행이나 GUI 시작 실패에서는 실행기가 자신이 만든 연결을 해제한다.

## 7. GUI에서 XGUI를 수정한 뒤

1. Package IP `Customization GUI`에서 Layout을 수정한다.
2. `Review and Package`로 저장한다.
3. Vivado를 닫는다.
4. `import_v2_xgui_from_packager.ps1`을 실행한다.
5. `open_v2_ip_packager_gui.ps1 -RefreshPackage -Recreate -ValidateOnly`를 실행한다.
6. 실제 Package IP와 Customize IP GUI를 다시 확인한다.

`Customization Parameters`의 `Merge changes`는 원본 HDL 경로를 외부 참조로 다시
끌어올 수 있으므로 사용하지 않는다.

## 8. 오류 복구

| 증상 | 조치 |
|---|---|
| Layout/Preview가 비어 있음 | 네이티브 XGUI 검사 후 package와 편집 project 재생성 |
| `Customization Tcl ... older` | 저장·unload·reopen·생성 순서를 공식 실행기로 복구 |
| 기본 Page 하나만 표시 | 같은 core 객체에서 생성된 XGUI를 폐기하고 재패키징 |
| `../HDL` 등장 | 저장 중단, package 재생성, Merge changes 사용 금지 |
| IP locked | 관련 Parent와 Customize 창을 닫고 package 갱신 |
| Windows 경로 초과 | 공식 실행기의 단축 드라이브 경로 사용 |
