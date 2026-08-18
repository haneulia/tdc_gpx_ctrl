# TDC-GPX LiDAR Controller V3 IP Packager 유지보수 가이드

## 1. 목적

이 문서는 다음 두 V3 통합 IP를 Vivado GUI의 **Package IP** 화면에서 점검하고
유지보수하는 절차를 고정한다. 두 IP의 기능 계약은 같고 H1~H4 HLS 구현을 포함하는
방법만 다르다.

- 생성 RTL 포함형: `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0`
- HLS 하위 IP 참조형: `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3_hls_ip:3.0`

GUI 프로젝트는 형상관리 원본이 아니라, 재현 가능한 일회성 편집·검토 환경이다.
버전 선택 기준과 전체 검증 절차는
`V3_DUAL_HLS_PACKAGING_GUIDE_KO.md`를 함께 따른다.

관리 대상은 다음과 같다.

- Identification, Compatibility 및 VLNV
- Synthesis/Simulation/Product Guide/XGUI File Groups
- 합성 Generic과 Customization Parameters
- AXI4-Lite, AXI4-Stream, clock/reset 및 물리 TDC-GPX Ports and Interfaces
- XGUI 탭, 계산값, 유효성 검사와 포트 표시 조건
- Review and Package의 IP 무결성 DRC

## 2. 원본과 생성물의 책임

| 구분 | 형상관리 원본 | GUI에서 직접 수정 |
|---|---|---|
| RTL 및 생성 RTL 포함형 목록 | `ip_package/v3_ip_package_manifest.tcl`과 그 원본 파일 | 금지 |
| HLS C++ 원본 | `hls/*/src`, `hls/common/include` | Vitis HLS에서 수정 |
| XGUI 구현 | `ip_package/tdc_gpx_lidar_ctrl_v3_xgui.tcl` | 실험만 허용 |
| IP-XACT 속성·Generic·Interface | `scripts/package_v3_ip.tcl` | 실험만 허용 |
| 생성 RTL 포함형 package | `ip_repo/tdc_gpx_lidar_ctrl_v3_3_0` | 스크립트로 재생성 |
| HLS 하위 IP 참조형 package | `ip_repo/tdc_gpx_lidar_ctrl_v3_hls_ip_3_0` | 스크립트로 재생성 |
| HLS 하위 IP 4개 | `ip_repo/gpx_*_hls_3_0` | Vitis export로 명시적 갱신 |
| 생성 RTL 포함형 GUI 프로젝트 | `.work/v3_ip_packager` | 자유롭게 재생성 |
| HLS 하위 IP 참조형 GUI 프로젝트 | `.work/v3_hls_ip_packager` | 자유롭게 재생성 |

`ip_repo/src`나 `ip_repo/xgui`를 직접 고친 내용은 다음 재패키징 때 지워진다.
유지할 변경은 반드시 위 표의 canonical 원본에 먼저 반영한 뒤 패키지를 다시 만든다.

## 3. GUI 열기

V3는 목적이 다른 두 GUI를 분리한다.

- **Package IP GUI**: File Groups, Generic, Ports and Interfaces, IP-XACT DRC를
  편집·점검한다.
- **Customize IP GUI**: Block Design 사용자가 실제로 보는 XGUI 탭, 선택 상자,
  입력 제약과 Optional port 표시를 점검한다.

canonical XGUI는 `ip_package/tdc_gpx_lidar_ctrl_v3_xgui.tcl`에서 코드로 관리한다.
따라서 Package IP의 `Customization GUI` 레이아웃 편집 화면은 이 Tcl의 동적
callback과 반복문을 완전하게 역변환하지 못해 빈 화면으로 보일 수 있다. 이것을
최종 XGUI 부재로 판정하지 않는다. 대신 아래 실제 Customize IP 창이 정상적으로
열리고 모든 탭이 표시되는 것을 필수 PASS 조건으로 사용한다.

### 3.1 Package IP GUI

생성 RTL 포함형을 최초 생성하거나 package가 변경된 뒤에는 다음 명령을 사용한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant EmbeddedRtl -Recreate
```

HLS 하위 IP 참조형은 다음과 같다.

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant HlsIp -Recreate
```

변경 없이 다시 열 때는 `-Recreate`를 생략한다. GUI를 열지 않고 package와 편집
프로젝트만 처음부터 검증하려면 선택한 명령에 `-ValidateOnly`를 추가한다.

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant HlsIp -Recreate -ValidateOnly
```

실행기는 GUI를 열기 전에 다음 항목을 자동 검사한다.

1. canonical RTL/HLS/XGUI/문서와 `ip_repo` 사본의 일치
2. 선택한 VLNV와 두 버전의 IP Catalog 공존
3. 생성 RTL 포함형의 self-contained source closure와 child `.xci` 부재
4. HLS 하위 IP 참조형의 정확한 subcore 4개와 생성 HLS RTL 중복 부재
5. `../HDL` 외부 참조 부재
6. 필수 Generic, 포트, AXI/clock interface 및 XGUI 계약
7. 32/64-bit customization과 IP Packager `check_integrity` DRC PASS

### 3.2 실제 Customize IP GUI

생성 RTL 포함형의 실제 사용자 화면은 다음 명령으로 연다.

```powershell
./system_integration/v3/scripts/open_v3_customize_ip_gui.ps1 `
  -Variant EmbeddedRtl -Recreate
```

HLS 하위 IP 참조형은 다음과 같다.

```powershell
./system_integration/v3/scripts/open_v3_customize_ip_gui.ps1 `
  -Variant HlsIp -Recreate
```

이 창에서 `Clock and Output`, `TDC-GPX Topology`, `Echo Frontend`,
`Physical Timing` 탭과 32/64-bit 선택, 1~4 Chip, Rising/Falling capability,
Echo Receiver enablement 및 지원 clock 목록을 확인한다. `OK`를 누르면 미리보기
XCI가 `.work/v3_customize_*` 아래에 생성되며, `Cancel`은 설정을 저장하지 않는다.

## 4. 원본 변경 후 재패키징

RTL, HLS 생성 RTL, XGUI 또는 IP-XACT 속성을 바꾼 뒤에는 다음 순서를 따른다.

1. 해당 단위·통합 테스트를 실행한다.
2. package를 사용하는 Vivado Parent와 IP Packager 창을 모두 닫는다.
3. package와 편집 프로젝트를 함께 갱신한다.

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant EmbeddedRtl -RefreshPackage -Recreate
```

검증된 HLS RTL을 이미 생성했고 HLS 합성을 반복할 필요가 없을 때만 다음 옵션을 쓴다.

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant EmbeddedRtl -RefreshPackage -SkipHlsSynthesis -Recreate
```

`-SkipHlsSynthesis`는 기존 `.work/v3_hls_*_component/hls/syn/verilog` 산출물을
사용한다. HLS C++ 또는 공통 Header가 변경된 경우에는 이 옵션을 사용하지 않는다.

HLS 하위 IP 참조형의 평상시 갱신은 형상관리된 하위 IP를 재사용한다.

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant HlsIp -RefreshPackage -SkipHlsSynthesis -Recreate
```

HLS C++ 또는 공통 Header가 변경되어 네 HLS 하위 IP를 실제로 다시 export해야 할
때만 `-RefreshHlsChildIp`를 추가한다. 이 옵션은 Vitis 도구 경로를 사용하므로 실행
전에 Vivado/Vitis GUI를 모두 닫는다.

## 5. GUI 항목별 점검 방법

### 5.1 Identification과 Compatibility

- VLNV와 Display Name을 확인한다.
- `core_revision`은 호환되는 package 재생성 때 증가시킨다.
- RTL/포트/CSR ABI가 호환되지 않으면 revision만 올리지 말고 IP version 정책을
  별도로 결정한다.
- 대상은 `xc7z020clg484-2`, 지원 family는 Zynq Production이다.

### 5.2 File Groups

- 생성 RTL 포함형의 Synthesis와 Simulation은 동일한 self-contained 소스 집합을
  사용한다.
- 파일명은 `src/...` 또는 `xgui/...`처럼 package 내부 상대 경로여야 한다.
- 두 버전 모두 `../HDL/...`와 절대 경로가 없어야 한다.
- 생성 RTL 포함형에는 child `.xci`가 없어야 하며 HLS 생성 Verilog는
  `src/hls_generated/...`에 포함된다.
- HLS 하위 IP 참조형에는 정확히 네 XCI가 Synthesis/Simulation에 각각 등록되고,
  `src/hls_generated/...`는 없어야 한다.

Customization Parameters의 **Merge changes**는 원본 HDL 프로젝트 경로를
`../HDL` 형태로 다시 끌어올 수 있다. 이 package에서는 Merge로 파일 목록을
재구성하지 않는다. Generic 변경은 top RTL과 `package_v3_ip.tcl`, XGUI 변경은
canonical XGUI Tcl에서 수행한다.

### 5.3 Customization Parameters와 XGUI

- 합성 시 고정되는 Generic만 노출한다.
- 계산 전용 값은 XGUI의 read-only 표시값으로만 둔다.
- Runtime 설정은 통합 AXI4-Lite CSR에서 관리하며 Generic으로 중복 노출하지 않는다.
- XGUI 유효성 검사와 HDL Generic 범위는 같은 계약이어야 한다.
- clock 주파수 표시는 실제 Block Design의 `FREQ_HZ`와 일치해야 한다.

### 5.4 Ports and Interfaces

- `s_axi_csr`: 통합 CSR AXI4-Lite
- `m_axis_rise`, `m_axis_fall`: PACKED17 거리 데이터 AXI4-Stream
- `m_axis_monitor`, `m_fire_count`, `m_stop_evt`: 상태·계측 스트림
- `s_axi_csr_aclk`, `proc_aclk`, `i_tdc_clk`: 서로 다른 clock 계약
- `io_tdc_d`, `o_tdc_csn` 등: 외부 TDC-GPX 물리 bus

포트 폭은 `G_NUM_CHIPS`, `G_STOPS_PER_CHIP`, `G_OUTPUT_WIDTH`와 일치해야 한다.
Optional 포트는 HDL entity에서 삭제하는 방식이 아니라 IP-XACT enablement로만
Block Design 표시 여부를 제어한다.

### 5.5 Review and Package

저장 전에 모든 error를 해소하고 `IP Integrity`가 PASS인지 확인한다. 현재 허용된
경고는 설치된 Board Store의 다른 device용 board 파일 경고와, 물리 GPX clock인
`i_tdc_clk`에 연결 AXI bus가 없다는 알림이다. 다음 항목은 허용하지 않는다.

`[IP_Flow 19-5661] Bus Interface 'i_tdc_clk' does not have any bus interfaces
associated with it`는 의도된 독립 clock-domain 경고다. `i_tdc_clk`는 내부 GPX
획득 상태기계와 `WRN/RDN/CSN/ADR/DATA/OEN` 같은 discrete 물리 핀을 구동하지만,
top-level AXI/AXIS bus를 직접 clock하지 않는다. `FREQ_HZ`와
`i_tdc_aresetn` 연관성을 유지하기 위해 clock interface 자체는 보존한다. 향후
물리 GPX 핀을 정식 custom bus interface로 묶을 때만 해당 bus를 연결해 경고를
제거한다.

- `[Common 17-680]` Windows 260자 경로 초과
- external source reference 유입
- 생성 RTL 포함형의 child XCI 또는 HLS 하위 IP 참조형의 잘못된 subcore 수
- VLNV, XGUI, Generic, interface 불일치
- `check_integrity` 실패

## 6. Windows 단축 드라이브

Vivado가 긴 자동 생성 이름을 붙여도 260자 제한을 넘지 않도록 실행기는 HDL root를
`P:`~`U:` 중 빈 드라이브에 임시 연결한다. GUI가 열려 있는 동안 이 연결을 유지해야
한다. GUI를 닫은 뒤 실행기가 출력한 드라이브를 다음처럼 해제할 수 있다.

```powershell
subst P: /D
```

실제 할당 문자가 `P:`가 아니면 출력된 문자를 사용한다. 검증 전용 실행 또는 GUI
시작 실패 때 실행기가 새로 만든 연결은 자동으로 해제한다.

## 7. 변경 보존과 Git

GUI에서 시험한 변경을 유지하려면 다음 위치로 옮긴다.

- XGUI 탭·표시·validation: `ip_package/tdc_gpx_lidar_ctrl_v3_xgui.tcl`
- IP 속성·Generic·port/interface: `scripts/package_v3_ip.tcl`
- source closure/order/type: `ip_package/v3_ip_package_manifest.tcl`
- 기능 RTL/HLS: 각 canonical source

그 뒤 해당 `-Variant`로 `-RefreshPackage -Recreate -ValidateOnly`를 실행하고,
canonical 원본과 `ip_repo` 변경만 Git에 포함한다. `.work/v3_ip_packager`,
`.work/v3_hls_ip_packager`, Vivado log/journal과 가상 드라이브는 형상관리 대상이
아니다.

## 8. 오류 복구

| 증상 | 조치 |
|---|---|
| IP가 locked 상태 | 해당 Parent를 닫고 package 갱신 후 개발 Parent를 재생성 |
| Simulation target reset 실패 | 관련 Vivado 창을 모두 닫고 disposable project 재생성 |
| XGUI가 이전 값 표시 | canonical XGUI 갱신, package 재생성, IP catalog refresh |
| Package IP의 Customization GUI가 비어 있음 | 동적 Tcl의 역변환 화면으로 판정하지 말고 `open_v3_customize_ip_gui.ps1`로 실제 사용자 XGUI 확인 |
| 실제 Customize IP 창이 비어 있거나 오류 | XGUI의 `init_gui`, Generic 이름, callback 인자를 수정하고 package 재생성 |
| File Group에 `../HDL` 등장 | 저장 중단, package 재생성, Merge changes 사용 금지 |
| HLS 하위 IP가 locked | `ip_repo` 전체를 등록하고 HLS-IP GUI 프로젝트를 `-Recreate` |
| `xcd.exe` 메모리 오류 | run manager를 쓰지 말고 dual OOC 검사 사용, 평상시 HLS IP export 생략 |
| IP Packager project 손상 | 해당 Variant의 `.work/v3_*_ip_packager`를 `-Recreate` |

배포 package 자체가 의심되면 다음 명령으로 HLS부터 package까지 완전 재생성한다.

```powershell
./system_integration/v3/scripts/run_v3_ip_package.ps1
```

두 패키지의 32/64-bit 합성은 별도 `xcd.exe` run manager를 사용하지 않는 다음
검사로 닫는다.

```powershell
./system_integration/v3/scripts/run_v3_dual_package_ooc.ps1 -Selector ALL
```
