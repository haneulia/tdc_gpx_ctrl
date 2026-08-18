# TDC-GPX LiDAR Controller V3 이중 HLS 패키징 가이드

## 1. 목적

V3 통합 제어기는 동일한 기능, 포트, Generic, CSR 및 PACKED17 데이터 계약을
유지하면서 H1~H4 HLS 구현을 포함하는 방법만 다른 두 IP로 배포한다.

| 구분 | 생성 RTL 포함형 | HLS 하위 IP 참조형 |
|---|---|---|
| Vivado VLNV | `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0` | `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3_hls_ip:3.0` |
| IP 폴더 | `ip_repo/tdc_gpx_lidar_ctrl_v3_3_0` | `ip_repo/tdc_gpx_lidar_ctrl_v3_hls_ip_3_0` |
| H1~H4 포함 방식 | Vitis HLS 생성 RTL을 통합 IP 안에 직접 포함 | 독립 HLS IP 4개를 `.xci` 하위 IP로 참조 |
| 배포 특성 | 단일 폴더로 닫힌 self-contained IP | 통합 IP와 HLS 하위 IP 4개가 모두 필요 |
| 권장 용도 | Parent 프로젝트, 보드 배포, 재현 가능한 Sign-off | HLS 모듈별 GUI 점검, 교체, 의존성 분석 |

두 IP를 한 데이터 경로에 동시에 연결하는 구조가 아니다. 같은 기능의 패키징
대안이므로 Parent 설계에서는 둘 중 하나만 선택한다.

## 2. 공통 기능 계약

두 버전은 다음 항목이 같아야 한다.

- 통합 Top 포트와 `G_OUTPUT_WIDTH=32/64`를 포함한 합성 Generic
- AXI4-Lite CSR, Shadow/Active, COMMIT 및 IRQ ABI
- 외부 TDC-GPX I-Mode Raw28 읽기와 Hit17 해석
- Return 1~7 Cell 수집, Rise/Fall 정렬 및 PACKED17 출력
- XGUI 탭, 계산값, 유효성 검사와 Optional port enablement
- Processing clock과 TDC-GPX 버스 clock 사이의 CDC 계약

패키징 방법이 기능 결과를 바꾸면 안 된다. 이 때문에 32/64-bit 네 조합을 각각
합성하고, 최종 Top의 black box 수가 0인지 확인한다.

## 3. HLS 하위 IP 구성

HLS 하위 IP 참조형은 다음 네 IP를 사용한다.

| 단계 | VLNV | 역할 |
|---|---|---|
| H1 | `victek.co.kr:hls_ip:gpx_hit_decoder_hls:3.0` | TDC-GPX Raw28을 Hit17과 제어 사건으로 해석 |
| H2 | `victek.co.kr:hls_ip:gpx_cell_collector_hls:3.0` | Stop 채널별 Return 1~7 Cell 수집 및 전시 Return 필터 |
| H3 | `victek.co.kr:hls_ip:gpx_frame_assembler_hls:3.0` | Rise/Fall Cell 순서 정렬과 Shot/Face 경계 조립 |
| H4 | `victek.co.kr:hls_ip:gpx_lane_word_formatter_hls:3.0` | Shot Metadata, PACKED17 Cell, Hole 및 Face Footer Word 생성 |

통합 IP의 Adapter는 HLS 함수의 직접 생성 RTL 이름 대신 위 하위 IP의 XCI 모듈
이름을 인스턴스화한다. 나머지 RTL과 외부 인터페이스는 생성 RTL 포함형과 같다.

## 4. 패키지 생성

### 4.1 생성 RTL 포함형

검증된 HLS 생성 RTL을 재사용할 때:

```powershell
./system_integration/v3/scripts/run_v3_ip_package.ps1 -SkipHlsSynthesis
```

HLS C++ 또는 공통 Header가 변경되었을 때는 `-SkipHlsSynthesis`를 제거하여 H1~H4
C synthesis를 먼저 수행한다.

### 4.2 HLS 하위 IP 참조형

형상관리된 HLS 하위 IP 4개를 재사용하는 평상시 경로:

```powershell
./system_integration/v3/scripts/run_v3_hls_ip_package.ps1 `
  -SkipHlsSynthesis
```

이 경로는 Vitis의 HLS IP export를 호출하지 않으므로 `xcd.exe`를 시작하지 않는다.
HLS C++ 또는 공통 Header를 실제로 변경하여 하위 IP도 갱신해야 할 때만 다음 옵션을
명시한다.

```powershell
./system_integration/v3/scripts/run_v3_hls_ip_package.ps1 `
  -RefreshHlsChildIp
```

`-RefreshHlsChildIp`는 H1~H4 C synthesis와 Vitis IP export를 수행하는 의도적인
유지보수 작업이다. 실행 전 Vivado와 Vitis GUI를 모두 닫는다.

## 5. IP Packager GUI

생성 RTL 포함형:

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant EmbeddedRtl -Recreate
```

HLS 하위 IP 참조형:

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant HlsIp -Recreate
```

GUI를 열지 않고 package와 편집 프로젝트를 검사하려면 `-ValidateOnly`를 추가한다.
각 프로젝트 위치는 다음과 같다.

- 생성 RTL 포함형: `.work/v3_ip_packager/tdc_gpx_lidar_ctrl_v3_ip_packager.xpr`
- HLS 하위 IP 참조형: `.work/v3_hls_ip_packager/tdc_gpx_lidar_ctrl_v3_hls_ip_packager.xpr`

## 6. 32/64-bit 합성 검증

```powershell
./system_integration/v3/scripts/run_v3_dual_package_ooc.ps1 -Selector ALL
```

개별 선택자는 `EMBEDDED32`, `EMBEDDED64`, `HLS32`, `HLS64`이다. 이 검사는
Vivado run manager와 별도 `xcd.exe` 합성 작업을 사용하지 않고, 한 Vivado 프로세스
안에서 `synth_design -mode out_of_context`를 실행한다.

PASS 조건은 다음과 같다.

1. 두 VLNV와 HLS 하위 IP 4개가 IP Catalog에 동시에 한 개씩 존재한다.
2. 생성 RTL 포함형에는 child XCI가 없다.
3. HLS 하위 IP 참조형에는 정확히 네 subcore가 있고 생성 HLS RTL이 중복되지 않는다.
4. 32/64-bit 네 Top 모두 합성되고 black box가 0이다.

## 7. `xcd.exe` 오류 대응

Windows의 `xcd.exe` 메모리 참조 오류는 RTL 기능 오류가 아니라 Vivado/Vitis의
별도 실행 관리자 프로세스가 종료되는 과정에서 발생한 도구 오류다. 다음 원칙으로
재발 범위를 제한한다.

- 평상시 패키징은 기존 HLS 하위 IP를 재사용한다.
- OOC 검증은 `run_v3_dual_package_ooc.ps1`만 사용한다.
- Vivado 사용자 Tcl Store가 손상되면 Vivado의 공식 `reset_tclstore` 절차로
  초기화한다.
- `-RefreshHlsChildIp`는 HLS 원본 변경 시에만 사용한다.
- 오류 뒤 남은 Vivado, Vitis 또는 `xcd.exe` 프로세스가 없고 IP package 검증과
  네 조합 OOC 합성이 PASS하면 생성물은 기능 검증 대상으로 사용할 수 있다.

## 8. 선택 기준

보드 Parent와 배포 기준은 **생성 RTL 포함형**을 권장한다. 한 IP 폴더만으로 소스
closure가 닫혀 프로젝트 이동과 장기 재현이 단순하기 때문이다.

HLS를 자주 수정하거나 H1~H4를 Vitis/Vivado GUI에서 각각 교체·분석할 때는
**HLS 하위 IP 참조형**을 사용한다. 기능 변경을 완료한 뒤에는 두 패키지와 네 OOC
조합을 모두 갱신해 동일성 검증을 다시 수행한다.
