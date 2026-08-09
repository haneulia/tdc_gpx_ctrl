# V2 Checkpoint K0-10 IP Package Sign-off 결과

## 1. 판정

Checkpoint K0-10은 **L1 IP Package Sign-off 완료**로 판정한다.

- 신규 VLNV `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0` 생성;
- 기존 v1 `victek.co.kr:my_ip:tdc_gpx_top:1.0`과 같은 catalog에서 병존;
- 합성에 실제 필요한 RTL 87개와 XGUI, 한글 Product Guide를 package 내부에 복사;
- package와 원본의 SHA-256 byte 비교 PASS;
- IP-XACT 무결성, interface clock/reset/IRQ 계약과 XGUI 의존성 검사 PASS;
- 대표 Customize IP 설정 3개 PASS;
- 동일 3개 설정을 package 내부 RTL만으로 OOC 합성하여 black box/latch 0 PASS;
- 실제 IP Integrator Block Design과 생성 VHDL wrapper를 동일 3개 설정으로
  합성하고 GUI 재개방 검증 PASS;
- public Top shell의 routine clock profile `150/200 MHz`, `200/150 MHz` 재회귀 PASS.

이 판정은 **재사용 가능한 v2 IP 산출물과 XGUI**에 대한 것이다. 실제 AXI VDMA,
HP port, DDR, PS cache API, Ethernet MAC/PHY 및 PCB 핀을 포함한 시스템/보드
Sign-off는 Stage L0에 남아 있다.

## 2. K0-9와 K0-10의 검증 경계

| 검증 항목 | Sign-off 단계 | 상태 |
|---|---|---|
| RTL DDR 캡처 대 HTML Golden Vector의 할당 Word exact compare | K0-9/L1 | PASS |
| PS cache 소유권 모델 이후 H-Line/Ethernet payload byte exact compare | K0-9/L1 | PASS |
| 최종 RTL을 IP-XACT로 package하고 원본과 byte 동기화 | K0-10/L1 | PASS |
| XGUI 설정, 포트 enablement, clock/interface metadata | K0-10/L1 | PASS |
| package 내부 source만 사용한 OOC 합성 | K0-10/L1 | PASS |
| 실제 IPI BD 생성 wrapper와 IP별 OOC 합성 | K0-10/L1 | PASS |
| 실제 VDMA가 HP port를 통해 기록한 DDR 대 Golden 비교 | L0 parent/board | 미검증 |
| FreeRTOS/PetaLinux cache API 이후 실제 Ethernet 수신 비교 | L0 software/board | 미검증 |

따라서 질문한 두 비교는 Sign-off 항목으로 사용할 수 있으며, 이미 K0-9의
실행 가능한 L1 Golden Sign-off로 닫혀 있다. K0-10은 그 최종 RTL이 package
과정에서 바뀌거나 누락되지 않았음을 별도로 보증한다.

## 3. 패키지 산출물

패키지 위치:

`system_integration/v2/ip_repo/tdc_gpx_lidar_ctrl_v2_2_0`

| 구성 | 계약 |
|---|---|
| Top | `tdc_gpx_lidar_ctrl_v2_top` |
| VLNV | `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0` |
| RTL | production dependency 87개, package `src/` 안에 자체 포함 |
| XGUI | `xgui/tdc_gpx_lidar_ctrl_v2_v2_0.tcl` |
| 설명서 | `doc/PRODUCT_GUIDE_KO.md` |
| 외부 source 경로 | 없음 |
| XCI/subcore 의존성 | 없음 |

K0-9의 compile manifest는 검증 보조 package와 당시 비교 대상을 포함한 89개
항목이었다. K0-10 package manifest는 test-only `px_utility_pkg.vhd`와 실제
Top에서 사용하지 않는 legacy `lidar_gpx_vdma_lane_formatter.vhd`를 제외한
**합성 의존 RTL 87개**로 고정했다. 기능 소스를 줄인 것이 아니라 package의
production dependency를 정확하게 좁힌 것이다.

## 4. XGUI 계약

XGUI는 다음 설정을 합성 전 값으로 제한한다.

| 항목 | 허용 값/범위 |
|---|---|
| Processing/TDC clock | `50/100/125/150/200 MHz` |
| CDC mode | `SYNC` 또는 `ASYNC`; SYNC는 두 주파수가 같아야 함 |
| AXIS 출력 폭 | `32/64/128 bit` |
| TDC-GPX Chip | `1..4` |
| STOP/Chip | `1..8` |
| 최대 Return | `1..7` |
| Mirror Face | `1..5` |
| Echo Receiver | 합성 시 enable/disable |

Slope topology 검사는 Rise 수가 Fall 수보다 작지 않은지, mask가 실제 Chip 수를
넘지 않는지 확인한다. dedicated 2-Rise/2-Fall, all-dual, rise-only 및 one-Chip
dual-edge 구성을 허용한다. Echo Receiver를 끄면 LVDS 입력과 GPX STOP 출력은
Block Design에서 숨겨지고 simulation Echo option도 false로 고정된다.

## 5. 검증 행렬

### 5.1 Package 정적 계약

| 항목 | 결과 |
|---|---|
| 원본 대 package source 동기화: 87 RTL + XGUI + 한글 가이드 3개 | PASS |
| VLNV, core revision, generic 범위/기본값 | PASS |
| AXI4-Lite/AXIS clock association과 `FREQ_HZ` dependency | PASS |
| active-low reset, level-high IRQ metadata | PASS |
| Echo port enablement와 XGUI validation | PASS |
| v1/v2 catalog 동시 검색 | PASS |
| 외부 경로, XCI, subcore 부재 | PASS |

### 5.2 Customize IP 및 packaged OOC

| Profile | Processing/TDC | CDC | 폭 | Echo | Customize | OOC 합성 |
|---|---|---|---:|---|---|---|
| `async32_tdc_faster` | 150/200 MHz | ASYNC | 32 | on | PASS | PASS |
| `async128_proc_faster` | 200/150 MHz | ASYNC | 128 | off | PASS | PASS |
| `sync64_equal` | 150/150 MHz | SYNC | 64 | on | PASS | PASS |

세 OOC profile 모두 black box 0, latch 0이며 public port 폭 계약도 일치했다.
또 package source/manifest 변경 후 Top shell을 `150/200 MHz`, `200/150 MHz`로
재검증하여 `LIDAR_V2_TOP_SHELL_REGRESSION_PASS`를 확인했다.

### 5.3 실제 IP Integrator 및 GUI 프로젝트

package-only OOC와 별도로 실제 IPI가 생성한 VHDL wrapper를 합성했다. 이 Gate에서
다음 두 package 결함을 발견하고 수정했다.

1. public generic 폭이 package 상수 `C_MAX_CHIPS`를 참조해, 생성 wrapper에는
   package context가 없어 compile이 실패했다. public 폭을 `3 downto 0`으로
   고정하고 component 검사에서 `C_MAX_CHIPS` 노출을 금지했다.
2. XGUI의 `std_logic_vector` mask가 wrapper generic map에 `0011`로 기록됐다.
   model-parameter callback이 VHDL bit string을 항상 `"0011"` 형태로 전달하도록
   수정하고 정적 회귀를 추가했다.

수정 후 세 BD validation과 합성 Run이 모두 완료됐고, 재개방 검증에서도 설정값,
Echo port enablement, black box 0 및 latch 0을 다시 확인했다. GUI 프로젝트는
다음 위치에서 직접 열 수 있다.

`C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/.work/tdc_gpx_lidar_ctrl_v2_gui/tdc_gpx_lidar_ctrl_v2_gui.xpr`

## 6. 실행 및 증거

재현 명령:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k010_ip_package.ps1
```

최종 marker:

`LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS`

최종 세션:

`signoff_results/sessions/260809_185237_k010_ip_package`

GUI 재개방 검증 세션:

`signoff_results/sessions/260807_211239_k010_gui_verify`

Top shell 재회귀 세션:

`signoff_results/sessions/260807_k010_package_top_recheck3_v2_top_shell`

## 7. 경고 계약

최종 세션은 `Critical Warning 0 / Error 0`이며, 일반 Warning은 다음 ID만
허용한다. 실행기는 새 Warning ID가 생기거나 아래 최대 발생 수가 증가하면
`LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS`를 내지 않는다. 실제 관측값은 세션의
`WARNING_AUDIT.txt`에 저장한다.

| 단계 | 메시지 ID | 최대 수 | 판정 근거 |
|---|---|---:|---|
| 초기 package 추론 | `IP_Flow 19-11770` | 3 | Generic 의존 `FREQ_HZ`를 설정하기 전의 clock 추론 경고 |
| 초기 package 추론 | `IP_Flow 19-3158` | 3 | AXIS와 Processing clock을 연계하기 전의 추론 경고 |
| 초기 package 추론 | `IP_Flow 19-5661` | 3 | clock의 `ASSOCIATED_BUSIF`를 설정하기 전의 추론 경고 |
| 초기 package 추론 | `IP_Flow 19-2187` | 1 | 한글 Product Guide file group을 추가하기 전의 추론 경고 |
| 초기 package 추론 | `IP_Flow 19-11888` | 1 | 최종 IP 설명을 설정하기 전의 기본 설명 경고 |
| OOC 3-profile 합계 | `Synth 8-7129` | 300 | 폭/프로파일별로 사용하지 않는 record 필드 또는 port 제거 |
| OOC 3-profile 합계 | `Synth 8-6014` | 287 | 사용하지 않는 순차요소 제거 |
| OOC 3-profile 합계 | `Synth 8-3917` | 123 | Generic profile에서 상수로 확정된 공개 출력 |
| OOC 3-profile 합계 | `Synth 8-3332` | 6 | 도달할 수 없는 one-hot FSM state 제거 |
| OOC 3-profile 합계 | `Constraints 18-5572` | 15 | 외부 pad가 없는 OOC에서 내부 IOB 속성 적용 불가 |

초기 package 경고 뒤에는 `ipx::check_integrity`, clock/reset association,
Product Guide 및 component 정적 검사를 다시 수행한다. OOC IOB 경고는 실제
`IOBUF`, Package Pin과 I/O Standard를 가진 L0 Parent 구현에서 재검증했고
Critical CDC, blocking DRC 및 unconstrained service pin이 모두 0임을 확인했다.

## 8. 다음 Gate

K1-4 통합 RTL/IP Sign-off와 L0 Parent 합성·배치·배선·bitstream은 완료됐다.
다음 단계는 **Stage L1 물리 보드 Runtime 증거**다. 실제 TDC-GPX 40 MHz Tref와
Register readback, VDMA/HP/DDR byte 무결성, FreeRTOS/PetaLinux DMA cache 관리,
레이저 안전 인터록 및 지속 Ethernet 처리량을 확인해야 전체 시스템 Release
Sign-off를 선언할 수 있다.
