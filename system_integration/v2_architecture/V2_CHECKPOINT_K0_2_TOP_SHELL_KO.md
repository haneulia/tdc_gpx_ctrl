# V2 Checkpoint K0-2 - 통합 Top 셸과 정식 컴파일 순서

## 1. 목적

K0-2는 기존 v2 하위 블록을 하나의 합성 가능한 public entity 아래에 놓을 수
있는지 확인하는 구조 체크포인트다. 이 단계는 기능 통합 Sign-off가 아니라 다음
단계에서 배선을 추가할 기준점이다.

## 2. 산출물

- 통합 Top 셸:
  `system_integration/v2/rtl/top/tdc_gpx_lidar_ctrl_v2_top.vhd`
- production 컴파일 순서:
  `system_integration/v2/scripts/v2_rtl_compile_order.txt`
- 독립 회귀 스크립트:
  `system_integration/v2/scripts/run_v2_top_shell.ps1`

Top 셸은 다음 public 경계를 먼저 고정한다.

- CSR, Processing, TDC의 세 clock/reset domain
- AXI4-Lite CSR slave
- Encoder, fire-done, laser permit
- Echo LVDS/STOP
- 칩당 28-bit 외부 TDC-GPX bus와 상태/제어 핀
- Processing monitor AXIS
- 독립 Rise/Fall DDR AXIS
- 독립 Rise/Fall VDMA profile request/acknowledgement

## 3. 현재 안전 상태

K0-2 architecture는 아직 하위 subsystem을 연결하지 않는다. 따라서 합성 또는
시뮬레이션 중 물리 장치를 오동작시키지 않도록 다음 값을 강제한다.

- `fire_pulse`, `start_tdc`, `shot_start`, `stop_tdc`: 비활성
- 모든 AXIS `TVALID`: 비활성
- TDC-GPX `CSN/RDN/WRN/OEN`: 비활성
- TDC-GPX `STOPDIS`: 활성
- TDC-GPX data bus: High-Z

이 상태는 release 동작이 아니다. `K0-3`부터 기능을 단계적으로 연결하며 각
경계를 독립 회귀로 닫는다.

## 4. 검증 결과

Vivado Simulator 2025.2.1에서 production 소스 78개를 VHDL-2008로 한 번에
컴파일한 뒤 아래 두 profile을 각각 elaboration했다.

| Processing clock | TDC clock | 판정 |
|---:|---:|---|
| 150 MHz | 200 MHz | PASS |
| 200 MHz | 150 MHz | PASS |

두 경우 모두 `LIDAR_V2_TOP_SHELL_PASS` marker와 fail-safe 초기 출력을
확인했다. 결과 세션은 다음 위치에 보관한다.

`signoff_results/sessions/260807_k02_top_shell_fix_v2_top_shell`

## 5. 발견 및 수정

최초 회귀는 RTL 오류가 아니라 Windows Vivado batch loader가
`--generic_top NAME=VALUE`를 `NAME VALUE`로 분리해서 실패했다. 회귀
스크립트가 해당 인수를 `cmd.exe` 경계까지 하나의 quoted token으로 유지하도록
수정했다. 같은 스크립트에서 임의 Processing/TDC generic 조합을 신뢰성 있게
전개할 수 있다.

## 6. Gate 판정

- production compile order: **닫힘**
- public top entity와 scalar generic/port contract: **닫힘**
- 두 routine clock profile elaboration: **닫힘**
- 기능 배선, CSR transaction, 데이터 경로, IRQ: **아직 열림**

다음 단계는 `K0-3`이다. Processing VDMA profile 승인을 configuration
Activate transaction에 포함하고, CSR의 `CLEAR_STATUS`와 `SOFT_RESET`
명령을 acknowledged CDC로 전달해야 한다.
