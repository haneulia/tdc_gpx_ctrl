# LiDAR Unified CSR Top 사용 및 검증 가이드

문서 기준일: 2026-07-28

대상 RTL: `system_integration/rtl/lidar_unified_csr_top.vhd`

## 1. 역할과 경계

`lidar_unified_csr_top`은 통합 모드에서 유일한 AXI4-Lite register owner다.
내부에 source-level `my_axil_csr32_top` 한 개만 두며 다음 기능만 수행한다.

1. 32 CTL / 32 STAT / 4 IRQ register 주소 decode
2. CTL0..25를 각 IP의 unified adapter 입력으로 고정 배선
3. IP status와 static TDC geometry를 STAT0..31에 고정 배선
4. config/reset epoch match, busy, reject, valid를 STAT31에 집계
5. Motor/Laser/Echo/TDC interrupt identity를 32-bit source에 고정 배선

Motor 위치, Laser fire/fire_done, TDC START/STOP, Echo STOP, GPX 28-bit bus,
VDMA AXIS는 이 모듈을 통과하지 않는다. 이 경계 덕분에 CSR 통합이 실시간 지연이나
200 MHz TDC 처리 경로를 늘리지 않는다.

## 2. 연결 원칙

- 모든 child IP는 `g_ENABLE_LOCAL_CSR=false`로 합성한다.
- `s_axi_csr_aclk`와 `s_axi_csr_aresetn`은 각 child unified configuration
  clock/reset에도 동일하게 연결한다.
- `o_sys_ctrl`은 Motor, Laser, Echo, TDC에 fan-out한다.
- `o_sys_cfg_apply`는 shared config epoch을 사용하는 Motor, Laser, TDC에
  fan-out한다. Echo profile은 CTL15/16의 독립 toggle/ack protocol을 사용한다.
- 각 IP의 status/transaction/IRQ output은 이 top의 동일 이름 input으로만 연결한다.
- 각 IP의 기존 local `s_axi`와 local IRQ는 Block Design에서 숨겨져야 한다.

Clock fan-out은 parent에서 직접 연결한다. 이 RTL은 AXI clock을 새 clock output으로
재생성하거나 buffer하지 않는다.

## 3. Reset 이후 초기화

통합 CSR의 CTL0..31 reset 값은 모두 0이다. Child adapter의 generic 초기 active 값과
software-visible staging 값이 우연히 같은 것으로 가정하지 않는다. 따라서 Laser는
CTL0 `LASER_EN=0` 상태로 유지하고 다음 full-profile 절차를 반드시 수행한다.

```text
1. CTL2..CTL25 중 사용되는 모든 staging/config word 작성
2. Echo indexed delay가 필요하면 CTL15/16 write/apply ack 완료
3. CTL1.CFG_EPOCH 변경
4. STAT31[18] ALL_CFG_ACCEPTED = 1 확인
5. 필요 시 CTL0.RESET_EPOCH 변경
6. STAT31[19] ALL_RESET_ACCEPTED = 1 확인
7. STAT31[21:20] ANY_REJECT/ANY_BUSY = 0 확인
8. 마지막에 CTL0.LASER_EN 설정
```

Epoch는 8-bit modulo 값이며 0으로 wrap될 수 있다. 값이 0인지가 아니라 adapter의
match와 valid bit를 기준으로 완료를 판정한다.

## 4. System status

| STAT | 내용 |
|---:|---|
| 0 | `0x4C010000`: LiDAR signature, ABI 1.0, RTL revision 0 |
| 1 | IP/transaction capability와 active CTL/STAT/IRQ source 수 |
| 2 | requested cfg/reset epoch, Laser/TDC accepted cfg epoch |
| 3..5 | static TDC max rows, cell size, max HSIZE |
| 6..11 | Motor |
| 12..18 | Laser |
| 19..22 | Echo |
| 23..29 | TDC runtime/read result |
| 30 | CTL20 index가 선택한 active GPX image word readback |
| 31 | adapter transaction 요약 |

STAT31 bit 순서:

| Bits | 내용 |
|---:|---|
| 2:0 | Motor/Laser/TDC config match + valid |
| 6:3 | Motor/Laser/Echo/TDC reset match |
| 10:7 | Motor/Laser/Echo/TDC busy |
| 14:11 | Motor/Laser/Echo/TDC reject |
| 17:15 | Motor/Laser/TDC valid |
| 18 | shared config 전체 승인 |
| 19 | reset 전체 승인 |
| 20 | 하나 이상의 busy |
| 21 | 하나 이상의 reject |

Echo busy/reject는 STAT21의 profile apply pending과 command reject에서 나온다. TDC
busy/reject는 config, serialized command, indexed image 경로를 모두 포함한다.

## 5. Interrupt

| Global bits | Source |
|---:|---|
| 0..3 | system reserved, 현재 0 |
| 4..7 | Motor |
| 8..10 | Laser |
| 11..15 | reserved, 0 |
| 16..17 | Echo |
| 18..20 | reserved, 0 |
| 21..27 | TDC-GPX |
| 28..31 | reserved, 0 |

`my_axil_csr32_top`은 source를 동기화하고 rising event를 `INTR_FLAG`에 저장한다.
운용 기본은 manual mode이며 ISR이 `INTR_FLAG`를 W1C할 때까지 pending이 유지된다.
IP-XACT interrupt sensitivity는 기존 CSR32 호환성을 따라 `EDGE_RISING`이다. PS GIC와
parent interrupt fabric도 이 계약에 맞춰야 한다.

## 6. 검증 결과

| Gate | 결과 |
|---|---|
| Static address/capability contract | `UNIFIED_CSR_CONTRACT_PASS` |
| AXI/control/status/epoch/IRQ TB | `[SUMMARY] Passed=7 Failed=0` |
| xc7z020clg484-2 synthesis | 0 errors, 0 critical warnings |
| Black boxes | 0 |
| CSR32 owners | 1 |
| Legacy/local CSR owners | 0 |
| Post-synth utilization | 1,021 LUT, 2,250 FF |

합성 warning은 `my_axil_csr32`가 AXI `AWPROT/ARPROT` 값을 기능에 사용하지 않아
해당 register/load가 제거됐다는 내용뿐이다.

검증 명령:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_unified_csr_contract.ps1

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_unified_csr_top.ps1

vivado.bat -mode batch `
  -source system_integration/scripts/test_unified_csr_top_synth.tcl `
  -tclargs C:/tmp/unified_csr_top_synth
```

## 7. Closure 범위

이 단계는 통합 **제어 평면**의 sign-off다. 다음 단계에서 실제 네 IP를 unified mode로
연결한 뒤 150/200 MHz 신호처리 회귀를 수행해야 한다. 그때 local/unified mode의
shot 수, START/STOP, Return-7, 28-bit I-Mode, 17-bit Hit, VDMA geometry, IRQ와
HTML contract가 동일해야 전체 시스템 sign-off가 된다.
