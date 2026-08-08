# V2 Checkpoint K1-1 TDC CLEAR_STATUS 종결 보고서

## 1. 목적과 판정

K1-1은 통합 CSR의 `CTL0.CLEAR_STATUS`가 TDC 진단 표시만 일부 지우던 문제를
닫는다. 설정, 진행 중 요청, FIFO 또는 acquisition 상태를 훼손하지 않으면서
모든 TDC lane 진단 이력을 같은 규칙으로 초기화하는 것이 목표다.

판정은 **Complete**다. legacy 소유 모듈, v2 상태 집계, IRQ 경로, 두 routine
clock profile의 시뮬레이션과 배치배선을 모두 통과했다.

## 2. 닫은 문제

기존에는 `TDC_LANE_FAULT_n[2] response mismatch`, `[3] raw drop`, controller
성분의 `[4] drain cap`, `[10] init/config coalesced`, `[11] command collision`,
`[12] bus fatal`이 각 legacy 소유 process에서 `CLEAR_STATUS`를 소비하지 않았다.
그 결과 원인이 이미 사라졌어도 `IRQ_STATUS[8] GPX_TRANSPORT`가 계속 1일 수
있었다.

## 3. 최종 RTL 계약

1. `TDC_LANE_FAULT_n[0:12]`의 이력 field는 각 실제 소유 process가 지운다.
2. `CLEAR_STATUS`는 FSM, FIFO, pending 요청, 초기화 또는 active 설정을 취소하지
   않는다.
3. clear와 새 fault가 같은 소유 clock에 겹치면 새 fault가 우선한다.
4. bus-fatal 기능 격리 상태 `s_bus_fatal_active_r`와 software 진단 이력
   `s_err_bus_fatal_r`는 분리한다.
5. 살아 있는 bus-fatal 기능 상태는 안정된 bus-idle 복구 또는 reset/reinit에서만
   해제된다. `CLEAR_STATUS`로 격리를 우회할 수 없다.
6. 원인 source가 0이 된 뒤에도 `IRQ_FLAG[8]`은 보존되며 software가 W1C한다.

## 4. 소유 모듈

| 소유 모듈 | 초기화 대상 | 보존 대상 |
|---|---|---|
| `tdc_gpx_chip_init` | init/config coalesced 이력 | 진행 중 init와 pending 설정 |
| `tdc_gpx_chip_reg` | Register request overflow 이력 | Register FSM와 queued 요청 |
| `tdc_gpx_chip_ctrl` | response mismatch, raw drop, drain cap, command collision, bus-fatal 이력 | acquisition FSM, raw FIFO, bus-fatal 기능 격리 |
| `lidar_tdc_status_source` | v2 domain의 pulse-history와 집계 이력 | 살아 있는 owner level과 TDC live 상태 |

## 5. 검증 구조

`run_v2_gpx_clear_status.ps1`은 다음 네 검증을 순차 실행한다.

1. 초기 설정 소유자 set/clear/new-event-wins;
2. Register 요청 소유자 set/clear/new-event-wins;
3. 이름 있는 controller fault event를 개별 강제 주입;
4. 기존 `tb_tdc_gpx_chip_ctrl` 22개 전체 기능 시나리오.

통합 상태와 IRQ는 `run_v2_k08_status_irq.ps1`로 처리 150/TDC 200 MHz와 처리
200/TDC 150 MHz를 모두 확인한다. `CLEAR_STATUS`가 두 native domain에 도달하고,
원인 source가 내려가도 pending IRQ가 W1C 전까지 보존되는 순서를 검증한다.
TDC pulse fault와 clear를 같은 TDC clock에 겹쳐 새 사건이 source/IRQ_FLAG에
보존되고, 두 번째 clear와 W1C로만 종료되는 우선순위도 직접 검증한다.

## 6. 결과

| 검증 | 결과 |
|---|---|
| K1-1 owner/fault 전체 회귀 | `LIDAR_V2_K11_GPX_CLEAR_STATUS_REGRESSION_PASS` |
| K08 처리 150/TDC 200 MHz | `LIDAR_V2_K08_STATUS_IRQ_PASS` |
| K08 처리 200/TDC 150 MHz | `LIDAR_V2_K08_STATUS_IRQ_PASS` |
| TDC lane 150 MHz 구현 | WNS `+1.532 ns`, latch 0 |
| TDC lane 200 MHz 구현 | WNS `+0.541 ns`, latch 0 |
| 통합 CSR 처리 150/TDC 200 MHz 구현 | WNS `+0.395 ns`, latch 0, CDC Critical 0 |
| 통합 CSR 처리 200/TDC 150 MHz 구현 | WNS `+0.608 ns`, latch 0, CDC Critical 0 |
| IP package source/XGUI/OOC matrix | `LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS` |

## 7. 유지보수 금지사항

- software clear를 기능 reset이나 abort로 확대하지 않는다.
- bus-fatal sticky 하나를 다시 기능 격리 상태로 겸용하지 않는다.
- clear를 새 사건보다 우선시켜 같은 clock의 fault를 잃지 않는다.
- 통합 K08만 실행하고 owner-level fault injection을 생략하지 않는다.
- `IRQ_STATUS=0` 확인 전에 `IRQ_FLAG`만 W1C하여 원인을 숨기지 않는다.

다음 단계 K1-2는 GPX Reg7 MTimer의 Shadow 입력, COMMIT 자동 대체, Active image,
실제 Chip readback을 하나의 연속 시나리오로 비교한다.
