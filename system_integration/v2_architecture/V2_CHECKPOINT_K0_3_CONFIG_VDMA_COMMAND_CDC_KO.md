# V2 Checkpoint K0-3 - 원자적 설정, VDMA 프로파일, 시스템 명령 CDC

## 1. 목적

K0-3은 통합 Top에서 다음 두 경계를 닫는다.

1. 새 설정이 Rise/Fall VDMA에 모두 반영되기 전에 Active 설정으로 공개되지 않게 한다.
2. CSR의 `CLEAR_STATUS`와 `SOFT_RESET` 1-cycle 명령을 Processing/TDC 두 도메인에
   유실 없이 전달하고, 수신 불가 상태를 소프트웨어에 오류로 알린다.

Processing/Echo/GPX 데이터 경로 자체는 K0-4 이후에 연결한다. 따라서 K0-3은 전체
LiDAR 기능 Sign-off가 아니라 설정과 시스템 명령의 통합 경계 Sign-off이다.

## 2. 원자적 설정 활성화 순서

```text
CSR Shadow 쓰기
  -> COMMIT W1S
  -> 순차 검증 및 파생값 계산
  -> Processing/TDC PREPARE 수락
  -> Processing ACTIVATE 시작
  -> Rise/Fall VDMA 프로파일 계산
  -> Rise VDMA 설정 요청 ----> Rise ACK
  -> Fall VDMA 설정 요청 ----> Fall ACK
  -> 두 lane 프로파일 활성 완료
  -> Processing ACTIVATE ACK
  -> TDC ACTIVATE ACK
  -> Active version 공개 + commit-success IRQ
```

Rise와 Fall의 VDMA ACK는 서로 독립적으로 도착할 수 있다. 한쪽만 ACK한 상태에서는
커밋이 계속 Busy이며 새 Active version과 성공 IRQ가 공개되지 않는다. 두 lane이 모두
완료된 뒤에만 Processing 설정 게이트웨이가 Activate를 승인한다.

프로파일 트랜잭션은 활성 Chip/STOP/Return/Shot 수를 Activate 시작 시점에 한 번
래치한다. Runtime Return 수가 바뀌면 Active HSIZE는 바뀌지만 STRIDE는 합성 시 정한
최대 물리 용량 기준으로 유지되어 DDR 버퍼 주소 계약이 흔들리지 않는다.

## 3. 시스템 명령 CDC

`lidar_system_command_cdc`는 CSR 소스와 Processing/TDC 두 목적지 사이에 한 항목
four-phase level handshake를 둔다.

- Idle일 때 `CLEAR_STATUS` 또는 `SOFT_RESET` 하나만 수락한다.
- 두 명령이 같은 사이클에 함께 오거나 이전 명령이 Busy이면 명령을 거부한다.
- 두 목적지에서 각각 정확히 한 clock pulse를 만들고 두 ACK를 모두 기다린다.
- 명령 CDC가 Ready가 아니면 CSR 쓰기는 access error로 기록된다.
- 목적지 reset이 전송 중 발생하면 명령은 유실되지 않도록 재전달될 수 있다.

마지막 항목은 임의의 비멱등 동작에는 쓸 수 없는 계약이다. 현재 두 명령은 상태를
지우거나 reset 상태로 만드는 멱등 명령이므로 재전달되어도 최종 의미가 같다. 정상
운용에서 reset이 개입하지 않으면 목적지별 정확히 한 번 실행된다.

## 4. RTL 변경 경계

| 블록 | 역할 |
|---|---|
| `lidar_config_subsystem` | Processing Activate ACK를 외부 완료까지 지연할 수 있는 경계 제공 |
| `lidar_csr_bank` | 시스템 명령 Ready/Rejected를 AXI access error와 진단 이벤트에 연결 |
| `lidar_system_command_cdc` | CSR 명령을 Processing/TDC에 ACK 기반으로 전달 |
| `lidar_gpx_vdma_profile_transaction` | Rise/Fall 프로파일 요청과 두 ACK를 하나의 설정 트랜잭션으로 묶음 |
| `tdc_gpx_lidar_ctrl_v2_top` | 위 블록을 단일 소유권으로 조립하고 실제 AXI CSR/VDMA 설정 포트를 구동 |

Production compile order는 K0-2의 78개에서 위 두 신규 합성 블록을 포함한 80개로
증가했다.

## 5. 검증 결과

### 5.1 K0-3 통합 회귀

| Processing | TDC | 시스템 명령 CDC | Top 설정/VDMA 통합 |
|---:|---:|---|---|
| 150 MHz | 200 MHz | PASS | PASS |
| 200 MHz | 150 MHz | PASS | PASS |

검증 내용:

- 목적지별 `CLEAR_STATUS`/`SOFT_RESET` 정확히 한 pulse;
- Busy 명령과 동시 두 명령 거부;
- Rise만 ACK한 상태에서 Active version과 IRQ가 공개되지 않음;
- Fall까지 ACK한 뒤 정확한 Active version과 commit-success IRQ 공개;
- Return 7에서 1로 변경 시 HSIZE 감소와 최대 STRIDE 유지;
- 아직 연결하지 않은 물리 출력의 fail-safe 상태 유지.

결과:

`signoff_results/sessions/260807_k03_final_v2_k03_integration`

### 5.2 기존 경로 무회귀

| 회귀 | 결과 |
|---|---|
| Unified CSR 시뮬레이션 | 주소맵, bank, GPX activation, 두 clock profile PASS |
| Unified CSR 구현 150/200 | WNS `+0.644 ns`, latch 0, CDC Critical 0 |
| Unified CSR 구현 200/150 | WNS `+0.613 ns`, latch 0, CDC Critical 0 |
| Config manager 구현 150/200 | WNS `+1.086 ns`, latch 0, CDC Critical 0 |
| Config manager 구현 200/150 | WNS `+1.201 ns`, latch 0, CDC Critical 0 |
| VDMA profile/Footer | 150/200 MHz와 32/64/128-bit 전 조합 PASS |

결과 폴더:

- `signoff_results/sessions/260807_k03_csr_reg2_v2_unified_csr`
- `signoff_results/sessions/260807_k03_cfg_reg_v2_config_manager`
- `signoff_results/sessions/260807_k03_footer_reg_v2_gpx_face_footer`

CDC 보고서의 다중 비트 mailbox Warning은 기존 coherent bundled-data 게이트웨이에
대한 구조적 경고이며 Critical은 0이다. K0-9에서 전체 Top 기준으로 다시 감사한다.

## 6. Gate 판정과 다음 단계

K0-3의 설정/VDMA 원자성과 시스템 명령 CDC 경계는 **완료**이다. 아직 남은 것은
K0-4의 Processing/Echo 기능 연결과 K0-8의 통합 status/IRQ 단일 소유자이다.

다음 단계에서는 다음 순서로 연결한다.

1. Active Processing 설정과 operation 명령을 `lidar_processing_subsystem`에 연결;
2. 실제 `fire_pulse/start_tdc/stop_tdc` 소유권을 Top의 fail-safe 상수에서 Processing으로 이전;
3. Echo build enable에 따른 physical/simulation STOP 경로 연결;
4. simulation 모드와 physical laser가 동시에 활성화되지 않는지 exact 검증.
