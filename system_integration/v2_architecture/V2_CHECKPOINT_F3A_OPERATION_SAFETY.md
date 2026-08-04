# v2 Checkpoint F3a: Operation and Safety Owner

## 1. 판정

Stage 3 / Checkpoint F의 F3a operation/safety 경계를 완료했다.
RUN/STOP/ARM/DISARM은 통합 CSR의 W1S event이고, Processing domain의
`lidar_operation_manager`만 RUN/ARM 상태를 소유한다. 외부 laser permit은
software가 우회할 수 없는 live safety 입력으로 분리했다.

**결론:** F3a는 통과했다. 다음 허용 단계는 F3b `shot_scheduler`뿐이다.
Scheduler는 `ACTIVE_VALID`나 source mode를 보고 허가를 재구성하지 않고,
F3a의 `scheduler_enable`만 소비해야 한다.

## 2. v1 비교와 변경 이유

v1 `laser_ctrl`은 `CTL0[0].laser_en` persistent level을 executor admission에
직접 사용했다. 이 방식은 configuration validity, 운용 상태와 laser safety
permission의 소유권이 분리되지 않고, 통합 CSR에서 RUN/ARM 순서와 외부 permit
상실 후 재승인 정책을 표현할 수 없다.

v2는 다음을 의도적으로 복제하지 않았다.

| v1 구조 | v2 결정 |
|---|---|
| writable `laser_en` level | W1S RUN/STOP/ARM/DISARM event |
| CSR clock의 1-cycle command pulse | ACK가 돌아올 때까지 보존되는 CDC mailbox |
| config valid로 사실상 동작 허가 | operation manager의 명시적 RUN+ARM |
| permit 입력 없음 | raw final gate와 2-FF qualified permit |

## 3. 상태 전이 계약

| 조건/명령 | RUN | ARM | 출력 결과 |
|---|---:|---:|---|
| reset 또는 active config 상실 | 0 | 0 | 모든 operation/fire enable 0 |
| RUN 승인 | 1 | 기존 0 | Processing만 활성 가능 |
| ARM 승인, physical | 1 | 1 | permit이 있을 때 scheduler/physical fire 허가 |
| ARM 승인, simulation | 1 | 1 | scheduler/simulation 허가, physical fire 0 |
| DISARM | 유지 | 0 | scheduler/fire 차단 |
| STOP | 0 | 0 | latent ARM 없이 안전 정지 |
| physical permit 상실 | 유지 | 0 | raw gate 즉시 차단, 새 ARM 필요 |
| atomic PREPARE | 기억값 유지 | 기억값 유지 | config gate가 출력만 차단; idle 후 ACK |

RUN은 Processing active 설정이 valid/released일 때만 승인한다. ARM은 RUN이
먼저 승인되어야 하며 physical mode에서는 qualified permit도 필요하다.
Simulation mode에서는 permit 없이 ARM할 수 있지만 `physical_fire_enable`은
항상 0이다.

Permit assertion은 2-FF 동기화 후에만 허용한다. Permit deassertion은 raw 입력을
짧은 AND gate에 직접 포함하므로 LOW/X/Z에서 Processing clock을 기다리지 않고
physical enable을 닫는다. Processing clock이 상실을 관측하면 ARM도 지워져
permit 복귀만으로 발사가 재개되지 않는다.

## 4. Command CDC

`lidar_operation_command_cdc`는 한 개 command를 보유하는 toggle/ACK mailbox다.

1. CSR domain이 command code를 등록하고 request toggle을 바꾼다.
2. Processing domain은 request와 3-bit payload를 각각 2단 동기화한다.
3. request 검출 후 payload settling clock을 한 번 더 기다려 event를 출력한다.
4. ACK가 CSR domain으로 돌아오기 전에는 `COMMAND_READY=0`이다.
5. busy 중 새 command는 overwrite하지 않고 reject/access error로 진단한다.
6. Processing reset은 in-flight RUN/ARM을 flush한다. reset 후 software가
   `COMMAND_READY=1`을 확인하고 다시 명령해야 한다.
7. CSR reset은 command authority를 제거한다. 이 상태를 Processing domain에
   동기화하여 request-toggle의 현재 값과 관계없이 RUN/ARM을 모두 해제한다.

3-bit command payload는 mailbox 보유 규칙으로 안정하며 CDC-6 warning은 이
bundled-data 구조에 해당한다. 8-bit operation status는 Processing에서 먼저
등록한 뒤 각 bit를 2단 동기화하는 진단용 live readback이다. 관련 bit가 전환되는
짧은 구간에는 CSR에서 서로 다른 cycle의 값이 섞여 보일 수 있으므로 software는
상태 전이 직후 STAT3을 두 번 연속 같은 값으로 읽은 뒤 판단한다. 제어 판단은 이
readback을 되먹임하지 않으며 두 crossing 모두 Critical CDC는 0이다.

## 5. 통합 CSR ABI 2.2

CTL0 command bit가 다음처럼 확장되었다.

| Bit | Command |
|---:|---|
| 0 | COMMIT |
| 1 | CLEAR_STATUS |
| 2 | SOFT_RESET_REQUEST |
| 3 | RUN |
| 4 | STOP |
| 5 | ARM |
| 6 | DISARM |

STAT3 `[15:0]`은 ACTIVE_VERSION을 유지하고 `[25:16]`에 RUNNING, ARMED,
permit/config/Processing/scheduler/physical/simulation enable과 command
READY/BUSY를 배치했다. 주소 형상은 32 CTL / 32 STAT / 4 IRQ 그대로이며 ABI
minor만 1에서 2로 증가했다. 기본 STAT0은 `0x3E250202`이다.

## 6. 기능 검증

`tb_lidar_operation_subsystem`을 Processing 50/150/200 MHz에서 실행했다.
50 MHz는 100 MHz CSR보다 느린 조건에서도 1-cycle CSR pulse를 놓치지 않는지
확인하는 추가 CDC 기능점이다.

| ID | 시나리오 | 결과 |
|---|---|---|
| P40 | reset, unknown permit, active config 없음 | PASS |
| P41 | physical RUN/ARM, permit qualification/loss, explicit re-ARM | PASS |
| P42 | commit gate, pipeline safe point, STOP clears latent ARM | PASS |
| P43 | simulation ARM과 physical fire 상호 배제 | PASS |
| P44 | mailbox busy overwrite reject, 첫 payload 보존 | PASS |
| P45 | destination reset 시 in-flight command flush | PASS |
| P46 | CSR source reset 시 command authority 회수와 STOP/DISARM | PASS |

최종 marker는 모든 profile에서 다음과 같다.

```text
LIDAR_V2_OPERATION_PASS proc_mhz=<50|150|200>
```

## 7. 구현 및 통합 회귀

독립 F3a session:
`signoff_results/sessions/260804203000_v2_operation`

| Processing clock | WNS | Latch | ASYNC_REG | Critical CDC |
|---:|---:|---:|---:|---:|
| 150 MHz | `+4.349 ns` | 0 | 32 | 0 |
| 200 MHz | `+2.638 ns` | 0 | 32 | 0 |

200 MHz 기준 자원은 37 LUT, 60 FF, BRAM 0, DSP 0이다. source-online
동기화 2 FF를 포함해 Critical CDC는 0이다.

Unified CSR/atomic manager 재검증 session:
`signoff_results/sessions/260804204000_v2_unified_csr`

| Processing / TDC | WNS | Latch | ASYNC_REG | Critical CDC |
|---:|---:|---:|---:|---:|
| 150 / 200 MHz | `+0.641 ns` | 0 | 124 | 0 |
| 200 / 150 MHz | `+0.529 ns` | 0 | 124 | 0 |

CSR map/bank, 첫 atomic commit, invalid commit 보존, 양 domain version/payload,
CTL0 operation command decode와 STAT3 readback이 모두 PASS했다. 기존 atomic
configuration mailbox의 CDC-15 2106건은 이전과 동일하며 새 Critical CDC는 없다.
인식된 정상 동기 경로는 source-online 경로 추가로 17개에서 18개가 되었다.

공용 event package 무회귀 session:

| 경계 | Session | 150 MHz WNS | 200 MHz WNS | Latch | Critical CDC |
|---|---|---:|---:|---:|---:|
| F1 motor position | `260804201000_v2_motor_position` | `+1.763 ns` | `+0.458 ns` | 0 | 0 |
| F2 face tracker | `260804202000_v2_face_tracker` | `+2.736 ns` | `+1.126 ns` | 0 | 0 |

## 8. 재현 명령

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_operation.ps1

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_unified_csr.ps1
```

## 9. 다음 단계 제약

F3b scheduler는 다음 입력만 사용한다.

- B1 `face_event_t`;
- active `shot_interval_states`와 Face/column geometry;
- F3a `scheduler_enable`;
- F4에서 제공할 executor ready/busy.

Due point에서 busy이면 늦은 위치로 미루지 않고 skip/overrun으로 기록한다.
F3b가 RUN, ARM, permit 또는 source mode를 별도로 저장하거나 추론하면 F3a의
단일 소유권을 깨므로 금지한다.
