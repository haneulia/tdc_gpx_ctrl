# V3 Zynq PS 제어와 VDMA 적용 가이드

## 1. 목적과 버전 관계

V3는 V2의 검증된 AXI4-Lite CSR Register Map을 바꾸지 않는다. 하드웨어 세대는
V3 HLS 데이터 처리 경로로 올라갔지만 PS 제어 ABI는 계속 `2.7`이다. 따라서
`lidar_v2_ps_control.h/.c`를 공통 CSR 접근 계층으로 재사용하고, V3에서 강화된
VDMA 트랜잭션과 보드 연결만 이 디렉터리가 소유한다.

```text
V2 공통 CSR ABI 2.7
    lidar_v2_ps_control.h/.c
              |
              v
V3 VDMA 트랜잭션 조정
    lidar_v3_vdma_transaction.h/.c
              |
              v
Xilinx XAxiVdma 연결
    xilinx_standalone/lidar_v3_xilinx_vdma_adapter.h/.c
```

V3 출력 폭은 합성 시 `32 bit` 또는 `64 bit`로 정한다. `128 bit`는 V3 계약에
포함되지 않으며, PS가 읽은 `STAT1`의 출력 폭과 VDMA `S_AXIS_S2MM` 폭은 반드시
같아야 한다. VDMA의 DDR 쪽 AXI 폭이나 Zynq HP Port 폭은 별도 계약이다.

## 2. 파일 역할

| 파일 | 역할 |
|---|---|
| `lidar_v3_control.h/.c` | COMMIT 시작 전 처리 idle/VDMA 사전 정리, VDMA 실패 뒤에도 RTL 종료 상태 회수 |
| `lidar_v3_vdma_transaction.h/.c` | Rise/Fall 독립 적용·ACK, 재시도, 프로파일 교체 경쟁 차단 |
| `xilinx_standalone/lidar_v3_xilinx_vdma_adapter.h/.c` | V3 조정 계층을 검증된 V2 `XAxiVdma` Lane 구현에 연결 |
| `xilinx_standalone/lidar_v3_xilinx_frame_service.h/.c` | CTL23/24 처리 idle 확인, 완료 Frame의 cache invalidate, PS 처리 callback, 소유권 반환 |
| `tests/test_lidar_v3_vdma_transaction.c` | 부분 실패, ACK 지연, 프로파일 교체, DDR 영역 검증 |
| `tests/test_lidar_v3_control.c` | VDMA 실패 후 RTL error/recovery 상태를 끝까지 회수하는지 검증 |
| `tests/test_lidar_v3_xilinx_vdma_adapter.c` | 정지 직전 완료 Frame, cache 범위, CPU 소유·미처리 Frame, overwrite, VDMA error 차단 검증 |

## 3. 안전한 Runtime 설정 순서

Runtime Return 수, 레이저 목표 왕복시간
`(2R/c, TARGET_RANGE_WINDOW_5NS)`, 인접한 레이저 발사 후보점 사이의 요청
광학각 또는 Runtime TDC-GPX 버스 읽기 타이밍을 변경할 때는 다음 순서를 지킨다.

```text
lidar_v2_stop_and_disarm()
    -> CTL0.STOP W1S
    -> STAT3에서 RUNNING=0, ARMED=0, COMMAND_READY=1 확인
    -> CTL23/24 진단 0x10의 새 sequence 응답 확인
       pipeline_idle=1, echo_idle=1
       GPX processing idle=1, GPX AXIS idle=1
       GPX CDC reset busy=0
    -> Rise/Fall VDMA 정지
    -> 모든 완료 Frame을 PS 큐로 인계하고 DMA 소유로 반환
    -> CTL1~20 Runtime Shadow 기록
    -> lidar_v3_commit_shadow()
       -> RTL이 CTL25~29에 Lane별 Pending geometry 게시
       -> 해당 Lane 실제 VDMA config/address/start 성공
       -> 성공한 Lane만 CTL25 ACK W1S
       -> 모든 필요한 Lane ACK 뒤 Active Version 전환
    -> COMMIT 결과와 Active Version 확인
    -> RUN + ARM
```

`lidar_v2_stop_and_disarm()`은 `STOP`과 `DISARM` 명령을 두 번 쓰는 함수가 아니다.
`CTL0.STOP` 하나가 모터/위치 처리와 레이저 발사를 정지하고 ARM도 함께 해제한다.

진단 `0x10`은 CTL24의 과거 값을 그대로 믿지 않는다. CTL23에 CAPTURE를 기록하기
전 sequence와 완료 뒤 sequence가 달라야 하고, `BUSY=0`, `VALID=1`, `ERROR=0`도
동시에 만족해야 한다. 이 확인 전에 VDMA를 정지하면 남아 있던 Footer/AXIS Beat가
`TREADY=0`에 막혀 RTL 설정 PREPARE가 끝나지 않을 수 있다.

`CTL25` ACK는 “프로파일을 읽었다”는 표시가 아니다. 해당 Lane의 실제 VDMA가
새 `HSIZE`, `VSIZE`, `STRIDE`, Frame 주소로 정상 시작됐다는 소프트웨어 승인이다.

## 4. 부분 실패 정책

Rise와 Fall은 독립 트랜잭션이다. Rise 적용이 성공하고 Fall이 일시적으로
적용되지 못하면 다음처럼 처리한다.

1. Rise만 즉시 ACK한다.
2. ACK 후 CDC 해제 지연 동안 Rise Pending이 잠시 남아도 Rise를 다시 설정하지 않는다.
3. Fall은 CPU 소유 또는 미처리 Frame이 모두 반환된 뒤 다시 시도한다.
4. Fall 성공 뒤 Fall만 ACK한다.
5. VDMA driver 오류, overwrite 또는 error sticky는 재시도하지 않고 치명 오류로 처리한다.

VDMA 적용 도중 `CTL25~29` 스냅샷이 달라지면 이전 스냅샷을 ACK하지 않는다. 새
프로파일을 다시 적용하고 일치 여부를 확인한 뒤에만 ACK한다.

VDMA를 정지하기 직전에 한 Frame이 완료될 수 있다. Adapter는 완료 IRQ를 지우기만
하지 않고 `ready_mask`에 복원한 뒤 이번 적용을 `RETRY`로 보류한다. 다음 COMMIT
Poll의 progress service가 이 Frame을 처리하고 반환한 뒤에만 새 Geometry를
설정한다. 옛 Geometry로 VDMA를 다시 시작하지 않으므로 Frame Store 0을 뜻하지
않게 덮어쓰는 경로도 없다.

완료 IRQ가 발생한 Face Frame만 보존 대상이다. 운용자가 활성 Face 중간에 `STOP`을
발행하면 Footer까지 끝나지 않은 현재 VDMA Frame은 완성 데이터가 아니므로 Viewer로
내보내지 않고 다음 VDMA 설정에서 폐기한다. 유효 Face를 버리지 않으려면 Face 완료
IRQ 직후의 비활성 미러 구간에서 `STOP`을 발행해야 하며, 이 경계는 H6-B3B 보드에서
실측한다.

## 5. DDR Frame과 캐시 소유권

각 Frame Store는 합성 최대 Return 수의 `STRIDE × VSIZE`보다 작지 않게 예약한다.
Rise와 Fall 및 각 Frame Store의 물리 DDR 영역은 서로 겹치면 안 된다.
각 Lane은 S2MM 전용 AXI VDMA와 최소 3개 Frame Store를 사용한다.
`lidar_v3_xilinx_vdma_adapter_init()`는 AMD 드라이버의 완료 임계값을 1 Face
Frame으로 설정한다. 이 호출이 실패하면 callback과 RUN을 활성화하지 않는다.
VDMA program의 `EnableFrameCounter=0`은 이 완료 임계값을 끄는 설정이 아니다.
S2MM이 Face 하나 뒤 멈추지 않고 3개 Frame Store를 계속 순환하도록 하는 설정이며,
AMD 드라이버는 별도로 설정한 1-Face 완료 임계값 필드를 보존한다.

```text
DMA-owned
    -> VDMA S2MM 완료 IRQ
    -> Xil_DCacheInvalidateRange(STRIDE × VSIZE)
    -> CPU-owned
    -> Face Footer, Shot Metadata, Cell Metadata, PACKED17 검증
    -> PS 소유 전송 큐로 Face Header와 H-Line payload 복사
    -> CPU 쓰기가 있었다면 cache flush
    -> DMA-owned로 반환
    -> 별도 Ethernet task가 PS 소유 큐를 송신
```

CPU-owned 또는 아직 처리하지 않은 ready Frame이 하나라도 있으면 Geometry 변경을
재시도 상태로 보류한다. 이 상태를 지우고 VDMA를 재설정하면 미처리 Frame의 소유권과
데이터를 잃으므로 금지한다. `overwrite_mask` 또는 `error_mask`가 있으면 이미 데이터
손실 또는 DMA 오류가 발생한 것이므로 자동 재시작하지 않고 운용 로그와 복구 절차로
넘긴다.

`lidar_v3_xilinx_vdma_take_completed()`와
`lidar_v3_xilinx_vdma_release_frame()`은 VDMA가 정상 실행 중일 때뿐 아니라 새
Geometry 적용을 위해 안전 정지된 Lane에서도 동작한다. Frame 처리 callback이
실패하면 Frame은 CPU-owned로 남는다. 실패 원인을 해결하고 같은 Frame을 다시
처리하기 전에는 VDMA Profile 적용과 ACK를 진행하면 안 된다.

## 6. 사용 예

```c
lidar_v3_xilinx_vdma_adapter_t adapter;
lidar_v3_xilinx_frame_service_t frame_service;
lidar_v3_vdma_transaction_t transaction;
lidar_v2_commit_result_t result;

lidar_v3_xilinx_vdma_adapter_init(
    &adapter, &rise_lane, &fall_lane, 1000000U);
lidar_v3_vdma_transaction_init(
    &transaction, &device,
    lidar_v3_xilinx_vdma_apply_lane, &adapter, 1000000U);
lidar_v3_xilinx_frame_service_init(
    &frame_service, &device, &adapter,
    process_one_completed_frame, &application);
lidar_v3_xilinx_frame_service_bind_transaction(
    &frame_service, &transaction);
lidar_v3_vdma_transaction_set_poll_wait(
    &transaction, ps_scheduler_yield, &application);

status = lidar_v2_stop_and_disarm(
    &device, 5000000U, NULL, NULL);
if (status != LIDAR_V2_CONTROL_OK) {
    /* STOP 실패: Shadow와 VDMA Geometry를 바꾸지 않는다. */
}

status = lidar_v3_commit_shadow(
    &device, &transaction, 5000000U, &result);
```

`lidar_v3_xilinx_vdma_adapter_init()`는 `XAxiVdma_CfgInitialize()`와 Frame Store
주소/용량을 Lane에 기록한 뒤, 첫 S2MM 시작 전에 호출한다. AXI VDMA를 reset한 뒤에는
동일 초기화를 다시 수행해야 한다. reset은 callback 등록과 1-Frame 완료 임계값을
운용 코드가 다시 확립해야 하는 경계다.

`process_one_completed_frame()`은 Face Footer와 Active Version을 확인하고 PACKED17을
PS가 소유하는 H-Line/Viewer 전송 큐로 복사한 뒤에만 `XST_SUCCESS`를 반환해야 한다.
물리 Ethernet 전송 완료까지 callback 안에서 기다릴 필요는 없다. 큐 공간이 없으면
`XST_NO_DATA`를 반환해 DDR Frame 소유권을 유지한다. 실패를 성공으로 숨기면 아직
필요한 DDR Frame을 새 VDMA 설정이 덮어쓸 수 있다.

FreeRTOS에서는 `ps_scheduler_yield()`에 `taskYIELD()` 또는 시스템 주기에 맞춘
`vTaskDelay()`를 연결한다. 이 callback이 없으면 사전 정리 Poll이 빠르게 반복되어
같은 Cortex-A9 코어의 Ethernet task가 전송 큐를 비울 기회를 잃을 수 있다.
Bare-metal은 callback을 생략할 수 있지만 watchdog service와 제한된 대기 함수를
등록하면 timeout을 실제 시간으로 해석하기 쉽다.

`retry_limit`와 `poll_limit`는 Poll 횟수이지 물리 시간이 아니다. 사전 Frame 인계는
CSR COMMIT을 쓰기 전에 수행되므로 `G_PHASE_TIMEOUT_US`를 소비하지 않는다. 반면
COMMIT 이후의 VDMA 적용과 Lane별 ACK는 반드시 `G_PHASE_TIMEOUT_US` 안에 끝나야
한다. 두 구간의 실제 시간을 PS task 주기와 함께 보드에서 측정해 한계를 정한다.

사전 정리에서 `LIDAR_V3_VDMA_PRECOMMIT_TIMEOUT`이 발생하면 CSR COMMIT은 쓰이지
않았지만 Rise/Fall VDMA 중 일부 또는 전부가 이미 정지됐을 수 있다. 이 상태에서
`RUN/ARM`을 임의 재개하지 않는다. 전송 큐 backpressure, CTL23 진단 BUSY 또는
미처리 CPU-owned Frame의 원인을 먼저 해소하고 같은 Shadow/COMMIT 절차를 다시
실행한다. `overwrite_mask`나 `error_mask`에 의한 치명 실패는 자동 재시도하지 않고
VDMA와 LiDAR 운용 상태를 함께 초기화하는 명시적 복구 절차로 넘긴다.

## 7. 자동 검증

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b3a_ps_board_preflight.ps1
```

실제 생성된 Zynq-7000 BSP까지 컴파일하려면 다음처럼 실행한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b3a_ps_board_preflight.ps1 `
  -BspInclude <platform>/export/<platform>/sw/<domain>/include `
  -CheckVivadoVdma
```

이 시험의 PASS는 PS 제어 코드, Cortex-A9 명령 호환성과 Xilinx 헤더 컴파일 및 실패
정책을 증명한다. 실제 VDMA Register 효과, 물리 DDR/cache 일관성, Ethernet MAC/PHY,
bitstream와 TDC-GPX·Encoder·Laser·Echo LVDS 물리 I/O는 보드 H6-B3B에서 닫는다.
