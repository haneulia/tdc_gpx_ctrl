# V3 H6-B3A PS 보드 투입 전 체크포인트

## 1. 판정

H6-B3A의 보드 없이 가능한 PS 운용 준비 범위는 PASS다. V3가 V2와 같은 CSR ABI
2.7을 사용한다는 사실을 자동 확인했고, 32/64-bit VDMA Geometry를 실제
Zynq-7000 Cortex-A9 BSP 헤더로 교차 컴파일했다.

이 판정은 물리 보드 Sign-off가 아니다. H6-B3B는 실제 bitstream, VDMA Register,
DDR S2MM, cache API, Ethernet MAC/PHY와 물리 I/O를 사용한 결과가 있어야 완료된다.

## 2. 이번 단계에서 보완한 문제

### 2.1 Rise 성공 뒤 Fall 실패

기존 참조 서비스는 Rise VDMA를 성공적으로 재설정한 뒤 Fall에서 실패하면 ACK를
하나도 쓰지 않고 종료했다. 다음 Poll에서 Rise를 다시 정지하고 재설정할 수 있었다.

V3 조정 계층은 Lane별 성공 직후 개별 ACK한다. Fall 재시도는 Rise를 다시 건드리지
않는다.

### 2.2 ACK와 CDC 해제 지연

PS가 ACK를 쓴 직후에도 `CTL25.PENDING`은 CDC 왕복 지연 동안 잠시 high일 수 있다.
V3는 적용 완료한 정확한 스냅샷을 기억해 이 구간에서 VDMA를 중복 설정하지 않는다.

### 2.3 CPU 소유 Frame과 Geometry 변경

기존 VDMA program 함수는 시작할 때 ready/cpu-owned mask를 초기화한다. 따라서
상위 소프트웨어가 미처리 Frame이 있는 상태로 호출하면 소유권 기록을 잃을 수 있다.
V3 Xilinx adapter는 다음 상태에서 program 호출 자체를 막는다.

- `cpu_owned_mask != 0`: CPU가 Frame을 처리 중
- `ready_mask != 0`: 완료됐지만 CPU가 아직 가져가지 않은 Frame 존재
- `overwrite_mask != 0`: Frame Store 재사용으로 데이터 손실 발생
- `error_mask != 0`: VDMA error IRQ 발생

앞의 두 상태는 제한된 재시도, 뒤의 두 상태는 치명 오류다.

### 2.4 VDMA 실패 뒤 RTL 상태 회수

VDMA callback 오류만 보고 즉시 반환하면 RTL 설정 트랜잭션이 Busy인 채 남을 수
있다. `lidar_v3_commit_shadow()`는 VDMA 작업 재시도는 중단하되, 검증된 V2 CSR
Poller를 계속 실행해 RTL의 error code와 recovery-required 상태까지 회수한다.

### 2.5 VDMA 정지 직전 완료 Frame

소프트웨어가 `ready_mask=0`을 확인한 직후, VDMA 정지 직전에 마지막 Frame 완료 IRQ가
발생할 수 있다. IRQ를 지우고 바로 Geometry를 바꾸면 그 Frame이 손실된다. V3는
다음 순서로 이 경쟁 조건을 닫는다.

1. Frame 완료 IRQ를 차단하고 VDMA S2MM을 정지한다.
2. 정지 완료 뒤 Pending IRQ를 읽고 Frame 완료이면 해당 Frame Store를 `ready`로
   복원한다.
3. 새 Geometry 적용을 `RETRY`로 보류한다.
4. 다음 COMMIT Poll에서 cache invalidate 후 Frame을 CPU-owned로 전환한다.
5. 등록된 Frame consumer가 PACKED17/Face Footer/H-Line 처리를 성공한 경우에만
   DMA-owned로 반환한다.
6. 모든 완료 Frame이 반환된 뒤 새 HSIZE/VSIZE/STRIDE를 적용하고 ACK한다.

Consumer 실패 Frame은 CPU-owned 상태로 보존된다. 옛 Geometry로 VDMA를 다시
시작하지 않으므로 기존 Frame Store를 덮어쓰지 않는다.

### 2.6 처리 경로 idle 증거와 정지 순서

`STAT3`의 `RUNNING=0`, `ARMED=0`만으로는 기존 Shot의 GPX/AXIS 데이터가 모두
끝났다고 판정할 수 없다. H6-B3A는 VDMA를 정지하기 전에 CTL23에 진단 인덱스
`0x10` CAPTURE를 요청하고 CTL24에서 다음 조건을 모두 확인한다.

| CTL24 bit | 의미 | VDMA 정지 허용값 |
|---:|---|---:|
| 12 | Processing pipeline idle | 1 |
| 13 | Echo 처리 idle | 1 |
| 14 | TDC-GPX 처리 경로 idle | 1 |
| 15 | TDC-GPX AXI4-Stream 출력 idle | 1 |
| 26 | TDC-GPX CDC reset busy | 0 |

CTL23은 `BUSY=0`, `VALID=1`, `ERROR=0`이어야 하고 CAPTURE 전후 sequence가 반드시
증가해야 한다. 다른 진단 요청이 BUSY인 경우, sequence가 그대로인 오래된 응답,
진단 ERROR, 위 네 idle 중 하나가 0인 경우와 CDC reset 중인 경우에는 VDMA 정지를
보류한다. 실패 주입 시험은 이 모든 경우에 `XAxiVdma_DmaStop()` 호출 수가 증가하지
않음을 확인한다.

완료 IRQ가 발생한 Frame은 모두 보존한다. 반면 활성 Face 중간에 운용자가 STOP을
발행해 Footer까지 끝나지 않은 현재 Frame은 완성된 Face가 아니므로 전송하지 않는다.
유효 Face를 버리지 않으려면 Face 완료 IRQ 직후 비활성 미러 구간에서 STOP해야 하며,
이 시간 여유와 다음 Face가 시작되지 않는지는 H6-B3B 보드 시험에서 확인한다.

### 2.7 COMMIT 제한시간과 Ethernet 분리

완료 DDR Frame은 callback 안에서 PS 소유 H-Line/Viewer 전송 큐로 복사한 뒤 VDMA에
반환한다. 물리 Ethernet 송신은 별도 task가 처리한다. 큐가 가득 차면 callback은
`XST_NO_DATA`를 반환하고 Frame을 CPU-owned로 유지한다. 이 사전 인계는 CSR COMMIT
명령 전에 끝나므로 `G_PHASE_TIMEOUT_US`를 소비하지 않는다. COMMIT 뒤에는 실제
VDMA 재설정과 Lane별 ACK만 제한시간 안에 수행한다.

재시도 루프에는 선택적 `poll_wait` callback을 둘 수 있다. FreeRTOS에서는
`taskYIELD()` 또는 `vTaskDelay()`를 연결해 Ethernet task가 전송 큐를 비울 기회를
보장한다. callback을 사용하지 않는 bare-metal은 watchdog 갱신과 Poll 1회 시간을
보드에서 측정해 `poll_limit`가 무한 busy-wait가 되지 않게 한다.

사전 정리 timeout은 CSR COMMIT 명령 전 오류이므로 Active 설정은 바뀌지 않는다.
그러나 VDMA는 이미 정지됐을 수 있다. 따라서 timeout 뒤 `RUN/ARM`을 바로 쓰지 않고,
진단/전송 큐/CPU-owned Frame 원인을 해소한 뒤 동일한 사전 정리와 COMMIT을 다시
수행한다. `overwrite_mask` 또는 `error_mask`가 set된 치명 실패는 자동 재시도하지
않고 VDMA와 LiDAR 운용 상태를 함께 초기화한다.

### 2.8 Face마다 한 번 발생하는 VDMA 완료 인터럽트

완료 callback이 `ready_mask`에 정확한 Frame Store를 게시하려면 S2MM 완료
인터럽트가 Face Frame마다 한 번 발생해야 한다. AMD AXI VDMA 드라이버도 인터럽트
사용 전에 `XAxiVdma_SetFrameCounter()`를 호출하도록 요구한다. V3 adapter는 초기화
과정에서 다음 계약을 강제한다.

- AXI VDMA는 `MM2S=0`, `S2MM=1`인 S2MM 전용 인스턴스다.
- Rise/Fall 각각 Frame Store가 최소 3개이고 드라이버의 `NumFrames`와 일치한다.
- S2MM 완료 인터럽트 임계값은 `1 Face Frame`이다.
- `XAxiVdma_DmaConfig().EnableFrameCounter=0`을 유지해 S2MM은 다음 Face로 계속
  순환한다. 이 값은 완료 인터럽트 임계값이 아니라 지정 Frame 수 뒤 DMA를 멈추는
  제어 비트다.
- `XAxiVdma_SetFrameCounter()`가 기능 없음 또는 설정 실패를 반환하면 RUN 전에
  초기화를 실패시킨다.
- VDMA channel reset 뒤에는 adapter를 다시 초기화하고 callback/임계값을 복구한
  뒤에만 RUN한다.

`C_ENABLE_DEBUG_INFO_14/15`는 Block Design에서 편집하는 `CONFIG`가 아니라 AXI VDMA
6.3이 생성하는 내부 Model Parameter다. Vivado 2025.2.1의 `component.xml`과 실제
생성 XCI에서 둘 다 `1`임을 확인했다. `INFO_14`는 S2MM delay counter field,
`INFO_15`는 S2MM frame completion counter를 구현한다. Parent가 존재하지 않는
`CONFIG.c_enable_debug_info_14/15`를 쓰면 안 된다.

AMD 2025.2.1 `XAxiVdma_DmaConfig()`는 연속 전송 제어 비트를 다시 쓰더라도
`XAxiVdma_SetFrameCounter()`가 설정한 완료 인터럽트 임계값 필드 `[23:16]`은
지우지 않는다. 따라서 이 시스템은 아래 두 조건을 동시에 만족한다.

```text
S2MM 수집       : Face 0 -> Face 1 -> Face 2 -> ... 계속 순환
PS 완료 알림    : Face 하나가 끝날 때마다 IRQ 1회
```

3개 Frame Store는 PS 처리 지연을 흡수하는 최소 운용 계약이다. 그래도 ISR가 한 Face
주기 이상 지연되면 완료된 Frame Store를 놓칠 가능성이 있으므로, H6-B3B에서 GIC
우선순위와 최악 ISR latency가 최소 Face 주기보다 짧은지 측정한다.

## 3. 자동 시험 결과

| 시험 | 검증 내용 | 결과 |
|---|---|---|
| VDMA transaction host test | Lane별 ACK, ACK 지연, snapshot 교체, retry limit | PASS |
| V3 control recovery host test | VDMA 실패 뒤 RTL terminal 상태 회수 | PASS |
| Xilinx adapter host test | CTL23/24 idle sequence, 정지 직전 완료 Frame, 1-Frame IRQ 설정, 3중 버퍼, cache 범위와 소유권/오류 정책 | PASS |
| Cortex-A9 compile | V2 공통 CSR/PACKED17 소유권, V3 transaction/control | PASS |
| 실제 BSP header compile | V3 adapter/frame service와 V2 XAxiVdma Lane | PASS |
| Vivado AXI VDMA 계약 | S2MM-only, Frame Store 3, generated XCI INFO14/15=1 | PASS |
| AMD VDMA 드라이버 계약 | 연속 S2MM 유지와 1-Face IRQ 임계값 필드 보존 | PASS |
| ABI/출력 폭 Gate | CSR ABI 2.7, V3 32/64-bit만 허용 | PASS |
| V2 L0 기준 Parent 토폴로지 | 새로 생성한 32-bit/64-bit 프로젝트의 S2MM/3중 버퍼/폭 재개방 검증 | PASS |

검증 BSP:

```text
C:/Projects/my_sp/ALINX/Logic/project_4_tdc_gpx_4chip/
project_4.vitis/platform2/export/platform2/sw/
standalone_ps7_cortexa9_0/include
```

최종 재현 명령:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b3a_ps_board_preflight.ps1 `
  -BspInclude C:/Projects/my_sp/ALINX/Logic/project_4_tdc_gpx_4chip/project_4.vitis/platform2/export/platform2/sw/standalone_ps7_cortexa9_0/include `
  -CheckVivadoVdma
```

PASS marker는 `LIDAR_V3_H6B3A_PS_BOARD_PREFLIGHT_PASS`다.

최종 재현 Stamp:

```text
.work/v3_h6b3a_ps_board_preflight/260812_h6b3a_frame_irq_final5
```

GUI 재확인용 V2 L0 기준 Parent 프로젝트:

```text
.work/p32/project_4_lidar_v2_l0.xpr
.work/p64/project_4_lidar_v2_l0.xpr
```

두 프로젝트는 V3 최종 bitstream가 아니다. V3와 동일한 Zynq-7000 VDMA 연결 규칙을
가진 검증 기준으로서 `S2MM-only`, Lane당 Frame Store 3개, 32/64-bit 입력 폭을
확인하는 용도다. V3 Top이 들어간 최종 Parent 생성·합성·보드 시험은 H6-B3B가
소유한다.

## 4. 보드 H6-B3 실행 순서

1. V3 4-Chip Parent bitstream와 XSA를 생성한다.
2. `STAT0/STAT1`에서 CSR ABI 2.7, Chip 수, STOP 수, Return 최대값, 32/64-bit
   출력 폭과 Processing/TDC clock을 읽어 빌드 설정과 비교한다.
3. Rise/Fall 각각 S2MM-only AXI VDMA와 최소 3개 Frame Store를 겹치지 않는 DDR
   영역에 예약한다.
4. Frame consumer를 `lidar_v3_xilinx_frame_service_init()`으로 연결하고
   `lidar_v3_xilinx_frame_service_bind_transaction()`으로 사전 정리와 COMMIT 중
   복구 callback을 함께 등록한다.
5. Face 완료 IRQ 직후 비활성 미러 구간에서 `lidar_v2_stop_and_disarm()`을 실행한다.
6. `STAT3` 운용 정지와 CTL23/24 진단 `0x10` 처리 idle을 확인한 뒤 Runtime Shadow를
   기록하고 `lidar_v3_commit_shadow()`를 실행한다.
7. VDMA Register readback으로 실제 `HSIZE/VSIZE/STRIDE`와 Frame 주소를 기록한다.
8. Adapter 초기화에서 1-Frame 완료 임계값 설정 성공을 확인하고, RUN/ARM 후 VDMA
   IRQ에서 완료 Frame index와 cache invalidate 범위를 기록한다.
9. DDR Word를 H6-B2 Golden과 비교한다.
10. PACKED17을 Face Header/H-Line으로 변환해 실제 Ethernet payload를 캡처하고
   H6-B2 Golden과 byte 단위로 비교한다.
11. Runtime Return `7 -> 3 -> 7`을 Face 경계에서 적용하고 Lane별 ACK, Frame
   경계, Active Version과 overwrite가 모두 정상인지 확인한다.
12. 장시간 운용에서 `overwrite_mask`, VDMA error, IRQ 원인별 진단 카운터와
    `PROCESSING_WARNING.schedule_overrun` Shot 시간 계약 오류를 기록한다.

## 5. H6-B3 종료 조건

- 실제 VDMA Register readback과 `CTL25~29` 일치
- 32-bit 또는 64-bit 선택 폭에서 DDR Golden 일치
- FreeRTOS/PetaLinux cache API 실호출 뒤 데이터 무결
- 실제 Ethernet payload Golden 일치
- Runtime Return 변경 중 Lane별 ACK와 Frame 경계 무결
- Rise/Fall 완료 IRQ가 Face마다 정확히 1회이며 최악 ISR latency가 Face 주기 미만
- 장시간 시험에서 설명되지 않는 overwrite, VDMA error, GPX transport/data fault 없음
- Parent timing, XDC, IOB와 물리 TDC-GPX/Encoder/Laser/Echo LVDS 시험 PASS
