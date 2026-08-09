# Zynq PS 제어·VDMA·Ethernet 예제

이 디렉터리는 `tdc_gpx_lidar_ctrl_v2`를 Zynq-7000 PS에서 운용하기 위한
참조 구현이다. 단순 레지스터 쓰기 예제가 아니라 다음 전체 순서를 묶는다.

1. CSR ABI와 합성 프로파일 확인
2. 안전 정지와 Runtime Shadow 설정
3. COMMIT 중 새 VDMA HSIZE/VSIZE/STRIDE 적용 및 ACK
4. RUN/ARM
5. VDMA S2MM 완료 버퍼의 캐시 소유권 전환
6. PACKED17 검증과 H-Line 변환
7. UDP payload 1,440 byte 이하로 Viewer 전송

## 1. 파일 역할

| 파일 | 역할 |
|---|---|
| `lidar_v2_ps_control.h/.c` | 운영체제와 무관한 CSR 접근, COMMIT, IRQ, GPX 레지스터 읽기 |
| `test_lidar_v2_ps_control.c` | 가짜 MMIO로 CSR API를 자동 검증하는 호스트 테스트 |
| `xilinx_standalone/lidar_v2_xilinx_vdma.h/.c` | Xilinx `XAxiVdma` S2MM 설정, IRQ 완료 추적, 캐시 소유권 전환 |
| `xilinx_standalone/lidar_v2_lwip_udp.h/.c` | lwIP raw UDP 전송 콜백 |
| `xilinx_standalone/lidar_v2_example_app.h/.c` | CSR부터 VDMA, PACKED17, UDP까지 연결한 실행 예제 |
| `xilinx_standalone/lidar_v2_main_example.c` | `xparameters.h` 주소와 IRQ를 연결하는 보드용 `main()` 골격 |
| `petalinux/lidar_v2_posix_udp.h/.c` | PetaLinux 사용자 공간용 POSIX UDP 전송 콜백 |
| `../lidar_packed17_ps_decoder.h/.c` | DDR Frame 검증, Cell 복원, Viewer Face Header/H-Line 생성 |

## 2. 하드웨어 연결 계약

- IP의 `S_AXI_CSR`은 PS `M_AXI_GP0` 같은 제어 경로에 연결한다.
- `m_axis_rise`와 `m_axis_fall`은 각각 독립된 AXI VDMA S2MM에 연결한다.
- IP의 합성 출력 폭 `G_OUTPUT_WIDTH`와 VDMA `S_AXIS_S2MM` 폭은 반드시
  동일한 32/64/128 bit여야 한다.
- Zynq-7000 HP 포트의 메모리 AXI 폭은 64 bit여도 된다. 이는 스트림 폭과
  다른 계약이며 AXI VDMA와 AXI protocol/data-width 변환 경로가 처리한다.
- Rise/Fall VDMA는 서로 겹치지 않는 DDR 영역과 최소 3개 Frame Store를
  갖는 것을 권장한다.
- VDMA IRQ는 `XAxiVdma_WriteIntrHandler()`에 연결한다. 완료 콜백이 어떤
  Frame Store를 CPU가 읽어도 되는지 표시한다.

## 3. 안전한 설정 순서

```text
STAT0/STAT1 ABI·합성값 확인
        ↓
STOP + DISARM
        ↓
CTL1..CTL20 Runtime Shadow 기록
        ↓
COMMIT 시작
        ↓
CTL25 PENDING 확인
        ↓
CTL26..CTL29의 VDMA HSIZE/VSIZE/STRIDE 적용
        ↓
실제 VDMA 설정 성공 뒤 CTL25 ACK
        ↓
COMMIT SUCCESS와 Active Version 확인
        ↓
RUN + ARM
```

VDMA ACK를 먼저 쓰면 RTL은 새 Frame 계약이 적용됐다고 판단하므로 금지한다.
`lidar_v2_commit_shadow()`의 poll callback으로
`lidar_v2_xilinx_service_vdma_profiles()`를 전달하면 이 순서를 지킨다.

주요 Runtime 입력의 뜻은 다음과 같다.

- `target_range_window_5ns`: 레이저 목표 왕복시간
  `(2R/c, TARGET_RANGE_WINDOW_5NS)`, 5 ns 단위
- `fire_done_timeout_5ns_ticks`: Fire 명령 후 `fire_done` 수신 최대 대기시간,
  5 ns 단위
- `optical_shot_interval_udeg`: 인접한 레이저 발사 후보점 사이의 요청 광학각,
  micro-degree 단위
- `tdc_bus_clk_div`, `tdc_bus_ticks`: Runtime TDC-GPX 버스 읽기 타이밍
- `visible_returns`: DDR와 Viewer에 남길 Return 수 `1..합성 최대값`

`Fire 명령 후 fire_done 수신 최대 대기시간`은 레이저 목표 왕복시간보다 클 수
없다. 요청 광학각 후보점까지 Fire 응답, 목표 왕복시간, GPX Drain과 처리 경로가
끝나지 않으면 `PROCESSING_WARNING.schedule_overrun` Shot 시간 계약 오류다.

## 4. VDMA Frame과 버퍼 크기

COMMIT 중 RTL이 다음 값을 `CTL25..CTL29`에 게시한다.

```text
GEOMETRY[15:0]  = HSIZE bytes
GEOMETRY[31:16] = VSIZE lines
STRIDE[15:0]    = DDR line stride bytes
```

PS가 예약할 한 Frame Store의 최소 크기는 `STRIDE × VSIZE`이다. 그러나 Runtime
Return 수에 따라 HSIZE와 VSIZE가 달라질 수 있으므로 실제 DDR 예약 크기는 합성
최대 Return 수에서 계산한 최대 Frame 크기로 고정한다. 프로파일 변경 때에는
VDMA 레지스터만 현재 HSIZE/VSIZE/STRIDE로 갱신한다.

## 5. DDR 캐시와 소유권

Zynq-7000 Cortex-A9 캐시는 VDMA HP 쓰기와 자동 일치하지 않는다.

1. VDMA에 버퍼를 넘기기 전 해당 범위의 dirty cache line을 flush한다.
2. VDMA 완료 IRQ 뒤 CPU가 읽기 전에 `STRIDE × VSIZE` 범위를 invalidate한다.
3. CPU가 PACKED17을 읽고 Ethernet 전송을 끝낼 때까지 Frame을 CPU-owned로 둔다.
4. 처리가 끝난 뒤 DMA-owned로 반환한다.

예제는 이 순서를 수행하고, VDMA가 CPU-owned 또는 아직 미처리인 Frame을 다시
덮으려 하면 `overwrite_mask`를 세운다. 이 값은 실제 데이터 손실 증거이므로
운용 중 무시하면 안 된다.

## 6. PACKED17에서 Ethernet까지

`lidar_v2_example_app_step()`은 완료된 Rise/Fall Frame마다 다음을 수행한다.

```text
Face Footer 검증
  → Face index와 Active Version 확인
  → Shot Line과 Cell Metadata 검증
  → 17-bit Hit 복원
  → 16채널 × 선택 Return의 H-Line 생성
  → 첫 1,440 B Viewer Face Header 전송
  → 이후 32 B H-Line Header + 3 B/sample UDP 전송
```

DDR에 기록된 Active Version과 현재 CSR Active Version이 다르면 예제는 처리를
거부한다. 실제 제품에서 설정 변경 전 Frame도 큐에 남길 수 있다면 PS는 Active
Version별 Face geometry snapshot을 보관해야 한다.

## 7. IRQ 처리

Zynq GIC에는 통합 IP의 `o_irq`를 level-high로 연결하는 방식을 권장한다.

1. `IRQ_FLAG & IRQ_ENABLE`을 읽는다.
2. 원인별 STAT와 누적 진단 카운터를 먼저 읽고 소프트웨어 로그에 보존한다.
3. 원인 레벨을 해소하거나 복구 절차를 완료한다.
4. 실제 처리가 끝난 bit만 `IRQ_FLAG`에 W1C로 쓴다.

`lidar_v2_main_example.c`는 `lidar_board_handle_irq_sources()`를 호출한다. 보드
구현은 pending bit별 진단값을 읽고 복구가 완료된 IRQ bit만 반환해야 한다. 반환하지
않은 bit는 `last_irq_flags`와 하드웨어 sticky flag에 남으므로 원인 정보를 읽기 전에
자동으로 지워지지 않는다.

한 IRQ를 처리하는 동안 다른 원인이 생겨도 각 bit는 독립적으로 sticky 보존된다.
같은 원인이 이미 high인 동안 반복된 횟수는 IRQ bit만으로 알 수 없으므로 해당
누적 진단 카운터도 같이 읽는다.

## 8. 외부 TDC-GPX 레지스터 읽기

`lidar_v2_read_gpx_register(device, chip, address, ...)`는 한 번에 한 칩의 한
28-bit 레지스터를 읽는다. 내부적으로 `CTL23`에
`0x100 | 0xC0 | (chip << 4) | address`를 기록하고, sequence 증가와 VALID를
확인한 뒤 `CTL24[27:0]`을 반환한다. RUN 중 버스 소유권 충돌을 피하려면 레이저
발사를 끄고 STOP/DISARM 상태에서 진단 읽기를 수행하는 것이 보수적이다.

## 9. FreeRTOS/lwIP와 PetaLinux

- lwIP raw API는 lwIP core thread 문맥에서만 호출한다. 다른 FreeRTOS task에서
  처리한다면 mailbox 또는 `tcpip_callback()`으로 전송 작업을 넘긴다.
- PetaLinux에서는 VDMA 레지스터와 DMA 메모리를 사용자 공간 `/dev/mem`으로 직접
  다루지 않는다. kernel DMAengine 또는 전용 driver가 dma-buf/완료 이벤트를
  사용자 공간에 제공하고, 사용자 공간은 `lidar_v2_posix_udp_packet_sink()`로
  Ethernet 전송만 담당하는 구성이 안전하다.

## 10. 자동 점검

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_ps_control_example.ps1
```

생성된 Vitis BSP까지 Xilinx adapter를 컴파일하려면 다음처럼 BSP include 경로를
추가한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_ps_control_example.ps1 `
  -BspInclude <exported-platform>/sw/<domain>/bspinclude/include
```

이 테스트는 CSR API의 호스트 실행, Cortex-A9 공통 코드, Xilinx VDMA/lwIP adapter와 `lidar_v2_main_example.c` 보드 골격의 BSP 컴파일을 검증한다. 실제 Ethernet
대역폭, DDR cache 일관성, VDMA Frame overwrite 여부는 최종 보드에서 별도 장시간
검증해야 한다.
