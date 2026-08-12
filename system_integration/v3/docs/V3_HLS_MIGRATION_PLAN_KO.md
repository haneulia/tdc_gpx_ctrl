# V3 HLS 마이그레이션 최종 계획

## 1. 목적

V3는 V2 기능을 새로 해석하지 않는다. V2의 이벤트 순서, 오류 분류,
PACKED17 DDR ABI 및 PS/Viewer 결과를 Golden 기준으로 사용하여 측정 후 데이터
처리 경로만 HLS로 교체한다.

## 2. 변경하지 않는 계약

1. 외부 TDC-GPX I-Mode word는 28 bit이며 하위 17 bit가 Hit 값이다.
2. 물리 IFIFO는 Runtime 직렬화(전시) Return 슬롯 수와 무관하게 EF 완료까지 Drain한다.
3. Return 1~7 중 설정값을 넘는 Hit는 의도적인 필터이며 fault가 아니다.
4. 8번째 이상의 물리 Return만 `return_overflow`로 기록한다.
5. 측정 시작 기준시점 (T0), Shot identity와 Active version은 데이터 끝까지
   변경하지 않고 전달한다.
6. HLS 출력은 폭에 독립적인 canonical 32-bit Word다.
7. RTL AXIS packer가 합성 시 32/64-bit Beat로 묶는다. V3는 128 bit를 지원하지
   않는다.

## 3. HLS 경계

```text
TDC-domain GPX acquisition RTL
    -> XPM async result FIFO
    -> Processing-domain HLS data pipeline
    -> Rise/Fall canonical 32-bit Word
    -> RTL 32/64-bit AXIS packer
    -> VDMA
```

HLS 내부 `hls::stream`은 동일 Processing clock에서만 사용한다. CDC, XPM FIFO,
Reset synchronizer 및 물리 I/O는 HLS에 넣지 않는다.

## 4. 데이터 타입 규칙

- 알고리즘은 `stdint.h`의 정확한 폭 타입을 사용한다.
- Hit17은 `uint32_t`에 저장하고 `0x1FFFF`로 제한한다.
- C/C++ bit-field와 컴파일러 종속 구조체 padding을 ABI로 사용하지 않는다.
- `ap_uint`는 HLS/RTL 경계의 비트 정확한 packed payload에만 사용한다.
- H0의 `lidar_v3_hls_contract.hpp`는 H1~H4 전체 계약을 포함하는 안정된 진입점이다.
- 실제 packed field 위치는 소유 단계별 `lidar_v3_h1_raw_hit_contract.hpp`,
  `lidar_v3_h2_cell_contract.hpp`, `lidar_v3_h3_frame_contract.hpp`,
  `lidar_v3_h4_word_contract.hpp`가 정의한다.
- VHDL Adapter는 `lidar_v3_hls_contract_pkg.vhd`의 같은 이름 상수를 사용한다.
- 숫자 Bit slice를 구현과 테스트에서 직접 반복하지 않는다.

## 5. 단계와 종료 조건

| 단계 | 상태 | 구현 | 종료 조건 |
|---|---|---|---|
| H0 | 완료 | 공통 상한, 의미 기반 Bit field, ABI 3.2, 실행 환경 | C++/VHDL 계약과 Vitis 2025.2 재현 실행 PASS |
| H1 | 완료 | Raw28-to-Hit17 | 모든 topology와 fault C/RTL co-sim 및 V2 차동 회귀 PASS |
| H2 | 완료 | Hit-to-Cell | Return 1~7, filter, overflow, timeout, abort PASS |
| H3 | 완료 | Cell-to-Frame | 5 topology, Rise/Fall 독립 stall, Face gap, abort, 150/200 MHz PASS |
| H4 | 완료 | PACKED17/Shot Metadata/Hole/Face Footer | V2 최종 AXI Beat 4개와 150/200 MHz OOC PASS |
| H5 | 완료 | 혼합 RTL/HLS Top | H1~H4와 유지 RTL packer 통합, V2 종단 차등, abort/stall/idle, 150/200 MHz x 32/64-bit OOC PASS |
| H6-A | 완료 | Parent 데이터 경계 | GPX bus/EF 전체 Drain, CDC, H1~H4, V2 종단 차분 5개, 150/200·200/150 MHz OOC PASS |
| H6-B1 | 완료 | 통합 제어·데이터 Top | V2 CSR/Shadow/Active/COMMIT/IRQ와 V3 HLS 데이터 경로, 4 Chip 기능 회귀, formatter 진단, 두 제품 clock OOC PASS |
| H6-B2 | 보드 독립 범위 완료 | Runtime VDMA/DDR/PS | 7→3→7 Return의 Lane별 VDMA ACK 원자성, 32/64-bit DDR Word Golden, Cortex-A9 PS cache 소유권 및 H-Line/Ethernet 바이트 비교 PASS |
| H6-B3A | 완료 | PS 보드 투입 준비 | CSR ABI 2.7, Lane별 적용·ACK, 처리-idle gate, 1-Face IRQ/3중 버퍼, Cortex-A9 BSP와 Vivado VDMA 계약 PASS |
| H6-B3B | 다음 | Parent 보드 증거 | 실제 VDMA/DDR/cache API/Ethernet, bitstream와 물리 I/O PASS |

## 6. 성능·안전 Gate

- HLS 내부 Metadata scrub과 Cell 방출 메모리 loop의 initiation interval은 1이다.
- 입력 승인 간격은 V2 상태 직렬 처리보다 시스템 처리율을 악화시키지 않아야 한다.
  H5는 상위 async FIFO 뒤에서 시작하므로 FIFO 최대 점유율과 Shot 처리 여유는
  H6 Parent 통합에서 실측한다. H5의 H1 inflight count를 FIFO 점유율로 해석하지
  않는다.
- H3의 Rise/Fall Cell Slot은 VDMA Line이 아니다. H4가 Slot을 canonical 32-bit
  Word로 만들고 H5의 RTL packer가 32/64-bit AXI4-Stream Beat로 결합한다.
- 출력 backpressure 동안 payload와 종료 정보는 변하지 않는다.
- HLS 내부 FIFO 깊이는 추론 기본값에 맡기지 않고 명시한다.
- Abort 또는 Reset 뒤 이전 Face payload가 다음 Face에 나타나지 않는다.
- HLS Idle은 부분 Cell/Line/Footer와 보류 출력이 모두 없는 상태다.
- HLS 적용으로 새로운 Hole 또는 `schedule_overrun`이 증가하지 않는다.
- 물리 `fire_done`에서 측정 시작 기준시점 (T0)까지의 경로는 변경하지 않는다.
- 150/200 MHz, 200/150 MHz 기능 회귀와 4:1 CDC 스트레스 결과를 유지한다.

## 7. 형상관리

V2는 읽기 전용 Golden 기준으로 유지한다. V3의 각 단계는 소스, 독립 테스트,
도구 설정, 결과 문서를 한 Git 체크포인트로 커밋한다. 생성된 HLS 프로젝트는
`.work/`에만 두고, 최종 통합 IP에 필요한 Export RTL은 도구 버전과 소스 hash를
기록한 Release 단계에서만 추적한다.

H0~H4의 전체 Bit Map과 수정 절차는
[`V3_H0_H4_HEADER_CONTRACT_KO.md`](V3_H0_H4_HEADER_CONTRACT_KO.md)를 단일 검토
문서로 사용한다. 단계 Header를 수정한 커밋은 해당 문서의 ABI 버전과 검증 Stamp를
같이 갱신해야 한다.

H5의 통합 경계와 결과는
[`V3_H5_MIXED_TOP_SIGNOFF_KO.md`](V3_H5_MIXED_TOP_SIGNOFF_KO.md), 테스트 소유권과
변경 영향은 [`V3_H5_TESTBENCH_GUIDE_KO.md`](V3_H5_TESTBENCH_GUIDE_KO.md)에
기록한다. H5 PASS는 Processing-domain 데이터 경로 체크포인트이며 Parent 전체
Sign-off가 아니다.

H6-A의 데이터 흐름·CDC·실패한 최적화 실험·배치·배선 근거는
[`V3_H6_PARENT_DATA_SIGNOFF_KO.md`](V3_H6_PARENT_DATA_SIGNOFF_KO.md), 테스트 소유
범위는 [`V3_H6_TESTBENCH_GUIDE_KO.md`](V3_H6_TESTBENCH_GUIDE_KO.md)에 기록한다.
H6-A PASS는 물리 GPX Drain부터 최종 AXI까지의 데이터 경계 판정이며,
CSR/VDMA/DDR/PS와 Parent bitstream을 포함한 전체 Sign-off는 아니다.

H6-B1의 통합 Top 구조, COMMIT 안전 유휴 조건, H4 formatter 진단 ABI,
4 Chip 기능 회귀와 OOC 구현 근거는
[`V3_H6B_INTEGRATED_TOP_CHECKPOINT_KO.md`](V3_H6B_INTEGRATED_TOP_CHECKPOINT_KO.md)에
기록한다. H6-B1 자체의 PASS 범위에는 Runtime VDMA/DDR/PS와 Parent 보드 검증이
포함되지 않는다.

H6-B2의 Face 경계 VDMA ACK, DDR Word Golden, PS cache 소유권 및 H-Line/Ethernet
결과는
[`V3_H6B2_RUNTIME_VDMA_DDR_PS_SIGNOFF_KO.md`](V3_H6B2_RUNTIME_VDMA_DDR_PS_SIGNOFF_KO.md)에
기록한다. 실제 AXI VDMA Register, 물리 DDR DMA, FreeRTOS/PetaLinux cache API와
Ethernet 송신은 H6-B3B에서만 닫는다. H6-B3A의 PS 트랜잭션과 보드 투입 절차는
[`V3_H6B3A_PS_BOARD_PREFLIGHT_KO.md`](V3_H6B3A_PS_BOARD_PREFLIGHT_KO.md)에 기록한다.
