# TDC-GPX LiDAR Integrated Controller V3 제품 가이드

## 1. 역할

`tdc_gpx_lidar_ctrl_v3_top`은 모터 위치 추적, 레이저 발사 안전 제어, 선택적
Echo LVDS 입력, 외부 TDC-GPX 4-Chip I-Mode 수집, HLS 기반 PACKED17 처리와
Rise/Fall AXI4-Stream 출력을 하나의 IP로 제공한다.

V3는 V2의 검증된 AXI4-Lite CSR, Shadow/Active COMMIT, IRQ, CDC, Reset,
TDC-GPX bus PHY와 IFIFO 전체 Drain을 유지한다. GPX 28-bit word 중 하위
17-bit 거리 Hit를 Cell, Shot Line, Face Footer로 바꾸는 Processing 데이터 경로만
H1~H4 HLS RTL로 교체한다.

## 2. 합성 전 고정 항목

- TDC-GPX Chip 수: 1~4
- Chip당 STOP 수: 1~8
- STOP당 최대 Return 저장 용량: 1~7
- Rising/Falling 가능 Chip mask: 같은 Chip을 양쪽에 포함할 수 있음
- 출력 AXI4-Stream 폭: 32 또는 64 bit
- 다면미러 면 수: 1~5
- Echo Receiver 및 Echo Simulation 포함 여부
- Processing clock과 TDC-GPX 버스/수집 clock 주파수와 동기/비동기 모드
- OEN PCB 배선 정책

외부 TDC-GPX 기준 clock은 위 Processing/TDC Generic과 별개다. 모든 외부
TDC-GPX에는 **40 MHz 기준 clock (Tref=25 ns)**을 PCB에서 공급해야 한다.

## 3. Runtime 핵심 계약

- `BUS_CLK_DIV/BUS_TICKS`: Runtime TDC-GPX 버스 읽기 타이밍
- 레이저 목표 왕복시간 (`TARGET_RANGE_WINDOW_5NS`): CTL12의 유일한 설정원
- Fire 명령 후 `fire_done` 수신 최대 대기시간 (`FIRE_DONE_TIMEOUT_5NS_TICKS`)
- 인접한 레이저 발사 후보점 사이의 요청 광학각 (`OPTICAL_SHOT_INTERVAL_UDEG`)
- Runtime Return 수: 합성 시 최대 용량 이내에서 Face 경계 COMMIT으로 적용

측정 시작 기준시점 (T0)은 물리 모드에서 동기화된 `fire_done`을 승인하고
`start_tdc`를 발생시키는 사건이다. TDC-GPX `Reg7.MTimer`는 CTL12의 5 ns 값을
40 MHz 기준 25 ns 단위로 올림 변환해 자동 생성한다.

## 4. Parent 연결

Rise/Fall AXI4-Stream의 폭은 `G_OUTPUT_WIDTH`와 동일한 VDMA S_AXIS에 연결한다.
Zynq-7000에서는 각 VDMA의 DDR Master를 64-bit AXI4-to-AXI3 변환 뒤 독립 HP
포트에 연결하는 구성을 검증 기준으로 사용한다. VDMA는 S2MM-only, Lane당
3 Frame Store, Face 1개 완료마다 IRQ를 발생시키는 계약이다.

Echo Receiver를 비활성화하면 LVDS 입력과 STOP 출력 포트는 XGUI/IPI에서
숨겨진다. OEN/LF/ERRFLAG는 IP 공개 계약에서 제거하지 않으며 Parent PCB 정책에
따라 외부 배선하거나 안전한 상수/미연결 정책을 적용한다.

## 5. 검증 범위

패키지는 H1~H4의 Vitis HLS 생성 Verilog와 LUTRAM 초기화 파일을 내부에 복사한
self-contained IP다. 패키지 무결성 검사, 4-Chip 32/64-bit Block Design 재개방,
대표 Parent 합성·배치배선·bitstream·XSA 결과를 별도 H6-B3B 문서에 기록한다.
물리 PCB의 40 MHz 기준 clock, GPX bus 파형, DDR cache API, Ethernet 지속 전송과
레이저 안전 인터록은 보드 시험 전용 Gate다.
