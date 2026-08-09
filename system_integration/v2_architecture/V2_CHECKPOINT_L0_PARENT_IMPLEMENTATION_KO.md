# V2 Checkpoint L0 Parent 구현 Sign-off

> 이 문서는 최초 L0 Parent 구현의 이력을 보존한다. 최신 4-chip IOB 고정,
> Register-read 200 MHz 경로 보완, PS VDMA/Ethernet 참조 및 `WNS +0.240 ns`


## 1. 결론

`tdc_gpx_lidar_ctrl_v2:2.0`을 Zynq-7000 `xc7z020clg484-2` Parent에 넣고
Rise/Fall VDMA, HP0/HP1, CSR, IRQ 및 VT_HRL_2 계열 서비스 핀까지 연결한
GUI 프로젝트가 합성·배치·배선·비트스트림 검증을 통과했다.

판정은 **Parent 구현 PASS / 물리 보드 Release 보류**다.

## 2. 이번 단계에서 닫은 문제

| 항목 | 보완 내용 |
|---|---|
| Processing↔TDC stream Reset | 비동기 FIFO Reset을 Source clock 순차회로가 소유하고 Destination Reset을 4단 XPM으로 전달 |
| stale payload | Source 또는 Destination Reset 전 보류 payload를 폐기하는 전용 TB 추가 |
| system command Reset | Processing/TDC sink pulse 생성 FSM을 각 목적지 clock의 동기 Reset으로 변경 |
| Shot ingress Reset | FIFO write-enable을 구동하는 pending slot을 Processing 동기 Reset으로 변경 |
| GPX 양방향 핀 | 공개 Top에 Chip/bit별 명시적 `IOBUF` 적용 |
| OEN IOB | `DYNAMIC_CONNECTED`일 때만 IOB 강제, pull-up/미연결 구성은 FALSE |
| Parent CDC | blanket clock group 제거, handshake/FIFO별 max-delay·bus-skew·narrow false path 적용 |
| TDC 출력 | false path 대신 8 ns register-to-pad 계약 적용 |
| VDMA profile 제어 | 외부 bridge/GPIO를 제거하고 통합 IP 내부 49-bit CDC와 CTL25~29로 이관 |
| 4-chip Parent | `G_NUM_CHIPS=4`, Rise `0011`, Fall `1100`, GPX D[111:0] 및 TDC0~3 핀 적용 |
| Sign-off gate | endpoint 수, 171핀, CDC, bus skew, methodology, DRC, WNS/WHS, bitstream 크기 검사 |
| 패키지 회귀 | 사용자 Tcl Store와 분리된 임시 캐시, Critical Warning hard-fail, 일반 Warning ID/최대 수 계약 추가 |

## 3. 검증 근거

| 검증 | 세션/마커 | 결과 |
|---|---|---|
| K03 150/200, 200/150 | `260809212016_v2_k03_integration` | 내부 VDMA profile CDC 포함 PASS |
| Stream Reset 200→150 | `260809190043_v2_stream_gateway_reset` | PASS |
| Shot coordinator 150/200 | `260809_coordinator_holdmask_impl01_v2_gpx_acquisition_coordinator` | WNS `+1.381/+0.558 ns`, latch 0 |
| 4-chip Parent synth | `260809_l0_parent_4chip_synth02` | WNS/WHS `+0.355/+0.036 ns` |
| 4-chip Parent 구현 | `260809_l0_parent_4chip_impl01` | WNS/WHS `+0.082/+0.023 ns`, bitstream PASS |
| K0~K10 package/OOC | `260809_220426_k010_ip_package` | 최신 92개 자산, 3 profile, Critical Warning 0, Error 0, Warning 계약 PASS |

8 ns는 기능을 느슨하게 만든 임의 숫자가 아니다. 최악 register-to-pad 실측은
`7.717 ns`이고, 별도 RTL/버스 회귀가 보장하는 최소 25 ns Runtime TDC-GPX
버스 읽기 유지시간 안에 `17.283 ns`의 핀 안정 구간을 남긴다.

## 4. CDC 예외 감사

- 일반 clock-domain 전체 false path: 없음
- GPX 데이터 waiver: 112개 포트→112개 IOB capture FF에만 적용
- 비동기 상태 입력 waiver: EF1/EF2/IRFLAG 12개
- 일반 비동기 서비스 입력: Encoder 3개, `fire_done` 1개, GPX 상태 12개
- 활성 Critical CDC: 0
- mailbox bus-skew 위반: 0

GPX 28-bit 응답은 2FF 동기화 버스가 아니다. RDN 선행, 외부 데이터 안정화,
최소 25 ns 유지시간과 IOB capture를 묶은 비동기 병렬 버스 프로토콜이다.

## 5. 구현 결과

| Clock/path | 여유 |
|---|---:|
| CSR 100 MHz | `+0.747 ns` |
| Processing 150 MHz | `+0.595 ns` |
| TDC 200 MHz | `+0.082 ns` |
| 전체 Setup | `+0.082 ns` |
| 전체 Hold | `+0.023 ns` |

Methodology의 vendor AXI bridge `LUTAR-1`, violation이 없는 `TIMING-9`,
mailbox fanout `TIMING-37`, DRC `REQP-181`은 원본 보고서에서 검토했으며
Critical 또는 blocking 항목은 0이다. 보드 서비스 핀 171개는 모두
`PACKAGE_PIN`과 `IOSTANDARD`를 가진다.

200 MHz의 최악 setup은 TDC0 bus PHY의 `tick[2]`에서 IOB `WRN`까지이며
2 LUT, data-path 4.014 ns, 배선 비중 85.252%다. 합성 단계에서 발견된
all-lane Shot 승인 경로는 유휴 시 dispatch mask를 유지하도록 정리해 제거했다.
따라서 현재 제한은 4-chip ready 축약 조합논리가 아니라 실제 GPX 핀까지의
물리 배선이다.

Rise/Fall VDMA는 각각 PS7 HP0/HP1과 1:1이므로 AXI4→AXI3 전용 Protocol
Converter를 사용한다. AXI Interconnect/SmartConnect도 변환은 가능하지만 이
경로에는 중재와 라우팅이 필요 없다. 여러 DMA Master가 하나의 HP 포트를 공유할
때만 SmartConnect로 전환한다. PS GP0가 통합 CSR과 두 VDMA AXI-Lite 포트를
제어하는 fan-out 경로에는 AXI Interconnect를 유지한다.

## 6. 범위 밖 항목

이 Checkpoint는 다음을 증명하지 않는다.

- 실제 GPX Chip의 40 MHz Tref, MTimer 및 Register readback
- PCB 신호 무결성과 TDC 200/150/125/100/50 MHz 장기 운용
- VDMA Frame Buffer를 통한 실제 DDR byte 무결성
- FreeRTOS/PetaLinux cache 동기화 API
- 실제 레이저 안전 인터록
- 지속 Ethernet 처리량

따라서 Stage L은 아직 끝나지 않았다. L0 Parent 구현은 닫혔고, 다음은 보드
Runtime 및 PS 소프트웨어 증거를 수집하는 L1 작업이다.

## 7. 유지보수 규칙

1. 공개 Top/Generic 변경 뒤에는 패키지 소스 동기화와 Parent 재생성을 모두 수행한다.
2. Clock-domain 전체 false path를 추가하지 않는다.
3. GPX pin 수 또는 capability mask 변경 시 171핀 계약을 새 profile에 맞게 갱신한다.
4. TDC 출력 8 ns를 변경할 때는 최소 버스 유지시간과 실제 최악 경로를 함께 기록한다.
5. Reset/CDC 변경 시 `tb_lidar_stream_gateway_reset`을 먼저 실행한다.
6. 최종 Parent 결과는 고유 세션 이름과 비트스트림 크기를 기록한다.
7. K0~K10 package runner의 허용 Warning ID나 최대 수를 늘릴 때는 원인 RTL과
   profile별 증가량을 먼저 문서화한다. 단순히 PASS를 얻기 위한 상향은 금지한다.
