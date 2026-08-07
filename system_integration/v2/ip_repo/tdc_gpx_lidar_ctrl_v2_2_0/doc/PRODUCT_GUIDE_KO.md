# TDC-GPX LiDAR Integrated Controller V2 사용 가이드

## 1. IP 식별

- VLNV: `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0`
- 합성 Top: `tdc_gpx_lidar_ctrl_v2_top`
- 대상 디바이스: `xc7z020clg484-2`
- CSR: AXI4-Lite 32 Control / 32 Status, IRQ는 Level-High
- 데이터 출력: Rise/Fall 독립 AXI4-Stream `PACKED17`

이 IP는 Motor/Face 추적, 레이저 Shot 실행, 선택형 Echo LVDS 수신,
TDC-GPX I-Mode 수집, Hit/Cell/Face 조립 및 VDMA 프로파일 생성을 하나의
Top으로 통합한다. 기존 `tdc_gpx_top:1.0`과 이름 및 버전이 다르므로 같은
Vivado IP Catalog에서 병행할 수 있다.

## 2. 클럭 계약

| 클럭 | 역할 | XGUI Generic |
|---|---|---|
| `s_axi_csr_aclk` | CSR 설정, 상태 및 IRQ | `G_CSR_CLK_MHZ` |
| `proc_aclk` | Motor/Laser, Echo 진단, Cell/Face, AXIS/VDMA | `G_PROC_CLK_MHZ` |
| `i_tdc_clk` | 외부 TDC-GPX 버스 및 I-Mode Drain | `G_TDC_CLK_MHZ` |

세 주파수는 `50/100/125/150/200 MHz` 중 하나여야 한다.
`G_STREAM_CLK_MODE=ASYNC`는 Processing/TDC 주파수의 크기 순서를 제한하지
않는다. `SYNC`는 두 주파수가 같고 두 포트가 같은 물리 클럭 net에 연결될
때만 사용한다. 정규 Sign-off 조합은 `150/200 MHz`, `200/150 MHz`이며,
동기 모드 패키지 검증은 `150/150 MHz`로 수행한다.

## 3. 합성 시 고정되는 항목

- `G_NUM_CHIPS`: 물리 TDC-GPX Chip 수, 1~4
- `G_STOPS_PER_CHIP`: Chip당 STOP 수, 1~8
- `G_MAX_RETURNS_PER_STOP`: STOP당 최대 Return 용량, 1~7
- `G_RISE_CAPABILITY_MASK`, `G_FALL_CAPABILITY_MASK`: Chip별 slope 기능
- `G_OUTPUT_WIDTH`: Rise/Fall AXIS 폭, 32/64/128 bit
- `G_NUM_FACES`: 다면 미러 면 수, 1~5
- `G_ENABLE_ECHO_RECEIVER`: 내부 LVDS-to-STOP frontend 합성 여부
- `G_ENABLE_ECHO_SIMULATION`: 합성 시뮬레이션 Echo source 포함 여부
- `G_OEN_MODE`: TDC-GPX OEN PCB 배선 계약

Rise/Fall mask는 겹칠 수 있으므로 한 Chip에서 두 slope를 모두 수집할 수
있다. 모든 활성 Chip은 한 slope 이상에 배정되어야 하며, 시스템 정책상
Rise 활성 Chip 수는 Fall 활성 Chip 수보다 작을 수 없다.

## 4. Echo Receiver 선택

`G_ENABLE_ECHO_RECEIVER=false`이면 LVDS frontend 논리는 합성에서 제외되고
`i_pd_lvds_p`, `i_pd_lvds_n`, `o_tdc_stop`은 IP Integrator에서 숨겨진다.
외부 회로가 TDC-GPX STOP을 직접 구동하는 구성에 사용한다.

`G_ENABLE_ECHO_SIMULATION=true`는 Receiver가 활성일 때만 허용된다. 이
모드는 물리 레이저 `fire_pulse`를 발생시키지 않으며, 시뮬레이션 사건과
물리 `fire_done` 사건은 서로 배타적으로 처리된다.

## 5. Runtime 레이저 정지와 GPX 재설정

운용 중 GPX 레지스터를 바꿀 수 있다. 이때 `STOP`은 모터 위치 추적까지
정지시키므로 권장 절차가 아니다. 다음 순서로 `DISARM`을 사용한다.

1. `CTL0.DISARM`을 W1S한다.
2. `STAT3.ARMED=0`, `SCHEDULER_ENABLE=0`, `PHYSICAL_FIRE_ENABLE=0`을 확인한다.
3. 목표 왕복시간만 바꾸면 `CTL12`만 쓴다. MTimer 이외의 GPX bit도 바꿀
   때만 `CTL21/CTL22` staging image를 추가 수정한다.
4. `CTL0.COMMIT`을 W1S한다. RTL이 진행 중인 Shot과 GPX Drain이 끝나는
   safe point를 기다린 뒤 Processing/TDC 설정을 원자적으로 전환한다.
5. `STAT2.BUSY=0`, `DONE_STICKY=1`, `SUCCESS_STICKY=1`, `ERROR_STICKY=0`을
   확인하고 `CTL21.VIEW_ACTIVE=1`로 승인된 effective image를 읽는다.
6. 실제 외부 Chip까지 확인할 때는 CTL23의 `11CCAAAA` 물리 read를 사용하고
   CTL24의 `{요청 주소[3:0], 실제 data[27:0]}`를 확인한다.
7. 외부 레이저 permit과 `STAT3.COMMAND_READY=1`을 확인한 뒤 `CTL0.ARM`을
   W1S한다.

`DISARM`은 RUN과 Encoder/Face 위치 추적을 유지하지만 새 Shot 후보와
`fire_pulse`를 차단한다. COMMIT 자체도 설정 전환 중 scheduler를 닫지만,
물리 레이저 안전성과 소프트웨어 절차의 명확성을 위해 먼저 DISARM한다.

CTL21/22 active view는 마지막 성공 COMMIT의 설정 이미지이며 물리 bus
readback이 아니다. 실제 read INDEX는 `0xC0 | (Chip<<4) | Register`이고,
CAPTURE bit를 더해 CTL23에 쓴다. 예를 들어 Chip 1 Reg7은 `0x1D7`이다.
물리 read 중 RTL은 acquisition만 자동 pause/resume하며 COMMIT/RUN/ARM은
완료될 때까지 거부한다.

### 5.1 목표 왕복시간과 GPX Reg7.MTimer

`CTL12.TARGET_RANGE`는 5 ns 단위의 **요청 목표 왕복시간**이며 유일한
Runtime 원본이다. GPX `Reg7.MTimer`는 40 MHz 기준이라 25 ns 단위이므로
RTL이 다음과 같이 자동 계산한다.

```text
Reg7.MTimer                    = ceil(CTL12.TARGET_RANGE / 5)
실효 목표 왕복시간(5 ns ticks) = Reg7.MTimer * 5
```

레이저의 `stop_tdc`, GPX 캡처 시간 및 실제 Reg7 image는 모두 이 실효값을
사용한다. 따라서 CTL22로 입력한 Reg7의 다른 bit는 유지되지만 MTimer bit는
CTL12에서 파생한 값으로 덮어쓴다. 기본 요청값 288 ticks는 1,440 ns이고,
실효값은 290 ticks, 즉 1,450 ns이며 MTimer는 58이다.

13-bit MTimer 때문에 CTL12 최대값은 40,955 ticks, 즉 204.775 us이다. 이를
초과한 COMMIT은 error code `0x33`으로 거부된다.

### 5.2 요청 광학각 후보점 우선 정책

`CTL15.SHOT_INTERVAL`이 정한 요청 광학각 후보점은 다른 시간 조건보다
우선한다. 후보점에서 다음 두 준비 조건을 모두 만족할 때만 Shot을 발행한다.

- 레이저 실행기 준비: 이전 `fire_done` 응답, 실효 목표 왕복시간,
  `stop_tdc` 및 2 Processing-clock re-arm 구간 완료
- GPX 획득기 준비: 이전 Shot의 활성 Lane Drain/merge 완료와 Shot CDC 수용 가능

준비되지 않았다고 후보점을 지난 뒤 늦게 발사하지 않는다. 해당 column은
결측 Shot 열(Hole)로 남기고 `schedule_overrun`을 기록한다. Hole은 요청
각도 격자에는 있지만 그 시점에 측정하지 못한 column이며, 뒤 H-Line 번호가
당겨지지 않도록 같은 HSIZE의 빈 Line으로 DDR에 기록한다. 실제 모터 속도와
`fire_done`, GPX Drain 및 AXIS backpressure가 Runtime에 달라질 수 있으므로
이 검사는 정적인 시간 추정 대신 후보점마다 실제 ready를 검사한다.

- 진단 `PROC_FLAGS` index `0x10`, bit 5: sticky `schedule_overrun`
- 진단 `PROC_OVERRUN_COUNT` index `0x13`: 누적 누락 후보점 수
- IRQ source 5 `PROCESSING_WARNING`: 기존 ABI 이름이며, `schedule_overrun`은
  요청 광학각 후보점의 Shot 시간 계약 오류로 처리

## 6. AXI 및 VDMA 계약

- `m_axis_rise`, `m_axis_fall`: 합성 폭과 같은 `TDATA/TKEEP/TSTRB`
- `m_axis_monitor`: 고정 64-bit 관측 스트림
- `o_vdma_*_cfg_*`: Face 경계에서 원자적으로 승인되는
  `HSIZE/VSIZE/STRIDE/enable` 프로파일
- Runtime Return 수 변경은 현재 Face 도중에 적용되지 않고, 승인된 다음
  Face 경계에서 출력 Geometry와 함께 전환된다.

DDR Word 배열은 HTML Golden Vector와 Word 단위로 비교되고, PS reference
decoder는 DMA cache ownership 전환 후 H-Line과 Ethernet Viewer packet을
바이트 단위로 비교한다. 이 두 검증은 K0-9 Sign-off 항목이다.

## 7. 패키지 검증

다음 회귀는 패키지 재생성, 원본 파일 동등성, XGUI 유효성, v1/v2 Catalog
공존 및 대표 3개 OOC 합성을 연속 수행한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k010_ip_package.ps1
```

최종 PASS marker는 `LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS`이다.

## 8. 상세 문서

통합 데이터 흐름, CSR bit map, PACKED17 ABI와 검증 근거는 저장소의
`system_integration/v2_architecture` 문서를 기준으로 한다. 특히
`V2_UNIFIED_CSR_REGISTER_MAP.md`, `V2_CLOCK_EVENT_DATA_CONTRACT.md`,
`V2_PS_HLINE_ETHERNET_ABI_KO.md`, `V2_RTL_MAINTENANCE_GUIDE_KO.md` 및 K0
체크포인트 문서를 함께 참조한다.
