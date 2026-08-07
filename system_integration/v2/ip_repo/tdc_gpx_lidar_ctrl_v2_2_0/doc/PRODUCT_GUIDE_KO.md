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

## 5. AXI 및 VDMA 계약

- `m_axis_rise`, `m_axis_fall`: 합성 폭과 같은 `TDATA/TKEEP/TSTRB`
- `m_axis_monitor`: 고정 64-bit 관측 스트림
- `o_vdma_*_cfg_*`: Face 경계에서 원자적으로 승인되는
  `HSIZE/VSIZE/STRIDE/enable` 프로파일
- Runtime Return 수 변경은 현재 Face 도중에 적용되지 않고, 승인된 다음
  Face 경계에서 출력 Geometry와 함께 전환된다.

DDR Word 배열은 HTML Golden Vector와 Word 단위로 비교되고, PS reference
decoder는 DMA cache ownership 전환 후 H-Line과 Ethernet Viewer packet을
바이트 단위로 비교한다. 이 두 검증은 K0-9 Sign-off 항목이다.

## 6. 패키지 검증

다음 회귀는 패키지 재생성, 원본 파일 동등성, XGUI 유효성, v1/v2 Catalog
공존 및 대표 3개 OOC 합성을 연속 수행한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k010_ip_package.ps1
```

최종 PASS marker는 `LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS`이다.

## 7. 상세 문서

통합 데이터 흐름, CSR bit map, PACKED17 ABI와 검증 근거는 저장소의
`system_integration/v2_architecture` 문서를 기준으로 한다. 특히
`V2_UNIFIED_CSR_REGISTER_MAP.md`, `V2_CLOCK_EVENT_DATA_CONTRACT.md`,
`V2_PS_HLINE_ETHERNET_ABI_KO.md` 및 K0 체크포인트 문서를 함께 참조한다.
