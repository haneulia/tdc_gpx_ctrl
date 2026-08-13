# V3 4-Chip Parent 프로젝트 사용 가이드

## 1. 목적

이 폴더는 `tdc_gpx_lidar_ctrl_v3` 통합 IP를 Zynq-7000
`xc7z020clg484-2`에 연결하여 Vivado GUI, 합성, 배치·배선 및 Bitstream 생성까지
재현하는 보드 상위(Parent) 설계를 소유한다. 원본 `project_4`는 수정하지 않으며,
생성 프로젝트는 저장소의 `.work/` 아래에 둔다.

이 단계의 PASS는 핀 배치와 내부 타이밍을 포함한 **보드 독립 구현 완료**다. 실제
TDC-GPX 40 MHz 기준 클럭, 외부 버스 파형, DDR cache 동기화, Ethernet 전송과
레이저 안전 동작은 실물 보드에서 별도로 확인해야 한다.

## 2. 고정 빌드 Profile

| 항목 | 값 |
|---|---|
| FPGA | `xc7z020clg484-2` |
| TDC-GPX Chip | 4개 |
| STOP | Chip당 8개 |
| 물리 최대 Return | STOP당 7개 |
| Slope capability | Rise `0011`, Fall `1100` |
| Echo Receiver | 비활성, LVDS 포트 미생성 |
| OEN 정책 | `PULLUP_OR_NOT_CONNECTED` |
| AXI4-Stream 폭 | 합성 시 32 또는 64 bit |
| VDMA | Rise/Fall 각각 S2MM 전용, 3 Frame Store |
| DDR 경로 | Rise→HP0, Fall→HP1, 각 64-bit AXI3 |

`G_OUTPUT_WIDTH`는 Runtime 설정이 아니다. 32-bit 프로젝트는 V3 AXIS와 VDMA
S_AXIS가 모두 32 bit이고, 64-bit 프로젝트는 양쪽 모두 64 bit다. VDMA의 DDR
Master와 Zynq-7000 HP Port는 두 Profile 모두 64 bit다.

## 3. Clock과 Reset

```text
PS FCLK0 100 MHz
  └─ AXI4-Lite CSR, AXI control interconnect, 두 VDMA Register

PS FCLK1 100 MHz
  └─ Clock Wizard 150 MHz
       ├─ V3 Processing/AXIS
       ├─ Rise/Fall VDMA S_AXIS 및 S2MM Master
       └─ PS HP0/HP1 clock

PS FCLK2 200 MHz
  └─ V3 TDC-GPX bus/IFIFO acquisition
```

세 Reset은 각각의 clock에서 `proc_sys_reset`으로 동기 해제된다. 150 MHz와
200 MHz는 다른 물리 clock이므로 V3는 `G_STREAM_CLK_MODE=ASYNC`로 생성한다.

## 4. 데이터와 제어 경로

```text
TDC-GPX 4 Chip I-Mode Raw28
  -> TDC 200 MHz IFIFO Drain
  -> 비동기 FIFO
  -> HLS H1~H4, Processing 150 MHz
  -> canonical 32-bit Word
  -> RTL AXIS packer 32/64 bit
  ├─ Rise AXIS -> Rise VDMA -> AXI4-to-AXI3 -> PS HP0 -> DDR
  └─ Fall AXIS -> Fall VDMA -> AXI4-to-AXI3 -> PS HP1 -> DDR
```

Zynq-7000 HP Port가 AXI3이고 AXI VDMA의 Memory Master가 AXI4이므로 각 Lane에
명시적인 AXI Protocol Converter를 둔다. AXI Interconnect/SmartConnect도 변환을
내부 삽입할 수 있지만, 현재 구조는 Rise/Fall의 1:1 경로와 변환 책임을 GUI에서
직접 확인할 수 있게 한다.

IRQ는 `IRQ_F2P[0]=통합 IP`, `[1]=Rise VDMA`, `[2]=Fall VDMA`이며 나머지는 0이다.
VDMA XCI에는 S2MM frame count와 delay count 기능이 활성화되어 Face 완료 IRQ
계약을 유지한다.

## 5. 외부 핀 정책

현재 Parent가 XDC로 고정하는 사용자 PL 서비스 핀은 171개다.

| 묶음 | 수 | 의미 |
|---|---:|---|
| `io_tdc_d[111:0]` | 112 | 4 Chip × 28-bit 양방향 데이터 |
| `o_tdc_adr[15:0]` | 16 | 4 Chip × 4-bit 주소 |
| CSN/RDN/WRN/STOPDIS/ALUTrigger/PURESN | 24 | 신호별 4개 |
| EF1/EF2/IRFLAG | 12 | 입력 상태 신호별 4개 |
| Encoder A/B/Z와 `fire_done` | 4 | 외부 실시간 입력 |
| `fire_pulse/start_tdc/stop_tdc` | 3 | 외부 실시간 출력 |

통합 IP의 OEN/LF/ERRFLAG 포트 자체는 제거하지 않았다. 이 Parent Profile에서만
OEN은 외부 핀으로 내보내지 않고, LF1/LF2/ERRFLAG는 비활성값 0에 묶는다. 다른
PCB가 이 신호를 배선하면 새 Parent Profile에서 해당 포트를 외부화하고 XDC와
진단 정책을 함께 추가해야 한다.

## 6. 프로젝트 생성과 GUI 확인

```powershell
# 32-bit 프로젝트 신규 생성
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/parent_l0/run_v3_l0_parent.ps1 `
  -OutputWidth 32 -Recreate

# 64-bit 프로젝트 신규 생성
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/parent_l0/run_v3_l0_parent.ps1 `
  -OutputWidth 64 -Recreate
```

GUI에서 여는 파일:

- 32-bit: `.work/v3_parent_l0/w32/project_4_lidar_v3_l0.xpr`
- 64-bit: `.work/v3_parent_l0/w64/project_4_lidar_v3_l0.xpr`
- Block Design: `design_1_lidar_ctrl_v3.bd`

이미 생성된 프로젝트의 저장 상태만 다시 검사할 때는 삭제·재생성하지 않는다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/parent_l0/run_v3_l0_parent.ps1 `
  -OutputWidth 32 -ValidateOnly
```

## 7. 합성 및 구현

```powershell
# 빠른 구조·합성 Gate
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/parent_l0/run_v3_l0_parent_signoff.ps1 `
  -ProjectRoot ./.work/v3_parent_l0/w64 -Mode SYNTH `
  -SessionTag <unique_tag>

# 배치·배선, Bitstream와 XSA (32/64-bit 모두 동일 명령)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/parent_l0/run_v3_l0_parent_signoff.ps1 `
  -ProjectRoot ./.work/v3_parent_l0/w32 -Mode IMPL `
  -SessionTag <unique_tag>
```

직접 배치·배선 흐름은 `.bit`와 고정 Hardware XSA를 나란히 생성한다. Vivado의
`write_hw_platform -include_bit`는 project run database 밖에서 직접 생성한
Bitstream을 찾지 못하므로 XSA에는 Bitstream을 넣지 않는다. PS 빌드는 XSA를,
FPGA programming은 같은 결과 폴더의 `.bit`를 사용한다.

## 8. PASS 판정

재개방 검증은 다음 표식이 모두 있어야 한다.

- `LIDAR_V3_L0_VDMA_XCI_CONTRACT_PASS`
- `LIDAR_V3_L0_PARENT_VALIDATE_PASS`
- `LIDAR_V3_L0_PARENT_RUNNER_PASS`

구현 Sign-off는 추가로 black box/latch 0, 내부 unconstrained endpoint 0, 활성 CDC
Critical 0, bus skew violation 0, blocking DRC 0, Setup/Hold 양수, 서비스 핀 171개와
GPX IOB Register 364개를 확인한다. 내부 TDC 200 MHz 경로는 더 엄격하게 Reset
Setup과 일반 상태/데이터 Setup이 각각 `0.100 ns` 이상이어야 PASS한다.

200 MHz 결과는 전체 WNS 한 값으로만 판정하지 않는다. Sign-off summary와 전용
리포트에서 다음 계약을 분리해 확인한다.

| 구분 | Summary/리포트 | 의미 |
|---|---|---|
| 내부 200 MHz Setup/Hold | `route_tdc_200mhz_wns_ns`, `route_tdc_200mhz_whs_ns` | `clk_fpga_2` register-to-register 경로 |
| 내부 민감 경로 수 | `route_tdc_setup_lt_0p100_count`, `route_tdc_hold_lt_0p020_count` | PASS보다 여유가 얇은 경로의 조기 경보 |
| 1차 Reset/일반 Setup | `route_initial_tdc_reset_wns_ns`, `route_initial_tdc_regular_wns_ns` | 첫 route에서 Reset 분배와 상태/데이터 경로를 분리한 여유 |
| Reset 물리 보정 | `route_reset_replication_applied` | Reset Setup이 `0.100 ns` 미만일 때만 FF 복제 후 재배선 |
| 최종 Reset/일반 Setup | `route_tdc_reset_wns_ns`, `route_tdc_regular_wns_ns` | 선택 보정 뒤 두 경로 종류의 최종 여유 |
| 최종 Reset fanout | `route_tdc_reset_max_fanout` | 최악 Reset 경로가 사용하는 복제 후 부하 수 |
| 외부 TDC 출력 8 ns 예산 | `route_tdc_output_budget_slack_ns` | TDC register-to-pad 보수적 제한 |
| 최악 출력 실제 지연 | `route_tdc_output_data_path_delay_ns` | OBUF/OBUFT를 포함한 register-to-pad 지연 |
| 최소 핀 안정시간 | `route_tdc_output_protocol_stable_margin_ns` | 최소 25 ns Runtime 버스 유지구간에서 출력 지연을 뺀 값 |

전용 파일은 `post_route_tdc_200mhz_setup_top100.rpt`,
`post_route_tdc_200mhz_hold_top100.rpt`,
`post_route_tdc_output_budget_top152.rpt`다. `BUS_CLK_DIV`와 `BUS_TICKS`는
Runtime TDC-GPX 버스 읽기 타이밍을 조절할 뿐 내부 200 MHz STA period를 바꾸지
않는다. 내부 여유를 늘리려면 `i_tdc_clk`와 `G_TDC_CLK_MHZ`가 일치하는 더 낮은
주파수 Profile을 합성하고 해당 비동기 clock 조합을 다시 검증해야 한다.

선택형 Reset 보정은 RTL 동작이나 Reset latency를 바꾸지 않는다. 구현 흐름이
1차 route 전에 `pre_route.dcp`를 저장하고, 1차 route의 TDC Reset Setup이
`0.100 ns` 미만일 때만 `rst_tdc` 동기 Reset FF를 물리 복제한 뒤 다시 route한다.
일반 상태/데이터 경로가 병목인 경우에는 Reset 복제를 적용하지 않는다.

## 9. 실물 보드에서 남은 Gate

1. TDC-GPX 기준 클럭은 반드시 40 MHz로 공급하고 Reg7.MTimer의 25 ns 단위 계약을
   계측한다.
2. Runtime TDC-GPX 버스 읽기 타이밍 (`BUS_CLK_DIV`, `BUS_TICKS`)별 RDN/CSN/Data
   setup/hold를 Logic Analyzer로 확인한다.
3. 실제 VDMA DDR Frame과 Golden Word를 비교하고 FreeRTOS/PetaLinux DMA cache
   동기화 API를 검증한다.
4. H-Line/Ethernet payload, IRQ latency, Frame Store 소유권을 장시간 운용한다.
5. 실제 레이저 허가 회로와 `fire_done`에서 측정 시작 기준시점 (T0)까지를 확인한다.
6. Echo Receiver를 활성화하는 PCB Profile에서는 LVDS 핀과 초저지연 STOP 경로를
   별도로 구현 Sign-off한다.
