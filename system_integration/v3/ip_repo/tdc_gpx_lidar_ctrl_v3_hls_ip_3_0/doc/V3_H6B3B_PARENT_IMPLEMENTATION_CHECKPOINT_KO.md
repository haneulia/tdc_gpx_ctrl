# V3 H6-B3B Parent 구현 체크포인트

## 1. 판정

2026-08-13 기준으로 4-Chip V3 Parent의 **보드 독립 구현 범위는 PASS**다.
32-bit와 64-bit Profile 모두 합성, 배치·배선, Bitstream 및 XSA 생성을 완료했다.
64-bit에서 발견된 TDC Reset 고팬아웃 경로는 1차 배선 결과로 판정한 뒤 필요한
경우에만 물리 FF 복제와 재배선을 수행하는 Sign-off 절차로 닫았다.

이 판정은 실물 보드 Sign-off가 아니다. 실제 TDC-GPX, DDR/cache API, Ethernet,
레이저 및 선택적 Echo LVDS 계측은 남아 있다.

## 2. 검증 대상

| 항목 | 계약 |
|---|---|
| Device | `xc7z020clg484-2` |
| V3 IP | `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0` |
| Topology | 4 Chip, STOP/chip 8, Return/STOP 7 |
| Slope | Rise `0011`, Fall `1100` |
| Clock | CSR 100 MHz, Processing/AXIS 150 MHz, TDC 200 MHz |
| Stream mode | `ASYNC` |
| Output | 합성 시 32 또는 64 bit, 128 bit 제외 |
| Echo | Receiver/Simulation 모두 비활성 |
| VDMA | Lane별 S2MM-only, 3 Frame Store, memory width 64 bit |
| DDR | Rise HP0, Fall HP1 |

## 3. 이번 수정의 이유

### 3.1 다중 Chip Shot-ready 경로

4개 TDC-GPX Lane의 ready 축약이 Shot payload 수락 경로까지 한 Cycle에 이어져
200 MHz 타이밍에 부담을 주었다. `lane_shot_ready_r`에 ready를 등록하여 경로를
분리했다. ready 상승은 한 TDC clock 늦어지지만 처리율 손실이나 Shot identity
변경은 없다.

등록된 이전 ready가 Run 해제 후 남아 새 Shot을 수락하지 않도록 최종 ready는 현재
`run/reset/config/register-read/dispatch` 상태를 다시 확인한다. 전용 TB는 Run을
내린 같은 Cycle에 `shot_ready=0`인지와 재활성화 후 정상 재무장을 검사한다.

### 3.2 Operation 상태 CDC

`scheduler_enable`은 `physical_fire_enable OR simulation_enable`의 파생 상태다.
같은 source FF를 독립 상태와 파생 상태의 두 synchronizer에 중복 연결하면 CDC-11
fan-out이 생긴다. 물리/시뮬레이션 독립 bit만 CSR clock으로 동기화하고, CSR 쪽에서
한 LUT의 OR로 파생 상태를 재구성했다. 이는 레이저 실시간 제어 경로가 아니라 읽기
전용 상태 경로이므로 조합 깊이와 물리 오차를 늘리지 않는다.

## 4. Parent 구조 검증

32/64-bit 프로젝트를 디스크에서 다시 열어 다음을 exact compare했다.

- V3 Generic, 4-Chip topology, Echo/OEN 정책
- PS FCLK/HP0/HP1 및 Clock Wizard 100→150 MHz
- V3 Rise/Fall AXIS와 각 VDMA S_AXIS의 동일 폭
- VDMA S2MM-only, MM2S disabled, 3 Frame Store
- 실제 생성 XCI의 frame count와 delay count 기능 활성
- Rise/Fall AXI4-to-AXI3-to-HP0/HP1 연결
- 통합 CSR, Rise VDMA, Fall VDMA 주소와 IRQ 연결
- 제거된 외부 VDMA profile bridge가 다시 생성되지 않음
- Echo Receiver 비활성 시 LVDS/STOP 포트가 Parent에 노출되지 않음

검증 표식은 두 폭 모두 다음 순서로 통과했다.

```text
LIDAR_V3_L0_VDMA_XCI_CONTRACT_PASS
LIDAR_V3_L0_PARENT_VALIDATE_PASS
LIDAR_V3_L0_PARENT_RUNNER_PASS
```

## 5. 구현 결과

### 5.1 200 MHz 재검토와 채택 RTL

기존 이진 인코딩의 최악 Setup 경로는 200 MHz TDC clock의 GPX bus PHY state
register에서 데이터 tri-state OLOGIC register까지였다. 2 LUT, 데이터 지연
`4.105 ns` 중 배선이 `3.412 ns`(83.1%)였고 routed WNS/WHS는
`+0.077/+0.017 ns`였다. 기능 PASS이지만 5 ns 주기의 1.54%뿐이므로 충분한
여유로 분류하지 않았다.

`tdc_gpx_bus_phy.s_state_r`만 one-hot 인코딩하도록 지정했다. 상태당 FF 사용량은
늘지만 28-bit DATA 방향 제어 IOB로 전파되는 상태 decode cone을 줄인다. 상태 전이,
TDC-GPX 외부 버스 파형, Runtime TDC-GPX 버스 읽기 타이밍
(`BUS_CLK_DIV`, `BUS_TICKS`)의 clock 단위 계약은 바꾸지 않는다.

| 후보 | TDC 200 MHz Setup/Hold | 판정 |
|---|---:|---|
| 기존 이진 FSM | `+0.077/+0.017 ns` | 기능 PASS, 여유 민감 |
| bus PHY one-hot만 적용 | `+0.133/+0.062 ns` | 채택 |
| one-hot + 응답 owner 변경 | `+0.075/+0.043 ns` | Setup 악화로 거부 |
| 응답 owner만 변경 | `+0.024/+0.075 ns` | Setup 악화로 거부 |
| 기존 RTL route directive sweep | 최대 `+0.033/+0.017 ns` | 기존보다 악화되어 거부 |
| 내부 IOB 제어 max-delay 강제 | 최대 `+0.061/+0.067 ns` | 3.75 ns에서 timing fail, 거부 |
| 모든 Profile에 Reset FF 복제 강제 | 32-bit Setup `+0.092 ns` | 일반 처리 경로 악화로 거부 |
| 1차 route에서 Reset `<0.100 ns`일 때만 복제 | 64-bit Setup `+0.033→+0.192 ns` | 채택 |

채택 후 최악 Setup은 `s_tick_r_reg[2]`에서 `s_d_tri_r_reg[1]/D`까지이며,
2 LUT4와 데이터 지연 `4.063 ns`로 구성된다. 이 중 배선이 `3.302 ns`(81.3%)다.
최악 Hold는 `s_wait_cap_r_reg[6]`에서 `s_wait_expired_r_reg/D`까지이고
`+0.062 ns`다. 상위 100개 200 MHz 경로 중 Setup `0.100 ns` 미만과 Hold
`0.020 ns` 미만은 모두 0개다.

현재 VT Parent는 `G_OEN_MODE=PULLUP_OR_NOT_CONNECTED`로 합성하며 OEN을 보드
외부 핀으로 내보내지 않는다. 따라서 아래 TDC 출력 152개와 GPX IOB Register
364개는 이 보드 Profile의 실제 핀 계약이며 OEN을 포함하지 않는다. 통합 IP의
`DYNAMIC_CONNECTED` OEN 상태기계와 200 MHz 합성은 V2 packaged OOC 회귀에서
검증했지만, OEN을 실제 PCB에 연결하는 다른 Parent Profile은 OEN 핀 XDC를 추가한
별도 배치·배선 Sign-off가 필요하다.

### 5.2 32-bit 대표 배치·배선

증적: `.work/v3_parent_signoff/260813_200mhz_adaptive_reset_w32_impl1/`

| Gate | 결과 |
|---|---:|
| 합성 Setup WNS | `+0.328 ns` |
| 합성 Hold WHS | `-0.001 ns`, 배치 전 추정 advisory |
| Routed 전체 Setup WNS / Hold WHS | `+0.003/+0.014 ns` |
| Routed 내부 TDC 200 MHz Setup/Hold | `+0.133/+0.062 ns` |
| Reset Setup / 일반 처리 Setup | `+0.188/+0.133 ns` |
| 선택형 Reset 물리 복제 | 미적용 |
| TDC 200 MHz Setup `<0.100 ns` / Hold `<0.020 ns` | 0 / 0 |
| TDC 출력 8 ns 예산 최악 slack | `+0.003 ns`, `o_tdc_stopdis[0]` |
| TDC 출력 8 ns 예산 `<0.100 ns` endpoint | 2 / 152 |
| 최악 TDC register-to-pad 지연 | `7.997 ns` |
| 최소 25 ns 핀 유지구간의 잔여 안정시간 | `17.003 ns` |
| Timing failing endpoint | 0 |
| Black box / Latch | 0 / 0 |
| 활성 CDC Critical | 0 |
| Bus skew violation | 0 |
| Methodology Critical / Blocking DRC | 0 / 0 |
| 서비스 핀 / GPX IOB Register | 171 / 364 |
| Bitstream | 4,045,708 bytes |
| XSA | 358,021 bytes, Bitstream 미포함 |

전체 WNS `+0.003 ns`는 내부 200 MHz register-to-register 경로가 아니다. 보수적으로
설정한 TDC 출력 register-to-pad 8 ns 예산에서 `STOPDIS[0]`가 사용하는 값이다.
Runtime TDC-GPX 버스 읽기 타이밍 (`BUS_CLK_DIV`, `BUS_TICKS`)은 최소 25 ns의
핀 유지구간을 보장하므로 최악 register-to-pad 지연 뒤에도 `17.003 ns`가 남는다.
따라서 timing constraint는 모두 PASS이고 물리 버스 운용 여유도 존재하지만, 8 ns
예산 자체가 민감하다는 advisory는 숨기지 않고 결과에 남긴다.

### 5.3 64-bit 배치·배선과 선택형 Reset 보정

증적: `.work/v3_parent_signoff/260813_200mhz_adaptive_reset_w64_impl1/`

| Gate | 결과 |
|---|---:|
| 합성 Setup WNS / Hold WHS | `+0.328 ns` / `-0.001 ns` advisory |
| 1차 route Reset Setup / 일반 처리 Setup | `+0.033/+0.253 ns` |
| 선택형 Reset 물리 복제 | 적용, fanout `109→최악 19` |
| 최종 전체 Setup WNS / Hold WHS | `+0.099/+0.014 ns` |
| 최종 TDC 200 MHz Setup/Hold | `+0.192/+0.060 ns` |
| 최종 Reset Setup / 일반 처리 Setup | `+0.192/+0.226 ns` |
| Reset Setup `<0.100 ns` / 일반 Setup `<0.100 ns` / Hold `<0.020 ns` | 0 / 0 / 0 |
| TDC 출력 8 ns 예산 최악 slack | `+0.151 ns`, `o_tdc_stopdis[1]` |
| 최악 TDC register-to-pad 지연 | `7.849 ns` |
| 최소 25 ns 핀 유지구간의 잔여 안정시간 | `17.151 ns` |
| Black box / Latch | 0 / 0 |
| 활성 CDC Critical | 0 |
| Bus skew / Methodology Critical / Blocking DRC | 0 / 0 / 0 |
| 서비스 핀 / GPX IOB Register | 171 / 364 |
| Bitstream | 4,045,708 bytes |
| XSA | 358,006 bytes, Bitstream 미포함 |

실제 XCI에서 V3 Rise/Fall AXIS와 VDMA S_AXIS가 모두 64 bit이고, VDMA Memory
Master와 PS HP Port는 64 bit임을 확인했다. 출력 폭은 TDC RTL 구조를 바꾸지 않지만
전체 배치 혼잡과 Reset 배선 위치에는 영향을 줄 수 있다. 실제 1차 route에서 일반
상태/데이터 경로는 `+0.253 ns`였지만, `rst_tdc`의 동기 Reset FF가 109개 부하로
퍼지는 경로가 `+0.033 ns`까지 좁아졌다.

Sign-off 흐름은 1차 route의 Reset 경로만 `0.100 ns` 미만일 때 pre-route
checkpoint로 돌아가 해당 Reset FF를 물리 복제한 후 재배선한다. RTL Reset 주기와
기능 latency는 바뀌지 않는다. 32-bit에서는 Reset 여유가 `+0.188 ns`라 보정하지
않았고, 64-bit에서만 적용해 Reset/일반 처리 경로를 모두 `0.100 ns` 이상으로
닫았다. 따라서 폭별 물리 차이를 숨기지 않으면서 불필요한 복제로 32-bit 일반
경로가 악화되는 것도 방지한다.

### 5.4 Runtime 버스 속도와 내부 200 MHz STA의 구분

`BUS_CLK_DIV`와 `BUS_TICKS`는 외부 RDN/WRN/CSN/ADR/DATA가 유지되는 Runtime
TDC-GPX 버스 읽기 타이밍만 느리게 하거나 빠르게 한다. 이 설정을 늘려도
`s_state_r`, `s_tick_r`, `s_d_tri_r`가 사용하는 200 MHz 내부 clock period는
계속 5 ns이므로 내부 STA WNS가 늘어나지 않는다.

PCB와 실제 TDC-GPX가 200 MHz PL bus/acquisition clock을 견디지 못하면
`i_tdc_clk`와 `G_TDC_CLK_MHZ`를 함께 150/125/100/50 MHz Profile로 낮추는 것이
올바른 대응이다. 이때 Processing/AXIS 150 MHz와의 관계가 달라지므로
`G_STREAM_CLK_MODE=ASYNC`를 유지하고 해당 주파수 조합의 CDC, IFIFO Drain 처리율,
레이저 목표 왕복시간 (`TARGET_RANGE_WINDOW_5NS`) 및 Shot 시간 계약을 다시
검증해야 한다.

## 6. CDC와 외부 Timing 판정

내부 clock endpoint는 모두 제약되며 unconstrained internal endpoint는 0이다.
외부 비동기 서비스 입력은 다음과 같이 별도 계약으로 관리한다.

- `io_tdc_d`: 112개 IOB capture FF. Runtime TDC-GPX 버스 읽기 타이밍
  (`BUS_CLK_DIV`, `BUS_TICKS`)이 보장하는 RDN 이후 25 ns 이상 안정창을 이용하므로
  일반 2-FF CDC로 잘못 분류하지 않고 endpoint 한정 CDC waiver를 적용한다.
- EF1/EF2/IRFLAG: 12개 비동기 level 입력. 명시적 첫 동기화 FF를 사용하며 해당
  포트에 한정한 methodology waiver를 적용한다.
- Encoder A/B/Z와 `fire_done`: Processing 영역의 명시적 동기화/저지연 계약을
  유지한다.

waiver 수는 GPX data 112개와 GPX 상태 12개로 고정 검사한다. 숫자만 일괄 무시하지
않으며 endpoint 수나 인스턴스 경로가 바뀌면 Sign-off가 실패한다.

## 7. 테스트 소유권

| 검증 자산 | 이번에 보강한 계약 |
|---|---|
| `tb_lidar_gpx_acquisition_coordinator.vhd` | 등록 ready, Run 즉시 차단, 재무장, Shot fanout/merge |
| `tb_lidar_operation_subsystem.vhd` | Simulation/Physical 상호배제와 CSR 파생 scheduler 상태 |
| `validate_v3_l0_parent.tcl` | 저장 BD뿐 아니라 생성 VDMA XCI의 IRQ counter 계약 |
| `run_v3_l0_parent_signoff.tcl` | 핀, IOB, timing endpoint, CDC, DRC, route, Bit/XSA |

관련 RTL 변경 시 위 단위 TB를 먼저 실행한 뒤 V3 IP package, 32/64 Parent
재개방, 32-bit 대표 구현 순으로 진행한다.

최종 재실행 증적:

| 세션 | 결과 |
|---|---|
| `260813_h6b3b_final_operation_v2_operation` | 50/150/200 MHz 기능 PASS, 150 MHz WNS `+3.940 ns`, 200 MHz WNS `+2.064 ns`, CDC Critical 0, latch 0 |
| `260813_h6b3b_final_coordinator_v2_gpx_acquisition_coordinator` | event merge 및 150/200 MHz coordinator 기능 PASS |
| `260813_h6b3b_shot_ready_pipe_rerun_v2_gpx_acquisition_coordinator` | 150 MHz WNS `+1.347 ns`, 200 MHz WNS `+0.578 ns`, latch 0 |
| `260813_111811_k010_ip_package` | one-hot 변경 후 V2 원본/패키지 88 RTL 재동기화, async32(150/200), async128(200/150, Echo Receiver 비활성), sync64(150/150) OOC PASS, 새 경고 ID 없음 |
| `260813_200mhz_onehot_final_bus_v2_gpx_bus` | 200/150 MHz dynamic/pullup 4 Profile 기능 PASS, 200 MHz OOC WNS `+1.425 ns`, latch 0 |
| `260813_200mhz_onehot_final_clear_v2_gpx_clear_status` | GPX init/register owner, fault injection, full chip controller CLEAR_STATUS PASS |
| `260813_200mhz_onehot_final_lane_v2_gpx_acquisition_lane` | 150/200 MHz acquisition lane 기능 PASS |
| `260813_200mhz_adaptive_reset_w32_impl1` | 32-bit 4-Chip route, Reset 보정 미적용, TDC Setup/Hold `+0.133/+0.062 ns`, Bit/XSA PASS |
| `260813_200mhz_adaptive_reset_w64_impl1` | 64-bit 4-Chip Reset 선택 보정, TDC Setup/Hold `+0.192/+0.060 ns`, Bit/XSA PASS |

마지막 V2 패키지 회귀 뒤 `tdc_gpx_bus_phy.vhd`의 원본, V2 패키지 및 V3 패키지
사본은 동일한 SHA-256
`D8479290F293831973BFE1668A9C47D7AC3DA8D516A2A46D96FE2BA8DEA04D56`로
확인했다. 따라서 200 MHz one-hot 변경이 한 배포 IP에만 누락되는 이중 소스
상태가 없다.

## 8. 남은 실물 보드 Gate

1. TDC-GPX 40 MHz 기준 클럭과 Reg7.MTimer 25 ns 단위 변환 계측
2. Runtime TDC-GPX 버스 읽기 타이밍 (`BUS_CLK_DIV`, `BUS_TICKS`) sweep
3. 실제 IFIFO/EF Drain과 28-bit I-Mode Word, 하위 17-bit 거리 Hit 비교
4. VDMA DDR dump와 Golden Word exact compare
5. FreeRTOS/PetaLinux cache invalidate/flush API 및 Frame Store 소유권
6. H-Line/Ethernet payload 장시간 전송
7. 레이저 안전 허가, `fire_done`과 측정 시작 기준시점 (T0) 계측
8. Echo Receiver 활성 Profile의 LVDS-to-STOP 저지연 계측

따라서 다음 단계는 새 기능 추가가 아니라 위 실물 증거를 수집하는 H6-B4 보드
Sign-off다.
