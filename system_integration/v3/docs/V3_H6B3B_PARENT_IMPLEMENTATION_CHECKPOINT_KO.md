# V3 H6-B3B Parent 구현 체크포인트

## 1. 판정

2026-08-13 기준으로 4-Chip V3 Parent의 **보드 독립 구현 범위는 PASS**다.
32-bit Profile은 합성, 배치·배선, Bitstream 및 XSA 생성을 완료했고, 64-bit
Profile은 같은 Block Design의 저장 계약과 실제 생성 XCI, 전체 합성을 통과했다.

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

### 5.1 32-bit 대표 배치·배선

증적: `.work/v3_parent_signoff/260813_h6b3b_v3_parent_w32_impl3/`

| Gate | 결과 |
|---|---:|
| 합성 Setup WNS | `+0.328 ns` |
| 합성 Hold WHS | `+0.035 ns` |
| 배치·배선 Setup WNS | `+0.077 ns` |
| 배치·배선 Hold WHS | `+0.017 ns` |
| Timing failing endpoint | 0 |
| Black box / Latch | 0 / 0 |
| 활성 CDC Critical | 0 |
| Bus skew violation | 0 |
| Methodology Critical / Blocking DRC | 0 / 0 |
| 완전 배선 Net / Routing error | 66,004 / 0 |
| 서비스 핀 / GPX IOB Register | 171 / 364 |
| Bitstream | 4,045,708 bytes |
| XSA | 358,023 bytes, Bitstream 미포함 |

최악 Setup 경로는 200 MHz TDC clock의 GPX bus PHY state register에서 데이터
tri-state OLOGIC register까지다. 2 LUT, 데이터 지연 `4.105 ns` 중 배선이
`3.412 ns`(83.1%)다. 타이밍은 충족하지만 `+0.077 ns`는 얇은 여유이므로 충분한
마진으로 표현하지 않는다. PCB 계측과 빌드 재현성을 확인한 뒤 필요하면 bus PHY의
출력 decode를 IOB 인접 register 단계로 더 분리하는 최적화를 검토한다.

### 5.2 64-bit 전체 합성

증적: `.work/v3_parent_signoff/260813_h6b3b_v3_parent_w64_synth1/`

| Gate | 결과 |
|---|---:|
| 합성 Setup WNS / Hold WHS | `+0.328 ns` / `+0.035 ns` |
| Black box / Latch | 0 / 0 |
| 활성 CDC Critical | 0 |
| Bus skew / Methodology Critical / Blocking DRC | 0 / 0 / 0 |
| 서비스 핀 / GPX IOB Register | 171 / 364 |

실제 XCI에서 V3 Rise/Fall AXIS와 VDMA S_AXIS가 모두 64 bit이고, VDMA Memory
Master와 PS HP Port는 64 bit임을 확인했다. 64-bit 배치·배선은 32-bit와 동일한
canonical Word/packer 구조를 사용하지만 Release 전에 별도 실행하는 것이 좋다.

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
| `260813_052023_k010_ip_package` | V2 원본/패키지 88 RTL 동기화, async32(150/200), async128(200/150, Echo Receiver 비활성), sync64(150/150) OOC PASS, 새 경고 ID 없음 |

마지막 V2 패키지 회귀 뒤 `lidar_operation_subsystem.vhd`,
`lidar_gpx_acquisition_coordinator.vhd`, 테스트벤치 가이드의 원본과 패키지
사본은 각각 SHA-256 exact match로 확인했다. 따라서 H6-B3B에서 수정한 공용 RTL이
V3에만 남고 V2 배포 IP에는 누락되는 이중 소스 상태가 없다.

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
