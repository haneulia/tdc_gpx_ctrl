# TDC-GPX LiDAR V2 Stage L0 Parent 사용 및 Sign-off 가이드

## 1. 목적과 현재 판정

이 폴더는 `tdc_gpx_lidar_ctrl_v2:2.0` IP를 Zynq-7000
`xc7z020clg484-2` Parent에 실제로 배치하고, Rise/Fall AXI VDMA와 PS7 HP
포트까지 연결한 재생성 가능한 Vivado 프로젝트를 소유한다.

2026-08-09 기준 판정은 다음과 같다.

- Parent Block Design, 합성, 배치·배선, DRC와 비트스트림 생성: **PASS**
- 물리 PCB의 TDC-GPX, 레이저, DDR cache 및 Ethernet 장시간 운용: **미검증**
- 따라서 이 결과는 **Parent 구현 Sign-off**이며 최종 보드 Release Sign-off는 아니다.

GUI 프로젝트:

```text
C:\Projects\my_sp\ALINX\Logic\project_4_lidar_v2_l0\project_4_lidar_v2_l0.xpr
```

## 2. Parent 데이터 흐름

```mermaid
flowchart LR
    PS["Zynq PS7"] -->|"M_AXI_GP0 / 100 MHz"| CSR["통합 CSR"]
    ENC["Encoder A/B/Z"] --> LIDAR["tdc_gpx_lidar_ctrl_v2"]
    GPX["TDC-GPX 2 chips"] <--> LIDAR
    LIDAR -->|"Rise AXIS 32-bit / 150 MHz"| RV["Rise AXI VDMA"]
    LIDAR -->|"Fall AXIS 32-bit / 150 MHz"| FV["Fall AXI VDMA"]
    RV -->|"AXI4→AXI3 64-bit"| HP0["PS7 HP0 / DDR"]
    FV -->|"AXI4→AXI3 64-bit"| HP1["PS7 HP1 / DDR"]
    LIDAR -->|"IRQ[0]"| IRQ["PS IRQ_F2P"]
    RV -->|"IRQ[1]"| IRQ
    FV -->|"IRQ[2]"| IRQ
```

VDMA의 입력 AXIS 폭은 합성 Generic `G_OUTPUT_WIDTH`와 같다. 현재 GUI
프로젝트는 32-bit이고, 각 VDMA의 DDR측 Master는 PS7 HP 포트 계약에 맞춰
64-bit로 변환한다. Rise는 HP0, Fall은 HP1을 사용하며 각 VDMA는 3개의 Frame
Buffer를 갖는다.

## 3. 고정 Build Profile

| 항목 | 현재 값 | 의미 |
|---|---:|---|
| FPGA | `xc7z020clg484-2` | ALINX AC7021B 계열 Parent 대상 |
| CSR clock | 100 MHz | PS FCLK0, AXI4-Lite 및 VDMA 제어 |
| Processing clock | 150 MHz | PS FCLK1 100 MHz를 Clock Wizard로 변환 |
| TDC clock | 200 MHz | PS FCLK2, TDC-GPX 버스/수집 도메인 |
| `G_STREAM_CLK_MODE` | `ASYNC` | Processing/TDC가 서로 다른 물리 clock |
| `G_OUTPUT_WIDTH` | 32 bit | VDMA S_AXIS 폭과 동일 |
| `G_NUM_CHIPS` | 2 | TDC0/TDC1 활성 |
| `G_STOPS_PER_CHIP` | 8 | Chip당 STOP 8개 |
| `G_MAX_RETURNS_PER_STOP` | 7 | STOP당 최대 Return 7개 |
| Rising capability | `0011` | 두 활성 Chip 모두 Rising |
| Falling capability | `0000` | 이 Parent에서는 Falling 비활성 |
| Echo receiver | 비활성 | 외부 TDC-GPX 수집 경로 사용 |
| OEN mode | `PULLUP_OR_NOT_CONNECTED` | PCB pull-up 또는 미연결 Parent 정책 |

## 4. 89개 PL 서비스 핀

PS DDR/FIXED_IO를 제외한 LiDAR 서비스 핀은 정확히 89개다.

| 그룹 | 수 | 포트 |
|---|---:|---|
| Encoder | 3 | `i_enc_a`, `i_enc_b`, `i_enc_z` |
| Laser feedback | 1 | `i_fire_done` |
| GPX 상태 입력 | 6 | `i_tdc_ef1[1:0]`, `i_tdc_ef2[1:0]`, `i_tdc_irflag[1:0]` |
| GPX 양방향 데이터 | 56 | `io_tdc_d[55:0]` |
| GPX 주소 | 8 | `o_tdc_adr[7:0]` |
| GPX Chip 제어 | 12 | Chip별 `CSN/RDN/WRN/STOPDIS/ALUTRIGGER/PURESN` |
| Laser/TDC 사건 출력 | 3 | `o_fire_pulse`, `o_start_tdc`, `o_stop_tdc` |

`OEN/LF/ERRFLAG`는 IP에서 제거하지 않았다. 이 Parent만 다음 정책을 사용한다.

- `o_tdc_oen`: `G_OEN_MODE=PULLUP_OR_NOT_CONNECTED`이므로 외부 핀으로 만들지 않는다.
- `i_tdc_lf1`, `i_tdc_lf2`: Parent 내부 상수 `0`에 연결한다.
- `i_tdc_errflag`: Parent 내부 상수 `0`에 연결한다.
- 동적 OEN과 LF/ERRFLAG 배선이 필요한 다른 PCB는 IP Generic과 Parent 배선/XDC를
  바꿔야 하며, IP 공개 포트는 그대로 재사용할 수 있다.

## 5. VDMA Profile 적용 절차

통합 IP가 Rise/Fall별 `HSIZE/VSIZE/STRIDE/enable` 후보값을 Processing
도메인에서 출력한다. `l0_vdma_profile_bridge`가 49-bit payload를 CSR 100 MHz
도메인으로 원자적으로 전달한다.

1. PS가 pending profile을 읽는다.
2. 해당 VDMA가 정지된 것을 확인한다.
3. VDMA `HSIZE`, `VSIZE`, `STRIDE`와 Frame Buffer 주소를 기록한다.
4. PS GPIO의 slope별 ACK bit를 한 번 pulse한다.
5. Bridge와 통합 IP가 ACK를 확인한 뒤 다음 Face부터 새 계약을 사용한다.

오래된 ACK가 HIGH인 상태는 새 요청의 승인으로 인정하지 않는다. Reset 또는
profile 변경 중 VDMA를 계속 실행하면 안 된다.

최신 브리지 회귀 `260809_190428_l0_vdma_profile_bridge`는 49-bit payload
원자성, 지연 ACK, 오래된 HIGH ACK 거부 및 Rise/Fall 요청 순서를 검증했다.

## 6. Reset과 CDC 계약

세 clock 전체를 `set_clock_groups -asynchronous`로 묶지 않는다. 그렇게 하면
설정 handshake, 진단 mailbox, XPM FIFO가 소유한 물리적 max-delay와 bus-skew
보호까지 사라진다.

| 교차 | 보호 방식 |
|---|---|
| 1-bit 상태/요청 | 명시적 2FF 이상 동기화, metastability 첫 단계만 false path |
| Encoder A/B/Z | 4단 동기화기 bit 0만 false path |
| 215-bit 설정 payload | 4-phase request/ACK + 5 ns datapath-only max delay |
| 16x28-bit GPX image | acknowledged activation + 5 ns datapath-only max delay |
| 8/33-bit 진단 mailbox | toggle handshake + 5 ns max delay + 5 ns bus skew |
| TDC→Processing event | `xpm_fifo_async`, 양쪽 Reset busy와 stale payload 폐기 |
| GPX D[27:0]/Chip | RDN 선행 후 IOB capture, 최소 25 ns Runtime 버스 유지시간 |

GPX 데이터 핀 56개에 대한 false path는 **포트에서 첫 IOB 입력 FF까지**로만
제한된다. 일반적인 2FF 동기화 데이터라고 주장하는 것이 아니다. 외부 GPX가
RDN 뒤 데이터를 안정화하고, Runtime TDC-GPX 버스 읽기 타이밍
(`BUS_CLK_DIV/BUS_TICKS`)이 최소 25 ns를 제공한다는 비동기 병렬 버스 계약이다.

별도 Reset 회귀는 200 MHz Source와 150 MHz Destination에서 다음을 검증한다.

- Source 또는 Destination Reset 중 `ready/valid` 누출 금지
- Reset 전에 FIFO에 보류된 payload 폐기
- Reset 해제 후 처음 수신한 payload가 새 값과 정확히 일치

최신 회귀 세션은 `260809190043_v2_stream_gateway_reset`이다.

## 7. 타이밍 계약과 최종 수치

TDC 제어 출력은 반환 clock이 없는 외부 장치 인터페이스다. false path로 숨기지
않고 register-to-pad에 8 ns max-delay를 적용한다. 실제 TDC-GPX 버스 상태는
최소 25 ns 유지되므로 8 ns 안에 핀에 도달하면 최소 17 ns의 안정 구간이 남는다.

최종 GUI Release 세션 `260809_l0_parent_release_impl04` 결과:

| 항목 | 결과 |
|---|---:|
| Synthesis WNS/WHS | `+0.379 / +0.035 ns` |
| Route WNS/WHS | `+0.174 / +0.018 ns` |
| CSR 100 MHz 내부 WNS | `+0.77 ns` |
| Processing 150 MHz 내부 WNS | `+0.66 ns` |
| TDC 200 MHz 내부 WNS | `+0.23 ns` |
| 최악 TDC 출력 | `o_tdc_stopdis[0]`, `7.826 ns` |
| 최소 핀 안정 여유 | `25 - 7.826 = 17.174 ns` |
| Active critical CDC | 0 |
| Bus-skew 위반 | 0 |
| Blocking DRC | 0 |
| Critical Warning | 0 |
| 비트스트림 | 4,045,708 bytes |

패키지 최종 세션 `260809_185237_k010_ip_package`는 32-bit 150/200 MHz,
128-bit 200/150 MHz Echo 비활성, 64-bit 150/150 MHz SYNC의 세 OOC profile을
`Critical Warning 0 / Error 0`으로 통과했다. 이 실행은 최종 91개
package 자산의 source/XGUI/guide 동기화까지 같은 runner에서 검사했다.
또한 초기 IP 추론 및 OOC 최적화에서 허용한 Warning ID와 최대 수를
`WARNING_AUDIT.txt`에 기록하며, 새 ID 또는 발생 수 증가 시 Sign-off를 거부한다.

강화된 Parent runner와 격리된 Vivado 사용자 캐시의 재현성은
`260809_l0_parent_isolated_cache_synth05`에서 다시 확인했다. 이 세션도
Synthesis WNS/WHS `+0.379/+0.035 ns`, Critical Warning 0, Error 0이다.

## 8. 재생성과 Sign-off 실행

프로젝트를 다시 만들 때 기존 대상 폴더 삭제는 `-Recreate`를 명시한 경우에만
허용된다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/parent_l0/run_v2_l0_parent.ps1 `
  -OutputWidth 32 -Recreate
```

합성만 확인:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/parent_l0/run_v2_l0_parent_signoff.ps1 `
  -Mode SYNTH -SessionTag <unique_session>
```

배치·배선과 비트스트림까지 확인:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/parent_l0/run_v2_l0_parent_signoff.ps1 `
  -Mode IMPL -SessionTag <unique_session>
```

## 9. 아직 남은 보드 Sign-off

다음 항목은 현재 PASS에 포함되지 않는다.

1. 실제 PCB에서 40 MHz GPX 기준 clock과 TDC-GPX Register readback 확인
2. TDC0/TDC1 56-bit 데이터 버스 setup/hold 및 장기 IFIFO Drain
3. PS가 VDMA profile을 적용한 뒤 HP0/HP1 DDR byte를 읽는 실측
4. FreeRTOS/PetaLinux DMA cache invalidate/ownership API 검증
5. Rise/Fall Frame Buffer 주소, overflow, 장기 backpressure 검증
6. 실제 레이저 안전 인터록과 `fire_done` 응답시간 검증
7. DDR H-Line을 Viewer Ethernet ABI로 전송하는 지속 처리량 검증

TDC 200 MHz 내부 WNS `+0.23 ns`와 전체 Hold `+0.018 ns`는 PASS지만 여유가
작다. 기능 변경 전후에는 Parent 구현을 반드시 반복하고, 필요하면 해당 경로의
배치 또는 파이프라인 최적화를 별도 단계로 수행한다.
