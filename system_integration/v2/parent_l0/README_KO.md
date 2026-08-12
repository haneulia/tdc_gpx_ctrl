# TDC-GPX LiDAR V2 Stage L0 Parent 사용 및 Sign-off 가이드

## 1. 목적과 현재 판정

이 폴더는 `tdc_gpx_lidar_ctrl_v2:2.0` IP를 Zynq-7000
`xc7z020clg484-2` Parent에 실제로 배치하고, Rise/Fall AXI VDMA와 PS7 HP
포트까지 연결한 재생성 가능한 Vivado 프로젝트를 소유한다.

2026-08-09 4-chip Parent 최종 구현 판정은 다음과 같다.

- Parent Block Design 생성과 재개방 계약 검사: **PASS**
- 4-chip 합성, 배치·배선, DRC와 비트스트림 생성: **PASS**
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
    GPX["TDC-GPX 4 chips"] <--> LIDAR
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

V3 PS adapter를 사용할 때 3개 Frame Buffer는 최소값이며, AXI VDMA는
`MM2S=0`, `S2MM=1`인 전용 인스턴스여야 한다. Adapter는 AMD 드라이버의 S2MM
완료 임계값을 `1 Face Frame`으로 설정한다. Vivado 2025.2.1 AXI VDMA 6.3의 내부
Model Parameter `C_ENABLE_DEBUG_INFO_14/15`는 생성 XCI에서 모두 1이다. 이 둘은
Block Design에서 편집하는 `CONFIG`가 아니므로 Parent Tcl에서 직접 설정하지 않는다.
재현 검사는 `check_vdma_frame_irq_contract.tcl`이 담당한다.
VDMA `EnableFrameCounter=0`은 S2MM이 Face마다 멈추지 않고 계속 순환한다는 뜻이며,
별도로 설정한 `1 Face Frame` 완료 인터럽트 임계값은 그대로 유지된다.

Rise/Fall VDMA의 메모리 Master는 AXI4이고 Zynq-7000 HP0/HP1은 AXI3이므로
각 1:1 경로에 전용 `AXI Protocol Converter`를 둔다. AXI Interconnect와
SmartConnect도 AXI4→AXI3 변환이 가능하지만, 현재처럼 Master 하나와 HP 포트
하나가 직접 대응할 때는 중재·라우팅 기능이 불필요하다. 여러 DMA Master가 한
HP 포트를 공유하게 되면 그때 SmartConnect로 전환하는 것이 합리적이다. 반면
`axi_control_interconnect`는 PS GP0 하나를 통합 CSR과 두 VDMA AXI-Lite 제어
포트로 분배하기 위해 계속 사용한다.

## 3. 고정 Build Profile

| 항목 | 현재 값 | 의미 |
|---|---:|---|
| FPGA | `xc7z020clg484-2` | ALINX AC7021B 계열 Parent 대상 |
| CSR clock | 100 MHz | PS FCLK0, AXI4-Lite 및 VDMA 제어 |
| Processing clock | 150 MHz | PS FCLK1 100 MHz를 Clock Wizard로 변환 |
| TDC clock | 200 MHz | PS FCLK2, TDC-GPX 버스/수집 도메인 |
| `G_STREAM_CLK_MODE` | `ASYNC` | Processing/TDC가 서로 다른 물리 clock |
| `G_OUTPUT_WIDTH` | 32 bit | VDMA S_AXIS 폭과 동일 |
| `G_NUM_CHIPS` | 4 | TDC0~TDC3 활성 |
| `G_STOPS_PER_CHIP` | 8 | Chip당 STOP 8개 |
| `G_MAX_RETURNS_PER_STOP` | 7 | STOP당 최대 Return 7개 |
| Rising capability | `0011` | TDC0/TDC1 Rising |
| Falling capability | `1100` | TDC2/TDC3 Falling |
| Echo receiver | 비활성 | 외부 TDC-GPX 수집 경로 사용 |
| OEN mode | `PULLUP_OR_NOT_CONNECTED` | PCB pull-up 또는 미연결 Parent 정책 |

## 4. 171개 PL 서비스 핀

PS DDR/FIXED_IO를 제외한 LiDAR 서비스 핀은 정확히 171개다. TDC0/1과 공통
서비스 핀은 `project4_vt_hrl2_tdc01_service.xdc`, TDC2/3은
`project4_vt_hrl2_tdc23_extension.xdc`가 소유한다. 두 XDC의 171개 PACKAGE_PIN과
171개 논리 포트는 모두 고유하며 중복이 없다.

| 그룹 | 수 | 포트 |
|---|---:|---|
| Encoder | 3 | `i_enc_a`, `i_enc_b`, `i_enc_z` |
| Laser feedback | 1 | `i_fire_done` |
| GPX 상태 입력 | 12 | `i_tdc_ef1[3:0]`, `i_tdc_ef2[3:0]`, `i_tdc_irflag[3:0]` |
| GPX 양방향 데이터 | 112 | `io_tdc_d[111:0]` |
| GPX 주소 | 16 | `o_tdc_adr[15:0]` |
| GPX Chip 제어 | 24 | Chip별 `CSN/RDN/WRN/STOPDIS/ALUTRIGGER/PURESN` |
| Laser/TDC 사건 출력 | 3 | `o_fire_pulse`, `o_start_tdc`, `o_stop_tdc` |

`OEN/LF/ERRFLAG`는 IP에서 제거하지 않았다. 이 Parent만 다음 정책을 사용한다.

- `o_tdc_oen`: `G_OEN_MODE=PULLUP_OR_NOT_CONNECTED`이므로 외부 핀으로 만들지 않는다.
- `i_tdc_lf1`, `i_tdc_lf2`: Parent 내부 상수 `0`에 연결한다.
- `i_tdc_errflag`: Parent 내부 상수 `0`에 연결한다.
- 동적 OEN과 LF/ERRFLAG 배선이 필요한 다른 PCB는 IP Generic과 Parent 배선/XDC를
  바꿔야 하며, IP 공개 포트는 그대로 재사용할 수 있다.

## 5. TDC-GPX 병렬 버스 IOB 고정 계약

합성 결과가 매번 다른 내부 배선으로 TDC-GPX 병렬 버스의 지연을 바꾸지 않도록,
PACKAGE_PIN과 버스 경계 레지스터를 함께 고정한다. XDC가 물리 핀을 소유하고,
`tdc_gpx_bus_phy`의 `IOB=TRUE` 속성이 다음 레지스터를 해당 핀의
ILOGIC/OLOGIC에 배치한다.

| 경계 기능 | 수 | 필수 물리 자원 |
|---|---:|---|
| GPX D 입력 capture | 112 | `ILOGICE2.IFF` |
| GPX D 출력 | 112 | `OLOGICE2.OUTFF` |
| GPX D tri-state | 112 | `OLOGICE2.TFF` |
| GPX ADR 출력 | 16 | `OLOGICE2.OUTFF` |
| GPX CSN 출력 | 4 | `OLOGICE2.OUTFF` |
| GPX RDN 출력 | 4 | `OLOGICE2.OUTFF` |
| GPX WRN 출력 | 4 | `OLOGICE2.OUTFF` |
| **합계** | **364** | 모든 cell `IOB=TRUE`, routed LOC/BEL 할당 |

`verify_v2_l0_gpx_iob.tcl`은 합성 후 cell 수와 `IOB=TRUE`를 검사하고, 배치·배선
후에는 각 cell의 LOC/BEL이 위 역할과 일치하는지 다시 검사한다. 하나라도 누락되면
Parent Sign-off를 실패시킨다. 최신 4-chip 구현은 364개 모두 통과했다.

현재 VT Parent는 `G_OEN_MODE=PULLUP_OR_NOT_CONNECTED`이므로 OEN을 외부 핀으로
내보내지 않으며 위 364개에 포함하지 않는다. 동적 OEN PCB에서는 OEN 출력 FF도
IOB 계약과 XDC 검사 대상에 추가해야 한다. `STOPDIS/ALUTRIGGER/PURESN`은 GPX
제어 출력이지만 28-bit 병렬 Read/Write 데이터 경계가 아니므로 별도 출력 지연
계약으로 검사한다. `EF1/EF2/IRFLAG`는 비동기 상태 입력이므로 IOB 저지연보다
명시적 CDC 동기화 안전성이 우선이다.

## 6. VDMA Profile 적용 절차

통합 IP가 Rise/Fall별 `HSIZE/VSIZE/STRIDE/enable` 후보값을 Processing
도메인에서 만들고, 통합 IP 내부 `lidar_vdma_profile_cdc`가 49-bit payload를
CSR 100 MHz 도메인으로 원자적으로 전달한다. Parent에는 별도 module-reference
bridge나 AXI GPIO가 없다.

1. PS가 통합 CSR `CTL25~CTL29`에서 pending profile을 읽는다.
2. 해당 VDMA가 정지된 것을 확인한다.
3. VDMA `HSIZE`, `VSIZE`, `STRIDE`와 Frame Buffer 주소를 기록한다.
4. Rise는 `CTL25[8]`, Fall은 `CTL25[9]`에 W1S ACK를 쓴다.
5. 통합 IP가 ACK를 확인한 뒤 다음 Face부터 새 계약을 사용한다.

오래된 ACK가 HIGH인 상태는 새 요청의 승인으로 인정하지 않는다. Reset 또는
profile 변경 중 VDMA를 계속 실행하면 안 된다.

`tb_lidar_vdma_profile_cdc`와 K0-3 통합 회귀는 49-bit payload 원자성, 지연
ACK, 오래된 HIGH ACK 거부 및 두 비동기 클럭 방향을 검증한다.

## 7. Reset과 CDC 계약

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
| Processing→CSR VDMA profile | 49-bit XPM handshake, CTL25 W1S ACK |
| GPX D[27:0]/Chip | RDN 선행 후 IOB capture, 최소 25 ns Runtime 버스 유지시간 |

GPX 데이터 핀 112개에 대한 false path는 **포트에서 첫 IOB 입력 FF까지**로만
제한된다. 일반적인 2FF 동기화 데이터라고 주장하는 것이 아니다. 외부 GPX가
RDN 뒤 데이터를 안정화하고, Runtime TDC-GPX 버스 읽기 타이밍
(`BUS_CLK_DIV/BUS_TICKS`)이 최소 25 ns를 제공한다는 비동기 병렬 버스 계약이다.

별도 Reset 회귀는 200 MHz Source와 150 MHz Destination에서 다음을 검증한다.

- Source 또는 Destination Reset 중 `ready/valid` 누출 금지
- Reset 전에 FIFO에 보류된 payload 폐기
- Reset 해제 후 처음 수신한 payload가 새 값과 정확히 일치

최신 회귀 세션은 `260809190043_v2_stream_gateway_reset`이다.

## 8. 타이밍 계약과 4-chip 구현 결과

TDC 제어 출력은 반환 clock이 없는 외부 장치 인터페이스다. false path로 숨기지
않고 register-to-pad에 8 ns max-delay를 적용한다. GPX D 입력은 RDN 이후 외부
데이터가 안정된다는 계약과 Runtime TDC-GPX 버스 읽기 타이밍
(`BUS_CLK_DIV/BUS_TICKS`)의 최소 25 ns 유지시간을 함께 사용한다.

최종 4-chip Parent 구현 세션은 `260809_iob_ps_ref_l0_impl_v2`이다.

| 항목 | 결과 |
|---|---:|
| Synthesis WNS/WHS | `+0.357 / +0.036 ns` |
| Route WNS/WHS | `+0.240 / +0.030 ns` |
| CSR 100 MHz 내부 WNS | `+1.021 ns` |
| Processing 150 MHz 내부 WNS | `+0.686 ns` |
| TDC 200 MHz 내부 WNS | `+0.240 ns` |
| 최악 TDC 내부 경로 | GPX bus FSM → D tri-state IOB FF, 2 LUT |
| 최악 경로 배선 비중 | `85.363%` |
| GPX 병렬 버스 IOB 계약 | `364 / 364 PASS` |
| Active critical CDC | 0 |
| Bus-skew 위반 | 0 |
| Blocking DRC | 0 |
| Critical Warning | 0 |
| 비트스트림 | 4,045,708 bytes |

첫 4-chip 구현에서는 Lane 상태와 BUS ready가 coordinator의 Register-read 요청
소유권 enable까지 한 사이클에 이어져 `WNS -0.007 ns`가 발생했다.
`lane_register_ready_r` 파이프라인을 추가해 요청 payload는 기존 1-entry 버퍼에
그대로 보존하면서 승인 경로만 끊었다. coordinator, acquisition subsystem,
B5-B8 회귀와 최종 Parent 구현을 모두 다시 실행한 결과 위 양수 여유를 확보했다.

최신 K0-10 패키지 runner는 32-bit 150/200 MHz,
128-bit 200/150 MHz Echo 비활성, 64-bit 150/150 MHz SYNC의 세 OOC profile을
통과했다. Parent 최종 결과는 4 chips, 8 STOP/chip, 최대 7 Return,
Rising `0011`, Falling `1100`, 32-bit AXIS 출력 조건이다.

TDC 200 MHz의 `+0.240 ns`는 Sign-off 기준을 통과했지만 장기 PCB 운용 여유까지
보장하는 값은 아니다. 실제 PCB 초도 검증은 150 MHz부터 시작해 Runtime
TDC-GPX 버스 읽기 타이밍(`BUS_CLK_DIV/BUS_TICKS`), 외부 GPX Register readback,
IFIFO Drain과 데이터 무결성을 확인한 뒤 200 MHz로 올린다. 핀맵, bus PHY,
합성 전략 또는 주변 로직이 바뀌면 이 Parent 구현 검사를 반드시 다시 실행한다.

PS의 CSR, VDMA, PACKED17 복원 및 Ethernet 전송 예제는


## 9. 재생성과 Sign-off 실행

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

## 10. 아직 남은 보드 Sign-off

다음 항목은 현재 PASS에 포함되지 않는다.

1. 실제 PCB에서 40 MHz GPX 기준 clock과 TDC-GPX Register readback 확인
2. TDC0~TDC3 112-bit 데이터 버스 setup/hold 및 장기 IFIFO Drain
3. PS가 VDMA profile을 적용한 뒤 HP0/HP1 DDR byte를 읽는 실측
4. FreeRTOS/PetaLinux DMA cache invalidate/ownership API 검증
5. Rise/Fall Frame Buffer 주소, overflow, 장기 backpressure 검증
6. 실제 레이저 안전 인터록과 `fire_done` 응답시간 검증
7. DDR H-Line을 Viewer Ethernet ABI로 전송하는 지속 처리량 검증

현재 4-chip 200 MHz 구현은 timing을 통과했지만 setup 여유가 `+0.240 ns`다.
핀맵, bus PHY, 합성 전략 또는 주변 로직이 바뀌면 Parent 구현을 반드시 다시
판정한다. 200 MHz 여유가 음수가 되면 외부 버스 파형을 바꾸는 임의 파이프라인보다
먼저 bus PHY register 복제, IOB 인접 배치 및 150 MHz 운용 프로파일을 검토한다.
