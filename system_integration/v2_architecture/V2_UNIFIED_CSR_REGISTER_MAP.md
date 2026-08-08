# TDC-GPX LiDAR v2 통합 CSR 레지스터 맵

## 1. 적용 범위

이 문서는 `lidar_csr_bank`의 AXI4-Lite software ABI를 정의한다. 주소는
9-bit byte address이고 data width는 32 bit이다. 현재 ABI는 `2.6`이다.

> **필수 PCB/HW 계약:** 외부 TDC-GPX reference-clock pin에는 반드시
> `40 MHz`를 공급한다. 기준 주기(Tref)는 `25 ns`이며, PL bus/acquisition
> clock인 `G_TDC_CLK_MHZ`와 다른 신호다. 이 IP는 외부 reference clock을
> 생성하거나 측정하지 않는다.

| 영역 | Word 수 | Byte 주소 | 용도 |
|---|---:|---:|---|
| CTL | 32 | `0x000..0x07C` | W1S 명령 및 runtime shadow 설정 |
| STAT | 32 | `0x080..0x0FC` | build 정보, transaction 상태, active/derived readback |
| IRQ | 4 | `0x100..0x10C` | enable, source status, pending flag, mode |

핵심 규칙은 다음과 같다.

- CTL write는 shadow만 변경하며 실제 동작값은 바꾸지 않는다.
- `COMMIT` 성공 후에만 Processing/TDC 두 domain의 active version이 함께
  바뀐다.
- CTL1..19와 STAT4..22는 같은 bit layout이다. CTL은 다음 후보값이고,
  STAT는 마지막으로 승인된 active 값이다.
- CTL20 Echo delay profile도 같은 atomic COMMIT에 포함되지만, 고정 32-STAT
  공간을 늘리지 않기 위해 별도 active-value mirror는 두지 않는다. Software는
  성공한 active version과 Echo subsystem의 profile-ready/version을 함께 확인한다.
- CTL21/22는 16개 GPX register image를 위한 indexed portal이다. 16개 word를
  CSR 주소에 펼치지 않으므로 CTL/STAT/IRQ 주소 경계와 전체 CTL 수 32를
  유지한다. GPX staging image도 같은 COMMIT snapshot에 포함된다.
- CTL23/24는 Processing/TDC runtime 상태를 위한 indexed portal이다. CTL23은
  page 선택과 CAPTURE W1S write 및 진행 상태 readback을 제공하고, CTL24만
  read-only 결과다. 한 번에 한 page만 원래 clock domain에서 snapshot하며,
  기존 STAT 주소와 CTL/STAT/IRQ 개수는 바꾸지 않는다.
- reset 직후에는 합법적인 기본 shadow가 준비되지만 아직 active 값은
  없다. 따라서 `ACTIVE_VALID=0`, `SHADOW_DIRTY=1`이며 최초 `COMMIT`이
  필요하다.
- reserved read는 0이다. reserved/unaligned/invalid-encoding write는 값을
  바꾸지 않고 `ACCESS_ERROR_STICKY`를 세운다.
- AXI write response는 호환 CSR32 frontend 계약에 따라 decode 오류에도
  `OKAY`이다. software는 STAT2 또는 IRQ source 4를 확인해야 한다.

## 2. 기본 build profile

아래 default hex는 기본 build profile 기준이다.

```text
PROC_CLK=150 MHz, TDC_CLK=200 MHz, ASYNC
NUM_FACES=5, NUM_CHIPS=4, STOPS/CHIP=8, MAX_RETURNS=7
RISE_CAP=0011, FALL_CAP=1100, OUTPUT_WIDTH=32
ECHO_RECEIVER=enabled, ECHO_SIMULATION=disabled
```

Face center, Face enable mask, active chip mask와 falling enable은 build
profile에 맞춰 reset 시 자동 생성된다. 예를 들어 default CPR 3600과 x4
decode의 14,400 states/revolution에서 Face center는 다음과 같다.

| `NUM_FACES` | 자동 Face center states |
|---:|---|
| 1 | 7200 |
| 2 | 3600, 10800 |
| 3 | 2400, 7200, 12000 |
| 4 | 1800, 5400, 9000, 12600 |
| 5 | 1440, 4320, 7200, 10080, 12960 |

## 3. CTL 레지스터

### 3.1 요약

| 주소 | 이름 | 접근 | 기본값 | 의미 |
|---:|---|---|---:|---|
| `0x000` | COMMAND | W1S/RO-zero | `00000000` | 일회성 system command |
| `0x004` | MOTOR_PROFILE | R/W | `00020E10` | CPR, decode, direction, source mode |
| `0x008` | VIRTUAL_TICKS_LO | R/W | `00000208` | virtual state 기본 520 clocks |
| `0x00C` | VIRTUAL_HI_COUNT | R/W | `00002EE0` | revolution당 `TICKS_LO+1` state 수 |
| `0x010` | Z_PROFILE | R/W | `00000000` | Z offset/width |
| `0x014` | FACE_CENTER_0 | R/W | `000005A0` | Face 0 center state |
| `0x018` | FACE_CENTER_1 | R/W | `000010E0` | Face 1 center state |
| `0x01C` | FACE_CENTER_2 | R/W | `00001C20` | Face 2 center state |
| `0x020` | FACE_CENTER_3 | R/W | `00002760` | Face 3 center state |
| `0x024` | FACE_CENTER_4 | R/W | `000032A0` | Face 4 center state |
| `0x028` | FACE_PROFILE | R/W | `001F04B0` | 공통 half-width와 enable mask |
| `0x02C` | LASER_FIRE_PROFILE | R/W | `0120000D` | fire width와 fire-done timeout |
| `0x030` | TARGET_RANGE | R/W | `00000120` | 요청 목표 왕복시간, 5 ns 단위 |
| `0x034` | TDC_PULSE_WIDTHS | R/W | `00050005` | START/STOP pulse width |
| `0x038` | SIM_START_DELAY | R/W | `00000085` | simulation START delay |
| `0x03C` | SHOT_INTERVAL | R/W | `0000C350` | 요청 optical shot interval |
| `0x040` | TDC_BUS_PROFILE | R/W | `000FF142` | GPX bus/mask/Return 설정 |
| `0x044` | TDC_START_OFFSET | R/W | `000004D2` | GPX START offset |
| `0x048` | TDC_SCAN_TIMEOUT | R/W | `00000000` | scan watchdog source time |
| `0x04C` | TDC_CAPTURE_ADJUST | R/W | `00000000` | signed board capture 보정 |
| `0x050` | ECHO_DELAY_PROFILE | R/W | `00000000` | CH0 지연과 채널 증가분 |
| `0x054` | GPX_IMAGE_INDEX | R/W | `00000000` | GPX register index와 staging/active view 선택 |
| `0x058` | GPX_IMAGE_DATA | R/W 또는 RO | `00000000` | 선택한 GPX image word; active view는 read-only |
| `0x05C` | DIAG_INDEX | R/W1S | `00000000` | 진단 page 선택, capture 요청, 완료 상태 |
| `0x060` | DIAG_DATA | RO | `00000000` | 진단 또는 실제 GPX read의 원자적 32-bit 결과 |
| `0x064..0x07C` | RESERVED | RO-zero | `00000000` | 쓰면 access error |

### 3.2 CTL0 COMMAND

| Bit | 이름 | 의미 |
|---:|---|---|
| 0 | COMMIT | 현재 shadow를 snapshot하여 검증/계산/atomic activation 시작 |
| 1 | CLEAR_STATUS | CSR/Processing/TDC 진단 이력 clear 요청; 아래 정확한 범위 참조 |
| 2 | SOFT_RESET_REQUEST | 외부 reset supervisor로 1 CSR clock request 출력 |
| 3 | RUN | Processing operation을 RUN 상태로 전환 |
| 4 | STOP | RUN과 ARM을 함께 해제하여 안전 정지 |
| 5 | ARM | shot/laser 실행을 허가하도록 ARM 요청 |
| 6 | DISARM | RUN은 유지하고 shot/laser 실행만 차단 |
| 31:7 | RESERVED | 1을 쓰면 command 전체를 실행하지 않고 access error |

`0` write는 no-op이다. 한 write에서 command bit를 둘 이상 세우면 어느
command도 실행되지 않는다. `COMMIT`을 BUSY 중 다시 쓰면 manager가
`CFG_TRANSACTION_BUSY`로 reject한다.

W1S(Write One to Set/Start)는 **write 의미**다. 해당 bit에 1을 쓰면 일회성
command를 시작하고 0은 아무 동작도 하지 않는다는 뜻이며, Register 전체가
read 불가라는 뜻은 아니다. CTL0은 read하면 0이고, CTL23의 CAPTURE[8]은
write 시 W1S이지만 같은 bit를 read할 때는 BUSY 상태를 반환한다.

`CLEAR_STATUS`는 active 설정, shadow, `SHADOW_DIRTY`, manager의
`RECOVERY_REQUIRED` 및 IRQ pending flag를 지우지 않는다. IRQ pending은
IRQ_FLAG에 W1C해야 한다. `SOFT_RESET_REQUEST`도 이 block 자체를 reset하지
않으며 parent reset supervisor가 실제 coordinated reset을 수행해야 한다.

#### CLEAR_STATUS가 정확히 지우는 값

`CLEAR_STATUS`는 **설정 초기화가 아니라 진단 이력 초기화**다. CSR clock에서
명령을 승인한 뒤 Processing/TDC clock domain에 각각 한 번 전달되므로, 아래
원격 page는 같은 CSR clock에 즉시 0이 되는 것이 아니다. Software는 관련
`IRQ_STATUS`와 진단 page를 다시 읽어 clear 완료를 확인해야 한다.

| 관측 위치 | 0으로 만드는 field | 유지되는 field |
|---|---|---|
| `STAT2 TRANSACTION` | `[1] DONE_STICKY`, `[2] SUCCESS_STICKY`, `[3] ERROR_STICKY`, `[4] REJECTED_STICKY`, `[7] ACCESS_ERROR_STICKY`, `[23:16] LAST_ERROR_CODE`, `[31:24] LAST_REJECT_CODE` | `[0] BUSY`, `[5] RECOVERY_REQUIRED`, `[6] ACTIVE_VALID`, `[8] SHADOW_DIRTY` |
| `CTL23 DIAG_INDEX` / `CTL24 DIAG_DATA` | `VALID[9]=0`, `ERROR[10]=0`, `DIAG_DATA=0` | `INDEX[7:0]`, `BUSY[8]`, `SEQUENCE[31:16]`; 진행 중 요청도 취소하지 않음 |
| 진단 `0x10 PROC_FLAGS` | `[0]` invalid-transition sticky, `[1]` source-switch sticky, `[4]` Face overlap, `[5]` schedule overrun, `[6]` Face-close overflow, `[7]` monitor drop, `[11:8]` Laser lifecycle sticky, `[23:20]` GPX Processing data-path fault, `[25:24]` Shot/STOP CDC drop | `[3:2]` virtual-Z 현재 fault와 `[19:12]`, `[26]` ready/idle/enable/reset-busy live state |
| 진단 `0x11..0x18` | 위치 전이, Face overlap, Shot overrun, monitor drop, Laser request-drop/timeout/abort/unexpected-done 누적 count | 해당 없음 |
| 진단 `0x1A GPX_PROC_FAULTS` | `[18:0]` Raw28/Hit17/Cell/Frame 및 Shot/STOP drop sticky | 해당 없음 |
| 진단 `0x20 ECHO_FLAGS` | `[4:2]` outside-window/overlap/profile-not-ready sticky, `[7:6]` timeout/aborted sticky | `[0]` window active, `[1]` simulation active, `[5]` 최근 snapshot valid |
| 진단 `0x21..0x23` | Echo outside-window/overlap/profile-not-ready 누적 count | 해당 없음 |
| 진단 `0x80 TDC_SUMMARY` | `[15]` 물리 GPX Register-read error sticky | `[14:0]` mask/ready/run/active live state |
| 진단 `0x88..0x8B TDC_LANE_FAULT_n` | `[0]` drain-timeout, `[1]` sequence, `[2]` response mismatch, `[3]` raw drop, `[4]` drain cap, `[5]` Register request overflow, `[6]` run-timeout, `[9:7]` timeout cause, `[10]` init/config coalesced, `[11]` command collision, `[12]` bus fatal 이력 | 진행 중 FSM/FIFO/pending 요청과 살아 있는 bus-fatal 격리 상태; 살아 있는 원인은 clear 뒤 다시 bit를 세움 |

다음 값은 `CLEAR_STATUS`로 지우지 않는다.

- `CTL1..20` Shadow, CTL21/22 staging image, 마지막 성공 Active 설정/image;
- `STAT3` operation/Active version, `STAT4..31` Active/Derived 값과
  `STAT31[31:16]` COMMIT 완료 횟수;
- 진단 `0x1B..0x1F`의 profile/geometry live 값;
- 진단 `0x24..0x26`, `0x40..0x5F`의 최근 Echo Shot snapshot;
- `IRQ_ENABLE`, `IRQ_FLAG`, `IRQ_MODE`. `IRQ_FLAG`는 원인 level을 먼저 내린 뒤
  별도로 W1C한다.

K1-1부터 `TDC_LANE_FAULT_n[0:12]`의 모든 이력 field는 각 실제 소유 process에서
`CLEAR_STATUS`를 소비한다. 단, 이 명령은 진행 중인 Chip 초기화, Register 요청,
IFIFO/FIFO 내용, acquisition FSM 또는 bus-fatal 격리 상태를 취소하지 않는다.
clear와 새 fault가 같은 TDC clock에 겹치면 **새 fault가 우선**되어 해당 bit가
1로 남는다. 이는 새 사건을 잃지 않기 위한 의도된 우선순위다.

`[12] bus fatal`은 특히 기능 상태와 진단 이력을 분리한다. 살아 있는 bus-fatal
격리 상태는 `CLEAR_STATUS`로 해제되지 않으며, 안정된 bus-idle 복구 또는 실제
reset/reinit만 기능 상태를 해제한다. 기능 상태가 살아 있으면 `[12]`는 clear와
동시에 다시 1이 된다. 기능 복구 뒤에는 `CLEAR_STATUS`로 post-mortem 진단 이력만
지울 수 있다. 이후 `IRQ_STATUS[8] GPX_TRANSPORT=0`을 확인하고 `IRQ_FLAG[8]`을
W1C한다.

RUN/STOP/ARM/DISARM은 저장되는 설정 bit가 아니라 Processing domain으로
전달되는 일회성 event이다. CSR-to-Processing mailbox가 이전 명령의 ACK를
기다리는 동안 `COMMAND_READY=0`이며, 이때 새 operation command를 쓰면 실행하지
않고 `ACCESS_ERROR_STICKY`를 세운다. Software는 명령 사이에 STAT3의
`COMMAND_READY=1`을 확인한다.

CSR reset은 operation command authority 상실로 취급한다. 이 상태는 Processing
domain으로 동기화되어 기존 RUN/ARM을 모두 해제하며, reset 해제 후 software는
`COMMAND_READY=1`과 안정된 STAT3을 확인한 뒤 RUN과 ARM을 다시 발행한다.

operation 상태 전이 규칙은 다음과 같다.

- reset 또는 `ACTIVE_VALID=0`: STOPPED/DISARMED;
- RUN: 유효하고 release된 Processing active 설정이 있을 때만 승인;
- ARM: RUN 상태이며, physical mode에서는 동기화된 외부 permit까지 있어야 승인;
- STOP: RUN과 ARM을 모두 해제하므로 다음 시작에는 RUN과 ARM이 다시 필요;
- DISARM: RUN은 유지하지만 scheduler와 physical fire를 차단;
- physical mode의 permit 상실: 물리 fire gate는 raw 입력으로 즉시 닫히고,
  Processing clock에서 상실을 관측하면 DISARM되어 명시적 ARM이 다시 필요;
- simulation mode: 외부 permit 없이 ARM할 수 있지만 physical fire enable은 항상 0.

Atomic COMMIT의 prepare 구간에서는 RUN/ARM 기억값을 유지하되 operation enable을
닫는다. pipeline이 idle이 된 뒤에만 prepare ACK가 가능하고, release 후 같은
operation 상태로 재개한다.

### 3.3 CTL1 MOTOR_PROFILE

| Bit | 이름 | 기본 | 의미 |
|---:|---|---:|---|
| 15:0 | CPR | 3600 | encoder cycles/revolution |
| 17:16 | DECODE_MODE | `10` | `00=x1`, `01=x2`, `10=x4`, `11=invalid` |
| 18 | DIRECTION | 0 | `0=CW`, `1=CCW` |
| 19 | Z_EARLY | 0 | Z event의 early 정책 |
| 20 | SIMULATION_MODE | 0 | `0=physical encoder/fire-done`, `1=virtual encoder/simulated T0` |
| 31:21 | RESERVED | 0 | 반드시 0; 1을 쓰면 해당 word를 거부 |

`decoded_states_per_rev = CPR * decode_multiplier`이다. physical decode는
runtime 변경 가능하지만 virtual encoder 내부 발생은 x4 계약을 유지한다.
`SIMULATION_MODE`는 지속 설정이며 COMMIT 후에만 active source가 바뀐다.
물리/가상 경로의 실측 latency는 동작을 지연시키는 설정이 아니다. F1/F5에서
K0-6에서 재측정한 값은 B0-to-accept 5 clocks, physical
first-sample-to-fire 9 clocks, internal virtual-source-to-accept 7 clocks이다.
`o_virtual_a/b/z` 관찰 포트부터 accept까지 직접 측정하면 6 clocks다. 현재
Checkpoint E의 고정 32-STAT ABI에는
새 주소를 임의로 끼워 넣지 않았고 F5 subsystem의 read-only output으로 먼저
노출한다. Stage K 통합 status ABI 검토에서 기존 예약 bit 또는 표준 indexed
status portal 중 하나를 확정한 뒤 CSR readback을 추가한다. 어떤 경우에도 CTL로
쓰거나 timing padding으로 사용하지 않는다.

### 3.4 CTL2..CTL10 Motor/Mirror

| 레지스터 | Bit | 의미 |
|---|---:|---|
| VIRTUAL_TICKS_LO | 31:0 | 한 virtual state의 낮은 integer clock 수 |
| VIRTUAL_HI_COUNT | 14:0 | 한 revolution에서 `TICKS_LO+1`을 쓰는 state 수 |
| VIRTUAL_HI_COUNT | 31:15 | Reserved, 반드시 0 |
| Z_PROFILE | 14:0 | Z_OFFSET |
| Z_PROFILE | 30:16 | Z_WIDTH |
| Z_PROFILE | 15,31 | Reserved, 반드시 0 |
| FACE_CENTER_n | 14:0 | decoded-state 좌표의 Face center |
| FACE_CENTER_n | 31:15 | Reserved, 반드시 0 |
| FACE_PROFILE | 14:0 | 모든 active Face가 공유하는 half-width |
| FACE_PROFILE | 20:16 | FACE_ENABLE_MASK `[4:0]` |
| FACE_PROFILE | 15,31:21 | Reserved, 반드시 0 |

합성 시 `NUM_FACES`보다 높은 enable bit는 commit validation error이다.
inactive Face center word는 보존되지만 동작에는 사용되지 않는다.

### 3.5 CTL11..CTL15 Laser/Range

| 레지스터 | Bit | 단위 | 의미 |
|---|---:|---|---|
| LASER_FIRE_PROFILE | 15:0 | 5 ns ticks | FIRE pulse width |
| LASER_FIRE_PROFILE | 31:16 | 5 ns ticks | FIRE_DONE timeout |
| TARGET_RANGE | 31:0 | 5 ns ticks | 요청 목표 왕복시간/수신 window |
| TDC_PULSE_WIDTHS | 15:0 | 5 ns ticks | START_TDC width |
| TDC_PULSE_WIDTHS | 31:16 | 5 ns ticks | STOP_TDC width |
| SIM_START_DELAY | 31:0 | 5 ns ticks | simulation START delay |
| SHOT_INTERVAL | 29:0 | micro-degree | 요청 optical point 간격 |
| SHOT_INTERVAL | 31:30 | - | Reserved, 반드시 0 |

5 ns tick 값은 runtime source ABI이다. commit calculator가 build의
`PROC_CLK_MHZ` 또는 `TDC_CLK_MHZ`에 맞춰 필요한 domain clock 수로 한 번만
변환한다. `FIRE_DONE_TIMEOUT <= TARGET_RANGE`여야 한다.

`TARGET_RANGE`는 software가 쓰는 유일한 목표 왕복시간 원본이다. 물리
TDC-GPX의 `Reg7.MTimer[27:15]`는 40 MHz 기준, 즉 25 ns/tick이므로 COMMIT이
다음 두 값을 자동 계산한다.

```text
Reg7.MTimer                     = ceil(TARGET_RANGE / 5)
EFFECTIVE_TARGET_RANGE_5NS      = Reg7.MTimer * 5
```

레이저 `stop_tdc`, TDC capture window와 실제 GPX active image는 요청값보다
작아질 수 없는 `EFFECTIVE_TARGET_RANGE_5NS`를 공통으로 사용한다. 기본값
`TARGET_RANGE=288`은 요청 1,440 ns, `Reg7.MTimer=58`, 실효 1,450 ns이다.
13-bit MTimer로 표현 가능한 최대 요청값은 `40,955` ticks, 즉 204.775 us다.

### 3.6 CTL16..CTL19 TDC-GPX

| 레지스터 | Bit | 단위 | 의미 |
|---|---:|---|---|
| TDC_BUS_PROFILE | 5:0 | TDC clocks | GPX bus divider, valid 1..63 |
| TDC_BUS_PROFILE | 11:6 | divided-bus ticks | bus timing ticks, valid 1..7 |
| TDC_BUS_PROFILE | 15:12 | mask | runtime active chip mask |
| TDC_BUS_PROFILE | 16 | boolean | falling path enable |
| TDC_BUS_PROFILE | 19:17 | Hits | maximum Returns/STOP, 1..build max |
| TDC_BUS_PROFILE | 31:20 | - | Reserved, 반드시 0 |
| TDC_START_OFFSET | 17:0 | TDC clocks | GPX START timing offset |
| TDC_START_OFFSET | 31:18 | - | Reserved, 반드시 0 |
| TDC_SCAN_TIMEOUT | 31:0 | 5 ns ticks | scan watchdog source |
| TDC_CAPTURE_ADJUST | 16:0 | signed 5 ns ticks | target range에 더하는 board 보정 |
| TDC_CAPTURE_ADJUST | 31:17 | - | Reserved, 반드시 0 |

`TDC_BUS_PROFILE.BUS_TICKS`는 CSR 호환성을 위해 6 bit를 유지하지만 물리
GPX bus FSM의 입력은 3 bit다. 따라서 `[11:9]`는 항상 0이어야 하며 8 이상은
commit 시 `CFG_RUNTIME_BUS_TIMING`으로 거부된다. 짧지만 1..7 범위에 있는
값은 board-safe read capture 시간을 지키도록 물리 FSM에서 위로 clamp된다.

`TDC_CAPTURE_ADJUST`는 17-bit two's-complement field이다. 음수도 상위
reserved bit를 sign-extension하지 않고 `[16:0]`에만 기록한다.

`EFFECTIVE_TARGET_RANGE_5NS + TDC_CAPTURE_ADJUST`를 TDC clock 수로 올림
변환한 값은
검증된 GPX acquisition watchdog 폭 때문에 65,535 clocks 이하여야 한다.
그보다 큰 shadow 설정은 COMMIT에서 `CFG_RUNTIME_CAPTURE_WINDOW`로 거부되며,
하위 16 bit로 잘라서 적용하지 않는다.

현재 `TDC_SCAN_TIMEOUT=0`은 0 TDC clocks로 그대로 파생된다. 실제 GPX
watchdog에서 0을 disable로 해석할지 금지할지는 Stage 5 acquisition 계약에서
최종 확정해야 하며 CSR block이 임의로 의미를 바꾸지 않는다.

### 3.7 CTL20 ECHO_DELAY_PROFILE

| Bit | 이름 | 기본 | 단위 | 의미 |
|---:|---|---:|---|---|
| 15:0 | CHANNEL_0_DELAY | 0 | 5 ns ticks | Echo simulation 채널 0의 지연 |
| 31:16 | CHANNEL_STEP | 0 | 5 ns ticks/channel | 다음 채널마다 더할 지연 |

채널 `n`의 지연은 다음 한 식으로 결정된다.

```text
channel_delay_5ns[n] = CHANNEL_0_DELAY + n * CHANNEL_STEP
```

최대 채널 수는 build의 `NUM_CHIPS * STOPS_PER_CHIP`이며 32채널이다. 두
source field는 16 bit지만 내부 전개 누산기는 32 bit이므로
`CHANNEL_0_DELAY + 31 * CHANNEL_STEP` 계산이 16 bit에서 잘리지 않는다.
active configuration version을 받은 뒤 한 Processing clock에 한 채널씩
전개하고, 각 누적 5 ns tick 값을 선택된 Processing 주파수의 clock 수로
올림 변환한다. 0 clock이 된 채널은 synthetic Echo를 생성하지 않는다.

이 레지스터는 `enable_echo_simulation=true` build에서만 synthetic source가
소비한다. 물리 LVDS-to-STOP 경로에는 영향을 주지 않는다. 별도 32-entry
table, INDEX/DATA portal 또는 Echo 전용 APPLY command는 없다.

`enable_echo_receiver`와 `enable_echo_simulation`은 합성 전 build option이다.

| Build option | 합성되는 Echo 경로 |
|---|---|
| `enable_echo_receiver=false` | physical receiver와 synthetic STOP 경로를 모두 제거 |
| `enable_echo_receiver=true`, `enable_echo_simulation=false` | physical LVDS-to-STOP만 합성 |
| `enable_echo_receiver=true`, `enable_echo_simulation=true` | physical 경로와 synthetic test source를 함께 합성 |

따라서 최대 32채널 지원을 위해 CSR를 채널별로 늘리지 않는다. CTL20의 두
16-bit 값만 저장하고 내부에서 채널 번호에 따라 확장한다.

CTL20 readback만으로는 적용 완료를 판정하지 않는다. CTL20은 마지막으로 쓴
Shadow 후보값이므로, 성공한 COMMIT 뒤 `SHADOW_DIRTY=0`, Echo profile
`READY=1`, `BUSY=0`, `PROFILE_VERSION=STAT3.ACTIVE_VERSION`을 함께 확인해야
그 값으로 현재 profile이 만들어졌음을 보장할 수 있다. 두 16-bit field가
32 bit 전체를 사용하므로 상위 bit를 적용 상태로 재사용하지 않는다.

### 3.8 CTL21/CTL22 GPX_REGISTER_IMAGE_PORTAL

CTL21은 포털 선택자이며 CTL22는 선택된 28-bit GPX register word이다.

이 두 레지스터는 목표 왕복시간을 바꾸기 위해 항상 수정하는 레지스터가
아니다. 목표 왕복시간만 바꾸면 `CTL12.TARGET_RANGE`만 수정한다. COMMIT이
`Reg7.MTimer`를 자동 계산한다. CTL21/22가 필요한 경우는 HSDiv, RefClkDiv,
interrupt route 등 MTimer 이외의 GPX 설정 bit를 바꾸거나, 마지막으로 승인한
설정 이미지를 확인할 때다.

| Register | Bit | 이름 | Reset | 의미 |
|---|---:|---|---:|---|
| CTL21 | 3:0 | GPX_IMAGE_INDEX | 0 | GPX register 0..15 선택 |
| CTL21 | 8 | VIEW_ACTIVE | 0 | `0=staging`, `1=마지막 성공 active image` |
| CTL21 | 31:9, 7:4 | RESERVED | 0 | 1을 쓰면 access error |
| CTL22 | 27:0 | GPX_IMAGE_DATA | register별 기본 image | 선택된 GPX register data |
| CTL22 | 31:28 | RESERVED | 0 | GPX 외부 bus가 28 bit이므로 1을 쓰면 전체 write 거부 |

`VIEW_ACTIVE=0`에서는 CTL22를 읽고 쓸 수 있다. staging image의 reset 값은
별도 복사본이 아니라 검증된 v1 `c_GPX_DEFAULT_IMAGE`를 단일 source로
변환한 값이다. `VIEW_ACTIVE=1`에서는 CTL22가 read-only이며, active image가
아직 없으면 0을 반환한다. active view write는 값을 바꾸지 않고
`ACCESS_ERROR_STICKY`를 세운다.

CTL21의 index/view 변경은 설정값 변경이 아니므로 `SHADOW_DIRTY` 또는 shadow
revision을 바꾸지 않는다. CTL22 staging data가 실제로 달라질 때만 revision이
증가한다. COMMIT이 승인되는 순간 16-entry staging image 전체를 한 번에
snapshot하며, 진행 중 software edit는 다음 COMMIT 후보로 남는다. TDC domain
ACTIVATE ACK는 모든 build-time present GPX chip의 register programming 완료
후에만 반환된다.
따라서 software의 DONE은 단순 CDC 전달 완료가 아니라 물리 GPX image 적용
완료를 뜻한다.

`GPX_IMAGE_DATA`로 쓴 Reg7 word 중 MTimer 이외의 bit는 그대로 유지된다.
다만 `Reg7.MTimer[27:15]`는 이중 시간 설정을 막기 위해 CTL12에서 파생한
값으로 덮어쓴다. `VIEW_ACTIVE=1`로 읽는 Reg7은 software staging 원본이
아니라 실제 적용된 MTimer를 포함한 effective image다.

중요하게도 `VIEW_ACTIVE=0`에서 Reg7.MTimer bit를 쓰면 **staging 저장소에는
그 값이 기록되고 다시 읽힌다**. 다만 COMMIT의 prepare 단계에서 그 13 bit만
`ceil(CTL12.TARGET_RANGE_5NS / 5)`로 덮어쓰므로, software가 쓴 MTimer는
Active image나 물리 Chip에 적용되지 않는다. COMMIT 뒤에도 staging image는
software 원본을 그대로 보존하므로 staging Reg7과 active Reg7이 다른 것은
정상이다.

| 값의 층 | 읽는 방법 | Reg7.MTimer 의미 | 쓰기 가능 여부 |
|---|---|---|---|
| Staging 후보 image | CTL21 `VIEW_ACTIVE=0`, CTL22 read | software가 마지막으로 쓴 원본; COMMIT 때 MTimer source로 사용하지 않음 | 가능 |
| Active effective image | CTL21 `VIEW_ACTIVE=1`, CTL22 read | CTL12에서 자동 계산되어 마지막 성공 COMMIT에 적용한 값 | 불가 |
| 물리 Chip Register | CTL23/24 `11CCAAAA` read | 외부 GPX pin/bus로 실제 다시 읽은 값 | 이 portal은 read-only |

권장 Reg7 운용 순서는 다음과 같다.

1. CTL21에서 `INDEX=7`, `VIEW_ACTIVE=0`을 선택하고 CTL22를 읽는다.
2. HSDiv, RefClkDiv, interrupt route 등 MTimer 이외 bit만 read-modify-write한다.
3. 목표 왕복시간은 CTL12에 5 ns tick으로 쓴다. staging MTimer를 별도로
   계산하거나 active 값과 맞추지 않는다.
4. COMMIT 성공과 `SHADOW_DIRTY=0`을 확인한다.
5. CTL21 `VIEW_ACTIVE=1`의 Reg7에서 자동 계산된 MTimer를 확인한다.
6. 보드 검증이나 고장 분석에서는 CTL23/24로 같은 Chip의 실제 Reg7을 읽어
   Active image와 비교한다.

### 3.9 Shadow, Active, Derived, Physical의 관계

설정값은 다음 순서로 이동한다.

```text
PS write
  -> Shadow 후보(CTL1..20, CTL21/22 staging image)
  -> CTL0.COMMIT 시점의 원자 snapshot
  -> 검증 + 시간 변환 + 정수 양자화 + Reg7.MTimer 자동 대체
  -> Processing/TDC 두 domain prepare/activate/release
  -> Active source(STAT4..22) + Derived(STAT23..31)
  -> GPX Active effective image(CTL21/22 active view)
  -> 외부 GPX 물리 Register(CTL23/24 readback으로 별도 확인)
```

| 설정 종류 | 후보값 확인 | 승인 source 확인 | 계산/적용 확인 |
|---|---|---|---|
| CTL1..19 일반 설정 | 같은 CTL read | 대응 `STAT4..22` | 대응 `STAT23..31` Derived와 `STAT3.ACTIVE_VERSION` |
| CTL20 Echo delay | CTL20 read | 별도 32-bit Active mirror 없음 | 진단 `0x1B`의 READY/BUSY/VERSION과 `SHADOW_DIRTY` |
| GPX 16-word image | CTL21/22 staging view | CTL21/22 active view | CTL23/24 물리 Chip readback |

COMMIT이 실패하면 Shadow 후보는 남지만 Active와 Active version은 바뀌지 않는다.
COMMIT 진행 중 software가 Shadow를 다시 쓰면 진행 중 transaction은 시작 시점의
snapshot만 적용하고, 새 write는 다음 transaction 후보가 된다. 따라서 성공
완료 뒤에도 `SHADOW_DIRTY=1`이면 오류가 아니라 "다음 COMMIT 후보가 이미
존재함"을 뜻한다. 반대로 다음 조건이 모두 참이어야 현재 Shadow revision과
Active가 일치한다고 판정한다.

```text
STAT2.BUSY           = 0
STAT2.SUCCESS_STICKY = 1
STAT2.ERROR_STICKY   = 0
STAT2.SHADOW_DIRTY   = 0
STAT3.ACTIVE_VERSION = 기대한 새 version
```

## 4. STAT 레지스터

### 4.1 STAT0 CORE_INFO (`0x080`)

| Bit | 의미 | 기본 profile |
|---:|---|---:|
| 7:0 | ABI minor | 6 |
| 15:8 | ABI major | 2 |
| 18:16 | NUM_FACES | 5 |
| 21:19 | NUM_CHIPS | 4 |
| 25:22 | STOPS_PER_CHIP | 8 |
| 28:26 | MAX_RETURNS_PER_STOP | 7 |
| 29 | ECHO_RECEIVER_INCLUDED | 1 |
| 30 | ECHO_SIMULATION_INCLUDED | 0 |
| 31 | STREAM_CLOCK_SYNC | 0 |

기본 profile 값은 `0x3E250206`이다.

### 4.2 STAT1 BUILD_INFO (`0x084`)

| Bit | 의미 | 기본 profile |
|---:|---|---:|
| 7:0 | PROC_CLK_MHZ | 150 |
| 15:8 | TDC_CLK_MHZ | 200 |
| 17:16 | OUTPUT_WIDTH_CODE | `00=32`, `01=64`, `10=128` |
| 19:18 | Reserved | 0 |
| 23:20 | RISE_CAPABILITY_MASK | `0011` |
| 27:24 | FALL_CAPABILITY_MASK | `1100` |
| 31:28 | Reserved | 0 |

기본 profile 값은 `0x0C30C896`이다.

### 4.3 STAT2 TRANSACTION (`0x088`)

| Bit | 이름 | 의미 |
|---:|---|---|
| 0 | BUSY | validation/prepare/activate/release 진행 중 |
| 1 | DONE_STICKY | 성공 또는 오류 completion 발생 |
| 2 | SUCCESS_STICKY | 성공 completion 발생 |
| 3 | ERROR_STICKY | 오류 completion 발생 |
| 4 | REJECTED_STICKY | BUSY/locked 상태의 추가 commit 거절 |
| 5 | RECOVERY_REQUIRED | 양 domain atomic 상태를 보장할 수 없어 reset 필요 |
| 6 | ACTIVE_VALID | STAT4..31의 active/derived payload가 유효 |
| 7 | ACCESS_ERROR_STICKY | CSR 주소/encoding/command 오류 |
| 8 | SHADOW_DIRTY | 현재 shadow가 active snapshot과 다름 |
| 15:9 | Reserved | 0 |
| 23:16 | LAST_ERROR_CODE | 마지막 완료 오류 코드 |
| 31:24 | LAST_REJECT_CODE | 마지막 commit reject 코드 |

reset 값은 `0x00000100`이다. commit 중 다음 shadow를 써도 revision을 별도
추적하므로 현재 commit 성공 후 DIRTY가 잘못 clear되지 않는다.

### 4.4 STAT3 Operation/Active version과 STAT4..STAT22 Active readback

STAT3 (`0x08C`)은 active configuration version과 read-only operation 상태를
한 word에 담는다.

| Bit | 이름 | 의미 |
|---:|---|---|
| 15:0 | ACTIVE_VERSION | 마지막 atomic activation version; `ACTIVE_VALID=0`이면 0 |
| 16 | RUNNING | RUN이 승인되어 유지 중 |
| 17 | ARMED | ARM이 승인되어 유지 중 |
| 18 | EXTERNAL_PERMIT | 두 단계 동기화까지 통과한 외부 laser permit |
| 19 | CONFIG_READY | Processing active 설정이 valid이고 release됨 |
| 20 | PROCESSING_ENABLE | RUNNING과 CONFIG_READY가 모두 1 |
| 21 | SCHEDULER_ENABLE | physical 또는 simulation shot 발행 허가 |
| 22 | PHYSICAL_FIRE_ENABLE | RUN+ARM+physical mode+permit 최종 허가 |
| 23 | SIMULATION_ENABLE | RUN+ARM+simulation mode 허가 |
| 24 | COMMAND_READY | operation command mailbox가 다음 W1S를 받을 수 있음 |
| 25 | COMMAND_BUSY | 이전 operation command ACK 대기 중 |
| 31:26 | RESERVED | 0 |

STAT4..22 (`0x090..0x0D8`)는 CTL1..CTL19와 동일 bit layout의 active
source이다.

`ACTIVE_VALID=0`일 때 STAT3의 ACTIVE_VERSION과 STAT4..STAT30의
active/derived 값은 0이다. STAT3 상위 operation 상태는 live readback이므로
COMMAND_READY 같은 CDC 상태는 계속 표시될 수 있다. reset 직후 CDC가 안정되기
전 STAT3은 `0x00000000`, 안정 후 command mailbox가 준비되면
`0x01000000`이다. 기본 설정을 최초 commit하고 아직 RUN하지 않은 정상 상태는
`0x01080001`이다.

STAT3의 operation bit는 제어 입력이 아닌 비동기 진단 readback이다. 상태 전환
직후에는 bit별 동기화 지연 차이가 보일 수 있으므로 software는 두 번 연속 같은
STAT3을 읽은 뒤 상태 전이를 확정한다.

### 4.5 STAT23..STAT31 Derived readback

| 주소 | 이름 | Bit | 의미 |
|---:|---|---:|---|
| `0x0DC` | DERIVED_GEOMETRY | 15:0 | TOTAL_STATES |
|  |  | 31:16 | SHOT_INTERVAL_STATES, ceil 적용 |
| `0x0E0` | DERIVED_FACE | 15:0 | FACE_ACTIVE_POSITIONS=`2*half+1` |
|  |  | 31:16 | COLUMNS_PER_FACE |
| `0x0E4..0x0F4` | FACE_BOUNDS_0..4 | 14:0 | LOWER, inclusive |
|  |  | 15 | Reserved |
|  |  | 30:16 | UPPER, inclusive |
|  |  | 31 | Reserved |
| `0x0F8` | CAPTURE_TDC_CLKS | 31:0 | `(EFFECTIVE_TARGET_RANGE_5NS+ADJUST)`의 TDC clock 변환값 |
| `0x0FC` | DERIVED_MASKS | 3:0 | PRESENT_CHIP_MASK |
|  |  | 7:4 | ACTIVE_RISE_MASK |
|  |  | 11:8 | ACTIVE_FALL_MASK |
|  |  | 15:12 | Reserved |
|  |  | 31:16 | 완료 transaction count, modulo 65536 |

Face bounds는 저장된 geometry이며 CW/CCW에 따라 다시 쓰지 않는다.
CW/CCW는 어느 bound가 Entry/Exit인지와 traversal 방향만 바꾼다.

### 4.6 CTL23/24 Runtime 진단 portal

| 주소 | 이름 | 접근 | Reset | 의미 |
|---:|---|---|---:|---|
| `0x05C` | DIAG_INDEX | R/W1S | `0x00000000` | index 선택, snapshot 시작 및 완료 상태 |
| `0x060` | DIAG_DATA | RO | `0x00000000` | 마지막으로 완료된 32-bit snapshot |

`DIAG_INDEX` write bit:

| Bit | 이름 | 의미 |
|---:|---|---|
| 7:0 | INDEX | 조회할 page 번호 |
| 8 | CAPTURE | W1S, 선택한 page의 snapshot 요청 |
| 31:9 | Reserved | 0만 허용 |

`DIAG_INDEX` read bit:

| Bit | 이름 | 의미 |
|---:|---|---|
| 7:0 | INDEX | 마지막 선택 page |
| 8 | BUSY | 요청 또는 응답이 진행 중 |
| 9 | VALID | DIAG_DATA가 마지막 요청의 완료 결과 |
| 10 | ERROR | 미지원 index, ordering 위반, 원격 reset, 물리 read timeout/응답 오류 |
| 15:11 | Reserved | 0 |
| 31:16 | SEQUENCE | 완료마다 modulo 65536 증가 |

Software는 `INDEX | 0x100`을 쓴 뒤 `BUSY=0 && VALID=1`과 SEQUENCE 증가를
확인하고 DIAG_DATA를 읽는다. BUSY 중 두 번째 요청은 기존 요청을 덮지 않으며
ACCESS_ERROR로 기록된다. 한 page는 원자적이지만 여러 page를 순서대로 읽은
결과가 모두 같은 순간의 snapshot이라는 보장은 없다.

#### 실제 외부 TDC-GPX Register read

`0xC0..0xFF`는 설정 이미지가 아니라 PCB에 연결된 실제 TDC-GPX Chip에서
Register를 읽는 maintenance page다. INDEX 형식은 다음과 같다.

```text
INDEX[7:0] = 11CCAAAA
  CC   = Chip 번호 0..3
  AAAA = TDC-GPX Register 주소 0..15
```

성공한 `DIAG_DATA`는 요청한 구조 그대로 한 Word다.

```text
31                 28 27                              0
+--------------------+--------------------------------+
| 요청 Register 주소 | 실제 외부 GPX read data[27:0]  |
+--------------------+--------------------------------+
```

예를 들어 Chip 1의 Reg7은 INDEX `0xD7`이다. 결과 `0x71ABCDE0`은 주소가
Reg7이고 실제 28-bit 값이 `0x1ABCDE0`임을 뜻한다.

물리 read는 반드시 먼저 `DISARM`하여 `STAT3.SCHEDULER_ENABLE=0`을 확인한
뒤 요청한다. 요청이 승인되면 RTL은 새 acquisition을 자동 pause하고 진행 중
drain이 끝나 `TDC_SAFE=1`이 될 때까지 기다린다. 그 뒤 한 번의 bus read를
수행하고 RUN acquisition을 복구하되 scheduler는 계속 DISARM 상태로 둔다.
물리 read 중 COMMIT/RUN/ARM은 access error로 거부된다. 전체 서비스가
10 ms 안에 끝나지 않거나 응답 Chip/주소가 다르면 `ERROR=1`,
`TDC_SUMMARY[15]=1`, IRQ source 8 `GPX_TRANSPORT`가 설정된다.

CTL21/22의 `VIEW_ACTIVE=1`은 마지막으로 승인해 쓰려고 한 **설정 이미지**다.
반면 이 page는 외부 Chip pin을 통해 다시 읽은 **실제 물리 상태**다. Sign-off와
고장 진단에서는 둘을 비교해야 한다.

#### Processing/Echo page

| Index | 이름 | Bit 의미 |
|---:|---|---|
| `0x10` | PROC_FLAGS | 0 invalid transition, 1 source switch, 3:2 virtual-Z fault, 4 Face overlap, 5 schedule overrun, 6 Face-close overflow, 7 monitor drop, 8 laser request drop, 9 fire_done timeout, 10 operation abort, 11 unexpected done, 12 pipeline idle, 13 Echo idle, 14 GPX Processing idle, 15 GPX AXIS idle, 16 Echo profile ready, 17 Echo profile busy, 18 Rise enabled, 19 Fall enabled, 20 GPX context fault, 21 Hit fault-any, 22 Cell fault-any, 23 Frame fault-any, 24 Shot CDC drop, 25 STOP CDC drop, 26 GPX CDC reset busy |
| `0x11` | PROC_INVALID_COUNT | 31:0 invalid transition count |
| `0x12` | PROC_FACE_OVERLAP_COUNT | 31:0 Face overlap count |
| `0x13` | PROC_OVERRUN_COUNT | 31:0 schedule overrun count |
| `0x14` | PROC_MON_DROP_COUNT | 31:0 monitor drop count |
| `0x15` | LASER_REQ_DROP_COUNT | 31:0 laser request drop count |
| `0x16` | LASER_TIMEOUT_COUNT | 31:0 fire_done timeout count |
| `0x17` | LASER_ABORT_COUNT | 31:0 operation abort count |
| `0x18` | LASER_UNEXPECTED_COUNT | 31:0 unexpected fire_done count |
| `0x19` | PROC_LATENCY_CONTRACT | 7:0 B0-to-accept, 15:8 physical-to-fire, 23:16 virtual-to-accept, 27:24 fire_done synchronizer, 31:28 re-arm margin; 모두 Processing clocks |
| `0x1A` | GPX_PROC_FAULTS | 0 context, 1 Chip index, 2 STOP index, 3 slope role, 4 Cell context, 5 Return overflow, 6 START number, 7 Hit capacity drop, 8 Frame context, 9 unexpected Cell, 10 duplicate Cell, 11 duplicate terminal, 12 missing Cell, 13 geometry, 14 column gap, 15 masked payload drop, 16 Face-close overflow, 17 Shot CDC drop, 18 STOP CDC drop |
| `0x1B` | PROC_PROFILE_STATE | 0 Echo ready, 1 Echo busy, 2 Rise valid, 3 Rise enabled, 4 Fall valid, 5 Fall enabled, 6 pipeline idle, 7 GPX AXIS idle, 8 GPX CDC reset busy, 31:16 Echo profile version |
| `0x1C` | RISE_GEOMETRY | 15:0 HSIZE bytes, 31:16 VSIZE lines |
| `0x1D` | RISE_STRIDE | 15:0 STRIDE bytes, 18:16 visible Returns, 24:19 Cell slots, 25 valid, 26 enabled, 28:27 Footer lines |
| `0x1E` | FALL_GEOMETRY | 15:0 HSIZE bytes, 31:16 VSIZE lines |
| `0x1F` | FALL_STRIDE | Rise STRIDE와 같은 layout |
| `0x20` | ECHO_FLAGS | 0 window active, 1 simulation active, 2 outside-window sticky, 3 overlap sticky, 4 profile-not-ready sticky, 5 snapshot valid, 6 timeout sticky, 7 aborted sticky |
| `0x21..0x23` | ECHO_COUNTS | outside-window, overlap, profile-not-ready count |
| `0x24` | ECHO_TOTALS | 15:0 total Rise, 31:16 total Fall |
| `0x25` | ECHO_RISE_MASK | channel 31:0 Rise mask |
| `0x26` | ECHO_FALL_MASK | channel 31:0 Fall mask |
| `0x40..0x5F` | ECHO_CHANNEL_0..31 | 7:0 Rise count, 15:8 Fall count |

#### 진단 0x1B PROC_PROFILE_STATE의 목적과 사용법

`0x1B`는 설정값 자체를 다시 저장하는 Register가 아니라, 성공한 Active 설정이
Processing-side Echo profile과 Rise/Fall VDMA geometry까지 전파되었는지 확인하는
**적용 상태 snapshot**이다.

| Bit | 이름 | 의미 |
|---:|---|---|
| 0 | ECHO_PROFILE_READY | Echo 지연 profile을 현재 사용할 수 있음 |
| 1 | ECHO_PROFILE_BUSY | 채널별 Echo 지연 profile을 순차 전개 중 |
| 2 | RISE_PROFILE_VALID | Rise HSIZE/VSIZE/STRIDE 계산 결과가 유효함 |
| 3 | RISE_PROFILE_ENABLED | 현재 Active topology에서 Rise 출력 lane을 사용함 |
| 4 | FALL_PROFILE_VALID | Fall HSIZE/VSIZE/STRIDE 계산 결과가 유효함 |
| 5 | FALL_PROFILE_ENABLED | 현재 Active topology에서 Fall 출력 lane을 사용함 |
| 6 | PROCESSING_PIPELINE_IDLE | Processing pipeline에 진행 중 데이터가 없음 |
| 7 | GPX_AXIS_IDLE | GPX 결과 AXIS/CDC 출력에 진행 중 데이터가 없음 |
| 8 | GPX_CDC_RESET_BUSY | GPX CDC reset handshake 진행 중; 1이면 새 운용 시작 금지 |
| 15:9 | RESERVED | 0 |
| 31:16 | ECHO_PROFILE_VERSION | 전개 완료한 Echo profile의 Active version |

조회할 때 CTL23에 page 번호 `0x1B`만 쓰는 것이 아니라 CAPTURE bit를 더한
`0x0000011B`를 쓴다. 그 뒤 `BUSY=0`, `VALID=1`, `ERROR=0`, `SEQUENCE` 증가를
확인하고 CTL24를 읽는다.

Echo simulation build에서 CTL20 적용 완료 조건은 다음과 같다.

```text
STAT2.SHADOW_DIRTY             = 0
0x1B.ECHO_PROFILE_READY        = 1
0x1B.ECHO_PROFILE_BUSY         = 0
0x1B.ECHO_PROFILE_VERSION      = STAT3.ACTIVE_VERSION
```

Echo simulation이 합성에서 제외되면 READY=1, BUSY=0, VERSION=0이 정상이며,
이때 CTL20은 기능 경로에서 사용되지 않는다. Rise/Fall은 version field가 따로
없으므로 `VALID/ENABLED`와 `STAT3.ACTIVE_VERSION`, 그리고 필요하면
`0x1C..0x1F` geometry를 함께 확인한다.

#### TDC page

| Index | 이름 | Bit 의미 |
|---:|---|---|
| `0x80` | TDC_SUMMARY | 3:0 active Chip mask, 7:4 terminal mask, 10 TDC safe, 12 run enable, 13 active valid, 14 config ready, 15 physical-register-read error sticky |
| `0x84..0x87` | TDC_LANE_STATUS_0..3 | 0 initialized, 1 run active, 2 Shot outstanding, 3 controller busy, 4 bus busy, 5 response pending, 6 EF1, 7 EF2, 8 LF1, 9 LF2, 10 IRFLAG, 11 ERRFLAG, 14:12 effective bus ticks, 31:16 Chip Shot sequence |
| `0x88..0x8B` | TDC_LANE_FAULT_0..3 | 0 drain timeout, 1 sequence, 2 response mismatch, 3 raw drop, 4 drain cap, 5 register overflow, 6 run timeout, 9:7 timeout cause, 10 init/config coalesced, 11 command collision, 12 bus fatal |

합성된 Chip 수보다 큰 lane page는 0이다. 정의되지 않은 index는 `ERROR=1`,
`DIAG_DATA=0`을 반환한다. 자세한 소유권과 reset 계약은
`V2_CHECKPOINT_K0_8_STATUS_IRQ_SINGLE_OWNER_KO.md`를 따른다.

## 5. IRQ 레지스터

| 주소 | 이름 | 접근 | 의미 |
|---:|---|---|---|
| `0x100` | IRQ_ENABLE | R/W | source별 IRQ enable |
| `0x104` | IRQ_STATUS | RO | 동기화된 현재 source level |
| `0x108` | IRQ_FLAG | RO/W1C | pending event flag |
| `0x10C` | IRQ_MODE | R/W | `0=manual level`, `1=automatic one-clock pulse` |

| Bit | 이름 | 정확한 발생 조건 | 원인 확인 | 원인 제거/운용 조치 |
|---:|---|---|---|---|
| 0 | COMMIT_SUCCESS | COMMIT transaction이 `CFG_OK`로 완료된 1 CSR clock event | STAT2 `SUCCESS_STICKY`, active version | 완료 확인 후 IRQ_FLAG[0] W1C |
| 1 | COMMIT_ERROR | 승인된 COMMIT이 validation/prepare/activate/release 오류로 완료 | STAT2 `LAST_ERROR_CODE` | 설정 수정 또는 reset 복구 후 IRQ_FLAG[1] W1C |
| 2 | COMMIT_REJECTED | BUSY 등으로 새 COMMIT 요청 자체가 시작되지 못함 | STAT2 `LAST_REJECT_CODE` | 진행 transaction 종료 후 재요청, IRQ_FLAG[2] W1C |
| 3 | RECOVERY_REQUIRED | 두 clock domain의 동일 active version을 보장할 수 없는 원자 적용 실패 | STAT2 bit 5 | `CLEAR_STATUS`가 아니라 coordinated reset으로 active authority 복구 후 W1C |
| 4 | ACCESS_ERROR | 주소 정렬, reserved/write encoding, command ordering, 진단 portal 사용 오류 | STAT2 bit 7, CTL23 ERROR | software 순서/값 수정, 필요 시 CLEAR_STATUS 후 W1C |
| 5 | PROCESSING_WARNING | 위치 전이, virtual Z, Face overlap, **Shot 일정 초과**, monitor drop, laser request drop/abort/unexpected done | 진단 `0x10..0x18` | 원인별 속도/각도/상태 조치 후 CLEAR_STATUS, source=0 확인, W1C |
| 6 | LASER_TIMEOUT | fire_pulse 뒤 설정한 `FIRE_DONE_TIMEOUT` 안에 fire_done을 받지 못함 | 진단 `0x10[9]`, `0x16` | 레이저/배선/timeout 점검 후 CLEAR_STATUS, source=0 확인, W1C |
| 7 | ECHO_DIAGNOSTIC | 수신 window 밖 Echo, overlap, profile 미준비, snapshot timeout/abort | 진단 `0x20..0x26`, `0x40..0x5F` | Echo timing/profile/입력 점검 후 CLEAR_STATUS, source=0 확인, W1C |
| 8 | GPX_TRANSPORT | Shot/STOP CDC drop, IFIFO drain/sequence/controller/bus 오류, 실제 Register read 실패 | 진단 `0x10[24:26]`, `0x80[15]`, `0x88..0x8B` | GPX bus/clock/pin/timeout 점검 후 CLEAR_STATUS, source=0 확인, W1C |
| 9 | GPX_DATA | Raw28→Hit17 해석, Return/Cell/Face-slot 순서·geometry 계약 위반 | 진단 `0x1A` | topology/Return/데이터 원인 수정 후 CLEAR_STATUS, source=0 확인, W1C |
| 31:10 | 미구현 | 항상 0 | - | - |

pending flag는 enable이 0일 때도 event를 보존한다. manual mode의 IRQ는
software가 IRQ_FLAG에 W1C할 때까지 high이고, 새 event와 W1C가 같은 clock에
겹치면 event가 우선한다.

Source 0..2와 4는 완료/거부 시점의 event를 pending으로 보존한다. Source 3과
runtime source 5..9는 원인이 유지되는 level이다. 따라서 level source는 먼저
`CTL0.CLEAR_STATUS`를 보내고 IRQ_STATUS source가 0으로 내려온 뒤 IRQ_FLAG에
W1C한다. 단, source 3 `RECOVERY_REQUIRED`는 CLEAR_STATUS로 지워지지 않으며
coordinated reset이 필요하다. 구현은 동기화된 source의 상승 edge를 pending으로
보존하므로 원인이 여전히 high일 때 IRQ_FLAG부터 W1C하면 flag가 다시 자동으로
set되지 않을 수 있다. 이 순서를 바꾸지 않는다.

## 6. 오류 코드

| Hex | 의미 | Hex | 의미 |
|---:|---|---:|---|
| `00` | OK | `20` | runtime CPR |
| `01` | build PROC clock | `21` | total states |
| `02` | build TDC clock | `22` | virtual ticks |
| `03` | SYNC clock mismatch | `23` | virtual high count |
| `04` | chip count | `24` | Z parameter |
| `05` | STOP count | `25` | Face center |
| `06` | Return count | `26` | Face half-width |
| `07` | slope mask unknown | `27` | Face overlap |
| `08` | slope outside chips | `28` | Face enable mask |
| `09` | slope role missing | `29` | shot angle |
| `0A` | rise/fall balance | `2A` | shot below one state |
| `0B` | output width | `2B` | fire width |
| `0C` | Face count | `2C` | fire-done timeout |
| `0D` | Echo simulation topology | `2D` | range window |
| `2E` | capture window | `70` | internal arithmetic |
| `2F` | active chip mask | `71` | transaction busy |
| `30` | maximum Hits | `72` | PREPARE timeout |
| `31` | GPX bus timing | `73` | gateway protocol |
| `32` | physical/simulation source mode | `74` | ACTIVATE timeout |
| `33` | GPX Reg7.MTimer 표현 범위 초과 | `75` | RELEASE timeout |
| `76` | request-clear timeout |  |  |

## 7. 권장 software 순서

1. reset 해제 후 STAT0/1로 ABI와 build profile을 확인한다.
2. CTL1..20의 default shadow를 읽거나 필요한 값만 수정한다.
3. GPX image를 바꿀 때는 CTL21에서 `VIEW_ACTIVE=0`과 index를 선택하고
   CTL22에 28-bit data를 쓴다. 필요한 0..15 entry에 반복한다.
4. CTL0.COMMIT을 W1S한다.
5. STAT2.BUSY가 0이 되고 DONE_STICKY가 1이 될 때까지 기다린다.
6. ERROR/REJECTED/RECOVERY와 error code를 확인한다.
7. 성공이면 STAT3 version, STAT4..22 active source, STAT23..31 derived 값을
   같은 snapshot으로 읽는다. Echo simulation build는 Echo
   profile-ready/version도 같은 active version에 도달했는지 확인한다.
8. GPX image를 확인할 때는 CTL21의 `VIEW_ACTIVE=1`과 index를 선택한 뒤
   CTL22를 읽는다. 이 값은 성공한 COMMIT의 image이다.
9. 외부 permit과 STAT3.COMMAND_READY를 확인하고 CTL0.RUN을 W1S한다.
10. 다시 COMMAND_READY를 확인한 뒤 CTL0.ARM을 W1S하고 RUNNING/ARMED 및
    선택된 mode의 enable 상태를 확인한다.
11. 운용 진단은 CTL23에 `INDEX|CAPTURE`를 쓰고 BUSY/VALID/SEQUENCE를
    확인한 뒤 CTL24를 읽는다.
12. CTL0.CLEAR_STATUS로 원래 domain의 transaction/runtime sticky를 clear한다.
13. IRQ_STATUS source가 0으로 내려온 뒤 IRQ_FLAG를 W1C한다.

운용 중 GPX image 또는 목표 왕복시간을 바꿀 때는 다음 안전 절차를 사용한다.

1. `CTL0.DISARM`을 W1S하고 `STAT3.ARMED=0`, `SCHEDULER_ENABLE=0`,
   `PHYSICAL_FIRE_ENABLE=0`을 확인한다. RUN과 Encoder/Face 추적은 유지된다.
2. 목표 왕복시간만 바꾸면 CTL12만 수정한다. MTimer 이외의 GPX bit도 바꿀
   때만 CTL21/22 staging image를 추가 수정한다. Reg7.MTimer를 별도로 맞추지
   않는다.
3. COMMIT 후 STAT2의 성공과 GPX active image readback을 확인한다.
4. 외부 permit과 `COMMAND_READY=1`을 확인한 뒤 ARM을 다시 W1S한다.

요청 광학각 후보점에서 레이저 실행기 또는 GPX 획득기가 준비되지 않으면
발사를 늦추지 않고 해당 column을 **결측 Shot 열(Hole)**로 남긴다. Hole은
요청 각도 격자에는 존재하지만 그 시점에 측정하지 못한 column이며, 후속
H-Line 번호가 당겨지지 않도록 DDR에는 같은 HSIZE의 명시적 빈 Line으로
기록된다. 이 Runtime 시간 부족은
indexed diagnostic `PROC_FLAGS(0x10)[5]`, `PROC_OVERRUN_COUNT(0x13)` 및 IRQ
source 5 `PROCESSING_WARNING`으로 확인한다. IRQ 이름은 기존 ABI를 유지한
것이며, 이 bit가 나타내는 `schedule_overrun`은 요청 광학각의 Shot 시간 계약
오류이므로 단순 운용 안내로 무시하면 안 된다.

운용 중 shadow write는 허용된다. 진행 중 transaction은 COMMIT 순간 snapshot을
사용하고, 그 뒤의 write는 다음 transaction 후보로 남아 DIRTY가 유지된다.
