# TDC-GPX LiDAR v2 통합 CSR 레지스터 맵

## 1. 적용 범위

이 문서는 `lidar_csr_bank`의 AXI4-Lite software ABI를 정의한다. 주소는
9-bit byte address이고 data width는 32 bit이다. 현재 ABI는 `2.3`이다.

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
| `0x030` | TARGET_RANGE | R/W | `00000120` | target round-trip window |
| `0x034` | TDC_PULSE_WIDTHS | R/W | `00050005` | START/STOP pulse width |
| `0x038` | SIM_START_DELAY | R/W | `00000085` | simulation START delay |
| `0x03C` | SHOT_INTERVAL | R/W | `0000C350` | 요청 optical shot interval |
| `0x040` | TDC_BUS_PROFILE | R/W | `000FF142` | GPX bus/mask/Return 설정 |
| `0x044` | TDC_START_OFFSET | R/W | `000004D2` | GPX START offset |
| `0x048` | TDC_SCAN_TIMEOUT | R/W | `00000000` | scan watchdog source time |
| `0x04C` | TDC_CAPTURE_ADJUST | R/W | `00000000` | signed board capture 보정 |
| `0x050` | ECHO_DELAY_PROFILE | R/W | `00000000` | CH0 지연과 채널 증가분 |
| `0x054..0x07C` | RESERVED | RO-zero | `00000000` | 쓰면 access error |

### 3.2 CTL0 COMMAND

| Bit | 이름 | 의미 |
|---:|---|---|
| 0 | COMMIT | 현재 shadow를 snapshot하여 검증/계산/atomic activation 시작 |
| 1 | CLEAR_STATUS | STAT2 sticky와 last error/reject code clear |
| 2 | SOFT_RESET_REQUEST | 외부 reset supervisor로 1 CSR clock request 출력 |
| 3 | RUN | Processing operation을 RUN 상태로 전환 |
| 4 | STOP | RUN과 ARM을 함께 해제하여 안전 정지 |
| 5 | ARM | shot/laser 실행을 허가하도록 ARM 요청 |
| 6 | DISARM | RUN은 유지하고 shot/laser 실행만 차단 |
| 31:7 | RESERVED | 1을 쓰면 command 전체를 실행하지 않고 access error |

`0` write는 no-op이다. 한 write에서 command bit를 둘 이상 세우면 어느
command도 실행되지 않는다. `COMMIT`을 BUSY 중 다시 쓰면 manager가
`CFG_TRANSACTION_BUSY`로 reject한다.

`CLEAR_STATUS`는 active 설정, shadow, `SHADOW_DIRTY`, manager의
`RECOVERY_REQUIRED` 및 IRQ pending flag를 지우지 않는다. IRQ pending은
IRQ_FLAG에 W1C해야 한다. `SOFT_RESET_REQUEST`도 이 block 자체를 reset하지
않으며 parent reset supervisor가 실제 coordinated reset을 수행해야 한다.

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
확정된 값은 B0-to-accept 4 clocks, physical first-sample-to-fire 8 clocks,
virtual transition-to-accept 5 clocks이다. 현재 Checkpoint E의 고정 32-STAT ABI에는
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
| TARGET_RANGE | 31:0 | 5 ns ticks | target round-trip/수신 window |
| TDC_PULSE_WIDTHS | 15:0 | 5 ns ticks | START_TDC width |
| TDC_PULSE_WIDTHS | 31:16 | 5 ns ticks | STOP_TDC width |
| SIM_START_DELAY | 31:0 | 5 ns ticks | simulation START delay |
| SHOT_INTERVAL | 29:0 | micro-degree | 요청 optical point 간격 |
| SHOT_INTERVAL | 31:30 | - | Reserved, 반드시 0 |

5 ns tick 값은 runtime source ABI이다. commit calculator가 build의
`PROC_CLK_MHZ` 또는 `TDC_CLK_MHZ`에 맞춰 필요한 domain clock 수로 한 번만
변환한다. `FIRE_DONE_TIMEOUT <= TARGET_RANGE`여야 한다.

### 3.6 CTL16..CTL19 TDC-GPX

| 레지스터 | Bit | 단위 | 의미 |
|---|---:|---|---|
| TDC_BUS_PROFILE | 5:0 | TDC clocks | GPX bus divider |
| TDC_BUS_PROFILE | 11:6 | divided-bus ticks | bus timing ticks |
| TDC_BUS_PROFILE | 15:12 | mask | runtime active chip mask |
| TDC_BUS_PROFILE | 16 | boolean | falling path enable |
| TDC_BUS_PROFILE | 19:17 | Hits | maximum Returns/STOP, 1..build max |
| TDC_BUS_PROFILE | 31:20 | - | Reserved, 반드시 0 |
| TDC_START_OFFSET | 17:0 | TDC clocks | GPX START timing offset |
| TDC_START_OFFSET | 31:18 | - | Reserved, 반드시 0 |
| TDC_SCAN_TIMEOUT | 31:0 | 5 ns ticks | scan watchdog source |
| TDC_CAPTURE_ADJUST | 16:0 | signed 5 ns ticks | target range에 더하는 board 보정 |
| TDC_CAPTURE_ADJUST | 31:17 | - | Reserved, 반드시 0 |

`TDC_CAPTURE_ADJUST`는 17-bit two's-complement field이다. 음수도 상위
reserved bit를 sign-extension하지 않고 `[16:0]`에만 기록한다.

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

## 4. STAT 레지스터

### 4.1 STAT0 CORE_INFO (`0x080`)

| Bit | 의미 | 기본 profile |
|---:|---|---:|
| 7:0 | ABI minor | 3 |
| 15:8 | ABI major | 2 |
| 18:16 | NUM_FACES | 5 |
| 21:19 | NUM_CHIPS | 4 |
| 25:22 | STOPS_PER_CHIP | 8 |
| 28:26 | MAX_RETURNS_PER_STOP | 7 |
| 29 | ECHO_RECEIVER_INCLUDED | 1 |
| 30 | ECHO_SIMULATION_INCLUDED | 0 |
| 31 | STREAM_CLOCK_SYNC | 0 |

기본 profile 값은 `0x3E250203`이다.

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
| `0x0F8` | CAPTURE_TDC_CLKS | 31:0 | `(TARGET_RANGE+ADJUST)`의 TDC clock 변환값 |
| `0x0FC` | DERIVED_MASKS | 3:0 | PRESENT_CHIP_MASK |
|  |  | 7:4 | ACTIVE_RISE_MASK |
|  |  | 11:8 | ACTIVE_FALL_MASK |
|  |  | 15:12 | Reserved |
|  |  | 31:16 | 완료 transaction count, modulo 65536 |

Face bounds는 저장된 geometry이며 CW/CCW에 따라 다시 쓰지 않는다.
CW/CCW는 어느 bound가 Entry/Exit인지와 traversal 방향만 바꾼다.

## 5. IRQ 레지스터

| 주소 | 이름 | 접근 | 의미 |
|---:|---|---|---|
| `0x100` | IRQ_ENABLE | R/W | source별 IRQ enable |
| `0x104` | IRQ_STATUS | RO | 동기화된 현재 source level |
| `0x108` | IRQ_FLAG | RO/W1C | pending event flag |
| `0x10C` | IRQ_MODE | R/W | `0=manual level`, `1=automatic one-clock pulse` |

| Source bit | Event |
|---:|---|
| 0 | COMMIT_SUCCESS |
| 1 | COMMIT_ERROR |
| 2 | COMMIT_REJECTED |
| 3 | RECOVERY_REQUIRED rising edge |
| 4 | CSR_ACCESS_ERROR |
| 31:5 | 구현되지 않음, 0 |

pending flag는 enable이 0일 때도 event를 보존한다. manual mode의 IRQ는
software가 IRQ_FLAG에 W1C할 때까지 high이고, 새 event와 W1C가 같은 clock에
겹치면 event가 우선한다.

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
| `75` | RELEASE timeout | `76` | request-clear timeout |

## 7. 권장 software 순서

1. reset 해제 후 STAT0/1로 ABI와 build profile을 확인한다.
2. CTL1..20의 default shadow를 읽거나 필요한 값만 수정한다.
3. CTL0.COMMIT을 W1S한다.
4. STAT2.BUSY가 0이 되고 DONE_STICKY가 1이 될 때까지 기다린다.
5. ERROR/REJECTED/RECOVERY와 error code를 확인한다.
6. 성공이면 STAT3 version, STAT4..22 active source, STAT23..31 derived 값을
   같은 snapshot으로 읽는다. Echo simulation build는 Echo
   profile-ready/version도 같은 active version에 도달했는지 확인한다.
7. 외부 permit과 STAT3.COMMAND_READY를 확인하고 CTL0.RUN을 W1S한다.
8. 다시 COMMAND_READY를 확인한 뒤 CTL0.ARM을 W1S하고 RUNNING/ARMED 및
   선택된 mode의 enable 상태를 확인한다.
9. CTL0.CLEAR_STATUS로 transaction sticky를 clear한다.
10. IRQ를 사용하면 별도로 IRQ_FLAG를 W1C한다.

운용 중 shadow write는 허용된다. 진행 중 transaction은 COMMIT 순간 snapshot을
사용하고, 그 뒤의 write는 다음 transaction 후보로 남아 DIRTY가 유지된다.
