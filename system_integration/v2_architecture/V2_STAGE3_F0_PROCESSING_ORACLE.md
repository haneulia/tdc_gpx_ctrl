# v2 Stage 3 F0: Processing Event Oracle

## 1. 판정

Stage 3의 구현 순서는 `B0 -> B1 -> B2 -> B3`가 맞다. 단, v1의 내부
Motor AXIS 지연을 그대로 복제하는 것은 검증 목표가 아니다. v2는 다음 두
가지를 분리해서 검증한다.

1. 위치, Face, Shot, Laser 이벤트의 **순서와 데이터 의미**는 v1의 정상 운용
   계약과 정확히 비교한다.
2. 절대 지연은 v2의 직접 등록 경로에서 다시 측정하고, 숨은 보정 지연 없이
   상태값과 HTML에 기록한다.

이 문서는 Checkpoint F의 F0 진입 조건을 닫고 F1
`motor_position_core`의 비교 기준을 고정한다.

## 2. 고정한 v1 소스

상위 소스는 `V2_BASELINE.md`의 2026-08-04 해시와 일치한다.

| 소스 | SHA-256 |
|---|---|
| `motor_decoder/HDL/motor_decoder_top.vhd` | `D17B8757748FCB58040444E318EC44E053F6E26206B94BB590B482FF5588CC60` |
| `laser_ctrl/HDL/laser_ctrl_top.vhd` | `669EF21EBA00EC848CBC8BC9A5C011A4FF00FE12D9E1282941A22ADB8D3AC477` |
| `motor_laser_ctrl/HDL/motor_laser_ctrl_top.vhd` | `4754EF743B722C1FBB0529329BF3C7B32FF8E9A9D14F3C125899F41B71C65AF3` |

Stage 3에서 직접 참조하는 하위 소스도 다음 해시로 고정한다.

| 경계 | 소스 | SHA-256 |
|---|---|---|
| B0 | `quad_decoder.vhd` | `D9BF29793FFC00B431305F5A7C875DD125615F20E320164B631F6E4363111435` |
| B1 | `mirror_active_detect.vhd` | `389F00678C3FB09B4DEE1A7CCE63AB401C968450D0BCDB9A27E9000EE9E91CD2` |
| B0-B1 adapter | `motor_axis_stream_out.vhd` | `5BC844D313BC9F72F21D65F491614867A3B083B898E61A9BA1EA27257F256600` |
| B1-B2 adapter | `laser_ctrl_axis_in.vhd` | `1B43CE9004AB70B78668F2FC94A15D3D82B4BA89C5B588B26712F09698E77748` |
| B2 | `laser_ctrl_scheduler.vhd` | `BB1CED5E13B410BDCBE7F3C73DB6A0DC5BE889DD1B77088AD4391A7FE515EEBF` |
| B3 | `laser_ctrl_executor.vhd` | `C52EE521CED609F539E1F52AAC5D517D4D9E8529B531AE506DA551B629A3F4C5` |
| B3 physical T0 | `laser_ctrl_fire_done_bridge.vhd` | `0F2B3A31CDE3C362F259DCA01699B4A4BCAE8A98646CB167C9BD7A0147ECBF3A` |
| B3 pulse width | `laser_ctrl_tdc.vhd` | `55A1CA35E5B9A7373806B82EECEE1FFB293DB0E28A4C47B8E90889BD62FACF6A` |

경로 기준 위치는 `C:/Projects/my_sp/lib/IP`이다. 해시가 바뀐 소스는 이
문서의 oracle을 자동으로 갱신하지 않는다. 변경 이유와 재검증 결과를 먼저
기록해야 한다.

## 3. 비교 원칙

### 3.1 공통 샘플 기준

- 동기 이벤트는 `proc_aclk` 상승 에지 직후의 등록값을 비교한다.
- 테스트벤치의 `proc_cycle`은 reset 해제 후 첫 상승 에지를 0으로 한다.
- 외부 Encoder와 `fire_done`은 비동기 입력이므로 입력 핀의 절대 위상과
  동기화 완료 에지를 따로 기록한다.
- 물리 `start_tdc` 상승은 비동기 저지연 경로이므로 `proc_cycle`만으로
  비교하지 않고 시뮬레이션 시간과 원인 `fire_done` edge를 함께 기록한다.
- 설정 버전은 모든 B0..B3 이벤트에 같은 `active_version`으로 붙는다.

### 3.2 비교 등급

| 등급 | 적용 항목 | 판정 |
|---|---|---|
| Exact data | position, direction, Face, shot index, simulation flag | 값과 순서 완전 일치 |
| Exact registered cycle | B0..B3 내부 동기 이벤트 | 고정 위상 TB에서 cycle 일치 |
| Exact physical ordering | `fire_pulse`, raw `fire_done`, `start_tdc`, `shot_start`, `stop_tdc` | 선후관계와 pulse width 일치 |
| Measured latency | Encoder 입력부터 B0/fire, raw START부터 동기 T0 | v2 실측값과 상태값이 일치 |
| Intentional divergence | v1 AXIS backpressure 및 중복 runtime scheduler knobs | Section 8의 v2 규칙 적용 |

## 4. B0 Encoder Oracle

### 4.1 입력과 출력

`motor_position_core`는 물리 A/B/Z 또는 가상 A/B/Z 중 하나만 선택하고
다음 `position_event`를 한 클럭 pulse로 출력한다.

| 필드 | 비교 의미 |
|---|---|
| `valid` | 합법적인 decode event 또는 qualified Z event가 반영된 등록 이벤트 |
| `position` | `0 .. total_states-1` 원형 카운터 |
| `direction` | 실제 적용 방향, CW=증가, CCW=감소 |
| `source_sim` | 물리 0, 가상 1 |
| `source_latency_clks` | 선택 입력에서 B0 이벤트까지의 활성 보정값 |
| `source_latency_valid` | 보정값이 실측/승인되었는지 표시 |
| `z_event` | 선택된 Z의 qualified rising event |

### 4.2 보존 규칙

1. x1은 A rising, x2는 A 양 edge, x4는 합법적인 Gray-code 한 bit 변화를
   위치 이벤트로 사용한다.
2. CW 순서는 `00 -> 10 -> 11 -> 01 -> 00`, CCW는 역순이다.
3. CW에서 최대 위치 다음은 0, CCW에서 0 다음은 최대 위치다.
4. A와 B가 동시에 변한 transition은 위치를 움직이지 않고 진단만 누적한다.
5. Z rising은 위치를 0으로 맞추고 현재 방향은 유지한다.
6. 물리 방향 반전 설정은 물리 입력에 한 번만 적용한다. 가상 방향은 가상
   발생기의 A/B 순서로 결정한다.
7. 물리/가상 source 전환 첫 클럭은 이전 상태를 다시 seed하며 가짜 위치나
   Z event를 만들지 않는다.
8. 물리 입력은 명시적인 synchronizer를 통과한다. 가상 입력은 같은
   Processing clock의 등록 출력이며 별도 CDC를 만들지 않는다.

### 4.3 지연 기준

v1의 9클럭(physical), 5클럭(virtual)은 Encoder에서 **Motor AXIS
handshake**까지의 값이다. v2에는 이 제어 AXIS가 없으므로 이 숫자를 맞추기
위한 padding register를 넣지 않는다. F1/F5에서 다음을 각각 측정한다.

- physical pin transition -> B0 `valid`;
- virtual generator transition -> B0 `valid`;
- B0 `valid` -> accepted physical `fire_pulse`;
- 전체 Encoder transition -> accepted `fire_pulse`.

활성 latency 보정값은 지연 소자가 아니라 metadata이다. F5 실측 후 reset
default, 상태 readback, HTML 계약을 같은 값으로 갱신한다.

## 5. B1 Face Oracle

### 5.1 이벤트 필드

| 필드 | 비교 의미 |
|---|---|
| `valid` | B0 위치 이벤트와 정렬된 Face 판정 |
| `inside` | 현재 위치가 활성 Face의 inclusive 범위에 있음 |
| `enter_event` | 직전 traversal은 밖, 현재 이벤트는 안 |
| `exit_event` | 직전 traversal은 안, 현재 이벤트는 밖 |
| `face_index` | 현재 또는 방금 빠져나온 Face |
| `overlap` | 둘 이상의 Face가 동시에 일치한 진단 |
| `position`, `direction`, `source_sim` | B0와 정렬된 context |

### 5.2 경계 규칙

- 저장 geometry는 방향과 무관한 inclusive `[lower, upper]`이다.
- non-wrap은 `lower <= position <= upper`이다.
- wrap은 `position >= lower OR position <= upper`이다.
- CW 증가는 lower에서 진입하고 upper 다음 상태에서 이탈한다.
- CCW 감소는 upper에서 진입하고 lower 이전 상태에서 이탈한다.
- 방향 전환은 geometry를 바꾸지 않는다. Face 안에서 방향이 바뀌면 이전
  traversal의 `exit_event`와 새 traversal의 `enter_event`를 같은 B1
  이벤트에서 모두 1로 만든다.
- gap 없이 Face A에서 Face B로 직접 이동해도 `exit_event=1`,
  `enter_event=1`이며 `face_index`는 현재 Face B를 가리킨다.
- overlap은 commit validator가 차단한다. 방어용 runtime 진단에서는 v1과
  같이 가장 낮은 Face index를 선택하고 `overlap=1`을 보고한다.
- 비활성 Face와 build의 `num_faces` 밖 Face는 비교 대상에서 제외한다.
- B1은 B0 입력에서 1 Processing clock 뒤에 등록되며 매 클럭 연속 B0
  이벤트를 손실 없이 처리한다.

## 6. B2 Shot Scheduler Oracle

### 6.1 Shot lattice

Face의 두 경계는 모두 Face 검출에 포함되지만 레이저 lattice는 다음처럼
정의한다.

```text
face_angular_intervals = 2 * common_half_width
shot offsets           = 0, interval, 2*interval, ...
valid offset           < face_angular_intervals
columns_per_face       = ceil(face_angular_intervals / interval)
```

따라서 방향별 entry 위치는 첫 shot 후보이고 exit endpoint는 중복 shot이
아니다. CW와 CCW는 절대 위치 순서만 반대이며 `shot_index=0..N-1`은 항상
진행 시간순으로 증가한다.

### 6.2 요청과 차단

- `shot_request.valid`는 한 클럭 registered event이다.
- Face mask, F3a `scheduler_enable`, 유효 interval, executor ready가 모두 참인
  due grid point에서만 요청한다.
- due point에서 executor가 busy이면 해당 위치는 skip하고 overrun을
  기록한다. ready가 늦게 돌아와도 off-grid 위치에서 지연 발사하지 않는다.
- 이전 Face 요청의 accept/drop이 새 Face shot index를 오염시키지 않는다.
- 방향 반전 또는 Face 변경은 새 lattice의 `shot_index=0`으로 시작한다.
- monitoring AXIS의 `tready`는 요청 생성 조건에 포함되지 않는다.
- `shot_index`는 executor가 승인한 발사 횟수가 아니라 Face lattice의 기하학적
  열 번호이다. due point가 busy로 skip되어도 해당 열 번호는 소비하며, 다음
  요청은 건너뛴 수만큼 증가한 index를 전달한다.
- v1의 accepted-shot count 기반 index 압축은 의도적으로 보존하지 않는다.
  v2에서는 이 차이가 VDMA 열의 좌우 이동을 방지하는 데이터 무결성 계약이다.

## 7. B3 Laser Oracle

### 7.1 물리 모드

1. accepted request에서 `fire_pulse`를 설정 폭만큼 출력한다.
2. 정상적으로 LOW가 확인된 `fire_done` 입력만 새 물리 shot을 arm할 수 있다.
3. raw `fire_done` rising이 물리 T0이며, arm된 shot일 때만 물리
   `start_tdc`를 저지연으로 assert한다.
4. synchronized T0 event가 `shot_start`와 range-window bookkeeping을
   시작한다.
5. fire-done timeout까지 T0가 없으면 START/STOP 없이 timeout을 보고한다.
6. T0와 timeout boundary가 같은 경우 T0가 우선한다.
7. range window 완료 시 `stop_tdc`를 설정 폭만큼 출력한다.
8. range가 끝나도 `fire_pulse`가 아직 HIGH이면 다음 shot을 허용하지 않는다.

### 7.2 시뮬레이션 모드

- `fire_pulse`와 physical fire-done arm은 항상 0이다.
- simulation delay 후 동기 T0 event, `start_tdc`, `shot_start`를 한 shot
  context에서 생성한다.
- physical T0와 simulation T0는 동시에 유효할 수 없다.
- range window와 `stop_tdc` 의미는 물리 모드와 같다.

### 7.3 필수 assertion

- physical mode에서 matching `fire_done` 전 `start_tdc` 금지;
- simulation mode에서 `fire_pulse` 금지;
- 한 shot에 START와 STOP은 최대 한 번;
- START 없는 STOP 금지;
- timeout shot의 START/STOP 금지;
- pulse width는 active config snapshot과 정확히 일치;
- shot metadata는 accept부터 완료까지 안정;
- `shot_start`, Face index와 TDC acquisition command의 shot identity 일치.

## 8. v1에서 의도적으로 복제하지 않는 구조

| v1 항목 | v2 결정 | 이유 |
|---|---|---|
| Motor-to-Laser private AXIS | 직접 등록 event record | 제어 경로의 backpressure와 packing 제거 |
| `start_skip_steps` | 제거 | Face lower/upper가 활성 시작 위치를 소유 |
| `active_window_steps` | 제거 | `2 * common_half_width`에서 유일하게 도출 |
| `max_shots_per_face` | 제거 | `columns_per_face`가 유일한 shot 수 계약 |
| runtime `rearm_guard` | 제거 | 구현 고정 re-arm latency를 측정하여 read-only 계약으로 관리 |
| v1 9/5 latency padding | 제거 | v2 실제 직접 경로 지연을 측정하고 보고 |
| malformed/overlap config의 runtime fail-close 의존 | commit reject 후 기존 active 유지 | 잘못된 geometry가 기능 경로에 들어오지 않게 함 |

물리 mode의 실제 회전 속도는 CSR 설정만으로 확정할 수 없다. 따라서
`angular interval time >= fire_done response + range window + re-arm`을 모든
물리 운용에 대해 commit 시점에 증명한다고 주장하지 않는다.

- virtual mode는 설정된 ticks로 정적 예산을 계산할 수 있다.
- physical mode는 Z/revolution period 실측으로 가용 시간을 상태화한다.
- 두 mode 모두 due point에서 executor busy이면 발사를 차단하고 sticky
  overrun과 누적 count를 남긴다.

## 9. F0 고정 시나리오

| ID | 경계 | 시나리오 | 합격 조건 |
|---|---|---|---|
| P00 | B0 | reset 및 source 전환 | synthetic 위치/Z event 0 |
| P01 | B0 | x1/x2/x4 CW/CCW | 선택 edge, 방향, 원형 wrap 정확 |
| P02 | B0 | invalid two-bit transition | 위치 보존, 진단 1 증가 |
| P03 | B0 | Z rising | 위치 0, 방향 유지, event 1회 |
| P04 | B0 | physical/virtual latency | 실측값과 metadata 일치 |
| P10 | B1 | non-wrap inclusive window | lower/upper 포함, 밖은 제외 |
| P11 | B1 | low/high circular wrap | modular membership 정확 |
| P12 | B1 | CW/CCW 및 reversal | entry/exit와 Face context 정확 |
| P13 | B1 | 1..5 Faces와 mask | build/runtime mask 밖 event 0 |
| P20 | B2 | interval 1 및 N | exact shot position/index sequence |
| P21 | B2 | exit endpoint | 중복 endpoint shot 0 |
| P22 | B2 | busy due point | late fire 0, overrun 1 |
| P23 | B2 | zero-gap Face/reversal | old accept 오염과 ghost request 0 |
| P24 | B2 | enable/context 방어 | stale pre-ARM event, mid-Face ARM, overlap/mask fail-close |
| P25 | F3a+B1+B2 | 직접 record 연동 | RUN-only, ARM, permit loss/re-ARM과 next-entry request |
| P30 | B3 | physical success | fire, raw START, sync shot_start, STOP 순서 정확 |
| P31 | B3 | stale-high/missing fire_done | 물리 발사 차단 또는 timeout, false START 0 |
| P32 | B3 | timeout boundary T0 | T0 우선, shot 1회 |
| P33 | B3 | simulation | fire 0, sim START/shot_start/STOP 각 1회 |
| P34 | B3 | pulse widths 및 busy | 설정 폭 정확, 중복 accept 0 |

## 10. F0b 소스 모드 및 CSR ABI 보정

F0 oracle 검토에서 F1의 필수 입력인 물리/가상 source 선택이 active
configuration에 없고, 반대로 v1의 9/5-clock Motor AXIS latency가 writable
source로 남아 있음을 확인했다. 구현 순서가 뒤집히지 않도록 F1 전에 다음과
같이 보정한다.

| 항목 | F0b 결정 |
|---|---|
| `CTL1[20]` | `SIMULATION_MODE`: `0=physical`, `1=virtual/simulation` |
| `CTL1[31:21]` | Reserved, 반드시 0 |
| 기본 `CTL1` | `0x00020E10` |
| CSR ABI | `2.1`; STAT0 기본값 `0x3E250201` |
| v1 physical/virtual 9/5 clocks | writable 설정에서 제거; F1/F5에서 v2 경로를 다시 측정 |
| v2 latency readback | 실측값과 valid 조건이 생긴 뒤 read-only status로만 추가 |

이 변경은 지연을 삽입하거나 Processing 기능을 구현하는 변경이 아니다. F1이
소비할 source ownership을 먼저 고정하는 Stage 3 진입 ABI 보정이다. 따라서
package, sequential calculator, atomic manager/gateway, unified CSR의 기존
Stage 2 회귀를 모두 다시 통과해야 F0b가 닫힌다.

RUN/STOP/ARM/DISARM 및 외부 laser permit은 source mode와 다른 operation
상태다. header-only command를 F0b에 미리 만들지 않고 F3a의 단일 operation
owner와 fail-safe 테스트에서 확정한다. F3a가 닫히기 전에는 scheduler를
동작 가능 상태로 연결하지 않는다.

## 11. F1 진입 조건

F1은 다음 순서만 허용한다.

1. F0b CSR/configuration 회귀 완료;
2. Processing event record package 정의;
3. physical synchronizer와 edge-safe source selector;
4. x1/x2/x4 circular decoder;
5. virtual x4 A/B/Z source;
6. P00..P04 self-checking TB;
7. 150 MHz와 200 MHz Processing clock 기능 비교;
8. 합성/배치 후 latency 실측값 기록.

B0가 통과하기 전에는 Face tracker를 구현하지 않는다.

## 12. F0 완료 증거

2026-08-04에 F0a oracle 동결과 F0b source-mode ABI 보정을 다음 순서로
검증했다. 뒤 단계의 PASS가 앞 단계 실패를 가리지 않도록 각 script는 독립
세션과 PASS marker를 사용했다.

| 순서 | 회귀/세션 | 핵심 결과 |
|---:|---|---|
| 1 | `260804155150_v2_config_pkg` | reference validation 및 error code `0x32` PASS |
| 2 | `260804155213_v2_commit_calculator` | Processing 150 MHz 및 200 MHz calculator PASS; WNS `+0.957/+0.379 ns`, latch 0 |
| 3 | `260804155514_v2_config_manager` | 150/200 및 200/150 atomic gateway PASS; WNS `+1.044/+1.126 ns`, Critical CDC 0, ASYNC_REG 28 |
| 4 | `260804160515_v2_unified_csr` | map/bank/active readback와 양 clock profile PASS; WNS `+0.825/+0.814 ns`, latch 0, Critical CDC 0, ASYNC_REG 92 |

마지막 CSR 통합 시나리오는 `CTL1=0x00120E10`을 commit하고 manager,
Processing/TDC active snapshot과 STAT4가 모두 같은 값을 갖는지 확인한다.
그 다음 잘못된 CPR commit은 `CFG_RUNTIME_CPR`로 끝나며 승인된 source mode와
active version을 바꾸지 않는다. `CTL1[31:21]` 또는 decode `11` write도 기존
shadow를 보존하면서 access error를 남긴다.

CDC-15 mailbox warning은 두 profile 모두 기존 atomic mailbox payload에 대한
동일한 2106건이며, protocol synchronizer는 14개로 인식되고 Critical CDC는
0이다. F0b에서 새 crossing이나 latency padding은 추가되지 않았다.

**판정:** F0a/F0b 완료. Checkpoint F 자체는 아직 진행 중이며 다음 허용 단계는
F1 `motor_position_core`뿐이다.
