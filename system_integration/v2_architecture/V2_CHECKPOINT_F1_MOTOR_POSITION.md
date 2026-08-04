# v2 Checkpoint F1: Motor Position Core

## 1. 판정

Stage 3 / Checkpoint F의 첫 기능 경계인 F1 `motor_position_core`를 완료했다.
F0a에서 고정한 B0 oracle과 F0b의 active source-mode 계약을 그대로
사용했으며, F1보다 뒤의 Face, Shot, Laser 기능은 구현하지 않았다.

**결론:** B0는 통과했다. Checkpoint F 전체는 아직 진행 중이며 다음에
허용되는 단계는 F2 `face_tracker`뿐이다.

위 순서는 F1 완료 당시 gate 기록이다. 현재 Checkpoint F는 F5까지 완료됐으며
최종 상태는 `V2_CHECKPOINT_F5_PROCESSING_SUBSYSTEM.md`를 따른다.

## 2. 구현 경계

| 파일 | 소유 책임 |
|---|---|
| `pkg/lidar_event_types_pkg.vhd` | `position_event_t`, 지연 및 가상 Z 진단 형식 |
| `rtl/proc/motor_virtual_index.vhd` | 가상 Z qualification, offset/width FSM과 fault |
| `rtl/proc/motor_virtual_source.vhd` | fractional timing, 가상 x4 A/B, 원형 위치와 Z trigger |
| `rtl/proc/motor_position_core.vhd` | 물리 4FF CDC, source 선택, x1/x2/x4 decode, B0 이벤트 |

코어는 CSR word를 해석하지 않는다. `lidar_active_config_t`만 소비하며,
event payload와 `valid`를 같은 Processing clock에서 함께 등록한다. AXIS는
제어 경로에 없고 F5에서 관측용 tap으로 추가됐다.

## 3. B0 계약

`position_event_t`는 다음 정보를 한 번에 전달한다.

| 필드 | F1 적용 의미 |
|---|---|
| `valid` | 합법적인 위치 전이 또는 qualified Z의 1-clock pulse |
| `position` | `0 .. total_states-1` 원형 위치 |
| `direction` | 실제 적용된 CW/CCW 방향 |
| `source_sim` | 물리 `0`, 가상 `1` |
| `source_latency_clks` | 승인된 B0 입력-to-event clock 수 |
| `source_latency_valid` | F1 실측 계약이 유효함을 나타내는 `1` |
| `z_event` | 선택 source의 qualified Z rising |
| `active_version` | 해당 이벤트가 사용한 atomic configuration version |

물리 입력 A/B/Z는 각각 4FF synchronizer를 통과한다. source 변경은
configuration quiesce 동안 새 source 값으로 previous sample을 다시 seed하므로
가짜 위치/Z event를 만들지 않는다. 위치는 관련 없는 runtime configuration
commit만으로 0에 초기화하지 않으며, 축소된 `total_states` 밖에 있을 때만 새
최대값으로 clamp한다.

## 4. 지연 정의

v1의 9/5 clock은 Motor AXIS handshake까지 포함한 값이므로 v2에 padding으로
복제하지 않았다. F1은 B0 경계 자체를 다음처럼 측정했다.

| Source | 시작 edge | B0 `valid` | 승인값 |
|---|---|---|---:|
| Physical | 안정된 비동기 핀 값을 첫 번째 FF가 샘플한 Processing 상승 edge | 네 번째 후속 Processing edge | 4 clocks |
| Virtual | 가상 source가 등록 A/B/Z 값을 변경한 Processing edge | 다음 Processing edge | 1 clock |

비동기 핀 전이의 절대 시점부터 첫 샘플 edge까지는 위상에 따라 달라지므로
4-clock metadata에 포함하지 않는다. 이 정의는 package 상수, TB 비교와 향후
read-only status가 공유한다.

## 5. 기능 검증

`tb_motor_position_core`는 150 MHz와 200 MHz에서 같은 P00..P04 벡터를
실행한다.

| ID | 검증 내용 | 결과 |
|---|---|---|
| P00 | reset, physical/simulation 전환, synthetic event 금지 | PASS |
| P01 | physical x1/x2/x4, CW/CCW, 설치 방향 반전, 원형 wrap | PASS |
| P02 | `00 -> 11` 불법 전이, 위치 보존, sticky/count, clear | PASS |
| P03 | physical Z rising 우선, position 0, 방향 보존 | PASS |
| P04 | virtual x4 CW/CCW, 첫 revolution Z guard, 두 번째 Z, 4/1-clock latency | PASS |

최종 PASS marker는 다음 두 개다.

```text
LIDAR_V2_MOTOR_POSITION_PASS proc_mhz=150
LIDAR_V2_MOTOR_POSITION_PASS proc_mhz=200
```

## 6. Timing 개선과 최종 결과

첫 route는 가상 Z의 32-bit `ticks_next` 비교, 폭 clamp와 상태 counter load가
한 경로에 합쳐져 150 MHz에서도 WNS `-0.493 ns`였다. 계산을 미리 등록하고
delay/active counter를 분리한 뒤 병목이 fractional scheduler로 이동했다.
LO/HI `interval-1`을 configuration quiesce에서 등록하고 Z clamp와 15-bit
active-count 연산을 분리해 최종 timing을 닫았다.

최종 보관 세션:
`signoff_results/sessions/260804170100_v2_motor_position`

| Processing clock | WNS | Latch | ASYNC_REG | Critical CDC |
|---:|---:|---:|---:|---:|
| 150 MHz | `+1.763 ns` | 0 | 12 | 0 |
| 200 MHz | `+0.458 ns` | 0 | 12 | 0 |

OOC `report_cdc`는 source clock이 없는 비동기 입력 port를 분석 대상에서
제외한다. 따라서 물리 CDC의 근거는 코드의 4FF/`ASYNC_REG` 계약, 합성 후
12개 속성 보존, 기능 latency 시험을 함께 사용한다. 실제 pin clock/input
constraint를 포함한 최종 CDC 판정은 parent 통합 release gate에서 다시 한다.

## 7. 재현 명령

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_motor_position.ps1
```

스크립트는 P00..P04 marker, WNS, latch, ASYNC_REG와 Critical CDC를 모두
검사한다. 실패 세션은 PASS 근거로 보관하지 않으며, 최종 세션에는 source
manifest와 필요한 log/report만 남긴다.

## 8. 다음 게이트

F2는 이 `position_event_t`를 입력으로 사용하는 `face_tracker`만 구현한다.
F2가 B1의 1..5 Face, inclusive/wrap 경계와 CW/CCW traversal을 통과하기 전에는
F3a operation/safety owner 또는 F3b scheduler를 시작하지 않는다.
