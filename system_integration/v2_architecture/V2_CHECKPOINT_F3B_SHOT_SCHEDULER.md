# v2 Checkpoint F3b: Shot Scheduler

## 1. 판정

Stage 3 / Checkpoint F의 F3b `shot_scheduler`와 B2 경계를 완료했다.
Face 진입점을 기준으로 고정된 각도 격자를 만들고, executor가 바쁜 due
point는 늦게 재발사하지 않고 해당 열을 비운 채 overrun으로 기록한다.

**결론:** B2는 통과했다. 다음 허용 단계는 F4 `laser_executor`이다.
Checkpoint F 전체, 통합 IP, VDMA 및 HTML 정합은 아직 완료 판정이 아니다.

## 2. 시스템 역할

`shot_scheduler`는 다음 한 가지 결정을 소유한다.

> 현재 Face traversal의 현재 위치가 발사 가능한 기하학적 격자점인지 판단하고,
> F3a가 허가했으며 executor가 받을 수 있을 때 한 클럭 요청을 만든다.

이 블록은 다음 상태를 소유하지 않는다.

- RUN/STOP, ARM/DISARM, 외부 laser permit;
- 물리 `fire_pulse`, `fire_done`, `start_tdc`, `stop_tdc` 수명주기;
- CSR word 해석과 시간/각도 나눗셈;
- AXIS `tready` 기반 흐름 제어;
- VDMA line 저장과 Ethernet 전송.

운용 허가는 F3a `operation_state.scheduler_enable`만 사용한다. source mode,
ACTIVE_VALID 또는 Face membership를 조합해 별도 laser permission을 재구성하지
않는다.

## 3. v1 비교와 의도적 변경

비교 기준은
`C:/Projects/my_sp/lib/IP/laser_ctrl/HDL/laser_ctrl_scheduler.vhd`이다.

| 항목 | v1 | v2 F3b |
|---|---|---|
| 격자 | position-event count와 interval countdown | 동일한 countdown 의미 보존 |
| due point busy | skip 후 sticky | skip, pulse/sticky/32-bit count |
| 요청 | 1-clock pulse와 unresolved guard | typed 1-clock record와 one-entry ownership |
| Face 전환 | epoch로 이전 accept 오염 방지 | 요청 index를 accept count와 분리하여 오염 원인 제거 |
| `start_skip_steps` | runtime knob | 제거, Face entry가 offset 0 |
| `active_window_steps` | runtime knob | 제거, `face_angular_intervals`에서 유일하게 파생 |
| `max_shots_per_face` | runtime knob | 제거, `columns_per_face`가 유일한 열 수 |
| shot index | 승인된 요청 수 | 기하학적 Face lattice/VDMA 열 번호 |
| laser enable/error | B2에서 다시 조합 | F3a `scheduler_enable`만 소비 |

가장 중요한 의도적 차이는 `shot_index`이다. v1에서는 첫 due point가 busy로
누락되면 다음 승인 요청이 다시 index 0이 될 수 있었다. v2는 누락된 due point도
열 번호를 소비하므로 다음 요청은 index 1이다. 이후 formatter가 이 번호를
보존하면 누락이 뒤의 모든 스캔점을 왼쪽으로 이동시키지 않는다.

## 4. 입력과 출력 계약

### 4.1 입력

| 입력 | 출처 | 의미 |
|---|---|---|
| `i_enable` | F3a operation manager | 새 shot 요청 허가 |
| `i_active_config` | Processing config gateway | interval, columns, Face mask, source mode, version |
| `i_face_event` | B1 `face_tracker` | Face traversal과 위치 문맥 |
| `i_executor_ready` | F4 executor | 현재 due point를 받을 수 있음 |
| `i_request_accept/drop` | F4 executor | 직전 요청의 상호 배타적 resolve |
| `i_clear_diagnostics` | 통합 status owner | overrun sticky/count clear |

설정값은 `i_enable=0`인 quiescent 구간에서만 지역 레지스터로 복사한다.
활성 중 version이 바뀌면 assertion으로 실패한다.

### 4.2 `shot_request_t`

| 필드 | 의미 |
|---|---|
| `valid` | 한 클럭 등록 요청 |
| `face_index` | 요청을 소유한 Face |
| `position` | 격자 판단에 실제 사용한 decoded state |
| `direction` | 해당 traversal의 실제 진행 방향 |
| `shot_index` | Face 내 기하학적 열 번호, `0..columns_per_face-1` |
| `last_in_face` | 이 요청이 마지막 기하학적 열임 |
| `source_sim` | 물리/가상 executor 선택 문맥 |
| `source_latency_*` | B0 입력 지연 metadata, 지연 삽입값이 아님 |
| `active_version` | 요청을 만든 atomic configuration version |

요청이 발생하면 `inflight=1`, `o_idle=0`이 된다. `accept` 또는 `drop` 중
하나가 들어오면 ownership을 해제한다. 둘이 동시에 1이면 contract failure다.

## 5. Shot lattice 규칙

Commit calculator가 미리 계산한다.

```text
face_angular_intervals = 2 * common_half_width
shot_interval_states   = ceil(requested_angle / optical_angle_per_state)
columns_per_face       = ceil(face_angular_intervals / shot_interval_states)
```

B2 실시간 경로에는 division과 modulo가 없다. Face event가 들어올 때
16-bit compare, decrement와 increment만 수행한다.

Face detection은 양 경계를 포함하지만 shot lattice는 `[entry, exit)`이다.
예를 들어 Face가 `[10..16]`, interval이 2라면 다음과 같다.

| 진행 state | Face 판정 | 격자 offset | 결과 | `shot_index` |
|---:|---|---:|---|---:|
| 10 | inside + enter | 0 | request | 0 |
| 11 | inside | 1 | 대기 | - |
| 12 | inside | 2 | request | 1 |
| 13 | inside | 3 | 대기 | - |
| 14 | inside | 4 | request, last | 2 |
| 15 | inside | 5 | 완료 후 무시 | - |
| 16 | inclusive Face endpoint | 6 | 중복 endpoint shot 없음 | - |

CCW에서는 절대 위치가 `16, 15, 14, ...`로 감소하지만 요청 index는 진행
시간순으로 항상 `0, 1, 2`로 증가한다. 저장된 lower/upper를 바꾸지 않는다.

## 6. 경계와 안전 동작

### 6.1 ARM과 Face entry

- `i_enable=0`이면 traversal session을 닫는다.
- `i_enable` 상승 뒤 B1의 두 등록 단계에 남아 있던 event를 2클럭 격리한다.
- Face 중간에서 ARM되어 `i_enable=1`이 되어도 즉시 발사하지 않는다.
- 다음 `face_event.enter_event=1`이 들어와야 새 lattice의 column 0을 연다.
- permit 상실 뒤 재-ARM도 동일하다.

이 규칙은 ARM 순간의 임의 위치뿐 아니라 ARM 전에 B1에 들어와 늦게 출력된
stale entry를 Face 시작점으로 오인하는 발사를 막는다. 2클럭은 runtime knob가
아니라 현재 B1의 고정 등록 깊이에 종속된 implementation constant다.

### 6.2 Face 변경과 방향 반전

`exit_event=1`과 `enter_event=1`이 같은 B1 event에 함께 들어올 수 있다.
새 Face 또는 새 방향 traversal은 같은 위치에서 column 0으로 시작한다.
이전 요청이 unresolved라면 새 column 0은 overrun으로 비워지며, 이전 요청의
늦은 accept가 새 Face index를 감소시키거나 증가시키지 않는다.

### 6.3 Busy due point

```text
column 0 due, executor busy -> request 없음, overrun=1, next_column=1
off-grid ready             -> request 없음
column 1 due, executor ready -> request(index=1)
```

ready가 돌아온 시점에 늦게 발사하지 않는다. 따라서 각도는 보존되고 누락은
명시적인 빈 열이 된다. overrun count는 32-bit modulo counter이며 overflow가
나도 sticky와 현재 누적값을 software가 관측할 수 있다.

## 7. RTL 구조

| 블록 | 소유 상태 | 특징 |
|---|---|---|
| `p_config` | interval, columns, mask, source mode, version | 비활성 구간에만 snapshot |
| `p_scheduler` | session, Face, countdown, next column, inflight, request | 117줄, 단일 sequential owner |
| `fn_context_is_valid` | 없음 | 작은 방어용 pure check |
| `fn_make_request` | 없음 | payload와 valid를 함께 조립하는 pure transform |
| `p_diagnostics` | sticky/count | clear와 새 event 동시이면 새 event 우선 |
| `p_contract` | 없음 | version, mode, handshake와 geometry assertion |

주 상태 프로세스는 RTL style review 기준 120줄 이내다. latch, RAM, DSP는
추론되지 않았다.

### 7.1 Local assertion

| ID | 차단하는 오류 |
|---|---|
| `V2-SCHED-001` | 불법 build topology |
| `V2-SCHED-002` | accept와 drop 동시 assertion |
| `V2-SCHED-003` | active config 없이 scheduler enable |
| `V2-SCHED-004` | 활성 중 config version 변경 |
| `V2-SCHED-005` | interval 또는 columns가 0인 파생 geometry |
| `V2-SCHED-006` | Face event와 local config version 불일치 |
| `V2-SCHED-007` | Face event와 local source mode 불일치 |
| `V2-SCHED-008` | request valid인데 in-flight ownership 없음 |
| `V2-SCHED-009` | in-flight 요청이 없는데 resolve 수신 |

## 8. 기능 및 연동 검증

`tb_shot_scheduler`와 `tb_v2_processing_control_chain`을 150 MHz, 200 MHz에서
각각 실행했다.

| ID | 범위 | 시나리오 | 결과 |
|---|---|---|---|
| P20 | B2 | interval 1/N, exact position/index/last 순서 | PASS |
| P21 | B2 | inclusive exit endpoint 중복 shot 금지 | PASS |
| P22 | B2 | busy due skip, off-grid late fire 금지, index hole | PASS |
| P23 | B2 | zero-gap Face, unresolved 이전 요청, 방향 반전 | PASS |
| P24 | B2 | enable quarantine, mid-Face ARM, mask/overlap fail-close, 진단 clear | PASS |
| P25 | F3a+B1+B2 | RUN-only, ARM, permit loss, re-ARM, next entry | PASS |

P25는 production 통합 top을 미리 만들지 않고 세 검증 블록을 test-only로 직접
연결한다. B0 event sampling edge에서 B2 request까지 두 Processing clock의
등록 경계를 확인했다. Encoder pin부터의 전체 latency는 F5에서 다시 측정한다.

## 9. 구현 결과

최종 세션:
`signoff_results/sessions/260804211500_v2_shot_scheduler`

| Processing clock | WNS | Latch | ASYNC_REG | Critical CDC |
|---:|---:|---:|---:|---:|
| 150 MHz | `+1.848 ns` | 0 | 0 | 0 |
| 200 MHz | `+0.495 ns` | 0 | 0 | 0 |

200 MHz route 기준 자원은 LUT 140, FF 190, BRAM 0, DSP 0이다. B2는 단일
Processing clock이므로 ASYNC_REG 0이 정상이다. OOC `HD.CLK_SRC`와 Zynq
parent 관련 warning은 최종 parent 통합에서 다시 판정한다.

공용 event package 확장 후 무회귀 결과는 다음과 같다.

| 경계 | Session | 150 MHz WNS | 200 MHz WNS | Latch | Critical CDC |
|---|---|---:|---:|---:|---:|
| F1/B0 | `260804210100_v2_motor_position` | `+1.763 ns` | `+0.458 ns` | 0 | 0 |
| F2/B1 | `260804210200_v2_face_tracker` | `+2.736 ns` | `+1.126 ns` | 0 | 0 |
| F3a | `260804210300_v2_operation` | `+4.349 ns` | `+2.638 ns` | 0 | 0 |

## 10. 재현 명령

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_shot_scheduler.ps1
```

스크립트는 150/200 MHz P20..P25 marker, route WNS, latch와 Critical CDC를
검사하고 source manifest를 보관한다. wave database와 임시 Vivado work는
PASS 증거에 포함하지 않는다.

## 11. 남은 통합 위험과 다음 단계

F3b 완료가 다음을 증명하지는 않는다.

1. F4가 request를 정확히 accept/drop하고 물리/가상 수명주기를 분리하는지;
2. `fire_pulse -> fire_done -> start_tdc -> stop_tdc` 순서와 timeout;
3. B3 이후 TDC acquisition이 같은 Face/index/version을 유지하는지;
4. frame/VDMA formatter가 busy로 비어 있는 `shot_index`를 압축하지 않는지;
5. 최종 AXIS monitor가 제어 경로에 backpressure를 걸지 않는지;
6. HTML이 overrun과 빈 열을 같은 의미로 표현하는지.

따라서 다음 단계는 F4 `laser_executor` 단독 경계이다. F4가 통과한 뒤 F5에서
B0..B3 직접 event path, 통합 idle, monitoring AXIS와 end-to-end latency를
검증한다.
