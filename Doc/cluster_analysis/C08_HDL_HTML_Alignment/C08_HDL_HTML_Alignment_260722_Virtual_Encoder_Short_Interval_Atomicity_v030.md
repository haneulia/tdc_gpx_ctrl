# C08 Virtual Encoder Short-Interval Atomicity v030

## 1. 목적

Virtual Encoder 최종 구조 감사에서 발견된 최소 phase interval 경계 문제를
해결하고, 활성 RTL의 추가 분리/최적화 여부를 최종 판정한다.

## 2. 발견된 문제

런타임 설정은 등록된 `wrap_event`를 받은 뒤 active epoch와 apply pulse로
전달된다. 정상 모터 조건에서는 phase 간격이 수백 clock이므로 문제가 없지만,
`ticks_lo/hi=1~2`에서는 이 전달 시간보다 다음 phase tick이 빨리 올 수 있다.

기존 순서는 다음과 같았다.

| 순서 | 동작 |
|---|---|
| E0 | position이 마지막 state에서 0으로 wrap, `wrap_event` 등록 |
| E1 | parameter controller가 pending epoch를 active로 이동 |
| E2 | phase/position/Z consumer가 apply pulse를 관찰 |

E1 또는 E2에 raw phase tick이 들어오면 position이 0에서 먼저 1로 진행한 뒤
새 방향이 적용될 수 있었다. `ticks=1`, CW에서 CCW로 변경하는 red test는
다음과 같이 실패했다.

```text
Failure: position advanced between wrap and atomic apply
Time: 95001 ps
```

## 3. 수정 방식

새 counter나 generic을 추가하지 않고 기존 상태에서 1-bit hold를 조합한다.

```text
phase_hold = apply_pulse OR
             (wrap_event AND (pending_request OR new_request))

phase_advance = raw_phase_tick AND NOT phase_hold
```

신호 사용은 다음과 같이 분리했다.

| 소비자 | 입력 |
|---|---|
| `enc_param_apply_ctrl` | raw `phase_tick`, 시작 여부 추적용 |
| `enc_phase_counter` | gated `phase_advance` |
| `enc_position_tracker` | gated `phase_advance` |
| `enc_index_pulse` | gated `phase_advance` |
| `enc_timing_generator` | 자체 raw tick 생성, apply pulse에서 epoch 재시작 |

pending 요청이 있는 wrap부터 모든 consumer가 apply pulse를 본 clock까지 위치와
A/B phase를 고정한다. apply pulse가 timing generator를 새 epoch의 첫 interval로
재시작하므로 버린 raw tick은 보상 계산이 필요하지 않다.

## 4. 검증

### 4.1 전용 경계 시험

`tb_enc_apply_hold.vhd` 조건:

| 항목 | 값 |
|---|---:|
| CPR | 1 |
| total states | 4 |
| ticks low/high | 1 / 1 |
| 초기 방향 | CW |
| 요청 방향 | CCW |

검증 결과:

1. 기존 RTL은 apply pulse에서 position=1이 되어 FAIL.
2. 수정 RTL은 apply pulse 동안 position=0 유지.
3. apply 이후 첫 이동은 CCW 순서인 0→3.
4. `o_param_applied`는 정확히 1 clock.

최종 marker는 `ENC_APPLY_HOLD_PASS`, 완료 시각은 125001 ps이다.

### 4.2 회귀 시험

| 시험 | 결과 |
|---|---|
| canonical `tb_enc_apply_hold` | PASS |
| canonical `tb_enc_param_boundary` | PASS |
| canonical `enc_top_tb` 18 scenarios | PASS |
| Motor 호환 `enc_top_tb` | PASS |
| Internal full integration | PASS |

통합 결과는 Laser start/stop 18/18, Echo rise/stop 18/18,
TDC rising/falling TLAST 18/18, `cfg_rejected=0`, `pipeline_abort=0`,
`face_valid` violation 0으로 유지되었다.

Archive:

```text
sim_results/vivado_xsim/sessions/260722_apply_hold_full_system_integration_smoke
```

## 5. 합성 비용

대상은 `xc7z020clg484-2`, 5.000 ns constraint이다.

| 단계 | Slice LUT | Slice FF | WNS | 판정 |
|---|---:|---:|---:|---|
| Atomic apply v028 | 703 | 620 | +0.345 ns | PASS |
| Short-interval hold 최종 | 705 | 620 | +0.345 ns | PASS |

추가 비용은 LUT 2개, FF 0개다. 합성 결과는 0 error, 0 critical warning,
0 warning이며 timing 여유는 변하지 않았다.

## 6. 활성 모듈 최종 검토

| 모듈 | line | 최종 판단 |
|---|---:|---|
| `enc_pkg` | 40 | 폭, 상수, runtime epoch type만 보유. 유지 |
| `enc_param_apply_ctrl` | 177 | 단일 process, last-request-wins, event 기반 apply. 유지 |
| `enc_phase_counter` | 70 | A/B LUT와 2-bit phase만 보유. 유지 |
| `enc_position_tracker` | 99 | position과 pre-wrap/wrap event만 보유. 유지 |
| `enc_index_pulse` | 251 | Z guard, offset/width FSM, fault가 하나의 timing 계약을 공유. 유지 |
| `enc_timing_generator` | 114 | 분수 scheduler와 tick counter 통합. 유지 |
| `enc_top` | 183 | 외부 flat contract와 모듈 연결만 보유. 유지 |
| 합계 | 934 | active RTL |

`enc_index_pulse`가 가장 크지만 내부 precompute와 Z FSM은 같은 offset/width
계약과 counter를 공유한다. 다시 파일을 나누면 포트와 pipeline 계약만 늘고
합성 논리는 줄지 않는다. 현재는 추가 분리보다 세 개의 process 이름과
책임 경계가 더 읽기 쉽다.

runtime에서 legacy Z와 variable-width Z를 모두 바꿀 수 있어야 하므로 mode별
generate 제거도 적용하지 않는다. 이를 합성 옵션으로 만들면 generic과
검증 matrix만 늘어나 현재 목표와 맞지 않는다.

## 7. 최종 판정

Virtual Encoder는 다음 기준으로 최적화 완료로 판정한다.

1. 비활성 RTL 751 canonical line 제거
2. 기능별 7개 active module로 책임 분리
3. runtime 설정 epoch 원자 적용
4. 최소 1-clock interval까지 wrap/apply 원자성 보장
5. 200 MHz timing closure 유지
6. canonical, Motor 호환, 전체 IP 통합 회귀 통과

다음 IP 최적화에서는 Motor Decoder의 실제 기능 모듈을 대상으로 같은 절차를
적용한다. Motor 저장소에 남은 `enc_*` 최신 호환 복사본의 완전 제거는 새 parent
source ownership이 확정된 뒤 진행한다.
