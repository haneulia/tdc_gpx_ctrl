# C08 Virtual Encoder Atomic Parameter Apply v028

## 1. 목적

Virtual Encoder의 런타임 설정 적용 경계를 명확히 하고, 설정 제어 코드의
중복 상태와 불필요한 위치 비교를 제거한다. 기능 변경은 다음 계약으로 제한한다.

1. 설정 입력 한 묶음은 하나의 epoch로 캡처한다.
2. 아직 encoder가 움직이지 않았다면 초기 유휴 상태에서 적용할 수 있다.
3. 한 번이라도 phase tick이 발생한 뒤에는 실제 회전 wrap event에서만 적용한다.
4. 적용 대기 중 새 요청이 들어오면 가장 최근 요청이 이전 요청을 대체한다.
5. 모든 active 설정 출력은 같은 clock에서 함께 바뀐다.

Canonical RTL 위치는 다음과 같다.

```text
C:/Projects/my_sp/lib/IP/virtual_encoder/HDL
```

## 2. 발견된 문제

### 2.1 위치 0 레벨을 경계로 오인

기존 `enc_param_apply_ctrl`은 `position=0`인 동안 pending 설정을 적용했다.
주석은 초기 유휴와 한 바퀴를 돈 뒤의 0 구간을 구분한다고 설명했지만,
`s_pos_was_nz_r`의 두 분기 모두 `s_apply_r <= '1'`이어서 합성 후 구분 로직이
사라졌다.

수정 전 RTL에 새 테스트를 적용하면 다음과 같이 실패했다.

```text
Failure: runtime configuration applied inside a zero-position phase
Time: 735001 ps
```

즉, position이 0인 여러 clock 중간에 요청이 들어오면 실제 wrap 순간이 아닌
곳에서 방향, 분수 주기, Z 설정이 바뀔 수 있었다.

### 2.2 첫 phase tick과 초기 요청의 경합

초기 유휴 판단을 position 값에만 의존하면, 첫 phase tick이 생성되는 시점과
설정 요청이 겹쳤을 때 이미 운동이 시작되었는데도 idle apply가 예약될 수 있다.
position은 phase tick보다 한 단계 뒤에서 갱신되므로 level 비교만으로는 이
경합을 완전히 구분할 수 없다.

### 2.3 같은 설정 묶음의 반복 선언

기존 구현은 pending과 active 각각에 8개 필드를 개별 signal로 선언하고,
capture, boundary detect, active apply를 3개 process에 나누었다. 한 필드가
추가될 때 선언, reset, capture, apply, output을 여러 곳에서 동시에 수정해야
하므로 누락 위험이 컸다.

## 3. 구조 변경

### 3.1 설정 epoch를 record로 관리

`enc_pkg.t_enc_runtime_cfg`를 다음 세 용도로 사용한다.

| record | 역할 |
|---|---|
| `s_request_cfg` | 현재 flat 입력을 하나의 조합 record로 묶음 |
| `s_pending_cfg_r` | 아직 적용되지 않은 최신 요청 보관 |
| `s_active_cfg_r` | datapath가 실제 사용하는 설정 |

외부 IP/CSR 포트는 기존 flat 신호를 유지하므로 상위 인터페이스 호환성은
변하지 않는다.

### 3.2 경계 판단을 event 기반으로 변경

`enc_param_apply_ctrl`은 더 이상 15-bit `i_position`을 비교하지 않는다.

| 입력 event | 의미 |
|---|---|
| `i_phase_tick` | encoder가 실제로 움직이기 시작했음을 기록 |
| `i_wrap_event` | 실행 중 설정을 적용할 수 있는 유일한 회전 경계 |

초기 요청은 한 clock 뒤에 다시 확인한다. 그 사이 첫 phase tick이 관찰되면
idle apply를 취소하고 pending 상태로 유지한 뒤 다음 wrap event를 기다린다.
따라서 phase tick 생성과 position 갱신 사이의 pipeline 지연에도 안전하다.

### 3.3 단일 순차 process

설정 캡처, pending 정책, 경계 적용, apply pulse 생성을 하나의 순차 process로
통합했다. 우선순위는 다음과 같다.

1. 새 요청 캡처: last request wins
2. 예약된 초기/경계 적용 처리
3. 기존 pending 요청의 wrap 적용

`o_param_en`과 `o_param_applied`는 같은 1-clock pulse이며, 모든 active 필드는
동일 edge에서 갱신된다.

## 4. 코드 복잡도

| 항목 | 수정 전 | 수정 후 |
|---|---:|---:|
| `enc_param_apply_ctrl.vhd` 물리 line | 240 | 172 |
| 순차 process | 3 | 1 |
| position 비교 입력 | 15 bit | 제거 |
| 내부 설정 표현 | 필드별 signal 반복 | `t_enc_runtime_cfg` record |
| 실행 중 적용 기준 | `position=0` level | 등록된 `wrap_event` pulse |

line 수는 68줄 감소했다. 더 중요한 변화는 설정 필드 추가 시 record와 flat
입출력 매핑만 확인하면 되고, 여러 process의 복사 코드를 함께 수정할 필요가
없어졌다는 점이다.

## 5. 합성 결과

대상은 `xc7z020clg484-2`, clock constraint는 5.000 ns이다.

| 단계 | Slice LUT | Slice FF | WNS | 판정 |
|---|---:|---:|---:|---|
| Position/Z split 기준점 v027 | 699 | 620 | +0.345 ns | PASS |
| Atomic apply 최종 | 703 | 620 | +0.345 ns | PASS |

기능 보강 비용은 LUT 4개, FF 0개이며 WNS는 유지되었다. 합성 결과는
0 error, 0 critical warning, 0 warning이다. 실행 환경의 Xilinx Tcl Store
저장 경고는 설계 합성 경고 집계 이전의 도구 환경 메시지이며 RTL 경고가 아니다.

## 6. 검증 결과

| 시험 | 결과 | 확인 내용 |
|---|---|---|
| 수정 전 red test | FAIL 재현 | 실행 중 zero-position 구간에서 잘못 적용 |
| `tb_enc_param_boundary` | PASS | zero-position 중간 요청은 다음 wrap까지 보류 |
| `tb_enc_param_boundary` startup 경합 | PASS | 첫 phase tick이 보이면 idle apply 취소 |
| canonical `enc_top_tb` 18 scenarios | PASS | CW/CCW, Z mode/offset/width/fault, runtime switch |
| Motor 호환 `enc_top_tb` | PASS | 동기 복사본 인터페이스 및 동작 무회귀 |
| `tb_motor_cfg_commit_atomic` | PASS | 상위 Motor 설정 commit 원자성 |
| Internal full integration | PASS | Motor, Laser, Echo, TDC GPX 연결 무회귀 |
| 200 MHz 독립 합성 | PASS | 703 LUT, 620 FF, WNS +0.345 ns |

최종 내부 통합 결과는 다음과 같다.

| 관찰 항목 | 결과 |
|---|---:|
| Laser start/stop | 18 / 18 |
| Laser result TLAST | 18 |
| Echo rise/stop event | 18 / 18 |
| TDC rising/falling line TLAST | 18 / 18 |
| `cfg_rejected` / `pipeline_abort` | 0 / 0 |
| `face_valid` violation | 0 |

최종 통합 archive:

```text
sim_results/vivado_xsim/sessions/260722_param_motion_full_system_integration_smoke
```

## 7. 적용 계약

소프트웨어 또는 상위 `motor_cfg_commit_ctrl`은 모든 설정 필드를 안정화한 뒤
`i_param_en`을 한 clock pulse로 발생시켜야 한다. 요청이 실행 중 들어오면
`o_param_applied`가 올 때까지 새 설정은 active datapath에 영향을 주지 않는다.
여러 요청을 연속으로 보내면 큐가 쌓이는 구조가 아니라 마지막 요청 하나만
보존한다.

이 정책은 설정 요청 수가 매우 낮고 회전 경계에서만 원자 적용해야 하는
Virtual Encoder의 용도에 맞다. 모든 요청을 순서대로 처리해야 하는 용도로
바뀐다면 FIFO가 필요하지만 현재 요구에는 불필요한 자원이다.

## 8. 판정 및 다음 단계

Atomic parameter apply 단계는 기능, 통합, 200 MHz 합성 기준으로 sign-off
가능하다. 다음 단계는 현재 active manifest에서 제외된 다음 legacy RTL의
참조를 전수 조사한 뒤 별도 커밋으로 제거하는 것이다.

```text
enc_position_counter.vhd
enc_fractional_scheduler.vhd
enc_tick_counter.vhd
```

Motor 저장소의 최신 `enc_*` 호환 복사본은 실제 parent project source ownership이
확정될 때까지 유지한다. 현재 통합 manifest는 canonical Virtual Encoder만 읽는다.
