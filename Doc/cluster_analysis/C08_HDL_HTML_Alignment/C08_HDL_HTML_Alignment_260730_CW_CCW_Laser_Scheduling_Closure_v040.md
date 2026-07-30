# C08 CW/CCW Laser Scheduling Closure v040

작성일: 2026-07-30

## 1. 목적

Motor Decoder의 position이 CW에서 증가하거나 CCW에서 감소하더라도 Laser
Controller가 동일한 광학 점 간격과 시간 interlock으로 발사하는지 확인한다. 운용 중
방향 전환 시 이전 방향의 부분 각도 카운트가 반대 방향의 첫 shot 위치를 오염시키는
경로도 닫는다.

## 2. 확정 계약

1. Motor Decoder는 정상 quadrature transition으로 position이 바뀔 때 실제 진행
   방향을 갱신한다.
2. Motor AXIS `TUSER[6]`은 position beat와 정렬된 실제 decoded direction이다.
   `0`은 position 증가/CW, `1`은 position 감소/CCW다.
3. Motor `CTL0[16]`은 Virtual 회전 명령 또는 physical A/B 설치 극성 보정 설정이다.
   실제 방향과 동일한 의미가 아니므로 별도로 관찰한다.
4. Local Motor `STAT0[17]`과 Unified `MOTOR_STATUS[17]`은 실제 decoded direction,
   Unified `MOTOR_STATUS[16]`은 applied 방향/극성 설정이다.
5. Laser scheduler는 signed position 차를 사용하지 않는다. 이전 position과 다른
   accepted beat 하나를 `pos_advance` 한 번으로 정규화한다. 따라서 같은
   `step_interval`은 CW/CCW에서 같은 각도 간격을 만든다.
6. 같은 face에서 실제 방향이 바뀌면 Laser AXIS 입력부가 `face_end`와 `face_start`를
   동시에 만들어 새 angular epoch를 시작한다. 이전 방향의 step countdown과
   face-local shot count는 승계하지 않는다.

## 3. 발사 조건

각 방향의 유효 grid point에서 다음 조건이 모두 참일 때만 shot request를 만든다.

```text
angular_grid_due
AND laser_enable
AND face/window_enable
AND shot_limit_ok
AND executor_ready
AND descriptor_ready
```

`executor_ready`에는 fire command 진행, fresh `fire_done`, target round-trip,
re-arm guard가 포함된다. 시간 조건이 아직 끝나지 않은 grid point에서는
`fire_pulse`를 만들지 않고 그 후보를 건너뛴다. 같은 위치에서 늦게 발사하지 않으며,
다음 유효 각도 grid에서 시간 조건을 다시 확인한다. 따라서 발사 간격은 설정한 각도
간격보다 짧아지지 않고, target round-trip 시간과 겹치는 shot도 허용하지 않는다.

## 4. RTL 보완

- `motor_axis_stream_out`: 실제 direction을 pipeline position과 함께 정렬하여
  `TUSER[6]`으로 전달
- `motor_decoder_csr`: local `STAT0[17]` readback 추가
- `motor_unified_csr_adapter`: Unified `MOTOR_STATUS[17]` 추가
- `laser_ctrl_axis_in`: 같은 face의 direction 변화 감지 및 traversal epoch 재시작
- `laser_ctrl_scheduler`: 방향에 독립적인 고유 position-step 계약 유지

기존 외부 port, AXI interface 폭, local CSR 주소, Unified CSR 주소는 변경하지 않았다.
기존 예약 비트만 사용했다.

## 5. 검증 결과

### Motor Decoder 전체 회귀

- x1/x2/x4 CW/CCW position 진행: PASS
- direction metadata와 backpressure 정렬: PASS
- local `STAT0[17]`: PASS
- Unified applied `[16]`과 actual `[17]` 분리: PASS
- 최종 marker: `MOTOR_DECODER_REGRESSION_PASS`

### Laser Controller 전체 회귀

- 같은 face에서 direction reversal 시 `face_end + face_start`: PASS
- 감소 position `1200..1194`, interval 3에서 정확히 3 shot: PASS
- 기존 scheduler/executor/CSR 회귀: PASS
- 최종 marker: `LASER_CTRL_REGRESSION_PASS`

### Motor/Laser 통합 회귀

| Processing clock | Runtime CW -> CCW 전환 | CCW 발사 재개 | 결과 |
|---:|---|---|---|
| 150 MHz | 새 angular epoch 확인 | 각도/time-ready 교집합에서 확인 | PASS |
| 200 MHz | 새 angular epoch 확인 | 각도/time-ready 교집합에서 확인 | PASS |

성공 marker:

```text
MOTOR_LASER_WRAPPER_DIRECTION_REVERSAL_PASS
MOTOR_LASER_CTRL_ALL_FREQUENCIES_PASS
```

## 6. 판정과 잔여 범위

RTL 수준에서는 CW/CCW 정상 회전과 runtime 방향 전환 모두 동일한 각도·시간 발사
규칙을 만족한다. 방향 전환 시 첫 shot의 기준점도 이전 traversal에서 분리된다.

실제 보드 sign-off에서는 physical A/B 배선 극성, 모터 관성에 의한 reversal 구간의
비정상 quadrature transition, encoder 입력부터 레이저 광출력까지의 실측 지연을
ILA/계측기로 확인해야 한다. `MOTOR_QUAD_INVALID`, Motor AXIS drop, Laser schedule
overrun과 fire-done timeout 증가량이 0인 구간만 유효 acquisition으로 판정한다.
