# C08 Virtual Encoder Timing Optimization v025

## 1. 목적

가상 엔코더의 활성 타이밍 경로를 단순화하고, 런타임 설정 경계와 200 MHz 동작 가능성을 검증한다. 이 문서는 Virtual Encoder부터 IP별로 구조를 정리하는 첫 번째 체크포인트다.

## 2. 변경 전 문제

1. `enc_fractional_scheduler`와 `enc_tick_counter`가 하나의 위상 간격 생성 기능을 두 모듈, 네 개 프로세스, 피드백 인터페이스로 나누어 구현했다.
2. 기존 3단 스케줄러는 상태 간격이 1~2 clock인 설정에서 다음 간격 계산을 제때 완료할 수 없었다.
3. 설정 적용 중 tick counter가 진행 중이던 이전 epoch의 잔여 시간을 유지해, 새 설정의 첫 간격이 혼합될 수 있었다.
4. Reset 해제 직후 아직 0인 CSR 값을 자동 commit하여 `ticks_lo=0`, `CPR=0`인 잘못된 epoch가 잠시 활성화될 수 있었다.
5. `enc_top` 설명은 깨진 문자와 제거 전 모듈 구성을 포함해 실제 구조와 일치하지 않았다.

## 3. 적용 구조

활성 타이밍 경로를 `enc_timing_generator` 하나로 통합했다.

```text
runtime epoch
    -> enc_param_apply_ctrl
    -> enc_timing_generator
       - fractional accumulator
       - phase interval down-counter
       - phase_tick
    -> enc_phase_counter / enc_position_counter
```

핵심 계약은 다음과 같다.

- `ticks_lo`와 `ticks_hi`의 Bresenham 분포는 한 회전마다 `hi_count`개를 정확히 생성한다.
- `i_param_en`이 들어오면 이전 잔여 간격을 폐기하고 새 epoch의 첫 `ticks_lo` 간격부터 다시 시작한다.
- `ticks_lo=1`을 포함한 최소 간격을 지원한다.
- Generic 값은 reset 직후의 유효한 boot epoch다.
- CSR 값은 명시적인 `CFG_APPLY`가 있을 때만 commit한다.

기존 `enc_fractional_scheduler.vhd`와 `enc_tick_counter.vhd`는 이전 프로젝트 호환을 위해 파일은 유지하지만, 신규 회귀 및 통합 소스 목록의 활성 경로에서는 제외했다.

## 4. 복잡도 변화

| 항목 | 변경 전 | 변경 후 |
|---|---:|---:|
| 활성 타이밍 모듈 | 2 | 1 |
| 타이밍 소스 전체 줄 | 344 | 114 |
| 타이밍 실코드 줄 | 155 | 91 |
| `enc_top.vhd` 전체 줄 | 232 | 164 |

## 5. 동일 조건 합성 비교

조건: Vivado 2025.2.1, `xc7z020clg484-2`, clock period 5.000 ns.

| 구현 | Slice LUT | FF | WNS | 판정 |
|---|---:|---:|---:|---|
| 기존 scheduler + counter | 211 | 177 | -0.289 ns | 200 MHz 실패 |
| 신규 timing generator | 220 | 128 | +0.345 ns | 200 MHz 통과 |

LUT는 9개 증가했지만 FF는 49개 감소했고 WNS는 0.634 ns 개선되었다. 200 MHz 타이밍 closure와 코드 단순화를 고려하면 수용 가능한 교환이다.

## 6. 검증 결과

| 단계 | 결과 | 주요 확인 내용 |
|---|---|---|
| `tb_enc_timing_generator` | PASS | 정확한 간격, 한 회전당 high 개수, remainder=0, ticks=1, runtime reload |
| `enc_top_tb` 18 scenarios | PASS | CW/CCW, A/B 순서, 회전, Z late/early/width/fault, runtime 변경 |
| `tb_motor_cfg_commit_atomic` | PASS | 설정 epoch 원자성, busy 중 불변, CSR zero boot 차단 |
| Full multi-IP smoke | PASS | Motor -> Laser -> Echo -> TDC GPX 통합 흐름 |

최종 통합 결과:

- marker: `SYSTEM_INTEGRATION_SMOKE_PASS`
- Motor AXIS beats: 4627
- virtual/decoded position changes: 3297 / 404
- rise/fall VDMA beats: 96 / 96
- `cfg_rejected=0`, `pipeline_abort=0`

## 7. 발견되어 함께 수정한 부팅 결함

초기 전체 회귀는 182.5 ns에서 `runtime encoder tick intervals must be non-zero`로 실패했다. 원인은 새 타이밍 생성기가 아니라 `motor_cfg_commit_ctrl`의 자동 boot commit이었다. CSR IP 출력이 초기화되기 전에 0 값을 캡처하고 있었다.

수정 후 계약은 다음처럼 단순해졌다.

1. Reset 동안 Generic 기반 active/commit 레지스터를 설정한다.
2. Reset 해제만으로 commit pulse를 만들지 않는다.
3. 소프트웨어의 `CFG_APPLY`만 CSR shadow를 새 epoch로 만든다.

이 동작은 CSR 입력을 모두 0으로 둔 신규 회귀로 고정했다.

## 8. 남은 최적화 항목

1. `enc_position_counter`가 position, Z pulse, fault, guard, 32-bit 사전 계산을 한 파일에 소유한다. 전체 `enc_top` 200 MHz 합성의 현재 최악 경로는 이 블록 내부이며 WNS는 -1.508 ns다.
2. Z 시간 카운터 일부가 `c_POS_W=15`를 사용해 32-bit tick 주기보다 좁다. 저속 회전에서 잘림 가능성을 별도 회귀로 재현한 뒤 폭을 정리해야 한다.
3. `enc_param_apply_ctrl`의 position=0 level 기반 적용은 명확한 revolution pulse 계약으로 바꾸는 편이 안전하다.
4. `motor_decoder/HDL`과 별도 `virtual_encoder/HDL`에 유사 구현이 중복되어 있다. 다음 단계 전에 canonical source ownership을 결정해 중복 수정 위험을 제거해야 한다.
5. 기존 Vivado XPR/IP package 메타데이터에는 신규 파일이 아직 반영되지 않았다. 실제 parent 프로젝트를 만들 때 source manifest를 기준으로 재생성해야 한다.

## 9. 체크포인트 판정

타이밍 생성기 통합과 boot 설정 계약은 sign-off 가능하다. 다음 변경은 이 체크포인트 이후 별도 커밋으로 진행한다.
