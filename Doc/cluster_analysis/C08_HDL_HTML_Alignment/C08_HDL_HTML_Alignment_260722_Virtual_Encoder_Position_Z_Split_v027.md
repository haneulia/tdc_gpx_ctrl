# C08 Virtual Encoder Position/Z Split v027

## 1. 목적

Virtual Encoder를 기능 단위로 읽고 검증할 수 있도록 위치 추적과 Z/index 생성을 분리하고,
기존 구조에서 가려졌던 시작 동작 및 장주기 Early-Z 결함을 수정한다.

Canonical RTL은 다음 저장소가 소유한다.

```text
C:/Projects/my_sp/lib/IP/virtual_encoder/HDL
```

## 2. 수정 전 확인된 문제

| ID | 문제 | 수정 전 재현 |
|---|---|---|
| VE-P0-01 | Z startup guard가 A/B까지 막아 첫 회전 동안 quadrature 출력이 정지 | `tb_enc_startup_ab`: 첫 phase tick에서 A/B 변화 없음 |
| VE-P0-02 | `ticks_next-offset` 결과를 15 bit position 폭으로 줄여 긴 주기에서 Early-Z 위치가 wrap | `tb_enc_z_long_interval`: 기대 10 clk 대신 32778 clk |
| VE-P1-01 | 위치, Z guard, Z delay/width FSM이 한 파일에 결합 | `enc_position_counter.vhd` 378 lines |
| VE-P1-02 | position 비교 결과가 Z 32-bit counter 제어까지 한 cycle에 연결 | 5 ns 합성 WNS -1.508 ns |
| VE-P1-03 | Motor Decoder 복사본이 통합 빌드의 활성 소스여서 canonical source와 분기 가능 | manifest가 `motor_decoder/HDL/enc_*.vhd` 참조 |

## 3. 구조 변경

| 모듈 | 단일 책임 |
|---|---|
| `enc_timing_generator` | 분수 주기 분배와 phase tick 생성 |
| `enc_phase_counter` | A/B quadrature phase만 생성 |
| `enc_position_tracker` | 위치와 CW/CCW 회전 경계 이벤트 생성 |
| `enc_index_pulse` | Z guard, Early/Late offset, width, fault 처리 |
| `enc_param_apply_ctrl` | runtime configuration epoch 보류 및 원자 적용 |
| `enc_top` | 위 모듈의 연결만 담당 |

핵심 변경은 다음과 같다.

1. A/B는 첫 phase tick부터 진행하고, 한 회전 startup guard는 Z에만 적용한다.
2. Early-Z 지연은 `c_TICK_CNT_W=32` 전체 폭을 유지한다.
3. 위치 모듈은 `pre_wrap`과 `wrap`을 등록된 1-cycle 이벤트로 전달한다.
4. 이벤트 등록으로 생긴 1 clk 지연은 Z offset/width count에서 보정한다.
5. `ticks_next`는 Z 모듈 입구에서 한 번 등록해 scheduler-to-Z 산술 경로를 자른다.
6. CW pre-wrap 기준 `total_states-2`는 설정 적용 때 계산해 동작 중 add-compare를 제거한다.

활성 position/Z 코드는 378-line 단일 파일에서 90-line tracker와 233-line index 모듈로
분리됐다. 총 line 수보다 책임과 검증 경계를 줄이는 것을 우선했다.

## 4. 합성 결과

대상은 `xc7z020clg484-2`, clock constraint는 5.000 ns다.

| 단계 | Slice LUT | Slice FF | WNS | 판정 |
|---|---:|---:|---:|---|
| 수정 전 monolithic | 475 | 569 | -1.508 ns | FAIL |
| 단순 파일 분리 | 608 | 605 | -1.916 ns | FAIL |
| 경계 이벤트 등록 | 653 | 573 | -0.760 ns | FAIL |
| tick 입력 등록 + pre-wrap 기준 등록 | 699 | 620 | +0.345 ns | PASS |

LUT/FF 증가는 수정 전 RTL이 잘라 버리던 32-bit Early-Z 시간 범위를 실제로 보존하고,
긴 조합 경로를 레지스터 경계로 분리한 비용이다. 최종 점유율은 LUT 1.31%, FF 0.58%다.
기능 및 타이밍 closure를 이번 단계의 우선 조건으로 두고 면적 축소는 다음 checkpoint에서 진행한다.

## 5. 검증 결과

| 시험 | 결과 | 확인 계약 |
|---|---|---|
| `tb_enc_timing_generator` | PASS | ticks=1 포함 분수 주기 분배 |
| `tb_enc_startup_ab` | PASS | 첫 phase tick A/B 진행, Z만 guard |
| `tb_enc_z_long_interval` | PASS | 40000-clk interval, Early-Z 10 clk |
| `enc_top_tb` 18 scenarios | PASS | CW/CCW, legacy/FSM Z, offset/width/fault/runtime switch |
| Motor compatibility `enc_top_tb` | PASS | Motor 저장소 동기 복사본 무회귀 |
| Internal full integration | PASS | start/stop/result/echo/rise/fall = 18/18 |
| External 150/200 MHz integration | PASS | start/stop/result/echo/rise/fall = 24/24 |

## 6. 통합 시험에서 추가로 발견한 계약

정상 A/B가 시작 직후 출력되자 기존 TB가 의존하던 숨은 순서 가정이 드러났다.

1. Motor-to-Laser 요청은 TDC configuration 및 START 완료 전에는 차단해야 한다.
2. 종료 drain은 고정 시간 대기가 아니라 발생한 shot과 result/VDMA 완료 수가 같아질 때까지 기다려야 한다.
3. 임의 encoder phase에서 관찰을 시작하면 다음 완전한 face 경계가 한 회전 뒤일 수 있으므로
   face-close timeout은 최소 한 회전을 포함해야 한다.

`tb_tdc_gpx_full_int.vhd`에 위 세 조건을 반영했다. 이는 테스트 편의를 위한 우회가 아니라
실제 parent 제어 시퀀스가 따라야 할 준비 완료/요청 허용 계약이다.

## 7. Source ownership

- 단위 RTL과 결함 재현 TB는 `virtual_encoder/HDL`이 소유한다.
- `run_smoke.ps1`과 `add_sibling_sources.tcl`은 canonical Virtual Encoder를 직접 읽는다.
- `motor_decoder/HDL`의 `enc_*.vhd`는 현재 호환 복사본이며 활성 통합 manifest에서는 제외했다.
- 구형 `enc_position_counter`, `enc_fractional_scheduler`, `enc_tick_counter`는 이번 단계에서
  삭제하지 않았지만 active source가 아니다.

## 8. 다음 최적화 순서

1. 이 상태를 Git checkpoint로 고정한다.
2. `enc_param_apply_ctrl`의 합성 제거 대상 `s_pos_was_nz_r`와 중복 상태를 정리한다.
3. Z delay와 width counter 분리 또는 mode별 generate가 LUT를 실제로 줄이는지 합성 A/B 비교한다.
4. 결과가 나빠지는 최적화는 채택하지 않는다.
5. parent project manifest가 확정되면 Motor 저장소의 호환 복사본과 구형 비활성 RTL 삭제를 별도 커밋한다.

## 9. 단계 판정

Virtual Encoder의 기능 분리, 장주기 정확성, 200 MHz timing, 내부/외부 통합은 sign-off 가능하다.
면적은 허용 범위지만 최적 상태로 확정하지 않고 다음 최적화 단계의 기준값으로 사용한다.
