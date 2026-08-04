# V2 Checkpoint C: Sequential Commit Calculator

## 1. 판정

Checkpoint C의 합성 가능한 순차형 설정 계산기는 **PASS**이다.

이 판정은 runtime source 설정을 원자적으로 캡처하고, 유효성을 검사한 뒤,
기준 모델과 동일한 derived 설정을 계산하여 한 번에 게시하는 블록에 한정한다.
통합 CSR, `PREPARE -> ACK -> ACTIVATE` 설정 관리자, clock-domain gateway 및
전체 LiDAR 데이터 경로의 sign-off를 의미하지는 않는다.

## 2. 구현 구조

| 모듈 | 역할 |
|---|---|
| `lidar_commit_calculator` | 요청 캡처, validate/derive 순서 제어, 결과 원자적 게시 |
| `lidar_config_validator_seq` | build/runtime 범위, face geometry, mask 및 시간 관계 검사 |
| `lidar_config_deriver_seq` | face 경계, 각도/state, shot 간격, column 수, domain clock 수 계산 |
| `lidar_u32_u16_multiplier_seq` | 16-cycle shift/add 곱셈기 |
| `lidar_u64_u32_divider_seq` | 64-cycle restoring 나눗셈기 |
| `lidar_config_reference_pkg` | TB에서만 사용하는 cycle-independent 정답 모델 |

곱셈기와 나눗셈기는 validator와 deriver가 시간 분할하여 공유한다. 큰 조합
나눗셈기나 실시간 shot 경로의 반복 계산은 사용하지 않는다.

## 3. 트랜잭션 계약

1. IDLE에서 `i_start='1'`을 받으면 `i_source` 전체를 같은 clock edge에서 캡처한다.
2. BUSY 동안 외부 `i_source`가 바뀌어도 진행 중 계산에는 영향을 주지 않는다.
3. BUSY 중 추가 start는 실행하지 않고 `o_start_rejected`를 1클럭 pulse로 낸다.
4. 검증 오류나 내부 산술 오류가 나면 마지막 정상 `o_derived`를 보존한다.
5. 정상 계산이 끝난 경우에만 working 결과를 `o_derived`에 한 번에 게시한다.
6. `o_done`은 성공과 실패 모두 1클럭 pulse이며, 결과는 `o_error`로 구분한다.
7. reset은 진행 중 작업을 중단하지만 마지막 게시 결과는 지우지 않는다.

내부 산술기에서 정상 계약상 발생할 수 없는 divide-by-zero가 검출되면 외부
안정 코드 `CFG_INTERNAL_ARITHMETIC = 0x70`으로 보고한다.

## 4. 순차 계산 결과

기본 build/runtime 설정의 정상 commit은 TB 관측 기준 **559 clocks**이다.

| 설정 clock | 관측 commit 시간 |
|---:|---:|
| 150 MHz | 약 3.727 us |
| 200 MHz | 약 2.795 us |

이 시간은 software가 설정을 commit할 때만 사용된다. motor position, laser
`fire_pulse`, `fire_done -> start_tdc`, Echo LVDS-to-STOP 및 GPX readout의
실시간 지연에는 포함되지 않는다.

## 5. 기능 검증

150 MHz와 200 MHz에서 같은 self-checking TB를 실행했다. 모든 정상 결과는
`lidar_config_reference_pkg`의 결과와 record 전체를 비교한다.

- 기본 설정과 ceil shot quantization
- signed capture calibration 및 5 ns tick의 domain-clock ceil 변환
- 최대에 가까운 32-bit 시간값의 곱셈 범위
- falling 비활성 시 모든 active chip의 rising 승격
- CW/CCW와 무관한 물리 face geometry
- CPR, virtual timing, Z, face center/overlap, face mask 오류
- 1-state 미만 shot angle, pulse/range/timeout 관계 오류
- capture underflow, active-chip mask, max-hits, GPX bus timing 오류
- invalid commit의 기존 결과 보존
- BUSY 요청 거절과 입력 snapshot 원자성
- 연산 중 reset, 결과 보존 및 재시작

Pass marker는 두 profile 모두 `LIDAR_V2_COMMIT_CALCULATOR_PASS`이다.

## 6. 구현 결과

대상은 `xc7z020clg484-2`, out-of-context 배치·배선 결과이다.

| 항목 | 150 MHz | 200 MHz |
|---|---:|---:|
| Post-route WNS | +1.555 ns | +0.261 ns |
| Inferred latch | 0 | 0 |
| LUT | 1,006 | 1,006 |
| FF | 2,071 | 2,071 |
| BRAM | 0 | 0 |
| DSP | 0 | 0 |

초기 조합 경로는 150 MHz에서도 실패했다. face pair 선택, face boundary 계산,
capture 합산, timeout 비교 및 오류 선택을 단계적으로 등록하여 200 MHz까지
닫았다. 최종 구조는 제약을 느슨하게 하지 않고 순차 단계로 타이밍을 확보했다.

이 결과는 내부 모듈 OOC 검증이므로 외부 input/output delay와 상위 clock-buffer
위치는 포함하지 않는다. 보고된 `HD.CLK_SRC`, `PARTPIN_LOCS` 및 제한된 OOC DRC
경고는 이 검증 방식의 경계이다. 최종 통합 top에서는 실제 clock source, CDC,
I/O delay 및 배치 혼잡도를 포함하여 다시 timing closure해야 한다.

## 7. 재현 정보

- Tool: Vivado 2025.2.1
- Package regression: `260804125059_v2_config_pkg`
- Calculator regression: `260804125422_v2_commit_calculator`
- Calculator archive:
  `signoff_results/sessions/260804125422_v2_commit_calculator`
- 실행 스크립트:
  `system_integration/v2/scripts/run_v2_commit_calculator.ps1`
- WDB와 Vivado journal은 보관하지 않는다.

현재 사용자 Tcl Store catalog 손상 경고가 있으나, 회귀 스크립트는 설치 영역의
공식 simulator adapter를 명시적으로 불러와 실행을 완료했다. 사용자 Tcl Store는
이 작업에서 변경하지 않았다.

## 8. 남은 경계

다음 Checkpoint는 calculator 앞뒤에 다음 블록을 연결한다.

1. unified CSR shadow store와 W1S commit command
2. configuration manager의 `PREPARE -> ACK -> ACTIVATE`
3. processing/TDC domain gateway와 sequence 확인
4. 새 shot 차단, 진행 중 데이터 drain 및 activation 안전 경계
5. 통합 top에서 clock, I/O delay 및 실제 배치 환경을 포함한 timing closure

따라서 현재 정확한 표현은 **"순차형 설정 계산기 sign-off 완료, V2 통합 모듈은
구현 진행 중"**이다.
