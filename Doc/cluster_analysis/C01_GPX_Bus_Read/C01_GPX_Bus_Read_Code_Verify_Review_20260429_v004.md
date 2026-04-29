# C01 GPX Bus Read Code Verify v004 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v004 리뷰 |
| 작성 시각 | 2026-04-29 17:16:07 +09:00 |
| 수정 시각 | 2026-04-29 17:19:38 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_20260429_v004.md`, `run_c01_regression.log`, 최신 `tmp/c01_verify` 로그, 회귀 스크립트 |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | v004 추가 검증 결과의 PASS 판정, v003 리뷰 finding 반영 여부, 통합 회귀 추적성 확인, C01 closure 선언 문구의 적정성 반영 |

## 1. 종합 판단

최신 개별 xsim 로그와 통합 transcript 기준으로 C01 기능 회귀는 PASS이다. v003 리뷰에서 제기한 하위 스크립트 종료코드 전파, no-marker 처리, transcript 보존 문제도 코드 구조상 대부분 닫혔다.

| 항목 | 판단 | 근거 |
|---|---|---|
| Stage 1 4 TB | PASS | `tmp/c01_verify/run_c01_regression.log:26`의 `RESULTS: 4 ok, 0 failed` |
| config_ctrl default SYNC marker | PASS | `tmp/c01_verify/run_c01_regression.log:23`, `sim_tb_tdc_gpx_config_ctrl.log:35` |
| Stage 2 SYNC/ASYNC | PASS | `tmp/c01_verify/run_c01_regression.log:43-53` |
| Stage 3 CSR clamp/readback | PASS | `tmp/c01_verify/run_c01_regression.log:59`, `sim_tb_tdc_gpx_csr_chip_clamp.log:111-127` |
| 통합 transcript 보존 | PASS | `tmp/c01_verify/run_c01_regression.log:1-63` |
| Stage 1 실패 전파 코드 | PASS | `tmp/c01_verify/run_regression.sh` 말미에서 `FAIL_COUNT != 0`이면 `exit 1` |
| Stage 2 실패 전파 코드 | PASS | `tmp/c01_verify/run_config_ctrl_two_modes.sh` 말미에서 `FAIL != 0`이면 `exit 1` |
| no-marker PASS 정책 제거 | PASS | `tmp/c01_verify/run_regression.sh`에서 no-marker 분기가 `FAIL_COUNT++`로 변경됨 |

따라서 기능 검증과 회귀 운영 보강의 방향은 맞다. 다만 v004 보고서의 일부 근거 표현은 현재 보존된 artifact와 1:1로 맞지 않아 추적성 보완이 필요하다.

## 2. 리뷰 발견 사항

### R-C01-V004-01. `INTEGRATED EXIT CODE`와 negative test 근거가 보존 artifact에서 재추적되지 않음

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 상태 | 기능 PASS는 유지, 문서 추적성 보완 필요 |
| 현상 | v004 보고서는 `INTEGRATED EXIT CODE = 0/1`과 negative test 결과를 기록하지만, 현재 `tmp/c01_verify/run_c01_regression.log`에는 `INTEGRATED EXIT CODE` 라인이 없고, `NEGATIVE_TEST`, `FAIL_COUNT=99`, `__NEGATIVE_TEST_FORCED_FAIL`, `stage1=1` 흔적도 보존되어 있지 않다. |
| 근거 | `C01_GPX_Bus_Read_Code_Verify_20260429_v004.md:35-37`, `:99-116`; `tmp/c01_verify/run_c01_regression.log:1-63`; `tmp/c01_verify` 전체 검색 결과 negative-test trace 없음 |
| 판단 | 스크립트 코드상 실패 전파 보강은 타당하다. 하지만 “내가 추적해서 찾아가 확인할 수 있도록”이라는 프로젝트 원칙 기준에서는 negative test transcript 또는 wrapper log가 별도로 남아야 한다. |
| 권장 조치 | positive 실행은 `run_c01_regression.log`에 남기고, negative test는 `run_c01_regression_negative_YYYYMMDD_vNNN.log`처럼 별도 파일로 보존한다. 또한 exit code는 wrapper에서 `bash ...; echo "INTEGRATED EXIT CODE=$?"` 형태로 기록하거나 스크립트 내부에서 최종 `RC_FINAL`을 echo한 뒤 exit한다. |

### R-C01-V004-02. 최신 ASYNC elaboration log에는 `xpm_fifo_async` compile 1건이 직접 남아 있지 않음

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 상태 | 기능 PASS는 유지, ASYNC generate 증거 보강 권장 |
| 현상 | v004 보고서는 ASYNC mode에서 `xpm_fifo_async` 1건 활성이라고 설명하지만, 최신 `elab_tb_tdc_gpx_config_ctrl_ASYNC.log`는 incremental no-change 링크 메시지만 남기며 `xpm_fifo_async` compile line이 없다. |
| 근거 | `C01_GPX_Bus_Read_Code_Verify_20260429_v004.md:83-88`; `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC.log` |
| 판단 | `sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29`의 mode echo와 `:35`의 PASS는 ASYNC generic 전달과 smoke PASS를 보여준다. 그러나 최신 로그만으로 “FIFO compile 1건”을 직접 재확인할 수는 없다. 이는 `xelab --incr`의 no-change 동작 때문으로 보인다. |
| 권장 조치 | ASYNC generate 증거가 필요한 회귀에서는 `--incr` 없는 clean elaboration 1회를 별도 evidence log로 저장하거나, TB/RTL에 ASYNC branch marker report를 추가해 simulation log에서 직접 확인한다. |

### R-C01-V004-03. v004 보고서의 통합 실행 시작 시간이 transcript와 다름

| 항목 | 내용 |
|---|---|
| 심각도 | Trivial |
| 상태 | 문서 정합성 보완 |
| 현상 | v004 보고서의 예시는 start 시간을 `17:10:30 +0900`으로 적었지만, 실제 transcript는 `17:10:19 +0900`이다. |
| 근거 | `C01_GPX_Bus_Read_Code_Verify_20260429_v004.md:99`; `tmp/c01_verify/run_c01_regression.log:1` |
| 판단 | 기능 판단에는 영향 없다. 다만 시간순 대화/검증 기록 원칙상 정확한 시각으로 맞추는 것이 좋다. |
| 권장 조치 | v004 문서 또는 후속 v005에서 실제 transcript 기준 시각으로 정정한다. |

## 3. v003 리뷰 반영 확인

| v003 리뷰 항목 | v004 반영 판단 | 근거 |
|---|---|---|
| R-C01-V003-01 Stage 1/2 실패 전파 | 닫힘, 단 negative trace 보존 보완 필요 | `run_regression.sh`, `run_config_ctrl_two_modes.sh` 말미 exit 코드 추가 |
| R-C01-V003-02 no-marker PASS 정책 | 닫힘 | `run_regression.sh` no-marker 분기가 FAIL로 변경, `TB:.*PASS` regex 추가 |
| R-C01-V003-03 통합 transcript 보존 | 닫힘 | `scripts/run_c01_regression.sh`의 `tee`, `tmp/c01_verify/run_c01_regression.log` 생성 |

## 4. C01 완료 판단

현재 v004 결과는 **기능 PASS + 회귀 운영 보강 PASS**로 판단한다. 남은 항목은 기능 결함이 아니라 evidence 추적성 보완이다.

| 항목 | 판단 |
|---|---|
| 기능 회귀 | PASS |
| CSR clamp/readback | PASS |
| SYNC/ASYNC smoke | PASS |
| 통합 transcript | PASS |
| 종료코드 전파 코드 | PASS |
| negative test artifact | 보완 필요 |
| ASYNC FIFO generate 최신 evidence | 보완 권장 |

따라서 C01에서 C02로 넘어가는 것은 가능하다. 단, C01 closure package를 더 단단하게 만들려면 후속 v005에서 negative transcript와 ASYNC clean elaboration evidence만 추가하면 된다.

## 5. 다음 조치 권장

1. `run_c01_regression.sh`가 최종 판정 직전에 `INTEGRATED EXIT CODE = <0|1>`을 transcript에 남기도록 수정한다.
2. negative test transcript를 overwrite되지 않는 별도 파일로 저장한다.
3. ASYNC FIFO generate 증거를 clean elaboration log 또는 simulation branch marker로 보존한다.
4. v004 문서의 start 시각을 실제 transcript의 `17:10:19 +0900`로 정정하거나 후속 v005 문서에서 정정 이력을 남긴다.

## 6. Closure 선언 검토 반영

사용자 요청 문장:

> C01 보완 검증은 운영 정합성 + 기능 PASS 모두 닫혔습니다. v004 / v006 / Plan v001 / lineage 일체 검토 부탁드립니다. 다음은 C02 진입 가능 상태입니다.

본 리뷰 v004에 대한 추가 검토 결과, 위 문장은 **대체로 타당하지만 “모두 닫힘”은 evidence 보존 관점에서 조건부 표현이 필요하다**고 판단한다.

| 판단 축 | 결론 | 근거 |
|---|---|---|
| 기능 PASS | 인정 | 최신 xsim 로그와 `run_c01_regression.log` 기준 Stage 1/2/3 PASS |
| 회귀 운영 코드 보강 | 인정 | Stage 1/2 exit code 전파, no-marker FAIL 격상, transcript 보존 구조 반영 |
| v006 운영 프로토콜 | 인정 | `cluster_analysis_operating_protocol_20260429_v006.md`에 Review -> Plan -> 승인 -> 진행 -> Result 규칙 추가 |
| Plan v001 | 인정 | retrospective 작성 사실과 v004 반영 위치가 기록됨 |
| Lineage | 인정 | `v003 -> v004`, `Plan v001 -> v004`, `v005 -> v006` forward trace 존재 |
| Closure evidence | 보완 필요 | Negative test transcript/exit code와 최신 ASYNC FIFO generate evidence는 현재 보존 artifact만으로 완전 재추적되지 않음 |
| C02 진입 | 가능 | 남은 항목은 C01 기능 결함이 아니라 evidence package 추적성 보완 |

따라서 본 리뷰 v004의 최종 closure 표현은 다음 문장으로 조정한다.

> C01 보완 검증은 기능 PASS와 회귀 운영 코드 보강 기준으로 닫혔습니다. v004 / v006 / Plan v001 / 주요 lineage는 연결되어 있으며, C02 진입은 가능합니다. 단, closure evidence 완성도를 위해 negative test transcript/exit code와 ASYNC FIFO generate 증거는 후속 v005로 보강 권장합니다.

추적 근거:

| 항목 | 위치 |
|---|---|
| Closure 별도 검토 문서 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Closure_Review_20260429_v001.md` |
| 본 리뷰의 기존 evidence finding | R-C01-V004-01, R-C01-V004-02, R-C01-V004-03 |
| C02 진입 판단 | 본 문서 section 4 및 본 section 6 |
