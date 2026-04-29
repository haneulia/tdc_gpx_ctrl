# C01 GPX Bus Read Code Verify v003 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v003 리뷰 |
| 작성 시각 | 2026-04-29 16:35:25 +09:00 |
| 수정 시각 | 2026-04-29 16:35:25 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_20260429_v003.md`, `scripts/run_c01_regression.sh`, `tmp/c01_verify` 최신 로그 |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | v003 추가 검증 결과의 PASS 판정 타당성, 통합 회귀 스크립트의 실패 전파 신뢰성, v002 리뷰 반영 여부 확인 |

## 1. 종합 판단

v003에서 제출된 최신 개별 xsim 로그 기준으로는 C01 보완 검증이 PASS이다.

| 검증 항목 | 판단 | 근거 |
|---|---|---|
| bus_phy C01 contract | PASS | `sim_tb_tdc_gpx_bus_phy_c01_contract.log`에 PASS marker 존재 |
| bus_phy unit | PASS | `sim_tb_tdc_gpx_bus_phy.log:108`의 `*** ALL TESTS PASSED *** (total_rsp=85)` |
| chip_ctrl integration | PASS | `sim_tb_tdc_gpx_chip_ctrl.log:357`의 `*** ALL TESTS PASSED *** (total_raw_words=224)` |
| config_ctrl default SYNC | PASS | `sim_tb_tdc_gpx_config_ctrl.log:29`, `sim_tb_tdc_gpx_config_ctrl.log:35` |
| config_ctrl SYNC/ASYNC 2-mode | PASS | `sim_tb_tdc_gpx_config_ctrl_SYNC.log:29,35`, `sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29,35` |
| ASYNC FIFO generate | PASS | `elab_tb_tdc_gpx_config_ctrl_ASYNC.log`에서 `xpm_fifo_async` compile 확인 |
| CSR clamp + readback | PASS | `sim_tb_tdc_gpx_csr_chip_clamp.log:111,117,119,123,127` |

따라서 현재 제공된 시뮬레이션 산출물 자체는 C01 범위의 기능 PASS로 판단한다.

다만 v003 보고서가 주장하는 “통합 entrypoint 종료 코드로 전체 PASS/FAIL 판정”은 아직 완전히 성립하지 않는다. 하위 스크립트 2개가 내부 실패를 집계하지만 마지막에 실패 종료코드를 반환하지 않기 때문이다.

## 2. 리뷰 발견 사항

### R-C01-V003-01. 통합 회귀 스크립트의 Stage 1/2 실패 전파가 불완전함

| 항목 | 내용 |
|---|---|
| 심각도 | Medium |
| 상태 | 현재 로그 PASS는 인정, 통합 스크립트 보완 필요 |
| 현상 | `scripts/run_c01_regression.sh`는 Stage 1/2 결과를 `RC1=$?`, `RC2=$?`로 판단한다. 그러나 `tmp/c01_verify/run_regression.sh`와 `tmp/c01_verify/run_config_ctrl_two_modes.sh`는 실패 개수를 계산만 하고 마지막에 `exit 1`을 반환하지 않는다. |
| 근거 | `scripts/run_c01_regression.sh:34-42`, `scripts/run_c01_regression.sh:87-95`, `tmp/c01_verify/run_regression.sh:108-115`, `tmp/c01_verify/run_config_ctrl_two_modes.sh:90-93` |
| 영향 | Stage 1 또는 Stage 2 내부에서 FAIL_COUNT/FAIL이 증가해도 하위 스크립트 종료코드가 0으로 끝날 수 있다. 그러면 통합 스크립트가 실제 실패를 놓치고 `C01 regression: ALL PASS`를 출력할 위험이 있다. |
| 판단 | 현재 제출된 개별 로그는 모두 PASS라 이번 결과를 뒤집지는 않는다. 하지만 “종료 코드로 통합 PASS/FAIL 판정”이라는 v003 보고서 표현은 보강 전까지 조건부로만 인정한다. |
| 권장 조치 | `run_regression.sh` 말미에 `if [ ${FAIL_COUNT} -ne 0 ]; then exit 1; else exit 0; fi`를 추가하고, `run_config_ctrl_two_modes.sh` 말미에도 `if [ ${FAIL} -ne 0 ]; then exit 1; else exit 0; fi`를 추가한다. |

### R-C01-V003-02. `run_regression.sh`가 explicit PASS marker 없는 실행 완료를 PASS로 집계함

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 상태 | 현재 로그 PASS는 인정, 회귀 정책 보완 필요 |
| 현상 | `run_regression.sh`는 PASS marker가 없어도 `SIM RAN (no explicit PASS marker)`를 PASS_COUNT에 더한다. |
| 근거 | `tmp/c01_verify/run_regression.sh:100-104` |
| 영향 | 테스트벤치가 assertion 없이 종료됐지만 self-check PASS marker를 출력하지 않은 경우에도 PASS로 계산된다. C01처럼 계약 기반 검증을 관리하는 경우에는 추적성이 약해진다. |
| 판단 | 이번 v003 최신 로그들은 주요 TB에서 explicit PASS marker가 확인되어 현재 결과에는 영향이 없다. |
| 권장 조치 | C01 회귀에서는 no-marker를 fail 또는 at least warning으로 격상한다. 예외가 필요한 TB는 allow-list로 명시한다. |

### R-C01-V003-03. 통합 entrypoint의 `ALL PASS` 콘솔 출력이 별도 로그로 보존되지 않음

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 상태 | 추적성 보완 필요 |
| 현상 | v003 보고서는 `bash HDL/scripts/run_c01_regression.sh`의 `C01 regression: ALL PASS` 출력을 제시하지만, `tmp/c01_verify`에는 해당 통합 실행 콘솔 로그가 별도 파일로 보존되어 있지 않다. |
| 근거 | `scripts/run_c01_regression.sh:85-95`, `tmp/c01_verify` 최신 파일 목록 |
| 영향 | 나중에 문서만 남고 실제 통합 스크립트 콘솔 출력 추적이 어려워질 수 있다. 개별 로그는 남아 있으므로 기능 판정 근거는 유지된다. |
| 권장 조치 | 통합 스크립트를 실행할 때 `tmp/c01_verify/run_c01_regression.log`에 console transcript를 남기거나, 스크립트 내부에서 summary log를 생성한다. |

## 3. v002 리뷰 반영 확인

| v002 리뷰 항목 | v003 반영 판단 | 근거 |
|---|---|---|
| R-C01-V002-01 glbl generic warning | 반영됨 | v003 section 6 allow-list에 추가, SYNC/ASYNC log의 mode echo 정상 |
| R-C01-V002-02 writer/reader 표현 불일치 | 반영됨 | `tb_tdc_gpx_csr_chip_clamp.vhd:9`, `tb_tdc_gpx_csr_chip_clamp.vhd:303`, `sim_tb_tdc_gpx_csr_chip_clamp.log:111-119` |
| R-C01-V002-03 2-mode 검증 분리 | 부분 반영 | `scripts/run_c01_regression.sh`가 Stage 1/2/3 통합. 단, R-C01-V003-01 때문에 종료코드 전파 보강 필요 |
| R-C01-V002-04 c_CDC_WAIT 설명 | 반영됨 | `tb_tdc_gpx_csr_chip_clamp.vhd:26`, `tb_tdc_gpx_csr_chip_clamp.vhd:49` |

## 4. C01 완료 판단

현재 개별 검증 로그 기준으로 C01 보완 RTL/TB 동작은 PASS이다. 그러나 통합 회귀 운영 관점에서는 `run_regression.sh`와 `run_config_ctrl_two_modes.sh`의 종료코드 정책을 보강한 뒤 다시 한 번 `run_c01_regression.sh`를 실행해야 완전한 “통합 회귀 PASS”로 닫을 수 있다.

따라서 v003 결과는 다음과 같이 판단한다.

| 항목 | 판단 |
|---|---|
| 기능 로그 PASS | 인정 |
| v002 리뷰 반영 | 대부분 완료 |
| 통합 entrypoint 신뢰성 | 조건부, 종료코드 전파 보강 필요 |
| C02 인계 가능 여부 | 가능. 단, C01 회귀 스크립트 보강은 C02 시작 전/병행으로 처리 권장 |

## 5. 다음 조치 권장

1. `tmp/c01_verify/run_regression.sh`와 `tmp/c01_verify/run_config_ctrl_two_modes.sh` 말미에 실패 시 `exit 1` 반환을 추가한다.
2. `run_regression.sh`의 no-marker PASS 정책을 C01에서는 fail 또는 allow-list 기반 warning으로 변경한다.
3. `scripts/run_c01_regression.sh` 실행 transcript를 `tmp/c01_verify/run_c01_regression.log`로 보존한다.
4. 위 3건 반영 후 `bash HDL/scripts/run_c01_regression.sh`를 재실행해 v004 검증 문서에 종료코드와 로그 위치를 남긴다.
