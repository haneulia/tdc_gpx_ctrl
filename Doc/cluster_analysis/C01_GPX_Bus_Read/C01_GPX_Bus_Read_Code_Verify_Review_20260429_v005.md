# C01 GPX Bus Read Code Verify v005 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v005 리뷰 |
| 작성 시각 | 2026-04-29 19:18:18 +09:00 |
| 수정 시각 | 2026-04-29 19:18:18 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md`, `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v006.md`, 최신 `tmp/c01_verify` 로그, 보강 코드 |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | Plan v006 실행 결과가 v005 결과 보고서와 일치하는지, C01 closure evidence가 충분히 닫혔는지 확인 |

## 1. 리뷰 결론

신규 blocking finding 없음. v005 결과는 **승인 가능**하다.

Plan v006에서 의도한 evidence 보강은 실제 코드와 로그에서 확인된다. Positive 통합 회귀는 `INTEGRATED EXIT CODE = 0`, negative test는 별도 transcript에서 `INTEGRATED EXIT CODE = 1`, ASYNC FIFO generate evidence는 Primary-A(clean elab)와 Primary-B(sim marker) 모두 PASS이다.

따라서 C01 보완 검증은 다음 기준으로 닫혔다고 판단한다.

| 판단 축 | 결론 |
|---|---|
| 기능 회귀 | PASS |
| 통합 positive transcript | PASS |
| negative test transcript | PASS |
| ASYNC FIFO generate evidence | PASS (Primary-A + Primary-B) |
| marker VHDL 컴파일 안정성 | PASS (`xvhdl` ERROR 0건) |
| expected note / warning 분류 | PASS |
| C02 진입 가능성 | 가능 |

## 2. 핵심 evidence 확인

| 항목 | 기대 | 확인 결과 |
|---|---|---|
| Positive integrated exit code | `INTEGRATED EXIT CODE = 0` | `tmp/c01_verify/run_c01_regression.log:66` |
| Positive Stage 1 | `RESULTS: 4 ok, 0 failed` | `tmp/c01_verify/run_c01_regression.log:27` |
| Positive Stage 2 | `config_ctrl two-mode results: 2 pass, 0 fail` | `tmp/c01_verify/run_c01_regression.log:54` |
| Positive Stage 3 | CSR clamp `cases=12` PASS | `tmp/c01_verify/run_c01_regression.log:60`, `sim_tb_tdc_gpx_csr_chip_clamp.log:127` |
| Negative transcript | 별도 파일 보존 | `tmp/c01_verify/run_c01_regression_negative_20260429T100450Z.log` |
| Negative forced fail | `RESULTS: 4 ok, 1 failed` + forced marker | `run_c01_regression_negative_20260429T100450Z.log:27-28` |
| Negative integrated exit code | `INTEGRATED EXIT CODE = 1` | `run_c01_regression_negative_20260429T100450Z.log:67` |
| ASYNC Primary-A | `xpm_fifo_async` compile line | `elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log:69` |
| ASYNC Primary-B | marker 1회 | `sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29`; grep count ASYNC=1, SYNC=0 |
| VHDL marker compile | `xvhdl` ERROR 0건 | `../tdc_gpx_ctrl.sim/sim_1/behav/xsim/__c01_all_xvhdl.log`에 ERROR 없음 |

## 3. 코드 보강 확인

| 파일 | 확인 내용 | 판단 |
|---|---|---|
| `scripts/run_c01_regression.sh` | `--negative` 분기, Windows-safe timestamp, `INTEGRATED EXIT CODE` echo 및 `exit ${RC_FINAL}` | PASS |
| `tmp/c01_verify/run_regression.sh` | `C01_FORCE_NEGATIVE_STAGE1` hook이 summary 직전에 동작 | PASS |
| `tdc_gpx_config_ctrl.vhd` | `gen_raw_async` 내부 `gen_async_marker : if i = 0 generate begin ... assert false ... severity note` + `synthesis translate_off/on` | PASS |

`tdc_gpx_config_ctrl.vhd`의 marker는 simulation-only expected note이며, `translate_off` 보호가 있어 기능 RTL/합성 결과에는 영향을 주지 않는 것으로 판단한다.

## 4. 로그 스캔 결과

최신 v005 대상 로그에서 기능 실패에 해당하는 `ERROR:`, `severity failure`, `RUNTIME ERROR`는 확인되지 않았다.

주의할 점:
- `tmp/c01_verify/add_c01_tb.log`의 `CRITICAL WARNING`은 오래된 project/top 자동 선택 관련 로그이며, 이번 v005 통합 회귀 실행 산출물은 아니다.
- `elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log`의 `VRFC 10-3532 glbl generic override` warning은 기존 allow-list 대상이며 DUT generic 전달/ASYNC evidence 판단을 뒤집지 않는다.
- `run_c01_regression_negative_*.log`의 `C01 regression: FAIL`은 의도된 negative test 결과이다.

## 5. Finding 갱신

| Finding | v004 상태 | v005 리뷰 판단 |
|---|---|---|
| R-C01-V004-01 / R-C01-CL-02 | evidence 추적성 보완 필요 | 닫힘 |
| R-C01-V004-02 / R-C01-CL-03 | ASYNC FIFO generate evidence 보완 필요 | 닫힘 |
| R-C01-V004-03 | timestamp 정정 필요 | 닫힘 |
| P-C01-V005-01 | nested generate `begin` 권고 | 닫힘 |
| P-C01-V005-02 | PowerShell wrapper cwd 권고 | 닫힘 |
| P-C01-V005-03 | expected note 표 권고 | 닫힘 |

신규 finding은 없다.

## 6. 최종 판단

v005 결과 보고서의 closure 문구는 현재 evidence와 일치한다.

> C01 보완 검증은 기능 PASS, 회귀 운영 코드 보강, negative test transcript/exit code 보존, ASYNC FIFO generate evidence 보강까지 완료되었습니다. v004 / v006 / Plan v001/v002/v003/v004/v005/v006 / 주요 lineage는 연결되어 있으며, C02 진입 가능합니다.

본 리뷰도 위 문구를 인정한다. C01 closure package는 이제 기능과 evidence 추적성 양쪽에서 닫힌 상태로 판단한다.
