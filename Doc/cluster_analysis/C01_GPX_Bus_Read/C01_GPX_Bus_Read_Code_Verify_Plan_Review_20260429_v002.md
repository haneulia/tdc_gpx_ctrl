# C01 GPX Bus Read Code Verify Plan v002 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v002 Plan 리뷰 |
| 작성 시각 | 2026-04-29 17:26:21 +09:00 |
| 수정 시각 | 2026-04-29 17:26:21 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md` |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | Plan v002가 R-C01-V004-01..03 및 R-C01-CL-01..04를 닫기에 충분한지, 승인 전 수정이 필요한지 판단 |

## 1. 종합 판단

Plan v002의 방향은 타당하다. v004 리뷰와 closure 리뷰에서 남긴 핵심 보완점인 `INTEGRATED EXIT CODE` 보존, negative transcript 보존, ASYNC clean evidence 확보, v005 closure 문구 갱신을 모두 다루고 있다. 또한 v006 운영 프로토콜의 첫 정상 적용 사이클로 Review -> Plan -> 사용자 승인 -> 진행 -> Result 순서를 명시한 점도 맞다.

다만 승인 전 아래 3가지는 수정 또는 명확화가 필요하다.

| 항목 | 판단 |
|---|---|
| finding coverage | 충분 |
| 진행 순서 | 대체로 적절 |
| 산출물 계획 | 대체로 적절 |
| 승인 가능성 | 조건부 승인 가능 |
| 승인 전 필수 보완 | P-C01-V002-01, P-C01-V002-02, P-C01-V002-03 |

## 2. 리뷰 발견 사항

### P-C01-V002-01. v005 closure 문구가 “보강 권장”과 “보강 완료”를 동시에 말함

| 항목 | 내용 |
|---|---|
| 심각도 | Medium |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md:109-112` |
| 현상 | Plan은 v005 closure 절에 기존 권장 문구를 “그대로 채택”한다고 적은 뒤, 바로 다음 줄에서 v005에서는 “보강 완료”로 갱신한다고 적고 있다. |
| 영향 | v005 결과 보고서 작성 시 closure 문구가 다시 모호해질 수 있다. v005는 evidence 보강을 수행한 결과 문서이므로 “후속 v005로 보강 권장”이라는 표현을 그대로 사용하면 안 된다. |
| 권장 수정 | v005에 채택할 최종 문구를 아래처럼 명시한다: “C01 보완 검증은 기능 PASS, 회귀 운영 코드 보강, negative test transcript/exit code 보존, ASYNC FIFO generate evidence 보강까지 완료되었습니다. v004 / v006 / Plan v001/v002 / 주요 lineage는 연결되어 있으며, C02 진입 가능합니다.” |

### P-C01-V002-02. Negative test injection 위치가 기대 transcript와 맞지 않을 수 있음

| 항목 | 내용 |
|---|---|
| 심각도 | Medium |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md:50`, `:94-102` |
| 현상 | Plan은 `run_regression.sh` “말미”에 `FAIL_COUNT=99`를 주입한다고 쓰지만, 기대 transcript는 Stage 1 summary에 `RESULTS: ... 99 failed`와 `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`가 남는 것이다. |
| 영향 | injection이 summary 출력 뒤 또는 exit 판정 직전에 들어가면 종료코드는 1이 될 수 있어도, 기대한 Stage 1 summary evidence는 남지 않는다. 반대로 summary 앞에 정확히 넣지 않으면 계획과 결과가 또 어긋난다. |
| 권장 수정 | injection 위치를 `run_regression.sh`의 summary 출력 직전, 즉 `echo "RESULTS: ..."` 이전으로 명시한다. 더 안전한 대안은 임시 파일 수정 대신 `C01_FORCE_NEGATIVE_STAGE1=1` 같은 명시적 test hook을 sub-script에 추가하고, negative 실행 때만 활성화하는 방식이다. |

### P-C01-V002-03. ASYNC clean elaboration evidence 생성 방식이 아직 하나로 고정되지 않음

| 항목 | 내용 |
|---|---|
| 심각도 | Medium |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md:52`, `:65`, `:104-107` |
| 현상 | 조치 표는 option A/B를 열어두고, Hard Order는 option B를 권장한다. 하지만 option B의 구체 명령, snapshot cleanup, incremental cache 회피 방법이 아직 문서화되지 않았다. |
| 영향 | clean evidence를 만들었는데도 `xpm_fifo_async` compile line이 남지 않으면 R-C01-V004-02가 다시 열릴 수 있다. |
| 권장 수정 | option B로 확정하고, 재현 가능한 명령을 Plan에 포함한다. 예: 고유 snapshot 사용, `--incr` 제거, 기존 clean snapshot 디렉터리 제거 또는 새 snapshot명 사용, 로그 파일을 `elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log`로 저장. 검증 조건도 “compile line 1건 또는 ASYNC raw_cdc branch marker 1건”처럼 fallback을 명시한다. |

### P-C01-V002-04. `--negative` 인자는 실패 주입이 아니라 transcript 분기임을 더 명확히 해야 함

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md:49-50`, `:94-101` |
| 현상 | `--negative`가 negative test 자체를 수행하는 것처럼 읽힐 수 있지만, 실제 계획상 실패는 `run_regression.sh` 임시 injection으로 발생하고 `--negative`는 transcript 파일명 분기만 담당한다. |
| 권장 수정 | “`--negative`는 negative transcript 보존 모드이며, 실패 유발은 별도 injection 또는 test hook으로 수행한다”를 명시한다. |

## 3. 승인 전 체크리스트

Plan v002를 승인하기 전 다음 4개가 반영되면 좋다.

| 체크 | 필요 조치 |
|---|---|
| closure 문구 | v005용 최종 문구를 “보강 완료” 기준으로 단일화 |
| negative evidence | injection 위치를 summary 이전으로 명시하거나 test hook 방식으로 변경 |
| ASYNC evidence | option B로 확정하고 clean elab 재현 명령/cleanup/snapshot명을 명시 |
| negative mode 의미 | `--negative`가 transcript 분기임을 명확화 |

## 4. 조건부 승인 판단

위 4개가 반영되면 Plan v002는 승인 가능하다. 특히 본 계획은 RTL 변경을 동반하지 않고 evidence package만 보강하는 것이므로 C01 기능 안정성에 대한 위험은 낮다.

권장 승인 문구:

> Plan v002는 조건부 승인 가능하다. v005 closure 문구를 “보강 완료” 기준으로 단일화하고, negative injection 위치와 ASYNC clean elaboration 명령을 명확히 한 뒤 진행한다.

## 5. C02 진입 영향

이 Plan v002는 C02 진입을 막는 기능 결함을 다루는 계획이 아니라, C01 closure evidence를 더 단단하게 만드는 계획이다. 따라서 Plan 보완 후 실행하면 C01 closure package가 더 깔끔하게 닫히고, C02 진입 판단도 더 강해진다.
