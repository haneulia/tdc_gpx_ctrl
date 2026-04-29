# C01_GPX_Bus_Read 코드 검증 조치 계획 v001

문서 버전: `v001`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 17:00:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md` (R-C01-V003-01..03) 사용자 리뷰 결과에 따른 조치 계획을 사전에 문서화하고, Codex의 동의 여부, 변경 위치, 검증 방법, 위험을 사용자가 추적/승인할 수 있게 한다. 절대 기준은 `Doc/TDC-GPX-Datasheet.pdf`이며, 본 계획은 RTL 변경을 동반하지 않고 회귀 운영 스크립트와 보고서만 수정한다.

---

## 1. 대응 대상 리뷰 finding

| Finding | 심각도 | 요지 | 사실관계 검증 |
|---|---|---|---|
| R-C01-V003-01 | Medium | `run_c01_regression.sh`의 Stage 1/2가 sub-script 종료코드(`RC1`/`RC2`)에 의존하지만, 두 sub-script가 내부 FAIL을 카운트만 할 뿐 `exit 1`을 반환하지 않음. 통합 entrypoint가 실패를 마스킹할 위험. | 확인됨 — `tmp/c01_verify/run_regression.sh:108-115`, `tmp/c01_verify/run_config_ctrl_two_modes.sh:90-93`에 `exit` 부재. |
| R-C01-V003-02 | Low | `run_regression.sh`가 explicit PASS marker가 없는 경우에도 PASS_COUNT를 증가시킴. 추가로, 정규식 `tb_.*PASS`(소문자)가 `TB: PASS`(대문자) marker를 매치하지 못해 config_ctrl이 매번 `RAN(no-marker)`로 분류됨. | 확인됨 — `tmp/c01_verify/run_regression.sh:95-105`. |
| R-C01-V003-03 | Low | 통합 entrypoint의 console transcript가 별도 파일로 보존되지 않아 기록 추적이 어렵다. | 확인됨 — `tmp/c01_verify` 내 통합 transcript 파일 부재. |

각 finding 모두 **운용/회귀 스크립트의 신뢰성 문제**이며, 현재까지 닫힌 기능 PASS (12/12 cases, ALL TESTS PASSED) 자체를 뒤집지는 않는다. 그러나 “종료 코드 기반 통합 PASS/FAIL 판정”이라는 v003 보고서 표현의 신뢰성을 확보하려면 보강 필요.

---

## 2. Codex 동의 여부

| Finding | Codex 판단 | 근거 |
|---|---|---|
| R-C01-V003-01 | **동의** | 두 sub-script가 `exit` 없이 echo만 하고 끝남. `bash`의 마지막 명령 종료코드(`echo` = 0)가 $?로 잡혀 통합 entrypoint가 성공으로 오판할 수 있음. 회귀 마스킹 risk는 실재. |
| R-C01-V003-02 | **동의** | 두 가지 결함 모두 확인. 첫째 regex `tb_.*PASS`는 소문자만 매치하므로 config_ctrl의 `Note: TB: PASS -- init complete` marker를 못 잡음. 둘째 no-marker → PASS 정책은 self-check 누락된 TB가 묵시적으로 통과되는 위험. |
| R-C01-V003-03 | **동의** | 개별 stage 로그(sim_*.log, elab_*.log)는 있으나 통합 console 출력은 휘발성. v003 보고서가 인용한 “C01 regression: ALL PASS” 메시지의 시점/run의 추적 근거가 약함. |

---

## 3. 조치 계획 표

| # | 조치 항목 | 대상 파일 | 변경 요지 | 추적 위치 (예상) | 영향 |
|---|---|---|---|---|---|
| 1 | run_regression.sh 종료코드 전파 | `HDL/tmp/c01_verify/run_regression.sh` | 말미에 `if [ ${FAIL_COUNT} -ne 0 ]; then exit 1; fi; exit 0` 추가 | `run_regression.sh:118-122` (예상) | 통합 entrypoint가 Stage 1 실패를 정확히 받아냄. 기존 PASS 동작은 동일. |
| 2 | run_config_ctrl_two_modes.sh 종료코드 전파 | `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | 말미에 동일 패턴 추가 | `run_config_ctrl_two_modes.sh:95-99` (예상) | Stage 2 실패가 정확히 전파됨. |
| 3 | regex marker 보완 | `HDL/tmp/c01_verify/run_regression.sh` | grep regex에 `TB:.*PASS` alternation 추가 | `run_regression.sh:95-96` 영역 | config_ctrl marker 정상 인식. 다른 TB는 영향 없음. |
| 4 | no-marker → FAIL 정책 격상 | `HDL/tmp/c01_verify/run_regression.sh` | else 분기를 `FAIL_COUNT++` + `RESULTS`에 fail 기록으로 변경. allow-list가 필요한 TB는 명시적 등록 (현재는 없음) | `run_regression.sh:100-105` 영역 | self-check 없는 TB는 fail. 현재 모든 TB는 marker가 있으므로 정상 PASS 유지. |
| 5 | 통합 transcript 보존 | `HDL/scripts/run_c01_regression.sh` | 시작부에 `TRANSCRIPT="${LOG_DIR}/run_c01_regression.log"` 정의 후 `exec > >(tee "${TRANSCRIPT}") 2>&1`로 redirect. 시작/종료 timestamp 기록. | `run_c01_regression.sh:22-29`, `run_c01_regression.sh:99` (예상) | 통합 console transcript가 파일로 보존. terminal 출력은 그대로 유지. |
| 6 | 정상 회귀 재실행 | `HDL/scripts/run_c01_regression.sh` 실행 | 변경 후 `bash run_c01_regression.sh` 실행. 모든 stage PASS + 종료코드 0 + transcript 파일 생성 확인. | `tmp/c01_verify/run_c01_regression.log` | 변경이 PASS 동작을 깨뜨리지 않음을 검증. |
| 7 | 음성 회귀 (negative test) | 임시: `tb_tdc_gpx_csr_chip_clamp` 또는 sub-script에 의도적 fail 주입 | 한 case의 expected 값을 일부러 틀리게 만들거나, sub-script에서 강제로 `FAIL_COUNT=1` 설정 → 통합 entrypoint 실행 → 종료코드 1 확인 → 즉시 원복. | (한 회 임시 검증) | 실패 전파 경로의 실제 동작 검증. R-C01-V003-01의 “fail시 exit 1” 정책이 실제로 작동하는지 회귀 신뢰성 확보. |
| 8 | 보고서 v004 작성 + lineage | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` (신규) `... v003.md` (forward-trace 추가) `... Plan v001.md` (forward-trace 추가) | R-C01-V003-01..03 닫힘 기록, 종료코드/transcript 위치 명시, positive/negative 결과 요약, finding 갱신 표 | (신규/추가) | 운영 정합성 닫힘. |

---

## 4. 진행 순서 (Hard Order)

1. 코드 수정: **#1 → #2 → #3 → #4 → #5** 순서. 같은 파일 변경(#1+#3+#4)은 하나의 edit 묶음으로 처리.
2. **#6 Positive run**: 변경 후 `run_c01_regression.sh` 실행. 종료코드 0, ALL PASS, transcript 파일 생성 확인.
3. **#7 Negative run**: 의도적 fail 주입 → exit 1 확인 → **즉시 원복** → 한번 더 positive run.
4. **#8 보고서 작성**: v004 신규, v003에 forward-trace, 본 Plan v001에도 “Plan v001 -> v004 반영 위치 기록” lineage 추가.

#3과 #4는 순서 중요 — regex 보완(#3) 없이 no-marker 격상(#4)을 적용하면 config_ctrl이 잘못 fail로 잡힘.

---

## 5. 위험 / 주의

| 위험 | 완화 |
|---|---|
| `exec > >(tee ...)` redirect가 일부 환경에서 buffering 문제 유발 | 직접 검증해 transcript에 모든 stage 메시지가 포함되는지 확인. 누락 시 stage별 `tee` 분리로 fallback. |
| no-marker 격상으로 기존 PASS 회귀가 fail 분류될 가능성 | regex 보완 후 즉시 positive run으로 검증. 현재 제출된 marker 표 기준으로 기존 6 회귀 모두 PASS 유지 예상. |
| Negative test에서 원복 누락 시 다음 회귀가 실제로 fail | 원복 직후 grep으로 변경 흔적 zero 확인. Plan 문서에 “원복 검증” 단계 명시. |
| sub-script 종료코드 추가가 상위 다른 caller(예: 향후 CI)에 영향 | 현 시점에서 caller는 `run_c01_regression.sh`뿐. 다른 caller는 영향 없음. |

---

## 6. 검증 방법

### Positive (#6)
- 종료코드: `bash HDL/scripts/run_c01_regression.sh; echo $?` -> `0`
- ALL PASS marker:
  - Stage 1: `RESULTS: 4 ok, 0 failed`
  - Stage 2: `config_ctrl two-mode results: 2 pass, 0 fail`
  - Stage 3: `*** ALL TESTS PASSED *** (cases=12)`
  - 최종: `# C01 regression: ALL PASS`
- Transcript: `tmp/c01_verify/run_c01_regression.log` 존재 + 모든 stage 메시지 포함

### Negative (#7)
- 임시 주입 패턴: `run_regression.sh` 안에 한 줄 `FAIL_COUNT=1` 강제 추가
- 기대 결과:
  - Stage 1 마지막: `RESULTS: ... 1 failed`
  - Stage 1 종료코드: 1
  - 통합 entrypoint: `# C01 regression: FAIL (stage1=1, ...)` + 종료코드 1
- 원복 검증: `grep -n "FAIL_COUNT=1" run_regression.sh` -> 결과 없음

---

## 7. 산출물 (예상)

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` | 본 문서 (계획) |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` | 결과 보고서 v004 (생성 예정) |
| `HDL/tmp/c01_verify/run_regression.sh` | exit 코드 + regex + no-marker 정책 정정 |
| `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | exit 코드 정정 |
| `HDL/scripts/run_c01_regression.sh` | transcript 보존 |
| `HDL/tmp/c01_verify/run_c01_regression.log` | 통합 console transcript (신규 파일, 매 실행 overwrite) |

---

## 8. Plan 문서 운영에 대한 메모 (회고)

본 v003 리뷰 처리 과정에서 “조치 계획 문서를 cluster_analysis 폴더에 규칙대로 생성하지 않았다”는 사용자 지적이 있었다. 향후 review 처리 사이클은 다음과 같이 운영한다.

1. 사용자 review 문서 수신.
2. **Codex가 `..._Code_Verify_Plan_YYYYMMDD_vNNN.md` 신규 생성 → 사용자 승인 대기.**
3. 사용자 승인 후 코드 변경 및 회귀 진행.
4. 결과를 `..._Code_Verify_YYYYMMDD_vNNN.md` 신규 버전에 기록.
5. 직전 Plan/Verify 문서에 forward-trace lineage 추가.

이 운영 패턴은 cluster_analysis 운영 프로토콜 다음 버전(v006 후보)에 추가 권고할 수 있다.

---

## 9. 사용자 피드백 기록

이번 v001 plan 단계에서 사용자가 직접 입력한 피드백:

1. v003 리뷰 (`C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md`) 수신 후, Codex가 chat 응답으로 8건 조치 계획을 제시했고 사용자가 “모두 동의해 진행해줘”로 승인. 이후 사용자가 “수정 계획 문서를 왜 규칙에 맞게 생성하지 않은거야?”로 plan 문서 누락을 지적.
2. 본 v001 plan 문서는 위 지적 이후 retrospective로 작성되었으며, 향후 review 처리에서는 plan → 승인 → 진행 → result 사이클을 지키도록 본 문서 section 8에 명시.

---

## 10. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` (RTL 변경 동반 안 됨, 직접 적용은 없으나 절대 기준 유지) |
| 사용자 리뷰 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md` |
| 직전 검증 보고서 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v003.md` |
| 본 계획 적용 대상 스크립트 | `HDL/tmp/c01_verify/run_regression.sh`, `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh`, `HDL/scripts/run_c01_regression.sh` |
| 운영 프로토콜 | `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v005.md` (산출물 버전 규칙 / version lineage 기록 규칙 / 문서 수정 시간 기록 규칙) |

---

## 11. Plan v001 -> Code_Verify v004 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | 본 Plan 문서의 8개 조치 항목을 모두 적용한 결과 + 사용자 “3가지 다 진행해줘” 지시 (운영 프로토콜 업데이트 포함) |
| 반영된 결과 보고서 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` |
| 결과 보고서 반영 위치 | section 2.1 (#1 #2 종료코드 전파 + Negative test 결과), section 2.2 (#3 regex + #4 no-marker FAIL 격상), section 2.3 (#5 transcript 보존 — `tee` redirect), section 5 (#0 운영 프로토콜 v006 신규 — 본 Plan section 8 회고에서 제안한 사이클 규칙이 운영 프로토콜로 승격), section 6 허용 warning 표 갱신, section 7 finding 갱신, section 8 LTPI 누적표 갱신, section 12 변경 이력 |
| 판단 변화 | Plan section 3 표의 #1~#5 모두 적용. #6 Positive run으로 ALL PASS + 종료코드 0 확인. #7 Negative test로 강제 fail 주입 시 통합 entrypoint exit 1 + `# C01 regression: FAIL (stage1=1, ...)` 출력 확인 후 즉시 원복, 다시 positive PASS 복귀 검증. #8 v004 보고서 + v003 forward-trace + 본 Plan v001 forward-trace(이 표) + operating_protocol v005 -> v006 + v005 forward-trace 모두 작성. Plan 문서 자체는 retrospective 작성이었으나, 이 사실을 v004 section 5 / 9 / 12 와 protocol v006 section 8 timeline에 영구 기록하여 향후 사이클은 정상 순서를 지키도록 운영 프로토콜로 강제. |
| 추적 근거 | `tmp/c01_verify/run_regression.sh` (#1 exit 코드 + #3 regex + #4 no-marker FAIL), `tmp/c01_verify/run_config_ctrl_two_modes.sh` (#2 exit 코드), `scripts/run_c01_regression.sh` (#5 tee redirect), `tmp/c01_verify/run_c01_regression.log` (#6 transcript), Negative test 콘솔 출력 (#7 stage1=1 exit 1), `cluster_analysis_operating_protocol_20260429_v006.md` section 5 신규 sub-section, `cluster_analysis_operating_protocol_20260429_v005.md` 끝 forward-trace |
