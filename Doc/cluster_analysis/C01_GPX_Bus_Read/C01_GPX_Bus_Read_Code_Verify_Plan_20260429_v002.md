# C01_GPX_Bus_Read 코드 검증 조치 계획 v002

문서 버전: `v002`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 17:23:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v004.md`(R-C01-V004-01..03) + `C01_GPX_Bus_Read_Closure_Review_20260429_v001.md`(R-C01-CL-01..04) 사용자 리뷰 결과를 처리하기 위한 통합 조치 계획. 두 리뷰의 finding은 evidence 보존(negative test, ASYNC clean compile)과 closure 선언 문구 조정 항목에서 겹치므로 단일 plan으로 통합 처리한다. 절대 기준은 `Doc/TDC-GPX-Datasheet.pdf`이며, 본 계획은 RTL 변경을 동반하지 않고 회귀 운영 스크립트와 보고서 evidence만 보강한다.

본 plan v002는 운영 프로토콜 v006 section 5 “Review 처리 사이클 규칙”의 **첫 정상 적용** 사이클이다 (v001 plan은 retrospective였음). 진행 순서: Review 수신 → 본 Plan 작성 → 사용자 승인 → 진행/회귀 → Code_Verify v005 결과 보고서.

---

## 1. 대응 대상 리뷰 finding (통합)

| Finding | 심각도 | 출처 | 요지 |
|---|---|---|---|
| R-C01-V004-01 / R-C01-CL-02 | Low | Code_Verify_Review_v004, Closure_Review_v001 | `INTEGRATED EXIT CODE`와 negative test 흔적이 보존 artifact에서 재추적되지 않음. transcript에 부재. |
| R-C01-V004-02 / R-C01-CL-03 | Low | Code_Verify_Review_v004, Closure_Review_v001 | 최신 ASYNC elaboration log에 `xpm_fifo_async` compile line이 없음 (`xelab --incr` no-change 결과). |
| R-C01-V004-03 | Trivial | Code_Verify_Review_v004 | v004 보고서의 transcript 시작 시각이 실제 transcript와 11초 차이. |
| R-C01-CL-01 | Low | Closure_Review_v001 | closure 선언 문구 “운영 정합성 + 기능 PASS 모두 닫힘”은 evidence 추적성 기준에서 강함. 조건부 표현 권장. |
| R-C01-CL-04 | 없음 | Closure_Review_v001 | v006 / Plan / lineage 구조는 운영 규칙으로 수용 가능. **조치 불필요** (인정만 기록). |

각 finding의 사실관계 검증 (Codex 직접 확인):

| Finding | 검증 명령 | 결과 |
|---|---|---|
| R-C01-V004-01 / R-CL-02 | `grep -nE "INTEGRATED EXIT CODE\|NEGATIVE_TEST\|FAIL_COUNT=99\|stage1=1" tmp/c01_verify/run_c01_regression.log` | 0 match — **사실** |
| R-C01-V004-02 / R-CL-03 | `wc -l elab_tb_tdc_gpx_config_ctrl_ASYNC.log` -> 10줄, `grep xpm_fifo_async` -> 0 match | **사실** (incremental 캐시 재사용) |
| R-C01-V004-03 | transcript line 1: `start: ... 17:10:19 +0900` vs v004:99 `17:10:30 +0900` | **사실** — 11초 차이 |

---

## 2. Codex 동의 여부

| Finding | Codex 판단 | 근거 |
|---|---|---|
| R-C01-V004-01 / R-CL-02 | **동의** | 통합 entrypoint가 stdout/stderr만 transcript에 남기고, `bash` wrapper의 `$?` echo와 negative test injection은 별도 보존 안 됨. |
| R-C01-V004-02 / R-CL-03 | **동의** | `xelab --incr` 캐시로 인해 ASYNC FIFO compile evidence가 최신 log에 직접 남지 않음. mode echo 외에 `xpm_fifo_async` 활성 증거가 1차 자료로 부족. |
| R-C01-V004-03 | **동의** | v004 작성 시 transcript 실시각을 직접 인용하지 않고 추정 시각(`17:10:30`)을 적었음. |
| R-C01-CL-01 | **동의** | 사용자 제안 조정 문구가 evidence 보존 기준을 더 정확히 반영함. v005 결과 보고서 closure 절에 채택 예정. |
| R-C01-CL-04 | **인정** | v006 규칙은 본 plan v002의 정상 사이클 첫 적용으로 운영상 검증된다. |

---

## 3. 조치 계획 표

| # | 조치 항목 | 대상 파일 / 위치 | 변경 요지 | 추적 위치 (예상) | 영향 |
|---|---|---|---|---|---|
| 1 | Integrated entrypoint 종료코드 transcript 기록 | `HDL/scripts/run_c01_regression.sh` | 최종 판정 직후 `RC_FINAL`을 변수로 결정 → `echo "INTEGRATED EXIT CODE = ${RC_FINAL}"`을 transcript에 남기고 `exit ${RC_FINAL}`. | `run_c01_regression.sh:90-105` 영역 | transcript에 종료코드가 항상 보존. caller도 echo로 별도 확인 가능. |
| 2 | Negative test 별도 transcript 보존 | `HDL/scripts/run_c01_regression.sh` | 환경변수 `C01_NEGATIVE_TEST` 또는 첫 인자가 `--negative`이면 transcript 파일명을 `run_c01_regression_negative_<timestamp>.log`로 분기. positive 실행은 기존 `run_c01_regression.log`에 그대로 저장. | `run_c01_regression.sh:22-29` 영역 | positive log overwrite 안 됨. negative 실행은 timestamped 별도 파일. |
| 3 | Negative test 회귀 실행 (artifact 생성) | `tmp/c01_verify/run_regression.sh` 임시 주입 + `bash --negative` 실행 | (i) `run_regression.sh` 말미에 `FAIL_COUNT=99` injection, (ii) `bash run_c01_regression.sh --negative` 실행, (iii) negative transcript 생성 확인, (iv) injection 즉시 원복. | `tmp/c01_verify/run_c01_regression_negative_<timestamp>.log` (신규) | negative test artifact가 보존되어 R-C01-V004-01/CL-02 closure evidence 충족. 원복으로 정상 회귀 영향 없음. |
| 4 | Positive 회귀 재실행 (정상 transcript 갱신) | `bash run_c01_regression.sh` | `RC_FINAL` echo + 정상 ALL PASS transcript 갱신. | `tmp/c01_verify/run_c01_regression.log` | INTEGRATED EXIT CODE = 0 라인이 transcript에 명시적으로 보존됨. |
| 5 | ASYNC clean elaboration evidence | `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` 또는 별도 1회 실행 | 옵션 A: 해당 스크립트에 `--clean` 모드 추가해 `xelab` 호출 시 `--incr` 제거. 옵션 B: ASYNC mode 1회만 별도로 clean elab한 결과를 `elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log`로 저장. | `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` (신규) | ASYNC FIFO compile evidence가 1차 자료(=elab log)에 직접 남음. 일상 회귀에서는 incremental 유지. |
| 6 | v005 결과 보고서 작성 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` (신규) | (i) R-C01-V004-01..03, R-C01-CL-01..03 닫힘 기록, (ii) closure 문구 사용자 제안대로 채택, (iii) v004의 transcript 시작 시각 정정 이력 반영, (iv) 본 plan v002 → v005 forward-trace 추가. | (신규) | C01 closure evidence package 완성. |
| 7 | lineage 추가 | `... Code_Verify_20260429_v004.md`, `... Code_Verify_Plan_20260429_v002.md`, `... Code_Verify_Review_20260429_v004.md`, `... Closure_Review_20260429_v001.md` | v004 끝에 “v004 → v005 반영 위치 기록”, plan v002 끝에 “Plan v002 → Code_Verify v005 반영 위치 기록” 추가. Review 문서들에는 Codex 표준상 forward-trace를 안 쓰지만, v005 본문 “리뷰 처리 결과” 표에서 인용 닫힘 명시. | (각 문서 끝) | lineage 4축 (Review/Plan/Result/Protocol)이 완전 연결. |

---

## 4. 진행 순서 (Hard Order)

본 사이클은 v006 규칙의 첫 정상 적용이므로 순서를 더욱 엄격히 지킨다.

1. **본 Plan v002 사용자 승인 대기** (이 단계에서 plan 자체에 대한 피드백 수용).
2. 승인 후: 코드 변경 **#1 → #2 → #5** (스크립트 보강, 같은 파일 변경은 묶음).
3. 회귀 실행 **#3 (negative) → #4 (positive)** 순. negative 먼저 실행 후 즉시 injection 원복하여 다음 positive run의 정확성 보장.
4. ASYNC clean elaboration evidence 생성 **#5 옵션 B로 1회 실행** 권장 (회귀 시간 비대화 방지).
5. **#6 v005 결과 보고서** 작성. closure 문구는 사용자 제안 조정 문구 채택.
6. **#7 lineage** 추가 (v004 / Plan v002).

---

## 5. 위험 / 완화

| 위험 | 완화 |
|---|---|
| `--negative` 분기를 추가하면 positive 실행 경로가 달라져 기존 회귀에 영향 | 분기는 transcript 파일명만 바꾸고 실행 로직은 동일 유지. unit-test 패턴 동일. |
| Negative test injection 원복 누락 시 다음 positive run이 실제로 fail | injection 직후 `grep -nE "NEGATIVE_TEST\|FAIL_COUNT=99"`로 zero match 검증 후 다음 단계 진행. positive run 결과의 ALL PASS 메시지로 이중 확인. |
| Clean elaboration이 IP source 캐시를 무효화해 회귀 시간이 길어짐 | clean elab은 ASYNC 1회로만 수행하고 결과를 `_clean.log`로 보존. 기본 회귀는 incremental 유지. |
| transcript 파일명 시점화로 디스크 파일 누적 | negative log는 timestamp 기반, positive log는 overwrite. 별도 housekeeping 불필요. 향후 필요 시 c01_verify에 `*_negative_*.log` 패턴 cleanup 정책 추가. |
| `RC_FINAL` echo가 `tee` 후에 실행돼서 transcript에 안 잡힐 가능성 | `exec > >(tee ...)` 이후 `echo`는 stdout에 가므로 자동 redirect. 검증 시 transcript에 라인 존재 확인. |

---

## 6. 검증 방법

### Positive (#4)
- 종료코드: `bash HDL/scripts/run_c01_regression.sh; echo $?` -> `0`
- transcript 라인:
  - `[run_c01_regression] start: ...`
  - Stage 1/2/3 PASS 메시지
  - `[run_c01_regression] end: ...`
  - **`INTEGRATED EXIT CODE = 0`** (신규 라인)
- 파일 위치: `tmp/c01_verify/run_c01_regression.log` (overwrite)

### Negative (#3)
- injection: `run_regression.sh`에 임시 `FAIL_COUNT=99` + `FAIL_LIST` 식별 marker 추가
- 실행: `bash HDL/scripts/run_c01_regression.sh --negative; echo $?` -> `1`
- transcript 라인 (별도 파일):
  - Stage 1: `RESULTS: ... 99 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`
  - 통합 entrypoint: `# C01 regression: FAIL (stage1=1, stage2=0, stage3=0)`
  - **`INTEGRATED EXIT CODE = 1`**
- 파일 위치: `tmp/c01_verify/run_c01_regression_negative_<timestamp>.log` (신규)
- 원복 검증: `grep -nE "NEGATIVE_TEST|FAIL_COUNT=99|FORCED_FAIL" tmp/c01_verify/run_regression.sh` -> 0 match

### ASYNC clean evidence (#5)
- 실행: 한 번 ASYNC mode에 대해 `xelab` 호출에서 `--incr` 제거하고 `--snapshot tb_tdc_gpx_config_ctrl_ASYNC_clean_snap`으로 별도 elab.
- 검증: `grep -nE "Compiling module xpm.xpm_fifo_async" tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` -> 1 match
- mode echo + simulation marker는 기존 `sim_tb_tdc_gpx_config_ctrl_ASYNC.log` 그대로 활용.

### v005 결과 보고서 (#6) closure 문구 채택
- v005 closure 절에 다음 문장 그대로 채택:
  > C01 보완 검증은 기능 PASS와 회귀 운영 코드 보강 기준으로 닫혔습니다. v004 / v006 / Plan v001 / 주요 lineage는 연결되어 있으며, C02 진입은 가능합니다. 단, closure evidence 완성도를 위해 negative test transcript/exit code와 ASYNC FIFO generate 증거는 후속 v005로 보강 권장합니다.
- 본 v005에서는 “보강 권장”이 아니라 **“보강 완료”**로 갱신해 닫힘 표현 수정.

---

## 7. 산출물 (예상)

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md` | 본 문서 (계획) |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` | 결과 보고서 v005 (생성 예정) |
| `HDL/scripts/run_c01_regression.sh` | RC_FINAL echo + `--negative` 분기 |
| `HDL/tmp/c01_verify/run_c01_regression.log` | positive transcript (overwrite, INTEGRATED EXIT CODE 라인 포함) |
| `HDL/tmp/c01_verify/run_c01_regression_negative_<timestamp>.log` | negative transcript (신규, 보존) |
| `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` | ASYNC FIFO compile evidence (신규) |

---

## 8. 사용자 피드백 기록

이번 v002 plan 단계에서 사용자가 직접 입력한 피드백:

1. v004 결과 보고서에 대한 review 작성 (`Code_Verify_Review_20260429_v004.md`) — R-C01-V004-01..03 finding.
2. closure 선언 문구에 대한 별도 review 작성 (`Closure_Review_20260429_v001.md`) — R-C01-CL-01..04 finding과 조정 문구 제안.
3. “검토 결과 나왔으니 확인해줘” 명령으로 두 리뷰의 통합 처리 요청.

본 plan v002는 v006 규칙의 정상 사이클 첫 적용 — review 수신 → plan 작성 → 사용자 승인 대기 → 진행 → result 순서를 명시적으로 따른다.

---

## 9. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` (RTL 변경 동반 안 됨) |
| 사용자 review (Code Verify) | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v004.md` |
| 사용자 review (Closure) | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Closure_Review_20260429_v001.md` |
| 직전 결과 보고서 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` |
| 직전 plan (retrospective) | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` |
| 본 계획 적용 대상 스크립트 | `HDL/scripts/run_c01_regression.sh`, `HDL/tmp/c01_verify/run_regression.sh`, `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` |
| 운영 프로토콜 (Review 처리 사이클 규칙) | `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v006.md` section 5 |
| 검증 evidence (현재) | `HDL/tmp/c01_verify/run_c01_regression.log` (positive only), `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC.log` (incremental, 10줄) |
| 검증 evidence (예정) | `HDL/tmp/c01_verify/run_c01_regression_negative_<timestamp>.log`, `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` |

---

## 11. v002 -> v003 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Plan_Review_20260429_v002.md` 사용자 review (P-C01-V002-01..04). closure 문구 모순, negative injection 위치 모호, ASYNC clean evidence option 미확정, `--negative` 인자 의미 모호의 4가지 정정 요구. |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md` |
| 다음 버전 반영 위치 | section 1 v002→v003 갱신 사유 표, section 3 #2 / #2-hook (P-C01-V002-02 env hook 채택), section 3 #5 + section 3.1 (P-C01-V002-03 option B 확정 + 구체 명령 + fallback), section 4 (P-C01-V002-01 closure 문구 단일안), section 5/6/7 진행 순서/위험/검증 정합성 갱신 |
| 판단 변화 | v002의 “말미에 FAIL_COUNT=99 injection” 패턴 폐기 → 영구 env hook으로 전환 (P-C01-V002-02). v002의 “v005에서 보강 권장 → 보강 완료로 갱신” 모순 제거, section 4 단일안으로 고정 (P-C01-V002-01). v002의 option A/B 미확정 → option B 확정 + 구체 xelab 명령 + Primary/Fallback 검증 정책 추가 (P-C01-V002-03). v002의 `--negative` 책임 모호 → transcript 분기 전용으로 분리 (P-C01-V002-04). 결과 보고서 finding coverage는 동일 (R-C01-V004-01..03 / R-C01-CL-01..04). |
| 추적 근거 | `... Code_Verify_Plan_Review_20260429_v002.md` (P-C01-V002-01..04), 본 v002 line 50 / 52 / 65 / 109-112 (정정 대상 위치), v003 본 plan section 3-7 |
