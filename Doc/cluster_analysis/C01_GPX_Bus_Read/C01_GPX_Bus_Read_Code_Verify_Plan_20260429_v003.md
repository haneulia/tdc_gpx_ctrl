# C01_GPX_Bus_Read 코드 검증 조치 계획 v003

문서 버전: `v003`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 17:29:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Plan_Review_20260429_v002.md`(P-C01-V002-01..04) 사용자 plan 리뷰를 반영해 Plan v002의 모호함을 정정한 갱신판. v002의 구조와 finding coverage는 유지하고, closure 문구·negative injection·ASYNC clean evidence·`--negative` 인자 의미 4가지를 단일안으로 확정한다. 본 plan은 RTL 변경을 동반하지 않고 회귀 운영 스크립트와 evidence 보존 인프라만 보강한다.

---

## 1. v002 -> v003 갱신 사유

본 v003는 v002의 4가지 모호점을 닫는 것이 유일한 차이다. 처리할 사용자 review finding (R-C01-V004-01..03 / R-C01-CL-01..04)은 v002와 동일하다.

| Plan 리뷰 finding | v003 반영 위치 |
|---|---|
| P-C01-V002-01 (closure 문구 “보강 권장” + “보강 완료” 동시 사용) | section 6 closure 문구 — 단일안 채택 |
| P-C01-V002-02 (negative injection 위치 모호) | section 3 #2 / #3, section 6 negative 검증 — env hook 채택 |
| P-C01-V002-03 (ASYNC clean elab option A/B 미확정) | section 3 #5, section 6 ASYNC 검증 — option B 확정 + 구체 명령 |
| P-C01-V002-04 (`--negative` 의미 모호) | section 3 #2 / section 6 — transcript 분기 전용으로 책임 분리 |

---

## 2. 대응 대상 사용자 review finding (v002와 동일)

| Finding | 심각도 | 출처 | 요지 |
|---|---|---|---|
| R-C01-V004-01 / R-C01-CL-02 | Low | Code_Verify_Review_v004, Closure_Review_v001 | INTEGRATED EXIT CODE / negative test 흔적 보존 부재 |
| R-C01-V004-02 / R-C01-CL-03 | Low | 동상 | ASYNC clean elaboration log에 `xpm_fifo_async` compile 미존재 |
| R-C01-V004-03 | Trivial | Code_Verify_Review_v004 | v004 보고서 transcript 시작 시각 11초 차이 |
| R-C01-CL-01 | Low | Closure_Review_v001 | closure 문구 “모두 닫힘” 조건부 표현 권장 |
| R-C01-CL-04 | 없음 | Closure_Review_v001 | v006 / Plan / lineage 구조 인정 — 조치 불필요 |

---

## 3. 조치 계획 (v003 단일안)

| # | 조치 항목 | 대상 파일 / 위치 | 변경 요지 (v003 단일안) | 추적 위치 (예상) | 영향 |
|---|---|---|---|---|---|
| 1 | INTEGRATED EXIT CODE를 transcript에 명시 | `HDL/scripts/run_c01_regression.sh` | 최종 판정 직후 `RC_FINAL`을 변수로 결정 → `echo "INTEGRATED EXIT CODE = ${RC_FINAL}"`을 transcript에 남기고 `exit ${RC_FINAL}`. v004 보고서의 stdout-only 추적성 결함 해소. | `run_c01_regression.sh:90-110` 영역 | transcript 마지막 줄에 종료코드 명시. caller도 echo로 별도 확인 가능. |
| 2 | `--negative` 인자: **transcript 파일명 분기 전용** (P-C01-V002-04) | `HDL/scripts/run_c01_regression.sh` | 첫 인자가 `--negative`이면 transcript 파일명을 `run_c01_regression_negative_<UTC_ISO_TS>.log`로 분기. positive log는 기존 `run_c01_regression.log`로 그대로 저장. **`--negative`는 실패를 유발하지 않는다.** 실패 유발은 #2-hook으로 분리. | `run_c01_regression.sh:22-32` 영역 | 인자의 책임이 단일화됨. 회귀 logic은 변경 없음. |
| 2-hook | Negative test hook (P-C01-V002-02) | `HDL/tmp/c01_verify/run_regression.sh` | 환경변수 `C01_FORCE_NEGATIVE_STAGE1` 영구 hook을 **summary echo 직전**(현 line 109 위치)에 삽입: `if [ "${C01_FORCE_NEGATIVE_STAGE1:-0}" = "1" ]; then FAIL_COUNT=$((FAIL_COUNT+1)); FAIL_LIST="${FAIL_LIST} __NEGATIVE_TEST_FORCED_FAIL"; fi`. 임시 파일 수정/원복 패턴 폐기. | `run_regression.sh:108` 직전 (영구) | summary 출력에 `RESULTS: ... 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`가 정확히 남고, exit 1로 전파됨. positive 실행 (`C01_FORCE_NEGATIVE_STAGE1` 미설정)은 영향 없음. |
| 3 | Negative test 실행 (P-C01-V002-02 단일안) | invocation only | 명령: `C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative`. 결과 검증 후 환경변수만 unset. **파일 임시 변경 없음 → 원복 절차 불필요**. | `tmp/c01_verify/run_c01_regression_negative_<TS>.log` (신규) | negative artifact 보존, 정상 회귀에 영향 0. 재현 가능. |
| 4 | Positive 회귀 재실행 | invocation only | 명령: `bash HDL/scripts/run_c01_regression.sh`. transcript에 `INTEGRATED EXIT CODE = 0`이 신규 라인으로 포함. | `tmp/c01_verify/run_c01_regression.log` (overwrite) | 정상 ALL PASS + 종료코드 명시. |
| 5 | ASYNC clean elaboration evidence (P-C01-V002-03 option B 확정) | (1회 실행) | 명령: `xelab --debug typical --relax --mt 2 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm -L work --snapshot tb_tdc_gpx_config_ctrl_ASYNC_clean_snap -generic_top "g_DUT_STREAM_CLK_MODE=ASYNC" xil_defaultlib.tb_tdc_gpx_config_ctrl work.glbl -log elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log`. **`--incr` 제거 + 신규 snapshot 이름 사용 (기존 캐시 미충돌)**. arg-file 형식이 안정적이므로 `__elab_ASYNC_clean.f` 별도 파일로 패키징 가능. | `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` (신규) | clean compile log에 `xpm_fifo_async` 라인 직접 포함 예상. 일상 회귀는 incremental 그대로. |
| 6 | v005 결과 보고서 작성 (P-C01-V002-01 단일 closure 문구) | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` (신규) | (i) R-C01-V004-01..03, R-C01-CL-01..03 닫힘 기록, (ii) v004 transcript 시각 정정 이력 반영(R-C01-V004-03), (iii) closure 문구는 **section 4의 단일안 채택**. (iv) v002 plan에 명시한 “보강 권장”은 v005 시점 “보강 완료”로 사용하지 않는다. | (신규) | C01 closure evidence package 완성. |
| 7 | lineage 추가 | `... Code_Verify_20260429_v004.md`, `... Code_Verify_Plan_20260429_v002.md`, `... Code_Verify_Plan_20260429_v003.md` (본 문서) | v004 끝에 v005 forward-trace, plan v002 끝에 v003 forward-trace, 본 plan v003 끝에 v005 forward-trace 추가. Review/Closure_Review 문서는 forward-trace 안 쓰지만 v005 본문 “리뷰 처리 결과” 표에서 인용 닫힘 명시. | (각 문서 끝) | lineage 4축 (Review/Plan/Result/Protocol) 완전 연결. |

### 3.1 검증 fallback (P-C01-V002-03 강화)

ASYNC clean elaboration log에서 `Compiling module xpm.xpm_fifo_async`가 직접 나오지 않는 경우 (xelab 캐시 정책 변경 가능성 대비), 다음 중 하나로 ASYNC FIFO generate를 입증한다.

1. **Primary**: `grep -nE "Compiling module xpm.xpm_fifo_async" elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` -> 1 match 이상.
2. **Fallback A**: simulation log에서 ASYNC branch marker 확인. TB 또는 RTL에 다음 type의 report를 임시 추가하지 않고도, `gen_raw_async` generate 안의 `xpm_fifo_async` 인스턴스 elaboration 트레이스가 `xsim` 시작 시점 또는 `--debug typical` log에서 관찰될 수 있음.
3. **Fallback B**: `xsim` simulation log에 ASYNC mode marker (`TB: g_DUT_STREAM_CLK_MODE = ASYNC`)와 함께 raw_axis_tready/tvalid handshake가 xpm_fifo_async 경로로 들어가는 것이 mode echo 결과와 모순 없는지를 cross-check.

Primary가 작동하지 않으면 v005에서 fallback 사용 사실을 명시하고 그 근거 라인을 인용한다.

---

## 4. v005 closure 문구 단일안 (P-C01-V002-01)

v005 결과 보고서 closure 절에 **다음 문장 그대로** 채택한다. v002 plan의 “v005에서는 보강 완료로 갱신” 표현은 폐기.

> C01 보완 검증은 기능 PASS, 회귀 운영 코드 보강, negative test transcript/exit code 보존, ASYNC FIFO generate evidence 보강까지 완료되었습니다. v004 / v006 / Plan v001/v002/v003 / 주요 lineage는 연결되어 있으며, C02 진입 가능합니다.

조건:
- 위 문장은 v005 작성 시점에 #1~#5가 모두 PASS인 경우에만 채택한다.
- 어느 하나라도 미달이면 v005에서 closure 문구를 “미달 항목 명시 + 후속 v006 plan 권장”으로 자동 다운그레이드.
- 단일안 채택은 plan 단계에서 미리 약속함으로써 결과 보고서 단계에서 문구 모호성이 다시 발생하지 않도록 한다.

---

## 5. 진행 순서 (Hard Order)

1. **본 Plan v003 사용자 승인 대기** (v002 plan은 v003으로 갱신 후 폐기 — v002 끝에 forward-trace).
2. 코드 변경 — **#1 RC_FINAL echo → #2 `--negative` 분기 → #2-hook env hook** 묶음으로 한 번에 적용.
3. Negative test 실행 — **#3** (`C01_FORCE_NEGATIVE_STAGE1=1 bash run_c01_regression.sh --negative`). transcript에 `RESULTS: ... 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`, `INTEGRATED EXIT CODE = 1` 확인.
4. Positive 재실행 — **#4** (`bash run_c01_regression.sh`). transcript에 ALL PASS + `INTEGRATED EXIT CODE = 0` 확인.
5. ASYNC clean elaboration — **#5** 1회 실행. log에 `xpm_fifo_async` compile line 확인 (또는 fallback).
6. **#6 v005 결과 보고서** 작성 (closure 문구는 section 4 단일안 채택).
7. **#7 lineage** 추가 (v004 / Plan v002 / Plan v003).

`#3 → #4` 순서가 중요하다. negative 먼저 실행해서 hook이 정상 작동함을 확인한 뒤, hook 미설정 (또는 unset) 상태로 positive 실행해 평소 회귀가 영향 없음을 보장.

---

## 6. 위험 / 완화

| 위험 | 완화 |
|---|---|
| `--negative` 인자가 fail 자체를 유발하는 것으로 오해 (P-C01-V002-04) | section 3 #2와 본 section에 “transcript 분기 전용” 명시. fail 유발은 `C01_FORCE_NEGATIVE_STAGE1` hook으로 분리. |
| Hook이 영구 코드라 운영 회귀에 영향 | hook은 `${C01_FORCE_NEGATIVE_STAGE1:-0}=1`일 때만 활성. 미설정 (default `0`)이면 회귀 로직 변경 없음. positive run 결과로 검증. |
| ASYNC clean elaboration 캐시 충돌 (snapshot 동일 이름) | 신규 snapshot 이름 `tb_tdc_gpx_config_ctrl_ASYNC_clean_snap`로 기존 incremental snapshot과 격리. |
| ASYNC clean log에 `xpm_fifo_async` compile line이 안 남는 경우 (xelab 동작 변경) | section 3.1 fallback 정책 적용. v005에서 fallback 사용 사실 명시. |
| Closure 문구가 결과 보고서 작성 시 다시 모호해질 수 있음 | section 4 단일안을 plan 단계에서 미리 fix. v005 작성자(Codex)는 단일안 외 표현을 쓰지 않는다. |
| transcript 파일 누적 (negative log timestamped) | negative log는 timestamp 기반. 실제 운영에서는 회귀 후 cleanup 정책 추가 가능 (현 plan 범위 밖). |

---

## 7. 검증 방법 (v003 단일안)

### Positive (#4)
- 명령: `bash HDL/scripts/run_c01_regression.sh`
- 종료코드: `$?` -> `0`
- transcript (`tmp/c01_verify/run_c01_regression.log`):
  - `[run_c01_regression] start: ...`
  - `Stage 1: ... RESULTS: 4 ok, 0 failed`
  - `Stage 2: ... 2 pass, 0 fail`
  - `Stage 3: ... ALL TESTS PASSED ... cases=12`
  - `[run_c01_regression] end: ...`
  - `# C01 regression: ALL PASS`
  - **`INTEGRATED EXIT CODE = 0`** (신규)

### Negative (#3)
- 명령: `C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative`
- 종료코드: `$?` -> `1`
- transcript (`tmp/c01_verify/run_c01_regression_negative_<TS>.log`, 신규 파일):
  - Stage 1 summary: `RESULTS: 4 ok, 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`
  - Stage 1 종료코드: 1 (sub-script exit)
  - 통합 entrypoint: `# C01 regression: FAIL (stage1=1, stage2=0, stage3=0)`
  - **`INTEGRATED EXIT CODE = 1`** (신규)
- 환경변수 unset (`unset C01_FORCE_NEGATIVE_STAGE1`) — 다음 positive run에 영향 없음. 파일 변경/원복 불필요.

### ASYNC clean evidence (#5)
- 명령: section 3 #5의 xelab 명령 그대로 (단발 실행).
- Primary 검증: `grep -nE "Compiling module xpm.xpm_fifo_async" tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` -> 1 match 이상.
- Primary 실패 시 section 3.1 fallback 적용.

### v005 closure 문구 (#6)
- v005 closure 절은 section 4 단일안 그대로.
- 미달 항목 발생 시 자동 다운그레이드 (section 4 조건).

---

## 8. 산출물 (예상, v002와 동일 + Plan v003)

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md` | 본 문서 (v002 정정판) |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md` | v002 끝에 “v002 -> v003 반영 위치 기록” 추가 (v002 본문은 보존) |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` | 결과 보고서 (생성 예정) |
| `HDL/scripts/run_c01_regression.sh` | RC_FINAL echo + `--negative` transcript 분기 |
| `HDL/tmp/c01_verify/run_regression.sh` | `C01_FORCE_NEGATIVE_STAGE1` hook 영구 삽입 |
| `HDL/tmp/c01_verify/run_c01_regression.log` | positive transcript (overwrite, EXIT CODE 라인 포함) |
| `HDL/tmp/c01_verify/run_c01_regression_negative_<TS>.log` | negative transcript (신규, 보존) |
| `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` | ASYNC FIFO compile evidence (신규) |

---

## 9. 사용자 피드백 기록

이번 v003 plan 단계에서 사용자가 직접 입력한 피드백:

1. v002 plan에 대한 review (`Code_Verify_Plan_Review_20260429_v002.md`) — P-C01-V002-01..04 제기.
2. P-C01-V002-02 추가 의견: “더 안전한 대안은 임시 파일 수정 대신 `C01_FORCE_NEGATIVE_STAGE1=1` 같은 명시적 test hook을 sub-script에 추가하고, negative 실행 때만 활성화하는 방식이다.” → 본 v003 plan에 정확히 채택.
3. P-C01-V002-04: “`--negative`는 negative transcript 보존 모드이며, 실패 유발은 별도 injection 또는 test hook으로 수행한다”는 명확화 권장 → section 3 #2 / #2-hook으로 책임 분리 명시.
4. P-C01-V002-01 closure 문구 단일안 사용자 제안: section 4에 그대로 채택.

---

## 10. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` |
| 사용자 review (Code Verify) | `... C01_GPX_Bus_Read_Code_Verify_Review_20260429_v004.md` |
| 사용자 review (Closure) | `... C01_GPX_Bus_Read_Closure_Review_20260429_v001.md` |
| 사용자 review (Plan v002) | `... C01_GPX_Bus_Read_Code_Verify_Plan_Review_20260429_v002.md` |
| 직전 결과 보고서 | `... C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` |
| 직전 plan (retrospective) | `... C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` |
| 직전 plan (정상 사이클 첫 적용, 본 v003에서 정정 대상) | `... C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002.md` |
| 본 계획 적용 대상 스크립트 | `HDL/scripts/run_c01_regression.sh`, `HDL/tmp/c01_verify/run_regression.sh` |
| 운영 프로토콜 (Review 처리 사이클 규칙) | `... cluster_analysis_operating_protocol_20260429_v006.md` section 5 |
| 검증 evidence (현재) | `tmp/c01_verify/run_c01_regression.log`, `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC.log` (incremental) |
| 검증 evidence (예정) | `tmp/c01_verify/run_c01_regression_negative_<TS>.log`, `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` |

---

## 11. v003 -> v004 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Plan_Review_20260429_v003.md` 사용자 review (P-C01-V003-01..04). Windows-safe timestamp, ASYNC clean 실행 경로 절대 명시, Fallback 강도 강화, shell 명시 4가지 정정 요구. |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v004.md` |
| 다음 버전 반영 위치 | section 1 v003->v004 갱신 사유 표, section 3 #2 (timestamp 형식 `$(date -u '+%Y%m%dT%H%M%SZ')`), section 3 #2-marker (ASYNC RTL branch marker 신규 추가), section 3 #3/#4 shell 명시 (Git Bash + PowerShell wrapper), section 3.1 Fallback 강도 강화 (mode echo + handshake fallback 제외, Primary-A/Primary-B 둘 중 하나), section 3.2 ASYNC clean elab wrapper 명시 (SIM_DIR/LOG_DIR/cd/-log 절대 경로), section 4 다운그레이드 조건 갱신 (Primary-A AND Primary-B 둘 다 FAIL이 다운그레이드 트리거), section 6 위험 표 갱신 (RTL marker / timestamp / shell), section 7 검증 명령 갱신, section 8 산출물 표에 RTL marker 항목 추가 |
| 판단 변화 | v003의 ISO timestamp `<UTC_ISO_TS>` placeholder → 명시적 `YYYYMMDDTHHMMSSZ` (P-C01-V003-01). v003의 상대 경로 xelab 명령 → SIM_DIR/LOG_DIR 절대 경로 wrapper (P-C01-V003-02). v003의 Fallback B (mode echo + handshake) → fallback에서 제외, RTL branch marker (Primary-B)로 직접 증거 확보 (P-C01-V003-03). v003의 bash prefix syntax → Git Bash 실행 명시 + PowerShell wrapper (P-C01-V003-04). v003의 단일안 (closure 문구, --negative 분리, env hook, option B)은 그대로 유지. |
| 추적 근거 | `... Code_Verify_Plan_Review_20260429_v003.md` (P-C01-V003-01..04), 본 v003 line 40 / 42 / 44 / 53-54 (정정 대상 위치), v004 본 plan section 3 / 3.1 / 3.2 / 4 / 6 / 7 / 8 |
