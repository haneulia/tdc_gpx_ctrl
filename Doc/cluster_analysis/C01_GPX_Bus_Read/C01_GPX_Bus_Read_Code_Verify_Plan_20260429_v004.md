# C01_GPX_Bus_Read 코드 검증 조치 계획 v004

문서 버전: `v004`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 17:37:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Plan_Review_20260429_v003.md`(P-C01-V003-01..04) 사용자 plan 리뷰를 반영해 Plan v003의 4가지 모호점(Windows-safe timestamp, ASYNC clean 실행 경로, ASYNC fallback 강도, shell 명시)을 정정한 갱신판. v003의 단일안(P-C01-V002-01..04 처리)은 그대로 유지하고 실행 재현성과 evidence 강도만 강화한다.

---

## 1. v003 -> v004 갱신 사유

| Plan v003 리뷰 finding | v004 반영 위치 |
|---|---|
| P-C01-V003-01 (Windows-safe timestamp) | section 3 #2, section 7 negative 검증 — `$(date -u '+%Y%m%dT%H%M%SZ')` 채택 |
| P-C01-V003-02 (ASYNC clean 실행 경로 절대 명시) | section 3 #5, section 7 ASYNC 검증 — `SIM_DIR`/`LOG_DIR` wrapper 추가 |
| P-C01-V003-03 (Fallback B 약함) | section 3.1 — ASYNC RTL branch marker 영구 추가, mode echo + handshake fallback 제외 |
| P-C01-V003-04 (Shell 명시) | section 3 / section 7 — Git Bash (MSYS) 실행 명시, PowerShell wrapper 예시 |

처리할 사용자 review finding (R-C01-V004-01..03 / R-C01-CL-01..04)은 v003과 동일.

---

## 2. 대응 대상 사용자 review finding (v003과 동일)

| Finding | 심각도 | 출처 | 요지 |
|---|---|---|---|
| R-C01-V004-01 / R-C01-CL-02 | Low | Code_Verify_Review_v004, Closure_Review_v001 | INTEGRATED EXIT CODE / negative test 흔적 보존 부재 |
| R-C01-V004-02 / R-C01-CL-03 | Low | 동상 | ASYNC clean elaboration log에 `xpm_fifo_async` compile 미존재 |
| R-C01-V004-03 | Trivial | Code_Verify_Review_v004 | v004 보고서 transcript 시작 시각 11초 차이 |
| R-C01-CL-01 | Low | Closure_Review_v001 | closure 문구 “모두 닫힘” 조건부 표현 권장 |
| R-C01-CL-04 | 없음 | Closure_Review_v001 | v006 / Plan / lineage 구조 인정 — 조치 불필요 |

---

## 3. 조치 계획 (v004 단일안)

| # | 조치 항목 | 대상 파일 / 위치 | 변경 요지 (v004 단일안) | 영향 |
|---|---|---|---|---|
| 1 | INTEGRATED EXIT CODE를 transcript에 명시 | `HDL/scripts/run_c01_regression.sh` | 최종 판정 직후 `RC_FINAL`을 결정 → `echo "INTEGRATED EXIT CODE = ${RC_FINAL}"`을 transcript에 남기고 `exit ${RC_FINAL}`. | transcript 마지막 줄에 종료코드 명시. |
| 2 | `--negative` 인자: **transcript 파일명 분기 전용** | `HDL/scripts/run_c01_regression.sh` | 첫 인자가 `--negative`이면 `TS=$(date -u '+%Y%m%dT%H%M%SZ')`로 timestamp 생성, transcript 파일명을 `run_c01_regression_negative_${TS}.log`로 분기 (P-C01-V003-01 — Windows 파일명 safe, 콜론 없음). positive log는 기존 `run_c01_regression.log`. **`--negative`는 실패를 유발하지 않는다** (P-C01-V002-04). 실패 유발은 #2-hook 담당. | 파일명 안전. 책임 단일화. |
| 2-hook | Negative test hook | `HDL/tmp/c01_verify/run_regression.sh` | `if [ "${C01_FORCE_NEGATIVE_STAGE1:-0}" = "1" ]; then FAIL_COUNT=$((FAIL_COUNT+1)); FAIL_LIST="${FAIL_LIST} __NEGATIVE_TEST_FORCED_FAIL"; fi`를 **summary echo 직전** (현 line 109 위치, 즉 `echo ""`/`echo "===..."` 직전)에 영구 삽입. | summary에 `RESULTS: ... 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`가 정확히 출력되고 exit 1로 전파. positive 영향 0. |
| 2-marker | ASYNC RTL branch marker (P-C01-V003-03) | `HDL/tdc_gpx_config_ctrl.vhd` | `gen_raw_async : if g_STREAM_CLK_MODE = "ASYNC" generate ... begin` 안에 `synthesis translate_off / report "ASYNC raw_cdc FIFO generate active" severity note; / synthesis translate_on` 1줄 추가. simulation 시작 시 elaboration phase에서 1회 emit. **synthesis 영향 0** (translate_off로 합성 도구 무시). gen_raw_sync에는 추가 안 함. | ASYNC FIFO generate가 실제 elaboration된 1차 증거로 simulation log에서 `grep`으로 직접 확인 가능. R-C01-V004-02 / R-C01-CL-03 closure 조건 강화. |
| 3 | Negative test 실행 (Git Bash 실행 P-C01-V003-04) | invocation only | **Git Bash / MSYS bash에서 실행**: `C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative`. PowerShell wrapper: `bash -lc 'C01_FORCE_NEGATIVE_STAGE1=1 HDL/scripts/run_c01_regression.sh --negative'`. 실행 후 `unset C01_FORCE_NEGATIVE_STAGE1`. **파일 임시 변경 없음 → 원복 불필요**. | negative artifact 보존. shell 재현성 확보. |
| 4 | Positive 회귀 재실행 | invocation only | **Git Bash 실행**: `bash HDL/scripts/run_c01_regression.sh`. transcript에 `INTEGRATED EXIT CODE = 0` 신규 라인 포함. | 정상 ALL PASS + 종료코드 명시. |
| 5 | ASYNC clean elaboration evidence (P-C01-V003-02 절대 경로) | (1회 단발) | **Git Bash 실행, 절대 경로 wrapper**: 아래 section 3.2 참조. | clean compile log에 `xpm_fifo_async` line 직접 포함 (Primary). 또는 simulation log에 ASYNC branch marker 1+ match (대안 Primary). |
| 6 | v005 결과 보고서 작성 (closure 문구 단일안) | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` (신규) | (i) R-C01-V004-01..03, R-C01-CL-01..03 닫힘, (ii) v004 시각 정정, (iii) closure 문구는 section 4 단일안 채택. | C01 closure evidence package 완성. |
| 7 | lineage 추가 | `... v004.md`, `... Plan_v002.md`, `... Plan_v003.md`, `... Plan_v004.md` (본 문서) | 각 직전 버전 끝에 forward-trace 추가. | 4축 lineage 완전 연결. |

### 3.1 ASYNC FIFO generate evidence 검증 정책 (P-C01-V003-03 강화)

R-C01-V004-02 / R-C01-CL-03을 닫는 인정 조건을 다음 둘 중 **하나**로 제한한다. mode echo + handshake는 **closure 인정 fallback에서 제외**, 단순 보조 정보로만 사용.

| 인정 조건 | 검증 명령 / 위치 |
|---|---|
| **Primary-A** (xelab compile evidence) | `grep -nE "Compiling module xpm.xpm_fifo_async" "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"` -> 1 match 이상 |
| **Primary-B** (RTL branch marker, P-C01-V003-03 권장) | `grep -nE "ASYNC raw_cdc FIFO generate active" "${LOG_DIR}/sim_tb_tdc_gpx_config_ctrl_ASYNC.log"` -> 1 match 이상. marker는 #2-marker로 영구 추가됨. |

운영 가이드:
- v005 결과 보고서에서 인정 조건 중 **어느 것이 PASS인지 명시적으로 인용**한다.
- 둘 다 PASS면 “Primary-A + Primary-B 모두 PASS”로 강하게 닫는다.
- 둘 다 FAIL이면 closure 문구를 “보강 미완료, v005 추가 plan 권장”으로 자동 다운그레이드 (section 4 조건 참조).

### 3.2 ASYNC clean elab 실행 명령 (P-C01-V003-02 절대 경로 명시)

```bash
# Git Bash (MSYS) 실행
SIM_DIR="C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim"
LOG_DIR="C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tmp/c01_verify"
export PATH="/c/AMDDesignTools/2025.2.1/Vivado/bin:$PATH"
cd "${SIM_DIR}"

# arg-file로 패키징 (string generic 안전 처리, xsim debug quirks memory)
cat > "${SIM_DIR}/__elab_ASYNC_clean.f" <<EOF
--debug typical
--relax
--mt 2
-L xil_defaultlib
-L unisims_ver
-L unimacro_ver
-L secureip
-L xpm
-L work
-generic_top "g_DUT_STREAM_CLK_MODE=ASYNC"
--snapshot tb_tdc_gpx_config_ctrl_ASYNC_clean_snap
xil_defaultlib.tb_tdc_gpx_config_ctrl
work.glbl
EOF

# clean compile (--incr 제거, 신규 snapshot 이름 사용)
xelab -f "${SIM_DIR}/__elab_ASYNC_clean.f" \
      -log "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"

# Primary-A 검증
grep -nE "Compiling module xpm.xpm_fifo_async" \
     "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"
```

PowerShell 환경에서는 `bash -lc '...'` 또는 Git Bash 직접 실행 권장.

---

## 4. v005 closure 문구 단일안 (P-C01-V002-01 유지)

v005 결과 보고서 closure 절에 **다음 문장 그대로** 채택한다.

> C01 보완 검증은 기능 PASS, 회귀 운영 코드 보강, negative test transcript/exit code 보존, ASYNC FIFO generate evidence 보강까지 완료되었습니다. v004 / v006 / Plan v001/v002/v003/v004 / 주요 lineage는 연결되어 있으며, C02 진입 가능합니다.

자동 다운그레이드 조건 (v003에서 정의, v004에서 강화):
- #1 INTEGRATED EXIT CODE positive transcript에 보존되지 않음 → 다운그레이드.
- #3 negative transcript에 `INTEGRATED EXIT CODE = 1` 보존되지 않음 → 다운그레이드.
- #5 Primary-A **그리고** Primary-B 둘 다 FAIL → 다운그레이드 (Primary-A 또는 Primary-B 중 하나라도 PASS면 인정).
- 다운그레이드 시 closure 문구를 “미달 항목 명시 + 후속 plan 권장”으로 자동 변경.

---

## 5. 진행 순서 (Hard Order, v004)

1. **본 Plan v004 사용자 승인 대기**.
2. 코드 변경 — **#1 RC_FINAL echo + #2 `--negative` 분기 + #2-hook env hook + #2-marker ASYNC RTL marker** 묶음. RTL marker 변경은 `synthesis translate_off`로 둘러싸여 합성 영향 0임을 확인.
3. 회귀 재컴파일 (xvhdl --incr) — RTL marker 추가 반영.
4. **#3 Negative**: `C01_FORCE_NEGATIVE_STAGE1=1 bash run_c01_regression.sh --negative` (Git Bash). transcript에 `RESULTS: ... 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`, `INTEGRATED EXIT CODE = 1` 확인.
5. **#4 Positive**: `bash run_c01_regression.sh` (Git Bash). transcript에 ALL PASS + `INTEGRATED EXIT CODE = 0` 확인.
6. **#5 ASYNC clean elab + simulation marker** — section 3.2 명령 1회 실행. Primary-A 또는 Primary-B 검증.
7. **#6 v005 결과 보고서** 작성 (closure 문구 section 4 단일안, 다운그레이드 조건 점검).
8. **#7 lineage** 추가.

순서 #2 → #3 사이에 RTL marker 추가로 인한 재컴파일 필요. #3-#5는 동일 컴파일 결과를 사용.

---

## 6. 위험 / 완화

| 위험 | 완화 |
|---|---|
| ASYNC RTL marker가 합성/생산 RTL에 잔존 | `synthesis translate_off / translate_on` pragma로 둘러싸 합성 도구 무시. Vivado/유사 도구 표준 지원. |
| RTL marker 추가로 기존 회귀 영향 | severity note는 simulation에서 단순 출력. ALL PASS 검증의 PASS 판정에는 영향 없음. positive 회귀로 검증. |
| timestamp 파일명 collision (1초 이내 두 번 negative 실행) | 콜론 없는 ISO basic 형식 + 초 단위 정밀도. 동일 초 두 번 실행은 사용자 의도 시 발생 가능하나, 두 번째 실행이 첫 번째를 overwrite. 동시성 risk 낮음. |
| ASYNC clean elab 명령에 환경 변수 누락 (PowerShell 환경) | section 3.2와 section 5에 명시적으로 “Git Bash (MSYS)” 표기. PowerShell 사용자는 `bash -lc` wrapper 사용. |
| Primary-A 실패 + Primary-B 실패 동시 발생 시 closure 다운그레이드로 v005 재작성 부담 | Primary-A는 `--incr` 제거 + 신규 snapshot으로 거의 확실 PASS 예상. Primary-B는 RTL marker 영구 추가로 simulation log에서 항상 emit. 둘 다 FAIL은 design 문제 신호이므로 다운그레이드가 정확한 운영. |
| Hook이 다른 sub-script (예: `run_config_ctrl_two_modes.sh`)에는 없음 | 본 plan은 Stage 1 (`run_regression.sh`) hook만 추가. Stage 2/3는 별도 hook이 필요할 때 후속 plan에서 처리. v004 closure 범위는 Stage 1 한정. |

---

## 7. 검증 방법 (v004 단일안)

### Positive (#4)
- 명령: `bash HDL/scripts/run_c01_regression.sh` (Git Bash)
- 종료코드: `$?` -> `0`
- transcript (`tmp/c01_verify/run_c01_regression.log`):
  - `[run_c01_regression] start: ...`, `end: ...`
  - Stage 1/2/3 PASS
  - `# C01 regression: ALL PASS`
  - **`INTEGRATED EXIT CODE = 0`**
- ASYNC simulation log: `tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log`에 **`Note: ASYNC raw_cdc FIFO generate active`** 라인 1+ 포함 (Primary-B 일상 회귀 시점에서도 항상 보존).

### Negative (#3)
- 명령: `C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative` (Git Bash)
- PowerShell wrapper: `bash -lc 'C01_FORCE_NEGATIVE_STAGE1=1 HDL/scripts/run_c01_regression.sh --negative'`
- 종료코드: `$?` -> `1`
- transcript 파일명: `tmp/c01_verify/run_c01_regression_negative_<YYYYMMDDTHHMMSSZ>.log` (Windows-safe, 콜론 없음)
- transcript 내용:
  - Stage 1 summary: `RESULTS: 4 ok, 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`
  - Stage 1 sub-script exit code: 1
  - 통합: `# C01 regression: FAIL (stage1=1, stage2=0, stage3=0)`
  - **`INTEGRATED EXIT CODE = 1`**
- 환경변수 정리: `unset C01_FORCE_NEGATIVE_STAGE1`. 파일 변경 없음 → 원복 불필요.

### ASYNC clean evidence (#5)
- 명령: section 3.2 wrapper 그대로 (Git Bash 실행).
- Primary-A: `grep -nE "Compiling module xpm.xpm_fifo_async" "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"` -> 1+ match.
- Primary-B (보조 확정): `grep -nE "ASYNC raw_cdc FIFO generate active" "${LOG_DIR}/sim_tb_tdc_gpx_config_ctrl_ASYNC.log"` -> 1+ match. (positive run에서도 보존됨.)

### v005 closure 문구 (#6)
- section 4 단일안 그대로. 자동 다운그레이드 조건 발생 시 (Primary-A AND Primary-B 둘 다 FAIL 등) 명시적 미달 항목 표기로 변경.

---

## 8. 산출물 (예상, v003 기준 + RTL marker)

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v004.md` | 본 문서 |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md` | 끝에 v003 -> v004 forward-trace 추가 |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` | 결과 보고서 (생성 예정) |
| `HDL/scripts/run_c01_regression.sh` | RC_FINAL echo + Windows-safe TS + `--negative` 분기 |
| `HDL/tmp/c01_verify/run_regression.sh` | `C01_FORCE_NEGATIVE_STAGE1` env hook (영구) |
| `HDL/tdc_gpx_config_ctrl.vhd` | gen_raw_async 안에 `synthesis translate_off ... ASYNC raw_cdc FIFO generate active ... translate_on` marker (영구, 합성 영향 0) |
| `HDL/tmp/c01_verify/run_c01_regression.log` | positive transcript (overwrite) |
| `HDL/tmp/c01_verify/run_c01_regression_negative_<TS>.log` | negative transcript (Windows-safe TS, 신규 보존) |
| `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` | ASYNC clean elab evidence (Primary-A) |
| `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log` (기존) | ASYNC branch marker 라인 보존 (Primary-B) |

---

## 9. 사용자 피드백 기록

이번 v004 plan 단계에서 사용자가 직접 입력한 피드백:

1. v003 plan에 대한 review (`Code_Verify_Plan_Review_20260429_v003.md`) — P-C01-V003-01..04 finding.
2. P-C01-V003-01: timestamp Windows-safe → 본 v004에서 `$(date -u '+%Y%m%dT%H%M%SZ')` 형식 채택.
3. P-C01-V003-02: 실행 디렉터리/로그 절대 경로 → section 3.2 wrapper로 명시.
4. P-C01-V003-03: Fallback B는 약함, mode echo + handshake 보조 → section 3.1에서 fallback 제외, RTL marker (Primary-B) 추가로 직접 evidence 확보.
5. P-C01-V003-04: shell 명시 → 모든 명령에 “Git Bash 실행” 표기, PowerShell wrapper 예시 추가.

---

## 10. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` |
| 사용자 review (Code Verify) | `... Code_Verify_Review_20260429_v004.md` |
| 사용자 review (Closure) | `... Closure_Review_20260429_v001.md` |
| 사용자 review (Plan v002) | `... Code_Verify_Plan_Review_20260429_v002.md` |
| 사용자 review (Plan v003) | `... Code_Verify_Plan_Review_20260429_v003.md` |
| 직전 plan (정상 사이클 첫 적용) | `... Code_Verify_Plan_20260429_v002.md` |
| 직전 plan (v002 정정) | `... Code_Verify_Plan_20260429_v003.md` (본 v004에서 정정 대상) |
| 직전 결과 보고서 | `... Code_Verify_20260429_v004.md` |
| 본 계획 적용 대상 스크립트 | `HDL/scripts/run_c01_regression.sh`, `HDL/tmp/c01_verify/run_regression.sh` |
| 본 계획 적용 대상 RTL | `HDL/tdc_gpx_config_ctrl.vhd` (gen_raw_async block) |
| 운영 프로토콜 (Review 처리 사이클 규칙) | `cluster_analysis_operating_protocol_20260429_v006.md` section 5 |

---

## 11. v004 -> v005 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Plan_Review_20260429_v004.md` 사용자 review (P-C01-V004-01..03). 핵심: ASYNC marker plain `report` VHDL 컴파일 위험(High), emit 횟수 부정확, cwd 혼재. |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v005.md` |
| 다음 버전 반영 위치 | section 1 v004->v005 갱신 사유 표, section 3 #2-marker (concurrent assertion + `if i = 0 generate` 형태로 교체), section 3.1 (P-C01-V004-01 + V004-02 정정 — VHDL 합법 구현 + chip 0 1회 emit), section 3 #3/#4 (P-C01-V004-03 — IP root cwd 통일), section 5 진행 순서 (재컴파일 단계로 marker 컴파일 검증), section 6 위험 표 갱신, section 7 검증 명령에 RTL marker 컴파일 검증 단계 추가 |
| 판단 변화 | v004의 plain `report "..." severity note;` → concurrent `assert FALSE report "..." severity note;` 형태로 교체 (P-C01-V004-01). emit 횟수 “1회” 표현은 유지하되 구현은 `if i = 0 generate` nested generate로 보장 (P-C01-V004-02). cwd는 IP root (`C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl`)로 단일화, PowerShell wrapper도 동일 (P-C01-V004-03). v004의 단일안 (Windows-safe TS, `SIM_DIR`/`LOG_DIR` wrapper, env hook, `--negative` 분기, fallback 정책)은 그대로 유지. |
| 추적 근거 | `... Code_Verify_Plan_Review_20260429_v004.md` (P-C01-V004-01..03), 본 v004 line 42 (정정 대상 marker 위치), `tdc_gpx_pkg.vhd:45` (c_N_CHIPS = 4), `tdc_gpx_config_ctrl.vhd:1454` (gen_chip for loop), `tdc_gpx_config_ctrl.vhd:1834` (gen_raw_async block) |
