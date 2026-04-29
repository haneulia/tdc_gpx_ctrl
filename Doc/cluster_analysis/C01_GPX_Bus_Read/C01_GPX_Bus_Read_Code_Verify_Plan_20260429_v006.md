# C01_GPX_Bus_Read 코드 검증 조치 계획 v006

문서 버전: `v006`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 18:58:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Plan_Review_20260429_v005.md`(P-C01-V005-01..03) 사용자 plan 리뷰의 minor 권고 3건을 plan 본문에 직접 반영한 갱신판. v005의 모든 단일안과 진행 순서는 그대로 유지하고 marker VHDL 스타일·PowerShell wrapper·expected note 분류만 정정한다. 사용자가 옵션 B (운영 프로토콜 v006 strict 준수)로 진행 결정.

---

## 1. v005 -> v006 갱신 사유

| Plan v005 리뷰 finding | v006 반영 위치 |
|---|---|
| P-C01-V005-01 (nested generate에 `begin` 명시 — repo 스타일) | section 3 #2-marker, section 3.1 — `if i = 0 generate begin ... end generate` 형태로 통일 |
| P-C01-V005-02 (PowerShell wrapper 내부 `cd` 포함) | section 3 #3 / #4, section 7 — `bash -lc 'cd /c/... && ...'` 형태로 wrapper 강화 |
| P-C01-V005-03 (marker note를 expected evidence로 분류) | section 3.3, section 4, **section 6.1 신규** — v005 결과 보고서의 expected note 목록에 명시 |

처리할 사용자 review finding (R-C01-V004-01..03 / R-C01-CL-01..04)은 v005와 동일.

---

## 2. 대응 대상 사용자 review finding (v005와 동일)

| Finding | 심각도 | 출처 | 요지 |
|---|---|---|---|
| R-C01-V004-01 / R-C01-CL-02 | Low | Code_Verify_Review_v004, Closure_Review_v001 | INTEGRATED EXIT CODE / negative test 흔적 보존 부재 |
| R-C01-V004-02 / R-C01-CL-03 | Low | 동상 | ASYNC clean elaboration log에 `xpm_fifo_async` compile 미존재 |
| R-C01-V004-03 | Trivial | Code_Verify_Review_v004 | v004 보고서 transcript 시작 시각 11초 차이 |
| R-C01-CL-01 | Low | Closure_Review_v001 | closure 문구 “모두 닫힘” 조건부 표현 권장 |
| R-C01-CL-04 | 없음 | Closure_Review_v001 | v006 / Plan / lineage 구조 인정 — 조치 불필요 |

---

## 3. 조치 계획 (v006 단일안)

표준 cwd: 모든 invocation은 **IP root (`C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl`)** 기준.

| # | 조치 항목 | 대상 파일 / 위치 | 변경 요지 (v006 단일안) | 영향 |
|---|---|---|---|---|
| 1 | INTEGRATED EXIT CODE를 transcript에 명시 | `HDL/scripts/run_c01_regression.sh` | 최종 판정 직후 `RC_FINAL` 결정 → `echo "INTEGRATED EXIT CODE = ${RC_FINAL}"`을 transcript에 남기고 `exit ${RC_FINAL}`. | transcript 마지막 줄에 종료코드 명시. |
| 2 | `--negative` 인자: transcript 파일명 분기 전용 | `HDL/scripts/run_c01_regression.sh` | 첫 인자가 `--negative`이면 `TS=$(date -u '+%Y%m%dT%H%M%SZ')`로 timestamp 생성, transcript 파일명 `run_c01_regression_negative_${TS}.log`로 분기. positive log는 기존 `run_c01_regression.log`. **`--negative`는 실패를 유발하지 않는다.** | 책임 단일화. |
| 2-hook | Negative test hook | `HDL/tmp/c01_verify/run_regression.sh` | `if [ "${C01_FORCE_NEGATIVE_STAGE1:-0}" = "1" ]; then FAIL_COUNT=$((FAIL_COUNT+1)); FAIL_LIST="${FAIL_LIST} __NEGATIVE_TEST_FORCED_FAIL"; fi`를 summary echo 직전에 영구 삽입. | summary에 `RESULTS: ... 1 failed` 정확히 출력. positive 영향 0. |
| **2-marker** | **ASYNC RTL branch marker (P-C01-V005-01 정정)** | `HDL/tdc_gpx_config_ctrl.vhd` | **section 3.1의 nested generate + `begin` 명시 + concurrent assertion** 형태. `gen_raw_sync`/`gen_raw_async`의 repo 스타일과 동일하게 `generate` 다음 줄에 `begin` 명시. | concurrent 영역에서 VHDL 문법 OK. repo 스타일 일관성. simulation log에 chip 0 1회 emit. 합성 영향 0. |
| 3 | Negative test 실행 (P-C01-V005-02 정정) | invocation only | **Git Bash, IP root cwd**: `cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl && C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative`. **PowerShell wrapper는 `bash -lc` 안에 `cd` 포함**: `bash -lc 'cd /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl && C01_FORCE_NEGATIVE_STAGE1=1 HDL/scripts/run_c01_regression.sh --negative'`. 실행 후 `unset C01_FORCE_NEGATIVE_STAGE1`. | login shell 차이 회피. cwd 보장. |
| 4 | Positive 회귀 재실행 (P-C01-V005-02 정정) | invocation only | **Git Bash, IP root cwd**: `cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl && bash HDL/scripts/run_c01_regression.sh`. **PowerShell wrapper**: `bash -lc 'cd /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl && HDL/scripts/run_c01_regression.sh'`. | 정상 ALL PASS + `INTEGRATED EXIT CODE = 0` + ASYNC marker 1회 emit. |
| 5 | ASYNC clean elaboration evidence | (1회 단발) | **Git Bash, 절대 경로 wrapper** (cwd 무관, section 3.2 그대로). | clean compile log에 `xpm_fifo_async` line (Primary-A) 또는 simulation log에 ASYNC branch marker (Primary-B) 1+ match. |
| 6 | v005 결과 보고서 (closure 문구 + expected note 분류 P-C01-V005-03) | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` (신규) | (i) R-C01-V004-01..03 / R-C01-CL-01..03 닫힘 기록, (ii) v004 시각 정정, (iii) closure 문구는 section 4 단일안, **(iv) section 6.1 expected note 표에 “ASYNC raw_cdc FIFO generate active” 추가**. | C01 closure evidence package 완성. expected note 명시로 후속 review 혼선 방지. |
| 7 | lineage 추가 | `... v004.md`, `... Plan_v002.md..v006.md` | 각 직전 버전 끝에 forward-trace 추가. | 4축 lineage 완전 연결. |

### 3.1 ASYNC RTL branch marker 구현 (P-C01-V005-01 정정)

`tdc_gpx_config_ctrl.vhd`의 `gen_raw_async` block (line 1834+) **마지막 부분**에 다음 구조를 추가한다. **repo 스타일과 동일하게 `generate` 다음 줄에 `begin` 명시**.

```vhdl
            -- =========================================================
            -- ASYNC FIFO generate marker (sim-only, P-C01-V003-03 / V004-01 / V005-01)
            -- chip 0 에서만 1회 emit. concurrent assertion 형태이므로 generate
            -- body에 그대로 합법. severity note 라 simulation은 중지하지 않음.
            -- repo 스타일 일관성을 위해 nested generate에 begin 명시.
            -- =========================================================
            -- synthesis translate_off
            gen_async_marker : if i = 0 generate
            begin
                assert false
                    report "ASYNC raw_cdc FIFO generate active"
                    severity note;
            end generate gen_async_marker;
            -- synthesis translate_on
```

규칙:
- VHDL 주석 형태의 pragma 사용 (`-- synthesis translate_off` / `-- synthesis translate_on`).
- generate body 안에 nested generate (`if i = 0 generate begin`) — `begin` 명시 (repo 스타일 일관성, P-C01-V005-01).
- nested generate body에는 concurrent assertion (`assert false report "..." severity note;`) 사용. plain sequential `report` 미사용.
- `gen_raw_sync` block에는 추가 안 함 (SYNC mode에서는 emit 안 됨).

검증:
- `xvhdl --2008 --relax`로 컴파일 성공 (P-C01-V004-01 / V005-01 컴파일 안전성 닫힘).
- ASYNC mode simulation log에 `Note: ASYNC raw_cdc FIFO generate active` 정확히 **1회** emit (`grep -c` 결과 = 1, P-C01-V004-02 닫힘).

### 3.2 ASYNC clean elab 실행 명령 (v005 그대로 유지)

```bash
# Git Bash (MSYS) 실행
SIM_DIR="C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim"
LOG_DIR="C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tmp/c01_verify"
export PATH="/c/AMDDesignTools/2025.2.1/Vivado/bin:$PATH"
cd "${SIM_DIR}"

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

xelab -f "${SIM_DIR}/__elab_ASYNC_clean.f" \
      -log "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"

grep -nE "Compiling module xpm.xpm_fifo_async" \
     "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"
```

PowerShell 환경에서는 `bash -lc 'cd /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim && ...'` wrapper 권장.

### 3.3 ASYNC FIFO generate evidence 인정 정책 (v005 그대로)

| 인정 조건 | 검증 명령 |
|---|---|
| **Primary-A** (xelab compile evidence) | `grep -nE "Compiling module xpm.xpm_fifo_async" "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"` -> 1 match 이상 |
| **Primary-B** (RTL branch marker, section 3.1로 구현) | `grep -cE "ASYNC raw_cdc FIFO generate active" "${LOG_DIR}/sim_tb_tdc_gpx_config_ctrl_ASYNC.log"` -> 정확히 1 (chip 0 1회) |

mode echo + handshake는 closure fallback에서 제외 (보조 정보만). marker note는 **expected note**로 분류 (section 6.1).

---

## 4. v005 closure 문구 단일안 (v005와 동일)

> C01 보완 검증은 기능 PASS, 회귀 운영 코드 보강, negative test transcript/exit code 보존, ASYNC FIFO generate evidence 보강까지 완료되었습니다. v004 / v006 / Plan v001/v002/v003/v004/v005/v006 / 주요 lineage는 연결되어 있으며, C02 진입 가능합니다.

자동 다운그레이드 조건:
- #1 INTEGRATED EXIT CODE positive transcript에 보존되지 않음 → 다운그레이드.
- #3 negative transcript에 `INTEGRATED EXIT CODE = 1` 보존되지 않음 → 다운그레이드.
- #5 Primary-A **AND** Primary-B 둘 다 FAIL → 다운그레이드.
- 다운그레이드 시 closure 문구를 “미달 항목 명시 + 후속 plan 권장”으로 자동 변경.

---

## 5. 진행 순서 (Hard Order, v006)

1. **본 Plan v006 사용자 승인 대기**.
2. 코드 변경 — **#1 RC_FINAL echo + #2 `--negative` 분기 + #2-hook env hook + #2-marker concurrent assertion (begin 명시)** 묶음. RTL marker 변경은 `synthesis translate_off`로 합성 영향 0 확인.
3. 회귀 재컴파일 (`xvhdl --incr --relax -prj __c01_all_vhdl.prj`) — RTL marker 추가 반영. **컴파일 ERROR 0건** 확인 (P-C01-V004-01 / V005-01 정정 1차 검증).
4. **#3 Negative**: `cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl && C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative` (Git Bash). PowerShell은 `bash -lc 'cd /c/... && ...'` wrapper. transcript에 `RESULTS: ... 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`, `INTEGRATED EXIT CODE = 1` 확인.
5. **#4 Positive**: `cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl && bash HDL/scripts/run_c01_regression.sh`. PowerShell wrapper도 동일 cwd 포함. transcript에 ALL PASS + `INTEGRATED EXIT CODE = 0` + ASYNC sim log marker 1회 (`grep -c` = 1) 확인.
6. **#5 ASYNC clean elab + Primary-A 검증** — section 3.2 실행. Primary-A `xpm_fifo_async` compile line 또는 Primary-B branch marker 둘 중 1+ match 확인.
7. **#6 v005 결과 보고서** 작성 (closure 문구 section 4 단일안, **section 6.1 expected note 표에 marker note 추가**, 다운그레이드 조건 점검).
8. **#7 lineage** 추가 (v004 / Plan v002 / v003 / v004 / v005 / v006).

---

## 6. 위험 / 완화

| 위험 | 완화 |
|---|---|
| RTL marker 컴파일 오류 | concurrent assertion + nested `if i = 0 generate begin ... end generate` 형태 채택 → VHDL-2008 합법 + repo 스타일 일관. xvhdl 컴파일 직후 결과로 검증. |
| marker emit 횟수 부정확 | `if i = 0 generate`로 chip 0 1회로 한정. positive run 후 `grep -c` 결과 = 1 확인. |
| 실행 cwd 혼재 | 모든 invocation을 IP root (`C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl`) cwd로 통일. PowerShell wrapper는 `bash -lc 'cd ... && ...'` 형태로 login shell 차이 회피 (P-C01-V005-02). |
| concurrent assertion이 PASS 회귀에서 fail로 보일 가능성 | `assert false report ... severity note`는 항상 false → report emit, severity note는 simulation 정상 진행. PASS 판정 영향 0. v005 결과 보고서 expected note 표에 명시 (P-C01-V005-03)로 추적성 강화. |
| `synthesis translate_off` pragma 누락으로 합성 영향 발생 | pragma는 `-- synthesis translate_off` / `-- synthesis translate_on` 형식으로 marker 위/아래 모두 명시. Vivado/Quartus 표준 지원. |

---

## 6.1 v005 결과 보고서 expected note 표 (P-C01-V005-03 신규)

v005 결과 보고서의 “허용 warning / expected note 목록”에 다음 항목을 명시한다.

| Note 텍스트 | 발생 위치 | 의미 / 근거 |
|---|---|---|
| `Note: ASYNC raw_cdc FIFO generate active` | `tdc_gpx_config_ctrl.vhd` `gen_async_marker` block (`gen_raw_async` 안의 `if i = 0 generate begin`) | ASYNC raw_cdc FIFO generate가 실제 elaboration된 1차 증거 (Primary-B). simulation log에서 chip 0 1회 emit. |
| `Warning: bus_phy: bus timing clamped (div=` | `tdc_gpx_bus_phy.vhd:430-434` | C01 contract test [1]에서 의도적 illegal 입력 — clamp 동작 검증 (기존). |
| `Warning: bus_phy: write request ignored (oen_permanent='1')` | `tdc_gpx_bus_phy.vhd:443-445` | INV-7 보호 (기존). |
| `WARNING: [VRFC 10-3532] module 'glbl' does not have a parameter named 'g_dut_stream_clk_mode' to override` | xelab static elaboration | xelab `-generic_top`이 모든 top 단위에 적용 (기존). |

운영 가이드:
- `severity failure`만 fail.
- 위 note/warning은 grep allow-list로 처리.
- 새 expected note는 본 표 확장.

---

## 7. 검증 방법 (v006 단일안)

### Positive (#4)
- 명령 (Git Bash):
  ```bash
  cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl
  bash HDL/scripts/run_c01_regression.sh
  echo "EXIT CODE = $?"
  ```
- PowerShell wrapper:
  ```powershell
  bash -lc 'cd /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl && HDL/scripts/run_c01_regression.sh'
  ```
- 종료코드: `0`
- transcript: `tmp/c01_verify/run_c01_regression.log`
  - Stage 1/2/3 PASS
  - `# C01 regression: ALL PASS`
  - **`INTEGRATED EXIT CODE = 0`**
- ASYNC simulation log: `tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log`에 expected note 1회:
  ```bash
  grep -cE "ASYNC raw_cdc FIFO generate active" tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log
  # -> 1
  ```

### Negative (#3)
- 명령 (Git Bash):
  ```bash
  cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl
  C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative
  echo "EXIT CODE = $?"
  unset C01_FORCE_NEGATIVE_STAGE1
  ```
- PowerShell wrapper (P-C01-V005-02 적용):
  ```powershell
  bash -lc 'cd /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl && C01_FORCE_NEGATIVE_STAGE1=1 HDL/scripts/run_c01_regression.sh --negative'
  ```
- 종료코드: `1`
- transcript 파일명: `tmp/c01_verify/run_c01_regression_negative_<YYYYMMDDTHHMMSSZ>.log`
- transcript 내용:
  - Stage 1 summary: `RESULTS: 4 ok, 1 failed`, `Failed: ... __NEGATIVE_TEST_FORCED_FAIL`
  - 통합: `# C01 regression: FAIL (stage1=1, stage2=0, stage3=0)`
  - **`INTEGRATED EXIT CODE = 1`**
- 환경변수 정리, 파일 변경 없음.

### ASYNC clean evidence (#5)
- 명령: section 3.2 wrapper.
- Primary-A: `grep -nE "Compiling module xpm.xpm_fifo_async" "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"` -> 1+ match.
- Primary-B: `grep -cE "ASYNC raw_cdc FIFO generate active" "${LOG_DIR}/sim_tb_tdc_gpx_config_ctrl_ASYNC.log"` -> 정확히 1.

### v005 closure 문구 (#6)
- section 4 단일안 그대로.
- 다운그레이드 조건 발생 시 명시적 미달 항목 표기.
- expected note 표는 section 6.1 그대로 v005 결과 보고서에 복사.

### RTL marker 컴파일 검증
- `xvhdl --2008 --relax -prj __c01_all_vhdl.prj` 결과 ERROR 0건.
- 검증 명령:
  ```bash
  cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim
  xvhdl --incr --relax -prj __c01_all_vhdl.prj -log __c01_all_xvhdl.log
  grep -cE "^ERROR:" __c01_all_xvhdl.log
  # -> 0
  ```

---

## 8. 산출물 (예상)

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v006.md` | 본 문서 |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v005.md` | 끝에 v005 -> v006 forward-trace 추가 |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` | 결과 보고서 (생성 예정, expected note 표 포함) |
| `HDL/scripts/run_c01_regression.sh` | RC_FINAL echo + Windows-safe TS + `--negative` 분기 |
| `HDL/tmp/c01_verify/run_regression.sh` | `C01_FORCE_NEGATIVE_STAGE1` env hook (영구) |
| `HDL/tdc_gpx_config_ctrl.vhd` | `gen_raw_async` 안에 nested generate + `begin` + concurrent assertion marker (`-- synthesis translate_off/on`) |
| `HDL/tmp/c01_verify/run_c01_regression.log` | positive transcript |
| `HDL/tmp/c01_verify/run_c01_regression_negative_<TS>.log` | negative transcript (Windows-safe) |
| `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` | ASYNC clean elab evidence (Primary-A) |
| `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log` (기존, 갱신) | ASYNC branch marker 1회 (Primary-B, expected note) |

---

## 9. 사용자 피드백 기록

이번 v006 plan 단계에서 사용자가 직접 입력한 피드백:

1. v005 plan에 대한 review (`Code_Verify_Plan_Review_20260429_v005.md`) — Plan v005를 “승인 가능”으로 명시 + minor 권고 P-C01-V005-01..03.
2. Codex 채팅 답변 (옵션 A 빠른 진행 vs 옵션 B 엄격 protocol 준수 선택지 제시) → 사용자 “옵션 B 진행” 지시.
3. 옵션 B는 v006 protocol section 5 “Plan이 수정되면 Plan 문서 자체도 새 버전으로 생성” 규칙을 strict하게 따르는 운영. 본 v006 plan이 이 규칙의 명시적 적용.

---

## 10. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` |
| 사용자 review (Code Verify) | `... Code_Verify_Review_20260429_v004.md` |
| 사용자 review (Closure) | `... Closure_Review_20260429_v001.md` |
| 사용자 review (Plan v002) | `... Code_Verify_Plan_Review_20260429_v002.md` |
| 사용자 review (Plan v003) | `... Code_Verify_Plan_Review_20260429_v003.md` |
| 사용자 review (Plan v004) | `... Code_Verify_Plan_Review_20260429_v004.md` |
| 사용자 review (Plan v005) | `... Code_Verify_Plan_Review_20260429_v005.md` |
| 직전 plan 들 | `... Code_Verify_Plan_20260429_v002.md ~ v005.md` (본 v006에서 정정 대상) |
| 직전 결과 보고서 | `... Code_Verify_20260429_v004.md` |
| 본 계획 적용 대상 스크립트 | `HDL/scripts/run_c01_regression.sh`, `HDL/tmp/c01_verify/run_regression.sh` |
| 본 계획 적용 대상 RTL | `HDL/tdc_gpx_config_ctrl.vhd` (gen_raw_async block, line 1834+, repo style begin 일관성) |
| RTL 환경 상수 | `HDL/tdc_gpx_pkg.vhd` (`c_N_CHIPS = 4`) |
| 운영 프로토콜 (Review 처리 사이클 규칙) | `cluster_analysis_operating_protocol_20260429_v006.md` section 5 |

---

## 11. Plan v006 -> Code_Verify v005 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | 본 Plan v006 사용자 승인 (“오케이 진행해줘”) → Hard Order #2~#7 실행 |
| 반영된 결과 보고서 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` |
| 결과 보고서 반영 위치 | section 2 (R-C01-V004-01..03 / R-C01-CL-01..04 닫힘 — Plan v006의 단일안 그대로 적용), section 3 (#1 #2 #2-hook #2-marker 실행 결과), section 3.5 (#5 ASYNC clean elab evidence Primary-A), section 4 (통합 회귀 결과 + Negative test), section 6 (expected note 표 — Plan v006 section 6.1 그대로 복사), section 7 (finding 갱신 — Plan v006 hard order 모든 단계 닫힘), section 9 (Plan v006 section 4 closure 문구 단일안 채택, 다운그레이드 조건 모두 통과), section 13 (변경 이력) |
| 판단 변화 | Plan v006의 모든 #1~#7 항목이 v005 결과 보고서에 반영 완료. RTL marker (`gen_async_marker`) compile clean (P-C01-V005-01 검증), emit 1회 (P-C01-V004-02 검증), repo 스타일 일관 (P-C01-V005-01 검증). PowerShell wrapper에 cd 포함 (P-C01-V005-02), expected note 표 등록 (P-C01-V005-03). 다운그레이드 조건 모두 통과로 closure 문구 단일안 그대로 채택. v006 protocol section 5 정상 사이클 첫 적용 완료. |
| 추적 근거 | `tdc_gpx_config_ctrl.vhd:1916-1929`, `tmp/c01_verify/run_regression.sh:111-118`, `scripts/run_c01_regression.sh:25-40,90-104`, `tmp/c01_verify/run_c01_regression.log` (positive `INTEGRATED EXIT CODE = 0`), `tmp/c01_verify/run_c01_regression_negative_20260429T100450Z.log` (negative `INTEGRATED EXIT CODE = 1`, `RESULTS: 4 ok, 1 failed`), `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log:69` (Primary-A `Compiling module xpm.xpm_fifo_async`), `tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29` (Primary-B `Note: ASYNC raw_cdc FIFO generate active`) |
