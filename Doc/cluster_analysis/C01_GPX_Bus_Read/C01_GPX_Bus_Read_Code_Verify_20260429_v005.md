# C01_GPX_Bus_Read 코드 검증 기록 v005

문서 버전: `v005`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 19:08:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v006.md` (사용자 승인된 Plan)을 정확히 실행한 결과를 기록한다. v004 결과 보고서의 evidence 추적성 결함(R-C01-V004-01..03 / R-C01-CL-01..04)을 R-C01-V003-01..03와 함께 운영 코드 보강(전 사이클) + evidence 보강(본 사이클)으로 닫는다. 본 v005는 RTL 변경을 동반하나 마커는 `synthesis translate_off` pragma로 합성 영향 0이며, 기능 RTL은 변경되지 않는다. 절대 기준은 `Doc/TDC-GPX-Datasheet.pdf`이며 RTL clock은 `i_tdc_clk = 200 MHz`.

본 v005는 운영 프로토콜 v006 section 5 “Review 처리 사이클 규칙”의 정상 사이클 첫 결과 산출물이다. 사이클 흐름: Review v004 + Closure_Review v001 + Plan_Review v002~v005 → Plan v006 (사용자 승인) → 본 v005 결과 보고서.

---

## 1. 검증 환경 (v004 동일 + Plan v006 신규 항목)

| 항목 | 값 |
|---|---|
| 시뮬레이터 | Vivado 2025.2.1 xsim |
| Vivado 경로 | `C:/AMDDesignTools/2025.2.1/Vivado` |
| 표준 cwd | **IP root** (`C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl`, P-C01-V004-03 / V005-02) |
| 통합 entrypoint | `HDL/scripts/run_c01_regression.sh` (Plan v006 #1 #2 적용) |
| Stage 1 sub-script | `HDL/tmp/c01_verify/run_regression.sh` (#2-hook 영구 삽입) |
| Stage 2 sub-script | `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` (변경 없음) |
| Stage 3 sub-script | `run_c01_regression.sh` 내 inline xelab/xsim |
| ASYNC clean elab | `${SIM_DIR}/__elab_ASYNC_clean.f` arg-file (Plan v006 section 3.2) |
| Negative test 환경변수 | `C01_FORCE_NEGATIVE_STAGE1=1` (영구 hook 활성, Plan v006 #2-hook) |
| Negative transcript 명명 | `run_c01_regression_negative_$(date -u '+%Y%m%dT%H%M%SZ').log` (Windows-safe, P-C01-V003-01) |
| ASYNC RTL marker | `tdc_gpx_config_ctrl.vhd:1916-1929` (Plan v006 #2-marker, repo style begin) |

운용 셸: Git Bash (MSYS) / PowerShell wrapper `bash -lc 'cd ... && ...'`.

---

## 2. v004 + Closure_Review 잔여 finding 닫힘 결과

### 2.1 R-C01-V004-01 / R-C01-CL-02 (INTEGRATED EXIT CODE / negative artifact 보존)

| 항목 | v004 상태 | v005 결과 |
|---|---|---|
| INTEGRATED EXIT CODE 보존 | transcript에 미존재 | **닫힘**. positive run의 `run_c01_regression.log` 마지막 줄에 `INTEGRATED EXIT CODE = 0`. negative run의 `run_c01_regression_negative_<TS>.log` 마지막 줄에 `INTEGRATED EXIT CODE = 1`. |
| Negative test artifact 별도 파일 | 존재하지 않음 | **닫힘**. 본 v005 실행에서 `run_c01_regression_negative_20260429T100450Z.log` 신규 생성, 보존됨. |
| Stage 1 summary에 forced fail 표시 | 미존재 | **닫힘**. Negative transcript에 `RESULTS: 4 ok, 1 failed`, `Failed: __NEGATIVE_TEST_FORCED_FAIL`. |

근거:
- `HDL/tmp/c01_verify/run_c01_regression.log`: `INTEGRATED EXIT CODE = 0` (마지막 줄)
- `HDL/tmp/c01_verify/run_c01_regression_negative_20260429T100450Z.log`: `INTEGRATED EXIT CODE = 1`, `RESULTS: 4 ok, 1 failed`, `Failed: __NEGATIVE_TEST_FORCED_FAIL`

### 2.2 R-C01-V004-02 / R-C01-CL-03 (ASYNC FIFO generate evidence)

| 항목 | v004 상태 | v005 결과 |
|---|---|---|
| Primary-A (xelab compile evidence) | 최신 elab log 10줄, `xpm_fifo_async` 0 match | **닫힘**. Plan v006 section 3.2 wrapper로 clean elab 1회 실행 → `elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log:69`에 `Compiling module xpm.xpm_fifo_async(FIFO_MEMORY_TYPE=...)` 정확히 1 match. |
| Primary-B (RTL branch marker) | 존재하지 않음 | **닫힘**. `tdc_gpx_config_ctrl.vhd`의 `gen_async_marker` (chip 0 한정) → `sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29`에 `Note: ASYNC raw_cdc FIFO generate active` 정확히 1 match. SYNC mode log는 0 match (gen_raw_async 비활성). |
| Closure 인정 정책 (Plan v006 section 3.3) | - | **Primary-A AND Primary-B 둘 다 PASS** → closure 인정 강화. |

근거:
- `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log:69` — `Compiling module xpm.xpm_fifo_async(FIFO_MEMORY_TYPE=...)`
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29` — `Note: ASYNC raw_cdc FIFO generate active`
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_SYNC.log` — marker 0 match (정상)

### 2.3 R-C01-V004-03 (v004 보고서 시각 정정)

| 항목 | 내용 |
|---|---|
| v004 표기 | `start: 2026-04-29 17:10:30 +0900` (추정 시각) |
| 실제 transcript | `start: 2026-04-29 17:10:19 +0900` (당시 보존된 transcript 1행) |
| v005 정정 | 본 보고서에서 v004의 timestamp가 추정값이었음을 명시. v005부터는 transcript 1행에서 직접 인용한다. 본 v005의 positive run 시작 시각: `2026-04-29 19:06:13 +0900` (`run_c01_regression.log` 1행 인용). |

이번 v005 보고서의 timestamps는 모두 transcript 1행 또는 `date` 명령 직접 출력에서 인용한 값이다.

### 2.4 R-C01-CL-01 (closure 문구 조정) — 최종 단일안 채택

Plan v006 section 4 단일안을 결과 보고서에 그대로 채택:

> C01 보완 검증은 기능 PASS, 회귀 운영 코드 보강, negative test transcript/exit code 보존, ASYNC FIFO generate evidence 보강까지 완료되었습니다. v004 / v006 / Plan v001/v002/v003/v004/v005/v006 / 주요 lineage는 연결되어 있으며, C02 진입 가능합니다.

자동 다운그레이드 조건 (Plan v006 section 4):
- #1 INTEGRATED EXIT CODE positive transcript 누락 — **PASS** (보존됨).
- #3 negative transcript에 EXIT CODE = 1 누락 — **PASS** (보존됨).
- #5 Primary-A AND Primary-B 둘 다 FAIL — **PASS** (둘 다 1 match로 통과).

→ 모든 조건 통과 → closure 문구 단일안 그대로 채택.

### 2.5 R-C01-CL-04 (v006 Protocol / Plan / lineage 구조)

조치 불필요로 인정됨. 본 v005는 v006 protocol section 5 “Review 처리 사이클 규칙”의 정상 사이클 첫 결과로, plan v006 사용자 승인 후 진행되어 규칙을 정확히 따랐다.

---

## 3. Plan v006 실행 결과 (#1 ~ #5)

### 3.1 #1 INTEGRATED EXIT CODE echo (Plan v006 #1)

`HDL/scripts/run_c01_regression.sh:90-104` 변경:
- 최종 판정 결과를 `RC_FINAL` 변수로 캡쳐
- `echo "INTEGRATED EXIT CODE = ${RC_FINAL}"`을 transcript에 출력 (tee redirect 안에서)
- `exit ${RC_FINAL}` 종료

검증:
- positive run: `INTEGRATED EXIT CODE = 0` 보존 (`run_c01_regression.log` 마지막 줄)
- negative run: `INTEGRATED EXIT CODE = 1` 보존 (`run_c01_regression_negative_20260429T100450Z.log` 마지막 줄)

### 3.2 #2 `--negative` transcript 분기 + Windows-safe TS (Plan v006 #2)

`HDL/scripts/run_c01_regression.sh:25-40` 변경:
- 첫 인자가 `--negative`이면 `NEGATIVE_MODE=1`
- `TS=$(date -u '+%Y%m%dT%H%M%SZ')`로 ISO 8601 basic 형식 (콜론 없음, Windows-safe)
- transcript 파일명을 `run_c01_regression_negative_${TS}.log`로 분기
- positive (`NEGATIVE_MODE=0`)는 기존 `run_c01_regression.log` 그대로
- mode 라인 (`positive`/`negative`)을 transcript 시작부에 echo

검증:
- positive transcript 첫 줄: `[run_c01_regression] mode: positive`
- negative transcript 첫 줄: `[run_c01_regression] mode: negative`
- negative TS 형식: `20260429T100450Z` (Z 끝, 콜론 0개)

### 3.3 #2-hook env hook (Plan v006 #2-hook)

`HDL/tmp/c01_verify/run_regression.sh:111-118` 영구 삽입:
```bash
if [ "${C01_FORCE_NEGATIVE_STAGE1:-0}" = "1" ]; then
    FAIL_COUNT=$((FAIL_COUNT+1))
    FAIL_LIST="${FAIL_LIST} __NEGATIVE_TEST_FORCED_FAIL"
fi
```
위치: summary echo 직전 (`echo "RESULTS: ..."` 출력 전).

검증:
- positive run (`C01_FORCE_NEGATIVE_STAGE1` unset, default 0): hook 비활성, 기존 회귀 영향 0, ALL PASS.
- negative run (`C01_FORCE_NEGATIVE_STAGE1=1`): hook 활성, summary에 `RESULTS: 4 ok, 1 failed`, `Failed: __NEGATIVE_TEST_FORCED_FAIL` 정확히 출력.

### 3.4 #2-marker ASYNC RTL branch marker (Plan v006 #2-marker, P-C01-V005-01)

`HDL/tdc_gpx_config_ctrl.vhd:1916-1929` 추가:
```vhdl
            -- =================================================================
            -- ASYNC FIFO generate marker (sim-only, Plan v006 / P-C01-V005-01)
            -- chip 0 에서만 1회 emit. concurrent assertion 형태이므로 generate
            -- body에 그대로 합법. severity note 라 simulation은 중지하지 않음.
            -- repo 스타일 일관성을 위해 nested generate에 begin 명시.
            -- =================================================================
            -- synthesis translate_off
            gen_async_marker : if i = 0 generate
            begin
                assert false
                    report "ASYNC raw_cdc FIFO generate active"
                    severity note;
            end generate gen_async_marker;
            -- synthesis translate_on
```

검증 (Plan v006 진행 순서 #3):
- `xvhdl --incr --relax -prj __c01_all_vhdl.prj`: ERROR 0건 (P-C01-V005-01 닫힘 — repo 스타일 + concurrent assertion + nested generate begin 형태가 VHDL-2008에서 합법).
- ASYNC mode simulation log marker emit: 정확히 1회 (`grep -c = 1`).
- SYNC mode simulation log marker emit: 0회 (gen_raw_async 비활성).
- `synthesis translate_off` pragma로 합성 영향 0 (Vivado 표준 지원).

### 3.5 #5 ASYNC clean elab (Plan v006 section 3.2)

명령 (Git Bash 실행, 절대 경로 wrapper):
```bash
SIM_DIR="C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim"
LOG_DIR="C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tmp/c01_verify"
cd "${SIM_DIR}"
xelab -f "${SIM_DIR}/__elab_ASYNC_clean.f" \
      -log "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"
```

결과:
- `Built simulation snapshot tb_tdc_gpx_config_ctrl_ASYNC_clean_snap`
- ELAB EXIT CODE = 0
- `elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log:69` — `Compiling module xpm.xpm_fifo_async(FIFO_MEMORY_TYPE=...)` (Primary-A PASS)

---

## 4. 통합 회귀 결과 요약 (v005)

### Stage 1 — 4 TB direct (positive)

| TB | 결과 | marker |
|---|---|---|
| `tb_tdc_gpx_bus_phy_c01_contract` | PASS | `tb_tdc_gpx_bus_phy_c01_contract PASS` |
| `tb_tdc_gpx_bus_phy` | PASS | `*** ALL TESTS PASSED *** (total_rsp=85)` |
| `tb_tdc_gpx_chip_ctrl` | PASS | `*** ALL TESTS PASSED *** (total_raw_words=224)` |
| `tb_tdc_gpx_config_ctrl` | PASS | `Note: TB: PASS -- init complete, active_chip_mask = 15` |

Stage 1 sub-script exit code: 0. Summary `RESULTS: 4 ok, 0 failed`.

### Stage 2 — config_ctrl 2-mode

| Mode | 결과 |
|---|---|
| `g_DUT_STREAM_CLK_MODE=SYNC` | PASS |
| `g_DUT_STREAM_CLK_MODE=ASYNC` | PASS (+ ASYNC raw_cdc FIFO marker 1회 emit) |

Stage 2 sub-script exit code: 0.

### Stage 3 — CSR clamp

`*** ALL TESTS PASSED *** (cases=12)`. Stage 3 exit code: 0.

### 통합 entrypoint

```text
[run_c01_regression] start: 2026-04-29 19:06:13 +0900
[run_c01_regression] mode: positive
[run_c01_regression] transcript: /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tmp/c01_verify/run_c01_regression.log
...
[run_c01_regression] end: 2026-04-29 19:06:57 +0900
############################################################
# C01 regression: ALL PASS                                 
############################################################
INTEGRATED EXIT CODE = 0
```

Bash 종료코드: **0**.

### Negative test (#3)

명령:
```bash
cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl
C01_FORCE_NEGATIVE_STAGE1=1 bash HDL/scripts/run_c01_regression.sh --negative
```

```text
[run_c01_regression] start: 2026-04-29 19:04:50 +0900
[run_c01_regression] mode: negative
[run_c01_regression] transcript: /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tmp/c01_verify/run_c01_regression_negative_20260429T100450Z.log
...
RESULTS: 4 ok, 1 failed
Failed: __NEGATIVE_TEST_FORCED_FAIL
...
[run_c01_regression] end: 2026-04-29 19:05:57 +0900
############################################################
# C01 regression: FAIL (stage1=1, stage2=0, stage3=0)
############################################################
INTEGRATED EXIT CODE = 1
```

Bash 종료코드: **1**. `unset C01_FORCE_NEGATIVE_STAGE1` 후 다음 positive run에 영향 0 검증됨.

---

## 5. 변경 파일 점검 (v004 -> v005)

| 파일 | 변경 의도 | 추적 위치 | 합성 영향 |
|---|---|---|---|
| `HDL/scripts/run_c01_regression.sh` | (i) `--negative` 인자 처리 + Windows-safe TS, (ii) mode echo, (iii) `RC_FINAL` 변수 + `INTEGRATED EXIT CODE` echo + `exit ${RC_FINAL}` | line 25-40, line 90-104 | N/A (script) |
| `HDL/tmp/c01_verify/run_regression.sh` | `C01_FORCE_NEGATIVE_STAGE1` env hook 영구 삽입 | line 111-118 (summary echo 직전) | N/A (script) |
| `HDL/tdc_gpx_config_ctrl.vhd` | `gen_raw_async` 안에 `gen_async_marker : if i = 0 generate begin ... end generate` (concurrent assertion + repo style begin), `synthesis translate_off/on` pragma | line 1916-1929 | **0** (translate_off) |
| `Doc/cluster_analysis/...Plan_20260429_v002~v006.md` | 사용자 review 사이클 처리 (4건 review + 5개 plan 버전) | 각 plan / forward-trace | N/A (문서) |
| `Doc/cluster_analysis/...Code_Verify_20260429_v005.md` | 본 결과 보고서 신규 | (신규) | N/A (문서) |

RTL 변경은 marker 1줄 추가 외 없음. 기능 RTL 동작 변화 없음.

---

## 6. 허용 warning / Expected note 표 (v005 갱신, P-C01-V005-03)

CI/회귀 스크립트는 다음을 fail로 분류하지 않는다.

| Note / Warning 텍스트 | 발생 위치 | 의미 / 근거 |
|---|---|---|
| **(v005 신규)** `Note: ASYNC raw_cdc FIFO generate active` | `tdc_gpx_config_ctrl.vhd:1922-1925` `gen_async_marker` (chip 0) | ASYNC raw_cdc FIFO generate가 실제 elaboration된 1차 증거 (Primary-B). `concurrent assertion + severity note` 형태로 simulation 정상 진행. |
| `Warning: bus_phy: bus timing clamped (div=` | `tdc_gpx_bus_phy.vhd:430-434` | C01 contract test [1]에서 의도적 illegal 입력 — clamp 동작 검증 (기존). |
| `Warning: bus_phy: write request ignored (oen_permanent='1')` | `tdc_gpx_bus_phy.vhd:443-445` | INV-7 보호: oen_permanent='1' 동안 WRITE 거부 (기존). |
| `WARNING: [VRFC 10-3532] module 'glbl' does not have a parameter named 'g_dut_stream_clk_mode' to override` | xelab static elaboration | xelab `-generic_top`이 모든 top 단위 (TB + glbl)에 적용 (기존). |

운영 가이드:
- `severity failure`만 fail.
- 위 4건은 grep allow-list로 처리.
- **(v005 신규)** no-marker는 PASS로 묵시 분류되지 않음. self-check 없는 TB는 명시적 allow-list 등록 필요 (현재 0건).

---

## 7. v004 + v003 review 잔여 finding 갱신 표

| Finding | v004 상태 | v005 상태 | 닫힘 근거 |
|---|---|---|---|
| F-C01-V01..V07 (구 finding) | 유지/닫힘 | **변동 없음** | RTL 기능 finding은 본 사이클 범위 밖 |
| R-C01-V002-01..04 | 닫힘 | **유지** | v002 사이클에서 닫힘 |
| R-C01-V003-01 (실패 전파) | 닫힘 | **재닫힘 강화**. v005 Negative test로 직접 검증 (`stage1=1` exit 1) | `run_c01_regression_negative_20260429T100450Z.log` |
| R-C01-V003-02 (no-marker) | 닫힘 | **유지** | regex `TB:.*PASS` + no-marker FAIL |
| R-C01-V003-03 (transcript) | 닫힘 | **재닫힘 강화**. v005에서 transcript에 `INTEGRATED EXIT CODE` 라인까지 보존 | `run_c01_regression.log` 마지막 줄 |
| **R-C01-V004-01 (EXIT CODE 추적성)** | 열림 | **닫힘** | positive transcript line `INTEGRATED EXIT CODE = 0`, negative `... = 1` |
| **R-C01-V004-02 (ASYNC clean evidence)** | 열림 | **닫힘** | Primary-A 1 match (`elab_..._ASYNC_clean.log:69`) + Primary-B 1 match (`sim_..._ASYNC.log:29`) |
| **R-C01-V004-03 (시각 정정)** | 열림 | **닫힘** | v005 timestamps 모두 transcript 1행 직접 인용 |
| **R-C01-CL-01 (closure 문구)** | 열림 | **닫힘** | Plan v006 section 4 단일안 채택, 다운그레이드 조건 모두 통과 |
| **R-C01-CL-02 (= V004-01)** | 열림 | **닫힘** | (V004-01과 동일) |
| **R-C01-CL-03 (= V004-02)** | 열림 | **닫힘** | (V004-02와 동일) |
| **R-C01-CL-04 (v006 protocol 인정)** | 인정 | **유지** | v006 protocol section 5 정상 사이클 첫 적용 |
| P-C01-V002-01..04 | 닫힘 (Plan v003) | **유지** | Plan v003 |
| P-C01-V003-01..04 | 닫힘 (Plan v004) | **유지** | Plan v004 |
| P-C01-V004-01..03 | 닫힘 (Plan v005) | **유지** | Plan v005 |
| **P-C01-V005-01 (begin 명시)** | 권고 (Plan v006) | **닫힘** | `tdc_gpx_config_ctrl.vhd:1923` — `gen_async_marker : if i = 0 generate begin` |
| **P-C01-V005-02 (PowerShell wrapper cd 포함)** | 권고 (Plan v006) | **닫힘** | Plan v006 section 3 / section 7에 `bash -lc 'cd ... && ...'` 형태 명시 |
| **P-C01-V005-03 (expected note 분류)** | 권고 (Plan v006) | **닫힘** | 본 v005 section 6에 expected note 표로 등록 |

신규 finding: 없음.

---

## 8. Latency / Throughput / Pipeline / II 누적표 (v001 + v002 + v003 + v004 + v005)

본 v005는 RTL 기능 변경이 없어 정량 항목은 v004와 동일. 회귀 운영 evidence package + ASYNC FIFO generate evidence만 보강.

| 항목 | 측정/근거 | 측정값 | 결과 |
|---|---|---|---|
| `RDN low pulse width` (`div=1,ticks=4 -> clamp 5`) | contract TB [1] | 15 ns | PASS |
| Burst READ II (`div=1,ticks=5`) | contract TB [2] | 25 ns | PASS |
| `tPW-RL` 실측 (default `div=2,ticks=5`) | bus_phy TB [13] | 15000 ps | PASS |
| `tS-AD` 실측 | bus_phy TB [13] | 5000 ps | PASS |
| `o_rsp_pending` set-to-clear latency | contract TB [4] | <= 1 clock | PASS |
| EF sync 2-FF latency | bus_phy TB [8] | 2 clock | PASS |
| AluTrigger 폭 | chip_ctrl TB [8] | 15000 ps | PASS |
| chip_ctrl drain cap | chip_ctrl TB [4][9b] | 17, 65 (cap=16+flush) | PASS |
| CSR clamp | csr_chip_clamp 11 case | 모두 일치 | PASS |
| CSR raw readback | csr_chip_clamp [c11] | x"000001C8" | PASS |
| config_ctrl SYNC raw stream | SYNC mode | direct passthrough | PASS |
| **(v005 강화)** config_ctrl ASYNC raw_cdc FIFO | ASYNC mode + Primary-A + Primary-B | xpm_fifo_async compile 1 + branch marker 1 | **PASS** |
| CDC handshake settling window | csr_chip_clamp `c_CDC_WAIT` | 200 ns = 40 cycles @ 200 MHz | PASS |
| **(v005 신규)** 통합 entrypoint EXIT CODE 추적성 | positive `= 0`, negative `= 1` | transcript line 보존 | PASS |
| **(v005 신규)** Negative test artifact 보존 | `run_c01_regression_negative_<TS>.log` | Windows-safe filename | PASS |

추가 정량 검증이 필요한 항목 (F-C01-V03 PH_RESP_DRAIN II, F-C01-V06 잔여, F-C01-V02 burst II div>=2)은 C02로 인계.

---

## 9. v005 Closure 선언

Plan v006 section 4 단일안 채택, 다운그레이드 조건 모두 통과:

> C01 보완 검증은 기능 PASS, 회귀 운영 코드 보강, negative test transcript/exit code 보존, ASYNC FIFO generate evidence 보강까지 완료되었습니다. v004 / v006 / Plan v001/v002/v003/v004/v005/v006 / 주요 lineage는 연결되어 있으며, C02 진입 가능합니다.

근거 매핑:
- “기능 PASS”: section 4 통합 회귀 결과 (Stage 1 4/4, Stage 2 2/2, Stage 3 12/12).
- “회귀 운영 코드 보강”: section 3.1 (#1), 3.2 (#2), 3.3 (#2-hook), 3.4 (#2-marker compile clean).
- “negative test transcript/exit code 보존”: section 4 Negative test + section 2.1.
- “ASYNC FIFO generate evidence 보강”: section 3.5 + section 2.2 (Primary-A AND Primary-B).
- “lineage 연결”: section 11 추적 근거 + 본 v005에 추가될 forward-trace.

---

## 10. 사용자 피드백 기록

이번 v005 결과 사이클에서 사용자가 직접 입력한 피드백:

1. v004 결과에 대한 review 두 건 — `Code_Verify_Review_20260429_v004.md`, `Closure_Review_20260429_v001.md`.
2. Plan v002~v005에 걸친 리뷰 — `Code_Verify_Plan_Review_20260429_v002.md ~ v005.md`.
3. Plan v005 사용자 review에서 “**옵션 B (strict protocol 준수) 진행**” 지시 → minor 권고 P-C01-V005-01..03을 채팅이 아닌 Plan v006 본문에 직접 반영.
4. Plan v006 “**오케이 진행해줘**” 명시적 승인 → 본 v005 결과 보고서 작성 사이클 트리거.

운영 시사:
- v006 protocol section 5 “Review 처리 사이클 규칙”의 **첫 정상 사이클** 완료.
- minor 권고도 strict하게 plan 본문에 반영하는 운영 패턴 (옵션 B)이 lineage 추적성을 강화함을 확인.
- 향후 모든 review 처리에 동일 패턴 적용 권장.

---

## 11. 산출물 위치 및 다음 단계

### 산출물

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` | 본 문서 (v005 결과 보고서) |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` | v004 → v005 forward-trace 추가 (예정) |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v002~v006.md` | Plan v002~v006 (각 v003 forward-trace 추가, v006는 본 v005 forward-trace 추가 예정) |
| `HDL/scripts/run_c01_regression.sh` | RC_FINAL echo + Windows-safe TS + `--negative` mode echo |
| `HDL/tmp/c01_verify/run_regression.sh` | `C01_FORCE_NEGATIVE_STAGE1` env hook (영구) |
| `HDL/tdc_gpx_config_ctrl.vhd:1916-1929` | `gen_async_marker` (chip 0 한정 concurrent assertion + repo style begin, `synthesis translate_off/on`) |
| `HDL/tmp/c01_verify/run_c01_regression.log` | positive transcript (overwrite) |
| `HDL/tmp/c01_verify/run_c01_regression_negative_20260429T100450Z.log` | negative transcript (Windows-safe TS, 보존) |
| `HDL/tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` | ASYNC clean elab evidence (Primary-A) |
| `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log` | ASYNC branch marker emit (Primary-B) |

### 다음 단계 후보

1. **C02 `Chip_Acquisition` 진입** — C01 closure 완료. 인계 항목: F-C01-V01/V02/V03/V04, F-C01-V06 잔여 (backpressure / IrFlag 시나리오).
2. 보드 회로도 확인 후 `g_OEN_MODE` default 결정 (F-C01-V04).
3. 상위 laser_ctrl 프로젝트에서 `tb_tdc_gpx_full_int`, `tb_tdc_gpx_top_int` 회귀 (F-C01-V01).

본 v005로 C01 보완 검증 사이클은 운영 정합성 + 기능 PASS + evidence package 모두 닫혔다. v006 protocol의 정상 사이클 운영도 첫 적용 검증 완료.

---

## 12. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` p.7 / p.8 / p.11~12 / p.27 |
| C01 분석 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v009.md` |
| C01 코드 보완 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Update_20260429_v001.md` |
| C01 검증 결과 | `... Code_Verify_20260429_v001.md ~ v004.md` (직전), 본 v005 (현 사이클) |
| C01 사용자 review | `... Code_Verify_Review_20260429_v001.md ~ v004.md`, `... Closure_Review_20260429_v001.md`, `... Code_Verify_Plan_Review_20260429_v002.md ~ v005.md` |
| C01 Plan 문서 | `... Code_Verify_Plan_20260429_v001.md ~ v006.md` |
| 운영 프로토콜 | `cluster_analysis_operating_protocol_20260429_v005.md` -> `... v006.md` (Review 처리 사이클 규칙) |
| 검증 transcript | `HDL/tmp/c01_verify/run_c01_regression.log` (positive), `... run_c01_regression_negative_20260429T100450Z.log` (negative), `... elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log` (Primary-A), `... sim_tb_tdc_gpx_config_ctrl_ASYNC.log` (Primary-B) |
| 사용자 라이브러리 | `px_utility_pkg.vhd` (AXI-Lite procedures) |

---

## 13. 변경 이력

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `Code_Verify_Review_v004` (R-C01-V004-01..03), `Closure_Review_v001` (R-C01-CL-01..04), `Code_Verify_Plan_Review_v002~v005` (P-C01-V00x-yy..) 사용자 review + Plan v002~v006 plan 사이클 |
| 직전 결과 보고서 | `Code_Verify_20260429_v004.md` |
| 직전 plan | `Code_Verify_Plan_20260429_v006.md` (사용자 승인) |
| 본 v005의 핵심 변경 | (i) 통합 entrypoint `INTEGRATED EXIT CODE` echo + transcript 보존, (ii) `--negative` 인자 transcript 분기 + Windows-safe TS, (iii) `C01_FORCE_NEGATIVE_STAGE1` env hook 영구 삽입, (iv) `gen_async_marker` (concurrent assertion + repo style begin) 영구 RTL marker, (v) ASYNC clean elab evidence (Primary-A), (vi) Negative test 실제 실행 + artifact 보존, (vii) closure 문구 단일안 채택 |
| 판단 변화 | R-C01-V004-01..03, R-C01-CL-01..03, R-C01-V003-01..03 모두 닫힘. P-C01-V005-01..03은 Plan v006에 본문 반영되어 본 v005 결과로 닫힘. 다운그레이드 조건 모두 통과 → closure 단일안 채택. F-C01-V01~V07 기능 finding은 본 사이클 범위 밖이므로 변동 없음. |
| 추적 근거 | section 11/12 표 + transcript line 직접 인용 |
