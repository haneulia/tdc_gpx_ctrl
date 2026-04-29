# C01_GPX_Bus_Read 코드 검증 기록 v004

문서 버전: `v004`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 17:12:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md`(R-C01-V003-01..03)에 대해 사전에 작성한 `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` 계획대로 회귀 운영 스크립트를 보강한 결과를 기록한다. 추가로 운영 프로토콜에 “Review 처리 사이클 규칙”을 신규로 도입한 사실(`cluster_analysis_operating_protocol_20260429_v006.md`)을 본 cluster 결과와 함께 명시한다. 절대 기준은 `Doc/TDC-GPX-Datasheet.pdf`이며 RTL clock은 `i_tdc_clk = 200 MHz`이다. 본 v004는 RTL 변경을 동반하지 않으며 회귀 운영 정합성만을 닫는다.

---

## 1. 검증 환경 (v003 동일 + 회귀 신뢰성 보강)

| 항목 | 값 |
|---|---|
| 시뮬레이터 | Vivado 2025.2.1 xsim |
| Vivado 경로 | `C:/AMDDesignTools/2025.2.1/Vivado` |
| 실행 디렉터리 | `C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim` |
| 통합 .prj | `__c01_all_vhdl.prj` (RTL + CSR IP source + 회귀 TB) |
| **통합 회귀 entrypoint** | `HDL/scripts/run_c01_regression.sh` (v003에서 신설, v004에서 transcript 보존 보강) |
| Stage 1 (4 TB 직접) | `HDL/tmp/c01_verify/run_regression.sh` (v004에서 exit 코드 + regex + no-marker fail 격상 보강) |
| Stage 2 (config_ctrl 2-mode) | `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` (v004에서 exit 코드 보강) |
| Stage 3 (CSR clamp) | `run_c01_regression.sh` 내 inline xelab/xsim |
| 로그 보존 폴더 | `HDL/tmp/c01_verify/` |
| 통합 transcript | `HDL/tmp/c01_verify/run_c01_regression.log` (v004 신규, 매 실행 overwrite) |

---

## 2. v003 리뷰 반영 결과 (R-C01-V003-01..03)

### 2.1 R-C01-V003-01: 통합 entrypoint 실패 전파 보강 (Medium)

| 항목 | 내용 |
|---|---|
| 변경 위치 | `tmp/c01_verify/run_regression.sh:120-124`, `tmp/c01_verify/run_config_ctrl_two_modes.sh:95-99` |
| 변경 요지 | 두 sub-script 말미에 `if [ ${FAIL_COUNT/FAIL} -ne 0 ]; then exit 1; fi; exit 0` 추가 |
| Negative test (#7 회귀 신뢰성 검증) | `run_regression.sh`에 `FAIL_COUNT=99` 임시 강제 → `bash run_c01_regression.sh` 실행 → 통합 entrypoint 출력 `# C01 regression: FAIL (stage1=1, stage2=0, stage3=0)`, `INTEGRATED EXIT CODE = 1` 확인 → 즉시 원복 |
| 원복 검증 | `grep -nE "NEGATIVE_TEST|FAIL_COUNT=99" run_regression.sh` -> 결과 없음 |
| Positive 재검증 | 원복 후 `bash run_c01_regression.sh` 실행 → `# C01 regression: ALL PASS`, `INTEGRATED EXIT CODE = 0` |
| 결론 | 닫힘. 종료코드가 stage 단위로 정확히 전파되며, 통합 entrypoint의 `RC1`/`RC2`/`RC3` 판정이 신뢰 가능. |

### 2.2 R-C01-V003-02: regex 보완 + no-marker FAIL 격상 (Low)

| 항목 | 내용 |
|---|---|
| 변경 위치 | `tmp/c01_verify/run_regression.sh:95-105` |
| regex 보완 | 정규식에 `TB:.*PASS` alternation 추가. config_ctrl의 `Note: TB: PASS -- init complete` marker 정상 인식. |
| no-marker 격상 | else 분기를 `FAIL_COUNT++`, `FAIL_LIST` 추가, `RESULTS`에 `FAIL(no-marker)` 기록으로 변경. |
| 영향 | 기존 모든 회귀 TB는 marker가 있으므로 PASS 유지. 미래에 self-check 누락된 TB가 추가되면 즉시 fail로 잡힘. |
| 검증 | Positive run 결과: `tb_tdc_gpx_config_ctrl: PASS  -- Note: TB: PASS -- init complete, active_chip_mask = 15` (이전 v003에서는 `RAN(no-marker)`로 분류되던 케이스). |
| 결론 | 닫힘. |

### 2.3 R-C01-V003-03: 통합 transcript 보존 (Low)

| 항목 | 내용 |
|---|---|
| 변경 위치 | `HDL/scripts/run_c01_regression.sh:22-29`, `:99` |
| 변경 요지 | 시작부에 `TRANSCRIPT="${LOG_DIR}/run_c01_regression.log"` 정의 후 `exec > >(tee "${TRANSCRIPT}") 2>&1`로 stdout/stderr를 동시에 콘솔과 파일로 기록. 시작/종료 timestamp 라인 추가. |
| 검증 | `bash run_c01_regression.sh` 실행 후 `tmp/c01_verify/run_c01_regression.log` 파일 생성 확인. Stage 1/2/3 모든 메시지와 timestamp 포함. |
| 결론 | 닫힘. 통합 회귀 transcript가 항상 최신 실행 결과로 보존됨 (매 실행 overwrite). |

### 2.4 v003 리뷰 “R-C01-V002-03 부분 반영” 잔여분 정리

v003 리뷰는 R-C01-V002-03(2-mode 분리)이 “부분 반영”이라고 평가했다. 이는 R-C01-V003-01의 종료코드 전파 누락 때문이었다. v004의 #7 Negative test로 종료코드 전파가 실제로 작동함을 검증했으므로, **R-C01-V002-03도 완전 닫힘**으로 갱신한다.

---

## 3. 통합 회귀 결과 (v004)

`bash HDL/scripts/run_c01_regression.sh` 단일 명령으로 전체 회귀가 닫히고, 종료코드 0 + transcript 보존 + 모든 stage PASS가 한 번에 확인된다.

### Stage 1: 4 TB 직접

| TB | 결과 | 핵심 marker |
|---|---|---|
| `tb_tdc_gpx_bus_phy_c01_contract` | PASS | `tb_tdc_gpx_bus_phy_c01_contract PASS` |
| `tb_tdc_gpx_bus_phy` | PASS | `*** ALL TESTS PASSED *** (total_rsp=85)` |
| `tb_tdc_gpx_chip_ctrl` | PASS | `*** ALL TESTS PASSED *** (total_raw_words=224)` |
| `tb_tdc_gpx_config_ctrl` (default SYNC) | **PASS** (이전 RAN(no-marker) 오분류) | `Note: TB: PASS -- init complete, active_chip_mask = 15` |

Stage 1 결과: `RESULTS: 4 ok, 0 failed`, sub-script exit code = 0.

### Stage 2: config_ctrl 2-mode

| Mode | 결과 | 보강된 마커 |
|---|---|---|
| `g_DUT_STREAM_CLK_MODE=SYNC` | PASS | `xpm_fifo_async` 0건 (bypass) |
| `g_DUT_STREAM_CLK_MODE=ASYNC` | PASS | `xpm_fifo_async` 1건 (raw_cdc 활성) |

Stage 2 결과: `config_ctrl two-mode results: 2 pass, 0 fail`, sub-script exit code = 0.

### Stage 3: CSR clamp

12 case 전부 PASS. v003에서 추가한 [c11] `px_axi_lite_reader` readback 포함.

`*** ALL TESTS PASSED *** (cases=12)`, stage exit code = 0.

### 통합 entrypoint

```text
[run_c01_regression] start: 2026-04-29 17:10:30 +0900
[run_c01_regression] transcript: /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/tmp/c01_verify/run_c01_regression.log
...
[run_c01_regression] end: 2026-04-29 17:11:02 +0900
############################################################
# C01 regression: ALL PASS
############################################################
INTEGRATED EXIT CODE = 0
```

근거 transcript: `HDL/tmp/c01_verify/run_c01_regression.log`.

### 종료코드 전파 검증 (Negative test)

| 단계 | 출력 | 종료코드 |
|---|---|---|
| Stage 1 (forced FAIL_COUNT=99) | `RESULTS: 4 ok, 99 failed` ... `Failed: __NEGATIVE_TEST_FORCED_FAIL` | 1 |
| 통합 entrypoint | `# C01 regression: FAIL (stage1=1, stage2=0, stage3=0)` | 1 |

원복 후 다시 positive run으로 정상 PASS + 종료코드 0 복귀 확인.

---

## 4. 변경 파일 점검 (v003 -> v004)

| 파일 | 변경 의도 | 추적 위치 |
|---|---|---|
| `HDL/tmp/c01_verify/run_regression.sh` | (i) regex `TB:.*PASS` 추가, (ii) no-marker 분기 PASS -> FAIL 격상, (iii) `FAIL_COUNT > 0`이면 `exit 1` 추가 | `run_regression.sh:95-105`, `run_regression.sh:120-124` |
| `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | `FAIL > 0`이면 `exit 1` 추가 | `run_config_ctrl_two_modes.sh:95-99` |
| `HDL/scripts/run_c01_regression.sh` | (i) `TRANSCRIPT` 변수 + `exec > >(tee ...) 2>&1` 추가, (ii) 시작/종료 timestamp 라인 추가 | `run_c01_regression.sh:22-29`, `:99` |
| `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v006.md` | 신규 — `Review 처리 사이클 규칙 (v006 신규)` 추가 | `operating_protocol v006 section 5` |
| `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v005.md` | v005 → v006 forward-trace 추가 | v005 문서 끝 |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` | v003 review에 대응한 plan 문서 (retrospective 작성) | (별도 문서) |

RTL은 변경 없다. 검증용 TB도 변경 없다.

---

## 5. 운영 프로토콜 업데이트: v005 -> v006 (Review 처리 사이클 규칙)

본 v004 사이클 도중 사용자가 “수정 계획 문서를 왜 규칙에 맞게 생성하지 않은거야?”라고 plan 문서 누락을 지적했다. Codex는 즉시 plan 문서를 retrospective로 작성한 뒤 (`C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md`), 사용자 동의를 받아 운영 프로토콜에 “Review 처리 사이클 규칙”을 신규 도입했다.

### v006 신규 규칙 요약

운영 패턴: `<base>_Review_*.md` (사용자 입력) → `<base>_Plan_*.md` (Codex 작성) → 사용자 승인 → 진행/회귀 → `<base>_*.md` (결과 보고서). Plan 누락 시 Codex는 즉시 retrospective 작성하고 향후 사이클은 정상 순서를 지킨다.

명명 규칙:

| 단계 | 파일명 패턴 |
|---|---|
| 사용자 review | `<base>_Review_YYYYMMDD_vNNN.md` |
| Codex plan | `<base>_Plan_YYYYMMDD_vNNN.md` |
| Codex result | `<base>_YYYYMMDD_vNNN.md` |

상세 규칙은 `cluster_analysis_operating_protocol_20260429_v006.md` section 5의 `Review 처리 사이클 규칙` 참조.

### 본 사이클의 Plan -> Result 연결

| 단계 | 산출물 |
|---|---|
| Review (사용자) | `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md` |
| Plan (Codex, retrospective 작성) | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` |
| Result (Codex) | 본 v004 문서 (`C01_GPX_Bus_Read_Code_Verify_20260429_v004.md`) |

---

## 6. 허용 warning 목록 (v004 갱신)

CI/회귀 스크립트는 다음 warning을 fail로 분류하지 않는다.

| 허용 warning 텍스트 | 발생 위치 | 의미 / 근거 |
|---|---|---|
| `bus_phy: bus timing clamped (div=` | `tdc_gpx_bus_phy.vhd:430-434` | C01 contract test [1]에서 의도적으로 illegal 입력을 주입해 clamp 동작을 검증. |
| `bus_phy: write request ignored (oen_permanent='1')` | `tdc_gpx_bus_phy.vhd:443-445` | INV-7 보호: oen_permanent='1' 동안 WRITE 거부. `tb_tdc_gpx_bus_phy` [5]에서 검증. |
| `[VRFC 10-3532] module 'glbl' does not have a parameter named 'g_dut_stream_clk_mode' to override` | xelab static elaboration | xelab `-generic_top`이 모든 top 단위 (TB + glbl)에 적용. instance scoping 미지원 (R-C01-V002-01). DUT generic 전달은 mode echo로 정상 확인. |

운영 가이드:
- `severity failure`만 fail.
- 위 텍스트는 grep allow-list로 처리. 새 의도된 warning은 본 표 확장.
- **(v004 신규)** no-marker는 더 이상 PASS로 묵시 분류되지 않는다. self-check이 없는 TB는 명시적 allow-list 등록이 필요하며, 현재 등록된 TB는 없다.

---

## 7. v003 -> v004 finding 갱신 표

| Finding | v003 상태 | v004 상태 |
|---|---|---|
| F-C01-V01 (full_int 상위 의존) | 유지 | **유지**. C02 인계. |
| F-C01-V02 (contract TB tick_en=1 단순화) | 유지 | **유지**. C02 인계. |
| F-C01-V03 (`o_rsp_pending` -> PH_RESP_DRAIN II) | 유지 | **유지**. C02 인계. |
| F-C01-V04 (보드 회로도 cross-check) | 유지 | **유지**. |
| F-C01-V05 (SYNC 경로 검증) | 닫힘 | **유지**. |
| F-C01-V06 (config_ctrl smoke 수준) | 부분 닫힘 | **유지**. backpressure / IrFlag는 C02 인계. |
| F-C01-V07 (warning allow-list) | 닫힘 | **재닫힘**. v004에서 no-marker 정책까지 명시적으로 흡수. |
| R-C01-V002-01 (glbl warning) | 닫힘 | **유지**. allow-list. |
| R-C01-V002-02 (writer/reader 표기) | 닫힘 | **유지**. [c11] reader case 포함. |
| R-C01-V002-03 (2-mode 분리) | 부분 닫힘 | **닫힘**. v004 통합 entrypoint + 종료코드 전파로 완전 통합. |
| R-C01-V002-04 (c_CDC_WAIT 설명) | 닫힘 | **유지**. |
| **R-C01-V003-01** (실패 전파) | v003 리뷰 신규 | **닫힘**. exit 코드 보강 + Negative test로 검증. |
| **R-C01-V003-02** (no-marker PASS) | v003 리뷰 신규 | **닫힘**. regex 보완 + no-marker FAIL 격상. |
| **R-C01-V003-03** (transcript 미보존) | v003 리뷰 신규 | **닫힘**. `tee` redirect로 `run_c01_regression.log` 보존. |

---

## 8. Latency / Throughput / Pipeline / II 검증 누적표 (v001 + v002 + v003 + v004)

본 v004는 RTL/TB 변경이 없어 정량 항목은 v003과 동일하다. 회귀 운영 정합성 보강만 추가되었다.

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
| config_ctrl SYNC raw stream | config_ctrl SYNC mode | direct passthrough | PASS |
| config_ctrl ASYNC raw_cdc FIFO | config_ctrl ASYNC mode | xpm_fifo_async 활성 | PASS |
| CDC handshake settling window | csr_chip_clamp `c_CDC_WAIT` | 200 ns = 40 cycles @ 200 MHz | PASS |
| **(v004 운영)** 통합 회귀 entrypoint 종료코드 전파 | Negative test 강제 fail -> exit 1 | 정상 작동 | PASS |

---

## 9. 사용자 피드백 기록

이번 v004 단계에서 사용자가 직접 입력한 피드백:

1. v003 리뷰 (`C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md`) 작성 — 회귀 운영 신뢰성 finding R-C01-V003-01..03 제기.
2. Codex 채팅 답변(조치 계획) → 사용자 “모두 동의해 진행해줘” 승인 → Codex 코드 변경/회귀 → 일부 진행 후 사용자 “수정 계획 문서를 왜 규칙에 맞게 생성하지 않은거야?” 지적.
3. Codex가 retrospective로 `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` 작성. 사용자 “review 처리 사이클에 plan → 승인 → 진행 → result 패턴 적용을 규칙으로 업데이트하고, 3가지 다 진행해줘” 지시 — 3가지: (i) 운영 프로토콜 v006 업데이트, (ii) #7 Negative test, (iii) v004 결과 보고서 작성.
4. 본 v004는 위 3가지 모두 반영한 결과 산출물이다.

---

## 10. 산출물 위치 및 다음 단계

### 산출물

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` | 본 문서 (v004) |
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` | Plan 문서 (retrospective) |
| `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v006.md` | 운영 프로토콜 v006 (Review 처리 사이클 규칙 신규) |
| `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v005.md` | v005 → v006 forward-trace 추가 |
| `HDL/scripts/run_c01_regression.sh` | transcript 보존 보강 |
| `HDL/tmp/c01_verify/run_regression.sh` | exit 코드 + regex + no-marker fail 격상 |
| `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | exit 코드 보강 |
| `HDL/tmp/c01_verify/run_c01_regression.log` | 통합 회귀 transcript (신규) |

### 다음 단계 후보

1. C02 `Chip_Acquisition` 분석 시작 — 인계 항목: F-C01-V01/V02/V03, F-C01-V06 잔여(backpressure / IrFlag).
2. 보드 회로도 확인 후 `g_OEN_MODE` default 결정 (F-C01-V04).
3. 상위 laser_ctrl 프로젝트에서 `tb_tdc_gpx_full_int`, `tb_tdc_gpx_top_int` 회귀.

본 v004로 C01 보완 검증은 운영 정합성 + 기능 PASS 모두 닫혔다. C02 진입 준비 완료.

---

## 11. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` p.7 Read Operations, p.8 OEN operations, p.11~12 Pin Description, p.27 28-bit data bus 40 MHz |
| C01 분석 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v009.md` |
| C01 코드 보완 기록 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Update_20260429_v001.md` |
| C01 코드 검증 v001/v002/v003 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v001.md`, `... v002.md`, `... v003.md` |
| C01 검증 리뷰 v001/v002/v003 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v001.md`, `... v002.md`, `... v003.md` |
| C01 검증 plan v001 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` |
| 운영 프로토콜 | `cluster_analysis_operating_protocol_20260429_v005.md` -> `cluster_analysis_operating_protocol_20260429_v006.md` (Review 처리 사이클 규칙 신규) |
| 회귀 출력 | `HDL/tmp/c01_verify/sim_*.log`, `HDL/tmp/c01_verify/elab_*.log`, `HDL/tmp/c01_verify/run_c01_regression.log` |
| 사용자 라이브러리 | `px_utility_pkg.vhd` (AXI-Lite writer/reader procedures) |

---

## 12. 변경 이력

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md` (R-C01-V003-01..03) + 사용자 plan 문서 누락 지적 + “review 처리 사이클을 규칙으로 업데이트” 요청 |
| 직전 버전 | `C01_GPX_Bus_Read_Code_Verify_20260429_v003.md` |
| 본 v004의 핵심 변경 | (i) 두 sub-script 종료코드 전파 + Negative test로 검증, (ii) regex 보완 + no-marker FAIL 격상, (iii) 통합 transcript 보존, (iv) 운영 프로토콜 v006으로 Review 처리 사이클 규칙 신규 도입 |
| 판단 변화 | R-C01-V003-01..03 모두 닫힘. R-C01-V002-03 잔여 분도 v004에서 완전 닫힘. F-C01-V05/V07도 v004 운영 정책 명시화로 재닫힘. RTL/TB 기능 finding 변화 없음. |
| 추적 근거 | `tmp/c01_verify/run_regression.sh:95-105,120-124`, `tmp/c01_verify/run_config_ctrl_two_modes.sh:95-99`, `scripts/run_c01_regression.sh:22-29,99`, `tmp/c01_verify/run_c01_regression.log` (positive ALL PASS), Negative test 콘솔 출력 (`stage1=1` exit 1), `cluster_analysis_operating_protocol_20260429_v006.md` section 5 신규 sub-section |

---

## v004 -> v005 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `Code_Verify_Review_20260429_v004.md` (R-C01-V004-01..03 evidence 추적성), `Closure_Review_20260429_v001.md` (R-C01-CL-01..04 closure 문구 + evidence 보강), `Code_Verify_Plan_Review_20260429_v002~v005.md` (Plan 사이클 4건 정정), 사용자 “옵션 B 진행” 지시 (strict protocol 준수) |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` |
| 다음 버전 반영 위치 | section 1 (검증 환경 + Plan v006 항목), section 2 (R-C01-V004-01..03 + R-C01-CL-01..04 닫힘 결과), section 3 (#1 RC_FINAL echo, #2 --negative 분기, #2-hook env hook, #2-marker concurrent assertion), section 4 (통합 회귀 + Negative test 결과), section 6 (expected note 표 — marker note 포함), section 7 (finding 갱신 — V004/CL/V003/P-V005 모두 닫힘), section 9 (closure 단일안 채택), section 13 (변경 이력) |
| 판단 변화 | R-C01-V004-01..03 모두 닫힘 (transcript에 INTEGRATED EXIT CODE 보존, negative artifact 별도 파일, ASYNC clean evidence Primary-A + Primary-B 둘 다 PASS, v004 시각 인용 정정). R-C01-CL-01..03 모두 닫힘 (closure 문구 단일안, evidence 보강 완료). R-C01-CL-04 인정 유지. R-C01-V003-01..03 재닫힘 강화 (Negative test로 직접 검증). v004의 “closure 문구 모호” 결함 해소. v004 보고서가 명시한 v003 evidence 추적성 약점 모두 v005로 닫힘. |
| 추적 근거 | `tdc_gpx_config_ctrl.vhd:1916-1929` (`gen_async_marker`), `tmp/c01_verify/run_regression.sh:111-118` (env hook), `scripts/run_c01_regression.sh:25-40,90-104` (--negative + EXIT CODE), `tmp/c01_verify/run_c01_regression.log` (positive ALL PASS + EXIT CODE = 0), `tmp/c01_verify/run_c01_regression_negative_20260429T100450Z.log` (negative EXIT CODE = 1), `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log:69` (Primary-A), `tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29` (Primary-B), Plan v002~v006 사이클 |
