# C02_Chip_Acquisition Code Fix Plan v001

문서 버전: `v001`  
작성일: `2026-04-30`  
최종 수정 시간: `2026-04-30 12:49:44 +09:00`  
작성 목적: `C02_Chip_Acquisition_C01_Handoff_20260430_v002.md`의 C01 v009 계약 32건 전체 수락 matrix를 기준으로 C02 RTL/TB/문서 보완 계획을 정의한다.

---

## 1. 계획 기준

본 계획은 아래 문서를 입력으로 한다.

| 기준 | 적용 방식 |
|---|---|
| `Doc/TDC-GPX-Datasheet.pdf` | 최상위 절대 기준. 특히 p.8/p.49 empty Interface FIFO read 금지, EF/LF active HIGH, p.27/p.49 40 MHz readout 제한을 따른다. |
| `C01_GPX_Bus_Read_20260429_v009.md:1023-1058` | C01에서 C02로 넘긴 32개 계약 원본. |
| `C02_Chip_Acquisition_C01_Handoff_20260430_v002.md` | 32개 계약을 C02 입력 계약으로 전부 수락한 최신 Handoff 문서. |
| `C02_Chip_Acquisition_20260429_v001.md` | C02 분석 v001. F-C02-01~05 finding과 보완 방향 후보. |
| 현재 RTL/TB | `tdc_gpx_chip_run.vhd`, `tdc_gpx_chip_ctrl.vhd`, `tb_tdc_gpx_chip_ctrl.vhd`, 필요 시 `tdc_gpx_config_ctrl.vhd`. |

이번 계획은 코드 변경을 바로 수행하는 문서가 아니다. 사용자 승인 후 이 계획을 기준으로 RTL/TB 보완과 xsim 검증을 진행한다.

---

## 2. Handoff 계약 우선순위화

C01 v009의 32개 계약은 모두 C02 입력 계약으로 수락한다. 다만 C02 code-fix 단계에서 직접 닫을 항목과 문서/후속 Cluster로 넘길 항목은 구분한다.

| 우선순위 | C01 계약 ID | C02 처리 판단 |
|---|---|---|
| P0, 데이터시트 위반 방지 | C01-C11, C01-C14, C01-C21, C01-C22 | empty FIFO read 금지, 40 MHz readout 제한, `tS-EF + 2FF` guard를 RTL/TB로 직접 검증한다. |
| P0, response/stop 안정성 | C01-C5, C01-C6, C01-C7, C01-C17, C01-C28, C01-C31 | `o_rsp_pending`, PH_RESP_DRAIN, non-burst/burst II, response hold/backpressure 영향을 검증한다. |
| P1, drain bound | C01-C3, C01-C8, C01-C18, C01-C29 | READ response와 status sync level 계약을 이용해 raw data path와 status event 해석을 닫는다. |
| P1, timing legality | C01-C9, C01-C10, C01-C13, C01-C19 | 200 MHz 기준, `div=1=>ticks>=5`, latency 기준점 분리를 C02 문서와 test expectation에 반영한다. |
| P2, mode/board 계약 | C01-C12, C01-C15, C01-C16, C01-C20 | 28-bit mode, OEN mode, stream clock domain은 C02에서 계약으로 유지하고, 전체 CDC/board 결정은 후속 Cluster와 연결한다. |
| P2, retiming 후보 | C01-C23~C01-C27, C01-C30 | 250 MHz retiming과 125~200 MHz 후보표는 이번 RTL 보완의 직접 변경 범위에서 제외하고 문서/후속 영향성 항목으로 둔다. |
| P0, 산출물 규칙 | C01-C32 | 결과 문서와 PPT에 latency, throughput, pipeline, II, timing diagram/block diagram을 포함한다. |

---

## 3. 수정 목표

### 목표 A. Empty FIFO read 금지 조건을 검증으로 먼저 보이게 만든다

연결 계약:

- C01-C11: EF1/EF2 active HIGH = empty, empty FIFO read 금지
- C01-C21/C22: `tS-EF max 11.8 ns` + 2-FF sync latency
- F-C02-01: 현재 TB가 extra read를 정상 범위로 허용
- F-C02-04: TB raw count가 data/control beat를 분리하지 않음

수정 대상:

| 파일 | 계획 |
|---|---|
| `tb_tdc_gpx_chip_ctrl.vhd` | chip model에서 Reg8/Reg9 IFIFO read 시 fill=0이면 empty-read violation으로 집계하고, strict mode에서는 즉시 FAIL 처리한다. |
| `tb_tdc_gpx_chip_ctrl.vhd` | `sv_raw_word_cnt`를 data beat count, control beat count, empty read count로 분리한다. `tuser(7)=1`은 control beat로 따로 센다. |
| `tb_tdc_gpx_chip_ctrl.vhd` | 기존 `12..16`, `24..28`처럼 extra read를 허용하던 pass 조건을 제거하고, data beat 수와 empty read 수를 별도로 검증한다. |

완료 기준:

- 정상 drain 시 `empty_read_count = 0`.
- control beat는 raw data word count에 섞이지 않는다.
- 기존 pass 문구가 "sync overhead ok"처럼 empty read 가능성을 정상화하지 않는다.

### 목표 B. RTL에서 EF sync 지연으로 인한 fallback over-read를 구조적으로 줄인다

연결 계약:

- C01-C21/C22: 200 MHz에서 raw `tS-EF` guard는 3 clocks지만 `EF_sync` 관측은 최소 4 clocks, 보수적으로 5 clocks 검토
- F-C02-01/F-C02-03: `expected_ififo`가 hard bound가 아니고, single EF drain에서 over-read 가능

수정 대상:

| 파일 | 계획 |
|---|---|
| `tdc_gpx_chip_run.vhd` | `ST_DRAIN_SETTLE`의 hardcoded 3 clock guard를 named constant 또는 generic으로 분리한다. 권장 기본값은 5 clocks @200 MHz이다. |
| `tdc_gpx_chip_run.vhd` | `expected_ififo1/2`가 0이 아닌 신뢰 가능한 값일 때는 `drain_cnt >= expected_ififo`를 done/hard bound 조건에 포함한다. |
| `tdc_gpx_chip_run.vhd` | expected count가 0인 EF fallback mode에서는 각 read 후 EF sync가 갱신될 시간을 충분히 주도록 guard를 적용한다. |
| `tdc_gpx_chip_ctrl.vhd` | `chip_run`에 guard generic/constant가 추가되면 instance mapping과 주석/계약을 정리한다. |

권장 구현 방향:

- 기본 안전값: `g_EF_SYNC_GUARD_CLKS = 5`.
- 예상 count가 0이 아닌 경우: expected count hard bound 우선 + EF high도 done 조건으로 병행.
- 예상 count가 0인 fallback 경우: EF sync guard 강화로 empty read 위험을 줄인다.
- exact generic/constant 이름은 구현 시 VHDL naming rule에 맞춰 `g_`, `c_`, `s_` prefix를 적용한다.

완료 기준:

- TB strict empty-read assertion 통과.
- 기존 정상 acquisition test가 data/control count 기준으로 통과.
- guard 증가로 인한 latency/II 변화가 C02 결과 문서에 반영된다.

### 목표 C. PH_RESP_DRAIN / pending / backpressure 계약을 검증한다

연결 계약:

- C01-C5/C6/C7: response hold, pending response 중 new request stall, `o_rsp_pending` 의미
- C01-C17: pending register boundary 개선 시 1 clock 지연 영향
- C01-C28/C29/C31: latency, throughput, burst/non-burst II 분리

수정 대상:

| 파일 | 계획 |
|---|---|
| `tb_tdc_gpx_chip_ctrl.vhd` | raw path backpressure 시 PH_RESP_DRAIN이 stale response를 안전하게 drain하는지 monitor를 추가한다. |
| `tb_tdc_gpx_chip_ctrl.vhd` | `i_bus_rsp_pending`, `i_s_axis_tvalid`, `o_s_axis_tready`, `drain_done`, raw output handshake를 함께 로깅한다. |
| `tdc_gpx_chip_ctrl.vhd` | 필요한 경우 PH_RESP_DRAIN exit 조건과 hard cap/fatal 관련 주석을 C01-C5~C7 계약 기준으로 정리한다. 기능 변경은 TB 결과를 본 뒤 결정한다. |

완료 기준:

- best-case II와 backpressure case II가 문서에 분리된다.
- PH_RESP_DRAIN이 per-shot drain 단계가 아니라 stale response flush/quarantine 단계임을 검증 결과와 문서에 명확히 남긴다.

### 목표 D. Quiet/M-mode 운용 범위를 명확히 제한한다

연결 계약:

- F-C02-02: Quiet/M-mode flow와 현재 `capture -> drain -> AluTrigger` 순서의 충돌 가능성
- C01-C18: status pin은 level로 사용, pulse event capture로 해석하지 않음

이번 code-fix 범위에서는 Quiet/M-mode full FSM을 새로 구현하지 않는다.

수정/문서 계획:

| 파일 | 계획 |
|---|---|
| `C02_Chip_Acquisition_20260430_v002.md` 예정 | nominal 운용을 `non-quiet + MasterAluTrig cleanup`으로 제한하는 계약을 명시한다. |
| RTL | Quiet/M-mode를 실제 지원해야 한다는 사용자 결정이 나오기 전까지 FSM 순서 변경은 하지 않는다. |

완료 기준:

- C02 결과 문서에 "지원 모드"와 "후속 확장 모드"가 분리된다.
- Quiet/M-mode 지원이 필요하면 별도 C02-Mode 또는 C03 이후 계획으로 분리한다.

### 목표 E. 결과 문서/PPT를 Handoff v002 기준으로 재작성한다

연결 계약:

- C01-C32: 모든 후속 Cluster 산출물은 latency, throughput, pipeline, II와 timing diagram/block diagram 포함
- Handoff v002: 32개 계약 원본 ID 유지

수정 대상:

| 파일 | 계획 |
|---|---|
| `C02_Chip_Acquisition_20260430_v002.md` | C01 32개 계약 full matrix를 C02 입구 조건으로 반영하고, 코드 보완 결과와 finding closure를 기록한다. |
| `C02_Chip_Acquisition_20260430_v002.pptx` | 32개 계약은 그룹화해 보여주되, slide note 또는 하단 근거에 C01 원본 ID 범위를 남긴다. |
| `C02_Chip_Acquisition_Code_Verify_20260430_v001.md` 예정 | xsim 검증 방법, 로그, positive/negative 결과, latency/II 측정 결과를 기록한다. |

완료 기준:

- Markdown에는 32개 계약 원본 ID가 유지된다.
- PPT에는 empty FIFO read guard timing diagram, drain pipeline/II block diagram이 포함된다.
- 검증 결과는 positive exit code와 negative/fail-detect evidence를 함께 남긴다.

---

## 4. 진행 순서

### Step 0. Baseline 확인

목적: 현재 RTL/TB 기준에서 어떤 test가 어떤 이유로 통과하는지 기록한다.

작업:

1. `git status --short` 확인.
2. 현재 `tb_tdc_gpx_chip_ctrl` xsim 실행 가능 여부 확인.
3. 기존 PASS 결과에서 `12..16`, `24..28` 같은 extra read 허용 조건을 baseline으로 기록한다.

산출물:

- baseline log 또는 실행 불가 사유.
- 이후 결과 문서의 "before" 근거.

### Step 1. TB-first strict 검증 보강

목적: 데이터시트 위반을 TB가 놓치지 않게 만든다.

작업:

1. chip model에서 fill=0 상태의 Reg8/Reg9 read를 `empty_read_violation`으로 검출한다.
2. strict mode에서는 즉시 FAIL로 처리한다.
3. raw output monitor에서 data beat와 control beat를 분리한다.
4. 기존 range pass 조건을 정확한 data/control/empty read 조건으로 바꾼다.

예상 결과:

- 현재 RTL이 EF sync 지연으로 empty read를 만든다면 Step 1 직후 simulation은 FAIL할 수 있다.
- 이 FAIL은 목표한 진단 실패이며, Step 2 RTL guard 보완의 근거가 된다.

### Step 2. RTL guard 보완

목적: empty read를 구조적으로 막거나 최소화한다.

작업:

1. `ST_DRAIN_SETTLE` guard를 5 clocks 기본값으로 강화한다.
2. `expected_ififo`가 0이 아닌 경우 hard bound done 조건을 추가한다.
3. burst path와 single EF fallback path가 같은 empty-read 금지 계약을 따르는지 점검한다.
4. 필요 시 `chip_ctrl` instance generic/constant 전달을 정리한다.

주의:

- guard 증가로 single EF fallback II가 증가한다.
- expected count가 stale이면 hard bound가 오동작할 수 있으므로, 기존 `ST_DRAIN_LATCH` settle과 mismatch sticky 정책을 함께 유지한다.

### Step 3. Backpressure / PH_RESP_DRAIN 검증

목적: C01 response 계약이 C02 phase FSM에서 깨지지 않는지 확인한다.

작업:

1. raw output tready stall 시나리오를 재검증한다.
2. `i_bus_rsp_pending`, `i_s_axis_tvalid`, `o_s_axis_tready`의 관계를 transcript에 남긴다.
3. PH_RESP_DRAIN exit, hard cap, bus fatal/auto-recover 시나리오가 기존 계약과 맞는지 확인한다.

### Step 4. xsim regression 정리

목적: C02 검증을 재실행 가능한 형태로 남긴다.

작업:

1. 가능하면 `scripts/run_c02_regression.sh` 또는 동등한 C02 entrypoint를 만든다.
2. positive run은 최종 `C02 INTEGRATED EXIT CODE = 0`를 남긴다.
3. negative run은 강제 empty-read violation 또는 monitor forced fail로 `C02 INTEGRATED EXIT CODE = 1`을 남긴다.
4. transcript 파일명을 Windows-safe timestamp로 남긴다.

### Step 5. 결과 문서와 PPT 갱신

목적: 수정 결과를 사용자가 추적 가능한 형태로 닫는다.

작업:

1. `C02_Chip_Acquisition_Code_Verify_20260430_v001.md` 생성.
2. `C02_Chip_Acquisition_20260430_v002.md` 생성.
3. `C02_Chip_Acquisition_20260430_v002.pptx` 생성.
4. 이전 문서에는 forward trace를 추가한다.
5. Git commit은 검증 결과 단위로 분리한다.

---

## 5. 검증 항목

| 검증 ID | 검증 내용 | 기대 결과 |
|---|---|---|
| V-C02-01 | Empty FIFO read strict assertion | 정상 run에서 violation 0 |
| V-C02-02 | Data/control beat 분리 count | data beat와 `tuser(7)=1` control beat가 별도 집계 |
| V-C02-03 | Fill 8/4 drain | data beat 12, empty read 0, drain_done control beat 관측 |
| V-C02-04 | Fill 16/8 drain | data beat 24, empty read 0, drain_done control beat 관측 |
| V-C02-05 | EF=1 empty start | data beat 0, empty read 0, drain_done 정상 |
| V-C02-06 | expected_ififo hard bound | expected count 초과 read 없음 |
| V-C02-07 | raw backpressure | no raw/control beat drop, PH_RESP_DRAIN 계약 유지 |
| V-C02-08 | PH_RESP_DRAIN stuck/fatal path | hard cap/fatal/auto-recover 동작이 transcript로 추적 가능 |
| V-C02-09 | Negative forced empty read | negative run이 exit code 1로 실패 전파 |
| V-C02-10 | Latency/Throughput/Pipeline/II 산출 | guard 변경 전후 차이 문서화 |

---

## 6. Latency / Throughput / Pipeline / II 영향 예측

| 항목 | 보완 전 | 보완 후 예상 |
|---|---|---|
| EF fallback settle | `ST_DRAIN_SETTLE` 3 clocks = 15 ns @200 MHz | 5 clocks = 25 ns @200 MHz 권장 |
| single EF drain II | C01 read latency + 3 clock settle + decision overhead | C01 read latency + 5 clock settle + decision overhead |
| burst/readout 상한 | C01 계약상 `bus_ticks * bus_clk_div * Tclk`, 40 MHz 이하 | 동일. 단 expected hard bound로 불필요한 tail read 제거 |
| throughput | fallback single read는 guard 증가로 낮아질 수 있음 | 데이터시트 empty-read 금지 만족을 우선 |
| pipeline | bus_phy capture -> chip_run raw -> chip_ctrl raw FIFO -> output | 동일. monitor에서 data/control beat 분리 |

판단:

- C02의 최우선 목표는 throughput 극대화가 아니라 데이터시트의 empty FIFO read 금지 조건을 닫는 것이다.
- expected count가 신뢰 가능한 경우 hard bound를 사용하면 guard 증가에 따른 throughput 손실을 일부 줄일 수 있다.
- fallback EF-only mode는 안전성을 위해 II 증가를 감수한다.

---

## 7. 범위 제외 / 후속 항목

| 항목 | 이번 범위 제외 이유 | 후속 처리 |
|---|---|---|
| Quiet/M-mode full sequence 지원 | 현재 RTL은 nominal `non-quiet + MasterAluTrig cleanup` 운용에 더 가깝다. FSM 순서 변경은 큰 기능 변경이다. | 사용자 결정 후 별도 mode-support plan |
| 250 MHz retiming | C01 계약상 `div=1,ticks=5`가 50 MHz readout이 되어 금지된다. cfg_pkg/CSR/TB/timeouts 재검토가 필요하다. | 별도 retiming 영향성 Cluster |
| output stream CDC 전체 재설계 | C01에서 ASYNC 기본값과 xpm_fifo_async evidence는 닫혔다. | C03/C04 stream CDC 분석 |
| 16-bit bus mode 지원 | 현재 RTL은 28-bit 기준이며 Reg14[4] 차단 정책 유지가 계약이다. | 별도 기능 확장 |
| OEN board default 최종 결정 | board schematic/연결 상태가 필요하다. | board integration 확인 후 결정 |

---

## 8. 위험과 완화

| 위험 | 영향 | 완화 |
|---|---|---|
| TB strict assertion 추가 직후 기존 test FAIL | 현재 RTL의 over-read 가능성이 드러남 | 의도된 진단 실패로 기록하고 Step 2 RTL guard로 닫는다. |
| EF guard 5 clocks로 throughput 저하 | fallback drain 시간이 증가 | expected_ififo hard bound를 병행하고, throughput/II 변화를 문서화한다. |
| expected_ififo stale value | hard bound가 너무 빠르거나 늦게 drain 종료 | 기존 16-cycle latch settle과 mismatch sticky 유지, expected=0 fallback 분리 |
| control beat를 data count에서 빼면서 기존 기대값 변경 | 기존 PASS 기준이 달라짐 | data/control/empty read count를 각각 명시해 더 강한 기준으로 바꾼다. |
| Quiet/M-mode 사용자 기대와 불일치 | mode 지원 범위 혼동 | 이번 계획에서는 mode 제한을 문서화하고, 지원 필요 시 별도 계획으로 분리한다. |

---

## 9. Git 운영 계획

| 단계 | Commit 단위 |
|---|---|
| 계획 작성 | `docs: plan C02 fixes from C01 handoff` |
| TB-first 보강 | `test: detect C02 empty FIFO reads strictly` |
| RTL guard 보완 | `rtl: guard C02 IFIFO drain against empty reads` |
| regression/script | `test: add C02 regression evidence flow` |
| 결과 문서/PPT | `docs: report C02 verification closure` |

각 commit 전후로 `git status --short`를 확인하고, unrelated untracked 파일은 포함하지 않는다.

---

## 10. 사용자 승인 요청 항목

본 계획 기준으로 진행하려면 아래 결정을 승인받아야 한다.

| ID | 승인 항목 | 추천 |
|---|---|---|
| A-C02-01 | TB strict empty-read assertion을 먼저 추가하고, 현재 RTL이 FAIL하면 진단 실패로 기록 | 승인 권장 |
| A-C02-02 | EF fallback guard 기본값을 5 clocks @200 MHz로 강화 | 승인 권장 |
| A-C02-03 | `expected_ififo > 0`일 때 hard read bound를 적용 | 승인 권장 |
| A-C02-04 | data/control beat count를 분리하고 기존 range pass 기준 제거 | 승인 권장 |
| A-C02-05 | Quiet/M-mode full support는 이번 범위에서 제외하고 nominal non-quiet 운용으로 문서화 | 승인 권장 |
| A-C02-06 | C02 xsim positive/negative regression entrypoint를 만든다 | 승인 권장 |

---

## 11. Version Lineage

| 항목 | 내용 |
|---|---|
| 입력 Handoff | `C02_Chip_Acquisition_C01_Handoff_20260430_v002.md` |
| 입력 분석 | `C02_Chip_Acquisition_20260429_v001.md` |
| 본 계획 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v001.md` |
| 다음 산출물 | 사용자 승인 후 RTL/TB 보완, `C02_Chip_Acquisition_Code_Verify_20260430_v001.md`, `C02_Chip_Acquisition_20260430_v002.md`, PPT v002 |
| 판단 변화 | C01 계약 32건 전체 수락을 C02 수정 우선순위와 검증 항목으로 확장 |
