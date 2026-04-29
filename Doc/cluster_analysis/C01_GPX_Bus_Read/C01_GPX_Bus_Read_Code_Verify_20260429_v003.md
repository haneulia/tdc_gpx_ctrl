# C01_GPX_Bus_Read 코드 검증 기록 v003

문서 버전: `v003`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 16:31:00 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v002.md`(R-C01-V002-01..04) 리뷰 결과를 반영해 v002 검증 산출물의 운용/문서 정합성을 정정한다. 절대 기준은 `Doc/TDC-GPX-Datasheet.pdf`이며 RTL clock은 `i_tdc_clk = 200 MHz` (`Tclk = 5 ns`)이다.

---

## 1. 검증 환경 (v002 동일 + 통합 entrypoint 추가)

| 항목 | 값 |
|---|---|
| 시뮬레이터 | Vivado 2025.2.1 xsim |
| Vivado 경로 | `C:/AMDDesignTools/2025.2.1/Vivado` |
| 실행 디렉터리 | `C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim` |
| 통합 .prj | `__c01_all_vhdl.prj` (RTL + CSR IP source + 회귀 TB) |
| **통합 회귀 entrypoint** | **`HDL/scripts/run_c01_regression.sh` (v003 신규)** |
| Stage 1 (4 TB 직접) | `HDL/tmp/c01_verify/run_regression.sh` |
| Stage 2 (config_ctrl 2-mode) | `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` |
| Stage 3 (CSR clamp) | `run_c01_regression.sh` 안에 inline xelab/xsim |
| 로그 보존 폴더 | `HDL/tmp/c01_verify/` |
| 테스트벤치 라이브러리 | `px_utility_pkg` (`px_axi_lite_writer` + `px_axi_lite_reader`) |

---

## 2. v002 리뷰 반영 정정 (R-C01-V002-01..04)

### 2.1 R-C01-V002-01: glbl generic override warning -> **allow-list 처리** (instance scoping 미지원 확인)

| 항목 | 내용 |
|---|---|
| 시도한 조치 | `xelab -generic_top "tb_tdc_gpx_config_ctrl.g_DUT_STREAM_CLK_MODE=${mode}"` (instance scoping) |
| 결과 | 실패. xelab error: `[XSIM 43-3281] Parameter/Generic tb_tdc_gpx_config_ctrl.g_DUT_STREAM_CLK_MODE specified in commandline not found in design.` |
| 결론 | Vivado 2025.2.1 xelab은 `-generic_top` 의 instance scoping 형식을 지원하지 않는다. 단일 형식 `-generic_top "<name>=<value>"`만 받으며, 다중 top 단위 (TB + glbl) 모두에 적용된다. |
| 채택 조치 | (B) 허용 warning 목록에 `VRFC 10-3532 glbl generic override` 항목 추가. v003 보고서 section 6에 명시. |
| 추적 근거 | `elab_tb_tdc_gpx_config_ctrl_SYNC.log:8`, `elab_tb_tdc_gpx_config_ctrl_ASYNC.log:8` |

### 2.2 R-C01-V002-02: writer/reader 표기와 실제 구현 불일치 -> **TB에 reader case 추가**

| 항목 | 내용 |
|---|---|
| v002 상태 | 헤더와 보고서가 “writer/reader 사용” 표기, 실제 TB는 writer만 호출 |
| v003 조치 | 두 가지 동시 적용 — (i) 실제 사용 사실에 맞게 헤더/보고서 정정, (ii) 마지막 case로 `px_axi_lite_reader`를 추가해 CTL1 raw readback 검증 (case [c11]) |
| 결과 | TB 케이스 11 -> 12 로 확장. 기존 11 clamp PASS + readback PASS 1건. |
| 추적 근거 | `tb_tdc_gpx_csr_chip_clamp.vhd:7-12` (헤더 정정), `tb_tdc_gpx_csr_chip_clamp.vhd:295-321` ([c11] reader case), `sim_tb_tdc_gpx_csr_chip_clamp.log`: `[c11] readback CTL1 via px_axi_lite_reader` -> `Read completed successfully` -> `[c11] PASS  CTL1 readback matches last write (raw)` -> `*** ALL TESTS PASSED *** (cases=12)` |

readback의 의미:
- CSR IP는 AXI write 결과를 register file에 raw로 저장한다.
- `o_bus_clk_div`/`o_bus_ticks`는 register file의 raw 값을 clamp한 결과다.
- 따라서 readback은 raw값(예: `(8,7)` -> x"000001C8")을 반환하고, clamp 동작은 별도 `o_bus_*` 출력으로 측정된다. v003 readback case는 이 분리를 명시적으로 검증한다.

### 2.3 R-C01-V002-03: 2-mode 회귀가 main entrypoint와 분리됨 -> **통합 entrypoint 신설**

| 항목 | 내용 |
|---|---|
| v002 상태 | `scripts/run_all_tbs.tcl`만 실행하면 ASYNC 누락. 별도 `run_config_ctrl_two_modes.sh` 필요. |
| v003 조치 | 통합 entrypoint `HDL/scripts/run_c01_regression.sh` 신설. Stage 1 (4 TB 회귀) + Stage 2 (config_ctrl SYNC/ASYNC) + Stage 3 (CSR clamp)을 한번에 실행. 종료 코드로 통합 PASS/FAIL 판정. |
| 추가 조치 | `scripts/run_all_tbs.tcl` 헤더 주석에 `run_c01_regression.sh` 참조 명시. |
| 추적 근거 | `HDL/scripts/run_c01_regression.sh:1-95`, `HDL/scripts/run_all_tbs.tcl:1-15` (헤더 주석) |

### 2.4 R-C01-V002-04: c_CDC_WAIT 설명 정합성 -> **주석 정정**

| 항목 | 내용 |
|---|---|
| v002 상태 | 코드 `c_CDC_WAIT : time := 200 ns`, 주석은 “~150 ns >= 30 i_axis_aclk cycles” |
| v003 조치 | 헤더 주석을 `200 ns = 40 i_axis_aclk cycles @ 200 MHz`로 정정. |
| 추적 근거 | `tb_tdc_gpx_csr_chip_clamp.vhd:24-26` |

---

## 3. v003 통합 회귀 결과

`bash HDL/scripts/run_c01_regression.sh` 단일 명령으로 전체 회귀가 닫힌다.

### Stage 1: 4 TB 직접 회귀

| TB | runtime | 결과 |
|---|---:|---|
| `tb_tdc_gpx_bus_phy_c01_contract` | 2 us | PASS (`tb_tdc_gpx_bus_phy_c01_contract PASS`) |
| `tb_tdc_gpx_bus_phy` | 200 us | PASS (`*** ALL TESTS PASSED *** (total_rsp=85)`) |
| `tb_tdc_gpx_chip_ctrl` | 200 us | PASS (`*** ALL TESTS PASSED *** (total_raw_words=224)`) |
| `tb_tdc_gpx_config_ctrl` (default SYNC) | 200 us | PASS (`TB: PASS -- init complete, active_chip_mask = 15`) |

### Stage 2: config_ctrl 2-mode

| Mode | 결과 | xpm_fifo_async 컴파일 |
|---|---|---|
| `g_DUT_STREAM_CLK_MODE=SYNC` | PASS | 0건 (bypass) |
| `g_DUT_STREAM_CLK_MODE=ASYNC` | PASS | 1건 (raw_cdc 활성) |

### Stage 3: CSR clamp 12 case

| Case | 입력 (div, ticks) | 기대 출력 / readback | 결과 |
|---|---|---|---|
| init | x"00000142" | div=2, ticks=5 | PASS |
| c01 | (0, 4) | div=1, ticks=5 | PASS |
| c02 | (1, 3) | div=1, ticks=5 | PASS |
| **c03** | **(1, 4)** | **div=1, ticks=5** (C01 contract 핵심) | **PASS** |
| c04 | (1, 5) | div=1, ticks=5 (legal boundary) | PASS |
| c05 | (1, 7) | div=1, ticks=7 | PASS |
| c06 | (2, 3) | div=2, ticks=4 | PASS |
| c07 | (2, 4) | div=2, ticks=4 (legal boundary) | PASS |
| c08 | (2, 5) | div=2, ticks=5 | PASS |
| c09 | (3, 4) | div=3, ticks=4 | PASS |
| c10 | (8, 7) | div=8, ticks=7 | PASS |
| **c11** | **`px_axi_lite_reader`** | **CTL1 raw readback = `fn_pack_bus_timing(8,7)`** | **PASS** |

전체 marker: `*** ALL TESTS PASSED *** (cases=12)`.

### 통합 entrypoint 종료 코드

```text
############################################################
# C01 regression: ALL PASS
############################################################
```

---

## 4. 변경 파일 점검 (v002 -> v003)

| 파일 | 변경 의도 | 추적 위치 |
|---|---|---|
| `tb_tdc_gpx_csr_chip_clamp.vhd` | (i) 헤더 “writer/reader” 정정, (ii) `c_CDC_WAIT` 주석 200 ns/40 cycles로 통일, (iii) `px_axi_lite_reader` readback case [c11] 추가 | `tb_tdc_gpx_csr_chip_clamp.vhd:7-12`, `tb_tdc_gpx_csr_chip_clamp.vhd:24-26`, `tb_tdc_gpx_csr_chip_clamp.vhd:64`, `tb_tdc_gpx_csr_chip_clamp.vhd:295-321` |
| `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | instance-scoped generic 시도 -> 미지원 확인 -> 원복. 운영 정책은 allow-list 적용 | (변경 없음, 시도 후 원복) |
| `HDL/scripts/run_c01_regression.sh` | 신규 — Stage 1/2/3 통합 entrypoint | `HDL/scripts/run_c01_regression.sh:1-95` |
| `HDL/scripts/run_all_tbs.tcl` | 헤더 주석에 `run_c01_regression.sh` 참조 안내 | `HDL/scripts/run_all_tbs.tcl:1-15` |

RTL은 변경 없다 (`tdc_gpx_cfg_pkg.vhd`, `tdc_gpx_csr_chip.vhd`, `tdc_gpx_bus_phy.vhd`, `tdc_gpx_config_ctrl.vhd`, `tdc_gpx_top.vhd`).

---

## 5. 사용자 피드백 기록

이번 v003 단계에서 사용자가 직접 입력한 피드백:

1. v002 검증 보고서에 대한 v002 리뷰 (`C01_GPX_Bus_Read_Code_Verify_Review_20260429_v002.md`) 작성. R-C01-V002-01..04 정정 권장 4건.
2. “진행해” — 4건 모두 v003에 반영.
3. 이전 단계의 “`px_utility_pkg`에 정의한 것을 활용해줘” 가이드는 v003에서도 유지된다 — `px_axi_lite_writer` + `px_axi_lite_reader` 모두 활용.

---

## 6. 허용 warning 목록 (v003 갱신)

CI/회귀 스크립트는 다음 warning을 fail로 분류하지 않는다.

| 허용 warning 텍스트 | 발생 위치 | 의미 / 근거 |
|---|---|---|
| `bus_phy: bus timing clamped (div=` | `tdc_gpx_bus_phy.vhd:430-434` | C01 contract test [1]에서 의도적으로 illegal 입력 주입해 clamp 동작을 검증. |
| `bus_phy: write request ignored (oen_permanent='1')` | `tdc_gpx_bus_phy.vhd:443-445` | INV-7 보호: oen_permanent='1' 동안 WRITE 거부. `tb_tdc_gpx_bus_phy` [5]에서 검증. |
| `[VRFC 10-3532] module 'glbl' does not have a parameter named 'g_dut_stream_clk_mode' to override` | `xelab` static elaboration | xelab `-generic_top`이 모든 top 단위 (TB + glbl)에 적용됨. instance scoping 미지원 (R-C01-V002-01). DUT generic 전달은 mode echo로 정상 확인. |

운영 가이드:
- `severity failure`만 fail.
- 위 텍스트는 grep allow-list로 처리. 새 의도된 warning이 추가되면 본 표를 확장.

---

## 7. v002 -> v003 finding 갱신 표

| Finding | v002 상태 | v003 상태 |
|---|---|---|
| F-C01-V01 (full_int 상위 의존) | 유지 | **유지**. 본 IP xpr 단독 검증 범위 밖. |
| F-C01-V02 (contract TB tick_en=1 단순화) | 유지 | **유지**. C02에서 div>=2 burst II 정량화 인계. |
| F-C01-V03 (`o_rsp_pending` -> PH_RESP_DRAIN II) | 유지 | **유지, C02 인계**. |
| F-C01-V04 (보드 회로도 cross-check) | 유지 | **유지**. |
| F-C01-V05 (SYNC 경로 검증) | v002에서 닫힘 | **닫힘 (유지)**. v003 통합 entrypoint로 매 회귀 자동 실행. |
| F-C01-V06 (config_ctrl smoke 수준) | 부분 닫힘 | **부분 닫힘 (유지)**. backpressure / IrFlag는 C02 인계. |
| F-C01-V07 (warning allow-list) | v002에서 닫힘 | **재닫힘**. v003 section 6에 `VRFC 10-3532 glbl override` 항목 추가. |
| **R-C01-V002-01** (glbl generic warning) | v002 리뷰 신규 | **닫힘** (allow-list 적용, instance scoping 미지원 확인). |
| **R-C01-V002-02** (writer/reader 표기) | v002 리뷰 신규 | **닫힘** (헤더 정정 + reader case [c11] 추가). |
| **R-C01-V002-03** (2-mode 분리) | v002 리뷰 신규 | **닫힘** (`run_c01_regression.sh` 통합 entrypoint). |
| **R-C01-V002-04** (c_CDC_WAIT 설명) | v002 리뷰 신규 | **닫힘** (200 ns/40 cycles로 통일). |

---

## 8. Latency / Throughput / Pipeline / II 검증 누적표 (v001 + v002 + v003)

| 항목 | 측정/근거 | 측정값 | 기준 | 결과 |
|---|---|---|---|---|
| `RDN low pulse width` (`div=1,ticks=4 -> clamp 5`) | contract TB [1] | 15 ns | `tPW-RL >= 6 ns` (datasheet p.7) | PASS |
| Burst READ II (`div=1,ticks=5`) | contract TB [2] | 25 ns | datasheet p.27 (40 MHz) | PASS |
| `tPW-RL` 실측 (default `div=2,ticks=5`) | bus_phy TB [13] | 15000 ps | `tPW-RL >= 6 ns` | PASS |
| `tS-AD` 실측 | bus_phy TB [13] | 5000 ps | `tS-AD >= 2 ns` | PASS |
| `o_rsp_pending` set-to-clear latency | contract TB [4] | <= 1 clock | C01 v009 F-C01-08 | PASS |
| EF sync 2-FF latency | bus_phy TB [8] | 2 clock | C01 v009 status sync | PASS |
| AluTrigger 폭 | chip_ctrl TB [8] | 15000 ps | C02 예비 (>= 10 ns) | PASS |
| chip_ctrl drain cap | chip_ctrl TB [4][9b] | 17, 65 (cap=16+flush) | Phase B | PASS |
| CSR clamp (AXI -> CDC -> output) | csr_chip_clamp TB 11 case | (0,4)->(1,5), (1,4)->(1,5), (2,3)->(2,4) 외 | C01 contract | PASS |
| **CSR raw readback (AXI write -> register persist)** | **csr_chip_clamp TB [c11]** | **`fn_pack_bus_timing(8,7) = x"000001C8"` 일치** | **R-C01-V002-02** | **PASS** |
| config_ctrl SYNC raw stream | config_ctrl SYNC mode | direct passthrough | C01 v009 F-C01-02 | PASS |
| config_ctrl ASYNC raw_cdc FIFO | config_ctrl ASYNC mode | xpm_fifo_async 활성 | 동상 | PASS |
| **CDC handshake settling window (AXI -> i_axis_aclk)** | **csr_chip_clamp `c_CDC_WAIT`** | **200 ns = 40 cycles @ 200 MHz** | **R-C01-V002-04** | **PASS** |

추가 정량 검증이 필요한 항목 (F-C01-V03 PH_RESP_DRAIN II, F-C01-V06 잔여 backpressure / IrFlag, F-C01-V02 burst II div>=2)은 C02로 인계.

---

## 9. 산출물 위치 및 다음 단계

### 산출물

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v003.md` | 본 문서 (v003) |
| `HDL/scripts/run_c01_regression.sh` | C01 통합 회귀 entrypoint (Stage 1/2/3) |
| `HDL/scripts/run_all_tbs.tcl` | (수정) 헤더에 `run_c01_regression.sh` 참조 안내 |
| `tb_tdc_gpx_csr_chip_clamp.vhd` | (수정) 헤더 정정, c_CDC_WAIT 주석 정정, [c11] reader case 추가 |
| `HDL/tmp/c01_verify/sim_tb_tdc_gpx_csr_chip_clamp.log` | 12 case 로그 (`*** ALL TESTS PASSED *** (cases=12)`) |
| `HDL/tmp/c01_verify/run_regression.sh` | (변경 없음) Stage 1 |
| `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | (변경 없음) Stage 2 |

### 다음 단계 후보

1. C02 `Chip_Acquisition` 분석 시작 — 인계 항목: F-C01-V03, F-C01-V06 잔여, F-C01-V02 burst II div>=2.
2. 보드 회로도 확인 후 `g_OEN_MODE` default 결정 (F-C01-V04).
3. 상위 laser_ctrl 프로젝트에서 `tb_tdc_gpx_full_int`, `tb_tdc_gpx_top_int` 회귀 (F-C01-V01).

---

## 10. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` p.7 Read Operations, p.8 OEN operations, p.11~12 Pin Description, p.27 28-bit data bus 40 MHz |
| C01 분석 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v009.md` |
| C01 코드 보완 기록 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Update_20260429_v001.md` |
| C01 코드 검증 v001 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v001.md` |
| C01 코드 검증 v002 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v002.md` |
| C01 검증 리뷰 v001 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v001.md` |
| C01 검증 리뷰 v002 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v002.md` |
| 검증 TB | `tb_tdc_gpx_bus_phy_c01_contract.vhd:1-283`, `tb_tdc_gpx_bus_phy.vhd`, `tb_tdc_gpx_chip_ctrl.vhd`, `tb_tdc_gpx_config_ctrl.vhd:18-25,204,323-324`, `tb_tdc_gpx_csr_chip_clamp.vhd:1-345 (v003)` |
| CSR IP source | `tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/{src,sim}/*` |
| 회귀 출력 | `HDL/tmp/c01_verify/sim_*.log`, `HDL/tmp/c01_verify/elab_*.log` |
| 사용자 라이브러리 | `px_utility_pkg.vhd` (AXI-Lite writer/reader procedures) |

---

## 11. 변경 이력

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v002.md` (R-C01-V002-01..04) |
| 직전 버전 | `C01_GPX_Bus_Read_Code_Verify_20260429_v002.md` |
| 본 v003의 핵심 변경 | (i) glbl warning -> allow-list (instance scoping 미지원 확인), (ii) `px_axi_lite_reader` readback case [c11] 추가, (iii) 통합 entrypoint `run_c01_regression.sh` 신설, (iv) `c_CDC_WAIT` 주석 200 ns/40 cycles 통일 |
| 판단 변화 | R-C01-V002-01..04 모두 닫힘. F-C01-V05/V06/V07은 v002 닫힘 상태 유지. v002 보고서의 “2-mode 별도 스크립트” 운영 절차는 v003 통합 entrypoint로 흡수. |
| 추적 근거 | `HDL/tmp/c01_verify/sim_tb_tdc_gpx_csr_chip_clamp.log` (12 cases PASS), `HDL/scripts/run_c01_regression.sh` (`# C01 regression: ALL PASS`), `tb_tdc_gpx_csr_chip_clamp.vhd:7-12`, `tb_tdc_gpx_csr_chip_clamp.vhd:24-26`, `tb_tdc_gpx_csr_chip_clamp.vhd:295-321`, `HDL/scripts/run_all_tbs.tcl:1-15` |

---

## v003 -> v004 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md` (R-C01-V003-01..03) + 사용자 “수정 계획 문서를 왜 규칙에 맞게 생성하지 않은거야?” 지적 + “review 처리 사이클을 규칙으로 업데이트” 요청 |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v004.md` |
| 다음 버전 반영 위치 | section 2.1 (R-C01-V003-01: 종료코드 전파 + Negative test), section 2.2 (R-C01-V003-02: regex 보완 + no-marker FAIL 격상), section 2.3 (R-C01-V003-03: tee transcript 보존), section 5 (운영 프로토콜 v005 -> v006 “Review 처리 사이클 규칙” 도입), section 7 finding 갱신, 본 v004 통합 entrypoint Positive 결과 + Negative test 결과 |
| 판단 변화 | R-C01-V003-01..03 모두 닫힘. R-C01-V002-03(2-mode 분리)도 v003에서 “부분 닫힘” → v004 종료코드 전파 검증으로 “완전 닫힘”. v003 보고서의 “종료 코드 기반 통합 PASS/FAIL 판정” 표현은 v004 Negative test로 신뢰성 확보. v003 보고서의 plan 문서 부재 운영상 결함은 v004에서 retrospective Plan 작성 + 운영 프로토콜 v006 규칙 도입으로 흡수. |
| 추적 근거 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` (Plan), `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v006.md` (Review 처리 사이클 규칙), `tmp/c01_verify/run_regression.sh:95-105,120-124`, `tmp/c01_verify/run_config_ctrl_two_modes.sh:95-99`, `scripts/run_c01_regression.sh:22-29,99`, `tmp/c01_verify/run_c01_regression.log` (positive ALL PASS), Negative test 콘솔 출력 (`stage1=1` exit 1) |
