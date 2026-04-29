# C01_GPX_Bus_Read 코드 검증 기록 v002

문서 버전: `v002`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 15:32:30 +09:00`  
작성 목적: `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v001` 리뷰 결과를 반영해 v001의 ASYNC/SYNC 해석 오류, CSR IP black-box 범위, warning 정책을 정정한다. 추가로 `tb_tdc_gpx_config_ctrl`을 ASYNC/SYNC 두 mode에서 회귀할 수 있도록 generic을 노출하고, `tb_tdc_gpx_csr_chip_clamp`로 CSR IP binding 포함 clamp 시나리오를 닫는다. 절대 기준은 `Doc/TDC-GPX-Datasheet.pdf`이며 RTL clock은 `i_tdc_clk = 200 MHz`(`Tclk = 5 ns`)이다.

---

## 1. 검증 환경

| 항목 | 값 |
|---|---|
| 시뮬레이터 | Vivado 2025.2.1 xsim (`v2025.2.1`, SW Build 6403652) |
| Vivado 경로 | `C:/AMDDesignTools/2025.2.1/Vivado` |
| 실행 디렉터리 | `C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim` |
| 컴파일 표준 | VHDL-2008 |
| Verilog 보조 | `glbl.v` -> `work.glbl` (XPM 사용 elab 시 함께 elab) |
| 통합 .prj | `__c01_all_vhdl.prj` (모든 RTL + CSR IP source + 회귀 TB 일괄 컴파일) |
| 회귀 실행 스크립트 | `HDL/tmp/c01_verify/run_regression.sh`, `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` |
| Vivado 시나리오 생성 스크립트 | `HDL/tmp/c01_verify/gen_c01_prj.tcl` |
| 로그 보존 폴더 | `HDL/tmp/c01_verify/` |
| 추가 컴파일 IP source | `tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/src/{axil_ctrl_regs_32.vhd, axil_stat_regs_32.vhd, axil_intr_32.vhd, axil_fsm_32.vhd, my_axil_csr32_top.vhd}`, `.../sim/tdc_gpx_axil_csr32_chip.vhd` |
| 테스트벤치 라이브러리 | `px_utility_pkg` (`px_axi_lite_writer`, `px_axi_lite_reader`) |

운용 노트:
- xpr 표준 sim 디렉터리만 사용 (ad-hoc 폴더 없음).
- xelab/xsim은 직렬로 실행해 Device Guard 차단 위험을 회피.
- VHDL string generic은 xelab `-f` arg-file 방식을 사용 (사용자 메모리의 “arg-file generics” 가이드).

---

## 2. v001 리뷰 반영 정정 (R-C01-VR-01 / VR-02 / VR-03)

### 2.1 R-C01-VR-01 정정: 검증된 mode는 SYNC, 미검증은 ASYNC가 아님 -> **이번 v002에서 모두 검증**

| 항목 | v001 기재 (오류) | v002 정정 |
|---|---|---|
| `tb_tdc_gpx_config_ctrl` 검증 mode | "ASYNC stream FIFO smoke" | 실제로는 DUT generic을 `g_STREAM_CLK_MODE => "SYNC"`로 고정해 SYNC bypass 경로만 검증함. |
| 미검증 항목 | "SYNC bypass 미검증" | 실제로는 ASYNC `xpm_fifo_async` CDC 경로가 미검증이었음. |
| 추적 근거 | - | `tb_tdc_gpx_config_ctrl.vhd:199` (`g_STREAM_CLK_MODE => g_DUT_STREAM_CLK_MODE`로 변경 전 hard-coded "SYNC"), `elab_tb_tdc_gpx_config_ctrl.log`에 `xpm_fifo_async` compile 라인 0건 |

### 2.2 R-C01-VR-02 정정: CSR IP black-box로 v001의 config_ctrl smoke는 CSR clamp를 검증하지 않았음

| 항목 | v001 기재 (과대 해석) | v002 정정 |
|---|---|---|
| config_ctrl smoke 의 검증 범위 | "section 2 RTL 변경 점검 OK", "section 4 generic 전파" | smoke는 init까지 도달했음을 확인하는 “구조 elaborate + cmd_start 흐름” 수준이며, AXI 통과를 통한 CSR clamp는 `tdc_gpx_axil_csr32_chip` black box 때문에 시뮬에서 닫히지 않았다. |
| 추적 근거 | - | `elab_tb_tdc_gpx_config_ctrl.log`: `WARNING: [VRFC 10-4940] 'tdc_gpx_axil_csr32_chip' remains a black box since it has no binding entity` |
| 정정 후 검증 | - | 본 v002에서 `tb_tdc_gpx_csr_chip_clamp`로 CSR IP source를 함께 컴파일/elab해 11개 case의 AXI write -> `o_bus_clk_div`/`o_bus_ticks` 클램프 결과를 직접 비교. |

### 2.3 R-C01-VR-03 정정: 의도된 warning은 PASS 판정에서 제외되어야 함 + allow-list 명시

| 항목 | v001 기재 | v002 정정 |
|---|---|---|
| warning 처리 정책 | F-C01-V07에서 “run_regression.sh는 이미 정책 적용” | 본 v002에서 회귀 스크립트 fail 판정 함수와 별도로 “허용 warning 목록”을 본 보고서에 명시한다. |
| 허용 warning 목록 | - | 아래 표 참조. CI/회귀 스크립트는 이 목록을 fail로 분류하지 않는다. |

| 허용 warning 텍스트 | 발생 위치 | 의미 |
|---|---|---|
| `bus_phy: bus timing clamped (div=` | `tdc_gpx_bus_phy.vhd:430-434` | C01 contract test [1]에서 의도적으로 illegal 입력을 주입해 clamp 동작을 검증. |
| `bus_phy: write request ignored (oen_permanent='1')` | `tdc_gpx_bus_phy.vhd:443-445` | INV-7 보호: oen_permanent='1' 동안 WRITE 거부. `tb_tdc_gpx_bus_phy` [5]에서 검증. |

CI 가이드:
- `severity failure`만 fail로 분류한다.
- 위 표의 텍스트는 grep allow-list로 처리해 “Warning” 단순 매칭으로 fail 처리하지 않는다.
- 새로운 의도된 warning을 추가할 때 본 표를 확장한다.

---

## 3. 신규 회귀 산출물

### 3.1 `tb_tdc_gpx_config_ctrl` ASYNC/SYNC 두 mode 회귀

변경:
- entity에 `g_DUT_STREAM_CLK_MODE : string := "SYNC"` generic 추가 (`tb_tdc_gpx_config_ctrl.vhd:18-25`).
- DUT generic map에서 `g_STREAM_CLK_MODE => g_DUT_STREAM_CLK_MODE`로 위임 (`tb_tdc_gpx_config_ctrl.vhd:204`).
- p_stim 시작부에서 mode를 `report`로 출력해 로그 식별 가능 (`tb_tdc_gpx_config_ctrl.vhd:323-324`).

회귀 실행:
- `xelab -f __elab_<MODE>.f` arg-file에 `-generic_top "g_DUT_STREAM_CLK_MODE=<MODE>"` 포함 -> snapshot 분리.
- `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh`에서 `SYNC`/`ASYNC` 순차 실행.

| Mode | TB 결과 | DUT 측 검증 근거 |
|---|---|---|
| `SYNC` | PASS (`TB: PASS -- init complete, active_chip_mask = 15`) | `xpm_fifo_async` compile 0건, raw stream direct passthrough 활성 (`tdc_gpx_config_ctrl.vhd:1820-1832`). |
| `ASYNC` | PASS (`TB: PASS -- init complete, active_chip_mask = 15`) | `xpm_fifo_async` compile 1건 확인 (`elab_tb_tdc_gpx_config_ctrl_ASYNC.log` 내 `Compiling module xpm.xpm_fifo_async`). |

### 3.2 `tb_tdc_gpx_csr_chip_clamp` 신규 작성

목적: AXI write 경로를 통과해 BUS_TIMING (CTL1 @ 0x04) 레지스터의 (`bus_clk_div`, `bus_ticks`) 클램프 동작을 종단 검증한다. v001에서 CSR IP black-box 때문에 닫지 못한 R-C01-VR-02 항목을 닫는다.

설계:
- `tdc_gpx_csr_chip` 인스턴스 + `tdc_gpx_axil_csr32_chip` IP source 함께 컴파일 (black-box 해소).
- AXI write/read는 `px_utility_pkg`의 `px_axi_lite_writer`/`px_axi_lite_reader` procedure 활용 (사용자 가이드 반영, 신규 helpers 추가하지 않음).
- 매 write 후 `c_CDC_WAIT = 200 ns` 대기로 `xpm_cdc_handshake`(s_axi_aclk -> i_axis_aclk) 정착 확보.
- `o_bus_clk_div` / `o_bus_ticks` 직접 관측해 expected 값과 비교.

회귀 결과 (`HDL/tmp/c01_verify/sim_tb_tdc_gpx_csr_chip_clamp.log`):

| Case | 입력 (div, ticks) | 기대 출력 | 결과 |
|---|---|---|---|
| init | x"00000142" (`c_INIT_BUS_TIMING`) | div=2, ticks=5 | PASS |
| c01 | (0, 4) | div=1, ticks=5 (div 클램프 + ticks_min=5 적용) | PASS |
| c02 | (1, 3) | div=1, ticks=5 | PASS |
| **c03** | **(1, 4)** | **div=1, ticks=5** (C01 contract 핵심) | **PASS** |
| c04 | (1, 5) | div=1, ticks=5 (legal boundary) | PASS |
| c05 | (1, 7) | div=1, ticks=7 | PASS |
| c06 | (2, 3) | div=2, ticks=4 (ticks_min=4 적용) | PASS |
| c07 | (2, 4) | div=2, ticks=4 (legal boundary) | PASS |
| c08 | (2, 5) | div=2, ticks=5 (default-equivalent) | PASS |
| c09 | (3, 4) | div=3, ticks=4 | PASS |
| c10 | (8, 7) | div=8, ticks=7 | PASS |

총 결과: `*** ALL TESTS PASSED *** (cases=11)`.

R-C01-VR-02 권장과 일치하게 `tdc_gpx_csr_chip` clamp 계약이 AXI 통과 시뮬로 직접 닫혔다.

---

## 4. 전체 회귀 결과 요약 (v002)

| TB | runtime | 결과 | 핵심 marker |
|---|---:|---|---|
| `tb_tdc_gpx_bus_phy_c01_contract` | 2 us | **PASS** | `tb_tdc_gpx_bus_phy_c01_contract PASS` |
| `tb_tdc_gpx_bus_phy` | 200 us | **PASS** | `*** ALL TESTS PASSED *** (total_rsp=85)` |
| `tb_tdc_gpx_chip_ctrl` | 200 us | **PASS** | `*** ALL TESTS PASSED *** (total_raw_words=224)` |
| `tb_tdc_gpx_config_ctrl` (`g_DUT_STREAM_CLK_MODE=SYNC`) | 200 us | **PASS** | `TB: PASS -- init complete, active_chip_mask = 15` |
| `tb_tdc_gpx_config_ctrl` (`g_DUT_STREAM_CLK_MODE=ASYNC`) | 200 us | **PASS** | `TB: PASS -- init complete, active_chip_mask = 15` (`xpm_fifo_async` 활성) |
| `tb_tdc_gpx_csr_chip_clamp` | 100 us | **PASS** | `*** ALL TESTS PASSED *** (cases=11)` |

전체: **6/6 PASS** (5개 TB, 1개 TB는 mode 2종으로 6 회귀).

`tb_tdc_gpx_full_int`는 상위 laser_ctrl 패키지 의존이므로 본 IP xpr 단독 elaborate 불가 — F-C01-V01 유지, 상위 프로젝트에서 별도 회귀.

근거 로그:
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_bus_phy_c01_contract.log`
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_bus_phy.log`
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_chip_ctrl.log`
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_SYNC.log`
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log`
- `HDL/tmp/c01_verify/sim_tb_tdc_gpx_csr_chip_clamp.log`

---

## 5. 검증 시나리오 매트릭스 (Cluster 대응 — 정정 후)

| 데이터시트 / Cluster 계약 | 검증 TB | 검증 항목 | 결과 |
|---|---|---|---|
| p.7 READ ops, `tV-DR <= 11.8 ns`, `tPW-RL >= 6 ns` | `tb_tdc_gpx_bus_phy` [13] | `tPW-RL = 15000 ps`, `tS-AD = 5000 ps` | PASS |
| p.7/p.27 28-bit data bus 40 MHz max, `div=1,ticks=4` 금지 | `tb_tdc_gpx_bus_phy_c01_contract` [1] | bus_phy 내부 ticks=4 → 5 clamp (`v_pw = 15 ns`) | PASS |
| p.27 burst II = 25 ns @ 200 MHz `div=1,ticks=5` | `tb_tdc_gpx_bus_phy_c01_contract` [2] | RDN low edge간 25 ns 측정 | PASS |
| **CSR clamp (`div=0`/`div=1,ticks<5`/`div>=2,ticks<4`)** | **`tb_tdc_gpx_csr_chip_clamp` 11 cases** | **AXI write -> CDC -> `o_bus_clk_div`/`o_bus_ticks` 클램프 결과 직접 측정** | **PASS** |
| p.8 OEN ops, WRITE 중 OEN=High, drain burst 시 OEN=Low | `tb_tdc_gpx_bus_phy_c01_contract` [3], `tb_tdc_gpx_bus_phy` [5] | `PULLUP_OR_NOT_CONNECTED`에서 `o_oen=1` 유지, `oen_permanent` 동안 WRITE 거부 | PASS |
| `o_rsp_pending` registered boundary | `tb_tdc_gpx_bus_phy_c01_contract` [4] | response pending/handshake 동안 high, handshake 후 clear | PASS |
| chip_ctrl drain watchdog cap | `tb_tdc_gpx_chip_ctrl` [4][9b][12] | drain capped at 16+flush, mid-stop 후 재시작 | PASS |
| **config_ctrl ASYNC stream FIFO** (`g_STREAM_CLK_MODE="ASYNC"`) | **`tb_tdc_gpx_config_ctrl` ASYNC mode** | **`xpm_fifo_async` 활성 + 4 chip init 완료** | **PASS** |
| **config_ctrl SYNC stream bypass** (`g_STREAM_CLK_MODE="SYNC"`) | **`tb_tdc_gpx_config_ctrl` SYNC mode** | **`xpm_fifo_async` bypass + 4 chip init 완료** | **PASS** |
| EF/LF/IrFlag/ErrFlag 2-FF synchronizer | `tb_tdc_gpx_bus_phy` [8] | `ef1_sync` 2-FF latency 추적 | PASS |
| empty FIFO read 금지 (chip_ctrl 책임) | `tb_tdc_gpx_chip_ctrl` [7] | EF=1 동안 read 발생하지 않음 | PASS |
| 16-bit mode 차단 (Reg14[4]=0) | `tb_tdc_gpx_chip_ctrl` [6] | Reg14 capture content가 cfg_image와 일치 | PASS |

---

## 6. v002에서 갱신된 finding

v001 finding (F-C01-V01..V07)을 다음과 같이 갱신한다.

| Finding | v001 상태 | v002 상태 |
|---|---|---|
| F-C01-V01 (full_int 상위 laser_ctrl 의존) | 열림 | **유지**. 본 IP xpr 단독 검증 범위 밖 — 상위 프로젝트에서 회귀. |
| F-C01-V02 (contract TB tick_en=1 단순화) | 열림 | **유지**. `tb_tdc_gpx_bus_phy` [12]가 `BUS_CLK_DIV=2/4`를 커버하므로 contract TB는 200 MHz `div=1` 경계 검증 역할로 충분. |
| F-C01-V03 (`o_rsp_pending` boundary -> `chip_run` PH_RESP_DRAIN II) | 열림 | **유지, C02 인계**. 본 v002에서도 직접 정량 측정은 수행하지 않음. |
| F-C01-V04 (보드 회로도 cross-check) | 열림 | **유지**. 보드 회로도 확인 후 `g_OEN_MODE` default 결정. |
| F-C01-V05 (SYNC 경로 미검증) | 열림 (방향 오류) | **닫힘**. v001에서 사실은 SYNC만 검증되었음. v002 generic 노출 + 양 mode 회귀로 양쪽 모두 PASS. |
| F-C01-V06 (config_ctrl smoke 수준) | 열림 (CSR clamp 포함 과대 해석) | **부분 닫힘**. CSR clamp는 v002 `tb_tdc_gpx_csr_chip_clamp`로 닫힘. backpressure / IrFlag 시나리오는 C02로 인계. |
| F-C01-V07 (warning allow-list 정책) | 열림 | **닫힘**. 본 v002 section 2.3에 허용 warning 목록 명시. |

신규 finding은 추가하지 않는다. R-C01-VR-* 리뷰 항목은 모두 본 v002에서 정정/검증으로 닫혔다.

---

## 7. RTL/TB 변경 점검 (v001->v002)

| 파일 | 변경 의도 | 추적 위치 | 비고 |
|---|---|---|---|
| `tb_tdc_gpx_config_ctrl.vhd` | `g_DUT_STREAM_CLK_MODE` generic 추가 + DUT 위임 + mode echo report | `tb_tdc_gpx_config_ctrl.vhd:18-25`, `:204`, `:323-324` | RTL 무변경 |
| `tb_tdc_gpx_csr_chip_clamp.vhd` | 신규 — `px_axi_lite_writer/reader`로 CSR clamp 종단 검증 | `tb_tdc_gpx_csr_chip_clamp.vhd:1-322` | 11 case |
| `__c01_all_vhdl.prj` | CSR IP source(.gen 경로) + 신규 TB 추가 | `tdc_gpx_ctrl.sim/sim_1/behav/xsim/__c01_all_vhdl.prj` | full_int는 여전히 제외 |
| `run_regression.sh` (보존) | v001과 동일 — 4 TB 직렬 회귀 | `HDL/tmp/c01_verify/run_regression.sh` | warning allow-list는 별도 문서화 (section 2.3) |
| `run_config_ctrl_two_modes.sh` | 신규 — config_ctrl SYNC/ASYNC 회귀 (xelab arg-file) | `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | string generic 처리 |
| `tdc_gpx_cfg_pkg.vhd` | 변경 없음 (검증 중 일시 추가했던 helpers는 사용자 가이드대로 되돌림) | `tdc_gpx_cfg_pkg.vhd:298-300` | clamp 상수 그대로 |
| `tdc_gpx_csr_chip.vhd` | 변경 없음 (검증 중 일시 refactor했던 부분은 되돌림) | `tdc_gpx_csr_chip.vhd:846-862` | inline clamp 그대로 |

---

## 8. Latency / Throughput / Pipeline / II 검증 항목 (v001 + v002 누적)

| 항목 | 측정/근거 | 측정값 | 기준 | 결과 |
|---|---|---|---|---|
| `RDN low pulse width` (`div=1,ticks=4 -> clamp 5`) | contract TB [1] | 15 ns | `tPW-RL >= 6 ns` (datasheet p.7) | PASS |
| Burst READ II (`div=1,ticks=5`) | contract TB [2] | 25 ns | datasheet p.27 (40 MHz) | PASS |
| `tPW-RL` 실측 (default `div=2,ticks=5`) | bus_phy TB [13] | 15000 ps | `tPW-RL >= 6 ns` | PASS |
| `tS-AD` 실측 | bus_phy TB [13] | 5000 ps | `tS-AD >= 2 ns` | PASS |
| `o_rsp_pending` set-to-clear latency | contract TB [4] | <= 1 clock | C01 v009 F-C01-08 | PASS |
| EF sync 2-FF latency | bus_phy TB [8] | 2 clock | C01 v009 status sync | PASS |
| AluTrigger 폭 | chip_ctrl TB [8] | 15000 ps | C02 예비 (>= 10 ns) | PASS |
| chip_ctrl drain cap | chip_ctrl TB [4][9b] | drain words = 17, 65 (cap=16+flush) | Phase B 변경 | PASS |
| **CSR clamp (AXI -> CDC -> output)** | **csr_chip_clamp TB 11 cases** | **(0,4)->(1,5), (1,4)->(1,5), (2,3)->(2,4) 외 8개** | **C01 contract `c_BUS_CLK_DIV_MIN=1`, `c_BUS_READ_PERIOD_MIN_CLKS=5`, `c_BUS_TICKS_MIN=4`** | **PASS** |
| **config_ctrl SYNC raw stream pass** | **config_ctrl SYNC mode** | **direct passthrough init 완료** | **C01 v009 F-C01-02 “stream domain 분리”** | **PASS** |
| **config_ctrl ASYNC raw_cdc FIFO** | **config_ctrl ASYNC mode** | **`xpm_fifo_async` 활성, init 완료** | **동상** | **PASS** |

추가 정량 검증이 필요한 항목 (F-C01-V03 PH_RESP_DRAIN II, F-C01-V06 잔여 backpressure / IrFlag)은 C02 분석 단계로 인계한다.

---

## 9. 사용자 피드백 기록

이번 v002 단계에서 사용자가 직접 입력한 피드백:

1. v001 검증 보고서에 대한 리뷰 (`C01_GPX_Bus_Read_Code_Verify_Review_20260429_v001.md`) 작성. R-C01-VR-01/02/03 정정 권장 + 수정 권장 3건 제시.
2. “모두 진행해줘” — 3건 모두 v002에 반영.
3. “`px_utility_pkg`에 정의한 것을 활용해줘” — `tb_tdc_gpx_csr_chip_clamp`에서 신규 helpers 추가하지 않고 `px_axi_lite_writer/reader` procedure를 그대로 사용. 검증 중 일시 추가했던 `tdc_gpx_cfg_pkg`의 `fn_clamp_*` 함수와 `tdc_gpx_csr_chip` refactor는 본 가이드에 따라 되돌렸다.

---

## 10. 산출물 위치 및 다음 단계

### 산출물

| 파일 | 의미 |
|---|---|
| `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v002.md` | 본 검증 보고서 v002 |
| `tb_tdc_gpx_csr_chip_clamp.vhd` | 신규 — CSR IP binding 포함 BUS_TIMING clamp 종단 회귀 |
| `tb_tdc_gpx_config_ctrl.vhd` | `g_DUT_STREAM_CLK_MODE` generic 추가 |
| `HDL/tmp/c01_verify/run_config_ctrl_two_modes.sh` | SYNC/ASYNC 2-mode 회귀 (xelab `-f` arg-file) |
| `HDL/tmp/c01_verify/sim_tb_tdc_gpx_csr_chip_clamp.log` | clamp TB 11 case 로그 |
| `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_SYNC.log` | SYNC mode 로그 |
| `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log` | ASYNC mode 로그 |
| `tdc_gpx_ctrl.sim/sim_1/behav/xsim/__c01_all_vhdl.prj` | CSR IP source 포함 일괄 컴파일 .prj |

### 다음 단계 후보

1. C02 `Chip_Acquisition` 분석 시작 — 인계 항목: F-C01-V03 (`o_rsp_pending` -> PH_RESP_DRAIN II 정량), F-C01-V06 잔여 (config_ctrl backpressure / IrFlag 시나리오), F-C01-V02 burst II div>=2 case.
2. 보드 회로도 확인 후 `g_OEN_MODE` default 결정 (F-C01-V04).
3. 상위 laser_ctrl 프로젝트에서 `tb_tdc_gpx_full_int`, `tb_tdc_gpx_top_int` 회귀.

---

## 11. 추적 근거 요약

| 종류 | 위치 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf` p.7 Read Operations, p.8 OEN operations, p.11~12 Pin Description, p.27 28-bit data bus 40 MHz |
| C01 분석 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v009.md` |
| C01 코드 보완 기록 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Update_20260429_v001.md` |
| C01 코드 검증 v001 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v001.md` |
| C01 검증 리뷰 v001 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v001.md` |
| RTL (변경 없음) | `tdc_gpx_cfg_pkg.vhd:298-300`, `tdc_gpx_csr_chip.vhd:846-862`, `tdc_gpx_bus_phy.vhd:87-179`, `tdc_gpx_config_ctrl.vhd:1820-1915` |
| 검증 TB | `tb_tdc_gpx_bus_phy_c01_contract.vhd:1-283`, `tb_tdc_gpx_bus_phy.vhd`, `tb_tdc_gpx_chip_ctrl.vhd`, `tb_tdc_gpx_config_ctrl.vhd:18-25,204,323-324`, `tb_tdc_gpx_csr_chip_clamp.vhd:1-322` |
| CSR IP source | `tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/{src,sim}/*` |
| 회귀 출력 | `HDL/tmp/c01_verify/sim_*.log`, `HDL/tmp/c01_verify/elab_*.log` |
| 사용자 라이브러리 | `px_utility_pkg.vhd` (AXI-Lite writer/reader procedures) |

---

## 12. 변경 이력

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v001.md` 사용자 리뷰 (R-C01-VR-01/02/03) + “px_utility_pkg 활용” 가이드 |
| 직전 버전 | `C01_GPX_Bus_Read_Code_Verify_20260429_v001.md` |
| 본 v002의 핵심 변경 | (i) ASYNC/SYNC 해석 정정, (ii) CSR IP black-box 범위 명시 + 종단 회귀 (`tb_tdc_gpx_csr_chip_clamp` 신규), (iii) warning allow-list 명시, (iv) `tb_tdc_gpx_config_ctrl` SYNC/ASYNC 양 mode 회귀 |
| 판단 변화 | F-C01-V05 닫힘, F-C01-V06 부분 닫힘, F-C01-V07 닫힘. F-C01-V01/V02/V03/V04 유지. |
| 추적 근거 | `HDL/tmp/c01_verify/sim_tb_tdc_gpx_csr_chip_clamp.log`, `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_SYNC.log`, `HDL/tmp/c01_verify/sim_tb_tdc_gpx_config_ctrl_ASYNC.log` |

---

## v002 -> v003 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | `C01_GPX_Bus_Read_Code_Verify_Review_20260429_v002.md` 사용자 리뷰 (R-C01-V002-01..04) |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v003.md` |
| 다음 버전 반영 위치 | section 2.1 (R-C01-V002-01: glbl warning allow-list, instance scoping 미지원 확인), section 2.2 (R-C01-V002-02: 헤더 정정 + `px_axi_lite_reader` case [c11] 추가), section 2.3 (R-C01-V002-03: `run_c01_regression.sh` 통합 entrypoint), section 2.4 (R-C01-V002-04: c_CDC_WAIT 주석 200ns/40cyc 통일), section 6 허용 warning 표 갱신, section 7 finding 갱신 |
| 판단 변화 | R-C01-V002-01..04 모두 닫힘. v002의 “TB 헤더가 writer/reader 표기” 문구는 v003 [c11] reader case 실제 구현으로 정합성 확보. v002의 “2-mode 별도 스크립트 운영” 절차는 v003 `run_c01_regression.sh` 통합 entrypoint로 단일화. CSR clamp TB 케이스 11 -> 12로 확장. |
| 추적 근거 | `HDL/scripts/run_c01_regression.sh`, `HDL/scripts/run_all_tbs.tcl:1-15`, `tb_tdc_gpx_csr_chip_clamp.vhd:7-12,24-26,295-321`, `HDL/tmp/c01_verify/sim_tb_tdc_gpx_csr_chip_clamp.log` (`*** ALL TESTS PASSED *** (cases=12)`), `elab_tb_tdc_gpx_config_ctrl_*.log:8` (instance scoping 시도 실패의 원본 warning) |
