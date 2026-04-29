# C01_GPX_Bus_Read 코드 보완 기록 v001

최종 수정 시간: 2026-04-29 13:48:30 +09:00

## 목적

`C01_GPX_Bus_Read_20260429_v009`까지 합의한 분석 내용을 RTL과 테스트벤치에 반영한다.
절대 기준은 `Doc/TDC-GPX-Datasheet.pdf`이며, 특히 READ/WRITE/OEN timing과 28-bit data bus 40 MHz transfer limit을 코드 계약으로 닫는 것이 목적이다.

## 반영 기준

| 기준 | 적용 내용 | 추적 근거 |
|---|---|---|
| Datasheet READ timing | 200 MHz에서 `div=1,ticks=5`를 최단 합법 READ/Burst II로 둔다. | `Doc/TDC-GPX-Datasheet.pdf` p.7 Read Operations, p.27 28-bit transfer rate |
| Datasheet OEN operation | OEN 정상 연결과 Pull-up/미연결 운용을 generic으로 분리한다. Low 고정은 지원하지 않는다. | `Doc/TDC-GPX-Datasheet.pdf` p.8 OEN operations, p.11~12 OEN pin |
| C01 timing 결론 | `div=1,ticks=4`는 capture margin과 40 MHz readout limit 관점에서 illegal이므로 clamp한다. | `C01_GPX_Bus_Read_20260429_v009.md` F-C01-01 |
| FF 경계 원칙 | `o_rsp_pending`은 조합 OR 대신 등록 출력으로 만든다. | `C01_GPX_Bus_Read_20260429_v009.md` F-C01-08 |

## RTL 변경 요약

| 파일 | 변경 내용 | 추적 위치 |
|---|---|---|
| `tdc_gpx_cfg_pkg.vhd` | `c_BUS_CLK_DIV_MIN=1`, `c_BUS_READ_PERIOD_MIN_CLKS=5` 추가 | `tdc_gpx_cfg_pkg.vhd:299-300` |
| `tdc_gpx_csr_chip.vhd` | CSR clamp를 `div=1 => ticks>=5`, `div>=2 => ticks>=4`로 변경 | `tdc_gpx_csr_chip.vhd:847-862` |
| `tdc_gpx_bus_phy.vhd` | `g_OEN_MODE`, `g_BUS_READ_PERIOD_MIN_CLKS`, `i_bus_clk_div` 추가 | `tdc_gpx_bus_phy.vhd:89-101` |
| `tdc_gpx_bus_phy.vhd` | `fn_min_ticks_for_div()`로 bus_phy 내부 local clamp 추가 | `tdc_gpx_bus_phy.vhd:166-179` |
| `tdc_gpx_bus_phy.vhd` | OEN 모드 `DYNAMIC_CONNECTED`, `PULLUP_OR_NOT_CONNECTED` 지원 | `tdc_gpx_bus_phy.vhd:163-164`, `tdc_gpx_bus_phy.vhd:403-516` |
| `tdc_gpx_bus_phy.vhd` | `o_rsp_pending` 등록 출력화 | `tdc_gpx_bus_phy.vhd:227`, `tdc_gpx_bus_phy.vhd:712` |
| `tdc_gpx_chip_ctrl.vhd` | bus_phy가 실제 transaction snapshot의 `bus_clk_div`를 볼 수 있도록 `o_bus_clk_div_snap` 추가 | `tdc_gpx_chip_ctrl.vhd:151`, `tdc_gpx_chip_ctrl.vhd:993` |
| `tdc_gpx_config_ctrl.vhd` | top generic과 chip snapshot을 bus_phy로 전달 | `tdc_gpx_config_ctrl.vhd:1451-1459`, `tdc_gpx_config_ctrl.vhd:1765-1766` |
| `tdc_gpx_config_ctrl.vhd` | Stream clock 운용을 `g_STREAM_CLK_MODE=ASYNC/SYNC`로 선택 가능하게 하고, ASYNC는 기존 `xpm_fifo_async`, SYNC는 FIFO bypass로 처리 | `tdc_gpx_config_ctrl.vhd:67`, `tdc_gpx_config_ctrl.vhd:699-700`, `tdc_gpx_config_ctrl.vhd:1820-1914` |
| `tdc_gpx_top.vhd` | OEN/timing/Stream clock generic을 top에서 설정 가능하도록 노출 | `tdc_gpx_top.vhd:40-42`, `tdc_gpx_top.vhd:458-461` |

## 테스트벤치 변경 요약

| 파일 | 검증 항목 | 추적 위치 |
|---|---|---|
| `tb_tdc_gpx_bus_phy_c01_contract.vhd` | `div=1,ticks=4` 요청이 내부에서 `ticks=5` 타이밍으로 clamp되는지 확인 | `tb_tdc_gpx_bus_phy_c01_contract.vhd:214-224` |
| `tb_tdc_gpx_bus_phy_c01_contract.vhd` | `div=1,ticks=5` burst READ II가 25 ns인지 확인 | `tb_tdc_gpx_bus_phy_c01_contract.vhd:241-252` |
| `tb_tdc_gpx_bus_phy_c01_contract.vhd` | `PULLUP_OR_NOT_CONNECTED`에서 `o_oen=1` 유지와 RDN-gated read 완료 확인 | `tb_tdc_gpx_bus_phy_c01_contract.vhd:259-272` |
| `tb_tdc_gpx_bus_phy_c01_contract.vhd` | `o_rsp_pending` 등록 출력이 response pending/handshake 동안 유지되고 clear되는지 확인 | `tb_tdc_gpx_bus_phy_c01_contract.vhd:227-238` |
| `tb_tdc_gpx_bus_phy.vhd` | 기존 bus_phy TB에 신규 `i_bus_clk_div` port 연결 | `tb_tdc_gpx_bus_phy.vhd:184-190` |
| `tb_tdc_gpx_chip_ctrl.vhd` | chip_ctrl integration TB에서 `bus_clk_div`와 `rsp_pending` 경로 연결 | `tb_tdc_gpx_chip_ctrl.vhd:112`, `tb_tdc_gpx_chip_ctrl.vhd:262`, `tb_tdc_gpx_chip_ctrl.vhd:311` |
| `scripts/run_all_tbs.tcl` | C01 계약 TB를 Vivado 일괄 TB 목록에 추가 | `scripts/run_all_tbs.tcl:12-29` |

## 남은 영향성

1. `o_rsp_pending` 등록 출력화가 `chip_run`의 `PH_RESP_DRAIN` 탈출 조건과 non-burst II에 주는 영향은 C02에서 별도 확인한다.
2. `PULLUP_OR_NOT_CONNECTED`는 `oen_permanent` 의존을 제거하는 운용 모드이다. 실제 회로가 Datasheet의 OEN-high/RDN-strobed read 조건을 만족하는지는 보드 회로도와 함께 확인해야 한다.
3. 250 MHz 또는 다른 control clock 후보로 변경할 경우 `c_BUS_READ_PERIOD_MIN_CLKS`, timeout 상수, TB clock period를 함께 재검토한다.

## 검증 상태

현재 로컬 PATH에서 `xvhdl`, `xelab`, `ghdl`, `vivado`를 찾지 못해 시뮬레이션은 실행하지 못했다.
대신 포트 연결 누락 검색, `git diff --check`, TB 목록 확인을 수행했다.
`git diff --check`는 이번 변경과 무관하게 기존 수정 파일 `tb_tdc_gpx_full_int.vhd`의 trailing whitespace를 보고했다.

---

## v001 -> Code_Verify v001 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | Vivado 2025.2.1 xsim 환경(`C:/AMDDesignTools/2025.2.1/Vivado`)에서 본 v001의 RTL/TB 변경을 실측 회귀해 닫을 필요. |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v001.md` |
| 다음 버전 반영 위치 | section 1 검증 환경 (xpr `sim_1/behav/xsim` 사용), section 2 RTL 변경 점검 (모든 변경 위치 OK), section 4 시뮬레이션 결과 요약 (4 TB pass: contract, bus_phy 23 PASS, chip_ctrl 46 PASS, config_ctrl smoke), section 6 finding F-C01-V01~V07. |
| 판단 변화 | v001 “남은 영향성 1” (`o_rsp_pending` boundary 영향) -> Code_Verify v001 F-C01-V03로 보류. v001 “남은 영향성 2” (보드 OEN) -> F-C01-V04로 보류. v001 “남은 영향성 3” (250 MHz 등 다른 control clock) -> 본 회귀 범위에서 200 MHz 기준만 닫음, 다른 후보는 C02 이후로 인계. |
| 추적 근거 | `HDL/tmp/c01_verify/sim_*.log`, `HDL/tmp/c01_verify/run_regression.sh`, `HDL/tmp/c01_verify/gen_c01_prj.tcl`, `tdc_gpx_ctrl.sim/sim_1/behav/xsim/__c01_all_vhdl.prj` |
