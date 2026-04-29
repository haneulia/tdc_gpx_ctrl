# C01_GPX_Bus_Read 코드 검증 보고서 리뷰 v001

최종 수정 시간: 2026-04-29 15:09:35 +09:00

## 검토 대상

| 항목 | 위치 |
|---|---|
| 검증 보고서 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v001.md` |
| xsim 로그 | `tmp/c01_verify/elab_*.log`, `tmp/c01_verify/sim_*.log` |
| 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| 기준 RTL clock | `i_tdc_clk = 200 MHz`, `Tclk = 5 ns` |

## 총평

검증 로그 기준으로 C01의 핵심 bus_phy 계약은 통과했다.

- `tb_tdc_gpx_bus_phy_c01_contract`: PASS
- `tb_tdc_gpx_bus_phy`: PASS
- `tb_tdc_gpx_chip_ctrl`: PASS
- `tb_tdc_gpx_config_ctrl`: PASS

특히 `div=1,ticks=4` 요청을 `ticks=5`로 clamp하는 보호 동작, `div=1,ticks=5` burst READ II 25 ns, OEN pull-up/NC 모드의 `o_oen=1` 유지, registered `o_rsp_pending` 동작은 C01 범위에서 의미 있게 닫혔다.

다만 보고서 해석에는 아래 정정 사항이 있다.

## 리뷰 Finding

### R-C01-VR-01. `tb_tdc_gpx_config_ctrl` smoke는 ASYNC가 아니라 SYNC 경로를 검증했다

| 항목 | 내용 |
|---|---|
| 심각도 | 중간 |
| 보고서 위치 | `C01_GPX_Bus_Read_Code_Verify_20260429_v001.md:74`, `:101`, `:156-165` |
| RTL/TB 근거 | `tb_tdc_gpx_config_ctrl.vhd:197-200`, `tdc_gpx_config_ctrl.vhd:1820-1915` |
| 로그 근거 | `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl.log`에 `xpm_fifo_async` compile 기록이 없음 |
| 판단 | 현재 `tb_tdc_gpx_config_ctrl`는 DUT generic을 `g_STREAM_CLK_MODE => "SYNC"`로 고정한다. 따라서 보고서의 “4 chip ASYNC stream FIFO PASS”, “default ASYNC만 검증했고 SYNC 미검증” 문장은 반대로 정정되어야 한다. |
| 권장 정정 | 검증 결과를 “SYNC smoke PASS, ASYNC raw_cdc FIFO path 미검증”으로 바꾼다. |
| 후속 | `tb_tdc_gpx_config_ctrl`에 top generic을 두고 ASYNC/SYNC 두 mode를 각각 회귀할 수 있게 만든다. |

### R-C01-VR-02. `tdc_gpx_axil_csr32_chip` black box 때문에 config_ctrl smoke는 CSR clamp를 시뮬레이션으로 검증하지 못했다

| 항목 | 내용 |
|---|---|
| 심각도 | 중간 |
| 보고서 위치 | `C01_GPX_Bus_Read_Code_Verify_20260429_v001.md:44-46`, `:167-176` |
| 로그 근거 | `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl.log`: `tdc_gpx_axil_csr32_chip remains a black box since it has no binding entity` |
| 판단 | `tdc_gpx_cfg_pkg.vhd`와 `tdc_gpx_csr_chip.vhd`의 clamp 변경은 정적 파일 점검으로는 OK이나, `tb_tdc_gpx_config_ctrl` smoke가 CSR IP 내부 동작까지 검증했다고 보면 안 된다. |
| 권장 정정 | 보고서의 config_ctrl 검증 범위를 “init smoke + 구조 elaborate”로 제한하고, CSR clamp 검증은 `tdc_gpx_csr_chip` 단위 TB 또는 CSR IP 포함 xpr에서 별도 수행한다고 명시한다. |
| 후속 | C02 전 또는 C02 중 `bus_clk_div=1,ticks=4/5` CSR write-read 시나리오를 추가한다. |

### R-C01-VR-03. 의도된 warning은 PASS 판정에서 제외하는 정책이 필요하다

| 항목 | 내용 |
|---|---|
| 심각도 | 낮음 |
| 로그 근거 | `sim_tb_tdc_gpx_bus_phy_c01_contract.log`: timing clamp warning, `sim_tb_tdc_gpx_bus_phy.log`: write ignored / timing clamp warning |
| 판단 | 해당 warning은 보호 정책을 검증하기 위해 일부러 발생시킨 것이다. 회귀 스크립트가 warning만으로 fail 처리하면 안 된다. |
| 권장 정정 | 보고서의 F-C01-V07 판단은 타당하다. 단, 허용 warning 목록을 회귀 스크립트 또는 문서에 명시해 CI 오판정을 방지한다. |

## 닫힌 항목

| 항목 | 판단 |
|---|---|
| C01 contract TB | `div=1,ticks=4` clamp, `div=1,ticks=5` 25 ns II, OEN pull-up/NC, registered pending PASS |
| bus_phy unit TB | READ/WRITE, turnaround, burst, status sync, timing assertion PASS |
| chip_ctrl integration TB | drain, soft reset, stop, watchdog, register access, burst cap PASS |
| report의 F-C01-V03 | C02 인계가 맞다. C01 로그만으로 PH_RESP_DRAIN II 영향이 완전히 닫힌 것은 아니다. |
| report의 F-C01-V04 | 보드 회로도 확인 항목으로 유지하는 판단이 맞다. |

## 수정 권장 요약

1. `C01_GPX_Bus_Read_Code_Verify_20260429_v001.md`를 v002로 복사해 ASYNC/SYNC 검증 해석을 정정한다.
2. `tb_tdc_gpx_config_ctrl`은 `g_DUT_STREAM_CLK_MODE` generic을 노출해 같은 TB를 ASYNC/SYNC 두 mode로 재사용할 수 있게 한다.
3. C02 진입 전 또는 C02 초반에 CSR IP binding이 포함된 `tdc_gpx_csr_chip` clamp TB를 추가한다.
