# C01 GPX Bus Read Code Verify v002 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v002 리뷰 |
| 작성 시각 | 2026-04-29 16:14:06 +09:00 |
| 수정 시각 | 2026-04-29 16:14:06 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_20260429_v002.md` 및 `tmp/c01_verify` 추가 xsim 로그 |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | 추가 검증 보고 자료의 PASS 판정 타당성, 남은 운용/검증 리스크, 추적 가능한 보완점 확인 |

## 1. 종합 판단

v002 추가 검증 보고서의 핵심 PASS 판정은 타당하다. 특히 v001 리뷰에서 지적된 다음 항목은 실질적으로 보완되었다.

| 항목 | v002 판단 | 근거 |
|---|---|---|
| CSR clamp black-box gap | 닫힘 | `elab_tb_tdc_gpx_csr_chip_clamp.log`에서 CSR IP source가 함께 elaborate되고, `sim_tb_tdc_gpx_csr_chip_clamp.log:117`에서 11 case PASS 확인 |
| config_ctrl SYNC/ASYNC 경로 구분 | 닫힘 | `sim_tb_tdc_gpx_config_ctrl_SYNC.log:29`, `sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29`에서 mode echo 확인 |
| ASYNC FIFO generate 활성 | 닫힘 | `elab_tb_tdc_gpx_config_ctrl_ASYNC.log:69`에서 `xpm_fifo_async` compile 확인 |
| SYNC bypass generate | 닫힘 | SYNC elaboration log에서 `xpm_fifo_async` compile이 없고, `tdc_gpx_config_ctrl.vhd`의 `g_STREAM_CLK_MODE`가 TB generic으로 전달됨 |
| C01 timing clamp 계약 | 닫힘 | `sim_tb_tdc_gpx_csr_chip_clamp.log`에서 `(0,4)->(1,5)`, `(1,4)->(1,5)`, `(2,3)->(2,4)` 등 11 case PASS |

따라서 v002의 `6/6 PASS` 결론은 C01 범위에서는 인정 가능하다. 단, 아래 항목은 보고서/검증 운영 품질을 위해 다음 문서 또는 스크립트에 반영해야 한다.

## 2. 리뷰 발견 사항

### R-C01-V002-01. `glbl` generic override warning을 허용 warning 목록에 추가해야 함

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 상태 | PASS 판정은 유지, 운영 규칙 보완 필요 |
| 현상 | SYNC/ASYNC 두 elaboration log 모두에서 `work.glbl`에 `g_dut_stream_clk_mode` generic을 override하려는 warning이 발생한다. |
| 근거 | `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_SYNC.log:8`, `tmp/c01_verify/elab_tb_tdc_gpx_config_ctrl_ASYNC.log:8` |
| 원인 | `tmp/c01_verify/run_config_ctrl_two_modes.sh:38`의 `-generic_top "g_DUT_STREAM_CLK_MODE=${mode}"`가 `tmp/c01_verify/run_config_ctrl_two_modes.sh:41`의 `work.glbl`에도 적용되는 것으로 보인다. |
| 판단 | `sim_tb_tdc_gpx_config_ctrl_SYNC.log:29`, `sim_tb_tdc_gpx_config_ctrl_ASYNC.log:29`의 mode echo가 올바르므로 DUT generic 전달 자체는 성공했다. 기능 실패는 아니다. |
| 조치 | v002 보고서의 허용 warning 목록에 해당 warning을 추가하거나, xelab 인자 구성을 수정해 generic override가 VHDL TB top에만 적용되도록 한다. |

### R-C01-V002-02. CSR clamp TB/보고서의 `writer/reader` 표현이 실제 구현과 다름

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 상태 | 검증 결론은 유지, 문구 수정 필요 |
| 현상 | 보고서와 TB 주석은 `px_axi_lite_writer/px_axi_lite_reader`를 사용한다고 설명하지만, 실제 TB는 `px_axi_lite_writer`만 호출하고 출력 clamp 결과를 직접 관측한다. |
| 근거 | `tb_tdc_gpx_csr_chip_clamp.vhd:7`, `tb_tdc_gpx_csr_chip_clamp.vhd:64`, `tb_tdc_gpx_csr_chip_clamp.vhd:218`, `C01_GPX_Bus_Read_Code_Verify_20260429_v002.md:94`, `C01_GPX_Bus_Read_Code_Verify_20260429_v002.md:187` |
| 판단 | 이 검증의 목적은 AXI write 이후 CDC를 거쳐 `o_bus_clk_div`/`o_bus_ticks` 출력이 clamp되는지 확인하는 것이므로 readback이 없어도 C01 clamp 검증은 성립한다. |
| 조치 | 보고서 문구를 `px_axi_lite_writer 사용 + 출력 직접 관측`으로 고친다. CSR register readback까지 검증하려면 별도 case로 `px_axi_lite_reader`를 추가한다. |

### R-C01-V002-03. ASYNC/SYNC 2-mode 검증이 기본 regression entrypoint와 분리되어 있음

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 상태 | 현재 v002 보고서 기준 PASS 인정, 반복 검증 절차 보완 필요 |
| 현상 | `tb_tdc_gpx_config_ctrl`의 기본 generic은 `"SYNC"`이며, ASYNC 검증은 별도 `run_config_ctrl_two_modes.sh`로 수행된다. |
| 근거 | `tb_tdc_gpx_config_ctrl.vhd:23`, `tb_tdc_gpx_config_ctrl.vhd:205`, `tmp/c01_verify/run_config_ctrl_two_modes.sh:2`, `tmp/c01_verify/run_config_ctrl_two_modes.sh:38` |
| 판단 | v002 자료에는 두 mode 로그가 모두 있으므로 이번 PASS 판정에는 문제가 없다. 다만 다음 회귀 실행자가 `scripts/run_all_tbs.tcl`만 실행하면 ASYNC 경로를 놓칠 수 있다. |
| 조치 | C01 완료 조건에 `run_config_ctrl_two_modes.sh` 실행을 명시하거나, 2-mode 실행을 주 회귀 스크립트에 편입한다. |

### R-C01-V002-04. `c_CDC_WAIT` 설명을 200 ns 기준으로 정리할 필요가 있음

| 항목 | 내용 |
|---|---|
| 심각도 | Trivial |
| 상태 | 기능 영향 없음 |
| 현상 | CSR clamp TB는 `c_CDC_WAIT = 200 ns`를 사용한다. 이전 설명에서 150 ns/30 cycle 기준 표현이 남아 있으면 실제 코드와 다르게 읽힐 수 있다. |
| 근거 | `tb_tdc_gpx_csr_chip_clamp.vhd:48`, `tb_tdc_gpx_csr_chip_clamp.vhd:236`, `tb_tdc_gpx_csr_chip_clamp.vhd:272` |
| 판단 | 200 ns는 200 MHz 기준 40 cycle로 보수적인 대기시간이다. 실패 요인은 아니며 설명 정합성 문제다. |
| 조치 | 문서와 주석을 `200 ns = 40 cycles @ 200 MHz`로 명확히 한다. |

## 3. 로그별 판단 요약

| 로그 | 판단 | 핵심 근거 |
|---|---|---|
| `sim_tb_tdc_gpx_csr_chip_clamp.log` | PASS | `*** ALL TESTS PASSED *** (cases=11)` |
| `elab_tb_tdc_gpx_csr_chip_clamp.log` | PASS | CSR IP source가 black-box 없이 elaborate됨 |
| `sim_tb_tdc_gpx_config_ctrl_SYNC.log` | PASS | `TB: g_DUT_STREAM_CLK_MODE = SYNC`, init 완료 |
| `sim_tb_tdc_gpx_config_ctrl_ASYNC.log` | PASS | `TB: g_DUT_STREAM_CLK_MODE = ASYNC`, init 완료 |
| `elab_tb_tdc_gpx_config_ctrl_SYNC.log` | PASS with allowed warning | `xpm_fifo_async` 미컴파일, `glbl` generic warning 존재 |
| `elab_tb_tdc_gpx_config_ctrl_ASYNC.log` | PASS with allowed warning | `xpm_fifo_async` 컴파일, `glbl` generic warning 존재 |

## 4. 다음 반영 권장

1. v002 검증 보고서의 warning allow-list에 `VRFC 10-3532 glbl generic override`를 추가한다.
2. CSR clamp 설명을 `AXI write + CDC + output observation`으로 정정한다.
3. C01 반복 검증 절차에 `run_config_ctrl_two_modes.sh`를 공식 포함하거나 주 회귀 스크립트에 통합한다.
4. `c_CDC_WAIT = 200 ns` 설명을 200 MHz 기준 40 cycle로 통일한다.

## 5. C01 완료 판단

위 보완점은 모두 검증 운영/문서 정합성 이슈이며, 현재 추가 검증 로그의 기능 PASS 판단을 뒤집지는 않는다. C01의 구현 보완 검증은 `조건부 완료`로 판단하고, 남은 정량 항목인 backpressure, IrFlag, PH_RESP_DRAIN II, div>=2 burst II는 v002 보고서의 인계 계획처럼 C02에서 다루는 것이 합리적이다.
