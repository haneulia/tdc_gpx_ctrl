# C02 Chip Acquisition - Code Fix Plan 미반영 항목 재평가 v001

- 최초 작성 시간: `2026-04-30 21:31:18 +09:00`
- 최종 수정 시간: `2026-04-30 23:07:32 +09:00`
- 기준 계획: `C02_Chip_Acquisition_260430143509_Code_Fix_Plan_v004.md`
- 절대 기준: `Doc/TDC-GPX-Datasheet.pdf`
- 관련 결과: `C02_Chip_Acquisition_260430151152_Code_Fix_Result_v001.md`, `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md`, `C02_Chip_Acquisition_260430205146_Skid_Sync_FIFO_Fix_Result_v001.md`, `C02_Chip_Acquisition_260430212306_T0_T1_Split_Timing_v001.md`

## 1. 결론

Plan v004의 C02 핵심 RTL/TB 보완 항목은 대부분 반영되었다. 2026-04-30 23:07:32 기준으로 OP-C02-01, OP-C02-02, OP-C02-04, OP-C02-05, OP-C02-06은 근거 문서와 xsim 로그로 닫혔다.

현재 C02에서 우선 확인할 잔여 항목은 `OP-C02-03 config_ctrl/top expected-count CDC integration`이다. 이 항목은 echo_receiver fire-count 기반 expected 계약이 config/top 통합 경로에서 chip drain까지 end-to-end로 닫히는지 확인해야 한다.

## 2. 반영 완료로 판단하는 항목

| Plan v004 항목 | 상태 | 근거 |
|---|---|---|
| 목표 A count-known burst / count-unknown EF-only 분리 | 반영 | `tdc_gpx_chip_run.vhd:490..523`, `C02_Chip_Acquisition_260430151152_Code_Fix_Result_v001.md` |
| empty IFIFO read 0회 strict monitor | 반영 | `tb_tdc_gpx_chip_ctrl.vhd:2120..2128`, `xsim_chip_ctrl.log:1001` |
| expected hard bound / stale mismatch fault | 반영 | `tb_tdc_gpx_chip_ctrl.vhd:2072..2112`, `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md` |
| EF-only fallback guard | 반영 | `tdc_gpx_chip_run.vhd:78`, `tdc_gpx_chip_run.vhd:206`, `C02_Chip_Acquisition_260430151152_Code_Fix_Result_v001.md` |
| raw data/control `tuser` monitor | 반영 | raw stream boundary와 downstream cell/output-stage boundary를 분리 검증 |
| raw backpressure positive 검증 | 반영 | `tb_tdc_gpx_chip_ctrl.vhd:1823..2064`, `xsim_chip_ctrl.log:984..991` |
| latency/throughput/pipeline/II 세분화 | 반영 강화 | `C02_Chip_Acquisition_260430210752_Timing_Metric_Detail_v001.md`, `C02_Chip_Acquisition_260430212306_T0_T1_Split_Timing_v001.md` |
| sequential rule 보완 | 반영 | `C02_Chip_Acquisition_260430202746_Sequential_Logic_Rule_Fix_Result_v001.md`, `C02_Chip_Acquisition_260430205146_Skid_Sync_FIFO_Fix_Result_v001.md` |

## 3. Open Item 상태

| ID | 항목 | 현재 상태 | 근거 / 다음 조치 |
|---|---|---|---|
| OP-C02-01 | forced empty read / forced `tuser` violation negative simulation 및 exit-code evidence | 반영 | `C02_Chip_Acquisition_260430214621_P0_Negative_PHRespDrain_Verify_v001.md`에서 forced negative exit-code evidence 검증 |
| OP-C02-02 | PH_RESP_DRAIN stuck/fatal 장기 격리 검증 | 반영 | `C02_Chip_Acquisition_260430214621_P0_Negative_PHRespDrain_Verify_v001.md`에서 stuck/fatal/quarantine/recovery 검증 |
| OP-C02-03 | config_ctrl/top expected-count CDC integration 검증 | 부분반영 | `tdc_gpx_config_ctrl.vhd`에 `xpm_cdc_handshake` 경로는 있으나 top/config 통합 시나리오에서 fire-count expected와 chip drain이 end-to-end로 닫혔는지 별도 검증 필요 |
| OP-C02-04 | downstream 전체 AXI-stream `tuser` boundary 검증 | 반영 | `C02_Chip_Acquisition_260430224233_Downstream_TUSER_Boundary_Fix_v001.md`에서 cell_pipe/output_stage `tuser` boundary와 row fault pulse 계약 검증 |
| OP-C02-05 | timing legality illegal combination matrix | 반영 | `C02_Chip_Acquisition_260430225644_Timing_Legality_Matrix_Fix_v001.md`에서 CSR clamp 68-case matrix와 Bus_Phy local timing clamp 검증 |
| OP-C02-06 | stale ready negative test | 반영 | `C02_Chip_Acquisition_260430230732_Stale_Ready_Negative_Fix_v001.md`에서 skid/sync FIFO stale-ready pop/drop/duplicate 방지 전용 TB 검증 |

## 4. 의도적으로 후속 또는 제외된 항목

| 항목 | 현재 판단 |
|---|---|
| Quiet/M-mode full sequence | 완전 제외. C02는 I-Mode single만 지원한다. |
| I-Mode continuous measurement | 완전 제외. Datasheet 2.11.2 Continuous Measurement는 C02 closure 범위 밖이다. |
| 250 MHz retiming | 제외. C02 code-fix 직접 범위가 아니다. |
| 16-bit bus mode | 28-bit closure 이후 검토한다. 현재는 unsupported 유지가 맞다. |
| OEN board default 최종 결정 | OEN 정상 연결과 OEN High 고정/pull-up/not-connected 두 경우만 board/integration 항목으로 남긴다. OEN Low fixed는 unsupported다. |
| output stream CDC 전체 재설계 | 후속 검토. C02 직접 code-fix 범위가 아니며 C03/C04 인계 항목으로 유지한다. |

## 5. VB Matrix 현재 판단

| Boundary ID | 현재 판단 |
|---|---|
| VB-C02-01 I-Mode single | 부분 close. Quiet/M/Continuous 제외 정책은 문서화됐고, top/config integration에서 mode 차단 확인 필요 |
| VB-C02-02 Datasheet 금지 조건 | close 가능. 기능 PASS와 negative forced fail evidence 확인 |
| VB-C02-03 Count-known burst | close 가능. stale mismatch fault까지 PASS |
| VB-C02-04 Count-unknown EF-only | 부분 close. `[2b]`, `[7]` PASS이나 EF guard timestamp 전용 검증은 후속 판단 |
| VB-C02-05 Pipeline/II | close 가능. T1a/T1b까지 분리 완료 |
| VB-C02-06 Response/backpressure | close 가능. bounded raw backpressure, PH_RESP_DRAIN stuck/fatal, stale-ready negative boundary 모두 PASS |
| VB-C02-07 AXI-stream sideband contract | close 가능. raw boundary와 downstream cell/output-stage boundary 모두 PASS |
| VB-C02-08 Negative/fail propagation | close 가능. stale mismatch fault와 forced negative exit-code evidence 모두 PASS |
| VB-C02-09 Timing legality | close 가능. 정상 조건과 illegal matrix 모두 PASS |
| VB-C02-10 Evidence boundary | 부분 close. Markdown/PPT/log는 계속 누적 중이며 OP-C02-03 통합 검증 문서가 필요 |

## 6. 우선순위 제안

| 우선순위 | 다음 작업 | 이유 |
|---|---|---|
| P1 | config_ctrl/top expected-count CDC integration | echo_receiver fire-count 기반 expected 계약이 실제 top/config 경로에서 닫히는지 확인 |

## 7. 최종 판단

OP-C02-06까지 반영되면서 C02 내부 보완 검증의 잔여 위험은 대부분 닫혔다. 다음 단계는 `OP-C02-03`을 기준으로 top/config 통합 검증을 수행하고, 이 결과에 따라 C02 closure 또는 C03/C04 인계 계약을 정리하는 것이다.

## 8. 후속 반영 기록

- 2026-04-30 21:46:21 +09:00: OP-C02-01 forced negative monitor evidence와 OP-C02-02 PH_RESP_DRAIN stuck/fatal 장기 격리 검증은 `C02_Chip_Acquisition_260430214621_P0_Negative_PHRespDrain_Verify_v001.md`에 반영했다.
- 반영 코드: `tdc_gpx_chip_ctrl.vhd:839`, `tdc_gpx_chip_ctrl.vhd:882`, `tdc_gpx_chip_ctrl.vhd:905..919`, `tb_tdc_gpx_chip_ctrl.vhd:52`, `tb_tdc_gpx_chip_ctrl.vhd:233..234`, `tb_tdc_gpx_chip_ctrl.vhd:453..458`, `tb_tdc_gpx_chip_ctrl.vhd:614..619`, `tb_tdc_gpx_chip_ctrl.vhd:839..877`, `tb_tdc_gpx_chip_ctrl.vhd:2251..2308`.
- 반영 로그: `xsim_chip_ctrl.log:1005..1343`, `xsim_chip_ctrl_neg_empty.log:35..40`, `xsim_chip_ctrl_neg_tuser.log:35..40`.
- 2026-04-30 22:42:33 +09:00: OP-C02-04 downstream AXI-stream `tuser` boundary 검증과 보완은 `C02_Chip_Acquisition_260430224233_Downstream_TUSER_Boundary_Fix_v001.md` 및 동일 timestamp PPT에 반영했다.
- 반영 코드: `tdc_gpx_face_assembler.vhd:543`, `tdc_gpx_face_assembler.vhd:580..597`, `tdc_gpx_face_assembler.vhd:896`, `tb_tdc_gpx_cell_pipe.vhd:135`, `tb_tdc_gpx_cell_pipe.vhd:166`, `tb_tdc_gpx_cell_pipe.vhd:228`, `tb_tdc_gpx_cell_pipe.vhd:293`, `tb_tdc_gpx_output_stage.vhd:190`, `tb_tdc_gpx_output_stage.vhd:273..274`, `tb_tdc_gpx_output_stage.vhd:317..326`, `tb_tdc_gpx_output_stage.vhd:387`, `tb_tdc_gpx_output_stage.vhd:419..422`, `tb_tdc_gpx_output_stage.vhd:431..438`, `tb_tdc_gpx_output_stage.vhd:543..551`.
- 반영 로그: `xsim_cell_pipe_tuser.log:28`, `xsim_output_stage_tuser.log:38..56`.
- 2026-04-30 22:56:44 +09:00: OP-C02-05 timing legality illegal combination matrix는 `C02_Chip_Acquisition_260430225644_Timing_Legality_Matrix_Fix_v001.md` 및 동일 timestamp PPT에 반영했다.
- 반영 코드: `tb_tdc_gpx_csr_chip_clamp.vhd:301..337`, `tb_tdc_gpx_csr_chip_clamp.vhd:348`, `tb_tdc_gpx_bus_phy_c01_contract.vhd:83..100`, `tb_tdc_gpx_bus_phy_c01_contract.vhd:225..289`.
- 반영 로그: `xsim_csr_chip_timing_matrix.log:570..574`, `xsim_bus_phy_c01_timing_matrix.log:28..42`.
- 2026-04-30 23:07:32 +09:00: OP-C02-06 stale ready negative test는 `C02_Chip_Acquisition_260430230732_Stale_Ready_Negative_Fix_v001.md` 및 동일 timestamp PPT에 반영했다.
- 반영 코드: `tb_tdc_gpx_stale_ready.vhd:30`, `tb_tdc_gpx_stale_ready.vhd:117`, `tb_tdc_gpx_stale_ready.vhd:198`, `tb_tdc_gpx_stale_ready.vhd:235`, `tb_tdc_gpx_stale_ready.vhd:278`, `tb_tdc_gpx_stale_ready.vhd:281`.
- 반영 로그: `xsim_stale_ready.log:30`, `xsim_stale_ready.log:34`, `xsim_stale_ready.log:36`.
