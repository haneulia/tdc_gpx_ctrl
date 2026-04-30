# C02 Chip Acquisition - Code Fix Plan 미반영 항목 점검 v001

- 작성/수정 시간: 2026-04-30 21:31:18 +09:00
- 기준 계획: `C02_Chip_Acquisition_260430143509_Code_Fix_Plan_v004.md`
- 기준 결과: `C02_Chip_Acquisition_260430151152_Code_Fix_Result_v001.md`, `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md`, `C02_Chip_Acquisition_260430155825_VB_Closure_Check_v002.md`, `C02_Chip_Acquisition_260430205146_Skid_Sync_FIFO_Fix_Result_v001.md`, `C02_Chip_Acquisition_260430212306_T0_T1_Split_Timing_v001.md`
- 절대 기준: `Doc/TDC-GPX-Datasheet.pdf`

## 1. 결론

Plan v004의 C02 핵심 RTL/TB 보완 항목은 대부분 반영되었다. 다만 Plan v004가 요구한 전체 검증 경계 기준으로는 아직 완전 close가 아니다.

남은 항목은 크게 두 종류다.

| 분류 | 상태 |
|---|---|
| 미반영 또는 부분반영 검증 | forced negative exit-code, PH_RESP_DRAIN stuck/fatal 장기 격리, timing legality illegal matrix, downstream 전체 `tuser` boundary, top/config expected CDC integration |
| 의도적 후속/범위 제외 | OEN board default, output stream CDC 전체 재설계, 16-bit bus mode, 250 MHz retiming |

## 2. 반영 완료로 판단하는 항목

| Plan v004 항목 | 반영 상태 | 근거 |
|---|---|---|
| 목표 A count-known burst / count-unknown EF-only 분리 | 반영 | `tdc_gpx_chip_run.vhd:490..523`, `C02_Chip_Acquisition_260430151152_Code_Fix_Result_v001.md` section 2.1 |
| empty IFIFO read 0회 strict monitor | 반영 | `tb_tdc_gpx_chip_ctrl.vhd:2120..2128`, `xsim_chip_ctrl.log:1001` |
| expected hard bound / stale mismatch fault | 반영 | `tb_tdc_gpx_chip_ctrl.vhd:2072..2112`, `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md` section 1 |
| EF-only fallback guard | 반영 | `tdc_gpx_chip_run.vhd:78`, `:206`, `C02_Chip_Acquisition_260430151152_Code_Fix_Result_v001.md` section 2.2 |
| raw data/control `tuser` monitor | 부분 반영 | raw stream boundary는 반영. downstream 전체 AXI-stream boundary는 보류 |
| raw backpressure positive 검증 | 반영 | `tb_tdc_gpx_chip_ctrl.vhd:1823..2064`, `xsim_chip_ctrl.log:984..991` |
| latency/throughput/pipeline/II 세분화 | 반영 강화 | `C02_Chip_Acquisition_260430210752_Timing_Metric_Detail_v001.md`, `C02_Chip_Acquisition_260430212306_T0_T1_Split_Timing_v001.md` |
| sequential rule 보완 | 반영 | `C02_Chip_Acquisition_260430202746_Sequential_Logic_Rule_Fix_Result_v001.md`, `C02_Chip_Acquisition_260430205146_Skid_Sync_FIFO_Fix_Result_v001.md` |

## 3. 미반영 또는 부분반영 항목

| ID | 항목 | 현재 상태 | 필요한 다음 조치 |
|---|---|---|---|
| OP-C02-01 | forced empty read / forced `tuser` violation negative simulation과 exit-code evidence | 미반영 | C02 negative TB 또는 generic hook을 만들고, xsim 실패가 exit code 1로 전파되는 transcript를 남긴다. |
| OP-C02-02 | PH_RESP_DRAIN stuck/fatal 장기 격리 검증 | 부분반영 | RTL fatal/quarantine 경로는 존재하지만, 장기 stuck/fatal/auto-recover 전용 TB transcript가 아직 부족하다. |
| OP-C02-03 | config_ctrl/top expected-count CDC integration 검증 | 부분반영 | `tdc_gpx_config_ctrl.vhd`에 `xpm_cdc_handshake` 경로는 있으나, top/config 통합 시나리오에서 fire-count expected와 chip drain이 end-to-end로 닫혔는지 별도 검증이 필요하다. |
| OP-C02-04 | downstream 전체 AXI-stream `tuser` boundary 검증 | 부분반영 | raw stream `tuser`는 검증됨. decoder/raw_event_builder/cell/header까지 전체 `tuser` 변환 matrix 검증은 아직 별도다. |
| OP-C02-05 | timing legality illegal combination matrix | 부분반영 | 현재 200 MHz 정상 조건은 PASS. illegal `bus_ticks/div` 조합 clamp/assertion matrix는 별도 검증 필요. |
| OP-C02-06 | stale ready negative test | 부분반영 | skid/sync FIFO 구조 보완은 PASS. stale ready로 잘못 pop/drop되는 negative 전용 TB는 아직 명시적으로 닫히지 않았다. |

## 4. 의도적으로 후속 또는 제외된 항목

| 항목 | Plan v004 분류 | 현재 판단 |
|---|---|---|
| Quiet/M-mode full sequence | 완전 제외 | 반영 필요 없음. C02는 I-Mode single만 지원한다. |
| I-Mode continuous measurement | 완전 제외 | 반영 필요 없음. Datasheet 2.11.2는 C02 closure 범위 밖이다. |
| 250 MHz retiming | 완전 제외 | C02 code-fix 미반영이 정상이다. C01 근거만 보존한다. |
| 16-bit bus mode | 조건부 후속 | 28-bit closure 이후 검토한다. 현재는 unsupported 유지가 맞다. |
| OEN board default 최종 결정 | 후속 검토 | OEN 정상 연결과 OEN High 고정/pull-up/not-connected 두 경우는 board/integration 항목으로 남아 있다. OEN Low fixed는 unsupported 유지. |
| output stream CDC 전체 재설계 | 후속 검토 | C02 code-fix 직접 범위가 아니라 C03/C04 후보로 유지한다. |

## 5. VB Matrix 현재 판단

| Boundary ID | 현재 판단 |
|---|---|
| VB-C02-01 I-Mode single | 부분 close. Quiet/M/Continuous 제외 정책은 문서화됐고, top/config integration에서 mode 차단 확인 필요 |
| VB-C02-02 Datasheet 금지 조건 | 기능 PASS. negative forced fail evidence는 OP-C02-01로 남음 |
| VB-C02-03 Count-known burst | close에 가까움. stale mismatch fault까지 PASS |
| VB-C02-04 Count-unknown EF-only | 부분 close. `[2b]`, `[7]` PASS이나 EF guard timestamp 전용 검증은 남음 |
| VB-C02-05 Pipeline/II | 강화됨. T1a/T1b까지 분리 완료 |
| VB-C02-06 Response/backpressure | 부분 close. bounded raw backpressure는 PASS, PH_RESP_DRAIN stuck/fatal은 남음 |
| VB-C02-07 AXI-stream sideband contract | 부분 close. raw boundary는 PASS, downstream 전체는 남음 |
| VB-C02-08 Negative/fail propagation | 부분 close. stale mismatch fault는 PASS, forced negative exit-code는 남음 |
| VB-C02-09 Timing legality | 부분 close. 현재 정상 조건 PASS, illegal matrix는 남음 |
| VB-C02-10 Evidence boundary | 부분 close. Markdown/PPT/log는 계속 누적 중이나 OP-C02-01~06 추적표가 필요 |

## 6. 우선순위 제안

| 우선순위 | 다음 작업 | 이유 |
|---|---|---|
| P0 | forced negative exit-code evidence | 검증 체계가 실패를 놓치지 않는지 확인하는 상위 안전장치 |
| P0 | PH_RESP_DRAIN stuck/fatal 장기 격리 TB | response/backpressure 계약의 가장 위험한 잔여 항목 |
| P1 | config_ctrl/top expected-count CDC integration | echo_receiver fire-count 기반 expected 계약이 실제 top에서 닫히는지 확인 |
| P1 | downstream 전체 `tuser` boundary 검증 | C02 이후 cluster로 데이터/제어 의미가 깨지지 않는지 확인 |
| P2 | timing legality illegal matrix | 정상 운용은 PASS이나 CSR/clamp 방어 검증을 보강 |
| P2 | stale ready negative TB | skid/sync FIFO 보완의 방어적 추가 검증 |

## 7. 최종 판단

코드 수정 계획의 핵심 기능 보완은 반영되었지만, Plan v004 기준의 전체 closure는 아직 아니다. 특히 negative evidence와 PH_RESP_DRAIN stuck/fatal 검증은 C02를 완전히 닫기 전에 먼저 처리하는 것이 맞다.
## 8. 후속 반영 기록

- 2026-04-30 21:46:21 +09:00: OP-C02-01 forced negative monitor evidence와 OP-C02-02 PH_RESP_DRAIN stuck/fatal 장기 격리 검증은 `C02_Chip_Acquisition_260430214621_P0_Negative_PHRespDrain_Verify_v001.md` 2장부터 5장에 반영되었다.
- 반영 코드: `tdc_gpx_chip_ctrl.vhd:839`, `tdc_gpx_chip_ctrl.vhd:882`, `tdc_gpx_chip_ctrl.vhd:905..919`, `tb_tdc_gpx_chip_ctrl.vhd:52`, `tb_tdc_gpx_chip_ctrl.vhd:233..234`, `tb_tdc_gpx_chip_ctrl.vhd:453..458`, `tb_tdc_gpx_chip_ctrl.vhd:614..619`, `tb_tdc_gpx_chip_ctrl.vhd:839..877`, `tb_tdc_gpx_chip_ctrl.vhd:2251..2308`.
- 반영 로그: `xsim_chip_ctrl.log:1005..1343`, `xsim_chip_ctrl_neg_empty.log:35..40`, `xsim_chip_ctrl_neg_tuser.log:35..40`.
