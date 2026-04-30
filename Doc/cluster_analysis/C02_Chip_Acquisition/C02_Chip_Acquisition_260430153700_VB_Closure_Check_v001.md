# C02_Chip_Acquisition VB Closure Check v001

문서 버전: `v001`  
작성일: `2026-04-30`  
최종 수정 시간: `2026-04-30 15:37:00 +09:00`  
작성 목적: 사용자 질문 “Plan v004의 기능 검증 경계 Matrix가 다 검증된 것인가?”에 대해, `C02_Chip_Acquisition_260430143509_Code_Fix_Plan_v004.md`의 `VB-C02-01~10` 경계를 현재 검증 결과와 매핑해 closure 상태를 명확히 기록한다.

---

## 1. 결론

아직 Plan v004의 기능 검증 경계 Matrix 전체가 닫힌 것은 아니다.

이번에 닫힌 것은 `tb_tdc_gpx_chip_ctrl` 단위의 핵심 positive 경계이다.

닫힌 범위:

- empty IFIFO read 0회 strict monitor
- count-known drain hard bound positive
- count-unknown EF fallback positive
- raw data/control beat 분리
- `chip_ctrl` raw AXI `tuser` control beat ID class
- `tb_tdc_gpx_chip_ctrl` xsim PASS

아직 남은 범위:

- negative forced fail run과 exit code evidence
- raw backpressure / PH_RESP_DRAIN stuck/fatal path
- config_ctrl/top integration에서 expected count CDC와 stale count
- Plan v004의 전체 AXI-stream `tuser` boundary
- timing legality 전체 조합과 pipeline/II 실측 table

---

## 2. 기능 검증 경계별 상태

| Boundary ID | 현재 상태 | 이번에 확인된 근거 | 남은 검증 |
|---|---|---|---|
| VB-C02-01 I-Mode single | 부분 검증 | `tb_tdc_gpx_chip_ctrl` 시나리오가 I-Mode single 흐름으로 동작하고 xsim PASS | config image/mode bit에서 Quiet/M/Continuous가 열리지 않는지 config/top 레벨 확인 필요 |
| VB-C02-02 Datasheet 금지 조건 | 부분 close | `tb_tdc_gpx_chip_ctrl.vhd:405-417`, `[16] empty-read=0`, xsim PASS | negative forced empty read에서 실패가 exit code로 전파되는지 확인 필요 |
| VB-C02-03 Count-known burst | 부분 close | `[2] data=12`, `[3] data=24`, `[4] data=16 cap`, `[9b] data=64`, empty-read=0 | expected_ififo stale/CDC drift 조건, config_ctrl handoff count 검증 필요 |
| VB-C02-04 Count-unknown EF-only | 부분 close | `[2b] expected=0/0 fallback data=5`, `[7] EF empty data=0`, empty-read=0 | EF guard cycle timestamp, raw EF -> 2FF -> decision timing 실측 필요 |
| VB-C02-05 Pipeline/II | 부분 분석 | Result v001 section 4에 latency/throughput/pipeline/II 영향 분석 기록 | best-case/guarded/backpressure II를 cycle monitor로 실측 필요 |
| VB-C02-06 Response/backpressure | 미검증 | 이번 TB는 `s_raw_axis_tready='1'` 고정이며 raw backpressure를 주지 않음 | raw FIFO full, pending hold, PH_RESP_DRAIN, fatal/stuck transcript 필요 |
| VB-C02-07 AXI-stream sideband contract | 부분 close | `chip_ctrl.o_m_raw_axis_tuser` data/control reserved bit와 IFIFO1/final control ID class 확인 | bus response, decoder, raw_event_builder, cell_builder, face_assembler, header_inserter, stop event input까지 전체 경계 검증 필요 |
| VB-C02-08 Negative/fail propagation | 미검증 | positive xsim PASS만 존재 | forced empty read, forced `tuser` violation 등 negative run exit code 1 evidence 필요 |
| VB-C02-09 Timing legality | 부분 검증 | 200 MHz, `bus_clk_div=1`, `bus_ticks=5` 조건으로 C02 TB PASS | illegal readout 조합 clamp/assertion, 40 MHz 이하 조합 전체 확인 필요 |
| VB-C02-10 Evidence boundary | 부분 close | Result v001/v002 Markdown/PPT, xsim command/result 기록 | 각 VB-C02 항목별 PASS/FAIL/보류 matrix와 로그 파일 경로를 별도 verify 문서로 완성 필요 |

상태 정의:

- `close`: 해당 경계가 Plan v004 pass/fail 기준까지 검증 문서와 로그로 닫힘
- `부분 close`: 핵심 positive 조건은 닫혔지만 Plan v004가 요구한 negative, integration, timing, CDC, full sideband 중 일부가 남음
- `미검증`: 이번 실행에서 직접 검증하지 않음

---

## 3. 이번 커밋이 실제로 닫은 항목

| 닫힌 범위 | 근거 |
|---|---|
| Empty IFIFO read positive 0회 | `tb_tdc_gpx_chip_ctrl.vhd:405-417`, `:1482-1486` |
| Count-known exact data count | `[2]`, `[3]`, `[4]`, `[9a]`, `[9b]` xsim PASS |
| Count-unknown EF fallback positive | `[2b]` xsim PASS |
| Raw data/control beat 분리 | `tb_tdc_gpx_chip_ctrl.vhd:455-499` |
| Raw control beat ID semantic | `tdc_gpx_chip_ctrl.vhd:1033-1039`, `[16] ififo1_done=14 final_done=16` |
| EF fallback guard 구현 | `tdc_gpx_chip_run.vhd:79`, `:207`, `g_EF_SYNC_GUARD_CLKS=5` |

---

## 4. 다음 검증 권장 순서

1. `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md`를 새로 작성해 VB-C02-01~10 closure matrix를 공식화한다.
2. Negative TB 또는 generic hook을 추가해 forced empty read / forced `tuser` violation이 xsim exit code fail로 전파되는지 확인한다.
3. `s_raw_axis_tready` backpressure scenario를 추가해 raw FIFO reserve, PH_RESP_DRAIN, pending hold, control beat preservation을 확인한다.
4. `tdc_gpx_config_ctrl`/top integration에서 expected_ififo CDC, stop event input `tuser`, downstream `tuser` 변환을 확인한다.
5. cycle timestamp monitor로 fallback II, burst II, backpressure II를 실측한다.

---

## 5. Lineage

| 기준 문서 | 본 문서 반영 |
|---|---|
| `C02_Chip_Acquisition_260430143509_Code_Fix_Plan_v004.md` section 6 | `VB-C02-01~10` 상태 재판정 |
| `C02_Chip_Acquisition_260430151152_Code_Fix_Result_v001.md` | 이번 xsim PASS로 닫힌 항목 근거 |
| `C02_Chip_Acquisition_260430152318_Code_Fix_Result_v002.md` | PPT 한글 수정본은 기술 검증 범위에 변화 없음 |

---

## 6. 다음 버전 반영 기록

최종 수정 시간: `2026-04-30 15:58:25 +09:00`

본 문서의 미검증/부분검증 분류는 다음 문서에 반영되었다.

| 반영 대상 | 반영 내용 |
|---|---|
| `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md` | raw backpressure, stale expected-count fault propagation, latency/II 실측 검증 수행 |
| `C02_Chip_Acquisition_260430155825_VB_Closure_Check_v002.md` | `VB-C02-03/05/06/08/10` 상태 갱신 및 남은 보류 항목 재분류 |
