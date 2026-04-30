# C02_Chip_Acquisition Code Fix Plan Review v003

문서 버전: `v003`
작성일: `2026-04-30`
최종 수정 시간: `2026-04-30 14:35:09 +09:00`
작성 목적: `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v003.md`를 사용자가 위에서부터 검토하면서 공유한 의문점과 수정 방향을 시간순으로 기록한다. 사용자의 "다 됐어" 확인 후 본 문서의 피드백은 `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v004.md`에 반영되었다.

---

## 1. 검토 운영 방식

| 항목 | 내용 |
|---|---|
| 검토 대상 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v003.md` |
| 검토 방식 | 사용자가 의문점과 수정 방향을 전달하면 Codex는 판단과 근거를 기록한다. |
| 반영 조건 | 사용자가 "다 됐어", "다 검토 했어" 또는 "반영해줘"라고 명시하면 Plan v004에 반영한다. |
| 현재 상태 | Plan v004 반영 완료 |

---

## 2. 사용자 검토 메모

### R-C02-P003-01. VB-C02-01은 I-Mode 중에서도 Datasheet 2.11.1 Single measurement만 포함해야 한다

| 항목 | 내용 |
|---|---|
| 입력 시각 | 2026-04-30 14:27:52 +09:00 |
| 관련 Plan 위치 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v003.md` section 6 `기능 검증 경계 Matrix`, `VB-C02-01 I-Mode single` |
| 사용자 피드백 | I-Mode 중에서도 오로지 Single 측정만 해당되며, Datasheet 2.11.2 `Continous Measurement`는 해당사항이 아니다. |
| Codex 1차 판단 | 사용자 지적이 맞다. Plan v003의 `I-Mode single` 문구만으로는 I-Mode 내부의 continuous/start-retrigger 운용까지 배제되는지 불명확하다. C02 범위는 Datasheet 2.11.1 `Single measurement` flow로 고정하고, 2.11.2 `Continous Measurement` flow는 명시적으로 제외해야 한다. |
| 반영 필요 방향 | Plan v004의 VB-C02-01 포함 범위에 `Datasheet 2.11.1 Single measurement only`를 명시한다. 제외 범위에는 `Datasheet 2.11.2 Continous Measurement`, internal start retrigger, Start# handling, Start01 read/remap, continuous EF polling을 추가한다. |
| Datasheet 근거 | `Doc/TDC-GPX-Datasheet.pdf p.28 section 2.11.1 Single measurement`: Start 후 IrFlag 확인, EF1/EF2가 Low일 때 IFIFO read, 이후 Alutrigger master reset. `p.29 section 2.11.2 Continous Measurement`: internal start retrigger, Start# read/remap, continuous EF polling flow. |
| 연결 계약 | C01-C32, C02 AP-C02-01, VB-C02-01 |
| 반영 상태 | Plan v004 반영 완료 |

#### Plan v004에 필요한 VB-C02-01 정정안

| 항목 | Plan v003 | Plan v004 후보 |
|---|---|---|
| 포함 범위 | I-Mode single shot start, IrFlag, IFIFO drain, cleanup | Datasheet 2.11.1 `Single measurement` only: single Start, IrFlag wait, EF-based IFIFO drain, post-drain Alutrigger/master-reset cleanup |
| 제외 범위 | Quiet/M-mode, R-mode quiet | Quiet/M-mode, R-mode quiet, Datasheet 2.11.2 `Continous Measurement`, internal start retrigger, Start# overflow/Start01 handling, continuous EF polling |
| 핵심 pass/fail 기준 | single measurement sequence만 실행 | 2.11.1 sequence만 실행하며 2.11.2 관련 상태/검증 항목이 C02 closure에 섞이면 안 됨 |

### R-C02-P003-02. VB-C02-02의 `fill=0`은 의미가 부족하며 Reg6 Fill과 구분해야 한다

| 항목 | 내용 |
|---|---|
| 입력 시각 | 2026-04-30 14:27:52 +09:00 |
| 관련 Plan 위치 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v003.md` section 6 `기능 검증 경계 Matrix`, `VB-C02-02 Datasheet 금지 조건` |
| 사용자 피드백 | `fill=0`일 때 read가 1회라도 있으면 Fail이라는 뜻인지 확인이 필요하다. 그런데 `fill=0`의 의미가 부족해서 맞는지 판단하기 어렵다. |
| Codex 1차 판단 | 사용자 지적이 맞다. Plan v003의 `fill=0 monitor` 표현은 부정확하다. Datasheet의 `Fill`은 Register 6의 LF load-level threshold이며 empty 여부 자체가 아니다. VB-C02-02에서 말한 의도는 TB/GPX model이 추적하는 해당 Interface FIFO의 실제 occupancy가 0인 상태에서 Reg8 또는 Reg9 read가 발생하면 FAIL이라는 뜻이다. |
| 반영 필요 방향 | Plan v004에서는 `fill=0`이라는 표현을 제거하거나 `model_ififo_occupancy_before_read = 0`으로 바꾼다. 또한 `Reg6 Fill threshold`와 `IFIFO empty/occupancy`를 분리해 설명한다. |
| Datasheet 근거 | `Doc/TDC-GPX-Datasheet.pdf p.7 Figure 2`: empty FIFO read 금지. `p.19 Register 6`: `Fill`은 LF flag threshold. `p.48~49 Internal Data Processing / Readout`: EF는 empty flag, LF는 load-level flag이고 `Fill`은 LF threshold이며 empty Interface FIFO read는 금지된다. |
| 연결 계약 | C01-C11, C01-C14, C01-C21, C01-C22, VB-C02-02, VB-C02-04 |
| 반영 상태 | Plan v004 반영 완료 |

#### Plan v004에 필요한 VB-C02-02 정정안

| 항목 | 의미 |
|---|---|
| `Reg6 Fill` | Datasheet register field. LF1/LF2 load-level flag가 set되는 threshold를 정한다. empty 판단 기준이 아니다. |
| `IFIFO occupancy` | TB/GPX model이 알고 있는 실제 Interface FIFO word count. 검증 monitor의 기준이다. |
| empty read FAIL 조건 | Reg8 read 시 `ififo1_occupancy_before_read = 0`이면 FAIL. Reg9 read 시 `ififo2_occupancy_before_read = 0`이면 FAIL. |
| 정상 마지막 read | `occupancy_before_read = 1`에서 read가 발생하고 read 후 0이 되는 것은 정상이다. 이때 EF는 Datasheet `tS-EF` window 이후 HIGH가 될 수 있다. |
| EF 기반 보조 관측 | 실제 RTL은 occupancy를 직접 알 수 없으므로 EF/LF/status를 사용하지만, TB 검증은 모델 occupancy를 절대 기준으로 empty read를 잡아야 한다. |

#### Plan v004 후보 문장

> `fill=0 monitor`라는 표현은 사용하지 않는다. C02의 데이터시트 금지 조건 검증은 `model_ififo_occupancy_before_read = 0` 상태에서 Reg8/Reg9 read가 발생했는지를 확인한다. 이는 Datasheet의 Register 6 `Fill` threshold와 다른 개념이다. Reg6 `Fill`은 LF load-level threshold이고, empty read 금지의 직접 기준은 EF/Interface FIFO empty 상태이다.

### R-C02-P003-03. VB-C02-07에는 모든 AXI-stream `tuser` contract matrix가 필요하다

| 항목 | 내용 |
|---|---|
| 입력 시각 | 2026-04-30 14:29:39 +09:00 |
| 관련 Plan 위치 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v003.md` section 6 `VB-C02-07 Data/control boundary`, section 7 `Latency / Throughput / Pipeline / II 분석 계획` |
| 사용자 피드백 | `VB-C02-07 Data/control boundary`처럼 AXI-stream 데이터의 `tuser` 영역에 대한 bit 구간별 의미 설명이 부족하다. 따라서 입/출력되는 모든 모듈의 AXI-stream에서 사용되는 `tuser` 정보가 필요하다. |
| Codex 1차 판단 | 사용자 지적이 맞다. v003의 `VB-C02-07`은 `tuser(7)` control beat만 언급하므로 data/control boundary를 닫기에는 부족하다. 현재 RTL에는 bus response 8-bit `tuser`, raw IFIFO 8-bit `tuser`, decoded event 8-bit `tuser`, enriched event 16-bit `tuser`, per-chip cell 1-bit `tuser`, final VDMA 1-bit `tuser`, 그리고 external stop event `tuser`가 서로 다른 의미로 존재한다. 이들은 같은 이름을 쓰지만 bit 의미가 다르므로 모듈별 contract matrix로 관리해야 한다. |
| 반영 필요 방향 | Plan v004의 VB-C02-07을 `AXI-stream sideband contract boundary`로 확장한다. 모든 AXI-stream 입출력 모듈에 대해 `module`, `port direction`, `tuser width`, `bit range`, `meaning`, `data/control beat별 유효성`, `source`, `consumer`, `verification monitor`를 표로 작성하도록 한다. |
| RTL 근거 | `tdc_gpx_bus_phy.vhd` write/read response `tuser`, `tdc_gpx_chip_ctrl.vhd:1019-1029` raw FIFO `tuser`, `tdc_gpx_decoder_i_mode.vhd` file header and decode logic, `tdc_gpx_raw_event_builder.vhd` file header and `:129-166`, `tdc_gpx_cell_builder.vhd` file header / `s_tuser_r`, `tdc_gpx_face_assembler.vhd` per-chip faulted `tuser(0)`, `tdc_gpx_header_inserter.vhd:470-502` final SOF `tuser(0)`, `tdc_gpx_stop_cfg_decode.vhd:220-286` stop event `tuser` count lanes |
| 연결 계약 | C01-C3, C01-C4, C01-C20, C01-C29, C01-C32, VB-C02-07, VB-C02-10 |
| 반영 상태 | Plan v004 반영 완료 |

#### Plan v004에 필요한 `tuser` contract matrix 초안

| 구간 | 모듈 / 포트 | Width | bit 구간 | 의미 |
|---|---|---:|---|---|
| bus response | `tdc_gpx_bus_phy.o_m_axis_tuser` -> `tdc_gpx_chip_ctrl.i_s_axis_tuser` | 8 | `[0]` | `0` READ response, `1` WRITE ack |
| bus response | same | 8 | `[4:1]` | target address |
| raw IFIFO stream | `tdc_gpx_chip_ctrl.o_m_raw_axis_tuser` -> `tdc_gpx_decoder_i_mode.i_s_axis_tuser` | 8 | `[0]` | data beat: IFIFO id (`0` IFIFO1, `1` IFIFO2) |
| raw IFIFO stream | same | 8 | `[5]` | control beat only: drain_done faulted flag |
| raw IFIFO stream | same | 8 | `[7]` | `1` control beat (`ififo1_done` or final `drain_done`), `0` data beat |
| decoded I-Mode stream | `tdc_gpx_decoder_i_mode.o_m_axis_tuser` -> `tdc_gpx_raw_event_builder.i_s_axis_tuser` | 8 | `[0]` | data beat: slope |
| decoded I-Mode stream | same | 8 | `[2:1]` | data beat: raw channel code within IFIFO |
| decoded I-Mode stream | same | 8 | `[5:3]` | data beat: reconstructed local stop id |
| decoded I-Mode stream | same | 8 | `[5]` | control beat only: faulted flag. `tuser[7]` disambiguates from data stop id |
| decoded I-Mode stream | same | 8 | `[6]` | IFIFO id |
| decoded I-Mode stream | same | 8 | `[7]` | drain_done control beat |
| enriched event stream | `tdc_gpx_raw_event_builder.o_m_axis_tuser` -> `cell_pipe/cell_builder.i_evt_sk_tuser` | 16 | `[0]` | slope |
| enriched event stream | same | 16 | `[2:1]` | chip id |
| enriched event stream | same | 16 | `[5:3]` | data beat: stop id |
| enriched event stream | same | 16 | `[5]` | control beat only: faulted flag. `tuser[7]` disambiguates |
| enriched event stream | same | 16 | `[6]` | IFIFO id |
| enriched event stream | same | 16 | `[7]` | drain_done control beat |
| enriched event stream | same | 16 | `[10:8]` | local hit sequence |
| enriched event stream | same | 16 | `[15:11]` | shot sequence lower 5 bits |
| cell stream | `tdc_gpx_cell_builder.o_m_axis_tuser(0)` -> `tdc_gpx_face_assembler.i_s_axis_tuser` | 1 per chip | `[0]` | faulted flag, valid on the chip `tlast` beat |
| final VDMA stream | `tdc_gpx_header_inserter.o_m_axis_tuser(0)` -> top output | 1 | `[0]` | SOF on first header beat of first line. Data beats drive `0` |
| stop event input | `tdc_gpx_top.i_stop_evt_tuser` -> `tdc_gpx_stop_cfg_decode` | `g_STOP_EVT_DWIDTH` | per chip 8-bit lane `[i*8+3:i*8]`, `[i*8+7:i*8+4]` | paired with `tdata` nibble lanes to form expected IFIFO1/IFIFO2 count totals |

#### Plan v004 후보 문장

> `VB-C02-07`은 단순히 data/control beat count를 분리하는 항목이 아니라, C02 이후 pipeline 전체의 AXI-stream sideband contract를 검증하는 경계로 확장한다. 같은 이름의 `tuser`라도 모듈 경계마다 bit 의미가 달라지므로, Plan v004와 검증 문서에는 모듈별 `tuser` bit map, 변환 규칙, control beat에서만 유효한 overlap bit, TB monitor 기준을 반드시 포함한다.

## 3. 반영 목록

| Review ID | 요약 | 우선순위 | 상태 |
|---|---|---|---|
| R-C02-P003-01 | VB-C02-01 범위를 Datasheet 2.11.1 Single measurement only로 명시하고 2.11.2 Continuous Measurement를 제외 | 높음 | 반영 완료 |
| R-C02-P003-02 | VB-C02-02의 `fill=0`을 TB/model IFIFO occupancy 기준으로 정정하고 Reg6 Fill threshold와 구분 | 높음 | 반영 완료 |
| R-C02-P003-03 | VB-C02-07을 모든 AXI-stream `tuser` bit map/변환/검증 monitor를 포함하는 sideband contract matrix로 확장 | 높음 | 반영 완료 |

## 4. Review v003 -> Plan v004 반영 위치 기록

| Review ID | Plan v004 반영 위치 |
|---|---|
| R-C02-P003-01 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v004.md` section 3.1 `C02 운용 범위`, section 6 `VB-C02-01`, section 9 `RK-C02-06`, section 10 `AP-C02-01` |
| R-C02-P003-02 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v004.md` section 6 `VB-C02-02`, section 6 `VB-C02-02 occupancy와 Datasheet Fill 구분`, section 9 `RK-C02-01` |
| R-C02-P003-03 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v004.md` section 6 `VB-C02-07`, section 6 `AXI-stream tuser contract matrix`, section 7.1 `AXI sideband integrity`, section 9 `RK-C02-10`, section 10 `AP-C02-13` |

| 항목 | 내용 |
|---|---|
| 반영된 계획 파일 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v004.md` |
| 반영 시각 | `2026-04-30 14:35:09 +09:00` |
