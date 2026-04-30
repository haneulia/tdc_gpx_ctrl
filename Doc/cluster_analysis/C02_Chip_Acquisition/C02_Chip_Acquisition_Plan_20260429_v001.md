# C02 Chip Acquisition Plan

문서 버전: `v001`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 20:34:21 +09:00`  
작성 목적: `C01_GPX_Bus_Read`에서 검증된 GPX bus READ primitive를 기반으로, `tdc_gpx_chip_ctrl` / `tdc_gpx_chip_run` 중심의 chip acquisition 운용 개념을 분석하기 위한 C02 시작 계획을 기록한다.

---

## 1. C02 시작 선언

사용자 요청:

> 좋아 C02 진행하자.

이에 따라 `Doc/cluster_analysis/C02_Chip_Acquisition` 폴더를 생성하고, C02 분석은 본 계획 문서를 기준으로 시작한다.

본 문서는 분석 계획이며, RTL 수정 또는 회귀 실행 결과 문서가 아니다. 코드 보완이 필요하다고 판단되면 운영 프로토콜 v006에 따라 별도 review / plan / 사용자 승인 / 실행 / result 순서로 진행한다.

---

## 2. 절대 기준과 우선순위

| 우선순위 | 기준 | C02 적용 방식 |
|---:|---|---|
| 1 | `Doc/TDC-GPX-Datasheet.pdf` | GPX IC의 `IrFlag`, `EF1/EF2`, `LF1/LF2`, `AluTrigger`, FIFO read 금지 조건, timing parameter를 최상위 기준으로 사용한다. |
| 2 | RTL 구현 | `tdc_gpx_chip_ctrl.vhd`, `tdc_gpx_chip_run.vhd`, `tdc_gpx_bus_phy.vhd`, `tdc_gpx_config_ctrl.vhd`가 데이터시트 기준을 어떻게 만족하는지 확인한다. |
| 3 | Testbench / log | `tb_tdc_gpx_chip_ctrl.vhd`와 기존 xsim 결과를 통해 실제 검증 범위와 누락을 판단한다. |
| 4 | 기존 주석 | 구현 의도를 파악하는 참고 자료로만 사용한다. 데이터시트와 충돌하면 데이터시트를 우선한다. |

고정 clock 가정:

| 항목 | 값 | 근거 |
|---|---:|---|
| GPX control / capture clock | `200 MHz`, `Tclk=5 ns` | 사용자 확정 기준 및 C01 문서 |
| GPX bus 최대 readout | `40 MHz`, `25 ns/word` 이상 간격 | C01 문서의 데이터시트 p.27 근거 |

---

## 3. C01에서 C02로 인계된 계약

| 인계 항목 | C02에서 확인할 내용 | 추적 근거 |
|---|---|---|
| Empty FIFO read 금지 | `EF1/EF2 active HIGH = empty` 조건에서 `chip_run`이 IFIFO read를 시작/지속하지 않는지 확인한다. | `C01_GPX_Bus_Read_20260429_v009.md` section 15, 항목 11 |
| `tS-EF max 11.8 ns` | 마지막 data read 후 EF가 empty를 반영하기까지의 지연과 2-FF synchronizer 지연을 drain stop 판단에 반영한다. | `C01_GPX_Bus_Read_20260429_v009.md` section 15, 항목 21~22 |
| Burst READ II | `bus_ticks * bus_clk_div * Tclk`가 최선 조건의 beat 간격이며, raw/downstream backpressure가 II를 늘리는지 확인한다. | `C01_GPX_Bus_Read_20260429_v009.md` section 15, 항목 29~31 |
| `o_rsp_pending` register boundary 영향 | C01 수정으로 pending 관측이 1 clock 늦어질 수 있으므로 `PH_RESP_DRAIN` exit 조건과 non-burst II 영향을 확인한다. | `C01_GPX_Bus_Read_20260429_v009.md` section 13 / section 15 |
| Stream clock 전략 | output stream clock은 `i_tdc_clk`와 별도 계약이다. C02에서는 raw stream이 어떤 clock domain에서 생성되고, C03 이후 CDC로 어디서 넘어가는지 경계를 확인한다. | `C01_GPX_Bus_Read_20260429_v009.md` section 15, 항목 20 |
| `div=1` 허용 계약 | `c_BUS_CLK_DIV_MIN=1` 전환 목표는 인정하되, `div=1=>ticks>=5` 또는 250 MHz 시 `bus_ticks * bus_clk_div >= 7` 계약이 acquisition path에서도 깨지지 않는지 확인한다. | `C01_GPX_Bus_Read_20260429_v009.md` section 15, 항목 19 / 23 |
| Backpressure / IrFlag 잔여 | C01 v005 결과가 C02로 넘긴 `F-C01-V06` 잔여 시나리오를 `chip_run`과 raw FIFO에서 확인한다. | `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` section 11 / 14 |

---

## 4. C02 분석 경계

### 포함 범위

| 구분 | 포함 대상 |
|---|---|
| 중심 RTL | `tdc_gpx_chip_ctrl.vhd`, `tdc_gpx_chip_run.vhd` |
| 연결 RTL | `tdc_gpx_bus_phy.vhd`, `tdc_gpx_config_ctrl.vhd`, `tdc_gpx_top.vhd` |
| 검증 자료 | `tb_tdc_gpx_chip_ctrl.vhd`, 사용자가 제공한 xsim 결과가 있으면 해당 로그 |
| 데이터시트 항목 | `IrFlag`, `EF1/EF2`, `LF1/LF2`, `AluTrigger`, IFIFO read, empty FIFO read 금지, timing parameter |

### 제외 또는 후속 범위

| 항목 | 후속 Cluster 후보 |
|---|---|
| Raw word semantic decode / hit-cell 변환 | C03 Raw Decode / Stream Decode |
| AXI stream output clock CDC 전체 구조 | C03 또는 C04 Output Stream CDC |
| 상위 laser_ctrl full integration regression | 별도 상위 integration Cluster |
| 보드 OEN default 최종 결정 | Board schematic 확인 후 별도 결정 |

---

## 5. 분석 순서

1. 데이터시트 기준을 먼저 정리한다.
   - `IrFlag`가 언제 valid acquisition 완료 신호인지 확인한다.
   - `EF1/EF2`, `LF1/LF2` polarity와 FIFO read 금지 조건을 확인한다.
   - `AluTrigger`와 `StopDis` 관련 timing/운용 의미를 확인한다.

2. RTL topology를 정리한다.
   - `chip_ctrl` coordinator phase: `PH_INIT`, `PH_IDLE`, `PH_RUN`, `PH_REG`, `PH_CFG_WRITE`, `PH_RESP_DRAIN`
   - `chip_run` sub-FSM: armed / capture / drain / ALU / timeout / overrun
   - bus request / response mux와 raw output FIFO 경계를 표시한다.

3. 정상 acquisition sequence를 timing diagram으로 표현한다.
   - start command
   - StopDis/armed
   - IrFlag 관측
   - expected count snapshot
   - IFIFO1 drain
   - IFIFO2 drain
   - drain_done control beat
   - PH_RESP_DRAIN

4. Latency / Throughput / Pipeline / II를 산출한다.
   - 기준점과 관측점을 분리한다.
   - 최선 조건과 backpressure 조건을 분리한다.
   - 200 MHz, `bus_ticks`, `bus_clk_div`, raw FIFO depth를 명시한다.

5. 코드 부실 또는 위험 항목을 review finding으로 기록한다.
   - 데이터시트 위반 가능성
   - EF/IrFlag timing 해석 불명확
   - empty FIFO read 위험
   - 조합 경계 / register boundary / CDC 위험
   - naming rule debt

6. C02 v001 Markdown 결과를 작성한다.
   - 상세 분석, 근거 표, Mermaid timing/block diagram 포함
   - 사용자 피드백 기록 절 포함

7. C02 v001 PPT를 작성한다.
   - 핵심 도식 중심
   - Timing Diagram 또는 Timing Block Diagram 필수 포함
   - 각 slide에 데이터시트/RTL 근거를 남긴다.

---

## 6. 이번 C02 v001의 완료 조건

| 조건 | 완료 기준 |
|---|---|
| 데이터시트 기준 | page / signal / timing parameter가 추적 가능하게 정리됨 |
| RTL mapping | 주요 FSM, bus handshake, raw stream boundary가 line 근거와 연결됨 |
| 운용 개념 | 정상 acquisition sequence가 단계별로 설명됨 |
| 정량 분석 | latency, throughput, pipeline, II가 별도 절로 포함됨 |
| Timing 도식 | Markdown에는 Mermaid 도식, PPT에는 timing diagram 또는 timing block diagram 포함 |
| Review | 부실 코드 또는 추가 검증 필요 항목을 finding으로 정리 |
| 다음 판단 | 코드 보완 필요 시 별도 C02 code-fix plan으로 넘길 수 있음 |

---

## 7. 사용자와의 소통 방식

- C02 분석 중 주요 판단이 갈리는 지점은 채팅에 먼저 짧게 공유한다.
- 근거는 반드시 문서에 기록하고, 사용자가 따라가 확인할 수 있도록 datasheet page와 RTL file/line을 남긴다.
- C02 분석 결과 v001이 생성되면 사용자의 검토/피드백을 받은 뒤 v002 또는 code-fix plan으로 진행한다.
- 코드 수정은 분석 문서에서 필요성이 정리된 뒤, 운영 프로토콜 v006의 review/plan/approval/result 절차에 맞춰 진행한다.

---

## 8. 현재 열린 질문

| ID | 질문 | C02에서 닫을 방법 |
|---|---|---|
| Q-C02-01 | `IrFlag`는 drain trigger로 충분한가, 아니면 sequence error detection과 분리되어야 하는가? | 데이터시트 pin/register 설명과 `chip_run` FSM 비교 |
| Q-C02-02 | `EF1/EF2` sync latency 때문에 마지막 word 이후 추가 empty read가 발생할 수 있는가? | `chip_run` drain loop와 `tS-EF + 2FF` timing 비교 |
| Q-C02-03 | expected IFIFO count가 0일 때 EF fallback이 안전한가? | count path, EF fallback branch, timeout branch 확인 |
| Q-C02-04 | raw FIFO / downstream backpressure가 bus burst II를 얼마나 늘리는가? | raw FIFO depth, `o_s_axis_tready`, `i_bus_rsp_pending`, `PH_RESP_DRAIN` 분석 |
| Q-C02-05 | `PH_RESP_DRAIN` hard cap / bus fatal / auto-recover 정책이 acquisition sequence를 안전하게 닫는가? | `chip_ctrl` phase FSM과 C01 pending 계약 비교 |

---

## 9. Version Lineage

| 항목 | 내용 |
|---|---|
| 직전 Cluster | `C01_GPX_Bus_Read` closure v005 승인 |
| 본 문서 | C02 시작 계획 v001 |
| 다음 산출물 | `C02_Chip_Acquisition_20260429_v001.md`, `C02_Chip_Acquisition_20260429_v001.pptx` |
| 판단 변화 | C01 bus primitive에서 C02 chip acquisition FSM / drain policy로 분석 범위 확장 |
| 추적 근거 | `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` section 11 / 14, `C01_GPX_Bus_Read_20260429_v009.md` section 15 |

### Plan v001 -> Analysis v001 반영 위치 기록

| 반영 대상 | 반영 위치 |
|---|---|
| 데이터시트 기준 우선 분석 | `C02_Chip_Acquisition_20260429_v001.md` section 2 |
| RTL topology / FSM mapping | `C02_Chip_Acquisition_20260429_v001.md` section 3 |
| 정상 acquisition sequence | `C02_Chip_Acquisition_20260429_v001.md` section 4 |
| Timing block diagram | `C02_Chip_Acquisition_20260429_v001.md` section 5, `C02_Chip_Acquisition_20260429_v001.pptx` slide 4 |
| Latency / Throughput / Pipeline / II | `C02_Chip_Acquisition_20260429_v001.md` section 6, PPT slide 5 |
| Code review finding | `C02_Chip_Acquisition_20260429_v001.md` section 7, PPT slide 6 |
| 사용자 확인 필요 | `C02_Chip_Acquisition_20260429_v001.md` section 9, PPT slide 7 |
