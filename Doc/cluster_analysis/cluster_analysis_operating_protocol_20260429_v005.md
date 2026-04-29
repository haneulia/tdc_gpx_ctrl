# Cluster Analysis Operating Protocol

문서 버전: `v005`  
작성일: `2026-04-29`  
최종 수정 시간: `2026-04-29 13:20:14 +09:00`  
작성 목적: `tdc_gpx_top` 및 하위 모듈 Cluster 분석을 진행할 때, 사용자와 Codex가 공유해야 하는 소통 방식, 기준 문서 우선순위, 근거 추적 방식, 컨텍스트 인계 절차를 고정한다.

---

## 1. 절대 기준

이 프로젝트의 기능 설명, timing 판단, 운용 개념 분석에서 최상위 기준은 다음 문서이다.

| 우선순위 | 기준 | 의미 |
|---:|---|---|
| 1 | `Doc/TDC-GPX-Datasheet.pdf` | 절대 기준. GPX IC의 pin 의미, bus timing, mode, register 의미, 금지 조건은 이 문서를 최우선으로 따른다. |
| 2 | RTL 구현 코드 | 데이터시트 요구가 현재 RTL에서 어떻게 구현되었는지 확인하는 대상이다. |
| 3 | testbench / 검증 코드 | 구현 의도를 보조적으로 확인하고, 실제 검증 범위와 누락을 판단하는 자료이다. |
| 4 | 코드 주석 / 기존 문서 | 참고 자료이다. 데이터시트와 충돌하면 데이터시트가 우선한다. |

운용 규칙:

- 코드 주석은 설명의 출발점이 될 수 있지만, 최종 판단의 절대 근거가 아니다.
- 데이터시트와 RTL이 다르게 보이면 “코드가 데이터시트를 어떻게 만족하거나 벗어나는지”를 명확히 분리해서 기록한다.
- 데이터시트 원문 page, figure, table, signal name을 문서에 남겨 사용자가 직접 추적할 수 있어야 한다.

---

## 2. 근거 추적 규칙

모든 Cluster 문서와 PPT는 사용자가 근거를 따라가서 확인할 수 있도록 작성한다.

### Markdown 근거 표기

Markdown 상세 문서에는 가능한 한 다음 형식으로 근거를 남긴다.

| 근거 종류 | 표기 방식 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf p.7 Figure 2 Read Operations`, `p.8 OEN operations`처럼 page와 항목명을 함께 표기 |
| RTL 코드 | `tdc_gpx_bus_phy.vhd:527-548`처럼 파일명과 line 범위 표기 |
| 상위 연결 | `tdc_gpx_config_ctrl.vhd:1445-1503`처럼 instance/port 연결 line 표기 |
| 검증 코드 | testbench 파일명, scenario 이름, line 범위 표기 |
| 분석 판단 | “데이터시트 기준”, “RTL 구현”, “추론”, “검증 필요”를 분리해서 표시 |

### PPT 근거 표기

PPT는 추상화된 도식과 핵심 메시지 중심으로 작성하되, 각 슬라이드 하단 또는 speaker note에 최소한의 추적 근거를 남긴다.

예:

- `근거: TDC-GPX-Datasheet.pdf p.7 / bus_phy:527-548 / cfg_pkg:266-305`
- `근거: TDC-GPX-Datasheet.pdf p.8 OEN operations`

### Review finding 표기

부실하거나 검증이 필요한 코드는 다음 요소를 함께 기록한다.

| 항목 | 의미 |
|---|---|
| Finding ID | 예: `F-C01-06` |
| 위험도 | 높음 / 중간 / 낮음 |
| 데이터시트 근거 | 해당 datasheet page/항목 |
| RTL 근거 | 파일명과 line |
| 문제 또는 리스크 | 확정 결함인지, 운용 제약인지, 검증 필요인지 구분 |
| 권장 조치 | 문서화, RTL 수정, CSR/SW 제한, simulation/assertion 등 |
| 다음 Cluster 인계 | 현재 Cluster에서 닫지 못하면 다음 Cluster의 검증 항목으로 명시 |

---

## 3. Cluster 진행 절차

Cluster 분석은 다음 순서로 진행한다.

1. Cluster 범위를 정의한다.
2. 해당 Cluster의 데이터시트 기준을 먼저 확인한다.
3. 데이터시트 기준을 RTL module / submodule / port / signal / FSM에 매핑한다.
4. 정상 운용 개념을 Markdown으로 상세하게 설명한다.
5. Mermaid 도표, 표, sequence/state diagram으로 논리 흐름을 기록한다.
6. 데이터시트 기준 대비 코드의 부실한 부분, 모호한 운용 계약, 검증 누락을 finding으로 정리한다.
7. PPT는 Markdown 내용을 발표/공유용으로 추상화해서 핵심 도식 중심으로 작성한다.
8. 산출물은 기존 파일을 덮어쓰지 않고 새 version 파일로 생성한다.
9. Cluster 분석이 끝나면 사용자 피드백을 기다린다.
10. 사용자 승인 또는 수정 피드백이 반영된 뒤 다음 Cluster로 넘어간다.

---

## 4. 추가 설계 분석 기준

### Module boundary 기준

모듈 경계는 가능한 한 FF 또는 register로 닫는다. 이 기준은 timing 분석을 단순화하고, Cluster 간 계약을 안정적으로 만들기 위한 것이다.

운용 규칙:

- 외부 모듈로 나가는 제어 신호는 가능하면 `_r` register에서 직접 출력한다.
- 다음 state를 만드는 큰 조합 process는 지양한다.
- 불가피한 조합 출력은 문서에 명시하고, timing risk 또는 설계 의도로 분류한다.
- async 입력은 synchronizer 또는 명시적 CDC primitive를 거친 뒤 내부 로직에 사용한다.
- bidirectional I/O, IOBUF, constant assignment처럼 불가피한 비등록 경계는 예외로 인정하되 근거를 남긴다.

### VHDL naming rule

VHDL 작성과 리뷰에는 다음 prefix/postfix 명명 규칙을 적용한다.

Prefix:

| 대상 | 규칙 |
|---|---|
| Input port | `i_` |
| Output port | `o_` |
| Inout port | `b_` |
| 내부 signal | `s_` |
| process variable | `v_` |
| constant | `c_` |
| generic | `g_` |
| type | `t_` |
| AXI slave interface | `s_axi_` |
| AXI master interface | `m_axi_` |

Postfix:

| 대상 | 규칙 |
|---|---|
| active-low signal | `_n` |
| enable | `_en` |
| registered signal | `_r` |
| next-state / next-value | `_next` |
| delayed signal | `_d1`, `_d2` |
| asynchronous signal | `_async` |

규칙에 명확히 포함되지 않는 경우:

- 위 규칙의 의미를 확장해 일관성 있게 적용한다.
- 반복적으로 필요한 새 규칙은 이 운영 프로토콜의 다음 버전에 추가한다.
- 기존 RTL이 규칙과 다를 경우 즉시 수정하지 않고, 해당 Cluster의 review finding 또는 naming debt로 기록한다.

---

## 5. 산출물 버전 규칙

산출물은 `Doc/cluster_analysis` 아래에 Cluster별 폴더를 만들고 저장한다.

예:

```text
Doc/cluster_analysis/
  cluster_analysis_communication_plan_20260429_v001.md
  cluster_analysis_operating_protocol_20260429_v001.md
  C01_GPX_Bus_Read/
    C01_GPX_Bus_Read_20260429_v001.md
    C01_GPX_Bus_Read_20260429_v001.pptx
    C01_GPX_Bus_Read_20260429_v002.md
    C01_GPX_Bus_Read_20260429_v002.pptx
```

규칙:

- 기존 산출물은 보존한다.
- 사용자 피드백이 반영되면 같은 Cluster 내에서 `v002`, `v003`처럼 새 파일을 만든다.
- 다음 Cluster는 별도 폴더를 만든다.
- PPT는 Markdown보다 더 추상적이고 핵심 도식 중심으로 만든다.

### Version lineage 기록 규칙

버전이 증가할 때는 새 버전에만 변경 내용을 쓰지 않는다. 직전 버전에도 “이 버전의 어떤 내용이 다음 버전 어디에 반영되었는지”를 추적 기록으로 남긴다.

운영 패턴:

1. `v001`을 검토해 `v002`를 만들면, `v001` 문서 끝에 `v001 -> v002 반영 위치 기록`을 추가한다.
2. `v002`를 검토해 `v003`를 만들면, `v002` 문서 끝에 `v002 -> v003 반영 위치 기록`을 추가한다.
3. 이 패턴을 모든 Cluster Markdown에 반복한다.
4. PPT는 발표용 산출물이므로 상세 lineage는 Markdown에 남기고, PPT에는 필요 시 핵심 변경만 요약한다.

기록 형식:

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | 사용자 피드백, 데이터시트 재검토, RTL 재검토, 검증 결과 등 |
| 반영된 다음 버전 파일 | 예: `C01_GPX_Bus_Read_20260429_v002.md` |
| 다음 버전 반영 위치 | section 번호/제목, finding ID, 필요 시 PPT slide 번호 |
| 판단 변화 | 기존 판단 유지, 수정, 폐기, 보류 중 무엇인지 |
| 추적 근거 | 데이터시트 page, RTL file/line, 사용자 피드백 요약 |

이 규칙의 목적:

- 사용자가 오래된 버전에서 출발해도, 어떤 검토가 다음 버전 어디에 반영되었는지 바로 따라갈 수 있게 한다.
- 문서 버전 간 변경 이유를 시간순으로 보존한다.
- Cluster 분석이 길어져도 판단 변경 이력을 잃지 않는다.

### PPT Timing Diagram 필수 규칙

Timing, handshake, CDC, bus transaction, FIFO drain처럼 시간 순서가 동작의 핵심인 Cluster PPT에는 Timing Diagram을 반드시 포함한다.

운영 규칙:

- 최소 1장 이상을 Timing Diagram 전용 slide로 만든다.
- READ/WRITE, valid/ready, flag sampling, CDC latency처럼 구분이 필요한 시간축은 별도 lane으로 표현한다.
- 각 lane에는 기준 clock, cycle/tick, signal polarity, capture 지점을 표시한다.
- Datasheet timing parameter가 관련되면 `tV-DR`, `tPW-RL`, `tS-EF`처럼 원문 기호를 그대로 표시한다.
- Timing Diagram slide 하단에는 데이터시트 page와 RTL file/line 근거를 남긴다.
- 단순 단계 카드만 있는 PPT는 timing 분석 산출물로 인정하지 않는다.

### Latency / Throughput / Pipeline / II 필수 분석 규칙

모든 Cluster 분석 결과에는 latency, throughput, pipeline, II(Initiation Interval)를 별도 항목으로 포함한다. 단순히 기능 흐름만 설명하면 운용 개념이 닫힌 것으로 보지 않는다.

운영 규칙:

- Markdown에는 `Latency / Throughput / Pipeline / II 분석` 또는 동등한 제목의 절을 둔다.
- Latency는 입력 event 기준점과 출력 관측 기준점을 분리한다. 예: `tick0->capture`, `capture->tvalid`, `CDC output valid`.
- Throughput은 이상적인 최대 처리율과 backpressure/CDC/FIFO/handshake 때문에 실제로 낮아질 수 있는 처리율을 분리한다.
- Pipeline은 stage, register boundary, FIFO/skid/CDC 위치, backpressure 전파 방향을 명시한다.
- II(Initiation Interval)는 다음 transaction/frame/beat/job을 시작할 수 있는 최소 간격으로 정의하고, burst/non-burst, sync/async, ready/stall 조건을 분리한다.
- Timing, handshake, CDC, bus, FIFO, frame drain과 관련된 Cluster는 Markdown에 Mermaid timing/block diagram 또는 table을 포함한다.
- PPT에는 Timing Diagram 또는 Timing Block Diagram을 반드시 포함하고, latency/throughput/pipeline/II가 한눈에 보이도록 도식화한다.
- 각 분석값에는 데이터시트 page, RTL file/line, clock/tick 가정, 산출 공식 또는 계산 근거를 함께 남긴다.

### 문서 수정 시간 기록 규칙

모든 새 Markdown 산출물은 header에 다음 metadata를 포함한다.

```text
문서 버전: `vNNN`
작성일: `YYYY-MM-DD`
최종 수정 시간: `YYYY-MM-DD HH:mm:ss +09:00`
```

운영 규칙:

- 새 버전을 만들 때마다 실제 작성/수정 시각을 `최종 수정 시간`에 기록한다.
- 기존 버전을 forward trace 목적으로 수정할 때도 해당 문서의 변경 이력 절에 변경 시각을 기록한다.
- 파일 시스템 timestamp만 믿지 않고, 문서 본문에서 사람이 읽을 수 있는 수정 시간을 남긴다.

---

## 6. 소통 방식

사용자와 Codex의 소통 방식은 다음을 유지한다.

- 한국어로 설명한다.
- 분석 중간에는 짧게 진행 상황과 배운 내용을 공유한다.
- 각 Cluster마다 “현재 범위”, “데이터시트 기준”, “RTL 구현”, “검토 결과”, “다음 Cluster로 넘길 계약”을 분리해서 설명한다.
- 사용자가 이해하고 검토할 수 있도록 상세하고 논리적으로 작성한다.
- Cluster가 끝나면 반드시 사용자 승인 또는 피드백을 받은 뒤 다음 Cluster로 넘어간다.
- 사용자와 나눈 중요한 피드백은 해당 Cluster Markdown의 “사용자 피드백 기록” 절과 이 운영 프로토콜의 “시간순 대화 기록”에 남긴다.

---

## 7. 컨텍스트 용량 관리 및 인계 절차

Codex가 현재 대화의 컨텍스트가 거의 찼다고 판단하면 다음 절차를 따른다.

### 보고 기준

- 남은 컨텍스트가 약 5% 이하로 떨어지기 전이라고 판단될 때
- 또는 다음 사용자 프롬프트를 처리하면 남은 컨텍스트가 5%보다 부족해질 가능성이 높다고 판단될 때

### 수행 절차

1. 사용자에게 “컨텍스트 용량이 임계치에 가까워졌다”고 명확히 보고한다.
2. `Doc/cluster_analysis` 아래에 새 handoff 문서를 만든다.
3. handoff 문서에는 다음을 기록한다.
   - 지금까지의 시간순 대화 요약
   - 확정된 프로젝트 규칙
   - 현재 Cluster와 산출물 위치
   - 완료된 분석
   - 열린 질문
   - 다음에 이어서 해야 할 작업
4. 사용자가 다음 섹션이나 새 대화에서 이어갈 수 있도록 파일 경로를 안내한다.

권장 파일명:

```text
Doc/cluster_analysis/context_handoff_YYYYMMDD_vNNN.md
```

주의:

- 컨텍스트 잔량은 정밀 계측값이 아니라 작업 중 판단해야 하는 운영 신호이다.
- 자동 요약 또는 컨텍스트 전환이 먼저 발생하면, 이어 받은 요약을 기준으로 즉시 continuity note를 확인하고 부족한 내용을 `Doc/cluster_analysis` 기록에서 복원한다.

---

## 8. 시간순 대화 기록

### 2026-04-29 초기 요청

사용자는 현재 작업 중인 `tdc_gpx_top` 모듈과 서브모듈에 대해 코드 분석된 PPT를 요청했다. 목적은 모듈 운용 개념을 상세히 이해하고, 부족한 운용 개념을 추가하며, 코드의 부실한 내용까지 검토해 보고하는 것이다.

### 2026-04-29 Cluster 방식 합의

사용자는 전체 pipeline을 한 번에 분석하기보다, GPX IC로부터 read하는 단계부터 합리적이고 논리적인 Cluster로 나누어 단계별로 검증하기를 요청했다. 각 Cluster는 분석 완료 후 사용자와 피드백을 주고받고, 승인 또는 수정 후 다음 Cluster로 확장한다.

기록 위치는 `Doc/cluster_analysis`로 합의했다.

### 2026-04-29 C01 생성

`C01_GPX_Bus_Read`를 첫 Cluster로 정의했다. 범위는 `tdc_gpx_bus_phy` 중심의 GPX 28-bit asynchronous parallel bus 접근 primitive이며, `tdc_gpx_config_ctrl`, `tdc_gpx_chip_ctrl`는 연결 계약 확인 범위로만 참조했다.

생성 산출물:

- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v001.md`
- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v001.pptx`

### 2026-04-29 C01 v002 보완

사용자는 C01의 기준 문서가 `Doc/TDC-GPX-Datasheet.pdf`이며, Bus_Phy sampling clock은 200 MHz라고 피드백했다.

반영 사항:

- C01의 절대 기준을 `Doc/TDC-GPX-Datasheet.pdf`로 변경했다.
- READ/WRITE/OEN timing을 데이터시트 p.7~8 기준으로 보완했다.
- Bus_Phy timing 기준을 `i_tdc_clk=200 MHz`, `T_clk=5 ns`로 고정했다.
- `EF1/EF2 active HIGH = empty FIFO` 및 empty FIFO read 금지를 C02 핵심 검증 계약으로 추가했다.
- 데이터시트 p.8, p.51의 16-bit mode 및 CSN bug report를 확인하고, 현재 RTL의 Reg14[4] 차단 정책을 유지해야 할 보호 조건으로 기록했다.

생성 산출물:

- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v002.md`
- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v002.pptx`

### 2026-04-29 운영 프로토콜 합의

사용자는 현재와 같은 대화 방식을 유지하고 절차를 기록할 것을 요청했다. 또한 이 프로젝트 전체에서 데이터시트를 코드 주석보다 상위의 절대 기준으로 삼고, 모든 문서에 근거를 명확히 남겨 사용자가 직접 추적할 수 있도록 제작할 것을 요청했다.

또한 컨텍스트 용량이 5% 근처로 줄어들기 전에 Codex가 명확히 보고하고, 대화 내용을 기록한 뒤 다음 섹션으로 이어갈 수 있도록 인계 문서를 만들 것을 요청했다.

본 문서가 이 합의를 기록한다.

### 2026-04-29 C01 v003 사용자 검토 반영

사용자는 C01 v001/v002의 READ timing 계산에서 `div=1,ticks=5`가 15 ns로 표시된 점을 검토했고, transaction 기준으로는 20 ns가 맞는 것으로 보인다고 지적했다. 또한 데이터시트 기준으로 GPX data bus 최대 readout 속도 40 MHz, 즉 25 ns보다 빠른 read가 가능한지 확인을 요청했다.

추가로 사용자는 OEN pin이 연결되지 않거나 pull-up/pull-down으로 고정되는 예외 상황을 generic으로 처리하는 방안, 모듈 내 조합논리 여부, 200 MHz main clock과 40 MHz 이하 status 신호 조건에서 2-FF synchronizer 충분성, 200 MHz GPX control clock과 150 MHz output Stream clock의 역할 분리를 검토 요청했다.

사용자는 새로운 분석 기준으로 모듈 경계를 FF/register로 닫는 원칙과 VHDL prefix/postfix naming rule을 추가했다. 본 v002 운영 프로토콜은 이 기준을 추가한다.

### 2026-04-29 Version lineage 규칙 추가

사용자는 v001에서 v002로, v002에서 v003로 변경이 이어질 때 이전 버전 문서에도 “다음 버전 어디에 수정 반영되었는지”를 남기도록 요청했다.

본 v003 운영 프로토콜은 `Version lineage 기록 규칙`을 추가한다. 즉, 앞으로 각 Cluster Markdown은 새 버전이 생성될 때 직전 버전 문서에도 forward trace를 남긴다.

### 2026-04-29 PPT Timing Diagram 필수 규칙 추가

사용자는 C01 v003 PPT에 실제 Timing Diagram이 부족해 이해와 소통에 문제가 있다고 피드백했다. 앞으로 다음 버전을 제시할 때는 Timing Diagram을 필수로 작성하도록 요청했다.

본 v004 운영 프로토콜은 `PPT Timing Diagram 필수 규칙`을 추가한다. 이 규칙은 C01 v004 PPT부터 즉시 적용한다.

---

## 9. 다음 진행 상태

현재 대기 상태:

- C01 v004는 Timing Diagram 필수화와 v003 review feedback을 반영해 작성되었다.
- C01 v001에는 `v001 -> v002` 반영 위치 기록을, C01 v002에는 `v002 -> v003` 반영 위치 기록을 추가했다.
- 다음 분석 후보는 `C02_Chip_Acquisition`이다.

### 2026-04-29 Latency / Throughput / Pipeline / II 및 수정 시간 규칙 추가

사용자는 C01 분석 마무리 단계에서 II(Initiation Interval) 분석을 추가하고, 앞으로 모든 결과마다 latency, throughput, pipeline, II를 분석하며 Timing Diagram 또는 Timing Block Diagram으로 Markdown과 PPT에서 이해할 수 있게 만들 것을 요청했다. 또한 문서에 수정 시간까지 포함하도록 운영 규칙에 추가해 달라고 요청했다.

본 v005 운영 프로토콜은 다음 규칙을 추가한다.

- `Latency / Throughput / Pipeline / II 필수 분석 규칙`
- `문서 수정 시간 기록 규칙`

이 규칙은 C01 v009부터 즉시 적용한다.
- C02는 데이터시트 기준으로 `EF1/EF2 active HIGH`, empty FIFO read 금지, IFIFO drain, capture/run FSM, force_reinit/bus_fatal 운용을 검증하는 방향으로 시작한다.

---

## v005 -> v006 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | C01_GPX_Bus_Read_Code_Verify_Review v003 처리 도중 사용자 지적 — “수정 계획 문서를 왜 규칙에 맞게 생성하지 않은거야?”. Codex가 review 동의/조치 계획을 채팅으로만 답하고 별도 Plan 문서를 cluster_analysis 폴더에 생성하지 않은 사실에 대한 운영 보강 요청. |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v006.md` |
| 다음 버전 반영 위치 | section 5 “산출물 버전 규칙” 안에 신규 sub-section `Review 처리 사이클 규칙 (v006 신규)` 추가. section 8 “시간순 대화 기록”에 “2026-04-29 Review 처리 사이클 규칙 추가 (v006)” 항목 추가. |
| 판단 변화 | v005까지의 lineage / timing diagram / latency-II / 수정 시간 규칙은 모두 그대로 유지. v006는 review-cycle 규칙 1개를 신규 추가. 향후 모든 사용자 review 수신 시 Plan 문서 작성을 코드 변경/회귀 진행 전 의무화. |
| 추적 근거 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v003.md` (사용자 review), `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v001.md` (retrospective Plan 문서, 신규 규칙의 첫 적용 사례), v006 본문 section 5 신규 sub-section |
