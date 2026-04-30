# Cluster Analysis Operating Protocol

문서 버전: `v011`
작성일: `2026-05-01`
최종 수정 시간: `2026-05-01 01:07:17 +09:00`
작성 목적: `tdc_gpx_top` 및 하위 모듈 Cluster 분석/수정/검증 과정에서 사용할 운영 규칙을 갱신한다. 본 버전은 v010까지의 규칙을 유지하면서, Vivado/xsim 시뮬레이션 기준 경로와 합성 가능한 VHDL 작성 규칙을 추가한다.

---

## 1. 절대 기준

이 프로젝트의 기능 설명, timing 판단, 운용 개념 분석에서 최상위 기준은 다음 문서이다.

| 우선순위 | 기준 | 의미 |
|---:|---|---|
| 1 | `Doc/TDC-GPX-Datasheet.pdf` | 절대 기준. GPX IC의 pin 의미, bus timing, mode, register 의미, 금지 조건은 이 문서를 최우선으로 따른다. |
| 2 | RTL 구현 코드 | 데이터시트 요구가 현재 RTL에서 어떻게 구현되었는지 확인하는 대상이다. |
| 3 | testbench / 검증 코드 | 구현 의도를 보조적으로 확인하고, 실제 검증 범위와 누락을 판단하는 자료이다. |
| 4 | 코드 주석 / 기존 문서 | 참고 자료이다. 데이터시트와 충돌하면 데이터시트가 우선한다. |

운영 규칙:

- 코드 주석은 설명의 출발점일 수 있지만 최종 판단의 절대 근거가 아니다.
- 데이터시트와 RTL이 다르게 보이면 `코드가 데이터시트를 어떻게 만족하거나 벗어나는지`를 명확히 분리해서 기록한다.
- 데이터시트의 전문 page, figure, table, signal name 등 문서 내 앵커를 사용자가 직접 추적할 수 있어야 한다.

---

## 2. 근거 추적 규칙

모든 Cluster 문서와 PPT는 사용자가 근거를 따라가서 확인할 수 있도록 작성한다.

| 근거 종류 | 표기 방식 |
|---|---|
| 데이터시트 | `Doc/TDC-GPX-Datasheet.pdf p.7 Figure 2 Read Operations`처럼 page와 항목명을 함께 표기 |
| RTL 코드 | `tdc_gpx_bus_phy.vhd:527-548`처럼 파일명과 line 범위 표기 |
| 상위 연결 | `tdc_gpx_config_ctrl.vhd:1445-1503`처럼 instance/port 연결 line 표기 |
| 검증 코드 | testbench 파일명, scenario 이름, line 범위 표기 |
| 분석 판단 | `데이터시트 기준`, `RTL 구현`, `추론`, `추가 검증 필요`를 분리해서 표기 |

PPT는 Markdown보다 추상화된 핵심 도식 중심으로 작성하되, 각 슬라이드 하단 또는 speaker note에 최소한의 추적 근거를 남긴다.

---

## 3. Cluster 진행 절차

Cluster 분석은 다음 순서로 진행한다.

1. Cluster 범위를 정의한다.
2. 해당 Cluster의 데이터시트 기준을 먼저 확인한다.
3. 데이터시트 기준을 RTL module / submodule / port / signal / FSM에 매핑한다.
4. 정상 운용 개념을 Markdown으로 상세히 설명한다.
5. Mermaid 도표, timing diagram, timing block diagram, state relation diagram, data flow diagram으로 흐름을 기록한다.
6. 데이터시트 기준 대비 코드의 부실한 부분, 모호한 운용 계약, 검증 누락을 finding으로 정리한다.
7. Latency / Throughput / Pipeline / II(Initiation Interval)를 매 결과마다 분석한다.
8. PPT는 Markdown 내용을 발표/공유용으로 추상화하여 핵심 도형, 도식, 그림 중심으로 작성한다.
9. 산출물은 기존 파일을 덮어쓰지 않고 새 version 파일로 생성한다.
10. Cluster 분석이 끝나면 사용자 피드백을 받고, 반영이 끝난 뒤 다음 Cluster로 넘어간다.

---

## 4. 문서 버전 및 파일명 규칙

산출물은 `Doc/cluster_analysis` 아래에 Cluster별 폴더를 만들고 저장한다.

기본 규칙:

- 기존 산출물은 보존한다.
- 사용자 피드백이 반영되면 같은 Cluster 안에서 `v002`, `v003`처럼 새 파일을 만든다.
- 다음 Cluster는 별도 폴더를 만든다.
- 문서에는 작성 시간과 최종 수정 시간을 포함한다.
- 파일명에는 가능하면 `YYMMDDHHMMSS` timestamp를 포함한다.

예시:

```text
Doc/cluster_analysis/
  cluster_analysis_260501010717_operating_protocol_v011.md
  C02_Chip_Acquisition/
    C02_Chip_Acquisition_260430192448_Timing_Breakdown_v002.md
    C02_Chip_Acquisition_260430155825_Code_Verify_v001.pptx
```

수정 이력 연결 규칙:

- `v001`에서 지적된 내용이 `v002`에 반영되면, `v001`에도 `v002의 어느 항목에 반영되었는지`를 기록한다.
- `v002`에서 지적된 내용이 `v003`에 반영되면, `v002`에도 `v003의 어느 항목에 반영되었는지`를 기록한다.
- 이 패턴을 반복해서 문서 간 lineage를 추적 가능하게 유지한다.

---

## 5. RTL 설계 및 코딩 규칙

### 5.1 합성 가능한 VHDL 작성 규칙

RTL 파일은 합성 가능한 VHDL 코드로 작성한다.

허용 기준:

- production RTL은 FPGA synthesis에서 해석 가능한 VHDL 구문만 사용한다.
- clocked process 기반의 순차 논리를 기본으로 한다.
- reset, enable, state, handshake, CDC, FIFO push/pop, output boundary는 FF/register로 닫는 것을 원칙으로 한다.
- generic contract, static range check처럼 합성 흐름과 충돌하지 않는 elaboration-time 검사는 사용할 수 있다.
- simulation-only 진단이 RTL에 필요하면 `-- synthesis translate_off` / `-- synthesis translate_on`으로 명확히 감싼다.

금지 또는 제한 기준:

- RTL에서 `wait for`, `after`, file I/O, `textio`, `std.env`, testbench 전용 시간 지연 구문을 사용하지 않는다.
- latch가 추론되는 불완전 combinational assignment를 작성하지 않는다.
- multi-driver가 발생하는 signal 구조를 만들지 않는다.
- synthesis 결과가 불명확한 access type, protected type, unconstrained dynamic behavior를 RTL 제어 경로에 사용하지 않는다.
- testbench 전용 helper나 assertion style을 production RTL로 끌어오지 않는다.

구분:

- testbench는 비합성 VHDL 구문을 사용할 수 있다.
- RTL은 합성 가능한 코드만 허용한다.
- 공통 package라도 RTL에서 참조되는 package와 testbench 전용 package의 사용 범위를 구분한다.

### 5.2 Module boundary 기준

모듈 경계는 가능한 한 FF 또는 register로 처리해서 timing 분석이 쉽도록 만든다.

- 외부 모듈로 나가는 제어 신호는 가능하면 `_r` register에서 직접 출력한다.
- 다음 state를 만드는 조합 process는 지정하되, 결과는 같은 module 내부 register를 통해 닫는다.
- 불가피한 조합 출력은 문서에 명시하고 timing risk 또는 설계 의도로 분류한다.
- async 입력은 synchronizer 또는 명시된 CDC primitive를 거친 후 내부 로직에 사용한다.

### 5.3 조합논리 제한 및 순차논리 우선 규칙

VHDL RTL 작성 시 기본 구현 방식은 순차논리 기반으로 한다. 조합논리는 timing 분석을 어렵게 만들고 Cluster 경계 계약을 불명확하게 만들 수 있으므로 다음 조건에서만 제한적으로 허용한다.

| 구분 | 허용 여부 | 예시 |
|---|---|---|
| 1-depth | 허용 | register 출력의 단순 AND/OR, 단일 compare, 단일 mux |
| 2-depth | 조건부 허용 | compare 결과를 enable로 사용하는 단일 mux, valid와 ready의 단순 qualify |
| 3-depth 이상 | 금지 원칙 | compare -> priority select -> mask -> mux, nested if/case 다단 decode |

운영 규칙:

- 제어 경로, 상태 전이, handshake, FIFO push/pop 판단, CDC 이후 fan-out, module boundary 출력은 register를 통과한다.
- 2-depth를 초과하는 decode, priority, mux chain, compare chain은 pipeline register 또는 순차 FSM stage로 분리한다.
- 예외를 두면 Markdown 문서에 `조합논리 예외 근거`로 RTL line, 예상 depth, timing 영향, 예외 이유를 기록한다.

### 5.4 VHDL naming rule

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

- 위 규칙의 의도를 확장 적용한다.
- 반복적으로 필요한 새 규칙은 다음 운영 프로토콜 버전에 추가한다.
- 기존 RTL이 규칙과 다른 경우 즉시 대규모 rename을 하지 않고, 해당 Cluster review finding 또는 naming debt로 기록한다.

---

## 6. Testbench 및 시뮬레이션 규칙

### 6.1 Vivado/xsim 기준 경로

이 프로젝트에서 Vivado/xsim 시뮬레이션을 수행할 때 기준 Vivado 설치 경로는 다음으로 고정한다.

```text
C:\AMDDesignTools\2025.2.1\Vivado
```

운영 규칙:

- 별도 지시가 없으면 모든 Vivado/xsim 점검은 위 경로를 기준으로 한다.
- xsim 계열 실행 파일은 다음 위치를 기준으로 찾는다.

```text
C:\AMDDesignTools\2025.2.1\Vivado\bin\xvhdl.bat
C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat
C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat
```

- 시뮬레이션 문서에는 사용한 Vivado 경로, testbench 이름, 실행 script/log 경로, PASS/FAIL 기준을 함께 기록한다.
- 시뮬레이션 결과가 코드 수정 판단의 근거가 되면 해당 log 또는 report 파일명을 Cluster 문서에 추적 가능하게 남긴다.
- 이 경로가 변경되면 운영 프로토콜 새 버전으로 변경 이력을 기록한다.

### 6.2 AXI4-Lite testbench 공통 유틸리티 규칙

모든 테스트벤치에서 AXI4-Lite interface로 데이터를 쓰고 읽을 때 자체 transaction helper를 새로 만들지 않는다. `px_utility_pkg.vhd`에 정의된 공통 절차를 사용한다.

기준 API:

| 용도 | 공통 절차 | 근거 |
|---|---|---|
| AXI4-Lite write | `px_axi_lite_writer` | `px_utility_pkg.vhd` package/body |
| AXI4-Lite read | `px_axi_lite_reader` | `px_utility_pkg.vhd` package/body |

운영 규칙:

- TB 내부에서 AXI4-Lite valid/ready/bresp/rresp handshake를 직접 구현하는 transaction helper를 새로 만들지 않는다.
- 여러 CSR port bundle을 선택해야 하는 통합 TB에서는 domain별 wrapper를 둘 수 있다. 단, wrapper 내부는 `px_axi_lite_writer` 또는 `px_axi_lite_reader` 호출만 수행한다.
- 새 readback 기능이 필요하면 먼저 공통 package 확장 여부를 문서화하고 승인 후 반영한다.

---

## 7. Latency / Throughput / Pipeline / II 분석 규칙

매 Cluster 결과 문서에는 다음 분석을 포함한다.

| 항목 | 분석 내용 |
|---|---|
| Latency | 입력 이벤트 기준점에서 출력 또는 다음 제어원 전달까지의 cycle 수와 clock domain |
| Throughput | 정상 연속 운용에서 단위 시간 또는 cycle당 처리 가능한 데이터량 |
| Pipeline | stage별 register/FIFO/CDC 경계와 데이터 이동 순서 |
| II(Initiation Interval) | 새 transaction 또는 shot/frame을 시작할 수 있는 최소 간격 |
| Timing diagram/block | 위 정보를 사용자가 이해할 수 있도록 diagram으로 표현 |

분석에는 `데이터시트 기준`, `RTL 현재 구조`, `수정 후 구조`, `검증 결과`, `남은 위험`을 분리해서 기록한다.

---

## 8. Context 및 인계 규칙

대화 context 용량이 가득 차기 5% 이전이거나 다음 prompt에 5%보다 부족해질 것으로 예상되면 다음 절차를 수행한다.

1. 사용자에게 context 한계가 가까워졌음을 명확히 보고한다.
2. 현재까지의 대화, 결정, 미해결 항목, 파일/commit 위치를 시간순으로 기록한다.
3. 다음 섹션 또는 새 대화에서 이어갈 수 있도록 handoff 문서를 작성한다.
4. handoff 문서는 `Doc/cluster_analysis` 아래에 versioned Markdown으로 저장한다.

---

## 9. Git 관리 규칙

수정 및 보완이 발생할 때마다 Git으로 관리한다.

운영 규칙:

- 관련 있는 변경만 stage한다.
- 사용자 또는 이전 작업에서 생긴 unrelated dirty file은 임의로 되돌리지 않는다.
- 코드, testbench, 문서 변경은 가능한 한 의미 단위로 commit한다.
- commit 메시지는 변경 성격을 드러내는 짧은 문장으로 작성한다.
- 코드 수정이 시뮬레이션 결과와 연결되면 commit 전후로 검증 근거를 문서에 남긴다.

---

## 10. v010 -> v011 변경 이력

| 항목 | v011 반영 위치 | 변경 내용 |
|---|---|---|
| Vivado/xsim 기준 경로 추가 | `6.1 Vivado/xsim 기준 경로` | 시뮬레이션 기준 경로를 `C:\AMDDesignTools\2025.2.1\Vivado`로 고정 |
| 합성 가능한 VHDL 작성 규칙 추가 | `5.1 합성 가능한 VHDL 작성 규칙` | production RTL은 synthesizable VHDL만 허용하고, testbench 전용 구문과 simulation-only 진단의 사용 범위를 명확히 구분 |
| 기존 운영 규칙 유지 | 전체 문서 | v010까지의 데이터시트 우선, 근거 추적, timing diagram, latency/throughput/pipeline/II, 파일명, AXI4-Lite TB 유틸리티, Git 관리 규칙을 유지 |

