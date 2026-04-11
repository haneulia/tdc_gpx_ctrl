# TDC-GPX 구현 방안 비교 분석 및 최적 설계안

## Context

`tdc_gpx_dataflow_frame_vdma_deep_analysis_update13.md`(이하 "원본 설계서")를 바탕으로 두 가지 구현 단계 분해안이 존재한다:
- **방안 1** (Claude): `tdc_gpx_implementation_steps_1.md` — 모듈 중심 7-Step 분해
- **방안 2** (ChatGPT): `tdc_gpx_implementation_steps_2.md` + 10개 하위 문서 — 계약/vertical slice 중심 8-Step 분해

현재 HDL 디렉토리(`tdc_gpx_ctrl\HDL`)에는 VHDL 소스가 아직 없으며, 같은 프로젝트의 `laser_ctrl`, `echo_receiver` IP가 참조 아키텍처로 존재한다.

---

## 1. 구조 비교

| 항목 | 방안 1 (모듈 중심) | 방안 2 (계약/slice 중심) |
|------|---------------------|--------------------------|
| **분해 단위** | VHDL 모듈 = Step | 기능 계약 = Step (모듈 여러 개 묶음) |
| **Step 수** | 0~7 (8 steps) | 0~8 (9 steps) |
| **문서 구조** | 단일 문서 내 Step별 섹션 | 10개 독립 파일 + 공통 템플릿 |
| **Phase 1/2 경계** | 암묵적 (Step 7 이후 확장) | 명시적 분리 (Step 8 = Phase 2) |
| **pkg 정의** | Step 0에 tdc_gpx_pkg 단일 패키지 | acquisition/event/cell/frame/transport 5개 pkg |
| **검증 전략** | 각 모듈 단위 TB → 통합 순서 표 제공 | 각 Step 완료 조건 + 09_verification_plan |
| **CSR** | Step 0.2에 독립 정의 | 00_scope_and_contract에 계약으로 분리 |
| **bus_phy ↔ chip_ctrl** | Step 1, 2 별도 | Step 1에 함께 묶음 ("칩 bring-up") |
| **decode** | Step 3 독립 | Step 1에 얇게 포함, Step 2에서 raw_event 계약 완성 |
| **header/parser** | Step 6 (HW 관점) | Step 5 (SW 계약 중심, golden parser 포함) |
| **timing** | 본문에 산재 | Step 7 독립 문서 |
| **의존성 방향** | 모듈 간 port map 기반 | `acquisition → event → cell → frame → transport` 단방향 |

---

## 2. 각 방안의 강점

### 방안 1 강점
1. **모듈 = Step 1:1 대응**: RTL 엔지니어가 바로 VHDL 파일을 만들 수 있는 구체성
2. **port map 수준 명세**: 각 모듈의 입출력 신호, 제너릭이 명확
3. **통합 순서 표**: Phase별 모듈 조합과 TB 방법이 한눈에 보임
4. **testbench 시나리오 구체적**: 입력 갯수, 기대 출력, 통과 기준이 정량적
5. **bus_phy와 chip_ctrl 분리**: bus timing 검증을 chip_ctrl FSM과 독립적으로 먼저 할 수 있음
6. **tdc_gpx_pkg 단일 패키지**: 타입 정의 중복 방지, 의존성 단순

### 방안 2 강점
1. **계약(contract) 우선**: 모듈 경계의 "약속"을 먼저 고정 → 후속 변경 영향 최소화
2. **vertical slice 관점**: 각 Step 완료 시 관측 가능한 중간 산출물(raw word, raw_event, cell, line, DDR dump)이 명확
3. **SW parser 계약 명시**: HW-SW 인터페이스를 05번 문서로 독립 → parser 팀과 병렬 작업 가능
4. **Phase 2 경계 명시적**: PACKET_SCOPE=1, CELL_FORMAT=1을 08번 문서로 격리 → Phase 1 scope creep 방지
5. **timing을 마지막으로 분리**: 기능 먼저, 성능 나중 — 올바른 우선순위
6. **공통 템플릿**: 10개 문서가 동일 구조(책임/비책임/입력/출력/불변식/완료조건) → 일관성
7. **"비책임" 명시**: 각 문서가 다루지 않는 범위를 선언 → scope 혼동 방지

---

## 3. 각 방안의 약점

### 방안 1 약점
1. **계약 개념 부재**: 모듈 인터페이스는 있지만, "이 모듈이 보장하는 것 vs 보장하지 않는 것"이 불명확
2. **SW parser 계약 누락**: header 생성은 있지만 SW 측 golden parser / parse 규칙이 없음
3. **timing이 산재**: T_service, shot_interval 제약이 여러 Step에 흩어져 있음
4. **Phase 2 경계 모호**: Phase 2 관련 필드가 Step 0 pkg에 포함되지만, 언제 구현하는지 불명확
5. **문서가 하위 문서 없이 단일 파일**: 600줄 이상 시 관리 어려움
6. **raw_event_builder 누락**: decode_i만 있고, hit_seq_local 생성 등 raw_event 완성 모듈이 별도로 없음

### 방안 2 약점
1. **하위 문서가 뼈대만 있음**: 00~09 파일이 목차(heading)만 있고 내용이 비어 있음
2. **모듈 경계가 모호**: "Step 1 = bus_phy + chip_ctrl + cfg_image + 얇은 decode_i"처럼 묶여 있어, 개별 모듈 port map이 불명확
3. **5개 pkg 분리 과다**: VHDL에서 5개 패키지 의존성 체인은 compile order 관리가 복잡
4. **testbench 시나리오 부족**: "완료 조건"은 있지만 구체적 입력 벡터, 기대 출력, 정량 기준이 약함
5. **통합 순서 표 없음**: 모듈 조합별 통합 테스트 순서가 명시되지 않음
6. **bus_phy 독립 검증 기회 손실**: chip_ctrl과 묶여 있어 bus timing만 먼저 검증하기 어려움

---

## 4. 최적 설계안: 두 방안의 장점 융합

### 4.1 핵심 원칙 (방안 2에서 채택)

- **계약 우선**: 모듈 인터페이스의 "보장/비보장"을 먼저 확정
- **Phase 1/2 명시적 분리**: SINGLE_SHOT + SCOPE=0 전용으로 제한
- **vertical slice**: 각 Step 완료 시 관측 가능한 산출물 필수
- **timing은 마지막**: 기능 경로 먼저, 성능 최적화 나중
- **공통 문서 템플릿**: 책임/비책임/입력/출력/불변식/완료조건

### 4.2 모듈 분해 (방안 1에서 채택)

- **모듈 = VHDL 파일 1:1**: 각 Step에서 만드는 .vhd 파일이 명확
- **bus_phy와 chip_ctrl 분리**: bus timing 독립 검증 → 칩 bring-up 순서
- **단일 tdc_gpx_pkg**: 타입/상수를 한 곳에 정의 (5개 pkg 분리 불필요)
- **구체적 testbench 시나리오**: 입력 벡터, 기대 출력, 통과 기준 정량화
- **통합 순서 표**: 모듈 조합별 검증 순서 명시

### 4.3 추가 보강

- **raw_event_builder 모듈 추가**: decode_i(순수 조합) + raw_event_builder(hit_seq_local counter) 분리
- **SW parser 계약 문서**: header 포맷 + parse_cell() 규칙 + golden vector (방안 2의 05번)
- **"비책임" 선언**: 각 Step 문서에 "이 Step에서 안 하는 것" 명시 (방안 2 템플릿)
- **하위 문서 분리**: 10개 이하의 독립 파일로 분리 (방안 2 구조)

---

## 5. 최적 설계안 — Step 구성

```
Step 0: 공통 인프라
  파일: tdc_gpx_pkg.vhd, tdc_gpx_csr.vhd, tdc_gpx_shot_seq.vhd
  산출물: 컴파일 통과, CSR read/write 정상
  문서: 00_scope_and_contract.md (계약+용어+불변식+Phase 경계)

Step 1: 물리 버스 (bus_phy) — 독립
  파일: tdc_gpx_bus_phy.vhd, tb_bus_phy.vhd
  산출물: TDC behavioral model과 bus timing 검증 통과
  문서: 01_bus_phy.md
  비책임: chip_ctrl FSM, drain 전략

Step 2: 칩 제어 FSM (chip_ctrl) — bus_phy 위에 구축
  파일: tdc_gpx_chip_ctrl.vhd, tb_chip_ctrl.vhd
  산출물: SINGLE_SHOT cycle (ARMED→CAPTURE→DRAIN→ALU→ARMED) 재현
  문서: 02_chip_ctrl.md
  비책임: decode, cell, frame

Step 3: 디코딩 + raw_event 계약
  파일: tdc_gpx_decode_i.vhd (순수 조합), tdc_gpx_raw_event_builder.vhd (hit_seq counter)
  산출물: raw_event record 생성, stop_id_local/hit_seq_local 정확
  문서: 03_raw_event_contract.md
  비책임: cell 변환

Step 4: Cell Builder + Chip Slice
  파일: tdc_gpx_cell_builder.vhd, tb_cell_builder.vhd
  산출물: sparse→dense 변환, overflow 처리, chip slice 출력
  문서: 04_cell_builder.md
  비책임: multi-lane 조립

Step 5: Face Assembler (Packed Row)
  파일: tdc_gpx_face_assembler.vhd, tb_face_assembler.vhd
  산출물: 4-lane → 1 VDMA line, Packed Row, timeout blank
  문서: 05_face_assembler.md
  비책임: header, VDMA

Step 6: Header Inserter + SW Parser 계약
  파일: tdc_gpx_header_inserter.vhd, tb_header_inserter.vhd
  산출물: header line + data lines, SOF/EOL
  문서: 06_header_and_parser_contract.md (HW header + SW golden parser 규칙)
  비책임: VDMA transport

Step 7: VDMA Transport
  파일: axis_data_fifo 설정, m_axis_vdma_video wrapper
  산출물: DDR dump → SW parser 복원 정상
  문서: 07_vdma_transport.md

Step 8: Timing Budget (기능 완료 후)
  문서: 08_timing_budget.md
  내용: T_service 계산, pipeline 조건, double buffer

Step 9: Phase 2 (별도)
  문서: 08_phase2_packet_scope1.md
  내용: PACKET_SCOPE=1, CELL_FORMAT=1 — Phase 1 완료 후
```

### 5.1 통합 테스트 순서 (방안 1에서 채택)

```
Phase      모듈 조합                         검증 방법
──────────────────────────────────────────────────────────
Step 0     pkg + csr + shot_seq              단위 TB
Step 1     bus_phy                            TDC behavioral model TB
Step 2     bus_phy + chip_ctrl               1-chip bring-up TB
Step 3     decode_i + raw_event_builder      단위 TB (조합+counter)
Step 1~3   bus_phy + chip_ctrl + decode      raw_event 출력 확인 TB
Step 4     cell_builder                       단위 TB
Step 1~4   bus_phy→chip_ctrl→decode→cell     1-chip full path TB
Step 5     face_assembler                     4-lane stub TB
Step 6     header_inserter                    assembler stub TB
Step 5~6   assembler + header                통합 TB
Step 1~6   전체 datapath (4 lanes)           full system TB
Step 7     + VDMA + DDR                      board test / DDR dump
```

---

## 6. 문서 파일 구조 (최종)

```
tdc_gpx_ctrl/docs/
  00_scope_and_contract.md        ← 용어, 불변식, Phase 경계, 제너릭/CSR/latch 규칙
  01_bus_phy.md                   ← bus timing, INV-1~6, TDC behavioral model
  02_chip_ctrl.md                 ← SINGLE_SHOT FSM, drain 정책, EF/LF 규칙
  03_raw_event_contract.md        ← decode + raw_event 스키마, hit_seq_local
  04_cell_builder.md              ← sparse→dense, cell 구조, overflow
  05_face_assembler.md            ← Packed Row, 4-lane concat, timeout
  06_header_and_parser_contract.md ← header layout + SW parse 규칙 + golden vector
  07_vdma_transport.md            ← AXI4-Stream Video, VDMA 설정, DDR layout
  08_timing_budget.md             ← T_service, pipeline, double buffer
  08_phase2_packet_scope1.md      ← PACKET_SCOPE=1, CELL_FORMAT=1 (Phase 2)
  10_verification_plan.md         ← regression matrix, release checklist

tdc_gpx_ctrl/HDL/
  tdc_gpx_pkg.vhd                ← 단일 패키지 (타입, 상수, 함수)
  tdc_gpx_csr.vhd
  tdc_gpx_shot_seq.vhd
  tdc_gpx_bus_phy.vhd
  tdc_gpx_chip_ctrl.vhd
  tdc_gpx_decode_i.vhd
  tdc_gpx_raw_event_builder.vhd
  tdc_gpx_cell_builder.vhd
  tdc_gpx_face_assembler.vhd
  tdc_gpx_header_inserter.vhd
  tdc_gpx_top.vhd                ← structural top (모든 모듈 인스턴스)

  tb/
    tb_bus_phy.vhd               ← TDC behavioral model 포함
    tb_chip_ctrl.vhd
    tb_decode_i.vhd
    tb_cell_builder.vhd
    tb_face_assembler.vhd
    tb_header_inserter.vhd
    tb_full_system.vhd           ← 4-lane end-to-end
```

### 공통 문서 템플릿 (각 Step 문서에 적용)

```
# [Step N]. [모듈명]

## 0. 책임 (이 문서가 다루는 범위)
## 1. 비책임 (다음 Step으로 넘기는 범위)
## 2. 입력 (port map + 타입)
## 3. 출력 (port map + 타입)
## 4. 제너릭 파라미터
## 5. 동작 명세 (FSM/타이밍 다이어그램)
## 6. 불변식 [INV-N]
## 7. 오류 / timeout / recovery
## 8. testbench 시나리오 (입력 벡터, 기대 출력, 통과 기준)
## 9. 완료 조건
## 10. 다음 Step handoff contract
```

---

## 7. 방안 2의 개발 순서 상세 (추가 정보)

방안 2의 실제 개발 흐름은 **관측 가능한 산출물 체인**으로 구성된다:

```
00: 계약 잠금 (용어, 불변식, Phase 경계, 제너릭/CSR/latch 규칙)
01: 1-chip SINGLE_SHOT bring-up → 산출물: raw 28-bit word (ILA/UART로 관측)
02: raw word → raw_event 정규화 → 산출물: raw_event record (stop_id, hit_seq 정확)
03: dense cell + chip slice → 산출물: chip slice (stops_per_chip × cell_size)
04: Packed Row line → 산출물: packed line (rows_per_face × cell_size)
05: header + parser golden contract → 산출물: header + line (SW parser로 복원 가능)
06: AXI/VDMA/DDR dump → 산출물: DDR dump (SW parser가 읽음)
07: timing/pipeline 계산 (기능 완료 후)
08: Phase 2 확장 (PACKET_SCOPE=1, CELL_FORMAT=1)
09: verification plan (매 단계 완료 시 갱신)
```

**핵심**: `raw word → raw_event → chip slice → packed line → header+line → DDR dump`
각 단계에서 중간 산출물이 명확하므로, 문제 발생 시 어디서 깨졌는지 즉시 절단(isolation) 가능.

---

## 8. 최종 판단: 방안 2 프레임워크 + 방안 1 구체성 보강

### 8.1 개발 순서는 방안 2를 채택

방안 2의 vertical slice 순서가 더 우수한 이유:
1. **관측 가능성**: 매 Step마다 눈에 보이는 산출물이 존재 → 디버깅 경계가 명확
2. **계약 잠금 선행**: Step 0에서 용어/불변식/Phase 경계를 먼저 고정 → scope creep 방지
3. **칩 bring-up 통합**: bus_phy + chip_ctrl + 얇은 decode를 묶어 "칩이 실제로 동작하는가"를 먼저 확인 → 이것이 실무적으로 가장 위험도 높은 구간
4. **SW parser 계약 독립**: header 포맷과 parse 규칙을 독립 문서로 → HW/SW 병렬 작업

### 8.2 방안 1에서 보강할 요소

| 보강 항목 | 방안 1 출처 | 적용 방법 |
|-----------|------------|-----------|
| **bus_phy 독립 검증** | Step 1 | 01 내부에서 sub-step 1A(bus_phy 단독 TB) → 1B(+chip_ctrl 통합)로 세분화 |
| **단일 tdc_gpx_pkg** | Step 0 | 5개 pkg 대신 1개 pkg + 필요시 body 분리 |
| **구체적 TB 시나리오** | Step 1~6 | 각 문서에 입력 벡터, 기대 출력, 통과 기준 정량화 추가 |
| **통합 순서 표** | 통합 순서 표 | 09_verification_plan에 모듈 조합별 통합 테스트 순서 명시 |
| **port map 수준 명세** | Step 1~6 | 각 문서의 "입력/출력" 섹션에 VHDL port map 스켈레톤 추가 |

### 8.3 최종 구현 파일 구조

```
tdc_gpx_ctrl/HDL/
  src/
    tdc_gpx_pkg.vhd                ← 단일 패키지 (모든 타입, 상수, 함수)
    tdc_gpx_csr.vhd                ← AXI-Lite slave
    tdc_gpx_shot_seq.vhd           ← shot 순번 / col_in_face
    tdc_gpx_bus_phy.vhd            ← 물리 버스 FSM
    tdc_gpx_chip_ctrl.vhd          ← SINGLE_SHOT FSM
    tdc_gpx_decode_i.vhd           ← 28-bit → structured (순수 조합)
    tdc_gpx_raw_event_builder.vhd  ← hit_seq_local counter
    tdc_gpx_cell_builder.vhd       ← sparse → dense
    tdc_gpx_face_assembler.vhd     ← 4-lane → packed row
    tdc_gpx_header_inserter.vhd    ← header line + SOF/EOL
    tdc_gpx_top.vhd                ← structural top
  tb/
    tb_bus_phy.vhd                 ← TDC behavioral model
    tb_chip_ctrl.vhd               ← bus_phy stub
    tb_decode_raw_event.vhd        ← decode + raw_event_builder
    tb_cell_builder.vhd
    tb_face_assembler.vhd
    tb_header_inserter.vhd
    tb_full_datapath.vhd           ← 4-lane end-to-end

docs/
  00_scope_and_contract.md
  01_chip_acquisition.md           ← bus_phy + chip_ctrl (sub-step 1A/1B/1C)
  02_raw_event_contract.md
  03_cell_builder_and_chip_slice.md
  04_packed_row_and_face_assembly.md
  05_header_and_parser_contract.md
  06_vdma_transport.md
  07_timing_budget.md
  08_phase2_packet_scope1.md
  09_verification_plan.md
```

### 8.4 개발 착수 순서 (최종)

```
① 00_scope_and_contract.md 내용 채우기 (계약 잠금)
② tdc_gpx_pkg.vhd 작성 (타입/상수)
③ tdc_gpx_csr.vhd 작성 + TB
④ tdc_gpx_bus_phy.vhd 작성 + TDC behavioral model TB (sub-step 1A)
⑤ tdc_gpx_chip_ctrl.vhd 작성 + bus_phy 통합 TB (sub-step 1B)
⑥ 이후 raw_event → cell → assembler → header → VDMA 순서로 진행
```

---

## 9. 결론

| 채택 요소 | 출처 | 이유 |
|-----------|------|------|
| 개발 순서 (vertical slice) | 방안 2 | 관측 가능한 산출물 체인 → 디버깅 경계 명확 |
| 계약 우선, Phase 분리, 비책임 선언 | 방안 2 | scope 관리와 HW-SW 병렬 작업 |
| SW parser 계약 독립 문서 | 방안 2 | HW-SW 인터페이스 명확화 |
| 문서 템플릿 통일 | 방안 2 | 일관성 |
| timing 마지막 분리 | 방안 2 | 기능 먼저, 성능 나중 |
| bus_phy 독립 검증 (sub-step) | 방안 1 | bus timing이 가장 위험한 구간 → 먼저 분리 검증 |
| 단일 tdc_gpx_pkg | 방안 1 | compile order 단순성 |
| 구체적 TB 시나리오 + 통합 순서 표 | 방안 1 | 검증 실행 가능성 |
| port map 수준 명세 | 방안 1 | RTL 직결 구현 가능성 |
| raw_event_builder 추가 | 보강 | 원본 설계서의 hit_seq_local 로직 반영 |

**한 줄 요약: 방안 2의 "vertical slice 순서 + 계약 우선" 프레임워크를 기본 골격으로 채택하되, 방안 1의 "모듈 1:1 구체성 + bus_phy 독립 검증 + 단일 pkg + 정량적 TB"로 실행력을 보강한다.**
