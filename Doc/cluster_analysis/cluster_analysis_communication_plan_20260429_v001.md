# TDC-GPX Cluster 단계별 분석 소통 계획

문서 버전: `v001`  
작성일: `2026-04-29`  
대상 경로: `C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL`  
목적: `tdc_gpx_top` 전체 파이프라인을 Cluster 단위로 나누어, GPX IC read 경로부터 단계적으로 검증하고 사용자 피드백을 반영하며 다음 Cluster로 확장한다.

---

## 1. 분석 진행 원칙

이번 분석은 전체 파이프라인을 한 번에 보고서화하는 방식이 아니라, GPX IC로부터 데이터를 읽어 오는 가장 앞단부터 Cluster를 하나씩 닫아 가는 방식으로 진행한다.

각 Cluster는 다음 조건을 만족해야 분석 완료로 본다.

1. Cluster의 입력, 출력, 내부 FSM, handshake, timeout, status 의미가 설명되어야 한다.
2. 해당 Cluster가 이전 Cluster와 어떤 계약으로 연결되는지 명확해야 한다.
3. 다음 Cluster가 의존해야 하는 신호, timing, status, 오류 조건이 정리되어야 한다.
4. 코드상 부실한 내용, 유지보수 위험, 문서 불일치가 별도 항목으로 기록되어야 한다.
5. 사용자 피드백이 문서에 반영되고, 새 버전 파일로 남아야 한다.

---

## 2. 산출물 규칙

각 Cluster마다 `Doc/cluster_analysis` 아래에 별도 폴더를 만든다.

예시:

```text
Doc/
  cluster_analysis/
    C01_GPX_Bus_Read/
      C01_GPX_Bus_Read_20260429_v001.md
      C01_GPX_Bus_Read_20260429_v001.pptx
      C01_GPX_Bus_Read_20260429_v002.md
      C01_GPX_Bus_Read_20260429_v002.pptx
```

파일은 덮어쓰지 않는다. 피드백 반영 시 `v002`, `v003`처럼 새 파일로 생성한다.

Markdown 문서는 상세 분석과 논리 기록용이다. PPT는 같은 내용을 더 추상적이고 핵심적인 도형, 흐름도, 구조도 중심으로 공유하기 위한 산출물이다.

---

## 3. Cluster 제안 구조

### C01_GPX_Bus_Read

GPX IC 물리 버스 접근 단계.

주요 대상:

- `tdc_gpx_bus_phy.vhd`
- TDC-GPX 28-bit async parallel bus
- IOBUF, read/write timing, turnaround
- status pin synchronizer
- AXI-Stream response mirror

핵심 질문:

- GPX IC에서 register/IFIFO 값을 읽는 기본 primitive가 timing상 안전한가?
- READ/WRITE 방향 전환에서 bus contention 가능성이 없는가?
- `bus_ticks`, `bus_clk_div` 운용 계약이 명확한가?
- response `tvalid/tready`가 상위 FSM에 어떤 보장을 제공하는가?

### C02_Chip_Acquisition

칩별 capture, drain, raw beat 생성 단계.

주요 대상:

- `tdc_gpx_chip_ctrl.vhd`
- `tdc_gpx_chip_run.vhd`
- `tdc_gpx_chip_init.vhd`
- `tdc_gpx_chip_reg.vhd`

핵심 질문:

- shot start 이후 capture, IrFlag, IFIFO drain, ALU pulse 흐름이 닫혀 있는가?
- drain timeout, mismatch, bus fatal, force reinit 정책이 운용 가능한가?
- raw data beat와 control beat가 downstream에 안전하게 전달되는가?

### C03_Raw_Decode_Event

GPX raw word를 구조화된 event로 바꾸는 단계.

주요 대상:

- `tdc_gpx_decode_pipe.vhd`
- `tdc_gpx_decoder_i_mode.vhd`
- `tdc_gpx_raw_event_builder.vhd`
- `tdc_gpx_skid_buffer.vhd`

핵심 질문:

- raw word field mapping이 I-Mode 계약과 일치하는가?
- drain_done, ififo_id, faulted flag가 손실 없이 전달되는가?
- stop_id, chip_id, shot_seq, hit_seq가 다음 단계에서 충분한 context를 제공하는가?

### C04_Cell_Build

slope 분기와 dense cell 생성 단계.

주요 대상:

- `tdc_gpx_cell_pipe.vhd`
- `tdc_gpx_cell_builder.vhd`

핵심 질문:

- rise/fall 분기가 backpressure와 함께 안전하게 동작하는가?
- dual buffer ownership FSM이 collect와 output을 충돌 없이 분리하는가?
- max_hits_cfg, stops_per_chip, timeout, quarantine 정책이 운용상 이해 가능한가?

### C05_Face_Output

chip slice를 row/frame/VDMA packet으로 조립하는 단계.

주요 대상:

- `tdc_gpx_output_stage.vhd`
- `tdc_gpx_face_assembler.vhd`
- `tdc_gpx_header_inserter.vhd`

핵심 질문:

- chip0..3 strict order row assembly가 VDMA/SW 파싱 계약과 맞는가?
- blank-fill, faulted row/frame, header metadata가 data 품질을 충분히 표현하는가?
- rise/fall output stream 분리 정책이 명확한가?

### C06_Control_Status_Integration

Top-level sequencing, CSR, status, recovery 통합 단계.

주요 대상:

- `tdc_gpx_top.vhd`
- `tdc_gpx_face_seq.vhd`
- `tdc_gpx_config_ctrl.vhd`
- `tdc_gpx_csr_pipeline.vhd`
- `tdc_gpx_csr_chip.vhd`
- `tdc_gpx_cmd_arb.vhd`
- `tdc_gpx_err_handler.vhd`
- `tdc_gpx_status_agg.vhd`
- `tdc_gpx_pkg.vhd`
- `tdc_gpx_cfg_pkg.vhd`

핵심 질문:

- 전체 start/stop/soft_reset/cfg_write/force_reinit 운용 절차가 닫혀 있는가?
- status bit, sticky, counter, clear semantic이 SW 입장에서 혼동 없이 설명되는가?
- Top glue에 unused signal, 문서 불일치, legacy port, VHDL flow 의존성이 남아 있는가?

---

## 4. Cluster별 소통 절차

각 Cluster는 다음 순서로 진행한다.

1. 분석 범위 확정
   - 대상 파일
   - 제외할 파일
   - 해당 Cluster에서 확인할 질문

2. `v001` Markdown 생성
   - 상세 설명
   - 코드 근거
   - 도표
   - 검증 포인트
   - 리스크 항목

3. `v001` PPT 생성
   - 핵심 흐름
   - 구조도
   - 상태 전이 요약
   - 인터페이스 계약
   - 주요 리스크

4. 사용자 리뷰
   - 이해가 안 되는 부분
   - 추가 설명이 필요한 운용 개념
   - 실제 설계 의도와 다른 해석
   - 더 깊게 볼 코드 구간

5. `v002` 생성
   - 사용자 피드백 반영
   - 변경 이력 추가
   - 이전 버전은 보존

6. Cluster 종료 판단
   - 사용자가 해당 Cluster 분석이 충분하다고 판단하면 다음 Cluster로 이동한다.

---

## 5. Markdown 문서 기본 목차

각 Cluster Markdown은 다음 목차를 따른다.

```text
# Cxx Cluster 이름

## 1. 이번 Cluster의 목적
## 2. 분석 대상 파일
## 3. Cluster 경계
## 4. 입력/출력 인터페이스
## 5. 정상 운용 시퀀스
## 6. 주요 FSM 또는 데이터 흐름
## 7. 상위/하위 Cluster와의 계약
## 8. 오류/timeout/recovery 동작
## 9. 상태 관측 및 SW 해석
## 10. 코드 리뷰 결과
## 11. 검증 체크리스트
## 12. 사용자 피드백 기록
## 13. 다음 Cluster로 넘길 계약
```

---

## 6. PPT 기본 구성

각 Cluster PPT는 8~12장 정도로 시작한다.

권장 슬라이드 구조:

1. Cluster 목적
2. 코드 범위
3. 입력/출력 block diagram
4. 정상 운용 flow
5. FSM 또는 timing 핵심
6. 오류/timeout/recovery flow
7. 다음 Cluster로 전달되는 계약
8. 코드 리뷰 findings
9. 검증 체크리스트
10. 사용자 논의 포인트

PPT는 문장을 길게 쓰기보다 도형, 화살표, 상태도, 신호 흐름 중심으로 작성한다.

---

## 7. 우선 피드백

첫 Cluster는 `tdc_gpx_bus_phy.vhd` 중심의 `C01_GPX_Bus_Read`로 시작하는 것이 좋다.

이유:

- GPX IC에서 읽어 오는 물리 primitive가 가장 앞단의 신뢰 기반이다.
- 여기서 read timing, response valid, bus direction, status sync 계약이 닫혀야 `chip_run` drain FSM을 논리적으로 검증할 수 있다.
- `chip_ctrl`까지 한 번에 묶으면 capture/drain 정책과 물리 bus timing이 섞여 Cluster 경계가 흐려진다.

따라서 `C01_GPX_Bus_Read`에서는 `bus_phy` 자체의 read/write 안전성과 timing 계약만 먼저 닫고, `C02_Chip_Acquisition`에서 `chip_ctrl/chip_run`이 그 primitive를 어떻게 사용하는지 확장하는 방식이 합리적이다.

---

## 8. 다음 작업 제안

다음 작업은 `C01_GPX_Bus_Read` 폴더를 생성하고, 아래 두 산출물을 `v001`로 작성하는 것이다.

```text
Doc/cluster_analysis/C01_GPX_Bus_Read/
  C01_GPX_Bus_Read_20260429_v001.md
  C01_GPX_Bus_Read_20260429_v001.pptx
```

`C01` 초안 작성 전에 확정할 질문:

1. `C01`을 `tdc_gpx_bus_phy.vhd` 단독으로 볼지, `chip_ctrl`의 bus request/response port map까지 포함할지?
2. PPT는 8장 내외의 간결한 공유용으로 할지, 12장 이상의 검토 회의용으로 할지?
3. 검증 관점을 simulation 중심으로 둘지, timing/spec audit 중심으로 둘지?
