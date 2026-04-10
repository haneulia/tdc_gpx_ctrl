# 00. Scope and Contract

> 원본 설계서: tdc_gpx_dataflow_frame_vdma_deep_analysis_update13.md
> 최종 갱신: 2026-04-10

---

## 0. 목적

이 문서는 TDC-GPX 4-Lane Multi-Hit FPGA 설계의 **최상위 계약**이다.
모든 하위 문서(01~09)는 이 문서의 용어, 불변식, Phase 경계를 공통 참조한다.

이 문서를 먼저 잠그고, 이후 구현 문서를 작성한다.

---

## 1. 설계 범위

### 1.1 Phase 1 (본 설계 대상)

- **SINGLE_SHOT 측정 모드** 전용 (데이터시트 §2.11.1)
- **PACKET_SCOPE = 0** (face 단위 VDMA 전송)
- Shot-Major 매핑 (고정)
- raw_event ~ cell ~ packed line ~ header ~ VDMA ~ DDR 전체 경로
- distance_conv
- 옵션 A 기본 (5면, 120° FOV, 2400 shots/face)

### 1.2 Phase 2 (별도 설계, 08번 문서)

- PACKET_SCOPE = 1 (scan_frame 단위 전송)
- CELL_FORMAT = 1 (MPSoC 64B)

---

## 2. 용어 정의

### 2.1 shot / face / vdma_frame / scan_frame

| 용어 | 정의 | 생성 주체 |
|------|------|-----------|
| **shot** | 하나의 레이저 발사~다음 발사 사이의 측정 구간. 1개의 start pulse에 대응 | laser_ctrl |
| **shot_seq** | 레이저 발사 순번. start pulse마다 1씩 증가하는 32-bit counter | tdc_gpx_shot_seq |
| **col_in_face** | 현재 face 안에서 이 shot이 몇 번째 수평 샘플인지 (0-based) | shot_seq_gen |
| **face** | 폴리곤 미러의 한 면. 한 face가 지나가는 동안 수평 화각 전체를 스캔 | 물리 |
| **vdma_frame** | 1 face 스캔의 전체 데이터 = VDMA의 SOF-to-SOF 단위 (SCOPE=0) | face_packet_assembler |
| **scan_frame** | 폴리곤 미러 1회전의 전체 데이터 = N_FACES개 vdma_frame | system level |

### 2.2 raw_event / cell / line / header

| 용어 | 정의 | 특성 |
|------|------|------|
| **raw_event** | decode된 한 건의 TDC hit 정보 | sparse, 발생한 것만 존재 |
| **cell** | vdma_frame 격자의 한 칸. 좌표 = {col_in_face, row_in_face}. MAX_HITS_PER_STOP개 hit slot 보유 | dense, 모든 활성 stop에 존재 |
| **line** | VDMA 관점에서 한 줄 = 1 segment = 1 shot의 모든 활성 수직 채널 pixel | rows_per_face개 cell |
| **header** | vdma_frame 첫 번째 VDMA line에 삽입되는 metadata | 64 bytes 유효 + padding |

### 2.3 TDC-GPX 칩 용어

| 용어 | 정의 |
|------|------|
| **chip_id** | 0~3. 어떤 TDC-GPX 칩인가 |
| **lane** | chip_id와 1:1 대응되는 독립 bus 경로 |
| **stop_id_local** | 칩 내부 stop 채널 번호. 0 ~ stops_per_chip-1 |
| **cha_code_raw** | Reg8/Reg9에서 읽은 2-bit 채널 코드 |
| **ififo_id** | 0 = IFIFO1(Reg8), 1 = IFIFO2(Reg9) |
| **raw_hit** | Stop - Start 시간 차이. 17-bit, BIN 단위 |
| **slope** | 에지 방향. 1=rising, 0=falling |
| **EF** | Empty Flag. HIGH active: EF=1이면 IFIFO 비어있음 |
| **LF** | Load-level Flag. HIGH active: LF=1이면 entries ≥ Fill 임계값 |
| **AluTrigger** | HIGH 레벨 펄스 (≥10ns) → master reset (MasterAluTrig=1일 때) |
| **IrFlag** | 인터럽트 플래그. MTimer 만료 시 active |

---

## 3. 독립변수 / 종속상수 / 런타임 산출값

### 3.1 제너릭 (합성 시 결정, 런타임 불변)

| VHDL generic | 기본값 | 의미 | 최소값 | 최대값 | 비고 |
|---|---|---|---|---|---|
| `N_CHIPS` | 2 | 병렬 칩 수 | 1 | 4 |  |
| `MAX_STOPS_PER_CHIP` | 8 | 칩당 최대 stop 수 | 1 | 8 | I-Mode |
| `MAX_HITS_PER_STOP` | 8 | cell 내 hit slot 수 | 1 | 8 |  |
| `HIT_SLOT_DATA_WIDTH` | 16 | hit slot 비트폭 | 16 | 17 | 16(Zynq-7000), 17(MPSoC) |
| `TDATA_WIDTH` | 32 | AXI bus Bit 폭 | 32 | 64 |  |
| `CELL_FORMAT` | 0 | 0=32B(Zynq-7000), 1=64B(Zynq-7000, MPSoC) | 0 | 1 |  |

### 3.2 종속 상수 (제너릭으로부터 자동 산출)

| VHDL constant | 산출식 | 기본값 | 최소값 | 최대값 | 비고 |
|---|---|---|---|---|---|
| `MAX_ROWS_PER_FACE` | N_CHIPS × MAX_STOPS_PER_CHIP | 16 | 1 | 32 |  |
| `CELL_SIZE_BYTES` | align_pow2(ceil((HIT_SLOT_DATA_WIDTH×MAX_HITS_PER_STOP + MAX_HITS_PER_STOP + MAX_HITS_PER_STOP + 4+1+1) / 8)) | 32 |  |  | 아래 산출 참조 |
| `MAX_VDMA_HSIZE_BYTES` | MAX_ROWS_PER_FACE × CELL_SIZE_BYTES | 1024 | 4 | 1024 | LiDAR 1 Point 정보 |
| `TDATA_BYTES` | TDATA_WIDTH / 8 | 4 |  |  |  |
| `BITS_PER_PIXEL` | CELL_SIZE_BYTES / TDATA_BYTES | 8 |  |  |  |
| `MAX_BITS_PER_LINE` | MAX_ROWS_PER_FACE × BITS_PER_PIXEL | 256 |  |  |  |

**CELL_SIZE_BYTES 산출 상세** (CELL_FORMAT=0, Zynq-7000 기본값 기준):

| 필드 | 단위(bit) | 개수 | 소계(bit) |
|------|-----------|------|-----------|
| hit_slot | HIT_SLOT_DATA_WIDTH (17) | MAX_HITS_PER_STOP (8) | 136 |  |
| hit_valid | 1 | MAX_HITS_PER_STOP (8) | 8 |
| slope_vec | 1 | MAX_HITS_PER_STOP (8) | 8 |
| hit_count_actual | 4 | 1 | 4 |
| hit_dropped | 1 | 1 | 1 |
| error_fill | 1 | 1 | 1 |
| **합계** | | | **158 bit** |

ceil(158/8) = 20 bytes → align_pow2(20) = **32 bytes** (CELL_SIZE_BYTES)

### 3.3 CSR (런타임 변경 가능, packet_start 시 latch)

모든 레지스터는 32-bit 정렬 (AXI-Lite 표준). 유효 비트 외 상위 비트는 0.

| CSR 이름 | Offset | 분류 | 유효 비트 | 의미 |
|---|---|---|---|---|
| `ACTIVE_CHIP_MASK` | 0x00 | R/W | [3:0] | chip 활성 bitmask |
| `STOPS_PER_CHIP` | 0x04 | R/W | [7:0] | 칩당 활성 stop 수 (2~MAX) |
| `COLS_PER_FACE` | 0x08 | R/W | [15:0] | face당 shot 수 (기본 2400) |
| `PACKET_SCOPE` | 0x0C | R/W | [7:0] | 0=face 단위, 1=scan_frame 단위 |
| `HIT_STORE_MODE` | 0x10 | R/W | [7:0] | 0=RAW, 1=CORRECTED, 2=DISTANCE |
| `DIST_SCALE` | 0x14 | R/W | [7:0] | 0=1mm ~ 4=2cm |
| `DRAIN_MODE` | 0x18 | R/W | [7:0] | 0=SourceGating, 1=FullDrain+LF |
| `N_DRAIN_CAP` | 0x1C | R/W | [7:0] | Capped Drain 읽기 제한 (0=무제한) |
| `PIPELINE_EN` | 0x20 | R/W | [7:0] | 0=순차, 1=double-buffer pipeline |
| `N_FACES` | 0x24 | R/W | [7:0] | 폴리곤 미러 면 수 (기본 5) |
| `BUS_CLK_DIV` | 0x28 | R/W | [7:0] | bus_clk 정수 분주기 (기본 1) |
| `BUS_TICKS` | 0x2C | R/W | [7:0] | ticks/transaction (3~6, 기본 5) |
| `STOPDIS_OVERRIDE` | 0x30 | R/W | [4:0] | [3:0] SW StopDis 제어, [4] enable |
| `SLICE_TIMEOUT_CLKS` | 0x34 | R/W | [7:0] | lane timeout (sys_clk cycles, 기본 200) |

TDC 설정 (R/W, cfg_image → 칩 write):

| CSR 이름 | Offset | 유효 비트 | 의미 |
|---|---|---|---|
| `START_OFF1` | 0x40 | [17:0] | Reg5 StartOff1 (18-bit) |
| `CFG_REG7` | 0x44 | [27:0] | HSDiv/RefClkDiv/MTimer. write 시 bin_ps, K_DIST 자동 갱신 |

cfg_image (R/W): 0x80~0xBC — Reg0~Reg14 × 32-bit each (16 regs × 4B = 64B)

Command (R/W):

| CSR 이름 | Offset | 유효 비트 | 의미 |
|---|---|---|---|
| `COMMAND` | 0xC0 | [3:0] | [0] start, [1] stop, [2] soft_reset, [3] cfg_write_trigger |

Status (R/O, 제너릭 상수 노출):

| CSR 이름 | Offset | 의미 |
|---|---|---|
| `HW_VERSION` | 0x100 | HW 버전 (major[31:16], minor[15:0]) |
| `HW_CONFIG` | 0x104 | N_CHIPS, STOPS_MAX, MAX_HITS, HIT_SLOT_DATA_WIDTH, TDATA, CELL_FORMAT packed |
| `MAX_ROWS_PER_FACE` | 0x108 | [15:0] 제너릭 최댓값 (stride/buffer 기준) |
| `CELL_SIZE_BYTES` | 0x10C | [15:0] 종속 상수 |
| `MAX_VDMA_HSIZE_BYTES` | 0x110 | stride 기준 최댓값 |

Status-Live (R/O, 실시간):

| CSR 이름 | Offset | 의미 |
|---|---|---|
| `STATUS` | 0x140 | [0] busy, [1] pipeline_overrun, [2] bin_mismatch, [7:4] lane_error_mask |
| `SHOT_SEQ_CURRENT` | 0x144 | 현재 shot_seq |
| `VDMA_FRAME_COUNT` | 0x148 | 완료된 vdma_frame 수 |
| `ERROR_COUNT` | 0x14C | 누적 에러 횟수 |
| `BIN_RESOLUTION_PS` | 0x150 | [15:0] Reg7로부터 자동 산출 |
| `K_DIST_FIXED` | 0x154 | 거리 변환 상수 (자동 산출) |

주소 영역: 0x00~0x154 (addr_width=9, 512-byte block)

### 3.4 packet_start latch 값

다음 파라미터는 **packet_start (= face_start, SCOPE=0)** 시점에 latch되며,
해당 vdma_frame이 끝날 때까지 변경 금지:

| 파라미터 | latch 시점 | 산출 | 이유 |
|---|---|---|---|
| `pkt_active_mask` | packet_start | CSR 복사 | rows_per_face 결정 |
| `pkt_stops_per_chip` | packet_start | CSR 복사 | rows_per_face 결정 |
| `pkt_cols` | packet_start | CSR 복사 | VSIZE 결정 |
| `rows_per_face` | packet_start | count_ones(mask) × stops | hsize_actual 결정 |
| `hsize_actual` | packet_start | rows_per_face × CELL_SIZE_BYTES | VDMA S2MM_HSIZE |

```vhdl
-- packet_start latch (동일 cycle 반영을 위해 variable 사용)
process(clk)
    variable v_rows : natural range 2 to MAX_ROWS_PER_FACE;
begin
    if rising_edge(clk) then
        if packet_start = '1' then
            pkt_active_mask    <= csr_active_chip_mask;
            pkt_stops_per_chip <= csr_stops_per_chip;
            pkt_cols           <= csr_cols_per_face;

            v_rows := count_ones(csr_active_chip_mask) * to_integer(csr_stops_per_chip);
            rows_per_face_reg <= v_rows;
            hsize_actual_reg  <= v_rows * CELL_SIZE_BYTES;
        end if;
    end if;
end process;
```

---

## 4. 최상위 데이터 경로

```
TDC-GPX chip × 4 (28-bit async parallel bus)
    │
    ▼
bus_phy ─────────── req/rsp 인터페이스
    │
    ▼
chip_ctrl ────────── SINGLE_SHOT FSM (ARMED→CAPTURE→DRAIN→ALU→ARMED)
    │
    ▼
decode_i ─────────── 28-bit raw word → structured fields (순수 조합)
    │
    ▼
raw_event_builder ── hit_seq_local 부여, raw_event record 생성
    │
    ▼
cell_builder ──────── sparse → dense (stops_per_chip개 cell)
    │
    ▼
chip_slice_out ────── stop 번호 오름차순 AXI-Stream 출력
    │
    ▼
face_packet_assembler ─ 4-lane → 1 VDMA line (Packed Row)
    │
    ▼
packet_header_inserter ─ header line + SOF/EOL
    │
    ▼
axis_data_fifo ────── VDMA backpressure 완충
    │
    ▼
AXI VDMA S2MM ────── DDR3 기록
```

의존성은 한 방향으로만 간다:
`acquisition → event → cell → frame → transport`

---

## 5. 핵심 불변식

### [INV-1] WRITE 중 OEN = 반드시 High
bus_phy: WRITE transaction에서 OEN 핀은 항상 '1'. 위반 시 bus contention.

### [INV-2] READ 중 FPGA D-bus = 반드시 input (Hi-Z)
bus_phy: READ transaction에서 FPGA는 D-bus를 drive하지 않음.

### [INV-3] IDLE 중 FPGA D-bus = input, OEN = High
bus_phy: IDLE 상태에서 양쪽 모두 Hi-Z.

### [INV-4] EF=1이면 Reg8/Reg9 read 금지
chip_ctrl: 빈 IFIFO를 읽으면 garbage. EF 확인 후에만 read.

### [INV-5] WRITE→READ: turnaround gap 필수
bus_phy: FPGA drive → chip drive 전환 시 최소 1 tick gap (양쪽 Hi-Z).

### [INV-6] READ→WRITE: OEN High 선행
bus_phy: chip drive 종료 후 OEN High → 다음 WRITE 시작.

### [INV-7] OEN permanent low 모드에서 WRITE 금지
bus_phy: OEN=0 유지 시 WRITE 불가. drain 구간 전용.

### [INV-8] PACKET_SCOPE=1: scan_frame 내 모든 face 동일 rows_per_face/hsize_actual
VDMA HSIZE 일관성 보장. (Phase 2 — 08번 문서)

### [INV-9] hsize_actual = rows_per_face × CELL_SIZE_BYTES
Packed Row 핵심 산출식.

### [INV-10] rows_per_face ≥ 2
header 64B ≤ hsize_actual 보장을 위한 최소값.
auto-clamp 규칙:
- active_chip_mask = 0x00 → 0x01 (최소 chip 1개)
- stops_per_chip < 2 → 2 (최소 stop 2개)
- stops_per_chip > MAX_STOPS_PER_CHIP → MAX

### [INV-11] hsize_actual ≤ MAX_VDMA_HSIZE_BYTES ≤ stride
VDMA stride 제약. stride = 2048 (고정).

### [INV-12] vdma_frame 진행 중 rows_per_face / hsize_actual 변경 금지
packet_start latch로 보장. CSR 변경은 다음 packet_start에서 적용.

---

## 6. Phase 분리 원칙

### 6.1 Phase 1 = SINGLE_SHOT + PACKET_SCOPE=0

| 경로 | Phase 1 |
|------|---------|
| chip_ctrl FSM | ARMED→CAPTURE→DRAIN→ALU 구현 |
| raw_event 생성 | 구현 |
| cell_builder | shot cycle = AluTrigger 기준 |
| face_packet_assembler | 구현 |
| header | 구현 |
| VDMA video path | 구현 |
| distance_conv | 구현 |

### 6.2 Phase 2 = PACKET_SCOPE=1, CELL_FORMAT=1

| 경로 | Phase 2 변경사항 |
|------|-----------------|
| face_packet_assembler | scan_frame 단위 전송 (SCOPE=1) |
| header | face 경계 마커 추가 |
| VDMA | VSIZE = N_FACES × (cols+1), DDR buffer 확대 |
| cell_builder | CELL_FORMAT=1 (64B, MPSoC) 지원 |

Phase 1과 Phase 2의 경계를 유지해야 문서가 다시 폭발하지 않는다.

---

## 7. 문서 용어 ↔ VHDL 식별자 매핑

| 문서 용어 | VHDL 식별자 | 비고 |
|---|---|---|
| rows_per_face | `rows_per_face_reg` | 런타임 가변, packet_start latch |
| MAX_ROWS_PER_FACE | `MAX_ROWS_PER_FACE` | 제너릭 상수 |
| cell_size | `CELL_SIZE_BYTES` | 종속 상수 |
| hsize_actual | `hsize_actual_reg` | 런타임 가변 |
| MAX_VDMA_HSIZE_BYTES | `MAX_VDMA_HSIZE_BYTES` | 종속 상수 |
| stop_id_local | `stop_id_local` | 0 ~ stops_per_chip-1 |
| chip_id | `chip_id` | 0 ~ N_CHIPS-1 |
| shot_seq | `shot_seq` | 32-bit counter |
| col_in_face | `col_in_face` | 0 ~ cols_per_face-1 |

**규칙:**
- 문서에서 `cell_size`라고 쓰면 VHDL의 `CELL_SIZE_BYTES`를 의미
- `stops_per_chip` (CSR, 가변)과 `MAX_STOPS_PER_CHIP` (제너릭, 고정)은 **다른 변수**
- `rows_per_face` (런타임, 가변)와 `MAX_ROWS_PER_FACE` (상수)는 **다른 변수**

---

## 8. Open Issue / 추후 확장 포인트

1. **OEN permanent low 모드**: SINGLE_SHOT drain 시 적용 여부 → 01번 문서에서 결정
2. **CELL_FORMAT=1 (MPSoC 64B)**: Phase 2에서 설계 → 08번 문서
3. **PACKET_SCOPE=1**: Phase 2에서 설계 → 08번 문서

---

## 9. 하위 문서 목록

| 번호 | 파일명 | 내용 | 산출물 |
|------|--------|------|--------|
| 01 | 01_chip_acquisition_single_shot.md | bus_phy + chip_ctrl | raw 28-bit word |
| 02 | 02_raw_event_contract.md | decode_i + raw_event_builder | raw_event record |
| 03 | 03_cell_builder_and_chip_slice.md | cell_builder + chip_slice_out | chip slice |
| 04 | 04_packed_row_and_face_assembly.md | face_packet_assembler | packed line |
| 05 | 05_header_and_sw_parser_contract.md | header_inserter + SW parser | header + line |
| 06 | 06_axi_vdma_ddr_transport.md | axis_data_fifo + VDMA | DDR dump |
| 07 | 07_timing_budget_and_pipeline.md | timing 계산 | T_service 검증 |
| 08 | 08_phase2_packet_scope1.md | PACKET_SCOPE=1, CELL_FORMAT=1 | Phase 2 설계 |
| 09 | 09_verification_plan.md | 검증 계획 | regression matrix |

---

## 10. 공통 문서 템플릿

각 하위 문서(01~08)는 다음 구조를 따른다:

```
## 0. 책임 (이 문서가 다루는 범위)
## 1. 비책임 (다음 단계 문서로 넘기는 범위)
## 2. 입력 (port map + 타입)
## 3. 출력 (port map + 타입)
## 4. 제너릭 파라미터
## 5. 동작 명세 (FSM / 타이밍 다이어그램 / 처리 순서)
## 6. 불변식 [INV-N]
## 7. 오류 / timeout / recovery
## 8. testbench 시나리오 (입력 벡터, 기대 출력, 통과 기준)
## 9. 완료 조건 (pass criteria)
## 10. 다음 단계 handoff contract
```
