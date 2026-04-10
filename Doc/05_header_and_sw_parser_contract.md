# 05. Header and SW Parser Contract

> 참조: 00_scope_and_contract.md, deep_analysis §9

---

## 0. 책임

- vdma_frame 첫 line에 metadata header 삽입 (HW)
- header만 보면 frame 전체를 해석할 수 있는 계약 수립 (HW-SW)
- SW parser 규칙 정의 (parse_header, parse_cell, row→chip/stop 복원)

**산출물**: header + data lines (SW parser로 구조 복원 가능)

---

## 1. 비책임

- packed line 생성 → 04번 문서
- VDMA transport (SOF/EOL timing) → 06번 문서

---

## 2. 입력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `s_axis_line` | AXI-Stream | assembler에서 data lines |
| `face_start` | 1 | vdma_frame 시작 |
| CSR fields | various | 모든 Control R/W (packet_start latch 값) |
| `shot_seq_start` | 32 | 이 frame의 첫 shot_seq |
| `timestamp_ns` | 64 | vdma_frame 시작 시각 |
| `lane_error_mask` | 4 | 누적 lane timeout bitmask |
| `lane_error_count[0..3]` | 8 each | lane별 누적 timeout 횟수 |

---

## 3. 출력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `m_axis_tdata` | TDATA_WIDTH | header + data lines |
| `m_axis_tvalid` | 1 | 유효 |
| `m_axis_tuser(0)` | 1 | SOF (header 첫 beat에서만 1) |
| `m_axis_tlast` | 1 | EOL (각 line 마지막 beat) |

---

## 4. header line 구조

header line 크기 = hsize_actual (data line과 동일).
유효 영역: 0x00~0x3F = 64 bytes. 나머지 0x40~end = 0 padding.

```
Offset  이름                크기      분류    설명
──────  ──────────────────  ────────  ──────  ──────────────────────
 ▶ 식별 (Identity)
 0x00   magic_word          4-byte    고정    b"TDCG" (바이트 시그니처)
 0x04   vdma_frame_id       32-bit    자동    vdma_frame 순번
 0x08   scan_frame_id       32-bit    자동    scan_frame 순번
 0x0C   face_id              8-bit    자동    face 번호 (0~N-1)
 0x0D   reserved_0d          8-bit    —       (reserved)
 0x0E   active_chip_mask     8-bit    CSR     [3:0] chip bitmask
 0x0F   n_faces              8-bit    CSR     미러 면 수

 ▶ 데이터 구조 (Structure)
 0x10   rows_per_face       16-bit    자동    actual (= active × stops, 가변)
 0x12   cols_per_face       16-bit    CSR     face당 shot 수
 0x14   max_hits_per_stop    8-bit    R/O     제너릭
 0x15   cell_size_bytes      8-bit    R/O     종속 상수
 0x16   hit_slot_width       8-bit    R/O     16 or 17
 0x17   hit_store_mode       8-bit    CSR     0=RAW, 1=CORR, 2=DIST

 ▶ 측정 설정 (Measurement)
 0x18   shot_seq_start      32-bit    자동    이 frame의 첫 shot_seq
 0x1C   bin_resolution_ps   16-bit    R/O     BIN (ps), 자동 산출
 0x1E   dist_scale           8-bit    CSR     0=1mm~4=2cm
 0x1F   drain_mode           8-bit    CSR     0=SrcGating, 1=Full

 ▶ TDC 보정 (Calibration)
 0x20   start_off1          32-bit    CSR     Reg5[17:0] (18-bit)
 0x24   reserved_24         32-bit    —       (reserved)
 0x28   reserved_28          8-bit    —       (reserved)
 0x29   n_drain_cap          8-bit    CSR     drain 읽기 제한
 0x2A   stops_per_chip       8-bit    CSR     칩당 활성 stop 수
 0x2B   pipeline_en          8-bit    CSR     0=순차, 1=pipeline

 ▶ 거리 변환 (Distance)
 0x2C   k_dist_fixed        32-bit    R/O     거리 변환 상수 (자동)

 ▶ 시각 (Timestamp)
 0x30   timestamp_ns        64-bit    자동    vdma_frame 시작 시각 (8-byte 정렬)

 ▶ 에러 정보 (Error Status)
 0x38   lane_error_mask      8-bit    자동    [3:0] timeout lane
 0x39   lane_error_cnt0      8-bit    자동    lane0 timeout 횟수
 0x3A   lane_error_cnt1      8-bit    자동    lane1
 0x3B   lane_error_cnt2      8-bit    자동    lane2
 0x3C   lane_error_cnt3      8-bit    자동    lane3

 ▶ HW 정보 (HW Discovery)
 0x3D   n_chips              8-bit    R/O     제너릭
 0x3E   stops_per_chip_max   8-bit    R/O     제너릭
 0x3F   format_scope         8-bit    R/O     [3:0] cell_format, [7:4] packet_scope

 0x40~  (reserved)           ...      —       0x00 padding
```

**모든 multi-byte 필드는 little-endian.**
**magic_word는 바이트 시그니처**: header[0:4] == b"TDCG" 로 검증.

---

## 5. SW parser 입력 계약

SW는 DDR에서 읽은 vdma_frame buffer만으로 데이터를 완전히 해석할 수 있어야 한다.
header 외부 정보(CSR 직접 읽기 등)에 의존하지 않는다.

### 5.1 parse_header()

```python
base   = vdma_frame_buffer_address
stride = vdma_stride  # 고정 2048
hdr    = read_memory(base, stride)

assert hdr[0:4] == b"TDCG", "bad magic"
rows = u16(hdr, 0x10)
cols = u16(hdr, 0x12)
cell_size = u8(hdr, 0x15)
active_chip_mask = u8(hdr, 0x0E)
stops_per_chip = u8(hdr, 0x2A)

active_chips = [i for i in range(n_chips) if (active_chip_mask >> i) & 1]
assert rows == len(active_chips) * stops_per_chip
```

### 5.2 parse_cell() (cell_format=0, Zynq-7000)

```python
def parse_cell(cell_bytes, max_hits=8, hit_slot_width=16):
    hit_slot = []
    offset = 0
    for i in range(max_hits):
        val = int.from_bytes(cell_bytes[offset:offset+2], 'little')
        hit_slot.append(val)
        offset += 2
    # offset = 16

    hit_valid = cell_bytes[offset]; offset += 1          # 16
    slope_vec = cell_bytes[offset]; offset += 1          # 17
    hit_count = cell_bytes[offset] & 0x0F; offset += 1   # 18 (lower nibble)
    meta_byte = cell_bytes[offset]; offset += 1           # 19
    hit_dropped = (meta_byte >> 0) & 1
    error_fill  = (meta_byte >> 1) & 1

    return CellData(hit_slot, hit_valid, slope_vec,
                    hit_count, hit_dropped, error_fill)
```

### 5.3 row → chip/stop 복원 (Packed Row)

```python
active_chips = [i for i in range(n_chips) if (active_chip_mask >> i) & 1]
chip_id  = active_chips[row // stops_per_chip]
stop_id  = row % stops_per_chip
```

---

## 6. 불변식

| 코드 | 규칙 |
|------|------|
| — | header에는 latch된 actual 값을 기록 (MAX 값이 아님) |
| — | CSR Status(R/O)에는 max 값을 노출 |
| — | header 유효 영역 ≤ 64 bytes, 나머지 = 0 padding |
| — | hsize_actual ≥ 64 보장 [INV-10] |

---

## 7. 오류 / timeout / recovery

| 상황 | SW 처리 |
|------|---------|
| magic mismatch | frame 폐기, error log |
| unsupported cell_format | frame 폐기 |
| lane_error_mask bit set | 해당 chip의 cell은 신뢰 불가 → skip |
| error_fill=1 | 해당 cell은 blank → point cloud 제외 |

---

## 8. testbench 시나리오

| 시나리오 | 입력 | 기대 출력 | 통과 기준 |
|----------|------|-----------|-----------|
| magic 확인 | header 생성 | 0x54,0x44,0x43,0x47 | byte-exact |
| header 필드 offset 전수 | 모든 CSR 값 설정 | 각 offset에 정확한 값 | bit-exact |
| rows_per_face actual | mask=0b0101, stops=4 | rows=8 | 값 일치 |
| SOF 위치 | header 첫 beat | tuser(0)=1 | 타이밍 정확 |
| EOL 위치 | 각 line 마지막 beat | tlast=1 | 위치 정확 |
| padding | 64B 이후 ~ hsize_actual | 0x00 | zero 확인 |

---

## 9. 완료 조건

- [ ] HW 없이도 header+payload dump만으로 parser가 구조 복원
- [ ] ROWS_PER_FACE_MAX와 actual 값 혼동 없음
- [ ] parser golden model이 RTL과 같은 header contract 사용
- [ ] 모든 header offset bit-exact 일치

---

## 10. 다음 단계 handoff contract

**이 Step의 출력 = 다음 Step(06)의 입력:**

```
m_axis_video : AXI-Stream (tdata, tvalid, tuser(0)=SOF, tlast=EOL)
              header line + cols_per_face data lines = VSIZE lines/frame
```

06번 문서는 이 stream을 axis_data_fifo를 거쳐 VDMA S2MM에 연결한다.
