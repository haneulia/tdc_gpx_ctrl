# 04. Packed Row and Face Assembly

> 참조: 00_scope_and_contract.md, deep_analysis §8

---

## 0. 책임

4개 lane(chip)의 chip slice를 1개 VDMA line으로 합치는 Packed Row 조립:
- 활성 chip slice만 수집, chip 번호 오름차순 concat
- Per-lane timeout 처리, blank slice 주입
- rows_per_face, hsize_actual 산출 및 적용

**산출물**: packed line (rows_per_face × cell_size bytes)

---

## 1. 비책임

- chip slice 생성 → 03번 문서
- header 삽입 → 05번 문서
- VDMA transport → 06번 문서

---

## 2. 입력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `s_axis_chip_slice[0..3]` | AXI-Stream × 4 | 4개 lane의 chip slice |
| `slice_ready[0..3]` | 4 | 각 lane의 slice 완료 플래그 |
| `pkt_active_mask` | 4 | 활성 chip bitmask (packet_start latch) |
| `pkt_stops_per_chip` | 4 | 칩당 활성 stop 수 (packet_start latch) |
| `shot_seq` | 32 | 현재 shot 순번 |

---

## 3. 출력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `m_axis_tdata` | TDATA_WIDTH | packed line 데이터 |
| `m_axis_tvalid` | 1 | 유효 |
| `m_axis_tlast` | 1 | line 마지막 beat (EOL) |
| `line_ready` | 1 | 1 shot의 packed line 출력 완료 |
| `lane_error_flags` | 4 | 현재 shot에서 timeout된 lane bitmask |
| `lane_error_count[0..3]` | 8 each | vdma_frame 내 각 lane 누적 timeout 횟수 |

---

## 4. Packed Row 정책

### 4.1 비활성 chip의 row는 존재하지 않음

```
active_chip_mask=0b1010 (chip 1,3 활성), stops_per_chip=4:

  row 0: chip 1, stop 0
  row 1: chip 1, stop 1
  row 2: chip 1, stop 2
  row 3: chip 1, stop 3
  row 4: chip 3, stop 0
  row 5: chip 3, stop 1
  row 6: chip 3, stop 2
  row 7: chip 3, stop 3

  rows_per_face = 2 × 4 = 8 (32가 아님)
  hsize_actual = 8 × 32 = 256 bytes (1024가 아님)
```

### 4.2 actual vs max 구분

| 변수 | 의미 | 가변? |
|------|------|-------|
| `rows_per_face` | count_ones(active_mask) × stops_per_chip | 가변 (packet_start latch) |
| `ROWS_PER_FACE_MAX` | N_CHIPS × STOPS_PER_CHIP_MAX = 32 | 고정 (제너릭) |
| `hsize_actual` | rows_per_face × CELL_SIZE_BYTES | 가변 |
| `HSIZE_MAX` | ROWS_PER_FACE_MAX × CELL_SIZE_BYTES = 1024 | 고정 |

---

## 5. 동작 명세

### 5.1 매 shot 동작

```
1. timeout counter 시작 (lane별)
2. 대기: slice_ready = active_chip_mask OR timeout 발생
3. 준비된 lane → 실제 slice 사용
   timeout된 lane → blank slice (error_fill=1) 주입
4. concat 출력:
   활성 chip 번호 오름차순으로 slice를 이어 붙임
   비활성 chip은 건너뜀 (concat에 참여하지 않음)
5. lane_error_flags 갱신
6. 다음 shot 대기
```

### 5.2 Per-Lane Timeout

```
SLICE_TIMEOUT_CLKS: CSR 0x0F (R/W, 8-bit, 기본 200 sys_clk cycles)

shot 시작 → lane별 timeout counter 개시
slice_ready[n] 도착 → 해당 lane counter 정지
timeout 도달 → 해당 lane TIMEOUT 판정 → blank slice 주입
```

### 5.3 Error-Filled Blank Slice

```
blank_slice (stops_per_chip개 cell):
  모든 hit_slot = 0
  모든 start_num = 0
  hit_valid = 0x00
  hit_count_actual = 0
  hit_dropped = 0
  slope_vec = 0x00
  error_fill = 1    ← assembler가 주입한 가짜 데이터
```

SW 식별: cell.error_fill == 1 → skip (point cloud 제외)

### 5.4 AXI-Stream 출력 (beat 단위)

```
32-bit tdata, rows_per_face=32 기준:

  beat 0~7:     chip0-stop0 cell (8 beats × 4B = 32B)
  beat 8~15:    chip0-stop1 cell
  ...
  beat 56~63:   chip0-stop7 cell
  beat 64~71:   chip1-stop0 cell
  ...
  beat 248~255: chip3-stop7 cell ← tlast = 1 (EOL)
```

---

## 6. 불변식

| 코드 | 규칙 |
|------|------|
| [INV-9] | hsize_actual = rows_per_face × CELL_SIZE_BYTES |
| [INV-10] | rows_per_face ≥ 2 (auto-clamp) |
| [INV-11] | hsize_actual ≤ HSIZE_MAX ≤ stride |
| [INV-12] | vdma_frame 진행 중 rows_per_face / hsize_actual 변경 금지 |
| — | VDMA line 크기는 항상 rows_per_face × cell_size (timeout lane도 blank으로 채움) |

---

## 7. 오류 / timeout / recovery

| 상황 | 감지 | 처리 |
|------|------|------|
| lane timeout | timeout counter expired | blank slice 주입, lane_error_flags bit set |
| 모든 chip timeout | slice_ready = 0x0 at timeout | 전체 blank line |
| active_mask=0 | packet_start 시 | auto-clamp to 0x01 [INV-10] |

---

## 8. testbench 시나리오

| 시나리오 | 입력 | 기대 출력 | 통과 기준 |
|----------|------|-----------|-----------|
| 4 chip 활성, 8 stops | 4 slices, all ready | rows=32, hsize=1024 | line 크기 정확 |
| 2 chip 활성 (mask=0b0101), 4 stops | 2 slices | rows=8, hsize=256 | Packed Row 정확 |
| 1 chip timeout | 3 ready + 1 timeout | blank slice 위치 정확 | error_fill=1 |
| 모든 chip timeout | 0 ready | 전체 blank line | lane_error_mask=0xF |
| 연속 2400 shots | 반복 | face 완료 | beat count 누적 정확 |

---

## 9. 완료 조건

- [ ] 활성 mask에 따라 line 폭이 달라짐
- [ ] 비활성 chip/stop row가 실제로 존재하지 않음
- [ ] line 크기 = rows_per_face × cell_size
- [ ] timeout lane의 blank slice에 error_fill=1
- [ ] chip 번호 오름차순 concat 확인

---

## 10. 다음 단계 handoff contract

**이 Step의 출력 = 다음 Step(05)의 입력:**

```
m_axis_line      : AXI-Stream (tdata, tvalid, tlast=EOL)
line_ready       : 1-bit
lane_error_flags : 4-bit (현재 shot timeout bitmask)
lane_error_count : 4 × 8-bit (vdma_frame 내 누적)
rows_per_face    : latched value
hsize_actual     : latched value
```

05번 문서는 이 line stream 앞에 header line을 삽입하고 SOF를 부여한다.
