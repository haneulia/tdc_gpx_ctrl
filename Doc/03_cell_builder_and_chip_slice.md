# 03. Cell Builder and Chip Slice

> 참조: 00_scope_and_contract.md, deep_analysis §7

---

## 0. 책임

sparse raw_event stream을 dense cell array로 변환하고, chip slice로 출력:
- cell_builder: raw_event → cell 격자에 저장, MAX_HITS 제한, zero-padding
- chip_slice_out: cell[0]~cell[stops_per_chip-1]을 stop 번호 오름차순으로 AXI-Stream 출력

**산출물**: chip slice (stops_per_chip × cell_size bytes)

---

## 1. 비책임

- decode / raw_event 생성 → 02번 문서
- multi-lane 조립 (face assembler) → 04번 문서

---

## 2. 입력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `raw_event` | record | t_raw_event (02번 문서 §4) |
| `raw_event_valid` | 1 | raw_event 유효 |
| `drain_done` | 1 | 현재 shot drain 완료 → cell 출력 개시 |
| `shot_start` | 1 | 새 shot 시작 → cell buffer clear |

---

## 3. 출력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `m_axis_tdata` | TDATA_WIDTH | cell 바이트 데이터 (beats_per_pixel beats/cell) |
| `m_axis_tvalid` | 1 | 유효 데이터 |
| `m_axis_tlast` | 1 | chip slice의 마지막 beat |
| `slice_ready` | 1 | chip slice 출력 완료 (assembler에게) |

---

## 4. cell 구조 (single source of truth)

```
t_cell = record
    -- hit data (MAX_HITS_PER_STOP개)
    hit_slot      : array(0 to MAX_HITS_PER_STOP-1) of unsigned(HIT_SLOT_DATA_WIDTH-1 downto 0)

    -- cell-level metadata
    hit_valid         : std_logic_vector(MAX_HITS_PER_STOP-1 downto 0)   -- bitmask
    slope_vec         : std_logic_vector(MAX_HITS_PER_STOP-1 downto 0)   -- bitmask
    hit_count_actual  : unsigned(3 downto 0)                              -- 0~8
    hit_dropped       : std_logic                                         -- MAX 초과 여부
    error_fill        : std_logic                                         -- timeout blank cell
end record;
```

### cell_size 산출 (cell_format=0, Zynq-7000 기본)

```
hit_slot:     16 × 8 = 128 bit
hit_valid:              8 bit
slope_vec:              8 bit
hit_count_actual:       4 bit
hit_dropped:            1 bit
error_fill:             1 bit
────────────────────────────
총 payload:           150 bit = 19 bytes → 정렬 32 bytes (CELL_SIZE_BYTES)
```

---

## 5. 동작 명세

### Phase A: 초기화

```
shot_start 시:
  for i in 0..stops_per_chip-1:
    cell_buffer[i].hit_slot[*]      = 0
    cell_buffer[i].hit_valid        = 0x00
    cell_buffer[i].slope_vec        = 0x00
    cell_buffer[i].hit_count_actual = 0
    cell_buffer[i].hit_dropped      = 0
    cell_buffer[i].error_fill       = 0
    hit_counter[i]                  = 0
```

### Phase B: 수집

```
raw_event 도착 시:
  row = raw_event.stop_id_local
  seq = raw_event.hit_seq_local

  if hit_counter[row] < MAX_HITS_PER_STOP:
    cell_buffer[row].hit_slot[seq]      = raw_event.raw_hit(HIT_SLOT_DATA_WIDTH-1 downto 0)
    cell_buffer[row].hit_valid(seq)     = '1'
    cell_buffer[row].slope_vec(seq)     = raw_event.slope
    cell_buffer[row].hit_count_actual  += 1
  else:
    cell_buffer[row].hit_dropped       = '1'

  hit_counter[row] += 1
```

**HIT_SLOT_DATA_WIDTH truncation**: raw_hit는 17-bit이지만 HIT_SLOT_DATA_WIDTH=16이면 하위 16-bit만 저장.

### Phase C: 출력

```
drain_done 수신 후:
  for row in 0..stops_per_chip-1:
    cell_buffer[row]를 AXI-Stream으로 출력
    (beats_per_pixel beats per cell, stop 오름차순)
  slice_ready = '1'
```

---

## 6. 불변식

| 코드 | 규칙 |
|------|------|
| — | cell_buffer 크기 = stops_per_chip (CSR, packet_start latch) |
| — | 비활성 stop의 cell은 존재하지 않음 (Packed Row) |
| — | hit가 없는 slot은 0 (Phase A 초기화로 보장) |
| — | hit_count_actual ≤ MAX_HITS_PER_STOP |
| — | error_fill은 cell_builder가 설정하지 않음 (assembler가 blank cell에만 설정) |

---

## 7. 오류 / timeout / recovery

| 상황 | 감지 | 처리 |
|------|------|------|
| MAX_HITS 초과 | hit_counter ≥ MAX | 저장 안 함, hit_dropped = 1 |
| stop_id 범위 초과 | stop_id ≥ stops_per_chip | raw_event 폐기 |
| drain_done 누락 | timeout counter | blank slice 출력 (error_fill=1) |

---

## 8. testbench 시나리오

| 시나리오 | 입력 | 기대 출력 | 통과 기준 |
|----------|------|-----------|-----------|
| 모든 stop 각 1 hit | 8 raw_events | 8 cells, 각 hit_count=1 | bit-exact |
| stop 0에 8 hit (full) | 8 raw_events (same stop) | hit_count=8, hit_dropped=0 | bit-exact |
| stop 0에 9 hit (overflow) | 9 raw_events (same stop) | hit_count=8, hit_dropped=1 | drop 정확 |
| stops_per_chip=4 | stop 0~3만 hit | 4 cells 출력 | slice 길이 정확 |
| hit 없는 shot | drain_done only | 모든 cell zero (hit_count=0) | zero-fill 정확 |
| multi-hit 시간순 | stop0: hit_seq 0,1,2 | slot[0]<slot[1]<slot[2] | 시간순 보존 |

---

## 9. 완료 조건

- [ ] 입력 raw_event에 대해 기대한 cell 내용 bit-exact 일치
- [ ] hit 없는 slot은 0
- [ ] MAX 초과 hit는 hit_dropped=1
- [ ] chip slice 길이 = stops_per_chip × cell_size
- [ ] stop 번호 오름차순 출력

---

## 10. 다음 단계 handoff contract

**이 Step의 출력 = 다음 Step(04)의 입력:**

```
m_axis_chip_slice : AXI-Stream (tdata=TDATA_WIDTH, tlast=chip slice 끝)
slice_ready       : 1-bit (chip slice 출력 완료)
chip_id           : 2-bit (어떤 lane인가)
```

04번 문서는 4개 lane의 chip slice를 모아 1개 VDMA line(Packed Row)으로 조립한다.
