# V2 Checkpoint J1 - GPX Cell Word Serializer

## 1. 역할

J1은 B8의 폭 독립적인 `gpx_frame_cell_event_t`를 v1과 동일한 canonical
32-bit word 열로 변환한다. AXIS 32/64/128-bit repack과 line buffer는 J2가
소유하므로 J1에는 출력 폭에 따른 분기나 wide mux가 없다.

```text
B8 typed Cell
  -> registered Cell holding
  -> canonical Hit word 0..3
  -> canonical metadata word
  -> registered word-event holding
  -> J2 block accumulator
```

## 2. RTL과 package

| 파일 | 역할 |
|---|---|
| `system_integration/v2/pkg/lidar_gpx_vdma_pkg.vhd` | Cell byte 수, HSIZE, beat 수, word/metadata mapping의 단일 공식 owner |
| `system_integration/v2/rtl/proc/lidar_gpx_cell_word_serializer.vhd` | Cell ready/valid를 32-bit word ready/valid로 순차 변환 |
| `system_integration/v2/tb/tb_lidar_gpx_cell_word_serializer.vhd` | max_hits, slope, blank, stall, 무-bubble 계약 검증 |
| `system_integration/v2/scripts/run_v2_gpx_cell_word_serializer.ps1` | 150/200 MHz xsim 및 OOC implementation |

`lidar_gpx_vdma_pkg`가 다음 공식을 단독 소유한다.

```text
cell_words = ceil(max_hits / 2) + 1
cell_bytes = 4 x cell_words
HSIZE = 48 + align16(slot_count x cell_bytes)
line_beats = HSIZE / (output_width / 8)
```

## 3. 순차 처리와 backpressure

Cell 전체를 한 번 등록한 뒤 word index를 0부터 증가시키며 word를 생성한다.
데이터와 line/cell sideband는 동일한 output register에 함께 저장된다.
`i_word_ready=0`이면 record 전체가 그대로 유지된다.

현재 Cell의 마지막 word를 output register에 적재하는 사이클에 다음 Cell을
동시에 수락할 수 있다. 따라서 초기 warm-up 이후 Cell 경계 bubble 없이 매
clock마다 word 하나를 생성한다. 출력 폭 선택 mux는 이 모듈에 존재하지 않는다.

## 4. 검증 범위

Session: `260806_j1_cell_word_v2_v2_gpx_cell_word_serializer`

| 항목 | 결과 |
|---|---|
| effective max_hits 1..7 | PASS |
| Cell bytes 8/8/12/12/16/16/20 | PASS |
| Rise slope bitmap 1, Fall slope bitmap 0 | PASS |
| Hit `[15:0]` word와 Hit `[16]` metadata 보존 | PASS |
| blank Cell zero Hit + `error_fill=1` | PASS |
| truncated hit_count에서 미사용 Return zero | PASS |
| 출력 stall 중 payload/sideband 안정 | PASS |
| 연속 두 Cell 경계 무-bubble | PASS |
| HSIZE 16 slots/max7 = 368 bytes | PASS |
| HSIZE 32 slots/max7 = 688 bytes | PASS |
| 32/64/128 beat 수 92/46/23 | PASS |

## 5. OOC 결과

| processing clock | WNS | latch | blocking DRC |
|---:|---:|---:|---:|
| 150 MHz | +3.126 ns | 0 | 0 |
| 200 MHz | +1.391 ns | 0 | 0 |

자원은 102 LUT, 438 FF, LUTRAM/BRAM/DSP 0이다. FF 대부분은 B8 Cell payload와
Shot identity를 backpressure 동안 보존하는 1-entry holding register다.

200 MHz 최악 경로는 word index에서 다음 Cell register enable까지의 3 LUT
경로이며, 배선 비중은 76.2%, WNS는 `+1.391 ns`이다. J1 자체에 추가
pipeline은 필요하지 않다. J2는 이 경로 뒤에 wide mux를 직접 붙이지 않고
별도의 128-bit block accumulator register를 둔다.

## 6. 다음 단계

J2는 canonical word 네 개를 128-bit alignment block으로 순차 적재하고 line
buffer에 저장한다. 출력할 때 동일 block을 32-bit 네 beat, 64-bit 두 beat,
128-bit 한 beat로 나누며, 세 출력 폭의 DDR byte 열은 완전히 같아야 한다.
