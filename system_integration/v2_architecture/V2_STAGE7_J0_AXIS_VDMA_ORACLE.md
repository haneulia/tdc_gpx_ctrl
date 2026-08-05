# V2 Stage 7 J0 - AXIS/VDMA Formatter Oracle

## 1. 목적과 범위

Checkpoint J는 B8의 Rise/Fall typed Cell lane을 B9의 AXI4-Stream Video/VDMA
바이트열로 바꾼다. Hit/Cell 의미와 Chip/STOP 순서는 B6-B8이 이미 확정했으며,
B9는 다음 항목만 소유한다.

- canonical 32-bit Cell word 생성
- 32/64/128-bit beat repack
- 48-byte line prefix와 line-end alignment
- SOF, EOL, TKEEP, TSTRB, TLAST
- lane별 HSIZE, 공통 VSIZE와 tightly-packed STRIDE
- leading/interior/trailing/all-hole column 보존
- bounded backpressure와 Face abort/recovery

Ethernet MTU/repack은 B9 이후의 전송 계층이며 이번 단계에 포함하지 않는다.

## 2. 고정 v1 oracle

| source | SHA-256 |
|---|---|
| `tdc_gpx_cell_builder.vhd` | `7B6A6698243E34ABAC22430856531E85A97CF747D5AEB5FDFF208E5F59D4CDEE` |
| `tdc_gpx_face_assembler.vhd` | `13BB5875B222EF07F9429590E8D7CE25AE2D84C04D26939AC672AA34EC11A347` |
| `tdc_gpx_line_packer.vhd` | `34E07B0D382C061FA48EC3C0C36E5304BA41C55800B087154E3DB7A025BB9D86` |
| `tdc_gpx_header_inserter.vhd` | `7EA124693219F5E54703830271C738626DBC83E4FBE0ACA2306E1F4A190CCDD3` |
| `tdc_gpx_output_stage.vhd` | `50F4A2000E67E3ABCBDED5EF4907E6BB509296ED6FE47112B4A4E4F8D4AC95B0` |
| `tdc_gpx_pkg.vhd` | `B9E8C10ABF3404B962ED03588E7607D47C7512C8676402FA2CA9503C291ECACE` |

Generated `ip_repo` 복사본은 oracle이 아니다. v1 RTL은 비교 대상으로만 읽고
Stage 7에서 수정하지 않는다.

## 3. canonical Cell ABI

한 Cell은 `ceil(max_hits/2)`개의 Hit word와 마지막 metadata word 하나로
구성된다. `max_hits`는 Face snapshot 동안 1..7로 고정된다.

```text
cell_words = ceil(max_hits / 2) + 1
cell_bytes = cell_words x 4
```

| effective max_hits | Hit words | Metadata words | Cell bytes |
|---:|---:|---:|---:|
| 1..2 | 1 | 1 | 8 |
| 3..4 | 2 | 1 | 12 |
| 5..6 | 3 | 1 | 16 |
| 7 | 4 | 1 | 20 |

Hit word `k`는 little-endian word이며 다음과 같다.

| bit | 의미 |
|---|---|
| `[15:0]` | Return `2k`의 GPX Hit `[15:0]`, 미사용이면 0 |
| `[31:16]` | Return `2k+1`의 GPX Hit `[15:0]`, 미사용이면 0 |

Hit `[16]`은 metadata에 별도로 보존하므로 17-bit 거리 원본을 잃지 않는다.

### 3.1 Metadata word

| bit | 의미 | blank Cell |
|---|---|---|
| `[31:25]` | Return 6..0 valid bitmap | 0 |
| `[24:18]` | Return 6..0 slope bitmap, Rising이면 valid bitmap | 0 |
| `[17:16]` | reserved | 0 |
| `[15:12]` | 실제 Hit 수 0..7 | 0 |
| `[11]` | Hit drop/overflow | 0 |
| `[10]` | error fill | 1 |
| `[9:8]` | 논리 Chip ID | 해당 slot Chip |
| `[7]` | reserved | 0 |
| `[6:0]` | Return 6..0의 Hit `[16]` | 0 |

STOP ID는 metadata에 반복하지 않는다. B8이 보장한 lane 내
`Chip 오름차순 -> STOP 오름차순` slot 위치로 결정한다.

## 4. line과 VDMA geometry

Rise/Fall은 별도 VDMA lane이다. 각 Shot column은 각 활성 slope lane에서 VDMA
line 하나가 된다.

```text
payload_bytes = slot_count x cell_bytes
aligned_payload_bytes = align16(payload_bytes)
HSIZE_lane = 48 + aligned_payload_bytes
VSIZE = columns_per_face
STRIDE_lane = HSIZE_lane
```

- lane의 `slot_count=0`이면 해당 lane은 존재하지 않으며 HSIZE는 0이다.
- 48-byte prefix는 세 개의 128-bit block이다.
- 첫 line prefix만 실제 Face header이고 이후 line의 prefix 영역은 0이다.
- line-end pad는 0/4/8/12 bytes이며 Cell 사이에는 padding이 없다.
- pad도 DDR에 기록되는 명시적인 0 byte이므로 모든 beat의 TKEEP/TSTRB는 1이다.
- 16-byte line alignment를 32/64/128-bit가 모두 정확히 나누므로 partial TKEEP는 없다.

따라서 폭이 넓어지면 beat 수와 전송 clock 수만 줄고 HSIZE, STRIDE 및 DDR byte
수는 변하지 않는다.

## 5. AXI4-Stream Video 계약

| signal | 계약 |
|---|---|
| `TUSER[0]` | Face 첫 line의 첫 beat에서만 1(SOF) |
| `TLAST` | 모든 VDMA line의 마지막 beat에서 1(EOL) |
| `TKEEP/TSTRB` | 모든 beat에서 전 bit 1 |
| `TVALID` | beat가 존재할 때 1, stall 중 payload와 sideband 고정 |
| `TREADY` | downstream backpressure, Rise/Fall 독립 |

`TUSER`와 `TLAST`는 handshake되지 않은 상태에서 변경하지 않는다. width별 beat
수는 `HSIZE / (output_width/8)`이며 32/64/128 순서로 정확히 4:2:1 비율이다.

## 6. hole 보존

B8은 수락된 Shot만 Cell로 전달하고 기하학적 누락 수를 별도 전달한다.

- `gap_before=N`: 현재 실제 line 전에 N개의 all-zero payload line 생성
- `trailing_gap=N`: Face close 뒤 N개의 all-zero payload line 생성
- `all_hole=1`: `columns_per_face` 전체를 all-zero payload line으로 생성

blank line도 정상 VDMA line과 동일한 HSIZE와 EOL을 가진다. 이를 통해 busy로
누락된 Shot이 다음 column을 앞으로 당기지 않는다.

## 7. 순차 구현 원칙

B9는 wide variable mux 대신 32-bit canonical word와 128-bit alignment block을
순차적으로 만든다.

```text
B8 Cell holding register
  -> 32-bit word serializer
  -> 4-word / 128-bit block accumulator
  -> aligned line buffer
  -> width adapter (32: 4 beats, 64: 2 beats, 128: 1 beat)
  -> AXIS holding register
```

이 구조는 Cell당 2..5개의 word를 순차 생성하고, line buffer read 뒤에 output
holding register를 둔다. `fire_done -> start_tdc` 저지연 경로와는 완전히 분리된다.

## 8. Face abort와 recovery

1. abort 이후 새 Cell 수락을 중지한다.
2. 이미 `TVALID=1`로 제시한 beat는 handshake될 때까지 변경하지 않는다.
3. sink가 진행하면 현재 line과 남은 VSIZE line을 zero-fill하여 정확한 EOL/Frame
   길이로 닫고 `frame_done_faulted`를 설정한다.
4. bounded-stall watchdog을 넘으면 AXIS 규약을 깨고 valid를 임의 해제하지 않는다.
   `recovery_reset_request`와 sticky를 올려 parent가 B9와 VDMA를 함께 reset한다.
5. 다음 Face는 새 SOF와 line index 0에서만 시작한다.

Face-close event는 각 활성 lane에 1-entry pending register로 복제한다. 두 lane이
모두 close를 보존할 수 있을 때만 B8에 ready를 반환한다. Falling 비활성 구성은
Rise lane만 handshake 대상이다.

## 9. Checkpoint J 분할

| sub-step | 산출물 | gate |
|---|---|---|
| J0 | 본 oracle과 geometry/header package | v1 ABI와 공식 단일화 |
| J1 | canonical Cell serializer | max_hits 1..7, blank/fault, 17-bit 보존 |
| J2 | single-lane line formatter | 32/64/128 byte exact, SOF/EOL/TKEEP |
| J3 | dual-lane close/abort owner | hole, close, abort, independent stall |
| J4 | B5-B9 통합 | 150/200 및 200/150, CDC/DRC/timing |

Checkpoint J는 J0-J4가 모두 통과한 뒤에만 Complete로 변경한다.
