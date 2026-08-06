# V2 Checkpoint J2 - GPX VDMA Lane Formatter

## 1. 목적

J2는 J1의 canonical 32-bit word 열을 한 slope lane의 AXI4-Stream Video
line으로 변환한다. Rise/Fall 복제, hole 생성, Face close와 시스템 abort 정책은
J3가 소유한다.

```text
J1 32-bit word event
  -> four-word / 128-bit accumulator
  -> 40-entry aligned payload LUTRAM
  -> registered 128-bit block prefetch
  -> 32/64/128-bit registered AXIS beat
```

저장소 선택과 AXIS 폭 변환 사이에 `block_data_r`가 있으므로 두 연산은 같은
clock 경로에 겹치지 않는다. `fire_done -> start_tdc` 직접 경로와도 완전히
분리되어 있다.

## 2. line 계약

```text
payload_words  = slot_count x cell_words
payload_blocks = ceil(payload_words / 4)
HSIZE          = 48 + 16 x payload_blocks
```

- 첫 column: 입력받은 3개의 128-bit Face prefix block을 출력한다.
- 후속 column: 동일한 48-byte prefix 위치를 zero로 출력한다.
- payload 마지막 block의 미사용 word는 zero line pad다.
- 모든 beat의 `TKEEP`와 `TSTRB`는 1이다.
- `TUSER[0]`은 첫 column의 첫 beat에서만 1이다.
- `TLAST`는 각 line의 마지막 beat에서만 1이다.
- stall 중 `TDATA/TUSER/TLAST/TVALID`는 고정된다.

## 3. 순차 처리와 폭 효과

최대 geometry는 32 slot, Cell당 5 word, payload 160 word, 40 block이다.
HSIZE는 모든 출력 폭에서 688 byte로 같다.

| output width | payload capture clocks | AXIS line beats | canonical DDR bytes |
|---:|---:|---:|---:|
| 32 | 160 | 172 | 688 |
| 64 | 160 | 86 | 688 |
| 128 | 160 | 43 | 688 |

폭이 넓어질수록 DDR 전송 beat 수는 정확히 감소한다. 다만 현재 J2는 명확한
단일-buffer store-and-forward 구조이므로 160-clock canonical serialization은
폭과 무관하다. 따라서 전체 line 처리시간은 AXIS 폭만큼 4:2:1로 줄지 않는다.
J3 통합 처리율이 실제 shot budget을 만족하지 못할 때만 ping-pong line buffer로
capture와 output을 겹친다.

## 4. abort 경계

- AXIS 출력 전 capture abort: 부분 line을 외부에 노출하지 않고 폐기한다.
- AXIS 출력 시작 후 abort: 이미 등록된 line을 EOL까지 유지하고
  `line_done_faulted=1`로 완료한다.
- 남은 VSIZE zero-fill과 coordinated reset request는 J3가 소유한다.

이 구분으로 `TVALID=1 && TREADY=0`인 beat를 abort가 임의로 지우지 않는다.

## 5. 기능 검증

Session: `260806_j2_abort_ready_v2_gpx_vdma_lane_formatter`

| 검증 항목 | 결과 |
|---|---|
| 32/64/128-bit x 150/200 MHz | PASS |
| Cell word count 2/3/4/5 | PASS |
| slot count 1/3/5/32 | PASS |
| actual first-line prefix와 후속 zero prefix | PASS |
| line-end 0/4/8/12-byte pad | PASS |
| SOF/EOL/TKEEP/TSTRB byte exact | PASS |
| continuous ready와 반복 stall | PASS |
| stalled payload/sideband 안정 | PASS |
| capture abort 무출력 | PASS |
| output abort EOL 보존과 fault completion | PASS |
| 최대 688-byte line의 byte 손실/중복/재정렬 없음 | PASS |

## 6. 구현 결과

Session: `260806_j2_final_impl_v2_gpx_vdma_lane_formatter`

| width | clock | WNS | LUT | LUTRAM | FF | latch | blocking DRC |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 32 | 150 MHz | +1.034 ns | 975 | 172 | 722 | 0 | 0 |
| 64 | 150 MHz | +2.048 ns | 993 | 172 | 752 | 0 | 0 |
| 128 | 150 MHz | +2.034 ns | 939 | 172 | 816 | 0 | 0 |
| 32 | 200 MHz | +0.394 ns | 976 | 172 | 722 | 0 | 0 |
| 64 | 200 MHz | +0.523 ns | 993 | 172 | 752 | 0 | 0 |
| 128 | 200 MHz | +0.380 ns | 939 | 172 | 816 | 0 | 0 |

최저 마진은 128-bit/200 MHz의 `+0.380 ns`다. 32/128-bit 최악 경로는 등록된
`write_block_r`에서 distributed RAM write address까지이며 논리 약 8.5%,
배선 약 91.5%다. 64-bit의 대표 최악 경로는 `read_block_r`에서
`block_data_r` CE까지다. 저장소 선택과 AXIS width mux가 한 경로에 합쳐진
경로는 최악 목록에 없다.

## 7. 추가 pipeline 판정

J2 내부에 레지스터를 한 단계 더 추가하지 않는다. 다음 두 후보를 실제 구현한
뒤 기각했다.

| 후보 | 200 MHz 결과 | 판정 |
|---|---|---|
| 128-bit RAM write-command pipeline | WADR 경로는 제거됐지만 153 FF 증가, 최저 WNS `+0.205 ns` | 기각 |
| `write_block_r` 강제 fanout 복제 16/32 | 폭별 22..90 LUT 증가, fanout 32에서 32-bit WNS `+0.089 ns` | 기각 |

1. 저장소 read와 AXIS width mux는 이미 서로 다른 clock 경계다.
2. 남은 경로의 약 77..92%가 낮은 논리 깊이의 배선이다.
3. 추가 write pipeline은 경로를 이동시켰지만 최소 WNS와 자원을 악화시켰다.
4. 강제 복제는 폭별 배치 편차가 커서 일반적인 RTL 최적화로 채택할 수 없다.

다음 최적화 판단은 J3 dual-lane 통합 배치에서 다시 수행한다. 통합 시 margin이
감소하면 LUTRAM/BRAM 선택, ping-pong buffer 또는 물리 계층 제약을 전체 배치와
함께 검토하며, 임의 delay register는 추가하지 않는다.
