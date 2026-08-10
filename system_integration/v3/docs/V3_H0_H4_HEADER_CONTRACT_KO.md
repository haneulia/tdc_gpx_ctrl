# V3 H0-H4 HLS Header와 Bit ABI 계약

## 1. 문서 목적

이 문서는 H0~H4에서 생성한 C++ Header 파일의 역할, packed payload의 Bit 위치,
생산자와 소비자, 수정 절차를 한 곳에 고정한다. 여기서 **Header 파일**은 C++
`*.hpp`를 뜻하며, DDR의 Face Footer 또는 Shot Line Metadata와는 다른 개념이다.

숫자만 직접 잘라 쓰는 코드는 금지한다. C++은 `bit_field_t`와 의미 이름을,
VHDL Adapter는 `lidar_v3_hls_contract_pkg.vhd`의 상수를 사용한다. 두 언어의
계약 일치는 V2/HLS 차등 테스트가 Word 단위로 검증한다.

## 2. H0 공통 Header

| 파일 | 소유 역할 | 하드웨어 데이터 생성 여부 |
|---|---|---|
| `lidar_v3_hls_contract.hpp` | H0~H4 전체 계약을 한 번에 포함하는 umbrella Header | 없음 |
| `lidar_v3_hls_bit_field.hpp` | `low/high/end/width`와 `read/write_field`, `read/write_flag` 제공 | 없음 |
| `lidar_v3_hls_limits.hpp` | ABI 3.2, 최대 Chip 4, STOP/Chip 8, Return/STOP 7, Face 5, slope 의미 | 없음 |
| `lidar_v3_h1_raw_hit_contract.hpp` | Raw Event, I-Mode 28-bit, Hit Event, Shot Context | 있음: H1 경계 |
| `lidar_v3_h2_cell_contract.hpp` | Hit 수집 입력, Cell, Collector 결과 | 있음: H2 경계 |
| `lidar_v3_h3_frame_contract.hpp` | Cell/Face-close 입력, 정렬 Lane Cell, Frame 제어 결과 | 있음: H3 경계 |
| `lidar_v3_h4_word_contract.hpp` | Shot Metadata, PACKED17 Cell, Face Footer용 canonical 32-bit Word | 있음: H4 경계 |

H0는 별도 HLS Top 함수가 아니다. 모든 단계가 공유하는 **이름·상한·Bit 배치의
단일 기준층**이다. 한 단계 구현은 자신의 계약 Header만 직접 include하고, 통합
테스트나 ABI 도구는 umbrella Header를 include한다.

`ap_uint`는 packed HLS/RTL 경계에만 사용한다. 제어 계산과 일반 알고리즘은
`std::uint8_t`, `std::uint16_t`, `std::uint32_t`를 우선한다.

## 3. 전체 데이터 흐름과 소유권

```text
V2 acquisition RTL
  │ Raw Event 216 bit
  ▼
H1 gpx_hit_decoder_hls
  │ Decoder Result 224 bit
  │   └─ Decoded Hit Event 218 bit
  ▼
H2 gpx_cell_collector_hls
  │ Collector Result 328 bit
  │   └─ Cell Event 319 bit
  ▼
H3 gpx_frame_assembler_hls
  ├─ Rise ordered Lane Cell 360 bit
  ├─ Fall ordered Lane Cell 360 bit
  └─ Assembler Control 264 bit
  ▼
H4 gpx_lane_word_formatter_hls
  ├─ Canonical Line Word 248 bit
  │   └─ 실제 DDR payload 32 bit + Line/Frame 제어 문맥
  └─ Formatter Control 32 bit
  ▼
RTL AXIS Word packer
  └─ 합성 시 선택한 32/64-bit AXI4-Stream Beat
```

| 계약 | 생산자 | 소비자 | 변경 권한 |
|---|---|---|---|
| Shot Context 162 bit | Shot scheduler/Adapter | H1, H2, H3, 이후 H4 | H1 계약이 소유 |
| TDC-GPX I-Mode Word 28 bit | GPX acquisition RTL | H1 | 외부 TDC-GPX I-Mode 물리 계약 |
| Decoded Hit Event | H1 | H2 | H1 계약이 소유 |
| Cell Event | H2 | H3 | H2 계약이 소유 |
| Ordered Lane Cell/Control | H3 | H4 | H3 계약이 소유 |
| Canonical Line Word/Formatter Control | H4 | RTL AXIS Word packer/진단 집계 | H4 계약이 소유 |

## 4. Shot Context 162-bit

Shot Context는 한 Shot의 모든 Hit와 Cell이 공유하는 불변 측정 문맥이다.
`측정 시작 기준시점 (T0)`은 물리 모드에서 동기화된 `fire_done`을 승인하고
`start_tdc`를 발생시킨 사건이다.

| Bit | C++ 의미 이름 | 의미 |
|---:|---|---|
| 0 | `request_valid` | Shot 요청 문맥이 유효함 |
| 3:1 | `mirror_face_index` | 현재 다면 미러 Face 번호 |
| 18:4 | `encoder_position_state` | Shot 후보점의 decoder state |
| 19 | `direction_is_ccw` | 1이면 State 감소 방향(CCW) |
| 35:20 | `shot_column_index` | Face 안의 Shot column 번호 |
| 36 | `is_last_shot_column_in_face` | Face의 마지막 Shot column |
| 37 | `source_is_simulation` | Virtual encoder/Simulation 경로 |
| 45:38 | `encoder_to_scheduler_latency_clks` | Encoder event부터 scheduler 승인까지 Processing clock 수 |
| 46 | `encoder_latency_is_valid` | 앞 latency 측정값 유효 |
| 62:47 | `active_configuration_version` | 해당 Shot에 실제 적용된 Active 설정 버전 |
| 94:63 | `fire_command_to_t0_latency_clks` | `fire_pulse`부터 측정 시작 기준시점 (T0)까지 Processing clock 수 |
| 158:95 | `measurement_start_t0_timestamp_ticks` | 측정 시작 기준시점 (T0)의 동기 시간원 timestamp |
| 159 | `measurement_start_timestamp_is_valid` | timestamp 값 유효 |
| 160 | `measurement_start_time_sync_is_valid` | 외부/시스템 시간 동기 상태 유효 |
| 161 | `context_is_valid` | 전체 Shot Context 사용 가능 |

H3는 162-bit 전체 비교를 긴 단일 조합 경로로 만들지 않기 위해 32-bit 다섯 조각과
마지막 2-bit로 비교한다. 이는 구현 분할이며 ABI 필드를 추가하는 것이 아니다.

## 5. H1 Raw-to-Hit 계약

### 5.1 Raw Event AXIS TDATA, 216-bit

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 1:0 | `event_kind` | 0 Data, 1 IFIFO1 완료, 2 Drain 완료, 3 timeout |
| 3:2 | `tdc_chip_index` | TDC-GPX Chip 0~3 |
| 4 | `ififo_bank_select` | IFIFO bank 선택 |
| 32:5 | `tdc_gpx_imode_word` | 외부 TDC-GPX I-Mode 28-bit Word |
| 33 | `upstream_event_faulted` | acquisition 단계에서 이미 fault 판정 |
| 36:34 | `timeout_cause_bitmap` | bounded Drain timeout 원인 |
| 198:37 | `shot_context` | 162-bit Shot Context |
| 214:199 | `tdc_chip_shot_sequence` | Chip별 Shot 순서 번호 |
| 215 | `reserved_zero` | AXIS byte 정렬용, 항상 0 |

### 5.2 외부 TDC-GPX I-Mode Word, 28-bit

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 16:0 | `distance_hit_17bit` | TDC 측정값, 하위 17-bit 거리 원자료 |
| 17 | `edge_slope_is_rise` | 1 Rising, 0 Falling |
| 25:18 | `start_number` | TDC-GPX START 번호 |
| 27:26 | `channel_index_within_ififo` | 해당 IFIFO 안의 Channel 번호 |

### 5.3 Decoded Hit Event, 218-bit

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 1:0 | `event_kind` | Raw Event 종류 보존 |
| 3:2 | `tdc_chip_index` | Chip 번호 |
| 4 | `ififo_bank_select` | IFIFO bank |
| 6:5 | `tdc_gpx_channel_index` | I-Mode channel 0~3 |
| 9:7 | `logical_stop_channel_index` | 논리 STOP 0~7 |
| 17:10 | `tdc_start_number` | START 번호 |
| 18 | `edge_slope_is_rise` | 1 Rising, 0 Falling |
| 35:19 | `distance_hit_17bit` | 복호화된 17-bit Hit |
| 36 | `upstream_event_faulted` | 상위 fault 보존 |
| 39:37 | `timeout_cause_bitmap` | timeout 원인 |
| 201:40 | `shot_context` | Shot Context |
| 217:202 | `tdc_chip_shot_sequence` | Chip별 Shot 순서 번호 |

### 5.4 Decoder Result AXIS TDATA, 224-bit

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 217:0 | `decoded_hit_event` | 위 Decoded Hit Event |
| 218 | `contains_hit_event` | 1일 때만 H2로 Hit 전달 |
| 219 | `tdc_chip_index_fault` | 합성된 Chip 수 범위 위반 |
| 220 | `stop_channel_index_fault` | STOP 수 범위 위반 |
| 221 | `edge_slope_assignment_fault` | Runtime 활성 slope mask 위반 |
| 223:222 | `reserved_zero` | 항상 0 |

## 6. H2 Hit-to-Cell 계약

### 6.1 Collector Input AXIS TDATA, 232-bit

| Bit | 의미 |
|---:|---|
| 217:0 | H1 `decoded_hit_event` |
| 223:218 | byte 정렬 reserve, 항상 0 |
| 231:224 | `reset_epoch`; 값이 바뀐 첫 입력에서 내부 보류 상태를 한 번 초기화 |

### 6.2 Cell Event, 319-bit

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 1:0 | `event_kind` | Data Cell 또는 Shot 종료 Cell 분류 |
| 3:2 | `tdc_chip_index` | Cell 소유 Chip |
| 4 | `ififo_bank_select` | IFIFO bank |
| 7:5 | `logical_stop_channel_index` | STOP 0~7 |
| 8 | `edge_slope_is_rise` | Rise/Fall lane 선택 |
| 11:9 | `visible_return_count` | 실제 수신되어 PACKED17에 유효하게 기록할 Return 수, 0~Runtime 슬롯 수 |
| 14:12 | `serialized_return_slot_count` | 현재 Face에 적용된 Runtime 직렬화 Return 슬롯 수, 1~7 |
| 133:15 | `packed_distance_hits_17bit` | Return 0~6의 17-bit Hit, 총 119 bit |
| 134 | `hit_was_dropped` | 의도하지 않은 내부 Hit 손실이 있었음 |
| 135 | `return_overflow` | 8번째 이상 물리 Return 감지 |
| 136 | `error_fill_inserted` | timeout 복구용 placeholder가 삽입됨 |
| 137 | `cell_is_faulted` | Cell 결과에 fault가 있음 |
| 140:138 | `timeout_cause_bitmap` | timeout 원인 |
| 302:141 | `shot_context` | Shot Context |
| 318:303 | `tdc_chip_shot_sequence` | Chip별 Shot 순서 번호 |

세 Return 수는 서로 다른 계약이다. 합성 물리 최대 Return 수는 7로 고정되고,
`serialized_return_slot_count`는 Runtime에서 1~7로 선택한다. 실제 유효 Return 수인
`visible_return_count`는 `min(실제 수신 수, Runtime 직렬화 슬롯 수)`다. Runtime 슬롯
수를 넘는 물리 Hit도 외부 TDC-GPX EF 완료까지 Drain하며, 의도적인 전시 필터이므로
fault가 아니다. 합성 물리 최대를 넘는 8번째 이상만 `return_overflow`다.

### 6.3 Collector Result AXIS TDATA, 328-bit

| Bit | 의미 |
|---:|---|
| 318:0 | `cell_event` |
| 319 | `contains_cell_event` |
| 320 | Shot Context 불일치 fault |
| 321 | 8번째 이상 Return overflow fault |
| 322 | 허용되지 않은 non-zero START 번호 fault |
| 323 | 합성 Return 용량을 넘겨 손실된 fault |
| 327:324 | reserve, 항상 0 |

## 7. H3 Cell-to-Frame 계약

### 7.1 Assembler Input AXIS TDATA, 328-bit

| Bit | 의미 |
|---:|---|
| 318:0 | `event_body`; Cell Event 또는 하위 69-bit Face-close Event |
| 319 | `event_kind`; 0 Cell, 1 Face-close |
| 327:320 | `reset_epoch` |

Face-close Event의 하위 69-bit는 `frame_identifier[31:0]`,
`mirror_face_index[34:32]`, `direction_is_ccw[35]`,
`source_is_simulation[36]`, `active_configuration_version[52:37]`,
`expected_shot_column_count[68:53]` 순서다.

### 7.2 Ordered Rise/Fall Lane Cell, 360-bit

| Bit | 의미 |
|---:|---|
| 318:0 | 원본 `cell_event` |
| 324:319 | `lane_cell_slot_index`; Chip/STOP 정렬 후 현재 Cell slot |
| 330:325 | `lane_cell_slot_count`; 해당 slope의 Shot당 전체 Cell 수 |
| 331 | Shot Line의 첫 Cell |
| 332 | Shot Line의 마지막 Cell |
| 333 | Face 첫 Shot column |
| 334 | Face 마지막 Shot column |
| 350:335 | 현재 Shot 전에 누락된 Shot column 수 |
| 351 | 누락 Cell을 나타내는 placeholder |
| 352 | 해당 Shot Line fault |
| 359:353 | reserve, 항상 0 |

`lane_cell_slot_count`는 VDMA Line 수가 아니다. 한 Shot Line 안에서 활성 Chip과
STOP을 순서화할 때 필요한 Cell 개수다. H4가 이 Cell을 canonical 32-bit Word로
변환하고 최종 RTL packer가 32/64-bit AXI4-Stream Beat로 묶는다.

### 7.3 Assembler Control AXIS TDATA, 264-bit

| Bit | 의미 |
|---:|---|
| 7:0 | fault bitmap |
| 8 | Face-close 결과 포함 |
| 95:9 | 87-bit Face-close 결과 |
| 96 | 현재 Shot의 필요한 Cell 생성 완료 |
| 258:97 | 완료된 Shot Context |
| 263:259 | reserve, 항상 0 |

fault bitmap은 Bit 0부터 순서대로 Context 불일치, 예상 밖 Cell, 중복 Cell,
중복 Shot 종료, 누락 Cell, Geometry 불일치, Shot column gap, 비활성 mask 입력이다.

Face-close 결과는 원본 69-bit Face-close Event 뒤에
`trailing_missing_shot_columns[84:69]`, `entire_face_is_missing[85]`,
`face_close_is_faulted[86]`을 붙인다.

## 8. H4 Frame-to-Word 계약

### 8.1 Formatter Input AXIS TDATA, 376-bit

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 359:0 | `event_body` | H3 Ordered Lane Cell 360-bit 또는 하위 87-bit Face-close 결과 |
| 360 | `event_kind` | 0 Ordered Lane Cell, 1 Face-close 결과 |
| 368:361 | `reset_epoch` | 값이 바뀐 첫 입력에서 미완료 Face/Shot 상태를 한 번 폐기 |
| 375:369 | `reserved_zero` | byte 정렬 reserve, 항상 0 |

### 8.2 Active Lane Profile, 104-bit

Profile은 COMMIT 후 Face 경계에서 등록된 값이다. H4는 Streaming 중 나눗셈이나
가변 폭 정렬을 하지 않고 이 값을 검증한 뒤 사용한다.

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 0 | `profile_is_valid` | Profile 전체가 유효함 |
| 1 | `lane_is_enabled` | 해당 Rise/Fall Lane 활성 |
| 2 | `edge_slope_is_rise` | 1 Rise, 0 Fall |
| 4:3 | `output_axis_width_code` | 0=32-bit, 1=64-bit, 2~3 금지 |
| 10:5 | `lane_cell_slot_count` | Shot Line 안의 Cell 수, 1~32 |
| 13:11 | `serialized_return_slot_count` | Runtime 직렬화 Return 슬롯 수, 1~7 |
| 16:14 | `serialized_cell_word_count` | Cell당 Word 수: `ceil(Return 슬롯/2)+1` |
| 32:17 | `planned_shot_column_count` | Face에서 계획한 Shot Line 수 |
| 41:33 | `raw_shot_line_word_count` | `4 + Cell 슬롯 수 × Cell당 Word 수` |
| 50:42 | `aligned_hsize_word_count` | 출력 폭 경계에 맞춘 HSIZE의 32-bit Word 수 |
| 66:51 | `hsize_bytes` | VDMA HSIZE byte 수 |
| 68:67 | `face_footer_line_count` | 32-byte Face Footer가 차지하는 Line 수, 1~2 |
| 84:69 | `vsize_line_count` | `계획 Shot Line 수 + Footer Line 수` |
| 100:85 | `active_configuration_version` | 이 Face에 실제 적용된 설정 버전 |
| 103:101 | `reserved_zero` | 항상 0 |

### 8.3 Canonical Line Word AXIS TDATA, 248-bit

한 전송의 실제 DDR payload는 Bit 31:0의 32-bit Word다. 나머지는 최종 RTL
AXIS Word packer가 `TUSER`, `TLAST`, 32/64-bit Beat 조립을 결정하기 위한 문맥이며
DDR에 별도 Word로 저장되지 않는다.

| Bit | 의미 이름 | 의미 |
|---:|---|---|
| 31:0 | `canonical_word_32bit` | Shot Metadata, PACKED17 Cell 또는 Face Footer Word |
| 33:32 | `word_kind` | 0 Shot Metadata, 1 PACKED17 Cell, 2 Face Footer |
| 42:34 | `word_index_within_line` | 현재 HSIZE Line 안의 32-bit Word 번호 |
| 51:43 | `line_word_count` | 현재 Line의 총 32-bit Word 수 |
| 52 | `is_line_start` | Line 첫 Word |
| 53 | `is_line_end` | Line 마지막 Word; 최종 packer가 `TLAST` 생성 |
| 54 | `is_frame_end` | 마지막 Footer Line의 마지막 Word |
| 55 | `is_first_shot_column` | Face 첫 Shot Line |
| 56 | `is_last_shot_column` | Face 마지막 Shot Line |
| 57 | `line_represents_missing_shot` | Hole Line |
| 58 | `shot_line_is_faulted` | 이 Shot Line의 fault 요약 |
| 74:59 | `unexpanded_missing_shot_columns` | H4 출력에서는 반드시 0 |
| 80:75 | `lane_cell_slot_count` | 이 Line의 Cell 수 |
| 83:81 | `serialized_cell_word_count` | Cell당 Word 수 |
| 245:84 | `shot_context` | 원본 Shot 문맥; Footer에서는 0 |
| 247:246 | `reserved_zero` | 항상 0 |

### 8.4 Shot Line Metadata, 4 Word/16 byte

| Word | Bit | 의미 |
|---:|---:|---|
| 0 | 31:0 | 측정 시작 기준시점 (T0) timestamp 하위 32-bit; Hole은 0 |
| 1 | 31:0 | 측정 시작 기준시점 (T0) timestamp 상위 32-bit; Hole은 0 |
| 2 | 15:0 | Face 안의 Shot column 번호 |
| 2 | 31:16 | Encoder state; Hole은 `0xFFFF` |
| 3 | 0 | 실제 데이터 Line 유효 |
| 3 | 1 | Hole Line |
| 3 | 2 | 회전 방향 CCW |
| 3 | 3 | Simulation 출처 |
| 3 | 4 | V2 Shot timeout 예약 필드; H3에 원인 Event가 없어 현재 0 |
| 3 | 5 | V2 Shot abort 예약 필드; H3에 원인 Event가 없어 현재 0 |
| 3 | 6 | Shot Line fault |
| 3 | 7 | 측정 시작 기준시점 (T0) timestamp 유효 |
| 3 | 8 | 시스템 시간 동기 상태 유효 |
| 3 | 9 | Face 마지막 Shot column |
| 3 | 10 | Encoder-to-scheduler latency 유효 |
| 3 | 18:11 | Encoder event부터 scheduler 승인까지 Processing clock 수 |
| 3 | 나머지 | reserve, 0 |

### 8.5 PACKED17 Cell

각 Cell은 `ceil(Runtime 직렬화 Return 슬롯 수/2)`개의 Hit Word와 마지막 Cell
Metadata Word 하나로 구성된다. Hit Word는 Return 두 개의 하위 16-bit를
`[15:0]`, `[31:16]`에 순서대로 넣는다. 홀수 번째 마지막 슬롯의 사용하지 않는
상위 절반은 0이다.

| Cell Metadata Bit | 의미 |
|---:|---|
| 6:0 | Return 0~6의 Hit[16] |
| 13:7 | Return 0~6 유효 bitmap |
| 16:14 | 실제 유효 Return 수 |
| 17 | 1 Rise, 0 Fall |
| 19:18 | TDC-GPX Chip 번호 |
| 22:20 | 논리 STOP 번호 |
| 23 | 누락 Cell placeholder |
| 24 | timeout 복구용 error fill |
| 25 | 의도하지 않은 내부 Hit 손실 |
| 26 | 8번째 이상 물리 Return 감지 |
| 27 | Cell 또는 Shot Line fault |
| 30:28 | timeout 원인 bitmap |
| 31 | Cell Metadata marker, 항상 1 |

### 8.6 Face Footer, 8 Word/32 byte

| Word | 내용 |
|---:|---|
| 0 | Magic `0x47504631` (`GPF1`) |
| 1 | Face frame identifier |
| 2 | `[2:0]` Face, `[3]` Rise, `[4]` CCW, `[5]` Simulation, `[7:6]` 출력 폭 code |
| 3 | `[15:0]` Active configuration version |
| 4 | `[15:0]` 계획 Shot 수, `[21:16]` Cell 슬롯 수, `[24:22]` Runtime 직렬화(전시) Return 슬롯 수 |
| 5 | `[15:0]` HSIZE byte, `[31:16]` VSIZE Line |
| 6 | `[15:0]` 완료 Shot Line 수, `[16]` Face-close fault, `[17]` 수 불일치, `[18]` Line fault, `[19]` Hole 존재, `[20]` V2 Shot timeout 요약, `[21]` V2 Shot abort 요약, `[22]` 전체 Face 누락 |
| 7 | Commit marker `0x434F4D54` (`COMT`) |

Footer는 논리적으로 항상 32 byte다. HSIZE가 32 byte 이상이면 Footer 1 Line,
최소 구성처럼 HSIZE가 24 또는 28 byte면 Footer 2 Line으로 나누며 각 Line의 남은
Word는 0으로 채운다. Face Footer는 Face 완료 후 기록되므로 불완전 Frame과 완전한
Frame을 PS가 구분하는 Commit 역할을 한다.

Footer Word 6의 Shot timeout/abort Bit 20/21은 현재 0이다. H3 입력에 명시적인
레이저 Shot timeout/abort Event가 없기 때문이다. Cell Metadata의
`timeout_cause_bitmap`은 TDC-GPX IFIFO Drain timeout이므로 물리 의미가 다른 이
두 Bit로 변환하지 않는다. 향후 H3 ABI에 원인 Event를 추가할 때만 활성화한다.

### 8.7 Formatter Control AXIS TDATA, 32-bit

| Bit | 의미 |
|---:|---|
| 6:0 | fault bitmap: invalid profile, slope 불일치, geometry 불일치, Shot 문맥/순서 불일치, Return 직렬화 계약 불일치, Face-close 계약 불일치, 입력 reserve non-zero |
| 7 | reserve, 항상 0 |
| 8 | 이번 입력 처리로 Face Footer를 방출함 |
| 25:9 | 이번 입력 처리로 방출한 Shot/Hole/Footer Line 수 |
| 31:26 | reserve, 항상 0 |

`formatter_input.event_kind`는 1-bit라 Ordered Cell과 Face-close 외의 잘못된 세 번째
encoding이 존재하지 않는다. 따라서 발생 불가능한 fault 이름을 만들지 않고 Bit 7을
예약 0으로 유지한다. H3가 H4에 Data Cell만 넘기는 내부 계약은 H3 차등 테스트와 V2
serializer assertion이 검증한다.

## 9. Build 설정과 Runtime Active 설정

| 종류 | 예 | 적용 규칙 |
|---|---|---|
| 합성 구조 상한 | Chip 수, STOP/Chip, 최대 Return | 합성 전 Generic으로 고정 |
| Runtime Active snapshot | Rise/Fall 활성 mask, 직렬화 Return 슬롯 수, Shot columns | COMMIT 성공 후 Face 안전 경계에서 적용 |
| 추적 정보 | Active version, reset epoch | 각 Event와 결과에 기록하여 적용 시점을 검증 |

Runtime 값은 합성 상한을 늘릴 수 없다. Return 수를 줄여도 외부 TDC-GPX IFIFO는
EF 완료까지 물리적으로 Drain하며, 줄어드는 것은 이후 전시·패킷화 데이터다.

## 10. 계약 수정 규칙

1. 기존 필드의 Bit 위치나 의미를 재사용하지 않는다.
2. 필드가 필요하면 먼저 reserve를 사용하거나 payload 끝에 추가한다.
3. C++ owning stage Header와 VHDL `lidar_v3_hls_contract_pkg.vhd`를 함께 수정한다.
4. `static_assert`로 전체 폭과 byte 정렬을 고정한다.
5. ABI minor를 올린다. 기존 필드 의미를 깨면 major 변경과 PS/Viewer 동시 전환이 필요하다.
6. HLS CSim, C synthesis, C/RTL CoSim을 수행한다.
7. V2/HLS 차등 테스트와 150/200 MHz OOC 배치·배선을 모두 다시 수행한다.
8. H4 이후에는 DDR Golden Word와 PS decoder 회귀도 함께 갱신한다.

## 11. ABI 3.2 검증 결과

검증 기준 Stamp는 `260810_h4_word_contract`이다. H4 최종 V2 직접 비교 실행은
`.work/v3_gpx_lane_word_formatter_diff/260810191631`, 최종 OOC 배치·배선 실행은
`.work/v3_gpx_lane_word_formatter_impl/260810185802`이다. `.work/`는 재생성 가능한
로컬 산출물이며 Git에는 넣지 않는다.

| 단계 | HLS CSim/CoSim | V2 차등 | 150 MHz WNS | 200 MHz WNS | Latch/차단 DRC |
|---|---|---:|---:|---:|---:|
| H1 Raw-to-Hit | 4 topology PASS | 8/8 PASS | +2.112 ns | +0.688 ns | 0/0 |
| H2 Hit-to-Cell | 4 topology PASS | 8/8 PASS | +0.106 ns | +0.080 ns | 0/0 |
| H3 Cell-to-Frame | 5 topology PASS | 10/10 PASS | +0.067 ns | +0.217 ns | 0/0 |
| H4 Frame-to-Word | 5 profile PASS | 4/4 최종 AXIS Beat PASS | +0.130 ns | +0.114 ns | 0/0 |

H2~H4의 최소 WNS는 양수지만 여유가 작다. 이는 각 단계 OOC Sign-off 결과이며
Parent 전체 배치·배선 Sign-off를 대신하지 않는다. H5 혼합 Top과 H6 Parent 단계에서
혼잡, CDC, Reset, 32/64-bit packer를 포함한 전체 타이밍을 다시 확인해야 한다.

생성된 프로젝트와 상세 로그는 저장소 루트 `.work/` 아래에 있으며 Git에는 넣지
않는다. 재현 가능한 소스, script, 계약 문서만 형상관리한다.
