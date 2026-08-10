# V3 H0-H3 HLS Header와 Bit ABI 계약

## 1. 문서 목적

이 문서는 H0~H3에서 생성한 C++ Header 파일의 역할, packed payload의 Bit 위치,
생산자와 소비자, 수정 절차를 한 곳에 고정한다. 여기서 **Header 파일**은 C++
`*.hpp`를 뜻하며, DDR의 Face Footer 또는 Shot Line Metadata와는 다른 개념이다.

숫자만 직접 잘라 쓰는 코드는 금지한다. C++은 `bit_field_t`와 의미 이름을,
VHDL Adapter는 `lidar_v3_hls_contract_pkg.vhd`의 상수를 사용한다. 두 언어의
계약 일치는 V2/HLS 차등 테스트가 Word 단위로 검증한다.

## 2. H0 공통 Header

| 파일 | 소유 역할 | 하드웨어 데이터 생성 여부 |
|---|---|---|
| `lidar_v3_hls_contract.hpp` | H0~H3 전체 계약을 한 번에 포함하는 umbrella Header | 없음 |
| `lidar_v3_hls_bit_field.hpp` | `low/high/end/width`와 `read/write_field`, `read/write_flag` 제공 | 없음 |
| `lidar_v3_hls_limits.hpp` | ABI 3.1, 최대 Chip 4, STOP/Chip 8, Return/STOP 7, Face 5, slope 의미 | 없음 |
| `lidar_v3_h1_raw_hit_contract.hpp` | Raw Event, I-Mode 28-bit, Hit Event, Shot Context | 있음: H1 경계 |
| `lidar_v3_h2_cell_contract.hpp` | Hit 수집 입력, Cell, Collector 결과 | 있음: H2 경계 |
| `lidar_v3_h3_frame_contract.hpp` | Cell/Face-close 입력, 정렬 Lane Cell, Frame 제어 결과 | 있음: H3 경계 |

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
```

| 계약 | 생산자 | 소비자 | 변경 권한 |
|---|---|---|---|
| Shot Context 162 bit | Shot scheduler/Adapter | H1, H2, H3, 이후 H4 | H1 계약이 소유 |
| TDC-GPX I-Mode Word 28 bit | GPX acquisition RTL | H1 | 외부 TDC-GPX I-Mode 물리 계약 |
| Decoded Hit Event | H1 | H2 | H1 계약이 소유 |
| Cell Event | H2 | H3 | H2 계약이 소유 |
| Ordered Lane Cell/Control | H3 | H4 | H3 계약이 소유 |

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
| 11:9 | `visible_return_count` | Runtime에서 사용자에게 보일 Return 수 |
| 14:12 | `configured_return_capacity` | 합성된 물리 수집 상한, 최대 7 |
| 133:15 | `packed_distance_hits_17bit` | Return 0~6의 17-bit Hit, 총 119 bit |
| 134 | `hit_was_dropped` | 의도하지 않은 내부 Hit 손실이 있었음 |
| 135 | `return_overflow` | 8번째 이상 물리 Return 감지 |
| 136 | `error_fill_inserted` | timeout 복구용 placeholder가 삽입됨 |
| 137 | `cell_is_faulted` | Cell 결과에 fault가 있음 |
| 140:138 | `timeout_cause_bitmap` | timeout 원인 |
| 302:141 | `shot_context` | Shot Context |
| 318:303 | `tdc_chip_shot_sequence` | Chip별 Shot 순서 번호 |

`visible_return_count`보다 많은 물리 Hit를 EF 완료까지 Drain하는 것은 fault가 아니다.
사용자 전시 수를 초과한 Return은 패킷화 단계에서 보이지 않게 하며, 8번째 이상만
`return_overflow`다.

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

## 8. Build 설정과 Runtime Active 설정

| 종류 | 예 | 적용 규칙 |
|---|---|---|
| 합성 구조 상한 | Chip 수, STOP/Chip, 최대 Return | 합성 전 Generic으로 고정 |
| Runtime Active snapshot | Rise/Fall 활성 mask, 사용자 전시 Return 수, Shot columns | COMMIT 성공 후 안전 경계에서 적용 |
| 추적 정보 | Active version, reset epoch | 각 Event와 결과에 기록하여 적용 시점을 검증 |

Runtime 값은 합성 상한을 늘릴 수 없다. Return 수를 줄여도 외부 TDC-GPX IFIFO는
EF 완료까지 물리적으로 Drain하며, 줄어드는 것은 이후 전시·패킷화 데이터다.

## 9. 계약 수정 규칙

1. 기존 필드의 Bit 위치나 의미를 재사용하지 않는다.
2. 필드가 필요하면 먼저 reserve를 사용하거나 payload 끝에 추가한다.
3. C++ owning stage Header와 VHDL `lidar_v3_hls_contract_pkg.vhd`를 함께 수정한다.
4. `static_assert`로 전체 폭과 byte 정렬을 고정한다.
5. ABI minor를 올린다. 기존 필드 의미를 깨면 major 변경과 PS/Viewer 동시 전환이 필요하다.
6. HLS CSim, C synthesis, C/RTL CoSim을 수행한다.
7. V2/HLS 차등 테스트와 150/200 MHz OOC 배치·배선을 모두 다시 수행한다.
8. H4 이후에는 DDR Golden Word와 PS decoder 회귀도 함께 갱신한다.

## 10. ABI 3.1 검증 결과

검증 기준 Stamp는 `260810_h31_header_contract`이다.

| 단계 | HLS CSim/CoSim | V2 차등 | 150 MHz WNS | 200 MHz WNS | Latch/차단 DRC |
|---|---|---:|---:|---:|---:|
| H1 Raw-to-Hit | 4 topology PASS | 8/8 PASS | +2.112 ns | +0.688 ns | 0/0 |
| H2 Hit-to-Cell | 4 topology PASS | 8/8 PASS | +0.106 ns | +0.080 ns | 0/0 |
| H3 Cell-to-Frame | 5 topology PASS | 10/10 PASS | +0.067 ns | +0.217 ns | 0/0 |

H2/H3의 최소 WNS는 양수지만 여유가 작다. 이는 각 단계 OOC Sign-off 결과이며
Parent 전체 배치·배선 Sign-off를 대신하지 않는다. H5 혼합 Top과 H6 Parent 단계에서
혼잡, CDC, Reset, 32/64-bit packer를 포함한 전체 타이밍을 다시 확인해야 한다.

생성된 프로젝트와 상세 로그는 저장소 루트 `.work/` 아래에 있으며 Git에는 넣지
않는다. 재현 가능한 소스, script, 계약 문서만 형상관리한다.
