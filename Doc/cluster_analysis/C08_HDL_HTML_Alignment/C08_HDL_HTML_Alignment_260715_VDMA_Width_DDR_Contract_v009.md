# C08-S12 VDMA Width and DDR Contract

## 변경 목적

C08-S11에서는 `g_OUTPUT_WIDTH`가 32→64→128 bit로 커질 때 AXIS 직렬화 시간은 감소하지만, Raw VDMA byte와 HTML의 DDR 시간이 증가했다. 이번 단계는 이 현상이 HTML 계산 오류인지 현재 RTL 구조의 결과인지 분리하고, 변수와 runtime 연산을 늘리지 않는 HDL 개선 기준을 함께 시뮬레이션한다.

대상 HTML:

`C08_HDL_HTML_Alignment_260715_VDMA_Width_DDR_Contract_Simulator_v012.html`

이번 단계는 HTML 검증 모델과 개발 방향 문서 변경이다. HDL 소스는 변경하지 않았다.

## 결론

1. 현재 HTML의 폭별 Raw VDMA byte 증가는 단순 계산 오류가 아니다.
2. 현재 RTL은 cell마다 새 beat 경계를 시작하고 `TKEEP/TSTRB`를 전부 1로 전달한다.
3. `max_hits_cfg=7`일 때 current cell storage는 32/64/128 bit에서 각각 20/24/32 B다.
4. 따라서 shared DDR effective를 800 MB/s로 고정하면 current RTL DDR service도 1.720/2.040/2.680 us로 증가한다.
5. 반면 가장 긴 Rise/Fall lane의 AXIS 직렬화 시간은 1.147/0.680/0.447 us로 정상적으로 감소한다.
6. 즉, 넓은 bus가 느린 것이 아니라 폭에 따라 증가한 full-beat cell padding을 DDR에 실제 byte로 기록하는 현재 HDL 계약이 문제다.
7. HTML 기본값은 현재 HDL을 검증하기 위해 `Current RTL full-keep`를 유지한다. `Packed 32-bit-word target`은 아직 구현되지 않은 개선 목표다.

## VDMA line과 cell slot 정의

`rows_per_face`라는 RTL 이름은 VDMA의 행 수처럼 읽혀 혼동을 만든다. 실제 의미는 다음과 같다.

- VDMA line: 하나의 shot이 하나의 slope lane에 만든 packet 한 줄
- VDMA lines/Face: `cols_per_face = accepted shots/Face`, 기본 450 lines
- cell: `(chip, slope, stop, shot)` 한 조합의 hit와 metadata 묶음
- cell slots/line: 한 shot의 한 slope lane 안에서 serializer가 할당한 `(chip, stop)` 위치 수

기본 dedicated topology는 논리적으로 Rise 16 cells와 Fall 16 cells다. 그러나 현재 두 assembler가 공통 `active_chip_mask=0xF`를 받으므로 각 lane에 32 cell slots가 할당되고, Rise/Fall 각각 16개가 반대 edge group의 blank slot이다.

## 현재 RTL 폭별 계산

조건은 `max_hits_cfg=7`, header 48 B/slope, current common mask 32 cell slots/slope, Rise/Fall 별도 동시 lane, `g_AXIS_CLK_MHZ=150`, shared DDR effective 800 MB/s다.

| 순서 | 항목 | 32 bit | 64 bit | 128 bit |
|---:|---|---:|---:|---:|
| 1 | Bus bytes/beat | 4 | 8 | 16 |
| 2 | Hit slots/beat | 2 | 4 | 8 |
| 3 | Hit-data beats/cell | 4 | 2 | 1 |
| 4 | Metadata beats/cell | 1 | 1 | 1 |
| 5 | Total beats/cell | 5 | 3 | 2 |
| 6 | Current bytes/cell | 20 | 24 | 32 |
| 7 | Header beats/slope | 12 | 6 | 3 |
| 8 | Current beats/slope lane | 172 | 102 | 67 |
| 9 | Current bytes/shot, two lanes | 1,376 | 1,632 | 2,144 |
| 10 | AXIS serialize, longest lane | 1.147 us | 0.680 us | 0.447 us |
| 11 | Shared DDR service | 1.720 us | 2.040 us | 2.680 us |
| 12 | VDMA→DDR done = max(AXIS, DDR) | 1.720 us | 2.040 us | 2.680 us |

9번이 증가하는 직접 원인은 6번이다. beat 수는 줄지만 cell마다 남는 byte를 다음 cell과 합치지 못해 full-beat 저장량이 더 크게 증가한다.

## Packed 32-bit-word 목표

최대 압축인 18 B/cell을 목표로 하면 마지막 16-bit hit slot과 metadata 위치까지 바꾸어야 한다. 이는 32-bit 기본 ABI를 깨고 pack/unpack 연산을 늘린다. 이번 개선 목표는 더 보수적으로 현재 32-bit word 형식을 canonical 포맷으로 사용한다.

`max_hits_cfg=7`의 canonical cell은 다음 5개 32-bit word, 총 20 B다.

1. Hit-data word 4개: word마다 16-bit slot 2개
2. Metadata word 1개: Hit[16], validity, slope, hit count, flags, chip ID

64-bit는 canonical word 2개, 128-bit는 4개를 한 beat로 묶는다. cell 경계에서 beat를 다시 시작하지 않고 다음 cell의 word를 이어 붙인다. 기본 logical slope mask를 적용하면 한 lane은 `48 B header + 16 cells x 20 B = 368 B`이고, 두 lane은 736 B/shot이다.

| 순서 | 항목 | 32 bit | 64 bit | 128 bit |
|---:|---|---:|---:|---:|
| 1 | Canonical bytes/cell | 20 | 20 | 20 |
| 2 | Packed bytes/shot | 736 | 736 | 736 |
| 3 | AXIS beats/longest lane | 92 | 46 | 23 |
| 4 | AXIS serialize @ 150 MHz | 0.613 us | 0.307 us | 0.153 us |
| 5 | Shared DDR service @ 800 MB/s | 0.920 us | 0.920 us | 0.920 us |
| 6 | VDMA→DDR done | 0.920 us | 0.920 us | 0.920 us |

고정 800 MB/s 조건에서는 DDR이 병목이므로 폭을 넓혀도 최종 완료 시간이 0.920 us보다 짧아지지 않는다. 이것은 정상이다. DDR 상한을 5,000 MB/s로 높인 회귀에서는 VDMA→DDR 완료가 0.613/0.307/0.153 us로 감소해 bus 폭 효과가 그대로 나타난다.

## HTML 반영 사항

- `Pipeline DDR contract`에 `RTL full-keep`와 `Packed 32b words`를 추가했다.
- shared DDR effective는 Rise/Fall 두 lane이 공유하는 aggregate 실측 대역폭으로 정의했다.
- lane당 AXIS 이론 상한과 활성 lane 합계 상한을 분리해 표시한다.
- 파이프라인의 주황색 구간을 순수 DDR 시간처럼 보이는 `DDR sink`가 아니라 `VDMA→DDR completion`으로 수정했다.
- Timing T06에 선택 계약, lane 동시성, current RTL 기준, packed 목표 기준, shared DDR service를 분리했다.
- Data D03은 semantic field, canonical 32-bit cell, width-induced padding을 단계적으로 표시한다.
- Data D04는 `rows/slope` 대신 `cell slots/shot/slope lane`을 사용하고 한 shot이 한 VDMA line임을 명시한다.
- Data D05에 32/64/128-bit의 current/target byte, AXIS, DDR, 완료 시간을 같은 순서로 비교한다.
- D06→D09는 Raw VDMA padding을 버리고 D02의 유효 sample 수로 Ethernet Repack을 계속한다.

## HDL 근거

- `tdc_gpx_pkg.vhd:881-896`: output width와 `max_hits_cfg`로 hit beats를 계산하고 metadata beat 1개를 더한다.
- `tdc_gpx_cell_builder.vhd:404`: metadata에 Hit[16] vector가 배치된다.
- `tdc_gpx_cell_builder.vhd:448`: 32/64/128-bit full-keep Phase A만 허용한다.
- `tdc_gpx_face_assembler.vhd:332`: face assembler가 full-keep Phase A를 명시한다.
- `tdc_gpx_face_assembler.vhd:368-369, 441-442`: cell stream의 `TKEEP/TSTRB`를 전부 1로 입력한다.
- `tdc_gpx_output_stage.vhd:217`: output stage가 full-keep Phase A를 명시한다.
- `tdc_gpx_output_stage.vhd:256, 298`: Rise/Fall assembler 양쪽에 같은 `i_face_active_mask`를 전달한다.
- `tdc_gpx_output_stage.vhd:344-345, 388-389`: downstream skid buffer에도 `TKEEP/TSTRB`를 전부 1로 전달한다.
- `tdc_gpx_output_stage.vhd:428, 471`: Rise/Fall header inserter에 같은 `i_rows_per_face`를 전달한다.
- `tdc_gpx_face_seq.vhd:368-374`: 공통 active mask popcount와 stops/chip으로 단일 rows 값을 만든다.

## 합리적인 HDL 수정 순서

새 CSR 수를 늘리지 않는 방향은 다음과 같다.

1. 내부 용어부터 `rows_per_face`를 `cell_slots_per_shot_per_slope`로 alias하거나 rename한다. `cols_per_face`는 `lines_per_face`라는 설명 alias를 둔다.
2. 기존 `active_chip_mask`와 고정 topology generic 또는 1-bit topology mode에서 Rise/Fall mask를 파생한다. 임의 mask CSR 두 개는 추가하지 않는다.
3. Rise/Fall assembler와 header config에 slope별 mask와 slope별 cell count를 전달해 반대 edge group의 blank cell slot을 제거한다.
4. cell builder의 의미 포맷은 그대로 두고 32-bit word sequence를 canonical 내부 스트림으로 정의한다.
5. output stage에서 `c_WORDS_PER_BEAT = g_OUTPUT_WIDTH/32`로 1/2/4 word를 묶는다. 이는 synthesis generic 상수이며 runtime 곱셈·나눗셈이나 CSR가 아니다.
6. cell 경계가 아니라 line 경계에서만 16-byte alignment를 적용한다. 모든 지원 폭이 16의 약수이므로 HSIZE/stride가 폭과 독립적이고 line 끝까지 full `TKEEP`를 유지할 수 있다.
7. 현재 32-bit packet은 ABI 기준으로 유지하고 64/128-bit가 같은 canonical byte sequence를 생성하는지 byte-for-byte 비교한다.

이 순서는 먼저 per-slope mask로 32개 slot을 16개로 줄이고, 다음으로 64/128-bit의 cell-boundary padding을 제거한다. 두 변경을 한 번에 넣기보다 각 단계별 system TB와 HTML 회귀를 통과시킨 뒤 다음 단계로 이동하는 것이 안전하다.

## 필수 검증

1. 32/64/128-bit가 동일한 canonical byte sequence를 생성한다.
2. 기본 dedicated mode에서 Rise/Fall 각각 16 valid cells/line이며 blank counterpart slot이 없다.
3. dual-edge mode도 Rise/Fall 별도 lane과 각각 16 cells/line을 유지한다.
4. 16-byte line alignment 이후 HSIZE와 stride가 세 폭에서 동일하다.
5. backpressure 중 word accumulator와 cell/line boundary가 보존된다.
6. `TLAST`는 shot line 끝, Face 종료는 `cols_per_face`번째 accepted line에서만 발생한다.
7. 32-bit 기존 parser와 C07 회귀가 유지된다.
8. 64/128-bit에서 current full-keep byte 증가가 제거되고 AXIS beat 수만 감소한다.

## 실행 검증

- JavaScript syntax 검사 통과
- 브라우저 console error/warning 없음
- 기본 32-bit RTL full-keep: 20 B/cell, 1,376 B/shot, AXIS 1.147 us, shared DDR 1.720 us 확인
- 64-bit RTL full-keep: 24 B/cell, 1,632 B/shot, AXIS 0.680 us, shared DDR 2.040 us 확인
- 128-bit RTL full-keep: 32 B/cell, 2,144 B/shot, AXIS 0.447 us, shared DDR 2.680 us 확인
- Packed target: 32/64/128-bit 모두 736 B/shot, shared DDR 0.920 us 확인
- Packed target AXIS: 0.613/0.307/0.153 us로 단조 감소 확인
- DDR 800 MB/s에서 packed VDMA→DDR 완료 0.920/0.920/0.920 us 확인
- DDR 5,000 MB/s에서 packed VDMA→DDR 완료 0.613/0.307/0.153 us 확인
- 폭과 DDR 계약 전환에도 Ethernet Repack margin 4.220 ms와 2 B/sample 계약이 유지됨을 확인
- 계산표 Width regression PASS, 목표 Frame rate PASS 확인
- 기본 화면을 32-bit, RTL full-keep, shared DDR 800 MB/s로 복원
