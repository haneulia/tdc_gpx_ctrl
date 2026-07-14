# C08-S11 Cell, Beat, and Edge Topology

## 변경 목적

C08-S10의 D03 이후에는 `cell`과 `beat`의 정의가 먼저 나오지 않아 `Total beats / cell`을 자연스럽게 해석하기 어려웠다. 또한 4개 chip 전체를 세는 현재 RTL row와 실제 slope별 유효 row가 한 값처럼 표시되어, 고정 slope 4-chip 구성과 2-chip dual-edge 구성을 구분하기 어려웠다.

대상 HTML:

`C08_HDL_HTML_Alignment_260715_Cell_Beat_Edge_Topology_Simulator_v011.html`

이번 단계는 HTML 검증 모델과 설명 구조 변경이다. HDL 소스는 변경하지 않았다.

## 질문에 대한 직접 답변

1. 기본 32-bit 조건의 `4 hit beats`는 정확히 `Hit-data beats / cell`을 의미한다.
2. `cell`은 `(chip, slope, stop/channel, shot)` 한 조합에서 수집한 최대 `max_hits_cfg`개 timestamp 묶음이다.
3. `beat`는 `tvalid && tready`가 성립할 때 `g_OUTPUT_WIDTH` bit를 전달하는 AXI4-Stream 전송 1회다.
4. chip0/1이 rising, chip2/3이 falling 전용인 구성의 논리 row는 `4 chips x 8 stops`가 아니라 slope별 `2 chips x 8 stops = 16 cells`가 맞다.
5. 현재 RTL이 slope별 32 rows를 출력하는 이유는 논리적으로 4개 chip이 모두 해당 slope를 검출해서가 아니라, Rise/Fall assembler 양쪽에 공통 `active_chip_mask=0xF`를 넣기 때문이다. 반대 slope group의 16개 blank row가 각 lane에 포함된다.
6. 같은 chip0/1이 두 edge를 모두 검출하는 구성은 `2 chips x 8 stops x 2 slopes = 32 logical cells/shot`가 맞다.
7. 두 구성 모두 VDMA 출력은 Rising lane과 Falling lane이 별도로 존재한다. dual-edge는 VDMA를 합치는 모드가 아니라, 같은 두 TDC chip이 두 slope event를 만들고 이후 slope bit로 두 VDMA lane에 분리되는 모드다.

## Cell과 beat 계산

기본값은 `g_OUTPUT_WIDTH=32`, `max_hits_cfg=7`이다.

| 순서 | 계산 | 결과 |
|---:|---|---:|
| 1 | `32 bit / 8` | 4 B/beat |
| 2 | `floor(32 bit / 16-bit hit slot)` | 2 hit slots/beat |
| 3 | `ceil(7 hits / 2 slots/beat)` | 4 Hit-data beats/cell |
| 4 | 마지막 Hit-data beat | 1 valid slot + 1 padding slot |
| 5 | 고정 metadata | 1 metadata beat/cell |
| 6 | `4 Hit-data + 1 metadata` | 5 total beats/cell |
| 7 | `5 beats x 4 B/beat` | 20 B/cell |

각 16-bit hit slot에는 `Hit[15:0]`을 저장한다. `Hit[16]` vector와 `hit_valid`, `slope_vec`, `hit_count`, flags, `chip_id`는 마지막 metadata beat에 저장한다.

## 두 topology의 정확한 의미

APD 16채널, chip당 8 stops, Rising/Falling 검출을 기준으로 한다.

| 항목 | 4-chip dedicated edge groups | 2-chip shared dual-edge |
|---|---:|---:|
| 물리 TDC chip | 4 | 2 |
| Rise mask | `0x3` | `0x3` |
| Fall mask | `0xC` | `0x3` |
| 공통 active mask | `0xF` | `0x3` |
| 논리 Rise cells/shot | `2 x 8 = 16` | `2 x 8 = 16` |
| 논리 Fall cells/shot | `2 x 8 = 16` | `2 x 8 = 16` |
| 논리 전체 cells/shot | 32 | 32 |
| 현재 RTL rows/slope | 32 | 16 |
| 현재 RTL padding rows | Rise 16 / Fall 16 | Rise 0 / Fall 0 |
| chip당 edge 읽기 부하 | x1 | x2 |
| VDMA 출력 | Rise/Fall 별도 | Rise/Fall 별도 |
| 현재 검증 상태 | C07 dedicated topology 검증, per-slope mask HDL gap | RTL 구조 지원, same-chip mixed-slope 통합 TB 미완료 |

두 모드의 유효 거리 sample 수는 동일하다. 차이는 물리 chip 수, 현재 Raw VDMA padding, chip당 TDC read 부하다. dual-edge는 chip 수를 절반으로 줄이지만 같은 chip이 두 slope event 집합을 읽으므로 HTML의 TDC read 시간은 기본값에서 2.8 us에서 5.6 us로 증가한다.

## Raw VDMA packet 비교

### 32-bit 기본값

| 구성 | Rows/slope | Header beats | Beats/cell | Beats/slope | Bytes/shot |
|---|---:|---:|---:|---:|---:|
| Dedicated, 현재 RTL 공통 mask | 32 | 12 | 5 | 172 | 1,376 |
| Dedicated, 논리 slope mask 적용 | 16 | 12 | 5 | 92 | 736 |
| Shared dual-edge, 현재 RTL | 16 | 12 | 5 | 92 | 736 |

### 128-bit 비교값

| 구성 | Rows/slope | Header beats | Beats/cell | Beats/slope | Bytes/shot |
|---|---:|---:|---:|---:|---:|
| Dedicated, 현재 RTL 공통 mask | 32 | 3 | 2 | 67 | 2,144 |
| Dedicated, 논리 slope mask 적용 | 16 | 3 | 2 | 35 | 1,120 |
| Shared dual-edge, 현재 RTL | 16 | 3 | 2 | 35 | 1,120 |

이 Raw VDMA byte는 DDR 저장 형상 진단값이다. Ethernet Repack은 D02의 유효 distance sample 수를 사용하므로 topology와 `g_OUTPUT_WIDTH`가 같아도 1440 B Face Header, Distance packet 수, Repack Face payload 계산은 별도 계약으로 유지된다.

## HDL 근거

- `tdc_gpx_pkg.vhd:598-610`: raw event가 event별 `slope` bit를 가진다.
- `tdc_gpx_pkg.vhd:633-641`: `t_cell`에 `slope_vec`와 hit metadata가 존재한다.
- `tdc_gpx_pkg.vhd:881-896`: runtime `max_hits_cfg`와 출력 폭으로 beats/cell을 계산한다.
- `tdc_gpx_cell_builder.vhd:62-72`: 32/64/128-bit의 runtime total beats/cell 표가 있다.
- `tdc_gpx_cell_builder.vhd:77-97`: Hit[15:0], Hit[16], slope, stop, shot sequence와 beat layout 계약이 정의되어 있다.
- `tdc_gpx_cell_pipe.vhd:162-167`: event slope bit를 Rising/Falling으로 해석한다.
- `tdc_gpx_cell_pipe.vhd:214-265`: chip별 registered slope demux가 event를 분리한다.
- `tdc_gpx_cell_pipe.vhd:310-360`: 모든 chip에 Rise builder와 Fall builder가 각각 존재한다.
- `tdc_gpx_cfg_pkg.vhd:145-154`: TDC Reg0에 stop별 `TRiseEn`과 `TFallEn`이 모두 존재한다.
- `tdc_gpx_output_stage.vhd:238-316`: Rise/Fall face assembler가 별도로 존재하지만 둘 다 같은 `i_face_active_mask`를 받는다.
- `tdc_gpx_output_stage.vhd:411-490`: Rise/Fall header inserter와 최종 AXIS/VDMA 출력이 별도로 존재하지만 같은 config와 `i_rows_per_face`를 받는다.
- `tdc_gpx_face_seq.vhd:361-383`: 공통 `active_chip_mask` popcount와 stops/chip으로 단일 rows 값을 계산한다.
- `tb_tdc_gpx_top_int_c07_4chip_target.vhd:33-34`: C07은 active `1111`, chip slope `0011`인 dedicated 구성을 검증한다.

## 합리적인 HDL 수정 방향

변수와 runtime 연산을 최소화하려면 임의의 Rise/Fall mask CSR 두 개를 추가하기보다 기존 `active_chip_mask`와 1-bit topology 계약을 사용하는 것이 단순하다.

권고 계약:

- `shared_dual_edge=0`: fixed dedicated mapping, Rise=`active_mask AND 0x3`, Fall=`active_mask AND 0xC`
- `shared_dual_edge=1`: shared mapping, Rise=`active_mask`, Fall=`active_mask`

물리 배선이 제품별로 고정된다면 이 값은 CSR보다 synthesis generic이 더 적합하다. runtime 전환이 실제 요구될 때만 1-bit CSR로 올린다.

적용 순서:

1. `tdc_gpx_output_stage`에 파생된 Rise/Fall mask를 각각 전달한다.
2. Rise/Fall face assembler에 각 slope mask를 연결한다.
3. header용 config의 `active_chip_mask`와 `rows_per_face`도 slope별 값으로 분리한다.
4. VDMA HSIZE, stride, software parser가 header의 slope별 rows를 사용하도록 ABI를 함께 갱신한다.
5. dual-edge에서는 동일 chip의 `TRiseEn`과 `TFallEn`을 모두 활성화한다.
6. 같은 chip에서 Rise/Fall event가 섞여 들어오는 system TB를 추가하고 IFIFO depth, drain timeout, 13.888889 us shot period 내 read closure를 검증한다.
7. dedicated 회귀는 기존 C07을 유지하고, per-slope mask 적용 후 반대 slope blank row가 없어지는지 추가 검사한다.

`cell_pipe`의 slope demux와 별도 VDMA lane 구조는 유지 대상이다. 새 연산 블록을 만드는 것보다 mask와 rows의 계약을 slope별로 정확히 전달하는 변경이 핵심이다.

## 실행 검증

- JavaScript syntax 검사 통과
- 브라우저 console error/warning 없음
- 기본 32-bit dedicated: `R 0x3 / F 0xC`, 물리 4 chips, 논리 16/16 cells, RTL 32 rows/slope, padding 16/16, 1,376 B/shot 확인
- 32-bit shared dual-edge: `R 0x3 / F 0x3`, 물리 2 chips, 논리 16/16 cells, RTL 16 rows/slope, padding 0/0, 736 B/shot 확인
- dual-edge 전환 시 TDC read 2.8 us에서 5.6 us, edge-read load x1에서 x2로 변경 확인
- 128-bit dedicated: 현재 2,144 B/shot, 논리 slope mask 1,120 B/shot 확인
- 128-bit shared dual-edge: 1,120 B/shot 확인
- 32/128-bit 변경에도 Repack 203,040 B/Face와 140 Distance packets가 동일함을 확인
- 최종 화면을 32-bit dedicated 기본값으로 복원
