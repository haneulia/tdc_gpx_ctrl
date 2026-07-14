# C08-S13 Packed VDMA Geometry Closure

## 목적

이번 단계는 다음 세 문제를 RTL과 HTML에서 동시에 닫는다.

1. dedicated slope 구성에서 Rise/Fall assembler가 공통 `active_chip_mask`를 사용해 각 lane에 32개 cell slot을 만들던 문제
2. `g_OUTPUT_WIDTH`가 64/128 bit로 커질수록 cell마다 남은 full beat가 DDR byte로 기록되던 문제
3. VDMA `HSIZE`, `VSIZE`, `STRIDE`가 어떤 데이터 구조에서 도출되는지 정의되지 않았던 문제

대상 HTML:

`C08_HDL_HTML_Alignment_260715_Packed_VDMA_Geometry_Closure_Simulator_v013.html`

## 결론

- 현재 RTL은 Rise/Fall lane별 chip mask를 사용한다.
- 기본 dedicated 2+2 구성에서 한 shot의 cell slot은 Rise 16개, Fall 16개다.
- cell slot은 VDMA line 수가 아니라 한 slope line 안의 `(chip, stop)` 고정 위치다.
- VDMA line 수는 `cols_per_face`, 즉 Face에서 accepted된 shot 수다.
- 32-bit canonical word를 cell 경계 너머로 이어 붙여 64/128-bit의 per-cell padding을 제거했다.
- `max_hits_cfg=7`의 저장 cell 크기는 모든 출력 폭에서 20 B다.
- 기본값의 `HSIZE`는 lane당 368 B, `VSIZE`는 `cols_per_face`, tight buffer의 `STRIDE`는 368 B다.
- 32/64/128-bit에서 한 lane의 출력 beat는 92/46/23으로 감소하지만 DDR byte는 368 B로 같다.
- 신규 CSR은 추가하지 않았다. topology는 compile-time generic 하나로 결정한다.

## 데이터 경로

```text
cell_builder
  -> face_assembler (slope-specific mask)
  -> face FIFO
  -> line_packer (canonical 32-bit words, cross-cell packing)
  -> header_inserter (48 B prefix per line)
  -> Rise/Fall AXI4-Stream
  -> VDMA S2MM / DDR
```

`tdc_gpx_line_packer`는 cell의 의미 포맷을 바꾸지 않는다. 기존 source beat에서 의미 있는 32-bit word만 뽑아 다음 cell의 word와 연속 배치한다.

## Cell과 Cell Slot

### Cell

Cell은 `(chip, slope, stop, shot)` 한 조합의 hit와 metadata 묶음이다.

`max_hits_cfg=7`이면 한 cell의 canonical word는 다음과 같다.

| 순서 | 내용 | 크기 |
|---:|---|---:|
| 1 | Hit[15:0] slot 0, 1 | 32 bit |
| 2 | Hit[15:0] slot 2, 3 | 32 bit |
| 3 | Hit[15:0] slot 4, 5 | 32 bit |
| 4 | Hit[15:0] slot 6, unused half-word | 32 bit |
| 5 | Hit[16] vector, valid, slope, hit count, flags, chip ID | 32 bit |

따라서 canonical cell은 `ceil(7/2) + 1 = 5 word = 20 B`다. Hit[16]은 metadata에 계속 보존된다.

### Cell Slot

Cell slot은 한 shot의 한 slope lane에서 `(chip, stop)`가 들어갈 고정 주소다.

```text
cell_slots_lane = popcount(lane_chip_mask) * stops_per_chip
```

기본 dedicated 구성은 다음과 같다.

| Lane | Chip mask | Chip 수 | Stops/chip | Cell slots/line |
|---|---:|---:|---:|---:|
| Rise | `0x3` | 2 | 8 | 16 |
| Fall | `0xC` | 2 | 8 | 16 |

Cell slot이 필요한 이유는 hit가 없는 stop도 같은 `(chip, stop)` 위치에서 찾을 수 있게 하기 위해서다. 즉, 16은 VDMA vertical line 수가 아니라 line 내부의 deterministic record count다.

이전 공통 mask 방식은 두 lane 모두 `0xF`를 사용해 32 slot을 만들었다. dedicated 구성에서는 lane마다 반대 slope group 16개가 blank였다. 현재 RTL은 이 blank slot을 제거했다.

## Slope Topology

`tdc_gpx_top`에 `g_SLOPE_CHIP_MODE` generic을 추가했다.

| Mode | Rise mask | Fall mask | 용도 |
|---|---|---|---|
| `DEDICATED_2X2` | `active_chip_mask and 0x3` | `active_chip_mask and 0xC` | chip0/1 Rise, chip2/3 Fall |
| `SHARED_DUAL_EDGE` | `active_chip_mask` | `active_chip_mask` | 같은 active chip을 두 slope lane에 사용 |

Slope mask CSR 두 개를 추가하지 않았다. board topology는 합성 시 고정되는 특성이므로 generic 하나가 더 직관적이고 runtime 연산도 늘리지 않는다.

## Padding 분류

Padding을 다음처럼 구분해야 한다.

| 종류 | 원인 | S13 처리 | 기본 max_hits=7 |
|---|---|---|---:|
| Canonical half-word | 홀수 hit 수에서 마지막 32-bit hit word의 절반이 남음 | 기존 32-bit ABI 보존을 위해 유지 | 2 B/cell |
| Wide source-beat unused lane | 64/128-bit source beat가 cell 끝에서 남음 | line packer가 제거 | DDR 0 B |
| Per-cell beat-boundary padding | 다음 cell을 새 beat에서 시작 | cell 경계를 넘어 word를 이어 붙여 제거 | DDR 0 B |
| Line-end alignment | 32/64/128-bit 모두 full `TKEEP/TSTRB`로 끝내기 위한 16 B 정렬 | 현재 유지 | 0 B/line |
| 48 B prefix reservation | 모든 VDMA line의 길이를 같게 유지 | 현재 유지 | 48 B/line |

### 왜 16 B line-end padding을 남겼는가

지원 beat 크기는 4/8/16 B이며 최소공배수는 16 B다. Line 끝을 16 B에 맞추면 다음 특성이 동시에 유지된다.

- 모든 출력 beat의 `TKEEP/TSTRB`가 전부 1
- 32/64/128-bit에서 동일한 `HSIZE`와 `STRIDE`
- partial final beat 또는 DRE 지원 여부에 의존하지 않음
- VDMA line parser와 DDR addressing이 단순함

이 padding은 cell마다 붙지 않고 line 전체에 0~12 B만 붙는다. 없애는 것도 가능하지만, 먼저 실제 memory path에서 partial final `TKEEP`, 비정렬 `HSIZE`, byte-enable write가 모두 보장되는지 검증해야 한다. 그 계약 없이 제거하면 마지막 beat의 무효 byte가 다음 line을 오염시킬 수 있다.

### 48 B prefix는 padding인가

엄밀히는 alignment padding이 아니라 고정 prefix 영역이다. 첫 line에는 실제 Face header가 들어가고, 이후 line에는 같은 48 B 위치에 0이 들어간다. VDMA가 한 frame에 하나의 `HSIZE`만 사용하므로 모든 line을 같은 구조로 만들기 위한 선택이다.

48 B 반복 비용을 제거하려면 다음 중 하나가 필요하다.

- Face header를 별도 buffer 또는 descriptor에 저장
- line 0과 나머지 line을 서로 다른 VDMA transaction으로 분리
- DDR 이후 repacker가 별도 header plane을 합성

현재 목표인 변수/연산 최소화에는 uniform line 구조가 더 합리적이다. 다만 Face당 `48 B * VSIZE`가 예약되므로 향후 메모리 효율 최적화 항목으로 분리한다.

## VDMA Geometry 정의

한 accepted shot이 각 slope lane의 VDMA line 하나가 된다.

```text
canonical_cell_bytes = 4 * (ceil(max_hits_cfg / 2) + 1)
cell_data_bytes_lane = cell_slots_lane * canonical_cell_bytes
aligned_cell_bytes   = align_up(cell_data_bytes_lane, 16)
HSIZE_lane           = 48 + aligned_cell_bytes
VSIZE                = cols_per_face
STRIDE_lane          = HSIZE_lane        -- tight buffer
buffer_bytes_lane    = STRIDE_lane * VSIZE
```

기본 `max_hits_cfg=7`, 2 chips/lane, 8 stops/chip 조건:

```text
canonical_cell_bytes = 4 * (ceil(7/2) + 1) = 20 B
cell_slots_lane      = 2 * 8 = 16
cell_data_bytes      = 16 * 20 = 320 B
line_end_padding     = align16(320) - 320 = 0 B
HSIZE                = 48 + 320 = 368 B
VSIZE                = cols_per_face
STRIDE               = 368 B
```

HTML 기본 `cols_per_face=450`이면 lane당 Face buffer는 `368 * 450 = 165,600 B`다. Rise/Fall 두 buffer 합계는 331,200 B/Face다.

`TLAST`는 shot line의 마지막 beat에 표시한다. `TUSER[0]`의 SOF는 Face의 첫 line 첫 beat에만 표시한다. Header word의 `rows_per_face` 필드는 이름을 유지하지만 의미는 해당 slope lane의 `cell_slots_per_line`이다.

## 출력 폭과 DDR 시간

기본 한 lane `HSIZE=368 B`, 두 lane 736 B/shot 기준:

| g_OUTPUT_WIDTH | Bytes/beat | Beats/lane | AXIS accepted-beat occupancy @150 MHz | Shared DDR @800 MB/s | VDMA→DDR done |
|---:|---:|---:|---:|---:|---:|
| 32 | 4 | 92 | 0.613 us | 0.920 us | 0.920 us |
| 64 | 8 | 46 | 0.307 us | 0.920 us | 0.920 us |
| 128 | 16 | 23 | 0.153 us | 0.920 us | 0.920 us |

폭이 커질수록 accepted beat 수와 순수 AXIS 점유시간은 절반으로 줄어든다. DDR byte는 동일하므로 DDR service 시간은 늘지 않는다.

800 MB/s 조건에서는 shared DDR이 병목이라 최종 완료 시간이 세 폭 모두 0.920 us일 수 있다. 이는 폭 증가가 무효라는 뜻이 아니다. Upstream producer가 beat를 연속으로 만들 수 있고 DDR 상한이 충분히 높을 때 32/64/128-bit 효과가 92/46/23 beat로 나타난다. 실제 end-to-end latency는 TDC read와 cell 생성 bubble이 더 클 수 있으므로 AXIS occupancy와 전체 producer latency를 구분해야 한다.

## RTL 변경

- `tdc_gpx_pkg.vhd`
  - canonical cell word/byte, align, VDMA line byte helper 추가
- `tdc_gpx_line_packer.vhd`
  - 의미 있는 32-bit word 추출
  - cell 경계 연속 packing
  - line 끝 16 B 정렬
- `tdc_gpx_top.vhd`
  - `g_SLOPE_CHIP_MODE` 추가
  - slope별 mask와 cell slot 계산
  - `o_vdma_hsize_bytes_rise/fall`, `o_vdma_vsize_lines` 출력 추가
- `tdc_gpx_output_stage.vhd`
  - Rise/Fall assembler에 별도 mask 적용
  - FIFO와 header 사이에 line packer 삽입
- `tdc_gpx_header_inserter.vhd`
  - lane mask와 lane cell count를 header에 기록
  - canonical cell byte를 header cell size로 기록
- `tdc_gpx_face_seq.vhd`
  - dedicated group 유효성 검사
  - legacy/common geometry도 canonical line byte 식으로 정정

## 검증 결과

### Standalone line packer

- 32-bit PASS
- 64-bit PASS
- 128-bit PASS
- backpressure와 line-end padding 포함

### Dedicated 4-chip integration

| Width | Rise beats | Fall beats | HSIZE Rise/Fall | VSIZE | 결과 |
|---:|---:|---:|---:|---:|---|
| 32 | 92 | 92 | 368/368 B | 1 | PASS |
| 64 | 46 | 46 | 368/368 B | 1 | PASS |
| 128 | 23 | 23 | 368/368 B | 1 | PASS |

- Rise/Fall metadata 각각 16 cell 확인
- Hit[16] metadata 보존 확인
- Header regression PASS
- `max_hits_cfg=1..7` 폭별 output-stage sweep PASS

### Shared dual-edge integration

- active physical chips: 2
- Rise/Fall lane mask: 동일한 2-chip group
- Rise/Fall cell: 각각 16
- 64-bit beats: 각각 46
- HSIZE: 각각 368 B
- metadata: Rise/Fall 각각 16, nonzero 각각 8
- 결과: PASS

이 test는 shared mask와 두 slope data path를 검증한다. 동일한 한 chip에서 실제 rising/falling event를 모두 발생시키는 TDC device mode는 board/TDC 설정과 함께 별도 검증해야 한다.

## 통합 시 확정할 항목

1. Board가 `DEDICATED_2X2`인지 `SHARED_DUAL_EDGE`인지 확정
2. VDMA S2MM의 Rise/Fall base address를 분리
3. 새 `o_vdma_hsize_bytes_rise/fall`와 `o_vdma_vsize_lines`를 VDMA programming logic에 연결
4. Tight buffer를 사용하면 `STRIDE=HSIZE`; line 간 guard가 필요하면 외부 address generator에서만 stride를 늘림
5. 48 B prefix 반복 비용을 유지할지, header plane을 분리할지 시스템 메모리 예산으로 결정
6. Line-end padding 제거를 원하면 partial `TKEEP`와 비정렬 HSIZE를 실제 VDMA/DDR 환경에서 먼저 검증

현재 구현은 신규 CSR와 runtime 곱셈/나눗셈을 추가하지 않는다. geometry 계산은 Face snapshot과 compile-time width 상수를 이용하며, 지원 폭별 데이터 선택은 고정 MUX/queue 구조로 합성된다.

## 도구 환경 주의

로컬 프로젝트 파일은 Vivado 2025.2.1 형식이고 설치된 simulator는 Vivado 2023.2였다. 따라서 프로젝트 자체는 2023.2에서 열 수 없었고, 생성된 CSR IP VHDL을 포함한 direct `xvhdl/xelab/xsim` 회귀로 검증했다.

`scripts/run_c08_vdma_contract_regression.tcl`은 Vivado 2025.2 이상에서 packer와 C08 TB를 project fileset에 추가하고 dedicated 32/64/128-bit 및 shared mode를 순차 실행하도록 준비했다.
