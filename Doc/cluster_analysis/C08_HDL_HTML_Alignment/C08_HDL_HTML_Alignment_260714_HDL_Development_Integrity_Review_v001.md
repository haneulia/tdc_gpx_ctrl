# C08 HDL Development and Integrity Review v001

- Date: 2026-07-14
- Scope: `TDC_GPX_TOP`, C07 representative configuration, C08 executable timing/packet contract
- Simulator: `C08_HDL_HTML_Alignment_260714_HDL_Integrity_Simulator_v003.html`

## 1. 결론

C08 기본 조건은 광학 주기와 현재 RTL packet 형상 기준으로 시뮬레이션 PASS이다. 다만 RTL 무결화 완료로 판정하기 전에 `max_range_clks`의 clock-domain 단위를 분리해야 한다. 현재 기본 조합인 AXIS 150 MHz / TDC 200 MHz에서는 timeout이 짧아지지 않고 보수적으로 길어지므로 즉시 데이터 손실을 만드는 조건은 아니다. 그러나 AXIS clock 변경 시 물리 시간이 달라지는 구조이므로 P0 설계 계약 문제로 분류한다.

## 2. C08 기준값

| 항목 | C08 기준 | 상태 |
|---|---:|---|
| Horizontal sampling | 450 centered bins, -44.9..+44.9 deg | 확정 권고 |
| `cols_per_face` 의미 | accepted laser shot count | 확정 권고 |
| Faces | 4 | C07 대표값 |
| Distance | 300 m | C07 대표값 |
| Echo / `max_hits_cfg` | 7 | C07 대표값 |
| Active chips | `0xF`, 4 chips | 현 RTL 기준 |
| Stops per chip | 8 | 현 RTL 기준 |
| TDC clock | 200 MHz | 확정 필요 |
| AXIS clock | 150 MHz | 확정 필요 |
| `max_range_tdc_clks` | 401 clocks | 확정 권고 |
| `max_range_axis_clks` | 301 clocks | HDL 파생값으로 추가 |
| Header prefix | 48 bytes | 현 RTL ABI |
| Output width | 128-bit C07 baseline | 64/128 최종 선택 필요 |

`451 inclusive endpoints`는 비교 모드로만 유지한다. 90 deg / 0.2 deg의 release scan은 양 끝점 중복 소유를 피하는 half-open 구간 `[-45, +45)`와 450개 bin 중심 샘플을 사용한다.

## 3. 구조 검토 결과

### P0: `max_range_clks` clock-domain 단위 혼용

- `tdc_gpx_pkg.vhd`는 값을 200 MHz 물리 왕복 시간 count로 정의한다.
- `tdc_gpx_chip_run`은 TDC 200 MHz domain에서 이 값을 사용한다.
- `tdc_gpx_stop_cfg_decode`와 `tdc_gpx_cell_builder`는 AXIS domain에서 같은 숫자를 그대로 사용한다.
- 기본 300 m에서는 401 TDC clocks = 2.005 us, AXIS domain에서 401 clocks = 2.673 us이다.
- AXIS 150 MHz에서는 보수적이지만, AXIS가 200 MHz보다 빨라지면 window가 물리 ToF보다 짧아질 수 있다.
- `g_WINDOW_MARGIN_CLKS=32`도 소스 주석은 200 MHz의 160 ns로 설명하지만 실제 인스턴스 clock은 AXIS 150 MHz라서 213 ns이다.

권고: CSR의 `max_range_clks`를 TDC-domain canonical 값으로 명시하고 AXIS-domain count를 명시적으로 ceil 변환한다. 이름도 내부에서 `max_range_tdc_clks`와 `max_range_axis_clks`로 분리한다.

### P1: C07 target regression의 시스템 조건 부족

현재 target wrapper는 AXIS 200 MHz, 1000 m, 1 face, 1 column이다. 4-chip/8-stop/7-hit packet 체인은 잘 검증하지만 150 MHz dual-clock, 450 columns, 4 faces, 연속 shot 및 face TLAST 조건을 검증하지 않는다.

권고: 빠른 packet regression은 유지하고 별도 system-contract regression을 추가한다. 모든 조합을 하나의 긴 testbench로 합치지 않는다.

### P1: 128-bit가 현재 packet ABI에서 항상 유리하지 않음

현 RTL은 cell마다 full-keep beat padding을 사용한다. 32 rows, 7 hits 조건의 정확한 packet은 다음과 같다.

| Width | Header beats | Cell beats | Beats/slope | Bytes/shot | C08 critical time |
|---:|---:|---:|---:|---:|---:|
| 64 | 6 | 3 | 102 | 1,632 B | 6.854718 us |
| 128 | 3 | 2 | 67 | 2,144 B | 7.494718 us |

128-bit는 AXIS 직렬화는 빠르지만 DDR byte 수가 31.4% 증가한다. C08의 DDR 800 MB/s 가정에서는 64-bit가 더 큰 shot margin을 제공한다. 따라서 `g_OUTPUT_WIDTH=128`은 C07 baseline으로 유지하되, 실제 VDMA 처리율과 packet ABI를 확인한 뒤 최종 확정한다.

### P2: rise/fall stream에 동일 active mask 사용

`tdc_gpx_output_stage`는 rise/fall assembler 모두 동일한 `i_face_active_mask`를 사용한다. 고정 배선이 chip0/1=rise, chip2/3=fall인 목표 시스템에서는 각 stream이 반대 slope chip의 blank cell도 출력한다.

- Current 128-bit: 32 rows/slope, 67 beats/slope, 2,144 B/shot
- Proposed fixed-slope masks `rise=0x3`, `fall=0xC`: 16 rows/slope, 35 beats/slope, 1,120 B/shot
- Packet byte reduction: 47.76%

이 변경은 header mask, HSIZE, VDMA stride 및 software parser ABI를 함께 바꾸므로 clock-domain 수정 이후의 선택적 최적화로 진행한다. chip별 slope가 runtime에 바뀔 수 있다면 적용하지 않는다.

### P2: packet 형상 계산의 중복

`tdc_gpx_pkg`에 `fn_hdr_prefix_beats`와 `fn_beats_per_cell_rt`가 있지만 face 전체 rows/bytes 계산은 `tdc_gpx_face_seq`, TB, 문서에서 다시 조합한다. `fn_rows_per_slope`, `fn_beats_per_slope`, `fn_hsize_bytes` 수준의 공통 helper를 package에 추가해 RTL과 TB가 같은 산식을 사용하도록 한다.

### P2: odd stops TB helper

`tb_tdc_gpx_top_int.vhd`의 `fn_expected_words_per_ififo`는 echo가 2개 이상일 때 `(stops_per_chip / 2) * echoes_per_stop`를 사용한다. odd stops 설정에서는 정수 나눗셈으로 기대 word가 누락될 수 있다. odd-stop test를 허용할 계획이면 IFIFO별 stop 배분을 ceil/floor로 명시해 helper를 분리한다.

## 4. 권고 HDL 적용 순서

1. **H1 clock contract**: top/config generic에 AXIS:TDC clock ratio 또는 명시적 clock frequency를 추가하고 ceil 변환 helper를 package에 구현한다. TDC path는 CSR 원값, AXIS path는 변환값을 사용한다.
2. **H2 naming/assertion**: 내부 signal과 port를 domain-qualified 이름으로 변경한다. elaboration assertion으로 ratio, 16-bit saturation, output width, margin 단위를 검사한다.
3. **H3 packet helper**: rows, beats, HSIZE 계산을 `tdc_gpx_pkg` 공통 함수로 통합하고 face sequencer의 max-hit case 중복을 제거한다.
4. **H4 verification closure**: 150/200 MHz, 300 m, 450 columns, 4 faces, 2개 이상 연속 shot, independent rise/fall backpressure를 추가한다.
5. **H5 optional packet optimization**: 고정 slope topology가 제품 계약이면 per-slope mask ABI를 별도 branch에서 적용하고 VDMA/software와 함께 전환한다.

H1-H4는 무결화 항목이고 H5는 대역폭 최적화 항목이다. 둘을 한 변경으로 묶지 않는다.

## 5. 검증 게이트

| Gate | 최소 검증 |
|---|---|
| Clock conversion | 150/200, 200/200, 250/200 MHz와 range count 0/1/267/401/1335 |
| Packet geometry | widths 32/64/128, max hits 1..7, stops 2..8, active masks 1..15 |
| Column contract | centered 450 PASS, endpoints 451 baseline-difference CHECK |
| Face sequencing | 4 faces, 450 accepted shots/face, rise/fall TLAST 각 4회 |
| Continuous shots | 최소 2 shots, 13.888889 us period, shot mixing/drop 없음 |
| Backpressure | rise-only, fall-only, both-lane independent stalls |
| System boundary | VDMA DDR write 완료와 Ethernet rest-window budget |

## 6. 현재 C08 실행 판정

기본 128-bit 조건에서 exact RTL packet은 67 beats/slope, 2,144 B/shot이다. ToF 2.001385 us, TDC read 2.8 us, AXIS/DDR 포함 critical time 7.494718 us이며 13.888889 us laser period에 대해 6.394171 us margin이 남는다.

따라서 현재 대표 조건은 timing/packet simulation PASS이다. 다만 HTML이 표시하는 `SIM PASS / CLOCK CONTRACT OPEN`이 최종 판정이다. H1 clock-domain 분리와 H4 system-contract regression이 통과한 뒤에만 RTL 무결화 완료로 승격한다.
