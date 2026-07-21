# C08 slope mask / falling 경로 closure

- 작성일: 2026-07-21
- 대상: `tdc_gpx_top`
- FPGA 기준: `xc7z020clg484-2`, PS FCLK 사용
- 판정: **모듈 범위 RTL 기능 및 OOC 합성 closure PASS**
- HTML: [C08 Slope Mask and Falling Contract Simulator v015](C08_HDL_HTML_Alignment_260721_Slope_Mask_Falling_Contract_Simulator_v015.html)
- 사용 설명서: [tdc_gpx_top 신호처리 상세 사용 설명서](../../tdc_gpx_top_신호처리_상세_사용_설명서.md)

## 1. 확정한 계약

| 구분 | 확정 내용 |
|---|---|
| 외부 ABI | chip slot은 4개로 고정한다. 실제 합성 chip은 `g_PRESENT_CHIP_MASK`로 선택한다. |
| 정적 slope 역할 | `g_RISE_CHIP_MASK`, `g_FALL_CHIP_MASK`; 같은 bit 중복은 same-chip dual-edge이다. |
| 정적 제약 | 모든 present chip은 한 개 이상의 역할을 가지며 Rise-capable 수는 Fall-capable 수 이상이다. |
| runtime Fall ON | Rise=`active AND g_RISE_CHIP_MASK`, Fall=`active AND g_FALL_CHIP_MASK` |
| runtime Fall OFF | 모든 active/present chip을 Rise로 사용하고 Fall lane은 idle, Fall HSIZE는 0이다. |
| 합성 제거 | `g_FALL_CHIP_MASK=0000`이면 Fall builder, assembler, FIFO, line packer, header가 generate되지 않는다. |
| CSR | Chip `CTL21/SCAN_TIMEOUT[19] falling_enable`, reset 1 |
| capability 표시 | Pipeline `HW_CONFIG[28] HAS_FALLING` |
| header 표시 | Face header word 3 bit 31에 accepted Face의 `falling_enable` snapshot 저장 |
| GPX Reg0 | 허용된 `CFG_IMAGE[0]` edge bit는 유지하고 chip 역할상 금지된 TRiseEn/TFallEn group은 0으로 제거한다. |
| 출력 정렬 | 각 slope에서 chip ID 오름차순, 각 chip 안에서 stop ID 오름차순이다. |

Fall-capable bitstream에서 CSR로 Fall을 끄는 것은 회로 제거가 아니라 runtime idle이다. 자원 절감이 필요하면 합성 전에 `g_FALL_CHIP_MASK=0000`을 사용해야 한다.

## 2. Git 기준점

| Commit | 내용 |
|---|---|
| `89890ca` | slope topology generic/CSR/RTL/TB 일반화 |
| `cd900b3` | parent reference와 OOC profile 정렬 |
| `c56a7c5` | OOC vector generic 전달 및 generated topology assertion |
| `6aad29c` | 사용 설명서, register map, top 주석의 slope 계약 정합화 |

## 3. 최종 xsim 회귀

- 실행: `scripts/run_c06_v002_regression.ps1 -NoArchiveOnExit`
- Vivado/xsim: 2025.2.1
- stamp: `260721183716`
- 종료 코드: 0
- 실행 시간: 796 s
- archive: `sim_results/vivado_xsim/sessions/260721183716_c08_slope_mask_falling_closure`
- 현재 stamp simulation log: 30개

| Profile | Rise | Fall | VDMA HSIZE R/F | 핵심 판정 |
|---|---:|---:|---:|---|
| runtime Fall OFF, 4 active, 2 stops, 64-bit | 14 beats / 1 TLAST | 0 / 0 | 112 / 0 B | PASS |
| static Rise-only, 4 chip x 8 stops, 32-bit | 76 / 1 | 0 / 0 | 304 / 0 B | 32-cell exact chip/stop order PASS |
| 3-chip 2R+1F, 2 stops, 64-bit | 10 / 1 | 8 / 1 | 80 / 64 B | PASS |
| 1-chip dual-edge, 2 stops, 64-bit | 8 / 1 | 8 / 1 | 64 / 64 B | PASS |

추가로 다음 항목을 같은 회귀에서 확인했다.

- 32/64/128-bit top integration 및 canonical packing
- AXIS 150 MHz / TDC 200 MHz 조합
- bounded output backpressure에서 beat/TLAST 보존
- same-chip Rise payload `0x0011`, Fall payload `0x0022` 독립 보존
- chip별 GPX Reg0 edge-role programming
- masked-slope hit의 `STAT7[15]` set, stop/abort 보존, soft-clear lifecycle
- build-profile limit 및 zero-alias 회귀

## 4. OOC 합성

두 clean run 모두 `xc7z020clg484-2`, output 32-bit, AXIS 150 MHz, TDC 200 MHz이며 timing/blackbox/no-clock/internal-unconstrained/CDC wrapper gate를 통과했다.

| Build | Generated topology | LUT | FF | LUTRAM | AXIS WNS | TDC WNS |
|---|---|---:|---:|---:|---:|---:|
| `P1111/R0011/F1100` | Rise builder 4, Fall builder 2, Fall output chain 존재 | 17,252 | 23,381 | 2,296 | 1.246 ns | 0.800 ns |
| `P1111/R1111/F0000` | Rise builder 4, Fall builder 0, Fall output chain 0 | 13,832 | 20,088 | 1,504 | 1.338 ns | 0.843 ns |

Rise-only build의 감소량은 LUT 3,420개(19.8%), FF 3,293개(14.1%), LUTRAM 792개(34.5%)이다. 따라서 Fall 비활성 합성 제거는 RTL generate 구조와 generated netlist instance count, utilization에서 모두 확인됐다.

OOC session:

- `signoff_results/sessions/260721174000_slope_split_clean_w32_a150_t200_p1111_r0011_f1100_synth`
- `signoff_results/sessions/260721174000_rise_only_clean_w32_a150_t200_p1111_r1111_f0000_synth`

## 5. Parent reference 범위

실제 parent 구현 프로젝트는 현재 없다. 새 reference BD는 `xc7z020clg484-2`, PS FCLK0/1/2=100/150/200 MHz 조건으로 생성·validate했으며 42개 contract check를 통과했다.

- session: `260721172000_slope_masks_ps_fclk_parent_ref`
- 범위: BD 생성, port/generic/clock/address contract validate
- 제외: implementation, route timing, board pin timing, VDMA/DDR/Ethernet 실장 측정

따라서 parent 결과는 통합 인터페이스 기준점이지 완성 시스템 timing sign-off가 아니다.

## 6. C08 HTML 검증

C08-S15는 build capability와 runtime effective mask를 분리한다. 다음 입력을 직접 변경할 수 있다.

- `g_PRESENT_CHIP_MASK`
- `g_RISE_CHIP_MASK`
- `g_FALL_CHIP_MASK`
- CSR `active_chip_mask`
- CSR `falling_enable`

Chrome headless에서 inline JavaScript parse, 내장 slope contract self-test, DOM render를 확인했다.

| Preset | APD | Runtime R/F | HSIZE R/F | 판정 |
|---|---:|---:|---:|---|
| 기본 2R+2F split | 16 | `0011/1100` | 368/368 B | PASS |
| 4-chip Rise-only | 32 | `1111/0000` | 688/0 B | PASS |
| 3-chip 2R+1F | 8 | `0011/0100` | 368/208 B | PASS |
| 1-chip dual-edge | 8 | `0001/0001` | 208/208 B | PASS |
| 4-chip dual-edge | 32 | `1111/1111` | 688/688 B | PASS |
| Rise-only build에서 Fall ON 요청 | 32 | `1111/0000` | 688/0 B | START REJECT / CHECK |
| split build, active request `0000` | 8 | clamp 후 `0001/0000` | 208/0 B | START REJECT / CHECK |

invalid profile도 HSIZE 계산만 계속하는 것이 아니라 실제 `face_seq`와 같은 reject 조건으로 상단 Frame-rate 판정을 `CHECK`로 내린다. zero-cell lane은 header-only 48 B로 계산하지 않고 RTL `fn_lane_hsize`와 같이 HSIZE 0으로 표시한다.

데스크톱 1920x1080과 좁은 430 px 화면도 렌더링해 새 mask 입력 및 상태표시가 겹치지 않음을 확인했다.

## 7. 최종 판정과 남은 경계

다음 범위는 closure 가능하다.

1. slope generic/CSR/GPX Reg0/Cell/VDMA 기능 정합성
2. 1/3/4-chip 및 split/dual/Rise-only profile
3. 32채널 chip/stop 출력 순서
4. static Fall 제거의 합성 효과
5. C08 계산 모델과 RTL geometry의 일치

다음은 실제 parent 구현 프로젝트가 생긴 뒤 별도 sign-off가 필요하다.

1. PS FCLK가 연결된 전체 design의 implementation/post-route timing
2. GPX I/O pin constraint와 보드 waveform
3. VDMA/DDR 장기 backpressure 및 실제 DDR arbitration
4. UDP/TCP 1440-byte payload repack의 software/network 실측

현재 기준점부터는 기능 변경보다 복잡도 감소와 최적화를 진행할 수 있다. 단, slope mask assertion, 32채널 exact-order TB, Reg0 역할 monitor, Fall netlist instance-count gate는 최적화 전후에 그대로 유지해야 한다.
