# TDC-GPX IP 사용 가이드

## 1. IP 역할

`tdc_gpx_top`은 1~4개의 TDC-GPX 칩을 초기화하고, 각 shot에서 TDC 데이터를 읽어 거리 sample로 변환한 뒤 rising/falling AXI4-Stream으로 출력한다.

신호 처리 순서는 다음과 같다.

1. `motor_decoder`가 고정된 다면 미러 수와 현재 face 정보를 제공한다.
2. `laser_ctrl`이 `i_shot_start`, `i_shot_face_index`, `i_stop_tdc`를 `i_axis_aclk`에 맞춰 제공한다.
3. TDC bus domain이 활성 칩의 IFIFO 데이터를 읽는다.
4. decode/cell pipeline이 stop, hit, slope 정보를 정렬한다.
5. output stage가 shot 한 개를 VDMA line 한 개로 pack한다.
6. rising/falling AXI4-Stream과 VDMA geometry를 출력한다.

상세 RTL 읽기 자료는 [tdc_gpx_top 신호처리 상세 사용 설명서](Doc/tdc_gpx_top_신호처리_상세_사용_설명서.md)를 참조한다.

## 2. 클럭 계약

| 도메인 | 포트 | 기본값 | 역할 |
|---|---|---:|---|
| AXIS processing | `i_axis_aclk` | 150 MHz | decode, cell, face, output stream, shot control |
| TDC bus | `i_tdc_clk` | 200 MHz | TDC-GPX pin bus, chip init/run/register access |
| AXI-Lite | `s_axi_aclk` | 외부 결정 | 두 CSR slave만 사용 |

`g_AXIS_CLK_MHZ`와 `g_TDC_CLK_MHZ`는 클럭을 생성하지 않는다. 실제 FCLK 및 XDC와 반드시 일치해야 한다.

- 지원 주파수: 50, 100, 125, 150, 200 MHz
- `g_STREAM_CLK_MODE="ASYNC"`: AXIS/TDC의 주파수 대소관계와 무관하게 사용할 수 있다.
- `g_STREAM_CLK_MODE="SYNC"`: 두 포트에 같은 clock net 또는 STA로 동기 관계가 증명된 동일 주파수 clock을 사용해야 한다.
- 기준 profile: AXIS 150 MHz/TDC 200 MHz와 AXIS 150 MHz/TDC 100 MHz, 모두 `ASYNC`

`ASYNC`가 주파수 순서를 허용한다는 것은 임의의 스캔 조건에서 처리량까지 보장한다는 뜻이 아니다. TDC BUS 수집 시간과 AXIS Cell/Face 출력 시간 중 더 늦게 끝나는 경로를 기준으로 shot 간 margin을 닫아야 한다. TDC-domain watchdog은 TDC clock으로, AXIS-domain watchdog은 AXIS clock으로 각각 변환된다.

GPX word 주기와 read capture 창은 다음과 같다.

```text
TdcClockPeriodNs = 1000 / g_TDC_CLK_MHZ
BusWordPeriodNs  = bus_clk_div * bus_ticks * TdcClockPeriodNs
ReadCaptureNs    = ((bus_ticks - 3) * bus_clk_div + 1) * TdcClockPeriodNs
```

`ReadCaptureNs`는 `g_BUS_READ_PERIOD_MIN_TIME_NS` 이상이어야 한다. 검증된 100 MHz 예시는 `div=1/ticks=5`의 50 ns/word와 `div=2/ticks=5`의 100 ns/word이다. 후자는 PCB margin은 커지지만 GPX drain 시간이 길어지므로 더 넓은 수평 각 간격 또는 더 낮은 RPM이 필요하다.

## 3. 합성 전 설정

| Generic | 기본값 | 의미 |
|---|---:|---|
| `g_NUM_CHIPS` | 4 | 실제 배치할 TDC-GPX 칩 수와 물리 핀 lane 수 |
| `g_PRESENT_CHIP_MASK` | `1111` | 4개의 논리 chip slot 중 존재하는 chip |
| `g_RISE_CHIP_MASK` | `0011` | rising lane을 생성하는 chip |
| `g_FALL_CHIP_MASK` | `1100` | falling lane을 생성하는 chip |
| `g_MAX_STOPS_PER_CHIP` | 8 | 칩당 합성 최대 STOP 수, 2~8 |
| `g_MAX_HITS_PER_STOP` | 7 | STOP당 합성 최대 hit 수, 1~7 |
| `g_OUTPUT_WIDTH` | 32 | AXIS TDATA 폭, 32/64/128 bit |
| `g_AXIS_CLK_MHZ` | 150 | AXIS processing clock metadata |
| `g_TDC_CLK_MHZ` | 200 | TDC bus clock metadata |

Topology 규칙:

- `popcount(g_PRESENT_CHIP_MASK) = g_NUM_CHIPS`
- present chip은 rising 또는 falling 역할을 최소 하나 가져야 한다.
- rise/fall mask는 겹칠 수 있다. 겹친 chip은 양 edge를 처리한다.
- rising-capable chip 수는 falling-capable chip 수보다 작을 수 없다.
- falling 전용 회로가 필요 없으면 `g_FALL_CHIP_MASK="0000"`으로 합성한다.

`g_OUTPUT_WIDTH`는 포트 폭과 내부 packing geometry를 바꾸는 합성 전
제너릭이다. CSR로 운용 중 변경할 수 없으며, Vivado XGUI에서는 32, 64,
128 bit 중 하나만 선택할 수 있다. 기본값은 32 bit다. 값을 바꾼 뒤에는
output products와 Block Design을 다시 생성하고, 연결된 VDMA/Data Width
Converter의 폭도 일치시켜야 한다.

유효 예:

| 목적 | NUM | PRESENT | RISE | FALL |
|---|---:|---:|---:|---:|
| 4 chip, 2 rising + 2 falling | 4 | `1111` | `0011` | `1100` |
| 4 chip, 전부 rising | 4 | `1111` | `1111` | `0000` |
| 3 chip, 2 rising + 1 falling | 3 | `0111` | `0011` | `0100` |
| 1 chip, 양 edge | 1 | `0001` | `0001` | `0001` |

물리 핀 폭은 `g_NUM_CHIPS`에 따라 바뀐다.

- `io_tdc_d`: `28 * g_NUM_CHIPS`
- `o_tdc_adr`: `4 * g_NUM_CHIPS`
- CS/RD/WR/OEN/STOPDIS/ALUTRIGGER/PURESN 및 flag: `g_NUM_CHIPS`

논리 chip ID는 present mask의 낮은 set bit부터 물리 lane 0, 1, ...에 대응한다.

## 4. IPI 연결

| 인터페이스/포트 | 연결 대상 | 주의점 |
|---|---|---|
| `s_axi` | PS AXI master | Chip CSR, 9-bit address |
| `s_axi_pipe` | PS AXI master | Pipeline CSR, 7-bit address |
| `o_m_axis` | rising VDMA/S2MM | `i_axis_aclk` domain |
| `o_m_axis_fall` | falling VDMA/S2MM | rising-only build에서는 미연결 가능 |
| `i_n_faces` | `motor_decoder.o_n_faces` | bitstream 동안 고정, 1~5 |
| `i_shot_start`, `i_shot_face_index` | `laser_ctrl` | AXIS domain pulse + payload |
| `i_stop_tdc` | `laser_ctrl` | AXIS domain 1-clock pulse |
| `i_bin_resolution_ps` | calibration source | TDC bin resolution |
| `i_k_dist_fixed` | calibration source | 고정소수점 거리 계수 |
| `o_irq` | PS IRQ concat | chip register transaction done |
| `o_irq_pipe` | 선택 연결 | 현재 source가 reserved이므로 STAT polling 사용 |

AXIS metadata의 `FREQ_HZ`는 `g_AXIS_CLK_MHZ`에 종속되고, `i_tdc_clk` metadata는 `g_TDC_CLK_MHZ`에 종속된다. CSR clock에는 고정 주파수를 두지 않는다.

## 5. Shot와 출력 계약

- `i_shot_start`가 한 shot의 시작이다.
- `i_stop_tdc`가 target range window의 종료를 알린다.
- 한 slope의 한 shot은 VDMA line 한 개가 된다.
- `TLAST`는 line 끝, `TUSER[0]`은 frame SOF 계약에 사용된다.
- AXIS backpressure는 데이터 손실 없이 보존되어야 한다. 출력 FIFO reset은 non-empty 상태에서 shot 경계 reset을 허용하지 않는다.

VDMA 설정값:

- `HSIZE_RISE = o_vdma_hsize_bytes_rise`
- `HSIZE_FALL = o_vdma_hsize_bytes_fall`
- `VSIZE = o_vdma_vsize_lines`
- tightly packed buffer이면 `STRIDE = 해당 HSIZE`

Line 형식은 48-byte prefix와 canonical cell 영역으로 구성된다. line 끝은 16-byte 정렬될 수 있으며, `g_OUTPUT_WIDTH` 증가는 같은 payload의 전송 beat 수를 줄인다. 출력 폭 자체가 DDR 시간을 늘리는 모델로 해석하면 안 된다.

## 6. 5 ns runtime tick

`max_range_5ns_ticks`와 `max_scan_5ns_ticks`는 항상 200 MHz 기준값이다.

```text
physical_time_ns = ticks_5ns * 5
local_clocks = ceil(ticks_5ns * local_clock_MHz / 200)
```

따라서 CSR에 200 MHz 기준 clock 수를 기록하면 RTL이 50/100/125/150/200 MHz의 실제 소비 domain count로 자동 변환한다. 반올림은 항상 올림이므로 요청 시간보다 짧아지지 않는다.

## 7. 권장 기동 순서

1. 두 reset을 assert하고 실제 FCLK가 안정된 뒤 release한다.
2. Chip CSR의 bus timing, chip register image, scan 정책을 설정한다.
3. Pipeline CSR의 active mask, range, columns, processing mode를 설정한다.
4. `cmd_cfg_write`를 0→1→0으로 pulse한다.
5. STATUS busy가 해제되고 chip 초기화 결과가 정상인지 확인한다.
6. `cmd_start`를 pulse한다.
7. laser shot 입력과 두 AXIS stream을 감시한다.
8. frame 종료 후 STAT5/6/7과 VDMA completion을 함께 확인한다.

## 8. 현재 검증 기준

- Device: `xc7z020clg484-2`
- 기준 회귀: AXIS/TDC 150/200 MHz
- 저속 TDC 회귀: AXIS/TDC 150/100 MHz, `ASYNC`
- Default topology: 4 chip, rise `0011`, fall `1100`
- Output: 32/64/128 bit
- Source-level CSR, black box 0
- 150/100 MHz 64-bit post-route: WNS `+0.770 ns`, WHS `+0.066 ns`, unrouted 0
- Packaged-IP OOC: 112 IOBUF, black box 0, synthesis error 0
- 150/100 MHz Return-7 기능 회귀:
  - `div=1/ticks=5`, 1200 RPM, 0.24 deg: point margin 202 AXIS clocks
  - `div=2/ticks=5`, 1200 RPM, 0.30 deg: point margin 322 AXIS clocks
- Internal encoder integration: 2 shots, rise/fall line 2/2, pipeline abort 0
- Physical encoder integration: 12 fire/start/stop/result, rise/fall line 12/12,
  AXIS TKEEP violation 0, pipeline abort 0

1200 RPM, 0.20 deg, Return-7 조건은 TDC 100 MHz에서 다음 shot 전에 이전 결과가 끝나지 않아 허용 profile이 아니다. 상위 보드 XDC에서 TDC 핀 위치, bank 전압, I/O standard, clock source를 최종 확정하고 실물 GPX read timing을 계측해야 전체 system sign-off가 된다.
