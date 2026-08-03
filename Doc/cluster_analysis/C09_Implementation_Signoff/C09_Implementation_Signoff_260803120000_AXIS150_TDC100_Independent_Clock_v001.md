# AXIS 150 MHz / TDC 100 MHz 독립 클럭 검증

## 1. 목적

실제 PCB와 TDC-GPX 물리 timing 때문에 200 MHz TDC clock을 사용할 수 없는 경우를 대비해, AXIS processing clock보다 TDC clock이 느린 구성을 검증한다.

대상 구성은 다음과 같다.

- Device: `xc7z020clg484-2`
- AXIS processing: 150 MHz
- TDC BUS/control: 100 MHz
- Stream mode: `ASYNC`
- GPX: 4 chips, Rise `0011`, Fall `1100`
- STOP: chip당 8개, 전체 16 APD의 slope 전용 pair
- Return: STOP당 최대 7개
- Result AXIS: 32/64/128 bit

## 2. 설계 판정

`ASYNC` mode에서는 `g_AXIS_CLK_MHZ`와 `g_TDC_CLK_MHZ`의 대소관계를 제한하지 않는다. TDC raw stream은 `xpm_fifo_async`를 거치고 제어·설정·상태 경로도 명시적인 CDC primitive를 사용한다.

`SYNC` mode는 예외이다. AXIS와 TDC 주파수가 같아야 하며, 실제로 같은 clock net이거나 STA에서 동기 관계가 보장되어야 한다.

독립 클럭 허용과 처리량 만족은 서로 다른 판정이다. 다음 식을 만족하는 운용점만 사용할 수 있다.

```text
point_interval
  > range_wait + GPX_bus_drain + CDC + Cell/Face + AXIS_output + guard
```

## 3. GPX BUS timing

```text
Tclk_ns         = 1000 / g_TDC_CLK_MHZ
bus_word_ns     = bus_clk_div * bus_ticks * Tclk_ns
read_capture_ns = ((bus_ticks - 3) * bus_clk_div + 1) * Tclk_ns
```

`read_capture_ns`는 `g_BUS_READ_PERIOD_MIN_TIME_NS` 이상이어야 한다.

| TDC | div/ticks | Word period | Capture | 판정 |
|---:|---:|---:|---:|---|
| 100 MHz | 1/5 | 50 ns | 30 ns | 25 ns board policy PASS |
| 100 MHz | 2/5 | 100 ns | 50 ns | 25 ns board policy PASS, 처리량 감소 |

## 4. 발견 및 보완

### 4.1 증상

100 MHz 기능 회귀에서 raw/Cell/Face 데이터는 모두 보존됐지만 `STAT5[7:4]` chip error가 모든 chip에서 세트됐다.

### 4.2 원인

`tdc_gpx_chip_ctrl.PH_RESP_DRAIN`이 BUS가 idle이 되기를 최대 15 TDC clocks만 기다렸다. 느린 TDC clock 또는 긴 runtime BUS 설정에서는 정상 soft-reset transaction도 15 clocks 안에 끝나지 않아 `drain_cap/raw_overflow`로 오인됐다.

### 4.3 수정

- response-drain grace counter를 4 bit에서 10 bit로 확장
- grace cap을 합성 상수 1023 clocks로 변경
- 최대 legal runtime 설정 `div=63/ticks=7`과 request/turnaround/setup/hold를 포함하도록 bound 설정
- 1023 clocks 뒤에도 BUS가 active일 때만 stuck-bus quarantine 진입
- runtime 변수와 곱셈기는 추가하지 않음

## 5. 기능 검증

| Profile | Width | Control | 결과 |
|---|---:|---|---|
| 100 MHz, div1/ticks5, 0.24 deg | 32/64/128 | unified CSR | PASS |
| 100 MHz, div2/ticks5, 0.30 deg | 32/128 | unified CSR | PASS |
| 100 MHz, div1/ticks5, 0.24 deg | 32 | local CSR | PASS |
| 기존 150/200 MHz 기준 profile | 32 | unified CSR | PASS |
| C06 전체 chip/controller 회귀 | 전체 기존 matrix | local RTL | PASS |

두 100 MHz profile 모두 다음 계약을 만족했다.

- `STAT5=0x00000001`
- `STAT6=0xF0000000`
- `STAT7=0x00000000`
- Rise/Fall Hit 17-bit exact compare PASS
- raw GPX word 28-bit compare PASS
- pipeline abort 0

### 5.1 실측 운용점

| BUS | RPM / 분해능 | Point interval | Fire-to-output max | Margin |
|---|---:|---:|---:|---:|
| div1/ticks5 | 1200 RPM / 0.24 deg | 2500 AXIS clk | 2298 AXIS clk | 202 clk, 1.347 us |
| div2/ticks5 | 1200 RPM / 0.30 deg | 3125 AXIS clk | 2803 AXIS clk | 322 clk, 2.147 us |

1200 RPM / 0.20 deg / Return-7은 TDC 100 MHz에서 PASS profile이 아니다. 다음 shot이 이전 결과의 완료보다 먼저 도착했다. 0.22 deg도 assembler margin이 지나치게 작아 권장 profile에서 제외했다.

## 6. 합성 및 구현

### 6.1 독립형 `tdc_gpx_top`

- Width: 64 bit
- AXIS/TDC: 150/100 MHz
- Post-route WNS: `+0.770 ns`
- Post-route WHS: `+0.066 ns`
- Unrouted nets: 0
- Timing constraints met: PASS

결과 위치:

`signoff_results/sessions/260803115656_axis150_tdc100_w64_a150_t100_p1111_r0011_f1100_impl`

### 6.2 통합 `tdc_gpx_lidar_ctrl`

revision 18 package에서 다음 여섯 OOC 합성이 모두 PASS했다.

- Echo enabled: 32/64/128 bit, IBUFDS 32
- External STOP: 32/64/128 bit, IBUFDS 0
- 각 build: `my_axil_csr32_top=1`, local CSR=0, black box=0, LUTAR-1=0

## 7. 운용 규칙

1. `g_TDC_CLK_MHZ`는 실제 TDC FCLK와 XDC 주파수에 맞춘다.
2. 분리 clock은 `g_STREAM_CLK_MODE="ASYNC"`로 합성한다.
3. `bus_clk_div/bus_ticks`는 PCB capture margin을 만족하도록 정한다.
4. 같은 BUS 설정으로 최악 STOP/Return에서 scan budget을 다시 계산한다.
5. `max_range_5ns_ticks`와 `max_scan_5ns_ticks`는 계속 5 ns 기준 CSR 값으로 입력한다. RTL이 각 소비 domain clocks로 올림 변환한다.
6. frame 운용 전에 `STAT5/6/7`, shot margin, schedule overrun을 함께 확인한다.

## 8. Sign-off 경계

RTL CDC, 기능 회귀, packaged OOC, 독립형 post-route는 PASS이다. 다음 항목은 실제 보드에서 닫아야 한다.

- AC7021B pin/bank voltage와 PCB skew
- 실제 GPX의 `tV-DR`, RDN/WRN pulse, D-bus capture margin
- 100 MHz 이하 후보에서의 logic analyzer 측정
- 목표 RPM/수평 분해능/최대거리/Return 조합별 장시간 운용

따라서 현재 결과는 **TDC clock이 AXIS보다 느린 구조의 FPGA sign-off**이며, 모든 스캔 운용점이나 실물 GPX timing의 무조건 보장은 아니다.
