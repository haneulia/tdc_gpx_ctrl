# C08 Echo Physical STOP Contract Closure v033

Date: 2026-07-24

## 1. 목적

`echo_receiver`의 LVDS 입력부터 TDC-GPX STOP pin까지의 물리 경로와
AXIS-domain 진단 경로를 분리하고, `motor_decoder -> laser_ctrl ->
echo_receiver -> tdc_gpx_top` 통합 시 clock generic과 측정-window 종료 순서를
검증한다.

## 2. 확정된 연결

```text
photodiode LVDS
  -> echo_receiver IBUFDS
  -> echo_receiver.o_tdc_stop
  -> physical TDC-GPX STOP pins

laser_ctrl.o_shot_start
  -> echo_receiver.i_shot_start
  -> tdc_gpx_top.i_shot_start

laser_ctrl.o_stop_tdc
  -> echo_receiver.i_stop_tdc
  -> tdc_gpx_top.i_stop_tdc
```

`o_tdc_stop`은 단일 physical waveform이다. Echo에서 rising/falling 신호를
별도 pin bus로 만들지 않는다. slope 검출 polarity는 GPX Reg0의
`TRiseEn/TFallEn`과 `tdc_gpx_top`의 chip slope topology가 소유한다.

## 3. 경로 소유권

| 경로 | clock/gate | 용도 |
|---|---|---|
| `IBUFDS -> o_tdc_stop` | 비동기 direct path, Echo window gate 없음 | GPX 실제 측정 |
| `IBUFDS -> 2FF -> status/AXIS` | `axis_aclk`, logical Shot window | 진단 전용 |
| `m_stop_evt`, `m_fire_count` | TREADY 없는 observation stream | 검증/telemetry 전용 |

Echo edge count는 GPX 앞단 관측값이므로 IFIFO fill이나 drain 종료의 authority가
아니다. GPX `EF1/EF2`, `IrFlag`, 실제 read response만 acquisition 완료를
결정한다.

## 4. 발견된 통합 결함

초기 full integration TB는 실제 `axis_aclk=200 MHz`였지만
`laser_ctrl_top.g_AXIS_CLK_MHZ`를 전달하지 않아 기본값 150 MHz를 사용했다.
그 결과 CSR의 676개 5 ns tick이 3.38 us가 아니라 약 2.54 us로 실행됐다.

```text
shot_start       117.3625 us
stop_tdc         119.9025 us  (delta 2.540 us, 잘못된 150/200 비율)
GPX IrFlag       120.7025 us  (delta 3.340 us)
STAT5            0x0000F001   (sequence_error_mask=0xF)
```

상위에서 다음 generic mapping을 추가한 뒤 오류가 제거됐다.

```vhdl
generic map (
    g_AXIS_CLK_MHZ => C_AXIS_DOMAIN_CLK_MHZ
)
```

## 5. 종료 마진 계약

GPX MTimer 종료 뒤 `IrFlag`가 pin과 synchronizer를 통과할 시간을 확보해야 한다.

```text
laser max_roundtrip_5ns_ticks
  >= tdc_gpx max_range_5ns_ticks + close_margin_5ns_ticks
```

이번 통합 검증은 `close_margin_5ns_ticks=8`, 즉 40 ns를 사용했다. GPX가
`IrFlag`를 먼저 관측한 뒤 Laser `stop_tdc`가 도착했고 STAT5의 sequence mask는
0이었다. 40 ns는 parent 구현 전의 RTL 검증값이며, 실제 보드 값은 post-route
pin timing과 GPX datasheet를 이용해 다시 확정한다.

## 6. 검증 결과

| 검증 | 설정 | 결과 |
|---|---|---|
| Echo core regression | 2/4 chip, sim/physical, 1.5 ns pulse, 32ch CSR | PASS |
| Echo IP package DRC | revision 5, 단일 `o_tdc_stop` | PASS |
| Echo OOC synthesis | xc7z020, 4x8, 125 MHz, simulation path off | PASS |
| Echo GUI project | T1..T7 | PASS |
| Full integration | AXIS/TDC 200/200 MHz | PASS, STAT5=`0x00000001` |
| Full integration | AXIS/TDC 150/200 MHz | PASS, STAT5=`0x00000001` |

150/200 MHz run에서 동일한 668개 5 ns tick은 AXIS 501 clocks와 TDC 668
clocks로 각각 변환되어 같은 실제 측정 시간을 유지했다.

## 7. 생산 빌드 최적화

`g_ENABLE_SIM_PATH=false`일 때 simulation delay CTL1..16의 16개 32-bit
handshake CDC를 제거했다.

| 자원 | 이전 | 이후 | 변화 |
|---|---:|---:|---:|
| Slice LUT | 2,588 | 2,046 | -20.9% |
| Slice register | 3,427 | 1,715 | -50.0% |
| CARRY4 | 234 | 186 | -48 |

## 8. 판정

Echo RTL과 TDC 통합 simulation 계약은 close다. 실제 parent project가 아직
없으므로 물리 STOP pin-to-pin delay, channel skew, pulse width, I/O bank 조건은
implementation sign-off 항목으로 남긴다. 또한 `m_stop_evt`가 chip별 count가
아니라 동일 STOP index의 chip 합산값이라는 ABI는 software 확정 전에 한 번 더
검토해야 한다.
