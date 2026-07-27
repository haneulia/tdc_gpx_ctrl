# C08 Motor, Range, and Return Feasibility Closure

> **대체 문서 안내:** 이 문서의 `Return 7 PASS`는 운용 RPM과 수평 각분해능에 따른 포인트 시간 예산을 RTL 계측으로 완전히 닫기 전 결과이다. 무조건적인 성능 Sign-off 근거로 사용하지 말고, 운용점과 잔여 마진을 명시한 `Operating_Point_Budget_Closure_v037`을 기준으로 판단한다.

## 1. 목적

이 문서는 모터(다면 미러) 회전 주기, 최대 도달거리, 실제 Multi-Echo(Return) 수가 하나의 Shot 시간 예산 안에서 함께 성립하는지 검증한 결과를 고정한다.

검증의 핵심은 다음 세 조건이다.

1. 다음 레이저 위치가 오기 전에 현재 Shot의 거리 캡처와 GPX 데이터 drain이 끝나야 한다.
2. 실제 Return 수는 RTL의 `max_hits` 저장 용량을 넘지 않아야 한다.
3. 32/64/128-bit 출력 폭을 바꾸어도 GPX에서 읽어야 하는 raw 28-bit word 수와 Hit[16:0] 값은 변하지 않아야 한다.

대응 HTML은 [C08-S23 시뮬레이터](C08_HDL_HTML_Alignment_260727_Motor_Range_Return_Feasibility_Simulator_v023.html)이다.

## 2. 시간 순서와 계산식

### 2.1 모터에서 Shot 간격까지

```text
revolution_period = 60 / RPM
mechanical_shot_interval = optical_shot_interval / 2
shot_period = revolution_period * mechanical_shot_interval / 360 deg
```

반사 미러의 광학 편향각은 기계 회전각의 2배이므로, 광학 Shot 간격을 시간으로 바꿀 때 기계각은 절반을 사용한다.

이번 기준값은 다음과 같다.

| 항목 | 값 |
|---|---:|
| 미러 1회전 주기 | 200 us |
| 모터 속도 | 300,000 RPM |
| 광학 Shot 간격 | 50 deg |
| 기계 Shot 간격 | 25 deg |
| 계산 Shot 주기 | 13.888889 us |
| 150 MHz 계획 Shot 간격 | 2,084 clocks |
| RTL 정상 구간 실측 최소 간격 | 2,083 clocks |

### 2.2 거리 캡처와 drain watchdog

```text
max_range_time = max_range_5ns_ticks * 5 ns
shot watchdog = max_range_time + g_DRAIN_MARGIN_TIME_NS
```

`max_range_5ns_ticks`는 Shot 시작부터 GPX 측정 창을 닫는 `IrFlag` 시점까지의 거리 캡처 시간이다. `g_DRAIN_MARGIN_TIME_NS`는 `IrFlag` 이후 GPX 내부 FIFO에 남은 raw word를 읽기 위한 별도 허용시간이다.

따라서 실제 타겟이 최대거리보다 가깝다는 이유로 남는 시간을 drain 예산으로 재사용하면 안 된다. RTL watchdog 계약과 HTML 모두 `IrFlag 이후 drain 허용시간 = g_DRAIN_MARGIN_TIME_NS`로 계산한다.

### 2.3 Return 수에서 GPX raw word 수까지

Dedicated 2-rise/2-fall 구성에서 APD 16채널의 양 slope를 모두 읽는다.

```text
raw_words_per_shot = APD_channels * active_slopes * actual_returns
                   = 16 * 2 * Return

limiting_chip_words = stops_per_chip * actual_returns
                    = 8 * Return
```

4개 GPX chip은 병렬로 동작하므로 drain 시간은 전체 32개 lane word의 직렬 합이 아니라, 가장 오래 걸리는 chip 한 개의 `8 * Return` word를 기준으로 한다.

200 MHz TDC clock과 현재 bus 설정에서 HTML 모델은 실측 RTL에 맞춰 다음과 같이 계산한다.

```text
one_word_service = max(g_BUS_READ_TIME_NS, bus minimum)
                 + (EF synchronizer guard + 3 control clocks) * TDC period
                 = 50 ns + 8 * 5 ns
                 = 90 ns

fixed_drain_control = (EF synchronizer guard + 4 control clocks) * TDC period
                    = 9 * 5 ns
                    = 45 ns

drain_time(Return N) = 45 ns + (8 * N * 90 ns)
```

| 실제 Return | Shot당 raw 28-bit word | 제한 chip word | 계산 drain | 1.28 us margin | 6.00 us margin |
|---:|---:|---:|---:|:---:|:---:|
| 1 | 32 | 8 | 0.765 us | PASS | PASS |
| 2 | 64 | 16 | 1.485 us | CHECK | PASS |
| 3 | 96 | 24 | 2.205 us | CHECK | PASS |
| 4 | 128 | 32 | 2.925 us | CHECK | PASS |
| 5 | 160 | 40 | 3.645 us | CHECK | PASS |
| 6 | 192 | 48 | 4.365 us | CHECK | PASS |
| 7 | 224 | 56 | 5.085 us | CHECK | PASS |

`max_hits`는 cell에 저장할 수 있는 합성 용량이고, `actual Return`은 해당 Shot에서 실제 발생한 echo 수이다. 두 값은 같은 의미가 아니다. 정상 조건은 `actual Return <= max_hits <= 7`이다.

## 3. 통합 검증 구성

| 항목 | 검증값 |
|---|---|
| AXIS / processing clock | 150 MHz |
| TDC clock | 200 MHz |
| APD 채널 | 16채널 전체 |
| GPX chip / STOP | 4 chip / chip당 8 STOP |
| slope topology | chip 0,1 rising / chip 2,3 falling |
| TDC-GPX data mode | I-Mode raw 28-bit, 하위 Hit[16:0] 거리정보 |
| 실제 Return sweep | 1..7 |
| `max_hits` | 실제 Return과 동일하게 1..7 sweep |
| 출력 폭 | 32, 64, 128 bit |
| 최대거리 | 1,000 m (`max_range_5ns_ticks=1335`) |
| echo 타겟 거리 | 950 m |
| 미러 회전 주기 / 광학 Shot 간격 | 200 us / 50 deg |
| VDMA backpressure | 없음 |

Echo stimulus는 APD 16채널의 물리 LVDS 입력 모두에 Return 펄스를 발생시킨다. 외부 GPX 모델은 STOP 시점으로부터 I-Mode 28-bit word를 만들고, `tdc_gpx_top`은 실제 GPX bus read 경로로 이를 수신한다.

## 4. 경계 재현

기존 `g_DRAIN_MARGIN_TIME_NS=1280` 조건에서는 다음 결과가 재현됐다.

| 조건 | 결과 | 근거 |
|---|---|---|
| Return 1, 32-bit | PASS | 8 word/chip이 1.28 us 안에 drain |
| Return 2, 32-bit | FAIL | `STAT5=0x00000F01`, chip 0..3 drain timeout |
| Return 7, 32-bit | FAIL | `STAT5[11:8]=0xF`, chip 0..3 drain timeout |

이 실패는 출력 폭이나 DDR 문제가 아니라 GPX raw bus drain 시간이 부족해서 발생한다. 따라서 기본 drain margin을 6,000 ns로 조정한 것은 Return 7 지원 계약을 만족시키기 위한 구조적 수정이다.

## 5. 6 us margin 회귀 결과

Return 1..7, 32-bit 전체와 Return 7의 64/128-bit를 모두 통과했다.

| Return | 출력 폭 | raw words/Shot | 결과 |
|---:|---:|---:|:---:|
| 1 | 32 | 32 | PASS |
| 2 | 32 | 64 | PASS |
| 3 | 32 | 96 | PASS |
| 4 | 32 | 128 | PASS |
| 5 | 32 | 160 | PASS |
| 6 | 32 | 192 | PASS |
| 7 | 32 | 224 | PASS |
| 7 | 64 | 224 | PASS |
| 7 | 128 | 224 | PASS |

Return 7, 32-bit 대표 결과는 다음과 같다.

- 9 Shot에서 raw I-Mode bus word 2,016개를 검사했다.
- rising Hit[16:0] 1,008개와 falling Hit[16:0] 1,008개가 모두 기대값과 일치했다.
- Shot당 raw word는 224개로 일치했다.
- `STAT5=0x00000001`, `STAT6=0xF0000000`, `STAT7=0x00000000`으로 drain timeout과 pipeline error가 없었다.
- `schedule_overrun=0`, `cfg_rejected=0`, `pipeline_abort=0`이었다.

32/64/128-bit에서 raw GPX word 수는 모두 224로 동일하다. 출력 폭은 이후 AXIS/VDMA serialization beat 수만 바꾸며, GPX chip에서 읽을 데이터 수나 drain 시간을 바꾸지 않는다.

## 6. C08-S23 HTML 반영

HTML은 다음 값을 위에서 아래 시간 순서로 표시한다.

1. fire 명령과 `fire_done` 지연
2. `max_range_5ns_ticks` 거리 캡처와 `IrFlag`
3. Return 수에 따른 GPX raw word drain
4. cell/AXIS format과 VDMA/DDR 전송
5. re-arm guard
6. 다음 미러 Shot까지의 최종 margin

또한 Return 1..7 표에서 각 Return의 raw word 수, 제한 chip word 수, drain 시간, scan/Shot 주기 적합성, `max_hits` 용량 적합성을 따로 보여준다.

`FIRE_DONE_TIMEOUT`은 허용 가능한 최대 응답시간 설정이고, 이번 통합 결과의 `fire_done_delay_clks=8`은 테스트 자극에서 실제 발생시킨 응답 지연이다. 둘을 같은 값으로 해석하지 않는다.

## 7. 자동 검사

```powershell
node system_integration/scripts/check_c08_v023_html.mjs
```

검사 대상은 Return 1/32-bit와 Return 7/32/64/128-bit RTL 결과이다. HTML 자체 self-test, RTL 계약 비교, raw 28-bit I-Mode bus 검사, Hit[16:0] 보존 검사가 모두 통과하며 마지막 marker는 다음과 같다.

```text
C08_V023_HTML_SELF_TEST_PASS
```

Vivado 2025.2로 `package_tdc_gpx_ip.tcl`을 다시 실행해 `ip_repo`도 갱신했다. 재생성된 `component.xml`의 user/model `g_DRAIN_MARGIN_TIME_NS`와 packaged `tdc_gpx_top.vhd` 기본값은 모두 6,000 ns이다. `check_tdc_gpx_ip_package.tcl`에는 이 값을 고정 검사하는 항목을 추가했으며 다음 marker를 확인했다.

```text
TDC_GPX_IP_PACKAGE_PASS
TDC_GPX_IP_PACKAGE_STATIC_PASS
```

로컬 `file:` URL은 Codex 브라우저 보안 정책에서 차단되어 이번 기록에서는 실제 브라우저 screenshot 자동 검사를 수행하지 못했다. 대신 HTML 계산 스크립트를 독립 실행하고, DOM ID/반응형 CSS 구조를 정적으로 검사했다. Vivado 사용자는 HTML을 일반 브라우저로 열어 최종 화면 배치만 확인하면 된다.

## 8. 판정과 남은 범위

이번 범위에서는 모터 회전 주기, 최대거리, Return 1..7, APD 16채널, 32/64/128-bit 출력 폭이 유기적으로 연결되어 검증됐다. 150 MHz AXIS / 200 MHz TDC 조건의 기능 sign-off 근거로 사용할 수 있다.

다음 항목은 별도 성능 sign-off 범위이다.

- VDMA 또는 DDR의 장기 backpressure가 있는 경우의 end-to-end Shot 중첩 한계
- 실제 보드에서 GPX bus timing과 `IrFlag` 지연의 post-implementation 측정
- 운용 RPM, 광학 Shot 간격, 최대거리 조합의 전체 제품 운용영역 sweep
