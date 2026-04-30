# C02 Chip Acquisition - Timing Breakdown 재검사 v002

- 문서 버전: `v002`
- 작성/수정 시간: 2026-04-30 19:24:48 +09:00
- 이전 문서: `C02_Chip_Acquisition_260430162612_Timing_Breakdown_v001.md`
- 목적: zero-stop shot 보완 이후 C02 drain timing, latency, throughput, pipeline, II를 다시 검사한다.
- 절대 기준: `Doc/TDC-GPX-Datasheet.pdf`
- 기준 clock: `i_tdc_clk = 200 MHz`, `1 clk = 5 ns`

---

## 1. 결론

zero-stop 보완 이후에도 기존 정상 burst drain 타이밍은 유지된다.

| 경로 | 최신 측정 결과 | 판단 |
|---|---:|---|
| 정상 burst `[16]`, first raw data accepted | 40 clk = 200 ns | 기존 v001과 동일 |
| 정상 burst `[16]`, chip_run internal complete | 167 clk = 835 ns | 기존 v001과 동일 |
| 정상 burst `[16]`, output drain_done | 168 clk = 840 ns | 기존 v001과 동일 |
| 정상 burst `[16]`, raw output II | min 1 clk, max 14 clk | 기존 v001과 동일 |
| zero-stop 정상 `[2c]`, output drain_done | 22 clk = 110 ns | 새로 계측 |
| zero-stop EF 충돌 `[2d]`, output drain_done | 22 clk = 110 ns | 새로 계측 |

핵심 판단:

- `[16]`의 `first_data=40clk`는 GPX IC READ latency 자체가 아니다. TB가 raw output backpressure를 일부러 걸어 `s_raw_axis_tready=0` 구간을 만든 값이다.
- GPX IC READ timing의 절대 판단은 C01 bus timing, 즉 Datasheet p.7 Read Operations와 `bus_clk_div=1`, `bus_ticks=5` 조합으로 판단해야 한다.
- zero-stop은 IFIFO read가 없으므로 GPX data bus throughput을 소비하지 않는다. `22clk`는 IrFlag 인식, expected final latch guard, drain check, raw control beat 출력 비용이다.

---

## 2. Datasheet 기준

이 문서는 C02 timing breakdown이지만, GPX IC 물리 read timing은 C01 bus timing 계약을 그대로 따른다.

| 기준 | 내용 |
|---|---|
| `Doc/TDC-GPX-Datasheet.pdf` p.7 Read Operations | `RDN` low/high pulse, data valid/hold, EF set timing 기준 |
| `tPW-RL` | RDN low pulse width >= 6 ns |
| `tPW-RH` | RDN high pulse width >= 6 ns |
| `tV-DR` | RDN low 이후 data valid max 11.8 ns |
| `tS-EF` | last data read 이후 EF set max 11.8 ns |
| GPX readout 한계 | 40 MHz, 즉 read beat period 25 ns 이상 |

RTL 근거:

- `tdc_gpx_bus_phy.vhd:20` - 1 transaction은 clamped `i_bus_ticks` ticks
- `tdc_gpx_bus_phy.vhd:21` - 200 MHz에서 `div=1`은 `ticks>=5`, GPX 40 MHz readout limit 이하
- `tdc_gpx_bus_phy.vhd:42` - `tV-DR <= 11.8 ns`
- `tdc_gpx_bus_phy.vhd:46` - `ticks=5`, `div=1`은 15 ns capture margin, fastest 40 MHz
- `tdc_gpx_bus_phy.vhd:49` - burst `tPW-RH` 조건
- `tdc_gpx_cfg_pkg.vhd:271` - `tV-DR <= 11.8 ns`
- `tdc_gpx_cfg_pkg.vhd:289` - `div=1,ticks=4`는 50 MHz라 illegal
- `tdc_gpx_cfg_pkg.vhd:290` - `div=1,ticks=5`는 40 MHz OK
- `tdc_gpx_cfg_pkg.vhd:300` - `c_BUS_READ_PERIOD_MIN_CLKS=5`

---

## 3. 정상 Burst Drain Timing

### 3.1 측정 조건

`tb_tdc_gpx_chip_ctrl` `[16]` 조건:

| 항목 | 값 |
|---|---|
| clock | 200 MHz |
| drain mode | burst |
| IFIFO fill | IFIFO1=12 word, IFIFO2=8 word |
| 총 data word | 20 word |
| LF threshold | 4 word |
| burst chunk | 5 chunks = 4 word x 5 |
| bus timing | `bus_clk_div=1`, `bus_ticks=5` |
| raw output backpressure | TB가 `i >= 8 and i < 38` 구간에서 `tready=0` |

근거:

- `tb_tdc_gpx_chip_ctrl.vhd:1649` - `[16]` latency/II 측정 시나리오
- `tb_tdc_gpx_chip_ctrl.vhd:1653` - burst mode 설정
- `tb_tdc_gpx_chip_ctrl.vhd:1691` - bounded backpressure 시작 조건
- `tb_tdc_gpx_chip_ctrl.vhd:1692` - `s_raw_axis_tready <= '0'`
- `tb_tdc_gpx_chip_ctrl.vhd:1756` - first_data latency 출력
- `xsim_chip_ctrl.log:408` - `first_data=40clk`, `run_complete=167clk`, `output_done=168clk`, `II_min=1clk`, `II_max=14clk`

### 3.2 Timing Block

```text
T0  IrFlag pin high
    |
    |  IrFlag sync/edge detect
    v
    ST_DRAIN_LATCH
    |  c_EXP_LATCH_SETTLE_LAST=15 -> total 16 clk guard
    v
    ST_DRAIN_CHECK
    |  IFIFO1 burst read decision
    v
    C01 bus READ chunk
    |  chunk = 4 words, bus_ticks=5, div=1
    v
    raw FIFO enqueue
    |  TB backpressure window delays accepted output
    v
T1  first raw data accepted = 40 clk
    |
    |  remaining burst chunks + chunk boundary guard
    v
T2  chip_run internal complete = 167 clk
    |
    |  raw/control output boundary
    v
T3  output drain_done = 168 clk
```

### 3.3 Breakdown

| 구간 | clk | 의미 |
|---|---:|---|
| IrFlag 인식 + expected latch 준비 | 약 20 clk | IrFlag sync/edge detect와 `ST_DRAIN_LATCH` 16clk guard 포함 |
| 첫 bus read response까지 | 약 6 clk | C01 READ response path, `bus_ticks=5`와 deferred response 포함 |
| raw output backpressure 영향 | 약 13 clk | `[16]` TB가 의도적으로 `tready=0`을 만든 구간 |
| T0 -> first data accepted | 40 clk | 측정값 |
| 5개 burst chunk service | 약 105 clk | `4 word/chunk`, `bus_ticks=5` 기준 |
| chunk boundary guard/settle/check | 약 36 clk | `ST_DRAIN_FLUSH`, `ST_DRAIN_SETTLE`, `ST_DRAIN_CHECK` 반복 |
| final completion/control boundary | 약 6 clk | 마지막 completion과 output done |
| T0 -> internal complete | 167 clk | 측정값 |
| T0 -> output done | 168 clk | 측정값 |

주의: 위 분해는 cycle-accurate waveform annotation이 아니라 RTL state와 xsim 측정값을 맞춘 구조적 breakdown이다. Datasheet timing 판단에는 C01 `bus_phy`의 READ timing을 우선한다.

---

## 4. Zero-stop Timing

### 4.1 측정 조건

zero-stop은 이번 보완으로 추가된 `expected_final_valid` 계약을 사용한다.

| 시나리오 | 조건 | 결과 |
|---|---|---|
| `[2c]` known-zero clean | expected final valid=1, expected=0, EF empty | no IFIFO read, no fault, output_done=22clk |
| `[2d]` known-zero conflict | expected final valid=1, expected=0, EF not-empty | no IFIFO read, faulted control, output_done=22clk |

근거:

- `tb_tdc_gpx_chip_ctrl.vhd:831` - `[2c]` known-zero clean 시나리오
- `tb_tdc_gpx_chip_ctrl.vhd:858` - `[2c]` zero-stop latency 측정
- `tb_tdc_gpx_chip_ctrl.vhd:878` - `[2d]` known-zero conflict 시나리오
- `tb_tdc_gpx_chip_ctrl.vhd:909` - `[2d]` zero-stop conflict latency 측정
- `xsim_chip_ctrl.log:68` - `[2c] output_done=22clk`
- `xsim_chip_ctrl.log:75` - `[2d] output_done=22clk`

### 4.2 Timing Block

```text
T0  IrFlag pin high
    |
    |  IrFlag sync/edge detect
    v
    ST_DRAIN_LATCH
    |  expected_ififo1/2 + expected_final_valid snapshot
    |  c_EXP_LATCH_SETTLE_LAST=15 -> total 16 clk guard
    v
    ST_DRAIN_CHECK
    |  expected_final_valid=1 and expected=0
    |
    +-- EF empty     -> clean final control beat
    |
    +-- EF not-empty -> faulted final control beat
    v
T1  output drain_done = 22 clk
```

### 4.3 Breakdown

| 구간 | clk | 의미 |
|---|---:|---|
| IrFlag sync/edge detect | 약 3 clk | status pin synchronizer와 edge detect 가시성 |
| expected final/count settle | 16 clk | `c_EXP_LATCH_SETTLE_LAST=15`, total 16 TDC clocks |
| drain check + control beat boundary | 약 3 clk | `ST_DRAIN_CHECK` 판정, raw control beat, `drain_done` output |
| 합계 | 22 clk | xsim 측정값 |

zero-stop에서는 IFIFO data read가 없다. 따라서 C01 bus READ service time, burst chunk, EF settle 반복이 모두 빠진다.

---

## 5. Pipeline / Throughput / II

| 항목 | 정상 burst | zero-stop |
|---|---|---|
| Pipeline | IrFlag -> latch guard -> check -> bus read chunks -> raw FIFO -> output | IrFlag -> latch guard -> check -> control beat |
| Latency | `output_done=168clk` | `output_done=22clk` |
| Throughput | GPX read beat는 `div=1,ticks=5`로 40 MHz 이하, raw output은 FIFO 후단에서 `II_min=1clk` 가능 | data beat 없음, control beat 1개 |
| II | raw output 기준 `II_min=1clk`, `II_max=14clk` | data II 없음 |
| GPX bus 사용 | 20 data reads + control path | 0 data reads |

정상 burst의 `II_min=1clk`는 GPX bus가 1clk마다 읽는다는 뜻이 아니다. GPX bus read는 C01에서 25 ns 이상으로 제한되고, raw FIFO가 쌓아둔 data를 downstream이 backpressure 해제 후 연속 수락할 수 있기 때문에 raw output II가 1clk까지 내려간다.

---

## 6. 검사 판정

1. v001의 `[16]` 정상 burst timing 수치는 zero-stop 보완 이후에도 유지된다.
2. v001의 한글 인코딩이 깨져 있어, v002는 추적 가능한 한국어 문서로 새로 작성했다.
3. zero-stop은 별도 timing path로 분리해야 한다. 정상 burst `168clk`와 zero-stop `22clk`는 같은 drain path가 아니다.
4. Datasheet 기준으로 중요한 점은 zero-stop에서 empty FIFO read를 만들지 않는 것이다. `expected_final_valid=1`이고 `expected=0`이면 C02는 IFIFO read를 생성하지 않는다.
5. EF not-empty와 expected zero가 충돌하는 경우도 read하지 않고 faulted control beat로 닫는다. 이는 데이터시트의 empty FIFO read 금지와 운용 정합성 보존을 우선한 설계다.

---

## 7. Lineage

| 이전 항목 | v002 반영 |
|---|---|
| `C02_Chip_Acquisition_260430162612_Timing_Breakdown_v001.md` | 정상 burst timing 해석을 유지하되, 깨진 한글과 zero-stop 미반영을 보완 |
| `C02_Chip_Acquisition_260430190711_Zero_Stop_Shot_v001.md` | zero-stop path를 timing breakdown에 통합 |
| `fa234b1 fix: handle C02 zero stop shots` | `expected_final_valid` 기반 zero-stop RTL/TB 보완 결과 반영 |
| `xsim_chip_ctrl.log` 최신 실행 | `[2c]`, `[2d]`, `[16]` 측정값 반영 |
