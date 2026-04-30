# C02 Chip Acquisition - T0/T1 분리 Timing 검토 v001

- 작성/수정 시간: 2026-04-30 21:23:06 +09:00
- 대상 Cluster: C02_Chip_Acquisition
- 목적: 기존 `T0 -> T1 = 40 clk` 측정값이 내부 drain 지연인지, downstream backpressure에 의해 밀린 accepted latency인지 분리한다.
- 절대 기준: `Doc/TDC-GPX-Datasheet.pdf`
- 보조 기준: `tb_tdc_gpx_chip_ctrl.vhd`, `xsim_chip_ctrl.log`
- PPT 요약: `C02_Chip_Acquisition_260430212306_T0_T1_Split_Timing_v001.pptx`

## 1. 결론

기존 `T0 -> T1 = 40 clk`는 C02 내부 첫 데이터 준비 시간이 아니다.

새 계측 결과, 첫 raw data는 `T0 + 16 clk`에 이미 valid 상태가 된다. 기존 40clk는 테스트벤치가 의도적으로 `m_raw_axis_tready=0`을 걸어 downstream handshake가 지연된 결과다.

| 조건 | first_valid | first_accept | 의미 |
|---|---:|---:|---|
| `[16a]` raw backpressure 없음 | 16 clk | 16 clk | 내부 raw data가 준비되는 즉시 downstream accepted |
| `[16b]` bounded raw backpressure 있음 | 16 clk | 40 clk | 내부 raw data는 16clk에 준비되나, ready release가 40clk라 accepted가 지연 |

따라서 앞으로 문서에서 `T1`은 단일 의미로 쓰지 않고 아래처럼 분리한다.

| 새 이름 | 정의 |
|---|---|
| T1a | first raw data valid 또는 output boundary pending 시점 |
| T1b | first raw data가 downstream에서 실제 handshake accepted 된 시점 |

## 2. 코드 보완 내용

`tb_tdc_gpx_chip_ctrl.vhd`의 `[16]` 검증을 두 구간으로 나누었다.

| 구간 | 목적 | 코드 근거 |
|---|---|---|
| `[16a]` | raw backpressure 없이 first-data latency 측정 | `tb_tdc_gpx_chip_ctrl.vhd:1720..1818` |
| `[16b]` | 기존 bounded backpressure 조건 유지, `first_valid`와 `first_accept` 분리 | `tb_tdc_gpx_chip_ctrl.vhd:1823..2064` |
| 변수 추가 | `v_first_valid_cycle`, `v_ready_low_start_cycle`, `v_ready_release_cycle` | `tb_tdc_gpx_chip_ctrl.vhd:621..623` |

## 3. 검증 결과

실행 명령:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xvhdl.bat' --2008 tb_tdc_gpx_chip_ctrl.vhd --nolog
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat' --debug typical tb_tdc_gpx_chip_ctrl -s tb_chip_ctrl_snap
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat' tb_chip_ctrl_snap --runall --log xsim_chip_ctrl.log
```

결과:

| 항목 | 값 | 근거 |
|---|---:|---|
| `[16a]` first_valid | 16 clk | `xsim_chip_ctrl.log:921` |
| `[16a]` first_accept | 16 clk | `xsim_chip_ctrl.log:921` |
| `[16a]` valid_to_accept | 0 clk | `xsim_chip_ctrl.log:921` |
| `[16a]` run_complete | 427 clk | `xsim_chip_ctrl.log:921` |
| `[16a]` output_done | 428 clk | `xsim_chip_ctrl.log:921` |
| `[16b]` first_valid | 16 clk | `xsim_chip_ctrl.log:988` |
| `[16b]` first_accept | 40 clk | `xsim_chip_ctrl.log:987..988` |
| `[16b]` valid_to_accept | 24 clk | `xsim_chip_ctrl.log:988` |
| `[16b]` ready_low_start | 10 clk | `xsim_chip_ctrl.log:988` |
| `[16b]` ready_release | 40 clk | `xsim_chip_ctrl.log:988` |
| `[16b]` release_to_accept | 0 clk | `xsim_chip_ctrl.log:988` |
| 전체 기능 검증 | PASS | `xsim_chip_ctrl.log:1006` |

## 4. Timing Block

```text
T0  IrFlag pin assert
 |
 | 16 clk
 v
T1a first raw data valid / output boundary pending
 |
 | [16a] ready=1이면 0 clk
 | [16b] ready=0이면 24 clk 대기
 v
T1b first raw data accepted

[16b] bounded backpressure 조건:

T0 + 10 clk  raw tready low start
T0 + 16 clk  first raw data valid
T0 + 40 clk  raw tready release
T0 + 40 clk  first raw data accepted
```

## 5. Latency / Throughput / Pipeline / II 해석

| 항목 | 판단 |
|---|---|
| First-data latency | 내부/output-boundary 기준은 16clk가 맞다. 기존 40clk는 downstream accepted 기준이다. |
| Total output latency | `[16a]`, `[16b]` 모두 `output_done=428clk`다. 초기 backpressure는 raw FIFO/hold 경계에서 흡수되어 전체 drain 완료 시점은 변하지 않았다. |
| Throughput | 전체 관측 기준은 56 words / 428 clk로 유지된다. 초기 accepted latency 차이는 이후 `II_min=1clk` 방출 구간에서 회복된다. |
| Pipeline | `IrFlag sync -> chip_run drain decision -> bus_phy READ -> chip_ctrl response/raw boundary -> raw AXI output` 순서이며, 이번 분리는 마지막 raw AXI output boundary의 valid/ready를 구분한 것이다. |
| II | `[16b]` 기준 `II_min=1clk`, `II_max=15clk`가 유지된다. 이는 GPX bus READ II가 아니라 raw output stream accepted interval이다. |

## 6. Datasheet 기준 해석

이번 `T1a/T1b` 분리는 GPX IC 물리 READ timing을 바꾸지 않는다. Datasheet 기준으로 관리해야 하는 항목은 C01에서 정의한 bus physical READ cadence이며, C02의 raw output accepted latency는 downstream AXI ready 상태의 영향을 받는다.

따라서 C02 문서에서는 아래 구분을 유지한다.

| 구분 | 기준 |
|---|---|
| GPX IC 물리 READ timing | `Doc/TDC-GPX-Datasheet.pdf`, C01 bus timing 계약 |
| C02 내부 첫 데이터 준비 | T1a = first raw valid |
| downstream 관측 첫 데이터 | T1b = first raw accepted |

## 7. 후속 반영

- `C02_Chip_Acquisition_260430210752_Timing_Metric_Detail_v001.md`의 기존 `T1 = first raw data handshake` 정의는 T1b로 재해석해야 한다.
- 다음 Timing Metric 문서 또는 PPT에서는 `T1a/T1b`를 반드시 분리 표기한다.
