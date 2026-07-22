# C08 HDL/HTML Alignment: AXIS/TDC Multi-Clock Closure

- Date: 2026-07-22
- Stage: I2 AXIS/TDC functional CDC
- Product reference: AXIS 150 MHz, TDC 200 MHz
- Verdict: **I2 PASS, three-clock and board sign-off remain OPEN**

## 1. Change

`tb_tdc_gpx_full_int` now drives two physical clocks:

| Domain | Clock | Consumers |
|---|---:|---|
| AXIS processing | scenario `axis_clock_mhz` | Motor decoder, Laser controller, Echo receiver, GPX Cell/Face/VDMA path |
| TDC physical bus | scenario `tdc_clock_mhz` | GPX bus controller and four-chip behavioral GPX model |

`tdc_gpx_top.g_STREAM_CLK_MODE` is explicitly `ASYNC`. The existing
`config_ctrl` pulse/handshake CDC and TDC-to-AXIS raw-data asynchronous FIFO are
therefore exercised at different rates in the 150/200 MHz profile.

## 2. Range conversion contract

The CSR still stores one physical value:

```text
max_range_5ns_ticks = 668
physical window     = 668 x 5 ns = 3.340 us
```

Each domain derives its own clock count:

| Profile | AXIS local clocks | TDC local clocks |
|---|---:|---:|
| 200/200 MHz | 668 | 668 |
| 150/200 MHz | 501 | 668 |

Laser/Echo scheduling and AXIS watchdogs consume the AXIS value. GPX capture,
physical IFIFO behavior, and TDC watchdogs consume the TDC value. Reusing one
domain's count in the other would change the physical range window and is now
observable in the machine contract.

## 3. Results

| Check | 200/200 internal | 150/200 internal | 150/200 external A/B/Z |
|---|---:|---:|---:|
| Laser start / stop / result TLAST | 3 / 3 / 3 | 4 / 4 / 4 | 24 / 24 / 24 |
| Rise / fall line TLAST | 3 / 3 | 4 / 4 | 24 / 24 |
| HSIZE rise / fall | 128 / 128 B | 128 / 128 B | 128 / 128 B |
| VSIZE | 2 | 2 | 2 |
| `STAT5` | `00000001` | `00000001` | `00000001` |
| `STAT6` | `F0000000` | `F0000000` | `F0000000` |
| `STAT7` | `00000000` | `00000000` | `00000000` |
| Face-valid / cfg reject / pipeline abort | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

Evidence:

- `sim_results/vivado_xsim/sessions/260722_i2_internal_axis200_tdc200_v001_system_integration_smoke/`
- `sim_results/vivado_xsim/sessions/260722_i2_internal_axis150_tdc200_v001_system_integration_smoke/`
- `sim_results/vivado_xsim/sessions/260722_i2_external_axis150_tdc200_v001_system_integration_smoke/`

## 4. Result schema version 2

The compact HTML handoff now reports:

- `axis_clk_mhz`
- `tdc_clk_mhz`
- `max_range_5ns_ticks`
- `max_range_axis_clks`
- `max_range_tdc_clks`

The ambiguous schema-v1 `max_range_local_clks` field was removed, so
`rtl_result.json` and `rtl_contract.json` use top-level `schema_version: 2`.
The runner still accepts an old scenario without `tdc_clock_mhz` and treats it
as a same-clock profile.

## 5. Efficient verification boundary

The supported 50/100/125/150/200 MHz values do not require fifteen long system
runs. Existing range-conversion regression tests cover the domain-local ceiling
arithmetic. The system TB uses two representative integration gates:

1. 200/200 MHz catches same-clock functional regressions.
2. 150/200 MHz exercises the actual product AXIS/TDC rate relationship and CDC.

Additional system profiles should be added only when a real product clock pair,
throughput corner, or failure hypothesis requires them.

## 6. Open sign-off gates

1. Dynamically exercise AXI 100 MHz independently from AXIS 150 MHz. The
   parent reference currently proves this connection structurally, not in this
   multi-IP behavioral TB.
2. Repeat 150/200 MHz with bounded and shot-boundary VDMA backpressure.
3. Import schema-v2 `rtl_contract.json` into the C08 HTML and make any clock,
   range, geometry, shot-accounting, or status mismatch fail visibly.
4. Run parent synthesis/implementation and board timing after the integration
   contract stabilizes.
