# C08 HDL/HTML Alignment: Multi-IP Geometry Ownership

- Date: 2026-07-22
- Stage: I1 single-clock multi-IP functional baseline
- Target: `xc7z020clg484-2`, PS FCLK parent reference
- Integrated IP: virtual/physical encoder, Motor decoder, Laser controller, Echo receiver, `tdc_gpx_top`
- Verdict: **I1 PASS, multi-clock and board sign-off remain OPEN**

## 1. Purpose and sign-off boundary

This checkpoint proves that the canonical sibling IP sources can be compiled
and exercised as one functional chain and that the resulting GPX geometry and
status can be exported as a machine-readable contract for the C08 HTML
simulator. The integration TB intentionally uses one clock. Therefore this
result does not prove the real 100/150/200 MHz clock-domain crossings, pin
timing, or board-level behavior.

## 2. Corrected interface contracts

| Order | Contract | Applied rule |
|---:|---|---|
| 1 | Encoder timing | Internal and external encoder modes use one clock-derived 13/14-clock fractional profile at 200 MHz. |
| 2 | External encoder observation | Observe 220 us so the first Z-index revolution can establish the reference and a complete following interval can be checked. |
| 3 | `stop_tdc` | End of the current Laser measurement window, not a session stop or next-shot deadline. |
| 4 | GPX range watchdog | For nonzero CSR range, use `max_range_local_clks + g_DRAIN_MARGIN_CLKS` with saturation. Zero keeps the documented disabled/fallback behavior. |
| 5 | Face geometry ownership | Pipeline CSR `RANGE_COLS[31:16]` is the only owner of `cols_per_face`. |
| 6 | Laser result stream | Bits 15:0 are `step_idx + 1`; bits 31:16 are remaining steps. This stream is observed independently and no longer drives GPX geometry. |
| 7 | HTML handoff | `rtl_contract.json` contains the compact scenario and RTL-observed metrics; `STAT5/6/7` remain fixed-width hexadecimal strings. |

## 3. P0 geometry ownership defect and closure

Before this correction, `tdc_gpx_csr_pipeline` latched
`i_lsr_tdata[15:0]` as a `cols_per_face` override on each valid Laser result
beat. The current Laser controller never defined that field as columns. It
defined it as the live step index. Consequently, VDMA geometry could change as
the Laser result stream advanced.

The old external v4 run illustrates why activity-only PASS markers were not
sufficient:

| Run | Scenario columns | RTL HSIZE rise/fall | RTL VSIZE | Old verdict | Interpretation |
|---|---:|---:|---:|---|---|
| external v4, before fix | 2 | 128 / 128 B | **4** | PASS | False-negative contract check: data flowed, but the VDMA line count was wrong. |
| external v5, after fix | 2 | 128 / 128 B | **2** | PASS | CSR geometry and RTL output agree. |

The mismatched input ports and override latch were removed from
`tdc_gpx_csr_pipeline`, `tdc_gpx_top`, the parent wrapper, and associated test
benches. This is both a correctness fix and a complexity reduction: one
configuration owner replaces a priority mux and a second live update source.

## 4. Post-fix functional results

| Metric | Internal encoder v10 | External A/B/Z v5 | Required relation |
|---|---:|---:|---|
| Laser fire | 0 | 24 | Internal model need not expose a physical fire pulse; external path must be active. |
| Laser start / stop / result TLAST | 3 / 3 / 3 | 24 / 24 / 24 | All three counts must be equal. |
| Rise / fall VDMA line TLAST | 3 / 3 | 24 / 24 | Each active slope lane must emit one line per accepted shot. |
| HSIZE rise / fall | 128 / 128 B | 128 / 128 B | Both are sampled from RTL outputs. |
| VSIZE | 2 | 2 | Must equal CSR `columns_per_face`. |
| `STAT5` | `00000001` | `00000001` | Run enabled, no base fault bits. |
| `STAT6` | `F0000000` | `F0000000` | All four active chips completed drain. |
| `STAT7` | `00000000` | `00000000` | No extended fault or masked-slope drop. |
| Face-valid / rejected cfg / pipeline abort | 0 / 0 / 0 | 0 / 0 / 0 | No protocol loss or abort. |

Evidence:

- `sim_results/vivado_xsim/sessions/260722_i1_internal_v10_system_integration_smoke/`
- `sim_results/vivado_xsim/sessions/260722_i1_external_v5_system_integration_smoke/`
- `sim_results/vivado_xsim/sessions/260722_i1_geometry_owner_c06_v002_regression/`

The post-fix C06 suite completed with 107 archived artifacts and all existing
configuration, range conversion, CDC, output width, slope, status lifecycle,
and backpressure scenarios passing.

## 5. Parent reference validation

The updated interface was rebuilt in the repository's PS FCLK parent reference:

| Check | Result |
|---|---|
| Top/parent generic parity | PASS, 22 generics |
| Parent contract | PASS, 85 checks |
| Part and clocks | `xc7z020clg484-2`, AXI 100 MHz, AXIS 150 MHz, TDC 200 MHz |
| Data paths | Rise/fall AXIS to two VDMA S2MM channels validated |

Evidence:
`parent_ref/results/sessions/260722_i1_geometry_owner_ps_fclk_parent_ref/`

This run is `VALIDATE`, not a new post-route timing result. Vivado also reported
a local Tcl Store catalog warning and recommended replacing legacy `xlconcat`
instances with inline HDL concat. Neither invalidated the block design, but the
concat replacement is a reasonable parent-project cleanup before the next full
implementation sign-off.

## 6. What the HTML must compare next

The C08 HTML importer should read `rtl_contract.json` and visibly compare, in
calculation order:

1. Scenario clock, output width, active chip mask, and slope mask.
2. 5 ns range ticks and RTL-local range clocks.
3. Stops per chip, maximum hits, columns per face, and faces per frame.
4. RTL-observed HSIZE rise/fall and VSIZE against the HTML geometry model.
5. Laser start/stop/result-TLAST equality and rise/fall line-TLAST counts.
6. `STAT5`, `STAT6`, and `STAT7` exact values.

An imported result must not show overall PASS when any geometry field differs,
even if all activity and status checks pass. The external v4 result is the
regression fixture for this rule.

## 7. Next gates

1. Add separate AXIS and TDC clocks to the integration TB and run at least
   200/200 MHz baseline and 150/200 MHz product-reference profiles.
2. Extend the C08 HTML with local `rtl_contract.json` import and field-by-field
   mismatch display.
3. Separate quick smoke and full regression entry points so routine interface
   edits do not require the full C06 run every time.
4. Establish independent Git baselines for sibling IP directories before any
   production RTL edits there; source hashes alone are evidence, not history.
5. After multi-clock closure, rerun parent synthesis/implementation and timing
   checks before using the term full sign-off.
