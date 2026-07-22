# C08 HDL/HTML Alignment: CTL21 Geometry Contract

- Date: 2026-07-22
- Stage: I3 runtime geometry truth source
- Product reference: AXIS 150 MHz, TDC 200 MHz, output 32 bit
- Verdict: **PASS, HTML contract importer remains OPEN**

## 1. Problem found

The integration TB calculated `max_hits=3` from the 500 m profile and printed
that value in `RTL_RESULT`, but did not write Chip CSR `CTL21[18:16]`.
Consequently, RTL used reset value zero, whose defined meaning is the build
maximum alias of seven hits. The previous HSIZE of 128 B was internally valid
for seven hits, but the machine contract incorrectly labelled it as three-hit
geometry.

This is a false-metric defect in the verification harness, not a datapath
arithmetic defect. It is important because an HTML model could match the
printed `max_hits` while disagreeing with the actual RTL stream.

## 2. Correction

Before `CFG_WRITE` and `START`, the TB now writes:

```text
CTL21 = 0x000B0000
        bit 19    falling_enable = 1
        bits18:16 max_hits_cfg   = 3
        bits15:0  max_scan ticks = 0
```

`falling_enable` is derived from whether any active behavioral chip emits a
falling slope. If all active chips are rising, it becomes zero and RTL routes
all active chips to the rising lane.

`max_scan_5ns_ticks` intentionally remains zero in this smoke. That field is a
per-chip Cell arrival timeout, not merely optical range. A nonzero value must
include GPX drain and AXIS service margin; directly copying
`max_range_5ns_ticks` would make the timeout too short under valid latency.

## 3. Geometry derivation

For each slope lane in the dedicated 2+2 profile:

| Step | Calculation | Result |
|---|---|---:|
| Active chips per slope | 2 | 2 chips |
| Stops per chip | CSR `stops_per_chip` | 2 |
| Cell slots per line | 2 chips x 2 stops | 4 cells |
| Canonical words per Cell | `ceil(3 hits / 2) + 1 metadata` | 3 words |
| Canonical Cell bytes | 3 x 4 B | 12 B |
| Cell payload | 4 x 12 B | 48 B |
| 16 B line alignment pad | `align16(48) - 48` | 0 B |
| Repeated line prefix | fixed RTL contract | 48 B |
| HSIZE | 48 B + 48 B | **96 B** |
| 32-bit AXIS beats per line | 96 B / 4 B | **24 beats** |

The TB now asserts both exported HSIZE values and checks every accepted TLAST
line individually against 24 beats. This avoids a total-count false pass where
one long line could be cancelled by one short line.

## 4. Face-boundary finding

The first 200/200 MHz run after adding a total-beat assertion reported 84
beats across three completed lines, although every completed line was exactly
24 beats. The extra 12 beats were the 48 B prefix of an open next line: the
fixed 100 us observation ended after three shots while `columns_per_face=2`.

That is an incomplete-Face test-harness condition, not a 28-beat RTL line. The
normal integration smoke now waits for the next complete Face boundary before
gating new Motor-to-Laser requests. Mid-Face cancellation is kept out of this
pass criterion and should be verified separately with an explicit abort
policy.

## 5. Results

| Check | 200/200 internal | 150/200 internal | 150/200 external A/B/Z |
|---|---:|---:|---:|
| Laser start / stop / result TLAST | 4 / 4 / 4 | 4 / 4 / 4 | 24 / 24 / 24 |
| Rise / fall line TLAST | 4 / 4 | 4 / 4 | 24 / 24 |
| Rise / fall total beats | 96 / 96 | 96 / 96 | 576 / 576 |
| Every completed line | 24 / 24 beats | 24 / 24 beats | 24 / 24 beats |
| HSIZE rise / fall | 96 / 96 B | 96 / 96 B | 96 / 96 B |
| VSIZE | 2 | 2 | 2 |
| `STAT5 / STAT6 / STAT7` | `00000001 / F0000000 / 00000000` | same | same |
| Face-valid / cfg reject / abort | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |

Evidence:

- `sim_results/vivado_xsim/sessions/260722_i3_faceclose_internal_axis200_tdc200_v001_system_integration_smoke/`
- `sim_results/vivado_xsim/sessions/260722_i3_final_internal_axis150_tdc200_v001_system_integration_smoke/`
- `sim_results/vivado_xsim/sessions/260722_i3_final_external_axis150_tdc200_v001_system_integration_smoke/`

## 6. Remaining work

1. Import schema-v2 `rtl_contract.json` into the C08 HTML.
2. Make clock, range, CTL21 geometry, shot accounting, and status differences
   visible as individual PASS/CHECK rows.
3. Add a separate mid-Face abort scenario before defining partial-frame
   recovery as sign-off behavior.
4. Keep independent AXI 100 MHz dynamic simulation, parent implementation,
   timing closure, and board validation as later gates.
