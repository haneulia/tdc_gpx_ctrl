# C08 GPX FIFO Ownership Contract Closure v020

> Date: 2026-07-22
> RTL checkpoints: `5cbb9c4`, `4c4fe96`
> HTML: `C08_HDL_HTML_Alignment_260722_GPX_FIFO_Ownership_Contract_Simulator_v018.html`

## 1. Closure decision

The previous `echo_receiver -> stop_cfg_decode -> expected_ififo -> chip_run` contract is removed. An Echo receiver observes LVDS edges before TDC-GPX acceptance and cannot know how many words the GPX actually stored in IFIFO1/2. Using that count as a read bound could leave valid GPX words unread or terminate a Shot on stale/mismatched information.

The closed architecture is:

| Signal or value | Meaning | Final role |
|---|---|---|
| Echo rising/falling count | Pre-GPX optical/front-end observation | Diagnostic only |
| `IrFlag` | GPX measurement/capture completion | Starts drain |
| `EF1`, `EF2` | GPX IFIFO empty state | Sole normal completion authority |
| `LF1`, `LF2` + Reg6 Fill | Guaranteed readable load region | Burst planning hint only |
| Accepted GPX bus response | One physically returned 28-bit word | Increments actual drain count |
| `n_drain_cap` | Per-IFIFO output bound in four-word steps | Truncation protection, never empty evidence |

If a cap is reached while EF remains low, RTL closes the bounded downstream output, records a fault, purges the physical tail until EF, and emits a faulted final control marker. This prevents the next Shot from inheriting stale IFIFO data.

## 2. RTL and interface changes

1. `tdc_gpx_top`, `tdc_gpx_config_ctrl`, `tdc_gpx_chip_ctrl`, and `tdc_gpx_chip_run` no longer expose or propagate expected IFIFO counts.
2. Top-level `stop_evt` and `fire_count` AXI-Stream inputs and their width generics are removed.
3. `tdc_gpx_stop_cfg_decode.vhd` and its TB are removed.
4. GPX Reg5/Reg7 image policy is retained in the smaller combinational `tdc_gpx_cfg_image_override.vhd` with a focused unit TB.
5. The 65-bit expected-count CDC handshake and its source/destination state are removed.
6. `STAT7[23:20]` now reads zero as a reserved field. The register address and surrounding bit positions are preserved.
7. The parent reference project was updated to the same 22-generic and no-Echo-stream top contract.

## 3. Verification evidence

| Verification | Result | Evidence checked |
|---|---|---|
| C06 regression, 30 xsim logs | PASS | No Failure/Fatal/ERROR marker; 32/64/128-bit, async clocks, backpressure, slope and status cases |
| `tb_tdc_gpx_chip_ctrl` | PASS | Cap/fault/purge behavior and full suite, 912 raw words |
| `tb_tdc_gpx_config_ctrl` | PASS | EF-authoritative config-control integration, data 16 / control 8 |
| `tb_tdc_gpx_top_int` | PASS | Physical FIFO loaded with 3 words; every Shot reads 3 and leaves 0 |
| Soft-reset recovery | PASS | Recovery run repeats EF-authoritative 3-word drain and output beat/TLAST checks |
| Force-reinit recovery | PASS | Recovery run repeats EF-authoritative 3-word drain and output beat/TLAST checks |
| Parent `VALIDATE` | PASS | 22/22 generic parity, Vivado BD validation, 85 contract checks |
| HTML JavaScript syntax | PASS | Standalone script parses without error |
| HTML browser self-test | PASS | `__C08_GPX_FIFO_OWNERSHIP_SELF_TEST__ = {pass:true, errors:[]}` |
| Desktop/mobile layout | PASS | 1440x1000 and 390x844, no page error or horizontal document overflow |

The top integration model intentionally loads more physical data than the former expected count path would have allowed. The observed `read=3, leftover=0` result proves that no hidden external-count termination remains.

## 4. HTML alignment

Stage 18 removes the deleted `g_STOP_WINDOW_MARGIN_TIME_NS` control and all related calculations. The D02A table presents the implemented order:

1. Echo receiver edge observation, diagnostic only.
2. GPX `IrFlag` capture boundary.
3. `EF1/EF2` normal completion authority.
4. `LF1/LF2` and Reg6 Fill burst planning.
5. Accepted bus responses as actual drained words.
6. `n_drain_cap` output bound.
7. Fault plus physical-tail purge when cap is reached before EF.

The sidebar and final verdict table expose the same fixed structural contract. These fields are not user-adjustable because they are architecture, not synthesis/runtime parameters. User-adjustable clock, topology, hit, geometry, VDMA, DDR, and Ethernet parameters remain simulated normally.

## 5. Remaining physical validation

RTL and simulation closure does not replace board validation. Before hardware sign-off:

- Confirm EF/LF/IrFlag polarity and compact chip-lane mapping against the schematic and XDC.
- Measure GPX bus timing and verify that Reg6 Fill agrees with the selected LF operating mode.
- Exercise no-hit, maximum-hit, cap-hit, GPX error, and prolonged downstream backpressure cases on hardware.
- Correlate Echo edge diagnostics with GPX output only as a post-run quality metric; never feed the difference back as an IFIFO read bound.

## 6. Supersession rule

Historical C02/C06/C07 documents that describe expected-count-bounded drain remain useful as design history, but they no longer define the current interface or completion policy. For current integration, use the VHDL at `4c4fe96`, this closure, the Stage 18 HTML, and `Doc/tdc_gpx_top_신호처리_상세_사용_설명서.md`.
