# V2 Checkpoint J10: PS H-Line and Ethernet Comparison

## Purpose

J10 closes the deterministic software boundary after J9. A portable C decoder
reads the exact DDR captures produced by the RTL/VDMA memory model, validates
the ordered Face Footer, transposes Shot-major PACKED17 data into H-Line-major
data, and emits Viewer packets for byte comparison against an executable HTML
Golden model.

The detailed Korean wire and ownership contract is
`V2_PS_HLINE_ETHERNET_ABI_KO.md`.

## Independent comparison chain

```text
HTML buildJ10PsGolden() -> checked PS/Ethernet Golden
                                     |
XSIM RTL -> J9 DDR capture -> portable C PS decoder
                                     |
                          packet-by-packet byte compare
```

The C decoder does not call the HTML model and the HTML model does not parse
the DDR image. This separation avoids comparing one implementation against
itself.

## Sign-off scenario and result

- one real Shot with 17-bit Hit `0x10001`;
- one explicit Hole Shot;
- one Rise Cell and one visible Return;
- valid ordered `GPF1 ... COMT` Face Footer;
- 150 and 200 MHz Processing tick frequencies;
- 32, 64 and 128-bit AXIS/VDMA source formats;
- first Viewer packet is a 1440-byte Face Header;
- second packet is a 38-byte H-Line packet containing two 3-byte samples.

| Processing clock | Input widths | Packet lengths | Result |
|---:|---|---|---|
| 150 MHz | 32/64/128 bits | 1440 + 38 B | byte-identical, PASS |
| 200 MHz | 32/64/128 bits | 1440 + 38 B | byte-identical, PASS |

The 150 and 200 MHz captures intentionally have different hashes because the
Viewer Face Header publishes the TDC measurement start reference time (T0)
tick frequency. Within one clock profile, all three input widths have the same
hash because transport geometry is not leaked into the Viewer ABI.

Archived sessions:

- J9 source captures: `260807_j10_full2_j9_v2_gpx_ddr_golden`
- full J9-to-J10 chain: `260807_j10_full2_v2_gpx_ps_hline`
- final decoder negative-test rerun on those captures:
  `260807_j10_final_v2_gpx_ps_hline`

Pass markers:

- `LIDAR_V2_GPX_DDR_GOLDEN_PASS`
- `LIDAR_V2_PS_HTML_GOLDEN_PASS`
- `LIDAR_V2_PS_DECODE_PASS`
- `LIDAR_V2_PS_HLINE_ETHERNET_COMPARE_PASS`
- `LIDAR_V2_GPX_PS_HLINE_SIGNOFF_PASS`

## Cache ownership evidence

Every C invocation verifies three negative/positive transitions:

1. decode while DMA owns the buffer is rejected;
2. CPU publication without a completed platform cache synchronization is
   rejected;
3. decode after CPU ownership succeeds, then decode after release to DMA is
   rejected again.

Before the successful decode it also flips the Footer commit marker and tries
an illegal transport width. Both must be rejected without emitting a packet;
the fixture is restored before the Golden comparison.

This proves the software API cannot silently bypass ownership. It does not
prove that a real Cortex-A9 cache line was invalidated; that remains J11 board
evidence. The decoder source also cross-compiles with `-Werror` for
`Cortex-A9`, ARM state, using the Vitis bare-metal compiler.

## Reproduction

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_ps_hline.ps1
```

By default the script first creates fresh J9 XSIM captures. The
`-SkipDdrRegression` switch is for focused software development only and is
not the full J9-to-J10 Sign-off path.

## Gate decision

J10 is complete for deterministic DDR parsing, transport-width normalization,
Viewer packet layout and post-synchronization output bytes. J11 must still
close the real AXI VDMA/HP-port path, OS-specific cache API, measured PS
processing time, Ethernet transmission time and sustained buffer ownership on
the `xc7z020clg484-2` board.
