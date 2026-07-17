# TDC GPX Sign-off Results

Raw Vivado sessions are generated under `signoff_results/sessions/` and are
ignored by Git. Each committed sign-off document records the exact session
name, Git commit, configuration, report findings, and accepted waivers.

The wrapper requires a clean Git worktree and refuses to reuse a session name.
Use `-AllowDirty` only while developing the flow; such a run is not sign-off
evidence. The generated OOC XDC is loaded before synthesis so synthesis and
reporting use the same clock and clock-domain contract.

Run synthesis only:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_ooc_signoff.ps1 `
  -Width 32 -AxisMhz 150 -TdcMhz 200 -Label baseline
```

Run placement and routing as well:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_ooc_signoff.ps1 `
  -Width 128 -AxisMhz 200 -TdcMhz 200 -StreamMode SYNC `
  -Label timing_corner -Implement
```

The OOC constraints cover the three input clocks and internal reg-to-reg
timing. Board pin constraints, I/O delays, clock generation, and top-level
false-path policy remain the responsibility of the parent FPGA design. Vivado
is run with `synth_design -mode out_of_context`, so top-level I/O buffers and
pin-count DRCs are not part of these IP-level results.
