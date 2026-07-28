# Unified CSR Migration and Verification Plan

## 1. Objective

Replace the duplicated per-IP AXI-Lite register banks with one stable software
contract while preserving each IP as an independently verifiable core.

The final software-visible bank is fixed at:

- 32 control registers at `0x000..0x07C`;
- 32 status registers at `0x080..0x0FC`;
- four interrupt registers at `0x100..0x10C`;
- 32 interrupt-source bits.

There are 26 active CTL slots and 30 active STAT slots in the initial map. Six
CTL and two STAT slots remain reserved. Including IRQ, 60 of the 68 addressable
registers have an initial owner.

The canonical numeric allocation is
`system_integration/rtl/lidar_unified_csr_pkg.vhd`. This document describes the
migration policy; it is not a second source of address constants.

The software transaction and bit-field interpretation is described in
`system_integration/UNIFIED_CSR_FIELD_MAP.md`. Field positions are still owned
by the package constants and are locked by the static contract test.

## 2. Architecture

Each IP is split conceptually into a processing core and a configuration
adapter. One packaged top may still expose both forms through a synthesis-time
generic and IP-XACT interface enablement.

```text
g_ENABLE_LOCAL_CSR = true
    AXI-Lite visible -> local CSR -> adapter -> core
    external config/status interface hidden

g_ENABLE_LOCAL_CSR = false
    AXI-Lite hidden
    external config/status interface visible -> adapter -> core
```

The VHDL entity keeps both port groups. IP-XACT hides the disabled bus interface
and every associated physical port in Block Design. RTL generate branches must
also remove the unused local CSR logic.

Hard real-time signals such as Motor-to-Laser eligibility, `fire_pulse`,
`fire_done`, `start_tdc`, `shot_start`, `stop_tdc`, and physical STOP remain
direct signals. They are not routed through the unified CSR.

Changing configuration is transactional:

1. software writes staging registers or an indexed data window;
2. software toggles an apply token;
3. the domain adapter transfers one coherent snapshot or command;
4. the destination accepts it only at its documented safe boundary;
5. status returns busy, accepted epoch, and rejection cause.

## 3. Indexed Configuration Windows

Two arrays must not consume one CSR word per element.

### Echo delay profile

The existing 32 channel x 16-bit simulation delays become two CTL slots:

- `ECHO_DELAY_CMD`: channel index, write token, and profile apply token;
- `ECHO_DELAY_DATA`: one 16-bit value in fixed 5 ns ticks.

The adapter writes an internal staging table. A profile apply swaps or copies
the complete table at an idle/shot boundary. Physical builds with
`g_ENABLE_SIM_PATH=false` remove the delay table and command path.

Echo status is reduced to:

- last completed-shot rising mask;
- last completed-shot falling mask;
- sticky/live/config status with a sequence value;
- selected delay readback.

The former rise-OR-fall mask is redundant. The transient 32-channel countdown
mask is not reliable software observability at scan rate and is replaced by a
selected-channel/aggregate live indication.

### GPX configuration image

The 16-word GPX register image becomes an indexed command/data window. The
512-bit active image still exists internally because chip initialization needs
an atomic image. Only a complete staged image may be committed.

## 4. Stage Gates

### Stage 0 - Baseline and ownership inventory

Entry: current canonical RTL and Git status recorded.

Checks:

- AXIS 150 MHz / TDC 200 MHz external Return-7 integration passes;
- internal encoder 150/200 MHz smoke passes;
- existing dirty files not owned by this migration remain untouched;
- every existing CTL, STAT, and IRQ source has an owner/disposition table.

Exit: two archived PASS runs and an inventory review. Commit only new baseline
scenario/contract files.

### Stage 1 - Address contract

Add the unified CSR package and a static testbench. The test must prove the
32/32/4 geometry, byte offsets, contiguous owner ranges, reserved ranges, and
60 active-word count.

Exit: `UNIFIED_CSR_CONTRACT_PASS`. Commit the package, test, runner, and plan.

### Stage 2 - Echo Receiver adapter

Implement the indexed delay staging table and stable last-shot status. Preserve
the legacy local CSR wrapper temporarily by translating its old direct delay
registers into the new adapter during the compatibility phase.

Required tests:

- all 32 channel writes and readbacks;
- out-of-range channel rejection;
- no partial profile visibility before apply;
- apply while a shot is active is deferred or rejected deterministically;
- physical build removes simulation delay logic;
- rising/falling masks and diagnostic sticky lifecycle;
- legacy Echo unit regression.

Exit: Echo uses 2 unified CTL and 4 unified STAT values. Commit in the Echo
repository after its unit gate passes.

### Stage 3 - Motor and Laser adapters

Move common reset/apply ownership to System control, combine selected-face
center/half readback, and return one shared configuration epoch. Do not move the
direct Motor-to-Laser timing path into CSR or AXIS buffering.

Exit: Motor uses 6 CTL/6 STAT, Laser uses 7 CTL/7 STAT, and the existing
150/200 MHz Motor/Laser integration tests pass. Commit each repository
independently.

### Stage 4 - TDC-GPX adapters

Replace the 16 direct image CTLs with the indexed image window. Keep the active
512-bit image atomic and preserve current chip/pipeline CDC safety. Map the four
chip read results and three live pipeline diagnostics into the unified STAT
range.

Exit: TDC uses 9 CTL/7 STAT, chip configuration/readback tests pass, and the
maintained 150/200 MHz TDC regression passes. Commit in this repository.

### Stage 5 - Optional local CSR and IP-XACT

Add `g_ENABLE_LOCAL_CSR` to each packaged top. Apply inverse IP-XACT enablement
to the local AXI bus and external config/status interface. Include clock/reset
ports and `ASSOCIATED_BUSIF` metadata in the condition.

Required package checks instantiate both parameter values in Block Design and
prove that only the selected interface is visible and required.

### Stage 6 - Unified CSR top

Instantiate one source-level `my_axil_csr32_top`, one adapter per processing
clock domain, and one interrupt aggregator. Reserved CTLs must have no effect;
reserved STATs return zero. Manual IRQ mode remains the production default.

Exit: direct register read/write, byte strobes, apply/ack, rejection, W1C, and
IRQ tests pass before connecting the signal-processing chain.

### Stage 7 - Integrated functional and HTML closure

Run only the maintained 150/200 MHz integration gates unless a changed clock
contract requires a wider sweep:

- internal encoder plus synthetic Echo;
- external A/B/Z plus physical 32-channel Return-7 Echo;
- local-CSR compatibility mode;
- unified-CSR mode with the same scenario values.

Compare both modes for shot counts, START/STOP timing, raw 28-bit I-Mode words,
17-bit Hit preservation, VDMA HSIZE/VSIZE, fault status, and point budget. Feed
the unified register snapshot into the C08 HTML contract panel.

Exit: RTL and HTML agree, no CHECK remains for a required field, and sign-off
evidence is archived.

## 5. Commit Policy

Commit only at a passed stage gate. Do not mix repositories in one commit.
Record the dependency commit/hash manifest in every integrated run because the
IP sources live in separate repositories. A failed gate is documented and
fixed before the next behavioral stage starts.

## 6. Current Baseline Note

The older `smoke_*_axis150_tdc200_v001` scenarios predate the operating-point
budget gate and currently request an angular grid that reports
`schedule_overrun`. They are not a valid post-change baseline. The maintained
external baseline is `return7_external_axis150_tdc200_v001`; the replacement
internal baseline is `smoke_internal_axis150_tdc200_v002`.
