# Local/Unified CSR Build-Mode Contract

## 1. Build-time selection

Every packaged signal-processing IP uses the same synthesis-time generic:

```vhdl
g_ENABLE_LOCAL_CSR : boolean := true
```

The default preserves the existing independently usable IP. This generic is
not writable at runtime and must not be copied into a CSR.

| Generic | Local AXI-Lite | Unified CSR link | Synthesized owner |
|---|---|---|---|
| `true` | Visible and required | Hidden | Existing local CSR |
| `false` | Hidden | Visible and required | Unified adapter |

Both port groups remain in the VHDL entity so one source top owns both build
profiles. Mutually exclusive `if generate` branches must remove the unused CSR
implementation; selecting a mode is not only an IP-XACT display operation.

## 2. IPI interface policy

Each IP exposes one custom bidirectional `unified_csr` interface when local CSR
is disabled. Its logical ports retain the canonical register-word names. This
keeps Block Design compact without broadcasting all 32 CTLs and 32 STATs to
every IP.

The interface and every mapped physical port use complementary IP-XACT
dependencies:

```text
local AXI bus/ports:
    MODELPARAM_VALUE.g_ENABLE_LOCAL_CSR = true

unified_csr bus/ports:
    MODELPARAM_VALUE.g_ENABLE_LOCAL_CSR = false
```

The mode-specific clock/reset pair follows the same dependency as its bus.
The local AXI-Lite and unified-CSR clocks do not publish a fixed `FREQ_HZ`;
their parent Block Design supplies the real control-clock metadata. Processing
interfaces may retain generic-dependent `FREQ_HZ` metadata, such as TDC-GPX
`g_AXIS_CLK_MHZ` and `g_TDC_CLK_MHZ`. The external unified configuration clock
is independent of each processing clock, and the adapter retains the verified
coherent CDC.

## 3. Per-IP payload

| IP | Unified CTL inputs | Unified STAT outputs | IRQ causes |
|---|---:|---:|---:|
| Motor Decoder | SYS 2 + Motor 6 | Motor 6 | 4 |
| Laser Controller | SYS 2 + Laser 7 | Laser 7 + transaction metadata | 3 |
| Echo Receiver | SYS 1 + Echo 2 | Echo 4 + transaction metadata | 2 active + 3 reserved |
| TDC-GPX | SYS 2 + TDC 9 | TDC 7 + transaction metadata | 7 |

System words are fanned out only to adapters that consume their fields. Static
capability values remain generics or status constants and do not cross a clock
domain on every cycle.

TDC-GPX local mode has two AXI-Lite slaves. Both Chip CSR and Pipeline CSR are
enabled or disabled together; mixed ownership is forbidden because it would
split one configuration epoch across two software contracts.

## 4. Runtime ownership

- Configuration words cross through coherent snapshots and apply only at the
  adapter's documented safe boundary.
- Reset, apply, table-write, and command requests use epochs/toggles rather
  than asynchronous one-cycle pulses.
- Real-time Motor-to-Laser eligibility, `fire_pulse`, `fire_done`,
  `start_tdc`, `shot_start`, `stop_tdc`, physical STOP, GPX pins, and VDMA AXIS
  data never traverse the CSR link.
- Local and unified modes must produce the same active internal configuration,
  data-path counts, fault lifecycle, and IRQ identities for equal scenarios.

## 5. Disabled-interface RTL behavior

Inputs in the inactive port group have declaration defaults so direct entity
instantiations remain source compatible. Inactive AXI outputs are tied to an
idle, non-responsive value and inactive unified outputs are zero. IP-XACT hides
those ports, so they are not legal software or parent-project interfaces in
that build profile.

## 6. Stage 5 gates

Each IP closes independently before the unified top is built:

1. legacy local-CSR unit and integration regressions pass with the default;
2. a focused unified-mode test proves equal configuration and status behavior;
3. elaboration or OOC synthesis proves the inactive CSR hierarchy is absent;
4. package metadata tests instantiate both generic values in Block Design;
5. local mode shows only AXI-Lite, unified mode shows only `unified_csr` and its
   clock/reset;
6. no black boxes, unconnected required interfaces, or fixed-frequency metadata
   remain.

Only a passed per-IP gate is committed. The final local-versus-unified
end-to-end comparison belongs to Stage 7.
