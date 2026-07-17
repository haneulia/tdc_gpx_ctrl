# C09 atomic register-response CDC checkpoint

## 1. Problem closed by this checkpoint

The previous OOC baseline reported four `CDC-4` Critical paths. Each path was
one chip's 28-bit TDC register-read result, sampled continuously in the AXIS
domain while a separate `rvalid` pulse crossed through `xpm_cdc_pulse`.

That structure had two independent risks:

1. The 28 data bits were not transferred atomically with their qualifier.
2. In a multi-chip read, an early chip's one-cycle `rvalid` could disappear
   before `cmd_arb` emitted the transaction-wide all-done pulse used by
   `err_handler`.

## 2. RTL change

`tdc_gpx_reg_rsp_cdc` now owns one chip's complete response contract:

- Source payload: `{read_valid, read_data[27:0]}`.
- Source event: register-operation `done`.
- Transfer: 29-bit `xpm_cdc_handshake`, with four synchronizer stages in each
  direction.
- Destination outputs: one-cycle `done`, one-cycle `read_valid`, atomic data,
  and a held read-valid qualifier.
- Retirement: the held qualifier clears on transaction-wide all-done, stop,
  soft reset, or error soft-clear.
- Dispatch interlock: response-pending is folded into the chip-busy level so
  the same chip cannot receive another operation until the handshake has
  returned to idle.
- Simulation invariant: a second source completion while a response is
  pending is a failure.

`tdc_gpx_config_ctrl` instantiates this block once per chip. The former
continuous destination snapshot and independent done/rvalid pulse crossings
were removed.

## 3. Functional verification

Dedicated CDC test:

- Source clock: 200 MHz.
- Destination clock: 50 MHz, the slowest supported AXIS assumption.
- Read and write completions exercised at multiple phase offsets.
- The source data bus is deliberately changed immediately after capture.
- Checks: original data preserved, no missing/duplicate completion, destination
  pulse exactly one cycle, held qualifier persists until retire, and the
  source handshake returns to idle.

Full regression:

- Command: `scripts/run_c06_v002_regression.ps1 -Stamp 260717_reg_rsp_atomic_v4`
- Result: all scenarios PASS, 49 archived artifacts.
- Archive: `sim_results/vivado_xsim/sessions/260717_reg_rsp_atomic_v4_c06_v002_regression`
- Coverage retained: 32/64/128-bit output widths, AXIS 150 MHz / TDC 200 MHz,
  bounded backpressure, lane topology, range conversion, STAT sticky lifecycle,
  and output beat/TLAST integrity.

## 4. Exact-source OOC result

Session:

`signoff_results/sessions/260717_reg_rsp_atomic_final_w32_a150_t200_dedicated_2x2_synth`

Configuration: 32-bit output, AXIS 150 MHz, TDC 200 MHz, AXI-Lite 100 MHz,
`DEDICATED_2X2`, `ASYNC`, `xc7z020clg484-2`, Vivado 2025.2.1.

| Metric | CDC-source checkpoint | Atomic-response checkpoint | Judgment |
|---|---:|---:|---|
| `CDC-4` Critical | 4 | 0 | Closed |
| `CDC-10` Critical | 0 | 0 | Remains closed |
| `TIMING-9` unknown CDC | 1 | 0 | Closed |
| `TIMING-10` synchronizer property | 0 | 0 | Remains closed |
| `CDC-3` Info | 195 | 195 | Unchanged |
| `CDC-6` Warning | 24 | 24 | Unchanged |
| `CDC-15` Warning | 1,567 | 1,683 | +116, exactly 29 payload bits x 4 recognized XPM handshakes |
| Total LUT | 28,542 | 28,471 | -71 |
| Total FF | 43,573 | 43,821 | +248 |
| AXI WNS | +5.957 ns | +5.957 ns | Unchanged |
| AXIS WNS | +1.074 ns | +1.074 ns | Unchanged |
| TDC WNS | -1.765 ns | -1.765 ns | Open blocker unchanged |
| TDC TNS | -414.595 ns | -444.109 ns | Open; raw FIFO restructuring required |
| TDC failing endpoints | 690 | 762 | Open; worst paths remain raw FIFO |
| `TIMING-16` large setup | 196 | 196 | Unchanged, all led by raw FIFO path |
| Control sets | not checkpointed | 1,939 | Above guideline; optimization item |

The FF cost is the expected price of four atomic 29-bit handshakes. LUT use
decreased because the old snapshot/pulse structure and related control logic
were removed. No black boxes or unconstrained internal endpoints were present.

## 5. Checkpoint judgment

The register-response CDC is functionally and structurally closed. It is ready
to retain as an independent commit. Overall implementation sign-off remains
blocked by the TDC-domain raw FIFO path at 200 MHz. The next checkpoint must
replace the array-shift/full-eviction datapath with a shallower queue structure,
then repeat this exact OOC and functional regression flow.
