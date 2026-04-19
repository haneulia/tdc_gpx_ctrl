-- =============================================================================
-- tdc_gpx_chip_ctrl.vhd
-- TDC-GPX Controller - Chip Control Coordinator
-- =============================================================================
--
-- Architecture:
--   Coordinator dispatching to 3 sub-FSMs:
--     tdc_gpx_chip_init  — powerup, cfg_write, master reset (10 states)
--     tdc_gpx_chip_run   — armed/capture/drain/ALU/overrun (13 states)
--     tdc_gpx_chip_reg   — individual register read/write (2 states)
--
--   Coordinator manages:
--     - ST_INIT / ST_IDLE phase tracking (plus PH_RESP_DRAIN between phases)
--     - Bus request mux (active sub-FSM → bus_phy)
--     - Bus response routing (bus_phy → active sub-FSM)
--     - Tick enable generation (bus_clk_div clock divider)
--     - Config snapshots (bus_clk_div, bus_ticks, drain_mode, n_drain_cap, max_range_clks)
--     - StopDis override (INTENTIONALLY LIVE for debug)
--     - Range counter (err_drain_timeout) and sequence error detection
--     - AXI-Stream raw word output (passthrough from chip_run)
--     - 2-deep raw hold/skid absorbing 1-cycle-late busy backpressure
--
-- Raw beat overflow diagnostics (Round 2 #5/#6, updated Round 6 B2):
--   s_err_raw_overflow_r (sticky) captures "some raw/control beat was
--   dropped for diagnostic reasons":
--     - chip_ctrl raw FIFO (3-slot array, Round 5 #3/#4) all slots full
--       → new beat silently lost
--     - PH_RESP_DRAIN hard cap (15 cycles) hit while bus still busy/pending
--       (s_err_drain_cap_r, OR'd into o_err_raw_overflow)
--   Round 5 #5 removed the overrun-drop path; the matching OR fold and
--   the chip_run s_run_overrun_drop signal were deleted in Round 6 B2.
--   Exposed via o_err_raw_overflow port.
--
-- PH_RESP_DRAIN hard-cap behavior (Round 3 #9):
--   Every PH_RUN completion enters PH_RESP_DRAIN to flush potential stale
--   bus responses. Normal exit: bus idle + ≥3 cycles. Hard cap 15 cycles
--   forces onward to PH_IDLE/PH_INIT but sets s_err_drain_cap_r if bus
--   was still active — SW can diagnose the forced drain.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_chip_ctrl is
    generic (
        g_BUS_DATA_WIDTH    : natural := c_TDC_BUS_WIDTH;   -- 28
        g_CHIP_ID           : natural := 0;                  -- 0..3
        g_POWERUP_CLKS      : positive := 48;    -- PuResN low duration (>200ns, ~240ns @ 200MHz)
        g_RECOVERY_CLKS     : positive := 8;      -- Reset/ALU recovery (~40ns @ 200MHz)
        g_ALU_PULSE_CLKS    : positive := 4        -- AluTrigger pulse width (~20ns @ 200MHz)
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- CSR configuration (latched at packet_start, stable during frame)
        i_cfg               : in  t_tdc_cfg;
        i_cfg_image         : in  t_cfg_image;

        -- CSR commands (active-high, 1-clk pulses from CSR)
        i_cmd_start         : in  std_logic;        -- IDLE -> ARMED
        i_cmd_stop          : in  std_logic;         -- ARMED/CAPTURE -> IDLE
        i_cmd_soft_reset    : in  std_logic;         -- any -> POWERUP (global)
        i_cmd_soft_reset_err: in  std_logic;         -- per-chip error recovery -> POWERUP
        -- Round 12 A1: SW-issued force re-init escape from PH_RESP_DRAIN
        -- permanent quarantine. Highest priority — bypasses PH_RESP_DRAIN
        -- and jumps directly to PH_INIT. SW is responsible for externally
        -- flushing the bus before issuing this pulse (e.g. via FPGA-side
        -- GPIO reset to the TDC chip); otherwise stale responses may
        -- pollute the init sequence. Use only when PH_RESP_DRAIN is stuck
        -- and s_err_drain_cap_r is observed. Setting the sticky below
        -- marks the fact that bus synchronization was bypassed.
        i_cmd_force_reinit  : in  std_logic := '0';
        i_cmd_cfg_write     : in  std_logic;         -- IDLE -> CFG_WRITE
        -- Round 7 B-5: SW-initiated sticky clear, forwarded to u_reg so its
        -- s_err_req_overflow_r follows the same clear semantic as status_agg.
        -- Default '0' keeps legacy instantiations unaffected.
        i_soft_clear        : in  std_logic := '0';

        -- Individual register access (from CSR, 1-clk pulses, IDLE only)
        i_cmd_reg_read      : in  std_logic;
        i_cmd_reg_write     : in  std_logic;
        i_cmd_reg_addr      : in  std_logic_vector(3 downto 0);
        i_cmd_reg_wdata     : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_cmd_reg_rdata     : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_cmd_reg_rvalid    : out std_logic;         -- 1-clk pulse with read data (read only)
        o_cmd_reg_done      : out std_logic;         -- 1-clk pulse: reg access done (read or write)

        -- Shot start (from laser_ctrl, 1-clk pulse)
        i_shot_start        : in  std_logic;

        -- Max range clock budget (from cfg, latched at shot_start)
        i_max_range_clks    : in  unsigned(15 downto 0);

        -- External stop signal (from laser_ctrl, already CDC'd to TDC domain).
        -- #13: wrapper config_ctrl.u_cdc_stop_tdc (xpm_cdc_pulse, DEST_SYNC_FF=4)
        -- converts the i_axis_aclk-domain source pulse into a clean 1-cycle
        -- pulse in i_clk (TDC) domain before it reaches this port.
        -- Used for ERROR DETECTION ONLY — NOT a drain trigger.
        -- Signals a sequence error during any active run phase (capture +
        -- drain + ALU), i.e. while armed='0' and busy='1', meaning the next
        -- shot deadline arrived before the current shot finished.
        i_stop_tdc          : in  std_logic;

        -- Per-IFIFO expected drain counts (from echo_receiver via tdc_gpx_top)
        -- CONTRACT:
        --   - IFIFO1 = stops 0~3 (rise+fall), IFIFO2 = stops 4~7 (rise+fall).
        --   - Values are CUMULATIVE totals from echo_receiver, overwritten on
        --     each tvalid. Reset by echo_receiver on new window start.
        --   - chip_ctrl captures these at IrFlag rising edge (ST_CAPTURE).
        --     By IrFlag time, MTimer has expired so all stop pulses are final.
        --   - Max per IFIFO = 4 stops x max_hits x 2 edges (rise+fall).
        --   - If both are 0 -> EF-based drain fallback (no count info).
        -- Per-chip expected IFIFO drain counts (from echo_receiver via top).
        -- Each = rise_count + fall_count for that chip's IFIFO.
        -- Used for burst sizing only (NOT drain completion — that uses EF).
        -- 0 = EF-based drain fallback (no burst optimization).
        i_expected_ififo1   : in  unsigned(7 downto 0);
        i_expected_ififo2   : in  unsigned(7 downto 0);

        -- bus_phy request interface
        o_bus_req_valid     : out std_logic;
        o_bus_req_rw        : out std_logic;          -- '0'=READ, '1'=WRITE
        o_bus_req_addr      : out std_logic_vector(3 downto 0);
        o_bus_req_wdata     : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_bus_oen_permanent : out std_logic;
        o_bus_req_burst     : out std_logic;          -- '1' = back-to-back burst read
        o_bus_ticks_snap    : out unsigned(2 downto 0);  -- bus_ticks snapshot for bus_phy

        -- bus_phy AXI-Stream slave (response): 32-bit tdata, 8-bit tuser
        --   tdata[27:0]  = read data (28-bit), tdata[31:28] = 0
        --   tuser[0]     = '0' READ, '1' WRITE
        --   tuser[4:1]   = register address
        i_s_axis_tvalid     : in  std_logic;
        i_s_axis_tdata      : in  std_logic_vector(31 downto 0);
        i_s_axis_tuser      : in  std_logic_vector(7 downto 0);
        o_s_axis_tready     : out std_logic;
        i_bus_busy          : in  std_logic;
        i_bus_rsp_pending   : in  std_logic;  -- bus_phy response pending or tvalid held

        -- bus_phy synchronized status inputs
        i_ef1_sync          : in  std_logic;          -- '1' = IFIFO1 empty
        i_ef2_sync          : in  std_logic;          -- '1' = IFIFO2 empty
        i_irflag_sync       : in  std_logic;          -- '1' = IrFlag active
        i_lf1_sync          : in  std_logic;          -- '1' = IFIFO1 loaded (reserved)
        i_lf2_sync          : in  std_logic;          -- '1' = IFIFO2 loaded (reserved)

        -- Tick enable output (to bus_phy)
        o_tick_en           : out std_logic;

        -- Physical pin control (directly to TDC-GPX chip)
        o_stopdis           : out std_logic;          -- '1' = stops disabled
        o_alutrigger        : out std_logic;
        o_puresn            : out std_logic;          -- '0' = chip in reset

        -- AXI-Stream master: raw word output (to decoder_i_mode)
        --   tdata[27:0]  = 28-bit raw IFIFO word (0 for drain_done beat)
        --   tdata[31:28] = 0 (zero-padded to 32-bit)
        --   tuser[0]     = ififo_id ('0'=IFIFO1, '1'=IFIFO2)
        --   tuser[6:1]   = 0 (reserved)
        --   tuser[7]     = drain_done flag ('1' = control-only beat, no data)
        o_m_raw_axis_tvalid : out std_logic;
        o_m_raw_axis_tdata  : out std_logic_vector(31 downto 0);
        o_m_raw_axis_tuser  : out std_logic_vector(7 downto 0);
        i_m_raw_axis_tready : in  std_logic;
        o_drain_done        : out std_logic;           -- 1-clk pulse when drain_done beat handshakes to downstream
        o_run_drain_complete : out std_logic;          -- 1-clk pulse when chip_run internally finishes drain (Round 5 #11)

        -- Status
        o_shot_seq          : out unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        o_busy              : out std_logic;

        -- Error flags (1-clk pulses)
        o_err_drain_timeout : out std_logic;    -- max_range_clks expired before drain_done
        o_err_sequence      : out std_logic;    -- IrFlag expected but not yet received
        o_err_rsp_mismatch  : out std_logic;    -- bus response tuser mismatch (sticky)
        o_err_raw_overflow  : out std_logic;    -- sticky: raw hold+skid both full (beat dropped)
        o_err_reg_overflow  : out std_logic;    -- sticky: chip_reg 3rd-pulse queue overflow (Round 5 #12)
        o_run_timeout       : out std_logic;    -- 1-clk pulse: chip_run abnormal drain exit
        -- Round 11 C: surface chip_run's timeout cause code for SW diagnosis.
        o_run_timeout_cause : out std_logic_vector(2 downto 0);
        -- Round 11 item 14: chip_init cfg_write coalesce sticky (per-chip).
        o_init_cfg_coalesced : out std_logic;
        -- Round 11 item 18 (C): cmd_arb contract violation sticky (per-chip).
        -- Fires on this chip's instance when PH_IDLE observes >1 command
        -- pulse in the same cycle. config_ctrl aggregates all c_N_CHIPS
        -- outputs into a mask so SW can see WHICH chip saw the collision.
        -- Investigate cmd_arb (source serialization failure), not the
        -- dropped command itself.
        o_err_cmd_collision  : out std_logic;
        -- Round 12 A1: force-reinit used sticky (SW bypassed PH_RESP_DRAIN).
        o_err_force_reinit   : out std_logic;
        -- Round 12 A2: control-beat drop sticky (distinct from data drop).
        o_err_raw_ctrl_drop  : out std_logic;
        -- Round 12 A4: chip_run drain count mismatch sticky.
        o_err_drain_mismatch : out std_logic;
        -- Round 12 A5: chip_reg concurrent R+W ambiguity sticky (per-chip).
        o_err_reg_rw_ambiguous : out std_logic;
        -- Round 12 B8: sticky for stopdis_override asserted during PH_RUN.
        o_err_stopdis_mid_shot : out std_logic
    );
end entity tdc_gpx_chip_ctrl;

architecture coordinator of tdc_gpx_chip_ctrl is

    -- =========================================================================
    -- Coordinator phase tracking
    -- =========================================================================
    type t_phase is (PH_INIT, PH_IDLE, PH_RUN, PH_REG, PH_CFG_WRITE, PH_RESP_DRAIN);
    signal s_phase_r      : t_phase := PH_INIT;
    signal s_drain_cnt_r    : unsigned(3 downto 0) := (others => '0');  -- stale response drain counter
    signal s_drain_to_init_r : std_logic := '0';  -- '1' = drain→PH_INIT (soft reset), '0' = drain→PH_IDLE (timeout)
    -- Round 9 #6 + Round 11 item 5: secondary quarantine counter.
    -- Runs while the phase is stuck in quarantine (cnt saturated at 15 AND
    -- bus still active). Round 9 #6 used its overflow (65K cycles) to
    -- force-exit to PH_INIT. Round 11 item 5 reverts that escalation: the
    -- forced PH_INIT transition risked routing stale responses into the
    -- init sub-FSM, so the counter now just saturates in place while the
    -- bus remains stuck. Recovery requires SW/supervisor action via full
    -- i_rst_n (soft_reset alone re-enters PH_RESP_DRAIN and stalls again).
    -- The counter is retained for potential future observability (time
    -- spent in quarantine) rather than functional escalation.
    signal s_drain_quarantine_cnt_r : unsigned(15 downto 0) := (others => '0');

    -- =========================================================================
    -- Sub-FSM signals: chip_init
    -- =========================================================================
    signal s_init_start      : std_logic := '0';
    signal s_init_cfg_write  : std_logic := '0';
    signal s_init_done       : std_logic;
    signal s_init_timeout    : std_logic;
    signal s_init_bus_valid  : std_logic;
    signal s_init_bus_rw     : std_logic;
    signal s_init_bus_addr   : std_logic_vector(3 downto 0);
    signal s_init_bus_wdata  : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    signal s_init_puresn     : std_logic;
    signal s_init_stopdis    : std_logic;
    signal s_init_busy       : std_logic;
    signal s_init_cfg_coalesced : std_logic;  -- Round 11 item 14: chip_init cfg_write coalesce sticky

    -- =========================================================================
    -- Sub-FSM signals: chip_run
    -- =========================================================================
    signal s_run_start       : std_logic := '0';
    signal s_run_done        : std_logic;
    signal s_run_armed       : std_logic;
    signal s_run_bus_valid   : std_logic;
    signal s_run_bus_rw      : std_logic;
    signal s_run_bus_addr    : std_logic_vector(3 downto 0);
    signal s_run_bus_wdata   : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    signal s_run_bus_oen     : std_logic;
    signal s_run_bus_burst   : std_logic;
    signal s_run_raw_word    : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    signal s_run_raw_valid   : std_logic;
    signal s_run_ififo_id    : std_logic;
    signal s_run_drain_done  : std_logic;
    signal s_run_drain_done_prev_r : std_logic := '0';  -- rising-edge detect for o_run_drain_complete
    signal s_run_ififo1_beat : std_logic;
    signal s_run_stopdis     : std_logic;
    signal s_run_alutrigger  : std_logic;
    signal s_run_busy         : std_logic;
    signal s_run_range_active : std_logic;
    signal s_run_shot_seq    : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
    signal s_run_timeout     : std_logic;
    -- Round 6 B2: s_run_overrun_drop removed. chip_run no longer drops beats
    -- on overrun (Round 5 #5), so the previously-OR-folded sticky was dead.

    -- =========================================================================
    -- Raw AXI-Stream N-deep holding FIFO (slot 0 = output, slot N-1 = tail)
    -- Slot 0 drives the output; new beats enqueue at the first empty slot.
    -- Provides (N-1) cycles of backpressure absorption so chip_run's burst
    -- response path is insensitive to short downstream stalls.
    --
    -- Depth sizing (Round 5 #3/#4):
    --   Backpressure to chip_run (s_raw_hold_busy) is registered → 1-cycle
    --   late. chip_run can still emit 1 more beat after busy deasserts while
    --   the registered value is stale. For raw beats alone, depth=2 would be
    --   enough. But control beats (drain_done, ififo1_done) share the same
    --   path and can be inserted by ST_DRAIN_CHECK between bus responses,
    --   creating a worst-case 3-beat burst. Depth=3 absorbs this cleanly so
    --   no control beat is ever dropped by the raw-path buffer.
    -- =========================================================================
    -- Round 9 #7: bumped 3 → 6 so a worst-case control + burst beat cluster
    -- (raw + raw + control + raw + raw + control) never drops a beat even
    -- when downstream backpressure holds tready low across the full cluster.
    -- The 3-slot version (Round 5 #3) covered the normal case but the
    -- control/data-sharing concern remained when the raw burst is long.
    constant c_RAW_FIFO_DEPTH : natural := 6;

    type t_raw_entry is record
        valid : std_logic;
        tdata : std_logic_vector(31 downto 0);
        tuser : std_logic_vector(7 downto 0);
        drain : std_logic;
    end record;
    constant c_RAW_ENTRY_EMPTY : t_raw_entry := (
        valid => '0',
        tdata => (others => '0'),
        tuser => (others => '0'),
        drain => '0'
    );
    type t_raw_fifo is array (0 to c_RAW_FIFO_DEPTH - 1) of t_raw_entry;
    signal s_raw_fifo_r    : t_raw_fifo := (others => c_RAW_ENTRY_EMPTY);
    signal s_raw_hold_busy : std_logic;  -- backpressure to chip_run (all slots full)

    -- =========================================================================
    -- Sub-FSM signals: chip_reg
    -- =========================================================================
    signal s_reg_start_rd    : std_logic := '0';
    signal s_reg_start_wr    : std_logic := '0';
    signal s_reg_done        : std_logic;
    signal s_reg_timeout     : std_logic;
    signal s_reg_rdata       : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    signal s_reg_rvalid      : std_logic;
    signal s_reg_busy        : std_logic;
    signal s_reg_bus_valid   : std_logic;
    signal s_reg_bus_rw      : std_logic;
    signal s_reg_bus_addr    : std_logic_vector(3 downto 0);
    signal s_reg_bus_wdata   : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);

    -- =========================================================================
    -- Bus response routing (to active sub-FSM)
    -- =========================================================================
    signal s_init_rsp_valid  : std_logic;
    signal s_run_rsp_valid   : std_logic;
    signal s_run_rsp_pending : std_logic;  -- "arrived at bus_phy/skid, not yet consumed"
    signal s_run_rsp_rdata   : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    signal s_reg_rsp_valid   : std_logic;
    signal s_reg_rsp_rdata   : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    signal s_bus_rsp_fire    : std_logic;  -- valid AND ready: true "accepted" pulse

    -- =========================================================================
    -- Config snapshots (latched at cmd_start, refreshed on cfg_write/reg)
    -- =========================================================================
    signal s_drain_mode_snap_r  : std_logic := '0';
    signal s_n_drain_cap_snap_r : unsigned(3 downto 0) := (others => '0');
    signal s_bus_clk_div_snap_r : unsigned(5 downto 0) := to_unsigned(2, 6);
    signal s_bus_ticks_snap_r   : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal s_max_range_snap_r   : unsigned(15 downto 0) := (others => '0');
    signal s_cfg_image_snap_r   : t_cfg_image := (others => (others => '0'));

    -- =========================================================================
    -- Tick enable generation
    -- =========================================================================
    signal s_div_cnt_r       : unsigned(7 downto 0) := (others => '0');
    signal s_tick_en_r       : std_logic := '0';

    -- =========================================================================
    -- Range counter + error detection
    -- =========================================================================
    signal s_range_cnt_r          : unsigned(15 downto 0) := (others => '0');
    signal s_range_active_r       : std_logic := '0';
    signal s_range_active_prev_r  : std_logic := '0';
    signal s_err_drain_timeout_r  : std_logic := '0';
    signal s_err_drain_to_fired_r : std_logic := '0';
    signal s_err_sequence_r       : std_logic := '0';
    signal s_err_rsp_mismatch_r   : std_logic := '0';  -- sticky: bus response tuser mismatch
    signal s_err_raw_overflow_r   : std_logic := '0';  -- sticky: raw hold+skid both full, beat dropped
    -- Round 12 A2: distinct control-beat drop sticky. Fires only when a
    -- control beat (drain_done / ififo1_done) was dropped — which with
    -- the 2-slot reserve below should only be possible if chip_run
    -- emits 5+ consecutive control beats without the downstream draining.
    -- If this ever fires, the separate-FIFO refactor (architectural,
    -- not done here) is required because the workload has changed.
    signal s_err_raw_ctrl_drop_r  : std_logic := '0';
    signal s_err_drain_mismatch   : std_logic;  -- Round 12 A4: from chip_run
    signal s_err_reg_rw_ambiguous : std_logic;  -- Round 12 A5: from chip_reg
    -- Round 12 B8: sticky for "stopdis_override asserted mid-shot".
    -- The override is intentionally live (debug escape hatch), but using
    -- it during PH_RUN almost certainly corrupts the in-flight shot.
    -- This sticky surfaces the event so SW can correlate dropped shots
    -- with override pulses. Intended use: leave override inactive in
    -- production; any set bit here implies debug intervention or bug.
    signal s_err_stopdis_mid_shot_r : std_logic := '0';
    signal s_err_drain_cap_r      : std_logic := '0';  -- sticky: PH_RESP_DRAIN hit hard cap while bus still active
    signal s_err_cmd_collision_r  : std_logic := '0';  -- Round 11 item 18 (C): per-chip PH_IDLE cmd collision sticky (cmd_arb contract violation)
    signal s_err_force_reinit_r   : std_logic := '0';  -- Round 12 A1: sticky — force-reinit was used (stale pollution risk acknowledged)
    -- #13: i_stop_tdc CDC moved to config_ctrl.u_cdc_stop_tdc (xpm_cdc_pulse);
    -- arrives here as a clean 1-cycle pulse in the TDC clock domain, so no
    -- internal 2-FF sync or edge-detect is needed.

    -- Effective reset for ALL sub-FSMs: hard reset OR soft_reset
    signal s_sub_rst_n            : std_logic;
    -- s_soft_reset_d1_r removed: PH_RESP_DRAIN handles delayed init start
    signal s_stopdis_latch_r      : std_logic := '1';  -- latched stopdis for PH_IDLE

begin

    -- Combine global soft_reset with per-chip error recovery reset
    s_sub_rst_n <= i_rst_n and (not i_cmd_soft_reset) and (not i_cmd_soft_reset_err);

    -- =========================================================================
    -- Sub-FSM instantiations
    -- =========================================================================
    u_init : entity work.tdc_gpx_chip_init
        generic map (
            g_BUS_DATA_WIDTH => g_BUS_DATA_WIDTH,
            g_POWERUP_CLKS   => g_POWERUP_CLKS,
            g_RECOVERY_CLKS  => g_RECOVERY_CLKS
        )
        port map (
            i_clk           => i_clk,
            i_rst_n         => s_sub_rst_n,
            i_start         => s_init_start,
            i_cfg_write_req => s_init_cfg_write,
            i_cfg_image     => s_cfg_image_snap_r,
            o_done          => s_init_done,
            o_timeout       => s_init_timeout,
            o_bus_req_valid => s_init_bus_valid,
            o_bus_req_rw    => s_init_bus_rw,
            o_bus_req_addr  => s_init_bus_addr,
            o_bus_req_wdata => s_init_bus_wdata,
            i_bus_rsp_valid => s_init_rsp_valid,
            o_puresn        => s_init_puresn,
            o_stopdis       => s_init_stopdis,
            o_busy          => s_init_busy,
            o_cfg_write_coalesced => s_init_cfg_coalesced
        );

    u_run : entity work.tdc_gpx_chip_run
        generic map (
            g_BUS_DATA_WIDTH => g_BUS_DATA_WIDTH,
            g_RECOVERY_CLKS  => g_RECOVERY_CLKS,
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => s_sub_rst_n,
            i_start             => s_run_start,
            i_cmd_stop          => i_cmd_stop,
            o_done              => s_run_done,
            o_range_active      => s_run_range_active,
            o_timeout           => s_run_timeout,
            o_timeout_cause     => o_run_timeout_cause,  -- Round 11 C: surface to SW
            o_err_drain_mismatch => s_err_drain_mismatch,  -- Round 12 A4
            o_armed             => s_run_armed,
            i_drain_mode        => s_drain_mode_snap_r,
            i_n_drain_cap       => s_n_drain_cap_snap_r,
            i_cfg_image         => s_cfg_image_snap_r,
            i_shot_start        => i_shot_start,
            i_expected_ififo1   => i_expected_ififo1,
            i_expected_ififo2   => i_expected_ififo2,
            o_bus_req_valid     => s_run_bus_valid,
            o_bus_req_rw        => s_run_bus_rw,
            o_bus_req_addr      => s_run_bus_addr,
            o_bus_req_wdata     => s_run_bus_wdata,
            o_bus_oen_permanent => s_run_bus_oen,
            o_bus_req_burst     => s_run_bus_burst,
            i_bus_rsp_valid     => s_run_rsp_valid,
            i_bus_rsp_pending   => s_run_rsp_pending,
            i_bus_rsp_rdata     => s_run_rsp_rdata,
            i_bus_busy          => i_bus_busy,
            i_ef1_sync          => i_ef1_sync,
            i_ef2_sync          => i_ef2_sync,
            i_irflag_sync       => i_irflag_sync,
            i_lf1_sync          => i_lf1_sync,
            i_lf2_sync          => i_lf2_sync,
            o_raw_word          => s_run_raw_word,
            o_raw_valid         => s_run_raw_valid,
            o_ififo_id          => s_run_ififo_id,
            o_drain_done        => s_run_drain_done,
            o_ififo1_done_beat  => s_run_ififo1_beat,
            i_raw_busy          => s_raw_hold_busy,
            o_stopdis           => s_run_stopdis,
            o_alutrigger        => s_run_alutrigger,
            o_busy              => s_run_busy,
            o_shot_seq          => s_run_shot_seq
        );

    u_reg : entity work.tdc_gpx_chip_reg
        generic map (g_BUS_DATA_WIDTH => g_BUS_DATA_WIDTH)
        port map (
            i_clk           => i_clk,
            i_rst_n         => s_sub_rst_n,
            i_start_read    => s_reg_start_rd,
            i_start_write   => s_reg_start_wr,
            i_addr          => i_cmd_reg_addr,
            i_wdata         => i_cmd_reg_wdata,
            o_rdata         => s_reg_rdata,
            o_rvalid        => s_reg_rvalid,
            o_done          => s_reg_done,
            o_timeout       => s_reg_timeout,
            o_busy          => s_reg_busy,
            o_bus_req_valid => s_reg_bus_valid,
            o_bus_req_rw    => s_reg_bus_rw,
            o_bus_req_addr  => s_reg_bus_addr,
            o_bus_req_wdata => s_reg_bus_wdata,
            i_bus_rsp_valid => s_reg_rsp_valid,
            i_bus_rsp_rdata => s_reg_rsp_rdata,
            o_err_req_overflow => o_err_reg_overflow,
            o_err_rw_ambiguous => s_err_reg_rw_ambiguous,  -- Round 12 A5
            i_soft_clear       => i_soft_clear
        );

    -- =========================================================================
    -- Bus request mux: active sub-FSM → bus_phy
    -- =========================================================================
    o_bus_req_valid  <= s_init_bus_valid when s_phase_r = PH_INIT or s_phase_r = PH_CFG_WRITE
                   else s_run_bus_valid  when s_phase_r = PH_RUN
                   else s_reg_bus_valid  when s_phase_r = PH_REG
                   else '0';
    o_bus_req_rw     <= s_init_bus_rw    when s_phase_r = PH_INIT or s_phase_r = PH_CFG_WRITE
                   else s_run_bus_rw     when s_phase_r = PH_RUN
                   else s_reg_bus_rw;
    o_bus_req_addr   <= s_init_bus_addr  when s_phase_r = PH_INIT or s_phase_r = PH_CFG_WRITE
                   else s_run_bus_addr   when s_phase_r = PH_RUN
                   else s_reg_bus_addr;
    o_bus_req_wdata  <= s_init_bus_wdata when s_phase_r = PH_INIT or s_phase_r = PH_CFG_WRITE
                   else s_run_bus_wdata  when s_phase_r = PH_RUN
                   else s_reg_bus_wdata;
    o_bus_oen_permanent <= s_run_bus_oen when s_phase_r = PH_RUN else '0';
    o_bus_req_burst     <= s_run_bus_burst when s_phase_r = PH_RUN else '0';

    -- Bus response tready: deassert during RUN drain when raw hold is full.
    -- PH_RESP_DRAIN always accepts (to drain stale responses).
    o_s_axis_tready  <= '0' when s_phase_r = PH_RUN and s_raw_hold_busy = '1'
                   else '1' when s_phase_r = PH_RESP_DRAIN
                   else '1' when s_init_busy = '1' or s_run_busy = '1' or s_reg_busy = '1'
                   else '0';

    -- Bus response FIRE pulse: valid AND ready. Sub-FSMs must use this,
    -- not raw tvalid, to avoid consuming the same beat multiple times
    -- when tready is deasserted (e.g., raw hold full during PH_RUN).
    s_bus_rsp_fire   <= i_s_axis_tvalid and o_s_axis_tready;

    -- Bus response routing (PH_RESP_DRAIN: all routing disabled)
    s_init_rsp_valid <= s_bus_rsp_fire when (s_phase_r = PH_INIT or s_phase_r = PH_CFG_WRITE) else '0';
    s_run_rsp_valid  <= s_bus_rsp_fire when s_phase_r = PH_RUN else '0';
    -- Pending routed to chip_run: "response arrived at bus_phy/skid but may
    -- not have fired this cycle." Combines the bus_phy pending flag with
    -- raw tvalid held high while tready is low (raw hold full during PH_RUN).
    -- chip_run uses this to:
    --   - freeze EF1/EF2/BURST wait watchdogs on downstream backpressure
    --   - decide DRAIN_FLUSH / OVERRUN_FLUSH completion (Round 5 #1/#2)
    s_run_rsp_pending <= (i_bus_rsp_pending or i_s_axis_tvalid)
                         when s_phase_r = PH_RUN else '0';
    s_run_rsp_rdata  <= i_s_axis_tdata(g_BUS_DATA_WIDTH - 1 downto 0);
    s_reg_rsp_valid  <= s_bus_rsp_fire when s_phase_r = PH_REG else '0';
    s_reg_rsp_rdata  <= i_s_axis_tdata(g_BUS_DATA_WIDTH - 1 downto 0);

    -- =========================================================================
    -- Tick enable generation (from bus_clk_div snapshot)
    -- =========================================================================
    p_tick_en : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_div_cnt_r <= (others => '0');
                s_tick_en_r <= '0';
            else
                if s_bus_clk_div_snap_r = 0
                   or s_div_cnt_r = s_bus_clk_div_snap_r - 1 then
                    s_div_cnt_r <= (others => '0');
                    s_tick_en_r <= '1';
                else
                    s_div_cnt_r <= s_div_cnt_r + 1;
                    s_tick_en_r <= '0';
                end if;
            end if;
        end if;
    end process p_tick_en;

    -- =========================================================================
    -- Coordinator FSM: dispatch to sub-FSMs
    -- =========================================================================
    p_coordinator : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_phase_r          <= PH_INIT;
                s_init_start       <= '1';
                s_init_cfg_write   <= '0';
                s_run_start        <= '0';
                s_reg_start_rd     <= '0';
                s_reg_start_wr     <= '0';
                s_drain_mode_snap_r  <= '0';
                s_n_drain_cap_snap_r <= (others => '0');
                s_bus_clk_div_snap_r <= to_unsigned(2, 6);
                s_bus_ticks_snap_r   <= to_unsigned(5, 3);
                s_err_drain_cap_r    <= '0';
                s_err_cmd_collision_r <= '0';
                s_err_force_reinit_r  <= '0';
                s_err_stopdis_mid_shot_r <= '0';
                s_drain_quarantine_cnt_r <= (others => '0');
                s_max_range_snap_r   <= (others => '0');
                s_cfg_image_snap_r   <= i_cfg_image;  -- use live image at power-up (not zeros)
            else
                -- Default: clear 1-clk dispatch pulses
                s_init_start     <= '0';
                s_init_cfg_write <= '0';
                s_run_start      <= '0';
                s_reg_start_rd   <= '0';
                s_reg_start_wr   <= '0';

                case s_phase_r is

                    when PH_INIT =>
                        s_stopdis_latch_r <= s_init_stopdis;
                        if s_init_done = '1' then
                            if s_init_timeout = '1' then
                                -- Timeout: drain stale responses before IDLE
                                s_phase_r         <= PH_RESP_DRAIN;
                                s_drain_cnt_r     <= (others => '0');
                                s_drain_to_init_r <= '0';
                            else
                                s_phase_r <= PH_IDLE;
                            end if;
                        end if;

                    when PH_IDLE =>
                        -- Priority: start > cfg_write > reg_read > reg_write
                        --
                        -- Round 11 item 18 (follow-up, option C): per-chip
                        -- runtime sticky. cmd_arb's mutual-exclusion
                        -- gating (source-side, AXI-Stream domain) means
                        -- simultaneous arrival here should be structurally
                        -- impossible in the nominal workflow. The sticky
                        -- below is therefore a SAFETY NET for:
                        --   - cmd_arb contract violation (its own bug)
                        --   - Unforeseen CDC skew in future clock-domain
                        --     restructuring
                        --   - SW that bypasses cmd_arb and drives pulses
                        --     directly (should never happen, but caught)
                        -- A fire of this sticky is a RED FLAG: investigate
                        -- cmd_arb, not the dropped command itself.
                        if (i_cmd_start and i_cmd_cfg_write) = '1'
                           or (i_cmd_start and (i_cmd_reg_read or i_cmd_reg_write)) = '1'
                           or (i_cmd_cfg_write and (i_cmd_reg_read or i_cmd_reg_write)) = '1' then
                            s_err_cmd_collision_r <= '1';
                            -- synthesis translate_off
                            assert false
                                report "chip_ctrl: multiple commands in PH_IDLE (lower priority dropped)"
                                severity warning;
                            -- synthesis translate_on
                        end if;
                        if i_cmd_start = '1' then
                            -- Snapshot ALL config for run (including cfg_image for chip_run)
                            s_cfg_image_snap_r   <= i_cfg_image;
                            s_drain_mode_snap_r  <= i_cfg.drain_mode;
                            s_n_drain_cap_snap_r <= i_cfg.n_drain_cap;
                            s_bus_clk_div_snap_r <= i_cfg.bus_clk_div;
                            s_bus_ticks_snap_r   <= i_cfg.bus_ticks;
                            s_max_range_snap_r   <= i_max_range_clks;
                            s_run_start          <= '1';
                            s_phase_r            <= PH_RUN;
                        elsif i_cmd_cfg_write = '1' then
                            s_cfg_image_snap_r   <= i_cfg_image;
                            s_bus_clk_div_snap_r <= i_cfg.bus_clk_div;
                            s_bus_ticks_snap_r   <= i_cfg.bus_ticks;
                            s_init_cfg_write     <= '1';
                            s_phase_r            <= PH_CFG_WRITE;
                        elsif i_cmd_reg_read = '1' then
                            s_bus_clk_div_snap_r <= i_cfg.bus_clk_div;
                            s_bus_ticks_snap_r   <= i_cfg.bus_ticks;
                            s_reg_start_rd       <= '1';
                            s_phase_r            <= PH_REG;
                        elsif i_cmd_reg_write = '1' then
                            s_bus_clk_div_snap_r <= i_cfg.bus_clk_div;
                            s_bus_ticks_snap_r   <= i_cfg.bus_ticks;
                            s_reg_start_wr       <= '1';
                            s_phase_r            <= PH_REG;
                        end if;

                    when PH_RUN =>
                        s_stopdis_latch_r <= s_run_stopdis;
                        -- Round 12 B8: detect mid-shot stopdis_override use.
                        -- The override bypasses FSM state to drive the pin
                        -- directly; firing during PH_RUN almost always means
                        -- the in-flight shot is being corrupted.
                        if i_cfg.stopdis_override(4) = '1' then
                            s_err_stopdis_mid_shot_r <= '1';
                        end if;
                        if s_run_done = '1' then
                            -- chip_run has multiple internal timeout paths;
                            -- any of them may leave stale responses in bus_phy.
                            -- Drain as precaution on every run completion.
                            s_phase_r         <= PH_RESP_DRAIN;
                            s_drain_cnt_r     <= (others => '0');
                            s_drain_to_init_r <= '0';
                        end if;

                    when PH_CFG_WRITE =>
                        -- Round 6 A4: always enter PH_RESP_DRAIN on completion
                        -- (matches PH_RUN's defensive policy). A late bus
                        -- response that fires after s_init_done could otherwise
                        -- land in the skid and be consumed as a bogus response
                        -- by the next PH_REG / PH_CFG_WRITE transaction.
                        if s_init_done = '1' then
                            s_phase_r         <= PH_RESP_DRAIN;
                            s_drain_cnt_r     <= (others => '0');
                            s_drain_to_init_r <= '0';
                        end if;

                    when PH_REG =>
                        -- Round 6 A4: same policy as PH_CFG_WRITE.
                        if s_reg_done = '1' then
                            s_phase_r         <= PH_RESP_DRAIN;
                            s_drain_cnt_r     <= (others => '0');
                            s_drain_to_init_r <= '0';
                        end if;

                    when PH_RESP_DRAIN =>
                        -- Drain stale bus responses after timeout or soft reset.
                        -- tready='1' (above), routing='0' (all discarded).
                        -- Counter saturates at 15 while the bus is still active
                        -- (quarantine — see below).
                        if s_drain_cnt_r /= to_unsigned(15, 4) then
                            s_drain_cnt_r <= s_drain_cnt_r + 1;
                        end if;
                        if i_bus_busy = '0' and i_bus_rsp_pending = '0'
                           and s_drain_cnt_r >= to_unsigned(3, 4) then
                            if s_drain_to_init_r = '1' then
                                s_phase_r    <= PH_INIT;
                                s_init_start <= '1';
                            else
                                s_phase_r <= PH_IDLE;
                            end if;
                            s_drain_to_init_r <= '0';  -- always clear on drain exit
                        elsif s_drain_cnt_r = to_unsigned(15, 4) then
                            -- Hard cap reached. Round 5 #9 + Round 9 #6 + Round 11 item 5:
                            --   Round 5 #9 made this a QUARANTINE — stay in
                            --   PH_RESP_DRAIN with cnt saturated at 15 and
                            --   routing disabled so any stale response is
                            --   absorbed. Round 9 #6 added a secondary
                            --   watchdog (s_drain_quarantine_cnt_r) that
                            --   forcibly re-entered PH_INIT at 65K cycles.
                            --   Round 11 item 5 REMOVES that forced re-init:
                            --   transitioning to PH_INIT while the bus is
                            --   still active risks stale responses arriving
                            --   into the init sub-FSM's routing window
                            --   ("phase pollution"). The safer policy is to
                            --   quarantine permanently and let SW notice via
                            --   s_err_drain_cap_r — recovery is a full
                            --   i_rst_n / power cycle, not an in-band re-init.
                            if i_bus_busy = '1' or i_bus_rsp_pending = '1' then
                                s_err_drain_cap_r <= '1';
                                if s_drain_quarantine_cnt_r /= x"FFFF" then
                                    s_drain_quarantine_cnt_r <=
                                        s_drain_quarantine_cnt_r + 1;
                                end if;
                                -- At x"FFFF": saturate in place. No escalation.
                            else
                                if s_drain_to_init_r = '1' then
                                    s_phase_r    <= PH_INIT;
                                    s_init_start <= '1';
                                else
                                    s_phase_r <= PH_IDLE;
                                end if;
                                s_drain_to_init_r <= '0';
                                s_drain_quarantine_cnt_r <= (others => '0');
                            end if;
                        else
                            s_drain_quarantine_cnt_r <= (others => '0');
                        end if;

                end case;

                -- Soft reset: global OR per-chip error recovery
                -- Drain stale responses first, then restart init
                -- (s_soft_reset_d1_r removed)
                if i_cmd_soft_reset = '1' or i_cmd_soft_reset_err = '1' then
                    s_cfg_image_snap_r   <= i_cfg_image;
                    s_phase_r            <= PH_RESP_DRAIN;
                    s_drain_cnt_r        <= (others => '0');
                    s_drain_to_init_r    <= '1';  -- soft reset → drain then init
                end if;

                -- Round 12 A1: force-reinit escape. Highest-priority override
                -- (last-assignment wins in this sequential process). Bypasses
                -- PH_RESP_DRAIN entirely — SW MUST have flushed the bus
                -- externally before pulsing this. Sets the force_reinit
                -- sticky so the event is SW-visible for post-mortem.
                if i_cmd_force_reinit = '1' then
                    s_cfg_image_snap_r       <= i_cfg_image;
                    s_phase_r                <= PH_INIT;
                    s_init_start             <= '1';
                    s_drain_cnt_r            <= (others => '0');
                    s_drain_to_init_r        <= '0';
                    s_drain_quarantine_cnt_r <= (others => '0');
                    s_err_force_reinit_r     <= '1';
                end if;
            end if;
        end if;
    end process p_coordinator;

    -- =========================================================================
    -- Range counter + error detection
    -- =========================================================================
    s_range_active_r <= s_run_range_active;  -- actual capture+drain window, not full run busy

    p_range_cnt : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_range_cnt_r          <= (others => '0');
                s_range_active_prev_r  <= '0';
                s_err_drain_timeout_r  <= '0';
                s_err_drain_to_fired_r <= '0';
                s_err_sequence_r       <= '0';
            else
                s_err_drain_timeout_r <= '0';
                s_err_sequence_r      <= '0';
                s_range_active_prev_r <= s_range_active_r;

                if s_range_active_r = '1' and s_range_active_prev_r = '0' then
                    s_range_cnt_r          <= (others => '0');
                    s_err_drain_to_fired_r <= '0';
                elsif s_range_active_r = '1' then
                    if s_max_range_snap_r /= 0 and s_range_cnt_r >= s_max_range_snap_r then
                        if s_err_drain_to_fired_r = '0' then
                            s_err_drain_timeout_r  <= '1';
                            s_err_drain_to_fired_r <= '1';
                        end if;
                    else
                        s_range_cnt_r <= s_range_cnt_r + 1;
                    end if;
                end if;

                -- Sequence error: stop_tdc pulse while run is active (armed already cleared).
                -- Covers capture + drain + ALU: any stop_tdc during active processing
                -- means the next shot deadline arrived before current shot finished.
                -- #13: i_stop_tdc is now a clean 1-cycle pulse from xpm_cdc_pulse in
                -- config_ctrl (no internal 2-FF / edge detect needed).
                if s_phase_r = PH_RUN and s_run_armed = '0' and s_run_busy = '1' then
                    if i_stop_tdc = '1' then
                        s_err_sequence_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_range_cnt;

    -- =========================================================================
    -- StopDis output: override OR FSM-controlled
    --
    -- Policy (#24, documented intent):
    --   INTENTIONALLY LIVE — debug/emergency override must take effect
    --   immediately, even mid-run. NOT snapshotted at cmd_start.
    --   SW implication: asserting i_cfg.stopdis_override(4) mid-shot forces
    --   the stops pin state on the next clock, independently of the internal
    --   FSM state. This can interrupt an in-flight shot at arbitrary
    --   positions; SW should treat any frame that straddles an override
    --   transition as potentially corrupt and correlate with status
    --   registers (chip_error_mask, shot_overrun, err_drain_timeout) to
    --   decide discard vs retain.
    --   No internal FSM recovery is triggered by the override — chip_run /
    --   chip_init continue on their own, so the pin state may briefly
    --   disagree with the FSM's notion of "stops enabled/disabled". This is
    --   intentional (debug takes priority); normal operation should keep
    --   stopdis_override inactive.
    -- =========================================================================
    o_stopdis <= i_cfg.stopdis_override(g_CHIP_ID)
                 when i_cfg.stopdis_override(4) = '1'
                 else s_init_stopdis when s_phase_r = PH_INIT or s_phase_r = PH_CFG_WRITE
                 else s_run_stopdis  when s_phase_r = PH_RUN
                 else s_stopdis_latch_r;  -- PH_IDLE/PH_REG: hold last value

    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    o_tick_en        <= s_tick_en_r;
    o_bus_ticks_snap <= s_bus_ticks_snap_r;
    o_puresn         <= s_init_puresn;
    o_alutrigger     <= s_run_alutrigger;

    -- AXI-Stream raw word: N-deep shift-register FIFO with tready handshake.
    -- Slot 0 drives the output; on accept, all valid entries shift down one
    -- slot. New beats from chip_run enqueue at the first empty slot.
    -- Same-cycle consume + enqueue is lossless because Step 1 (consume) feeds
    -- Step 2 (enqueue) through a variable snapshot.
    p_raw_fifo : process(i_clk)
        variable v_new  : t_raw_entry;
        variable v_fifo : t_raw_fifo;
        variable v_placed : boolean;
        variable v_free : natural range 0 to c_RAW_FIFO_DEPTH;
    begin
        if rising_edge(i_clk) then
            if s_sub_rst_n = '0' then
                s_raw_fifo_r <= (others => c_RAW_ENTRY_EMPTY);
                s_err_raw_overflow_r <= '0';
                s_err_raw_ctrl_drop_r <= '0';
            else
                -- Round 6 B2: previously OR-folded chip_run's
                -- s_err_overrun_drop here; that sticky has been removed
                -- because Round 5 #5 eliminated the overrun drop.

                -- Capture new beat from chip_run (at most one per cycle)
                v_new := c_RAW_ENTRY_EMPTY;
                if s_run_raw_valid = '1' then
                    v_new.valid := '1';
                    v_new.tdata(g_BUS_DATA_WIDTH - 1 downto 0) := s_run_raw_word;
                    v_new.tuser := "0000000" & s_run_ififo_id;
                elsif s_run_drain_done = '1' or s_run_ififo1_beat = '1' then
                    v_new.valid := '1';
                    v_new.tuser := '1' & "000000" & s_run_ififo_id;
                    v_new.drain := s_run_drain_done;
                end if;

                -- Snapshot current fifo into a variable so Step 1 feeds Step 2.
                v_fifo := s_raw_fifo_r;

                -- Step 1: consume slot 0 on downstream accept, shift all down.
                if v_fifo(0).valid = '1' and i_m_raw_axis_tready = '1' then
                    for i in 0 to c_RAW_FIFO_DEPTH - 2 loop
                        v_fifo(i) := v_fifo(i + 1);
                    end loop;
                    v_fifo(c_RAW_FIFO_DEPTH - 1) := c_RAW_ENTRY_EMPTY;
                end if;

                -- Step 2: enqueue new beat at the first empty slot (from head).
                -- Round 11 item 6: reserve one slot for control beats.
                -- tuser(7)='1' marks a control beat (drain_done / ififo1_beat);
                -- tuser(7)='0' is a data beat (raw hit word).
                --
                -- Old policy (pre-item-6): any beat (data OR control) could use
                -- any free slot, and the LAST free slot could be consumed by
                -- data. A control beat arriving right after could then find
                -- the FIFO full and get dropped — losing a completion /
                -- drain boundary marker, which is far more damaging than a
                -- single data beat loss.
                --
                -- New policy: data beats only enqueue if 2+ slots are free,
                -- so one slot always remains reservable for a control beat.
                -- Control beats enqueue into any free slot. The overflow
                -- sticky now fires for data beats at a lower threshold, but
                -- control beats are preserved as long as the FIFO is not
                -- completely saturated with earlier control beats.
                if v_new.valid = '1' then
                    v_placed := false;

                    v_free := 0;
                    for i in 0 to c_RAW_FIFO_DEPTH - 1 loop
                        if v_fifo(i).valid = '0' then
                            v_free := v_free + 1;
                        end if;
                    end loop;

                    -- Round 12 A2: reserve policy strengthened to 2 slots.
                    -- Was Round 11 item 6 (reserve=1). Data beats now require
                    -- 3+ free slots (data capacity 6−2 = 4), so at any time
                    -- at least 2 slots are guaranteed available for control.
                    -- This keeps control-beat drops structurally impossible
                    -- for the normal workload (chip_run emits at most 2
                    -- consecutive control beats per phase). Control only
                    -- drops when 5+ back-to-back control beats collide with
                    -- stuck downstream — a chip_run misbehavior, flagged by
                    -- the separate s_err_raw_ctrl_drop_r sticky.
                    if v_new.tuser(7) = '1' then
                        -- Control beat: enqueue if any slot is free.
                        if v_free >= 1 then
                            for i in 0 to c_RAW_FIFO_DEPTH - 1 loop
                                if not v_placed and v_fifo(i).valid = '0' then
                                    v_fifo(i) := v_new;
                                    v_placed := true;
                                end if;
                            end loop;
                        end if;
                    else
                        -- Data beat: enqueue only if 3+ slots are free, so
                        -- two remain reserved for future control beats.
                        if v_free >= 3 then
                            for i in 0 to c_RAW_FIFO_DEPTH - 1 loop
                                if not v_placed and v_fifo(i).valid = '0' then
                                    v_fifo(i) := v_new;
                                    v_placed := true;
                                end if;
                            end loop;
                        end if;
                    end if;

                    if not v_placed then
                        s_err_raw_overflow_r <= '1';
                        -- Round 12 A2: control vs data drop distinction.
                        if v_new.tuser(7) = '1' then
                            s_err_raw_ctrl_drop_r <= '1';
                        end if;
                        -- synthesis translate_off
                        assert false
                            report "chip_ctrl: raw beat DROPPED (kind=" &
                                   std_logic'image(v_new.tuser(7)) &
                                   ", drain=" &
                                   std_logic'image(v_new.drain) &
                                   ", free=" &
                                   integer'image(v_free) & ")"
                            severity error;
                        -- synthesis translate_on
                    end if;
                end if;

                s_raw_fifo_r <= v_fifo;
            end if;
        end if;
    end process p_raw_fifo;

    o_m_raw_axis_tvalid <= s_raw_fifo_r(0).valid;
    o_m_raw_axis_tdata  <= s_raw_fifo_r(0).tdata;
    o_m_raw_axis_tuser  <= s_raw_fifo_r(0).tuser;

    -- o_drain_done semantic (#27):
    --   Pulses when the final drain-done control beat HANDSHAKES to
    --   downstream (tvalid='1' AND tready='1'), NOT at the moment chip_run
    --   internally finishes draining. This means downstream backpressure
    --   can delay o_drain_done relative to the actual IFIFO drain completion
    --   inside chip_run.
    --   Upstream modules (face_seq, err_handler, status_agg) consuming this
    --   pulse must interpret it as "chip_run's drain_done beat was accepted
    --   by the raw-path consumer", not as "chip_run exited the drain state".
    o_drain_done        <= s_raw_fifo_r(0).drain and s_raw_fifo_r(0).valid
                           and i_m_raw_axis_tready;

    -- o_run_drain_complete (Round 5 #11):
    --   Pulses for 1 cycle on the rising edge of chip_run's internal drain
    --   completion — i.e. the cycle chip_run finishes draining its IFIFOs,
    --   independent of downstream backpressure on the raw AXI stream. Use
    --   this when the consumer needs to know "chip_run exited drain" rather
    --   than "the drain_done control beat was accepted downstream".
    p_run_drain_edge : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if s_sub_rst_n = '0' then
                s_run_drain_done_prev_r <= '0';
            else
                s_run_drain_done_prev_r <= s_run_drain_done;
            end if;
        end if;
    end process p_run_drain_edge;

    o_run_drain_complete <= s_run_drain_done and (not s_run_drain_done_prev_r);

    -- Backpressure: registered for 200MHz timing closure.
    -- 1-cycle late busy is safe because depth > 1 slots absorb the delay.
    p_raw_busy_reg : process(i_clk)
        variable v_all_full : std_logic;
    begin
        if rising_edge(i_clk) then
            if s_sub_rst_n = '0' then
                s_raw_hold_busy <= '0';
            else
                v_all_full := '1';
                for i in 0 to c_RAW_FIFO_DEPTH - 1 loop
                    v_all_full := v_all_full and s_raw_fifo_r(i).valid;
                end loop;
                s_raw_hold_busy <= v_all_full and (not i_m_raw_axis_tready);
            end if;
        end if;
    end process;

    o_shot_seq       <= s_run_shot_seq;
    -- Busy includes PH_RESP_DRAIN and PH_INIT to prevent premature dispatch
    o_busy           <= '0' when s_phase_r = PH_IDLE else '1';

    o_cmd_reg_rdata  <= s_reg_rdata;
    o_cmd_reg_rvalid <= s_reg_rvalid;
    o_cmd_reg_done   <= s_reg_done;

    o_err_drain_timeout <= s_err_drain_timeout_r;
    o_err_sequence      <= s_err_sequence_r;
    o_err_rsp_mismatch  <= s_err_rsp_mismatch_r;
    o_init_cfg_coalesced <= s_init_cfg_coalesced;
    o_err_cmd_collision  <= s_err_cmd_collision_r;
    o_err_force_reinit   <= s_err_force_reinit_r;
    o_err_raw_ctrl_drop  <= s_err_raw_ctrl_drop_r;
    o_err_drain_mismatch <= s_err_drain_mismatch;
    o_err_reg_rw_ambiguous <= s_err_reg_rw_ambiguous;
    o_err_stopdis_mid_shot <= s_err_stopdis_mid_shot_r;
    -- raw_overflow aggregates multiple "integrity compromised" events so SW
    -- only needs to observe one flag per chip. Individual cause codes can be
    -- split into dedicated ports later if needed:
    --   s_err_raw_overflow_r -> raw hold/skid full (beat dropped)
    --                         OR chip_run overrun override dropped response
    --   s_err_drain_cap_r    -> PH_RESP_DRAIN hard cap hit while bus busy
    o_err_raw_overflow  <= s_err_raw_overflow_r or s_err_drain_cap_r;
    o_run_timeout       <= s_run_timeout;

    -- =========================================================================
    -- Bus response tuser mismatch detector (sticky, all phases)
    -- Checks tuser[0]=RW and tuser[4:1]=addr against the active sub-FSM's
    -- last dispatched request.  Covers INIT, CFG_WRITE, RUN, and REG.
    -- =========================================================================
    p_rsp_check : process(i_clk)
        variable v_exp_rw   : std_logic;
        variable v_exp_addr : std_logic_vector(3 downto 0);
        variable v_check    : boolean;
    begin
        if rising_edge(i_clk) then
            if s_sub_rst_n = '0' then
                s_err_rsp_mismatch_r <= '0';
            elsif s_bus_rsp_fire = '1' then
                v_check := false;
                case s_phase_r is
                    when PH_INIT | PH_CFG_WRITE =>
                        v_exp_rw   := s_init_bus_rw;
                        v_exp_addr := s_init_bus_addr;
                        v_check    := true;
                    when PH_RUN =>
                        -- Check ALL run responses including burst (addr/rw stay constant)
                        v_exp_rw   := s_run_bus_rw;
                        v_exp_addr := s_run_bus_addr;
                        v_check    := true;
                    when PH_REG =>
                        v_exp_rw   := s_reg_bus_rw;
                        v_exp_addr := s_reg_bus_addr;
                        v_check    := true;
                    when others =>
                        null;
                end case;
                if v_check then
                    if i_s_axis_tuser(0) /= v_exp_rw
                       or i_s_axis_tuser(4 downto 1) /= v_exp_addr then
                        s_err_rsp_mismatch_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_rsp_check;

end architecture coordinator;
