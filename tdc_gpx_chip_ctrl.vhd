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
--     - Config snapshots (bus_clk_div, bus_ticks, drain_mode, n_drain_cap,
--       max_range_tdc_clks)
--     - StopDis override (INTENTIONALLY LIVE for debug)
--     - Range counter (err_drain_timeout) and sequence error detection
--     - AXI-Stream raw word output (passthrough from chip_run)
--     - 8-entry circular raw FIFO with explicit data/control credits
--
-- Raw beat overflow diagnostics (Round 2 #5/#6, updated Round 6 B2):
--   s_err_raw_overflow_r (sticky) captures "some raw/control beat was
--   dropped for diagnostic reasons":
--     - chip_ctrl raw FIFO exhausted its protocol credits
--     - PH_RESP_DRAIN protocol grace cap hit while bus still busy/pending
--       (s_err_drain_cap_r, OR'd into o_err_raw_overflow)
--   Round 5 #5 removed the overrun-drop path; the matching OR fold and
--   the chip_run s_run_overrun_drop signal were deleted in Round 6 B2.
--   Exposed via o_err_raw_overflow port.
--
-- PH_RESP_DRAIN grace-cap behavior (Round 3 #9, clock-profile closure):
--   Every PH_RUN completion enters PH_RESP_DRAIN to flush potential stale
--   bus responses. Normal exit requires bus idle plus a settling guard.
--   The 1023-cycle cap covers every legal runtime div/ticks transaction;
--   activity beyond that point is diagnosed as a stuck physical bus.
--
-- PH_RESP_DRAIN auto-recover (Round 13 follow-up, audit 3번):
--   After s_err_bus_fatal_r is latched (quarantine counter reached 65K
--   with bus still active), a secondary observer tracks how long the bus
--   has been stably idle. If it stays idle for g_BUS_IDLE_STABLE_CLKS
--   cycles, the phase auto-
--   transitions to PH_INIT. This reclaims liveness when the bus recovers
--   on its own, without relying on SW force_reinit. The latched bus-fatal
--   status remains the software-visible evidence of the recovery event.
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
        g_POWERUP_CLKS      : positive := c_DEFAULT_POWERUP_CLKS;
        g_RECOVERY_CLKS     : positive := c_DEFAULT_RECOVERY_CLKS;
        g_ALU_PULSE_CLKS    : positive := c_DEFAULT_ALU_PULSE_CLKS;
        -- Round 13 follow-up P4 (audit 3번): bus-idle stability window for
        -- auto-recover from PH_RESP_DRAIN quarantine. After s_err_bus_fatal_r
        -- latches, the bus must stay idle (busy='0' AND rsp_pending='0') for
        -- this many consecutive cycles before auto-transitioning to PH_INIT.
        -- Production top derives this count from a physical-time generic and
        -- g_TDC_CLK_MHZ. Standalone default is 4096 clocks at 200 MHz.
        g_BUS_IDLE_STABLE_CLKS : positive := c_DEFAULT_BUS_IDLE_STABLE_CLKS;
        g_DRAIN_MARGIN_CLKS    : positive := c_DEFAULT_DRAIN_MARGIN_CLKS;
        g_EF_SYNC_GUARD_CLKS   : positive := c_DEFAULT_EF_SYNC_GUARD_CLKS
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
        -- and the bus has been externally flushed.
        i_cmd_force_reinit  : in  std_logic := '0';
        i_cmd_cfg_write     : in  std_logic;         -- IDLE -> CFG_WRITE
        -- Round 7 B-5: SW-initiated sticky clear. In this legacy controller
        -- it is currently forwarded only to u_reg, so it clears register
        -- request overflow but not response-mismatch/raw-drop/init-coalesce/
        -- coordinator quarantine stickies. The v2 CSR clear table records
        -- this Sign-off gap explicitly; do not describe this port as a full
        -- controller-status clear until every owning process consumes it.
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

        -- Measurement-window budget converted from CSR 5 ns ticks to
        -- TDC-local clocks. The controller adds g_DRAIN_MARGIN_CLKS for the
        -- post-IrFlag physical FIFO drain.
        i_max_range_tdc_clks : in unsigned(15 downto 0);

        -- External stop signal (from laser_ctrl, already CDC'd to TDC domain).
        -- #13: wrapper config_ctrl.u_cdc_stop_tdc (xpm_cdc_pulse, DEST_SYNC_FF=4)
        -- converts the i_axis_aclk-domain source pulse into a clean 1-cycle
        -- pulse in i_clk (TDC) domain before it reaches this port.
        -- Used for error detection only; it is not a drain trigger.
        -- stop_tdc is the current measurement-window end. It is a sequence
        -- error only if GPX capture is still waiting for IrFlag; arrival
        -- during post-IrFlag drain/ALU is normal.
        i_stop_tdc          : in  std_logic;

        -- bus_phy request interface
        o_bus_req_valid     : out std_logic;
        o_bus_req_rw        : out std_logic;          -- '0'=READ, '1'=WRITE
        o_bus_req_addr      : out std_logic_vector(3 downto 0);
        o_bus_req_wdata     : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_bus_oen_permanent : out std_logic;
        o_bus_req_burst     : out std_logic;          -- '1' = back-to-back burst read
        o_bus_clk_div_snap  : out unsigned(5 downto 0);  -- bus_clk_div snapshot for bus_phy
        o_bus_ticks_snap    : out unsigned(2 downto 0);  -- bus_ticks snapshot for bus_phy

        -- bus_phy AXI-Stream slave (response): 32-bit tdata, 8-bit tuser
        --   tdata[27:0]  = read data (28-bit), tdata[31:28] = 0
        --   tuser[0]     = '0' READ, '1' WRITE
        --   tuser[4:1]   = register address
        i_s_axis_tvalid     : in  std_logic;
        i_s_axis_tdata      : in  t_bus_rsp_tdata;
        i_s_axis_tuser      : in  t_bus_rsp_tuser;
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
        --   tuser[4:1]   = 0 (reserved)
        --   tuser[5]     = faulted flag — only meaningful on drain_done beat
        --                  ('1' = chip_run's final drain fallback detected a
        --                   drain-count mismatch; downstream should treat the
        --                   associated shot as degraded). Always '0' on data
        --                   beats and on the intermediate IFIFO1-done beat.
        --   tuser[6]     = 0 (reserved)
        --   tuser[7]     = drain_done flag ('1' = control-only beat, no data)
        o_m_raw_axis_tvalid : out std_logic;
        o_m_raw_axis_tdata  : out t_raw_axis_tdata;
        o_m_raw_axis_tuser  : out t_raw_axis_tuser;
        i_m_raw_axis_tready : in  std_logic;
        o_drain_done        : out std_logic;           -- 1-clk pulse when drain_done beat handshakes to downstream
        o_run_drain_complete : out std_logic;          -- 1-clk pulse when chip_run internally finishes drain (Round 5 #11)

        -- Status
        o_shot_seq          : out unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        o_busy              : out std_logic;

        -- Error flags (1-clk pulses)
        o_err_drain_timeout : out std_logic;    -- range + drain-margin budget expired before completion
        o_err_sequence      : out std_logic;    -- stop_tdc arrived before synchronized IrFlag
        o_err_rsp_mismatch  : out std_logic;    -- bus response tuser mismatch (sticky)
        o_err_raw_overflow  : out std_logic;    -- sticky: OR of raw-drop + drain-cap (legacy, retained)
        -- Distinguish a raw FIFO beat loss from the response-drain hard cap.
        o_err_raw_drop      : out std_logic;    -- sticky: raw FIFO beat dropped
        o_err_drain_cap     : out std_logic;    -- sticky: protocol grace cap hit while bus busy
        o_err_reg_overflow  : out std_logic;    -- sticky: chip_reg 3rd-pulse queue overflow (Round 5 #12)
        o_run_timeout       : out std_logic;    -- 1-clk pulse: chip_run abnormal drain exit
        -- Round 11 C: surface chip_run's timeout cause code for SW diagnosis.
        o_run_timeout_cause : out std_logic_vector(2 downto 0);
        -- Round 11 item 14: chip_init cfg_write coalesce sticky (per-chip).
        o_init_cfg_coalesced : out std_logic;
        -- Round 11 item 18 (C): cmd_arb contract violation sticky (per-chip).
        -- Fires on this chip's instance when PH_IDLE observes >1 command
        -- pulse in the same cycle. config_ctrl aggregates all c_MAX_CHIPS
        -- outputs into a mask so SW can see WHICH chip saw the collision.
        -- Investigate cmd_arb (source serialization failure), not the
        -- dropped command itself.
        o_err_cmd_collision  : out std_logic;
        -- Round 13 axis 2: bus fatal sticky (PH_RESP_DRAIN quarantine cap
        -- reached AND bus still stuck). In-band recovery impossible.
        o_err_bus_fatal        : out std_logic
    );
end entity tdc_gpx_chip_ctrl;

architecture coordinator of tdc_gpx_chip_ctrl is

    -- =========================================================================
    -- Coordinator phase tracking
    -- =========================================================================
    type t_phase is (PH_INIT, PH_IDLE, PH_RUN, PH_REG, PH_CFG_WRITE, PH_RESP_DRAIN);
    -- synthesis translate_off
    function fn_phase_name(p_phase : t_phase) return string is
    begin
        case p_phase is
            when PH_INIT       => return "PH_INIT";
            when PH_IDLE       => return "PH_IDLE";
            when PH_RUN        => return "PH_RUN";
            when PH_REG        => return "PH_REG";
            when PH_CFG_WRITE  => return "PH_CFG_WRITE";
            when PH_RESP_DRAIN => return "PH_RESP_DRAIN";
        end case;
    end function;
    -- synthesis translate_on
    -- 10 bits cover every legal runtime bus profile without a multiplier:
    -- max div=63, ticks=7 plus request/turnaround/setup/hold is <1023 clocks.
    constant c_RESP_DRAIN_GRACE_CLKS : positive := 1023;
    signal s_phase_r      : t_phase := PH_INIT;
    signal s_drain_cnt_r  : unsigned(9 downto 0) := (others => '0');
    signal s_drain_to_init_r : std_logic := '0';  -- '1' = drain→PH_INIT (soft reset), '0' = drain→PH_IDLE (timeout)
    -- Round 9 #6 + Round 11 item 5: secondary quarantine counter.
    -- Runs while the phase is stuck in quarantine (grace counter saturated AND
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
    -- Raw AXI-Stream circular FIFO contract.
    --
    -- A shot emits at most two semantic control beats: IFIFO1-done and final
    -- drain-done. Those two slots are never available to data. Source busy is
    -- registered at occupancy 4; two additional data slots absorb the raw
    -- valid/response-skid reaction latency before the source stops. Thus:
    --
    --   8 total = 4 backlog + 2 backpressure reaction + 2 control reserve
    --
    -- The old shift-register implementation scanned every valid bit, shifted
    -- all entries on dequeue, and searched/compacted again for control-beat
    -- eviction. Besides being difficult to prove, it asserted source busy
    -- only at full even though data was rejected at occupancy 5. The circular
    -- FIFO below uses one read pointer, one write pointer, and one count.
    constant c_RAW_FIFO_DEPTH : natural := 8;
    constant c_RAW_CONTROL_RESERVE : natural := 2;
    constant c_RAW_BP_REACTION     : natural := 2;
    constant c_RAW_DATA_LIMIT      : natural :=
        c_RAW_FIFO_DEPTH - c_RAW_CONTROL_RESERVE;
    constant c_RAW_BUSY_LEVEL      : natural :=
        c_RAW_DATA_LIMIT - c_RAW_BP_REACTION;

    type t_raw_fifo_mem is array (0 to c_RAW_FIFO_DEPTH - 1)
        of std_logic_vector(c_RAW_AXIS_PACK_WIDTH - 1 downto 0);
    signal s_raw_fifo_mem       : t_raw_fifo_mem;
    signal s_raw_rd_ptr_r       : natural range 0 to c_RAW_FIFO_DEPTH - 1 := 0;
    signal s_raw_wr_ptr_r       : natural range 0 to c_RAW_FIFO_DEPTH - 1 := 0;
    signal s_raw_count_r        : natural range 0 to c_RAW_FIFO_DEPTH := 0;
    signal s_raw_head           : std_logic_vector(c_RAW_AXIS_PACK_WIDTH - 1 downto 0);
    signal s_raw_hold_busy      : std_logic := '0';

    attribute ram_style : string;
    attribute ram_style of s_raw_fifo_mem : signal is "distributed";

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
    signal s_bus_rsp_pending_i : std_logic;
    signal s_rsp_sk_sdata    : std_logic_vector(c_BUS_RSP_PACK_WIDTH - 1 downto 0);
    signal s_rsp_sk_mdata    : std_logic_vector(c_BUS_RSP_PACK_WIDTH - 1 downto 0);
    signal s_rsp_sk_tvalid   : std_logic;
    signal s_rsp_sk_tready   : std_logic;
    signal s_rsp_sk_tdata    : t_bus_rsp_tdata;
    signal s_rsp_sk_tuser    : t_bus_rsp_tuser;

    -- =========================================================================
    -- Config snapshots (latched at cmd_start, refreshed on cfg_write/reg)
    -- =========================================================================
    signal s_drain_mode_snap_r  : std_logic := '0';
    signal s_n_drain_cap_snap_r : unsigned(3 downto 0) := (others => '0');
    signal s_bus_clk_div_snap_r : unsigned(5 downto 0) := to_unsigned(2, 6);
    signal s_bus_ticks_snap_r   : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal s_range_budget_tdc_snap_r : unsigned(15 downto 0) := (others => '0');
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
    signal s_err_raw_overflow_r   : std_logic := '0';  -- sticky: FIFO credit violation
    signal s_drain_done_faulted   : std_logic;  -- Round 13 axis 1a: from chip_run
    -- Round 13 axis 2: bus fatal sticky.
    -- Set when PH_RESP_DRAIN quarantine reaches saturation (~65K cycles
    -- @200MHz = ~327us) with bus still busy/rsp_pending. Means in-band
    -- recovery is impossible; SW must hard-reset or power-cycle.
    -- OR-folded into top-level status.err_fatal so it joins the
    -- err_handler fatal path that SW already watches.
    signal s_err_bus_fatal_r        : std_logic := '0';
    -- Round 13 follow-up (audit 3번): bounded auto-recover from quarantine.
    -- After s_err_bus_fatal_r is set, if the bus later stabilises at idle
    -- (busy='0' AND rsp_pending='0') for g_BUS_IDLE_STABLE_CLKS consecutive
    -- cycles, auto-transition to PH_INIT. This reclaims liveness when the
    -- bus recovers on its own without requiring SW force_reinit, while
    -- still guarding against phase pollution via the idle-stable window.
    -- s_err_bus_fatal_r intentionally stays latched for post-mortem diagnosis.
    -- Counter width: 24-bit accommodates generic override up to ~84 ms.
    constant c_BUS_IDLE_CNT_WIDTH   : natural := 24;
    signal   s_bus_idle_stable_cnt_r : unsigned(c_BUS_IDLE_CNT_WIDTH - 1 downto 0)
                                       := (others => '0');
    signal s_err_drain_cap_r      : std_logic := '0';  -- sticky: PH_RESP_DRAIN hit hard cap while bus still active
    signal s_err_cmd_collision_r  : std_logic := '0';  -- Round 11 item 18 (C): per-chip PH_IDLE cmd collision sticky (cmd_arb contract violation)
    -- #13: i_stop_tdc CDC moved to config_ctrl.u_cdc_stop_tdc (xpm_cdc_pulse);
    -- arrives here as a clean 1-cycle pulse in the TDC clock domain, so no
    -- internal 2-FF sync or edge-detect is needed.

    -- Effective reset for ALL sub-FSMs: hard reset OR soft_reset
    signal s_sub_rst_n            : std_logic;
    -- s_soft_reset_d1_r removed: PH_RESP_DRAIN handles delayed init start
    signal s_force_reinit_start_pending_r : std_logic := '0';
    signal s_stopdis_latch_r      : std_logic := '1';  -- latched stopdis for PH_IDLE

begin

    -- Combine global soft_reset with per-chip error recovery reset
    s_sub_rst_n <= i_rst_n
                   and (not i_cmd_soft_reset)
                   and (not i_cmd_soft_reset_err)
                   and (not i_cmd_force_reinit);

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
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS,
            g_DRAIN_MARGIN_CLKS => g_DRAIN_MARGIN_CLKS,
            g_EF_SYNC_GUARD_CLKS => g_EF_SYNC_GUARD_CLKS
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
            o_drain_done_faulted => s_drain_done_faulted,  -- Round 13 axis 1a
            o_armed             => s_run_armed,
            i_drain_mode        => s_drain_mode_snap_r,
            i_n_drain_cap       => s_n_drain_cap_snap_r,
            i_cfg_image         => s_cfg_image_snap_r,
            i_shot_start        => i_shot_start,
            -- Pass through the TDC-local count. chip_run takes its own
            -- shot-bounded snapshot internally
            -- on the ST_ARMED→ST_CAPTURE edge, so passing the live
            -- value (rather than a coordinator-latched copy) is fine.
            i_max_range_tdc_clks => i_max_range_tdc_clks,
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

    -- Bus response input boundary.
    -- v010 sequential-logic rule: chip_ctrl owns the bus response skid, so
    -- the module boundary o_s_axis_tready is registered by tdc_gpx_skid_buffer.
    -- The sub-FSMs consume from the skid output; PH_RESP_DRAIN still drains
    -- stale responses without routing them.
    s_rsp_sk_sdata <= i_s_axis_tdata & i_s_axis_tuser;

    u_rsp_skid : entity work.tdc_gpx_skid_buffer
        generic map (g_DATA_WIDTH => c_BUS_RSP_PACK_WIDTH)
        port map (
            i_clk     => i_clk,
            i_rst_n   => s_sub_rst_n,
            i_flush   => '0',
            i_s_valid => i_s_axis_tvalid,
            o_s_ready => o_s_axis_tready,
            i_s_data  => s_rsp_sk_sdata,
            o_m_valid => s_rsp_sk_tvalid,
            i_m_ready => s_rsp_sk_tready,
            o_m_data  => s_rsp_sk_mdata
        );

    s_rsp_sk_tdata <= s_rsp_sk_mdata(c_BUS_RSP_PACK_WIDTH - 1 downto c_BUS_RSP_TUSER_WIDTH);
    s_rsp_sk_tuser <= s_rsp_sk_mdata(c_BUS_RSP_TUSER_WIDTH - 1 downto 0);

    s_rsp_sk_tready <= '0' when s_phase_r = PH_RUN and s_raw_hold_busy = '1'
                  else '1' when s_phase_r = PH_RESP_DRAIN
                  else '1' when s_init_busy = '1' or s_run_busy = '1' or s_reg_busy = '1'
                  else '0';

    -- Bus response FIRE pulse: valid AND ready. Sub-FSMs must use this,
    -- not raw tvalid, to avoid consuming the same beat multiple times
    -- when tready is deasserted (e.g., raw hold full during PH_RUN).
    s_bus_rsp_fire   <= s_rsp_sk_tvalid and s_rsp_sk_tready;
    s_bus_rsp_pending_i <= i_bus_rsp_pending or i_s_axis_tvalid or s_rsp_sk_tvalid;

    -- Bus response routing (PH_RESP_DRAIN: all routing disabled)
    s_init_rsp_valid <= s_bus_rsp_fire when (s_phase_r = PH_INIT or s_phase_r = PH_CFG_WRITE) else '0';
    s_run_rsp_valid  <= s_bus_rsp_fire when s_phase_r = PH_RUN else '0';
    -- Pending routed to chip_run: "response arrived at bus_phy/skid but may
    -- not have fired this cycle." Combines the bus_phy pending flag with
    -- raw tvalid held high while tready is low (raw hold full during PH_RUN).
    -- chip_run uses this to:
    --   - freeze EF1/EF2/BURST wait watchdogs on downstream backpressure
    --   - decide DRAIN_FLUSH / OVERRUN_FLUSH completion (Round 5 #1/#2)
    s_run_rsp_pending <= s_bus_rsp_pending_i
                         when s_phase_r = PH_RUN else '0';
    s_run_rsp_rdata  <= s_rsp_sk_tdata(g_BUS_DATA_WIDTH - 1 downto 0);
    s_reg_rsp_valid  <= s_bus_rsp_fire when s_phase_r = PH_REG else '0';
    s_reg_rsp_rdata  <= s_rsp_sk_tdata(g_BUS_DATA_WIDTH - 1 downto 0);

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
                s_err_bus_fatal_r        <= '0';
                s_drain_quarantine_cnt_r <= (others => '0');
                s_bus_idle_stable_cnt_r  <= (others => '0');
                s_force_reinit_start_pending_r <= '0';
                s_range_budget_tdc_snap_r <= (others => '0');
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
                            s_range_budget_tdc_snap_r <= fn_range_budget_clks(
                                i_max_range_tdc_clks,
                                g_DRAIN_MARGIN_CLKS);
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
                        -- The 10-bit counter covers every legal runtime bus
                        -- transaction before quarantine (see below).
                        if s_drain_cnt_r /= to_unsigned(
                            c_RESP_DRAIN_GRACE_CLKS,
                            s_drain_cnt_r'length) then
                            s_drain_cnt_r <= s_drain_cnt_r + 1;
                        end if;
                        if i_bus_busy = '0' and i_bus_rsp_pending = '0'
                           and s_drain_cnt_r >= to_unsigned(
                               3, s_drain_cnt_r'length)
                           and s_err_bus_fatal_r = '0' then
                            if s_drain_to_init_r = '1' then
                                -- synthesis translate_off
                                assert false
                                    report "chip_ctrl[" & integer'image(g_CHIP_ID) &
                                           "]: PH_RESP_DRAIN exit to PH_INIT cnt=" &
                                           integer'image(to_integer(s_drain_cnt_r)) &
                                           " bus_busy=" & std_logic'image(i_bus_busy) &
                                           " rsp_pending=" & std_logic'image(i_bus_rsp_pending) &
                                           " axis_valid=" & std_logic'image(i_s_axis_tvalid) &
                                           " skid_valid=" & std_logic'image(s_rsp_sk_tvalid)
                                    severity note;
                                -- synthesis translate_on
                                s_phase_r    <= PH_INIT;
                                s_init_start <= '1';
                            else
                                -- synthesis translate_off
                                assert false
                                    report "chip_ctrl[" & integer'image(g_CHIP_ID) &
                                           "]: PH_RESP_DRAIN exit to PH_IDLE cnt=" &
                                           integer'image(to_integer(s_drain_cnt_r)) &
                                           " bus_busy=" & std_logic'image(i_bus_busy) &
                                           " rsp_pending=" & std_logic'image(i_bus_rsp_pending) &
                                           " axis_valid=" & std_logic'image(i_s_axis_tvalid) &
                                           " skid_valid=" & std_logic'image(s_rsp_sk_tvalid)
                                    severity note;
                                -- synthesis translate_on
                                s_phase_r <= PH_IDLE;
                            end if;
                            s_drain_to_init_r <= '0';  -- always clear on drain exit
                        elsif s_drain_cnt_r = to_unsigned(
                            c_RESP_DRAIN_GRACE_CLKS,
                            s_drain_cnt_r'length) then
                            -- Protocol grace cap reached. Round 5 #9 +
                            -- Round 9 #6 + Round 11 item 5:
                            --   Round 5 #9 made this a QUARANTINE — stay in
                            --   PH_RESP_DRAIN with its grace counter saturated and
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
                                if s_drain_quarantine_cnt_r = x"0000" then
                                    -- synthesis translate_off
                                    assert false
                                        report "chip_ctrl[" & integer'image(g_CHIP_ID) &
                                               "]: PH_RESP_DRAIN cap entered bus_busy=" &
                                               std_logic'image(i_bus_busy) &
                                               " rsp_pending=" & std_logic'image(i_bus_rsp_pending) &
                                               " axis_valid=" & std_logic'image(i_s_axis_tvalid) &
                                               " skid_valid=" & std_logic'image(s_rsp_sk_tvalid) &
                                               " drain_to_init=" & std_logic'image(s_drain_to_init_r)
                                        severity warning;
                                    -- synthesis translate_on
                                end if;
                                s_err_drain_cap_r <= '1';
                                if s_drain_quarantine_cnt_r /= x"FFFF" then
                                    s_drain_quarantine_cnt_r <=
                                        s_drain_quarantine_cnt_r + 1;
                                elsif s_err_bus_fatal_r = '0' then
                                    -- Round 13 axis 2: at quarantine cap AND
                                    -- bus still stuck → escalate to fatal.
                                    -- This is the "dead-end" signal for SW:
                                    -- in-band recovery (soft_reset, force_
                                    -- reinit) cannot fix a bus that has been
                                    -- unresponsive for 65K consecutive cycles.
                                    -- OR-folded into status.err_fatal at top
                                    -- so it joins err_handler's fatal path.
                                    -- synthesis translate_off
                                    assert false
                                        report "chip_ctrl[" & integer'image(g_CHIP_ID) &
                                               "]: PH_RESP_DRAIN bus fatal latched bus_busy=" &
                                               std_logic'image(i_bus_busy) &
                                               " rsp_pending=" & std_logic'image(i_bus_rsp_pending) &
                                               " axis_valid=" & std_logic'image(i_s_axis_tvalid) &
                                               " skid_valid=" & std_logic'image(s_rsp_sk_tvalid)
                                        severity warning;
                                    -- synthesis translate_on
                                    s_err_bus_fatal_r <= '1';
                                end if;
                                -- At x"FFFF": saturate in place. No in-band
                                -- escalation — but fatal sticky above is now
                                -- set the first time we reach saturation.
                            elsif s_err_bus_fatal_r = '0' then
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

                -- Round 13 follow-up (audit 3번): bounded auto-recover after
                -- s_err_bus_fatal_r is latched. Counter increments only while
                -- in PH_RESP_DRAIN AND the bus has stabilised at idle. Any
                -- activity (busy or rsp_pending) resets the counter. When
                -- the window is reached, transition to PH_INIT. Under the
                -- current legacy wiring the bus-fatal sticky remains set
                -- until hard reset; i_soft_clear does not yet reach this
                -- coordinator-owned register.
                -- If the bus never clears, the counter never reaches the
                -- threshold and the module stays in quarantine as before.
                if s_phase_r = PH_RESP_DRAIN and s_err_bus_fatal_r = '1' then
                    if i_bus_busy = '0' and i_bus_rsp_pending = '0' then
                        if s_bus_idle_stable_cnt_r <
                           to_unsigned(g_BUS_IDLE_STABLE_CLKS,
                                       s_bus_idle_stable_cnt_r'length) then
                            s_bus_idle_stable_cnt_r <= s_bus_idle_stable_cnt_r + 1;
                        else
                            -- synthesis translate_off
                            assert false
                                report "chip_ctrl[" & integer'image(g_CHIP_ID) &
                                       "]: PH_RESP_DRAIN auto-recover to PH_INIT after idle-stable window"
                                severity note;
                            -- synthesis translate_on
                            s_phase_r                <= PH_INIT;
                            s_init_start             <= '1';
                            s_drain_cnt_r            <= (others => '0');
                            s_drain_to_init_r        <= '0';
                            s_drain_quarantine_cnt_r <= (others => '0');
                            s_bus_idle_stable_cnt_r  <= (others => '0');
                        end if;
                    else
                        s_bus_idle_stable_cnt_r <= (others => '0');
                    end if;
                else
                    s_bus_idle_stable_cnt_r <= (others => '0');
                end if;

                if s_force_reinit_start_pending_r = '1' then
                    s_init_start <= '1';
                    s_force_reinit_start_pending_r <= '0';
                end if;

                -- Soft reset: global OR per-chip error recovery
                -- Drain stale responses first, then restart init
                -- (s_soft_reset_d1_r removed)
                if i_cmd_soft_reset = '1' or i_cmd_soft_reset_err = '1' then
                    -- synthesis translate_off
                    assert false
                        report "chip_ctrl[" & integer'image(g_CHIP_ID) &
                               "]: soft_reset enter PH_RESP_DRAIN old_phase=" &
                               fn_phase_name(s_phase_r) &
                               " soft=" & std_logic'image(i_cmd_soft_reset) &
                               " err_soft=" & std_logic'image(i_cmd_soft_reset_err) &
                               " bus_busy=" & std_logic'image(i_bus_busy) &
                               " rsp_pending=" & std_logic'image(i_bus_rsp_pending) &
                               " axis_valid=" & std_logic'image(i_s_axis_tvalid) &
                               " skid_valid=" & std_logic'image(s_rsp_sk_tvalid)
                        severity note;
                    -- synthesis translate_on
                    s_cfg_image_snap_r   <= i_cfg_image;
                    s_phase_r            <= PH_RESP_DRAIN;
                    s_drain_cnt_r        <= (others => '0');
                    s_drain_to_init_r    <= '1';  -- soft reset → drain then init
                end if;

                -- Round 12 A1: force-reinit escape. Highest-priority override
                -- (last-assignment wins in this sequential process). Bypasses
                -- PH_RESP_DRAIN entirely — SW MUST have flushed the bus
                -- externally before pulsing this. The command itself is the
                -- software-owned audit point for this manual recovery action.
                if i_cmd_force_reinit = '1' then
                    -- synthesis translate_off
                    assert false
                        report "chip_ctrl[" & integer'image(g_CHIP_ID) &
                               "]: force_reinit to PH_INIT old_phase=" &
                               fn_phase_name(s_phase_r) &
                               " bus_busy=" & std_logic'image(i_bus_busy) &
                               " rsp_pending=" & std_logic'image(i_bus_rsp_pending) &
                               " axis_valid=" & std_logic'image(i_s_axis_tvalid) &
                               " skid_valid=" & std_logic'image(s_rsp_sk_tvalid)
                        severity note;
                    -- synthesis translate_on
                    s_cfg_image_snap_r       <= i_cfg_image;
                    s_phase_r                <= PH_INIT;
                    s_init_start             <= '0';
                    s_drain_cnt_r            <= (others => '0');
                    s_drain_to_init_r        <= '0';
                    s_drain_quarantine_cnt_r <= (others => '0');
                    s_force_reinit_start_pending_r <= '1';
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
                    if s_range_budget_tdc_snap_r /= 0
                       and s_range_cnt_r >= s_range_budget_tdc_snap_r then
                        if s_err_drain_to_fired_r = '0' then
                            s_err_drain_timeout_r  <= '1';
                            s_err_drain_to_fired_r <= '1';
                        end if;
                    else
                        s_range_cnt_r <= s_range_cnt_r + 1;
                    end if;
                end if;

                -- Sequence error: the Laser measurement window ended before
                -- the synchronized GPX IrFlag closed capture. Once IrFlag is
                -- visible, stop_tdc during drain/ALU is the normal ordering.
                -- #13: i_stop_tdc is now a clean 1-cycle pulse from xpm_cdc_pulse in
                -- config_ctrl (no internal 2-FF / edge detect needed).
                if s_phase_r = PH_RUN
                   and s_run_armed = '0'
                   and s_run_range_active = '1'
                   and i_irflag_sync = '0'
                   and i_stop_tdc = '1' then
                    s_err_sequence_r <= '1';
                end if;
            end if;
        end if;
    end process p_range_cnt;

    -- =========================================================================
    -- StopDis output: override OR FSM-controlled
    --
    -- Policy (#24, documented intent):
    --   INTENTIONALLY LIVE: debug/emergency override must take effect
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
    o_bus_clk_div_snap <= s_bus_clk_div_snap_r;
    o_bus_ticks_snap <= s_bus_ticks_snap_r;
    o_puresn         <= s_init_puresn;
    o_alutrigger     <= s_run_alutrigger;

    -- AXI-Stream raw word FIFO. Pop is applied before push in the variable
    -- count snapshot, so a full FIFO can still sustain one pop + one push in
    -- the same cycle. Memory contents are not reset; count=0 defines empty.
    p_raw_fifo : process(i_clk)
        variable v_rd_ptr  : natural range 0 to c_RAW_FIFO_DEPTH - 1;
        variable v_wr_ptr  : natural range 0 to c_RAW_FIFO_DEPTH - 1;
        variable v_count   : natural range 0 to c_RAW_FIFO_DEPTH;
        variable v_push    : boolean;
        variable v_control : boolean;
        variable v_accept  : boolean;
        variable v_tdata   : t_raw_axis_tdata;
        variable v_tuser   : t_raw_axis_tuser;
    begin
        if rising_edge(i_clk) then
            if s_sub_rst_n = '0' then
                s_raw_rd_ptr_r <= 0;
                s_raw_wr_ptr_r <= 0;
                s_raw_count_r  <= 0;
                s_raw_hold_busy <= '0';
                s_err_raw_overflow_r <= '0';
            else
                v_rd_ptr := s_raw_rd_ptr_r;
                v_wr_ptr := s_raw_wr_ptr_r;
                v_count  := s_raw_count_r;

                if v_count > 0 and i_m_raw_axis_tready = '1' then
                    if v_rd_ptr = c_RAW_FIFO_DEPTH - 1 then
                        v_rd_ptr := 0;
                    else
                        v_rd_ptr := v_rd_ptr + 1;
                    end if;
                    v_count := v_count - 1;
                end if;

                -- Control beats have priority under an impossible overlap.
                -- chip_run's state machine normally makes all three sources
                -- mutually exclusive; the assertion protects that contract.
                v_push    := false;
                v_control := false;
                v_tdata   := (others => '0');
                v_tuser   := (others => '0');

                if s_run_drain_done = '1' then
                    v_push      := true;
                    v_control   := true;
                    v_tuser(7)  := '1';
                    v_tuser(5)  := s_drain_done_faulted;
                    v_tuser(0)  := '1';
                elsif s_run_ififo1_beat = '1' then
                    v_push      := true;
                    v_control   := true;
                    v_tuser(7)  := '1';
                    v_tuser(0)  := '0';
                elsif s_run_raw_valid = '1' then
                    v_push := true;
                    v_tdata(g_BUS_DATA_WIDTH - 1 downto 0) := s_run_raw_word;
                    v_tuser(0) := s_run_ififo_id;
                end if;

                -- synthesis translate_off
                assert not (s_run_raw_valid = '1'
                            and (s_run_drain_done = '1'
                                 or s_run_ififo1_beat = '1'))
                    report "chip_ctrl: raw data/control sources overlapped"
                    severity error;
                assert not (s_run_drain_done = '1'
                            and s_run_ififo1_beat = '1')
                    report "chip_ctrl: both raw control sources overlapped"
                    severity error;
                -- synthesis translate_on

                if v_push then
                    if v_control then
                        v_accept := v_count < c_RAW_FIFO_DEPTH;
                    else
                        v_accept := v_count < c_RAW_DATA_LIMIT;
                    end if;

                    if v_accept then
                        s_raw_fifo_mem(v_wr_ptr) <= v_tdata & v_tuser;
                        if v_wr_ptr = c_RAW_FIFO_DEPTH - 1 then
                            v_wr_ptr := 0;
                        else
                            v_wr_ptr := v_wr_ptr + 1;
                        end if;
                        v_count := v_count + 1;
                    else
                        s_err_raw_overflow_r <= '1';
                        -- synthesis translate_off
                        assert false
                            report "chip_ctrl: raw FIFO credit violation (control="
                                   & boolean'image(v_control)
                                   & ", occupancy=" & integer'image(v_count)
                                   & ")"
                            severity error;
                        -- synthesis translate_on
                    end if;
                end if;

                s_raw_rd_ptr_r <= v_rd_ptr;
                s_raw_wr_ptr_r <= v_wr_ptr;
                s_raw_count_r  <= v_count;

                -- Registered source throttle. At level 4, two data credits
                -- remain for response-skid/reaction latency and two slots are
                -- still reserved exclusively for the shot's control beats.
                if i_m_raw_axis_tready = '0'
                   and v_count >= c_RAW_BUSY_LEVEL then
                    s_raw_hold_busy <= '1';
                else
                    s_raw_hold_busy <= '0';
                end if;
            end if;
        end if;
    end process p_raw_fifo;

    s_raw_head <= s_raw_fifo_mem(s_raw_rd_ptr_r);

    o_m_raw_axis_tvalid <= '1' when s_raw_count_r > 0 else '0';
    o_m_raw_axis_tdata  <= s_raw_head(c_RAW_AXIS_PACK_WIDTH - 1
                                      downto c_RAW_AXIS_TUSER_WIDTH)
                           when s_raw_count_r > 0 else (others => '0');
    o_m_raw_axis_tuser  <= s_raw_head(c_RAW_AXIS_TUSER_WIDTH - 1 downto 0)
                           when s_raw_count_r > 0 else (others => '0');

    -- o_drain_done semantic (#27):
    --   Pulses when the final drain-done control beat HANDSHAKES to
    --   downstream (tvalid='1' AND tready='1'), NOT at the moment chip_run
    --   internally finishes draining. This means downstream backpressure
    --   can delay o_drain_done relative to the actual IFIFO drain completion
    --   inside chip_run.
    --   Upstream modules (face_seq, err_handler, status_agg) consuming this
    --   pulse must interpret it as "chip_run's drain_done beat was accepted
    --   by the raw-path consumer", not as "chip_run exited the drain state".
    o_drain_done <= '1' when s_raw_count_r > 0
                            and i_m_raw_axis_tready = '1'
                            and s_raw_head(7) = '1'
                            and s_raw_head(0) = '1'
                    else '0';

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
    o_err_bus_fatal        <= s_err_bus_fatal_r;
    -- raw_overflow is the legacy summary; the two following outputs retain
    -- the individual causes for the existing error-handler contract:
    --   s_err_raw_overflow_r -> raw FIFO credit violation (beat dropped)
    --   s_err_drain_cap_r    -> PH_RESP_DRAIN hard cap hit while bus busy
    o_err_raw_overflow  <= s_err_raw_overflow_r or s_err_drain_cap_r;
    -- Round 12 #15: distinct cause outputs.
    o_err_raw_drop      <= s_err_raw_overflow_r;
    o_err_drain_cap     <= s_err_drain_cap_r;
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
                    if s_rsp_sk_tuser(0) /= v_exp_rw
                       or s_rsp_sk_tuser(4 downto 1) /= v_exp_addr then
                        s_err_rsp_mismatch_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_rsp_check;

end architecture coordinator;
