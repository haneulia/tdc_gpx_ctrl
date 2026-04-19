-- =============================================================================
-- tdc_gpx_config_ctrl.vhd
-- TDC-GPX Controller - Cluster 1: Chip Configuration & Control
-- =============================================================================
--
-- Purpose:
--   Pure structural wrapper for TDC-GPX chip configuration and control.
--   Instantiates:
--     csr_chip       : AXI-Lite CSR + SRM + CDC for chip-owned registers
--     cmd_arb        : command arbitration (cfg_write, reg read/write gating)
--     err_handler    : automatic ErrFlag detection, Reg11 read, recovery FSM
--     stop_decode    : stop event decode + cfg_image override
--     bus_phy x4     : TDC-GPX bus physical layer (IOBUF + timing FSM)
--     sk_brsp x4     : skid buffer bus_phy -> chip_ctrl (40b)
--     chip_ctrl x4   : chip FSM coordinator (powerup/cfg/arm/capture/drain)
--     raw_cdc x4     : xpm_fifo_async CDC chip_ctrl -> decode_pipe (40b)
--
--   Config merging: pipeline fields from i_cfg_pipeline, chip fields from
--   csr_chip outputs.  Merged config drives stop_decode and chip_ctrl.
--
-- Clock domains:
--   i_axis_aclk  : AXI-Stream processing domain (150 MHz)
--   i_tdc_clk    : TDC-GPX bus control domain (200 MHz)
--                  Drives bus_phy, sk_brsp, chip_ctrl.
--                  raw_cdc (xpm_fifo_async) crosses TDC -> AXI-Stream.
--   s_axi_aclk   : AXI4-Lite PS domain
--
-- Reset-domain map (Round 6 C1):
--   AXI-Stream clock (i_axis_aclk) uses i_axis_aresetn directly:
--     - u_cmd_arb         (cmd_arb)
--     - u_err_handler     (err_handler)
--     - u_stop_decode     (stop_cfg_decode)
--     - p_frame_done_latch, MUXes, per-chip CDC instances source sides
--   TDC clock (i_tdc_clk) uses s_tdc_aresetn (xpm_cdc_async_rst of
--   i_axis_aresetn, Round 5 #8):
--     - u_bus_phy         (bus_phy)
--     - u_sk_brsp         (skid buffer)
--     - u_chip_ctrl       (chip_ctrl sub-FSM coordinator)
--     - p_raw_fifo_rst / s_run_drain_complete_sticky_r (TDC stickies)
--   Async, no dedicated reset sync:
--     - u_raw_cdc         (xpm_fifo_async — rst combines i_axis_aresetn
--                          with the stretched soft_reset pulse, Round 6 A1)
--   When adding new modules, drive i_rst_n from the reset that matches
--   the module's clock domain. Do not cross-wire.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library xpm;
use xpm.vcomponents.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_config_ctrl is
    generic (
        g_HW_VERSION      : std_logic_vector(31 downto 0) := x"00010000";
        g_POWERUP_CLKS    : positive := 48;
        g_RECOVERY_CLKS   : positive := 8;
        g_ALU_PULSE_CLKS  : positive := 4;
        g_STOP_EVT_DWIDTH : natural := 32
    );
    port (
        -- Clock / Reset: processing domain (AXI-Stream, 150 MHz)
        i_axis_aclk          : in  std_logic;
        i_axis_aresetn       : in  std_logic;

        -- TDC-GPX bus control clock (200 MHz)
        -- Drives bus_phy and chip_ctrl (same clock group).
        i_tdc_clk            : in  std_logic;

        -- Clock / Reset: AXI-Lite domain (PS)
        s_axi_aclk           : in  std_logic;
        s_axi_aresetn        : in  std_logic;

        -- =====================================================================
        -- AXI4-Lite slave pass-through (9-bit address)
        -- =====================================================================
        s_axi_awvalid        : in  std_logic;
        s_axi_awready        : out std_logic;
        s_axi_awaddr         : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_awprot         : in  std_logic_vector(2 downto 0);
        s_axi_wvalid         : in  std_logic;
        s_axi_wready         : out std_logic;
        s_axi_wdata          : in  std_logic_vector(31 downto 0);
        s_axi_wstrb          : in  std_logic_vector(3 downto 0);
        s_axi_bvalid         : out std_logic;
        s_axi_bready         : in  std_logic;
        s_axi_bresp          : out std_logic_vector(1 downto 0);
        s_axi_arvalid        : in  std_logic;
        s_axi_arready        : out std_logic;
        s_axi_araddr         : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_arprot         : in  std_logic_vector(2 downto 0);
        s_axi_rvalid         : out std_logic;
        s_axi_rready         : in  std_logic;
        s_axi_rdata          : out std_logic_vector(31 downto 0);
        s_axi_rresp          : out std_logic_vector(1 downto 0);

        -- =====================================================================
        -- TDC-GPX physical pins pass-through (per chip, x4)
        -- =====================================================================
        io_tdc_d             : inout t_tdc_bus_array;
        o_tdc_adr            : out   t_tdc_adr_array;
        o_tdc_csn            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_rdn            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_wrn            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_oen            : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_stopdis        : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_alutrigger     : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_puresn         : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_ef1            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_ef2            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_lf1            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_lf2            : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_irflag         : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_errflag        : in    std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- =====================================================================
        -- External inputs (stop event stream + deadline pulse)
        -- =====================================================================
        i_stop_evt_tvalid    : in  std_logic;
        i_stop_evt_tdata     : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        i_stop_evt_tkeep     : in  std_logic_vector(g_STOP_EVT_DWIDTH/8 - 1 downto 0);
        i_stop_evt_tuser     : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        o_stop_evt_tready    : out std_logic;
        i_stop_tdc           : in  std_logic;

        -- =====================================================================
        -- Control inputs from face_seq / csr_pipeline (top-level routing)
        -- =====================================================================
        i_cmd_start          : in  std_logic;
        i_cmd_start_accepted : in  std_logic;
        i_cmd_stop           : in  std_logic;
        i_cmd_soft_reset     : in  std_logic;
        -- Round 12 A1: SW-issued force re-init (AXI-Stream domain pulse).
        -- Routed through xpm_cdc_pulse to each chip_ctrl. Use only when
        -- PH_RESP_DRAIN is observed stuck (o_err_drain_cap_mask set).
        i_cmd_force_reinit   : in  std_logic := '0';
        i_cmd_cfg_write      : in  std_logic;
        -- SW-initiated clear for err_handler fatal state + error history
        -- (shared with status_agg soft_clear). Default '0' keeps legacy.
        i_err_soft_clear     : in  std_logic := '0';
        i_shot_start_per_chip : in  std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_shot_start_gated   : in  std_logic;
        i_cfg_pipeline       : in  t_tdc_cfg;

        -- =====================================================================
        -- Cluster 4 idle inputs (for cmd_arb gating)
        -- =====================================================================
        i_face_asm_idle      : in  std_logic;
        i_face_asm_fall_idle : in  std_logic;
        i_hdr_idle           : in  std_logic;
        i_hdr_fall_idle      : in  std_logic;

        -- =====================================================================
        -- Frame boundary (from output_stage, for err_handler)
        -- =====================================================================
        i_frame_done         : in  std_logic;
        i_frame_fall_done    : in  std_logic;

        -- Pipeline abort: flush raw-path skid buffers
        i_pipeline_abort     : in  std_logic;

        -- =====================================================================
        -- Output to Cluster 2: AXI-Stream x4 (from sk_raw)
        -- =====================================================================
        o_raw_sk_tvalid      : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_raw_sk_tdata       : out t_slv32_array;
        o_raw_sk_tuser       : out t_slv8_array;
        i_raw_sk_tready      : in  std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- =====================================================================
        -- Configuration outputs (merged from csr_chip + csr_pipeline)
        -- =====================================================================
        o_cfg                : out t_tdc_cfg;
        o_cfg_image          : out t_cfg_image;

        -- =====================================================================
        -- Command outputs
        -- =====================================================================
        o_cmd_start          : out std_logic;
        o_cmd_cfg_write_g    : out std_logic;

        -- =====================================================================
        -- Chip status outputs (for face_seq + status_agg)
        -- =====================================================================
        o_chip_busy          : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_chip_shot_seq      : out t_shot_seq_array;
        o_errflag_sync       : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_drain_timeout  : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_sequence       : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_rsp_mismatch   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_raw_overflow   : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_reg_outstanding    : out std_logic;
        o_reg_loop_resume    : out std_logic;
        o_cdc_idle           : out std_logic;

        -- =====================================================================
        -- Error handler status outputs
        -- =====================================================================
        o_err_active         : out std_logic;
        o_err_fatal          : out std_logic;
        o_err_chip_mask      : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_cause          : out std_logic_vector(2 downto 0);

        -- =====================================================================
        -- Additional status outputs
        -- =====================================================================
        o_run_timeout        : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_reg_arb_timeout    : out std_logic;

        -- Round 5 follow-up: pipeline-wide sticky observability (from sub-blocks)
        o_err_read_timeout   : out std_logic;  -- err_handler ST_WAIT_READ watchdog fired
        -- Round 11 item 11: separate sticky for ST_WAIT_FRAME_DONE escape
        o_err_frame_wait_escape : out std_logic;
        o_reg_rejected       : out std_logic;  -- cmd_arb overlap queue full (request lost)
        o_reg_zero_mask      : out std_logic;  -- cmd_arb got zero chip_mask request

        -- Round 6 B1: per-chip observability (TDC-domain sources, CDC'd here)
        o_err_reg_overflow   : out std_logic_vector(c_N_CHIPS - 1 downto 0);  -- chip_reg 3rd-pulse sticky
        o_run_drain_complete : out std_logic_vector(c_N_CHIPS - 1 downto 0);  -- chip_run internal drain-complete sticky

        -- Round 11 C: Category C observability surface
        o_reg_timeout_mask   : out std_logic_vector(c_N_CHIPS - 1 downto 0);  -- cmd_arb per-chip reg timeout
        o_run_timeout_cause  : out std_logic_vector(2 downto 0);              -- chip_run last timeout cause (OR-aggregated)

        -- Round 11 item 10: stop_cfg_decode monotonic violation sticky.
        o_mono_violation_mask : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 #16: stop_cfg_decode orphan-event sticky.
        o_orphan_stop_evt_sticky : out std_logic;

        -- Round 11 item 14: per-chip chip_init cfg_write coalesce sticky.
        o_init_cfg_coalesced_mask : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 11 item 18 (C): per-chip PH_IDLE cmd-collision sticky mask.
        o_cmd_collision_mask      : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 A1: per-chip force-reinit used sticky.
        o_err_force_reinit_mask   : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 A2: per-chip raw control-beat drop sticky.
        o_err_raw_ctrl_drop_mask  : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 A4: per-chip chip_run drain mismatch sticky.
        o_err_drain_mismatch_mask : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 A5: concurrent R+W ambiguity stickies.
        o_err_rw_ambiguous_arb    : out std_logic;  -- cmd_arb (AXI-Stream domain)
        o_err_rw_ambiguous_reg    : out std_logic_vector(c_N_CHIPS - 1 downto 0);  -- chip_reg per chip (CDC'd)

        -- Round 12 B8: per-chip stopdis_override mid-shot sticky.
        o_err_stopdis_mid_shot_mask : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 #20: PH_IDLE command collision vector (OR across chips).
        o_err_cmd_collision_vec  : out std_logic_vector(3 downto 0);

        -- Round 12 #15: distinct raw-overflow cause masks (per-chip).
        o_err_raw_drop_mask      : out std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_err_drain_cap_mask     : out std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- Round 12 #17: per-chip run_timeout_cause (packed 3-bit × N_CHIPS).
        --   chip i → [3*i + 2 : 3*i]
        -- Replaces the "last cause only" behavior of o_run_timeout_cause.
        o_run_timeout_cause_per_chip : out std_logic_vector(3 * c_N_CHIPS - 1 downto 0);

        -- =====================================================================
        -- Interrupt
        -- =====================================================================
        o_irq                : out std_logic
    );
end entity tdc_gpx_config_ctrl;

architecture rtl of tdc_gpx_config_ctrl is

    attribute KEEP_HIERARCHY : string;
    attribute KEEP_HIERARCHY of rtl : architecture is "yes";

    -- =========================================================================
    -- csr_chip configuration outputs (i_axis_aclk domain)
    -- =========================================================================
    signal s_bus_clk_div     : unsigned(5 downto 0);
    signal s_bus_ticks       : unsigned(2 downto 0);
    signal s_start_off1      : unsigned(17 downto 0);
    signal s_cfg_reg7        : std_logic_vector(31 downto 0);
    signal s_max_scan_clks   : unsigned(15 downto 0);
    signal s_max_hits_cfg    : unsigned(2 downto 0);
    signal s_cfg_image_raw   : t_cfg_image;

    -- =========================================================================
    -- Merged configuration (pipeline + chip fields)
    -- =========================================================================
    signal s_cfg_merged      : t_tdc_cfg;

    -- =========================================================================
    -- cmd_arb internal signals
    -- =========================================================================
    signal s_cmd_reg_read    : std_logic;
    signal s_cmd_reg_write   : std_logic;
    signal s_cmd_reg_addr    : std_logic_vector(3 downto 0);
    signal s_cmd_reg_chip    : unsigned(1 downto 0);
    signal s_cmd_reg_wdata   : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal s_cmd_reg_rdata   : t_slv28_array;
    signal s_cmd_reg_rvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_done    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_read_g  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_write_g : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_chip_address    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_done_pulse   : std_logic;
    signal s_reg_outstanding      : std_logic;
    signal s_cmd_reg_done_chip    : unsigned(1 downto 0);
    signal s_reg_loop_resume      : std_logic;
    signal s_cmd_reg_addr_out     : std_logic_vector(3 downto 0);

    -- =========================================================================
    -- stop_decode outputs
    -- =========================================================================
    signal s_expected_ififo1 : t_expected_array;
    signal s_expected_ififo2 : t_expected_array;

    -- =========================================================================
    -- Per-chip: bus_phy <-> chip_ctrl
    -- =========================================================================
    signal s_bus_req_valid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_rw      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_addr    : t_slv4_array;
    signal s_bus_req_wdata   : t_slv28_array;
    signal s_bus_oen_perm    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_burst   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_ticks_snap  : t_u3_array;
    signal s_bus_busy           : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_rsp_pending_raw : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_rsp_pending     : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: bus_phy response AXI-Stream
    signal s_brsp_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_brsp_axis_tdata  : t_slv32_array;
    signal s_brsp_axis_tkeep  : t_slv4_array;
    signal s_brsp_axis_tuser  : t_slv8_array;
    signal s_brsp_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: skid buffer bus_phy -> chip_ctrl
    signal s_brsp_sk_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_brsp_sk_tdata   : t_slv32_array;
    signal s_brsp_sk_tuser   : t_slv8_array;
    signal s_brsp_sk_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: bus_phy synchronized status
    signal s_ef1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_ef2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_irflag_sync     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_errflag_sync    : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: chip_ctrl -> downstream raw AXI-Stream
    signal s_raw_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_axis_tdata  : t_slv32_array;
    signal s_raw_axis_tuser  : t_slv8_array;
    signal s_raw_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: chip_ctrl status
    signal s_drain_done      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_shot_seq   : t_shot_seq_array;
    signal s_chip_busy       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_tick_en         : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_drain_timeout : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_sequence      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_rsp_mismatch  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_raw_overflow  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 12 #15: per-chip distinct cause stickies
    signal s_err_raw_drop      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_drain_cap     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 6 B1: additional per-chip observability from chip_ctrl (TDC domain)
    signal s_err_reg_overflow  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_run_drain_complete : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- pulse
    signal s_run_drain_complete_sticky_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_run_timeout       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 11 C: per-chip timeout_cause + aggregated "last cause" signal
    type t_timeout_cause_arr is array (0 to c_N_CHIPS - 1) of std_logic_vector(2 downto 0);
    signal s_run_timeout_cause     : t_timeout_cause_arr;
    -- Round 11 item 14: per-chip chip_init cfg_write coalesce sticky
    signal s_init_cfg_coalesced    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 11 item 18 (C): per-chip PH_IDLE cmd-collision sticky mask
    signal s_cmd_collision_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_run_timeout_cause_last_r : std_logic_vector(2 downto 0) := (others => '0');
    -- Round 12 #17: per-chip cause latch (each chip keeps its last cause)
    signal s_run_timeout_cause_per_chip_r : t_timeout_cause_arr := (others => (others => '0'));
    signal s_reg_arb_timeout   : std_logic;

    -- =========================================================================
    -- err_handler outputs
    -- =========================================================================
    signal s_err_cmd_soft_reset    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_cmd_reg_read      : std_logic;
    signal s_err_cmd_reg_addr      : std_logic_vector(3 downto 0);
    signal s_err_cmd_reg_chip_addr : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_fill              : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_active            : std_logic;

    -- err_handler i_frame_done: latch rise+fall frame_done observations
    -- independently, clear on next shot_start. Combinational AND of two
    -- 1-clk pulses (previous design) deadlocks when they fire on different
    -- cycles — err_handler's ST_WAIT_FRAME_DONE would never satisfy.
    signal s_frame_rise_seen_r : std_logic := '0';
    signal s_frame_fall_seen_r : std_logic := '0';
    signal s_frame_done_both   : std_logic;

    -- =========================================================================
    -- MUX signals: err_handler / csr_chip -> cmd_arb
    -- =========================================================================
    signal s_cmd_reg_read_mux         : std_logic;
    signal s_cmd_reg_write_mux        : std_logic;
    signal s_cmd_reg_addr_mux         : std_logic_vector(3 downto 0);
    signal s_cmd_reg_chip_address_mux : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Per-chip: sk_raw output (before err_fill gating)
    -- =========================================================================
    signal s_sk_raw_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_sk_raw_tdata  : t_slv32_array;
    signal s_sk_raw_tuser  : t_slv8_array;

    -- CDC FIFO: chip_ctrl (TDC clock) -> decode_pipe (AXI-Stream clock)
    signal s_raw_cdc_full  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_cdc_empty : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Round 6 A1: per-chip reset stretcher for u_raw_cdc flush.
    -- xpm_fifo_async.rst requires a multi-cycle hold; soft_reset pulses are
    -- 1-clk so they must be extended. Counter stretches the pulse to
    -- c_RAW_FIFO_RST_CLKS cycles before deassert. Covers both the global
    -- soft_reset (s_cmd_soft_reset_tdc) and per-chip err recovery
    -- (s_err_cmd_soft_reset_tdc(i)).
    -- Round 7 B-1: widened from 8 to 16 TDC cycles so the reset holds ≥ 5
    -- cycles of the slowest expected AXI clock (even if it drops below the
    -- current 150 MHz). XPM async FIFO requires rst hold ≥ 5 × slowest-clk.
    constant c_RAW_FIFO_RST_CLKS : natural := 16;
    type t_raw_fifo_rst_cnt_arr is array (0 to c_N_CHIPS - 1)
        of unsigned(4 downto 0);
    signal s_raw_fifo_rst_cnt_r : t_raw_fifo_rst_cnt_arr := (others => (others => '0'));
    signal s_raw_fifo_rst       : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- CDC signals: AXI-Stream -> TDC domain (command pulses)
    -- =========================================================================
    -- Common command pulses (CDC'd once, shared across all chip_ctrl)
    signal s_cmd_start_tdc       : std_logic;
    signal s_cmd_stop_tdc        : std_logic;
    signal s_cmd_soft_reset_tdc  : std_logic;
    signal s_cmd_force_reinit_tdc : std_logic;  -- Round 12 A1
    signal s_err_force_reinit    : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 A1: per-chip force-reinit sticky
    signal s_err_raw_ctrl_drop   : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 A2: per-chip control-beat drop sticky
    signal s_err_drain_mismatch  : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 A4: per-chip drain mismatch sticky
    -- Round 12 A3 (extended): drain_mismatch also CDC'd
    signal s_err_drain_mismatch_meta_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_drain_mismatch_axi_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- Round 12 A5: cmd_arb + chip_reg R+W ambiguity stickies. cmd_arb is in
    -- AXI-Stream domain; chip_reg is in TDC domain (one per chip).
    signal s_err_rw_ambiguous_arb : std_logic;  -- AXI-Stream domain (cmd_arb)
    signal s_err_rw_ambiguous_reg : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- TDC domain (chip_reg per chip)
    signal s_err_rw_ambiguous_reg_meta_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_rw_ambiguous_reg_axi_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- Round 12 B8: per-chip stopdis_override mid-shot sticky (TDC domain)
    signal s_err_stopdis_mid_shot : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_stopdis_mid_shot_meta_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_stopdis_mid_shot_axi_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- Round 12 #20: per-chip collision vector (4 bits each)
    type t_collision_vec_arr is array(0 to c_N_CHIPS - 1) of std_logic_vector(3 downto 0);
    signal s_cmd_collision_vec_per : t_collision_vec_arr;
    -- Aggregated across chips: [0]=any-chip-start-collision, [1]=cfg_write,
    -- [2]=reg_read, [3]=reg_write. OR-across-chip combinational reduction.
    signal s_cmd_collision_vec_comb   : std_logic_vector(3 downto 0);
    signal s_cmd_collision_vec_meta_r : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cmd_collision_vec_axi_r  : std_logic_vector(3 downto 0) := (others => '0');

    -- Round 12 A3: 2-FF sync for TDC-domain status signals before crossing
    -- to AXI-Stream domain. These are OR-accumulated stickies / latched
    -- cause fields — per-bit metastability is the only concern (no
    -- atomic-bundle requirement because bits are independent or the
    -- latch source itself holds the value stable). ASYNC_REG attribute
    -- marks both stages as a known synchronizer.
    signal s_run_timeout_cause_meta_r  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_run_timeout_cause_axi_r   : std_logic_vector(2 downto 0) := (others => '0');
    -- Round 12 #17: packed 12-bit per-chip cause (chip i at [3*i+2:3*i])
    signal s_run_timeout_cause_packed   : std_logic_vector(3 * c_N_CHIPS - 1 downto 0);
    signal s_run_timeout_cause_arr_meta_r : std_logic_vector(3 * c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_run_timeout_cause_arr_axi_r  : std_logic_vector(3 * c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_init_cfg_coalesced_meta_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_init_cfg_coalesced_axi_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_cmd_collision_meta_r      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_cmd_collision_axi_r       : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_force_reinit_meta_r   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_force_reinit_axi_r    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_raw_ctrl_drop_meta_r  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_raw_ctrl_drop_axi_r   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- ASYNC_REG attribute declaration (scoped to this section; the later
    -- declaration at the cfg CDC section is for a separate set of signals).
    attribute ASYNC_REG : string;
    attribute ASYNC_REG of s_run_timeout_cause_meta_r  : signal is "TRUE";
    attribute ASYNC_REG of s_run_timeout_cause_axi_r   : signal is "TRUE";
    attribute ASYNC_REG of s_init_cfg_coalesced_meta_r : signal is "TRUE";
    attribute ASYNC_REG of s_init_cfg_coalesced_axi_r  : signal is "TRUE";
    attribute ASYNC_REG of s_cmd_collision_meta_r      : signal is "TRUE";
    attribute ASYNC_REG of s_cmd_collision_axi_r       : signal is "TRUE";
    attribute ASYNC_REG of s_err_force_reinit_meta_r   : signal is "TRUE";
    attribute ASYNC_REG of s_err_force_reinit_axi_r    : signal is "TRUE";
    attribute ASYNC_REG of s_err_raw_ctrl_drop_meta_r  : signal is "TRUE";
    attribute ASYNC_REG of s_err_raw_ctrl_drop_axi_r   : signal is "TRUE";
    attribute ASYNC_REG of s_err_drain_mismatch_meta_r : signal is "TRUE";
    attribute ASYNC_REG of s_err_drain_mismatch_axi_r  : signal is "TRUE";
    attribute ASYNC_REG of s_err_rw_ambiguous_reg_meta_r : signal is "TRUE";
    attribute ASYNC_REG of s_err_rw_ambiguous_reg_axi_r  : signal is "TRUE";
    attribute ASYNC_REG of s_err_stopdis_mid_shot_meta_r : signal is "TRUE";
    attribute ASYNC_REG of s_err_stopdis_mid_shot_axi_r  : signal is "TRUE";
    attribute ASYNC_REG of s_cmd_collision_vec_meta_r    : signal is "TRUE";
    attribute ASYNC_REG of s_cmd_collision_vec_axi_r     : signal is "TRUE";
    attribute ASYNC_REG of s_run_timeout_cause_arr_meta_r : signal is "TRUE";
    attribute ASYNC_REG of s_run_timeout_cause_arr_axi_r  : signal is "TRUE";
    signal s_cmd_cfg_write_g_tdc : std_logic;
    signal s_stop_tdc_tdc        : std_logic;  -- stop_tdc after xpm_cdc_pulse (#13)
    -- Round 7 B-5: i_err_soft_clear CDC'd to TDC domain so chip_reg can
    -- follow the same sticky-clear policy as status_agg / err_handler.
    signal s_soft_clear_tdc      : std_logic;
    -- Per-chip command pulses
    signal s_err_cmd_soft_reset_tdc : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_start_tdc         : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_read_tdc       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_write_tdc      : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- CDC signals: AXI-Stream -> TDC domain (quasi-static config snapshot)
    -- =========================================================================
    signal s_cfg_tdc             : t_tdc_cfg;
    signal s_cfg_image_tdc       : t_cfg_image;
    signal s_expected_ififo1_tdc : t_expected_array;
    signal s_expected_ififo2_tdc : t_expected_array;
    signal s_cmd_reg_addr_tdc    : std_logic_vector(3 downto 0);
    signal s_cmd_reg_wdata_tdc   : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);

    -- =========================================================================
    -- CDC signals: TDC -> AXI-Stream domain (status)
    -- =========================================================================
    -- Per-chip level status (xpm_cdc_single)
    signal s_chip_busy_axi         : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_drain_timeout_axi : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_sequence_axi      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_rsp_mismatch_axi  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_raw_overflow_axi  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 12 #15: distinct cause stickies (CDC'd)
    signal s_err_raw_drop_axi      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_drain_cap_axi     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 6 B1: additional per-chip observability in AXI-Stream domain
    signal s_err_reg_overflow_axi  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_run_drain_complete_axi : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_errflag_sync_axi      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Per-chip pulse status (xpm_cdc_pulse)
    signal s_cmd_reg_done_axi      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_rvalid_axi    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_run_timeout_axi       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Per-chip multi-bit status (xpm_cdc_gray for shot_seq, snapshot for rdata)
    signal s_chip_shot_seq_axi     : t_shot_seq_array;
    signal s_chip_shot_seq_axi_slv : t_slv16_array;  -- intermediate for xpm_cdc_gray output
    signal s_cmd_reg_rdata_axi     : t_slv28_array;

    -- =========================================================================
    -- TDC-domain reset (Round 5 #8):
    -- AXI reset (i_axis_aresetn) is async to i_tdc_clk. Without a sync stage,
    -- reset deassert could release TDC-domain registers metastably / non-
    -- uniformly. xpm_cdc_async_rst provides async-assert + synchronous-deassert
    -- in the TDC domain, which is the standard pattern for cross-domain reset.
    -- =========================================================================
    signal s_tdc_aresetn : std_logic;

    -- =========================================================================
    -- 2-FF synchronizer stages for quasi-static config bundles (Round 5 #7).
    -- Each bundle gets a dedicated metastability slot before the final _tdc
    -- register. ASYNC_REG attributes mark both stages for Vivado so the tool
    -- does not optimize/relocate them and recognises the path as a known
    -- CDC synchronizer. Atomicity of multi-bit payloads still relies on the
    -- documented contract (SW holds these stable for several i_tdc_clk cycles
    -- before issuing the related command pulse); a full xpm_cdc_handshake
    -- refactor is tracked as a follow-up if atomicity ever fails in test.
    --
    -- Round 11 item 9: explicit atomicity contract.
    --   The per-bit metastability guarantee is bounded by the ASYNC_REG
    --   attribute + 2 register stages. Per-bundle atomicity however is a
    --   SOFTWARE RESPONSIBILITY:
    --     1. SW writes CFG register fields in the AXI-Lite domain.
    --     2. SW MUST wait long enough for the new values to propagate
    --        through the 2-FF sync (> 3 i_tdc_clk periods) before issuing
    --        any command pulse (cmd_start, cmd_cfg_write, cmd_reg_rw, ...).
    --     3. Commands cross via xpm_cdc_pulse (~4 dest cycles on top of the
    --        sync); by then the bundle has already settled.
    --   If SW violates this contract (writes CFG + pulses command in the
    --   same AXI-Lite transaction), some fields may be sampled old and
    --   others new on the TDC side. Run-critical fields that tolerate
    --   this skew poorly (geometry: stops_per_chip, cols_per_face,
    --   n_faces, active_chip_mask, max_hits_cfg) are additionally latched
    --   at packet_start inside face_seq (see s_cfg_face_r), giving a
    --   second settle window before they are consumed.
    -- =========================================================================
    -- Round 11 item 9: atomic CDC handshake for cfg + cfg_image. 2-FF
    -- sync signals are removed — the handshake flattens the bundle into
    -- a fixed-width vector and guarantees atomic delivery on the dest
    -- clock (matches the existing Round 9 #5 cmd_reg handshake pattern).
    --
    -- Two independent handshakes (one per bundle) because the two change
    -- rates differ: cfg is AXI-Lite register writes; cfg_image is raw
    -- register overrides. Combining them would couple unrelated SW
    -- updates into a single transfer slot.
    signal s_cfg_src_packed   : std_logic_vector(c_TDC_CFG_BITS - 1 downto 0);
    signal s_cfg_dst_packed   : std_logic_vector(c_TDC_CFG_BITS - 1 downto 0) := (others => '0');
    signal s_cfg_src_send_r   : std_logic := '0';
    signal s_cfg_src_rcv      : std_logic;
    signal s_cfg_dst_req      : std_logic;
    signal s_cfg_d1_r         : std_logic_vector(c_TDC_CFG_BITS - 1 downto 0) := (others => '1');

    -- cfg_image is an array of 8 × 32-bit words = 256 bits.
    constant c_CFG_IMAGE_BITS : natural := 8 * 32;
    signal s_cfg_image_src_packed : std_logic_vector(c_CFG_IMAGE_BITS - 1 downto 0);
    signal s_cfg_image_dst_packed : std_logic_vector(c_CFG_IMAGE_BITS - 1 downto 0) := (others => '0');
    signal s_cfg_image_src_send_r : std_logic := '0';
    signal s_cfg_image_src_rcv    : std_logic;
    signal s_cfg_image_dst_req    : std_logic;
    signal s_cfg_image_d1_r       : std_logic_vector(c_CFG_IMAGE_BITS - 1 downto 0) := (others => '1');

    -- ASYNC_REG already declared above (Round 12 A3 sync signals).

    -- Round 9 #5: cmd_reg bundle moved from 2-FF sync to xpm_cdc_handshake.
    -- Per-reg-op the bundle changes every time SW issues a read/write, so
    -- bit-skew risk is much higher than for quasi-static cfg/cfg_image.
    -- 32-bit handshake packs addr[3:0] + wdata[27:0].
    -- (cfg / cfg_image remain on 2-FF + ASYNC_REG — atomicity there is
    --  guaranteed by SW holding the payload stable for several cycles
    --  before the cfg_write / cmd_start pulse; the race window closed by
    --  the command pulse's own ~4-cycle CDC latency.)
    signal s_cmd_reg_src_packed : std_logic_vector(31 downto 0);
    signal s_cmd_reg_dst_packed : std_logic_vector(31 downto 0) := (others => '0');
    signal s_cmd_reg_src_send_r : std_logic := '0';
    signal s_cmd_reg_src_rcv    : std_logic;
    signal s_cmd_reg_dst_req    : std_logic;
    signal s_cmd_reg_d1_r       : std_logic_vector(31 downto 0) := (others => '1');

    -- Round 6 A3: expected_ififo1/2 now transfer via xpm_cdc_handshake.
    -- Unlike cfg/cfg_image (quasi-static) these are running totals updated
    -- per stop event in AXI domain, so 2-FF sync would expose torn samples
    -- to chip_run's ST_DRAIN_LATCH. Handshake provides atomic 32-bit
    -- delivery per bundle; any change in the source retriggers a transfer.
    signal s_expected_ififo1_src_packed : std_logic_vector(31 downto 0);
    signal s_expected_ififo2_src_packed : std_logic_vector(31 downto 0);
    signal s_expected_ififo1_dst_packed : std_logic_vector(31 downto 0) := (others => '0');
    signal s_expected_ififo2_dst_packed : std_logic_vector(31 downto 0) := (others => '0');
    signal s_exp1_src_send_r : std_logic := '0';
    signal s_exp1_src_rcv    : std_logic;
    signal s_exp1_dst_req    : std_logic;
    signal s_exp1_d1_r       : std_logic_vector(31 downto 0) := (others => '1');
    signal s_exp2_src_send_r : std_logic := '0';
    signal s_exp2_src_rcv    : std_logic;
    signal s_exp2_dst_req    : std_logic;
    signal s_exp2_d1_r       : std_logic_vector(31 downto 0) := (others => '1');

begin

    -- =========================================================================
    -- TDC-domain reset synchronizer (Round 5 #8)
    -- =========================================================================
    u_tdc_rst_sync : xpm_cdc_async_rst
        generic map (
            DEST_SYNC_FF    => 4,
            INIT_SYNC_FF    => 0,
            RST_ACTIVE_HIGH => 0
        )
        port map (
            src_arst  => i_axis_aresetn,
            dest_clk  => i_tdc_clk,
            dest_arst => s_tdc_aresetn
        );

    -- =========================================================================
    -- frame_done latch for err_handler
    -- Rise and fall frame_done pulses may not coincide; latch each observation
    -- and hold until next shot_start. err_handler sees the AND, which becomes
    -- '1' for one cycle per shot after both slopes have reported.
    -- =========================================================================
    p_frame_done_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or i_shot_start_gated = '1' then
                s_frame_rise_seen_r <= '0';
                s_frame_fall_seen_r <= '0';
            else
                if i_frame_done = '1' then
                    s_frame_rise_seen_r <= '1';
                end if;
                if i_frame_fall_done = '1' then
                    s_frame_fall_seen_r <= '1';
                end if;
            end if;
        end if;
    end process;

    s_frame_done_both <= s_frame_rise_seen_r and s_frame_fall_seen_r;

    -- =========================================================================
    -- Config merging: pipeline fields + chip fields -> s_cfg_merged
    -- =========================================================================

    -- Pipeline fields (from i_cfg_pipeline)
    s_cfg_merged.active_chip_mask <= i_cfg_pipeline.active_chip_mask;
    s_cfg_merged.packet_scope     <= i_cfg_pipeline.packet_scope;
    s_cfg_merged.hit_store_mode   <= i_cfg_pipeline.hit_store_mode;
    s_cfg_merged.dist_scale       <= i_cfg_pipeline.dist_scale;
    s_cfg_merged.drain_mode       <= i_cfg_pipeline.drain_mode;
    s_cfg_merged.pipeline_en      <= i_cfg_pipeline.pipeline_en;
    s_cfg_merged.n_faces          <= i_cfg_pipeline.n_faces;
    s_cfg_merged.stops_per_chip   <= i_cfg_pipeline.stops_per_chip;
    s_cfg_merged.n_drain_cap      <= i_cfg_pipeline.n_drain_cap;
    s_cfg_merged.stopdis_override <= i_cfg_pipeline.stopdis_override;
    s_cfg_merged.max_range_clks   <= i_cfg_pipeline.max_range_clks;
    s_cfg_merged.cols_per_face    <= i_cfg_pipeline.cols_per_face;

    -- Chip fields (from csr_chip outputs)
    s_cfg_merged.bus_clk_div      <= s_bus_clk_div;
    s_cfg_merged.bus_ticks        <= s_bus_ticks;
    s_cfg_merged.start_off1       <= s_start_off1;
    s_cfg_merged.cfg_reg7         <= s_cfg_reg7;
    s_cfg_merged.max_scan_clks    <= s_max_scan_clks;
    s_cfg_merged.max_hits_cfg     <= s_max_hits_cfg;

    -- Drive merged config to output
    o_cfg <= s_cfg_merged;

    -- =========================================================================
    -- Pass-through outputs (CDC'd versions for TDC->AXI-Stream signals)
    -- =========================================================================
    o_cmd_start         <= i_cmd_start;
    o_reg_loop_resume   <= s_reg_loop_resume;
    o_chip_busy         <= s_chip_busy_axi;
    o_chip_shot_seq     <= s_chip_shot_seq_axi;
    o_errflag_sync      <= s_errflag_sync_axi;
    o_err_drain_timeout <= s_err_drain_timeout_axi;
    o_err_sequence      <= s_err_sequence_axi;
    o_reg_outstanding   <= s_reg_outstanding;
    o_err_rsp_mismatch  <= s_err_rsp_mismatch_axi;
    o_err_raw_overflow  <= s_err_raw_overflow_axi;
    o_run_timeout       <= s_run_timeout_axi;
    -- Round 11 C: most-recent timeout cause across all chips. Latched on any
    -- run_timeout pulse so SW always sees the latest non-stale value. TDC
    -- domain; consumers are quasi-static so a 2-FF sync at consumer side is
    -- sufficient (handled by the status CDC that carries STAT7).
    -- Round 12 #17: per-chip cause latch (keeps the last cause that fired
    -- on each chip, instead of the previous behavior that overwrote a
    -- single register with whichever chip iterated last).
    p_run_timeout_cause_per_chip : process(i_tdc_clk)
    begin
        if rising_edge(i_tdc_clk) then
            if s_tdc_aresetn = '0' then
                for i in 0 to c_N_CHIPS - 1 loop
                    s_run_timeout_cause_per_chip_r(i) <= (others => '0');
                end loop;
            else
                for i in 0 to c_N_CHIPS - 1 loop
                    if s_run_timeout(i) = '1' then
                        s_run_timeout_cause_per_chip_r(i) <= s_run_timeout_cause(i);
                    end if;
                end loop;
            end if;
        end if;
    end process p_run_timeout_cause_per_chip;

    p_run_timeout_cause_latch : process(i_tdc_clk)
    begin
        if rising_edge(i_tdc_clk) then
            if s_tdc_aresetn = '0' then
                s_run_timeout_cause_last_r <= (others => '0');
            else
                for i in 0 to c_N_CHIPS - 1 loop
                    if s_run_timeout(i) = '1' then
                        s_run_timeout_cause_last_r <= s_run_timeout_cause(i);
                    end if;
                end loop;
            end if;
        end if;
    end process p_run_timeout_cause_latch;
    -- Round 12 A3: 2-FF sync into AXI-Stream domain.
    p_tdc_to_axi_status_sync : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_run_timeout_cause_meta_r  <= (others => '0');
                s_run_timeout_cause_axi_r   <= (others => '0');
                s_init_cfg_coalesced_meta_r <= (others => '0');
                s_init_cfg_coalesced_axi_r  <= (others => '0');
                s_cmd_collision_meta_r      <= (others => '0');
                s_cmd_collision_axi_r       <= (others => '0');
                s_err_force_reinit_meta_r   <= (others => '0');
                s_err_force_reinit_axi_r    <= (others => '0');
                s_err_raw_ctrl_drop_meta_r  <= (others => '0');
                s_err_raw_ctrl_drop_axi_r   <= (others => '0');
                s_err_drain_mismatch_meta_r <= (others => '0');
                s_err_drain_mismatch_axi_r  <= (others => '0');
                s_err_rw_ambiguous_reg_meta_r <= (others => '0');
                s_err_rw_ambiguous_reg_axi_r  <= (others => '0');
                s_err_stopdis_mid_shot_meta_r <= (others => '0');
                s_err_stopdis_mid_shot_axi_r  <= (others => '0');
                s_cmd_collision_vec_meta_r    <= (others => '0');
                s_cmd_collision_vec_axi_r     <= (others => '0');
                s_run_timeout_cause_arr_meta_r <= (others => '0');
                s_run_timeout_cause_arr_axi_r  <= (others => '0');
            else
                s_run_timeout_cause_meta_r  <= s_run_timeout_cause_last_r;
                s_run_timeout_cause_axi_r   <= s_run_timeout_cause_meta_r;
                s_init_cfg_coalesced_meta_r <= s_init_cfg_coalesced;
                s_init_cfg_coalesced_axi_r  <= s_init_cfg_coalesced_meta_r;
                s_cmd_collision_meta_r      <= s_cmd_collision_mask;
                s_cmd_collision_axi_r       <= s_cmd_collision_meta_r;
                s_err_force_reinit_meta_r   <= s_err_force_reinit;
                s_err_force_reinit_axi_r    <= s_err_force_reinit_meta_r;
                s_err_raw_ctrl_drop_meta_r  <= s_err_raw_ctrl_drop;
                s_err_raw_ctrl_drop_axi_r   <= s_err_raw_ctrl_drop_meta_r;
                s_err_drain_mismatch_meta_r <= s_err_drain_mismatch;
                s_err_drain_mismatch_axi_r  <= s_err_drain_mismatch_meta_r;
                s_err_rw_ambiguous_reg_meta_r <= s_err_rw_ambiguous_reg;
                s_err_rw_ambiguous_reg_axi_r  <= s_err_rw_ambiguous_reg_meta_r;
                s_err_stopdis_mid_shot_meta_r <= s_err_stopdis_mid_shot;
                s_err_stopdis_mid_shot_axi_r  <= s_err_stopdis_mid_shot_meta_r;
                s_cmd_collision_vec_meta_r    <= s_cmd_collision_vec_comb;
                s_cmd_collision_vec_axi_r     <= s_cmd_collision_vec_meta_r;
                s_run_timeout_cause_arr_meta_r <= s_run_timeout_cause_packed;
                s_run_timeout_cause_arr_axi_r  <= s_run_timeout_cause_arr_meta_r;
            end if;
        end if;
    end process p_tdc_to_axi_status_sync;

    o_run_timeout_cause <= s_run_timeout_cause_axi_r;
    o_reg_arb_timeout   <= s_reg_arb_timeout;
    o_err_active        <= s_err_active;
    -- Round 6 B1: surface per-chip stickies to top
    o_err_reg_overflow    <= s_err_reg_overflow_axi;
    o_run_drain_complete  <= s_run_drain_complete_axi;
    -- Round 12 A3: all TDC-domain status signals now go through 2-FF sync
    -- above (p_tdc_to_axi_status_sync) before being driven on AXI-Stream
    -- domain outputs. Per-bit metastability protected; per-chip
    -- independence makes atomic-bundle unnecessary.
    o_init_cfg_coalesced_mask <= s_init_cfg_coalesced_axi_r;
    o_cmd_collision_mask      <= s_cmd_collision_axi_r;
    o_err_force_reinit_mask   <= s_err_force_reinit_axi_r;
    o_err_raw_ctrl_drop_mask  <= s_err_raw_ctrl_drop_axi_r;
    o_err_drain_mismatch_mask <= s_err_drain_mismatch_axi_r;
    o_err_rw_ambiguous_arb    <= s_err_rw_ambiguous_arb;  -- AXI-Stream domain, no CDC needed
    o_err_rw_ambiguous_reg    <= s_err_rw_ambiguous_reg_axi_r;
    o_err_stopdis_mid_shot_mask <= s_err_stopdis_mid_shot_axi_r;
    -- Round 12 #20: aggregate collision vec across chips (OR-reduction)
    -- then feed through CDC.
    s_cmd_collision_vec_comb <=
        s_cmd_collision_vec_per(0) or s_cmd_collision_vec_per(1) or
        s_cmd_collision_vec_per(2) or s_cmd_collision_vec_per(3);
    o_err_cmd_collision_vec <= s_cmd_collision_vec_axi_r;
    -- Round 12 #15: distinct cause masks.
    o_err_raw_drop_mask     <= s_err_raw_drop_axi;
    o_err_drain_cap_mask    <= s_err_drain_cap_axi;
    -- Round 12 #17: pack per-chip cause into one vector then feed CDC output.
    gen_cause_pack : for i in 0 to c_N_CHIPS - 1 generate
        s_run_timeout_cause_packed(3*i + 2 downto 3*i) <= s_run_timeout_cause_per_chip_r(i);
    end generate;
    o_run_timeout_cause_per_chip <= s_run_timeout_cause_arr_axi_r;

    -- =========================================================================
    -- MUX: when err_handler is active, it owns the cmd_arb reg-access path
    --
    -- Policy (#38, documented intent):
    --   While s_err_active='1', SW-initiated reg read/write requests are
    --   blocked (read muxed to err_handler's address, write forced to '0').
    --   This is intentional — err_handler must own the bus during its
    --   Reg12 read classification sequence. Without bounded recovery,
    --   manual SW reg access could be starved indefinitely.
    --
    --   Starvation is bounded by:
    --     - err_handler.ST_WAIT_READ watchdog (Round 2 #2) — forces
    --       recovery progression even if reg read never completes.
    --     - i_soft_clear (Round 3 #10 / Round 4 #40) — lets SW clear
    --       fatal state and restore the mux to SW ownership without a
    --       hard reset.
    --   SW should check o_err_active before issuing reg commands and
    --   retry after recovery, or use i_soft_clear for emergency access.
    -- =========================================================================
    s_cmd_reg_read_mux         <= s_err_cmd_reg_read      when s_err_active = '1'
                                  else s_cmd_reg_read;
    s_cmd_reg_addr_mux         <= s_err_cmd_reg_addr      when s_err_active = '1'
                                  else s_cmd_reg_addr;
    s_cmd_reg_chip_address_mux <= s_err_cmd_reg_chip_addr when s_err_active = '1'
                                  else s_cmd_reg_chip_address;
    s_cmd_reg_write_mux        <= '0' when s_err_active = '1'
                                  else s_cmd_reg_write;

    -- =========================================================================
    -- Per-chip err_fill gating on raw AXI-Stream output
    -- =========================================================================
    gen_err_fill : for i in 0 to c_N_CHIPS - 1 generate
        -- Error-fill: replace ONLY hit[16:0] with all-ones (0x1FFFF).
        -- Preserve upper raw bits (stop_id, slope, cha_code, ififo_id)
        -- so downstream decode/cell_builder see correct stop/slope routing.
        o_raw_sk_tdata(i)(16 downto 0)  <= s_sk_raw_tdata(i)(16 downto 0) when s_err_fill(i) = '0'
                                           else (others => '1');
        o_raw_sk_tdata(i)(31 downto 17) <= s_sk_raw_tdata(i)(31 downto 17);  -- always pass through
        o_raw_sk_tvalid(i) <= s_sk_raw_tvalid(i);
        o_raw_sk_tuser(i)  <= s_sk_raw_tuser(i);
    end generate gen_err_fill;

    -- =========================================================================
    -- s_cmd_reg_wdata: cfg_image indexed by target register address
    -- =========================================================================
    s_cmd_reg_wdata <= o_cfg_image(to_integer(unsigned(s_cmd_reg_addr_mux)))(c_TDC_BUS_WIDTH - 1 downto 0);

    -- =========================================================================
    -- [1] csr_chip: AXI-Lite CSR + SRM + CDC
    -- =========================================================================
    u_csr_chip : entity work.tdc_gpx_csr_chip
        port map (
            s_axi_aclk      => s_axi_aclk,
            s_axi_aresetn   => s_axi_aresetn,
            s_axi_awvalid   => s_axi_awvalid,
            s_axi_awready   => s_axi_awready,
            s_axi_awaddr    => s_axi_awaddr,
            s_axi_awprot    => s_axi_awprot,
            s_axi_wvalid    => s_axi_wvalid,
            s_axi_wready    => s_axi_wready,
            s_axi_wdata     => s_axi_wdata,
            s_axi_wstrb     => s_axi_wstrb,
            s_axi_bvalid    => s_axi_bvalid,
            s_axi_bready    => s_axi_bready,
            s_axi_bresp     => s_axi_bresp,
            s_axi_arvalid   => s_axi_arvalid,
            s_axi_arready   => s_axi_arready,
            s_axi_araddr    => s_axi_araddr,
            s_axi_arprot    => s_axi_arprot,
            s_axi_rvalid    => s_axi_rvalid,
            s_axi_rready    => s_axi_rready,
            s_axi_rdata     => s_axi_rdata,
            s_axi_rresp     => s_axi_rresp,
            i_axis_aclk     => i_axis_aclk,
            i_axis_aresetn  => i_axis_aresetn,
            o_cfg_image     => s_cfg_image_raw,
            o_bus_clk_div   => s_bus_clk_div,
            o_bus_ticks     => s_bus_ticks,
            o_start_off1    => s_start_off1,
            o_cfg_reg7      => s_cfg_reg7,
            o_max_scan_clks => s_max_scan_clks,
            o_max_hits_cfg  => s_max_hits_cfg,
            o_cmd_reg_read  => s_cmd_reg_read,
            o_cmd_reg_write => s_cmd_reg_write,
            o_cmd_reg_addr  => s_cmd_reg_addr,
            o_cmd_reg_chip  => s_cmd_reg_chip,
            i_cmd_reg_rdata_0    => s_cmd_reg_rdata_axi(0),
            i_cmd_reg_rdata_1    => s_cmd_reg_rdata_axi(1),
            i_cmd_reg_rdata_2    => s_cmd_reg_rdata_axi(2),
            i_cmd_reg_rdata_3    => s_cmd_reg_rdata_axi(3),
            i_cmd_reg_rvalid     => s_cmd_reg_rvalid_axi,
            i_cmd_reg_done_pulse => s_cmd_reg_done_pulse,
            i_cmd_reg_addr_done  => s_cmd_reg_addr_out,
            o_cmd_reg_chip_address  => s_cmd_reg_chip_address,
            o_cdc_idle      => o_cdc_idle,
            o_irq           => o_irq
        );

    -- =========================================================================
    -- [2] cmd_arb: command arbitration
    -- =========================================================================
    u_cmd_arb : entity work.tdc_gpx_cmd_arb
        port map (
            i_clk                => i_axis_aclk,
            i_rst_n              => i_axis_aresetn,
            i_cmd_start          => i_cmd_start,
            i_cmd_start_accepted => i_cmd_start_accepted,
            i_cmd_stop           => i_cmd_stop,
            i_cmd_soft_reset     => i_cmd_soft_reset,
            i_cmd_cfg_write      => i_cmd_cfg_write,
            i_cmd_reg_read       => s_cmd_reg_read_mux,
            i_cmd_reg_write      => s_cmd_reg_write_mux,
            i_cmd_reg_chip       => s_cmd_reg_chip,
            i_cmd_reg_chip_address  => s_cmd_reg_chip_address_mux,
            i_cmd_reg_addr       => s_cmd_reg_addr_mux,
            i_chip_busy          => s_chip_busy_axi,
            i_face_asm_idle      => i_face_asm_idle,
            i_face_asm_fall_idle => i_face_asm_fall_idle,
            i_hdr_idle           => i_hdr_idle,
            i_hdr_fall_idle      => i_hdr_fall_idle,
            i_cmd_reg_done       => s_cmd_reg_done_axi,
            o_cmd_cfg_write_g    => o_cmd_cfg_write_g,
            o_cmd_reg_read_g     => s_cmd_reg_read_g,
            o_cmd_reg_write_g    => s_cmd_reg_write_g,
            o_reg_outstanding    => s_reg_outstanding,
            o_outstanding_chip   => open,
            o_cmd_reg_done_pulse => s_cmd_reg_done_pulse,
            o_cmd_reg_done_chip  => s_cmd_reg_done_chip,
            o_reg_loop_resume    => s_reg_loop_resume,
            o_cmd_reg_addr_out   => s_cmd_reg_addr_out,
            o_reg_timeout        => s_reg_arb_timeout,
            o_reg_timeout_mask   => o_reg_timeout_mask,  -- Round 11 C: surface to top
            o_reg_rejected       => o_reg_rejected,  -- surfaced to top via config_ctrl port
            o_reg_zero_mask      => o_reg_zero_mask,  -- surfaced to top via config_ctrl port
            o_err_rw_ambiguous   => s_err_rw_ambiguous_arb  -- Round 12 A5
        );

    -- =========================================================================
    -- [2b] err_handler: automatic ErrFlag detection and recovery
    -- =========================================================================
    u_err_handler : entity work.tdc_gpx_err_handler
        port map (
            i_clk                => i_axis_aclk,
            i_rst_n              => i_axis_aresetn,
            i_errflag_sync       => s_errflag_sync_axi,
            i_chip_busy          => s_chip_busy_axi,
            i_reg11_data_0       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata_axi(0),
            i_reg11_data_1       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata_axi(1),
            i_reg11_data_2       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata_axi(2),
            i_reg11_data_3       => (31 downto c_TDC_BUS_WIDTH => '0') & s_cmd_reg_rdata_axi(3),
            i_cmd_reg_done_pulse => s_cmd_reg_done_pulse,
            i_cmd_reg_rvalid    => s_cmd_reg_rvalid_axi,
            i_reg_outstanding    => s_reg_outstanding,
            i_frame_done         => s_frame_done_both,
            i_shot_start         => i_shot_start_gated,
            i_soft_clear         => i_err_soft_clear,
            o_cmd_soft_reset     => s_err_cmd_soft_reset,
            o_cmd_reg_read       => s_err_cmd_reg_read,
            o_cmd_reg_addr       => s_err_cmd_reg_addr,
            o_cmd_reg_chip_addr  => s_err_cmd_reg_chip_addr,
            o_err_fill           => s_err_fill,
            o_err_active         => s_err_active,
            o_err_chip_mask      => o_err_chip_mask,
            o_err_cause          => o_err_cause,
            o_err_fatal          => o_err_fatal,
            o_err_read_timeout   => o_err_read_timeout,  -- surfaced to top via config_ctrl port
            o_err_frame_wait_escape => o_err_frame_wait_escape  -- Round 11 item 11
        );

    -- =========================================================================
    -- [3] stop_decode: stop event decode + cfg_image override
    -- =========================================================================
    u_stop_decode : entity work.tdc_gpx_stop_cfg_decode
        generic map (g_STOP_EVT_DWIDTH => g_STOP_EVT_DWIDTH)
        port map (
            i_clk              => i_axis_aclk,
            i_rst_n            => i_axis_aresetn,
            i_stop_evt_tvalid  => i_stop_evt_tvalid,
            i_stop_evt_tdata   => i_stop_evt_tdata,
            i_stop_evt_tuser   => i_stop_evt_tuser,
            o_stop_evt_tready  => o_stop_evt_tready,
            i_shot_start_gated => i_shot_start_gated,
            o_expected_ififo1  => s_expected_ififo1,
            o_expected_ififo2  => s_expected_ififo2,
            i_cfg              => s_cfg_merged,
            i_cfg_image_raw    => s_cfg_image_raw,
            o_cfg_image        => o_cfg_image,
            o_monotonic_violation_mask => o_mono_violation_mask,
            o_orphan_stop_evt_sticky   => o_orphan_stop_evt_sticky
        );

    -- =========================================================================
    -- CDC Stage 1: Command pulses AXI-Stream -> TDC (common, one instance each)
    -- =========================================================================

    u_cdc_cmd_start : xpm_cdc_pulse
        generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => '0',
            src_pulse  => i_cmd_start_accepted,
            dest_clk   => i_tdc_clk,
            dest_rst   => '0',
            dest_pulse => s_cmd_start_tdc
        );

    u_cdc_cmd_stop : xpm_cdc_pulse
        generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => '0',
            src_pulse  => i_cmd_stop,
            dest_clk   => i_tdc_clk,
            dest_rst   => '0',
            dest_pulse => s_cmd_stop_tdc
        );

    u_cdc_cmd_soft_reset : xpm_cdc_pulse
        generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => '0',
            src_pulse  => i_cmd_soft_reset,
            dest_clk   => i_tdc_clk,
            dest_rst   => '0',
            dest_pulse => s_cmd_soft_reset_tdc
        );

    -- Round 12 A1: force-reinit CDC (global, fan-out to all 4 chip_ctrl).
    u_cdc_cmd_force_reinit : xpm_cdc_pulse
        generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => '0',
            src_pulse  => i_cmd_force_reinit,
            dest_clk   => i_tdc_clk,
            dest_rst   => '0',
            dest_pulse => s_cmd_force_reinit_tdc
        );

    u_cdc_cmd_cfg_write : xpm_cdc_pulse
        generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => '0',
            src_pulse  => o_cmd_cfg_write_g,
            dest_clk   => i_tdc_clk,
            dest_rst   => '0',
            dest_pulse => s_cmd_cfg_write_g_tdc
        );

    -- #13: stop_tdc crosses AXI-Stream → TDC clock. Previously a bare 2-FF
    -- synchronizer in chip_ctrl; now routed through xpm_cdc_pulse for
    -- proper short-pulse capture (DEST_SYNC_FF=4 for higher MTBF).
    u_cdc_stop_tdc : xpm_cdc_pulse
        generic map (DEST_SYNC_FF => 4, RST_USED => 0, SIM_ASSERT_CHK => 0)
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => '0',
            src_pulse  => i_stop_tdc,
            dest_clk   => i_tdc_clk,
            dest_rst   => '0',
            dest_pulse => s_stop_tdc_tdc
        );

    -- Round 7 B-5: i_err_soft_clear crosses AXI-Stream → TDC clock so all
    -- chip_reg instances can clear their s_err_req_overflow_r sticky on
    -- SW-initiated soft_clear, matching the status_agg / err_handler policy.
    u_cdc_soft_clear : xpm_cdc_pulse
        generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => '0',
            src_pulse  => i_err_soft_clear,
            dest_clk   => i_tdc_clk,
            dest_rst   => '0',
            dest_pulse => s_soft_clear_tdc
        );

    -- =========================================================================
    -- CDC Stage 3: Atomic config snapshot (AXI-Stream -> TDC) — Round 11 item 9
    --
    -- Replaces the Round 5 #7 2-FF bundle sync. Each bundle crosses via its
    -- own xpm_cdc_handshake (same pattern as Round 9 #5 cmd_reg). Change
    -- detection on the source side triggers src_send; the handshake
    -- guarantees the destination sees a coherent snapshot with no torn
    -- fields — per-bit metastability AND per-bundle atomicity are now
    -- structurally protected.
    --
    -- SW timing contract is relaxed: writing CFG + pulsing a command in
    -- the same AXI-Lite transaction is now safe (previously the 2-FF sync
    -- could torn-sample a mid-update bundle).
    -- =========================================================================

    -- Pack t_tdc_cfg into the handshake payload.
    s_cfg_src_packed <= fn_pack_tdc_cfg(s_cfg_merged);

    -- Pack cfg_image (8 × 32-bit) into the handshake payload.
    gen_cfg_image_pack : for i in 0 to 7 generate
        s_cfg_image_src_packed(32 * (i + 1) - 1 downto 32 * i) <= o_cfg_image(i);
    end generate;

    u_cdc_cfg : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 0,
            SRC_SYNC_FF    => 4,
            WIDTH          => c_TDC_CFG_BITS
        )
        port map (
            src_clk  => i_axis_aclk,
            src_in   => s_cfg_src_packed,
            src_send => s_cfg_src_send_r,
            src_rcv  => s_cfg_src_rcv,
            dest_clk => i_tdc_clk,
            dest_req => s_cfg_dst_req,
            dest_ack => s_cfg_dst_req,
            dest_out => s_cfg_dst_packed
        );

    u_cdc_cfg_image : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 0,
            SRC_SYNC_FF    => 4,
            WIDTH          => c_CFG_IMAGE_BITS
        )
        port map (
            src_clk  => i_axis_aclk,
            src_in   => s_cfg_image_src_packed,
            src_send => s_cfg_image_src_send_r,
            src_rcv  => s_cfg_image_src_rcv,
            dest_clk => i_tdc_clk,
            dest_req => s_cfg_image_dst_req,
            dest_ack => s_cfg_image_dst_req,
            dest_out => s_cfg_image_dst_packed
        );

    -- Source-side send control: trigger new handshake on payload change.
    p_cfg_send : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_cfg_src_send_r <= '0';
                s_cfg_d1_r       <= (others => '1');
            else
                if s_cfg_src_send_r = '0' and s_cfg_src_packed /= s_cfg_d1_r then
                    s_cfg_src_send_r <= '1';
                    s_cfg_d1_r       <= s_cfg_src_packed;
                elsif s_cfg_src_rcv = '1' then
                    s_cfg_src_send_r <= '0';
                end if;
            end if;
        end if;
    end process p_cfg_send;

    p_cfg_image_send : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_cfg_image_src_send_r <= '0';
                s_cfg_image_d1_r       <= (others => '1');
            else
                if s_cfg_image_src_send_r = '0'
                    and s_cfg_image_src_packed /= s_cfg_image_d1_r then
                    s_cfg_image_src_send_r <= '1';
                    s_cfg_image_d1_r       <= s_cfg_image_src_packed;
                elsif s_cfg_image_src_rcv = '1' then
                    s_cfg_image_src_send_r <= '0';
                end if;
            end if;
        end if;
    end process p_cfg_image_send;

    -- Unpack destination-side handshake output into the TDC-domain
    -- record and array. These are combinational (the handshake already
    -- presents a registered, stable output on dest_out).
    s_cfg_tdc <= fn_unpack_tdc_cfg(s_cfg_dst_packed);

    gen_cfg_image_unpack : for i in 0 to 7 generate
        s_cfg_image_tdc(i) <= s_cfg_image_dst_packed(32 * (i + 1) - 1 downto 32 * i);
    end generate;

    -- =========================================================================
    -- Round 9 #5: cmd_reg bundle atomic transfer via xpm_cdc_handshake.
    -- cmd_reg_addr + cmd_reg_wdata change per reg-op (unlike quasi-static
    -- cfg / cfg_image), so a torn sample would dispatch the wrong register
    -- or wrong data. Single 32-bit handshake packs both.
    -- =========================================================================
    s_cmd_reg_src_packed(3 downto 0)  <= s_cmd_reg_addr_mux;
    s_cmd_reg_src_packed(31 downto 4) <= s_cmd_reg_wdata;
    s_cmd_reg_addr_tdc  <= s_cmd_reg_dst_packed(3 downto 0);
    s_cmd_reg_wdata_tdc <= s_cmd_reg_dst_packed(31 downto 4);

    u_cdc_cmd_reg : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK => 1, DEST_SYNC_FF => 4, INIT_SYNC_FF => 0,
            SIM_ASSERT_CHK => 0, SRC_SYNC_FF => 4, WIDTH => 32
        )
        port map (
            src_clk  => i_axis_aclk,
            src_in   => s_cmd_reg_src_packed,
            src_send => s_cmd_reg_src_send_r,
            src_rcv  => s_cmd_reg_src_rcv,
            dest_clk => i_tdc_clk,
            dest_req => s_cmd_reg_dst_req,
            dest_ack => s_cmd_reg_dst_req,
            dest_out => s_cmd_reg_dst_packed
        );

    p_cmd_reg_send : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_cmd_reg_src_send_r <= '0';
                s_cmd_reg_d1_r       <= (others => '1');
            else
                if s_cmd_reg_src_send_r = '0' and s_cmd_reg_src_packed /= s_cmd_reg_d1_r then
                    s_cmd_reg_src_send_r <= '1';
                    s_cmd_reg_d1_r       <= s_cmd_reg_src_packed;
                elsif s_cmd_reg_src_rcv = '1' then
                    s_cmd_reg_src_send_r <= '0';
                end if;
            end if;
        end if;
    end process p_cmd_reg_send;

    -- =========================================================================
    -- Round 6 A3: expected_ififo1/2 atomic transfer via xpm_cdc_handshake
    -- =========================================================================
    -- Pack 4 per-chip 8-bit counts into 32 bits for the handshake.
    gen_exp_pack : for i in 0 to c_N_CHIPS - 1 generate
        s_expected_ififo1_src_packed(8 * (i + 1) - 1 downto 8 * i)
            <= std_logic_vector(s_expected_ififo1(i));
        s_expected_ififo2_src_packed(8 * (i + 1) - 1 downto 8 * i)
            <= std_logic_vector(s_expected_ififo2(i));
        s_expected_ififo1_tdc(i)
            <= unsigned(s_expected_ififo1_dst_packed(8 * (i + 1) - 1 downto 8 * i));
        s_expected_ififo2_tdc(i)
            <= unsigned(s_expected_ififo2_dst_packed(8 * (i + 1) - 1 downto 8 * i));
    end generate gen_exp_pack;

    u_cdc_exp1 : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK => 1, DEST_SYNC_FF => 4, INIT_SYNC_FF => 0,
            SIM_ASSERT_CHK => 0, SRC_SYNC_FF => 4, WIDTH => 32
        )
        port map (
            src_clk  => i_axis_aclk,
            src_in   => s_expected_ififo1_src_packed,
            src_send => s_exp1_src_send_r,
            src_rcv  => s_exp1_src_rcv,
            dest_clk => i_tdc_clk,
            dest_req => s_exp1_dst_req,
            dest_ack => s_exp1_dst_req,
            dest_out => s_expected_ififo1_dst_packed
        );

    u_cdc_exp2 : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK => 1, DEST_SYNC_FF => 4, INIT_SYNC_FF => 0,
            SIM_ASSERT_CHK => 0, SRC_SYNC_FF => 4, WIDTH => 32
        )
        port map (
            src_clk  => i_axis_aclk,
            src_in   => s_expected_ififo2_src_packed,
            src_send => s_exp2_src_send_r,
            src_rcv  => s_exp2_src_rcv,
            dest_clk => i_tdc_clk,
            dest_req => s_exp2_dst_req,
            dest_ack => s_exp2_dst_req,
            dest_out => s_expected_ififo2_dst_packed
        );

    p_exp_send : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_exp1_src_send_r <= '0';
                s_exp1_d1_r       <= (others => '1');
                s_exp2_src_send_r <= '0';
                s_exp2_d1_r       <= (others => '1');
            else
                if s_exp1_src_send_r = '0' and s_expected_ififo1_src_packed /= s_exp1_d1_r then
                    s_exp1_src_send_r <= '1';
                    s_exp1_d1_r       <= s_expected_ififo1_src_packed;
                elsif s_exp1_src_rcv = '1' then
                    s_exp1_src_send_r <= '0';
                end if;
                if s_exp2_src_send_r = '0' and s_expected_ififo2_src_packed /= s_exp2_d1_r then
                    s_exp2_src_send_r <= '1';
                    s_exp2_d1_r       <= s_expected_ififo2_src_packed;
                elsif s_exp2_src_rcv = '1' then
                    s_exp2_src_send_r <= '0';
                end if;
            end if;
        end if;
    end process p_exp_send;

    -- =========================================================================
    -- CDC Stage 2c: TDC -> AXI-Stream rdata snapshot
    -- rdata is stable when rvalid is asserted; rvalid is CDC'd via
    -- xpm_cdc_pulse, so rdata has settled by the time rvalid_axi arrives.
    -- =========================================================================
    p_axi_rdata_snapshot : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            for k in 0 to c_N_CHIPS - 1 loop
                s_cmd_reg_rdata_axi(k) <= s_cmd_reg_rdata(k);
            end loop;
        end if;
    end process p_axi_rdata_snapshot;

    -- =========================================================================
    -- [4-19] Per-chip pipeline (generate x4)
    --   bus_phy + sk_brsp + chip_ctrl + sk_raw + per-chip CDC
    -- =========================================================================
    gen_chip : for i in 0 to c_N_CHIPS - 1 generate

        -- ----- bus_phy: physical bus timing FSM + IOBUF + 2-FF sync -----
        u_bus_phy : entity work.tdc_gpx_bus_phy
            port map (
                i_clk           => i_tdc_clk,
                i_rst_n         => s_tdc_aresetn,
                i_tick_en       => s_tick_en(i),
                i_bus_ticks     => s_bus_ticks_snap(i),
                i_req_valid     => s_bus_req_valid(i),
                i_req_rw        => s_bus_req_rw(i),
                i_req_addr      => s_bus_req_addr(i),
                i_req_wdata     => s_bus_req_wdata(i),
                i_oen_permanent => s_bus_oen_perm(i),
                i_req_burst     => s_bus_req_burst(i),
                o_busy          => s_bus_busy(i),
                o_m_axis_tvalid => s_brsp_axis_tvalid(i),
                o_m_axis_tdata  => s_brsp_axis_tdata(i),
                o_m_axis_tkeep  => s_brsp_axis_tkeep(i),
                o_m_axis_tuser  => s_brsp_axis_tuser(i),
                i_m_axis_tready => s_brsp_axis_tready(i),
                o_adr           => o_tdc_adr(i),
                o_csn           => o_tdc_csn(i),
                o_rdn           => o_tdc_rdn(i),
                o_wrn           => o_tdc_wrn(i),
                o_oen           => o_tdc_oen(i),
                io_d            => io_tdc_d(i),
                i_ef1_pin       => i_tdc_ef1(i),
                i_ef2_pin       => i_tdc_ef2(i),
                i_lf1_pin       => i_tdc_lf1(i),
                i_lf2_pin       => i_tdc_lf2(i),
                i_irflag_pin    => i_tdc_irflag(i),
                i_errflag_pin   => i_tdc_errflag(i),
                o_ef1_sync      => s_ef1_sync(i),
                o_ef2_sync      => s_ef2_sync(i),
                o_lf1_sync      => s_lf1_sync(i),
                o_lf2_sync      => s_lf2_sync(i),
                o_irflag_sync   => s_irflag_sync(i),
                o_errflag_sync  => s_errflag_sync(i),
                o_rsp_pending   => s_bus_rsp_pending_raw(i)
            );

        -- ----- skid buffer: bus_phy -> chip_ctrl (32b tdata + 8b tuser = 40b) -----
        u_sk_brsp : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => 40)
            port map (
                i_clk     => i_tdc_clk,
                i_rst_n   => s_tdc_aresetn,
                -- Flush on any soft reset (CDC'd to TDC domain)
                i_flush   => s_cmd_soft_reset_tdc or s_err_cmd_soft_reset_tdc(i),
                i_s_valid => s_brsp_axis_tvalid(i),
                o_s_ready => s_brsp_axis_tready(i),
                i_s_data  => s_brsp_axis_tdata(i) & s_brsp_axis_tuser(i),
                o_m_valid => s_brsp_sk_tvalid(i),
                i_m_ready => s_brsp_sk_tready(i),
                o_m_data(39 downto 8) => s_brsp_sk_tdata(i),
                o_m_data(7 downto 0)  => s_brsp_sk_tuser(i)
            );

        -- Combined pending: bus_phy internal + brsp skid output
        s_bus_rsp_pending(i) <= s_bus_rsp_pending_raw(i) or s_brsp_sk_tvalid(i);

        -- =================================================================
        -- Per-chip CDC Stage 1: Command pulses AXI-Stream -> TDC
        -- =================================================================

        u_cdc_err_soft_reset : xpm_cdc_pulse
            generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
            port map (
                src_clk    => i_axis_aclk,
                src_rst    => '0',
                src_pulse  => s_err_cmd_soft_reset(i),
                dest_clk   => i_tdc_clk,
                dest_rst   => '0',
                dest_pulse => s_err_cmd_soft_reset_tdc(i)
            );

        u_cdc_shot_start : xpm_cdc_pulse
            generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
            port map (
                src_clk    => i_axis_aclk,
                src_rst    => '0',
                src_pulse  => i_shot_start_per_chip(i),
                dest_clk   => i_tdc_clk,
                dest_rst   => '0',
                dest_pulse => s_shot_start_tdc(i)
            );

        u_cdc_reg_read : xpm_cdc_pulse
            generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
            port map (
                src_clk    => i_axis_aclk,
                src_rst    => '0',
                src_pulse  => s_cmd_reg_read_g(i),
                dest_clk   => i_tdc_clk,
                dest_rst   => '0',
                dest_pulse => s_cmd_reg_read_tdc(i)
            );

        u_cdc_reg_write : xpm_cdc_pulse
            generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
            port map (
                src_clk    => i_axis_aclk,
                src_rst    => '0',
                src_pulse  => s_cmd_reg_write_g(i),
                dest_clk   => i_tdc_clk,
                dest_rst   => '0',
                dest_pulse => s_cmd_reg_write_tdc(i)
            );

        -- =================================================================
        -- Per-chip CDC Stage 2: Status TDC -> AXI-Stream
        -- =================================================================

        -- Level signals: xpm_cdc_single (1-bit, 2-FF synchronizer)
        u_cdc_busy : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_chip_busy(i),
                dest_clk => i_axis_aclk,
                dest_out => s_chip_busy_axi(i)
            );

        u_cdc_drain_timeout : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_err_drain_timeout(i),
                dest_clk => i_axis_aclk,
                dest_out => s_err_drain_timeout_axi(i)
            );

        u_cdc_seq_err : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_err_sequence(i),
                dest_clk => i_axis_aclk,
                dest_out => s_err_sequence_axi(i)
            );

        u_cdc_rsp_mismatch : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_err_rsp_mismatch(i),
                dest_clk => i_axis_aclk,
                dest_out => s_err_rsp_mismatch_axi(i)
            );

        u_cdc_raw_overflow : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_err_raw_overflow(i),
                dest_clk => i_axis_aclk,
                dest_out => s_err_raw_overflow_axi(i)
            );

        -- Round 12 #15: distinct cause CDCs so SW can read them separately.
        u_cdc_raw_drop : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_err_raw_drop(i),
                dest_clk => i_axis_aclk,
                dest_out => s_err_raw_drop_axi(i)
            );

        u_cdc_drain_cap : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_err_drain_cap(i),
                dest_clk => i_axis_aclk,
                dest_out => s_err_drain_cap_axi(i)
            );

        -- Round 6 B1: per-chip chip_reg 3rd-pulse overflow (level sticky)
        u_cdc_err_reg_overflow : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_err_reg_overflow(i),
                dest_clk => i_axis_aclk,
                dest_out => s_err_reg_overflow_axi(i)
            );

        -- Round 6 B1: chip_run internal drain-complete is a 1-clk pulse. Latch
        -- to a per-shot sticky in TDC domain, then CDC the level.
        -- Round 7 A-3: the sticky now clears on the next i_shot_start so SW
        -- polling sees "this shot's drain finished" rather than "at least one
        -- shot drain finished since reset" (which the Round 6 version could
        -- only ever report — permanently '1' after the first shot).
        p_run_drain_sticky : process(i_tdc_clk)
        begin
            if rising_edge(i_tdc_clk) then
                if s_tdc_aresetn = '0' then
                    s_run_drain_complete_sticky_r(i) <= '0';
                elsif s_shot_start_tdc(i) = '1' then
                    -- New shot starting: clear stale sticky so the next
                    -- drain_complete event is visible edge-to-edge.
                    s_run_drain_complete_sticky_r(i) <= '0';
                elsif s_run_drain_complete(i) = '1' then
                    s_run_drain_complete_sticky_r(i) <= '1';
                end if;
            end if;
        end process p_run_drain_sticky;

        u_cdc_run_drain_complete : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_run_drain_complete_sticky_r(i),
                dest_clk => i_axis_aclk,
                dest_out => s_run_drain_complete_axi(i)
            );

        u_cdc_errflag : xpm_cdc_single
            generic map (DEST_SYNC_FF => 2, SRC_INPUT_REG => 0)
            port map (
                src_clk  => i_tdc_clk,
                src_in   => s_errflag_sync(i),
                dest_clk => i_axis_aclk,
                dest_out => s_errflag_sync_axi(i)
            );

        -- Pulse signals: xpm_cdc_pulse (TDC -> AXI-Stream)
        u_cdc_reg_done : xpm_cdc_pulse
            generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
            port map (
                src_clk    => i_tdc_clk,
                src_rst    => '0',
                src_pulse  => s_cmd_reg_done(i),
                dest_clk   => i_axis_aclk,
                dest_rst   => '0',
                dest_pulse => s_cmd_reg_done_axi(i)
            );

        u_cdc_reg_rvalid : xpm_cdc_pulse
            generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
            port map (
                src_clk    => i_tdc_clk,
                src_rst    => '0',
                src_pulse  => s_cmd_reg_rvalid(i),
                dest_clk   => i_axis_aclk,
                dest_rst   => '0',
                dest_pulse => s_cmd_reg_rvalid_axi(i)
            );

        u_cdc_run_timeout : xpm_cdc_pulse
            generic map (DEST_SYNC_FF => 2, RST_USED => 0, SIM_ASSERT_CHK => 0)
            port map (
                src_clk    => i_tdc_clk,
                src_rst    => '0',
                src_pulse  => s_run_timeout(i),
                dest_clk   => i_axis_aclk,
                dest_rst   => '0',
                dest_pulse => s_run_timeout_axi(i)
            );

        -- Multi-bit counter: xpm_cdc_gray (16-bit shot sequence)
        u_cdc_shot_seq : xpm_cdc_gray
            generic map (DEST_SYNC_FF => 2, WIDTH => c_SHOT_SEQ_WIDTH)
            port map (
                src_clk       => i_tdc_clk,
                src_in_bin    => std_logic_vector(s_chip_shot_seq(i)),
                dest_clk      => i_axis_aclk,
                dest_out_bin  => s_chip_shot_seq_axi_slv(i)
            );
        s_chip_shot_seq_axi(i) <= unsigned(s_chip_shot_seq_axi_slv(i));

        -- =================================================================
        -- chip_ctrl: single-shot FSM (powerup/cfg/arm/capture/drain)
        -- All inputs now use CDC'd versions from AXI-Stream domain.
        -- =================================================================
        u_chip_ctrl : entity work.tdc_gpx_chip_ctrl
            generic map (
                g_CHIP_ID        => i,
                g_POWERUP_CLKS   => g_POWERUP_CLKS,
                g_RECOVERY_CLKS  => g_RECOVERY_CLKS,
                g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS
            )
            port map (
                i_clk               => i_tdc_clk,
                i_rst_n             => s_tdc_aresetn,
                i_cfg               => s_cfg_tdc,
                i_cfg_image         => s_cfg_image_tdc,
                i_cmd_start         => s_cmd_start_tdc,
                i_cmd_stop          => s_cmd_stop_tdc,
                i_cmd_soft_reset    => s_cmd_soft_reset_tdc,
                i_cmd_soft_reset_err => s_err_cmd_soft_reset_tdc(i),
                i_cmd_force_reinit  => s_cmd_force_reinit_tdc,
                i_soft_clear        => s_soft_clear_tdc,  -- Round 7 B-5
                i_cmd_cfg_write     => s_cmd_cfg_write_g_tdc,
                i_cmd_reg_read      => s_cmd_reg_read_tdc(i),
                i_cmd_reg_write     => s_cmd_reg_write_tdc(i),
                i_cmd_reg_addr      => s_cmd_reg_addr_tdc,
                i_cmd_reg_wdata     => s_cmd_reg_wdata_tdc,
                o_cmd_reg_rdata     => s_cmd_reg_rdata(i),
                o_cmd_reg_rvalid    => s_cmd_reg_rvalid(i),
                o_cmd_reg_done      => s_cmd_reg_done(i),
                i_shot_start        => s_shot_start_tdc(i),
                i_max_range_clks    => s_cfg_tdc.max_range_clks,
                i_stop_tdc          => s_stop_tdc_tdc,
                i_expected_ififo1   => s_expected_ififo1_tdc(i),
                i_expected_ififo2   => s_expected_ififo2_tdc(i),
                o_bus_req_valid     => s_bus_req_valid(i),
                o_bus_req_rw        => s_bus_req_rw(i),
                o_bus_req_addr      => s_bus_req_addr(i),
                o_bus_req_wdata     => s_bus_req_wdata(i),
                o_bus_oen_permanent => s_bus_oen_perm(i),
                o_bus_req_burst     => s_bus_req_burst(i),
                o_bus_ticks_snap    => s_bus_ticks_snap(i),
                i_s_axis_tvalid     => s_brsp_sk_tvalid(i),
                i_s_axis_tdata      => s_brsp_sk_tdata(i),
                i_s_axis_tuser      => s_brsp_sk_tuser(i),
                o_s_axis_tready     => s_brsp_sk_tready(i),
                i_bus_busy          => s_bus_busy(i),
                i_bus_rsp_pending   => s_bus_rsp_pending(i),
                i_ef1_sync          => s_ef1_sync(i),
                i_ef2_sync          => s_ef2_sync(i),
                i_irflag_sync       => s_irflag_sync(i),
                i_lf1_sync          => s_lf1_sync(i),
                i_lf2_sync          => s_lf2_sync(i),
                o_tick_en           => s_tick_en(i),
                o_stopdis           => o_tdc_stopdis(i),
                o_alutrigger        => o_tdc_alutrigger(i),
                o_puresn            => o_tdc_puresn(i),
                o_m_raw_axis_tvalid => s_raw_axis_tvalid(i),
                o_m_raw_axis_tdata  => s_raw_axis_tdata(i),
                o_m_raw_axis_tuser  => s_raw_axis_tuser(i),
                i_m_raw_axis_tready => s_raw_axis_tready(i),
                o_drain_done        => s_drain_done(i),
                o_run_drain_complete => s_run_drain_complete(i),  -- Round 6 B1: latched+CDC'd below
                o_shot_seq          => s_chip_shot_seq(i),
                o_busy              => s_chip_busy(i),
                o_err_drain_timeout => s_err_drain_timeout(i),
                o_err_sequence      => s_err_sequence(i),
                o_err_rsp_mismatch  => s_err_rsp_mismatch(i),
                o_err_raw_overflow  => s_err_raw_overflow(i),
                -- Round 12 #15: distinct cause outputs.
                o_err_raw_drop      => s_err_raw_drop(i),
                o_err_drain_cap     => s_err_drain_cap(i),
                o_err_reg_overflow  => s_err_reg_overflow(i),  -- Round 6 B1: CDC'd below
                o_run_timeout       => s_run_timeout(i),
                o_run_timeout_cause => s_run_timeout_cause(i),
                o_init_cfg_coalesced => s_init_cfg_coalesced(i),
                o_err_cmd_collision  => s_cmd_collision_mask(i),
                o_err_force_reinit   => s_err_force_reinit(i),
                o_err_raw_ctrl_drop  => s_err_raw_ctrl_drop(i),
                o_err_drain_mismatch => s_err_drain_mismatch(i),
                o_err_reg_rw_ambiguous => s_err_rw_ambiguous_reg(i),
                o_err_stopdis_mid_shot => s_err_stopdis_mid_shot(i),
                o_err_cmd_collision_vec => s_cmd_collision_vec_per(i)
            );

        -- ----- CDC FIFO: chip_ctrl (TDC clk) -> decode_pipe (AXI-S clk) -----
        -- Replaces skid buffer with async FIFO for clock-domain crossing.
        -- Write side: TDC clock (i_tdc_clk), read side: AXI-Stream (i_axis_aclk).
        s_raw_axis_tready(i) <= not s_raw_cdc_full(i);
        s_sk_raw_tvalid(i)   <= not s_raw_cdc_empty(i);

        -- Round 6 A1: stretch 1-clk soft_reset pulses into a multi-cycle
        -- reset so xpm_fifo_async can drop any stale beats left from the
        -- previous shot. Without this, post-soft_reset traffic into cell_pipe
        -- could start with previous-shot residue (the consumer FSMs cannot
        -- distinguish a stale drain_done from a fresh one).
        p_raw_fifo_rst : process(i_tdc_clk)
        begin
            if rising_edge(i_tdc_clk) then
                if s_tdc_aresetn = '0' then
                    s_raw_fifo_rst_cnt_r(i) <= (others => '0');
                elsif s_cmd_soft_reset_tdc = '1'
                   or s_err_cmd_soft_reset_tdc(i) = '1' then
                    s_raw_fifo_rst_cnt_r(i) <=
                        to_unsigned(c_RAW_FIFO_RST_CLKS, s_raw_fifo_rst_cnt_r(i)'length);
                elsif s_raw_fifo_rst_cnt_r(i) /= 0 then
                    s_raw_fifo_rst_cnt_r(i) <= s_raw_fifo_rst_cnt_r(i) - 1;
                end if;
            end if;
        end process p_raw_fifo_rst;

        -- Round 7 B-2: use s_tdc_aresetn (synchronized) instead of raw
        -- i_axis_aresetn in this combinational gate. The stretcher counter
        -- already lives in TDC domain; mixing an AXI-domain async reset
        -- into the same expression created an unnecessary CDC boundary.
        s_raw_fifo_rst(i) <= '1' when s_raw_fifo_rst_cnt_r(i) /= 0
                                   or s_tdc_aresetn = '0'
                             else '0';

        u_raw_cdc : xpm_fifo_async
            generic map (
                CDC_SYNC_STAGES  => 2,
                FIFO_MEMORY_TYPE => "distributed",
                FIFO_READ_LATENCY => 0,
                FIFO_WRITE_DEPTH => 16,
                READ_DATA_WIDTH  => 40,
                READ_MODE        => "fwft",
                WRITE_DATA_WIDTH => 40
            )
            port map (
                -- Write side (TDC clock domain)
                wr_clk        => i_tdc_clk,
                wr_en         => s_raw_axis_tvalid(i) and not s_raw_cdc_full(i),
                din           => s_raw_axis_tdata(i) & s_raw_axis_tuser(i),
                full          => s_raw_cdc_full(i),
                -- Read side (AXI-Stream clock domain)
                rd_clk        => i_axis_aclk,
                rd_en         => not s_raw_cdc_empty(i) and i_raw_sk_tready(i),
                dout(39 downto 8) => s_sk_raw_tdata(i),
                dout(7 downto 0)  => s_sk_raw_tuser(i),
                empty         => s_raw_cdc_empty(i),
                -- Reset (active-high, synchronized internally by XPM)
                -- Round 6 A1: stretched to include soft_reset pulses so
                -- stale beats from the previous shot don't survive a reset.
                rst           => s_raw_fifo_rst(i),
                wr_rst_busy   => open,
                rd_rst_busy   => open,
                -- Unused status / ECC ports
                almost_empty  => open,
                almost_full   => open,
                data_valid    => open,
                dbiterr       => open,
                overflow      => open,
                prog_empty    => open,
                prog_full     => open,
                rd_data_count => open,
                sbiterr       => open,
                underflow     => open,
                wr_ack        => open,
                wr_data_count => open,
                -- Unused inputs
                sleep         => '0',
                injectdbiterr => '0',
                injectsbiterr => '0'
            );

    end generate gen_chip;

end architecture rtl;
