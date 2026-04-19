-- =============================================================================
-- tdc_gpx_top.vhd
-- TDC-GPX Controller - Top-level structural wrapper
-- =============================================================================
--
-- Purpose:
--   Instantiates and connects the 4 cluster wrappers plus face_seq and
--   status_agg.  No processes -- pure structural + concurrent assignments.
--
--   [1] csr_pipeline    : Pipeline CSR (AXI4-Lite #2, 7-bit address)
--   [2] config_ctrl     : Cluster 1 - Chip CSR + cmd_arb + stop_cfg_decode +
--                         bus_phy x4 + chip_ctrl x4 + skid buffers
--   [3] decode_pipe     : Cluster 2 - decoder_i_mode + raw_event_builder + skids
--   [4] cell_pipe       : Cluster 3 - slope demux + cell_builder x8
--   [5] output_stage    : Cluster 4 - face_asm x2 + sync_fifo x2 + header x2
--   [6] face_seq        : Face/shot sequencer
--   [7] status_agg      : Status aggregation + timestamp + error counter
--
-- Clock domains:
--   i_axis_aclk  : TDC processing / AXI-Stream domain (~200 MHz)
--   s_axi_aclk   : AXI4-Lite PS domain
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_top is
    generic (
        g_HW_VERSION      : std_logic_vector(31 downto 0) := x"00010000";
        g_OUTPUT_WIDTH    : natural := 32;     -- output AXI-Stream tdata width (32 or 64)
        g_POWERUP_CLKS    : positive := 48;
        g_RECOVERY_CLKS   : positive := 8;
        g_ALU_PULSE_CLKS  : positive := 4;
        -- Stop event AXI-Stream interface parameters
        g_STOP_CNT_WIDTH  : natural := c_STOP_CNT_WIDTH;
        g_STOP_EVT_DWIDTH : natural := c_STOP_EVT_DATA_WIDTH
    );
    port (
        -- Processing / AXI-Stream clock and reset (150 MHz)
        i_axis_aclk      : in  std_logic;
        i_axis_aresetn   : in  std_logic;

        -- TDC-GPX bus control clock (200 MHz)
        -- Drives bus_phy and chip_ctrl (same clock group).
        -- May be same as i_axis_aclk for single-clock designs.
        i_tdc_clk        : in  std_logic;

        -- AXI-Lite clock / reset (PS domain, 100 MHz)
        s_axi_aclk       : in  std_logic;
        s_axi_aresetn    : in  std_logic;

        -- AXI4-Lite Slave #1: Chip CSR (9-bit address)
        s_axi_awvalid    : in  std_logic;
        s_axi_awready    : out std_logic;
        s_axi_awaddr     : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_awprot     : in  std_logic_vector(2 downto 0);
        s_axi_wvalid     : in  std_logic;
        s_axi_wready     : out std_logic;
        s_axi_wdata      : in  std_logic_vector(31 downto 0);
        s_axi_wstrb      : in  std_logic_vector(3 downto 0);
        s_axi_bvalid     : out std_logic;
        s_axi_bready     : in  std_logic;
        s_axi_bresp      : out std_logic_vector(1 downto 0);
        s_axi_arvalid    : in  std_logic;
        s_axi_arready    : out std_logic;
        s_axi_araddr     : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_arprot     : in  std_logic_vector(2 downto 0);
        s_axi_rvalid     : out std_logic;
        s_axi_rready     : in  std_logic;
        s_axi_rdata      : out std_logic_vector(31 downto 0);
        s_axi_rresp      : out std_logic_vector(1 downto 0);

        -- AXI4-Lite Slave #2: Pipeline CSR (7-bit address)
        s_axi_pipe_awvalid    : in  std_logic;
        s_axi_pipe_awready    : out std_logic;
        s_axi_pipe_awaddr     : in  std_logic_vector(6 downto 0);
        s_axi_pipe_awprot     : in  std_logic_vector(2 downto 0);
        s_axi_pipe_wvalid     : in  std_logic;
        s_axi_pipe_wready     : out std_logic;
        s_axi_pipe_wdata      : in  std_logic_vector(31 downto 0);
        s_axi_pipe_wstrb      : in  std_logic_vector(3 downto 0);
        s_axi_pipe_bvalid     : out std_logic;
        s_axi_pipe_bready     : in  std_logic;
        s_axi_pipe_bresp      : out std_logic_vector(1 downto 0);
        s_axi_pipe_arvalid    : in  std_logic;
        s_axi_pipe_arready    : out std_logic;
        s_axi_pipe_araddr     : in  std_logic_vector(6 downto 0);
        s_axi_pipe_arprot     : in  std_logic_vector(2 downto 0);
        s_axi_pipe_rvalid     : out std_logic;
        s_axi_pipe_rready     : in  std_logic;
        s_axi_pipe_rdata      : out std_logic_vector(31 downto 0);
        s_axi_pipe_rresp      : out std_logic_vector(1 downto 0);

        -- laser_ctrl_result stream input (i_axis_aclk domain)
        i_lsr_tvalid     : in  std_logic;
        i_lsr_tdata      : in  std_logic_vector(31 downto 0);

        -- Shot trigger (from laser_ctrl, 1-clk pulse, i_axis_aclk domain)
        i_shot_start     : in  std_logic;

        -- Stop TDC pulse (from laser_ctrl, 1-clk pulse, i_axis_aclk domain)
        i_stop_tdc        : in  std_logic;

        -- Stop event AXI Stream slave (from echo_receiver, i_axis_aclk domain)
        i_stop_evt_tvalid : in  std_logic;
        i_stop_evt_tdata  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        i_stop_evt_tkeep  : in  std_logic_vector(g_STOP_EVT_DWIDTH/8 - 1 downto 0);
        i_stop_evt_tuser  : in  std_logic_vector(g_STOP_EVT_DWIDTH - 1 downto 0);
        o_stop_evt_tready : out std_logic;

        -- TDC-GPX physical pins (per chip, x4)
        io_tdc_d         : inout t_tdc_bus_array;
        o_tdc_adr        : out   t_tdc_adr_array;
        o_tdc_csn        : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_rdn        : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_wrn        : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_oen        : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_stopdis    : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_alutrigger : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        o_tdc_puresn     : out   std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_ef1        : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_ef2        : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_lf1        : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_lf2        : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_irflag     : in    std_logic_vector(c_N_CHIPS - 1 downto 0);
        i_tdc_errflag    : in    std_logic_vector(c_N_CHIPS - 1 downto 0);

        -- AXI-Stream master output: RISING pipeline (to CDC FIFO / VDMA)
        o_m_axis_tdata   : out std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_tvalid  : out std_logic;
        o_m_axis_tlast   : out std_logic;
        o_m_axis_tuser   : out std_logic_vector(0 downto 0);
        i_m_axis_tready  : in  std_logic;

        -- AXI-Stream master output: FALLING pipeline (to CDC FIFO / VDMA)
        o_m_axis_fall_tdata  : out std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_fall_tvalid : out std_logic;
        o_m_axis_fall_tlast  : out std_logic;
        o_m_axis_fall_tuser  : out std_logic_vector(0 downto 0);
        i_m_axis_fall_tready : in  std_logic;

        -- Calibration inputs (from external computation, i_axis_aclk domain)
        i_bin_resolution_ps : in  unsigned(15 downto 0);
        i_k_dist_fixed      : in  unsigned(31 downto 0);

        -- Interrupts
        o_irq            : out std_logic;
        o_irq_pipe       : out std_logic
    );
end entity tdc_gpx_top;

architecture rtl of tdc_gpx_top is

    -- =========================================================================
    -- Configuration signals
    -- =========================================================================
    signal s_cfg              : t_tdc_cfg;
    signal s_cfg_pipeline     : t_tdc_cfg;
    signal s_cfg_image        : t_cfg_image;
    signal s_cfg_face_r       : t_tdc_cfg;
    signal s_cdc_idle         : std_logic;

    -- =========================================================================
    -- Command signals (from csr_pipeline)
    -- =========================================================================
    signal s_cmd_start        : std_logic;
    signal s_cmd_stop         : std_logic;
    signal s_cmd_soft_reset   : std_logic;
    signal s_cmd_force_reinit : std_logic;  -- Round 12 A1
    signal s_err_force_reinit_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_raw_ctrl_drop_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 A2
    signal s_err_drain_mismatch_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 A4
    signal s_err_rw_ambiguous_arb    : std_logic;  -- Round 12 A5
    signal s_err_rw_ambiguous_reg_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 A5
    signal s_err_stopdis_mid_shot_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 B8
    signal s_err_cmd_collision_vec : std_logic_vector(3 downto 0);  -- Round 12 #20
    signal s_err_raw_drop_mask     : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 #15
    signal s_err_drain_cap_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 12 #15
    signal s_run_timeout_cause_per_chip : std_logic_vector(3 * c_N_CHIPS - 1 downto 0);  -- Round 12 #17
    signal s_err_bus_fatal_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 13 axis 2
    signal s_drain_faulted_mask    : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- Round 13 axis 1a
    -- Round 13 follow-up P1 (audit 4번): per-chip cell tuser(0) = faulted
    -- flag, carried on each chip's tlast beat. Threaded cell_pipe →
    -- output_stage → face_assembler (via its per-chip xpm_fifo_axis).
    signal s_cell_rise_tuser : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tuser : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_cfg_write    : std_logic;
    -- Shared SW-initiated error clear (Q&A #40). Drives BOTH err_handler
    -- (fatal/retry state) and status_agg (sticky errors + error cycle count).
    -- Round 13 follow-up: now driven by csr_pipeline o_err_soft_clear
    -- (CTL2[1] rising-edge pulse, i_axis_aclk domain).
    signal s_err_soft_clear   : std_logic;
    signal s_cmd_cfg_write_g  : std_logic;
    signal s_cmd_start_accepted : std_logic;

    -- =========================================================================
    -- Cluster 1 -> Cluster 2 (AXI-Stream x4)
    -- =========================================================================
    signal s_raw_sk_tvalid    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_sk_tdata     : t_slv32_array;
    signal s_raw_sk_tuser     : t_slv8_array;
    signal s_raw_sk_tready    : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Cluster 2 -> Cluster 3 (AXI-Stream x4)
    -- =========================================================================
    signal s_evt_sk_tvalid    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_evt_sk_tdata     : t_slv32_array;
    signal s_evt_sk_tuser     : t_slv16_array;
    signal s_evt_sk_tready    : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Cluster 3 -> Cluster 4 (AXI-Stream x4, dual slope)
    -- =========================================================================
    signal s_cell_rise_tdata_0 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_rise_tdata_1 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_rise_tdata_2 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_rise_tdata_3 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_rise_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_rise_tlast   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_rise_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tdata_0 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_fall_tdata_1 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_fall_tdata_2 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_fall_tdata_3 : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_cell_fall_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tlast   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Chip status (config_ctrl -> face_seq + status_agg)
    -- =========================================================================
    signal s_chip_busy          : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_shot_seq      : t_shot_seq_array;
    signal s_errflag_sync       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_drain_timeout  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_sequence       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_reg_outstanding    : std_logic;

    -- =========================================================================
    -- Output stage status (-> face_seq + status_agg)
    -- =========================================================================
    signal s_face_asm_idle        : std_logic;
    signal s_face_asm_fall_idle   : std_logic;
    signal s_hdr_idle             : std_logic;
    signal s_hdr_fall_idle        : std_logic;
    signal s_hdr_draining         : std_logic;
    signal s_hdr_fall_draining    : std_logic;
    signal s_row_done             : std_logic;
    signal s_row_fall_done        : std_logic;
    signal s_chip_error_flags     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_fall_error      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_overrun         : std_logic;
    signal s_shot_fall_overrun    : std_logic;
    -- Round 7 C-1: s_face_abort / s_face_fall_abort declarations removed.
    -- The face_seq inputs are now tied to '0' literals at the port map so
    -- there is no dangling named wire to later be "reconnected" by accident.
    -- See Round 6 B5 for the background — face_assembler.o_face_abort has
    -- been a permanent '0' since Round 4.
    signal s_frame_done           : std_logic;
    signal s_frame_fall_done      : std_logic;

    -- Pipeline tvalid monitors (from output_stage)
    signal s_face_tvalid          : std_logic;
    signal s_face_fall_tvalid     : std_logic;
    signal s_face_buf_tvalid      : std_logic;
    signal s_face_fall_buf_tvalid : std_logic;

    -- =========================================================================
    -- Decode pipe status
    -- =========================================================================
    signal s_stop_id_error    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hit_dropped      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hit_fall_dropped : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_dropped     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_fall_dropped : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_slice_timeout    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_slice_fall_timeout : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Face sequencer signals
    -- =========================================================================
    signal s_shot_start_gated       : std_logic;
    signal s_face_start_gated       : std_logic;
    signal s_pipeline_abort         : std_logic;
    -- #22 Sprint 2: per-slope abort signals for slope-independent pipelines.
    -- Sprint 2 keeps both equal to the global pipeline_abort (legacy
    -- behavior). Sprint 3 will drive them from face_seq's separate rise/fall
    -- outputs so a fall-only abort no longer kills the rise side.
    signal s_pipeline_abort_rise    : std_logic;
    signal s_pipeline_abort_fall    : std_logic;
    signal s_shot_start_per_chip    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_face_id_r              : unsigned(7 downto 0);
    signal s_frame_id_r             : unsigned(31 downto 0);
    signal s_global_shot_seq_r      : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
    signal s_face_active_mask_r     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_face_stops_per_chip_r  : unsigned(3 downto 0);
    signal s_face_cols_per_face_r   : unsigned(15 downto 0);
    signal s_rows_per_face_r        : unsigned(15 downto 0);
    signal s_hsize_bytes_r          : unsigned(15 downto 0);
    signal s_face_state_idle        : std_logic;
    signal s_face_closing           : std_logic;
    signal s_packet_start           : std_logic;
    signal s_face_start_r           : std_logic;
    signal s_shot_drop_cnt_r        : unsigned(15 downto 0);
    signal s_cfg_rejected_r         : std_logic;
    signal s_frame_abort_cnt_r      : unsigned(15 downto 0);
    signal s_frame_done_both        : std_logic;
    signal s_face_n_faces_r         : unsigned(3 downto 0);

    -- =========================================================================
    -- Status signals
    -- =========================================================================
    signal s_status               : t_tdc_status := c_TDC_STATUS_INIT;
    signal s_timestamp_r          : unsigned(63 downto 0);
    signal s_error_count_r        : unsigned(31 downto 0);
    signal s_err_drain_to_sticky_r : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_seq_sticky_r     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_error_merged    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_error_raw       : std_logic_vector(c_N_CHIPS - 1 downto 0);  -- unmasked: all chips
    signal s_err_rsp_mismatch     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_raw_overflow     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_run_timeout          : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_reg_arb_timeout      : std_logic;
    -- Round 5 follow-up: pipeline-wide sticky observability
    signal s_err_read_timeout     : std_logic;
    signal s_err_frame_wait_escape : std_logic;  -- Round 11 item 11
    signal s_reg_rejected         : std_logic;
    signal s_reg_zero_mask        : std_logic;
    -- Round 6 B1: per-chip observability (CDC'd to AXI-S by config_ctrl)
    signal s_err_reg_overflow     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_run_drain_complete   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 6 follow-up: per-slope face_assembler stickies (AXI-Stream domain)
    signal s_shot_flush_drop_rise    : std_logic;
    signal s_shot_flush_drop_mask_rise : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_flush_drop_mask_fall : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_flush_drop_fall    : std_logic;
    signal s_shot_overrun_count_rise : unsigned(7 downto 0);
    signal s_shot_overrun_count_fall : unsigned(7 downto 0);
    -- Round 11 C: Category C observability surface
    signal s_reg_timeout_mask        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 11 item 10: per-chip stop_cfg_decode monotonic violation sticky
    signal s_mono_violation_mask     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_orphan_stop_evt_sticky  : std_logic;  -- Round 12 #16
    -- Round 11 item 14: per-chip chip_init cfg_write coalesce sticky
    signal s_init_cfg_coalesced_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 11 item 18 (C): per-chip PH_IDLE cmd-collision sticky
    signal s_cmd_collision_mask      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_run_timeout_cause_last  : std_logic_vector(2 downto 0);
    signal s_hdr_face_start_collapsed_rise : unsigned(7 downto 0);
    signal s_hdr_drain_timeout_rise : std_logic;
    signal s_hdr_drain_timeout_fall : std_logic;
    signal s_hdr_abort_truncated_rise : std_logic;  -- Round 12 #19
    signal s_hdr_abort_truncated_fall : std_logic;  -- Round 12 #19
    signal s_frame_done_faulted_rise  : std_logic;  -- Round 13 axis 1b (pulse)
    signal s_frame_done_faulted_fall  : std_logic;  -- Round 13 axis 1b (pulse)
    -- Latched sticky at top for SW observability
    signal s_frame_done_faulted_sticky_r : std_logic := '0';
    signal s_row_done_faulted_rise    : std_logic;  -- Round 13 axis 1c (pulse)
    signal s_row_done_faulted_fall    : std_logic;  -- Round 13 axis 1c (pulse)
    signal s_row_done_faulted_sticky_r : std_logic := '0';
    -- Round 12 #18: partial/blank chip_error split per slope
    signal s_chip_error_partial_rise : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_error_blank_rise   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_error_partial_fall : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_error_blank_fall   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hdr_face_start_collapsed_fall : unsigned(7 downto 0);
    -- cell_pipe stop_id_error per-chip (rise and fall); sticky aggregation below
    signal s_stop_id_error_rise   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_stop_id_error_fall   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Round 11 item 4: cell_builder QUARANTINE escalation sticky, per chip / slope
    signal s_quarantine_escape_rise : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_quarantine_escape_fall : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_quarantine_escape_mask_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_stop_id_error_mask_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_run_timeout_sticky_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- Error handler status (from config_ctrl)
    signal s_err_active    : std_logic;
    signal s_err_fatal     : std_logic;
    signal s_err_chip_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_cause     : std_logic_vector(2 downto 0);

begin

    -- =========================================================================
    -- Chip error merged (concurrent glue)
    -- =========================================================================
    -- Unmasked: all chip errors visible for SW diagnostics / status
    s_chip_error_raw    <= s_errflag_sync or s_chip_error_flags or s_chip_fall_error;
    -- Masked: only active chips, used for recovery gating and header
    s_chip_error_merged <= s_chip_error_raw and s_face_active_mask_r;

    -- =========================================================================
    -- [1] csr_pipeline (Pipeline CSR, separate AXI4-Lite port)
    -- =========================================================================
    u_csr_pipeline : entity work.tdc_gpx_csr_pipeline
        generic map (
            g_HW_VERSION     => g_HW_VERSION,
            g_OUTPUT_WIDTH   => g_OUTPUT_WIDTH
        )
        port map (
            s_axi_aclk          => s_axi_aclk,
            s_axi_aresetn       => s_axi_aresetn,
            s_axi_awvalid       => s_axi_pipe_awvalid,
            s_axi_awready       => s_axi_pipe_awready,
            s_axi_awaddr        => s_axi_pipe_awaddr,
            s_axi_awprot        => s_axi_pipe_awprot,
            s_axi_wvalid        => s_axi_pipe_wvalid,
            s_axi_wready        => s_axi_pipe_wready,
            s_axi_wdata         => s_axi_pipe_wdata,
            s_axi_wstrb         => s_axi_pipe_wstrb,
            s_axi_bvalid        => s_axi_pipe_bvalid,
            s_axi_bready        => s_axi_pipe_bready,
            s_axi_bresp         => s_axi_pipe_bresp,
            s_axi_arvalid       => s_axi_pipe_arvalid,
            s_axi_arready       => s_axi_pipe_arready,
            s_axi_araddr        => s_axi_pipe_araddr,
            s_axi_arprot        => s_axi_pipe_arprot,
            s_axi_rvalid        => s_axi_pipe_rvalid,
            s_axi_rready        => s_axi_pipe_rready,
            s_axi_rdata         => s_axi_pipe_rdata,
            s_axi_rresp         => s_axi_pipe_rresp,
            i_axis_aclk         => i_axis_aclk,
            i_axis_aresetn      => i_axis_aresetn,
            i_lsr_tvalid        => i_lsr_tvalid,
            i_lsr_tdata         => i_lsr_tdata,
            i_chip_csr_cdc_idle => s_cdc_idle,
            o_cfg               => s_cfg_pipeline,
            o_cmd_start         => s_cmd_start,
            i_cmd_start_accepted => s_cmd_start_accepted,
            o_cmd_stop          => s_cmd_stop,
            o_cmd_soft_reset    => s_cmd_soft_reset,
            o_cmd_force_reinit  => s_cmd_force_reinit,   -- Round 12 A1
            o_err_soft_clear    => s_err_soft_clear,     -- Round 13 follow-up
            o_cmd_cfg_write     => s_cmd_cfg_write,
            i_status            => s_status,
            o_irq               => o_irq_pipe
        );

    -- =========================================================================
    -- [2] config_ctrl (Cluster 1)
    -- =========================================================================
    u_config_ctrl : entity work.tdc_gpx_config_ctrl
        generic map (
            g_HW_VERSION      => g_HW_VERSION,
            g_POWERUP_CLKS    => g_POWERUP_CLKS,
            g_RECOVERY_CLKS   => g_RECOVERY_CLKS,
            g_ALU_PULSE_CLKS  => g_ALU_PULSE_CLKS,
            g_STOP_EVT_DWIDTH => g_STOP_EVT_DWIDTH
        )
        port map (
            i_axis_aclk          => i_axis_aclk,
            i_axis_aresetn       => i_axis_aresetn,
            i_tdc_clk            => i_tdc_clk,
            s_axi_aclk           => s_axi_aclk,
            s_axi_aresetn        => s_axi_aresetn,
            -- AXI4-Lite pass-through (chip CSR)
            s_axi_awvalid        => s_axi_awvalid,
            s_axi_awready        => s_axi_awready,
            s_axi_awaddr         => s_axi_awaddr,
            s_axi_awprot         => s_axi_awprot,
            s_axi_wvalid         => s_axi_wvalid,
            s_axi_wready         => s_axi_wready,
            s_axi_wdata          => s_axi_wdata,
            s_axi_wstrb          => s_axi_wstrb,
            s_axi_bvalid         => s_axi_bvalid,
            s_axi_bready         => s_axi_bready,
            s_axi_bresp          => s_axi_bresp,
            s_axi_arvalid        => s_axi_arvalid,
            s_axi_arready        => s_axi_arready,
            s_axi_araddr         => s_axi_araddr,
            s_axi_arprot         => s_axi_arprot,
            s_axi_rvalid         => s_axi_rvalid,
            s_axi_rready         => s_axi_rready,
            s_axi_rdata          => s_axi_rdata,
            s_axi_rresp          => s_axi_rresp,
            -- TDC-GPX physical pins
            io_tdc_d             => io_tdc_d,
            o_tdc_adr            => o_tdc_adr,
            o_tdc_csn            => o_tdc_csn,
            o_tdc_rdn            => o_tdc_rdn,
            o_tdc_wrn            => o_tdc_wrn,
            o_tdc_oen            => o_tdc_oen,
            o_tdc_stopdis        => o_tdc_stopdis,
            o_tdc_alutrigger     => o_tdc_alutrigger,
            o_tdc_puresn         => o_tdc_puresn,
            i_tdc_ef1            => i_tdc_ef1,
            i_tdc_ef2            => i_tdc_ef2,
            i_tdc_lf1            => i_tdc_lf1,
            i_tdc_lf2            => i_tdc_lf2,
            i_tdc_irflag         => i_tdc_irflag,
            i_tdc_errflag        => i_tdc_errflag,
            -- Stop event stream
            i_stop_evt_tvalid    => i_stop_evt_tvalid,
            i_stop_evt_tdata     => i_stop_evt_tdata,
            i_stop_evt_tkeep     => i_stop_evt_tkeep,
            i_stop_evt_tuser     => i_stop_evt_tuser,
            o_stop_evt_tready    => o_stop_evt_tready,
            i_stop_tdc           => i_stop_tdc,
            -- Control inputs
            i_cmd_start          => s_cmd_start,
            i_cmd_start_accepted => s_cmd_start_accepted,
            i_cmd_stop           => s_cmd_stop,
            i_cmd_soft_reset     => s_cmd_soft_reset,
            i_cmd_force_reinit   => s_cmd_force_reinit,  -- Round 12 A1
            i_cmd_cfg_write      => s_cmd_cfg_write,
            i_err_soft_clear     => s_err_soft_clear,
            i_shot_start_per_chip => s_shot_start_per_chip,
            i_shot_start_gated   => s_shot_start_gated,
            i_cfg_pipeline       => s_cfg_pipeline,
            -- Cluster 4 idle inputs (for cmd_arb)
            i_face_asm_idle      => s_face_asm_idle,
            i_face_asm_fall_idle => s_face_asm_fall_idle,
            i_hdr_idle           => s_hdr_idle,
            i_hdr_fall_idle      => s_hdr_fall_idle,
            -- Frame done (for err_handler)
            i_frame_done         => s_frame_done,
            i_frame_fall_done    => s_frame_fall_done,
            -- Pipeline abort (flush raw skid buffers)
            i_pipeline_abort     => s_pipeline_abort,
            -- Error handler status
            o_err_active         => s_err_active,
            o_err_fatal          => s_err_fatal,
            o_err_chip_mask      => s_err_chip_mask,
            o_err_cause          => s_err_cause,
            -- AXI-Stream output to Cluster 2
            o_raw_sk_tvalid      => s_raw_sk_tvalid,
            o_raw_sk_tdata       => s_raw_sk_tdata,
            o_raw_sk_tuser       => s_raw_sk_tuser,
            i_raw_sk_tready      => s_raw_sk_tready,
            -- Configuration outputs
            o_cfg                => s_cfg,
            o_cfg_image          => s_cfg_image,
            -- Command outputs
            o_cmd_start          => open,
            o_cmd_cfg_write_g    => s_cmd_cfg_write_g,
            -- Chip status outputs
            o_chip_busy          => s_chip_busy,
            o_chip_shot_seq      => s_chip_shot_seq,
            o_errflag_sync       => s_errflag_sync,
            o_err_drain_timeout  => s_err_drain_timeout,
            o_err_sequence       => s_err_sequence,
            o_err_rsp_mismatch   => s_err_rsp_mismatch,
            o_err_raw_overflow   => s_err_raw_overflow,
            o_reg_outstanding    => s_reg_outstanding,
            o_reg_loop_resume    => open,  -- reserved: future use for gating face_seq resume
            o_run_timeout        => s_run_timeout,
            o_reg_arb_timeout    => s_reg_arb_timeout,
            -- Round 5 follow-up: pipeline-wide observability stickies
            o_err_read_timeout   => s_err_read_timeout,
            o_err_frame_wait_escape => s_err_frame_wait_escape,
            o_reg_rejected       => s_reg_rejected,
            o_reg_zero_mask      => s_reg_zero_mask,
            -- Round 6 B1: per-chip observability (CDC'd inside config_ctrl)
            o_err_reg_overflow   => s_err_reg_overflow,
            o_run_drain_complete => s_run_drain_complete,
            -- Round 11 C: Category C observability surface
            o_reg_timeout_mask   => s_reg_timeout_mask,
            o_run_timeout_cause  => s_run_timeout_cause_last,
            -- Round 11 item 10: stop_cfg_decode monotonic violation sticky
            o_mono_violation_mask => s_mono_violation_mask,
            -- Round 12 #16: stop_cfg_decode orphan-event sticky
            o_orphan_stop_evt_sticky => s_orphan_stop_evt_sticky,
            -- Round 11 item 14: per-chip chip_init cfg_write coalesce sticky
            o_init_cfg_coalesced_mask => s_init_cfg_coalesced_mask,
            -- Round 11 item 18 (C): per-chip PH_IDLE cmd-collision sticky
            o_cmd_collision_mask      => s_cmd_collision_mask,
            -- Round 12 A1: per-chip force-reinit used sticky
            o_err_force_reinit_mask   => s_err_force_reinit_mask,
            -- Round 12 A2: per-chip raw control-beat drop sticky
            o_err_raw_ctrl_drop_mask  => s_err_raw_ctrl_drop_mask,
            -- Round 12 A4: per-chip chip_run drain mismatch sticky
            o_err_drain_mismatch_mask => s_err_drain_mismatch_mask,
            -- Round 12 A5: concurrent R+W ambiguity stickies
            o_err_rw_ambiguous_arb    => s_err_rw_ambiguous_arb,
            o_err_rw_ambiguous_reg    => s_err_rw_ambiguous_reg_mask,
            -- Round 12 B8: per-chip stopdis_override mid-shot sticky
            o_err_stopdis_mid_shot_mask => s_err_stopdis_mid_shot_mask,
            -- Round 12 #20: PH_IDLE command collision vector
            o_err_cmd_collision_vec    => s_err_cmd_collision_vec,
            -- Round 12 #15: distinct raw-overflow cause masks
            o_err_raw_drop_mask        => s_err_raw_drop_mask,
            o_err_drain_cap_mask       => s_err_drain_cap_mask,
            -- Round 12 #17: per-chip run_timeout_cause packed vector
            o_run_timeout_cause_per_chip => s_run_timeout_cause_per_chip,
            -- Round 13 axis 2: per-chip bus fatal sticky mask
            o_err_bus_fatal_mask       => s_err_bus_fatal_mask,
            -- Round 13 axis 1a: per-chip drain_done_faulted sticky
            o_drain_faulted_mask       => s_drain_faulted_mask,
            o_cdc_idle           => s_cdc_idle,
            -- Interrupt
            o_irq                => o_irq
        );

    -- =========================================================================
    -- [3] decode_pipe (Cluster 2)
    -- =========================================================================
    u_decode_pipe : entity work.tdc_gpx_decode_pipe
        port map (
            i_clk               => i_axis_aclk,
            i_rst_n             => i_axis_aresetn,
            -- Input from Cluster 1
            i_raw_sk_tvalid     => s_raw_sk_tvalid,
            i_raw_sk_tdata      => s_raw_sk_tdata,
            i_raw_sk_tuser      => s_raw_sk_tuser,
            o_raw_sk_tready     => s_raw_sk_tready,
            -- Context from Cluster 1
            i_chip_shot_seq     => s_chip_shot_seq,
            i_face_stops_per_chip => s_face_stops_per_chip_r,
            -- Output to Cluster 3
            o_evt_sk_tvalid     => s_evt_sk_tvalid,
            o_evt_sk_tdata      => s_evt_sk_tdata,
            o_evt_sk_tuser      => s_evt_sk_tuser,
            i_evt_sk_tready     => s_evt_sk_tready,
            -- Status
            o_stop_id_error     => s_stop_id_error,
            -- Pipeline abort: flush internal skid buffers
            i_flush             => s_pipeline_abort
        );

    -- =========================================================================
    -- [4] cell_pipe (Cluster 3)
    -- =========================================================================
    u_cell_pipe : entity work.tdc_gpx_cell_pipe
        generic map (
            g_OUTPUT_WIDTH => g_OUTPUT_WIDTH
        )
        port map (
            i_clk                   => i_axis_aclk,
            i_rst_n                 => i_axis_aresetn,
            -- Event input from Cluster 2
            i_evt_sk_tvalid         => s_evt_sk_tvalid,
            i_evt_sk_tdata          => s_evt_sk_tdata,
            i_evt_sk_tuser          => s_evt_sk_tuser,
            o_evt_sk_tready         => s_evt_sk_tready,
            -- Control / Config
            i_shot_start_per_chip   => s_shot_start_per_chip,
            i_abort                 => s_pipeline_abort,       -- global (legacy)
            i_abort_rise            => s_pipeline_abort_rise,  -- #22 Sprint 2
            i_abort_fall            => s_pipeline_abort_fall,  -- #22 Sprint 2
            i_face_stops_per_chip   => s_face_stops_per_chip_r,
            -- Round 11 item 2: use face snapshot (latched at packet_start)
            -- instead of live s_cfg, so cell_builder's beat-per-cell lookup
            -- stays aligned with the header metadata for the same face.
            i_max_hits_cfg          => s_cfg_face_r.max_hits_cfg,
            -- Rising cell output
            o_cell_rise_tdata_0     => s_cell_rise_tdata_0,
            o_cell_rise_tdata_1     => s_cell_rise_tdata_1,
            o_cell_rise_tdata_2     => s_cell_rise_tdata_2,
            o_cell_rise_tdata_3     => s_cell_rise_tdata_3,
            o_cell_rise_tvalid      => s_cell_rise_tvalid,
            o_cell_rise_tlast       => s_cell_rise_tlast,
            i_cell_rise_tready      => s_cell_rise_tready,
            -- Falling cell output
            o_cell_fall_tdata_0     => s_cell_fall_tdata_0,
            o_cell_fall_tdata_1     => s_cell_fall_tdata_1,
            o_cell_fall_tdata_2     => s_cell_fall_tdata_2,
            o_cell_fall_tdata_3     => s_cell_fall_tdata_3,
            o_cell_fall_tvalid      => s_cell_fall_tvalid,
            o_cell_fall_tlast       => s_cell_fall_tlast,
            i_cell_fall_tready      => s_cell_fall_tready,
            -- Status
            o_hit_dropped           => s_hit_dropped,
            o_hit_fall_dropped      => s_hit_fall_dropped,
            o_shot_dropped          => s_shot_dropped,
            o_shot_fall_dropped     => s_shot_fall_dropped,
            o_slice_timeout         => s_slice_timeout,
            o_slice_fall_timeout    => s_slice_fall_timeout,
            -- Round 11 C: distinct stop_id_error per-chip (separate from hit overflow)
            o_stop_id_error         => s_stop_id_error_rise,
            o_stop_id_fall_error    => s_stop_id_error_fall,
            -- Round 11 item 4: per-chip cell_builder QUARANTINE escalation sticky
            o_quarantine_escape_rise => s_quarantine_escape_rise,
            o_quarantine_escape_fall => s_quarantine_escape_fall,
            -- Round 13 follow-up P1: per-chip cell tuser (faulted flag on tlast)
            o_cell_rise_tuser => s_cell_rise_tuser,
            o_cell_fall_tuser => s_cell_fall_tuser
        );

    -- =========================================================================
    -- [5] output_stage (Cluster 4)
    -- =========================================================================
    u_output_stage : entity work.tdc_gpx_output_stage
        generic map (
            g_OUTPUT_WIDTH   => g_OUTPUT_WIDTH,
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS
        )
        port map (
            i_clk                => i_axis_aclk,
            i_rst_n              => i_axis_aresetn,
            -- Rising cell input from Cluster 3
            i_cell_rise_tdata_0  => s_cell_rise_tdata_0,
            i_cell_rise_tdata_1  => s_cell_rise_tdata_1,
            i_cell_rise_tdata_2  => s_cell_rise_tdata_2,
            i_cell_rise_tdata_3  => s_cell_rise_tdata_3,
            i_cell_rise_tvalid   => s_cell_rise_tvalid,
            i_cell_rise_tlast    => s_cell_rise_tlast,
            o_cell_rise_tready   => s_cell_rise_tready,
            -- Falling cell input from Cluster 3
            i_cell_fall_tdata_0  => s_cell_fall_tdata_0,
            i_cell_fall_tdata_1  => s_cell_fall_tdata_1,
            i_cell_fall_tdata_2  => s_cell_fall_tdata_2,
            i_cell_fall_tdata_3  => s_cell_fall_tdata_3,
            i_cell_fall_tvalid   => s_cell_fall_tvalid,
            i_cell_fall_tlast    => s_cell_fall_tlast,
            o_cell_fall_tready   => s_cell_fall_tready,
            -- Round 13 follow-up P1: per-chip cell tuser (faulted on tlast)
            i_cell_rise_tuser    => s_cell_rise_tuser,
            i_cell_fall_tuser    => s_cell_fall_tuser,
            -- Control from face_seq
            i_shot_start_gated   => s_shot_start_gated,
            i_pipeline_abort     => s_pipeline_abort,       -- legacy global
            i_pipeline_abort_rise => s_pipeline_abort_rise,  -- #22 Sprint 3
            i_pipeline_abort_fall => s_pipeline_abort_fall,  -- #22 Sprint 3
            i_face_start_gated   => s_face_start_gated,
            -- Configuration (latched at face_start)
            i_face_active_mask   => s_face_active_mask_r,
            i_face_stops_per_chip => s_face_stops_per_chip_r,
            -- Round 11 item 2: use face snapshot so output_stage's beat count
            -- and scan-timeout match header metadata and cell_builder within
            -- the same face (both driven by s_cfg_face_r).
            i_max_hits_cfg       => s_cfg_face_r.max_hits_cfg,
            i_max_scan_clks      => s_cfg_face_r.max_scan_clks,
            i_rows_per_face      => s_rows_per_face_r,
            -- Header metadata
            i_cfg_face           => s_cfg_face_r,
            i_frame_id           => s_frame_id_r,
            i_face_id            => s_face_id_r,
            i_global_shot_seq    => s_global_shot_seq_r,
            i_timestamp          => s_timestamp_r,
            i_chip_error_merged  => s_chip_error_merged,
            i_error_count        => s_error_count_r,
            i_bin_resolution_ps  => i_bin_resolution_ps,
            i_k_dist_fixed       => i_k_dist_fixed,
            -- VDMA output: Rising
            o_m_axis_tdata       => o_m_axis_tdata,
            o_m_axis_tvalid      => o_m_axis_tvalid,
            o_m_axis_tlast       => o_m_axis_tlast,
            o_m_axis_tuser       => o_m_axis_tuser,
            i_m_axis_tready      => i_m_axis_tready,
            -- VDMA output: Falling
            o_m_axis_fall_tdata  => o_m_axis_fall_tdata,
            o_m_axis_fall_tvalid => o_m_axis_fall_tvalid,
            o_m_axis_fall_tlast  => o_m_axis_fall_tlast,
            o_m_axis_fall_tuser  => o_m_axis_fall_tuser,
            i_m_axis_fall_tready => i_m_axis_fall_tready,
            -- Status outputs
            o_row_done           => s_row_done,
            o_row_fall_done      => s_row_fall_done,
            o_chip_error_flags   => s_chip_error_flags,
            o_chip_fall_error    => s_chip_fall_error,
            o_shot_overrun       => s_shot_overrun,
            o_shot_fall_overrun  => s_shot_fall_overrun,
            -- Round 6 B5: o_face_abort has been a permanent '0' since Round 4
            -- (face self-completes via blank-fill instead of aborting). The
            -- port is retained for backward compatibility but we intentionally
            -- ground the downstream signal so a future re-enable in
            -- face_assembler cannot silently reactivate s_pipeline_abort
            -- through face_seq's OR logic without an explicit opt-in here.
            o_face_abort         => open,
            o_face_fall_abort    => open,
            o_face_asm_idle      => s_face_asm_idle,
            o_face_asm_fall_idle => s_face_asm_fall_idle,
            o_frame_done         => s_frame_done,
            o_frame_fall_done    => s_frame_fall_done,
            o_hdr_draining       => s_hdr_draining,
            o_hdr_fall_draining  => s_hdr_fall_draining,
            o_hdr_idle           => s_hdr_idle,
            o_hdr_fall_idle      => s_hdr_fall_idle,
            -- Pipeline tvalid monitors
            o_face_tvalid          => s_face_tvalid,
            o_face_fall_tvalid     => s_face_fall_tvalid,
            o_face_buf_tvalid      => s_face_buf_tvalid,
            o_face_fall_buf_tvalid => s_face_fall_buf_tvalid,
            -- Round 6 follow-up: per-slope face_assembler observability
            o_shot_flush_drop_rise    => s_shot_flush_drop_rise,
            o_shot_flush_drop_fall    => s_shot_flush_drop_fall,
            -- Round 11 item 15: per-chip breakdown of shot_flush_drop stickies
            o_shot_flush_drop_mask_rise => s_shot_flush_drop_mask_rise,
            o_shot_flush_drop_mask_fall => s_shot_flush_drop_mask_fall,
            o_shot_overrun_count_rise => s_shot_overrun_count_rise,
            o_shot_overrun_count_fall => s_shot_overrun_count_fall,
            -- Round 11 C: per-slope header_inserter face_start collapse count
            o_hdr_face_start_collapsed_rise => s_hdr_face_start_collapsed_rise,
            o_hdr_face_start_collapsed_fall => s_hdr_face_start_collapsed_fall,
            -- Round 11 item 3: header_inserter drain-watchdog sticky per slope
            o_hdr_drain_timeout_rise => s_hdr_drain_timeout_rise,
            o_hdr_drain_timeout_fall => s_hdr_drain_timeout_fall,
            -- Round 12 #19: per-slope abort-truncation sticky
            o_hdr_abort_truncated_rise => s_hdr_abort_truncated_rise,
            o_hdr_abort_truncated_fall => s_hdr_abort_truncated_fall,
            -- Round 13 axis 1b: per-slope frame_done_faulted pulse
            o_frame_done_faulted_rise  => s_frame_done_faulted_rise,
            o_frame_done_faulted_fall  => s_frame_done_faulted_fall,
            -- Round 13 axis 1c: per-slope row_done_faulted pulse
            o_row_done_faulted_rise    => s_row_done_faulted_rise,
            o_row_done_faulted_fall    => s_row_done_faulted_fall,
            -- Round 12 #18: partial/blank chip_error split per slope
            o_chip_error_partial_rise => s_chip_error_partial_rise,
            o_chip_error_blank_rise   => s_chip_error_blank_rise,
            o_chip_error_partial_fall => s_chip_error_partial_fall,
            o_chip_error_blank_fall   => s_chip_error_blank_fall
        );

    -- =========================================================================
    -- [6] face_seq (face/shot sequencer)
    -- =========================================================================
    u_face_seq : entity work.tdc_gpx_face_seq
        generic map (
            g_OUTPUT_WIDTH => g_OUTPUT_WIDTH
        )
        port map (
            i_clk                  => i_axis_aclk,
            i_rst_n                => i_axis_aresetn,
            i_cmd_start            => s_cmd_start,
            i_cmd_stop             => s_cmd_stop,
            i_cmd_soft_reset       => s_cmd_soft_reset,
            i_chip_busy            => s_chip_busy,
            i_reg_outstanding      => s_reg_outstanding,
            i_face_asm_idle        => s_face_asm_idle,
            i_face_asm_fall_idle   => s_face_asm_fall_idle,
            i_hdr_idle             => s_hdr_idle,
            i_hdr_fall_idle        => s_hdr_fall_idle,
            i_face_tvalid          => s_face_tvalid,
            i_face_fall_tvalid     => s_face_fall_tvalid,
            i_face_buf_tvalid      => s_face_buf_tvalid,
            i_face_fall_buf_tvalid => s_face_fall_buf_tvalid,
            i_m_axis_tvalid        => o_m_axis_tvalid,        -- VHDL-2008: read output port
            i_m_axis_fall_tvalid   => o_m_axis_fall_tvalid,   -- VHDL-2008: read output port
            i_shot_start_raw       => i_shot_start,
            i_frame_done           => s_frame_done,
            i_frame_fall_done      => s_frame_fall_done,
            -- Round 7 C-1: ground explicit (see removed signal comment above)
            i_face_abort           => '0',
            i_face_fall_abort      => '0',
            i_shot_overrun         => s_shot_overrun,
            i_shot_fall_overrun    => s_shot_fall_overrun,
            i_hdr_draining         => s_hdr_draining,
            i_hdr_fall_draining    => s_hdr_fall_draining,
            i_cfg                  => s_cfg,
            o_cmd_start_accepted   => s_cmd_start_accepted,
            o_face_state_idle      => s_face_state_idle,
            o_packet_start         => s_packet_start,
            o_face_start           => s_face_start_r,
            o_face_start_gated     => s_face_start_gated,
            o_shot_start_gated     => s_shot_start_gated,
            o_face_closing         => s_face_closing,
            o_pipeline_abort       => s_pipeline_abort,
            o_pipeline_abort_rise  => s_pipeline_abort_rise,
            o_pipeline_abort_fall  => s_pipeline_abort_fall,
            o_shot_drop_cnt        => s_shot_drop_cnt_r,
            o_cfg_rejected         => s_cfg_rejected_r,
            o_shot_start_per_chip  => s_shot_start_per_chip,
            o_face_id              => s_face_id_r,
            o_frame_id             => s_frame_id_r,
            o_global_shot_seq      => s_global_shot_seq_r,
            o_frame_abort_cnt      => s_frame_abort_cnt_r,
            o_frame_done_both      => s_frame_done_both,
            o_face_active_mask     => s_face_active_mask_r,
            o_face_stops_per_chip  => s_face_stops_per_chip_r,
            o_face_cols_per_face   => s_face_cols_per_face_r,
            o_face_n_faces         => s_face_n_faces_r,
            o_rows_per_face        => s_rows_per_face_r,
            o_hsize_bytes          => s_hsize_bytes_r,
            o_cfg_face             => s_cfg_face_r
        );

    -- =========================================================================
    -- [7] status_agg (status aggregation + timestamp + error counter)
    -- =========================================================================
    u_status_agg : entity work.tdc_gpx_status_agg
        port map (
            i_clk                  => i_axis_aclk,
            i_rst_n                => i_axis_aresetn,
            i_cmd_soft_reset       => s_cmd_soft_reset,
            i_cmd_start_accepted   => s_cmd_start_accepted,
            i_soft_clear           => s_err_soft_clear,
            i_face_state_idle      => s_face_state_idle,
            i_chip_busy            => s_chip_busy,
            i_reg_outstanding      => s_reg_outstanding,
            i_face_asm_idle        => s_face_asm_idle,
            i_face_asm_fall_idle   => s_face_asm_fall_idle,
            i_hdr_idle             => s_hdr_idle,
            i_hdr_fall_idle        => s_hdr_fall_idle,
            i_face_tvalid          => s_face_tvalid,
            i_face_fall_tvalid     => s_face_fall_tvalid,
            i_face_buf_tvalid      => s_face_buf_tvalid,
            i_face_fall_buf_tvalid => s_face_fall_buf_tvalid,
            i_m_axis_tvalid        => o_m_axis_tvalid,        -- VHDL-2008: read output port
            i_m_axis_fall_tvalid   => o_m_axis_fall_tvalid,   -- VHDL-2008: read output port
            i_stop_id_error        => s_stop_id_error,
            i_hit_dropped          => s_hit_dropped,
            i_hit_fall_dropped     => s_hit_fall_dropped,
            i_err_drain_timeout    => s_err_drain_timeout,
            i_err_sequence         => s_err_sequence,
            i_chip_error_merged    => s_chip_error_merged,
            i_face_active_mask     => s_face_active_mask_r,
            i_shot_overrun         => s_shot_overrun,
            i_shot_fall_overrun    => s_shot_fall_overrun,
            o_status               => s_status,
            o_timestamp            => s_timestamp_r,
            o_error_cycle_count    => s_error_count_r,
            o_err_drain_sticky     => s_err_drain_to_sticky_r,
            o_err_seq_sticky       => s_err_seq_sticky_r
        );

    -- =========================================================================
    -- Status field assignments (remaining fields not populated by status_agg)
    -- =========================================================================
    -- Round 13 axis 2: err_fatal now folds BOTH err_handler fatal recovery
    -- failure AND any chip's bus_fatal (PH_RESP_DRAIN quarantine + stuck bus).
    -- SW's existing err_fatal watcher captures both without code changes;
    -- granularity is in bus_fatal_mask for which-chip diagnosis.
    s_status.err_fatal        <= s_err_fatal when s_err_bus_fatal_mask = (s_err_bus_fatal_mask'range => '0')
                                 else '1';
    s_status.chip_error_mask     <= s_chip_error_raw;  -- unmasked: SW sees all chips
    s_status.drain_timeout_mask  <= s_err_drain_to_sticky_r;
    s_status.sequence_error_mask <= s_err_seq_sticky_r;
    s_status.shot_seq_current    <= s_global_shot_seq_r;
    s_status.vdma_frame_count    <= s_frame_id_r;
    s_status.error_cycle_count         <= s_error_count_r;
    s_status.shot_drop_count     <= s_shot_drop_cnt_r;
    s_status.frame_abort_count   <= s_frame_abort_cnt_r;
    s_status.err_active          <= s_err_active;
    s_status.err_chip_mask       <= s_err_chip_mask;
    s_status.err_cause           <= s_err_cause;
    s_status.rsp_mismatch_mask   <= s_err_rsp_mismatch;
    s_status.raw_overflow_mask   <= s_err_raw_overflow;
    s_status.cfg_rejected        <= s_cfg_rejected_r;
    s_status.run_timeout_mask    <= s_run_timeout_sticky_r;
    s_status.reg_arb_timeout     <= s_reg_arb_timeout;
    -- Round 5 follow-up: newly exposed stickies
    s_status.err_read_timeout    <= s_err_read_timeout;
    s_status.err_frame_wait_escape <= s_err_frame_wait_escape;
    s_status.reg_rejected        <= s_reg_rejected;
    s_status.reg_zero_mask       <= s_reg_zero_mask;
    -- Round 6 B1: per-chip stickies now CDC'd through config_ctrl
    s_status.err_reg_overflow_mask   <= s_err_reg_overflow;
    s_status.run_drain_complete_mask <= s_run_drain_complete;
    -- Round 6 follow-up: per-slope face_assembler stickies (AXI-Stream domain;
    -- no CDC needed because csr_pipeline packs STAT6 in the same domain).
    s_status.rise_shot_flush_drop    <= s_shot_flush_drop_rise;
    s_status.fall_shot_flush_drop    <= s_shot_flush_drop_fall;
    s_status.rise_shot_overrun_count <= s_shot_overrun_count_rise;
    s_status.fall_shot_overrun_count <= s_shot_overrun_count_fall;

    -- Round 11 C: Category C observability → STAT7
    s_status.reg_timeout_mask        <= s_reg_timeout_mask;
    s_status.run_timeout_cause_last  <= s_run_timeout_cause_last;
    s_status.rise_face_start_collapsed_count <= s_hdr_face_start_collapsed_rise;
    s_status.fall_face_start_collapsed_count <= s_hdr_face_start_collapsed_fall;
    -- Round 11 item 3: header_inserter drain-watchdog sticky.
    s_status.rise_hdr_drain_timeout <= s_hdr_drain_timeout_rise;
    s_status.fall_hdr_drain_timeout <= s_hdr_drain_timeout_fall;

    -- Round 11 item 4: aggregate cell_builder QUARANTINE escalation stickies.
    -- cell_builder's internal sticky is only cleared by its own i_rst_n so
    -- s_err_soft_clear cannot reset this mask — SW must re-issue pipeline
    -- reset to clear. The OR-into-latch is still useful: it lets STAT read
    -- either slope's event on a single per-chip bit.
    p_quarantine_escape_mask : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_quarantine_escape_mask_r <= (others => '0');
            else
                s_quarantine_escape_mask_r <= s_quarantine_escape_mask_r
                                              or s_quarantine_escape_rise
                                              or s_quarantine_escape_fall;
            end if;
        end if;
    end process p_quarantine_escape_mask;
    s_status.quarantine_escape_mask <= s_quarantine_escape_mask_r;

    -- Round 11 item 10: stop_cfg_decode monotonic violation sticky
    s_status.mono_violation_mask <= s_mono_violation_mask;
    -- Round 11 item 14: per-chip chip_init cfg_write coalesce sticky
    s_status.init_cfg_coalesced_mask <= s_init_cfg_coalesced_mask;
    -- Round 11 item 15: per-chip shot_flush_drop mask (OR of rise+fall slopes)
    s_status.shot_flush_drop_mask <=
        s_shot_flush_drop_mask_rise or s_shot_flush_drop_mask_fall;
    -- Round 11 item 18 (C): per-chip PH_IDLE cmd-collision sticky
    s_status.cmd_collision_mask <= s_cmd_collision_mask;
    -- Round 12 A1: per-chip force-reinit used sticky
    s_status.force_reinit_mask  <= s_err_force_reinit_mask;
    -- Round 12 A2: per-chip raw control-beat drop sticky
    s_status.raw_ctrl_drop_mask <= s_err_raw_ctrl_drop_mask;
    -- Round 12 A4: per-chip chip_run drain mismatch sticky
    s_status.drain_mismatch_mask <= s_err_drain_mismatch_mask;
    -- Round 12 A5: R+W ambiguity stickies
    s_status.rw_ambiguous_arb      <= s_err_rw_ambiguous_arb;
    s_status.rw_ambiguous_reg_mask <= s_err_rw_ambiguous_reg_mask;
    -- Round 12 B8: stopdis_override mid-shot sticky
    s_status.stopdis_mid_shot_mask <= s_err_stopdis_mid_shot_mask;
    -- Round 12 #19: per-slope abort-truncation sticky
    s_status.rise_hdr_abort_truncated <= s_hdr_abort_truncated_rise;
    s_status.fall_hdr_abort_truncated <= s_hdr_abort_truncated_fall;
    -- Round 12 #20: PH_IDLE collision vector
    s_status.cmd_collision_vec  <= s_err_cmd_collision_vec;
    -- Round 12 #15: distinct raw-overflow cause masks
    s_status.raw_drop_mask     <= s_err_raw_drop_mask;
    s_status.drain_cap_mask    <= s_err_drain_cap_mask;
    -- Round 12 #17: per-chip run_timeout_cause
    s_status.run_timeout_cause_per_chip <= s_run_timeout_cause_per_chip;
    -- Round 12 #18: partial/blank chip_error split per slope
    s_status.rise_chip_error_partial <= s_chip_error_partial_rise;
    s_status.rise_chip_error_blank   <= s_chip_error_blank_rise;
    s_status.fall_chip_error_partial <= s_chip_error_partial_fall;
    s_status.fall_chip_error_blank   <= s_chip_error_blank_fall;
    -- Round 12 #16: stop_cfg_decode orphan-event sticky
    s_status.orphan_stop_evt_sticky <= s_orphan_stop_evt_sticky;
    -- Round 13 axis 2: bus fatal sticky — mask and OR-folded into err_fatal
    s_status.bus_fatal_mask <= s_err_bus_fatal_mask;
    -- Round 13 axis 1a: drain-completion-faulted per-chip sticky
    s_status.drain_faulted_mask <= s_drain_faulted_mask;

    -- Round 13 axis 1b: latch frame_done_faulted pulses into a sticky
    -- (either slope firing sets it). AXI-Stream domain; no CDC needed
    -- since source is in the same domain.
    p_frame_faulted_sticky : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_frame_done_faulted_sticky_r <= '0';
            elsif s_frame_done_faulted_rise = '1' or s_frame_done_faulted_fall = '1' then
                s_frame_done_faulted_sticky_r <= '1';
            end if;
        end if;
    end process p_frame_faulted_sticky;
    s_status.frame_done_faulted_sticky <= s_frame_done_faulted_sticky_r;

    -- Round 13 axis 1c: row_done_faulted sticky latch.
    p_row_faulted_sticky : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_row_done_faulted_sticky_r <= '0';
            elsif s_row_done_faulted_rise = '1' or s_row_done_faulted_fall = '1' then
                s_row_done_faulted_sticky_r <= '1';
            end if;
        end if;
    end process p_row_faulted_sticky;
    s_status.row_done_faulted_sticky <= s_row_done_faulted_sticky_r;

    -- stop_id_error_mask: per-chip sticky aggregating rise + fall pulses.
    p_stop_id_error_sticky : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_err_soft_clear = '1' then
                s_stop_id_error_mask_r <= (others => '0');
            else
                s_stop_id_error_mask_r <= s_stop_id_error_mask_r
                                          or s_stop_id_error_rise
                                          or s_stop_id_error_fall;
            end if;
        end if;
    end process p_stop_id_error_sticky;
    s_status.stop_id_error_mask <= s_stop_id_error_mask_r;
    s_status.shot_drop_any       <= '1' when (s_shot_dropped or s_shot_fall_dropped)
                                              /= (s_shot_dropped'range => '0') else '0';
    s_status.slice_timeout_any   <= '1' when (s_slice_timeout or s_slice_fall_timeout)
                                              /= (s_slice_timeout'range => '0') else '0';

    -- =========================================================================
    -- Sticky latch for run_timeout (1-clk pulses -> persistent mask)
    -- =========================================================================
    p_run_timeout_sticky : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_run_timeout_sticky_r <= (others => '0');
            else
                s_run_timeout_sticky_r <= s_run_timeout_sticky_r or s_run_timeout;
            end if;
        end if;
    end process p_run_timeout_sticky;

end architecture rtl;
