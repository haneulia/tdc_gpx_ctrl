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
        g_POWERUP_CLKS    : natural := 48;
        g_RECOVERY_CLKS   : natural := 8;
        g_ALU_PULSE_CLKS  : natural := 4;
        -- Stop event AXI-Stream interface parameters
        g_STOP_CNT_WIDTH  : natural := c_STOP_CNT_WIDTH;
        g_STOP_EVT_DWIDTH : natural := c_STOP_EVT_DATA_WIDTH
    );
    port (
        -- Processing / AXI-Stream clock and reset
        i_axis_aclk      : in  std_logic;
        i_axis_aresetn   : in  std_logic;

        -- AXI-Lite clock / reset (PS domain)
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
    signal s_cmd_cfg_write    : std_logic;
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
    signal s_face_abort           : std_logic;
    signal s_face_fall_abort      : std_logic;
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

    -- =========================================================================
    -- Face sequencer signals
    -- =========================================================================
    signal s_shot_start_gated       : std_logic;
    signal s_face_start_gated       : std_logic;
    signal s_pipeline_abort         : std_logic;
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

    -- Error handler status (from config_ctrl)
    signal s_err_active    : std_logic;
    signal s_err_fatal     : std_logic;
    signal s_err_chip_mask : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_cause     : std_logic_vector(2 downto 0);

begin

    -- =========================================================================
    -- Chip error merged (concurrent glue)
    -- =========================================================================
    s_chip_error_merged <= (s_errflag_sync or s_chip_error_flags or s_chip_fall_error)
                           and s_face_active_mask_r;

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
            i_cmd_cfg_write      => s_cmd_cfg_write,
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
            o_reg_outstanding    => s_reg_outstanding,
            o_reg_loop_resume    => open,  -- reserved: future use for gating face_seq resume
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
            -- Status
            o_stop_id_error     => s_stop_id_error
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
            -- Control / Config
            i_shot_start_per_chip   => s_shot_start_per_chip,
            i_face_stops_per_chip   => s_face_stops_per_chip_r,
            i_max_hits_cfg          => s_cfg.max_hits_cfg,
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
            o_hit_fall_dropped      => s_hit_fall_dropped
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
            -- Control from face_seq
            i_shot_start_gated   => s_shot_start_gated,
            i_pipeline_abort     => s_pipeline_abort,
            i_face_start_gated   => s_face_start_gated,
            -- Configuration (latched at face_start)
            i_face_active_mask   => s_face_active_mask_r,
            i_face_stops_per_chip => s_face_stops_per_chip_r,
            i_max_hits_cfg       => s_cfg.max_hits_cfg,
            i_max_scan_clks      => s_cfg.max_scan_clks,
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
            o_face_abort         => s_face_abort,
            o_face_fall_abort    => s_face_fall_abort,
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
            o_face_fall_buf_tvalid => s_face_fall_buf_tvalid
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
            i_face_abort           => s_face_abort,
            i_face_fall_abort      => s_face_fall_abort,
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
            o_shot_drop_cnt        => s_shot_drop_cnt_r,
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
            o_error_count          => s_error_count_r,
            o_err_drain_sticky     => s_err_drain_to_sticky_r,
            o_err_seq_sticky       => s_err_seq_sticky_r
        );

    -- =========================================================================
    -- Status field assignments (remaining fields not populated by status_agg)
    -- =========================================================================
    s_status.bin_mismatch        <= s_err_fatal;  -- repurposed: err_handler fatal recovery failure
    s_status.chip_error_mask     <= s_chip_error_merged;
    s_status.drain_timeout_mask  <= s_err_drain_to_sticky_r;
    s_status.sequence_error_mask <= s_err_seq_sticky_r;
    s_status.shot_seq_current    <= s_global_shot_seq_r;
    s_status.vdma_frame_count    <= s_frame_id_r;
    s_status.error_count         <= s_error_count_r;
    s_status.shot_drop_count     <= s_shot_drop_cnt_r;
    s_status.frame_abort_count   <= s_frame_abort_cnt_r;

end architecture rtl;
