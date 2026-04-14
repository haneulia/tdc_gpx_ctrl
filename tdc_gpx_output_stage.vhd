-- =============================================================================
-- tdc_gpx_output_stage.vhd
-- TDC-GPX Controller - Output Stage (Cluster 4)
-- =============================================================================
--
-- Purpose:
--   Pure structural wrapper for the TDC-GPX output pipeline.
--   Instantiates face assemblers, sync FIFOs, and header inserters for
--   both rising and falling slopes.
--
-- Instances (6):
--   u_face_asm       : tdc_gpx_face_assembler   (rising)
--   u_face_asm_fall  : tdc_gpx_face_assembler   (falling)
--   u_face_rise_fifo : xpm_fifo_axis             (rising, 16-deep)
--   u_face_fall_fifo : xpm_fifo_axis             (falling, 16-deep)
--   u_header         : tdc_gpx_header_inserter   (rising)
--   u_header_fall    : tdc_gpx_header_inserter   (falling)
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

entity tdc_gpx_output_stage is
    generic (
        g_OUTPUT_WIDTH   : natural := 32;   -- 32 or 64
        g_ALU_PULSE_CLKS : natural := 4
    );
    port (
        -- Clock / Reset
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;

        -- Cell input from Cluster 3 -- Rising (AXI-Stream x4 chips)
        i_cell_rise_tdata_0  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_rise_tdata_1  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_rise_tdata_2  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_rise_tdata_3  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_rise_tvalid   : in  std_logic_vector(3 downto 0);
        i_cell_rise_tlast    : in  std_logic_vector(3 downto 0);
        o_cell_rise_tready   : out std_logic_vector(3 downto 0);

        -- Cell input from Cluster 3 -- Falling (AXI-Stream x4 chips)
        i_cell_fall_tdata_0  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_fall_tdata_1  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_fall_tdata_2  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_fall_tdata_3  : in  std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        i_cell_fall_tvalid   : in  std_logic_vector(3 downto 0);
        i_cell_fall_tlast    : in  std_logic_vector(3 downto 0);
        o_cell_fall_tready   : out std_logic_vector(3 downto 0);

        -- Control from face_seq
        i_shot_start_gated   : in  std_logic;
        i_pipeline_abort     : in  std_logic;
        i_face_start_gated   : in  std_logic;

        -- Configuration (latched at face_start by caller)
        i_face_active_mask   : in  std_logic_vector(3 downto 0);
        i_face_stops_per_chip: in  unsigned(3 downto 0);
        i_max_hits_cfg       : in  unsigned(2 downto 0);
        i_max_scan_clks      : in  unsigned(15 downto 0);
        i_rows_per_face      : in  unsigned(15 downto 0);

        -- Header metadata
        i_cfg_face           : in  t_tdc_cfg;
        i_frame_id           : in  unsigned(31 downto 0);
        i_face_id            : in  unsigned(7 downto 0);
        i_global_shot_seq    : in  unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        i_timestamp          : in  unsigned(63 downto 0);
        i_chip_error_merged  : in  std_logic_vector(3 downto 0);
        i_error_count        : in  unsigned(31 downto 0);
        i_bin_resolution_ps  : in  unsigned(15 downto 0);
        i_k_dist_fixed       : in  unsigned(31 downto 0);

        -- VDMA output -- Rising (AXI-Stream)
        o_m_axis_tdata       : out std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_tvalid      : out std_logic;
        o_m_axis_tlast       : out std_logic;
        o_m_axis_tuser       : out std_logic_vector(0 downto 0);
        i_m_axis_tready      : in  std_logic;

        -- VDMA output -- Falling (AXI-Stream)
        o_m_axis_fall_tdata  : out std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
        o_m_axis_fall_tvalid : out std_logic;
        o_m_axis_fall_tlast  : out std_logic;
        o_m_axis_fall_tuser  : out std_logic_vector(0 downto 0);
        i_m_axis_fall_tready : in  std_logic;

        -- Status outputs
        o_row_done           : out std_logic;
        o_row_fall_done      : out std_logic;
        o_chip_error_flags   : out std_logic_vector(3 downto 0);
        o_chip_fall_error    : out std_logic_vector(3 downto 0);
        o_shot_overrun       : out std_logic;
        o_shot_fall_overrun  : out std_logic;
        o_face_abort         : out std_logic;
        o_face_fall_abort    : out std_logic;
        o_face_asm_idle      : out std_logic;
        o_face_asm_fall_idle : out std_logic;
        o_frame_done         : out std_logic;
        o_frame_fall_done    : out std_logic;
        o_hdr_draining       : out std_logic;
        o_hdr_fall_draining  : out std_logic;
        o_hdr_idle           : out std_logic;
        o_hdr_fall_idle      : out std_logic;

        -- Pipeline tvalid monitors (for face_seq / status_agg drain detection)
        o_face_tvalid          : out std_logic;   -- face_asm output valid (pre-FIFO)
        o_face_fall_tvalid     : out std_logic;   -- face_asm_fall output valid (pre-FIFO)
        o_face_buf_tvalid      : out std_logic;   -- post-FIFO valid (rising)
        o_face_fall_buf_tvalid : out std_logic    -- post-FIFO valid (falling)
    );
end entity tdc_gpx_output_stage;

architecture rtl of tdc_gpx_output_stage is

    attribute KEEP_HIERARCHY : string;
    attribute KEEP_HIERARCHY of rtl : architecture is "yes";

    -- =========================================================================
    -- Internal signals: face_assembler -> xpm_fifo_axis (rising)
    -- =========================================================================
    signal s_face_tdata      : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_face_tvalid     : std_logic;
    signal s_face_tlast      : std_logic;
    signal s_face_tready     : std_logic;

    -- =========================================================================
    -- Internal signals: face_assembler -> xpm_fifo_axis (falling)
    -- =========================================================================
    signal s_face_fall_tdata  : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_face_fall_tvalid : std_logic;
    signal s_face_fall_tlast  : std_logic;
    signal s_face_fall_tready : std_logic;

    -- =========================================================================
    -- Internal signals: xpm_fifo_axis -> header_inserter (rising)
    -- =========================================================================
    signal s_face_buf_tdata   : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_face_buf_tvalid  : std_logic;
    signal s_face_buf_tlast   : std_logic;
    signal s_face_buf_tready  : std_logic;

    -- =========================================================================
    -- Internal signals: xpm_fifo_axis -> header_inserter (falling)
    -- =========================================================================
    signal s_face_fall_buf_tdata  : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_face_fall_buf_tvalid : std_logic;
    signal s_face_fall_buf_tlast  : std_logic;
    signal s_face_fall_buf_tready : std_logic;

    -- FIFO reset: active-low, gated by pipeline_abort for flush
    signal s_fifo_rst_n : std_logic;

begin

    -- FIFO flush: pipeline_abort drives reset (active-low)
    s_fifo_rst_n <= i_rst_n and not i_pipeline_abort;

    -- =========================================================================
    -- Rising face assembler
    -- =========================================================================
    u_face_asm_rise : entity work.tdc_gpx_face_assembler
        generic map (
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS,
            g_TDATA_WIDTH    => g_OUTPUT_WIDTH
        )
        port map (
            i_clk              => i_clk,
            i_rst_n            => i_rst_n,
            i_s_axis_tdata_0   => i_cell_rise_tdata_0,
            i_s_axis_tdata_1   => i_cell_rise_tdata_1,
            i_s_axis_tdata_2   => i_cell_rise_tdata_2,
            i_s_axis_tdata_3   => i_cell_rise_tdata_3,
            i_s_axis_tvalid    => i_cell_rise_tvalid,
            i_s_axis_tlast     => i_cell_rise_tlast,
            o_s_axis_tready    => o_cell_rise_tready,
            i_shot_start       => i_shot_start_gated,
            i_abort            => i_pipeline_abort,
            i_active_chip_mask => i_face_active_mask,
            i_stops_per_chip   => i_face_stops_per_chip,
            i_max_hits_cfg     => i_max_hits_cfg,
            i_max_scan_clks    => i_max_scan_clks,
            o_m_axis_tdata     => s_face_tdata,
            o_m_axis_tvalid    => s_face_tvalid,
            o_m_axis_tlast     => s_face_tlast,
            i_m_axis_tready    => s_face_tready,
            o_row_done         => o_row_done,
            o_chip_error_flags => o_chip_error_flags,
            o_shot_overrun     => o_shot_overrun,
            o_face_abort       => o_face_abort,
            o_idle             => o_face_asm_idle
        );

    -- =========================================================================
    -- Falling face assembler
    -- =========================================================================
    u_face_asm_fall : entity work.tdc_gpx_face_assembler
        generic map (
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS,
            g_TDATA_WIDTH    => g_OUTPUT_WIDTH
        )
        port map (
            i_clk              => i_clk,
            i_rst_n            => i_rst_n,
            i_s_axis_tdata_0   => i_cell_fall_tdata_0,
            i_s_axis_tdata_1   => i_cell_fall_tdata_1,
            i_s_axis_tdata_2   => i_cell_fall_tdata_2,
            i_s_axis_tdata_3   => i_cell_fall_tdata_3,
            i_s_axis_tvalid    => i_cell_fall_tvalid,
            i_s_axis_tlast     => i_cell_fall_tlast,
            o_s_axis_tready    => o_cell_fall_tready,
            i_shot_start       => i_shot_start_gated,
            i_abort            => i_pipeline_abort,
            i_active_chip_mask => i_face_active_mask,
            i_stops_per_chip   => i_face_stops_per_chip,
            i_max_hits_cfg     => i_max_hits_cfg,
            i_max_scan_clks    => i_max_scan_clks,
            o_m_axis_tdata     => s_face_fall_tdata,
            o_m_axis_tvalid    => s_face_fall_tvalid,
            o_m_axis_tlast     => s_face_fall_tlast,
            i_m_axis_tready    => s_face_fall_tready,
            o_row_done         => o_row_fall_done,
            o_chip_error_flags => o_chip_fall_error,
            o_shot_overrun     => o_shot_fall_overrun,
            o_face_abort       => o_face_fall_abort,
            o_idle             => o_face_asm_fall_idle
        );

    -- =========================================================================
    -- Rising AXI-Stream FIFO (xpm_fifo_axis, 16-deep)
    -- =========================================================================
    u_face_rise_fifo : xpm_fifo_axis
        generic map (
            CASCADE_HEIGHT    => 0,
            CDC_SYNC_STAGES   => 2,
            CLOCKING_MODE     => "common_clock",
            ECC_MODE          => "no_ecc",
            FIFO_DEPTH        => 16,
            FIFO_MEMORY_TYPE  => "distributed",
            PACKET_FIFO       => "false",
            TDATA_WIDTH       => g_OUTPUT_WIDTH,
            TDEST_WIDTH       => 1,
            TID_WIDTH         => 1,
            TUSER_WIDTH       => 1,
            USE_ADV_FEATURES  => "0000"
        )
        port map (
            s_aclk          => i_clk,
            s_aresetn       => s_fifo_rst_n,
            s_axis_tvalid   => s_face_tvalid,
            s_axis_tready   => s_face_tready,
            s_axis_tdata    => s_face_tdata,
            s_axis_tlast    => s_face_tlast,
            s_axis_tkeep    => (others => '1'),
            s_axis_tstrb    => (others => '1'),
            s_axis_tuser    => (others => '0'),
            s_axis_tid      => (others => '0'),
            s_axis_tdest    => (others => '0'),
            m_axis_tvalid   => s_face_buf_tvalid,
            m_axis_tready   => s_face_buf_tready,
            m_axis_tdata    => s_face_buf_tdata,
            m_axis_tlast    => s_face_buf_tlast,
            m_axis_tkeep    => open,
            m_axis_tstrb    => open,
            m_axis_tuser    => open,
            m_axis_tid      => open,
            m_axis_tdest    => open,
            m_aclk          => '0',
            injectsbiterr_axis => '0',
            injectdbiterr_axis => '0'
        );

    -- =========================================================================
    -- Falling AXI-Stream FIFO (xpm_fifo_axis, 16-deep)
    -- =========================================================================
    u_face_fall_fifo : xpm_fifo_axis
        generic map (
            CASCADE_HEIGHT    => 0,
            CDC_SYNC_STAGES   => 2,
            CLOCKING_MODE     => "common_clock",
            ECC_MODE          => "no_ecc",
            FIFO_DEPTH        => 16,
            FIFO_MEMORY_TYPE  => "distributed",
            PACKET_FIFO       => "false",
            TDATA_WIDTH       => g_OUTPUT_WIDTH,
            TDEST_WIDTH       => 1,
            TID_WIDTH         => 1,
            TUSER_WIDTH       => 1,
            USE_ADV_FEATURES  => "0000"
        )
        port map (
            s_aclk          => i_clk,
            s_aresetn       => s_fifo_rst_n,
            s_axis_tvalid   => s_face_fall_tvalid,
            s_axis_tready   => s_face_fall_tready,
            s_axis_tdata    => s_face_fall_tdata,
            s_axis_tlast    => s_face_fall_tlast,
            s_axis_tkeep    => (others => '1'),
            s_axis_tstrb    => (others => '1'),
            s_axis_tuser    => (others => '0'),
            s_axis_tid      => (others => '0'),
            s_axis_tdest    => (others => '0'),
            m_axis_tvalid   => s_face_fall_buf_tvalid,
            m_axis_tready   => s_face_fall_buf_tready,
            m_axis_tdata    => s_face_fall_buf_tdata,
            m_axis_tlast    => s_face_fall_buf_tlast,
            m_axis_tkeep    => open,
            m_axis_tstrb    => open,
            m_axis_tuser    => open,
            m_axis_tid      => open,
            m_axis_tdest    => open,
            m_aclk          => '0',
            injectsbiterr_axis => '0',
            injectdbiterr_axis => '0'
        );

    -- =========================================================================
    -- Rising header inserter
    -- =========================================================================
    u_header_rise : entity work.tdc_gpx_header_inserter
        generic map (
            g_TDATA_WIDTH => g_OUTPUT_WIDTH
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_face_start        => i_face_start_gated,
            i_face_abort        => i_pipeline_abort,
            i_cfg               => i_cfg_face,
            i_vdma_frame_id     => i_frame_id,
            i_face_id           => i_face_id,
            i_shot_seq_start    => i_global_shot_seq,
            i_timestamp_ns      => i_timestamp,
            i_chip_error_mask   => i_chip_error_merged,
            i_chip_error_cnt    => std_logic_vector(i_error_count),
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            i_rows_per_face     => i_rows_per_face,
            i_s_axis_tdata      => s_face_buf_tdata,
            i_s_axis_tvalid     => s_face_buf_tvalid,
            i_s_axis_tlast      => s_face_buf_tlast,
            o_s_axis_tready     => s_face_buf_tready,
            o_m_axis_tdata      => o_m_axis_tdata,
            o_m_axis_tvalid     => o_m_axis_tvalid,
            o_m_axis_tlast      => o_m_axis_tlast,
            o_m_axis_tuser      => o_m_axis_tuser,
            i_m_axis_tready     => i_m_axis_tready,
            o_frame_done        => o_frame_done,
            o_draining          => o_hdr_draining,
            o_last_line         => open,
            o_idle              => o_hdr_idle
        );

    -- =========================================================================
    -- Falling header inserter
    -- =========================================================================
    u_header_fall : entity work.tdc_gpx_header_inserter
        generic map (
            g_TDATA_WIDTH => g_OUTPUT_WIDTH
        )
        port map (
            i_clk               => i_clk,
            i_rst_n             => i_rst_n,
            i_face_start        => i_face_start_gated,
            i_face_abort        => i_pipeline_abort,
            i_cfg               => i_cfg_face,
            i_vdma_frame_id     => i_frame_id,
            i_face_id           => i_face_id,
            i_shot_seq_start    => i_global_shot_seq,
            i_timestamp_ns      => i_timestamp,
            i_chip_error_mask   => i_chip_error_merged,
            i_chip_error_cnt    => std_logic_vector(i_error_count),
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            i_rows_per_face     => i_rows_per_face,
            i_s_axis_tdata      => s_face_fall_buf_tdata,
            i_s_axis_tvalid     => s_face_fall_buf_tvalid,
            i_s_axis_tlast      => s_face_fall_buf_tlast,
            o_s_axis_tready     => s_face_fall_buf_tready,
            o_m_axis_tdata      => o_m_axis_fall_tdata,
            o_m_axis_tvalid     => o_m_axis_fall_tvalid,
            o_m_axis_tlast      => o_m_axis_fall_tlast,
            o_m_axis_tuser      => o_m_axis_fall_tuser,
            i_m_axis_tready     => i_m_axis_fall_tready,
            o_frame_done        => o_frame_fall_done,
            o_draining          => o_hdr_fall_draining,
            o_last_line         => open,
            o_idle              => o_hdr_fall_idle
        );

    -- =========================================================================
    -- Pipeline tvalid monitors
    -- =========================================================================
    o_face_tvalid          <= s_face_tvalid;
    o_face_fall_tvalid     <= s_face_fall_tvalid;
    o_face_buf_tvalid      <= s_face_buf_tvalid;
    o_face_fall_buf_tvalid <= s_face_fall_buf_tvalid;

end architecture rtl;
