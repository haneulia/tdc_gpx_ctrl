-- =============================================================================
-- tdc_gpx_top.vhd
-- TDC-GPX Controller - Top-level structural wrapper
-- =============================================================================
--
-- Purpose:
--   Instantiates and connects all TDC-GPX controller submodules:
--     CSR (AXI-Lite + CDC, pending latches, CDC idle gating)
--     Per-chip pipeline x4:
--       bus_phy → [skid] → chip_ctrl → [skid] → decode_i →
--       [skid] → raw_event_builder → [skid] → cell_builder (ping-pong)
--     face_assembler (4× xpm_fifo_axis inputs → packed row → xpm_fifo_axis)
--     header_inserter (header ROM + SOF/EOL → VDMA frame)
--
-- Generics:
--   g_OUTPUT_WIDTH : 32 or 64 (output AXI-Stream tdata width)
--     32-bit: 8 beats/cell, 12 header beats
--     64-bit: 4 beats/cell, 6 header beats (halves face_assembler latency)
--   max_hits_cfg (CTL21[18:16], runtime, 1~7):
--     Dynamically reduces beats/cell for distance-adaptive throughput.
--     Buffer always MAX=7; output beat count truncated at runtime.
--
-- Performance (64-bit mode, 2 chips, 8 stops, @200MHz, bus_ticks=5):
--
--   Throughput (face_assembler row processing):
--   Distance | max_hits | beats/cell | face_asm/row | shot period | margin
--   ---------|----------|-----------|-------------|------------|-------
--    100m    |    1     |     1     |  0.095 μs   |   1.0 μs   |  90%
--    500m    |    3     |     1     |  0.095 μs   |   3.33 μs  |  97%
--    700m    |    5     |     2     |  0.185 μs   |   4.67 μs  |  96%
--   1000m    |    7     |     4     |  0.365 μs   |   6.67 μs  |  95%
--
--   First-data latency (bus_phy read → VDMA first beat):
--     = IFIFO1_drain + pipeline_overhead(21 clk)
--     IFIFO1_drain = 4_stops × max_hits × bus_ticks
--   Distance | IFIFO1 drain | total latency
--   ---------|-------------|---------------
--    100m    |  20 clk     |  41 clk = 0.205 μs
--    500m    |  60 clk     |  81 clk = 0.405 μs
--    700m    | 100 clk     | 121 clk = 0.605 μs
--   1000m    | 140 clk     | 161 clk = 0.805 μs
--
--   Peak throughput: 1 beat/clk = 800 MB/s @32b, 1.6 GB/s @64b
--
-- Pipeline features:
--   - Skid buffers (4 per chip, 16 total): registered tready at every
--     AXI-Stream boundary for timing closure
--   - drain_done propagated via tuser[7] through decode_i → event_bld → cell_bld
--   - cell_builder ping-pong: collect(shot N+1) overlaps output(shot N)
--
-- Command arbitration:
--   - Demux gates on both raw s_cmd_start AND s_cmd_start_accepted
--   - Reg demux checks target chip busy + cfg_write collision
--   - cfg_write gated on full pipeline idle (same as start accept)
--   - CSR pending latches cleared on stop/soft_reset
--   - i_cmd_start_accepted fed back to CSR for laser override clear
--   - s_reg_outstanding_r included in status.busy
--
--   Includes glue logic:
--     cfg_image override (CTL3→Reg5 StartOff1, CTL4→Reg7 HSDiv/MTimer)
--     Per-chip stop decode (echo_receiver → per-chip expected drain counts)
--     Face sequencer (shot counting, face/frame ID management)
--     Status aggregation (t_tdc_status assembly)
--     Timestamp counter (free-running cycle counter)
--     Error counter
--
-- Clock domains:
--   i_axis_aclk  : TDC processing / AXI-Stream domain (~200 MHz)
--   s_axi_aclk   : AXI4-Lite PS domain
--
-- SW contract — config freeze during busy:
--   chip_ctrl snapshots at cmd_start: drain_mode, n_drain_cap,
--   bus_clk_div, bus_ticks, max_range_clks. Header snapshots at
--   face_start. Only stopdis_override is INTENTIONALLY LIVE (debug).
--   All commands (start, cfg_write, reg_read/write) are CDC idle gated
--   in CSR and pipeline idle gated in top. cfg_write requires full
--   pipeline idle (same conditions as start accept).
--
--   cfg_image freeze: additionally, cfg_image registers MUST NOT be
--   written while busy = '1'. In particular:
--     - Reg0 TRiseEn/TFallEn bits affect echo_receiver's stop pulse
--       counting upstream.  Changing mid-shot corrupts expected drain
--       counts.
--     - Reg6 LF threshold (Fill) is latched at IrFlag edge.
--   General rule: ALL cfg_image writes (cmd_cfg_write, cmd_reg_write)
--   require busy = '0'.
--
-- cmd_start: self-protecting — ignored while any chip_ctrl is busy or
--   assembler/header output is still valid.  Side effects (shot_seq
--   reset, sticky error clear) fire only on actual acceptance.
--
-- cmd_soft_reset: FULL PIPELINE RESET.  Resets the face sequencer,
--   chip_ctrl instances (→ ST_POWERUP), pending shot/face_start,
--   face_assembler (→ ST_IDLE + input/output flush), face FIFOs
--   (flushed), and header_inserter (→ ST_IDLE or ST_ABORT_DRAIN if
--   a beat is pending).  SW must wait for busy = '0' after soft_reset.
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
        -- FIXED FORMAT: p_stop_decode assumes exactly 4 chips x 8-bit slices
        -- (4b IFIFO2 | 4b IFIFO1). These generics exist for AXI-Stream width
        -- validation only; changing them does NOT make the decode generic.
        g_STOP_CNT_WIDTH  : natural := c_STOP_CNT_WIDTH;       -- FIXED at 4
        g_STOP_EVT_DWIDTH : natural := c_STOP_EVT_DATA_WIDTH   -- FIXED at 32
    );
    port (
        -- Processing / AXI-Stream clock and reset
        i_axis_aclk      : in  std_logic;
        i_axis_aresetn   : in  std_logic;

        -- AXI-Lite clock / reset (PS domain)
        s_axi_aclk       : in  std_logic;
        s_axi_aresetn    : in  std_logic;

        -- AXI4-Lite Slave (CSR, 9-bit address)
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

        -- laser_ctrl_result stream input (i_axis_aclk domain)
        i_lsr_tvalid     : in  std_logic;
        i_lsr_tdata      : in  std_logic_vector(31 downto 0);

        -- Shot trigger (from laser_ctrl, 1-clk pulse, i_axis_aclk domain)
        i_shot_start     : in  std_logic;

        -- Stop TDC pulse (from laser_ctrl, 1-clk pulse, i_axis_aclk domain)
        -- 한계도달거리 deadline. Error detection ONLY: stop_tdc↑ before
        -- IrFlag↑ in ST_CAPTURE → err_sequence.
        -- NOT used as a drain trigger (IrFlag is the sole drain trigger).
        i_stop_tdc        : in  std_logic;

        -- Stop event AXI Stream slave (from echo_receiver, i_axis_aclk domain)
        -- Per-chip packed counts:
        --   tdata[31:0] = [chip3(8b) | chip2(8b) | chip1(8b) | chip0(8b)]
        --   Each 8-bit chip slice: [IFIFO2(4b) | IFIFO1(4b)]
        -- tuser: same layout for falling-edge counts
        -- tkeep: reserved for future multi-beat extensions (not used by p_stop_decode)
        -- echo_receiver가 내부에서 누적하므로 tdata/tuser는 총 누적값.
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

        -- Interrupt
        o_irq            : out std_logic
    );
end entity tdc_gpx_top;

architecture rtl of tdc_gpx_top is

    -- =========================================================================
    -- Architecture-local array types (per-chip pipeline signals)
    -- =========================================================================
    type t_slv28_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    type t_slv4_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(3 downto 0);
    type t_raw_hit_array is array(0 to c_N_CHIPS - 1)
        of unsigned(c_RAW_HIT_WIDTH - 1 downto 0);
    type t_u2_array is array(0 to c_N_CHIPS - 1)
        of unsigned(1 downto 0);
    type t_u3_array is array(0 to c_N_CHIPS - 1)
        of unsigned(2 downto 0);
    type t_raw_event_array is array(0 to c_N_CHIPS - 1)
        of t_raw_event;
    type t_shot_seq_array is array(0 to c_N_CHIPS - 1)
        of unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);

    -- =========================================================================
    -- Zero constants
    -- =========================================================================
    constant C_ZEROS_CHIPS : std_logic_vector(c_N_CHIPS - 1 downto 0)
        := (others => '0');

    -- =========================================================================
    -- CSR outputs (i_axis_aclk domain)
    -- =========================================================================
    signal s_cfg            : t_tdc_cfg;
    signal s_cfg_face_r     : t_tdc_cfg;    -- snapshot at packet_start for header
    signal s_cfg_image_raw  : t_cfg_image;  -- raw from CSR
    signal s_cfg_image      : t_cfg_image;  -- effective (with overrides)
    signal s_bus_clk_div_8  : unsigned(7 downto 0);
    signal s_cmd_start      : std_logic;
    signal s_cmd_stop       : std_logic;
    signal s_cmd_soft_reset : std_logic;
    signal s_cmd_cfg_write  : std_logic;
    signal s_cmd_cfg_write_g : std_logic;  -- gated: all chips idle + no start
    signal s_cfg_write_top_pending_r : std_logic := '0';

    -- Individual register access (CSR -> chip_ctrl, per-chip)
    signal s_cmd_reg_read     : std_logic;
    signal s_cmd_reg_write    : std_logic;
    signal s_cmd_reg_addr     : std_logic_vector(3 downto 0);
    signal s_cmd_reg_wdata    : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal s_cmd_reg_chip     : unsigned(1 downto 0);  -- target chip for reg access
    signal s_cmd_reg_rdata    : t_slv28_array;
    signal s_cmd_reg_rvalid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_done     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    -- Per-chip gated reg access commands (chip demux)
    signal s_cmd_reg_read_g   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_write_g  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_outstanding_reg_chip_r : unsigned(1 downto 0) := (others => '0');
    signal s_reg_outstanding_r      : std_logic := '0';
    signal s_status           : t_tdc_status := c_TDC_STATUS_INIT;

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
    signal s_bus_busy        : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: bus_phy → chip_ctrl AXI-Stream (response)
    type t_slv32_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(31 downto 0);
    type t_slv8_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(7 downto 0);
    signal s_brsp_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_brsp_axis_tdata  : t_slv32_array;
    signal s_brsp_axis_tkeep  : t_slv4_array;
    signal s_brsp_axis_tuser  : t_slv8_array;
    signal s_brsp_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: bus_phy synchronized status
    signal s_ef1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_ef2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_irflag_sync     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_errflag_sync    : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: chip_ctrl -> decode_i (AXI-Stream, 32b tdata + 8b tuser)
    signal s_raw_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_axis_tdata  : t_slv32_array;
    signal s_raw_axis_tuser  : t_slv8_array;
    signal s_raw_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_drain_done      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_shot_seq   : t_shot_seq_array;
    signal s_chip_busy       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_tick_en         : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: skid buffer outputs (bus_phy → chip_ctrl)
    signal s_brsp_sk_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_brsp_sk_tdata   : t_slv32_array;
    signal s_brsp_sk_tuser   : t_slv8_array;
    signal s_brsp_sk_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: skid buffer outputs (chip_ctrl → decode_i)
    signal s_raw_sk_tvalid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_sk_tdata    : t_slv32_array;
    signal s_raw_sk_tuser    : t_slv8_array;
    signal s_raw_sk_tready   : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: skid buffer outputs (decode_i → raw_event_builder)
    signal s_dec_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_dec_axis_tdata  : t_slv32_array;
    signal s_dec_axis_tuser  : t_slv8_array;
    signal s_dec_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_dec_sk_tvalid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_dec_sk_tdata    : t_slv32_array;
    signal s_dec_sk_tuser    : t_slv8_array;
    signal s_dec_sk_tready   : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: skid buffer outputs (raw_event_builder → cell_builder)
    type t_slv16_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(15 downto 0);
    signal s_evt_axis_tvalid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_evt_axis_tdata  : t_slv32_array;
    signal s_evt_axis_tuser  : t_slv16_array;
    signal s_evt_axis_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_evt_sk_tvalid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_evt_sk_tdata    : t_slv32_array;
    signal s_evt_sk_tuser    : t_slv16_array;
    signal s_evt_sk_tready   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_stop_id_error   : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Slope demux: split raw_event_valid by slope bit
    -- slope='1' → rising pipeline (existing), slope='0' → falling pipeline (new)
    signal s_raw_evt_rise_valid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_evt_fall_valid : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Output-width cell data array type
    type t_out_tdata_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);

    -- Per-chip: cell_builder (rising) -> face_assembler (AXI-Stream)
    signal s_cell_tdata      : t_out_tdata_array;
    signal s_cell_tvalid     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_tlast      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_tready     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_slice_done      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hit_dropped     : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: cell_builder (falling) -> face_assembler_fall (AXI-Stream)
    signal s_cell_fall_tdata   : t_out_tdata_array;
    signal s_cell_fall_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tlast   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_slice_fall_done   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hit_fall_dropped  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- face_assembler (rising) -> header_inserter (AXI-Stream)
    -- =========================================================================
    signal s_face_tdata      : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_face_tvalid     : std_logic;
    signal s_face_tlast      : std_logic;
    signal s_face_tready     : std_logic;
    signal s_row_done        : std_logic;
    signal s_chip_error_flags: std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- face_assembler_fall (falling) -> header_inserter_fall (AXI-Stream)
    -- =========================================================================
    signal s_face_fall_tdata   : std_logic_vector(g_OUTPUT_WIDTH - 1 downto 0);
    signal s_face_fall_tvalid  : std_logic;
    signal s_face_fall_tlast   : std_logic;
    signal s_face_fall_tready  : std_logic;

    -- Buffered face → header (after sync FIFO)
    signal s_face_buf_tdata      : std_logic_vector(g_OUTPUT_WIDTH downto 0);  -- tdata & tlast
    signal s_face_buf_tvalid     : std_logic;
    signal s_face_buf_tready     : std_logic;
    signal s_face_fall_buf_tdata : std_logic_vector(g_OUTPUT_WIDTH downto 0);
    signal s_face_fall_buf_tvalid: std_logic;
    signal s_face_fall_buf_tready: std_logic;
    signal s_row_fall_done     : std_logic;
    signal s_chip_fall_error   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_shot_fall_overrun : std_logic;
    signal s_frame_fall_done   : std_logic;

    -- =========================================================================
    -- VDMA line geometry (computed once, shared with header_inserter & VDMA)
    -- =========================================================================
    signal s_rows_per_face_r : unsigned(15 downto 0) := (others => '0');
    signal s_hsize_bytes_r   : unsigned(15 downto 0) := (others => '0');

    -- =========================================================================
    -- Face sequencer
    -- =========================================================================
    type t_face_state is (ST_IDLE, ST_WAIT_SHOT, ST_IN_FACE);
    signal s_face_state_r    : t_face_state := ST_IDLE;
    signal s_face_id_r       : unsigned(7 downto 0) := (others => '0');
    signal s_frame_id_r      : unsigned(31 downto 0) := (others => '0');
    signal s_frame_done        : std_logic;  -- rising pipeline
    signal s_frame_done_both   : std_logic;  -- combined rise+fall
    signal s_face_closing      : std_logic;  -- either pipeline done → block mid-face shots
    signal s_frame_rise_done_r : std_logic := '0';
    signal s_frame_fall_done_r : std_logic := '0';

    -- =========================================================================
    -- Gated shot_start: only active when face sequencer is in run state.
    -- Prevents downstream FSMs from reacting to stray shot pulses before
    -- cmd_start or after cmd_stop.
    -- =========================================================================
    signal s_shot_start_gated    : std_logic;
    signal s_shot_pending_r      : std_logic := '0';  -- registered accept decision
    signal s_face_start_gated    : std_logic;          -- face_start_r killed by stop/reset
    signal s_shot_start_per_chip : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_global_shot_seq_r   : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0) := (others => '0');
    signal s_cmd_start_accepted  : std_logic := '0';  -- 1-clk pulse: start actually accepted
    signal s_shot_deferred_r     : std_logic := '0';  -- shot arrived during face-close window
    signal s_shot_drop_cnt_r     : unsigned(15 downto 0) := (others => '0');
    signal s_frame_abort_cnt_r   : unsigned(15 downto 0) := (others => '0');

    -- =========================================================================
    -- Timestamp (free-running cycle counter, i_axis_aclk domain)
    -- =========================================================================
    signal s_timestamp_r     : unsigned(63 downto 0) := (others => '0');

    -- =========================================================================
    -- Per-chip error signals from chip_ctrl (pulse, sticky in status)
    -- =========================================================================
    signal s_err_drain_timeout : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_sequence      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_err_drain_to_sticky_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal s_err_seq_sticky_r      : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Unified error mask: physical ErrFlag OR assembler timeout (blank insert)
    -- Single source for header, status, and error counter.
    -- =========================================================================
    signal s_chip_error_merged : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Error counter (edge-detected: counts error events, not level duration)
    -- =========================================================================
    signal s_error_count_r       : unsigned(31 downto 0) := (others => '0');
    signal s_chip_error_prev_r   : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Face-start latched config: snapshot at face_start for downstream modules
    -- that must see a stable config throughout the face.
    -- =========================================================================
    signal s_face_stops_per_chip_r : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_face_cols_per_face_r  : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal s_face_n_faces_r        : unsigned(3 downto 0)  := to_unsigned(1, 4);
    signal s_face_active_mask_r    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');

    -- =========================================================================
    -- Face boundary signals:
    --   s_packet_start    : combinational, fires ONLY when p_face_seq actually
    --                       accepts the shot (ST_WAIT_SHOT + shot_start + NOT
    --                       cmd_stop). Used by p_geometry and p_face_cfg_latch
    --                       to latch config at this edge.
    --   s_face_start_r    : 1-cycle delayed registered version of packet_start.
    --                       Used by header_inserter and face_assembler so they
    --                       see the STABLE latched values (VHDL signal semantics:
    --                       registers updated at edge N are visible at edge N+1).
    --   s_shot_start_gated: accepted-shot pulse. ALL downstream shot_start
    --                       signals go through this single gate.
    --                       First shot: delayed 1 cycle (s_face_start_r).
    --                       Mid-face shots: gated by s_face_active_r (set 1 cycle
    --                       after face_start, cleared on frame_done/cmd_stop).
    --                       This prevents raw i_shot_start from leaking through
    --                       on face-close boundaries (frame_done_both/cmd_stop
    --                       + shot_start same cycle race).
    --   s_face_active_r   : registered flag, '1' during accepted face period.
    --                       Set at s_face_start_r (1 cycle after packet_start).
    --                       Cleared on frame_done_both or cmd_stop.
    --                       Mid-face shots only pass when this flag is '1'.
    -- =========================================================================
    signal s_packet_start   : std_logic;
    signal s_face_start_r   : std_logic := '0';
    signal s_face_active_r  : std_logic := '0';

    -- =========================================================================
    -- Shot overrun: from face_assembler (real truncation), not face_seq
    -- =========================================================================
    signal s_shot_overrun         : std_logic;
    signal s_shot_overrun_r       : std_logic := '0';
    signal s_face_state_idle      : std_logic;
    signal s_face_abort           : std_logic;  -- rise assembler face abort pulse
    signal s_face_fall_abort      : std_logic;  -- fall assembler face abort pulse
    signal s_face_asm_idle        : std_logic;  -- rise assembler idle
    signal s_face_asm_fall_idle   : std_logic;  -- fall assembler idle
    signal s_hdr_draining         : std_logic;  -- rise header in ST_DRAIN_LAST
    signal s_hdr_fall_draining    : std_logic;  -- fall header in ST_DRAIN_LAST
    signal s_pipeline_abort       : std_logic;  -- one-shot: any overrun + stop + reset
    signal s_hdr_abort            : std_logic;  -- alias for readability (= s_pipeline_abort)
    signal s_hdr_fall_abort       : std_logic;  -- alias for readability (= s_pipeline_abort)
    signal s_hdr_idle             : std_logic;  -- rise header idle
    signal s_hdr_fall_idle        : std_logic;  -- fall header idle
    signal s_face_shot_cnt_r      : unsigned(15 downto 0) := (others => '0');
    signal s_all_shots_fired      : std_logic;  -- all shots for this face accepted

    -- =========================================================================
    -- Stop event stream: per-IFIFO expected drain counts
    -- AXI Stream slave: always ready (tready='1').
    -- echo_receiver가 누적 카운터를 관리하므로, 여기서는 tvalid 시 overwrite.
    -- tdata/tuser = N stops x g_STOP_CNT_WIDTH bits contiguous.
    -- Per-stop counts → per-IFIFO sum (rise+fall):
    --   IFIFO1 = stops 0~3, IFIFO2 = stops 4~7.
    -- =========================================================================
    -- Derived constants from generics
    constant c_STOP_TOTAL_BITS  : natural := c_MAX_STOPS_PER_CHIP * g_STOP_CNT_WIDTH;
    constant c_STOP_VALID_BYTES : natural :=
        fn_stop_evt_valid_bytes(c_MAX_STOPS_PER_CHIP, g_STOP_CNT_WIDTH);
    constant c_STOP_TKEEP_MASK  : std_logic_vector(g_STOP_EVT_DWIDTH/8 - 1 downto 0) :=
        fn_stop_evt_tkeep(c_MAX_STOPS_PER_CHIP, g_STOP_CNT_WIDTH, g_STOP_EVT_DWIDTH/8);

    -- IFIFO boundary: stops 0..IFIFO_SPLIT-1 = IFIFO1, IFIFO_SPLIT..7 = IFIFO2
    constant c_IFIFO_SPLIT : natural := c_MAX_STOPS_PER_CHIP / 2;  -- 4

    -- Per-chip, per-IFIFO expected drain counts.
    -- Decoded from echo_receiver stop event stream:
    --   tdata/tuser 32-bit: [chip3(8b) | chip2(8b) | chip1(8b) | chip0(8b)]
    --   Each 8-bit:         [IFIFO2(4b) | IFIFO1(4b)]
    --   tdata = rising counts, tuser = falling counts.
    -- Per-chip expected = rise_count + fall_count for that IFIFO.
    -- t_expected_array: now in tdc_gpx_pkg
    signal s_expected_ififo1 : t_expected_array := (others => (others => '0'));
    signal s_expected_ififo2 : t_expected_array := (others => (others => '0'));

begin

    s_bus_clk_div_8 <= resize(s_cfg.bus_clk_div, 8);

    -- =========================================================================
    -- cfg_image override: enforce t_tdc_cfg → TDC-GPX register dependencies
    --   Reg5[17:0]  ← s_cfg.start_off1  (CTL3)
    --   Reg5[23]    ← '1' (MasterAluTrig) — HARDWARE FORCED
    --                  AluTrigger pin must trigger master reset to clear
    --                  IFIFOs + ALU state between shots and after overrun.
    --                  Without this, AluTrigger pulse has no effect and
    --                  TDC-GPX internal state carries over to next shot.
    --   Reg5[24]    ← '0' (PartialAluTrig) — HARDWARE FORCED
    --                  Partial reset would leave IFIFOs dirty; must be off.
    --   Reg7[31:0]  ← s_cfg.cfg_reg7    (CTL4)
    -- Other registers pass through from CSR raw image unchanged.
    -- =========================================================================
    -- =========================================================================
    -- [0] Stop decode + cfg_image override (extracted module)
    -- =========================================================================
    u_stop_decode : entity work.tdc_gpx_stop_decode
        generic map (g_STOP_EVT_DWIDTH => g_STOP_EVT_DWIDTH)
        port map (
            i_clk              => i_axis_aclk,
            i_rst_n            => i_axis_aresetn,
            i_stop_evt_tvalid  => i_stop_evt_tvalid,
            i_stop_evt_tdata   => i_stop_evt_tdata,
            i_stop_evt_tuser   => i_stop_evt_tuser,
            o_stop_evt_tready  => o_stop_evt_tready,
            i_shot_start_gated => s_shot_start_gated,
            o_expected_ififo1  => s_expected_ififo1,
            o_expected_ififo2  => s_expected_ififo2,
            i_cfg              => s_cfg,
            i_cfg_image_raw    => s_cfg_image_raw,
            o_cfg_image        => s_cfg_image
        );

    -- =========================================================================
    -- Generic parameter validation (elaboration-time)
    -- =========================================================================
    assert c_STOP_TOTAL_BITS <= g_STOP_EVT_DWIDTH
        report "tdc_gpx_top: g_STOP_EVT_DWIDTH (" & integer'image(g_STOP_EVT_DWIDTH)
             & ") too narrow for " & integer'image(c_MAX_STOPS_PER_CHIP)
             & " stops x" & integer'image(g_STOP_CNT_WIDTH) & " bits = "
             & integer'image(c_STOP_TOTAL_BITS) & " bits"
        severity failure;

    assert g_STOP_EVT_DWIDTH mod 8 = 0
        report "tdc_gpx_top: g_STOP_EVT_DWIDTH must be a multiple of 8"
        severity failure;

    assert g_STOP_CNT_WIDTH = 4
        report "tdc_gpx_top: p_stop_decode hardcodes 4-bit per IFIFO count"
        severity failure;

    -- (stop decode + cfg_image override: moved to u_stop_decode above)

    -- (packet_start, face_start_delay, shot gating, pipeline abort:
    --  all now in u_face_seq module)

    -- =========================================================================
    -- Chip error merged (kept in top as simple concurrent logic)
    -- =========================================================================
    s_chip_error_merged <= (s_errflag_sync or s_chip_error_flags or s_chip_fall_error)
                           and s_face_active_mask_r;

    s_hdr_abort      <= s_pipeline_abort;
    s_hdr_fall_abort <= s_pipeline_abort;
    --   Guards against cmd_stop + shot_start same-cycle race:
    -- (All shot/face logic now in u_face_seq)

    -- =========================================================================
    -- [1] CSR instance (AXI-Lite + CDC)
    -- =========================================================================
    u_csr : entity work.tdc_gpx_csr
        generic map (
            g_HW_VERSION     => g_HW_VERSION
        )
        port map (
            s_axi_aclk       => s_axi_aclk,
            s_axi_aresetn    => s_axi_aresetn,
            s_axi_awvalid    => s_axi_awvalid,
            s_axi_awready    => s_axi_awready,
            s_axi_awaddr     => s_axi_awaddr,
            s_axi_awprot     => s_axi_awprot,
            s_axi_wvalid     => s_axi_wvalid,
            s_axi_wready     => s_axi_wready,
            s_axi_wdata      => s_axi_wdata,
            s_axi_wstrb      => s_axi_wstrb,
            s_axi_bvalid     => s_axi_bvalid,
            s_axi_bready     => s_axi_bready,
            s_axi_bresp      => s_axi_bresp,
            s_axi_arvalid    => s_axi_arvalid,
            s_axi_arready    => s_axi_arready,
            s_axi_araddr     => s_axi_araddr,
            s_axi_arprot     => s_axi_arprot,
            s_axi_rvalid     => s_axi_rvalid,
            s_axi_rready     => s_axi_rready,
            s_axi_rdata      => s_axi_rdata,
            s_axi_rresp      => s_axi_rresp,
            i_axis_aclk      => i_axis_aclk,
            i_axis_aresetn   => i_axis_aresetn,
            i_lsr_tvalid     => i_lsr_tvalid,
            i_lsr_tdata      => i_lsr_tdata,
            o_cfg            => s_cfg,
            o_cfg_image      => s_cfg_image_raw,
            o_cmd_start      => s_cmd_start,
            i_cmd_start_accepted => s_cmd_start_accepted,
            o_cmd_stop       => s_cmd_stop,
            o_cmd_soft_reset => s_cmd_soft_reset,
            o_cmd_cfg_write  => s_cmd_cfg_write,
            o_cmd_reg_read   => s_cmd_reg_read,
            o_cmd_reg_write  => s_cmd_reg_write,
            o_cmd_reg_addr   => s_cmd_reg_addr,
            o_cmd_reg_chip   => s_cmd_reg_chip,
            i_cmd_reg_rdata  => s_cmd_reg_rdata(to_integer(s_outstanding_reg_chip_r)),
            i_cmd_reg_rvalid => s_cmd_reg_rvalid(to_integer(s_outstanding_reg_chip_r)),
            i_status            => s_status,
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            o_irq            => o_irq
        );

    -- =========================================================================
    -- [1a] Command arbitration (extracted module)
    -- =========================================================================
    u_cmd_arb : entity work.tdc_gpx_cmd_arb
        port map (
            i_clk                => i_axis_aclk,
            i_rst_n              => i_axis_aresetn,
            i_cmd_start          => s_cmd_start,
            i_cmd_start_accepted => s_cmd_start_accepted,
            i_cmd_stop           => s_cmd_stop,
            i_cmd_soft_reset     => s_cmd_soft_reset,
            i_cmd_cfg_write      => s_cmd_cfg_write,
            i_cmd_reg_read       => s_cmd_reg_read,
            i_cmd_reg_write      => s_cmd_reg_write,
            i_cmd_reg_chip       => s_cmd_reg_chip,
            i_chip_busy          => s_chip_busy,
            i_face_asm_idle      => s_face_asm_idle,
            i_face_asm_fall_idle => s_face_asm_fall_idle,
            i_hdr_idle           => s_hdr_idle,
            i_hdr_fall_idle      => s_hdr_fall_idle,
            i_cmd_reg_done       => s_cmd_reg_done,
            o_cmd_cfg_write_g    => s_cmd_cfg_write_g,
            o_cmd_reg_read_g     => s_cmd_reg_read_g,
            o_cmd_reg_write_g    => s_cmd_reg_write_g,
            o_reg_outstanding    => s_reg_outstanding_r,
            o_outstanding_chip   => s_outstanding_reg_chip_r
        );

    -- Write data: from cfg_image indexed by target register address.
    -- SW contract: cfg_image CDC must be idle (all handshakes complete)
    -- before reg_write is triggered.  This is guaranteed when SW follows
    -- the "busy='0' required" rule, since busy includes chip_busy which
    -- only goes to '0' after cfg_write completes.
    s_cmd_reg_wdata <= s_cfg_image(to_integer(unsigned(s_cmd_reg_addr)))(c_TDC_BUS_WIDTH - 1 downto 0);

    -- =========================================================================
    -- [2] Per-chip pipeline (generate x4)
    --     bus_phy -> chip_ctrl -> decode_i -> raw_event_builder -> cell_builder
    -- =========================================================================
    gen_chip : for i in 0 to c_N_CHIPS - 1 generate

        -- ----- bus_phy: physical bus timing FSM + IOBUF + 2-FF sync -----
        u_bus_phy : entity work.tdc_gpx_bus_phy
            port map (
                i_clk           => i_axis_aclk,
                i_rst_n         => i_axis_aresetn,
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
                i_m_axis_tready => s_brsp_axis_tready(i),  -- from skid
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
                o_errflag_sync  => s_errflag_sync(i)
            );

        -- ----- skid buffer: bus_phy → chip_ctrl (32b tdata + 8b tuser = 40b) -----
        u_sk_brsp : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => 40)
            port map (
                i_clk    => i_axis_aclk,
                i_rst_n  => i_axis_aresetn,
                i_flush  => '0',
                i_s_valid => s_brsp_axis_tvalid(i),
                o_s_ready => s_brsp_axis_tready(i),
                i_s_data  => s_brsp_axis_tdata(i) & s_brsp_axis_tuser(i),
                o_m_valid => s_brsp_sk_tvalid(i),
                i_m_ready => s_brsp_sk_tready(i),
                o_m_data(39 downto 8) => s_brsp_sk_tdata(i),
                o_m_data(7 downto 0)  => s_brsp_sk_tuser(i)
            );

        -- ----- chip_ctrl: single-shot FSM (powerup/cfg/arm/capture/drain) -----
        u_chip_ctrl : entity work.tdc_gpx_chip_ctrl
            generic map (
                g_CHIP_ID        => i,
                g_POWERUP_CLKS   => g_POWERUP_CLKS,
                g_RECOVERY_CLKS  => g_RECOVERY_CLKS,
                g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS
            )
            port map (
                i_clk               => i_axis_aclk,
                i_rst_n             => i_axis_aresetn,
                i_cfg               => s_cfg,
                i_cfg_image         => s_cfg_image,
                i_cmd_start         => s_cmd_start_accepted,
                i_cmd_stop          => s_cmd_stop,
                i_cmd_soft_reset    => s_cmd_soft_reset,
                i_cmd_cfg_write     => s_cmd_cfg_write_g,
                i_cmd_reg_read      => s_cmd_reg_read_g(i),
                i_cmd_reg_write     => s_cmd_reg_write_g(i),
                i_cmd_reg_addr      => s_cmd_reg_addr,
                i_cmd_reg_wdata     => s_cmd_reg_wdata,
                o_cmd_reg_rdata     => s_cmd_reg_rdata(i),
                o_cmd_reg_rvalid    => s_cmd_reg_rvalid(i),
                o_cmd_reg_done      => s_cmd_reg_done(i),
                i_shot_start        => s_shot_start_per_chip(i),
                i_max_range_clks    => s_cfg.max_range_clks,
                i_stop_tdc          => i_stop_tdc,
                i_expected_ififo1   => s_expected_ififo1(i),
                i_expected_ififo2   => s_expected_ififo2(i),
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
                o_shot_seq          => s_chip_shot_seq(i),
                o_busy              => s_chip_busy(i),
                o_err_drain_timeout => s_err_drain_timeout(i),
                o_err_sequence      => s_err_sequence(i)
            );

        -- ----- skid buffer: chip_ctrl → decode_i (32b + 8b = 40b) -----
        u_sk_raw : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => 40)
            port map (
                i_clk    => i_axis_aclk,
                i_rst_n  => i_axis_aresetn,
                i_flush  => '0',
                i_s_valid => s_raw_axis_tvalid(i),
                o_s_ready => s_raw_axis_tready(i),
                i_s_data  => s_raw_axis_tdata(i) & s_raw_axis_tuser(i),
                o_m_valid => s_raw_sk_tvalid(i),
                i_m_ready => s_raw_sk_tready(i),
                o_m_data(39 downto 8) => s_raw_sk_tdata(i),
                o_m_data(7 downto 0)  => s_raw_sk_tuser(i)
            );

        -- ----- decode_i: registered 28-bit I-Mode field extraction -----
        u_decode : entity work.tdc_gpx_decode_i
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_s_axis_tvalid   => s_raw_sk_tvalid(i),
                i_s_axis_tdata    => s_raw_sk_tdata(i),
                i_s_axis_tuser    => s_raw_sk_tuser(i),
                o_s_axis_tready   => s_raw_sk_tready(i),
                o_m_axis_tvalid   => s_dec_axis_tvalid(i),
                o_m_axis_tdata    => s_dec_axis_tdata(i),
                o_m_axis_tuser    => s_dec_axis_tuser(i),
                i_m_axis_tready   => s_dec_axis_tready(i)
            );

        -- ----- skid buffer: decode_i → raw_event_builder (32b + 8b = 40b) -----
        u_sk_dec : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => 40)
            port map (
                i_clk    => i_axis_aclk,
                i_rst_n  => i_axis_aresetn,
                i_flush  => '0',
                i_s_valid => s_dec_axis_tvalid(i),
                o_s_ready => s_dec_axis_tready(i),
                i_s_data  => s_dec_axis_tdata(i) & s_dec_axis_tuser(i),
                o_m_valid => s_dec_sk_tvalid(i),
                i_m_ready => s_dec_sk_tready(i),
                o_m_data(39 downto 8) => s_dec_sk_tdata(i),
                o_m_data(7 downto 0)  => s_dec_sk_tuser(i)
            );

        -- ----- raw_event_builder: enrich with chip/shot context -----
        u_event_bld : entity work.tdc_gpx_raw_event_builder
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_s_axis_tvalid   => s_dec_sk_tvalid(i),
                i_s_axis_tdata    => s_dec_sk_tdata(i),
                i_s_axis_tuser    => s_dec_sk_tuser(i),
                o_s_axis_tready   => s_dec_sk_tready(i),
                i_chip_id         => to_unsigned(i, 2),
                i_shot_seq        => s_chip_shot_seq(i),
                i_stops_per_chip  => s_face_stops_per_chip_r,
                o_m_axis_tvalid   => s_evt_axis_tvalid(i),
                o_m_axis_tdata    => s_evt_axis_tdata(i),
                o_m_axis_tuser    => s_evt_axis_tuser(i),
                i_m_axis_tready   => s_evt_axis_tready(i),
                o_stop_id_error   => s_stop_id_error(i)
            );

        -- ----- skid buffer: raw_event_builder → cell_builder (32b + 16b = 48b) -----
        u_sk_evt : entity work.tdc_gpx_skid_buffer
            generic map (g_DATA_WIDTH => 48)
            port map (
                i_clk    => i_axis_aclk,
                i_rst_n  => i_axis_aresetn,
                i_flush  => '0',
                i_s_valid => s_evt_axis_tvalid(i),
                o_s_ready => s_evt_axis_tready(i),
                i_s_data  => s_evt_axis_tdata(i) & s_evt_axis_tuser(i),
                o_m_valid => s_evt_sk_tvalid(i),
                i_m_ready => '1',   -- cell_builder always accepts in ST_COLLECT
                o_m_data(47 downto 16) => s_evt_sk_tdata(i),
                o_m_data(15 downto 0)  => s_evt_sk_tuser(i)
            );

        -- ----- slope demux: split tvalid by slope bit (tuser[0]) -----
        -- drain_done (tuser[7]) control beat goes to BOTH pipelines
        -- Uses skid buffer output (s_evt_sk_*)
        s_raw_evt_rise_valid(i) <= s_evt_sk_tvalid(i)
                                   and (s_evt_sk_tuser(i)(0) or s_evt_sk_tuser(i)(7));
        s_raw_evt_fall_valid(i) <= s_evt_sk_tvalid(i)
                                   and (not s_evt_sk_tuser(i)(0) or s_evt_sk_tuser(i)(7));

        -- ----- cell_builder (rising): sparse events -> dense cell -> AXI-Stream -----
        u_cell_bld : entity work.tdc_gpx_cell_builder
            generic map (
                g_CHIP_ID     => i,
                g_TDATA_WIDTH => g_OUTPUT_WIDTH
            )
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_s_axis_tvalid   => s_raw_evt_rise_valid(i),
                i_s_axis_tdata    => s_evt_sk_tdata(i),
                i_s_axis_tuser    => s_evt_sk_tuser(i),
                o_s_axis_tready   => open,
                i_shot_start      => s_shot_start_per_chip(i),
                i_stops_per_chip  => s_face_stops_per_chip_r,
                i_max_hits_cfg    => s_cfg.max_hits_cfg,
                o_m_axis_tdata    => s_cell_tdata(i),
                o_m_axis_tvalid   => s_cell_tvalid(i),
                o_m_axis_tlast    => s_cell_tlast(i),
                i_m_axis_tready   => s_cell_tready(i),
                o_slice_done      => s_slice_done(i),
                o_hit_dropped_any => s_hit_dropped(i)
            );

        -- ----- cell_builder (falling): same raw_event, fall-filtered valid -----
        u_cell_bld_fall : entity work.tdc_gpx_cell_builder
            generic map (
                g_CHIP_ID     => i,
                g_TDATA_WIDTH => g_OUTPUT_WIDTH
            )
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_s_axis_tvalid   => s_raw_evt_fall_valid(i),
                i_s_axis_tdata    => s_evt_sk_tdata(i),
                i_s_axis_tuser    => s_evt_sk_tuser(i),
                o_s_axis_tready   => open,
                i_shot_start      => s_shot_start_per_chip(i),
                i_stops_per_chip  => s_face_stops_per_chip_r,
                i_max_hits_cfg    => s_cfg.max_hits_cfg,
                o_m_axis_tdata    => s_cell_fall_tdata(i),
                o_m_axis_tvalid   => s_cell_fall_tvalid(i),
                o_m_axis_tlast    => s_cell_fall_tlast(i),
                i_m_axis_tready   => s_cell_fall_tready(i),
                o_slice_done      => s_slice_fall_done(i),
                o_hit_dropped_any => s_hit_fall_dropped(i)
            );

    end generate gen_chip;

    -- =========================================================================
    -- [3] Face assembler (4 chip streams -> packed row)
    -- =========================================================================
    u_face_asm : entity work.tdc_gpx_face_assembler
        generic map (
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS,
            g_TDATA_WIDTH    => g_OUTPUT_WIDTH
        )
        port map (
            i_clk              => i_axis_aclk,
            i_rst_n            => i_axis_aresetn,
            i_s_axis_tdata_0   => s_cell_tdata(0),
            i_s_axis_tdata_1   => s_cell_tdata(1),
            i_s_axis_tdata_2   => s_cell_tdata(2),
            i_s_axis_tdata_3   => s_cell_tdata(3),
            i_s_axis_tvalid    => s_cell_tvalid,
            i_s_axis_tlast     => s_cell_tlast,
            o_s_axis_tready    => s_cell_tready,
            i_shot_start       => s_shot_start_gated,
            i_abort            => s_pipeline_abort,
            i_active_chip_mask => s_face_active_mask_r,
            i_stops_per_chip   => s_face_stops_per_chip_r,
            i_max_hits_cfg     => s_cfg.max_hits_cfg,
            i_max_scan_clks    => s_cfg.max_scan_clks,
            o_m_axis_tdata     => s_face_tdata,
            o_m_axis_tvalid    => s_face_tvalid,
            o_m_axis_tlast     => s_face_tlast,
            i_m_axis_tready    => s_face_tready,
            o_row_done         => s_row_done,
            o_chip_error_flags => s_chip_error_flags,
            o_shot_overrun     => s_shot_overrun,
            o_face_abort       => s_face_abort,
            o_idle             => s_face_asm_idle
        );

    -- =========================================================================
    -- [3f] Face assembler - FALLING pipeline (4 chip streams -> packed row)
    -- =========================================================================
    u_face_asm_fall : entity work.tdc_gpx_face_assembler
        generic map (
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS,
            g_TDATA_WIDTH    => g_OUTPUT_WIDTH
        )
        port map (
            i_clk              => i_axis_aclk,
            i_rst_n            => i_axis_aresetn,
            i_s_axis_tdata_0   => s_cell_fall_tdata(0),
            i_s_axis_tdata_1   => s_cell_fall_tdata(1),
            i_s_axis_tdata_2   => s_cell_fall_tdata(2),
            i_s_axis_tdata_3   => s_cell_fall_tdata(3),
            i_s_axis_tvalid    => s_cell_fall_tvalid,
            i_s_axis_tlast     => s_cell_fall_tlast,
            o_s_axis_tready    => s_cell_fall_tready,
            i_shot_start       => s_shot_start_gated,
            i_abort            => s_pipeline_abort,
            i_active_chip_mask => s_face_active_mask_r,
            i_stops_per_chip   => s_face_stops_per_chip_r,
            i_max_hits_cfg     => s_cfg.max_hits_cfg,
            i_max_scan_clks    => s_cfg.max_scan_clks,
            o_m_axis_tdata     => s_face_fall_tdata,
            o_m_axis_tvalid    => s_face_fall_tvalid,
            o_m_axis_tlast     => s_face_fall_tlast,
            i_m_axis_tready    => s_face_fall_tready,
            o_row_done         => s_row_fall_done,
            o_chip_error_flags => s_chip_fall_error,
            o_shot_overrun     => s_shot_fall_overrun,
            o_face_abort       => s_face_fall_abort,
            o_idle             => s_face_asm_fall_idle
        );

    -- =========================================================================
    -- [3.5] Sync FIFO: face_assembler → header_inserter (absorbs prefix stall)
    -- =========================================================================
    u_face_fifo : entity work.tdc_gpx_sync_fifo
        generic map (
            g_DATA_WIDTH => g_OUTPUT_WIDTH + 1,  -- tdata(31:0) & tlast(0) = 33 bits
            g_DEPTH      => 16,
            g_LOG2_DEPTH => 4,
            g_IN_REG     => false,
            g_OUT_REG    => false
        )
        port map (
            i_clk     => i_axis_aclk,
            i_rst_n   => i_axis_aresetn,
            i_flush   => s_hdr_abort,    -- flush on overrun, stop, or soft_reset
            i_s_valid => s_face_tvalid,
            o_s_ready => s_face_tready,
            i_s_data  => s_face_tdata & s_face_tlast,
            o_m_valid => s_face_buf_tvalid,
            i_m_ready => s_face_buf_tready,
            o_m_data  => s_face_buf_tdata
        );

    u_face_fall_fifo : entity work.tdc_gpx_sync_fifo
        generic map (
            g_DATA_WIDTH => g_OUTPUT_WIDTH + 1,
            g_DEPTH      => 16,
            g_LOG2_DEPTH => 4,
            g_IN_REG     => false,
            g_OUT_REG    => false
        )
        port map (
            i_clk     => i_axis_aclk,
            i_rst_n   => i_axis_aresetn,
            i_flush   => s_hdr_fall_abort,   -- flush on overrun, stop, or soft_reset
            i_s_valid => s_face_fall_tvalid,
            o_s_ready => s_face_fall_tready,
            i_s_data  => s_face_fall_tdata & s_face_fall_tlast,
            o_m_valid => s_face_fall_buf_tvalid,
            i_m_ready => s_face_fall_buf_tready,
            o_m_data  => s_face_fall_buf_tdata
        );

    -- =========================================================================
    -- [4] Header inserter - RISING pipeline (header + SOF/EOL -> VDMA frame)
    -- =========================================================================
    u_header : entity work.tdc_gpx_header_inserter
        generic map (g_TDATA_WIDTH => g_OUTPUT_WIDTH)
        port map (
            i_clk               => i_axis_aclk,
            i_rst_n             => i_axis_aresetn,
            i_face_start        => s_face_start_gated,
            i_face_abort        => s_hdr_abort,     -- overrun + stop + reset
            i_cfg               => s_cfg_face_r,   -- face-start snapshot
            i_vdma_frame_id     => s_frame_id_r,
            i_face_id           => s_face_id_r,
            i_shot_seq_start    => s_global_shot_seq_r,
            i_timestamp_ns      => s_timestamp_r,
            i_chip_error_mask   => s_chip_error_merged,
            i_chip_error_cnt    => std_logic_vector(s_error_count_r),
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            i_rows_per_face     => s_rows_per_face_r,
            i_s_axis_tdata      => s_face_buf_tdata(g_OUTPUT_WIDTH downto 1),
            i_s_axis_tvalid     => s_face_buf_tvalid,
            i_s_axis_tlast      => s_face_buf_tdata(0),
            o_s_axis_tready     => s_face_buf_tready,
            o_m_axis_tdata      => o_m_axis_tdata,
            o_m_axis_tvalid     => o_m_axis_tvalid,
            o_m_axis_tlast      => o_m_axis_tlast,
            o_m_axis_tuser      => o_m_axis_tuser,
            i_m_axis_tready     => i_m_axis_tready,
            o_frame_done        => s_frame_done,
            o_draining          => s_hdr_draining,
            o_last_line         => open,
            o_idle              => s_hdr_idle
        );

    -- =========================================================================
    -- [4f] Header inserter - FALLING pipeline (header + SOF/EOL -> VDMA frame)
    -- =========================================================================
    u_header_fall : entity work.tdc_gpx_header_inserter
        generic map (g_TDATA_WIDTH => g_OUTPUT_WIDTH)
        port map (
            i_clk               => i_axis_aclk,
            i_rst_n             => i_axis_aresetn,
            i_face_start        => s_face_start_gated,
            i_face_abort        => s_hdr_fall_abort,  -- overrun + stop + reset
            i_cfg               => s_cfg_face_r,   -- face-start snapshot
            i_vdma_frame_id     => s_frame_id_r,
            i_face_id           => s_face_id_r,
            i_shot_seq_start    => s_global_shot_seq_r,
            i_timestamp_ns      => s_timestamp_r,
            i_chip_error_mask   => s_chip_error_merged,
            i_chip_error_cnt    => std_logic_vector(s_error_count_r),
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            i_rows_per_face     => s_rows_per_face_r,
            i_s_axis_tdata      => s_face_fall_buf_tdata(g_OUTPUT_WIDTH downto 1),
            i_s_axis_tvalid     => s_face_fall_buf_tvalid,
            i_s_axis_tlast      => s_face_fall_buf_tdata(0),
            o_s_axis_tready     => s_face_fall_buf_tready,
            o_m_axis_tdata      => o_m_axis_fall_tdata,
            o_m_axis_tvalid     => o_m_axis_fall_tvalid,
            o_m_axis_tlast      => o_m_axis_fall_tlast,
            o_m_axis_tuser      => o_m_axis_fall_tuser,
            i_m_axis_tready     => i_m_axis_fall_tready,
            o_frame_done        => s_frame_fall_done,
            o_draining          => s_hdr_fall_draining,
            o_last_line         => open,
            o_idle              => s_hdr_fall_idle
        );

    -- =========================================================================
    -- [5] VDMA line geometry
    --   Computed from CSR config, shared with header_inserter and VDMA.
    --   rows_per_face = active_chips × stops_per_chip (clamp >= 2)
    --   hsize_bytes   = (data_beats + c_HDR_PREFIX_BEATS) × TDATA_BYTES
    --   Latched at s_packet_start (combinational, guarded by cmd_stop)
    --   so that the registered values are stable by s_face_start_r (+1 cycle).
    -- =========================================================================
    -- =========================================================================
    -- [5-6] Face sequencer (extracted module)
    -- =========================================================================
    u_face_seq : entity work.tdc_gpx_face_seq
        port map (
            i_clk                  => i_axis_aclk,
            i_rst_n                => i_axis_aresetn,
            i_cmd_start            => s_cmd_start,
            i_cmd_stop             => s_cmd_stop,
            i_cmd_soft_reset       => s_cmd_soft_reset,
            i_chip_busy            => s_chip_busy,
            i_reg_outstanding      => s_reg_outstanding_r,
            i_face_asm_idle        => s_face_asm_idle,
            i_face_asm_fall_idle   => s_face_asm_fall_idle,
            i_hdr_idle             => s_hdr_idle,
            i_hdr_fall_idle        => s_hdr_fall_idle,
            i_face_tvalid          => s_face_tvalid,
            i_face_fall_tvalid     => s_face_fall_tvalid,
            i_face_buf_tvalid      => s_face_buf_tvalid,
            i_face_fall_buf_tvalid => s_face_fall_buf_tvalid,
            i_m_axis_tvalid        => o_m_axis_tvalid,
            i_m_axis_fall_tvalid   => o_m_axis_fall_tvalid,
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
    -- [7-9] Status aggregation (extracted module)
    -- =========================================================================
    u_status_agg : entity work.tdc_gpx_status_agg
        port map (
            i_clk                  => i_axis_aclk,
            i_rst_n                => i_axis_aresetn,
            i_cmd_soft_reset       => s_cmd_soft_reset,
            i_cmd_start_accepted   => s_cmd_start_accepted,
            i_face_state_idle      => s_face_state_idle,
            i_chip_busy            => s_chip_busy,
            i_reg_outstanding      => s_reg_outstanding_r,
            i_face_asm_idle        => s_face_asm_idle,
            i_face_asm_fall_idle   => s_face_asm_fall_idle,
            i_hdr_idle             => s_hdr_idle,
            i_hdr_fall_idle        => s_hdr_fall_idle,
            i_face_tvalid          => s_face_tvalid,
            i_face_fall_tvalid     => s_face_fall_tvalid,
            i_face_buf_tvalid      => s_face_buf_tvalid,
            i_face_fall_buf_tvalid => s_face_fall_buf_tvalid,
            i_m_axis_tvalid        => o_m_axis_tvalid,
            i_m_axis_fall_tvalid   => o_m_axis_fall_tvalid,
            i_stop_id_error        => s_stop_id_error,
            i_hit_dropped          => s_hit_dropped,
            i_hit_fall_dropped     => s_hit_fall_dropped,
            i_err_drain_timeout    => s_err_drain_timeout,
            i_err_sequence         => s_err_sequence,
            i_chip_error_merged    => s_chip_error_merged,
            i_face_active_mask     => s_face_active_mask_r,
            i_shot_overrun         => s_shot_overrun_r,
            o_status               => s_status,
            o_timestamp            => s_timestamp_r,
            o_error_count          => s_error_count_r,
            o_err_drain_sticky     => s_err_drain_to_sticky_r,
            o_err_seq_sticky       => s_err_seq_sticky_r
        );

    -- Remaining status fields not in status_agg
    s_status.bin_mismatch        <= '0';
    s_status.chip_error_mask     <= s_chip_error_merged;
    s_status.drain_timeout_mask  <= s_err_drain_to_sticky_r;
    s_status.sequence_error_mask <= s_err_seq_sticky_r;
    s_status.shot_seq_current    <= s_global_shot_seq_r;
    s_status.vdma_frame_count    <= s_frame_id_r;
    s_status.error_count         <= s_error_count_r;
    s_status.shot_drop_count     <= s_shot_drop_cnt_r;
    s_status.frame_abort_count   <= s_frame_abort_cnt_r;

end architecture rtl;
