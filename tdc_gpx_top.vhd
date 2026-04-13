-- =============================================================================
-- tdc_gpx_top.vhd
-- TDC-GPX Controller - Top-level structural wrapper
-- =============================================================================
--
-- Purpose:
--   Instantiates and connects all TDC-GPX controller submodules:
--     CSR (AXI-Lite + CDC)
--     Per-chip pipeline x4: bus_phy → chip_ctrl → decode_i →
--                           raw_event_builder → cell_builder
--     face_assembler (4 chip inputs → packed row)
--     header_inserter (header + SOF/EOL → VDMA frame)
--
--   Includes glue logic:
--     cfg_image override (CTL3→Reg5 StartOff1, CTL4→Reg7 HSDiv/MTimer)
--     Per-chip stop decode (echo_receiver tdata/tuser → per-chip, per-IFIFO
--       expected drain counts; edge-enable gating done upstream)
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
--   The following CSR fields are latched at face_start only for the
--   data-path (active_chip_mask, stops_per_chip, rows_per_face).
--   Other fields (bus_clk_div, bus_ticks, drain_mode, n_drain_cap,
--   max_range_clks) are read LIVE by chip_ctrl / bus_phy.
--   SW MUST NOT write these CTL registers while o_status.busy = '1'.
--   Violating this can corrupt bus timing mid-transaction or change
--   drain policy mid-shot, leading to undefined behavior.
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
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_top is
    generic (
        g_HW_VERSION      : std_logic_vector(31 downto 0) := x"00010000";
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
        o_m_axis_tdata   : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid  : out std_logic;
        o_m_axis_tlast   : out std_logic;
        o_m_axis_tuser   : out std_logic_vector(0 downto 0);
        i_m_axis_tready  : in  std_logic;

        -- AXI-Stream master output: FALLING pipeline (to CDC FIFO / VDMA)
        o_m_axis_fall_tdata  : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
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
    signal s_bus_rsp_valid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_rsp_rdata   : t_slv28_array;
    signal s_bus_busy        : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: bus_phy synchronized status
    signal s_ef1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_ef2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf1_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_lf2_sync        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_irflag_sync     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_errflag_sync    : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: chip_ctrl -> decode_i
    signal s_raw_word        : t_slv28_array;
    signal s_raw_word_valid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_ififo_id        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_drain_done      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_chip_shot_seq   : t_shot_seq_array;
    signal s_chip_busy       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_tick_en         : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: decode_i -> raw_event_builder (combinational)
    signal s_dec_raw_hit     : t_raw_hit_array;
    signal s_dec_slope       : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_dec_cha_code    : t_u2_array;
    signal s_dec_stop_id     : t_u3_array;
    signal s_dec_valid       : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: raw_event_builder -> slope demux -> cell_builders
    signal s_raw_event       : t_raw_event_array;
    signal s_raw_event_valid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_stop_id_error   : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Slope demux: split raw_event_valid by slope bit
    -- slope='1' → rising pipeline (existing), slope='0' → falling pipeline (new)
    signal s_raw_evt_rise_valid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_raw_evt_fall_valid : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: cell_builder (rising) -> face_assembler (AXI-Stream)
    signal s_cell_tdata      : t_axis_tdata_array;
    signal s_cell_tvalid     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_tlast      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_tready     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_slice_done      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hit_dropped     : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: cell_builder (falling) -> face_assembler_fall (AXI-Stream)
    signal s_cell_fall_tdata   : t_axis_tdata_array;
    signal s_cell_fall_tvalid  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tlast   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_fall_tready  : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_slice_fall_done   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hit_fall_dropped  : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- face_assembler (rising) -> header_inserter (AXI-Stream)
    -- =========================================================================
    signal s_face_tdata      : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    signal s_face_tvalid     : std_logic;
    signal s_face_tlast      : std_logic;
    signal s_face_tready     : std_logic;
    signal s_row_done        : std_logic;
    signal s_chip_error_flags: std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- face_assembler_fall (falling) -> header_inserter_fall (AXI-Stream)
    -- =========================================================================
    signal s_face_fall_tdata   : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    signal s_face_fall_tvalid  : std_logic;
    signal s_face_fall_tlast   : std_logic;
    signal s_face_fall_tready  : std_logic;

    -- Buffered face → header (after sync FIFO)
    signal s_face_buf_tdata      : std_logic_vector(c_TDATA_WIDTH downto 0);  -- tdata & tlast
    signal s_face_buf_tvalid     : std_logic;
    signal s_face_buf_tready     : std_logic;
    signal s_face_fall_buf_tdata : std_logic_vector(c_TDATA_WIDTH downto 0);
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
    type t_expected_array is array(0 to c_N_CHIPS - 1) of unsigned(7 downto 0);
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
    p_cfg_image_override : process(s_cfg_image_raw, s_cfg)
        variable v_img : t_cfg_image;
    begin
        v_img := s_cfg_image_raw;
        -- Reg5: StartOff1 from CTL3
        v_img(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO)
            := std_logic_vector(s_cfg.start_off1);
        -- Reg5: force MasterAluTrig='1', PartialAluTrig='0'
        v_img(5)(c_REG5_MASTER_ALU_TRIG)  := '1';
        v_img(5)(c_REG5_PARTIAL_ALU_TRIG) := '0';
        -- Reg7: HSDiv/RefClkDiv/MTimer from CTL4
        v_img(7) := s_cfg.cfg_reg7;
        s_cfg_image <= v_img;
    end process;

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

    -- =========================================================================
    -- Stop event AXI Stream slave: always accept
    -- =========================================================================
    o_stop_evt_tready <= '1';

    -- =========================================================================
    -- Per-chip stop decode: extract IFIFO1/2 expected counts from echo_receiver.
    --   tdata/tuser 32-bit: [chip3(8b) | chip2(8b) | chip1(8b) | chip0(8b)]
    --   Each 8-bit slice: [IFIFO2(4b) | IFIFO1(4b)]
    --   tdata = rising edge counts, tuser = falling edge counts.
    --   Per-chip IFIFO expected = rise_slice + fall_slice.
    --   NOTE: edge-enable gating (TRiseEn/TFallEn) is handled upstream
    --   by echo_receiver — p_stop_decode sees already-gated values.
    --   i_stop_evt_tkeep: reserved for future multi-beat extensions.
    --   Cleared on shot_start (new measurement boundary).
    -- =========================================================================
    p_stop_decode : process(i_axis_aclk)
        variable v_lo : natural;
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                for i in 0 to c_N_CHIPS - 1 loop
                    s_expected_ififo1(i) <= (others => '0');
                    s_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif s_shot_start_gated = '1' then
                -- Shot boundary: clear for new measurement
                for i in 0 to c_N_CHIPS - 1 loop
                    s_expected_ififo1(i) <= (others => '0');
                    s_expected_ififo2(i) <= (others => '0');
                end loop;
            elsif i_stop_evt_tvalid = '1' then
                for i in 0 to c_N_CHIPS - 1 loop
                    v_lo := i * 8;
                    -- IFIFO1: lower 4 bits of each chip's 8-bit slice
                    s_expected_ififo1(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 3 downto v_lo)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 3 downto v_lo)), 8);
                    -- IFIFO2: upper 4 bits of each chip's 8-bit slice
                    s_expected_ififo2(i) <=
                        resize(unsigned(i_stop_evt_tdata(v_lo + 7 downto v_lo + 4)), 8)
                      + resize(unsigned(i_stop_evt_tuser(v_lo + 7 downto v_lo + 4)), 8);
                end loop;
            end if;
        end if;
    end process p_stop_decode;

    -- =========================================================================
    -- [0a] Combinational packet_start: accepted first-shot pulse.
    --   Fires ONLY when p_face_seq actually accepts the transition:
    --     ST_WAIT_SHOT + shot_start=1 + cmd_stop=0.
    --   Guards against cmd_stop + shot_start same-cycle race:
    --     p_face_seq gives cmd_stop priority → no ST_IN_FACE transition
    --     → packet_start must NOT fire.
    --   frame_done_both + shot_start same-cycle in ST_IN_FACE:
    --     p_face_seq closes face → ST_WAIT_SHOT; shot is not accepted this
    --     cycle (state is still ST_IN_FACE) → packet_start=0.
    --     Next cycle in ST_WAIT_SHOT: shot_start is gone (1-clk pulse) →
    --     waits for next shot_start cleanly.
    -- =========================================================================
    s_packet_start  <= '1'  when s_face_state_r = ST_WAIT_SHOT
                                and (i_shot_start = '1' or s_shot_deferred_r = '1')
                                and s_cmd_stop = '0'
                                and s_cmd_soft_reset = '0'
                                and s_hdr_idle = '1'
                                and s_hdr_fall_idle = '1'
                            else '0';

    -- =========================================================================
    -- [0b] face_start delay + face_active flag + registered shot acceptance.
    --
    --   s_face_start_r: 1-cycle delayed packet_start, used by header_inserter
    --     for config latching.  Timing: fires at N+1 so downstream sees stable
    --     face config values from p_geometry / p_face_cfg_latch.
    --
    --   s_face_active_r: tracks the "accepted face" window.
    --     Set on packet_start, cleared on frame_done_both or cmd_stop.
    --
    --   s_shot_pending_r: registered shot acceptance (1-clk pulse).
    --     On edge N the acceptance conditions are sampled.  The pulse fires
    --     at N+1.  On that next edge s_frame_done / s_face_closing from
    --     header_inserter are already visible, so a combinational kill on
    --     the output closes the same-edge race that a purely combinational
    --     gate cannot prevent (VHDL signal semantics: registered outputs
    --     from header_inserter are NOT visible on the edge they are set).
    -- =========================================================================
    p_face_start_delay : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_soft_reset = '1' then
                s_face_start_r    <= '0';
                s_face_active_r   <= '0';
                s_shot_pending_r  <= '0';
                s_shot_deferred_r <= '0';
                s_shot_drop_cnt_r <= (others => '0');
            elsif s_cmd_stop = '1' then
                s_face_start_r    <= '0';
                s_face_active_r   <= '0';
                s_shot_pending_r  <= '0';
                s_shot_deferred_r <= '0';
            else
                s_face_start_r <= s_packet_start;

                -- face_active management
                if s_packet_start = '1' then
                    s_face_active_r <= '1';
                elsif s_frame_done_both = '1' then
                    s_face_active_r <= '0';
                end if;

                -- Deferred shot latch: captures a shot that arrives during
                -- the face-close window.  Consumed by s_packet_start when
                -- the sequencer returns to ST_WAIT_SHOT.
                -- If a second shot arrives while deferred is already set,
                -- it is dropped and s_shot_drop_cnt_r increments.
                if s_packet_start = '1' then
                    -- Consumed: clear the latch.
                    -- If this was a deferred-consume AND a new raw shot
                    -- arrives on the same cycle, re-defer it.
                    if s_shot_deferred_r = '1' and i_shot_start = '1' then
                        s_shot_deferred_r <= '1';  -- re-defer new shot
                    else
                        s_shot_deferred_r <= '0';
                    end if;
                elsif i_shot_start = '1' and s_shot_deferred_r = '0' then
                    -- Defer shot if:
                    --   (a) mid-face and closing (normal close window), or
                    --   (b) ST_WAIT_SHOT but packet_start blocked (e.g.
                    --       header still in abort-drain, or assembler
                    --       non-idle after overrun).
                    if (s_face_state_r = ST_IN_FACE and s_face_closing = '1')
                       or (s_face_state_r = ST_WAIT_SHOT
                           and s_packet_start = '0') then
                        s_shot_deferred_r <= '1';
                    end if;
                elsif i_shot_start = '1' and s_shot_deferred_r = '1' then
                    -- Already deferred — this shot is dropped
                    s_shot_drop_cnt_r <= s_shot_drop_cnt_r + 1;
                end if;

                -- Auto-defer: if pending was set last cycle but got killed
                -- at s_shot_start_gated (closing/abort went high between
                -- acceptance and delivery), save it to deferred.
                if s_shot_pending_r = '1' and s_shot_start_gated = '0'
                   and s_cmd_stop = '0' and s_cmd_soft_reset = '0'
                   and s_pipeline_abort = '0' then
                    -- Shot was legitimately accepted but killed by closing.
                    -- Defer it for the next face.
                    if s_shot_deferred_r = '0' then
                        s_shot_deferred_r <= '1';
                    else
                        s_shot_drop_cnt_r <= s_shot_drop_cnt_r + 1;
                    end if;
                end if;

                -- Registered shot acceptance:
                --   First shot : ST_WAIT_SHOT + (shot_start or deferred)
                --                + no stop/reset
                --   Mid-face   : face_active + not closing + not done + no stop
                if (s_face_state_r = ST_WAIT_SHOT
                        and (i_shot_start = '1' or s_shot_deferred_r = '1')
                        and s_cmd_stop = '0')
                   or (s_face_active_r = '1'
                        and s_frame_done_both = '0'
                        and s_face_closing = '0'
                        and s_cmd_stop = '0'
                        and i_shot_start = '1') then
                    s_shot_pending_r <= '1';
                else
                    s_shot_pending_r <= '0';
                end if;
            end if;
        end if;
    end process p_face_start_delay;

    -- =========================================================================
    -- [0c] Accepted shot_start output.
    --   s_shot_pending_r fires at N+1 (one cycle after acceptance).
    --   Combinational kill: if s_face_closing or s_cmd_stop went high between
    --   the acceptance edge (N) and the delivery edge (N+1), suppress the
    --   pulse.  At N+1 the registered frame_done from header_inserter IS
    --   visible, so the kill is effective against the same-edge race.
    -- =========================================================================
    -- Accepted shot delivery with closing/abort kill.
    -- If s_shot_pending_r was set but gets killed here (closing went
    -- high between acceptance and delivery), the shot is auto-deferred
    -- in p_face_start_delay below — not silently lost.
    s_shot_start_gated <= s_shot_pending_r
                          when s_face_closing = '0'
                               and s_cmd_stop = '0'
                               and s_cmd_soft_reset = '0'
                               and s_pipeline_abort = '0'
                          else '0';

    -- Gated face_start: suppresses orphan-face if cmd_stop or soft_reset
    -- arrives on the same cycle that the registered s_face_start_r fires.
    s_face_start_gated <= s_face_start_r
                          when s_cmd_stop = '0'
                               and s_cmd_soft_reset = '0'
                          else '0';

    -- Pipeline-wide abort one-shot:
    --   ANY assembler self-overrun OR cmd_stop OR cmd_soft_reset.
    --   Sent to BOTH assemblers (i_abort), BOTH headers (i_face_abort),
    --   BOTH face FIFOs (i_flush), and frame_done_both combiner.
    --   No feedback loop: assembler i_abort handler does NOT emit
    --   o_face_abort (only self-overrun does), so the signal dies
    --   after the originating pulse ends.
    s_pipeline_abort <= s_face_abort or s_face_fall_abort
                        or s_cmd_stop or s_cmd_soft_reset;
    s_hdr_abort      <= s_pipeline_abort;
    s_hdr_fall_abort <= s_pipeline_abort;

    -- Per-chip shot gating: inactive chips (per active_chip_mask) do not
    -- receive shot_start and therefore never enter CAPTURE/DRAIN.
    gen_shot_mask : for i in 0 to c_N_CHIPS - 1 generate
        s_shot_start_per_chip(i) <= s_shot_start_gated and s_face_active_mask_r(i);
    end generate gen_shot_mask;

    -- =========================================================================
    -- [0b] Unified chip error mask: ErrFlag (physical) OR timeout (assembler)
    --   Masked by active_chip_mask so inactive chips cannot pollute status.
    -- =========================================================================
    s_chip_error_merged <= (s_errflag_sync or s_chip_error_flags or s_chip_fall_error)
                           and s_face_active_mask_r;

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
    -- [1a] Per-chip reg access demux: route cmd to targeted chip only
    -- =========================================================================
    -- 1-outstanding lock: blocks new reg access while one is in flight.
    gen_reg_demux : for i in 0 to c_N_CHIPS - 1 generate
        s_cmd_reg_read_g(i)  <= s_cmd_reg_read
                                when to_integer(s_cmd_reg_chip) = i
                                 and s_reg_outstanding_r = '0'
                                else '0';
        s_cmd_reg_write_g(i) <= s_cmd_reg_write
                                when to_integer(s_cmd_reg_chip) = i
                                 and s_reg_outstanding_r = '0'
                                else '0';
    end generate gen_reg_demux;

    -- Latch target chip + manage 1-outstanding flag.
    p_reg_chip_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_soft_reset = '1' then
                s_outstanding_reg_chip_r <= (others => '0');
                s_reg_outstanding_r      <= '0';
            else
                -- Set on accepted reg access
                if (s_cmd_reg_read = '1' or s_cmd_reg_write = '1')
                   and s_reg_outstanding_r = '0' then
                    s_outstanding_reg_chip_r <= s_cmd_reg_chip;
                    s_reg_outstanding_r      <= '1';
                end if;
                -- Clear when targeted chip completes (reg_done pulse)
                if s_reg_outstanding_r = '1'
                   and s_cmd_reg_done(to_integer(s_outstanding_reg_chip_r)) = '1' then
                    s_reg_outstanding_r <= '0';
                end if;
            end if;
        end if;
    end process p_reg_chip_latch;

    -- Write data: from cfg_image indexed by target register address
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
                i_bus_ticks     => s_cfg.bus_ticks,
                i_req_valid     => s_bus_req_valid(i),
                i_req_rw        => s_bus_req_rw(i),
                i_req_addr      => s_bus_req_addr(i),
                i_req_wdata     => s_bus_req_wdata(i),
                i_oen_permanent => s_bus_oen_perm(i),
                i_req_burst     => s_bus_req_burst(i),
                o_rsp_valid     => s_bus_rsp_valid(i),
                o_rsp_rdata     => s_bus_rsp_rdata(i),
                o_busy          => s_bus_busy(i),
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
                i_cmd_cfg_write     => s_cmd_cfg_write,
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
                i_bus_rsp_valid     => s_bus_rsp_valid(i),
                i_bus_rsp_rdata     => s_bus_rsp_rdata(i),
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
                o_raw_word          => s_raw_word(i),
                o_raw_word_valid    => s_raw_word_valid(i),
                o_ififo_id          => s_ififo_id(i),
                o_drain_done        => s_drain_done(i),
                o_shot_seq          => s_chip_shot_seq(i),
                o_busy              => s_chip_busy(i),
                o_err_drain_timeout => s_err_drain_timeout(i),
                o_err_sequence      => s_err_sequence(i)
            );

        -- ----- decode_i: combinational 28-bit I-Mode field extraction -----
        u_decode : entity work.tdc_gpx_decode_i
            port map (
                i_raw_word       => s_raw_word(i),
                i_raw_word_valid => s_raw_word_valid(i),
                i_ififo_id       => s_ififo_id(i),
                o_raw_hit        => s_dec_raw_hit(i),
                o_slope          => s_dec_slope(i),
                o_cha_code_raw   => s_dec_cha_code(i),
                o_stop_id_local  => s_dec_stop_id(i),
                o_decoded_valid  => s_dec_valid(i)
            );

        -- ----- raw_event_builder: enrich with chip/shot context -----
        u_event_bld : entity work.tdc_gpx_raw_event_builder
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_raw_hit         => s_dec_raw_hit(i),
                i_slope           => s_dec_slope(i),
                i_cha_code_raw    => s_dec_cha_code(i),
                i_stop_id_local   => s_dec_stop_id(i),
                i_decoded_valid   => s_dec_valid(i),
                i_ififo_id        => s_ififo_id(i),
                i_chip_id         => to_unsigned(i, 2),
                i_shot_seq        => s_chip_shot_seq(i),
                i_drain_done      => s_drain_done(i),
                i_stops_per_chip  => s_face_stops_per_chip_r,
                o_raw_event       => s_raw_event(i),
                o_raw_event_valid => s_raw_event_valid(i),
                o_stop_id_error   => s_stop_id_error(i)
            );

        -- ----- slope demux: split raw_event_valid by slope bit -----
        s_raw_evt_rise_valid(i) <= s_raw_event_valid(i) and     s_raw_event(i).slope;
        s_raw_evt_fall_valid(i) <= s_raw_event_valid(i) and not s_raw_event(i).slope;

        -- ----- cell_builder (rising): sparse events -> dense cell -> AXI-Stream -----
        u_cell_bld : entity work.tdc_gpx_cell_builder
            generic map (
                g_CHIP_ID => i
            )
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_raw_event       => s_raw_event(i),
                i_raw_event_valid => s_raw_evt_rise_valid(i),
                i_shot_start      => s_shot_start_per_chip(i),
                i_drain_done      => s_drain_done(i),
                i_stops_per_chip  => s_face_stops_per_chip_r,
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
                g_CHIP_ID => i
            )
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_raw_event       => s_raw_event(i),
                i_raw_event_valid => s_raw_evt_fall_valid(i),
                i_shot_start      => s_shot_start_per_chip(i),
                i_drain_done      => s_drain_done(i),
                i_stops_per_chip  => s_face_stops_per_chip_r,
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
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS
        )
        port map (
            i_clk              => i_axis_aclk,
            i_rst_n            => i_axis_aresetn,
            i_s_axis_tdata     => s_cell_tdata,
            i_s_axis_tvalid    => s_cell_tvalid,
            i_s_axis_tlast     => s_cell_tlast,
            o_s_axis_tready    => s_cell_tready,
            i_shot_start       => s_shot_start_gated,
            i_abort            => s_pipeline_abort,
            i_active_chip_mask => s_face_active_mask_r,
            i_stops_per_chip   => s_face_stops_per_chip_r,
            i_max_range_clks   => s_cfg.max_range_clks,
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
            g_ALU_PULSE_CLKS => g_ALU_PULSE_CLKS
        )
        port map (
            i_clk              => i_axis_aclk,
            i_rst_n            => i_axis_aresetn,
            i_s_axis_tdata     => s_cell_fall_tdata,
            i_s_axis_tvalid    => s_cell_fall_tvalid,
            i_s_axis_tlast     => s_cell_fall_tlast,
            o_s_axis_tready    => s_cell_fall_tready,
            i_shot_start       => s_shot_start_gated,
            i_abort            => s_pipeline_abort,
            i_active_chip_mask => s_face_active_mask_r,
            i_stops_per_chip   => s_face_stops_per_chip_r,
            i_max_range_clks   => s_cfg.max_range_clks,
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
            g_DATA_WIDTH => c_TDATA_WIDTH + 1,  -- tdata(31:0) & tlast(0) = 33 bits
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
            g_DATA_WIDTH => c_TDATA_WIDTH + 1,
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
            i_s_axis_tdata      => s_face_buf_tdata(c_TDATA_WIDTH downto 1),
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
            i_s_axis_tdata      => s_face_fall_buf_tdata(c_TDATA_WIDTH downto 1),
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
    p_geometry : process(i_axis_aclk)
        variable v_active_cnt  : natural range 0 to c_N_CHIPS;
        variable v_rows        : natural range 0 to c_MAX_ROWS_PER_FACE;
        variable v_data_beats  : natural range 0 to c_DATA_BEATS_MAX;
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_rows_per_face_r <= to_unsigned(c_MAX_ROWS_PER_FACE, 16);
                s_hsize_bytes_r   <= to_unsigned(c_HSIZE_MAX, 16);
            elsif s_packet_start = '1' then
                v_active_cnt := fn_count_ones(s_cfg.active_chip_mask);
                v_rows := v_active_cnt * to_integer(s_cfg.stops_per_chip);
                if v_rows < 2 then
                    v_rows := 2;
                end if;
                s_rows_per_face_r <= to_unsigned(v_rows, 16);
                v_data_beats := v_rows * c_BEATS_PER_CELL;
                s_hsize_bytes_r <= to_unsigned((v_data_beats + c_HDR_PREFIX_BEATS) * c_TDATA_BYTES, 16);
            end if;
        end if;
    end process p_geometry;

    -- =========================================================================
    -- [5b] Face-start config latch: snapshot for downstream data-path modules
    --   Latched at s_packet_start (combinational, guarded, edge N) so values
    --   are stable by s_face_start_r (edge N+1). face_assembler reads these
    --   at shot_start = s_face_start_r, seeing the correctly latched values.
    -- =========================================================================
    p_face_cfg_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_face_stops_per_chip_r <= to_unsigned(8, 4);
                s_face_active_mask_r    <= (others => '1');
                s_face_cols_per_face_r  <= to_unsigned(1, 16);
                s_face_n_faces_r        <= to_unsigned(1, 4);
                s_cfg_face_r            <= s_cfg;   -- safe default (avoids X propagation)
            elsif s_packet_start = '1' then
                s_face_stops_per_chip_r <= s_cfg.stops_per_chip;
                s_face_active_mask_r    <= s_cfg.active_chip_mask;
                s_face_cols_per_face_r  <= s_cfg.cols_per_face;
                s_face_n_faces_r        <= resize(s_cfg.n_faces, 4);
                s_cfg_face_r            <= s_cfg;  -- full config snapshot for header
            end if;
        end if;
    end process p_face_cfg_latch;

    -- =========================================================================
    -- [5c] Frame-done combiner: both rise and fall pipelines must complete.
    --   Each pipeline's frame_done is a 1-clk pulse.  We latch each one and
    --   generate s_frame_done_both when both have fired.  Cleared at the
    --   next packet_start (= next face).
    -- =========================================================================
    p_frame_done_both : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_packet_start = '1' then
                s_frame_rise_done_r <= '0';
                s_frame_fall_done_r <= '0';
            else
                if s_frame_done = '1' or s_pipeline_abort = '1' then
                    s_frame_rise_done_r <= '1';
                end if;
                if s_frame_fall_done = '1' or s_pipeline_abort = '1' then
                    s_frame_fall_done_r <= '1';
                end if;
            end if;
        end if;
    end process p_frame_done_both;

    -- Combinational: both done (could be same cycle or different cycles)
    s_frame_done_both   <= '1'  when (s_frame_rise_done_r = '1' or s_frame_done = '1' or s_pipeline_abort = '1')
                                    and (s_frame_fall_done_r = '1' or s_frame_fall_done = '1' or s_pipeline_abort = '1')
                                else '0';

    -- '1' when all shots for this face have been accepted, OR either
    -- pipeline has fired frame_done / is draining / aborted.
    -- s_all_shots_fired closes the pre-drain window without blocking
    -- the final shot itself (unlike the old o_last_line approach).
    s_all_shots_fired   <= '1'  when s_face_shot_cnt_r >= s_face_cols_per_face_r
                                     and s_face_cols_per_face_r /= 0
                                else '0';

    s_face_closing      <= '1'  when s_all_shots_fired = '1'
                                      or s_pipeline_abort = '1'
                                      or (s_frame_rise_done_r = '1' or s_frame_done = '1'
                                          or s_hdr_draining = '1')
                                      or (s_frame_fall_done_r = '1' or s_frame_fall_done = '1'
                                          or s_hdr_fall_draining = '1')
                                else '0';

    -- =========================================================================
    -- [6] Face sequencer
    --   ST_IDLE → cmd_start → ST_WAIT_SHOT
    --   ST_WAIT_SHOT → shot_start → face_start pulse → ST_IN_FACE
    --   ST_IN_FACE → frame_done → advance face_id → ST_WAIT_SHOT (or wrap)
    -- =========================================================================
    p_face_seq : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_soft_reset = '1' then
                s_face_state_r        <= ST_IDLE;
                s_face_id_r           <= (others => '0');
                s_frame_id_r          <= (others => '0');
                s_shot_overrun_r      <= '0';
                s_cmd_start_accepted  <= '0';
            else
                -- Default: clear single-cycle pulse
                s_cmd_start_accepted <= '0';

                -- Shot overrun from face_assembler: sticky until cmd_start
                if s_shot_overrun = '1' or s_shot_fall_overrun = '1' then
                    s_shot_overrun_r <= '1';
                end if;

                case s_face_state_r is

                    when ST_IDLE =>
                        -- Accept cmd_start only when the entire pipeline
                        -- is idle: no chip_ctrl busy, no assembler/header
                        -- data in flight.  Self-protects against SW
                        -- issuing cmd_start before previous output drains.
                        if s_cmd_start = '1'
                           and s_chip_busy = C_ZEROS_CHIPS
                           and s_face_asm_idle = '1'
                           and s_face_asm_fall_idle = '1'
                           and s_hdr_idle = '1'
                           and s_hdr_fall_idle = '1'
                           and s_face_tvalid = '0'
                           and s_face_fall_tvalid = '0'
                           and s_face_buf_tvalid = '0'
                           and s_face_fall_buf_tvalid = '0'
                           and o_m_axis_tvalid = '0'
                           and o_m_axis_fall_tvalid = '0' then
                            s_face_id_r          <= (others => '0');
                            s_shot_overrun_r     <= '0';
                            s_cmd_start_accepted <= '1';
                            s_face_state_r       <= ST_WAIT_SHOT;
                        end if;

                    when ST_WAIT_SHOT =>
                        if s_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif s_packet_start = '1' then
                            -- Use s_packet_start (not raw i_shot_start) so
                            -- the sequencer accepts exactly when downstream
                            -- control pulses (face_start, shot_start_gated)
                            -- are also generated.  This ensures all modules
                            -- see the same shot boundary.
                            s_face_state_r <= ST_IN_FACE;
                        end if;

                    when ST_IN_FACE =>
                        if s_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif s_frame_done_both = '1' then
                            s_frame_id_r <= s_frame_id_r + 1;
                            if s_face_id_r >= resize(s_face_n_faces_r, 8) - 1 then
                                -- All faces done: wrap to face 0 (continuous)
                                s_face_id_r    <= (others => '0');
                            else
                                s_face_id_r <= s_face_id_r + 1;
                            end if;
                            s_face_state_r <= ST_WAIT_SHOT;
                        end if;

                end case;
            end if;
        end if;
    end process p_face_seq;

    -- =========================================================================
    -- [6b] Global shot sequence counter (independent of per-chip counters).
    --   Increments on every accepted shot pulse so that header/status always
    --   reflect the true shot count regardless of which chips are active.
    -- =========================================================================
    p_global_shot_seq : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_start_accepted = '1' then
                s_global_shot_seq_r <= (others => '0');
            elsif s_shot_start_gated = '1' then
                s_global_shot_seq_r <= s_global_shot_seq_r + 1;
            end if;
        end if;
    end process p_global_shot_seq;

    -- =========================================================================
    -- [6d] Frame abort counter: counts faces aborted by overrun/stop/reset.
    --   SW uses this to know how many frames were discarded (corrupted).
    --   Cleared on cmd_start_accepted (new measurement run).
    -- =========================================================================
    p_frame_abort_cnt : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_start_accepted = '1' then
                s_frame_abort_cnt_r <= (others => '0');
            elsif s_pipeline_abort = '1' and s_face_state_r = ST_IN_FACE then
                s_frame_abort_cnt_r <= s_frame_abort_cnt_r + 1;
            end if;
        end if;
    end process p_frame_abort_cnt;

    -- =========================================================================
    -- [6c] Per-face shot counter: tracks how many shots have been accepted
    --   for the current face.  When it reaches cols_per_face, no more
    --   mid-face shots are accepted (s_all_shots_fired → s_face_closing).
    -- =========================================================================
    p_face_shot_cnt : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_packet_start = '1'
               or s_frame_done_both = '1' then
                -- Reset at face boundary.  s_packet_start resets to 0;
                -- the first shot's s_shot_start_gated fires 1 cycle later
                -- and increments to 1 (no double-count).
                s_face_shot_cnt_r <= (others => '0');
            elsif s_shot_start_gated = '1' then
                -- Count every accepted shot (first + mid-face)
                s_face_shot_cnt_r <= s_face_shot_cnt_r + 1;
            end if;
        end if;
    end process p_face_shot_cnt;

    -- =========================================================================
    -- [7] Timestamp counter (free-running, i_axis_aclk domain)
    --   SW interprets as cycles; multiply by clock period for nanoseconds.
    -- =========================================================================
    p_timestamp : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_timestamp_r <= (others => '0');
            else
                s_timestamp_r <= s_timestamp_r + 1;
            end if;
        end if;
    end process p_timestamp;

    -- =========================================================================
    -- [8] Error counter: per-cycle any-error counter.
    --   Increments by 1 for each clock cycle where ANY error source is
    --   active.  Multiple simultaneous errors in the same cycle count as 1.
    --   This is NOT a total error event count — it measures error-active
    --   cycles.  SW should use per-chip sticky masks for individual errors.
    --   Sources:
    --   - stop_id_error: 1-clk pulse from raw_event_builder
    --   - hit_dropped:   1-clk pulse from cell_builder
    --   - chip_error_merged: level → edge-detected (rising only)
    -- =========================================================================
    p_error_cnt : process(i_axis_aclk)
        variable v_merged_rising : std_logic_vector(c_N_CHIPS - 1 downto 0);
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_soft_reset = '1' then
                s_error_count_r     <= (others => '0');
                s_chip_error_prev_r <= (others => '0');
            else
                -- Edge detect: new bits that just went high
                v_merged_rising := s_chip_error_merged
                                   and (not s_chip_error_prev_r);
                s_chip_error_prev_r <= s_chip_error_merged;

                if s_stop_id_error /= C_ZEROS_CHIPS or
                   s_hit_dropped /= C_ZEROS_CHIPS or
                   s_hit_fall_dropped /= C_ZEROS_CHIPS or
                   (s_err_drain_timeout and s_face_active_mask_r) /= C_ZEROS_CHIPS or
                   (s_err_sequence and s_face_active_mask_r) /= C_ZEROS_CHIPS or
                   v_merged_rising /= C_ZEROS_CHIPS then
                    s_error_count_r <= s_error_count_r + 1;
                end if;
            end if;
        end if;
    end process p_error_cnt;

    -- =========================================================================
    -- [8b] Sticky error latch: drain_timeout and sequence_error per chip
    --   Pulses from chip_ctrl → sticky until cmd_start clears them.
    -- =========================================================================
    p_err_sticky : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_start_accepted = '1' then
                s_err_drain_to_sticky_r <= (others => '0');
                s_err_seq_sticky_r      <= (others => '0');
            else
                s_err_drain_to_sticky_r <= s_err_drain_to_sticky_r or (s_err_drain_timeout and s_face_active_mask_r);
                s_err_seq_sticky_r      <= s_err_seq_sticky_r      or (s_err_sequence and s_face_active_mask_r);
            end if;
        end if;
    end process p_err_sticky;

    -- =========================================================================
    -- [9] Status aggregation (-> CSR -> STAT registers)
    -- =========================================================================
    s_status.busy               <= '1'  when s_face_state_r /= ST_IDLE
                                            or s_chip_busy /= C_ZEROS_CHIPS
                                            or s_face_asm_idle = '0'
                                            or s_face_asm_fall_idle = '0'
                                            or s_hdr_idle = '0'
                                            or s_hdr_fall_idle = '0'
                                            or s_face_tvalid = '1'
                                            or s_face_fall_tvalid = '1'
                                            or s_face_buf_tvalid = '1'
                                            or s_face_fall_buf_tvalid = '1'
                                            or o_m_axis_tvalid = '1'
                                            or o_m_axis_fall_tvalid = '1'
                                        else '0';
    s_status.pipeline_overrun   <= '1'  when s_chip_error_flags /= C_ZEROS_CHIPS
                                            or s_chip_fall_error /= C_ZEROS_CHIPS
                                            or s_shot_overrun_r = '1'
                                        else '0';
    s_status.bin_mismatch       <= '0';  -- Phase 2: calibration check
    s_status.chip_error_mask    <= s_chip_error_merged;
    s_status.drain_timeout_mask <= s_err_drain_to_sticky_r;
    s_status.sequence_error_mask<= s_err_seq_sticky_r;
    s_status.shot_seq_current   <= s_global_shot_seq_r;
    s_status.vdma_frame_count   <= s_frame_id_r;
    s_status.error_count        <= s_error_count_r;
    s_status.shot_drop_count    <= s_shot_drop_cnt_r;
    s_status.frame_abort_count  <= s_frame_abort_cnt_r;

end architecture rtl;
