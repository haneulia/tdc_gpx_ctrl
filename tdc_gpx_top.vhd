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
--     Face sequencer (shot counting, face/frame ID management)
--     Status aggregation (t_tdc_status assembly)
--     Timestamp counter (free-running cycle counter)
--     Error counter
--
-- Clock domains:
--   i_axis_aclk  : TDC processing / AXI-Stream domain (~200 MHz)
--   s_axi_aclk   : AXI4-Lite PS domain
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
        g_HW_VERSION     : std_logic_vector(31 downto 0) := x"00010000";
        g_POWERUP_CLKS   : natural := 48;
        g_RECOVERY_CLKS  : natural := 8;
        g_ALU_PULSE_CLKS : natural := 4
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
        -- 매 사이클 stop pulse 발생 시 즉시 전송 (실시간 이벤트).
        -- tdata: 해당 사이클 per-chip rising stop count (8bit × 4chips)
        -- tuser: 해당 사이클 per-chip falling stop count (8bit × 4chips)
        i_stop_evt_tvalid : in  std_logic;
        i_stop_evt_tdata  : in  std_logic_vector(31 downto 0);
        i_stop_evt_tuser  : in  std_logic_vector(31 downto 0);
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
    signal s_cfg_image      : t_cfg_image;
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
    -- Per-chip gated reg access commands (chip demux)
    signal s_cmd_reg_read_g   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cmd_reg_write_g  : std_logic_vector(c_N_CHIPS - 1 downto 0);
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
    signal s_frame_rise_done_r : std_logic := '0';
    signal s_frame_fall_done_r : std_logic := '0';

    -- =========================================================================
    -- Gated shot_start: only active when face sequencer is in run state.
    -- Prevents downstream FSMs from reacting to stray shot pulses before
    -- cmd_start or after cmd_stop.
    -- =========================================================================
    signal s_shot_start_gated : std_logic;

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
    signal s_face_active_mask_r    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');

    -- =========================================================================
    -- Combinational packet_start: same-cycle face boundary signal.
    -- Visible to all processes at the same clock edge as the first shot,
    -- so geometry/config/header all latch simultaneously with data-path.
    -- =========================================================================
    signal s_packet_start : std_logic;

    -- =========================================================================
    -- Shot overrun: from face_assembler (real truncation), not face_seq
    -- =========================================================================
    signal s_shot_overrun    : std_logic;
    signal s_shot_overrun_r  : std_logic := '0';

    -- =========================================================================
    -- Stop event stream: per-chip hit count latch
    -- AXI Stream slave: always ready (tready='1').
    -- echo_receiver가 누적 카운터를 관리하므로, 여기서는 tvalid 시 overwrite.
    -- tdata/tuser = window 시작 이후 누적된 총 count (echo_receiver 측 누적).
    -- =========================================================================
    type t_stop_hit_cnt_array is array(0 to c_N_CHIPS - 1)
        of unsigned(7 downto 0);
    signal s_stop_rise_cnt_r : t_stop_hit_cnt_array := (others => (others => '0'));
    signal s_stop_fall_cnt_r : t_stop_hit_cnt_array := (others => (others => '0'));

begin

    -- =========================================================================
    -- Stop event AXI Stream slave: always accept, latch per-chip counts
    -- =========================================================================
    o_stop_evt_tready <= '1';

    -- Latch per-chip hit counts on tvalid (overwrite, not accumulate).
    -- echo_receiver가 내부에서 누적하므로 tdata/tuser가 이미 총 누적값.
    -- start_rising(새 window)에서 echo_receiver가 0 리셋 → 여기도 자동 반영.
    p_stop_decode : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_stop_rise_cnt_r <= (others => (others => '0'));
                s_stop_fall_cnt_r <= (others => (others => '0'));
            elsif i_stop_evt_tvalid = '1' then
                for c in 0 to c_N_CHIPS - 1 loop
                    s_stop_rise_cnt_r(c) <= unsigned(i_stop_evt_tdata(c*8 + 7 downto c*8));
                    s_stop_fall_cnt_r(c) <= unsigned(i_stop_evt_tuser(c*8 + 7 downto c*8));
                end loop;
            end if;
        end if;
    end process p_stop_decode;

    -- =========================================================================
    -- [0] Gated shot_start: suppress when face sequencer is not in run state
    -- =========================================================================
    s_shot_start_gated <= i_shot_start
                          when s_face_state_r /= ST_IDLE
                          else '0';

    -- =========================================================================
    -- [0a] Combinational packet_start: first shot of a new face.
    --   Arrives on the SAME clock edge as shot_start, so geometry/config/header
    --   latch at the same time as cell_builder/face_assembler react to the shot.
    -- =========================================================================
    s_packet_start  <= '1'  when s_face_state_r = ST_WAIT_SHOT
                                and i_shot_start = '1'
                            else '0';

    -- =========================================================================
    -- [0b] Unified chip error mask: ErrFlag (physical) OR timeout (assembler)
    -- =========================================================================
    s_chip_error_merged <= s_errflag_sync or s_chip_error_flags or s_chip_fall_error;

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
            o_cfg_image      => s_cfg_image,
            o_cmd_start      => s_cmd_start,
            o_cmd_stop       => s_cmd_stop,
            o_cmd_soft_reset => s_cmd_soft_reset,
            o_cmd_cfg_write  => s_cmd_cfg_write,
            o_cmd_reg_read   => s_cmd_reg_read,
            o_cmd_reg_write  => s_cmd_reg_write,
            o_cmd_reg_addr   => s_cmd_reg_addr,
            o_cmd_reg_chip   => s_cmd_reg_chip,
            i_cmd_reg_rdata  => s_cmd_reg_rdata(to_integer(s_cmd_reg_chip)),
            i_cmd_reg_rvalid => s_cmd_reg_rvalid(to_integer(s_cmd_reg_chip)),
            i_status            => s_status,
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            o_irq            => o_irq
        );

    -- =========================================================================
    -- [1a] Per-chip reg access demux: route cmd to targeted chip only
    -- =========================================================================
    gen_reg_demux : for i in 0 to c_N_CHIPS - 1 generate
        s_cmd_reg_read_g(i)  <= s_cmd_reg_read  when to_integer(s_cmd_reg_chip) = i else '0';
        s_cmd_reg_write_g(i) <= s_cmd_reg_write when to_integer(s_cmd_reg_chip) = i else '0';
    end generate gen_reg_demux;

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
                i_cmd_start         => s_cmd_start,
                i_cmd_stop          => s_cmd_stop,
                i_cmd_soft_reset    => s_cmd_soft_reset,
                i_cmd_cfg_write     => s_cmd_cfg_write,
                i_cmd_reg_read      => s_cmd_reg_read_g(i),
                i_cmd_reg_write     => s_cmd_reg_write_g(i),
                i_cmd_reg_addr      => s_cmd_reg_addr,
                i_cmd_reg_wdata     => s_cmd_reg_wdata,
                o_cmd_reg_rdata     => s_cmd_reg_rdata(i),
                o_cmd_reg_rvalid    => s_cmd_reg_rvalid(i),
                i_shot_start        => s_shot_start_gated,
                i_max_range_clks    => s_cfg.max_range_clks,
                i_stop_tdc          => i_stop_tdc,
                i_stop_rise_cnt     => s_stop_rise_cnt_r(i),
                i_stop_fall_cnt     => s_stop_fall_cnt_r(i),
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
                i_shot_start      => s_shot_start_gated,
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
                i_shot_start      => s_shot_start_gated,
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
            i_active_chip_mask => s_face_active_mask_r,
            i_stops_per_chip   => s_face_stops_per_chip_r,
            i_max_range_clks   => s_cfg.max_range_clks,
            i_bus_ticks        => s_cfg.bus_ticks,
            i_bus_clk_div      => resize(s_cfg.bus_clk_div, 8),
            o_m_axis_tdata     => s_face_tdata,
            o_m_axis_tvalid    => s_face_tvalid,
            o_m_axis_tlast     => s_face_tlast,
            i_m_axis_tready    => s_face_tready,
            o_row_done         => s_row_done,
            o_chip_error_flags => s_chip_error_flags,
            o_shot_overrun     => s_shot_overrun
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
            i_active_chip_mask => s_face_active_mask_r,
            i_stops_per_chip   => s_face_stops_per_chip_r,
            i_max_range_clks   => s_cfg.max_range_clks,
            i_bus_ticks        => s_cfg.bus_ticks,
            i_bus_clk_div      => resize(s_cfg.bus_clk_div, 8),
            o_m_axis_tdata     => s_face_fall_tdata,
            o_m_axis_tvalid    => s_face_fall_tvalid,
            o_m_axis_tlast     => s_face_fall_tlast,
            i_m_axis_tready    => s_face_fall_tready,
            o_row_done         => s_row_fall_done,
            o_chip_error_flags => s_chip_fall_error,
            o_shot_overrun     => s_shot_fall_overrun
        );

    -- =========================================================================
    -- [4] Header inserter - RISING pipeline (header + SOF/EOL -> VDMA frame)
    -- =========================================================================
    u_header : entity work.tdc_gpx_header_inserter
        port map (
            i_clk               => i_axis_aclk,
            i_rst_n             => i_axis_aresetn,
            i_face_start        => s_packet_start,
            i_cfg               => s_cfg,
            i_vdma_frame_id     => s_frame_id_r,
            i_face_id           => s_face_id_r,
            i_shot_seq_start    => s_chip_shot_seq(0),
            i_timestamp_ns      => s_timestamp_r,
            i_chip_error_mask   => s_chip_error_merged,
            i_chip_error_cnt    => std_logic_vector(s_error_count_r),
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            i_rows_per_face     => s_rows_per_face_r,
            i_s_axis_tdata      => s_face_tdata,
            i_s_axis_tvalid     => s_face_tvalid,
            i_s_axis_tlast      => s_face_tlast,
            o_s_axis_tready     => s_face_tready,
            o_m_axis_tdata      => o_m_axis_tdata,
            o_m_axis_tvalid     => o_m_axis_tvalid,
            o_m_axis_tlast      => o_m_axis_tlast,
            o_m_axis_tuser      => o_m_axis_tuser,
            i_m_axis_tready     => i_m_axis_tready,
            o_frame_done        => s_frame_done
        );

    -- =========================================================================
    -- [4f] Header inserter - FALLING pipeline (header + SOF/EOL -> VDMA frame)
    -- =========================================================================
    u_header_fall : entity work.tdc_gpx_header_inserter
        port map (
            i_clk               => i_axis_aclk,
            i_rst_n             => i_axis_aresetn,
            i_face_start        => s_packet_start,
            i_cfg               => s_cfg,
            i_vdma_frame_id     => s_frame_id_r,
            i_face_id           => s_face_id_r,
            i_shot_seq_start    => s_chip_shot_seq(0),
            i_timestamp_ns      => s_timestamp_r,
            i_chip_error_mask   => s_chip_error_merged,
            i_chip_error_cnt    => std_logic_vector(s_error_count_r),
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            i_rows_per_face     => s_rows_per_face_r,
            i_s_axis_tdata      => s_face_fall_tdata,
            i_s_axis_tvalid     => s_face_fall_tvalid,
            i_s_axis_tlast      => s_face_fall_tlast,
            o_s_axis_tready     => s_face_fall_tready,
            o_m_axis_tdata      => o_m_axis_fall_tdata,
            o_m_axis_tvalid     => o_m_axis_fall_tvalid,
            o_m_axis_tlast      => o_m_axis_fall_tlast,
            o_m_axis_tuser      => o_m_axis_fall_tuser,
            i_m_axis_tready     => i_m_axis_fall_tready,
            o_frame_done        => s_frame_fall_done
        );

    -- =========================================================================
    -- [5] VDMA line geometry
    --   Computed from CSR config, shared with header_inserter and VDMA.
    --   rows_per_face = active_chips × stops_per_chip (clamp >= 2)
    --   hsize_bytes   = (data_beats + c_HDR_PREFIX_BEATS) × TDATA_BYTES
    --   Latched at face_start to prevent mid-frame geometry changes.
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
    --   These feed raw_event_builder, cell_builder, face_assembler so they
    --   all see the same config throughout the face — consistent with header
    --   and geometry which also latch at face_start.
    -- =========================================================================
    p_face_cfg_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_face_stops_per_chip_r <= to_unsigned(8, 4);
                s_face_active_mask_r    <= (others => '1');
            elsif s_packet_start = '1' then
                s_face_stops_per_chip_r <= s_cfg.stops_per_chip;
                s_face_active_mask_r    <= s_cfg.active_chip_mask;
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
                if s_frame_done = '1' then
                    s_frame_rise_done_r <= '1';
                end if;
                if s_frame_fall_done = '1' then
                    s_frame_fall_done_r <= '1';
                end if;
            end if;
        end if;
    end process p_frame_done_both;

    -- Combinational: both done (could be same cycle or different cycles)
    s_frame_done_both   <= '1'  when (s_frame_rise_done_r = '1' or s_frame_done = '1')
                                    and (s_frame_fall_done_r = '1' or s_frame_fall_done = '1')
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
                s_face_state_r   <= ST_IDLE;
                s_face_id_r      <= (others => '0');
                s_frame_id_r     <= (others => '0');
                s_shot_overrun_r <= '0';
            else
                -- Shot overrun from face_assembler: sticky until cmd_start
                if s_shot_overrun = '1' or s_shot_fall_overrun = '1' then
                    s_shot_overrun_r <= '1';
                end if;

                case s_face_state_r is

                    when ST_IDLE =>
                        if s_cmd_start = '1' then
                            s_face_id_r      <= (others => '0');
                            s_shot_overrun_r <= '0';
                            s_face_state_r   <= ST_WAIT_SHOT;
                        end if;

                    when ST_WAIT_SHOT =>
                        if s_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif i_shot_start = '1' then
                            s_face_state_r <= ST_IN_FACE;
                        end if;

                    when ST_IN_FACE =>
                        if s_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif s_frame_done_both = '1' then
                            s_frame_id_r <= s_frame_id_r + 1;
                            if s_face_id_r >= resize(s_cfg.n_faces, 8) - 1 then
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
    -- [8] Error counter (increments on error events, not level duration)
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
                   s_err_drain_timeout /= C_ZEROS_CHIPS or
                   s_err_sequence /= C_ZEROS_CHIPS or
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
            if i_axis_aresetn = '0' or s_cmd_start = '1' then
                s_err_drain_to_sticky_r <= (others => '0');
                s_err_seq_sticky_r      <= (others => '0');
            else
                s_err_drain_to_sticky_r <= s_err_drain_to_sticky_r or s_err_drain_timeout;
                s_err_seq_sticky_r      <= s_err_seq_sticky_r      or s_err_sequence;
            end if;
        end if;
    end process p_err_sticky;

    -- =========================================================================
    -- [9] Status aggregation (-> CSR -> STAT registers)
    -- =========================================================================
    s_status.busy               <= '1'  when s_face_state_r /= ST_IDLE else '0';
    s_status.pipeline_overrun   <= '1'  when s_chip_error_flags /= C_ZEROS_CHIPS
                                            or s_shot_overrun_r = '1'
                                        else '0';
    s_status.bin_mismatch       <= '0';  -- Phase 2: calibration check
    s_status.chip_error_mask    <= s_chip_error_merged;
    s_status.drain_timeout_mask <= s_err_drain_to_sticky_r;
    s_status.sequence_error_mask<= s_err_seq_sticky_r;
    s_status.shot_seq_current   <= s_chip_shot_seq(0);
    s_status.vdma_frame_count   <= s_frame_id_r;
    s_status.error_count        <= s_error_count_r;

end architecture rtl;
