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

        -- AXI-Stream master output (to CDC FIFO / VDMA)
        o_m_axis_tdata   : out std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
        o_m_axis_tvalid  : out std_logic;
        o_m_axis_tlast   : out std_logic;
        o_m_axis_tuser   : out std_logic_vector(0 downto 0);
        i_m_axis_tready  : in  std_logic;

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
    signal s_status         : t_tdc_status := c_TDC_STATUS_INIT;

    -- =========================================================================
    -- Per-chip: bus_phy <-> chip_ctrl
    -- =========================================================================
    signal s_bus_req_valid   : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_rw      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_bus_req_addr    : t_slv4_array;
    signal s_bus_req_wdata   : t_slv28_array;
    signal s_bus_oen_perm    : std_logic_vector(c_N_CHIPS - 1 downto 0);
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

    -- Per-chip: raw_event_builder -> cell_builder
    signal s_raw_event       : t_raw_event_array;
    signal s_raw_event_valid : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_stop_id_error   : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Per-chip: cell_builder -> face_assembler (AXI-Stream)
    signal s_cell_tdata      : t_axis_tdata_array;
    signal s_cell_tvalid     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_tlast      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_cell_tready     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_slice_done      : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_hit_dropped     : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- face_assembler -> header_inserter (AXI-Stream)
    -- =========================================================================
    signal s_face_tdata      : std_logic_vector(c_TDATA_WIDTH - 1 downto 0);
    signal s_face_tvalid     : std_logic;
    signal s_face_tlast      : std_logic;
    signal s_face_tready     : std_logic;
    signal s_row_done        : std_logic;
    signal s_chip_error_flags: std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- =========================================================================
    -- Face sequencer
    -- =========================================================================
    type t_face_state is (ST_IDLE, ST_WAIT_SHOT, ST_IN_FACE);
    signal s_face_state_r    : t_face_state := ST_IDLE;
    signal s_face_start_r    : std_logic := '0';
    signal s_face_id_r       : unsigned(7 downto 0) := (others => '0');
    signal s_frame_id_r      : unsigned(31 downto 0) := (others => '0');
    signal s_frame_done      : std_logic;

    -- =========================================================================
    -- Timestamp (free-running cycle counter, i_axis_aclk domain)
    -- =========================================================================
    signal s_timestamp_r     : unsigned(63 downto 0) := (others => '0');

    -- =========================================================================
    -- Error counter
    -- =========================================================================
    signal s_error_count_r   : unsigned(31 downto 0) := (others => '0');

begin

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
            i_status            => s_status,
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            o_irq            => o_irq
        );

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
                i_shot_start        => i_shot_start,
                o_bus_req_valid     => s_bus_req_valid(i),
                o_bus_req_rw        => s_bus_req_rw(i),
                o_bus_req_addr      => s_bus_req_addr(i),
                o_bus_req_wdata     => s_bus_req_wdata(i),
                o_bus_oen_permanent => s_bus_oen_perm(i),
                i_bus_rsp_valid     => s_bus_rsp_valid(i),
                i_bus_rsp_rdata     => s_bus_rsp_rdata(i),
                i_bus_busy          => s_bus_busy(i),
                i_ef1_sync          => s_ef1_sync(i),
                i_ef2_sync          => s_ef2_sync(i),
                i_irflag_sync       => s_irflag_sync(i),
                o_tick_en           => s_tick_en(i),
                o_stopdis           => o_tdc_stopdis(i),
                o_alutrigger        => o_tdc_alutrigger(i),
                o_puresn            => o_tdc_puresn(i),
                o_raw_word          => s_raw_word(i),
                o_raw_word_valid    => s_raw_word_valid(i),
                o_ififo_id          => s_ififo_id(i),
                o_drain_done        => s_drain_done(i),
                o_shot_seq          => s_chip_shot_seq(i),
                o_busy              => s_chip_busy(i)
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
                i_stops_per_chip  => s_cfg.stops_per_chip,
                o_raw_event       => s_raw_event(i),
                o_raw_event_valid => s_raw_event_valid(i),
                o_stop_id_error   => s_stop_id_error(i)
            );

        -- ----- cell_builder: sparse events -> dense cell -> AXI-Stream -----
        u_cell_bld : entity work.tdc_gpx_cell_builder
            generic map (
                g_CHIP_ID => i
            )
            port map (
                i_clk             => i_axis_aclk,
                i_rst_n           => i_axis_aresetn,
                i_raw_event       => s_raw_event(i),
                i_raw_event_valid => s_raw_event_valid(i),
                i_shot_start      => i_shot_start,
                i_drain_done      => s_drain_done(i),
                i_stops_per_chip  => s_cfg.stops_per_chip,
                o_m_axis_tdata    => s_cell_tdata(i),
                o_m_axis_tvalid   => s_cell_tvalid(i),
                o_m_axis_tlast    => s_cell_tlast(i),
                i_m_axis_tready   => s_cell_tready(i),
                o_slice_done      => s_slice_done(i),
                o_hit_dropped_any => s_hit_dropped(i)
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
            i_shot_start       => i_shot_start,
            i_active_chip_mask => s_cfg.active_chip_mask,
            i_stops_per_chip   => s_cfg.stops_per_chip,
            i_max_range_clks   => s_cfg.max_range_clks,
            i_bus_ticks        => s_cfg.bus_ticks,
            i_bus_clk_div      => resize(s_cfg.bus_clk_div, 8),
            o_m_axis_tdata     => s_face_tdata,
            o_m_axis_tvalid    => s_face_tvalid,
            o_m_axis_tlast     => s_face_tlast,
            i_m_axis_tready    => s_face_tready,
            o_row_done         => s_row_done,
            o_chip_error_flags => s_chip_error_flags
        );

    -- =========================================================================
    -- [4] Header inserter (header + SOF/EOL -> VDMA frame)
    -- =========================================================================
    u_header : entity work.tdc_gpx_header_inserter
        port map (
            i_clk               => i_axis_aclk,
            i_rst_n             => i_axis_aresetn,
            i_face_start        => s_face_start_r,
            i_cfg               => s_cfg,
            i_vdma_frame_id     => s_frame_id_r,
            i_face_id           => s_face_id_r,
            i_shot_seq_start    => s_chip_shot_seq(0),
            i_timestamp_ns      => s_timestamp_r,
            i_lane_error_mask   => s_errflag_sync,
            i_lane_error_cnt    => std_logic_vector(s_error_count_r),
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
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
    -- [5] Face sequencer
    --   ST_IDLE → cmd_start → ST_WAIT_SHOT
    --   ST_WAIT_SHOT → shot_start → face_start pulse → ST_IN_FACE
    --   ST_IN_FACE → frame_done → advance face_id → ST_WAIT_SHOT (or wrap)
    -- =========================================================================
    p_face_seq : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_soft_reset = '1' then
                s_face_state_r <= ST_IDLE;
                s_face_start_r <= '0';
                s_face_id_r    <= (others => '0');
                s_frame_id_r   <= (others => '0');
            else
                -- Default: clear pulse
                s_face_start_r <= '0';

                case s_face_state_r is

                    when ST_IDLE =>
                        if s_cmd_start = '1' then
                            s_face_id_r    <= (others => '0');
                            s_face_state_r <= ST_WAIT_SHOT;
                        end if;

                    when ST_WAIT_SHOT =>
                        if s_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif i_shot_start = '1' then
                            s_face_start_r <= '1';
                            s_face_state_r <= ST_IN_FACE;
                        end if;

                    when ST_IN_FACE =>
                        if s_cmd_stop = '1' then
                            s_face_state_r <= ST_IDLE;
                        elsif s_frame_done = '1' then
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
    -- [6] Timestamp counter (free-running, i_axis_aclk domain)
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
    -- [7] Error counter (increments on stop_id_error or hit overflow)
    -- =========================================================================
    p_error_cnt : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' or s_cmd_soft_reset = '1' then
                s_error_count_r <= (others => '0');
            else
                if s_stop_id_error /= C_ZEROS_CHIPS or
                   s_hit_dropped /= C_ZEROS_CHIPS then
                    s_error_count_r <= s_error_count_r + 1;
                end if;
            end if;
        end if;
    end process p_error_cnt;

    -- =========================================================================
    -- [8] Status aggregation (-> CSR -> STAT registers)
    -- =========================================================================
    s_status.busy              <= '1' when s_face_state_r /= ST_IDLE else '0';
    s_status.pipeline_overrun  <= '1' when s_chip_error_flags /= C_ZEROS_CHIPS
                                      else '0';
    s_status.bin_mismatch      <= '0';  -- Phase 2: calibration check
    s_status.lane_error_mask   <= s_errflag_sync;
    s_status.shot_seq_current  <= s_chip_shot_seq(0);
    s_status.vdma_frame_count  <= s_frame_id_r;
    s_status.error_count       <= s_error_count_r;

end architecture rtl;
