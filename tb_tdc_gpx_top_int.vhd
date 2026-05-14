-- =============================================================================
-- tb_tdc_gpx_top_int.vhd
-- TDC-GPX Controller - xpr integrated simulation TB (500 m / 64-bit, I-mode single)
-- =============================================================================
--
-- Purpose
--   Exercise the full tdc_gpx_top (4 chips * 4 stops * 4 clusters) in a single
--   AXIS/AXI/TDC clock domain, driving start_tdc / stop_tdc pulses like the
--   laser_ctrl testbench does, and follow the TDC-GPX datasheet
--   "I-Mode Single Measurement" flow per shot.
--
-- Scenario (single distance / single tdata width)
--   * distance        = 500 m     -> max_range_clks = 667 (round-trip @200 MHz)
--   * g_OUTPUT_WIDTH  = 64        -- default cell / VDMA stream width
--   * active chip mask = 4'hF, stops_per_chip = 2, cols_per_face = 2, n_faces = 1
--   * drain_mode      = count-known expected drain, with EF fallback disabled
--                       by fire_count final ownership for this shot
--
-- Flow (mirrors laser_ctrl tb's start_tdc / stop_tdc pattern)
--   [S0] reset -> pipeline/chip CSR default values settle
--   [S1] Pipeline CSR: write RANGE_COLS / MAIN_CTRL (500 m / 2 cols / 2 stops)
--   [S2] Chip CSR    : write cfg_image (Reg0, Reg5, Reg6) defaults
--   [S3] CFG_WRITE command pulse  (MAIN_CTRL[31] 1->0)
--   [S4] wait powerup + cfg_write + master reset (chip_busy = 0)
--   [S5] START command pulse (MAIN_CTRL[28] 1->0) -> cmd_start_accepted
--   [S6] Emulate laser_ctrl.o_start_tdc:
--          i_shot_start 1-clk pulse -> preload IFIFO -> assert IrFlag
--          wait for drain complete, deassert IrFlag, wait for ALU pulse
--          (shot #1, col 0)
--   [S7] Wait shot_period = 1000 cycles (500 m: 1.5 * round-trip)
--   [S8] Shot #2 (col 1) -> face complete, frame emitted on m_axis
--   [S9] Emulate laser_ctrl.o_stop_tdc:
--          i_stop_tdc pulse -> config_ctrl routes to chip_ctrl stop path
--   [S10] Check frame_done, print summary and exit
--
-- Notes
--   * Targets Xilinx xsim (VHDL-2008).
--   * Extends tb_tdc_gpx_chip_ctrl's behavioral chip model (FIFO fill, EF/LF,
--     IrFlag) to an array of 4 chips.
--   * Drives top-level stop_evt/fire_count ports directly with per-chip
--     expected counts. The physical IFIFO load is intentionally one word
--     larger than the expected count, proving chip_run stops by expected
--     count instead of merely draining to EF fallback.
--   * AXI-Lite writes are driven by px_axi_lite_writer from px_utility_pkg.
--   * ALL comments and text output are ASCII only (xsim rejects non-graphic
--     literals; also keeps the source encoding-agnostic for any editor).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;
use work.px_utility_pkg.all;

entity tb_tdc_gpx_top_int is
    generic (
        -- =================================================================
        -- INDEPENDENT (primary) variables. Every dependent constant below
        -- is derived from these, so changing the distance / clock / tdata
        -- width here propagates consistently through the whole TB (DUT
        -- generics + CSR values + chip model preload + timing waits).
        -- =================================================================
        G_AXIS_CLK_MHZ    : real    := 200.0;   -- common clock freq (MHz)
        G_MAX_RANGE_M     : real    := 500.0;   -- LiDAR max range (m)
        G_TDATA_WIDTH     : natural := 64;       -- VDMA tdata width (32|64|128)
        G_STOPS_PER_CHIP  : natural := 2;        -- active stops per chip (1..8)
        G_COLS_PER_FACE   : natural := 2;        -- shots per face
        G_N_FACES         : natural := 1;
        -- CTL21.max_hits_cfg timing mode:
        --   0: leave CTL21 unset     -> RTL alias 0 to 7
        --   1: write range max_hits before first packet_start
        --   2: write CTL21 max_hits=0 before first packet_start -> alias 7
        --   3: write range max_hits after first packet_start; face0 keeps old
        --      value, later faces can use the new value after CDC settle
        G_MAX_HITS_WRITE_MODE : natural := 1;
        -- All 4 chips active by default (top_int has no echo_receiver, so
        -- the 16-channel STAT packing limit does not apply).
        G_ACTIVE_CHIP_MASK: std_logic_vector(3 downto 0) := "1111";
        G_POWERUP_CLKS    : positive := 16;
        G_RECOVERY_CLKS   : positive := 4;
        G_ALU_PULSE_CLKS  : positive := 3;
        -- 0 = output sinks always ready. N = hold both output tready low for
        -- two clocks every N clocks to verify bounded downstream stall.
        G_BP_TREADY_GAP   : natural := 0;
        -- 0 = stall both rise/fall lanes, 1 = stall rise lane only,
        -- 2 = stall fall lane only. Used by C06 lane-imbalance stress.
        G_BP_LANE_MODE    : natural := 0;
        -- 0 = no recovery. 1 = normal run -> soft_reset -> normal run.
        -- 2 = normal run -> force_reinit -> normal run.
        G_RECOVERY_MODE   : natural := 0
    );
end entity tb_tdc_gpx_top_int;

architecture sim of tb_tdc_gpx_top_int is

    -- =========================================================================
    -- DEPENDENT (derived) constants -- all computed from the generics above.
    -- =========================================================================
    constant C_LIGHT_M_PER_US : real    := 299.792;
    constant C_CLK_PERIOD_PS  : natural := natural(1000000.0 / G_AXIS_CLK_MHZ);
    constant C_CLK_PERIOD     : time    := C_CLK_PERIOD_PS * 1 ps;
    constant C_RST_HOLD       : time    := 24 * C_CLK_PERIOD;

    -- Range-derived counters
    constant C_MAX_RANGE_CLKS : natural :=
        natural(2.0 * G_MAX_RANGE_M / C_LIGHT_M_PER_US * G_AXIS_CLK_MHZ);
    constant C_SHOT_PERIOD    : natural := (C_MAX_RANGE_CLKS * 3) / 2;

    -- max_hits table (see Doc/260419/task_distance_bounded_windows_2026-04-19.md)
    function fn_max_hits(r_m : real) return natural is
    begin
        if r_m <=  150.0 then return 1;
        elsif r_m <= 300.0 then return 2;
        elsif r_m <= 600.0 then return 3;
        elsif r_m <= 850.0 then return 6;
        else                   return 7;
        end if;
    end function;
    constant C_MAX_HITS : natural := fn_max_hits(G_MAX_RANGE_M);

    -- TDC sub-module generic override values
    constant C_OUTPUT_W     : natural := G_TDATA_WIDTH;
    constant C_KEEP_W       : natural := fn_axis_keep_width(C_OUTPUT_W);
    constant C_STOP_DW      : natural := c_STOP_EVT_DATA_WIDTH;  -- 32 from pkg

    -- Chip model fixed
    constant C_FIFO_DEPTH   : natural := 32;
    constant C_LF_THRESH    : natural := 4;

    -- Packed MAIN_CTRL / RANGE_COLS values for pipeline CSR
    function fn_pack_main_ctrl(mask  : std_logic_vector(3 downto 0);
                               faces : natural;
                               stops : natural) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v( 3 downto  0) := mask;
        v(14 downto 12) := std_logic_vector(to_unsigned(faces, 3));
        v(18 downto 15) := std_logic_vector(to_unsigned(stops, 4));
        return v;
    end function;

    function fn_pack_scan_timeout(max_hits : natural;
                                  scan_clks : natural) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(c_ST_MAX_SCAN_HI downto c_ST_MAX_SCAN_LO) :=
            std_logic_vector(to_unsigned(scan_clks, c_ST_MAX_SCAN_HI - c_ST_MAX_SCAN_LO + 1));
        v(c_ST_MAX_HITS_HI downto c_ST_MAX_HITS_LO) :=
            std_logic_vector(to_unsigned(max_hits, c_ST_MAX_HITS_HI - c_ST_MAX_HITS_LO + 1));
        return v;
    end function;

    function fn_face_axis_beats(cols_per_face  : natural;
                                active_chips   : natural;
                                stops_per_chip : natural;
                                max_hits       : natural;
                                tdata_width    : natural) return natural is
    begin
        return cols_per_face *
               (fn_hdr_prefix_beats(tdata_width) +
                active_chips * stops_per_chip *
                fn_beats_per_cell_rt(max_hits, tdata_width));
    end function;

    function fn_expected_axis_beats(mode             : natural;
                                    n_faces          : natural;
                                    target_face_beats : natural;
                                    default_face_beats : natural) return natural is
    begin
        if mode = 0 or mode = 2 then
            return n_faces * default_face_beats;
        elsif mode = 3 then
            if n_faces = 0 then
                return 0;
            elsif n_faces = 1 then
                return default_face_beats;
            else
                return default_face_beats + (n_faces - 1) * target_face_beats;
            end if;
        else
            return n_faces * target_face_beats;
        end if;
    end function;

    function fn_mode_name(mode : natural) return string is
    begin
        case mode is
            when 0      => return "unset";
            when 1      => return "early";
            when 2      => return "zero-alias";
            when 3      => return "late-after-packet-start";
            when others => return "invalid";
        end case;
    end function;

    function fn_recovery_runs(mode : natural) return natural is
    begin
        if mode = 0 then
            return 1;
        else
            return 2;
        end if;
    end function;

    function fn_recovery_mode_name(mode : natural) return string is
    begin
        case mode is
            when 0      => return "none";
            when 1      => return "soft_reset";
            when 2      => return "force_reinit";
            when others => return "invalid";
        end case;
    end function;

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal sim_done     : boolean   := false;

    -- =========================================================================
    -- AXI4-Lite #1: chip CSR (9-bit addr)
    -- =========================================================================
    signal s_axi_awvalid : std_logic := '0';
    signal s_axi_awready : std_logic;
    signal s_axi_awaddr  : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_axi_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_axi_wvalid  : std_logic := '0';
    signal s_axi_wready  : std_logic;
    signal s_axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axi_wstrb   : std_logic_vector(3 downto 0)  := "1111";
    signal s_axi_bvalid  : std_logic;
    signal s_axi_bready  : std_logic := '0';
    signal s_axi_bresp   : std_logic_vector(1 downto 0);
    signal s_axi_arvalid : std_logic := '0';
    signal s_axi_arready : std_logic;
    signal s_axi_araddr  : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal s_axi_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal s_axi_rvalid  : std_logic;
    signal s_axi_rready  : std_logic := '0';
    signal s_axi_rdata   : std_logic_vector(31 downto 0);
    signal s_axi_rresp   : std_logic_vector(1 downto 0);

    -- =========================================================================
    -- AXI4-Lite #2: pipeline CSR (7-bit addr)
    -- =========================================================================
    signal sp_awvalid : std_logic := '0';
    signal sp_awready : std_logic;
    signal sp_awaddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal sp_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal sp_wvalid  : std_logic := '0';
    signal sp_wready  : std_logic;
    signal sp_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal sp_wstrb   : std_logic_vector(3 downto 0)  := "1111";
    signal sp_bvalid  : std_logic;
    signal sp_bready  : std_logic := '0';
    signal sp_bresp   : std_logic_vector(1 downto 0);
    signal sp_arvalid : std_logic := '0';
    signal sp_arready : std_logic;
    signal sp_araddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal sp_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal sp_rvalid  : std_logic;
    signal sp_rready  : std_logic := '0';
    signal sp_rdata   : std_logic_vector(31 downto 0);
    signal sp_rresp   : std_logic_vector(1 downto 0);

    -- =========================================================================
    -- laser_ctrl output emulation (start_tdc / stop_tdc pulse)
    -- =========================================================================
    signal lc_start_tdc : std_logic := '0';   -- -> i_shot_start
    signal lc_stop_tdc  : std_logic := '0';   -- -> i_stop_tdc

    -- =========================================================================
    -- TDC-GPX physical pins (4 chips)
    -- =========================================================================
    signal io_tdc_d         : t_tdc_bus_array;
    signal o_tdc_adr        : t_tdc_adr_array;
    signal o_tdc_csn        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_rdn        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_wrn        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_oen        : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_stopdis    : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_alutrigger : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal o_tdc_puresn     : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal i_tdc_ef1        : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');
    signal i_tdc_ef2        : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '1');
    signal i_tdc_lf1        : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_lf2        : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_irflag     : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_errflag    : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Stop event AXI Stream (expected-count contract driver)
    -- =========================================================================
    signal stp_tvalid : std_logic := '0';
    signal stp_tdata  : std_logic_vector(C_STOP_DW - 1 downto 0) := (others => '0');
    signal stp_tkeep  : std_logic_vector(C_STOP_DW/8 - 1 downto 0) := (others => '0');
    signal stp_tuser  : std_logic_vector(C_STOP_DW - 1 downto 0) := (others => '0');
    signal stp_tready : std_logic;
    signal fire_count_tvalid : std_logic := '0';
    signal fire_count_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal fire_count_tkeep  : std_logic_vector(3 downto 0) := (others => '0');
    signal fire_count_tlast  : std_logic := '0';

    -- =========================================================================
    -- VDMA AXI-Stream master (rising / falling) - sink holds tready='1'
    -- =========================================================================
    signal m_rise_tdata  : std_logic_vector(C_OUTPUT_W - 1 downto 0);
    signal m_rise_tkeep  : std_logic_vector(C_KEEP_W - 1 downto 0);
    signal m_rise_tstrb  : std_logic_vector(C_KEEP_W - 1 downto 0);
    signal m_rise_tvalid : std_logic;
    signal m_rise_tlast  : std_logic;
    signal m_rise_tuser  : std_logic_vector(0 downto 0);
    signal m_rise_tready : std_logic := '1';

    signal m_fall_tdata  : std_logic_vector(C_OUTPUT_W - 1 downto 0);
    signal m_fall_tkeep  : std_logic_vector(C_KEEP_W - 1 downto 0);
    signal m_fall_tstrb  : std_logic_vector(C_KEEP_W - 1 downto 0);
    signal m_fall_tvalid : std_logic;
    signal m_fall_tlast  : std_logic;
    signal m_fall_tuser  : std_logic_vector(0 downto 0);
    signal m_fall_tready : std_logic := '1';

    -- =========================================================================
    -- Calibration constants
    -- =========================================================================
    signal i_lsr_tvalid       : std_logic := '0';
    signal i_lsr_tdata        : std_logic_vector(31 downto 0) := (others => '0');
    signal i_bin_resolution_ps : unsigned(15 downto 0) := to_unsigned(81, 16);
    signal i_k_dist_fixed      : unsigned(31 downto 0) := to_unsigned(54321, 32);
    signal o_irq, o_irq_pipe   : std_logic;

    -- =========================================================================
    -- 4-chip behavioral model state
    -- =========================================================================
    type t_fill_array is array (0 to c_N_CHIPS - 1) of natural;
    signal fifo1_fill : t_fill_array := (others => 0);
    signal fifo2_fill : t_fill_array := (others => 0);
    signal fifo1_rd_cnt : t_fill_array := (others => 0);
    signal fifo2_rd_cnt : t_fill_array := (others => 0);

    -- FIFO load request (stim pulses prior to each shot)
    signal fifo_load_req : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal fifo_load_n1  : t_fill_array := (others => 0);
    signal fifo_load_n2  : t_fill_array := (others => 0);

    -- Bus tri-state drive per chip
    type t_chip_d_array is array (0 to c_N_CHIPS - 1)
        of std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal chip_d_out : t_chip_d_array := (others => (others => '0'));
    signal chip_d_oe  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- Monitor counters
    signal mon_rise_beats     : natural := 0;
    signal mon_rise_frame_end : natural := 0;  -- tlast count (rising stream)
    signal mon_fall_beats     : natural := 0;
    signal mon_fall_frame_end : natural := 0;
    signal sim_cycle          : natural := 0;
    signal mon_rise_first_seen : std_logic := '0';
    signal mon_fall_first_seen : std_logic := '0';
    signal mon_rise_first_cycle : natural := 0;
    signal mon_fall_first_cycle : natural := 0;
    signal mon_rise_last_cycle  : natural := 0;
    signal mon_fall_last_cycle  : natural := 0;
    signal mon_bp_stall_cycles  : natural := 0;
    signal mon_irq_cnt          : natural := 0;
    signal mon_irq_pipe_cnt     : natural := 0;

    -- =========================================================================
    -- Pipeline CSR register offsets (csr_pipeline internal)
    --   CTL0 = 0x00 MAIN_CTRL   [31:28]=COMMAND, [22:19]=n_drain_cap,
    --                            [18:15]=stops, [14:12]=n_faces, [3:0]=mask
    --   CTL1 = 0x04 RANGE_COLS  [31:16]=cols_per_face, [15:0]=max_range_clks
    -- =========================================================================
    constant C_PIPE_MAIN_CTRL  : std_logic_vector(6 downto 0) := "0000000";  -- 0x00
    constant C_PIPE_RANGE_COLS : std_logic_vector(6 downto 0) := "0000100";  -- 0x04
    constant C_PIPE_AUX_CMD    : std_logic_vector(6 downto 0) := "0001000";  -- 0x08
    constant C_PIPE_STATUS     : std_logic_vector(6 downto 0) := "1010100";  -- 0x54
    constant C_PIPE_STATUS_EXT : std_logic_vector(6 downto 0) := "1011000";  -- 0x58
    constant C_PIPE_STATUS_EXT2: std_logic_vector(6 downto 0) := "1011100";  -- 0x5C

    -- MAIN_CTRL + RANGE_COLS packed from the entity generics so changing
    -- G_ACTIVE_CHIP_MASK / G_N_FACES / G_STOPS_PER_CHIP / G_COLS_PER_FACE /
    -- G_MAX_RANGE_M at instantiation time consistently updates both CSRs.
    constant C_MAIN_CTRL_BASE : std_logic_vector(31 downto 0) :=
        fn_pack_main_ctrl(G_ACTIVE_CHIP_MASK, G_N_FACES, G_STOPS_PER_CHIP);
    constant C_RANGE_COLS_VAL : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(G_COLS_PER_FACE, 16)) &
        std_logic_vector(to_unsigned(C_MAX_RANGE_CLKS, 16));
    constant C_ACTIVE_CHIPS : natural := fn_count_ones(G_ACTIVE_CHIP_MASK);
    constant C_TOTAL_LINES  : natural := G_N_FACES * G_COLS_PER_FACE;
    constant C_TARGET_FACE_BEATS : natural :=
        fn_face_axis_beats(G_COLS_PER_FACE, C_ACTIVE_CHIPS, G_STOPS_PER_CHIP,
                           C_MAX_HITS, C_OUTPUT_W);
    constant C_DEFAULT_FACE_BEATS : natural :=
        fn_face_axis_beats(G_COLS_PER_FACE, C_ACTIVE_CHIPS, G_STOPS_PER_CHIP,
                           c_MAX_HITS_PER_STOP, C_OUTPUT_W);
    constant C_EXPECTED_AXIS_BEATS : natural :=
        fn_expected_axis_beats(G_MAX_HITS_WRITE_MODE, G_N_FACES,
                               C_TARGET_FACE_BEATS, C_DEFAULT_FACE_BEATS);
    constant C_RECOVERY_RUNS : natural := fn_recovery_runs(G_RECOVERY_MODE);
    constant C_EXPECTED_TOTAL_LINES : natural := C_TOTAL_LINES * C_RECOVERY_RUNS;
    constant C_EXPECTED_TOTAL_AXIS_BEATS : natural := C_EXPECTED_AXIS_BEATS * C_RECOVERY_RUNS;

    -- Chip CSR addresses (9-bit)
    constant C_CHIP_CFG_REG0   : std_logic_vector(8 downto 0) := "0" & x"14";  -- CTL5
    constant C_CHIP_CFG_REG5   : std_logic_vector(8 downto 0) := "0" & x"28";  -- CTL10
    constant C_CHIP_CFG_REG6   : std_logic_vector(8 downto 0) := "0" & x"2C";
    constant C_CHIP_SCAN_CFG   : std_logic_vector(8 downto 0) := "0" & x"54";  -- CTL21

begin

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    clk <= not clk after C_CLK_PERIOD / 2 when not sim_done else '0';

    p_cycle : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                sim_cycle <= 0;
            else
                sim_cycle <= sim_cycle + 1;
            end if;
        end if;
    end process p_cycle;

    p_bp : process(clk)
        variable v_cnt : natural := 0;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_cnt := 0;
                m_rise_tready <= '1';
                m_fall_tready <= '1';
            elsif G_BP_TREADY_GAP > 0 then
                v_cnt := v_cnt + 1;
                if v_cnt < G_BP_TREADY_GAP then
                    m_rise_tready <= '1';
                    m_fall_tready <= '1';
                elsif v_cnt < G_BP_TREADY_GAP + 2 then
                    case G_BP_LANE_MODE is
                        when 1 =>
                            m_rise_tready <= '0';
                            m_fall_tready <= '1';
                        when 2 =>
                            m_rise_tready <= '1';
                            m_fall_tready <= '0';
                        when others =>
                            m_rise_tready <= '0';
                            m_fall_tready <= '0';
                    end case;
                    mon_bp_stall_cycles <= mon_bp_stall_cycles + 1;
                else
                    v_cnt := 0;
                    m_rise_tready <= '1';
                    m_fall_tready <= '1';
                end if;
            else
                m_rise_tready <= '1';
                m_fall_tready <= '1';
            end if;
        end if;
    end process p_bp;

    p_reset : process
    begin
        rst_n <= '0';
        wait for C_RST_HOLD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait;
    end process p_reset;

    -- =========================================================================
    -- 4-chip virtual TDC-GPX model
    -- =========================================================================
    -- EF / LF pins driven by fill levels
    gen_flags : for i in 0 to c_N_CHIPS - 1 generate
        i_tdc_ef1(i) <= '1' when fifo1_fill(i) = 0 else '0';
        i_tdc_ef2(i) <= '1' when fifo2_fill(i) = 0 else '0';
        i_tdc_lf1(i) <= '1' when fifo1_fill(i) >= C_LF_THRESH else '0';
        i_tdc_lf2(i) <= '1' when fifo2_fill(i) >= C_LF_THRESH else '0';
    end generate;

    -- Chip model body (FIFO state + writes per chip)
    gen_chip : for i in 0 to c_N_CHIPS - 1 generate

        p_chip : process(clk)
            variable v_rdn_prev  : std_logic := '1';
            variable v_load_prev : std_logic := '0';
            variable v_my_fill1  : natural   := 0;
            variable v_my_fill2  : natural   := 0;
            variable v_my_rd1    : natural   := 0;
            variable v_my_rd2    : natural   := 0;
        begin
            if rising_edge(clk) then
                if rst_n = '0' then
                    v_my_fill1 := 0;
                    v_my_fill2 := 0;
                    v_my_rd1   := 0;
                    v_my_rd2   := 0;
                    chip_d_oe(i)  <= '0';
                    chip_d_out(i) <= (others => '0');
                else
                    -- Default: D-bus hi-Z
                    chip_d_oe(i) <= '0';

                    -- FIFO load request (stim pulses prior to shot)
                    if fifo_load_req(i) = '1' and v_load_prev = '0' then
                        v_my_fill1 := fifo_load_n1(i);
                        v_my_fill2 := fifo_load_n2(i);
                        v_my_rd1   := 0;
                        v_my_rd2   := 0;
                    end if;
                    v_load_prev := fifo_load_req(i);

                    -- On READ strobe: drive D-bus with 28-bit TDC-GPX I-Mode
                    -- raw word.
                    --   [27:26] ChaCode  = 00
                    --   [25:18] StartNum = 0 (SINGLE_SHOT)
                    --   [17]    Slope    = 0 (rising)
                    --   [16: 0] Hit      = sequential test pattern
                    -- Chip addresses: IFIFO1 = 8, IFIFO2 = 9.
                    if o_tdc_oen(i) = '0' and o_tdc_rdn(i) = '0'
                       and o_tdc_csn(i) = '0' then
                        chip_d_oe(i) <= '1';
                        if o_tdc_adr(i) = c_TDC_REG8_IFIFO1 then
                            chip_d_out(i) <= "00" &                          -- ChaCode
                                             x"00" &                         -- StartNum
                                             '0' &                           -- Slope=rising
                                             std_logic_vector(to_unsigned(
                                                 (i * 256) + v_my_rd1 + 1,
                                                 c_RAW_HIT_WIDTH));
                        elsif o_tdc_adr(i) = c_TDC_REG9_IFIFO2 then
                            chip_d_out(i) <= "00" & x"00" & '0' &
                                             std_logic_vector(to_unsigned(
                                                 (i * 256) + 128 + v_my_rd2 + 1,
                                                 c_RAW_HIT_WIDTH));
                        else
                            chip_d_out(i) <= (others => '0');
                        end if;
                    end if;

                    -- RDN rising edge: FIFO pop
                    if o_tdc_rdn(i) = '1' and v_rdn_prev = '0' then
                        if o_tdc_adr(i) = c_TDC_REG8_IFIFO1 and v_my_fill1 > 0 then
                            v_my_fill1 := v_my_fill1 - 1;
                            v_my_rd1   := v_my_rd1 + 1;
                        elsif o_tdc_adr(i) = c_TDC_REG9_IFIFO2 and v_my_fill2 > 0 then
                            v_my_fill2 := v_my_fill2 - 1;
                            v_my_rd2   := v_my_rd2 + 1;
                        end if;
                    end if;
                    v_rdn_prev := o_tdc_rdn(i);

                    -- Publish fill state (consumed by gen_flags)
                    fifo1_fill(i)   <= v_my_fill1;
                    fifo2_fill(i)   <= v_my_fill2;
                    fifo1_rd_cnt(i) <= v_my_rd1;
                    fifo2_rd_cnt(i) <= v_my_rd2;
                end if;
            end if;
        end process p_chip;

        -- Tri-state bus
        io_tdc_d(i) <= chip_d_out(i) when chip_d_oe(i) = '1'
                                     else (others => 'Z');
    end generate gen_chip;

    -- =========================================================================
    -- DUT: tdc_gpx_top
    -- =========================================================================
    u_dut : entity work.tdc_gpx_top
        generic map (
            g_HW_VERSION     => x"00010000",
            g_OUTPUT_WIDTH   => C_OUTPUT_W,
            g_POWERUP_CLKS   => G_POWERUP_CLKS,
            g_RECOVERY_CLKS  => G_RECOVERY_CLKS,
            g_ALU_PULSE_CLKS => G_ALU_PULSE_CLKS,
            g_STOP_CNT_WIDTH  => c_STOP_CNT_WIDTH,
            g_STOP_EVT_DWIDTH => C_STOP_DW
        )
        port map (
            -- Common clock / reset (single domain)
            i_axis_aclk     => clk,
            i_axis_aresetn  => rst_n,
            i_tdc_clk       => clk,
            s_axi_aclk      => clk,
            s_axi_aresetn   => rst_n,
            -- Chip CSR (9-bit)
            s_axi_awvalid => s_axi_awvalid,
            s_axi_awready => s_axi_awready,
            s_axi_awaddr  => s_axi_awaddr,
            s_axi_awprot  => s_axi_awprot,
            s_axi_wvalid  => s_axi_wvalid,
            s_axi_wready  => s_axi_wready,
            s_axi_wdata   => s_axi_wdata,
            s_axi_wstrb   => s_axi_wstrb,
            s_axi_bvalid  => s_axi_bvalid,
            s_axi_bready  => s_axi_bready,
            s_axi_bresp   => s_axi_bresp,
            s_axi_arvalid => s_axi_arvalid,
            s_axi_arready => s_axi_arready,
            s_axi_araddr  => s_axi_araddr,
            s_axi_arprot  => s_axi_arprot,
            s_axi_rvalid  => s_axi_rvalid,
            s_axi_rready  => s_axi_rready,
            s_axi_rdata   => s_axi_rdata,
            s_axi_rresp   => s_axi_rresp,
            -- Pipeline CSR (7-bit)
            s_axi_pipe_awvalid => sp_awvalid,
            s_axi_pipe_awready => sp_awready,
            s_axi_pipe_awaddr  => sp_awaddr,
            s_axi_pipe_awprot  => sp_awprot,
            s_axi_pipe_wvalid  => sp_wvalid,
            s_axi_pipe_wready  => sp_wready,
            s_axi_pipe_wdata   => sp_wdata,
            s_axi_pipe_wstrb   => sp_wstrb,
            s_axi_pipe_bvalid  => sp_bvalid,
            s_axi_pipe_bready  => sp_bready,
            s_axi_pipe_bresp   => sp_bresp,
            s_axi_pipe_arvalid => sp_arvalid,
            s_axi_pipe_arready => sp_arready,
            s_axi_pipe_araddr  => sp_araddr,
            s_axi_pipe_arprot  => sp_arprot,
            s_axi_pipe_rvalid  => sp_rvalid,
            s_axi_pipe_rready  => sp_rready,
            s_axi_pipe_rdata   => sp_rdata,
            s_axi_pipe_rresp   => sp_rresp,
            -- laser_ctrl result stream (unused)
            i_lsr_tvalid => i_lsr_tvalid,
            i_lsr_tdata  => i_lsr_tdata,
            -- laser_ctrl pulses
            i_shot_start => lc_start_tdc,
            i_stop_tdc   => lc_stop_tdc,
            -- Stop event stream (unused; sink tready only)
            i_stop_evt_tvalid => stp_tvalid,
            i_stop_evt_tdata  => stp_tdata,
            i_stop_evt_tkeep  => stp_tkeep,
            i_stop_evt_tuser  => stp_tuser,
            o_stop_evt_tready => stp_tready,
            i_fire_count_tvalid => fire_count_tvalid,
            i_fire_count_tdata  => fire_count_tdata,
            i_fire_count_tkeep  => fire_count_tkeep,
            i_fire_count_tlast  => fire_count_tlast,
            -- TDC physical pins
            io_tdc_d         => io_tdc_d,
            o_tdc_adr        => o_tdc_adr,
            o_tdc_csn        => o_tdc_csn,
            o_tdc_rdn        => o_tdc_rdn,
            o_tdc_wrn        => o_tdc_wrn,
            o_tdc_oen        => o_tdc_oen,
            o_tdc_stopdis    => o_tdc_stopdis,
            o_tdc_alutrigger => o_tdc_alutrigger,
            o_tdc_puresn     => o_tdc_puresn,
            i_tdc_ef1        => i_tdc_ef1,
            i_tdc_ef2        => i_tdc_ef2,
            i_tdc_lf1        => i_tdc_lf1,
            i_tdc_lf2        => i_tdc_lf2,
            i_tdc_irflag     => i_tdc_irflag,
            i_tdc_errflag    => i_tdc_errflag,
            -- VDMA rising
            o_m_axis_tdata  => m_rise_tdata,
            o_m_axis_tkeep  => m_rise_tkeep,
            o_m_axis_tstrb  => m_rise_tstrb,
            o_m_axis_tvalid => m_rise_tvalid,
            o_m_axis_tlast  => m_rise_tlast,
            o_m_axis_tuser  => m_rise_tuser,
            i_m_axis_tready => m_rise_tready,
            -- VDMA falling
            o_m_axis_fall_tdata  => m_fall_tdata,
            o_m_axis_fall_tkeep  => m_fall_tkeep,
            o_m_axis_fall_tstrb  => m_fall_tstrb,
            o_m_axis_fall_tvalid => m_fall_tvalid,
            o_m_axis_fall_tlast  => m_fall_tlast,
            o_m_axis_fall_tuser  => m_fall_tuser,
            i_m_axis_fall_tready => m_fall_tready,
            -- Calibration
            i_bin_resolution_ps => i_bin_resolution_ps,
            i_k_dist_fixed      => i_k_dist_fixed,
            -- IRQ
            o_irq      => o_irq,
            o_irq_pipe => o_irq_pipe
        );

    -- =========================================================================
    -- Monitor: VDMA rising / falling beat counter
    -- =========================================================================
    p_mon : process(clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                mon_rise_beats     <= 0;
                mon_rise_frame_end <= 0;
                mon_fall_beats     <= 0;
                mon_fall_frame_end <= 0;
                mon_rise_first_seen <= '0';
                mon_fall_first_seen <= '0';
                mon_rise_first_cycle <= 0;
                mon_fall_first_cycle <= 0;
                mon_rise_last_cycle <= 0;
                mon_fall_last_cycle <= 0;
                mon_irq_cnt <= 0;
                mon_irq_pipe_cnt <= 0;
            else
                if o_irq = '1' then
                    mon_irq_cnt <= mon_irq_cnt + 1;
                end if;
                if o_irq_pipe = '1' then
                    mon_irq_pipe_cnt <= mon_irq_pipe_cnt + 1;
                end if;
                if m_rise_tvalid = '1' and m_rise_tready = '1' then
                    assert m_rise_tkeep = (m_rise_tkeep'range => '1')
                        report "top_int: rising tkeep must be all ones on accepted output beats"
                        severity error;
                    assert m_rise_tstrb = (m_rise_tstrb'range => '1')
                        report "top_int: rising tstrb must be all ones on accepted output beats"
                        severity error;
                    if mon_rise_first_seen = '0' then
                        mon_rise_first_seen <= '1';
                        mon_rise_first_cycle <= sim_cycle;
                        report "C06_MARKER T4_FIRST_RISE_BEAT cycle="
                               & integer'image(sim_cycle)
                               & " time=" & time'image(now)
                            severity note;
                    end if;
                    mon_rise_beats <= mon_rise_beats + 1;
                    if m_rise_tlast = '1' then
                        mon_rise_frame_end <= mon_rise_frame_end + 1;
                        mon_rise_last_cycle <= sim_cycle;
                        report "C06_MARKER T5_RISE_TLAST cycle="
                               & integer'image(sim_cycle)
                               & " time=" & time'image(now)
                            severity note;
                    end if;
                end if;
                if m_fall_tvalid = '1' and m_fall_tready = '1' then
                    assert m_fall_tkeep = (m_fall_tkeep'range => '1')
                        report "top_int: falling tkeep must be all ones on accepted output beats"
                        severity error;
                    assert m_fall_tstrb = (m_fall_tstrb'range => '1')
                        report "top_int: falling tstrb must be all ones on accepted output beats"
                        severity error;
                    if mon_fall_first_seen = '0' then
                        mon_fall_first_seen <= '1';
                        mon_fall_first_cycle <= sim_cycle;
                        report "C06_MARKER T4_FIRST_FALL_BEAT cycle="
                               & integer'image(sim_cycle)
                               & " time=" & time'image(now)
                            severity note;
                    end if;
                    mon_fall_beats <= mon_fall_beats + 1;
                    if m_fall_tlast = '1' then
                        mon_fall_frame_end <= mon_fall_frame_end + 1;
                        mon_fall_last_cycle <= sim_cycle;
                        report "C06_MARKER T5_FALL_TLAST cycle="
                               & integer'image(sim_cycle)
                               & " time=" & time'image(now)
                            severity note;
                    end if;
                end if;
            end if;
        end if;
    end process p_mon;

    -- =========================================================================
    -- Stimulus
    -- =========================================================================
    p_stim : process

        ----------------------------------------------------------------
        -- print helper (timestamp + message), ASCII only
        ----------------------------------------------------------------
        procedure pl(msg : string) is
            variable lv : line;
        begin
            write(lv, now, right, 12);
            write(lv, string'("  "));
            write(lv, msg);
            writeline(output, lv);
        end procedure;

        ----------------------------------------------------------------
        -- Wait N clocks
        ----------------------------------------------------------------
        procedure wait_clk(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(clk);
            end loop;
        end procedure;

        ----------------------------------------------------------------
        -- AXI4-Lite write: Pipeline CSR (7-bit addr)
        -- Handshake is owned by px_utility_pkg; this wrapper only selects ports.
        ----------------------------------------------------------------
        procedure pipe_wr(addr : std_logic_vector(6 downto 0);
                          val  : std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => addr,
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => sp_awaddr,
                axi_awprot  => sp_awprot,
                axi_awvalid => sp_awvalid,
                axi_awready => sp_awready,
                axi_wdata   => sp_wdata,
                axi_wstrb   => sp_wstrb,
                axi_wvalid  => sp_wvalid,
                axi_wready  => sp_wready,
                axi_bresp   => sp_bresp,
                axi_bvalid  => sp_bvalid,
                axi_bready  => sp_bready
            );
        end procedure;

        ----------------------------------------------------------------
        -- AXI4-Lite read: Pipeline CSR (7-bit addr)
        -- Handshake and optional compare are owned by px_utility_pkg.
        ----------------------------------------------------------------
        procedure pipe_rd(addr          : std_logic_vector(6 downto 0);
                          expected      : std_logic_vector(31 downto 0);
                          compare_en    : std_logic;
                          fail_on_error : std_logic) is
        begin
            px_axi_lite_reader(
                addr          => addr,
                val           => expected,
                comp          => compare_en,
                fail_on_error => fail_on_error,
                axi_aclk      => clk,
                axi_araddr    => sp_araddr,
                axi_arprot    => sp_arprot,
                axi_arvalid   => sp_arvalid,
                axi_arready   => sp_arready,
                axi_rdata     => sp_rdata,
                axi_rresp     => sp_rresp,
                axi_rvalid    => sp_rvalid,
                axi_rready    => sp_rready
            );
        end procedure;

        ----------------------------------------------------------------
        -- AXI4-Lite write: Chip CSR (9-bit addr)
        -- Handshake is owned by px_utility_pkg; this wrapper only selects ports.
        ----------------------------------------------------------------
        procedure chip_wr(addr : std_logic_vector(8 downto 0);
                          val  : std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => addr,
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => s_axi_awaddr,
                axi_awprot  => s_axi_awprot,
                axi_awvalid => s_axi_awvalid,
                axi_awready => s_axi_awready,
                axi_wdata   => s_axi_wdata,
                axi_wstrb   => s_axi_wstrb,
                axi_wvalid  => s_axi_wvalid,
                axi_wready  => s_axi_wready,
                axi_bresp   => s_axi_bresp,
                axi_bvalid  => s_axi_bvalid,
                axi_bready  => s_axi_bready
            );
        end procedure;

        ----------------------------------------------------------------
        -- Pipeline status readback checkpoint.
        -- STAT5 is expected to be clean/idle. STAT6 is logged without exact
        -- compare because run_drain_complete_mask is a valid non-zero result.
        -- STAT7 is expected to stay zero in the nominal recovery scenario.
        ----------------------------------------------------------------
        procedure read_pipeline_status(tag : string) is
        begin
            wait_clk(64);
            pl("C06_MARKER T7_STATUS_READ tag=" & tag
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            pipe_rd(C_PIPE_STATUS, x"00000000", '1', '1');
            pipe_rd(C_PIPE_STATUS_EXT, x"00000000", '0', '0');
            pipe_rd(C_PIPE_STATUS_EXT2, x"00000000", '1', '1');
        end procedure;

        ----------------------------------------------------------------
        -- Load all chip FIFOs simultaneously (prior to shot)
        ----------------------------------------------------------------
        procedure load_all_fifos(n1 : natural; n2 : natural) is
        begin
            for i in 0 to c_N_CHIPS - 1 loop
                fifo_load_n1(i) <= n1;
                fifo_load_n2(i) <= n2;
            end loop;
            wait until rising_edge(clk);
            fifo_load_req <= (others => '1');
            wait until rising_edge(clk);
            fifo_load_req <= (others => '0');
            wait until rising_edge(clk);
        end procedure;

        ----------------------------------------------------------------
        -- Emit one per-chip expected-count update and its final qualifier.
        -- Layout follows tdc_gpx_pkg.vhd:
        --   tdata[chip*8+3:chip*8]   = IFIFO1 running total
        --   tdata[chip*8+7:chip*8+4] = IFIFO2 running total
        -- tuser is kept zero, so stop_cfg_decode observes exactly tdata.
        ----------------------------------------------------------------
        procedure emit_expected_counts(shot_count : natural;
                                       ififo1_cnt : natural;
                                       ififo2_cnt : natural) is
            variable v_data : std_logic_vector(C_STOP_DW - 1 downto 0);
            variable v_user : std_logic_vector(C_STOP_DW - 1 downto 0);
            variable v_lo   : natural;
        begin
            assert ififo1_cnt <= 15 and ififo2_cnt <= 15
                report "tb_tdc_gpx_top_int: expected-count field overflow"
                severity failure;

            v_data := (others => '0');
            v_user := (others => '0');
            for i in 0 to c_N_CHIPS - 1 loop
                v_lo := i * 8;
                if G_ACTIVE_CHIP_MASK(i) = '1' then
                    v_data(v_lo + 3 downto v_lo) :=
                        std_logic_vector(to_unsigned(ififo1_cnt, 4));
                    v_data(v_lo + 7 downto v_lo + 4) :=
                        std_logic_vector(to_unsigned(ififo2_cnt, 4));
                end if;
            end loop;

            wait until rising_edge(clk);
            stp_tdata  <= v_data;
            stp_tuser  <= v_user;
            stp_tkeep  <= (others => '1');
            stp_tvalid <= '1';
            fire_count_tdata  <= std_logic_vector(to_unsigned(shot_count, 32));
            fire_count_tkeep  <= (others => '1');
            fire_count_tlast  <= '0';
            fire_count_tvalid <= '1';

            wait until rising_edge(clk);
            assert stp_tready = '1'
                report "tb_tdc_gpx_top_int: stop_evt_tready was not asserted"
                severity failure;
            stp_tvalid <= '0';
            stp_tdata  <= (others => '0');
            stp_tuser  <= (others => '0');
            stp_tkeep  <= (others => '0');
            fire_count_tvalid <= '0';

            wait until rising_edge(clk);
            fire_count_tdata  <= std_logic_vector(to_unsigned(shot_count, 32));
            fire_count_tkeep  <= (others => '1');
            fire_count_tlast  <= '1';
            fire_count_tvalid <= '1';
            pl("C06_MARKER T1_FIRE_COUNT_FINAL shot="
               & integer'image(shot_count)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));

            wait until rising_edge(clk);
            fire_count_tvalid <= '0';
            fire_count_tlast  <= '0';
            fire_count_tkeep  <= (others => '0');
            fire_count_tdata  <= (others => '0');
        end procedure;

        ----------------------------------------------------------------
        -- One I-Mode single-measurement shot
        --   1) i_shot_start (start_tdc) pulse
        --   2) Preload physical IFIFO with expected+1 words
        --   3) Emit expected counts + fire_count final for this shot
        --   4) Assert IrFlag (emulates MTimer expiry)
        --   5) Wait for expected-count-bounded drain
        --   6) Deassert IrFlag and assert exact read count
        ----------------------------------------------------------------
        procedure do_shot(shot_count : natural; expected_hits : natural; tag : string) is
            constant c_PHYSICAL_HITS : natural := expected_hits + 1;
        begin
            pl("  ---- shot [" & tag & "] start, expected="
               & integer'image(expected_hits)
               & " physical=" & integer'image(c_PHYSICAL_HITS));

            -- start_tdc (= i_shot_start), emulating laser_ctrl.o_start_tdc
            pl("C06_MARKER T0_START_TDC shot="
               & integer'image(shot_count)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            lc_start_tdc <= '1';
            wait_clk(1);
            lc_start_tdc <= '0';

            -- Preload IFIFO above expected. A fallback-to-EF drain would read
            -- the extra word and fail the exact count check below.
            wait_clk(4);
            load_all_fifos(c_PHYSICAL_HITS, c_PHYSICAL_HITS);

            emit_expected_counts(shot_count, expected_hits, expected_hits);

            -- Let the expected tuple cross config_ctrl's xpm_cdc_handshake
            -- before IrFlag makes chip_run snapshot it at ST_DRAIN_LATCH.
            wait_clk(80);

            -- Assert IrFlag after a few clk (MTimer expiry emulation)
            pl("C06_MARKER T2_IRFLAG_ASSERT shot="
               & integer'image(shot_count)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            i_tdc_irflag <= (others => '1');

            -- Wait for drain (max ~5 us = 1000 clks)
            wait_clk(1000);

            -- Deassert IrFlag + wait ALU recovery
            pl("C06_MARKER T3_DRAIN_WAIT_END shot="
               & integer'image(shot_count)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            i_tdc_irflag <= (others => '0');
            wait_clk(G_ALU_PULSE_CLKS + G_RECOVERY_CLKS + 8);

            for i in 0 to c_N_CHIPS - 1 loop
                if G_ACTIVE_CHIP_MASK(i) = '1' then
                    assert fifo1_rd_cnt(i) = expected_hits
                        report "tb_tdc_gpx_top_int: IFIFO1 expected-count drain mismatch on chip "
                               & integer'image(i) & ", actual="
                               & integer'image(fifo1_rd_cnt(i)) & ", expected="
                               & integer'image(expected_hits)
                        severity failure;
                    assert fifo2_rd_cnt(i) = expected_hits
                        report "tb_tdc_gpx_top_int: IFIFO2 expected-count drain mismatch on chip "
                               & integer'image(i) & ", actual="
                               & integer'image(fifo2_rd_cnt(i)) & ", expected="
                               & integer'image(expected_hits)
                        severity failure;
                    assert fifo1_fill(i) = c_PHYSICAL_HITS - expected_hits
                        report "tb_tdc_gpx_top_int: IFIFO1 did not stop before EF fallback on chip "
                               & integer'image(i)
                        severity failure;
                    assert fifo2_fill(i) = c_PHYSICAL_HITS - expected_hits
                        report "tb_tdc_gpx_top_int: IFIFO2 did not stop before EF fallback on chip "
                               & integer'image(i)
                        severity failure;
                end if;
            end loop;
            pl("  ---- shot [" & tag
               & "] expected-count bound PASS: read="
               & integer'image(expected_hits)
               & ", leftover="
               & integer'image(c_PHYSICAL_HITS - expected_hits));
            pl("  ---- shot [" & tag & "] drain done");
        end procedure;

        ----------------------------------------------------------------
        -- Start/run/stop helpers for optional recovery regression.
        ----------------------------------------------------------------
        procedure start_measurement_run(run_idx : natural) is
        begin
            pl("[S5] START command pulse -> face_seq active, run="
               & integer'image(run_idx));
            pipe_wr(C_PIPE_MAIN_CTRL, (C_MAIN_CTRL_BASE or x"10000000"));
            wait_clk(4);
            pipe_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
            if G_RECOVERY_MODE = 1 and run_idx > 1 then
                -- Soft reset enters chip_ctrl PH_RESP_DRAIN before re-init.
                -- The pipeline CSR holds START pending until face_seq accepts,
                -- so the TB waits for the worst-case black-box recovery window
                -- before applying shot stimulus.
                wait_clk(12000);
            else
                wait_clk(80);  -- cmd_start_accepted latency + CDC
            end if;
        end procedure;

        procedure run_all_shots(run_idx : natural) is
        begin
            for f in 0 to G_N_FACES - 1 loop
                for c in 0 to G_COLS_PER_FACE - 1 loop
                    do_shot(c + 1,
                            G_STOPS_PER_CHIP,
                            "run" & integer'image(run_idx)
                            & "/face" & integer'image(f)
                            & "/col" & integer'image(c));

                    if G_MAX_HITS_WRITE_MODE = 3 and f = 0 and c = 0 then
                        pl("[S_LATE] CTL21.max_hits_cfg write after first packet_start -> "
                           & integer'image(C_MAX_HITS));
                        chip_wr(C_CHIP_SCAN_CFG, fn_pack_scan_timeout(C_MAX_HITS, 0));
                        wait_clk(20);
                    end if;

                    if not (f = G_N_FACES - 1 and c = G_COLS_PER_FACE - 1) then
                        pl("[S7] shot_period wait (500m -> 1000 clks)");
                        wait_clk(C_SHOT_PERIOD);
                    end if;
                end loop;
            end loop;
        end procedure;

        procedure stop_measurement_run(run_idx : natural) is
        begin
            pl("[S9] wait VDMA frame emit, then stop_tdc pulse, run="
               & integer'image(run_idx));
            wait_clk(2000);

            -- laser_ctrl.o_stop_tdc emulation
            pl("C06_MARKER T6_STOP_TDC run="
               & integer'image(run_idx)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            lc_stop_tdc <= '1';
            wait_clk(2);
            lc_stop_tdc <= '0';
            wait_clk(500);
        end procedure;

        procedure issue_recovery(mode : natural) is
        begin
            case mode is
                when 1 =>
                    pl("C06_MARKER T8_SOFT_RESET cycle="
                       & integer'image(sim_cycle)
                       & " time=" & time'image(now));
                    pipe_wr(C_PIPE_MAIN_CTRL, (C_MAIN_CTRL_BASE or x"40000000"));
                    wait_clk(4);
                    pipe_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
                    wait_clk(2800);
                    read_pipeline_status("post-soft-reset");
                when 2 =>
                    pl("C06_MARKER T8_FORCE_REINIT cycle="
                       & integer'image(sim_cycle)
                       & " time=" & time'image(now));
                    pipe_wr(C_PIPE_AUX_CMD, x"00000001");
                    wait_clk(4);
                    pipe_wr(C_PIPE_AUX_CMD, x"00000000");
                    wait_clk(2800);
                    read_pipeline_status("post-force-reinit");
                when others =>
                    null;
            end case;
        end procedure;

    begin
        -- Wait reset deassert
        wait until rst_n = '1';
        wait_clk(10);

        assert G_MAX_HITS_WRITE_MODE <= 3
            report "tb_tdc_gpx_top_int: G_MAX_HITS_WRITE_MODE must be 0..3"
            severity failure;
        assert G_RECOVERY_MODE <= 2
            report "tb_tdc_gpx_top_int: G_RECOVERY_MODE must be 0..2"
            severity failure;
        assert G_BP_LANE_MODE <= 2
            report "tb_tdc_gpx_top_int: G_BP_LANE_MODE must be 0..2"
            severity failure;
        assert not (G_MAX_HITS_WRITE_MODE = 3 and G_N_FACES < 2)
            report "tb_tdc_gpx_top_int: late max_hits mode requires G_N_FACES >= 2"
            severity failure;

        pl("====================================================");
        pl(" tdc_gpx_top integrated sim start (500m / "
           & integer'image(G_TDATA_WIDTH) & "-bit / I-mode / max_hits_mode="
           & fn_mode_name(G_MAX_HITS_WRITE_MODE)
           & " / recovery=" & fn_recovery_mode_name(G_RECOVERY_MODE) & ")");
        pl("====================================================");

        ----------------------------------------------------------------
        -- [S1] Pipeline CSR setup
        ----------------------------------------------------------------
        pl("[S1] Pipeline CSR: RANGE_COLS / MAIN_CTRL write");
        pipe_wr(C_PIPE_RANGE_COLS, C_RANGE_COLS_VAL);   -- 500m / 2 cols
        wait_clk(4);
        pipe_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);    -- mask=F, n_faces=1, stops=2
        wait_clk(20);

        ----------------------------------------------------------------
        -- [S2] Chip CSR: cfg_image defaults (Reg0 / Reg5 / Reg6)
        --   Reg0 : TRiseEn (TStart + TStop1..2) bits 10,11,12 -> 0x0000_1C00
        --   Reg5 : MasterAluTrig (bit 23) + PartialAluTrig (bit 24) -> 0x0180_0000
        --   Reg6 : LF threshold = 4 -> 0x0000_0004
        ----------------------------------------------------------------
        pl("[S2] Chip CSR: cfg_image Reg0/Reg5/Reg6 write");
        chip_wr(C_CHIP_CFG_REG0, x"00001C00");  -- Reg0: rise edge on start+stop1..2
        chip_wr(C_CHIP_CFG_REG5, x"01800000");  -- Reg5: ALU trig bits
        chip_wr(C_CHIP_CFG_REG6, x"00000004");  -- Reg6: LF threshold
        case G_MAX_HITS_WRITE_MODE is
            when 0 =>
                pl("[S2] CTL21.max_hits_cfg left unset -> expect RTL alias to 7");
            when 1 =>
                pl("[S2] CTL21.max_hits_cfg early write -> "
                   & integer'image(C_MAX_HITS));
                chip_wr(C_CHIP_SCAN_CFG, fn_pack_scan_timeout(C_MAX_HITS, 0));
            when 2 =>
                pl("[S2] CTL21.max_hits_cfg early write 0 -> expect alias to 7");
                chip_wr(C_CHIP_SCAN_CFG, fn_pack_scan_timeout(0, 0));
            when 3 =>
                pl("[S2] CTL21.max_hits_cfg late mode: skip early write");
            when others =>
                null;
        end case;
        wait_clk(20);

        ----------------------------------------------------------------
        -- [S3] CFG_WRITE command pulse
        --   MAIN_CTRL[31] is cfg_write bit. Rising edge -> 1-cycle pulse.
        ----------------------------------------------------------------
        pl("[S3] CFG_WRITE command pulse");
        pipe_wr(C_PIPE_MAIN_CTRL, (C_MAIN_CTRL_BASE or x"80000000"));
        wait_clk(4);
        pipe_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
        -- cfg_write -> CDC -> chip_init 11 writes + master reset completion:
        --   11 regs * (bus_ticks=5 * div=2) + powerup + recovery ~= 2000 clks
        wait_clk(2500);

        ----------------------------------------------------------------
        -- [S4] Wait until cfg_write sequence finishes (chip_busy = 0)
        ----------------------------------------------------------------
        pl("[S4] wait cfg_write completion");
        wait_clk(200);

        start_measurement_run(1);
        run_all_shots(1);
        stop_measurement_run(1);
        read_pipeline_status("after-run1");

        if G_RECOVERY_MODE > 0 then
            issue_recovery(G_RECOVERY_MODE);
            start_measurement_run(2);
            run_all_shots(2);
            stop_measurement_run(2);
            read_pipeline_status("after-recovery-run");
        end if;

        ----------------------------------------------------------------
        -- [S10] Summary
        ----------------------------------------------------------------
        pl("====================================================");
        pl(" integrated sim end");
        pl("  config          : width=" & integer'image(G_TDATA_WIDTH)
           & "  max_hits=" & integer'image(C_MAX_HITS)
           & "  max_hits_mode=" & fn_mode_name(G_MAX_HITS_WRITE_MODE)
           & "  active_chips=" & integer'image(C_ACTIVE_CHIPS)
           & "  stops_per_chip=" & integer'image(G_STOPS_PER_CHIP)
           & "  faces=" & integer'image(G_N_FACES)
           & "  cols=" & integer'image(G_COLS_PER_FACE)
           & "  expected_beats_per_run=" & integer'image(C_EXPECTED_AXIS_BEATS)
           & "  recovery_runs=" & integer'image(C_RECOVERY_RUNS)
           & "  expected_beats_total=" & integer'image(C_EXPECTED_TOTAL_AXIS_BEATS)
           & "  bp_gap=" & integer'image(G_BP_TREADY_GAP)
           & "  bp_lane_mode=" & integer'image(G_BP_LANE_MODE)
           & "  recovery=" & fn_recovery_mode_name(G_RECOVERY_MODE));
        pl("  rising  stream  : beats=" & integer'image(mon_rise_beats)
           & "  tlast_cnt=" & integer'image(mon_rise_frame_end));
        pl("  falling stream  : beats=" & integer'image(mon_fall_beats)
           & "  tlast_cnt=" & integer'image(mon_fall_frame_end));
        pl("  c06 timing       : rise_first_cycle=" & integer'image(mon_rise_first_cycle)
           & "  rise_last_cycle=" & integer'image(mon_rise_last_cycle)
           & "  fall_first_cycle=" & integer'image(mon_fall_first_cycle)
           & "  fall_last_cycle=" & integer'image(mon_fall_last_cycle)
           & "  bp_stall_cycles=" & integer'image(mon_bp_stall_cycles));
        pl("  irq counters     : o_irq=" & integer'image(mon_irq_cnt)
           & "  o_irq_pipe=" & integer'image(mon_irq_pipe_cnt));
        pl("====================================================");
        assert mon_rise_beats > 0
            report "tb_tdc_gpx_top_int: no rising beats observed"
            severity error;
        assert mon_fall_beats > 0
            report "tb_tdc_gpx_top_int: no falling beats observed"
            severity error;
        assert mon_rise_frame_end = C_EXPECTED_TOTAL_LINES
            report "tb_tdc_gpx_top_int: rising tlast count mismatch"
            severity error;
        assert mon_fall_frame_end = C_EXPECTED_TOTAL_LINES
            report "tb_tdc_gpx_top_int: falling tlast count mismatch"
            severity error;
        assert mon_rise_beats = C_EXPECTED_TOTAL_AXIS_BEATS
            report "tb_tdc_gpx_top_int: rising beat count mismatch"
            severity error;
        assert mon_fall_beats = C_EXPECTED_TOTAL_AXIS_BEATS
            report "tb_tdc_gpx_top_int: falling beat count mismatch"
            severity error;
        assert mon_irq_pipe_cnt = 0
            report "tb_tdc_gpx_top_int: o_irq_pipe must stay reserved/low"
            severity error;
        assert mon_irq_cnt = 0
            report "tb_tdc_gpx_top_int: o_irq must not fire in normal C06 top_int run"
            severity error;
        if G_BP_TREADY_GAP > 0 then
            assert mon_bp_stall_cycles > 0
                report "tb_tdc_gpx_top_int: backpressure mode did not create stall cycles"
                severity error;
            report "tb_tdc_gpx_top_int: bounded output backpressure preserved beats/tlast - PASS"
                severity note;
        end if;
        if G_RECOVERY_MODE = 1 then
            report "tb_tdc_gpx_top_int: recovery mode soft_reset PASS"
                severity note;
        elsif G_RECOVERY_MODE = 2 then
            report "tb_tdc_gpx_top_int: recovery mode force_reinit PASS"
                severity note;
        end if;
        report "tb_tdc_gpx_top_int: output streams emitted beats/tlast as expected - PASS"
            severity note;

        sim_done <= true;
        wait;
    end process p_stim;

    -- =========================================================================
    -- Watchdog: full-sim timeout
    -- =========================================================================
    p_wdog : process
    begin
        wait for 300 us;
        if not sim_done then
            report "tb_tdc_gpx_top_int: watchdog timeout (200 us)"
                severity failure;
        end if;
        wait;
    end process p_wdog;

end architecture sim;
