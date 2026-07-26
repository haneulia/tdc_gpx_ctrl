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
--   * distance        = 500 m     -> max_range_5ns_ticks = 668 (ceil, 5 ns unit)
--   * g_OUTPUT_WIDTH  = 64        -- default cell / VDMA stream width
--   * active chip mask = 4'hF, stops_per_chip = 2, cols_per_face = 2, n_faces = 1
--   * drain_mode      = LF burst optimization with EF-authoritative completion
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
--   [S7] Wait shot_period = 1002 cycles (500 m: ceil(1.5 * round-trip))
--   [S8] Shot #2 (col 1) -> face complete, frame emitted on m_axis
--   [S9] Emulate laser_ctrl.o_stop_tdc:
--          i_stop_tdc pulse -> config_ctrl routes to chip_ctrl stop path
--   [S10] Check frame_done, print summary and exit
--
-- Notes
--   * Targets Xilinx xsim (VHDL-2008).
--   * Extends tb_tdc_gpx_chip_ctrl's behavioral chip model (FIFO fill, EF/LF,
--     IrFlag) to an array of 4 chips.
--   * Loads the behavioral GPX FIFOs directly. Drain length is determined by
--     the GPX EF pins and accepted read responses only.
--   * AXI-Lite writes are driven by px_axi_lite_writer from px_utility_pkg.
--   * ALL comments and text output are ASCII only (xsim rejects non-graphic
--     literals; also keeps the source encoding-agnostic for any editor).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
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
        G_AXIS_CLK_MHZ    : real    := 150.0;   -- Processing clock (MHz)
        G_TDC_CLK_MHZ     : real    := 200.0;   -- TDC bus/control clock (MHz)
        G_MAX_RANGE_M     : real    := 500.0;   -- LiDAR max range (m)
        G_TDATA_WIDTH     : natural := 64;       -- VDMA tdata width (32|64|128)
        G_RISE_CHIP_MASK  : std_logic_vector(3 downto 0) := c_DEFAULT_RISE_CHIP_MASK;
        G_FALL_CHIP_MASK  : std_logic_vector(3 downto 0) := c_DEFAULT_FALL_CHIP_MASK;
        G_FALLING_ENABLE  : boolean := true;
        G_CHECK_32CH_ORDER : boolean := false;
        G_CHECK_EDGE_ROLE_WRITES : boolean := true;
        G_PRESENT_CHIP_MASK : std_logic_vector(3 downto 0) := c_ALL_CHIPS_MASK;
        G_BUILD_MAX_STOPS_PER_CHIP : positive range 2 to c_MAX_STOPS_PER_CHIP :=
            c_MAX_STOPS_PER_CHIP;
        G_BUILD_MAX_HITS_PER_STOP : positive range 1 to c_MAX_HITS_PER_STOP :=
            c_MAX_HITS_PER_STOP;
        G_STOPS_PER_CHIP  : natural := 2;        -- active stops per chip (1..8)
        G_COLS_PER_FACE   : natural := 2;        -- shots per face
        G_N_FACES         : positive range 1 to 5 := 1;
        -- Behavioral IFIFO load per stop/channel. Default 1 keeps the
        -- historical top_int smoke tests small; target 7 models 7 echoes
        -- per channel.
        G_ECHOES_PER_STOP : natural := 1;
        -- 0 = derive from G_MAX_RANGE_M. 1..7 = force CTL21.max_hits_cfg
        -- for C07 width/max_hits chain stress.
        G_MAX_HITS_OVERRIDE : natural := 0;
        -- CTL21.max_hits_cfg timing mode:
        --   0: leave CTL21 unset     -> RTL alias 0 to 7
        --   1: write range max_hits before first packet_start
        --   2: write CTL21 max_hits=0 before first packet_start -> alias 7
        --   3: write range max_hits after first packet_start; face0 keeps old
        --      value, later faces can use the new value after CDC settle
        G_MAX_HITS_WRITE_MODE : natural := 1;
        -- All 4 physical GPX chips are active by default.
        G_ACTIVE_CHIP_MASK: std_logic_vector(3 downto 0) := "1111";
        -- Bit i = 1 drives chip i raw events as rising slope; bit i = 0
        -- drives falling slope. Target 4-chip split is "0011":
        -- chip0/1 rising, chip2/3 falling.
        G_CHIP_SLOPE_MASK : std_logic_vector(3 downto 0) := "0011";
        -- Force raw Hit[16]=1 in the behavioral GPX model. This makes
        -- long-range metadata preservation visible at final VDMA output.
        G_FORCE_HIT16     : boolean := false;
        -- Optional release check: require at least this many metadata beats
        -- per slope stream to carry non-zero [6:0] Hit[16] vectors.
        G_EXPECT_HIT16_META_MIN : natural := 0;
        G_POWERUP_CLKS    : positive := 16;
        G_RECOVERY_CLKS   : positive := 4;
        G_ALU_PULSE_CLKS  : positive := 3;
        -- Passed through to tdc_gpx_top.g_STREAM_CLK_MODE.
        -- "ASYNC" uses raw CDC FIFO; "SYNC" bypasses raw CDC in config_ctrl.
        G_STREAM_CLK_MODE : string := "ASYNC";
        -- 0 = output sinks always ready. N = hold both output tready low for
        -- two clocks every N clocks to verify bounded downstream stall.
        G_BP_TREADY_GAP   : natural := 0;
        -- 0 = stall both rise/fall lanes, 1 = stall rise lane only,
        -- 2 = stall fall lane only. Used by C06 lane-imbalance stress.
        G_BP_LANE_MODE    : natural := 0;
        -- CHAIN-P0-01 shot-boundary stall. When >0, both output lanes'
        -- tready are held low from the first output beat attempt after
        -- shot 1 until this many clocks AFTER the next shot_start pulse,
        -- so the previous line's tail is provably buffered across a
        -- mid-face shot boundary (the window the output-FIFO flush guard
        -- protects). Mutually exclusive with G_BP_TREADY_GAP.
        G_BP_SHOT_STALL_CLKS : natural := 0;
        -- 0 = no recovery. 1 = normal run -> soft_reset -> normal run.
        -- 2 = normal run -> force_reinit -> normal run.
        G_RECOVERY_MODE   : natural := 0;
        -- CHAIN P1: expect STAT7[15] (masked_slope_drop_any) to be set at
        -- end of run. Use with a chip-model slope mask that deliberately
        -- mismatches the DUT slope topology (e.g. G_CHIP_SLOPE_MASK="1111"
        -- with DEDICATED_2X2, so chips 2/3 emit rising hits into their
        -- masked rise lane). false = STAT7 read stays untouched.
        G_EXPECT_MASKED_SLOPE_DROP : boolean := false
    );
end entity tb_tdc_gpx_top_int;

architecture sim of tb_tdc_gpx_top_int is

    -- =========================================================================
    -- DEPENDENT (derived) constants -- all computed from the generics above.
    -- =========================================================================
    constant C_LIGHT_M_PER_US : real    := 299.792;
    constant C_CLK_PERIOD_PS  : natural := natural(round(1000000.0 / G_AXIS_CLK_MHZ));
    constant C_CLK_PERIOD     : time    := C_CLK_PERIOD_PS * 1 ps;
    constant C_TDC_CLK_PERIOD_PS : natural := natural(round(1000000.0 / G_TDC_CLK_MHZ));
    constant C_TDC_CLK_PERIOD : time    := C_TDC_CLK_PERIOD_PS * 1 ps;
    constant C_RST_HOLD       : time    := 24 * C_CLK_PERIOD;

    -- CSR range is always encoded in 200 MHz reference ticks (5 ns), while
    -- local shot scheduling uses the selected AXIS clock.
    constant C_DOMAIN_CLK_MHZ : positive := positive(integer(G_AXIS_CLK_MHZ));
    constant C_TDC_DOMAIN_CLK_MHZ : positive := positive(integer(G_TDC_CLK_MHZ));
    constant C_MAX_RANGE_5NS_TICKS : natural := natural(ceil(
        2.0 * G_MAX_RANGE_M / C_LIGHT_M_PER_US * real(c_RANGE_REF_CLK_MHZ)));
    constant C_MAX_RANGE_CLKS : natural :=
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(C_MAX_RANGE_5NS_TICKS, 16), C_DOMAIN_CLK_MHZ));
    constant C_SHOT_PERIOD    : natural := (C_MAX_RANGE_CLKS * 3 + 1) / 2;

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

    function fn_effective_max_hits(r_m : real; override : natural) return natural is
    begin
        if override = 0 then
            return fn_max_hits(r_m);
        else
            return override;
        end if;
    end function;

    function fn_min(a : natural; b : natural) return natural is
    begin
        if a < b then return a; end if;
        return b;
    end function;

    function fn_clamp_stops(requested : natural; build_max : positive) return natural is
    begin
        if requested < 2 then return 2; end if;
        return fn_min(requested, build_max);
    end function;

    function fn_effective_chip_mask(
        requested : std_logic_vector(3 downto 0);
        present   : std_logic_vector(3 downto 0)
    ) return std_logic_vector is
        variable v_mask : std_logic_vector(3 downto 0);
    begin
        v_mask := requested and present;
        if v_mask = "0000" then
            return fn_first_one_mask(present);
        end if;
        return v_mask;
    end function;

    constant C_REQUESTED_MAX_HITS : natural :=
        fn_effective_max_hits(G_MAX_RANGE_M, G_MAX_HITS_OVERRIDE);
    constant C_MAX_HITS : natural :=
        fn_min(C_REQUESTED_MAX_HITS, G_BUILD_MAX_HITS_PER_STOP);
    constant C_EFFECTIVE_STOPS_PER_CHIP : natural :=
        fn_clamp_stops(G_STOPS_PER_CHIP, G_BUILD_MAX_STOPS_PER_CHIP);
    constant C_EFFECTIVE_ACTIVE_MASK : std_logic_vector(3 downto 0) :=
        fn_effective_chip_mask(G_ACTIVE_CHIP_MASK, G_PRESENT_CHIP_MASK);

    -- TDC sub-module generic override values
    constant C_OUTPUT_W     : natural := G_TDATA_WIDTH;
    constant C_KEEP_W       : natural := fn_axis_keep_width(C_OUTPUT_W);

    -- Chip model fixed
    constant C_FIFO_DEPTH   : natural := 32;
    constant C_LF_THRESH    : natural := 4;

    -- Packed MAIN_CTRL / RANGE_COLS values for pipeline CSR
    function fn_pack_main_ctrl(mask  : std_logic_vector(3 downto 0);
                               stops : natural) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v( 3 downto  0) := mask;
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
        if G_FALLING_ENABLE then
            v(c_ST_FALLING_ENABLE) := '1';
        end if;
        return v;
    end function;

    function fn_pack_hw_config return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(c_HWCFG_N_CHIPS_HI downto c_HWCFG_N_CHIPS_LO) :=
            std_logic_vector(to_unsigned(fn_count_ones(G_PRESENT_CHIP_MASK), 4));
        v(c_HWCFG_MAX_STOPS_HI downto c_HWCFG_MAX_STOPS_LO) :=
            std_logic_vector(to_unsigned(G_BUILD_MAX_STOPS_PER_CHIP, 4));
        v(c_HWCFG_MAX_HITS_HI downto c_HWCFG_MAX_HITS_LO) :=
            std_logic_vector(to_unsigned(G_BUILD_MAX_HITS_PER_STOP, 4));
        v(c_HWCFG_HIT_WIDTH_HI downto c_HWCFG_HIT_WIDTH_LO) :=
            std_logic_vector(to_unsigned(c_HIT_SLOT_DATA_WIDTH, 5));
        v(c_HWCFG_TDATA_HI downto c_HWCFG_TDATA_LO) :=
            std_logic_vector(to_unsigned(G_TDATA_WIDTH, 8));
        v(c_HWCFG_CELL_FMT_HI downto c_HWCFG_CELL_FMT_LO) :=
            std_logic_vector(to_unsigned(c_CELL_FORMAT, 3));
        if fn_count_ones(G_FALL_CHIP_MASK and G_PRESENT_CHIP_MASK) > 0 then
            v(c_HWCFG_HAS_FALLING) := '1';
        end if;
        v(c_HWCFG_N_FACES_HI downto c_HWCFG_N_FACES_LO) :=
            std_logic_vector(to_unsigned(G_N_FACES, 3));
        return v;
    end function;

    function fn_face_axis_beats(cols_per_face  : natural;
                                active_chips   : natural;
                                stops_per_chip : natural;
                                max_hits       : natural;
                                tdata_width    : natural) return natural is
    begin
        if active_chips = 0 then
            return 0;
        end if;
        return cols_per_face
             * fn_vdma_line_bytes(active_chips * stops_per_chip, max_hits)
             / (tdata_width / 8);
    end function;

    function fn_slope_chip_mask(
        active_mask : std_logic_vector(3 downto 0);
        edge_mask   : std_logic_vector(3 downto 0);
        fall_enable : boolean;
        rise_lane   : boolean
    ) return std_logic_vector is
    begin
        if not fall_enable and rise_lane then
            return active_mask;
        elsif not fall_enable then
            return (active_mask'range => '0');
        else
            return active_mask and edge_mask;
        end if;
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

    function fn_model_words_per_ififo(
        stops_per_chip : natural;
        echoes_per_stop : natural
    ) return natural is
    begin
        if echoes_per_stop = 1 then
            return stops_per_chip;
        else
            return (stops_per_chip / 2) * echoes_per_stop;
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

    function fn_expected_reg0(chip_id : natural) return std_logic_vector is
        variable v_word : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) :=
            x"0381C00";
        variable v_rise_enable : boolean;
        variable v_fall_enable : boolean;
    begin
        v_rise_enable := G_PRESENT_CHIP_MASK(chip_id) = '1'
                         and (not G_FALLING_ENABLE
                              or G_RISE_CHIP_MASK(chip_id) = '1');
        v_fall_enable := G_PRESENT_CHIP_MASK(chip_id) = '1'
                         and G_FALLING_ENABLE
                         and G_FALL_CHIP_MASK(chip_id) = '1';
        if not v_rise_enable then
            v_word(c_REG0_TRISEEN_HI downto c_REG0_TRISEEN_LO) := (others => '0');
        end if;
        if not v_fall_enable then
            v_word(c_REG0_TFALLEN_HI downto c_REG0_TFALLEN_LO) := (others => '0');
        end if;
        return v_word;
    end function;

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    signal clk          : std_logic := '0';
    signal tdc_clk      : std_logic := '0';
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
    signal lc_face_index : std_logic_vector(2 downto 0) := (others => '0');
    signal lc_stop_tdc  : std_logic := '0';   -- -> i_stop_tdc

    -- =========================================================================
    -- TDC-GPX logical model slots and compact physical DUT pins
    -- =========================================================================
    constant C_PHYSICAL_CHIPS : positive := fn_physical_chip_count(G_PRESENT_CHIP_MASK);
    signal io_tdc_d         : std_logic_vector(C_PHYSICAL_CHIPS * c_TDC_BUS_WIDTH - 1 downto 0);
    signal p_o_tdc_adr      : std_logic_vector(C_PHYSICAL_CHIPS * c_TDC_ADR_WIDTH - 1 downto 0);
    signal p_o_tdc_csn      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_o_tdc_rdn      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_o_tdc_wrn      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_o_tdc_oen      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_o_tdc_stopdis  : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_o_tdc_alutrigger : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_o_tdc_puresn   : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_i_tdc_ef1      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_i_tdc_ef2      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_i_tdc_lf1      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_i_tdc_lf2      : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_i_tdc_irflag   : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);
    signal p_i_tdc_errflag  : std_logic_vector(C_PHYSICAL_CHIPS - 1 downto 0);

    signal s_tdc_d_sample   : t_tdc_bus_array;
    signal o_tdc_adr        : t_tdc_adr_array;
    signal o_tdc_csn        : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal o_tdc_rdn        : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal o_tdc_wrn        : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal o_tdc_oen        : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal o_tdc_stopdis    : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal o_tdc_alutrigger : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal o_tdc_puresn     : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal i_tdc_ef1        : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '1');
    signal i_tdc_ef2        : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '1');
    signal i_tdc_lf1        : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_lf2        : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_irflag     : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal i_tdc_errflag    : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');

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
    signal vdma_hsize_rise : unsigned(15 downto 0);
    signal vdma_hsize_fall : unsigned(15 downto 0);
    signal vdma_vsize_lines : unsigned(15 downto 0);

    -- =========================================================================
    -- Calibration constants
    -- =========================================================================
    signal i_bin_resolution_ps : unsigned(15 downto 0) := to_unsigned(81, 16);
    signal i_k_dist_fixed      : unsigned(31 downto 0) := to_unsigned(54321, 32);
    signal o_irq, o_irq_pipe   : std_logic;

    -- =========================================================================
    -- 4-chip behavioral model state
    -- =========================================================================
    type t_fill_array is array (0 to c_MAX_CHIPS - 1) of natural;
    signal fifo1_fill : t_fill_array := (others => 0);
    signal fifo2_fill : t_fill_array := (others => 0);
    signal fifo1_rd_cnt : t_fill_array := (others => 0);
    signal fifo2_rd_cnt : t_fill_array := (others => 0);

    -- FIFO load request (stim pulses prior to each shot)
    signal fifo_load_req : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal fifo_load_n1  : t_fill_array := (others => 0);
    signal fifo_load_n2  : t_fill_array := (others => 0);

    -- Bus tri-state drive per chip
    type t_chip_d_array is array (0 to c_MAX_CHIPS - 1)
        of std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal chip_d_out : t_chip_d_array := (others => (others => '0'));
    signal chip_d_oe  : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal reg0_write_seen : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal reg0_write_data : t_slv28_array := (others => (others => '0'));

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
    -- CHAIN-P0-01: set when a shot_start pulse was observed while the
    -- shot-boundary stall window held tready low (source marker that the
    -- hazard window was actually exercised).
    signal mon_shot_stall_overlap : boolean := false;
    signal mon_irq_cnt          : natural := 0;
    signal mon_irq_pipe_cnt     : natural := 0;
    signal mon_rise_metadata_count : natural := 0;
    signal mon_fall_metadata_count : natural := 0;
    signal mon_rise_hit16_meta_nonzero : natural := 0;
    signal mon_fall_hit16_meta_nonzero : natural := 0;

    -- =========================================================================
    -- Pipeline CSR register offsets (csr_pipeline internal)
    --   CTL0 = 0x00 MAIN_CTRL   [31:28]=COMMAND, [22:19]=n_drain_cap,
    --                            [18:15]=stops, [14:12]=reserved, [3:0]=mask
    --   CTL1 = 0x04 RANGE_COLS  [31:16]=cols_per_face,
    --                             [15:0]=max_range_5ns_ticks
    -- =========================================================================
    constant C_PIPE_MAIN_CTRL  : std_logic_vector(6 downto 0) := "0000000";  -- 0x00
    constant C_PIPE_RANGE_COLS : std_logic_vector(6 downto 0) := "0000100";  -- 0x04
    constant C_PIPE_AUX_CMD    : std_logic_vector(6 downto 0) := "0001000";  -- 0x08
    constant C_PIPE_NATIVE_ALIAS : std_logic_vector(6 downto 0) := "0100000"; -- 0x20, reserved
    constant C_PIPE_HW_VERSION : std_logic_vector(6 downto 0) := "1000000";  -- 0x40
    constant C_PIPE_HW_CONFIG  : std_logic_vector(6 downto 0) := "1000100";  -- 0x44
    constant C_PIPE_MAX_ROWS   : std_logic_vector(6 downto 0) := "1001000";  -- 0x48
    constant C_PIPE_CELL_SIZE  : std_logic_vector(6 downto 0) := "1001100";  -- 0x4C
    constant C_PIPE_MAX_HSIZE  : std_logic_vector(6 downto 0) := "1010000";  -- 0x50
    constant C_PIPE_STATUS     : std_logic_vector(6 downto 0) := "1010100";  -- 0x54
    constant C_PIPE_STATUS_EXT : std_logic_vector(6 downto 0) := "1011000";  -- 0x58
    constant C_PIPE_STATUS_EXT2: std_logic_vector(6 downto 0) := "1011100";  -- 0x5C
    constant C_PIPE_HIGH_RESERVED : std_logic_vector(6 downto 0) := "1100000"; -- 0x60

    -- MAIN_CTRL + RANGE_COLS packed from the entity generics so changing
    -- G_ACTIVE_CHIP_MASK / G_STOPS_PER_CHIP / G_COLS_PER_FACE /
    -- G_MAX_RANGE_M at instantiation time consistently updates both CSRs.
    constant C_MAIN_CTRL_BASE : std_logic_vector(31 downto 0) :=
        fn_pack_main_ctrl(G_ACTIVE_CHIP_MASK, G_STOPS_PER_CHIP);
    constant C_RANGE_COLS_VAL : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(G_COLS_PER_FACE, 16)) &
        std_logic_vector(to_unsigned(C_MAX_RANGE_5NS_TICKS, 16));
    constant C_ACTIVE_CHIPS : natural := fn_count_ones(C_EFFECTIVE_ACTIVE_MASK);
    constant C_RISE_LANE_CHIP_MASK : std_logic_vector(3 downto 0) :=
        fn_slope_chip_mask(C_EFFECTIVE_ACTIVE_MASK, G_RISE_CHIP_MASK,
                           G_FALLING_ENABLE, true);
    constant C_FALL_LANE_CHIP_MASK : std_logic_vector(3 downto 0) :=
        fn_slope_chip_mask(C_EFFECTIVE_ACTIVE_MASK, G_FALL_CHIP_MASK,
                           G_FALLING_ENABLE, false);
    constant C_RISE_ACTIVE_CHIPS : natural :=
        fn_count_ones(C_RISE_LANE_CHIP_MASK);
    constant C_FALL_ACTIVE_CHIPS : natural :=
        fn_count_ones(C_FALL_LANE_CHIP_MASK);
    constant C_TOTAL_LINES  : natural := G_N_FACES * G_COLS_PER_FACE;
    constant C_TARGET_FACE_BEATS_RISE : natural :=
        fn_face_axis_beats(G_COLS_PER_FACE, C_RISE_ACTIVE_CHIPS, C_EFFECTIVE_STOPS_PER_CHIP,
                           C_MAX_HITS, C_OUTPUT_W);
    constant C_TARGET_FACE_BEATS_FALL : natural :=
        fn_face_axis_beats(G_COLS_PER_FACE, C_FALL_ACTIVE_CHIPS, C_EFFECTIVE_STOPS_PER_CHIP,
                           C_MAX_HITS, C_OUTPUT_W);
    constant C_DEFAULT_FACE_BEATS_RISE : natural :=
        fn_face_axis_beats(G_COLS_PER_FACE, C_RISE_ACTIVE_CHIPS, C_EFFECTIVE_STOPS_PER_CHIP,
                           G_BUILD_MAX_HITS_PER_STOP, C_OUTPUT_W);
    constant C_DEFAULT_FACE_BEATS_FALL : natural :=
        fn_face_axis_beats(G_COLS_PER_FACE, C_FALL_ACTIVE_CHIPS, C_EFFECTIVE_STOPS_PER_CHIP,
                           G_BUILD_MAX_HITS_PER_STOP, C_OUTPUT_W);
    constant C_EXPECTED_AXIS_BEATS_RISE : natural :=
        fn_expected_axis_beats(G_MAX_HITS_WRITE_MODE, G_N_FACES,
                               C_TARGET_FACE_BEATS_RISE, C_DEFAULT_FACE_BEATS_RISE);
    constant C_EXPECTED_AXIS_BEATS_FALL : natural :=
        fn_expected_axis_beats(G_MAX_HITS_WRITE_MODE, G_N_FACES,
                               C_TARGET_FACE_BEATS_FALL, C_DEFAULT_FACE_BEATS_FALL);
    constant C_RECOVERY_RUNS : natural := fn_recovery_runs(G_RECOVERY_MODE);
    constant C_EXPECTED_TOTAL_LINES : natural := C_TOTAL_LINES * C_RECOVERY_RUNS;
    constant C_EXPECTED_TOTAL_AXIS_BEATS_RISE : natural :=
        C_EXPECTED_AXIS_BEATS_RISE * C_RECOVERY_RUNS;
    constant C_EXPECTED_TOTAL_AXIS_BEATS_FALL : natural :=
        C_EXPECTED_AXIS_BEATS_FALL * C_RECOVERY_RUNS;
    constant C_HDR_PREFIX_BEATS : natural := fn_hdr_prefix_beats(C_OUTPUT_W);
    constant C_WORDS_PER_BEAT : natural := C_OUTPUT_W / 32;
    constant C_CANONICAL_WORDS_PER_CELL : natural :=
        fn_canonical_cell_words(C_MAX_HITS);
    constant C_RISE_DATA_WORDS : natural :=
        C_RISE_ACTIVE_CHIPS * C_EFFECTIVE_STOPS_PER_CHIP * C_CANONICAL_WORDS_PER_CELL;
    constant C_FALL_DATA_WORDS : natural :=
        C_FALL_ACTIVE_CHIPS * C_EFFECTIVE_STOPS_PER_CHIP * C_CANONICAL_WORDS_PER_CELL;
    constant C_MODEL_WORDS_PER_IFIFO : natural :=
        fn_model_words_per_ififo(C_EFFECTIVE_STOPS_PER_CHIP, G_ECHOES_PER_STOP);
    constant C_BUILD_MAX_ROWS : natural :=
        fn_count_ones(G_PRESENT_CHIP_MASK) * G_BUILD_MAX_STOPS_PER_CHIP;
    constant C_BUILD_CELL_BYTES : natural :=
        fn_canonical_cell_bytes(G_BUILD_MAX_HITS_PER_STOP);
    constant C_BUILD_VDMA_BYTES : natural :=
        fn_vdma_line_bytes(C_BUILD_MAX_ROWS, G_BUILD_MAX_HITS_PER_STOP);
    constant C_EXPECTED_HW_CONFIG : std_logic_vector(31 downto 0) :=
        fn_pack_hw_config;

    -- Chip CSR addresses (9-bit)
    constant C_CHIP_CFG_REG0   : std_logic_vector(8 downto 0) := "0" & x"14";  -- CTL5
    constant C_CHIP_CFG_REG5   : std_logic_vector(8 downto 0) := "0" & x"28";  -- CTL10
    constant C_CHIP_CFG_REG6   : std_logic_vector(8 downto 0) := "0" & x"2C";
    constant C_CHIP_SCAN_CFG   : std_logic_vector(8 downto 0) := "0" & x"54";  -- CTL21

begin

    p_physical_pin_contract : process
    begin
        assert io_tdc_d'length = C_PHYSICAL_CHIPS * c_TDC_BUS_WIDTH
            report "physical TDC data pin width does not match present-chip count"
            severity failure;
        assert p_o_tdc_adr'length = C_PHYSICAL_CHIPS * c_TDC_ADR_WIDTH
            report "physical TDC address pin width does not match present-chip count"
            severity failure;
        assert p_o_tdc_csn'length = C_PHYSICAL_CHIPS and
               p_o_tdc_rdn'length = C_PHYSICAL_CHIPS and
               p_o_tdc_wrn'length = C_PHYSICAL_CHIPS and
               p_o_tdc_oen'length = C_PHYSICAL_CHIPS and
               p_o_tdc_stopdis'length = C_PHYSICAL_CHIPS and
               p_o_tdc_alutrigger'length = C_PHYSICAL_CHIPS and
               p_o_tdc_puresn'length = C_PHYSICAL_CHIPS and
               p_i_tdc_ef1'length = C_PHYSICAL_CHIPS and
               p_i_tdc_ef2'length = C_PHYSICAL_CHIPS and
               p_i_tdc_lf1'length = C_PHYSICAL_CHIPS and
               p_i_tdc_lf2'length = C_PHYSICAL_CHIPS and
               p_i_tdc_irflag'length = C_PHYSICAL_CHIPS and
               p_i_tdc_errflag'length = C_PHYSICAL_CHIPS
            report "physical TDC scalar-vector width does not match present-chip count"
            severity failure;
        report "PHYSICAL_PIN_CONTRACT PASS: chips=" &
               integer'image(C_PHYSICAL_CHIPS) &
               " d=" & integer'image(io_tdc_d'length) &
               " adr=" & integer'image(p_o_tdc_adr'length)
            severity note;
        wait;
    end process p_physical_pin_contract;

    gen_tb_physical_map : for logical_chip in 0 to c_MAX_CHIPS - 1 generate
        gen_present_pin : if G_PRESENT_CHIP_MASK(logical_chip) = '1' generate
            constant C_PHYSICAL_CHIP : natural :=
                fn_physical_chip_index(G_PRESENT_CHIP_MASK, logical_chip);
        begin
            o_tdc_adr(logical_chip) <=
                p_o_tdc_adr((C_PHYSICAL_CHIP + 1) * c_TDC_ADR_WIDTH - 1 downto
                            C_PHYSICAL_CHIP * c_TDC_ADR_WIDTH);
            o_tdc_csn(logical_chip)        <= p_o_tdc_csn(C_PHYSICAL_CHIP);
            o_tdc_rdn(logical_chip)        <= p_o_tdc_rdn(C_PHYSICAL_CHIP);
            o_tdc_wrn(logical_chip)        <= p_o_tdc_wrn(C_PHYSICAL_CHIP);
            o_tdc_oen(logical_chip)        <= p_o_tdc_oen(C_PHYSICAL_CHIP);
            o_tdc_stopdis(logical_chip)    <= p_o_tdc_stopdis(C_PHYSICAL_CHIP);
            o_tdc_alutrigger(logical_chip) <= p_o_tdc_alutrigger(C_PHYSICAL_CHIP);
            o_tdc_puresn(logical_chip)     <= p_o_tdc_puresn(C_PHYSICAL_CHIP);

            p_i_tdc_ef1(C_PHYSICAL_CHIP)     <= i_tdc_ef1(logical_chip);
            p_i_tdc_ef2(C_PHYSICAL_CHIP)     <= i_tdc_ef2(logical_chip);
            p_i_tdc_lf1(C_PHYSICAL_CHIP)     <= i_tdc_lf1(logical_chip);
            p_i_tdc_lf2(C_PHYSICAL_CHIP)     <= i_tdc_lf2(logical_chip);
            p_i_tdc_irflag(C_PHYSICAL_CHIP)  <= i_tdc_irflag(logical_chip);
            p_i_tdc_errflag(C_PHYSICAL_CHIP) <= i_tdc_errflag(logical_chip);

            s_tdc_d_sample(logical_chip) <=
                io_tdc_d((C_PHYSICAL_CHIP + 1) * c_TDC_BUS_WIDTH - 1 downto
                         C_PHYSICAL_CHIP * c_TDC_BUS_WIDTH);
            io_tdc_d((C_PHYSICAL_CHIP + 1) * c_TDC_BUS_WIDTH - 1 downto
                     C_PHYSICAL_CHIP * c_TDC_BUS_WIDTH) <=
                chip_d_out(logical_chip) when chip_d_oe(logical_chip) = '1'
                else (others => 'Z');
        end generate gen_present_pin;

        gen_absent_pin : if G_PRESENT_CHIP_MASK(logical_chip) = '0' generate
        begin
            s_tdc_d_sample(logical_chip)   <= (others => '0');
            o_tdc_adr(logical_chip)        <= (others => '0');
            o_tdc_csn(logical_chip)        <= '1';
            o_tdc_rdn(logical_chip)        <= '1';
            o_tdc_wrn(logical_chip)        <= '1';
            o_tdc_oen(logical_chip)        <= '1';
            o_tdc_stopdis(logical_chip)    <= '1';
            o_tdc_alutrigger(logical_chip) <= '0';
            o_tdc_puresn(logical_chip)     <= '0';
        end generate gen_absent_pin;
    end generate gen_tb_physical_map;

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    clk <= not clk after C_CLK_PERIOD / 2 when not sim_done else '0';
    tdc_clk <= not tdc_clk after C_TDC_CLK_PERIOD / 2 when not sim_done else '0';

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
        -- CHAIN-P0-01 shot-boundary stall state
        variable v_shot_cnt : natural := 0;     -- lc_start_tdc pulses seen
        variable v_stall_on : boolean := false; -- window active
        variable v_stall_done : boolean := false; -- single window per sim
        variable v_rel_cnt  : natural := 0;     -- post-shot-2 release count
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_cnt := 0;
                v_shot_cnt   := 0;
                v_stall_on   := false;
                v_stall_done := false;
                v_rel_cnt    := 0;
                m_rise_tready <= '1';
                m_fall_tready <= '1';
            elsif G_BP_SHOT_STALL_CLKS > 0 then
                if lc_start_tdc = '1' then
                    v_shot_cnt := v_shot_cnt + 1;
                    if v_stall_on then
                        mon_shot_stall_overlap <= true;
                    end if;
                end if;

                -- Arm on the first output beat attempt after shot 1: the
                -- line is now in flight and will be caught in the FIFOs.
                if not v_stall_on and not v_stall_done
                   and v_shot_cnt = 1
                   and (m_rise_tvalid = '1' or m_fall_tvalid = '1') then
                    v_stall_on := true;
                end if;

                -- Release G_BP_SHOT_STALL_CLKS after the second shot pulse,
                -- i.e. well past shot_start_gated and the (guarded) FIFO
                -- reset pulse inside the DUT.
                if v_stall_on and v_shot_cnt >= 2 then
                    v_rel_cnt := v_rel_cnt + 1;
                    if v_rel_cnt >= G_BP_SHOT_STALL_CLKS then
                        v_stall_on   := false;
                        v_stall_done := true;
                    end if;
                end if;

                if v_stall_on then
                    m_rise_tready <= '0';
                    m_fall_tready <= '0';
                    mon_bp_stall_cycles <= mon_bp_stall_cycles + 1;
                else
                    m_rise_tready <= '1';
                    m_fall_tready <= '1';
                end if;
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
    gen_flags : for i in 0 to c_MAX_CHIPS - 1 generate
        i_tdc_ef1(i) <= '1' when fifo1_fill(i) = 0 else '0';
        i_tdc_ef2(i) <= '1' when fifo2_fill(i) = 0 else '0';
        i_tdc_lf1(i) <= '1' when fifo1_fill(i) >= C_LF_THRESH else '0';
        i_tdc_lf2(i) <= '1' when fifo2_fill(i) >= C_LF_THRESH else '0';
    end generate;

    -- Chip model body (FIFO state + writes per chip)
    gen_chip : for i in 0 to c_MAX_CHIPS - 1 generate

        p_chip : process(tdc_clk)
            variable v_rdn_prev  : std_logic := '1';
            variable v_load_prev : std_logic := '0';
            variable v_my_fill1  : natural   := 0;
            variable v_my_fill2  : natural   := 0;
            variable v_my_rd1    : natural   := 0;
            variable v_my_rd2    : natural   := 0;
            variable v_hit_value : unsigned(c_RAW_HIT_WIDTH - 1 downto 0);
            variable v_cha_code  : std_logic_vector(1 downto 0);
        begin
            if rising_edge(tdc_clk) then
                if rst_n = '0' then
                    v_my_fill1 := 0;
                    v_my_fill2 := 0;
                    v_my_rd1   := 0;
                    v_my_rd2   := 0;
                    chip_d_oe(i)  <= '0';
                    chip_d_out(i) <= (others => '0');
                    reg0_write_seen(i) <= '0';
                    reg0_write_data(i) <= (others => '0');
                else
                    -- Default: D-bus hi-Z
                    chip_d_oe(i) <= '0';

                    if o_tdc_csn(i) = '0' and o_tdc_wrn(i) = '0'
                       and o_tdc_adr(i) = c_TDC_REG0 then
                        reg0_write_seen(i) <= '1';
                        reg0_write_data(i) <= s_tdc_d_sample(i);
                    end if;

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
                    --   [27:26] ChaCode  = read_count mod 4
                    --   [25:18] StartNum = 0 (SINGLE_SHOT)
                    --   [17]    Slope    = G_CHIP_SLOPE_MASK(i)
                    --                     (1=rising, 0=falling)
                    --   [16: 0] Hit      = sequential test pattern
                    -- Chip addresses: IFIFO1 = 8, IFIFO2 = 9.
                    if o_tdc_oen(i) = '0' and o_tdc_rdn(i) = '0'
                       and o_tdc_csn(i) = '0' then
                        chip_d_oe(i) <= '1';
                        if o_tdc_adr(i) = c_TDC_REG8_IFIFO1 then
                            v_cha_code := std_logic_vector(to_unsigned(v_my_rd1 mod 4, 2));
                            v_hit_value := to_unsigned(
                                (i * 256) + v_my_rd1 + 1,
                                c_RAW_HIT_WIDTH);
                            if G_FORCE_HIT16 then
                                v_hit_value(c_RAW_HIT_WIDTH - 1) := '1';
                            end if;
                            chip_d_out(i) <= v_cha_code &                    -- ChaCode
                                             x"00" &                         -- StartNum
                                             G_CHIP_SLOPE_MASK(i) &
                                             std_logic_vector(v_hit_value);
                        elsif o_tdc_adr(i) = c_TDC_REG9_IFIFO2 then
                            v_cha_code := std_logic_vector(to_unsigned(v_my_rd2 mod 4, 2));
                            v_hit_value := to_unsigned(
                                (i * 256) + 128 + v_my_rd2 + 1,
                                c_RAW_HIT_WIDTH);
                            if G_FORCE_HIT16 then
                                v_hit_value(c_RAW_HIT_WIDTH - 1) := '1';
                            end if;
                            chip_d_out(i) <= v_cha_code & x"00" &
                                             G_CHIP_SLOPE_MASK(i) &
                                             std_logic_vector(v_hit_value);
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

    end generate gen_chip;

    -- =========================================================================
    -- DUT: tdc_gpx_top
    -- =========================================================================
    u_dut : entity work.tdc_gpx_top
        generic map (
            g_HW_VERSION     => x"00010000",
            g_OUTPUT_WIDTH   => C_OUTPUT_W,
            g_RISE_CHIP_MASK => G_RISE_CHIP_MASK,
            g_FALL_CHIP_MASK => G_FALL_CHIP_MASK,
            g_NUM_CHIPS => C_PHYSICAL_CHIPS,
            g_PRESENT_CHIP_MASK => G_PRESENT_CHIP_MASK,
            g_MAX_STOPS_PER_CHIP => G_BUILD_MAX_STOPS_PER_CHIP,
            g_MAX_HITS_PER_STOP => G_BUILD_MAX_HITS_PER_STOP,
            g_AXIS_CLK_MHZ   => C_DOMAIN_CLK_MHZ,
            g_TDC_CLK_MHZ    => C_TDC_DOMAIN_CLK_MHZ,
            -- Keep the accelerated simulation knobs cycle-based, then express
            -- the equivalent physical time at the production top boundary.
            g_POWERUP_TIME_NS   => (G_POWERUP_CLKS * 1000) / C_TDC_DOMAIN_CLK_MHZ,
            g_RECOVERY_TIME_NS  => (G_RECOVERY_CLKS * 1000) / C_TDC_DOMAIN_CLK_MHZ,
            g_ALU_PULSE_TIME_NS => (G_ALU_PULSE_CLKS * 1000) / C_TDC_DOMAIN_CLK_MHZ,
            g_STREAM_CLK_MODE => G_STREAM_CLK_MODE
        )
        port map (
            -- AXIS/AXI-Lite and TDC clocks may run at different rates.
            i_axis_aclk     => clk,
            i_axis_aresetn  => rst_n,
            i_tdc_clk       => tdc_clk,
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
            -- laser_ctrl pulses
            i_n_faces        => std_logic_vector(to_unsigned(G_N_FACES, 3)),
            i_shot_start      => lc_start_tdc,
            i_shot_face_index => lc_face_index,
            i_stop_tdc        => lc_stop_tdc,
            -- TDC physical pins
            io_tdc_d         => io_tdc_d,
            o_tdc_adr        => p_o_tdc_adr,
            o_tdc_csn        => p_o_tdc_csn,
            o_tdc_rdn        => p_o_tdc_rdn,
            o_tdc_wrn        => p_o_tdc_wrn,
            o_tdc_oen        => p_o_tdc_oen,
            o_tdc_stopdis    => p_o_tdc_stopdis,
            o_tdc_alutrigger => p_o_tdc_alutrigger,
            o_tdc_puresn     => p_o_tdc_puresn,
            i_tdc_ef1        => p_i_tdc_ef1,
            i_tdc_ef2        => p_i_tdc_ef2,
            i_tdc_lf1        => p_i_tdc_lf1,
            i_tdc_lf2        => p_i_tdc_lf2,
            i_tdc_irflag     => p_i_tdc_irflag,
            i_tdc_errflag    => p_i_tdc_errflag,
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
            o_vdma_hsize_bytes_rise => vdma_hsize_rise,
            o_vdma_hsize_bytes_fall => vdma_hsize_fall,
            o_vdma_vsize_lines      => vdma_vsize_lines,
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
        variable v_rise_line_beat : natural := 0;
        variable v_fall_line_beat : natural := 0;
        variable v_data_word_idx  : natural := 0;
        variable v_header_word_idx : natural := 0;
        variable v_meta_inc       : natural range 0 to 4 := 0;
        variable v_nonzero_inc    : natural range 0 to 4 := 0;
        variable v_word           : std_logic_vector(31 downto 0);
        variable v_header_hits    : natural range 0 to 255 := 0;
        variable v_cell_idx       : natural := 0;
        variable v_word_in_cell   : natural := 0;
        variable v_expected_chip  : natural range 0 to c_MAX_CHIPS - 1 := 0;
        variable v_expected_stop  : natural range 0 to c_MAX_STOPS_PER_CHIP - 1 := 0;
        variable v_expected_hit   : natural := 0;
        variable v_rise_first_line : boolean := false;
        variable v_fall_first_line : boolean := false;
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
                mon_rise_metadata_count <= 0;
                mon_fall_metadata_count <= 0;
                mon_rise_hit16_meta_nonzero <= 0;
                mon_fall_hit16_meta_nonzero <= 0;
                v_rise_line_beat := 0;
                v_fall_line_beat := 0;
                v_rise_first_line := false;
                v_fall_first_line := false;
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
                    if m_rise_tuser(0) = '1' then
                        v_rise_line_beat := 0;
                        v_rise_first_line := true;
                    end if;
                    if v_rise_first_line
                       and v_rise_line_beat < C_HDR_PREFIX_BEATS then
                        for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                            v_header_word_idx := v_rise_line_beat * C_WORDS_PER_BEAT + lane;
                            if v_header_word_idx = 3 then
                                v_word := m_rise_tdata(32 * lane + 31 downto 32 * lane);
                                assert v_word(7 downto 0) = std_logic_vector(
                                    resize(unsigned(lc_face_index), 8))
                                    report "top_int: rising header motor face_index mismatch"
                                    severity failure;
                                assert v_word(11 downto 8) = C_RISE_LANE_CHIP_MASK
                                    report "top_int: rising header active-chip mask mismatch"
                                    severity failure;
                                assert v_word(14 downto 12) = std_logic_vector(
                                    to_unsigned(G_N_FACES, 3))
                                    report "top_int: rising header motor n_faces mismatch"
                                    severity failure;
                                assert (v_word(31) = '1') = G_FALLING_ENABLE
                                    report "top_int: rising header falling-enable mismatch"
                                    severity failure;
                            elsif v_header_word_idx = 5 then
                                v_word := m_rise_tdata(32 * lane + 31 downto 32 * lane);
                                v_header_hits := to_integer(unsigned(v_word(7 downto 0)));
                                assert v_header_hits >= 1
                                       and v_header_hits <= G_BUILD_MAX_HITS_PER_STOP
                                    report "top_int: rising header max-hits outside build profile"
                                    severity failure;
                                assert v_word(15 downto 8) = std_logic_vector(to_unsigned(
                                    fn_canonical_cell_bytes(v_header_hits), 8))
                                    report "top_int: rising header canonical cell-bytes mismatch"
                                    severity failure;
                                if G_MAX_HITS_WRITE_MODE = 1 then
                                    assert v_header_hits = C_MAX_HITS
                                        report "top_int: rising header early max-hits mismatch"
                                        severity failure;
                                elsif G_MAX_HITS_WRITE_MODE = 0
                                      or G_MAX_HITS_WRITE_MODE = 2 then
                                    assert v_header_hits = G_BUILD_MAX_HITS_PER_STOP
                                        report "top_int: rising header zero-alias max-hits mismatch"
                                        severity failure;
                                end if;
                                assert v_word(27 downto 24) = std_logic_vector(to_unsigned(
                                    fn_count_ones(G_PRESENT_CHIP_MASK), 4))
                                    report "top_int: rising header implemented-chip count mismatch"
                                    severity failure;
                                assert v_word(31 downto 28) = std_logic_vector(to_unsigned(
                                    G_BUILD_MAX_STOPS_PER_CHIP, 4))
                                    report "top_int: rising header build max-stops mismatch"
                                    severity failure;
                            end if;
                        end loop;
                    end if;
                    if v_rise_line_beat >= C_HDR_PREFIX_BEATS then
                        v_meta_inc    := 0;
                        v_nonzero_inc := 0;
                        for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                            v_data_word_idx :=
                                (v_rise_line_beat - C_HDR_PREFIX_BEATS)
                                * C_WORDS_PER_BEAT + lane;
                             if v_data_word_idx < C_RISE_DATA_WORDS
                               and (v_data_word_idx mod C_CANONICAL_WORDS_PER_CELL)
                                   = C_CANONICAL_WORDS_PER_CELL - 1 then
                                v_meta_inc := v_meta_inc + 1;
                                v_word := m_rise_tdata(32 * lane + 31 downto 32 * lane);
                                 if v_word(6 downto 0) /= "0000000" then
                                     v_nonzero_inc := v_nonzero_inc + 1;
                                 end if;
                             end if;
                            if G_CHECK_32CH_ORDER
                               and v_data_word_idx < C_RISE_DATA_WORDS then
                                assert C_OUTPUT_W = 32
                                       and C_RISE_ACTIVE_CHIPS = 4
                                       and C_EFFECTIVE_STOPS_PER_CHIP = 8
                                       and C_MAX_HITS = 1
                                    report "top_int: 32-channel order check requires 32b/4chip/8stop/max_hits=1"
                                    severity failure;
                                v_cell_idx := v_data_word_idx / C_CANONICAL_WORDS_PER_CELL;
                                v_word_in_cell := v_data_word_idx mod C_CANONICAL_WORDS_PER_CELL;
                                v_expected_chip := v_cell_idx / C_EFFECTIVE_STOPS_PER_CHIP;
                                v_expected_stop := v_cell_idx mod C_EFFECTIVE_STOPS_PER_CHIP;
                                v_word := m_rise_tdata(32 * lane + 31 downto 32 * lane);
                                if v_expected_stop < 4 then
                                    v_expected_hit := v_expected_chip * 256
                                                      + v_expected_stop + 1;
                                else
                                    v_expected_hit := v_expected_chip * 256 + 128
                                                      + (v_expected_stop - 4) + 1;
                                end if;
                                if v_word_in_cell = 0 then
                                    assert unsigned(v_word(15 downto 0)) =
                                           to_unsigned(v_expected_hit, 16)
                                        report "top_int: 32-channel hit order mismatch at cell "
                                               & integer'image(v_cell_idx)
                                        severity failure;
                                    assert v_word(31 downto 16) = x"0000"
                                        report "top_int: unexpected upper hit slot in max_hits=1 cell"
                                        severity failure;
                                else
                                    assert unsigned(v_word(9 downto 8)) =
                                           to_unsigned(v_expected_chip, 2)
                                        report "top_int: 32-channel metadata chip order mismatch at cell "
                                               & integer'image(v_cell_idx)
                                        severity failure;
                                    assert unsigned(v_word(15 downto 12)) = 1
                                           and v_word(25) = '1'
                                           and v_word(18) = '1'
                                        report "top_int: 32-channel metadata content mismatch at cell "
                                               & integer'image(v_cell_idx)
                                        severity failure;
                                end if;
                            end if;
                        end loop;
                        if v_meta_inc > 0 then
                            mon_rise_metadata_count <= mon_rise_metadata_count + v_meta_inc;
                            mon_rise_hit16_meta_nonzero <=
                                mon_rise_hit16_meta_nonzero + v_nonzero_inc;
                        end if;
                    end if;
                    mon_rise_beats <= mon_rise_beats + 1;
                    if m_rise_tlast = '1' then
                        mon_rise_frame_end <= mon_rise_frame_end + 1;
                        mon_rise_last_cycle <= sim_cycle;
                        report "C06_MARKER T5_RISE_TLAST cycle="
                               & integer'image(sim_cycle)
                               & " time=" & time'image(now)
                            severity note;
                        v_rise_line_beat := 0;
                        v_rise_first_line := false;
                    else
                        v_rise_line_beat := v_rise_line_beat + 1;
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
                    if m_fall_tuser(0) = '1' then
                        v_fall_line_beat := 0;
                        v_fall_first_line := true;
                    end if;
                    if v_fall_first_line
                       and v_fall_line_beat < C_HDR_PREFIX_BEATS then
                        for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                            v_header_word_idx := v_fall_line_beat * C_WORDS_PER_BEAT + lane;
                            if v_header_word_idx = 3 then
                                v_word := m_fall_tdata(32 * lane + 31 downto 32 * lane);
                                assert v_word(7 downto 0) = std_logic_vector(
                                    resize(unsigned(lc_face_index), 8))
                                    report "top_int: falling header motor face_index mismatch"
                                    severity failure;
                                assert v_word(11 downto 8) = C_FALL_LANE_CHIP_MASK
                                    report "top_int: falling header active-chip mask mismatch"
                                    severity failure;
                                assert v_word(14 downto 12) = std_logic_vector(
                                    to_unsigned(G_N_FACES, 3))
                                    report "top_int: falling header motor n_faces mismatch"
                                    severity failure;
                                assert v_word(31) = '1'
                                    report "top_int: falling header falling-enable mismatch"
                                    severity failure;
                            elsif v_header_word_idx = 5 then
                                v_word := m_fall_tdata(32 * lane + 31 downto 32 * lane);
                                v_header_hits := to_integer(unsigned(v_word(7 downto 0)));
                                assert v_header_hits >= 1
                                       and v_header_hits <= G_BUILD_MAX_HITS_PER_STOP
                                    report "top_int: falling header max-hits outside build profile"
                                    severity failure;
                                assert v_word(15 downto 8) = std_logic_vector(to_unsigned(
                                    fn_canonical_cell_bytes(v_header_hits), 8))
                                    report "top_int: falling header canonical cell-bytes mismatch"
                                    severity failure;
                                if G_MAX_HITS_WRITE_MODE = 1 then
                                    assert v_header_hits = C_MAX_HITS
                                        report "top_int: falling header early max-hits mismatch"
                                        severity failure;
                                elsif G_MAX_HITS_WRITE_MODE = 0
                                      or G_MAX_HITS_WRITE_MODE = 2 then
                                    assert v_header_hits = G_BUILD_MAX_HITS_PER_STOP
                                        report "top_int: falling header zero-alias max-hits mismatch"
                                        severity failure;
                                end if;
                                assert v_word(27 downto 24) = std_logic_vector(to_unsigned(
                                    fn_count_ones(G_PRESENT_CHIP_MASK), 4))
                                    report "top_int: falling header implemented-chip count mismatch"
                                    severity failure;
                                assert v_word(31 downto 28) = std_logic_vector(to_unsigned(
                                    G_BUILD_MAX_STOPS_PER_CHIP, 4))
                                    report "top_int: falling header build max-stops mismatch"
                                    severity failure;
                            end if;
                        end loop;
                    end if;
                    if v_fall_line_beat >= C_HDR_PREFIX_BEATS then
                        v_meta_inc    := 0;
                        v_nonzero_inc := 0;
                        for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                            v_data_word_idx :=
                                (v_fall_line_beat - C_HDR_PREFIX_BEATS)
                                * C_WORDS_PER_BEAT + lane;
                            if v_data_word_idx < C_FALL_DATA_WORDS
                               and (v_data_word_idx mod C_CANONICAL_WORDS_PER_CELL)
                                   = C_CANONICAL_WORDS_PER_CELL - 1 then
                                v_meta_inc := v_meta_inc + 1;
                                v_word := m_fall_tdata(32 * lane + 31 downto 32 * lane);
                                if v_word(6 downto 0) /= "0000000" then
                                    v_nonzero_inc := v_nonzero_inc + 1;
                                end if;
                            end if;
                        end loop;
                        if v_meta_inc > 0 then
                            mon_fall_metadata_count <= mon_fall_metadata_count + v_meta_inc;
                            mon_fall_hit16_meta_nonzero <=
                                mon_fall_hit16_meta_nonzero + v_nonzero_inc;
                        end if;
                    end if;
                    mon_fall_beats <= mon_fall_beats + 1;
                    if m_fall_tlast = '1' then
                        mon_fall_frame_end <= mon_fall_frame_end + 1;
                        mon_fall_last_cycle <= sim_cycle;
                        report "C06_MARKER T5_FALL_TLAST cycle="
                               & integer'image(sim_cycle)
                               & " time=" & time'image(now)
                            severity note;
                        v_fall_line_beat := 0;
                        v_fall_first_line := false;
                    else
                        v_fall_line_beat := v_fall_line_beat + 1;
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
        -- STAT6 is logged without exact compare because
        -- run_drain_complete_mask is a valid non-zero result.
        --
        -- LATENT-BUG FIX (2026-07-17): STAT5/7 expectations are now
        -- per-call parameters. The old fixed all-zero expectations only
        -- ever "passed" because the pipeline CSR STAT reads decoded
        -- outside the generated IP's register window (and the
        -- t_tdc_status record had a 'U' source), so every read returned
        -- zero vacuously. With live reads: after a run that was stopped
        -- with stop_tdc only (no CMD_STOP), face_seq legitimately sits in
        -- ST_WAIT_SHOT and STAT5[0] busy = '1'.
        ----------------------------------------------------------------
        procedure read_pipeline_status(
            tag          : string;
            stat5_expect : std_logic_vector(31 downto 0) := x"00000000";
            stat7_expect : std_logic_vector(31 downto 0) := x"00000000") is
        begin
            wait_clk(64);
            pl("C06_MARKER T7_STATUS_READ tag=" & tag
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            pipe_rd(C_PIPE_STATUS, stat5_expect, '1', '1');
            pipe_rd(C_PIPE_STATUS_EXT, x"00000000", '0', '0');
            pipe_rd(C_PIPE_STATUS_EXT2, stat7_expect, '1', '1');
        end procedure;

        ----------------------------------------------------------------
        -- Load all chip FIFOs simultaneously (prior to shot)
        ----------------------------------------------------------------
        procedure load_all_fifos(n1 : natural; n2 : natural) is
        begin
            for i in 0 to c_MAX_CHIPS - 1 loop
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
        -- One I-Mode single-measurement shot
        --   1) i_shot_start (start_tdc) pulse
        --   2) Preload each physical IFIFO
        --   3) Assert IrFlag (emulates MTimer expiry)
        --   4) Wait for EF-authoritative drain
        --   5) Deassert IrFlag and assert all physical words were read
        ----------------------------------------------------------------
        procedure do_shot(shot_count : natural; base_fifo_words : natural; tag : string) is
            constant c_PHYSICAL_HITS : natural := base_fifo_words + 1;
        begin
            pl("  ---- shot [" & tag & "] start, physical_words="
               & integer'image(c_PHYSICAL_HITS));

            -- start_tdc (= i_shot_start), emulating laser_ctrl.o_start_tdc
            pl("C06_MARKER T0_START_TDC shot="
               & integer'image(shot_count)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            lc_start_tdc <= '1';
            wait_clk(1);
            lc_start_tdc <= '0';

            -- The extra word keeps this regression sensitive to an early
            -- fixed-count termination accidentally being reintroduced.
            wait_clk(4);
            load_all_fifos(c_PHYSICAL_HITS, c_PHYSICAL_HITS);

            -- Hold capture long enough for the behavioral GPX inputs to settle.
            wait_clk(80);

            -- Assert IrFlag after a few clk (MTimer expiry emulation)
            pl("C06_MARKER T2_IRFLAG_ASSERT shot="
               & integer'image(shot_count)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            i_tdc_irflag <= (others => '1');

            -- Allow 1000 AXIS clocks for the behavioral drain to complete.
            wait_clk(1000);

            -- Deassert IrFlag + wait ALU recovery
            pl("C06_MARKER T3_DRAIN_WAIT_END shot="
               & integer'image(shot_count)
               & " cycle=" & integer'image(sim_cycle)
               & " time=" & time'image(now));
            i_tdc_irflag <= (others => '0');
            wait_clk(G_ALU_PULSE_CLKS + G_RECOVERY_CLKS + 8);

            for i in 0 to c_MAX_CHIPS - 1 loop
                if C_EFFECTIVE_ACTIVE_MASK(i) = '1' then
                    assert fifo1_rd_cnt(i) = c_PHYSICAL_HITS
                        report "tb_tdc_gpx_top_int: IFIFO1 physical drain mismatch on chip "
                               & integer'image(i) & ", actual="
                               & integer'image(fifo1_rd_cnt(i)) & ", physical="
                               & integer'image(c_PHYSICAL_HITS)
                        severity failure;
                    assert fifo2_rd_cnt(i) = c_PHYSICAL_HITS
                        report "tb_tdc_gpx_top_int: IFIFO2 physical drain mismatch on chip "
                               & integer'image(i) & ", actual="
                               & integer'image(fifo2_rd_cnt(i)) & ", physical="
                               & integer'image(c_PHYSICAL_HITS)
                        severity failure;
                    assert fifo1_fill(i) = 0
                        report "tb_tdc_gpx_top_int: IFIFO1 not empty after drain on chip "
                               & integer'image(i)
                        severity failure;
                    assert fifo2_fill(i) = 0
                        report "tb_tdc_gpx_top_int: IFIFO2 not empty after drain on chip "
                               & integer'image(i)
                        severity failure;
                else
                    assert fifo1_rd_cnt(i) = 0 and fifo2_rd_cnt(i) = 0
                        report "tb_tdc_gpx_top_int: inactive/absent chip performed a bus drain on chip "
                               & integer'image(i)
                        severity failure;
                end if;
            end loop;
            pl("  ---- shot [" & tag
               & "] EF authority PASS: read="
               & integer'image(c_PHYSICAL_HITS)
               & ", leftover=0");
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
                    lc_face_index <= std_logic_vector(to_unsigned(f, 3));
                    do_shot(c + 1,
                            C_MODEL_WORDS_PER_IFIFO,
                            "run" & integer'image(run_idx)
                            & "/face" & integer'image(f)
                            & "/col" & integer'image(c));

                    if G_MAX_HITS_WRITE_MODE = 3 and f = 0 and c = 0 then
                        pl("[S_LATE] CTL21.max_hits_cfg write after first packet_start -> "
                           & integer'image(C_REQUESTED_MAX_HITS));
                        chip_wr(C_CHIP_SCAN_CFG, fn_pack_scan_timeout(C_REQUESTED_MAX_HITS, 0));
                        wait_clk(20);
                    end if;

                    if not (f = G_N_FACES - 1 and c = G_COLS_PER_FACE - 1) then
                        pl("[S7] shot_period wait ("
                           & integer'image(C_SHOT_PERIOD) & " AXIS clks)");
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
        assert G_MAX_HITS_OVERRIDE <= c_MAX_HITS_PER_STOP
            report "tb_tdc_gpx_top_int: G_MAX_HITS_OVERRIDE must be 0..7"
            severity failure;
        assert G_RECOVERY_MODE <= 2
            report "tb_tdc_gpx_top_int: G_RECOVERY_MODE must be 0..2"
            severity failure;
        assert not (G_BP_TREADY_GAP > 0 and G_BP_SHOT_STALL_CLKS > 0)
            report "tb_tdc_gpx_top_int: G_BP_TREADY_GAP and G_BP_SHOT_STALL_CLKS are mutually exclusive"
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

        -- Published CSR geometry is canonical packed storage, independent of
        -- AXIS output width. Also prove that native/generated aliases stay
        -- hidden outside the published 0x40..0x5C status window.
        pipe_rd(C_PIPE_HW_VERSION, x"00010000", '1', '1');
        pipe_rd(C_PIPE_HW_CONFIG, C_EXPECTED_HW_CONFIG, '1', '1');
        pipe_rd(C_PIPE_MAX_ROWS,
                std_logic_vector(to_unsigned(C_BUILD_MAX_ROWS, 32)), '1', '1');
        pipe_rd(C_PIPE_CELL_SIZE,
                std_logic_vector(to_unsigned(C_BUILD_CELL_BYTES, 32)), '1', '1');
        pipe_rd(C_PIPE_MAX_HSIZE,
                std_logic_vector(to_unsigned(C_BUILD_VDMA_BYTES, 32)), '1', '1');
        pipe_rd(C_PIPE_NATIVE_ALIAS, x"00000000", '1', '1');
        pipe_rd(C_PIPE_HIGH_RESERVED, x"00000000", '1', '1');
        report "tb_tdc_gpx_top_int: canonical CSR geometry/address windows - PASS"
            severity note;

        ----------------------------------------------------------------
        -- [S1] Pipeline CSR setup
        ----------------------------------------------------------------
        pl("[S1] Pipeline CSR: RANGE_COLS / MAIN_CTRL write");
        pipe_wr(C_PIPE_RANGE_COLS, C_RANGE_COLS_VAL);   -- 500m / 2 cols
        wait_clk(4);
        pipe_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);    -- mask=F, stops=2; faces are generic
        wait_clk(20);

        ----------------------------------------------------------------
        -- [S2] Chip CSR: cfg_image defaults (Reg0 / Reg5 / Reg6)
        --   Reg0 : TRiseEn (TStart + TStop1..2) bits 10,11,12 -> 0x0000_1C00
        --   Reg5 : MasterAluTrig (bit 23) + PartialAluTrig (bit 24) -> 0x0180_0000
        --   Reg6 : LF threshold = 4 -> 0x0000_0004
        ----------------------------------------------------------------
        pl("[S2] Chip CSR: cfg_image Reg0/Reg5/Reg6 write");
        chip_wr(C_CHIP_CFG_REG0, x"00381C00");  -- Reg0 template: both edges on start+stop1..2
        chip_wr(C_CHIP_CFG_REG5, x"01800000");  -- Reg5: ALU trig bits
        chip_wr(C_CHIP_CFG_REG6, x"00000004");  -- Reg6: LF threshold
        case G_MAX_HITS_WRITE_MODE is
            when 0 =>
                pl("[S2] CTL21.max_hits_cfg left unset -> expect build maximum");
            when 1 =>
                pl("[S2] CTL21.max_hits_cfg early request -> "
                   & integer'image(C_REQUESTED_MAX_HITS));
                chip_wr(C_CHIP_SCAN_CFG,
                        fn_pack_scan_timeout(C_REQUESTED_MAX_HITS, 0));
            when 2 =>
                pl("[S2] CTL21.max_hits_cfg early write 0 -> expect build maximum");
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
        -- stop_tdc-only stop leaves face_seq armed in ST_WAIT_SHOT, so the
        -- honest STAT5 expectation is busy='1'. STAT7 carries the
        -- masked-slope sticky when the wrong-slope scenario is active.
        if G_EXPECT_MASKED_SLOPE_DROP then
            read_pipeline_status("after-run1", x"00000001", x"00008000");
        else
            read_pipeline_status("after-run1", x"00000001");
        end if;

        if G_RECOVERY_MODE > 0 then
            issue_recovery(G_RECOVERY_MODE);
            start_measurement_run(2);
            run_all_shots(2);
            stop_measurement_run(2);
            read_pipeline_status("after-recovery-run", x"00000001");
        end if;

        ----------------------------------------------------------------
        -- [S10] Summary
        ----------------------------------------------------------------
        pl("====================================================");
        pl(" integrated sim end");
        pl("  config          : width=" & integer'image(G_TDATA_WIDTH)
           & "  axis_mhz=" & real'image(G_AXIS_CLK_MHZ)
           & "  tdc_mhz=" & real'image(G_TDC_CLK_MHZ)
           & "  max_hits=" & integer'image(C_MAX_HITS)
           & "  requested_max_hits=" & integer'image(C_REQUESTED_MAX_HITS)
           & "  build_max_hits=" & integer'image(G_BUILD_MAX_HITS_PER_STOP)
           & "  max_hits_override=" & integer'image(G_MAX_HITS_OVERRIDE)
           & "  max_hits_mode=" & fn_mode_name(G_MAX_HITS_WRITE_MODE)
           & "  stream_clk_mode=" & G_STREAM_CLK_MODE
           & "  active_chips=" & integer'image(C_ACTIVE_CHIPS)
           & "  chip_slope_mask=" & std_logic'image(G_CHIP_SLOPE_MASK(3))
           & std_logic'image(G_CHIP_SLOPE_MASK(2))
           & std_logic'image(G_CHIP_SLOPE_MASK(1))
           & std_logic'image(G_CHIP_SLOPE_MASK(0))
           & "  stops_per_chip=" & integer'image(C_EFFECTIVE_STOPS_PER_CHIP)
           & "  requested_stops=" & integer'image(G_STOPS_PER_CHIP)
           & "  build_max_stops=" & integer'image(G_BUILD_MAX_STOPS_PER_CHIP)
           & "  echoes_per_stop=" & integer'image(G_ECHOES_PER_STOP)
           & "  model_words_per_ififo=" & integer'image(C_MODEL_WORDS_PER_IFIFO)
           & "  faces=" & integer'image(G_N_FACES)
           & "  cols=" & integer'image(G_COLS_PER_FACE)
           & "  expected_beats_per_run_rise=" & integer'image(C_EXPECTED_AXIS_BEATS_RISE)
           & "  expected_beats_per_run_fall=" & integer'image(C_EXPECTED_AXIS_BEATS_FALL)
           & "  recovery_runs=" & integer'image(C_RECOVERY_RUNS)
           & "  expected_beats_total_rise=" & integer'image(C_EXPECTED_TOTAL_AXIS_BEATS_RISE)
           & "  expected_beats_total_fall=" & integer'image(C_EXPECTED_TOTAL_AXIS_BEATS_FALL)
           & "  bp_gap=" & integer'image(G_BP_TREADY_GAP)
           & "  bp_lane_mode=" & integer'image(G_BP_LANE_MODE)
           & "  recovery=" & fn_recovery_mode_name(G_RECOVERY_MODE));
        pl("  rising  stream  : beats=" & integer'image(mon_rise_beats)
           & "  tlast_cnt=" & integer'image(mon_rise_frame_end));
        pl("  falling stream  : beats=" & integer'image(mon_fall_beats)
           & "  tlast_cnt=" & integer'image(mon_fall_frame_end));
        pl("  vdma geometry   : hsize_rise=" & integer'image(to_integer(vdma_hsize_rise))
           & "  hsize_fall=" & integer'image(to_integer(vdma_hsize_fall))
           & "  vsize=" & integer'image(to_integer(vdma_vsize_lines)));
        pl("  c06 timing       : rise_first_cycle=" & integer'image(mon_rise_first_cycle)
           & "  rise_last_cycle=" & integer'image(mon_rise_last_cycle)
           & "  fall_first_cycle=" & integer'image(mon_fall_first_cycle)
           & "  fall_last_cycle=" & integer'image(mon_fall_last_cycle)
           & "  bp_stall_cycles=" & integer'image(mon_bp_stall_cycles));
        pl("  hit16 metadata   : force_hit16=" & boolean'image(G_FORCE_HIT16)
           & "  expect_min=" & integer'image(G_EXPECT_HIT16_META_MIN)
           & "  rise_meta=" & integer'image(mon_rise_metadata_count)
           & "  rise_nonzero=" & integer'image(mon_rise_hit16_meta_nonzero)
           & "  fall_meta=" & integer'image(mon_fall_metadata_count)
           & "  fall_nonzero=" & integer'image(mon_fall_hit16_meta_nonzero));
        pl("  irq counters     : o_irq=" & integer'image(mon_irq_cnt)
           & "  o_irq_pipe=" & integer'image(mon_irq_pipe_cnt));
        pl("====================================================");
        assert mon_rise_beats > 0
            report "tb_tdc_gpx_top_int: no rising beats observed"
            severity error;
        assert mon_rise_frame_end = C_EXPECTED_TOTAL_LINES
            report "tb_tdc_gpx_top_int: rising tlast count mismatch"
            severity error;
        if C_FALL_ACTIVE_CHIPS > 0 then
            assert mon_fall_beats > 0
                report "tb_tdc_gpx_top_int: no falling beats observed"
                severity error;
            assert mon_fall_frame_end = C_EXPECTED_TOTAL_LINES
                report "tb_tdc_gpx_top_int: falling tlast count mismatch"
                severity error;
        else
            assert mon_fall_beats = 0 and mon_fall_frame_end = 0
                report "tb_tdc_gpx_top_int: disabled falling lane emitted data"
                severity error;
        end if;
        assert mon_rise_beats = C_EXPECTED_TOTAL_AXIS_BEATS_RISE
            report "tb_tdc_gpx_top_int: rising beat count mismatch"
            severity error;
        assert mon_fall_beats = C_EXPECTED_TOTAL_AXIS_BEATS_FALL
            report "tb_tdc_gpx_top_int: falling beat count mismatch"
            severity error;
        if G_MAX_HITS_WRITE_MODE = 1 then
            assert to_integer(vdma_hsize_rise) =
                   fn_vdma_line_bytes(C_RISE_ACTIVE_CHIPS * C_EFFECTIVE_STOPS_PER_CHIP,
                                      C_MAX_HITS)
                report "tb_tdc_gpx_top_int: rising HSIZE mismatch"
                severity error;
            if C_FALL_ACTIVE_CHIPS > 0 then
                assert to_integer(vdma_hsize_fall) =
                       fn_vdma_line_bytes(C_FALL_ACTIVE_CHIPS * C_EFFECTIVE_STOPS_PER_CHIP,
                                          C_MAX_HITS)
                    report "tb_tdc_gpx_top_int: falling HSIZE mismatch"
                    severity error;
            else
                assert vdma_hsize_fall = 0
                    report "tb_tdc_gpx_top_int: disabled falling HSIZE must be zero"
                    severity error;
            end if;
        end if;
        assert to_integer(vdma_vsize_lines) = G_COLS_PER_FACE
            report "tb_tdc_gpx_top_int: VSIZE mismatch"
            severity error;
        assert mon_irq_pipe_cnt = 0
            report "tb_tdc_gpx_top_int: o_irq_pipe must stay reserved/low"
            severity error;
        assert mon_irq_cnt = 0
            report "tb_tdc_gpx_top_int: o_irq must not fire in normal C06 top_int run"
            severity error;
        if G_EXPECT_HIT16_META_MIN > 0 then
            assert mon_rise_hit16_meta_nonzero >= G_EXPECT_HIT16_META_MIN
                report "tb_tdc_gpx_top_int: rising Hit[16] metadata preservation count below expected"
                severity error;
            assert mon_fall_hit16_meta_nonzero >= G_EXPECT_HIT16_META_MIN
                report "tb_tdc_gpx_top_int: falling Hit[16] metadata preservation count below expected"
                severity error;
            report "tb_tdc_gpx_top_int: Hit[16] final metadata preservation - PASS"
                severity note;
        end if;
        if G_CHECK_EDGE_ROLE_WRITES then
            for i in 0 to c_MAX_CHIPS - 1 loop
                if G_PRESENT_CHIP_MASK(i) = '1' then
                    assert reg0_write_seen(i) = '1'
                        report "tb_tdc_gpx_top_int: GPX Reg0 write not observed on chip "
                               & integer'image(i)
                        severity error;
                    assert reg0_write_data(i) = fn_expected_reg0(i)
                        report "tb_tdc_gpx_top_int: GPX Reg0 edge-role mismatch on chip "
                               & integer'image(i)
                        severity error;
                end if;
            end loop;
            report "tb_tdc_gpx_top_int: per-chip GPX Reg0 edge-role programming - PASS"
                severity note;
        end if;
        if G_CHECK_32CH_ORDER then
            report "tb_tdc_gpx_top_int: 32-channel chip/stop exact-data order - PASS"
                severity note;
        end if;
        if G_BP_TREADY_GAP > 0 then
            assert mon_bp_stall_cycles > 0
                report "tb_tdc_gpx_top_int: backpressure mode did not create stall cycles"
                severity error;
            report "tb_tdc_gpx_top_int: bounded output backpressure preserved beats/tlast - PASS"
                severity note;
        end if;
        if G_BP_SHOT_STALL_CLKS > 0 then
            assert mon_bp_stall_cycles > 0
                report "tb_tdc_gpx_top_int: shot-boundary stall mode did not create stall cycles"
                severity error;
            assert mon_shot_stall_overlap
                report "tb_tdc_gpx_top_int: stall window did not overlap a shot_start "
                       & "(hazard window not exercised - test premise broken)"
                severity error;
            report "tb_tdc_gpx_top_int: shot-boundary stall preserved beats/tlast - PASS"
                severity note;
        end if;
        if G_EXPECT_MASKED_SLOPE_DROP then
            -- CHAIN P1: wrong-slope hits must surface as STAT7[15] with all
            -- other STAT7 stickies clean (exact compare).
            wait_clk(64);
            pipe_rd(C_PIPE_STATUS_EXT2, x"00008000", '1', '1');
            report "tb_tdc_gpx_top_int: masked-slope drop surfaced in STAT7[15] - PASS"
                severity note;
            -- Post-run evidence must remain readable through stop/abort, then
            -- CTL2[1] explicitly starts a clean diagnostic epoch.
            pipe_wr(C_PIPE_AUX_CMD, x"00000002");
            wait_clk(4);
            pipe_wr(C_PIPE_AUX_CMD, x"00000000");
            wait_clk(64);
            pipe_rd(C_PIPE_STATUS_EXT2, x"00000000", '1', '1');
            report "tb_tdc_gpx_top_int: masked-slope sticky soft-clear lifecycle - PASS"
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
