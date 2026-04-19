-- =============================================================================
-- tb_tdc_gpx_full_int.vhd
-- Full system integration TB:
--   enc_top -> motor_decoder_top -> laser_ctrl_top -> echo_receiver_top
--                                                 -> tdc_gpx_top
-- =============================================================================
--
-- Purpose
--   End-to-end health check TB wiring the four top modules the way they run
--   on the board, covering Phase 0..8 of the TDC-GPX flow plus the upstream
--   motor/laser scheduling path. Not a full functional verification -- the
--   goal is to confirm that CSR writes + encoder stimulus produce plausible
--   activity at every handoff (motor AXIS, laser AXIS, laser fire/start/stop
--   pulses, echo stop_evt AXIS, TDC VDMA stream).
--
-- Architecture
--   Single 200 MHz clock for all domains (all modules use the same clock group
--   in this TB to keep wiring simple; the real board uses 3 domains which
--   the CDCs inside each module still exercise because async helpers fall back
--   gracefully when src/dest clocks tie together).
--
--   enc_top  drives A/B/Z pins of motor_decoder_top (virtual encoder).
--   motor_decoder_top AXI-Stream output drives laser_ctrl_top slave port.
--     (Only md_tuser(8 downto 0) are consumed by laser_ctrl which still uses
--      the legacy 9-bit tuser interface; bit 9 "face_valid" is dropped here.)
--   laser_ctrl_top o_fire_pulse drives a photodiode pulse generator that
--     produces a rising pulse on echo_receiver_top.i_pd_lvds_p after a fixed
--     echo delay. That emulates the optical return path.
--   laser_ctrl_top o_start_tdc / o_stop_tdc drive tdc_gpx_top.i_shot_start and
--     tdc_gpx_top.i_stop_tdc.
--   laser_ctrl_top m_axis (result stream) drives tdc_gpx_top.i_lsr_tvalid /
--     i_lsr_tdata.
--   echo_receiver_top o_stop_evt_* feeds tdc_gpx_top.i_stop_evt_*.
--   A 4-chip behavioral TDC-GPX model (same as tb_tdc_gpx_top_int) fills the
--   IFIFOs and drives EF / LF / IrFlag pins.
--
-- CSR sequence (AXI-Lite writes, 200 MHz)
--   - motor_decoder : SIM_EN = 1                                  (CTL0[0])
--   - laser_ctrl    : LASER_EN + STREAM_EN + minimal sched config
--   - echo_receiver : SIM_EN = 1, multi-hit limit default
--   - tdc_gpx_top   : 500 m / 64 bit / cols=2 / stops=2 (same as top_int)
--
-- Scenario
--   [S0] Reset
--   [S1] Configure all CSRs
--   [S2] Enable encoder motion (s_enc_run)
--   [S3] Issue tdc_gpx_top CFG_WRITE + START
--   [S4] Let it run for ~120 us (several encoder face windows)
--   [S5] Summary: print activity counters at each handoff
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;
use work.laser_ctrl_cfg_pkg.all;   -- c_RES_TUSER_WIDTH = 21
-- Proven constants from the laser_ctrl TB (C_MD_CPR, C_MD_TICKS_LO,
-- C_MD_HI_COUNT, C_MD_FACE_CENTER_*, C_MD_CTL*_*, C_LC_CTL*_*).
use work.tb_laser_ctrl_pkg.all;
use work.tb_laser_ctrl_tests_pkg.all;

entity tb_tdc_gpx_full_int is
    generic (
        -- =================================================================
        -- INDEPENDENT (primary) variables -- system specification inputs.
        -- User overrides these via xelab --generic_top or Vivado simset.
        -- =================================================================
        G_AXIS_CLK_MHZ    : real    := 200.0;   -- common clock freq (MHz)
        G_MAX_RANGE_M     : real    := 500.0;   -- LiDAR max range (meters)
        G_SIM_TARGET_M    : real    := 375.0;   -- TB photodiode target (m)
        G_TDATA_WIDTH     : natural := 64;       -- VDMA tdata width (32|64)
        G_STOPS_PER_CHIP  : natural := 2;        -- active stops per chip (1..8)
        G_COLS_PER_FACE   : natural := 2;        -- shots per face
        G_N_FACES         : natural := 1;        -- faces per frame
        G_ACTIVE_CHIP_MASK: std_logic_vector(3 downto 0) := "1111";
        G_POWERUP_CLKS    : positive := 16;      -- chip_ctrl powerup (short sim)
        G_RECOVERY_CLKS   : positive := 4;
        G_ALU_PULSE_CLKS  : positive := 3;
        G_ENC_RUN_US      : real    := 100.0     -- how long encoder spins
    );
end entity tb_tdc_gpx_full_int;

architecture sim of tb_tdc_gpx_full_int is

    -- =========================================================================
    -- DEPENDENT (derived) constants -- all computed from the generics above.
    -- =========================================================================
    -- Physical constants
    constant C_LIGHT_M_PER_US : real    := 299.792;   -- speed of light (m/us)

    -- Clock period (rounded to integer ps to keep xsim time arithmetic exact)
    constant C_CLK_PERIOD_PS  : natural := natural(1000000.0 / G_AXIS_CLK_MHZ);
    constant C_CLK_PERIOD     : time    := C_CLK_PERIOD_PS * 1 ps;
    constant C_RST_HOLD       : time    := 30 * C_CLK_PERIOD;

    -- Range-derived counters (clk units)
    --   round-trip(D) = 2 * D / c * f_clk
    constant C_MAX_RANGE_CLKS : natural :=
        natural(2.0 * G_MAX_RANGE_M  / C_LIGHT_M_PER_US * G_AXIS_CLK_MHZ);
    constant C_SIM_TARGET_CLKS: natural :=
        natural(2.0 * G_SIM_TARGET_M / C_LIGHT_M_PER_US * G_AXIS_CLK_MHZ);

    -- PRF headroom (shot_period = 1.5 x round-trip per TDC window design memo)
    constant C_SHOT_PERIOD_CLKS : natural := (C_MAX_RANGE_CLKS * 3) / 2;

    -- TB photodiode echo delay (clk units)
    constant C_ECHO_DELAY     : natural := C_SIM_TARGET_CLKS;

    -- motor_decoder step rate: enc_top emits one state every TICKS_LO clk.
    -- Use tb_laser_ctrl_pkg derivation (assumes @150 MHz) but scale if needed.
    -- On our single 200 MHz clock the TICKS_LO stays numerically the same
    -- (encoder + motor_decoder run on same clock so the ratio matches).
    constant C_TICKS_LO       : natural := C_MD_TICKS_LO;   -- ~= 13
    constant C_STEP_INTERVAL  : natural :=
        (C_SHOT_PERIOD_CLKS + C_TICKS_LO - 1) / C_TICKS_LO;  -- ceiling

    -- max_hits table per distance memo (100m=1 / 250m=2 / 500m=3 / 750m=6 / 1000m=7)
    function fn_max_hits(r_m : real) return natural is
    begin
        if r_m <=  150.0 then return 1;
        elsif r_m <= 300.0 then return 2;
        elsif r_m <= 600.0 then return 3;
        elsif r_m <= 850.0 then return 6;
        else                   return 7;
        end if;
    end function;
    constant C_MAX_HITS       : natural := fn_max_hits(G_MAX_RANGE_M);

    -- pulse / trigger widths (kept short but >= 1 clk, >= 4 for safety)
    function fn_max_nat(a, b : natural) return natural is
    begin if a > b then return a; else return b; end if; end function;
    constant C_FIRE_WIDTH     : natural := fn_max_nat(C_FIRE_WIDTH_CLKS, 4);
    constant C_START_TDC_W    : natural := fn_max_nat(C_START_TDC_CLKS, 4);
    constant C_STOP_TDC_W     : natural := fn_max_nat(C_STOP_TDC_CLKS, 4);
    constant C_CTL3_VAL_LOCAL : natural := C_STOP_TDC_W * 65536 + C_START_TDC_W;

    -- CTL5 packed for laser_ctrl:
    --   [20:16] face_enable = 0x1F (all 5 mirror faces present in pkg),
    --   [15:0]  step_interval = C_STEP_INTERVAL
    function fn_pack_ctl5(face_en : std_logic_vector(4 downto 0);
                          step_interval : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(0, 11)) & face_en
             & std_logic_vector(to_unsigned(step_interval, 16));
    end function;
    constant C_LC_CTL5_DERIVED : std_logic_vector(31 downto 0) :=
        fn_pack_ctl5("11111", C_STEP_INTERVAL);

    -- tdc_gpx pipeline CSR values (packed from generics)
    --   MAIN_CTRL  [3:0]=chip_mask, [14:12]=n_faces, [18:15]=stops
    --   RANGE_COLS [15:0]=max_range_clks, [31:16]=cols_per_face
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
    constant C_MAIN_CTRL_BASE : std_logic_vector(31 downto 0) :=
        fn_pack_main_ctrl(G_ACTIVE_CHIP_MASK, G_N_FACES, G_STOPS_PER_CHIP);

    constant C_RANGE_COLS_VAL : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(G_COLS_PER_FACE, 16)) &
        std_logic_vector(to_unsigned(C_MAX_RANGE_CLKS, 16));

    -- Output width + stop-event width (for sub-module generic override)
    constant C_OUTPUT_W   : natural := G_TDATA_WIDTH;
    constant C_STOP_DW    : natural := c_STOP_EVT_DATA_WIDTH;  -- 32 (from tdc_gpx_pkg)

    -- Echo-receiver geometry matches tdc_gpx_pkg (c_N_CHIPS=4, c_MAX_STOPS=8).
    -- These are package-level locked constants -- echo_receiver MUST see them
    -- so the PD vector length and stop_evt packing line up with tdc_gpx_top.
    constant C_ER_N_CHIPS : natural := c_N_CHIPS;
    constant C_ER_N_STOPS : natural := c_MAX_STOPS_PER_CHIP;
    constant C_PD_WIDTH   : natural := C_ER_N_CHIPS * C_ER_N_STOPS;

    -- Chip model fixed parameters (not distance dependent)
    constant C_FIFO_DEPTH : natural := 32;
    constant C_LF_THRESH  : natural := 4;

    -- Encoder run wall-time (derived from generic)
    constant C_ENC_RUN_CLKS : natural :=
        natural(G_ENC_RUN_US * G_AXIS_CLK_MHZ);

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    signal clk       : std_logic := '0';
    signal rst_n     : std_logic := '0';
    signal sim_done  : boolean   := false;

    -- =========================================================================
    -- Encoder (enc_top outputs)
    -- =========================================================================
    signal enc_run   : std_logic := '0';
    signal enc_rst_n : std_logic := '0';
    signal enc_a, enc_b, enc_z : std_logic;
    signal enc_pos   : std_logic_vector(14 downto 0);
    signal enc_phase : std_logic_vector(1 downto 0);

    -- =========================================================================
    -- AXI-Lite #1: motor_decoder CSR (7-bit)
    -- =========================================================================
    signal md_awaddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal md_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal md_awvalid : std_logic := '0';
    signal md_awready : std_logic;
    signal md_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal md_wstrb   : std_logic_vector(3 downto 0)  := "1111";
    signal md_wvalid  : std_logic := '0';
    signal md_wready  : std_logic;
    signal md_bvalid  : std_logic;
    signal md_bready  : std_logic := '0';
    signal md_bresp   : std_logic_vector(1 downto 0);
    signal md_araddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal md_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal md_arvalid : std_logic := '0';
    signal md_arready : std_logic;
    signal md_rdata   : std_logic_vector(31 downto 0);
    signal md_rresp   : std_logic_vector(1 downto 0);
    signal md_rvalid  : std_logic;
    signal md_rready  : std_logic := '0';
    signal md_irq     : std_logic;

    -- motor_decoder AXI-Stream output
    signal md_tvalid  : std_logic;
    signal md_tdata   : std_logic_vector(31 downto 0);
    signal md_tuser_full : std_logic_vector(9 downto 0);
    signal md_tlast   : std_logic;

    -- =========================================================================
    -- AXI-Lite #2: laser_ctrl CSR (7-bit)
    -- =========================================================================
    signal lc_awaddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal lc_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal lc_awvalid : std_logic := '0';
    signal lc_awready : std_logic;
    signal lc_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal lc_wstrb   : std_logic_vector(3 downto 0)  := "1111";
    signal lc_wvalid  : std_logic := '0';
    signal lc_wready  : std_logic;
    signal lc_bvalid  : std_logic;
    signal lc_bready  : std_logic := '0';
    signal lc_bresp   : std_logic_vector(1 downto 0);
    signal lc_arvalid : std_logic := '0';
    signal lc_arready : std_logic;
    signal lc_araddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal lc_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal lc_rvalid  : std_logic;
    signal lc_rready  : std_logic := '0';
    signal lc_rdata   : std_logic_vector(31 downto 0);
    signal lc_rresp   : std_logic_vector(1 downto 0);
    signal lc_irq     : std_logic;

    -- laser_ctrl key outputs
    signal lc_fire_pulse   : std_logic;
    signal lc_start_tdc    : std_logic;
    signal lc_stop_tdc     : std_logic;
    signal lc_laser_active : std_logic;
    signal lc_warning      : std_logic_vector(4 downto 0);
    signal lc_warning_any  : std_logic;
    signal lc_m_tvalid     : std_logic;
    signal lc_m_tdata      : std_logic_vector(31 downto 0);
    signal lc_m_tuser      : std_logic_vector(c_RES_TUSER_WIDTH - 1 downto 0);
    signal lc_m_tlast      : std_logic;
    signal lc_fire_done    : std_logic := '0';

    -- laser debug outs (unused)
    signal lc_dbg_fire_trig, lc_dbg_fire_busy, lc_dbg_sim_flag   : std_logic;
    signal lc_dbg_sim_done, lc_dbg_fire_done_i, lc_dbg_timeout   : std_logic;
    signal lc_dbg_fire_delay : std_logic_vector(31 downto 0);
    signal lc_dbg_fsm_state  : std_logic_vector(2 downto 0);
    signal lc_dbg_shot_cnt   : std_logic_vector(15 downto 0);
    signal lc_dbg_face_start, lc_dbg_face_end, lc_dbg_shot_accept : std_logic;

    -- =========================================================================
    -- AXI-Lite #3: echo_receiver CSR (9-bit)
    -- =========================================================================
    signal er_awaddr  : std_logic_vector(8 downto 0) := (others => '0');
    signal er_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal er_awvalid : std_logic := '0';
    signal er_awready : std_logic;
    signal er_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal er_wstrb   : std_logic_vector(3 downto 0)  := "1111";
    signal er_wvalid  : std_logic := '0';
    signal er_wready  : std_logic;
    signal er_bvalid  : std_logic;
    signal er_bready  : std_logic := '0';
    signal er_bresp   : std_logic_vector(1 downto 0);
    signal er_arvalid : std_logic := '0';
    signal er_arready : std_logic;
    signal er_araddr  : std_logic_vector(8 downto 0) := (others => '0');
    signal er_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal er_rvalid  : std_logic;
    signal er_rready  : std_logic := '0';
    signal er_rdata   : std_logic_vector(31 downto 0);
    signal er_rresp   : std_logic_vector(1 downto 0);
    signal er_irq     : std_logic;

    -- echo_receiver PD input + stop_evt output
    signal pd_p, pd_n : std_logic_vector(C_PD_WIDTH - 1 downto 0) := (others => '0');
    signal er_pulse_rise, er_pulse_fall : std_logic_vector(C_PD_WIDTH - 1 downto 0);
    signal er_stop_tvalid : std_logic;
    signal er_stop_tdata  : std_logic_vector(C_STOP_DW - 1 downto 0);
    signal er_stop_tkeep  : std_logic_vector(C_STOP_DW/8 - 1 downto 0);
    signal er_stop_tuser  : std_logic_vector(C_STOP_DW - 1 downto 0);
    signal er_stop_tready : std_logic;

    -- =========================================================================
    -- AXI-Lite #4a: tdc_gpx_top chip CSR (9-bit)
    -- =========================================================================
    signal td_awaddr  : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal td_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal td_awvalid : std_logic := '0';
    signal td_awready : std_logic;
    signal td_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal td_wstrb   : std_logic_vector(3 downto 0)  := "1111";
    signal td_wvalid  : std_logic := '0';
    signal td_wready  : std_logic;
    signal td_bvalid  : std_logic;
    signal td_bready  : std_logic := '0';
    signal td_bresp   : std_logic_vector(1 downto 0);
    signal td_arvalid : std_logic := '0';
    signal td_arready : std_logic;
    signal td_araddr  : std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal td_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal td_rvalid  : std_logic;
    signal td_rready  : std_logic := '0';
    signal td_rdata   : std_logic_vector(31 downto 0);
    signal td_rresp   : std_logic_vector(1 downto 0);

    -- =========================================================================
    -- AXI-Lite #4b: tdc_gpx_top pipeline CSR (7-bit)
    -- =========================================================================
    signal tp_awaddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal tp_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal tp_awvalid : std_logic := '0';
    signal tp_awready : std_logic;
    signal tp_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal tp_wstrb   : std_logic_vector(3 downto 0)  := "1111";
    signal tp_wvalid  : std_logic := '0';
    signal tp_wready  : std_logic;
    signal tp_bvalid  : std_logic;
    signal tp_bready  : std_logic := '0';
    signal tp_bresp   : std_logic_vector(1 downto 0);
    signal tp_arvalid : std_logic := '0';
    signal tp_arready : std_logic;
    signal tp_araddr  : std_logic_vector(6 downto 0) := (others => '0');
    signal tp_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal tp_rvalid  : std_logic;
    signal tp_rready  : std_logic := '0';
    signal tp_rdata   : std_logic_vector(31 downto 0);
    signal tp_rresp   : std_logic_vector(1 downto 0);

    -- =========================================================================
    -- TDC-GPX top physical pins (driven by 4-chip behavioral model)
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

    -- VDMA sinks
    signal m_rise_tdata  : std_logic_vector(C_OUTPUT_W - 1 downto 0);
    signal m_rise_tvalid : std_logic;
    signal m_rise_tlast  : std_logic;
    signal m_rise_tuser  : std_logic_vector(0 downto 0);
    signal m_rise_tready : std_logic := '1';
    signal m_fall_tdata  : std_logic_vector(C_OUTPUT_W - 1 downto 0);
    signal m_fall_tvalid : std_logic;
    signal m_fall_tlast  : std_logic;
    signal m_fall_tuser  : std_logic_vector(0 downto 0);
    signal m_fall_tready : std_logic := '1';

    -- Calibration
    signal i_bin_res_ps : unsigned(15 downto 0) := to_unsigned(81, 16);
    signal i_k_dist     : unsigned(31 downto 0) := to_unsigned(54321, 32);
    signal td_irq, td_irq_pipe : std_logic;

    -- =========================================================================
    -- 4-chip behavioral TDC-GPX model state (same as tb_tdc_gpx_top_int)
    -- =========================================================================
    type t_fill_array is array (0 to c_N_CHIPS - 1) of natural;
    signal fifo1_fill   : t_fill_array := (others => 0);
    signal fifo2_fill   : t_fill_array := (others => 0);
    signal fifo1_rd_cnt : t_fill_array := (others => 0);
    signal fifo2_rd_cnt : t_fill_array := (others => 0);
    signal fifo_load_req : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');
    signal fifo_load_n1  : t_fill_array := (others => 0);
    signal fifo_load_n2  : t_fill_array := (others => 0);

    type t_chip_d_array is array (0 to c_N_CHIPS - 1)
        of std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal chip_d_out : t_chip_d_array := (others => (others => '0'));
    signal chip_d_oe  : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Activity counters (per handoff point)
    -- =========================================================================
    signal mon_md_beats     : natural := 0;
    signal mon_lc_fire_cnt  : natural := 0;
    signal mon_lc_start_cnt : natural := 0;
    signal mon_lc_stop_cnt  : natural := 0;
    signal mon_lc_m_beats   : natural := 0;
    signal mon_er_stop_beats : natural := 0;
    signal mon_er_pulse_rise_any : natural := 0;
    signal mon_td_rise_beats : natural := 0;
    signal mon_td_fall_beats : natural := 0;
    signal mon_td_rise_frame_end : natural := 0;
    signal mon_td_fall_frame_end : natural := 0;

    -- pipeline CSR addresses (C_MAIN_CTRL_BASE / C_RANGE_COLS_VAL are now
    -- declared in the derived-constants block above from the generics).
    constant C_PIPE_MAIN_CTRL  : std_logic_vector(6 downto 0) := "0000000";  -- 0x00
    constant C_PIPE_RANGE_COLS : std_logic_vector(6 downto 0) := "0000100";  -- 0x04

    -- laser_ctrl minimal config addresses (duplicate of laser_ctrl_cfg_pkg.c_ADDR_* but kept
    -- local to keep the TB self-contained)
    constant C_TB_LC_CTL0  : std_logic_vector(6 downto 0) := "0000000";  -- 0x00
    constant C_TB_LC_CTL1  : std_logic_vector(6 downto 0) := "0000100";  -- 0x04 fire width
    constant C_TB_LC_CTL3  : std_logic_vector(6 downto 0) := "0001100";  -- 0x0C tdc widths
    constant C_TB_LC_CTL5  : std_logic_vector(6 downto 0) := "0010100";  -- 0x14 sched cfg0
    constant C_TB_LC_CTL6  : std_logic_vector(6 downto 0) := "0011000";  -- 0x18 sched cfg1
    constant C_TB_LC_CTL7  : std_logic_vector(6 downto 0) := "0011100";  -- 0x1C sched cfg2

    -- motor_decoder / echo_receiver CTL0 addresses
    constant C_MD_CTL0 : std_logic_vector(6 downto 0) := "0000000";
    constant C_ER_CTL0 : std_logic_vector(8 downto 0) := "0" & x"00";

begin

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    clk <= not clk after C_CLK_PERIOD / 2 when not sim_done else '0';

    p_reset : process
    begin
        rst_n <= '0';
        wait for C_RST_HOLD;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait;
    end process p_reset;

    enc_rst_n <= rst_n and enc_run;

    -- =========================================================================
    -- Encoder (synthetic A/B/Z generator)
    -- =========================================================================
    u_enc : entity work.enc_top
        generic map (
            g_CPR      => 65536,
            g_DIR      => '0',
            g_TICKS_LO => 10,
            g_TICKS_HI => 10,
            g_HI_COUNT => 0
        )
        port map (
            i_clk           => clk,
            i_rst_n         => enc_rst_n,
            i_param_en      => '0',
            i_dir           => '0',
            i_ticks_lo      => (others => '0'),
            i_ticks_hi      => (others => '0'),
            i_hi_count      => (others => '0'),
            i_total_states  => (others => '0'),
            i_z_offset      => (others => '0'),
            i_z_early       => '0',
            i_z_width       => (others => '0'),
            o_ch_a          => enc_a,
            o_ch_b          => enc_b,
            o_ch_z          => enc_z,
            o_z_fault       => open,
            o_param_applied => open,
            o_position      => enc_pos,
            o_phase         => enc_phase
        );

    -- =========================================================================
    -- motor_decoder_top
    -- =========================================================================
    u_md : entity work.motor_decoder_top
        generic map (
            has_irq => true
        )
        port map (
            s_axi_aclk      => clk,
            s_axi_aresetn   => rst_n,
            i_enc_a         => enc_a,
            i_enc_b         => enc_b,
            i_enc_z         => enc_z,
            s_axi_awvalid   => md_awvalid,
            s_axi_awready   => md_awready,
            s_axi_awaddr    => md_awaddr,
            s_axi_awprot    => md_awprot,
            s_axi_wvalid    => md_wvalid,
            s_axi_wready    => md_wready,
            s_axi_wdata     => md_wdata,
            s_axi_wstrb     => md_wstrb,
            s_axi_bvalid    => md_bvalid,
            s_axi_bready    => md_bready,
            s_axi_bresp     => md_bresp,
            s_axi_arvalid   => md_arvalid,
            s_axi_arready   => md_arready,
            s_axi_araddr    => md_araddr,
            s_axi_arprot    => md_arprot,
            s_axi_rvalid    => md_rvalid,
            s_axi_rready    => md_rready,
            s_axi_rdata     => md_rdata,
            s_axi_rresp     => md_rresp,
            irq             => md_irq,
            m_axis_aclk     => clk,
            m_axis_aresetn  => rst_n,
            m_axis_tvalid   => md_tvalid,
            m_axis_tdata    => md_tdata,
            m_axis_tuser    => md_tuser_full,
            m_axis_tlast    => md_tlast,
            o_dbg_virt_a    => open,
            o_dbg_virt_b    => open,
            o_dbg_virt_z    => open,
            o_dbg_virt_pos  => open,
            o_dbg_virt_phase => open,
            o_dbg_dec_count => open,
            o_dbg_dec_dir   => open,
            o_dbg_z_pulse   => open,
            o_dbg_active    => open,
            o_dbg_active_any => open,
            o_dbg_face_index => open,
            o_dbg_sim_en    => open,
            o_dbg_rst_n_int => open,
            o_dbg_z_fault   => open,
            o_dbg_cfg_busy  => open
        );

    -- =========================================================================
    -- laser_ctrl_top
    --   Consumes motor_decoder AXI-S output (md_tuser(8:0) only, bit 9 dropped)
    -- =========================================================================
    u_lc : entity work.laser_ctrl_top
        port map (
            s_axi_aclk      => clk,
            s_axi_aresetn   => rst_n,
            s_axi_awvalid   => lc_awvalid,
            s_axi_awready   => lc_awready,
            s_axi_awaddr    => lc_awaddr,
            s_axi_awprot    => lc_awprot,
            s_axi_wvalid    => lc_wvalid,
            s_axi_wready    => lc_wready,
            s_axi_wdata     => lc_wdata,
            s_axi_wstrb     => lc_wstrb,
            s_axi_bvalid    => lc_bvalid,
            s_axi_bready    => lc_bready,
            s_axi_bresp     => lc_bresp,
            s_axi_arvalid   => lc_arvalid,
            s_axi_arready   => lc_arready,
            s_axi_araddr    => lc_araddr,
            s_axi_arprot    => lc_arprot,
            s_axi_rvalid    => lc_rvalid,
            s_axi_rready    => lc_rready,
            s_axi_rdata     => lc_rdata,
            s_axi_rresp     => lc_rresp,
            o_irq           => lc_irq,
            s_axis_aclk     => clk,
            s_axis_aresetn  => rst_n,
            s_axis_tvalid   => md_tvalid,
            s_axis_tdata    => md_tdata,
            s_axis_tuser    => md_tuser_full(8 downto 0),   -- drop bit 9
            s_axis_tlast    => md_tlast,
            i_fire_done     => lc_fire_done,
            o_fire_pulse    => lc_fire_pulse,
            o_start_tdc     => lc_start_tdc,
            o_stop_tdc      => lc_stop_tdc,
            o_laser_active  => lc_laser_active,
            o_warning       => lc_warning,
            o_warning_any   => lc_warning_any,
            m_axis_tvalid   => lc_m_tvalid,
            m_axis_tdata    => lc_m_tdata,
            m_axis_tuser    => lc_m_tuser,
            m_axis_tlast    => lc_m_tlast,
            o_dbg_fire_trigger => lc_dbg_fire_trig,
            o_dbg_fire_busy    => lc_dbg_fire_busy,
            o_dbg_sim_flag     => lc_dbg_sim_flag,
            o_dbg_sim_done     => lc_dbg_sim_done,
            o_dbg_fire_done    => lc_dbg_fire_done_i,
            o_dbg_fire_delay   => lc_dbg_fire_delay,
            o_dbg_fsm_state    => lc_dbg_fsm_state,
            o_dbg_shot_cnt     => lc_dbg_shot_cnt,
            o_dbg_timeout      => lc_dbg_timeout,
            o_dbg_face_start   => lc_dbg_face_start,
            o_dbg_face_end     => lc_dbg_face_end,
            o_dbg_shot_accept  => lc_dbg_shot_accept
        );

    -- =========================================================================
    -- echo_receiver_top
    --   Takes start_tdc/stop_tdc + PD pulses; emits stop_evt AXI-S
    -- =========================================================================
    u_er : entity work.echo_receiver_top
        generic map (
            g_N_CHIPS         => C_ER_N_CHIPS,
            g_STOPS_PER_CHIP  => C_ER_N_STOPS,
            g_STOP_CNT_WIDTH  => c_STOP_CNT_WIDTH,
            g_STOP_EVT_DWIDTH => C_STOP_DW
        )
        port map (
            s_axi_aclk       => clk,
            s_axi_aresetn    => rst_n,
            axis_aclk        => clk,
            axis_aresetn     => rst_n,
            s_axi_awvalid    => er_awvalid,
            s_axi_awready    => er_awready,
            s_axi_awaddr     => er_awaddr,
            s_axi_awprot     => er_awprot,
            s_axi_wvalid     => er_wvalid,
            s_axi_wready     => er_wready,
            s_axi_wdata      => er_wdata,
            s_axi_wstrb      => er_wstrb,
            s_axi_bvalid     => er_bvalid,
            s_axi_bready     => er_bready,
            s_axi_bresp      => er_bresp,
            s_axi_arvalid    => er_arvalid,
            s_axi_arready    => er_arready,
            s_axi_araddr     => er_araddr,
            s_axi_arprot     => er_arprot,
            s_axi_rvalid     => er_rvalid,
            s_axi_rready     => er_rready,
            s_axi_rdata      => er_rdata,
            s_axi_rresp      => er_rresp,
            i_start_tdc      => lc_start_tdc,
            i_stop_tdc       => lc_stop_tdc,
            i_pd_lvds_p      => pd_p,
            i_pd_lvds_n      => pd_n,
            o_stop_pulse_rise => er_pulse_rise,
            o_stop_pulse_fall => er_pulse_fall,
            o_stop_evt_tvalid => er_stop_tvalid,
            o_stop_evt_tdata  => er_stop_tdata,
            o_stop_evt_tkeep  => er_stop_tkeep,
            o_stop_evt_tuser  => er_stop_tuser,
            i_stop_evt_tready => er_stop_tready,
            o_irq             => er_irq
        );

    -- =========================================================================
    -- tdc_gpx_top (DUT, same config as tb_tdc_gpx_top_int)
    -- =========================================================================
    u_td : entity work.tdc_gpx_top
        generic map (
            g_HW_VERSION      => x"00010000",
            g_OUTPUT_WIDTH    => C_OUTPUT_W,
            g_POWERUP_CLKS    => G_POWERUP_CLKS,
            g_RECOVERY_CLKS   => G_RECOVERY_CLKS,
            g_ALU_PULSE_CLKS  => G_ALU_PULSE_CLKS,
            g_STOP_CNT_WIDTH  => c_STOP_CNT_WIDTH,
            g_STOP_EVT_DWIDTH => C_STOP_DW
        )
        port map (
            i_axis_aclk     => clk,
            i_axis_aresetn  => rst_n,
            i_tdc_clk       => clk,
            s_axi_aclk      => clk,
            s_axi_aresetn   => rst_n,
            s_axi_awvalid   => td_awvalid,
            s_axi_awready   => td_awready,
            s_axi_awaddr    => td_awaddr,
            s_axi_awprot    => td_awprot,
            s_axi_wvalid    => td_wvalid,
            s_axi_wready    => td_wready,
            s_axi_wdata     => td_wdata,
            s_axi_wstrb     => td_wstrb,
            s_axi_bvalid    => td_bvalid,
            s_axi_bready    => td_bready,
            s_axi_bresp     => td_bresp,
            s_axi_arvalid   => td_arvalid,
            s_axi_arready   => td_arready,
            s_axi_araddr    => td_araddr,
            s_axi_arprot    => td_arprot,
            s_axi_rvalid    => td_rvalid,
            s_axi_rready    => td_rready,
            s_axi_rdata     => td_rdata,
            s_axi_rresp     => td_rresp,
            s_axi_pipe_awvalid => tp_awvalid,
            s_axi_pipe_awready => tp_awready,
            s_axi_pipe_awaddr  => tp_awaddr,
            s_axi_pipe_awprot  => tp_awprot,
            s_axi_pipe_wvalid  => tp_wvalid,
            s_axi_pipe_wready  => tp_wready,
            s_axi_pipe_wdata   => tp_wdata,
            s_axi_pipe_wstrb   => tp_wstrb,
            s_axi_pipe_bvalid  => tp_bvalid,
            s_axi_pipe_bready  => tp_bready,
            s_axi_pipe_bresp   => tp_bresp,
            s_axi_pipe_arvalid => tp_arvalid,
            s_axi_pipe_arready => tp_arready,
            s_axi_pipe_araddr  => tp_araddr,
            s_axi_pipe_arprot  => tp_arprot,
            s_axi_pipe_rvalid  => tp_rvalid,
            s_axi_pipe_rready  => tp_rready,
            s_axi_pipe_rdata   => tp_rdata,
            s_axi_pipe_rresp   => tp_rresp,
            i_lsr_tvalid      => lc_m_tvalid,
            i_lsr_tdata       => lc_m_tdata,
            i_shot_start      => lc_start_tdc,
            i_stop_tdc        => lc_stop_tdc,
            i_stop_evt_tvalid => er_stop_tvalid,
            i_stop_evt_tdata  => er_stop_tdata,
            i_stop_evt_tkeep  => er_stop_tkeep,
            i_stop_evt_tuser  => er_stop_tuser,
            o_stop_evt_tready => er_stop_tready,
            io_tdc_d          => io_tdc_d,
            o_tdc_adr         => o_tdc_adr,
            o_tdc_csn         => o_tdc_csn,
            o_tdc_rdn         => o_tdc_rdn,
            o_tdc_wrn         => o_tdc_wrn,
            o_tdc_oen         => o_tdc_oen,
            o_tdc_stopdis     => o_tdc_stopdis,
            o_tdc_alutrigger  => o_tdc_alutrigger,
            o_tdc_puresn      => o_tdc_puresn,
            i_tdc_ef1         => i_tdc_ef1,
            i_tdc_ef2         => i_tdc_ef2,
            i_tdc_lf1         => i_tdc_lf1,
            i_tdc_lf2         => i_tdc_lf2,
            i_tdc_irflag      => i_tdc_irflag,
            i_tdc_errflag     => i_tdc_errflag,
            o_m_axis_tdata    => m_rise_tdata,
            o_m_axis_tvalid   => m_rise_tvalid,
            o_m_axis_tlast    => m_rise_tlast,
            o_m_axis_tuser    => m_rise_tuser,
            i_m_axis_tready   => m_rise_tready,
            o_m_axis_fall_tdata  => m_fall_tdata,
            o_m_axis_fall_tvalid => m_fall_tvalid,
            o_m_axis_fall_tlast  => m_fall_tlast,
            o_m_axis_fall_tuser  => m_fall_tuser,
            i_m_axis_fall_tready => m_fall_tready,
            i_bin_resolution_ps => i_bin_res_ps,
            i_k_dist_fixed      => i_k_dist,
            o_irq      => td_irq,
            o_irq_pipe => td_irq_pipe
        );

    -- =========================================================================
    -- 4-chip virtual TDC-GPX model (same as tb_tdc_gpx_top_int)
    -- =========================================================================
    gen_flags : for i in 0 to c_N_CHIPS - 1 generate
        i_tdc_ef1(i) <= '1' when fifo1_fill(i) = 0 else '0';
        i_tdc_ef2(i) <= '1' when fifo2_fill(i) = 0 else '0';
        i_tdc_lf1(i) <= '1' when fifo1_fill(i) >= C_LF_THRESH else '0';
        i_tdc_lf2(i) <= '1' when fifo2_fill(i) >= C_LF_THRESH else '0';
    end generate;

    gen_chip : for i in 0 to c_N_CHIPS - 1 generate
        p_chip : process(clk)
            variable v_rdn_prev : std_logic := '1';
            variable v_load_prev : std_logic := '0';
            variable v_my_fill1, v_my_fill2, v_my_rd1, v_my_rd2 : natural := 0;
        begin
            if rising_edge(clk) then
                if rst_n = '0' then
                    v_my_fill1 := 0; v_my_fill2 := 0; v_my_rd1 := 0; v_my_rd2 := 0;
                    chip_d_oe(i) <= '0';
                    chip_d_out(i) <= (others => '0');
                else
                    chip_d_oe(i) <= '0';
                    if fifo_load_req(i) = '1' and v_load_prev = '0' then
                        v_my_fill1 := fifo_load_n1(i);
                        v_my_fill2 := fifo_load_n2(i);
                        v_my_rd1 := 0; v_my_rd2 := 0;
                    end if;
                    v_load_prev := fifo_load_req(i);

                    if o_tdc_oen(i) = '0' and o_tdc_rdn(i) = '0'
                       and o_tdc_csn(i) = '0' then
                        chip_d_oe(i) <= '1';
                        if o_tdc_adr(i) = c_TDC_REG8_IFIFO1 then
                            chip_d_out(i) <= "00" & x"00" & '0' &
                                std_logic_vector(to_unsigned(
                                    (i * 256) + v_my_rd1 + 1, c_RAW_HIT_WIDTH));
                        elsif o_tdc_adr(i) = c_TDC_REG9_IFIFO2 then
                            chip_d_out(i) <= "00" & x"00" & '0' &
                                std_logic_vector(to_unsigned(
                                    (i * 256) + 128 + v_my_rd2 + 1, c_RAW_HIT_WIDTH));
                        else
                            chip_d_out(i) <= (others => '0');
                        end if;
                    end if;

                    if o_tdc_rdn(i) = '1' and v_rdn_prev = '0' then
                        if o_tdc_adr(i) = c_TDC_REG8_IFIFO1 and v_my_fill1 > 0 then
                            v_my_fill1 := v_my_fill1 - 1; v_my_rd1 := v_my_rd1 + 1;
                        elsif o_tdc_adr(i) = c_TDC_REG9_IFIFO2 and v_my_fill2 > 0 then
                            v_my_fill2 := v_my_fill2 - 1; v_my_rd2 := v_my_rd2 + 1;
                        end if;
                    end if;
                    v_rdn_prev := o_tdc_rdn(i);

                    fifo1_fill(i) <= v_my_fill1;
                    fifo2_fill(i) <= v_my_fill2;
                    fifo1_rd_cnt(i) <= v_my_rd1;
                    fifo2_rd_cnt(i) <= v_my_rd2;
                end if;
            end if;
        end process p_chip;
        io_tdc_d(i) <= chip_d_out(i) when chip_d_oe(i) = '1'
                                     else (others => 'Z');
    end generate gen_chip;

    -- =========================================================================
    -- Photodiode pulse generator
    --   On each lc_fire_pulse rising edge, schedule a rising pulse on pd_p(0)
    --   after C_ECHO_DELAY cycles. pd_n is kept low (differential simplified).
    --   Also preloads chip FIFOs so the TDC chip model has data to drain.
    -- =========================================================================
    p_pd : process(clk)
        variable v_fire_prev : std_logic := '0';
        variable v_delay_cnt : natural := 0;
        variable v_pulse_wait : boolean := false;
        variable v_pulse_hold : natural := 0;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                pd_p <= (others => '0');
                pd_n <= (others => '1');
                v_fire_prev := '0';
                v_delay_cnt := 0;
                v_pulse_wait := false;
                v_pulse_hold := 0;
                fifo_load_req <= (others => '0');
                for i in 0 to c_N_CHIPS - 1 loop
                    fifo_load_n1(i) <= 0;
                    fifo_load_n2(i) <= 0;
                end loop;
            else
                fifo_load_req <= (others => '0');
                pd_n <= (others => '1');   -- differential complement, inactive HI

                -- Detect fire_pulse rising edge -> start echo delay
                if lc_fire_pulse = '1' and v_fire_prev = '0' then
                    v_delay_cnt := C_ECHO_DELAY;
                    v_pulse_wait := true;
                    -- Preload chip FIFOs so chip_ctrl drains them after IrFlag
                    for i in 0 to c_N_CHIPS - 1 loop
                        fifo_load_n1(i) <= G_STOPS_PER_CHIP;
                        fifo_load_n2(i) <= G_STOPS_PER_CHIP;
                    end loop;
                    fifo_load_req <= (others => '1');
                end if;
                v_fire_prev := lc_fire_pulse;

                -- Emit pd rising pulse after delay, hold for 10 cycles
                if v_pulse_wait then
                    if v_delay_cnt = 0 then
                        v_pulse_wait := false;
                        v_pulse_hold := 10;
                    else
                        v_delay_cnt := v_delay_cnt - 1;
                    end if;
                end if;

                if v_pulse_hold > 0 then
                    pd_p(0) <= '1';
                    pd_n(0) <= '0';
                    v_pulse_hold := v_pulse_hold - 1;
                else
                    pd_p(0) <= '0';
                end if;
            end if;
        end if;
    end process p_pd;

    -- =========================================================================
    -- IrFlag model:  assert for all chips ~50 cycles after fire_pulse
    --   (TDC-GPX MTimer expiry emulation)
    -- =========================================================================
    p_irflag : process(clk)
        variable v_fire_prev : std_logic := '0';
        variable v_cnt : natural := 0;
        variable v_asserting : boolean := false;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                i_tdc_irflag <= (others => '0');
                v_fire_prev := '0';
                v_cnt := 0;
                v_asserting := false;
            else
                if lc_fire_pulse = '1' and v_fire_prev = '0' then
                    v_cnt := 50;
                    v_asserting := true;
                end if;
                v_fire_prev := lc_fire_pulse;

                if v_asserting then
                    if v_cnt = 0 then
                        i_tdc_irflag <= (others => '1');
                    else
                        v_cnt := v_cnt - 1;
                    end if;
                end if;

                -- Auto-clear after a while
                if lc_stop_tdc = '1' then
                    i_tdc_irflag <= (others => '0');
                    v_asserting := false;
                end if;
            end if;
        end if;
    end process p_irflag;

    -- =========================================================================
    -- Monitors
    -- =========================================================================
    p_mon : process(clk)
        variable v_prev_fire, v_prev_start, v_prev_stop : std_logic := '0';
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                mon_md_beats <= 0;
                mon_lc_fire_cnt <= 0;
                mon_lc_start_cnt <= 0;
                mon_lc_stop_cnt <= 0;
                mon_lc_m_beats <= 0;
                mon_er_stop_beats <= 0;
                mon_er_pulse_rise_any <= 0;
                mon_td_rise_beats <= 0;
                mon_td_fall_beats <= 0;
                mon_td_rise_frame_end <= 0;
                mon_td_fall_frame_end <= 0;
                v_prev_fire := '0';
                v_prev_start := '0';
                v_prev_stop := '0';
            else
                if md_tvalid = '1' then
                    mon_md_beats <= mon_md_beats + 1;
                end if;
                if lc_fire_pulse = '1' and v_prev_fire = '0' then
                    mon_lc_fire_cnt <= mon_lc_fire_cnt + 1;
                end if;
                if lc_start_tdc = '1' and v_prev_start = '0' then
                    mon_lc_start_cnt <= mon_lc_start_cnt + 1;
                end if;
                if lc_stop_tdc = '1' and v_prev_stop = '0' then
                    mon_lc_stop_cnt <= mon_lc_stop_cnt + 1;
                end if;
                v_prev_fire := lc_fire_pulse;
                v_prev_start := lc_start_tdc;
                v_prev_stop := lc_stop_tdc;

                if lc_m_tvalid = '1' then
                    mon_lc_m_beats <= mon_lc_m_beats + 1;
                end if;
                if er_stop_tvalid = '1' and er_stop_tready = '1' then
                    mon_er_stop_beats <= mon_er_stop_beats + 1;
                end if;
                if er_pulse_rise /= (er_pulse_rise'range => '0') then
                    mon_er_pulse_rise_any <= mon_er_pulse_rise_any + 1;
                end if;
                if m_rise_tvalid = '1' and m_rise_tready = '1' then
                    mon_td_rise_beats <= mon_td_rise_beats + 1;
                    if m_rise_tlast = '1' then
                        mon_td_rise_frame_end <= mon_td_rise_frame_end + 1;
                    end if;
                end if;
                if m_fall_tvalid = '1' and m_fall_tready = '1' then
                    mon_td_fall_beats <= mon_td_fall_beats + 1;
                    if m_fall_tlast = '1' then
                        mon_td_fall_frame_end <= mon_td_fall_frame_end + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_mon;

    -- =========================================================================
    -- Stimulus
    -- =========================================================================
    p_stim : process

        procedure pl(msg : string) is
            variable lv : line;
        begin
            write(lv, now, right, 12);
            write(lv, string'("  "));
            write(lv, msg);
            writeline(output, lv);
        end procedure;

        procedure wait_clk(n : natural) is
        begin
            for i in 1 to n loop
                wait until rising_edge(clk);
            end loop;
        end procedure;

        -- AXI-Lite write helper (generic address width)
        procedure axilw_7b(signal awaddr  : out std_logic_vector(6 downto 0);
                           signal awvalid : out std_logic;
                           signal awready : in  std_logic;
                           signal wdata   : out std_logic_vector(31 downto 0);
                           signal wvalid  : out std_logic;
                           signal wready  : in  std_logic;
                           signal bvalid  : in  std_logic;
                           signal bready  : out std_logic;
                           addr : std_logic_vector(6 downto 0);
                           val  : std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clk);
            awaddr <= addr; awvalid <= '1';
            wdata <= val; wvalid <= '1';
            bready <= '1';
            while awready = '0' or wready = '0' loop
                wait until rising_edge(clk);
            end loop;
            wait until rising_edge(clk);
            awvalid <= '0'; wvalid <= '0';
            while bvalid = '0' loop
                wait until rising_edge(clk);
            end loop;
            wait until rising_edge(clk);
            bready <= '0';
        end procedure;

        procedure axilw_9b(signal awaddr  : out std_logic_vector(8 downto 0);
                           signal awvalid : out std_logic;
                           signal awready : in  std_logic;
                           signal wdata   : out std_logic_vector(31 downto 0);
                           signal wvalid  : out std_logic;
                           signal wready  : in  std_logic;
                           signal bvalid  : in  std_logic;
                           signal bready  : out std_logic;
                           addr : std_logic_vector(8 downto 0);
                           val  : std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clk);
            awaddr <= addr; awvalid <= '1';
            wdata <= val; wvalid <= '1';
            bready <= '1';
            while awready = '0' or wready = '0' loop
                wait until rising_edge(clk);
            end loop;
            wait until rising_edge(clk);
            awvalid <= '0'; wvalid <= '0';
            while bvalid = '0' loop
                wait until rising_edge(clk);
            end loop;
            wait until rising_edge(clk);
            bready <= '0';
        end procedure;

    begin
        wait until rst_n = '1';
        wait_clk(10);

        pl("======================================================");
        pl(" full system integration TB start");
        pl(" (enc_top + motor_decoder + laser_ctrl + echo_receiver + tdc_gpx)");
        pl("======================================================");

        --------------------------------------------------------------
        -- [S1] CSR configuration (mirrors tb_laser_ctrl TEST 1 / TEST 2)
        --------------------------------------------------------------
        pl("[S1] motor_decoder bring-up (PHYS mode + CPR + ticks + faces + APPLY)");
        axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                 md_bvalid, md_bready, C_MD_CTL0, C_MD_CTL0_PHYS);
        axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                 md_bvalid, md_bready, C_MD_CTL1,
                 std_logic_vector(to_unsigned(C_MD_CPR, 32)));
        axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                 md_bvalid, md_bready, C_MD_CTL2,
                 std_logic_vector(to_unsigned(C_MD_TICKS_LO, 32)));
        axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                 md_bvalid, md_bready, C_MD_CTL3,
                 std_logic_vector(to_unsigned(C_MD_HI_COUNT, 32)));
        wait_clk(100);
        -- sw reset pulse (CTL0 SWRST_PHYS then clear)
        axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                 md_bvalid, md_bready, C_MD_CTL0, C_MD_CTL0_SWRST_PHYS);
        wait_clk(5);
        axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                 md_bvalid, md_bready, C_MD_CTL0, C_MD_CTL0_PHYS);
        wait_clk(30);
        -- per-face centers (CTL5/CTL6/CTL7 loop)
        for f in 0 to C_N_FACES - 1 loop
            case f is
                when 0 =>
                    axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid,
                             md_wready, md_bvalid, md_bready, C_MD_CTL5,
                             std_logic_vector(to_unsigned(C_MD_FACE_CENTER_0, 32)));
                when 1 =>
                    axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid,
                             md_wready, md_bvalid, md_bready, C_MD_CTL5,
                             std_logic_vector(to_unsigned(C_MD_FACE_CENTER_1, 32)));
                when 2 =>
                    axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid,
                             md_wready, md_bvalid, md_bready, C_MD_CTL5,
                             std_logic_vector(to_unsigned(C_MD_FACE_CENTER_2, 32)));
                when 3 =>
                    axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid,
                             md_wready, md_bvalid, md_bready, C_MD_CTL5,
                             std_logic_vector(to_unsigned(C_MD_FACE_CENTER_3, 32)));
                when 4 =>
                    axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid,
                             md_wready, md_bvalid, md_bready, C_MD_CTL5,
                             std_logic_vector(to_unsigned(C_MD_FACE_CENTER_4, 32)));
                when others => null;
            end case;
            axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                     md_bvalid, md_bready, C_MD_CTL6,
                     std_logic_vector(to_unsigned(C_MD_FACE_HALF_ST, 32)));
            axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                     md_bvalid, md_bready, C_MD_CTL7,
                     std_logic_vector(to_unsigned(f, 32)));
        end loop;
        axilw_7b(md_awaddr, md_awvalid, md_awready, md_wdata, md_wvalid, md_wready,
                 md_bvalid, md_bready, C_MD_CTL7, C_MD_CTL7_APPLY);
        wait_clk(200);

        pl("[S1] echo_receiver CSR: physical mode (SIM_EN=0) + multi_hit_limit=7");
        -- CTL0: SIM_EN=0 (physical pd_lvds path), multi_hit_limit=7 ([12:8])
        axilw_9b(er_awaddr, er_awvalid, er_awready, er_wdata, er_wvalid, er_wready,
                 er_bvalid, er_bready, C_ER_CTL0, x"00000700");
        wait_clk(10);

        pl("[S1] laser_ctrl CSR: 500 m profile (CTL2/CTL4/CTL5 overridden)");
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL1,
                 std_logic_vector(to_unsigned(C_FIRE_WIDTH, 32)));
        -- CTL2 max_roundtrip -- derived from G_MAX_RANGE_M / G_AXIS_CLK_MHZ
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL2,
                 std_logic_vector(to_unsigned(C_MAX_RANGE_CLKS, 32)));
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL3,
                 std_logic_vector(to_unsigned(C_CTL3_VAL_LOCAL, 32)));
        -- CTL4 sim target -- derived from G_SIM_TARGET_M / G_AXIS_CLK_MHZ
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL4,
                 std_logic_vector(to_unsigned(C_SIM_TARGET_CLKS, 32)));
        -- CTL5 step_interval -- derived to guarantee shot_period >= 1.5 * round-trip
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL5, C_LC_CTL5_DERIVED);
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL6, C_LC_CTL6_DEFAULT);
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL7, C_LC_CTL7_DEFAULT);
        -- sw_rst + enable (CTL0 SWRST then EN)
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL0, C_LC_CTL0_SWRST_EN);
        wait_clk(5);
        axilw_7b(lc_awaddr, lc_awvalid, lc_awready, lc_wdata, lc_wvalid, lc_wready,
                 lc_bvalid, lc_bready, C_LC_CTL0, C_LC_CTL0_EN);
        wait_clk(100);

        pl("[S1] tdc_gpx chip CSR: cfg_image Reg0/Reg5/Reg6");
        axilw_9b(td_awaddr, td_awvalid, td_awready, td_wdata, td_wvalid, td_wready,
                 td_bvalid, td_bready, "0" & x"14", x"00001C00");  -- Reg0 TRiseEn
        axilw_9b(td_awaddr, td_awvalid, td_awready, td_wdata, td_wvalid, td_wready,
                 td_bvalid, td_bready, "0" & x"28", x"01800000");  -- Reg5 ALU trig
        axilw_9b(td_awaddr, td_awvalid, td_awready, td_wdata, td_wvalid, td_wready,
                 td_bvalid, td_bready, "0" & x"2C", x"00000004");  -- Reg6 LF threshold
        wait_clk(20);

        pl("[S1] tdc_gpx pipeline CSR: RANGE_COLS / MAIN_CTRL");
        axilw_7b(tp_awaddr, tp_awvalid, tp_awready, tp_wdata, tp_wvalid, tp_wready,
                 tp_bvalid, tp_bready, C_PIPE_RANGE_COLS, C_RANGE_COLS_VAL);
        axilw_7b(tp_awaddr, tp_awvalid, tp_awready, tp_wdata, tp_wvalid, tp_wready,
                 tp_bvalid, tp_bready, C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
        wait_clk(20);

        pl("[S2] tdc_gpx CFG_WRITE pulse");
        axilw_7b(tp_awaddr, tp_awvalid, tp_awready, tp_wdata, tp_wvalid, tp_wready,
                 tp_bvalid, tp_bready, C_PIPE_MAIN_CTRL,
                 (C_MAIN_CTRL_BASE or x"80000000"));
        wait_clk(4);
        axilw_7b(tp_awaddr, tp_awvalid, tp_awready, tp_wdata, tp_wvalid, tp_wready,
                 tp_bvalid, tp_bready, C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
        wait_clk(2500);

        pl("[S3] tdc_gpx START pulse");
        axilw_7b(tp_awaddr, tp_awvalid, tp_awready, tp_wdata, tp_wvalid, tp_wready,
                 tp_bvalid, tp_bready, C_PIPE_MAIN_CTRL,
                 (C_MAIN_CTRL_BASE or x"10000000"));
        wait_clk(4);
        axilw_7b(tp_awaddr, tp_awvalid, tp_awready, tp_wdata, tp_wvalid, tp_wready,
                 tp_bvalid, tp_bready, C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
        wait_clk(80);

        --------------------------------------------------------------
        -- [S4] Start encoder motion and let chain run for long time
        --------------------------------------------------------------
        pl("[S4] enable encoder motion");
        enc_run <= '1';
        wait_clk(C_ENC_RUN_CLKS);  -- G_ENC_RUN_US worth of encoder motion
        pl("[S5] stop encoder, wait for pending frames");
        enc_run <= '0';
        wait_clk(C_ENC_RUN_CLKS / 4);  -- 25% drain tail

        --------------------------------------------------------------
        -- [S6] Summary
        --------------------------------------------------------------
        pl("======================================================");
        pl(" full integration TB summary:");
        pl("  md     AXI-S beats  = " & integer'image(mon_md_beats));
        pl("  lc     fire pulses  = " & integer'image(mon_lc_fire_cnt));
        pl("  lc     start_tdc    = " & integer'image(mon_lc_start_cnt));
        pl("  lc     stop_tdc     = " & integer'image(mon_lc_stop_cnt));
        pl("  lc     result beats = " & integer'image(mon_lc_m_beats));
        pl("  er     pd rise hits = " & integer'image(mon_er_pulse_rise_any));
        pl("  er     stop_evt beats = " & integer'image(mon_er_stop_beats));
        pl("  td     rise VDMA beats = " & integer'image(mon_td_rise_beats)
           & "  tlast cnt = " & integer'image(mon_td_rise_frame_end));
        pl("  td     fall VDMA beats = " & integer'image(mon_td_fall_beats)
           & "  tlast cnt = " & integer'image(mon_td_fall_frame_end));
        pl("======================================================");

        if mon_md_beats > 0 and mon_lc_start_cnt > 0 then
            report "tb_tdc_gpx_full_int: chain activity observed - PASS"
                severity note;
        else
            report "tb_tdc_gpx_full_int: NO chain activity - CHECK CSR/config"
                severity warning;
        end if;

        sim_done <= true;
        wait;
    end process p_stim;

    -- Watchdog
    p_wdog : process
    begin
        wait for 500 us;
        if not sim_done then
            report "tb_tdc_gpx_full_int: watchdog timeout (500 us)" severity failure;
        end if;
        wait;
    end process p_wdog;

end architecture sim;
