-- =============================================================================
-- tb_tdc_gpx_full_int.vhd
-- Full system integration TB:
--   enc_top -> motor_laser_ctrl_top -> echo_receiver_top -> external GPX model
--                                           \-------> tdc_gpx_top
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
--   Motor/Laser/Echo and GPX processing use G_AXIS_CLK_MHZ.  The GPX physical
--   bus model and chip controller use the independent G_TDC_CLK_MHZ clock.
--   Setting both generics to the same value preserves the single-clock smoke;
--   150/200 MHz exercises the product-reference AXIS/TDC CDC boundary.
--
--   enc_top drives A/B/Z pins of motor_laser_ctrl_top in physical-source mode.
--   motor_laser_ctrl_top keeps the Motor-to-Laser AXI4-Stream private.
--     TKEEP is always 1111; TUSER carries latency, overlap, simulation, and
--     face-index metadata according to the current 8-bit Motor/Laser ABI.
--   echo_receiver_top owns the optional synthetic target path. Its CSR delay
--     table produces STOP waveforms exactly where the physical LVDS frontend
--     would otherwise produce them.
--   motor_laser_ctrl_top o_shot_start / o_stop_tdc drive tdc_gpx_top lifecycle;
--   o_start_tdc is reserved for the physical GPX START pin.
--   motor_laser_ctrl_top m_axis drives echo_receiver's backpressured one-beat Shot
--     descriptor input. Its fire-count/face/encoder payload does not drive
--     GPX geometry.
--   echo_receiver_top o_stop_evt_* remains a read-only diagnostic stream.
--   A simulation-only external TDC-GPX model measures START-to-STOP, builds
--   the 28-bit I-Mode word, fills IFIFOs, and drives EF/LF/IrFlag plus the
--   physical bus. tdc_gpx_top remains only an external-chip bus reader.
--
-- CSR sequence (AXI-Lite writes via px_utility_pkg, AXIS clock in this TB)
--   - motor_decoder : SIM_EN = 1                                  (CTL0[0])
--   - laser_ctrl    : LASER_EN + STREAM_EN + minimal sched config
--   - echo_receiver : per-channel synthetic STOP delays in fixed 5 ns ticks
--   - tdc_gpx_top   : 500 m / G_TDATA_WIDTH / cols=2 / stops=2 (same as top_int)
--
-- Scenario
--   [S0] Reset
--   [S1] Configure all CSRs
--   [S2] Enable encoder motion (s_enc_run)
--   [S3] Issue tdc_gpx_top CFG_WRITE + START
--   [S4] Observe the encoder for the scenario-selected interval
--   [S5] Summary: print activity counters at each handoff
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
use work.lidar_unified_csr_pkg.all;
use work.px_utility_pkg.all;
use work.laser_ctrl_cfg_pkg.all;   -- c_RES_TUSER_WIDTH = 21
-- Geometry and register constants are reused from the laser_ctrl TB package.
-- Encoder tick counts are derived locally from this TB's actual AXIS clock.
use work.tb_laser_ctrl_pkg.all;
use work.tb_laser_ctrl_tests_pkg.all;

entity tb_tdc_gpx_full_int is
    generic (
        -- =================================================================
        -- INDEPENDENT (primary) variables -- system specification inputs.
        -- User overrides these via xelab --generic_top or Vivado simset.
        -- =================================================================
        G_AXIS_CLK_MHZ    : real    := 200.0;   -- processing clock (MHz)
        G_TDC_CLK_MHZ     : real    := 200.0;   -- GPX physical-bus clock (MHz)
        G_MAX_RANGE_M     : real    := 500.0;   -- LiDAR max range (meters)
        G_SIM_TARGET_M    : real    := 375.0;   -- Echo synthetic target (m)
        -- Motor/optical timing inputs. The mirror mechanical step is one half
        -- of the requested optical scan-angle step.
        G_REV_TIME_US     : real    := 100.0;   -- mirror revolution period
        G_OPTICAL_SHOT_INTERVAL_DEG : real := 36.5;
        -- Product operating point represented by the accelerated encoder
        -- profile above. Zero keeps legacy direct-elaboration behavior by
        -- treating the simulated RPM/interval as the operating values.
        G_OPERATING_MOTOR_RPM : real := 0.0;
        G_HORIZONTAL_RESOLUTION_DEG : real := 0.0;
        G_TDATA_WIDTH     : natural := 64;       -- VDMA tdata width (32|64|128)
        G_STOPS_PER_CHIP  : natural := 2;        -- active stops per chip (1..8)
        -- Physical GPX words generated per enabled STOP and the downstream
        -- Cell capacity are independent contracts. The former must not exceed
        -- the latter in a healthy operating profile.
        G_RETURNS_PER_STOP : positive range 1 to 7 := 1;
        G_MAX_HITS_CFG      : positive range 1 to 7 := 3;
        -- Existing tdc_gpx_top physical-time policy, exposed here so every
        -- integration result records the exact post-IrFlag drain budget.
        G_TDC_DRAIN_MARGIN_TIME_NS : positive := 6000;
        G_COLS_PER_FACE   : natural := 2;        -- shots per face
        G_N_FACES         : positive range 1 to 5 := 1; -- compile-time faces
        -- All four behavioral GPX chips are active in the default profile.
        G_ACTIVE_CHIP_MASK: std_logic_vector(3 downto 0) := "1111";
        -- Raw GPX I-Mode Slope bit generated by each behavioral chip.
        -- '1' = rising, '0' = falling.  The default matches the product's
        -- dedicated topology: chip 0/1 rising and chip 2/3 falling.
        G_CHIP_SLOPE_MASK : std_logic_vector(3 downto 0) := "0011";
        G_POWERUP_CLKS    : positive := 16;      -- chip_ctrl powerup (short sim)
        G_RECOVERY_CLKS   : positive := 4;
        G_ALU_PULSE_CLKS  : positive := 3;
        G_ENC_RUN_US      : real    := 100.0;    -- how long encoder spins
        -- Encoder ownership:
        --   "internal" = motor_decoder_top's built-in enc_top (product SIM path)
        --   "external" = this TB's enc_top drives the physical A/B/Z inputs
        -- The two sources are mutually exclusive so the selected path is real.
        G_ENCODER_SOURCE  : string  := "internal";
        -- "synthetic_single": Echo CSR generates one STOP0/chip (legacy smoke)
        -- "physical_multi"  : TB drives every LVDS channel through
        --                     echo_receiver, with G_RETURNS_PER_STOP pulses.
        G_ECHO_STIM_MODE  : string  := "synthetic_single";
        G_REARM_GUARD_5NS_TICKS : natural range 0 to 65535 := 0;
        -- Debug / experiment:
        --   'lc'     = i_shot_start / i_stop_tdc driven from laser_ctrl (normal)
        --   'direct' = TB drives them directly as 1-clk pulses (bypass lc), used
        --              to isolate whether the chain breaks inside TDC vs at the
        --              laser_ctrl -> TDC handshake.
        G_TDC_STIM_MODE   : string  := "lc";
        -- R4 scenario knobs
        --   G_BP_TREADY_GAP : natural := 0
        --     0  = VDMA tready always '1' (no back-pressure)
        --     N  = toggle m_rise_tready / m_fall_tready low for 2 clks
        --          every N clks (emulates downstream VDMA stall)
        G_BP_TREADY_GAP   : natural := 0;
        -- Emit a one-line config banner at sim start (helps correlate waves
        -- with chosen generics when runs are batched).
        G_PRINT_BANNER    : boolean := true;
        -- "local" keeps the four legacy AXI-Lite banks. "unified" disables
        -- them and drives every IP through one 32-CTL / 32-STAT CSR owner.
        G_CONTROL_MODE    : string := "local"
    );
end entity tb_tdc_gpx_full_int;

architecture sim of tb_tdc_gpx_full_int is

    type t_uword_array is array (natural range <>) of
        std_logic_vector(31 downto 0);
    constant C_USE_LOCAL_CSR : boolean := G_CONTROL_MODE = "local";

    -- =========================================================================
    -- DEPENDENT (derived) constants -- all computed from the generics above.
    -- =========================================================================
    -- Physical constants
    constant C_LIGHT_M_PER_US : real    := 299.792;   -- speed of light (m/us)

    function fn_positive_or_default(
        value    : real;
        fallback : real
    ) return real is
    begin
        if value > 0.0 then
            return value;
        end if;
        return fallback;
    end function;

    constant C_OPERATING_MOTOR_RPM : real := fn_positive_or_default(
        G_OPERATING_MOTOR_RPM, 60000000.0 / G_REV_TIME_US);
    constant C_HORIZONTAL_RESOLUTION_DEG : real := fn_positive_or_default(
        G_HORIZONTAL_RESOLUTION_DEG, G_OPTICAL_SHOT_INTERVAL_DEG);
    constant C_OPERATING_REV_TIME_US : real :=
        60000000.0 / C_OPERATING_MOTOR_RPM;
    -- Optical angle is twice the mirror mechanical angle.
    constant C_OPERATING_POINT_INTERVAL_US : real :=
        C_OPERATING_REV_TIME_US * C_HORIZONTAL_RESOLUTION_DEG / 720.0;
    constant C_SIM_POINT_INTERVAL_US : real :=
        G_REV_TIME_US * G_OPTICAL_SHOT_INTERVAL_DEG / 720.0;
    constant C_OPERATING_POINT_INTERVAL_CLKS : natural := natural(ceil(
        C_OPERATING_POINT_INTERVAL_US * G_AXIS_CLK_MHZ));

    -- Clock periods (rounded to integer ps to keep xsim arithmetic exact).
    constant C_AXIS_CLK_PERIOD_PS : natural := natural(1000000.0 / G_AXIS_CLK_MHZ);
    constant C_TDC_CLK_PERIOD_PS  : natural := natural(1000000.0 / G_TDC_CLK_MHZ);
    constant C_AXIS_CLK_PERIOD    : time := C_AXIS_CLK_PERIOD_PS * 1 ps;
    constant C_TDC_CLK_PERIOD     : time := C_TDC_CLK_PERIOD_PS * 1 ps;
    -- AXIS is contractually no faster than TDC, so this covers both domains.
    constant C_RST_HOLD           : time := 30 * C_AXIS_CLK_PERIOD;

    -- Range-derived counters. The CSR stores one physical 5 ns reference
    -- value, then each consumer converts it to its own domain clock count.
    constant C_AXIS_DOMAIN_CLK_MHZ : positive := positive(integer(G_AXIS_CLK_MHZ));
    constant C_TDC_DOMAIN_CLK_MHZ  : positive := positive(integer(G_TDC_CLK_MHZ));
    constant C_MAX_RANGE_5NS_TICKS : natural := natural(ceil(
        2.0 * G_MAX_RANGE_M / C_LIGHT_M_PER_US * real(c_RANGE_REF_CLK_MHZ)));
    constant C_MAX_RANGE_AXIS_CLKS : natural :=
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(C_MAX_RANGE_5NS_TICKS, 16), C_AXIS_DOMAIN_CLK_MHZ));
    constant C_MAX_RANGE_TDC_CLKS : natural :=
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(C_MAX_RANGE_5NS_TICKS, 16), C_TDC_DOMAIN_CLK_MHZ));
    constant C_DRAIN_MARGIN_TDC_CLKS : positive := fn_time_ns_to_clks_ceil(
        G_TDC_DRAIN_MARGIN_TIME_NS, C_TDC_DOMAIN_CLK_MHZ);
    -- stop_tdc is an ordering diagnostic, not the GPX capture deadline. Give
    -- synchronized IrFlag time to become visible before Laser closes its
    -- logical window. The GPX MTimer and accepted distance remain unchanged.
    constant C_TDC_CLOSE_MARGIN_5NS_TICKS : natural := 8; -- 40 ns
    constant C_LASER_RANGE_5NS_TICKS : natural :=
        C_MAX_RANGE_5NS_TICKS + C_TDC_CLOSE_MARGIN_5NS_TICKS;
    constant C_SIM_TARGET_5NS_TICKS : natural := natural(ceil(
        2.0 * G_SIM_TARGET_M / C_LIGHT_M_PER_US * real(c_RANGE_REF_CLK_MHZ)));
    constant C_SIM_TARGET_CLKS : natural :=
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(C_SIM_TARGET_5NS_TICKS, 16), C_AXIS_DOMAIN_CLK_MHZ));

    -- Behavioral laser-module response. fire_done is optical T0 and is
    -- independent of the later target Return generated by echo_receiver.
    -- Keep this physical-time contract in the common 5 ns CSR unit so the
    -- 150/200 MHz regressions exercise the same laser response time.
    constant C_LASER_T0_DELAY_5NS_TICKS : positive := 8; -- 40 ns
    constant C_LASER_T0_DELAY_AXIS_CLKS : positive := positive(
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(C_LASER_T0_DELAY_5NS_TICKS, 16),
            C_AXIS_DOMAIN_CLK_MHZ)));

    -- One clock-derived encoder profile is shared by both ownership modes.
    -- The sibling TB package has separate 100 MHz and 150 MHz constants;
    -- reusing either at another clock changes RPM and therefore shot PRF.
    constant C_ENC_TOTAL_CLKS_LOCAL : natural :=
        natural(ceil(G_REV_TIME_US * G_AXIS_CLK_MHZ));
    constant C_MOTOR_RPM_LOCAL : positive :=
        positive(integer(round(60000000.0 / G_REV_TIME_US)));
    constant C_ENC_TICKS_LO_LOCAL : natural :=
        C_ENC_TOTAL_CLKS_LOCAL / C_MD_TOTAL_STATES;
    constant C_ENC_HI_COUNT_LOCAL : natural :=
        C_ENC_TOTAL_CLKS_LOCAL
        - C_ENC_TICKS_LO_LOCAL * C_MD_TOTAL_STATES;
    constant C_ENC_TICKS_HI_LOCAL : natural := C_ENC_TICKS_LO_LOCAL + 1;
    -- Optical reflection doubles mirror motion. Convert the requested optical
    -- interval to a mechanical angle first, then round up to a decoder state.
    constant C_STEP_INTERVAL  : natural :=
        natural(ceil((G_OPTICAL_SHOT_INTERVAL_DEG / 2.0)
                     * real(C_MD_TOTAL_STATES) / 360.0));
    constant C_SHOT_PERIOD_AXIS_CLKS : natural :=
        (C_STEP_INTERVAL * C_ENC_TOTAL_CLKS_LOCAL
         + C_MD_TOTAL_STATES - 1) / C_MD_TOTAL_STATES;
    constant C_MD_COMMIT_TIMEOUT_CLKS : positive :=
        2 * C_ENC_TICKS_HI_LOCAL * C_MD_TOTAL_STATES + 100;
    -- A fixed observation time can end halfway through a face. Allow enough
    -- time for the current face to reach its configured column boundary before
    -- gating new Motor-to-Laser requests, so the smoke produces closed VDMA
    -- lines/frames. Mid-face cancellation belongs in an explicit abort test.
    constant C_FACE_CLOSE_TIMEOUT_CLKS : positive :=
        C_ENC_TOTAL_CLKS_LOCAL +
        4 * C_STEP_INTERVAL * C_ENC_TICKS_HI_LOCAL * G_COLS_PER_FACE + 1000;
    constant C_PIPE_DRAIN_TIMEOUT_CLKS : positive :=
        C_FACE_CLOSE_TIMEOUT_CLKS + 4 * C_MAX_RANGE_AXIS_CLKS;

    -- Distance owns only the acquisition window. Return count is an explicit
    -- scene/stimulus input, while max_hits is the independent Cell capacity.
    constant C_MAX_HITS : natural := G_MAX_HITS_CFG;

    -- The behavioral GPX model emits one slope bit per chip.  If every active
    -- chip emits rising data, disable the runtime falling lane so all present
    -- chips are routed to rising.  Otherwise use the product's static 2+2
    -- capability masks and enable the falling lane.
    function fn_has_active_fall(
        active_mask : std_logic_vector(3 downto 0);
        slope_mask  : std_logic_vector(3 downto 0)
    ) return boolean is
    begin
        return (active_mask and (not slope_mask)) /= "0000";
    end function;

    function fn_bool_to_nat(value : boolean) return natural is
    begin
        if value then return 1; else return 0; end if;
    end function;

    constant C_FALLING_ENABLE : boolean :=
        fn_has_active_fall(G_ACTIVE_CHIP_MASK, G_CHIP_SLOPE_MASK);
    -- This integration smoke keeps the assembler's programmable per-chip
    -- timeout disabled. The acquisition watchdog remains range-bounded in the
    -- TDC domain. A nonzero scan timeout needs a separately budgeted drain and
    -- AXIS service margin, so copying max_range directly would be unsafe.
    constant C_MAX_SCAN_5NS_TICKS : natural := 0;

    -- Laser timing CSRs are fixed 5 ns ticks. The Laser timebase converts them
    -- to the selected AXIS clock and rounds up, preserving minimum pulse width.
    function fn_max_nat(a, b : natural) return natural is
    begin if a > b then return a; else return b; end if; end function;
    constant C_FIRE_WIDTH     : natural := fn_max_nat(C_FIRE_WIDTH_5NS_TICKS, 1);
    -- A zero timeout is a blocking configuration error in the current Laser
    -- contract. This watchdog bounds command-to-T0 response only; it is not the
    -- target round-trip window. Give the fixed behavioral laser response a 2x
    -- margin while keeping the two physical contracts independent.
    constant C_FIRE_DONE_TIMEOUT_5NS_TICKS : natural :=
        2 * C_LASER_T0_DELAY_5NS_TICKS;
    constant C_FIRE_DONE_TIMEOUT_AXIS_CLKS : natural :=
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(C_FIRE_DONE_TIMEOUT_5NS_TICKS, 16),
            C_AXIS_DOMAIN_CLK_MHZ));
    constant C_LASER_RANGE_AXIS_CLKS : natural :=
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(C_LASER_RANGE_5NS_TICKS, 16),
            C_AXIS_DOMAIN_CLK_MHZ));
    constant C_REARM_GUARD_AXIS_CLKS : natural :=
        to_integer(fn_range_5ns_ticks_to_clks(
            to_unsigned(G_REARM_GUARD_5NS_TICKS, 16),
            C_AXIS_DOMAIN_CLK_MHZ));
    -- Registered request, T0-to-range, and guard-to-ready state boundaries.
    constant C_LASER_FSM_OVERHEAD_CLKS : natural := 4;
    constant C_LASER_WORST_REARM_CLKS : natural :=
        C_FIRE_DONE_TIMEOUT_AXIS_CLKS + C_LASER_RANGE_AXIS_CLKS
        + C_REARM_GUARD_AXIS_CLKS + C_LASER_FSM_OVERHEAD_CLKS;
    constant C_CTL1_VAL_LOCAL : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(C_FIRE_DONE_TIMEOUT_5NS_TICKS, 16)) &
        std_logic_vector(to_unsigned(C_FIRE_WIDTH, 16));
    constant C_START_TDC_W    : natural := fn_max_nat(C_START_TDC_5NS_TICKS, 1);
    constant C_STOP_TDC_W     : natural := fn_max_nat(C_STOP_TDC_5NS_TICKS, 1);
    constant C_CTL3_VAL_LOCAL : natural := C_STOP_TDC_W * 65536 + C_START_TDC_W;
    constant C_LC_CTL7_LOCAL : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(G_REARM_GUARD_5NS_TICKS, 16)) & x"0000";

    function fn_lc_ctl0(
        laser_enable : std_logic;
        reset_toggle : std_logic
    ) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := c_CTL0_INIT;
    begin
        v(c_CTL0_LASER_EN)  := laser_enable;
        v(c_CTL0_SW_RST)    := reset_toggle;
        v(c_CTL0_STREAM_EN) := '1';
        return v;
    end function;
    constant C_LC_CTL0_RESET_LOCAL  : std_logic_vector(31 downto 0) :=
        fn_lc_ctl0('0', '1');
    constant C_LC_CTL0_ENABLE_LOCAL : std_logic_vector(31 downto 0) :=
        fn_lc_ctl0('1', '1');

    -- CTL5 packed for laser_ctrl:
    --   [20:16] face_enable = 0x1F (all 5 mirror faces present in pkg),
    --   [15:0]  step_interval = C_STEP_INTERVAL
    function fn_pack_ctl5(face_en : std_logic_vector(4 downto 0);
                          step_interval : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(0, 11)) & face_en
             & std_logic_vector(to_unsigned(step_interval, 16));
    end function;

    function fn_face_mask(n_faces : positive range 1 to 5)
        return std_logic_vector is
        variable v : std_logic_vector(4 downto 0) := (others => '0');
    begin
        for i in 0 to n_faces - 1 loop
            v(i) := '1';
        end loop;
        return v;
    end function;

    function fn_md_ctl7(
        face_index   : natural;
        stage_toggle : std_logic
    ) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(2 downto 0) := std_logic_vector(to_unsigned(face_index, 3));
        v(7) := stage_toggle;
        return v;
    end function;

    function fn_bool_to_sl(value : boolean) return std_logic is
    begin
        if value then return '1'; else return '0'; end if;
    end function;

    function fn_nat_bit_to_sl(value : natural) return std_logic is
    begin
        if value = 0 then return '0'; else return '1'; end if;
    end function;

    function fn_u_sys_ctrl(
        motor_sim, laser_en, echo_sim : std_logic;
        reset_epoch : natural
    ) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_SYS_CTRL_MOTOR_SIM_EN_BIT) := motor_sim;
        v(C_SYS_CTRL_LASER_EN_BIT) := laser_en;
        v(C_SYS_CTRL_LASER_STREAM_EN_BIT) := '1';
        v(C_SYS_CTRL_ECHO_SIM_EN_BIT) := echo_sim;
        v(C_SYS_CTRL_RESET_EPOCH_HI downto C_SYS_CTRL_RESET_EPOCH_LO) :=
            std_logic_vector(to_unsigned(reset_epoch, 8));
        return v;
    end function;

    function fn_u_motor_cfg return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_MOTOR_CFG_CPR_HI downto C_MOTOR_CFG_CPR_LO) :=
            std_logic_vector(to_unsigned(C_MD_CPR, 16));
        v(C_MOTOR_CFG_DIR_BIT) := '0';
        v(C_MOTOR_CFG_DEC_MODE_HI downto C_MOTOR_CFG_DEC_MODE_LO) :=
            std_logic_vector(to_unsigned(C_MD_DEC_MODE, 2));
        v(C_MOTOR_CFG_Z_EARLY_BIT) := '0';
        v(C_MOTOR_CFG_VALID_HOLD_HI downto C_MOTOR_CFG_VALID_HOLD_LO) :=
            std_logic_vector(to_unsigned(C_MD_VALID_HOLD, 8));
        return v;
    end function;

    function fn_u_motor_sched return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_MOTOR_SCHED_HI_COUNT_HI downto C_MOTOR_SCHED_HI_COUNT_LO) :=
            std_logic_vector(to_unsigned(C_ENC_HI_COUNT_LOCAL, 15));
        v(C_MOTOR_SCHED_PHYS_LAT_HI downto C_MOTOR_SCHED_PHYS_LAT_LO) :=
            std_logic_vector(to_unsigned(C_MD_PHYSICAL_LATENCY_CLKS, 6));
        v(C_MOTOR_SCHED_VIRT_LAT_HI downto C_MOTOR_SCHED_VIRT_LAT_LO) :=
            std_logic_vector(to_unsigned(C_MD_VIRTUAL_LATENCY_CLKS, 6));
        return v;
    end function;

    function fn_u_face_geometry(center, half : natural)
        return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_MOTOR_FACE_CENTER_HI downto C_MOTOR_FACE_CENTER_LO) :=
            std_logic_vector(to_unsigned(center, 15));
        v(C_MOTOR_FACE_HALF_HI downto C_MOTOR_FACE_HALF_LO) :=
            std_logic_vector(to_unsigned(half, 15));
        return v;
    end function;

    function fn_u_face_index(
        write_index, read_index, epoch : natural
    ) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_MOTOR_FACE_WRITE_INDEX_HI downto C_MOTOR_FACE_WRITE_INDEX_LO) :=
            std_logic_vector(to_unsigned(write_index, 3));
        v(C_MOTOR_FACE_READ_INDEX_HI downto C_MOTOR_FACE_READ_INDEX_LO) :=
            std_logic_vector(to_unsigned(read_index, 3));
        v(C_MOTOR_FACE_WRITE_EPOCH_HI downto C_MOTOR_FACE_WRITE_EPOCH_LO) :=
            std_logic_vector(to_unsigned(epoch, 8));
        return v;
    end function;

    function fn_u_echo_cmd(
        channel, write_toggle, apply_toggle : natural
    ) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_ECHO_DELAY_INDEX_HI downto C_ECHO_DELAY_INDEX_LO) :=
            std_logic_vector(to_unsigned(channel, 5));
        if write_toggle /= 0 then
            v(C_ECHO_DELAY_WRITE_TOGGLE_BIT) := '1';
        end if;
        if apply_toggle /= 0 then
            v(C_ECHO_DELAY_APPLY_TOGGLE_BIT) := '1';
        end if;
        return v;
    end function;

    function fn_u_tdc_image_cmd(index, epoch : natural)
        return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_TDC_IMAGE_INDEX_HI downto C_TDC_IMAGE_INDEX_LO) :=
            std_logic_vector(to_unsigned(index, 5));
        v(C_TDC_IMAGE_WRITE_EPOCH_HI downto C_TDC_IMAGE_WRITE_EPOCH_LO) :=
            std_logic_vector(to_unsigned(epoch, 8));
        return v;
    end function;

    function fn_u_tdc_aux(opcode, epoch : natural)
        return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(C_TDC_CMD_OPCODE_HI downto C_TDC_CMD_OPCODE_LO) :=
            std_logic_vector(to_unsigned(opcode, 3));
        v(C_TDC_CMD_EPOCH_HI downto C_TDC_CMD_EPOCH_LO) :=
            std_logic_vector(to_unsigned(epoch, 8));
        return v;
    end function;

    constant C_LC_CTL5_DERIVED : std_logic_vector(31 downto 0) :=
        fn_pack_ctl5(fn_face_mask(G_N_FACES), C_STEP_INTERVAL);

    -- tdc_gpx pipeline CSR values (packed from generics)
    --   MAIN_CTRL  [3:0]=chip_mask, [14:12]=reserved, [18:15]=stops
    --   RANGE_COLS [15:0]=max_range_5ns_ticks, [31:16]=cols_per_face
    function fn_pack_main_ctrl(mask  : std_logic_vector(3 downto 0);
                               stops : natural) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v( 3 downto  0) := mask;
        v(18 downto 15) := std_logic_vector(to_unsigned(stops, 4));
        return v;
    end function;
    constant C_MAIN_CTRL_BASE : std_logic_vector(31 downto 0) :=
        fn_pack_main_ctrl(G_ACTIVE_CHIP_MASK, G_STOPS_PER_CHIP);

    constant C_RANGE_COLS_VAL : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(G_COLS_PER_FACE, 16)) &
        std_logic_vector(to_unsigned(C_MAX_RANGE_5NS_TICKS, 16));

    function fn_pack_scan_timeout(
        max_hits        : natural;
        max_scan_ticks  : natural;
        falling_enable  : boolean
    ) return std_logic_vector is
        variable v : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v(c_ST_MAX_SCAN_HI downto c_ST_MAX_SCAN_LO) :=
            std_logic_vector(to_unsigned(
                max_scan_ticks, c_ST_MAX_SCAN_HI - c_ST_MAX_SCAN_LO + 1));
        v(c_ST_MAX_HITS_HI downto c_ST_MAX_HITS_LO) :=
            std_logic_vector(to_unsigned(
                max_hits, c_ST_MAX_HITS_HI - c_ST_MAX_HITS_LO + 1));
        if falling_enable then
            v(c_ST_FALLING_ENABLE) := '1';
        end if;
        return v;
    end function;

    function fn_runtime_slope_mask(
        active_mask   : std_logic_vector(3 downto 0);
        edge_mask     : std_logic_vector(3 downto 0);
        fall_enable   : boolean;
        rise_lane     : boolean
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

    function fn_expected_lane_hsize(
        chip_mask : std_logic_vector(3 downto 0)
    ) return natural is
        variable v_cells : natural;
    begin
        v_cells := fn_count_ones(chip_mask) * G_STOPS_PER_CHIP;
        if v_cells = 0 then
            return 0;
        end if;
        return fn_vdma_line_bytes(v_cells, C_MAX_HITS);
    end function;

    constant C_SCAN_TIMEOUT_VAL : std_logic_vector(31 downto 0) :=
        fn_pack_scan_timeout(
            C_MAX_HITS, C_MAX_SCAN_5NS_TICKS, C_FALLING_ENABLE);
    constant C_RISE_ACTIVE_MASK : std_logic_vector(3 downto 0) :=
        fn_runtime_slope_mask(
            G_ACTIVE_CHIP_MASK, c_DEFAULT_RISE_CHIP_MASK,
            C_FALLING_ENABLE, true);
    constant C_FALL_ACTIVE_MASK : std_logic_vector(3 downto 0) :=
        fn_runtime_slope_mask(
            G_ACTIVE_CHIP_MASK, c_DEFAULT_FALL_CHIP_MASK,
            C_FALLING_ENABLE, false);
    constant C_EXPECT_HSIZE_RISE : natural :=
        fn_expected_lane_hsize(C_RISE_ACTIVE_MASK);
    constant C_EXPECT_HSIZE_FALL : natural :=
        fn_expected_lane_hsize(C_FALL_ACTIVE_MASK);
    constant C_U_TDC_MAX_ROWS_WORD : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(c_MAX_ROWS_PER_FACE, 32));
    constant C_U_TDC_CELL_SIZE_WORD : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(c_CANONICAL_CELL_BYTES_MAX, 32));
    constant C_U_TDC_MAX_HSIZE_WORD : std_logic_vector(31 downto 0) :=
        std_logic_vector(to_unsigned(c_VDMA_LINE_BYTES_MAX, 32));

    -- Output and Echo diagnostic stream widths.
    constant C_OUTPUT_W   : natural := G_TDATA_WIDTH;
    constant C_KEEP_W     : natural := fn_axis_keep_width(C_OUTPUT_W);
    constant C_WORDS_PER_BEAT : positive := C_OUTPUT_W / 32;
    constant C_HEADER_WORD_INDEX : natural := 3;
    constant C_MON_SHOT_DEPTH : positive := 64;
    type t_face_index_queue is array (0 to C_MON_SHOT_DEPTH - 1) of
        std_logic_vector(2 downto 0);
    type t_cycle_queue is array (0 to C_MON_SHOT_DEPTH - 1) of natural;
    type t_hit_queue is array (0 to C_MON_SHOT_DEPTH - 1) of
        unsigned(c_RAW_HIT_WIDTH - 1 downto 0);
    constant C_HIT_WORDS_PER_CELL : positive := fn_ceil_div(C_MAX_HITS, 2);
    constant C_CELL_WORDS         : positive := C_HIT_WORDS_PER_CELL + 1;
    constant C_FIRST_CELL_HIT_WORD_INDEX : natural :=
        c_HDR_PREFIX_BYTES / 4;
    constant C_FIRST_CELL_META_WORD_INDEX : natural :=
        C_FIRST_CELL_HIT_WORD_INDEX + C_HIT_WORDS_PER_CELL;
    constant C_STOP_DW    : natural := 32;
    constant C_STOP_CNT_WIDTH : natural := 4;

    -- Echo-receiver geometry for this integration profile.
    constant C_ER_N_STOPS : natural := c_MAX_STOPS_PER_CHIP;
    constant C_PD_WIDTH   : natural := c_MAX_CHIPS * C_ER_N_STOPS;
    constant C_EXPECT_REF_DEPTH : positive :=
        C_MON_SHOT_DEPTH * c_MAX_HITS_PER_STOP;
    type t_hit_reference_array is array (0 to C_EXPECT_REF_DEPTH - 1) of
        unsigned(c_RAW_HIT_WIDTH - 1 downto 0);
    type t_channel_count_array is array (0 to C_PD_WIDTH - 1) of natural;
    type t_chip_count_array is array (0 to c_MAX_CHIPS - 1) of natural;

    function fn_hit_ref_index(
        shot_idx : natural;
        channel  : natural;
        hit_idx  : natural
    ) return natural is
    begin
        -- Every integration profile drives the same Return time to all active
        -- channels. One canonical pin timestamp is therefore sufficient; the
        -- raw bus and every Cell still compare their own words against it.
        return shot_idx * c_MAX_HITS_PER_STOP + hit_idx;
    end function;

    function fn_expected_returns(channel : natural) return natural is
    begin
        if G_ECHO_STIM_MODE = "physical_multi" then
            return G_RETURNS_PER_STOP;
        elsif channel mod C_ER_N_STOPS = 0 then
            return 1;
        else
            return 0;
        end if;
    end function;

    function fn_lane_channel(
        chip_mask : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
        cell_idx  : natural
    ) return natural is
        variable v_cell_base : natural := 0;
    begin
        for chip in 0 to c_MAX_CHIPS - 1 loop
            if chip_mask(chip) = '1' then
                if cell_idx < v_cell_base + G_STOPS_PER_CHIP then
                    return chip * C_ER_N_STOPS + (cell_idx - v_cell_base);
                end if;
                v_cell_base := v_cell_base + G_STOPS_PER_CHIP;
            end if;
        end loop;
        assert false
            report "full_int: lane cell index exceeds active chip geometry"
            severity failure;
        return 0;
    end function;

    function fn_active_return_words_per_shot return natural is
        variable v_total : natural := 0;
    begin
        for chip in 0 to c_MAX_CHIPS - 1 loop
            if G_ACTIVE_CHIP_MASK(chip) = '1' then
                for stop_id in 0 to G_STOPS_PER_CHIP - 1 loop
                    v_total := v_total + fn_expected_returns(
                        chip * C_ER_N_STOPS + stop_id);
                end loop;
            end if;
        end loop;
        return v_total;
    end function;

    function fn_lane_return_words_per_shot(
        chip_mask : std_logic_vector(c_MAX_CHIPS - 1 downto 0)
    ) return natural is
        variable v_total : natural := 0;
    begin
        for chip in 0 to c_MAX_CHIPS - 1 loop
            if chip_mask(chip) = '1' then
                for stop_id in 0 to G_STOPS_PER_CHIP - 1 loop
                    v_total := v_total + fn_expected_returns(
                        chip * C_ER_N_STOPS + stop_id);
                end loop;
            end if;
        end loop;
        return v_total;
    end function;

    constant C_EXPECT_RAW_WORDS_PER_SHOT : natural :=
        fn_active_return_words_per_shot;
    constant C_RISE_CELL_COUNT : natural :=
        fn_count_ones(C_RISE_ACTIVE_MASK) * G_STOPS_PER_CHIP;
    constant C_FALL_CELL_COUNT : natural :=
        fn_count_ones(C_FALL_ACTIVE_MASK) * G_STOPS_PER_CHIP;
    constant C_EXPECT_RISE_HITS_PER_SHOT : natural :=
        fn_lane_return_words_per_shot(C_RISE_ACTIVE_MASK);
    constant C_EXPECT_FALL_HITS_PER_SHOT : natural :=
        fn_lane_return_words_per_shot(C_FALL_ACTIVE_MASK);

    -- External GPX model calibration. The DUT receives the same value in its
    -- Face header, but never performs the START-to-STOP calculation itself.
    constant C_BIN_RESOLUTION_PS : positive := 81;
    constant C_SIM_TARGET_DELAY : time := C_SIM_TARGET_5NS_TICKS * 5 ns;
    constant C_ECHO_PULSE_WIDTH : time := 2 * C_AXIS_CLK_PERIOD;

    -- Encoder run wall-time (derived from generic)
    constant C_ENC_RUN_CLKS : natural :=
        natural(G_ENC_RUN_US * G_AXIS_CLK_MHZ);

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    signal clk       : std_logic := '0';  -- AXIS / upstream integration clock
    signal tdc_clk   : std_logic := '0';
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
    signal ml_irq     : std_logic;
    signal md_dbg_virt_pos   : std_logic_vector(14 downto 0);
    signal md_dbg_dec_count  : std_logic_vector(14 downto 0);
    signal md_dbg_active     : std_logic_vector(4 downto 0);
    signal md_dbg_active_any : std_logic;
    signal md_dbg_face_index : std_logic_vector(2 downto 0);
    signal md_n_faces        : std_logic_vector(2 downto 0);
    signal md_dbg_sim_en     : std_logic;
    signal md_dbg_rst_n_int  : std_logic;
    signal md_dbg_cfg_busy   : std_logic;

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
    -- laser_ctrl key outputs
    signal lc_fire_pulse   : std_logic;
    signal lc_start_tdc    : std_logic;
    signal lc_shot_start   : std_logic;
    signal lc_shot_face_index : std_logic_vector(2 downto 0);
    signal lc_stop_tdc     : std_logic;

    -- TB-generated direct-stim pulses (only used when G_TDC_STIM_MODE="direct")
    signal tb_shot_start   : std_logic := '0';
    signal tb_stop_tdc     : std_logic := '0';
    signal td_shot_start_mux : std_logic;
    signal td_shot_face_index_mux : std_logic_vector(2 downto 0);
    signal td_stop_tdc_mux   : std_logic;
    signal lc_laser_active : std_logic;
    signal lc_warning      : std_logic_vector(4 downto 0);
    signal lc_warning_any  : std_logic;
    signal lc_m_tvalid     : std_logic;
    signal lc_m_tdata      : std_logic_vector(31 downto 0);
    signal lc_m_tuser      : std_logic_vector(c_RES_TUSER_WIDTH - 1 downto 0);
    signal lc_m_tlast      : std_logic;
    signal lc_m_tready     : std_logic;
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

    -- echo_receiver PD input + stop_evt/fire_count output
    signal pd_p : std_logic_vector(C_PD_WIDTH - 1 downto 0) := (others => '0');
    signal pd_n : std_logic_vector(C_PD_WIDTH - 1 downto 0) := (others => '1');
    signal er_tdc_stop : std_logic_vector(C_PD_WIDTH - 1 downto 0);
    signal er_stop_tvalid : std_logic;
    signal er_stop_tdata  : std_logic_vector(C_STOP_DW - 1 downto 0);
    signal er_stop_tkeep  : std_logic_vector(C_STOP_DW/8 - 1 downto 0);
    signal er_stop_tuser  : std_logic_vector(C_STOP_DW - 1 downto 0);
    signal er_fire_count_tvalid : std_logic;
    signal er_fire_count_tdata  : std_logic_vector(31 downto 0);
    signal er_fire_count_tkeep  : std_logic_vector(3 downto 0);
    signal er_fire_count_tlast  : std_logic;

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
    -- Optional single-owner unified AXI-Lite CSR (9-bit, 32 CTL / 32 STAT)
    -- =========================================================================
    signal uc_awaddr  : std_logic_vector(8 downto 0) := (others => '0');
    signal uc_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal uc_awvalid : std_logic := '0';
    signal uc_awready : std_logic;
    signal uc_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal uc_wstrb   : std_logic_vector(3 downto 0) := "1111";
    signal uc_wvalid  : std_logic := '0';
    signal uc_wready  : std_logic;
    signal uc_bvalid  : std_logic;
    signal uc_bready  : std_logic := '0';
    signal uc_bresp   : std_logic_vector(1 downto 0);
    signal uc_araddr  : std_logic_vector(8 downto 0) := (others => '0');
    signal uc_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal uc_arvalid : std_logic := '0';
    signal uc_arready : std_logic;
    signal uc_rvalid  : std_logic;
    signal uc_rready  : std_logic := '0';
    signal uc_rdata   : std_logic_vector(31 downto 0);
    signal uc_rresp   : std_logic_vector(1 downto 0);
    signal uc_irq     : std_logic;
    signal uc_ctl     : t_uword_array(0 to 25) := (others => (others => '0'));
    signal uc_stat    : t_uword_array(3 to 30) := (others => (others => '0'));

    signal uc_motor_irq_cause : std_logic_vector(3 downto 0);
    signal uc_laser_cfg_epoch, uc_laser_reset_epoch :
        std_logic_vector(7 downto 0);
    signal uc_laser_cfg_busy, uc_laser_cfg_reject, uc_laser_cfg_valid :
        std_logic;
    signal uc_laser_irq_cause : std_logic_vector(2 downto 0);
    signal uc_echo_reset_epoch : std_logic_vector(7 downto 0);
    signal uc_echo_irq_cause : std_logic_vector(4 downto 0);
    signal uc_tdc_cfg_epoch, uc_tdc_reset_epoch, uc_tdc_cmd_epoch,
           uc_tdc_image_epoch : std_logic_vector(7 downto 0);
    signal uc_tdc_cfg_busy, uc_tdc_cfg_reject, uc_tdc_cfg_valid,
           uc_tdc_cmd_busy, uc_tdc_cmd_reject, uc_tdc_image_reject :
        std_logic;
    signal uc_tdc_irq_cause : std_logic_vector(6 downto 0);

    -- =========================================================================
    -- TDC-GPX top physical pins (driven by 4-chip behavioral model)
    -- =========================================================================
    signal io_tdc_d         : std_logic_vector(c_MAX_CHIPS * c_TDC_BUS_WIDTH - 1 downto 0);
    signal o_tdc_adr        : std_logic_vector(c_MAX_CHIPS * c_TDC_ADR_WIDTH - 1 downto 0);
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
    signal i_tdc_irflag     : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');  -- driven by per-chip p_chip
    signal i_tdc_errflag    : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');

    -- VDMA sinks
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

    -- Calibration
    signal i_bin_res_ps : unsigned(15 downto 0) :=
        to_unsigned(C_BIN_RESOLUTION_PS, 16);
    signal i_k_dist     : unsigned(31 downto 0) := to_unsigned(54321, 32);
    signal td_irq, td_irq_pipe : std_logic;
    signal td_vdma_hsize_rise : unsigned(15 downto 0);
    signal td_vdma_hsize_fall : unsigned(15 downto 0);
    signal td_vdma_vsize      : unsigned(15 downto 0);

    -- External TDC-GPX behavioral-chip observability.
    signal gpx_start_tdc_mux : std_logic;
    signal gpx_last_hit : std_logic_vector(
        c_MAX_CHIPS * c_RAW_HIT_WIDTH - 1 downto 0);
    signal gpx_capture_count : std_logic_vector(
        c_MAX_CHIPS * 16 - 1 downto 0);

    -- =========================================================================
    -- DUT internal observability NOTE
    --   VHDL-2008 external names (<<signal .x.y.z : type>>) were attempted
    --   to mirror tdc_gpx_top internals like s_cmd_start_accepted, s_chip_busy,
    --   s_shot_start_gated, but xsim 2025.2.1 rejects them with
    --   "only elaborated instance can be referenced in an external name".
    --   Waveform inspection via --debug typical + xsim gui is the intended
    --   follow-up path. Stubs kept so the monitor process can compile.
    -- =========================================================================
    signal dbg_cmd_start        : std_logic := '0';
    signal dbg_cmd_start_accept : std_logic := '0';
    signal dbg_chip_busy        : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal dbg_shot_gated       : std_logic := '0';
    signal dbg_shot_per_chip    : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal dbg_cfg_rejected_r   : std_logic := '0';
    signal dbg_pipeline_abort   : std_logic := '0';

    -- =========================================================================
    -- Activity counters (per handoff point)
    -- =========================================================================
    signal mon_cmd_start_cnt   : natural := 0;
    signal mon_cmd_accept_cnt  : natural := 0;
    signal mon_chip_busy_any   : natural := 0;   -- cycles with any chip busy
    signal mon_shot_gated_cnt  : natural := 0;
    signal mon_shot_chip_any   : natural := 0;
    signal mon_cfg_rejected    : natural := 0;
    signal mon_pipe_abort      : natural := 0;

    -- Sticky "have I ever been high" latches so slow-poll probes catch short pulses
    signal dbg_lc_start_ever : std_logic := '0';
    signal dbg_lc_fire_ever  : std_logic := '0';


    signal mon_md_virt_pos_changes : natural := 0;
    signal mon_md_dec_count_changes : natural := 0;
    signal mon_md_active_cycles     : natural := 0;
    signal mon_md_cfg_busy_cycles   : natural := 0;
    signal mon_lc_fire_cnt  : natural := 0;
    signal mon_lc_start_cnt : natural := 0;
    signal mon_lc_stop_cnt  : natural := 0;
    signal mon_lc_m_beats   : natural := 0;
    signal mon_lc_m_tlast   : natural := 0;
    signal mon_er_stop_beats : natural := 0;
    signal mon_er_fire_count_beats : natural := 0;
    signal mon_er_stop_high_cycles : natural := 0;
    signal mon_td_rise_beats : natural := 0;
    signal mon_td_fall_beats : natural := 0;
    signal mon_td_rise_line_end : natural := 0;
    signal mon_td_fall_line_end : natural := 0;
    signal mon_td_rise_header_checks : natural := 0;
    signal mon_td_fall_header_checks : natural := 0;
    signal mon_td_rise_hit_checks : natural := 0;
    signal mon_td_fall_hit_checks : natural := 0;
    signal mon_i_mode_bus_checks : natural := 0;
    signal mon_expected_shots : natural range 0 to C_MON_SHOT_DEPTH := 0;
    signal mon_expected_hit_refs : t_hit_reference_array :=
        (others => (others => '0'));
    signal mon_td_rise_face_seen : std_logic_vector(4 downto 0) := (others => '0');
    signal mon_td_fall_face_seen : std_logic_vector(4 downto 0) := (others => '0');
    signal mon_shot_interval_min_clks : natural := 0;
    signal mon_shot_interval_max_clks : natural := 0;
    signal mon_fire_done_latency_min_clks : natural := 0;
    signal mon_fire_done_latency_max_clks : natural := 0;
    signal mon_range_wait_min_clks : natural := 0;
    signal mon_range_wait_max_clks : natural := 0;
    signal mon_shot_to_rise_tlast_min_clks : natural := 0;
    signal mon_shot_to_rise_tlast_max_clks : natural := 0;
    signal mon_shot_to_fall_tlast_min_clks : natural := 0;
    signal mon_shot_to_fall_tlast_max_clks : natural := 0;
    signal mon_schedule_overrun : natural := 0;
    signal mon_last_expected_hit : unsigned(c_RAW_HIT_WIDTH - 1 downto 0) :=
        (others => '0');
    signal mon_last_rise_hit : unsigned(c_RAW_HIT_WIDTH - 1 downto 0) :=
        (others => '0');
    signal mon_last_fall_hit : unsigned(c_RAW_HIT_WIDTH - 1 downto 0) :=
        (others => '0');

    -- pipeline CSR addresses (C_MAIN_CTRL_BASE / C_RANGE_COLS_VAL are now
    -- declared in the derived-constants block above from the generics).
    constant C_PIPE_MAIN_CTRL  : std_logic_vector(6 downto 0) := "0000000";  -- 0x00
    constant C_PIPE_RANGE_COLS : std_logic_vector(6 downto 0) := "0000100";  -- 0x04
    constant C_PIPE_HW_CONFIG  : std_logic_vector(6 downto 0) := "1000100";  -- 0x44
    constant C_EXPECT_STAT5 : std_logic_vector(31 downto 0) := x"00000001";
    constant C_EXPECT_STAT6 : std_logic_vector(31 downto 0) :=
        G_ACTIVE_CHIP_MASK & x"0000000";
    constant C_EXPECT_STAT7 : std_logic_vector(31 downto 0) := x"00000000";

    -- laser_ctrl minimal config addresses (duplicate of laser_ctrl_cfg_pkg.c_ADDR_* but kept
    -- local to keep the TB self-contained)
    constant C_TB_LC_CTL0  : std_logic_vector(6 downto 0) := "0000000";  -- 0x00
    constant C_TB_LC_CTL1  : std_logic_vector(6 downto 0) := "0000100";  -- 0x04 fire width
    constant C_TB_LC_CTL3  : std_logic_vector(6 downto 0) := "0001100";  -- 0x0C tdc widths
    constant C_TB_LC_CTL5  : std_logic_vector(6 downto 0) := "0010100";  -- 0x14 sched cfg0
    constant C_TB_LC_CTL6  : std_logic_vector(6 downto 0) := "0011000";  -- 0x18 sched cfg1
    constant C_TB_LC_CTL7  : std_logic_vector(6 downto 0) := "0011100";  -- 0x1C sched cfg2
    -- motor_decoder / echo_receiver local CSR addresses. Echo Receiver uses
    -- the compact indexed delay-profile contract introduced in CSR Stage 2.
    constant C_MD_CTL0  : std_logic_vector(6 downto 0) := "0000000";
    constant C_MD_STAT0 : std_logic_vector(6 downto 0) := "0100000";  -- 0x20
    constant C_ER_CTL0  : std_logic_vector(8 downto 0) := "0" & x"00";
    constant C_ER_CTL1  : std_logic_vector(8 downto 0) := "0" & x"04";
    constant C_ER_STAT2 : std_logic_vector(8 downto 0) := "0" & x"10";
    constant C_ER_SIM_EN_BIT             : natural := 0;
    constant C_ER_DELAY_INDEX_LO         : natural := 4;
    constant C_ER_DELAY_INDEX_HI         : natural := 8;
    constant C_ER_DELAY_WRITE_TOGGLE_BIT : natural := 9;
    constant C_ER_APPLY_TOGGLE_BIT       : natural := 10;
    constant C_ER_WRITE_ACK_BIT          : natural := 9;
    constant C_ER_APPLY_ACK_BIT          : natural := 10;
    constant C_ER_APPLY_PENDING_BIT      : natural := 11;
    constant C_ER_CMD_REJECT_BIT         : natural := 12;

begin
    assert G_CONTROL_MODE = "local" or G_CONTROL_MODE = "unified"
        report "G_CONTROL_MODE must be local or unified"
        severity failure;

    assert G_ENCODER_SOURCE = "internal" or G_ENCODER_SOURCE = "external"
        report "tb_tdc_gpx_full_int: G_ENCODER_SOURCE must be internal or external"
        severity failure;

    assert G_TDC_STIM_MODE = "lc" or G_TDC_STIM_MODE = "direct"
        report "tb_tdc_gpx_full_int: G_TDC_STIM_MODE must be lc or direct"
        severity failure;

    assert G_ECHO_STIM_MODE = "synthetic_single"
        or G_ECHO_STIM_MODE = "physical_multi"
        report "tb_tdc_gpx_full_int: G_ECHO_STIM_MODE must be synthetic_single or physical_multi"
        severity failure;

    assert G_RETURNS_PER_STOP <= G_MAX_HITS_CFG
        report "tb_tdc_gpx_full_int: actual returns exceed configured max_hits capacity"
        severity failure;

    assert G_REV_TIME_US > 0.0 and C_ENC_TOTAL_CLKS_LOCAL >= C_MD_TOTAL_STATES
        report "tb_tdc_gpx_full_int: revolution period is too short for one clock per encoder state"
        severity failure;

    assert C_OPERATING_MOTOR_RPM > 0.0
        and C_HORIZONTAL_RESOLUTION_DEG > 0.0
        report "tb_tdc_gpx_full_int: operating RPM and horizontal resolution must be positive"
        severity failure;

    assert abs(C_OPERATING_POINT_INTERVAL_US
               - real(C_SHOT_PERIOD_AXIS_CLKS) / G_AXIS_CLK_MHZ)
           <= 1.0 / G_AXIS_CLK_MHZ + 1.0E-9
        report "tb_tdc_gpx_full_int: accelerated encoder profile does not represent the requested operating point interval"
        severity failure;

    assert G_OPTICAL_SHOT_INTERVAL_DEG > 0.0
        and G_OPTICAL_SHOT_INTERVAL_DEG <= 360.0
        and C_STEP_INTERVAL >= 1 and C_STEP_INTERVAL <= 65535
        report "tb_tdc_gpx_full_int: optical shot interval is outside the supported state range"
        severity failure;

    assert G_SIM_TARGET_M <= G_MAX_RANGE_M
        report "tb_tdc_gpx_full_int: synthetic target exceeds the configured maximum range"
        severity failure;

    assert G_ECHO_STIM_MODE /= "physical_multi"
        or C_SIM_TARGET_DELAY / G_RETURNS_PER_STOP > C_ECHO_PULSE_WIDTH
        report "tb_tdc_gpx_full_int: Return spacing is not wider than the LVDS pulse"
        severity failure;

    assert G_ECHO_STIM_MODE /= "physical_multi"
        or G_STOPS_PER_CHIP = C_ER_N_STOPS
        report "tb_tdc_gpx_full_int: physical_multi requires all eight STOP channels per chip"
        severity failure;

    assert G_AXIS_CLK_MHZ = real(C_AXIS_DOMAIN_CLK_MHZ)
        report "tb_tdc_gpx_full_int: G_AXIS_CLK_MHZ must be an integer MHz value"
        severity failure;

    assert G_TDC_CLK_MHZ = real(C_TDC_DOMAIN_CLK_MHZ)
        report "tb_tdc_gpx_full_int: G_TDC_CLK_MHZ must be an integer MHz value"
        severity failure;

    assert G_AXIS_CLK_MHZ <= G_TDC_CLK_MHZ
        report "tb_tdc_gpx_full_int: AXIS clock must not exceed TDC clock"
        severity failure;

    assert C_SIM_TARGET_5NS_TICKS <= 65535
        report "tb_tdc_gpx_full_int: Echo delay exceeds 16-bit CSR field"
        severity failure;

    assert C_LASER_T0_DELAY_5NS_TICKS > C_FIRE_WIDTH
        report "tb_tdc_gpx_full_int: laser T0 delay must exceed fire pulse width"
        severity failure;

    assert C_LASER_T0_DELAY_5NS_TICKS < C_FIRE_DONE_TIMEOUT_5NS_TICKS
        report "tb_tdc_gpx_full_int: fire_done timeout must exceed modeled T0 delay"
        severity failure;

    assert C_FIRE_DONE_TIMEOUT_5NS_TICKS <= C_LASER_RANGE_5NS_TICKS
        report "tb_tdc_gpx_full_int: fire_done timeout must not exceed target round-trip window"
        severity failure;

    assert C_SHOT_PERIOD_AXIS_CLKS >= C_LASER_WORST_REARM_CLKS
        report "tb_tdc_gpx_full_int: angular interval is shorter than fire_done timeout + range + guard"
        severity failure;

    -- =========================================================================
    -- Clock / Reset
    -- =========================================================================
    clk <= not clk after C_AXIS_CLK_PERIOD / 2 when not sim_done else '0';
    tdc_clk <= not tdc_clk after C_TDC_CLK_PERIOD / 2 when not sim_done else '0';

    uc_stat(C_STAT_TDC_MAX_ROWS)  <= C_U_TDC_MAX_ROWS_WORD;
    uc_stat(C_STAT_TDC_CELL_SIZE) <= C_U_TDC_CELL_SIZE_WORD;
    uc_stat(C_STAT_TDC_MAX_HSIZE) <= C_U_TDC_MAX_HSIZE_WORD;

    gen_unified_csr : if not C_USE_LOCAL_CSR generate
        u_unified_csr : entity work.lidar_unified_csr_top
            port map (
                s_axi_csr_aclk => clk, s_axi_csr_aresetn => rst_n,
                s_axi_csr_awaddr => uc_awaddr, s_axi_csr_awprot => uc_awprot,
                s_axi_csr_awvalid => uc_awvalid, s_axi_csr_awready => uc_awready,
                s_axi_csr_wdata => uc_wdata, s_axi_csr_wstrb => uc_wstrb,
                s_axi_csr_wvalid => uc_wvalid, s_axi_csr_wready => uc_wready,
                s_axi_csr_bresp => uc_bresp, s_axi_csr_bvalid => uc_bvalid,
                s_axi_csr_bready => uc_bready,
                s_axi_csr_araddr => uc_araddr, s_axi_csr_arprot => uc_arprot,
                s_axi_csr_arvalid => uc_arvalid, s_axi_csr_arready => uc_arready,
                s_axi_csr_rdata => uc_rdata, s_axi_csr_rresp => uc_rresp,
                s_axi_csr_rvalid => uc_rvalid, s_axi_csr_rready => uc_rready,
                o_irq => uc_irq,

                o_sys_ctrl => uc_ctl(C_CTL_SYS_CTRL),
                o_sys_cfg_apply => uc_ctl(C_CTL_SYS_CFG_APPLY),
                o_motor_cfg => uc_ctl(C_CTL_MOTOR_CFG),
                o_motor_ticks_lo => uc_ctl(C_CTL_MOTOR_TICKS_LO),
                o_motor_sched_latency => uc_ctl(C_CTL_MOTOR_SCHED_LATENCY),
                o_motor_z_param => uc_ctl(C_CTL_MOTOR_Z_PARAM),
                o_motor_face_index => uc_ctl(C_CTL_MOTOR_FACE_INDEX),
                o_motor_face_geometry => uc_ctl(C_CTL_MOTOR_FACE_GEOMETRY),
                o_laser_fire_cfg => uc_ctl(C_CTL_LASER_FIRE_CFG),
                o_laser_roundtrip => uc_ctl(C_CTL_LASER_ROUNDTRIP),
                o_laser_tdc_width => uc_ctl(C_CTL_LASER_TDC_WIDTH),
                o_laser_sim_delay => uc_ctl(C_CTL_LASER_SIM_DELAY),
                o_laser_sched0 => uc_ctl(C_CTL_LASER_SCHED0),
                o_laser_sched1 => uc_ctl(C_CTL_LASER_SCHED1),
                o_laser_sched2 => uc_ctl(C_CTL_LASER_SCHED2),
                o_echo_delay_cmd => uc_ctl(C_CTL_ECHO_DELAY_CMD),
                o_echo_delay_data => uc_ctl(C_CTL_ECHO_DELAY_DATA),
                o_tdc_bus_timing => uc_ctl(C_CTL_TDC_BUS_TIMING),
                o_tdc_start_offset => uc_ctl(C_CTL_TDC_START_OFFSET),
                o_tdc_cfg_reg7 => uc_ctl(C_CTL_TDC_CFG_REG7),
                o_tdc_image_cmd => uc_ctl(C_CTL_TDC_IMAGE_CMD),
                o_tdc_image_data => uc_ctl(C_CTL_TDC_IMAGE_DATA),
                o_tdc_scan_cfg => uc_ctl(C_CTL_TDC_SCAN_CFG),
                o_tdc_pipeline_main => uc_ctl(C_CTL_TDC_PIPELINE_MAIN),
                o_tdc_range_cols => uc_ctl(C_CTL_TDC_RANGE_COLS),
                o_tdc_aux_cmd => uc_ctl(C_CTL_TDC_AUX_CMD),

                i_tdc_max_rows => uc_stat(C_STAT_TDC_MAX_ROWS),
                i_tdc_cell_size => uc_stat(C_STAT_TDC_CELL_SIZE),
                i_tdc_max_hsize => uc_stat(C_STAT_TDC_MAX_HSIZE),
                i_motor_status => uc_stat(C_STAT_MOTOR_STATUS),
                i_motor_face_geometry => uc_stat(C_STAT_MOTOR_FACE_GEOMETRY),
                i_motor_cfg_status => uc_stat(C_STAT_MOTOR_CFG_STATUS),
                i_motor_quad_invalid => uc_stat(C_STAT_MOTOR_QUAD_INVALID),
                i_motor_axis_drop => uc_stat(C_STAT_MOTOR_AXIS_DROP),
                i_motor_rev_period => uc_stat(C_STAT_MOTOR_REV_PERIOD),
                i_motor_irq_cause => uc_motor_irq_cause,
                i_laser_status => uc_stat(C_STAT_LASER_STATUS),
                i_laser_encoder_to_fire => uc_stat(C_STAT_LASER_ENCODER_TO_FIRE),
                i_laser_fire_done => uc_stat(C_STAT_LASER_FIRE_DONE),
                i_laser_fire_to_tdc => uc_stat(C_STAT_LASER_FIRE_TO_TDC),
                i_laser_metric_flags => uc_stat(C_STAT_LASER_METRIC_FLAGS),
                i_laser_frame_count => uc_stat(C_STAT_LASER_FRAME_COUNT),
                i_laser_timeout_count => uc_stat(C_STAT_LASER_TIMEOUT_COUNT),
                i_laser_cfg_epoch_accepted => uc_laser_cfg_epoch,
                i_laser_reset_epoch_accepted => uc_laser_reset_epoch,
                i_laser_cfg_busy => uc_laser_cfg_busy,
                i_laser_cfg_reject => uc_laser_cfg_reject,
                i_laser_cfg_valid => uc_laser_cfg_valid,
                i_laser_irq_cause => uc_laser_irq_cause,
                i_echo_rise_mask => uc_stat(C_STAT_ECHO_RISE_MASK),
                i_echo_fall_mask => uc_stat(C_STAT_ECHO_FALL_MASK),
                i_echo_status => uc_stat(C_STAT_ECHO_STATUS),
                i_echo_delay_readback => uc_stat(C_STAT_ECHO_DELAY_READBACK),
                i_echo_reset_epoch_accepted => uc_echo_reset_epoch,
                i_echo_irq_cause => uc_echo_irq_cause(1 downto 0),
                i_tdc_chip0_result => uc_stat(C_STAT_TDC_CHIP0_RESULT),
                i_tdc_chip1_result => uc_stat(C_STAT_TDC_CHIP1_RESULT),
                i_tdc_chip2_result => uc_stat(C_STAT_TDC_CHIP2_RESULT),
                i_tdc_chip3_result => uc_stat(C_STAT_TDC_CHIP3_RESULT),
                i_tdc_pipeline_status => uc_stat(C_STAT_TDC_PIPELINE_STATUS),
                i_tdc_status_ext => uc_stat(C_STAT_TDC_STATUS_EXT),
                i_tdc_status_ext2 => uc_stat(C_STAT_TDC_STATUS_EXT2),
                i_tdc_image_selected_data => uc_stat(C_STAT_TDC_IMAGE_SELECTED_DATA),
                i_tdc_cfg_epoch_accepted => uc_tdc_cfg_epoch,
                i_tdc_reset_epoch_accepted => uc_tdc_reset_epoch,
                i_tdc_cfg_busy => uc_tdc_cfg_busy,
                i_tdc_cfg_reject => uc_tdc_cfg_reject,
                i_tdc_cfg_valid => uc_tdc_cfg_valid,
                i_tdc_cmd_busy => uc_tdc_cmd_busy,
                i_tdc_command_reject => uc_tdc_cmd_reject,
                i_tdc_image_reject => uc_tdc_image_reject,
                i_tdc_irq_cause => uc_tdc_irq_cause
            );
    end generate;

    gen_no_unified_csr : if C_USE_LOCAL_CSR generate
        uc_awready <= '0'; uc_wready <= '0';
        uc_bvalid <= '0'; uc_bresp <= "10";
        uc_arready <= '0'; uc_rvalid <= '0';
        uc_rdata <= (others => '0'); uc_rresp <= "10";
        uc_irq <= '0';
        uc_ctl <= (others => (others => '0'));
    end generate;

    -- Physical laser-driver model. A real fire_done marks optical T0; it is
    -- deliberately independent from the later photodiode target return.
    p_fire_done_driver : process(clk)
        variable v_fire_d1 : std_logic := '0';
        variable v_waiting : boolean := false;
        variable v_count   : natural := 0;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                lc_fire_done <= '0';
                v_fire_d1 := '0';
                v_waiting := false;
                v_count := 0;
            else
                lc_fire_done <= '0';
                if lc_fire_pulse = '1' and v_fire_d1 = '0' then
                    v_waiting := true;
                    v_count := C_LASER_T0_DELAY_AXIS_CLKS;
                end if;
                v_fire_d1 := lc_fire_pulse;

                if v_waiting then
                    if v_count = 0 then
                        lc_fire_done <= '1';
                        v_waiting := false;
                    else
                        v_count := v_count - 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_fire_done_driver;

    -- The physical START assertion must not wait for another AXIS clock edge.
    p_start_zero_cycle_check : process
    begin
        loop
            wait until rising_edge(lc_fire_done);
            wait for 1 ps;
            assert lc_start_tdc = '1'
                report "full_int: physical START did not assert directly from fire_done"
                severity failure;
        end loop;
    end process p_start_zero_cycle_check;

    -- Physical multi-Return source. Every pulse traverses the Echo Receiver's
    -- differential input and zero-added-latency STOP path before reaching the
    -- external GPX model. Returns are evenly distributed from near range to
    -- G_SIM_TARGET_M; the final Return therefore exercises the selected range.
    p_physical_echo_driver : process
        variable v_edge_time : time;
        variable v_elapsed   : time;
    begin
        pd_p <= (others => '0');
        pd_n <= (others => '1');
        wait until rst_n = '1';
        loop
            wait until rising_edge(gpx_start_tdc_mux);
            if G_ECHO_STIM_MODE = "physical_multi" then
                v_elapsed := 0 ns;
                for return_idx in 1 to G_RETURNS_PER_STOP loop
                    v_edge_time := C_SIM_TARGET_DELAY * return_idx
                                   / G_RETURNS_PER_STOP;
                    if v_edge_time > v_elapsed then
                        wait for v_edge_time - v_elapsed;
                    end if;
                    pd_p <= (others => '1');
                    pd_n <= (others => '0');
                    wait for C_ECHO_PULSE_WIDTH;
                    pd_p <= (others => '0');
                    pd_n <= (others => '1');
                    v_elapsed := v_edge_time + C_ECHO_PULSE_WIDTH;
                end loop;
            end if;
        end loop;
    end process p_physical_echo_driver;

    -- Echo must receive the descriptor for this shot no later than the
    -- synchronized logical T0. With the direct laser->echo AXIS connection,
    -- TREADY is permanently high and this is a hard integration contract.
    p_descriptor_before_shot : process(clk)
        variable v_descriptor_pending : boolean := false;
        variable v_shot_start_d1      : std_logic := '0';
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_descriptor_pending := false;
                v_shot_start_d1 := '0';
            else
                if lc_m_tvalid = '1' and lc_m_tready = '1' then
                    assert lc_m_tlast = '1' and lc_m_tuser(c_RES_SHOT_OPEN) = '1'
                        report "full_int: malformed laser shot descriptor"
                        severity failure;
                    v_descriptor_pending := true;
                end if;

                if lc_shot_start = '1' and v_shot_start_d1 = '0' then
                    assert v_descriptor_pending
                        report "full_int: logical T0 arrived before descriptor handshake"
                        severity failure;
                    v_descriptor_pending := false;
                end if;
                v_shot_start_d1 := lc_shot_start;
            end if;
        end if;
    end process p_descriptor_before_shot;

    -- i_shot_start / i_stop_tdc source mux. Default ("lc") routes from the
    -- laser_ctrl outputs; "direct" exposes a TB-controlled pulse for debug.
    td_shot_start_mux <= tb_shot_start when G_TDC_STIM_MODE = "direct"
                                       else lc_shot_start;
    td_shot_face_index_mux <= (others => '0') when G_TDC_STIM_MODE = "direct"
                                             else lc_shot_face_index;
    td_stop_tdc_mux   <= tb_stop_tdc   when G_TDC_STIM_MODE = "direct"
                                       else lc_stop_tdc;

    -- =========================================================================
    -- Back-pressure generator on VDMA tready lines.
    -- G_BP_TREADY_GAP > 0 toggles both tready low for 2 clks every N clks.
    -- =========================================================================
    p_bp : process(clk)
        variable v_cnt : natural := 0;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                v_cnt := 0;
                if G_BP_TREADY_GAP > 0 then
                    m_rise_tready <= '1';
                    m_fall_tready <= '1';
                end if;
            elsif G_BP_TREADY_GAP > 0 then
                v_cnt := v_cnt + 1;
                if v_cnt < G_BP_TREADY_GAP then
                    m_rise_tready <= '1';
                    m_fall_tready <= '1';
                elsif v_cnt < G_BP_TREADY_GAP + 2 then
                    m_rise_tready <= '0';
                    m_fall_tready <= '0';
                else
                    v_cnt := 0;
                    m_rise_tready <= '1';
                    m_fall_tready <= '1';
                end if;
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
    -- External Encoder source (physical-input-path verification only).
    -- In internal mode motor_decoder_top owns its built-in enc_top and these
    -- physical inputs are tied low, avoiding the former unused duplicate source.
    -- =========================================================================
    gen_external_encoder : if G_ENCODER_SOURCE = "external" generate
        enc_rst_n <= rst_n and enc_run;

        u_enc : entity work.enc_top
            generic map (
                g_CPR       => C_MD_CPR,
                g_DIR       => '0',
                g_TICKS_LO  => C_ENC_TICKS_LO_LOCAL,
                g_TICKS_HI  => C_ENC_TICKS_HI_LOCAL,
                g_HI_COUNT  => C_ENC_HI_COUNT_LOCAL
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
    end generate gen_external_encoder;

    gen_internal_encoder : if G_ENCODER_SOURCE = "internal" generate
        enc_rst_n <= '0';
        enc_a     <= '0';
        enc_b     <= '0';
        enc_z     <= '0';
        enc_pos   <= (others => '0');
        enc_phase <= (others => '0');
    end generate gen_internal_encoder;

    -- =========================================================================
    -- Motor + Laser integration IP. The hard-real-time Motor-to-Laser AXIS
    -- link is private to this wrapper and is required to remain always-ready.
    -- =========================================================================
    u_motor_laser : entity work.motor_laser_ctrl_top
        generic map (
            has_irq         => true,
            g_ENABLE_LOCAL_CSR => C_USE_LOCAL_CSR,
            g_PROC_CLK_MHZ  => C_AXIS_DOMAIN_CLK_MHZ,
            g_MOTOR_RPM     => C_MOTOR_RPM_LOCAL,
            g_CPR           => C_MD_CPR,
            g_DEC_MODE      => 4,
            g_DIR           => '0',
            g_TICKS_LO      => C_ENC_TICKS_LO_LOCAL,
            g_TICKS_HI      => C_ENC_TICKS_HI_LOCAL,
            g_HI_COUNT      => C_ENC_HI_COUNT_LOCAL,
            g_PHYSICAL_ENCODER_TO_AXIS_LATENCY_CLKS =>
                C_MD_PHYSICAL_LATENCY_CLKS,
            g_VIRTUAL_ENCODER_TO_AXIS_LATENCY_CLKS =>
                C_MD_VIRTUAL_LATENCY_CLKS,
            g_N_FACES       => G_N_FACES,
            g_TOTAL_STATES  => C_MD_TOTAL_STATES,
            g_FACE_CENTER_0 => C_MD_FACE_CENTER_0,
            g_FACE_CENTER_1 => C_MD_FACE_CENTER_1,
            g_FACE_CENTER_2 => C_MD_FACE_CENTER_2,
            g_FACE_CENTER_3 => C_MD_FACE_CENTER_3,
            g_FACE_CENTER_4 => C_MD_FACE_CENTER_4,
            g_FACE_HALF_0   => C_MD_FACE_HALF_ST,
            g_FACE_HALF_1   => C_MD_FACE_HALF_ST,
            g_FACE_HALF_2   => C_MD_FACE_HALF_ST,
            g_FACE_HALF_3   => C_MD_FACE_HALF_ST,
            g_FACE_HALF_4   => C_MD_FACE_HALF_ST,
            g_FIRE_DONE_TIMEOUT_5NS_TICKS =>
                C_FIRE_DONE_TIMEOUT_5NS_TICKS,
            g_TARGET_ROUNDTRIP_5NS_TICKS =>
                C_LASER_RANGE_5NS_TICKS,
            g_STEP_INTERVAL_STATES => C_STEP_INTERVAL
        )
        port map (
            s_axi_aclk          => clk,
            s_axi_aresetn       => rst_n,
            s_axi_motor_awvalid => md_awvalid,
            s_axi_motor_awready => md_awready,
            s_axi_motor_awaddr  => md_awaddr,
            s_axi_motor_awprot  => md_awprot,
            s_axi_motor_wvalid  => md_wvalid,
            s_axi_motor_wready  => md_wready,
            s_axi_motor_wdata   => md_wdata,
            s_axi_motor_wstrb   => md_wstrb,
            s_axi_motor_bvalid  => md_bvalid,
            s_axi_motor_bready  => md_bready,
            s_axi_motor_bresp   => md_bresp,
            s_axi_motor_arvalid => md_arvalid,
            s_axi_motor_arready => md_arready,
            s_axi_motor_araddr  => md_araddr,
            s_axi_motor_arprot  => md_arprot,
            s_axi_motor_rvalid  => md_rvalid,
            s_axi_motor_rready  => md_rready,
            s_axi_motor_rdata   => md_rdata,
            s_axi_motor_rresp   => md_rresp,
            s_axi_laser_awvalid => lc_awvalid,
            s_axi_laser_awready => lc_awready,
            s_axi_laser_awaddr  => lc_awaddr,
            s_axi_laser_awprot  => lc_awprot,
            s_axi_laser_wvalid  => lc_wvalid,
            s_axi_laser_wready  => lc_wready,
            s_axi_laser_wdata   => lc_wdata,
            s_axi_laser_wstrb   => lc_wstrb,
            s_axi_laser_bvalid  => lc_bvalid,
            s_axi_laser_bready  => lc_bready,
            s_axi_laser_bresp   => lc_bresp,
            s_axi_laser_arvalid => lc_arvalid,
            s_axi_laser_arready => lc_arready,
            s_axi_laser_araddr  => lc_araddr,
            s_axi_laser_arprot  => lc_arprot,
            s_axi_laser_rvalid  => lc_rvalid,
            s_axi_laser_rready  => lc_rready,
            s_axi_laser_rdata   => lc_rdata,
            s_axi_laser_rresp   => lc_rresp,
            i_unified_sys_ctrl => uc_ctl(C_CTL_SYS_CTRL),
            i_unified_sys_cfg_apply => uc_ctl(C_CTL_SYS_CFG_APPLY),
            i_unified_motor_cfg => uc_ctl(C_CTL_MOTOR_CFG),
            i_unified_motor_ticks_lo => uc_ctl(C_CTL_MOTOR_TICKS_LO),
            i_unified_motor_sched_latency =>
                uc_ctl(C_CTL_MOTOR_SCHED_LATENCY),
            i_unified_motor_z_param => uc_ctl(C_CTL_MOTOR_Z_PARAM),
            i_unified_motor_face_index => uc_ctl(C_CTL_MOTOR_FACE_INDEX),
            i_unified_motor_face_geometry =>
                uc_ctl(C_CTL_MOTOR_FACE_GEOMETRY),
            o_unified_motor_status => uc_stat(C_STAT_MOTOR_STATUS),
            o_unified_motor_face_geometry =>
                uc_stat(C_STAT_MOTOR_FACE_GEOMETRY),
            o_unified_motor_cfg_status => uc_stat(C_STAT_MOTOR_CFG_STATUS),
            o_unified_motor_quad_invalid =>
                uc_stat(C_STAT_MOTOR_QUAD_INVALID),
            o_unified_motor_axis_drop => uc_stat(C_STAT_MOTOR_AXIS_DROP),
            o_unified_motor_rev_period => uc_stat(C_STAT_MOTOR_REV_PERIOD),
            o_unified_motor_irq_cause => uc_motor_irq_cause,
            i_unified_laser_fire_cfg => uc_ctl(C_CTL_LASER_FIRE_CFG),
            i_unified_laser_roundtrip => uc_ctl(C_CTL_LASER_ROUNDTRIP),
            i_unified_laser_tdc_width => uc_ctl(C_CTL_LASER_TDC_WIDTH),
            i_unified_laser_sim_delay => uc_ctl(C_CTL_LASER_SIM_DELAY),
            i_unified_laser_sched0 => uc_ctl(C_CTL_LASER_SCHED0),
            i_unified_laser_sched1 => uc_ctl(C_CTL_LASER_SCHED1),
            i_unified_laser_sched2 => uc_ctl(C_CTL_LASER_SCHED2),
            o_unified_laser_status => uc_stat(C_STAT_LASER_STATUS),
            o_unified_laser_encoder_to_fire =>
                uc_stat(C_STAT_LASER_ENCODER_TO_FIRE),
            o_unified_laser_fire_done => uc_stat(C_STAT_LASER_FIRE_DONE),
            o_unified_laser_fire_to_tdc => uc_stat(C_STAT_LASER_FIRE_TO_TDC),
            o_unified_laser_metric_flags =>
                uc_stat(C_STAT_LASER_METRIC_FLAGS),
            o_unified_laser_frame_count => uc_stat(C_STAT_LASER_FRAME_COUNT),
            o_unified_laser_timeout_count =>
                uc_stat(C_STAT_LASER_TIMEOUT_COUNT),
            o_unified_laser_cfg_epoch_accepted => uc_laser_cfg_epoch,
            o_unified_laser_reset_epoch_accepted => uc_laser_reset_epoch,
            o_unified_laser_cfg_busy => uc_laser_cfg_busy,
            o_unified_laser_cfg_reject => uc_laser_cfg_reject,
            o_unified_laser_cfg_valid => uc_laser_cfg_valid,
            o_unified_laser_irq_cause => uc_laser_irq_cause,
            proc_aclk           => clk,
            proc_aresetn        => rst_n,
            i_enc_a             => enc_a,
            i_enc_b             => enc_b,
            i_enc_z             => enc_z,
            i_fire_done         => lc_fire_done,
            o_fire_pulse        => lc_fire_pulse,
            o_start_tdc         => lc_start_tdc,
            o_shot_start        => lc_shot_start,
            o_shot_face_index   => lc_shot_face_index,
            o_stop_tdc          => lc_stop_tdc,
            o_laser_active      => lc_laser_active,
            o_warning           => lc_warning,
            o_warning_any       => lc_warning_any,
            o_n_faces           => md_n_faces,
            o_irq               => ml_irq,
            m_axis_tvalid       => lc_m_tvalid,
            m_axis_tdata        => lc_m_tdata,
            m_axis_tuser        => lc_m_tuser,
            m_axis_tlast        => lc_m_tlast,
            m_axis_tready       => lc_m_tready,
            o_dbg_virt_a        => open,
            o_dbg_virt_b        => open,
            o_dbg_virt_z        => open,
            o_dbg_virt_pos      => md_dbg_virt_pos,
            o_dbg_virt_phase    => open,
            o_dbg_dec_count     => md_dbg_dec_count,
            o_dbg_dec_dir       => open,
            o_dbg_z_pulse       => open,
            o_dbg_active        => md_dbg_active,
            o_dbg_active_any    => md_dbg_active_any,
            o_dbg_face_index    => md_dbg_face_index,
            o_dbg_sim_en        => md_dbg_sim_en,
            o_dbg_rst_n_int     => md_dbg_rst_n_int,
            o_dbg_z_fault       => open,
            o_dbg_cfg_busy      => md_dbg_cfg_busy,
            o_dbg_fire_trigger  => lc_dbg_fire_trig,
            o_dbg_fire_busy     => lc_dbg_fire_busy,
            o_dbg_sim_flag      => lc_dbg_sim_flag,
            o_dbg_sim_done      => lc_dbg_sim_done,
            o_dbg_fire_done     => lc_dbg_fire_done_i,
            o_dbg_fire_delay    => lc_dbg_fire_delay,
            o_dbg_fsm_state     => lc_dbg_fsm_state,
            o_dbg_shot_cnt      => lc_dbg_shot_cnt,
            o_dbg_timeout       => lc_dbg_timeout,
            o_dbg_face_start    => lc_dbg_face_start,
            o_dbg_face_end      => lc_dbg_face_end,
            o_dbg_shot_accept   => lc_dbg_shot_accept
        );

    -- =========================================================================
    -- echo_receiver_top
    --   Takes start_tdc/stop_tdc + laser result + PD pulses; emits stop_evt
    --   and fire_count AXI-S.
    -- =========================================================================
    u_er : entity work.echo_receiver_top
        generic map (
            -- Formal name belongs to the sibling echo_receiver interface;
            -- this integration profile consumes the shared logical ABI max.
            g_N_CHIPS         => c_MAX_CHIPS,
            g_STOPS_PER_CHIP  => C_ER_N_STOPS,
            g_AXIS_CLK_MHZ    => C_AXIS_DOMAIN_CLK_MHZ,
            g_ENABLE_LOCAL_CSR => C_USE_LOCAL_CSR,
            -- Synthetic target ownership remains inside echo_receiver. Its
            -- CSR delay table creates STOP waveforms; the external GPX model
            -- below converts those waveforms into 28-bit I-Mode words.
            g_ENABLE_SIM_PATH => true,
            g_STOP_CNT_WIDTH  => C_STOP_CNT_WIDTH,
            g_STOP_EVT_DWIDTH => C_STOP_DW,
            g_FIRE_COUNT_DWIDTH => 32
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
            i_unified_cfg_clk => clk,
            i_unified_cfg_rst_n => rst_n,
            i_unified_sys_ctrl => uc_ctl(C_CTL_SYS_CTRL),
            i_unified_delay_cmd => uc_ctl(C_CTL_ECHO_DELAY_CMD),
            i_unified_delay_data => uc_ctl(C_CTL_ECHO_DELAY_DATA),
            o_unified_stat0 => uc_stat(C_STAT_ECHO_RISE_MASK),
            o_unified_stat1 => uc_stat(C_STAT_ECHO_FALL_MASK),
            o_unified_stat2 => uc_stat(C_STAT_ECHO_STATUS),
            o_unified_stat3 => uc_stat(C_STAT_ECHO_DELAY_READBACK),
            o_unified_reset_epoch_accepted => uc_echo_reset_epoch,
            o_unified_irq_cause => uc_echo_irq_cause,
            i_shot_start     => td_shot_start_mux,
            i_stop_tdc       => td_stop_tdc_mux,
            i_laser_evt_tvalid => lc_m_tvalid,
            i_laser_evt_tdata  => lc_m_tdata,
            i_laser_evt_tuser  => lc_m_tuser,
            i_laser_evt_tlast  => lc_m_tlast,
            o_laser_evt_tready => lc_m_tready,
            i_pd_lvds_p      => pd_p,
            i_pd_lvds_n      => pd_n,
            o_tdc_stop        => er_tdc_stop,
            o_stop_evt_tvalid => er_stop_tvalid,
            o_stop_evt_tdata  => er_stop_tdata,
            o_stop_evt_tkeep  => er_stop_tkeep,
            o_stop_evt_tuser  => er_stop_tuser,
            o_fire_count_tvalid => er_fire_count_tvalid,
            o_fire_count_tdata  => er_fire_count_tdata,
            o_fire_count_tkeep  => er_fire_count_tkeep,
            o_fire_count_tlast  => er_fire_count_tlast,
            o_irq             => er_irq
        );

    -- =========================================================================
    -- tdc_gpx_top (DUT, same config as tb_tdc_gpx_top_int)
    -- =========================================================================
    u_td : entity work.tdc_gpx_top
        generic map (
            g_HW_VERSION      => x"00010000",
            g_OUTPUT_WIDTH    => C_OUTPUT_W,
            -- Keep the build-time topology explicit in the integration TB.
            -- The runtime active/slope masks reported below are derived from
            -- these same values, so rtl_contract.json cannot silently assume
            -- a topology that differs from the elaborated DUT.
            g_NUM_CHIPS         => c_MAX_CHIPS,
            g_PRESENT_CHIP_MASK => c_ALL_CHIPS_MASK,
            g_RISE_CHIP_MASK    => c_DEFAULT_RISE_CHIP_MASK,
            g_FALL_CHIP_MASK    => c_DEFAULT_FALL_CHIP_MASK,
            g_MAX_STOPS_PER_CHIP => c_MAX_STOPS_PER_CHIP,
            g_MAX_HITS_PER_STOP  => c_MAX_HITS_PER_STOP,
            g_ENABLE_LOCAL_CSR => C_USE_LOCAL_CSR,
            g_AXIS_CLK_MHZ    => C_AXIS_DOMAIN_CLK_MHZ,
            g_TDC_CLK_MHZ     => C_TDC_DOMAIN_CLK_MHZ,
            g_DRAIN_MARGIN_TIME_NS => G_TDC_DRAIN_MARGIN_TIME_NS,
            g_STREAM_CLK_MODE => "ASYNC",
            -- Convert accelerated TB cycle knobs to the top-level physical
            -- time contract without changing the intended TDC cycle count.
            g_POWERUP_TIME_NS   => (G_POWERUP_CLKS * 1000) / C_TDC_DOMAIN_CLK_MHZ,
            g_RECOVERY_TIME_NS  => (G_RECOVERY_CLKS * 1000) / C_TDC_DOMAIN_CLK_MHZ,
            g_ALU_PULSE_TIME_NS => (G_ALU_PULSE_CLKS * 1000) / C_TDC_DOMAIN_CLK_MHZ
        )
        port map (
            i_axis_aclk     => clk,
            i_axis_aresetn  => rst_n,
            i_tdc_clk       => tdc_clk,
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
            i_unified_cfg_clk => clk,
            i_unified_cfg_rst_n => rst_n,
            i_unified_sys_ctrl => uc_ctl(C_CTL_SYS_CTRL),
            i_unified_sys_cfg_apply => uc_ctl(C_CTL_SYS_CFG_APPLY),
            i_unified_tdc_bus_timing => uc_ctl(C_CTL_TDC_BUS_TIMING),
            i_unified_tdc_start_offset => uc_ctl(C_CTL_TDC_START_OFFSET),
            i_unified_tdc_cfg_reg7 => uc_ctl(C_CTL_TDC_CFG_REG7),
            i_unified_tdc_image_cmd => uc_ctl(C_CTL_TDC_IMAGE_CMD),
            i_unified_tdc_image_data => uc_ctl(C_CTL_TDC_IMAGE_DATA),
            i_unified_tdc_scan_cfg => uc_ctl(C_CTL_TDC_SCAN_CFG),
            i_unified_tdc_pipeline_main => uc_ctl(C_CTL_TDC_PIPELINE_MAIN),
            i_unified_tdc_range_cols => uc_ctl(C_CTL_TDC_RANGE_COLS),
            i_unified_tdc_aux_cmd => uc_ctl(C_CTL_TDC_AUX_CMD),
            o_unified_tdc_chip0_result => uc_stat(C_STAT_TDC_CHIP0_RESULT),
            o_unified_tdc_chip1_result => uc_stat(C_STAT_TDC_CHIP1_RESULT),
            o_unified_tdc_chip2_result => uc_stat(C_STAT_TDC_CHIP2_RESULT),
            o_unified_tdc_chip3_result => uc_stat(C_STAT_TDC_CHIP3_RESULT),
            o_unified_tdc_pipeline_status => uc_stat(C_STAT_TDC_PIPELINE_STATUS),
            o_unified_tdc_status_ext => uc_stat(C_STAT_TDC_STATUS_EXT),
            o_unified_tdc_status_ext2 => uc_stat(C_STAT_TDC_STATUS_EXT2),
            o_unified_cfg_epoch_accepted => uc_tdc_cfg_epoch,
            o_unified_reset_epoch_accepted => uc_tdc_reset_epoch,
            o_unified_cfg_busy => uc_tdc_cfg_busy,
            o_unified_cfg_reject => uc_tdc_cfg_reject,
            o_unified_cfg_valid => uc_tdc_cfg_valid,
            o_unified_cmd_epoch_accepted => uc_tdc_cmd_epoch,
            o_unified_cmd_busy => uc_tdc_cmd_busy,
            o_unified_command_reject => uc_tdc_cmd_reject,
            o_unified_image_epoch_accepted => uc_tdc_image_epoch,
            o_unified_image_reject => uc_tdc_image_reject,
            o_unified_image_selected_data =>
                uc_stat(C_STAT_TDC_IMAGE_SELECTED_DATA),
            o_unified_irq_cause => uc_tdc_irq_cause,
            i_n_faces         => md_n_faces,
            i_shot_start      => td_shot_start_mux,
            i_shot_face_index => td_shot_face_index_mux,
            i_stop_tdc        => td_stop_tdc_mux,
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
            o_m_axis_tkeep    => m_rise_tkeep,
            o_m_axis_tstrb    => m_rise_tstrb,
            o_m_axis_tvalid   => m_rise_tvalid,
            o_m_axis_tlast    => m_rise_tlast,
            o_m_axis_tuser    => m_rise_tuser,
            i_m_axis_tready   => m_rise_tready,
            o_m_axis_fall_tdata  => m_fall_tdata,
            o_m_axis_fall_tkeep  => m_fall_tkeep,
            o_m_axis_fall_tstrb  => m_fall_tstrb,
            o_m_axis_fall_tvalid => m_fall_tvalid,
            o_m_axis_fall_tlast  => m_fall_tlast,
            o_m_axis_fall_tuser  => m_fall_tuser,
            i_m_axis_fall_tready => m_fall_tready,
            o_vdma_hsize_bytes_rise => td_vdma_hsize_rise,
            o_vdma_hsize_bytes_fall => td_vdma_hsize_fall,
            o_vdma_vsize_lines      => td_vdma_vsize,
            i_bin_resolution_ps => i_bin_res_ps,
            i_k_dist_fixed      => i_k_dist,
            o_irq      => td_irq,
            o_irq_pipe => td_irq_pipe
        );

    -- =========================================================================
    -- External TDC-GPX chips. Echo owns the STOP pattern; this model owns the
    -- GPX time interpolation, IFIFO contents, status pins and 28-bit bus.
    -- tdc_gpx_top remains an external-chip reader in every configuration.
    -- =========================================================================
    gpx_start_tdc_mux <= tb_shot_start when G_TDC_STIM_MODE = "direct"
                                       else lc_start_tdc;

    u_gpx_model : entity work.tdc_gpx_external_chip_model
        generic map (
            g_NUM_CHIPS                => c_MAX_CHIPS,
            g_STOPS_PER_CHIP           => C_ER_N_STOPS,
            g_TDC_CLK_MHZ              => C_TDC_DOMAIN_CLK_MHZ,
            g_CAPTURE_WINDOW_5NS_TICKS => C_MAX_RANGE_5NS_TICKS,
            g_BIN_RESOLUTION_PS        => C_BIN_RESOLUTION_PS,
            g_FIFO_DEPTH               => 32,
            g_LF_THRESHOLD             => 4,
            g_CHIP_SLOPE_MASK          => G_CHIP_SLOPE_MASK
        )
        port map (
            i_tdc_clk        => tdc_clk,
            i_rst_n          => rst_n,
            i_start_tdc      => gpx_start_tdc_mux,
            i_tdc_stop       => er_tdc_stop,
            io_tdc_d         => io_tdc_d,
            i_tdc_adr        => o_tdc_adr,
            i_tdc_csn        => o_tdc_csn,
            i_tdc_rdn        => o_tdc_rdn,
            i_tdc_wrn        => o_tdc_wrn,
            i_tdc_oen        => o_tdc_oen,
            i_tdc_stopdis    => o_tdc_stopdis,
            i_tdc_alutrigger => o_tdc_alutrigger,
            i_tdc_puresn     => o_tdc_puresn,
            o_tdc_ef1        => i_tdc_ef1,
            o_tdc_ef2        => i_tdc_ef2,
            o_tdc_lf1        => i_tdc_lf1,
            o_tdc_lf2        => i_tdc_lf2,
            o_tdc_irflag     => i_tdc_irflag,
            o_tdc_errflag    => i_tdc_errflag,
            o_last_hit       => gpx_last_hit,
            o_capture_count  => gpx_capture_count
        );

    -- Independent pin-domain reference. It timestamps every physical STOP
    -- channel and every Return instead of trusting the GPX model's internal
    -- calculation. The flattened reference is later consumed by both the raw
    -- 28-bit bus checker and the packed VDMA Cell checker.
    p_expected_hit : process(rst_n, gpx_start_tdc_mux, er_tdc_stop(0))
        variable v_t0           : time := 0 ns;
        variable v_armed        : boolean := false;
        variable v_shot_count   : natural range 0 to C_MON_SHOT_DEPTH := 0;
        variable v_current_shot : natural range 0 to C_MON_SHOT_DEPTH - 1 := 0;
        variable v_hit_count    : natural range 0 to c_MAX_HITS_PER_STOP := 0;
        variable v_elapsed_ps   : natural;
        variable v_hit          : natural;
        variable v_ref_idx      : natural;
    begin
        if rst_n = '0' then
            v_t0 := 0 ns;
            v_armed := false;
            v_shot_count := 0;
            v_current_shot := 0;
            v_hit_count := 0;
            mon_expected_hit_refs <= (others => (others => '0'));
            mon_expected_shots <= 0;
            mon_last_expected_hit <= (others => '0');
        else
            if rising_edge(gpx_start_tdc_mux) then
                assert v_shot_count < C_MON_SHOT_DEPTH
                    report "full_int: expected-hit shot queue overflow"
                    severity failure;
                if v_shot_count < C_MON_SHOT_DEPTH then
                    v_current_shot := v_shot_count;
                    v_shot_count := v_shot_count + 1;
                    mon_expected_shots <= v_shot_count;
                end if;
                v_t0 := now;
                v_armed := true;
                v_hit_count := 0;
            end if;

            if rising_edge(er_tdc_stop(0)) then
                assert v_armed
                    report "full_int: Echo STOP arrived without a GPX START"
                    severity failure;
                assert v_hit_count < fn_expected_returns(0)
                    report "full_int: Echo emitted more Returns than configured"
                    severity failure;
                if v_armed and v_hit_count < fn_expected_returns(0) then
                    v_elapsed_ps := (now - v_t0) / 1 ps;
                    v_hit := v_elapsed_ps / C_BIN_RESOLUTION_PS;
                    assert v_hit < 2 ** c_RAW_HIT_WIDTH
                        report "full_int: reference hit exceeds 17-bit I-Mode field"
                        severity failure;
                    v_ref_idx := fn_hit_ref_index(
                        v_current_shot, 0, v_hit_count);
                    mon_expected_hit_refs(v_ref_idx) <=
                        to_unsigned(v_hit, c_RAW_HIT_WIDTH);
                    v_hit_count := v_hit_count + 1;
                    mon_last_expected_hit <=
                        to_unsigned(v_hit, c_RAW_HIT_WIDTH);
                end if;
            end if;
        end if;
    end process p_expected_hit;

    -- Check the actual 28-bit GPX bus at the first cycle of every IFIFO1
    -- read. This makes the datasheet I-Mode contract explicit before the DUT
    -- decoder transforms the word into internal events.
    p_i_mode_bus_check : process(tdc_clk)
        variable v_read_checked : std_logic_vector(c_MAX_CHIPS - 1 downto 0) :=
            (others => '0');
        variable v_read_active : std_logic;
        variable v_addr : std_logic_vector(c_TDC_ADR_WIDTH - 1 downto 0);
        variable v_raw : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        variable v_count : natural := 0;
        variable v_stop_id : natural range 0 to C_ER_N_STOPS - 1;
        variable v_channel : natural range 0 to C_PD_WIDTH - 1;
        variable v_hit_seq : natural range 0 to c_MAX_HITS_PER_STOP;
        variable v_ref_idx : natural;
        variable v_read_shot : t_chip_count_array := (others => 0);
        variable v_stop_hits : t_channel_count_array := (others => 0);
        variable v_all_complete : boolean;
    begin
        if rising_edge(tdc_clk) then
            if rst_n = '0' then
                v_read_checked := (others => '0');
                v_count := 0;
                v_read_shot := (others => 0);
                v_stop_hits := (others => 0);
                mon_i_mode_bus_checks <= 0;
            else
                for chip in 0 to c_MAX_CHIPS - 1 loop
                    v_read_active := not o_tdc_csn(chip)
                                     and not o_tdc_oen(chip)
                                     and not o_tdc_rdn(chip);
                    v_addr := o_tdc_adr(
                        (chip + 1) * c_TDC_ADR_WIDTH - 1 downto
                        chip * c_TDC_ADR_WIDTH);
                    v_raw := io_tdc_d(
                        (chip + 1) * c_TDC_BUS_WIDTH - 1 downto
                        chip * c_TDC_BUS_WIDTH);

                    if v_read_active = '0' then
                        v_read_checked(chip) := '0';
                    elsif v_read_checked(chip) = '0'
                          and (v_addr = c_TDC_REG8_IFIFO1
                               or v_addr = c_TDC_REG9_IFIFO2)
                          and unsigned(v_raw(c_RAW_HIT_HI downto c_RAW_HIT_LO)) /= 0 then
                        if v_addr = c_TDC_REG8_IFIFO1 then
                            v_stop_id := to_integer(unsigned(
                                v_raw(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO)));
                        else
                            v_stop_id := 4 + to_integer(unsigned(
                                v_raw(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO)));
                        end if;
                        v_channel := chip * C_ER_N_STOPS + v_stop_id;
                        v_hit_seq := v_stop_hits(v_channel);

                        assert v_stop_id < G_STOPS_PER_CHIP
                            report "full_int: GPX read an inactive STOP channel"
                            severity failure;
                        assert v_raw(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) =
                               std_logic_vector(to_unsigned(v_stop_id mod 4, 2))
                            report "full_int: GPX I-Mode ChaCode differs from IFIFO STOP order"
                            severity failure;
                        assert v_raw(c_RAW_STARTNUM_HI downto c_RAW_STARTNUM_LO) = x"00"
                            report "full_int: GPX I-Mode StartNum is not zero in SINGLE_SHOT"
                            severity failure;
                        assert v_raw(c_RAW_SLOPE_BIT) = G_CHIP_SLOPE_MASK(chip)
                            report "full_int: GPX I-Mode Slope differs from chip profile"
                            severity failure;
                        assert v_hit_seq < fn_expected_returns(v_channel)
                            report "full_int: GPX returned more hits than the Echo reference"
                            severity failure;
                        assert v_read_shot(chip) < mon_expected_shots
                            report "full_int: GPX read has no matching START reference"
                            severity failure;
                        v_ref_idx := fn_hit_ref_index(
                            v_read_shot(chip), v_channel, v_hit_seq);
                        assert v_raw(c_RAW_HIT_HI downto c_RAW_HIT_LO) =
                               std_logic_vector(mon_expected_hit_refs(v_ref_idx))
                            report "full_int: GPX I-Mode Hit[16:0] differs from pin timestamp reference"
                            severity failure;
                        v_stop_hits(v_channel) := v_hit_seq + 1;
                        v_count := v_count + 1;
                        v_read_checked(chip) := '1';

                        v_all_complete := true;
                        for stop_id in 0 to G_STOPS_PER_CHIP - 1 loop
                            v_channel := chip * C_ER_N_STOPS + stop_id;
                            if v_stop_hits(v_channel) <
                               fn_expected_returns(v_channel) then
                                v_all_complete := false;
                            end if;
                        end loop;
                        if v_all_complete then
                            v_read_shot(chip) := v_read_shot(chip) + 1;
                            for stop_id in 0 to G_STOPS_PER_CHIP - 1 loop
                                v_channel := chip * C_ER_N_STOPS + stop_id;
                                v_stop_hits(v_channel) := 0;
                            end loop;
                        end if;
                    end if;
                end loop;
                mon_i_mode_bus_checks <= v_count;
            end if;
        end if;
    end process p_i_mode_bus_check;

    -- =========================================================================
    -- Monitors
    -- =========================================================================
    p_mon : process(clk)
        variable v_prev_fire, v_prev_start, v_prev_stop : std_logic := '0';
        variable v_prev_td_shot_start                  : std_logic := '0';
        variable v_prev_irflag                         : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
        variable v_prev_alutrigger                     : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
        variable v_prev_cmd_start, v_prev_cmd_accept   : std_logic := '0';
        variable v_prev_shot_gated                     : std_logic := '0';
        variable v_prev_cfg_rej, v_prev_pipe_abort     : std_logic := '0';
        variable v_prev_md_virt_pos  : std_logic_vector(14 downto 0) := (others => '0');
        variable v_prev_md_dec_count : std_logic_vector(14 downto 0) := (others => '0');
        variable v_rise_line_beats   : natural := 0;
        variable v_fall_line_beats   : natural := 0;
        variable v_shot_face_queue   : t_face_index_queue :=
            (others => (others => '0'));
        variable v_shot_cycle_queue  : t_cycle_queue := (others => 0);
        variable v_fire_cycle_queue  : t_cycle_queue := (others => 0);
        variable v_shot_face_count   : natural := 0;
        variable v_fire_cycle_count  : natural := 0;
        variable v_start_cycle_count : natural := 0;
        variable v_stop_cycle_index  : natural := 0;
        variable v_rise_header_count : natural := 0;
        variable v_fall_header_count : natural := 0;
        variable v_rise_header_shot_index : natural := 0;
        variable v_fall_header_shot_index : natural := 0;
        variable v_rise_face_header_line  : boolean := false;
        variable v_fall_face_header_line  : boolean := false;
        variable v_header_word_idx   : natural := 0;
        variable v_header_word       : std_logic_vector(31 downto 0);
        variable v_rise_line_index   : natural := 0;
        variable v_fall_line_index   : natural := 0;
        variable v_rise_hit_check_count : natural := 0;
        variable v_fall_hit_check_count : natural := 0;
        variable v_data_word_idx     : natural;
        variable v_cell_idx          : natural;
        variable v_cell_word_idx     : natural;
        variable v_hit_seq           : natural;
        variable v_channel           : natural;
        variable v_expected_returns  : natural;
        variable v_ref_idx           : natural;
        variable v_expected_mask     : std_logic_vector(
            c_MAX_HITS_PER_STOP - 1 downto 0);
        variable v_expected_hit      : unsigned(c_RAW_HIT_WIDTH - 1 downto 0);
        variable v_axis_cycle        : natural := 0;
        variable v_prev_start_cycle  : natural := 0;
        variable v_have_start_cycle  : boolean := false;
        variable v_interval_clks     : natural;
        variable v_latency_clks      : natural;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                mon_md_virt_pos_changes <= 0;
                mon_md_dec_count_changes <= 0;
                mon_md_active_cycles <= 0;
                mon_md_cfg_busy_cycles <= 0;
                mon_lc_fire_cnt <= 0;
                mon_lc_start_cnt <= 0;
                mon_lc_stop_cnt <= 0;
                mon_lc_m_beats <= 0;
                mon_lc_m_tlast <= 0;
                mon_er_stop_beats <= 0;
                mon_er_fire_count_beats <= 0;
                mon_er_stop_high_cycles <= 0;
                mon_td_rise_beats <= 0;
                mon_td_fall_beats <= 0;
                mon_td_rise_line_end <= 0;
                mon_td_fall_line_end <= 0;
                mon_td_rise_header_checks <= 0;
                mon_td_fall_header_checks <= 0;
                mon_td_rise_hit_checks <= 0;
                mon_td_fall_hit_checks <= 0;
                mon_td_rise_face_seen <= (others => '0');
                mon_td_fall_face_seen <= (others => '0');
                mon_last_rise_hit <= (others => '0');
                mon_last_fall_hit <= (others => '0');
                mon_cmd_start_cnt  <= 0;
                mon_cmd_accept_cnt <= 0;
                mon_chip_busy_any  <= 0;
                mon_shot_gated_cnt <= 0;
                mon_shot_chip_any  <= 0;
                mon_cfg_rejected   <= 0;
                mon_pipe_abort     <= 0;
                v_prev_fire := '0';
                v_prev_start := '0';
                v_prev_stop := '0';
                v_prev_td_shot_start := '0';
                v_prev_irflag := (others => '0');
                v_prev_alutrigger := (others => '0');
                v_prev_cmd_start := '0';
                v_prev_cmd_accept := '0';
                v_prev_shot_gated := '0';
                v_prev_cfg_rej := '0';
                v_prev_pipe_abort := '0';
                v_prev_md_virt_pos := (others => '0');
                v_prev_md_dec_count := (others => '0');
                v_rise_line_beats := 0;
                v_fall_line_beats := 0;
                v_shot_face_count := 0;
                v_fire_cycle_count := 0;
                v_start_cycle_count := 0;
                v_stop_cycle_index := 0;
                v_rise_header_count := 0;
                v_fall_header_count := 0;
                v_rise_header_shot_index := 0;
                v_fall_header_shot_index := 0;
                v_rise_face_header_line := false;
                v_fall_face_header_line := false;
                v_rise_line_index := 0;
                v_fall_line_index := 0;
                v_rise_hit_check_count := 0;
                v_fall_hit_check_count := 0;
                v_axis_cycle := 0;
                v_prev_start_cycle := 0;
                v_have_start_cycle := false;
                mon_shot_interval_min_clks <= 0;
                mon_shot_interval_max_clks <= 0;
                mon_fire_done_latency_min_clks <= 0;
                mon_fire_done_latency_max_clks <= 0;
                mon_range_wait_min_clks <= 0;
                mon_range_wait_max_clks <= 0;
                mon_shot_to_rise_tlast_min_clks <= 0;
                mon_shot_to_rise_tlast_max_clks <= 0;
                mon_shot_to_fall_tlast_min_clks <= 0;
                mon_shot_to_fall_tlast_max_clks <= 0;
                mon_schedule_overrun <= 0;
                dbg_lc_start_ever <= '0';
                dbg_lc_fire_ever  <= '0';
            else
                v_axis_cycle := v_axis_cycle + 1;
                -- The shot-start pulse and its face index are one qualified
                -- transfer. Queue the payload at the transfer boundary so
                -- delayed/deferred TDC output can be checked in order.
                if td_shot_start_mux = '1' and v_prev_td_shot_start = '0' then
                    report "full_int timing: shot_start, IrFlag="
                        & to_hstring(i_tdc_irflag)
                        severity note;
                    assert v_shot_face_count < C_MON_SHOT_DEPTH
                        report "full_int: shot face monitor queue overflow"
                        severity failure;
                    v_shot_face_queue(v_shot_face_count) :=
                        td_shot_face_index_mux;
                    v_shot_cycle_queue(v_shot_face_count) := v_axis_cycle;
                    v_shot_face_count := v_shot_face_count + 1;
                    assert unsigned(td_shot_face_index_mux) < unsigned(md_n_faces)
                        report "full_int: laser shot face index is outside motor n_faces"
                        severity failure;
                end if;
                v_prev_td_shot_start := td_shot_start_mux;

                -- DUT internal rising-edge counters (via VHDL-2008 aliases)
                if dbg_cmd_start = '1' and v_prev_cmd_start = '0' then
                    mon_cmd_start_cnt <= mon_cmd_start_cnt + 1;
                end if;
                if dbg_cmd_start_accept = '1' and v_prev_cmd_accept = '0' then
                    mon_cmd_accept_cnt <= mon_cmd_accept_cnt + 1;
                end if;
                if dbg_chip_busy /= (dbg_chip_busy'range => '0') then
                    mon_chip_busy_any <= mon_chip_busy_any + 1;
                end if;
                if dbg_shot_gated = '1' and v_prev_shot_gated = '0' then
                    mon_shot_gated_cnt <= mon_shot_gated_cnt + 1;
                end if;
                if dbg_shot_per_chip /= (dbg_shot_per_chip'range => '0') then
                    mon_shot_chip_any <= mon_shot_chip_any + 1;
                end if;
                if dbg_cfg_rejected_r = '1' and v_prev_cfg_rej = '0' then
                    mon_cfg_rejected <= mon_cfg_rejected + 1;
                end if;
                if dbg_pipeline_abort = '1' and v_prev_pipe_abort = '0' then
                    mon_pipe_abort <= mon_pipe_abort + 1;
                end if;
                v_prev_cmd_start  := dbg_cmd_start;
                v_prev_cmd_accept := dbg_cmd_start_accept;
                v_prev_shot_gated := dbg_shot_gated;
                v_prev_cfg_rej    := dbg_cfg_rejected_r;
                v_prev_pipe_abort := dbg_pipeline_abort;

                -- Sticky latches (never clear except on reset) so slow probes
                -- can observe that an event happened at least once
                if lc_start_tdc = '1' then dbg_lc_start_ever <= '1'; end if;
                if lc_fire_pulse = '1' then dbg_lc_fire_ever <= '1'; end if;
                if md_dbg_virt_pos /= v_prev_md_virt_pos then
                    mon_md_virt_pos_changes <= mon_md_virt_pos_changes + 1;
                end if;
                if md_dbg_dec_count /= v_prev_md_dec_count then
                    mon_md_dec_count_changes <= mon_md_dec_count_changes + 1;
                end if;
                if md_dbg_active_any = '1' then
                    mon_md_active_cycles <= mon_md_active_cycles + 1;
                end if;
                if md_dbg_cfg_busy = '1' then
                    mon_md_cfg_busy_cycles <= mon_md_cfg_busy_cycles + 1;
                end if;
                v_prev_md_virt_pos := md_dbg_virt_pos;
                v_prev_md_dec_count := md_dbg_dec_count;
                if lc_fire_pulse = '1' and v_prev_fire = '0' then
                    assert v_fire_cycle_count < C_MON_SHOT_DEPTH
                        report "full_int: fire timing queue overflow"
                        severity failure;
                    v_fire_cycle_queue(v_fire_cycle_count) := v_axis_cycle;
                    v_fire_cycle_count := v_fire_cycle_count + 1;
                    mon_lc_fire_cnt <= mon_lc_fire_cnt + 1;
                end if;
                if lc_start_tdc = '1' and v_prev_start = '0' then
                    -- The internal encoder smoke path can inject synthetic
                    -- Shot events without a one-to-one laser fire reference.
                    -- End-to-end fire latency is valid on the external path.
                    if G_ENCODER_SOURCE = "external" then
                        assert v_start_cycle_count < v_fire_cycle_count
                            report "full_int: start_tdc has no matching fire timing reference"
                            severity failure;
                        v_latency_clks := v_axis_cycle
                            - v_fire_cycle_queue(v_start_cycle_count);
                        if mon_fire_done_latency_min_clks = 0
                           or v_latency_clks < mon_fire_done_latency_min_clks then
                            mon_fire_done_latency_min_clks <= v_latency_clks;
                        end if;
                        if v_latency_clks > mon_fire_done_latency_max_clks then
                            mon_fire_done_latency_max_clks <= v_latency_clks;
                        end if;
                    end if;
                    v_start_cycle_count := v_start_cycle_count + 1;
                    mon_lc_start_cnt <= mon_lc_start_cnt + 1;
                    if v_have_start_cycle then
                        v_interval_clks := v_axis_cycle - v_prev_start_cycle;
                        if mon_shot_interval_min_clks = 0
                           or v_interval_clks < mon_shot_interval_min_clks then
                            mon_shot_interval_min_clks <= v_interval_clks;
                        end if;
                        if v_interval_clks > mon_shot_interval_max_clks then
                            mon_shot_interval_max_clks <= v_interval_clks;
                        end if;
                    end if;
                    v_prev_start_cycle := v_axis_cycle;
                    v_have_start_cycle := true;
                end if;
                if lc_stop_tdc = '1' and v_prev_stop = '0' then
                    assert v_stop_cycle_index < v_shot_face_count
                        report "full_int: stop_tdc has no matching shot timing reference"
                        severity failure;
                    v_latency_clks := v_axis_cycle
                        - v_shot_cycle_queue(v_stop_cycle_index);
                    if mon_range_wait_min_clks = 0
                       or v_latency_clks < mon_range_wait_min_clks then
                        mon_range_wait_min_clks <= v_latency_clks;
                    end if;
                    if v_latency_clks > mon_range_wait_max_clks then
                        mon_range_wait_max_clks <= v_latency_clks;
                    end if;
                    v_stop_cycle_index := v_stop_cycle_index + 1;
                    report "full_int timing: stop_tdc, IrFlag="
                        & to_hstring(i_tdc_irflag)
                        & ", AluTrigger=" & to_hstring(o_tdc_alutrigger)
                        severity note;
                    mon_lc_stop_cnt <= mon_lc_stop_cnt + 1;
                end if;
                if i_tdc_irflag /= v_prev_irflag then
                    report "full_int timing: IrFlag "
                        & to_hstring(v_prev_irflag) & "->" & to_hstring(i_tdc_irflag)
                        severity note;
                end if;
                if o_tdc_alutrigger /= v_prev_alutrigger then
                    report "full_int timing: AluTrigger "
                        & to_hstring(v_prev_alutrigger) & "->" & to_hstring(o_tdc_alutrigger)
                        severity note;
                end if;
                v_prev_fire := lc_fire_pulse;
                v_prev_start := lc_start_tdc;
                v_prev_stop := lc_stop_tdc;
                v_prev_irflag := i_tdc_irflag;
                v_prev_alutrigger := o_tdc_alutrigger;
                if lc_warning(c_WARNING_SCHEDULE_OVERRUN) = '1' then
                    mon_schedule_overrun <= 1;
                end if;

                if lc_m_tvalid = '1' then
                    mon_lc_m_beats <= mon_lc_m_beats + 1;
                    if lc_m_tlast = '1' then
                        mon_lc_m_tlast <= mon_lc_m_tlast + 1;
                    end if;
                end if;
                if er_stop_tvalid = '1' then
                    mon_er_stop_beats <= mon_er_stop_beats + 1;
                end if;
                if er_fire_count_tvalid = '1' then
                    mon_er_fire_count_beats <= mon_er_fire_count_beats + 1;
                end if;
                if er_tdc_stop /= (er_tdc_stop'range => '0') then
                    mon_er_stop_high_cycles <= mon_er_stop_high_cycles + 1;
                end if;
                if m_rise_tvalid = '1' and m_rise_tready = '1' then
                    assert m_rise_tkeep = (m_rise_tkeep'range => '1')
                        report "full_int: rising tkeep must be all ones on accepted output beats"
                        severity error;
                    assert m_rise_tstrb = (m_rise_tstrb'range => '1')
                        report "full_int: rising tstrb must be all ones on accepted output beats"
                        severity error;
                    if m_rise_tuser(0) = '1' then
                        assert v_rise_line_beats = 0
                            report "full_int: rising SOF arrived mid-line"
                            severity failure;
                        v_rise_face_header_line := true;
                    end if;
                    for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                        v_header_word_idx :=
                            v_rise_line_beats * C_WORDS_PER_BEAT + lane;
                        if v_rise_face_header_line
                           and v_header_word_idx = C_HEADER_WORD_INDEX then
                            v_header_word := m_rise_tdata(
                                32 * lane + 31 downto 32 * lane);
                            assert v_rise_header_shot_index < v_shot_face_count
                                report "full_int: rising face header has no matching first-shot payload"
                                severity failure;
                            assert v_header_word(7 downto 0) =
                                   std_logic_vector(resize(unsigned(
                                       v_shot_face_queue(
                                           v_rise_header_shot_index)), 8))
                                report "full_int: rising header face index diverged from accepted laser shot"
                                severity failure;
                            assert v_header_word(14 downto 12) = md_n_faces
                                report "full_int: rising header n_faces diverged from motor static sideband"
                                severity failure;
                            assert unsigned(v_header_word(7 downto 0)) <
                                   resize(unsigned(md_n_faces), 8)
                                report "full_int: rising header face index is outside n_faces"
                                severity failure;
                            if to_integer(unsigned(v_header_word(7 downto 0))) <= 4 then
                                mon_td_rise_face_seen(to_integer(unsigned(
                                    v_header_word(7 downto 0)))) <= '1';
                            end if;
                            v_rise_header_count := v_rise_header_count + 1;
                            v_rise_header_shot_index :=
                                v_rise_header_shot_index + G_COLS_PER_FACE;
                            mon_td_rise_header_checks <=
                                mon_td_rise_header_checks + 1;
                        end if;
                        if v_header_word_idx >= C_FIRST_CELL_HIT_WORD_INDEX
                           and v_header_word_idx <
                               C_FIRST_CELL_HIT_WORD_INDEX
                               + C_RISE_CELL_COUNT * C_CELL_WORDS then
                            v_data_word_idx := v_header_word_idx
                                               - C_FIRST_CELL_HIT_WORD_INDEX;
                            v_cell_idx := v_data_word_idx / C_CELL_WORDS;
                            v_cell_word_idx := v_data_word_idx mod C_CELL_WORDS;
                            v_channel := fn_lane_channel(
                                C_RISE_ACTIVE_MASK, v_cell_idx);
                            v_expected_returns := fn_expected_returns(v_channel);
                            v_header_word := m_rise_tdata(
                                32 * lane + 31 downto 32 * lane);

                            if v_cell_word_idx < C_HIT_WORDS_PER_CELL then
                                for slot in 0 to 1 loop
                                    v_hit_seq := v_cell_word_idx * 2 + slot;
                                    if v_hit_seq < C_MAX_HITS then
                                        if v_hit_seq < v_expected_returns then
                                            v_ref_idx := fn_hit_ref_index(
                                                v_rise_line_index, v_channel,
                                                v_hit_seq);
                                            v_expected_hit :=
                                                mon_expected_hit_refs(v_ref_idx);
                                            assert v_header_word(
                                                       slot * 16 + 15 downto
                                                       slot * 16) =
                                                   std_logic_vector(
                                                       v_expected_hit(15 downto 0))
                                                report "full_int: rising VDMA Hit[15:0] differs from pin reference"
                                                severity failure;
                                        else
                                            assert v_header_word(
                                                       slot * 16 + 15 downto
                                                       slot * 16) = x"0000"
                                                report "full_int: rising inactive Hit slot is not zero"
                                                severity failure;
                                        end if;
                                    end if;
                                end loop;
                            else
                                v_expected_mask := (others => '0');
                                for hit_idx in 0 to c_MAX_HITS_PER_STOP - 1 loop
                                    if hit_idx < v_expected_returns then
                                        v_expected_mask(hit_idx) := '1';
                                        v_ref_idx := fn_hit_ref_index(
                                            v_rise_line_index, v_channel,
                                            hit_idx);
                                        v_expected_hit :=
                                            mon_expected_hit_refs(v_ref_idx);
                                        assert v_header_word(hit_idx) =
                                               v_expected_hit(c_RAW_HIT_WIDTH - 1)
                                            report "full_int: rising VDMA Hit[16] differs from pin reference"
                                            severity failure;
                                        mon_last_rise_hit <= v_expected_hit;
                                    end if;
                                end loop;
                                assert v_header_word(31 downto 25) =
                                       v_expected_mask
                                    report "full_int: rising Cell hit_valid mask mismatch"
                                    severity failure;
                                assert v_header_word(24 downto 18) =
                                       v_expected_mask
                                    report "full_int: rising Cell slope mask mismatch"
                                    severity failure;
                                assert to_integer(unsigned(
                                           v_header_word(15 downto 12))) =
                                       v_expected_returns
                                    report "full_int: rising Cell hit_count mismatch"
                                    severity failure;
                                assert v_header_word(11 downto 10) = "00"
                                    report "full_int: rising Cell drop/error flag set"
                                    severity failure;
                                assert v_header_word(9 downto 8) =
                                       std_logic_vector(to_unsigned(
                                           v_channel / C_ER_N_STOPS, 2))
                                    report "full_int: rising Cell chip_id mismatch"
                                    severity failure;
                                v_rise_hit_check_count :=
                                    v_rise_hit_check_count + v_expected_returns;
                                mon_td_rise_hit_checks <=
                                    v_rise_hit_check_count;
                            end if;
                        end if;
                    end loop;
                    mon_td_rise_beats <= mon_td_rise_beats + 1;
                    v_rise_line_beats := v_rise_line_beats + 1;
                    if m_rise_tlast = '1' then
                        assert v_rise_line_index < v_shot_face_count
                            report "full_int: rising TLAST has no matching shot timing reference"
                            severity failure;
                        v_latency_clks := v_axis_cycle
                            - v_shot_cycle_queue(v_rise_line_index);
                        if mon_shot_to_rise_tlast_min_clks = 0
                           or v_latency_clks < mon_shot_to_rise_tlast_min_clks then
                            mon_shot_to_rise_tlast_min_clks <= v_latency_clks;
                        end if;
                        if v_latency_clks > mon_shot_to_rise_tlast_max_clks then
                            mon_shot_to_rise_tlast_max_clks <= v_latency_clks;
                        end if;
                        mon_td_rise_line_end <= mon_td_rise_line_end + 1;
                        report "full_int: rising line beats="
                             & integer'image(v_rise_line_beats)
                             & " expected="
                             & integer'image(C_EXPECT_HSIZE_RISE / (G_TDATA_WIDTH / 8))
                            severity note;
                        assert v_rise_line_beats =
                               C_EXPECT_HSIZE_RISE / (G_TDATA_WIDTH / 8)
                            report "full_int: rising line beat count diverged from HSIZE"
                            severity failure;
                        v_rise_line_beats := 0;
                        v_rise_line_index := v_rise_line_index + 1;
                        v_rise_face_header_line := false;
                    end if;
                end if;
                if m_fall_tvalid = '1' and m_fall_tready = '1' then
                    assert m_fall_tkeep = (m_fall_tkeep'range => '1')
                        report "full_int: falling tkeep must be all ones on accepted output beats"
                        severity error;
                    assert m_fall_tstrb = (m_fall_tstrb'range => '1')
                        report "full_int: falling tstrb must be all ones on accepted output beats"
                        severity error;
                    if m_fall_tuser(0) = '1' then
                        assert v_fall_line_beats = 0
                            report "full_int: falling SOF arrived mid-line"
                            severity failure;
                        v_fall_face_header_line := true;
                    end if;
                    for lane in 0 to C_WORDS_PER_BEAT - 1 loop
                        v_header_word_idx :=
                            v_fall_line_beats * C_WORDS_PER_BEAT + lane;
                        if v_fall_face_header_line
                           and v_header_word_idx = C_HEADER_WORD_INDEX then
                            v_header_word := m_fall_tdata(
                                32 * lane + 31 downto 32 * lane);
                            assert v_fall_header_shot_index < v_shot_face_count
                                report "full_int: falling face header has no matching first-shot payload"
                                severity failure;
                            assert v_header_word(7 downto 0) =
                                   std_logic_vector(resize(unsigned(
                                       v_shot_face_queue(
                                           v_fall_header_shot_index)), 8))
                                report "full_int: falling header face index diverged from accepted laser shot"
                                severity failure;
                            assert v_header_word(14 downto 12) = md_n_faces
                                report "full_int: falling header n_faces diverged from motor static sideband"
                                severity failure;
                            assert unsigned(v_header_word(7 downto 0)) <
                                   resize(unsigned(md_n_faces), 8)
                                report "full_int: falling header face index is outside n_faces"
                                severity failure;
                            if to_integer(unsigned(v_header_word(7 downto 0))) <= 4 then
                                mon_td_fall_face_seen(to_integer(unsigned(
                                    v_header_word(7 downto 0)))) <= '1';
                            end if;
                            v_fall_header_count := v_fall_header_count + 1;
                            v_fall_header_shot_index :=
                                v_fall_header_shot_index + G_COLS_PER_FACE;
                            mon_td_fall_header_checks <=
                                mon_td_fall_header_checks + 1;
                        end if;
                        if v_header_word_idx >= C_FIRST_CELL_HIT_WORD_INDEX
                           and v_header_word_idx <
                               C_FIRST_CELL_HIT_WORD_INDEX
                               + C_FALL_CELL_COUNT * C_CELL_WORDS then
                            v_data_word_idx := v_header_word_idx
                                               - C_FIRST_CELL_HIT_WORD_INDEX;
                            v_cell_idx := v_data_word_idx / C_CELL_WORDS;
                            v_cell_word_idx := v_data_word_idx mod C_CELL_WORDS;
                            v_channel := fn_lane_channel(
                                C_FALL_ACTIVE_MASK, v_cell_idx);
                            v_expected_returns := fn_expected_returns(v_channel);
                            v_header_word := m_fall_tdata(
                                32 * lane + 31 downto 32 * lane);

                            if v_cell_word_idx < C_HIT_WORDS_PER_CELL then
                                for slot in 0 to 1 loop
                                    v_hit_seq := v_cell_word_idx * 2 + slot;
                                    if v_hit_seq < C_MAX_HITS then
                                        if v_hit_seq < v_expected_returns then
                                            v_ref_idx := fn_hit_ref_index(
                                                v_fall_line_index, v_channel,
                                                v_hit_seq);
                                            v_expected_hit :=
                                                mon_expected_hit_refs(v_ref_idx);
                                            assert v_header_word(
                                                       slot * 16 + 15 downto
                                                       slot * 16) =
                                                   std_logic_vector(
                                                       v_expected_hit(15 downto 0))
                                                report "full_int: falling VDMA Hit[15:0] differs from pin reference"
                                                severity failure;
                                        else
                                            assert v_header_word(
                                                       slot * 16 + 15 downto
                                                       slot * 16) = x"0000"
                                                report "full_int: falling inactive Hit slot is not zero"
                                                severity failure;
                                        end if;
                                    end if;
                                end loop;
                            else
                                v_expected_mask := (others => '0');
                                for hit_idx in 0 to c_MAX_HITS_PER_STOP - 1 loop
                                    if hit_idx < v_expected_returns then
                                        v_expected_mask(hit_idx) := '1';
                                        v_ref_idx := fn_hit_ref_index(
                                            v_fall_line_index, v_channel,
                                            hit_idx);
                                        v_expected_hit :=
                                            mon_expected_hit_refs(v_ref_idx);
                                        assert v_header_word(hit_idx) =
                                               v_expected_hit(c_RAW_HIT_WIDTH - 1)
                                            report "full_int: falling VDMA Hit[16] differs from pin reference"
                                            severity failure;
                                        mon_last_fall_hit <= v_expected_hit;
                                    end if;
                                end loop;
                                assert v_header_word(31 downto 25) =
                                       v_expected_mask
                                    report "full_int: falling Cell hit_valid mask mismatch"
                                    severity failure;
                                assert v_header_word(24 downto 18) = "0000000"
                                    report "full_int: falling Cell slope mask is not zero"
                                    severity failure;
                                assert to_integer(unsigned(
                                           v_header_word(15 downto 12))) =
                                       v_expected_returns
                                    report "full_int: falling Cell hit_count mismatch"
                                    severity failure;
                                assert v_header_word(11 downto 10) = "00"
                                    report "full_int: falling Cell drop/error flag set"
                                    severity failure;
                                assert v_header_word(9 downto 8) =
                                       std_logic_vector(to_unsigned(
                                           v_channel / C_ER_N_STOPS, 2))
                                    report "full_int: falling Cell chip_id mismatch"
                                    severity failure;
                                v_fall_hit_check_count :=
                                    v_fall_hit_check_count + v_expected_returns;
                                mon_td_fall_hit_checks <=
                                    v_fall_hit_check_count;
                            end if;
                        end if;
                    end loop;
                    mon_td_fall_beats <= mon_td_fall_beats + 1;
                    v_fall_line_beats := v_fall_line_beats + 1;
                    if m_fall_tlast = '1' then
                        assert v_fall_line_index < v_shot_face_count
                            report "full_int: falling TLAST has no matching shot timing reference"
                            severity failure;
                        v_latency_clks := v_axis_cycle
                            - v_shot_cycle_queue(v_fall_line_index);
                        if mon_shot_to_fall_tlast_min_clks = 0
                           or v_latency_clks < mon_shot_to_fall_tlast_min_clks then
                            mon_shot_to_fall_tlast_min_clks <= v_latency_clks;
                        end if;
                        if v_latency_clks > mon_shot_to_fall_tlast_max_clks then
                            mon_shot_to_fall_tlast_max_clks <= v_latency_clks;
                        end if;
                        mon_td_fall_line_end <= mon_td_fall_line_end + 1;
                        report "full_int: falling line beats="
                             & integer'image(v_fall_line_beats)
                             & " expected="
                             & integer'image(C_EXPECT_HSIZE_FALL / (G_TDATA_WIDTH / 8))
                            severity note;
                        assert v_fall_line_beats =
                               C_EXPECT_HSIZE_FALL / (G_TDATA_WIDTH / 8)
                            report "full_int: falling line beat count diverged from HSIZE"
                            severity failure;
                        v_fall_line_beats := 0;
                        v_fall_line_index := v_fall_line_index + 1;
                        v_fall_face_header_line := false;
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

        procedure wait_md_commit(timeout_clks : positive) is
            variable v_seen_busy : boolean := false;
        begin
            for i in 0 to timeout_clks loop
                wait until rising_edge(clk);
                if md_dbg_cfg_busy = '1' then
                    v_seen_busy := true;
                elsif v_seen_busy then
                    return;
                end if;
            end loop;
            assert false
                report "full_int: motor cfg_apply did not complete"
                severity failure;
        end procedure;

        -- AXI-Lite wrappers. Handshake is owned by px_utility_pkg; these
        -- procedures only select each module's CSR port bundle.
        procedure md_wr(addr : std_logic_vector(6 downto 0);
                        val  : std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => addr,
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => md_awaddr,
                axi_awprot  => md_awprot,
                axi_awvalid => md_awvalid,
                axi_awready => md_awready,
                axi_wdata   => md_wdata,
                axi_wstrb   => md_wstrb,
                axi_wvalid  => md_wvalid,
                axi_wready  => md_wready,
                axi_bresp   => md_bresp,
                axi_bvalid  => md_bvalid,
                axi_bready  => md_bready
            );
        end procedure;

        procedure md_rd_capture(
            addr         : std_logic_vector(6 downto 0);
            msg          : string;
            variable got : out std_logic_vector(31 downto 0)
        ) is
        begin
            pl("  md_rd " & msg);
            px_axi_lite_reader(
                addr          => addr,
                val           => (others => '0'),
                comp          => '0',
                fail_on_error => '1',
                axi_aclk      => clk,
                axi_araddr    => md_araddr,
                axi_arprot    => md_arprot,
                axi_arvalid   => md_arvalid,
                axi_arready   => md_arready,
                axi_rdata     => md_rdata,
                axi_rresp     => md_rresp,
                axi_rvalid    => md_rvalid,
                axi_rready    => md_rready
            );
            got := md_rdata;
        end procedure;

        procedure lc_wr(addr : std_logic_vector(6 downto 0);
                        val  : std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => addr,
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => lc_awaddr,
                axi_awprot  => lc_awprot,
                axi_awvalid => lc_awvalid,
                axi_awready => lc_awready,
                axi_wdata   => lc_wdata,
                axi_wstrb   => lc_wstrb,
                axi_wvalid  => lc_wvalid,
                axi_wready  => lc_wready,
                axi_bresp   => lc_bresp,
                axi_bvalid  => lc_bvalid,
                axi_bready  => lc_bready
            );
        end procedure;

        procedure er_wr(addr : std_logic_vector(8 downto 0);
                        val  : std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => addr,
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => er_awaddr,
                axi_awprot  => er_awprot,
                axi_awvalid => er_awvalid,
                axi_awready => er_awready,
                axi_wdata   => er_wdata,
                axi_wstrb   => er_wstrb,
                axi_wvalid  => er_wvalid,
                axi_wready  => er_wready,
                axi_bresp   => er_bresp,
                axi_bvalid  => er_bvalid,
                axi_bready  => er_bready
            );
        end procedure;

        procedure er_rd(
            addr         : in  std_logic_vector(8 downto 0);
            variable got : out std_logic_vector(31 downto 0)
        ) is
        begin
            px_axi_lite_reader(
                addr          => addr,
                val           => (others => '0'),
                comp          => '0',
                fail_on_error => '1',
                axi_aclk      => clk,
                axi_araddr    => er_araddr,
                axi_arprot    => er_arprot,
                axi_arvalid   => er_arvalid,
                axi_arready   => er_arready,
                axi_rdata     => er_rdata,
                axi_rresp     => er_rresp,
                axi_rvalid    => er_rvalid,
                axi_rready    => er_rready
            );
            got := er_rdata;
        end procedure;

        procedure er_stage_delay(
            channel       : in natural;
            ticks         : in natural;
            variable ctl0 : inout std_logic_vector(31 downto 0)
        ) is
            variable v_stat2 : std_logic_vector(31 downto 0);
            variable v_acked : boolean := false;
        begin
            er_wr(C_ER_CTL1, std_logic_vector(to_unsigned(ticks, 32)));
            ctl0(C_ER_DELAY_INDEX_HI downto C_ER_DELAY_INDEX_LO) :=
                std_logic_vector(to_unsigned(channel, 5));
            ctl0(C_ER_DELAY_WRITE_TOGGLE_BIT) :=
                not ctl0(C_ER_DELAY_WRITE_TOGGLE_BIT);
            er_wr(C_ER_CTL0, ctl0);

            for poll in 0 to 255 loop
                er_rd(C_ER_STAT2, v_stat2);
                if v_stat2(C_ER_WRITE_ACK_BIT) =
                   ctl0(C_ER_DELAY_WRITE_TOGGLE_BIT) then
                    v_acked := true;
                    exit;
                end if;
            end loop;
            assert v_acked
                report "full_int: Echo delay write acknowledgement timeout"
                severity failure;
            assert v_stat2(C_ER_CMD_REJECT_BIT) = '0'
                report "full_int: Echo delay write was rejected"
                severity failure;
        end procedure;

        procedure er_apply_profile(
            variable ctl0 : inout std_logic_vector(31 downto 0)
        ) is
            variable v_stat2 : std_logic_vector(31 downto 0);
            variable v_acked : boolean := false;
        begin
            ctl0(C_ER_APPLY_TOGGLE_BIT) := not ctl0(C_ER_APPLY_TOGGLE_BIT);
            er_wr(C_ER_CTL0, ctl0);

            for poll in 0 to 1023 loop
                er_rd(C_ER_STAT2, v_stat2);
                if v_stat2(C_ER_APPLY_ACK_BIT) = ctl0(C_ER_APPLY_TOGGLE_BIT)
                   and v_stat2(C_ER_APPLY_PENDING_BIT) = '0' then
                    v_acked := true;
                    exit;
                end if;
            end loop;
            assert v_acked
                report "full_int: Echo delay profile apply acknowledgement timeout"
                severity failure;
            assert v_stat2(C_ER_CMD_REJECT_BIT) = '0'
                report "full_int: Echo delay profile apply was rejected"
                severity failure;
        end procedure;

        procedure td_wr(addr : std_logic_vector(8 downto 0);
                        val  : std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => addr,
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => td_awaddr,
                axi_awprot  => td_awprot,
                axi_awvalid => td_awvalid,
                axi_awready => td_awready,
                axi_wdata   => td_wdata,
                axi_wstrb   => td_wstrb,
                axi_wvalid  => td_wvalid,
                axi_wready  => td_wready,
                axi_bresp   => td_bresp,
                axi_bvalid  => td_bvalid,
                axi_bready  => td_bready
            );
        end procedure;

        procedure tp_wr(addr : std_logic_vector(6 downto 0);
                        val  : std_logic_vector(31 downto 0)) is
        begin
            px_axi_lite_writer(
                addr        => addr,
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => tp_awaddr,
                axi_awprot  => tp_awprot,
                axi_awvalid => tp_awvalid,
                axi_awready => tp_awready,
                axi_wdata   => tp_wdata,
                axi_wstrb   => tp_wstrb,
                axi_wvalid  => tp_wvalid,
                axi_wready  => tp_wready,
                axi_bresp   => tp_bresp,
                axi_bvalid  => tp_bvalid,
                axi_bready  => tp_bready
            );
        end procedure;

        -- AXI-Lite read: Pipeline CSR (7-bit addr) - for diagnostic readback
        procedure tp_rd(addr : std_logic_vector(6 downto 0);
                        msg  : string) is
        begin
            pl("  tp_rd " & msg);
            px_axi_lite_reader(
                addr          => addr,
                val           => (others => '0'),
                comp          => '0',
                fail_on_error => '0',
                axi_aclk      => clk,
                axi_araddr    => tp_araddr,
                axi_arprot    => tp_arprot,
                axi_arvalid   => tp_arvalid,
                axi_arready   => tp_arready,
                axi_rdata     => tp_rdata,
                axi_rresp     => tp_rresp,
                axi_rvalid    => tp_rvalid,
                axi_rready    => tp_rready
            );
        end procedure;

        procedure tp_rd_capture(
            addr         : std_logic_vector(6 downto 0);
            msg          : string;
            variable got : out std_logic_vector(31 downto 0)
        ) is
        begin
            pl("  tp_rd " & msg);
            px_axi_lite_reader(
                addr          => addr,
                val           => (others => '0'),
                comp          => '0',
                fail_on_error => '1',
                axi_aclk      => clk,
                axi_araddr    => tp_araddr,
                axi_arprot    => tp_arprot,
                axi_arvalid   => tp_arvalid,
                axi_arready   => tp_arready,
                axi_rdata     => tp_rdata,
                axi_rresp     => tp_rresp,
                axi_rvalid    => tp_rvalid,
                axi_rready    => tp_rready
            );
            got := tp_rdata;
        end procedure;

        procedure tp_rd_expect(
            addr         : std_logic_vector(6 downto 0);
            expected     : std_logic_vector(31 downto 0);
            msg          : string;
            variable got : out std_logic_vector(31 downto 0)
        ) is
        begin
            pl("  tp_rd " & msg);
            px_axi_lite_reader(
                addr          => addr,
                val           => expected,
                comp          => '1',
                fail_on_error => '1',
                axi_aclk      => clk,
                axi_araddr    => tp_araddr,
                axi_arprot    => tp_arprot,
                axi_arvalid   => tp_arvalid,
                axi_arready   => tp_arready,
                axi_rdata     => tp_rdata,
                axi_rresp     => tp_rresp,
                axi_rvalid    => tp_rvalid,
                axi_rready    => tp_rready
            );
            got := tp_rdata;
        end procedure;

        procedure uc_wr(
            index : natural;
            val   : std_logic_vector(31 downto 0)
        ) is
        begin
            px_axi_lite_writer(
                addr        => std_logic_vector(to_unsigned(
                    fn_ctl_byte_offset(index), uc_awaddr'length)),
                val         => val,
                axi_aclk    => clk,
                axi_awaddr  => uc_awaddr,
                axi_awprot  => uc_awprot,
                axi_awvalid => uc_awvalid,
                axi_awready => uc_awready,
                axi_wdata   => uc_wdata,
                axi_wstrb   => uc_wstrb,
                axi_wvalid  => uc_wvalid,
                axi_wready  => uc_wready,
                axi_bresp   => uc_bresp,
                axi_bvalid  => uc_bvalid,
                axi_bready  => uc_bready
            );
        end procedure;

        procedure uc_rd_capture(
            index        : natural;
            msg          : string;
            variable got : out std_logic_vector(31 downto 0)
        ) is
        begin
            pl("  uc_rd " & msg);
            px_axi_lite_reader(
                addr          => std_logic_vector(to_unsigned(
                    fn_stat_byte_offset(index), uc_araddr'length)),
                val           => (others => '0'),
                comp          => '0',
                fail_on_error => '1',
                axi_aclk      => clk,
                axi_araddr    => uc_araddr,
                axi_arprot    => uc_arprot,
                axi_arvalid   => uc_arvalid,
                axi_arready   => uc_arready,
                axi_rdata     => uc_rdata,
                axi_rresp     => uc_rresp,
                axi_rvalid    => uc_rvalid,
                axi_rready    => uc_rready
            );
            got := uc_rdata;
        end procedure;

        procedure wait_u8_match(
            signal actual : in std_logic_vector(7 downto 0);
            expected      : natural;
            timeout_clks  : positive;
            msg           : string
        ) is
        begin
            for i in 0 to timeout_clks loop
                wait until rising_edge(clk);
                if to_integer(unsigned(actual)) = expected then
                    return;
                end if;
            end loop;
            assert false report msg severity failure;
        end procedure;

        variable v_stat5 : std_logic_vector(31 downto 0) := (others => '0');
        variable v_stat6 : std_logic_vector(31 downto 0) := (others => '0');
        variable v_stat7 : std_logic_vector(31 downto 0) := (others => '0');
        variable v_md_stat0 : std_logic_vector(31 downto 0) := (others => '0');
        variable v_md_stage_toggle : std_logic := '0';
        variable v_er_ctl0 : std_logic_vector(31 downto 0) := (others => '0');
        variable v_hw_config : std_logic_vector(31 downto 0) := (others => '0');
        variable v_face_close_wait : natural := 0;
        variable v_pipeline_drain_wait : natural := 0;
        variable v_output_completion_max_clks : natural := 0;
        variable v_fire_to_output_max_clks : natural := 0;
        variable v_point_budget_margin_clks : integer := 0;
        variable v_point_budget_pass : natural range 0 to 1 := 0;
        variable v_uc_sys_ctrl : std_logic_vector(31 downto 0) :=
            (others => '0');
        variable v_uc_word : std_logic_vector(31 downto 0) :=
            (others => '0');
        variable v_uc_face_epoch : natural := 0;
        variable v_uc_echo_write_toggle : natural range 0 to 1 := 0;
        variable v_uc_echo_apply_toggle : natural range 0 to 1 := 0;
        variable v_uc_image_epoch : natural := 0;

    begin
        wait until rst_n = '1';
        wait_clk(10);

        pl("======================================================");
        pl(" full system integration TB start");
        pl(" (enc_top + motor_laser_ctrl + echo_receiver + GPX model + tdc_gpx)");
        pl("======================================================");
        if G_PRINT_BANNER then
            pl("  config :  axis/tdc clk="
               & integer'image(integer(G_AXIS_CLK_MHZ)) & "/"
               & integer'image(integer(G_TDC_CLK_MHZ))
               & "MHz  range=" & integer'image(integer(G_MAX_RANGE_M)) & "m"
               & "  sim_tgt=" & integer'image(integer(G_SIM_TARGET_M)) & "m"
               & "  tdata=" & integer'image(G_TDATA_WIDTH) & "b");
            pl("  derived:  max_range_5ns_ticks=" & integer'image(C_MAX_RANGE_5NS_TICKS)
               & "  axis/tdc_range_clks="
               & integer'image(C_MAX_RANGE_AXIS_CLKS) & "/"
               & integer'image(C_MAX_RANGE_TDC_CLKS)
               & "  sim_target_clks=" & integer'image(C_SIM_TARGET_CLKS)
               & "  laser_t0_clks=" & integer'image(C_LASER_T0_DELAY_AXIS_CLKS)
               & "  fire_done_timeout_clks="
               & integer'image(C_FIRE_DONE_TIMEOUT_AXIS_CLKS)
               & "  worst_rearm_clks=" & integer'image(C_LASER_WORST_REARM_CLKS)
               & "  shot_period=" & integer'image(C_SHOT_PERIOD_AXIS_CLKS)
               & "  step_interval=" & integer'image(C_STEP_INTERVAL)
               & "  rev_us=" & integer'image(integer(G_REV_TIME_US))
               & "  optical_step_mdeg="
               & integer'image(integer(G_OPTICAL_SHOT_INTERVAL_DEG * 1000.0))
               & "  enc_ticks_lo/hi=" & integer'image(C_ENC_TICKS_LO_LOCAL)
               & "/" & integer'image(C_ENC_TICKS_HI_LOCAL)
               & "  returns/max_hits="
               & integer'image(G_RETURNS_PER_STOP) & "/"
               & integer'image(C_MAX_HITS));
            pl("  stim   :  mode=" & G_TDC_STIM_MODE
               & "  control=" & G_CONTROL_MODE
               & "  echo=" & G_ECHO_STIM_MODE
               & "  bp_gap=" & integer'image(G_BP_TREADY_GAP)
               & "  enc_run_us=" & integer'image(integer(G_ENC_RUN_US))
               & "  chip_slope_mask=" & to_hstring(G_CHIP_SLOPE_MASK));
            pl("======================================================");
        end if;

        --------------------------------------------------------------
        -- [S1] CSR configuration (mirrors tb_laser_ctrl TEST 1 / TEST 2)
        --------------------------------------------------------------
        if C_USE_LOCAL_CSR then
        pl("[S1] motor_decoder bring-up (selected encoder + CPR + ticks + faces + APPLY)");
        md_wr(C_MD_CTL0, C_MD_CTL0_INIT);
        md_wr(C_MD_CTL1,
              std_logic_vector(to_unsigned(C_ENC_TICKS_LO_LOCAL, 32)));
        md_wr(C_MD_CTL2,
              std_logic_vector(to_unsigned(C_ENC_HI_COUNT_LOCAL, 32)));
        if G_ENCODER_SOURCE = "internal" then
            md_wr(C_MD_CTL3, C_MD_CTL3_SIM);
        else
            md_wr(C_MD_CTL3, C_MD_CTL3_PHYS);
        end if;
        wait_clk(100);
        -- per-face centers (CTL5/CTL6/CTL7 loop)
        for f in 0 to G_N_FACES - 1 loop
            case f is
                when 0 =>
                    md_wr(C_MD_CTL5,
                          std_logic_vector(to_unsigned(C_MD_FACE_CENTER_0, 32)));
                when 1 =>
                    md_wr(C_MD_CTL5,
                          std_logic_vector(to_unsigned(C_MD_FACE_CENTER_1, 32)));
                when 2 =>
                    md_wr(C_MD_CTL5,
                          std_logic_vector(to_unsigned(C_MD_FACE_CENTER_2, 32)));
                when 3 =>
                    md_wr(C_MD_CTL5,
                          std_logic_vector(to_unsigned(C_MD_FACE_CENTER_3, 32)));
                when 4 =>
                    md_wr(C_MD_CTL5,
                          std_logic_vector(to_unsigned(C_MD_FACE_CENTER_4, 32)));
                when others => null;
            end case;
            md_wr(C_MD_CTL6, std_logic_vector(to_unsigned(C_MD_FACE_HALF_ST, 32)));
            v_md_stage_toggle := not v_md_stage_toggle;
            md_wr(C_MD_CTL7, fn_md_ctl7(f, v_md_stage_toggle));
            wait_clk(20);
        end loop;
        md_wr(C_MD_CTL0, fn_md_ctl0('1'));
        wait_md_commit(C_MD_COMMIT_TIMEOUT_CLKS);

        assert md_n_faces = std_logic_vector(to_unsigned(G_N_FACES, 3))
            report "full_int: motor static n_faces sideband diverged from its generic"
            severity failure;
        md_rd_capture(C_MD_STAT0, "STAT0 n_faces", v_md_stat0);
        assert v_md_stat0(11 downto 9) = md_n_faces
            report "full_int: motor STAT0 n_faces diverged from static sideband"
            severity failure;

        -- cfg_apply may wait for a revolution boundary. Restart at position 0
        -- after it completes so the A/B guard interval is deterministic.
        if G_ENCODER_SOURCE = "internal" then
            md_wr(C_MD_CTL3, C_MD_CTL3_SWRST_SIM);
        else
            md_wr(C_MD_CTL3, C_MD_CTL3_SWRST_PHYS);
        end if;
        wait_clk(5);
        if G_ENCODER_SOURCE = "internal" then
            md_wr(C_MD_CTL3, C_MD_CTL3_SIM);
        else
            md_wr(C_MD_CTL3, C_MD_CTL3_PHYS);
        end if;
        wait_clk(30);

        if G_ECHO_STIM_MODE = "synthetic_single" then
            pl("[S1] echo_receiver CSR: synthetic STOP0 per GPX chip");
            -- Stage channel 0 of every chip through the compact indexed
            -- command window. All other channels remain zero, then one apply
            -- atomically promotes the complete staging profile.
            for chip in 0 to c_MAX_CHIPS - 1 loop
                er_stage_delay(
                    chip * C_ER_N_STOPS,
                    C_SIM_TARGET_5NS_TICKS,
                    v_er_ctl0
                );
            end loop;
            er_apply_profile(v_er_ctl0);
            v_er_ctl0(C_ER_SIM_EN_BIT) := '1';
            er_wr(C_ER_CTL0, v_er_ctl0);
        else
            pl("[S1] echo_receiver physical LVDS: all STOPs, multi-Return");
            -- Keep sim_en clear. p_physical_echo_driver drives the LVDS pins,
            -- so every Return traverses the production fast path.
            er_wr(C_ER_CTL0, x"00000000");
        end if;
        wait_clk(100);

        pl("[S1] laser_ctrl CSR: independent T0, range, and angle profile");
        lc_wr(C_LC_CTL1, C_CTL1_VAL_LOCAL);
        -- CTL2 max_roundtrip uses the shared fixed 5 ns CSR timebase.
        lc_wr(C_LC_CTL2,
              std_logic_vector(to_unsigned(C_LASER_RANGE_5NS_TICKS, 32)));
        lc_wr(C_LC_CTL3, std_logic_vector(to_unsigned(C_CTL3_VAL_LOCAL, 32)));
        -- CTL4 models command-to-fire_done (T0), never target propagation.
        -- Echo target delay is configured independently above.
        lc_wr(C_LC_CTL4, std_logic_vector(to_unsigned(
            C_LASER_T0_DELAY_5NS_TICKS, 32)));
        -- CTL5 is derived only from mirror period and optical angular step.
        -- Whether that operating point is fast enough is an outcome of this
        -- regression, not an input chosen from the range window.
        lc_wr(C_LC_CTL5, C_LC_CTL5_DERIVED);
        lc_wr(C_LC_CTL6, C_LC_CTL6_DEFAULT);
        lc_wr(C_LC_CTL7, C_LC_CTL7_LOCAL);
        -- Toggle reset while disabled. Laser admission is enabled only after
        -- tdc_gpx configuration and START have completed.
        lc_wr(C_LC_CTL0, C_LC_CTL0_RESET_LOCAL);
        wait_clk(20);

        pl("[S1] tdc_gpx chip CSR: cfg_image Reg0/Reg5/Reg6 + CTL21");
        td_wr("0" & x"14", c_GPX_DEFAULT_IMAGE(0)); -- Reg0 safe edge template
        td_wr("0" & x"28", c_GPX_DEFAULT_IMAGE(5)); -- Reg5 proven control/offset
        td_wr("0" & x"2C", x"00000004");  -- Reg6 LF threshold
        -- CTL21 must be written before START/packet_start. Otherwise max_hits=0
        -- aliases to the build maximum (7), while the test/report incorrectly
        -- claims the distance-derived C_MAX_HITS value.
        td_wr("0" & x"54", C_SCAN_TIMEOUT_VAL);
        wait_clk(20);

        pl("[S1] tdc_gpx pipeline CSR: RANGE_COLS / MAIN_CTRL");
        tp_wr(C_PIPE_RANGE_COLS, C_RANGE_COLS_VAL);
        tp_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
        wait_clk(20);

        pl("[S2] tdc_gpx CFG_WRITE pulse");
        tp_wr(C_PIPE_MAIN_CTRL, (C_MAIN_CTRL_BASE or x"80000000"));
        wait_clk(4);
        tp_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
        wait_clk(2500);

        pl("[S3] tdc_gpx START pulse");
        tp_wr(C_PIPE_MAIN_CTRL, (C_MAIN_CTRL_BASE or x"10000000"));
        wait_clk(4);
        tp_wr(C_PIPE_MAIN_CTRL, C_MAIN_CTRL_BASE);
        wait_clk(80);

        else
            pl("[S1] unified CSR: identity and reset-first transaction order");
            uc_rd_capture(C_STAT_SYS_VERSION, "STAT0 SYS_VERSION", v_uc_word);
            assert v_uc_word = C_UCSR_VERSION_WORD
                report "full_int: unified CSR version mismatch"
                severity failure;
            uc_rd_capture(C_STAT_SYS_CAPABILITY, "STAT1 SYS_CAPABILITY", v_uc_word);
            assert v_uc_word = C_UCSR_CAPABILITY_WORD
                report "full_int: unified CSR capability mismatch"
                severity failure;

            v_uc_sys_ctrl := fn_u_sys_ctrl(
                fn_bool_to_sl(G_ENCODER_SOURCE = "internal"), '0',
                fn_bool_to_sl(G_ECHO_STIM_MODE = "synthetic_single"), 0);
            uc_wr(C_CTL_SYS_CTRL, v_uc_sys_ctrl);

            -- Reset is the first transaction delimiter. Motor invalidates its
            -- accepted configuration and Echo clears its delay profile here.
            v_uc_sys_ctrl(C_SYS_CTRL_RESET_EPOCH_HI downto
                          C_SYS_CTRL_RESET_EPOCH_LO) := x"01";
            uc_wr(C_CTL_SYS_CTRL, v_uc_sys_ctrl);
            for i in 0 to 5000 loop
                wait until rising_edge(clk);
                exit when uc_stat(C_STAT_MOTOR_CFG_STATUS)(
                              C_MOTOR_CFG_STATUS_RESET_EPOCH_HI downto
                              C_MOTOR_CFG_STATUS_RESET_EPOCH_LO) = x"01"
                    and uc_laser_reset_epoch = x"01"
                    and uc_echo_reset_epoch = x"01"
                    and uc_tdc_reset_epoch = x"01";
            end loop;
            assert uc_stat(C_STAT_MOTOR_CFG_STATUS)(
                       C_MOTOR_CFG_STATUS_RESET_EPOCH_HI downto
                       C_MOTOR_CFG_STATUS_RESET_EPOCH_LO) = x"01"
                and uc_laser_reset_epoch = x"01"
                and uc_echo_reset_epoch = x"01"
                and uc_tdc_reset_epoch = x"01"
                report "full_int: unified shared reset did not complete"
                severity failure;
            wait_clk(2500);

            pl("[S1] unified Motor staging and indexed Face geometry");
            uc_wr(C_CTL_MOTOR_CFG, fn_u_motor_cfg);
            uc_wr(C_CTL_MOTOR_TICKS_LO,
                  std_logic_vector(to_unsigned(C_ENC_TICKS_LO_LOCAL, 32)));
            uc_wr(C_CTL_MOTOR_SCHED_LATENCY, fn_u_motor_sched);
            uc_wr(C_CTL_MOTOR_Z_PARAM, (others => '0'));
            for f in 0 to G_N_FACES - 1 loop
                case f is
                    when 0 => v_uc_word := fn_u_face_geometry(
                        C_MD_FACE_CENTER_0, C_MD_FACE_HALF_ST);
                    when 1 => v_uc_word := fn_u_face_geometry(
                        C_MD_FACE_CENTER_1, C_MD_FACE_HALF_ST);
                    when 2 => v_uc_word := fn_u_face_geometry(
                        C_MD_FACE_CENTER_2, C_MD_FACE_HALF_ST);
                    when 3 => v_uc_word := fn_u_face_geometry(
                        C_MD_FACE_CENTER_3, C_MD_FACE_HALF_ST);
                    when 4 => v_uc_word := fn_u_face_geometry(
                        C_MD_FACE_CENTER_4, C_MD_FACE_HALF_ST);
                    when others => v_uc_word := (others => '0');
                end case;
                uc_wr(C_CTL_MOTOR_FACE_GEOMETRY, v_uc_word);
                v_uc_face_epoch := v_uc_face_epoch + 1;
                uc_wr(C_CTL_MOTOR_FACE_INDEX,
                      fn_u_face_index(f, f, v_uc_face_epoch));
                for i in 0 to 1000 loop
                    wait until rising_edge(clk);
                    exit when to_integer(unsigned(uc_stat(
                        C_STAT_MOTOR_CFG_STATUS)(
                        C_MOTOR_CFG_STATUS_FACE_EPOCH_HI downto
                        C_MOTOR_CFG_STATUS_FACE_EPOCH_LO))) = v_uc_face_epoch;
                end loop;
                assert to_integer(unsigned(uc_stat(C_STAT_MOTOR_CFG_STATUS)(
                    C_MOTOR_CFG_STATUS_FACE_EPOCH_HI downto
                    C_MOTOR_CFG_STATUS_FACE_EPOCH_LO))) = v_uc_face_epoch
                    report "full_int: unified Motor Face write was not accepted"
                    severity failure;
            end loop;

            if G_ECHO_STIM_MODE = "synthetic_single" then
                pl("[S1] unified Echo indexed STOP0 profile");
                for chip in 0 to c_MAX_CHIPS - 1 loop
                    uc_wr(C_CTL_ECHO_DELAY_DATA,
                          std_logic_vector(to_unsigned(
                              C_SIM_TARGET_5NS_TICKS, 32)));
                    v_uc_echo_write_toggle := 1 - v_uc_echo_write_toggle;
                    uc_wr(C_CTL_ECHO_DELAY_CMD, fn_u_echo_cmd(
                        chip * C_ER_N_STOPS, v_uc_echo_write_toggle,
                        v_uc_echo_apply_toggle));
                    for i in 0 to 1000 loop
                        wait until rising_edge(clk);
                        exit when uc_stat(C_STAT_ECHO_STATUS)(
                            C_ER_WRITE_ACK_BIT) =
                            fn_nat_bit_to_sl(v_uc_echo_write_toggle);
                    end loop;
                    assert uc_stat(C_STAT_ECHO_STATUS)(C_ER_WRITE_ACK_BIT) =
                        fn_nat_bit_to_sl(v_uc_echo_write_toggle)
                        report "full_int: unified Echo delay write was not accepted"
                        severity failure;
                end loop;
                v_uc_echo_apply_toggle := 1 - v_uc_echo_apply_toggle;
                uc_wr(C_CTL_ECHO_DELAY_CMD, fn_u_echo_cmd(
                    0, v_uc_echo_write_toggle, v_uc_echo_apply_toggle));
                for i in 0 to 1000 loop
                    wait until rising_edge(clk);
                    exit when uc_stat(C_STAT_ECHO_STATUS)(C_ER_APPLY_ACK_BIT) =
                            fn_nat_bit_to_sl(v_uc_echo_apply_toggle)
                        and uc_stat(C_STAT_ECHO_STATUS)(
                            C_ER_APPLY_PENDING_BIT) = '0';
                end loop;
                assert uc_stat(C_STAT_ECHO_STATUS)(C_ER_APPLY_ACK_BIT) =
                        fn_nat_bit_to_sl(v_uc_echo_apply_toggle)
                    and uc_stat(C_STAT_ECHO_STATUS)(
                        C_ER_APPLY_PENDING_BIT) = '0'
                    report "full_int: unified Echo profile apply did not complete"
                    severity failure;
            end if;

            pl("[S1] unified Laser and TDC staging");
            uc_wr(C_CTL_LASER_FIRE_CFG, C_CTL1_VAL_LOCAL);
            uc_wr(C_CTL_LASER_ROUNDTRIP,
                  std_logic_vector(to_unsigned(C_LASER_RANGE_5NS_TICKS, 32)));
            uc_wr(C_CTL_LASER_TDC_WIDTH,
                  std_logic_vector(to_unsigned(C_CTL3_VAL_LOCAL, 32)));
            uc_wr(C_CTL_LASER_SIM_DELAY,
                  std_logic_vector(to_unsigned(
                      C_LASER_T0_DELAY_5NS_TICKS, 32)));
            uc_wr(C_CTL_LASER_SCHED0, C_LC_CTL5_DERIVED);
            uc_wr(C_CTL_LASER_SCHED1, C_LC_CTL6_DEFAULT);
            uc_wr(C_CTL_LASER_SCHED2, C_LC_CTL7_LOCAL);

            uc_wr(C_CTL_TDC_IMAGE_DATA, c_GPX_DEFAULT_IMAGE(0));
            v_uc_image_epoch := v_uc_image_epoch + 1;
            uc_wr(C_CTL_TDC_IMAGE_CMD,
                  fn_u_tdc_image_cmd(0, v_uc_image_epoch));
            wait_u8_match(uc_tdc_image_epoch, v_uc_image_epoch, 1000,
                "full_int: TDC Reg0 image write was not accepted");
            uc_wr(C_CTL_TDC_IMAGE_DATA, c_GPX_DEFAULT_IMAGE(5));
            v_uc_image_epoch := v_uc_image_epoch + 1;
            uc_wr(C_CTL_TDC_IMAGE_CMD,
                  fn_u_tdc_image_cmd(5, v_uc_image_epoch));
            wait_u8_match(uc_tdc_image_epoch, v_uc_image_epoch, 1000,
                "full_int: TDC Reg5 image write was not accepted");
            uc_wr(C_CTL_TDC_IMAGE_DATA, x"00000004");
            v_uc_image_epoch := v_uc_image_epoch + 1;
            uc_wr(C_CTL_TDC_IMAGE_CMD,
                  fn_u_tdc_image_cmd(6, v_uc_image_epoch));
            wait_u8_match(uc_tdc_image_epoch, v_uc_image_epoch, 1000,
                "full_int: TDC Reg6 image write was not accepted");
            assert uc_tdc_image_reject = '0'
                report "full_int: valid TDC image write was rejected"
                severity failure;

            uc_wr(C_CTL_TDC_BUS_TIMING, c_INIT_BUS_TIMING);
            uc_wr(C_CTL_TDC_START_OFFSET, c_INIT_START_OFF1);
            uc_wr(C_CTL_TDC_CFG_REG7, c_INIT_CFG_REG7);
            uc_wr(C_CTL_TDC_SCAN_CFG, C_SCAN_TIMEOUT_VAL);
            uc_wr(C_CTL_TDC_PIPELINE_MAIN, C_MAIN_CTRL_BASE);
            uc_wr(C_CTL_TDC_RANGE_COLS, C_RANGE_COLS_VAL);

            pl("[S2] unified shared CFG_EPOCH=1");
            uc_wr(C_CTL_SYS_CFG_APPLY, x"00000001");
            for i in 0 to C_MD_COMMIT_TIMEOUT_CLKS + 5000 loop
                wait until rising_edge(clk);
                exit when uc_stat(C_STAT_MOTOR_CFG_STATUS)(
                              C_MOTOR_CFG_STATUS_CFG_EPOCH_HI downto
                              C_MOTOR_CFG_STATUS_CFG_EPOCH_LO) = x"01"
                    and uc_stat(C_STAT_MOTOR_CFG_STATUS)(
                              C_MOTOR_CFG_STATUS_VALID_BIT) = '1'
                    and uc_laser_cfg_epoch = x"01"
                    and uc_laser_cfg_valid = '1'
                    and uc_tdc_cfg_epoch = x"01"
                    and uc_tdc_cfg_valid = '1'
                    and uc_laser_cfg_busy = '0'
                    and uc_tdc_cfg_busy = '0';
            end loop;
            assert uc_stat(C_STAT_MOTOR_CFG_STATUS)(
                       C_MOTOR_CFG_STATUS_CFG_EPOCH_HI downto
                       C_MOTOR_CFG_STATUS_CFG_EPOCH_LO) = x"01"
                and uc_laser_cfg_epoch = x"01"
                and uc_tdc_cfg_epoch = x"01"
                and uc_laser_cfg_reject = '0'
                and uc_tdc_cfg_reject = '0'
                report "full_int: unified shared configuration did not commit"
                severity failure;
            wait_clk(2500);

            pl("[S3] unified TDC START command epoch=1");
            uc_wr(C_CTL_TDC_AUX_CMD, fn_u_tdc_aux(C_TDC_CMD_START, 1));
            wait_u8_match(uc_tdc_cmd_epoch, 1, 2000,
                "full_int: unified TDC START was not accepted");
            assert uc_tdc_cmd_reject = '0'
                report "full_int: unified TDC START was rejected"
                severity failure;
            wait_clk(80);
        end if;

        --------------------------------------------------------------
        -- [S4] Start encoder motion and let chain run for long time
        --   In "direct" mode the TB also fires two shots via tb_shot_start
        --   so the TDC path gets triggered regardless of laser_ctrl.
        --------------------------------------------------------------
        if G_TDC_STIM_MODE = "direct" then
            pl("[S4] DIRECT mode: firing 2 TB-generated shots into TDC");
            -- Fire 2 shots. The p_pd / p_irflag processes key off lc_fire_pulse
            -- which is zero in direct mode, so we manually drive IrFlag here.
            for shot in 0 to 1 loop
                tb_shot_start <= '1';
                wait_clk(1);
                tb_shot_start <= '0';
                -- FIFO preload + IrFlag generation are handled by p_pd and
                -- p_irflag (both keyed on td_shot_start_mux). Leaving those
                -- as the sole drivers of i_tdc_irflag avoids a multi-driver
                -- 'U' resolution bug that blocked IrFlag in lc mode.
                wait_clk(1200);  -- cover capture + IrFlag + drain + ALU tail
                if shot = 0 then wait_clk(C_SHOT_PERIOD_AXIS_CLKS); end if;
            end loop;
            wait_clk(2000);
            tb_stop_tdc <= '1'; wait_clk(2); tb_stop_tdc <= '0';
            wait_clk(500);
        else
            pl("[S4] enable motor_laser_ctrl after TDC START");
            if C_USE_LOCAL_CSR then
                lc_wr(C_LC_CTL0, C_LC_CTL0_ENABLE_LOCAL);
            else
                v_uc_sys_ctrl(C_SYS_CTRL_LASER_EN_BIT) := '1';
                uc_wr(C_CTL_SYS_CTRL, v_uc_sys_ctrl);
            end if;
            wait_clk(20);
            if G_ENCODER_SOURCE = "external" then
                pl("[S4] enable external encoder motion (physical A/B/Z path)");
                enc_run <= '1';
            else
                pl("[S4] observe motor_decoder internal encoder motion");
            end if;
            wait_clk(C_ENC_RUN_CLKS);
            pl("[S5] close at the next complete face boundary");
            v_face_close_wait := 0;
            while (mon_lc_start_cnt < G_COLS_PER_FACE or
                   (mon_lc_start_cnt mod G_COLS_PER_FACE) /= 0)
                  and v_face_close_wait < C_FACE_CLOSE_TIMEOUT_CLKS loop
                wait_clk(1);
                v_face_close_wait := v_face_close_wait + 1;
            end loop;
            assert mon_lc_start_cnt >= G_COLS_PER_FACE and
                   (mon_lc_start_cnt mod G_COLS_PER_FACE) = 0
                report "full_int: timed out waiting for a complete face boundary"
                severity failure;
        end if;

        -- Stop the encoder source at a complete face boundary, then let the
        -- already accepted shot drain before disabling Laser admission. This
        -- preserves the wrapper's private always-ready AXIS contract.
        pl("[S5] stop encoder source and drain in-flight shot");
        if G_ENCODER_SOURCE = "external" then
            enc_run <= '0';
        else
            -- Physical inputs are tied low in internal-source mode, so this
            -- runtime source switch stops new position events without adding
            -- a test-only gate inside motor_laser_ctrl_top.
            if C_USE_LOCAL_CSR then
                md_wr(C_MD_CTL3, C_MD_CTL3_PHYS);
            else
                v_uc_sys_ctrl(C_SYS_CTRL_MOTOR_SIM_EN_BIT) := '0';
                uc_wr(C_CTL_SYS_CTRL, v_uc_sys_ctrl);
            end if;
        end if;
        v_pipeline_drain_wait := 0;
        while (mon_lc_stop_cnt < mon_lc_start_cnt or
               mon_lc_m_tlast < mon_lc_start_cnt or
               mon_td_rise_line_end < mon_lc_start_cnt or
               mon_td_fall_line_end < mon_lc_start_cnt) and
              v_pipeline_drain_wait < C_PIPE_DRAIN_TIMEOUT_CLKS loop
            wait_clk(1);
            v_pipeline_drain_wait := v_pipeline_drain_wait + 1;
        end loop;
        assert mon_lc_stop_cnt = mon_lc_start_cnt and
               mon_lc_m_tlast = mon_lc_start_cnt and
               mon_td_rise_line_end = mon_lc_start_cnt and
               mon_td_fall_line_end = mon_lc_start_cnt
            report "full_int: timed out draining accepted shot outputs"
            severity failure;
        if C_USE_LOCAL_CSR then
            lc_wr(C_LC_CTL0, C_LC_CTL0_RESET_LOCAL);
        else
            v_uc_sys_ctrl(C_SYS_CTRL_LASER_EN_BIT) := '0';
            uc_wr(C_CTL_SYS_CTRL, v_uc_sys_ctrl);
        end if;
        wait_clk(20);

        --------------------------------------------------------------
        -- [S6] Diagnostic readback: pipeline CSR STAT5/STAT6/STAT7
        --   STAT5 (0x54) STATUS    : [0] busy [1] overrun [2] err_fatal
        --                             [7:4] chip_err [11:8] drain_to [15:12] seq_err
        --   STAT6 (0x58) STATUS_EXT: [0] err_read_timeout [1] reg_rejected
        --                             [2] reg_zero_mask [3/4] shot_flush_drop r/f
        --                             [5/6] hdr_drain_timeout r/f [7] err_frame_wait
        --   STAT7 (0x5C) STATUS_EXT2: [3:0] reg_timeout_mask [7:4] stop_id_err
        --                              [10:8] run_timeout_cause_last
        --   Healthy while the GPX run remains enabled:
        --     STAT5 = 0x00000001 (run busy only)
        --     STAT6[31:28] = active-chip run_drain_complete_mask; with 1111,
        --                     0xF0000000 is completion evidence, not an error.
        --     STAT7 = 0x00000000.
        --------------------------------------------------------------
        if C_USE_LOCAL_CSR then
            tp_rd_expect("1010100", C_EXPECT_STAT5, "STAT5 STATUS", v_stat5);
            tp_rd_expect("1011000", C_EXPECT_STAT6, "STAT6 STATUS_EXT", v_stat6);
            tp_rd_expect("1011100", C_EXPECT_STAT7, "STAT7 STATUS_EXT2", v_stat7);
            tp_rd_capture(C_PIPE_HW_CONFIG, "STAT1 HW_CONFIG", v_hw_config);
            assert v_hw_config(31 downto 29) = md_n_faces
                report "full_int: TDC HW_CONFIG n_faces diverged from motor static sideband"
                severity failure;
            tp_rd("1010000", "STAT4 MAX_HSIZE");
        else
            uc_rd_capture(C_STAT_TDC_PIPELINE_STATUS,
                          "STAT27 TDC_PIPELINE_STATUS", v_uc_word);
            v_stat5 := (others => '0');
            v_stat5(15 downto 0) := v_uc_word(15 downto 0);
            assert v_stat5 = C_EXPECT_STAT5
                report "full_int: unified TDC pipeline status mismatch"
                severity failure;
            uc_rd_capture(C_STAT_TDC_STATUS_EXT,
                          "STAT28 TDC_STATUS_EXT", v_stat6);
            assert v_stat6 = C_EXPECT_STAT6
                report "full_int: unified TDC status-ext mismatch"
                severity failure;
            uc_rd_capture(C_STAT_TDC_STATUS_EXT2,
                          "STAT29 TDC_STATUS_EXT2", v_stat7);
            assert v_stat7 = C_EXPECT_STAT7
                report "full_int: unified TDC status-ext2 mismatch"
                severity failure;
            uc_rd_capture(C_STAT_MOTOR_STATUS,
                          "STAT6 MOTOR_STATUS", v_hw_config);
            assert v_hw_config(C_MOTOR_STATUS_N_FACES_HI downto
                               C_MOTOR_STATUS_N_FACES_LO) = md_n_faces
                report "full_int: unified Motor n_faces readback mismatch"
                severity failure;
            uc_rd_capture(C_STAT_TDC_MAX_HSIZE,
                          "STAT5 TDC_MAX_HSIZE", v_uc_word);
            assert v_uc_word = C_U_TDC_MAX_HSIZE_WORD
                report "full_int: unified static TDC max HSIZE mismatch"
                severity failure;
            uc_rd_capture(C_STAT_TDC_IMAGE_SELECTED_DATA,
                          "STAT30 TDC_IMAGE_SELECTED_DATA", v_uc_word);
            assert v_uc_word = x"00000004"
                report "full_int: unified selected TDC image readback mismatch"
                severity failure;
            uc_rd_capture(C_STAT_SYS_ADAPTER_STATE,
                          "STAT31 SYS_ADAPTER_STATE", v_uc_word);
            assert v_uc_word(C_SYS_STATE_ALL_CFG_ACCEPTED_BIT) = '1'
                and v_uc_word(C_SYS_STATE_ALL_RESET_ACCEPTED_BIT) = '1'
                and v_uc_word(C_SYS_STATE_ANY_BUSY_BIT) = '0'
                and v_uc_word(C_SYS_STATE_ANY_REJECT_BIT) = '0'
                report "full_int: unified adapter summary is not clean"
                severity failure;
        end if;

        v_output_completion_max_clks := fn_max_nat(
            mon_shot_to_rise_tlast_max_clks,
            mon_shot_to_fall_tlast_max_clks);
        -- Use the configured command-to-T0 watchdog and the requested angular
        -- lattice. Accepted-shot spacing can grow after a skipped grid point
        -- and must never manufacture a false positive margin.
        v_fire_to_output_max_clks := C_FIRE_DONE_TIMEOUT_AXIS_CLKS
            + v_output_completion_max_clks;
        v_point_budget_margin_clks := integer(C_SHOT_PERIOD_AXIS_CLKS)
            - integer(v_fire_to_output_max_clks);
        if v_point_budget_margin_clks >= 0 then
            v_point_budget_pass := 1;
        else
            v_point_budget_pass := 0;
        end if;

        --------------------------------------------------------------
        -- [S7] Summary
        --------------------------------------------------------------
        pl("======================================================");
        pl(" full integration TB summary:");
        pl("  source encoder     = " & G_ENCODER_SOURCE);
        pl("  md     virt/decoded position changes = "
           & integer'image(mon_md_virt_pos_changes) & "/"
           & integer'image(mon_md_dec_count_changes));
        pl("  md     active/cfg_busy cycles = "
           & integer'image(mon_md_active_cycles) & "/"
           & integer'image(mon_md_cfg_busy_cycles));
        pl("  md     final sim/rst/busy = "
           & std_logic'image(md_dbg_sim_en) & "/"
           & std_logic'image(md_dbg_rst_n_int) & "/"
           & std_logic'image(md_dbg_cfg_busy));
        pl("  md     final virt/decoded/face/active = "
           & integer'image(to_integer(unsigned(md_dbg_virt_pos))) & "/"
           & integer'image(to_integer(unsigned(md_dbg_dec_count))) & "/"
           & integer'image(to_integer(unsigned(md_dbg_face_index))) & "/"
           & integer'image(to_integer(unsigned(md_dbg_active))));
        pl("  lc     fire pulses  = " & integer'image(mon_lc_fire_cnt));
        pl("  lc     start_tdc    = " & integer'image(mon_lc_start_cnt));
        pl("  lc     stop_tdc     = " & integer'image(mon_lc_stop_cnt));
        pl("  lc     result beats = " & integer'image(mon_lc_m_beats));
        pl("  lc     result tlast = " & integer'image(mon_lc_m_tlast));
        pl("  lc     shot interval min/max clocks = "
           & integer'image(mon_shot_interval_min_clks) & "/"
           & integer'image(mon_shot_interval_max_clks));
        pl("  lc     fire_done latency min/max clocks = "
           & integer'image(mon_fire_done_latency_min_clks) & "/"
           & integer'image(mon_fire_done_latency_max_clks));
        pl("  lc     range wait min/max clocks = "
           & integer'image(mon_range_wait_min_clks) & "/"
           & integer'image(mon_range_wait_max_clks));
        pl("  lc     schedule overrun = "
           & integer'image(mon_schedule_overrun));
        pl("  er     STOP high cycles = " & integer'image(mon_er_stop_high_cycles));
        pl("  er     stop_evt beats = " & integer'image(mon_er_stop_beats));
        pl("  er     fire_count beats = " & integer'image(mon_er_fire_count_beats));
        pl("  td     rise VDMA beats = " & integer'image(mon_td_rise_beats)
           & "  line TLAST = " & integer'image(mon_td_rise_line_end));
        pl("  td     fall VDMA beats = " & integer'image(mon_td_fall_beats)
           & "  line TLAST = " & integer'image(mon_td_fall_line_end));
        pl("  td     shot->rise TLAST min/max clocks = "
           & integer'image(mon_shot_to_rise_tlast_min_clks) & "/"
           & integer'image(mon_shot_to_rise_tlast_max_clks));
        pl("  td     shot->fall TLAST min/max clocks = "
           & integer'image(mon_shot_to_fall_tlast_min_clks) & "/"
           & integer'image(mon_shot_to_fall_tlast_max_clks));
        pl("  budget fire->output max / point margin clocks = "
           & integer'image(v_fire_to_output_max_clks) & "/"
           & integer'image(v_point_budget_margin_clks));
        pl("  td     rise/fall header face checks = "
           & integer'image(mon_td_rise_header_checks) & "/"
           & integer'image(mon_td_fall_header_checks));
        pl("  td     rise/fall 17-bit Hit checks = "
           & integer'image(mon_td_rise_hit_checks) & "/"
           & integer'image(mon_td_fall_hit_checks));
        pl("  td     raw 28-bit I-Mode bus checks = "
           & integer'image(mon_i_mode_bus_checks));
        pl("  td     expected raw pin hits = "
           & integer'image(mon_expected_shots * C_EXPECT_RAW_WORDS_PER_SHOT));
        pl("  td     last expected/rise/fall Hit = "
           & integer'image(to_integer(mon_last_expected_hit)) & "/"
           & integer'image(to_integer(mon_last_rise_hit)) & "/"
           & integer'image(to_integer(mon_last_fall_hit)));
        pl("  td     rise/fall face seen masks = "
           & to_hstring(mon_td_rise_face_seen) & "/"
           & to_hstring(mon_td_fall_face_seen));
        pl("  -- DUT internal (VHDL-2008 alias) --");
        pl("  td     cmd_start rising     = " & integer'image(mon_cmd_start_cnt));
        pl("  td     cmd_start_accepted   = " & integer'image(mon_cmd_accept_cnt));
        pl("  td     chip_busy any cycles = " & integer'image(mon_chip_busy_any));
        pl("  td     shot_start_gated     = " & integer'image(mon_shot_gated_cnt));
        pl("  td     shot_per_chip any    = " & integer'image(mon_shot_chip_any));
        pl("  td     cfg_rejected         = " & integer'image(mon_cfg_rejected));
        pl("  td     pipeline_abort       = " & integer'image(mon_pipe_abort));
        pl("======================================================");

        report "RTL_RESULT encoder_source=" & G_ENCODER_SOURCE
             & " control_mode=" & G_CONTROL_MODE
             & " axis_clk_mhz=" & integer'image(integer(G_AXIS_CLK_MHZ))
             & " tdc_clk_mhz=" & integer'image(integer(G_TDC_CLK_MHZ))
             & " output_width=" & integer'image(G_TDATA_WIDTH)
             & " max_range_5ns_ticks=" & integer'image(C_MAX_RANGE_5NS_TICKS)
             & " max_range_axis_clks=" & integer'image(C_MAX_RANGE_AXIS_CLKS)
             & " max_range_tdc_clks=" & integer'image(C_MAX_RANGE_TDC_CLKS)
             & " tdc_drain_margin_ns=" & integer'image(G_TDC_DRAIN_MARGIN_TIME_NS)
             & " tdc_drain_margin_clks=" & integer'image(C_DRAIN_MARGIN_TDC_CLKS)
             & " max_scan_5ns_ticks=" & integer'image(C_MAX_SCAN_5NS_TICKS)
             & " max_hits=" & integer'image(C_MAX_HITS)
             & " returns_per_stop=" & integer'image(G_RETURNS_PER_STOP)
             & " echo_stim_mode=" & G_ECHO_STIM_MODE
             & " revolution_period_ns="
             & integer'image(integer(G_REV_TIME_US * 1000.0))
             & " motor_rpm=" & integer'image(C_MOTOR_RPM_LOCAL)
             & " operating_motor_rpm="
             & integer'image(integer(round(C_OPERATING_MOTOR_RPM)))
             & " horizontal_resolution_mdeg="
             & integer'image(integer(round(
                 C_HORIZONTAL_RESOLUTION_DEG * 1000.0)))
             & " operating_revolution_period_ns="
             & integer'image(integer(round(C_OPERATING_REV_TIME_US * 1000.0)))
             & " operating_point_interval_clks="
             & integer'image(C_OPERATING_POINT_INTERVAL_CLKS)
             & " optical_shot_interval_mdeg="
             & integer'image(integer(G_OPTICAL_SHOT_INTERVAL_DEG * 1000.0))
             & " mechanical_shot_interval_mdeg="
             & integer'image(integer(G_OPTICAL_SHOT_INTERVAL_DEG * 500.0))
             & " planned_shot_interval_clks="
             & integer'image(C_SHOT_PERIOD_AXIS_CLKS)
             & " measured_shot_interval_min_clks="
             & integer'image(mon_shot_interval_min_clks)
             & " measured_shot_interval_max_clks="
             & integer'image(mon_shot_interval_max_clks)
             & " fire_done_delay_clks="
             & integer'image(mon_fire_done_latency_max_clks)
             & " laser_t0_delay_5ns_ticks="
             & integer'image(C_LASER_T0_DELAY_5NS_TICKS)
             & " laser_t0_delay_axis_clks="
             & integer'image(C_LASER_T0_DELAY_AXIS_CLKS)
             & " fire_done_timeout_5ns_ticks="
             & integer'image(C_FIRE_DONE_TIMEOUT_5NS_TICKS)
             & " fire_done_timeout_axis_clks="
             & integer'image(C_FIRE_DONE_TIMEOUT_AXIS_CLKS)
             & " worst_rearm_clks="
             & integer'image(C_LASER_WORST_REARM_CLKS)
             & " range_wait_min_clks="
             & integer'image(mon_range_wait_min_clks)
             & " range_wait_max_clks="
             & integer'image(mon_range_wait_max_clks)
             & " shot_to_rise_tlast_min_clks="
             & integer'image(mon_shot_to_rise_tlast_min_clks)
             & " shot_to_rise_tlast_max_clks="
             & integer'image(mon_shot_to_rise_tlast_max_clks)
             & " shot_to_fall_tlast_min_clks="
             & integer'image(mon_shot_to_fall_tlast_min_clks)
             & " shot_to_fall_tlast_max_clks="
             & integer'image(mon_shot_to_fall_tlast_max_clks)
             & " fire_to_output_max_clks="
             & integer'image(v_fire_to_output_max_clks)
             & " point_budget_margin_clks="
             & integer'image(v_point_budget_margin_clks)
             & " point_budget_pass="
             & integer'image(v_point_budget_pass)
             & " rearm_guard_5ns_ticks="
             & integer'image(G_REARM_GUARD_5NS_TICKS)
             & " schedule_overrun=" & integer'image(mon_schedule_overrun)
             & " falling_enable=" & integer'image(fn_bool_to_nat(C_FALLING_ENABLE))
             & " stops_per_chip=" & integer'image(G_STOPS_PER_CHIP)
             & " cols_per_face=" & integer'image(G_COLS_PER_FACE)
             & " faces_per_frame=" & integer'image(G_N_FACES)
             & " present_chip_mask=" & to_hstring(c_ALL_CHIPS_MASK)
             & " rise_capability_mask=" & to_hstring(c_DEFAULT_RISE_CHIP_MASK)
             & " fall_capability_mask=" & to_hstring(c_DEFAULT_FALL_CHIP_MASK)
             & " active_chip_mask=" & to_hstring(G_ACTIVE_CHIP_MASK)
             & " chip_slope_mask=" & to_hstring(G_CHIP_SLOPE_MASK)
             & " runtime_rise_chip_mask=" & to_hstring(C_RISE_ACTIVE_MASK)
             & " runtime_fall_chip_mask=" & to_hstring(C_FALL_ACTIVE_MASK)
             & " encoder_ticks_lo=" & integer'image(C_ENC_TICKS_LO_LOCAL)
             & " encoder_ticks_hi=" & integer'image(C_ENC_TICKS_HI_LOCAL)
             & " encoder_hi_count=" & integer'image(C_ENC_HI_COUNT_LOCAL)
             & " step_interval=" & integer'image(C_STEP_INTERVAL)
             & " vdma_hsize_rise=" & integer'image(to_integer(td_vdma_hsize_rise))
             & " vdma_hsize_fall=" & integer'image(to_integer(td_vdma_hsize_fall))
             & " vdma_vsize=" & integer'image(to_integer(td_vdma_vsize))
             & " md_virt_changes=" & integer'image(mon_md_virt_pos_changes)
             & " md_dec_changes=" & integer'image(mon_md_dec_count_changes)
             & " md_active_cycles=" & integer'image(mon_md_active_cycles)
             & " md_cfg_busy_cycles=" & integer'image(mon_md_cfg_busy_cycles)
             & " laser_fire=" & integer'image(mon_lc_fire_cnt)
             & " laser_start=" & integer'image(mon_lc_start_cnt)
             & " laser_stop=" & integer'image(mon_lc_stop_cnt)
             & " laser_result_beats=" & integer'image(mon_lc_m_beats)
             & " laser_result_tlast=" & integer'image(mon_lc_m_tlast)
             & " echo_stop_high_cycles=" & integer'image(mon_er_stop_high_cycles)
             & " echo_stop_beats=" & integer'image(mon_er_stop_beats)
             & " stat5=" & to_hstring(v_stat5)
             & " stat6=" & to_hstring(v_stat6)
             & " stat7=" & to_hstring(v_stat7)
             & " stale_alias_gpx_shots=" & integer'image(mon_shot_gated_cnt)
             & " rise_beats=" & integer'image(mon_td_rise_beats)
             & " rise_tlast=" & integer'image(mon_td_rise_line_end)
             & " fall_beats=" & integer'image(mon_td_fall_beats)
             & " fall_tlast=" & integer'image(mon_td_fall_line_end)
             & " i_mode_hit_refs="
             & integer'image(mon_expected_shots * C_EXPECT_RISE_HITS_PER_SHOT)
             & " i_mode_raw_hit_refs="
             & integer'image(mon_expected_shots * C_EXPECT_RAW_WORDS_PER_SHOT)
             & " i_mode_words_per_shot="
             & integer'image(C_EXPECT_RAW_WORDS_PER_SHOT)
             & " i_mode_rise_checks=" & integer'image(mon_td_rise_hit_checks)
             & " i_mode_fall_checks=" & integer'image(mon_td_fall_hit_checks)
             & " i_mode_bus_checks=" & integer'image(mon_i_mode_bus_checks)
             & " i_mode_expected_hit=" & integer'image(to_integer(mon_last_expected_hit))
             & " i_mode_rise_hit=" & integer'image(to_integer(mon_last_rise_hit))
             & " i_mode_fall_hit=" & integer'image(to_integer(mon_last_fall_hit))
             & " cfg_rejected=" & integer'image(mon_cfg_rejected)
             & " pipeline_abort=" & integer'image(mon_pipe_abort)
            severity note;

        assert mon_md_dec_count_changes > 0
            report "full_int: no motor decoder position activity"
            severity failure;
        if G_ENCODER_SOURCE = "internal" then
            assert mon_lc_fire_cnt = 0 and mon_lc_start_cnt > 0
                report "full_int: SIM encoder must suppress physical fire while producing start_tdc"
                severity failure;
        else
            assert mon_lc_fire_cnt > 0 and mon_lc_start_cnt > 0
                report "full_int: physical encoder path did not produce fire/start events"
                severity failure;
        end if;
        assert mon_lc_m_beats > 0 and mon_lc_m_tlast > 0
            report "full_int: laser result stream did not complete a shot" severity failure;
        assert mon_lc_start_cnt = mon_lc_stop_cnt
               and mon_lc_stop_cnt = mon_lc_m_tlast
            report "full_int: laser start/stop/result-TLAST shot accounting mismatch"
            severity failure;
        assert mon_er_stop_high_cycles > 0 and mon_er_stop_beats > 0
            report "full_int: no Echo-to-STOP diagnostic activity" severity failure;
        assert mon_td_rise_beats > 0 and mon_td_fall_beats > 0
            report "full_int: no GPX rising/falling output activity"
            severity failure;
        assert mon_td_rise_line_end > 0 and mon_td_fall_line_end > 0
            report "full_int: VDMA streams did not close a line"
            severity failure;
        assert mon_td_rise_line_end = mon_lc_start_cnt
               and mon_td_fall_line_end = mon_lc_start_cnt
            report "full_int: accepted shots and VDMA line TLAST counts differ"
            severity failure;
        assert mon_expected_shots = mon_lc_start_cnt
            report "full_int: GPX START/Echo STOP reference count differs from accepted shots"
            severity failure;
        assert mon_td_rise_hit_checks =
                   mon_td_rise_line_end * C_EXPECT_RISE_HITS_PER_SHOT
               and mon_td_fall_hit_checks =
                   mon_td_fall_line_end * C_EXPECT_FALL_HITS_PER_SHOT
            report "full_int: not every VDMA Cell Return passed the exact 17-bit check"
            severity failure;
        assert mon_i_mode_bus_checks =
               mon_expected_shots * C_EXPECT_RAW_WORDS_PER_SHOT
            report "full_int: not every active GPX chip read passed the 28-bit I-Mode field check"
            severity failure;
        assert mon_last_rise_hit = mon_last_expected_hit
               and mon_last_fall_hit = mon_last_expected_hit
            report "full_int: final 17-bit Hit differs across GPX reference and VDMA lanes"
            severity failure;
        for chip in 0 to c_MAX_CHIPS - 1 loop
            assert to_integer(unsigned(gpx_capture_count(
                       (chip + 1) * 16 - 1 downto chip * 16))) =
                   mon_expected_shots
                   * fn_lane_return_words_per_shot(
                       std_logic_vector(to_unsigned(2 ** chip, c_MAX_CHIPS))
                       and G_ACTIVE_CHIP_MASK)
                report "full_int: external GPX chip capture count mismatch"
                severity failure;
        end loop;
        assert to_integer(td_vdma_vsize) = G_COLS_PER_FACE
            report "full_int: VDMA VSIZE diverged from CSR cols_per_face"
            severity failure;
        assert to_integer(td_vdma_hsize_rise) = C_EXPECT_HSIZE_RISE
            report "full_int: rising VDMA HSIZE diverged from CTL21 max_hits geometry"
            severity failure;
        assert to_integer(td_vdma_hsize_fall) = C_EXPECT_HSIZE_FALL
            report "full_int: falling VDMA HSIZE diverged from CTL21 max_hits geometry"
            severity failure;
        assert mon_td_rise_beats =
               mon_td_rise_line_end * C_EXPECT_HSIZE_RISE / (G_TDATA_WIDTH / 8)
            report "full_int: rising beat count diverged from HSIZE/TLAST contract"
            severity failure;
        assert mon_td_fall_beats =
               mon_td_fall_line_end * C_EXPECT_HSIZE_FALL / (G_TDATA_WIDTH / 8)
            report "full_int: falling beat count diverged from HSIZE/TLAST contract"
            severity failure;
        assert mon_td_rise_header_checks * G_COLS_PER_FACE =
                   mon_td_rise_line_end
               and mon_td_fall_header_checks * G_COLS_PER_FACE =
                   mon_td_fall_line_end
            report "full_int: checked face headers do not match VSIZE line groups"
            severity failure;
        assert mon_td_rise_face_seen = fn_face_mask(G_N_FACES)
               and mon_td_fall_face_seen = fn_face_mask(G_N_FACES)
            report "full_int: VDMA headers did not cover every motor-owned face"
            severity failure;
        assert mon_cfg_rejected = 0 and mon_pipe_abort = 0
            report "full_int: configuration reject or pipeline abort observed"
            severity failure;
        assert mon_schedule_overrun = 0
            report "full_int: mirror angular grid outran the Laser/TDC shot path"
            severity failure;
        assert mon_lc_start_cnt < 2 or mon_shot_interval_min_clks > 0
            report "full_int: consecutive Shot interval was not measured"
            severity failure;
        assert v_point_budget_pass = 1
            report "full_int: fire-to-VDMA completion exceeded the minimum laser point interval"
            severity failure;

        report "SYSTEM_INTEGRATION_SMOKE_PASS" severity note;

        sim_done <= true;
        wait;
    end process p_stim;

    -- Watchdog
    p_wdog : process
    begin
        wait for 10 ms;
        if not sim_done then
            report "tb_tdc_gpx_full_int: watchdog timeout (10 ms)" severity failure;
        end if;
        wait;
    end process p_wdog;

end architecture sim;
