-- =============================================================================
-- tdc_gpx_cfg_pkg.vhd
-- TDC-GPX Controller - CSR register map constants and init values
-- =============================================================================
--
-- Purpose:
--   Defines CSR register address offsets, bitfield indices, and init values.
--   All offsets are 32-bit aligned (AXI-Lite standard).
--
--   Address layout (9-bit, my_axil_csr32 compatible):
--     CTL (R/W): 0x00~0x7C  — 32 registers (addr[7]=0)
--     STAT (R/O): 0x80~0xA8 — 11 registers (addr[7]=1)
--
--   CTL[0]      = MAIN_CTRL (packed control + COMMAND[31:28])
--   CTL[1]      = BUS_TIMING
--   CTL[2]      = RANGE_COLS (max_range_clks + cols_per_face)
--   CTL[3]      = START_OFF1
--   CTL[4]      = CFG_REG7
--   CTL[5..20]  = cfg_image[0..15]
--   CTL[21..31] = reserved
--
--   TDC-GPX chip register bitfield constants (for cfg_image override):
--     Reg0: TRiseEn[0:8], TFallEn[0:8] (edge sensitivity per stop)
--     Reg5: StartOff1[17:0]
--     Reg6: LF threshold[7:0] (Fill value for burst drain)
--     Reg7: HSDiv, RefClkDiv, MTimer (via CTL4 override)
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

package tdc_gpx_cfg_pkg is

    -- =========================================================================
    -- CSR address width
    -- =========================================================================
    constant c_CSR_ADDR_WIDTH       : natural := 9;     -- 512-byte block

    -- =========================================================================
    -- Number of CTL / STAT registers (IP configuration)
    -- =========================================================================
    constant c_NUM_CTL_REGS         : natural := 32;
    constant c_NUM_STAT_REGS        : natural := 11;

    -- =========================================================================
    -- Number of active CTL registers (CDC'd to i_clk domain)
    -- =========================================================================
    constant c_NUM_CTL_CDC          : natural := 5;     -- CTL0~4

    -- =========================================================================
    -- CTL0: MAIN_CTRL (packed control fields + COMMAND)
    -- =========================================================================
    constant c_ADDR_MAIN_CTRL        : natural := 16#00#;   -- CTL0

    -- CTL0 bitfield positions
    constant c_MC_ACTIVE_MASK_HI     : natural := 3;    -- [3:0]   active_chip_mask
    constant c_MC_ACTIVE_MASK_LO     : natural := 0;
    constant c_MC_PACKET_SCOPE       : natural := 4;    -- [4]     packet_scope   (HEADER-ONLY)
    constant c_MC_HIT_STORE_HI       : natural := 6;    -- [6:5]   hit_store_mode (HEADER-ONLY)
    constant c_MC_HIT_STORE_LO       : natural := 5;
    constant c_MC_DIST_SCALE_HI      : natural := 9;    -- [9:7]   dist_scale     (HEADER-ONLY)
    constant c_MC_DIST_SCALE_LO      : natural := 7;
    constant c_MC_DRAIN_MODE         : natural := 10;   -- [10]    drain_mode
    constant c_MC_PIPELINE_EN        : natural := 11;   -- [11]    pipeline_en    (HEADER-ONLY)
    constant c_MC_N_FACES_HI         : natural := 14;   -- [14:12] n_faces
    constant c_MC_N_FACES_LO         : natural := 12;
    constant c_MC_STOPS_HI           : natural := 18;   -- [18:15] stops_per_chip
    constant c_MC_STOPS_LO           : natural := 15;
    constant c_MC_N_DRAIN_CAP_HI     : natural := 22;   -- [22:19] n_drain_cap
    constant c_MC_N_DRAIN_CAP_LO     : natural := 19;
    constant c_MC_STOPDIS_HI         : natural := 27;   -- [27:23] stopdis_override
    constant c_MC_STOPDIS_LO         : natural := 23;
    -- [31:28] COMMAND (edge-detect → 1-clk pulse)
    constant c_CMD_START             : natural := 28;   -- CTL0[28]
    constant c_CMD_STOP              : natural := 29;   -- CTL0[29]
    constant c_CMD_SOFT_RESET        : natural := 30;   -- CTL0[30]
    constant c_CMD_CFG_WRITE         : natural := 31;   -- CTL0[31]

    -- =========================================================================
    -- CTL1: BUS_TIMING
    -- =========================================================================
    constant c_ADDR_BUS_TIMING       : natural := 16#04#;   -- CTL1

    constant c_BT_CLK_DIV_HI        : natural := 5;    -- [5:0]   bus_clk_div
    constant c_BT_CLK_DIV_LO        : natural := 0;
    constant c_BT_TICKS_HI          : natural := 8;    -- [8:6]   bus_ticks
    constant c_BT_TICKS_LO          : natural := 6;
    -- [9] reserved
    constant c_BT_REG_ADDR_HI       : natural := 13;   -- [13:10] reg_target_addr
    constant c_BT_REG_ADDR_LO       : natural := 10;
    constant c_BT_REG_CHIP_HI       : natural := 15;   -- [15:14] reg_target_chip_id (단일 칩 모드)
    constant c_BT_REG_CHIP_LO       : natural := 14;
    constant c_BT_REG_CHIP_MASK_HI  : natural := 19;   -- [19:16] reg_target_chip_mask (멀티칩)
    constant c_BT_REG_CHIP_MASK_LO  : natural := 16;
    -- [29:20] reserved
    constant c_BT_REG_READ_TRIG     : natural := 30;   -- [30] reg_read trigger (edge)
    constant c_BT_REG_WRITE_TRIG    : natural := 31;   -- [31] reg_write trigger (edge)

    -- =========================================================================
    -- CTL2: RANGE_COLS (max_range_clks + cols_per_face)
    -- =========================================================================
    constant c_ADDR_RANGE_COLS       : natural := 16#08#;   -- CTL2

    constant c_RC_MAX_RANGE_HI       : natural := 15;   -- [15:0]  max_range_clks
    constant c_RC_MAX_RANGE_LO       : natural := 0;
    constant c_RC_COLS_HI            : natural := 31;   -- [31:16] cols_per_face
    constant c_RC_COLS_LO            : natural := 16;

    -- =========================================================================
    -- CTL3: START_OFF1
    -- =========================================================================
    constant c_ADDR_START_OFF1       : natural := 16#0C#;   -- CTL3 [17:0]

    -- =========================================================================
    -- CTL4: CFG_REG7
    -- =========================================================================
    constant c_ADDR_CFG_REG7         : natural := 16#10#;   -- CTL4 [31:0]

    -- =========================================================================
    -- CTL21: SCAN_TIMEOUT
    -- =========================================================================
    constant c_ADDR_SCAN_TIMEOUT     : natural := 16#54#;   -- CTL21
    constant c_ST_MAX_SCAN_HI        : natural := 15;   -- [15:0] max_scan_clks
    constant c_ST_MAX_SCAN_LO        : natural := 0;
    constant c_ST_MAX_HITS_HI        : natural := 18;   -- [18:16] max_hits_cfg (1~7, 0=default 7)
    constant c_ST_MAX_HITS_LO        : natural := 16;

    -- CFG_REG7 bitfields
    constant c_REG7_HSDIV_HI        : natural := 7;
    constant c_REG7_HSDIV_LO        : natural := 0;
    constant c_REG7_REFCLKDIV_HI    : natural := 10;
    constant c_REG7_REFCLKDIV_LO    : natural := 8;
    constant c_REG7_MTIMER_HI       : natural := 27;
    constant c_REG7_MTIMER_LO       : natural := 15;

    -- =========================================================================
    -- cfg_image Reg0 bitfields (TDC-GPX Register 0: edge sensitivity)
    --   [10:18] TRiseEn — bit10=TStart, bit11=TStop1 .. bit18=TStop8
    --   [19:27] TFallEn — bit19=TStart, bit20=TStop1 .. bit27=TStop8
    -- =========================================================================
    constant c_REG0_TRISEEN_LO      : natural := 10;   -- TRiseEn[0]=TStart
    constant c_REG0_TRISEEN_HI      : natural := 18;   -- TRiseEn[8]=TStop8
    constant c_REG0_TFALLEN_LO      : natural := 19;   -- TFallEn[0]=TStart
    constant c_REG0_TFALLEN_HI      : natural := 27;   -- TFallEn[8]=TStop8
    -- Stop channel offset: TRiseEn[1]=TStop1 = bit 11, ..., TRiseEn[8]=TStop8 = bit 18
    --                       TFallEn[1]=TStop1 = bit 20, ..., TFallEn[8]=TStop8 = bit 27
    constant c_REG0_STOP_OFFSET     : natural := 1;    -- bit index offset: stop N = base + N

    -- =========================================================================
    -- cfg_image Reg5 bitfields (TDC-GPX Register 5: StartOff1 + control)
    -- =========================================================================
    constant c_REG5_STARTOFF1_HI    : natural := 17;
    constant c_REG5_STARTOFF1_LO    : natural := 0;
    -- Reg5 bit23: MasterAluTrig — AluTrigger pin causes master reset
    --   (clears IFIFOs + ALU state). Required for overrun/shot-boundary cleanup.
    -- Reg5 bit24: PartialAluTrig — AluTrigger pin causes partial reset
    --   (clears ALU state only, keeps IFIFOs). Not used by this controller.
    constant c_REG5_MASTER_ALU_TRIG : natural := 23;
    constant c_REG5_PARTIAL_ALU_TRIG : natural := 24;

    -- =========================================================================
    -- cfg_image Reg6 bitfields (TDC-GPX Register 6: FIFO threshold)
    -- =========================================================================
    constant c_REG6_LF_THRESH_HI    : natural := 7;
    constant c_REG6_LF_THRESH_LO    : natural := 0;

    -- =========================================================================
    -- cfg_image (R/W) : CTL5~CTL20 → 0x14 ~ 0x50  (Reg0..Reg15 x 32-bit)
    -- =========================================================================
    constant c_ADDR_CFG_IMAGE_BASE   : natural := 16#14#;
    constant c_ADDR_CFG_IMAGE_END    : natural := 16#50#;
    constant c_CFG_IMAGE_N_REGS      : natural := 16;

    -- =========================================================================
    -- Status (R/O) : STAT0~STAT4 → 0x80 ~ 0x90  (compile-time constants)
    -- =========================================================================
    constant c_ADDR_HW_VERSION       : natural := 16#80#;  -- STAT0
    constant c_ADDR_HW_CONFIG        : natural := 16#84#;  -- STAT1
    constant c_ADDR_MAX_ROWS         : natural := 16#88#;  -- STAT2
    constant c_ADDR_CELL_SIZE        : natural := 16#8C#;  -- STAT3
    constant c_ADDR_MAX_HSIZE        : natural := 16#90#;  -- STAT4

    -- HW_CONFIG bitfields
    constant c_HWCFG_N_CHIPS_HI      : natural := 3;
    constant c_HWCFG_N_CHIPS_LO      : natural := 0;
    constant c_HWCFG_MAX_STOPS_HI    : natural := 7;
    constant c_HWCFG_MAX_STOPS_LO    : natural := 4;
    constant c_HWCFG_MAX_HITS_HI     : natural := 11;
    constant c_HWCFG_MAX_HITS_LO     : natural := 8;
    constant c_HWCFG_HIT_WIDTH_HI    : natural := 16;
    constant c_HWCFG_HIT_WIDTH_LO    : natural := 12;
    constant c_HWCFG_TDATA_HI        : natural := 24;
    constant c_HWCFG_TDATA_LO        : natural := 17;
    constant c_HWCFG_CELL_FMT_HI     : natural := 27;
    constant c_HWCFG_CELL_FMT_LO     : natural := 25;

    -- =========================================================================
    -- Status-Live (R/O) : STAT5~STAT10 → 0x94 ~ 0xA8  (runtime, CDC)
    -- =========================================================================
    constant c_ADDR_STATUS           : natural := 16#94#;   -- STAT5
    constant c_ADDR_SHOT_SEQ         : natural := 16#98#;   -- STAT6
    constant c_ADDR_FRAME_COUNT      : natural := 16#9C#;   -- STAT7
    constant c_ADDR_ERROR_COUNT      : natural := 16#A0#;   -- STAT8
    constant c_ADDR_BIN_PS           : natural := 16#A4#;   -- STAT9
    constant c_ADDR_K_DIST           : natural := 16#A8#;   -- STAT10
    constant c_ADDR_REG_RDATA        : natural := 16#AC#;   -- STAT11: reg access read data

    -- STATUS bitfields (STAT5)
    constant c_STAT_BUSY             : natural := 0;
    constant c_STAT_OVERRUN          : natural := 1;
    constant c_STAT_BIN_MISMATCH     : natural := 2;
    constant c_STAT_CHIP_ERR_HI      : natural := 7;
    constant c_STAT_CHIP_ERR_LO      : natural := 4;
    constant c_STAT_DRAIN_TO_HI      : natural := 11;   -- drain_timeout_mask[3:0]
    constant c_STAT_DRAIN_TO_LO      : natural := 8;
    constant c_STAT_SEQ_ERR_HI       : natural := 15;   -- sequence_error_mask[3:0]
    constant c_STAT_SEQ_ERR_LO       : natural := 12;

    -- =========================================================================
    -- Bus timing constraints (TDC-GPX datasheet @ 200 MHz, T_clk = 5 ns)
    --
    -- Datasheet parameters:
    --   tS-AD  >= 2 ns    address setup before strobe low
    --   tPW-RL >= 6 ns    RDN/WRN low pulse width
    --   tV-DR  <= 11.8 ns data valid delay after RDN low (worst case 40pF)
    --   tPW-RH >= 6 ns    RDN high pulse width (burst inter-read gap)
    --
    -- bus_phy read sample path:
    --   RDN low at tick 1, sample_en at tick (ticks-2), IOB FF at +1 clk
    --   => capture delay = ((ticks-3) * div + 1) * T_clk from RDN low
    --
    -- bus_phy burst path:
    --   Burst restarts at tick 0 (Phase A gap included).
    --   RDN high = 2 ticks = 2 * div * T_clk (Phase H + Phase A).
    --   Per-burst-read cost = (ticks + 1) ticks total.
    --
    -- Combined constraint: (ticks - 3) * div >= 2
    --   div=1 => ticks >= 5;  div >= 2 => ticks >= 4.
    --
    -- Legal combination table (T_clk = 5 ns):
    --   div  ticks  capture   tPW-RL   tPW-RH(burst)  rate      status
    --   ---  -----  --------  -------  -------------  --------  --------
    --    1     4     10 ns     10 ns    10 ns          50 MHz    ILLEGAL (tV-DR)
    --    1     5     15 ns     15 ns    10 ns          40 MHz    OK (fastest)
    --    1     6     25 ns     20 ns    10 ns          33 MHz    OK
    --    1     7     35 ns     25 ns    10 ns          29 MHz    OK
    --    2     4     15 ns     20 ns    20 ns          25 MHz    OK
    --    2     5     25 ns     30 ns    20 ns          20 MHz    OK (default)
    --    3     4     20 ns     30 ns    30 ns          17 MHz    OK
    --    2     7     45 ns     50 ns    20 ns          14 MHz    OK (slowest 3-bit)
    -- =========================================================================
    constant c_BUS_TICKS_MIN        : natural := 4;     -- absolute minimum (div>=2)
    constant c_BUS_TICKS_MIN_DIV1   : natural := 5;     -- div=1 needs extra tick for tV-DR
    constant c_BUS_CLK_DIV_MIN      : natural := 1;     -- div=1 OK with ticks>=5

    -- =========================================================================
    -- Init values
    -- =========================================================================
    -- CTL0: MAIN_CTRL init (packed)
    --   [3:0]=0xF, [4]=0, [6:5]=00, [9:7]=000, [10]=0, [11]=0,
    --   [14:12]=5(101), [18:15]=8(1000), [22:19]=0, [27:23]=0, [31:28]=0
    constant c_INIT_MAIN_CTRL        : std_logic_vector(31 downto 0) := x"0004500F";
    -- breakdown: n_faces=5→[14:12]=101=0x5000, stops=8→[18:15]=1000_0=0x40000
    --            0x00040000 + 0x00005000 + 0x0000000F = 0x0004500F

    -- CTL1: BUS_TIMING init: clk_div=2→[5:0]=000010, ticks=5→[8:6]=101
    constant c_INIT_BUS_TIMING       : std_logic_vector(31 downto 0) := x"00000142";
    -- breakdown: ticks=5→101<<6=0x140, clk_div=2→0x02 → 0x142

    -- CTL2: RANGE_COLS init: max_range=267→[15:0]=0x010B, cols=2400→[31:16]=0x0960
    constant c_INIT_RANGE_COLS       : std_logic_vector(31 downto 0) := x"0960010B";

    -- CTL3: START_OFF1 init
    constant c_INIT_START_OFF1       : std_logic_vector(31 downto 0) := x"00000000";

    -- CTL4: CFG_REG7 init
    constant c_INIT_CFG_REG7         : std_logic_vector(31 downto 0) := x"00000000";

end package tdc_gpx_cfg_pkg;
