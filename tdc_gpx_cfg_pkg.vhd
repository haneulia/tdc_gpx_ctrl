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
    constant c_MC_PACKET_SCOPE       : natural := 4;    -- [4]     packet_scope
    constant c_MC_HIT_STORE_HI       : natural := 6;    -- [6:5]   hit_store_mode
    constant c_MC_HIT_STORE_LO       : natural := 5;
    constant c_MC_DIST_SCALE_HI      : natural := 9;    -- [9:7]   dist_scale
    constant c_MC_DIST_SCALE_LO      : natural := 7;
    constant c_MC_DRAIN_MODE         : natural := 10;   -- [10]    drain_mode
    constant c_MC_PIPELINE_EN        : natural := 11;   -- [11]    pipeline_en
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

    -- CFG_REG7 bitfields
    constant c_REG7_HSDIV_HI        : natural := 7;
    constant c_REG7_HSDIV_LO        : natural := 0;
    constant c_REG7_REFCLKDIV_HI    : natural := 10;
    constant c_REG7_REFCLKDIV_LO    : natural := 8;
    constant c_REG7_MTIMER_HI       : natural := 27;
    constant c_REG7_MTIMER_LO       : natural := 15;

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

    -- STATUS bitfields (STAT5)
    constant c_STAT_BUSY             : natural := 0;
    constant c_STAT_OVERRUN          : natural := 1;
    constant c_STAT_BIN_MISMATCH     : natural := 2;
    constant c_STAT_LANE_ERR_HI      : natural := 7;
    constant c_STAT_LANE_ERR_LO      : natural := 4;

    -- =========================================================================
    -- Init values
    -- =========================================================================
    -- CTL0: MAIN_CTRL init (packed)
    --   [3:0]=0xF, [4]=0, [6:5]=00, [9:7]=000, [10]=0, [11]=0,
    --   [14:12]=5(101), [18:15]=8(1000), [22:19]=0, [27:23]=0, [31:28]=0
    constant c_INIT_MAIN_CTRL        : std_logic_vector(31 downto 0) := x"0004500F";
    -- breakdown: n_faces=5→[14:12]=101=0x5000, stops=8→[18:15]=1000_0=0x40000
    --            0x00040000 + 0x00005000 + 0x0000000F = 0x0004500F

    -- CTL1: BUS_TIMING init: clk_div=1→[5:0]=000001, ticks=5→[8:6]=101
    constant c_INIT_BUS_TIMING       : std_logic_vector(31 downto 0) := x"00000141";
    -- breakdown: ticks=5→101<<6=0x140, clk_div=1→0x01 → 0x141

    -- CTL2: RANGE_COLS init: max_range=267→[15:0]=0x010B, cols=2400→[31:16]=0x0960
    constant c_INIT_RANGE_COLS       : std_logic_vector(31 downto 0) := x"0960010B";

    -- CTL3: START_OFF1 init
    constant c_INIT_START_OFF1       : std_logic_vector(31 downto 0) := x"00000000";

    -- CTL4: CFG_REG7 init
    constant c_INIT_CFG_REG7         : std_logic_vector(31 downto 0) := x"00000000";

end package tdc_gpx_cfg_pkg;
