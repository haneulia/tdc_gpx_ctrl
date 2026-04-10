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
--   CTL[0..13]  = control registers
--   CTL[14..15] = TDC settings (start_off1, cfg_reg7)
--   CTL[16..31] = cfg_image[0..15]
--   COMMAND bits embedded in CTL0[31:28] (edge-detect → 1-clk pulse)
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
    -- Control registers (R/W) : CTL0~CTL13 → 0x00 ~ 0x34
    -- =========================================================================
    constant c_ADDR_ACTIVE_CHIP_MASK : natural := 16#00#;   -- CTL0  [3:0] mask, [31:28] COMMAND
    constant c_ADDR_STOPS_PER_CHIP   : natural := 16#04#;   -- CTL1  [3:0]
    constant c_ADDR_COLS_PER_FACE    : natural := 16#08#;   -- CTL2  [15:0]
    constant c_ADDR_PACKET_SCOPE     : natural := 16#0C#;   -- CTL3  [0]
    constant c_ADDR_HIT_STORE_MODE   : natural := 16#10#;   -- CTL4  [1:0]
    constant c_ADDR_DIST_SCALE       : natural := 16#14#;   -- CTL5  [2:0]
    constant c_ADDR_DRAIN_MODE       : natural := 16#18#;   -- CTL6  [0]
    constant c_ADDR_N_DRAIN_CAP      : natural := 16#1C#;   -- CTL7  [7:0]
    constant c_ADDR_PIPELINE_EN      : natural := 16#20#;   -- CTL8  [0]
    constant c_ADDR_N_FACES          : natural := 16#24#;   -- CTL9  [7:0]
    constant c_ADDR_BUS_CLK_DIV      : natural := 16#28#;   -- CTL10 [7:0]
    constant c_ADDR_BUS_TICKS        : natural := 16#2C#;   -- CTL11 [2:0]
    constant c_ADDR_STOPDIS_OVERRIDE : natural := 16#30#;   -- CTL12 [4:0]
    constant c_ADDR_MAX_RANGE_CLKS   : natural := 16#34#;   -- CTL13 [15:0]

    -- =========================================================================
    -- TDC settings (R/W) : CTL14~CTL15 → 0x38 ~ 0x3C
    -- =========================================================================
    constant c_ADDR_START_OFF1       : natural := 16#38#;   -- CTL14 [17:0]
    constant c_ADDR_CFG_REG7         : natural := 16#3C#;   -- CTL15 [31:0]

    -- CFG_REG7 bitfields
    constant c_REG7_HSDIV_HI        : natural := 7;
    constant c_REG7_HSDIV_LO        : natural := 0;
    constant c_REG7_REFCLKDIV_HI    : natural := 10;
    constant c_REG7_REFCLKDIV_LO    : natural := 8;
    constant c_REG7_MTIMER_HI       : natural := 27;
    constant c_REG7_MTIMER_LO       : natural := 15;

    -- =========================================================================
    -- cfg_image (R/W) : CTL16~CTL31 → 0x40 ~ 0x7C  (Reg0..Reg15 x 32-bit)
    -- =========================================================================
    constant c_ADDR_CFG_IMAGE_BASE   : natural := 16#40#;
    constant c_ADDR_CFG_IMAGE_END    : natural := 16#7C#;
    constant c_CFG_IMAGE_N_REGS      : natural := 16;

    -- =========================================================================
    -- COMMAND bits embedded in CTL0[31:28]
    --   SW writes '1' to bit → HW detects rising edge → 1-clk pulse
    --   SW should clear bit after use (or let HW auto-clear via edge-detect)
    -- =========================================================================
    constant c_CMD_START             : natural := 28;   -- CTL0[28]
    constant c_CMD_STOP              : natural := 29;   -- CTL0[29]
    constant c_CMD_SOFT_RESET        : natural := 30;   -- CTL0[30]
    constant c_CMD_CFG_WRITE         : natural := 31;   -- CTL0[31]

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
    -- Init values for Control registers (CTL0~CTL13)
    -- =========================================================================
    constant c_INIT_ACTIVE_MASK      : std_logic_vector(31 downto 0) := x"0000000F";
    constant c_INIT_STOPS_PER_CHIP   : std_logic_vector(31 downto 0) := x"00000008";
    constant c_INIT_COLS_PER_FACE    : std_logic_vector(31 downto 0) := x"00000960";  -- 2400
    constant c_INIT_PACKET_SCOPE     : std_logic_vector(31 downto 0) := x"00000000";
    constant c_INIT_HIT_STORE_MODE   : std_logic_vector(31 downto 0) := x"00000000";  -- RAW
    constant c_INIT_DIST_SCALE       : std_logic_vector(31 downto 0) := x"00000000";  -- 1mm
    constant c_INIT_DRAIN_MODE       : std_logic_vector(31 downto 0) := x"00000000";  -- SrcGating
    constant c_INIT_N_DRAIN_CAP      : std_logic_vector(31 downto 0) := x"00000000";  -- unlimited
    constant c_INIT_PIPELINE_EN      : std_logic_vector(31 downto 0) := x"00000000";  -- sequential
    constant c_INIT_N_FACES          : std_logic_vector(31 downto 0) := x"00000005";
    constant c_INIT_BUS_CLK_DIV      : std_logic_vector(31 downto 0) := x"00000001";
    constant c_INIT_BUS_TICKS        : std_logic_vector(31 downto 0) := x"00000005";
    constant c_INIT_STOPDIS_OVR      : std_logic_vector(31 downto 0) := x"00000000";
    constant c_INIT_MAX_RANGE_CLKS   : std_logic_vector(31 downto 0) := x"0000010B";  -- 267

end package tdc_gpx_cfg_pkg;
