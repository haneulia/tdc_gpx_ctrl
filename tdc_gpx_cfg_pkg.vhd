-- =============================================================================
-- tdc_gpx_cfg_pkg.vhd
-- TDC-GPX Controller - CSR register map constants and init values
-- =============================================================================
--
-- Purpose:
--   Defines CSR register address offsets, bitfield indices, and init values.
--   All offsets are 32-bit aligned (AXI-Lite standard).
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
    -- Control registers (R/W) : 0x00 ~ 0x3C
    -- =========================================================================
    constant c_ADDR_ACTIVE_CHIP_MASK : natural := 16#00#;   -- [3:0]
    constant c_ADDR_STOPS_PER_CHIP   : natural := 16#04#;   -- [7:0]
    constant c_ADDR_COLS_PER_FACE    : natural := 16#08#;   -- [15:0]
    constant c_ADDR_PACKET_SCOPE     : natural := 16#0C#;   -- [7:0]
    constant c_ADDR_HIT_STORE_MODE   : natural := 16#10#;   -- [7:0]
    constant c_ADDR_DIST_SCALE       : natural := 16#14#;   -- [7:0]
    constant c_ADDR_DRAIN_MODE       : natural := 16#18#;   -- [7:0]
    constant c_ADDR_N_DRAIN_CAP      : natural := 16#1C#;   -- [7:0]
    constant c_ADDR_PIPELINE_EN      : natural := 16#20#;   -- [7:0]
    constant c_ADDR_N_FACES          : natural := 16#24#;   -- [7:0]
    constant c_ADDR_BUS_CLK_DIV      : natural := 16#28#;   -- [7:0]
    constant c_ADDR_BUS_TICKS        : natural := 16#2C#;   -- [7:0]
    constant c_ADDR_STOPDIS_OVERRIDE : natural := 16#30#;   -- [4:0]
    constant c_ADDR_SLICE_TIMEOUT    : natural := 16#34#;   -- [7:0]

    -- =========================================================================
    -- TDC settings (R/W) : 0x40 ~ 0x4C
    -- =========================================================================
    constant c_ADDR_START_OFF1       : natural := 16#40#;   -- [17:0]
    constant c_ADDR_CFG_REG7         : natural := 16#44#;   -- [31:0]

    -- CFG_REG7 bitfields
    constant c_REG7_HSDIV_HI        : natural := 7;
    constant c_REG7_HSDIV_LO        : natural := 0;
    constant c_REG7_REFCLKDIV_HI    : natural := 10;
    constant c_REG7_REFCLKDIV_LO    : natural := 8;
    constant c_REG7_MTIMER_HI       : natural := 27;
    constant c_REG7_MTIMER_LO       : natural := 15;

    -- =========================================================================
    -- cfg_image (R/W) : 0x80 ~ 0xBC  (Reg0..Reg14 x 32-bit)
    -- =========================================================================
    constant c_ADDR_CFG_IMAGE_BASE   : natural := 16#80#;
    constant c_ADDR_CFG_IMAGE_END    : natural := 16#BC#;
    constant c_CFG_IMAGE_N_REGS      : natural := 16;

    -- =========================================================================
    -- Command (R/W) : 0xC0
    -- =========================================================================
    constant c_ADDR_COMMAND          : natural := 16#C0#;

    -- COMMAND bitfields
    constant c_CMD_START             : natural := 0;    -- [0] start
    constant c_CMD_STOP              : natural := 1;    -- [1] stop
    constant c_CMD_SOFT_RESET        : natural := 2;    -- [2] soft_reset
    constant c_CMD_CFG_WRITE         : natural := 3;    -- [3] cfg_write_trigger

    -- =========================================================================
    -- Status (R/O) : 0x100 ~ 0x110  (generic constants, compile-time fixed)
    -- =========================================================================
    constant c_ADDR_HW_VERSION       : natural := 16#100#;
    constant c_ADDR_HW_CONFIG        : natural := 16#104#;
    constant c_ADDR_MAX_ROWS         : natural := 16#108#;
    constant c_ADDR_CELL_SIZE        : natural := 16#10C#;
    constant c_ADDR_MAX_HSIZE        : natural := 16#110#;

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
    -- Status-Live (R/O) : 0x140 ~ 0x154  (runtime updated)
    -- =========================================================================
    constant c_ADDR_STATUS           : natural := 16#140#;
    constant c_ADDR_SHOT_SEQ         : natural := 16#144#;
    constant c_ADDR_FRAME_COUNT      : natural := 16#148#;
    constant c_ADDR_ERROR_COUNT      : natural := 16#14C#;
    constant c_ADDR_BIN_PS           : natural := 16#150#;
    constant c_ADDR_K_DIST           : natural := 16#154#;

    -- STATUS bitfields
    constant c_STAT_BUSY             : natural := 0;
    constant c_STAT_OVERRUN          : natural := 1;
    constant c_STAT_BIN_MISMATCH     : natural := 2;
    constant c_STAT_LANE_ERR_HI      : natural := 7;
    constant c_STAT_LANE_ERR_LO      : natural := 4;

    -- =========================================================================
    -- Init values for Control registers
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
    constant c_INIT_SLICE_TIMEOUT    : std_logic_vector(31 downto 0) := x"000000C8";  -- 200

end package tdc_gpx_cfg_pkg;
