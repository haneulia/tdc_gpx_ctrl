-- =============================================================================
-- tdc_gpx_csr_chip.vhd
-- TDC-GPX Controller - Chip Configuration / Register Access CSR
-- =============================================================================
--
-- Purpose:
--   AXI4-Lite register interface for TDC-GPX chip configuration and individual
--   register read/write access.  Uses tdc_gpx_axil_csr32_chip IP (32 CTL + 32 STAT,
--   9-bit address).  CDC transfers control registers to i_axis_aclk domain.
--
-- Owned registers:
--   CTL1  (0x04) BUS_TIMING     [5:0] bus_clk_div, [8:6] bus_ticks,
--                                [13:10] reg_target_addr, [15:14] reg_target_chip,
--                                [19:16] reg_target_chip_mask,
--                                [30] reg_read_trigger, [31] reg_write_trigger
--   CTL3  (0x0C) START_OFF1     [17:0]
--   CTL4  (0x10) CFG_REG7       [31:0]
--   CTL5~20 (0x14~0x50) CFG_IMAGE[0..15]  16 x 32-bit chip register mirrors
--   CTL21 (0x54) SCAN_TIMEOUT   [15:0] max_scan_clks, [18:16] max_hits_cfg
--
-- Unused CTL outputs (owned by csr_pipeline via separate IP):
--   CTL0  (MAIN_CTRL)   — left open
--   CTL2  (RANGE_COLS)  — left open
--   CTL22~31            — left open
--
-- STAT registers:
--   STAT0  (0x80) chip0 result  [27:0] rdata, [31:28] addr
--   STAT1  (0x84) chip1 result  [27:0] rdata, [31:28] addr
--   STAT2  (0x88) chip2 result  [27:0] rdata, [31:28] addr
--   STAT3  (0x8C) chip3 result  [27:0] rdata, [31:28] addr
--   STAT4~31                    — tied to zero
--
-- CDC structure:
--   CTL1   : 1 x xpm_cdc_handshake (s_axi_aclk -> i_axis_aclk) — bus timing
--   CTL3   : 1 x xpm_cdc_handshake — start_off1
--   CTL4   : 1 x xpm_cdc_handshake — cfg_reg7
--   CTL5~20: 16 x xpm_cdc_handshake — cfg_image, per-register
--   CTL21  : 1 x xpm_cdc_handshake — scan_timeout + max_hits
--   STAT0~3: 4 x xpm_cdc_handshake (i_axis_aclk -> s_axi_aclk) — per-chip results
--   IRQ    : 1 x xpm_cdc_pulse (i_axis_aclk -> s_axi_aclk) — done_pulse
--   Total: 25 CDC instances
--
-- CDC idle flag:
--   NOR of all 20 src_send signals (ctl1, ctl3, ctl4, img x16, ctl21).
--   Output as o_cdc_idle (s_axi_aclk domain) for csr_pipeline to use.
--   Note: STAT CDCs (i_axis->s_axi direction) are NOT included in idle flag.
--
-- Clock domains:
--   s_axi_aclk  : AXI4-Lite domain (PS clock)
--   i_axis_aclk : TDC processing / AXI-Stream domain (200 MHz)
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

library xpm;
use xpm.vcomponents.all;

entity tdc_gpx_csr_chip is
    port (
        -- AXI4-Lite clock / reset
        s_axi_aclk          : in  std_logic;
        s_axi_aresetn       : in  std_logic;

        -- AXI4-Lite Slave (9-bit address, tdc_gpx_axil_csr32)
        s_axi_awvalid       : in  std_logic;
        s_axi_awready       : out std_logic;
        s_axi_awaddr        : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_awprot        : in  std_logic_vector(2 downto 0);
        s_axi_wvalid        : in  std_logic;
        s_axi_wready        : out std_logic;
        s_axi_wdata         : in  std_logic_vector(31 downto 0);
        s_axi_wstrb         : in  std_logic_vector(3 downto 0);
        s_axi_bvalid        : out std_logic;
        s_axi_bready        : in  std_logic;
        s_axi_bresp         : out std_logic_vector(1 downto 0);
        s_axi_arvalid       : in  std_logic;
        s_axi_arready       : out std_logic;
        s_axi_araddr        : in  std_logic_vector(c_CSR_ADDR_WIDTH - 1 downto 0);
        s_axi_arprot        : in  std_logic_vector(2 downto 0);
        s_axi_rvalid        : out std_logic;
        s_axi_rready        : in  std_logic;
        s_axi_rdata         : out std_logic_vector(31 downto 0);
        s_axi_rresp         : out std_logic_vector(1 downto 0);

        -- TDC processing clock / reset
        i_axis_aclk         : in  std_logic;
        i_axis_aresetn      : in  std_logic;

        -- Configuration outputs (i_axis_aclk domain)
        o_cfg_image         : out t_cfg_image;
        o_bus_clk_div       : out unsigned(5 downto 0);
        o_bus_ticks         : out unsigned(2 downto 0);
        o_start_off1        : out unsigned(17 downto 0);
        o_cfg_reg7          : out std_logic_vector(31 downto 0);
        o_max_scan_clks     : out unsigned(15 downto 0);
        o_max_hits_cfg      : out unsigned(2 downto 0);

        -- Reg access commands (i_axis_aclk domain)
        o_cmd_reg_read      : out std_logic;
        o_cmd_reg_write     : out std_logic;
        o_cmd_reg_addr      : out std_logic_vector(3 downto 0);
        o_cmd_reg_chip      : out unsigned(1 downto 0);

        -- Per-chip reg result (i_axis_aclk domain)
        -- Each chip independently returns read data when its reg operation completes
        i_cmd_reg_rdata_0   : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rdata_1   : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rdata_2   : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rdata_3   : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rvalid    : in  std_logic_vector(c_N_CHIPS - 1 downto 0);  -- per-chip valid

        -- All-done feedback from cmd_arb (i_axis_aclk domain)
        i_cmd_reg_done_pulse : in  std_logic;  -- 1-clk: all target chips completed
        i_cmd_reg_addr_done  : in  std_logic_vector(3 downto 0);  -- addr that was accessed

        -- Chip mask output from CTL1 CDC (i_axis_aclk domain)
        o_cmd_reg_chip_address : out std_logic_vector(c_N_CHIPS - 1 downto 0);  -- CTL1[19:16]

        -- CDC idle output (to csr_pipeline, s_axi_aclk domain)
        o_cdc_idle          : out std_logic;

        -- Interrupt (directly from SRM IP)
        o_irq               : out std_logic
    );
end entity tdc_gpx_csr_chip;

architecture rtl of tdc_gpx_csr_chip is

    -- =========================================================================
    -- tdc_gpx_axil_csr32_chip component (Vivado IP: 32 CTL, 32 STAT, 1 IRQ)
    -- =========================================================================
    component tdc_gpx_axil_csr32_chip is
        port (
            s_axi_csr_aclk      : in  std_logic;
            s_axi_csr_aresetn   : in  std_logic;
            s_axi_csr_awaddr    : in  std_logic_vector(8 downto 0);
            s_axi_csr_awprot    : in  std_logic_vector(2 downto 0);
            s_axi_csr_awvalid   : in  std_logic;
            s_axi_csr_awready   : out std_logic;
            s_axi_csr_wdata     : in  std_logic_vector(31 downto 0);
            s_axi_csr_wstrb     : in  std_logic_vector(3 downto 0);
            s_axi_csr_wvalid    : in  std_logic;
            s_axi_csr_wready    : out std_logic;
            s_axi_csr_bresp     : out std_logic_vector(1 downto 0);
            s_axi_csr_bvalid    : out std_logic;
            s_axi_csr_bready    : in  std_logic;
            s_axi_csr_araddr    : in  std_logic_vector(8 downto 0);
            s_axi_csr_arprot    : in  std_logic_vector(2 downto 0);
            s_axi_csr_arvalid   : in  std_logic;
            s_axi_csr_arready   : out std_logic;
            s_axi_csr_rdata     : out std_logic_vector(31 downto 0);
            s_axi_csr_rresp     : out std_logic_vector(1 downto 0);
            s_axi_csr_rvalid    : out std_logic;
            s_axi_csr_rready    : in  std_logic;
            -- Init values (32 CTL registers)
            reg0_init_val  : in std_logic_vector(31 downto 0);
            reg1_init_val  : in std_logic_vector(31 downto 0);
            reg2_init_val  : in std_logic_vector(31 downto 0);
            reg3_init_val  : in std_logic_vector(31 downto 0);
            reg4_init_val  : in std_logic_vector(31 downto 0);
            reg5_init_val  : in std_logic_vector(31 downto 0);
            reg6_init_val  : in std_logic_vector(31 downto 0);
            reg7_init_val  : in std_logic_vector(31 downto 0);
            reg8_init_val  : in std_logic_vector(31 downto 0);
            reg9_init_val  : in std_logic_vector(31 downto 0);
            reg10_init_val : in std_logic_vector(31 downto 0);
            reg11_init_val : in std_logic_vector(31 downto 0);
            reg12_init_val : in std_logic_vector(31 downto 0);
            reg13_init_val : in std_logic_vector(31 downto 0);
            reg14_init_val : in std_logic_vector(31 downto 0);
            reg15_init_val : in std_logic_vector(31 downto 0);
            reg16_init_val : in std_logic_vector(31 downto 0);
            reg17_init_val : in std_logic_vector(31 downto 0);
            reg18_init_val : in std_logic_vector(31 downto 0);
            reg19_init_val : in std_logic_vector(31 downto 0);
            reg20_init_val : in std_logic_vector(31 downto 0);
            reg21_init_val : in std_logic_vector(31 downto 0);
            reg22_init_val : in std_logic_vector(31 downto 0);
            reg23_init_val : in std_logic_vector(31 downto 0);
            reg24_init_val : in std_logic_vector(31 downto 0);
            reg25_init_val : in std_logic_vector(31 downto 0);
            reg26_init_val : in std_logic_vector(31 downto 0);
            reg27_init_val : in std_logic_vector(31 downto 0);
            reg28_init_val : in std_logic_vector(31 downto 0);
            reg29_init_val : in std_logic_vector(31 downto 0);
            reg30_init_val : in std_logic_vector(31 downto 0);
            reg31_init_val : in std_logic_vector(31 downto 0);
            -- CTL outputs (32)
            ctl0_out  : out std_logic_vector(31 downto 0);
            ctl1_out  : out std_logic_vector(31 downto 0);
            ctl2_out  : out std_logic_vector(31 downto 0);
            ctl3_out  : out std_logic_vector(31 downto 0);
            ctl4_out  : out std_logic_vector(31 downto 0);
            ctl5_out  : out std_logic_vector(31 downto 0);
            ctl6_out  : out std_logic_vector(31 downto 0);
            ctl7_out  : out std_logic_vector(31 downto 0);
            ctl8_out  : out std_logic_vector(31 downto 0);
            ctl9_out  : out std_logic_vector(31 downto 0);
            ctl10_out : out std_logic_vector(31 downto 0);
            ctl11_out : out std_logic_vector(31 downto 0);
            ctl12_out : out std_logic_vector(31 downto 0);
            ctl13_out : out std_logic_vector(31 downto 0);
            ctl14_out : out std_logic_vector(31 downto 0);
            ctl15_out : out std_logic_vector(31 downto 0);
            ctl16_out : out std_logic_vector(31 downto 0);
            ctl17_out : out std_logic_vector(31 downto 0);
            ctl18_out : out std_logic_vector(31 downto 0);
            ctl19_out : out std_logic_vector(31 downto 0);
            ctl20_out : out std_logic_vector(31 downto 0);
            ctl21_out : out std_logic_vector(31 downto 0);
            ctl22_out : out std_logic_vector(31 downto 0);
            ctl23_out : out std_logic_vector(31 downto 0);
            ctl24_out : out std_logic_vector(31 downto 0);
            ctl25_out : out std_logic_vector(31 downto 0);
            ctl26_out : out std_logic_vector(31 downto 0);
            ctl27_out : out std_logic_vector(31 downto 0);
            ctl28_out : out std_logic_vector(31 downto 0);
            ctl29_out : out std_logic_vector(31 downto 0);
            ctl30_out : out std_logic_vector(31 downto 0);
            ctl31_out : out std_logic_vector(31 downto 0);
            -- STAT inputs (32)
            stat0_in  : in std_logic_vector(31 downto 0);
            stat1_in  : in std_logic_vector(31 downto 0);
            stat2_in  : in std_logic_vector(31 downto 0);
            stat3_in  : in std_logic_vector(31 downto 0);
            stat4_in  : in std_logic_vector(31 downto 0);
            stat5_in  : in std_logic_vector(31 downto 0);
            stat6_in  : in std_logic_vector(31 downto 0);
            stat7_in  : in std_logic_vector(31 downto 0);
            stat8_in  : in std_logic_vector(31 downto 0);
            stat9_in  : in std_logic_vector(31 downto 0);
            stat10_in : in std_logic_vector(31 downto 0);
            stat11_in : in std_logic_vector(31 downto 0);
            stat12_in : in std_logic_vector(31 downto 0);
            stat13_in : in std_logic_vector(31 downto 0);
            stat14_in : in std_logic_vector(31 downto 0);
            stat15_in : in std_logic_vector(31 downto 0);
            stat16_in : in std_logic_vector(31 downto 0);
            stat17_in : in std_logic_vector(31 downto 0);
            stat18_in : in std_logic_vector(31 downto 0);
            stat19_in : in std_logic_vector(31 downto 0);
            stat20_in : in std_logic_vector(31 downto 0);
            stat21_in : in std_logic_vector(31 downto 0);
            stat22_in : in std_logic_vector(31 downto 0);
            stat23_in : in std_logic_vector(31 downto 0);
            stat24_in : in std_logic_vector(31 downto 0);
            stat25_in : in std_logic_vector(31 downto 0);
            stat26_in : in std_logic_vector(31 downto 0);
            stat27_in : in std_logic_vector(31 downto 0);
            stat28_in : in std_logic_vector(31 downto 0);
            stat29_in : in std_logic_vector(31 downto 0);
            stat30_in : in std_logic_vector(31 downto 0);
            stat31_in : in std_logic_vector(31 downto 0);
            -- Interrupt
            intrpt_src_in : in  std_logic_vector(0 downto 0);
            irq           : out std_logic
        );
    end component tdc_gpx_axil_csr32_chip;

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant C_ZERO32 : std_logic_vector(31 downto 0) := (others => '0');

    -- =========================================================================
    -- Internal types
    -- =========================================================================
    type t_cdc_data_array is array(natural range <>) of std_logic_vector(31 downto 0);

    -- Per-chip STAT registers (i_axis_aclk domain)
    type t_stat_array is array(0 to c_N_CHIPS - 1) of std_logic_vector(31 downto 0);

    -- Per-chip STAT CDC output (s_axi_aclk domain)
    type t_stat_cdc_array is array(0 to c_N_CHIPS - 1) of std_logic_vector(31 downto 0);

    -- =========================================================================
    -- CTL raw outputs from SRM (s_axi_aclk domain)
    -- =========================================================================
    signal s_ctl1_src   : std_logic_vector(31 downto 0);  -- BUS_TIMING
    signal s_ctl3_src   : std_logic_vector(31 downto 0);  -- START_OFF1
    signal s_ctl4_src   : std_logic_vector(31 downto 0);  -- CFG_REG7
    signal s_ctl21_src  : std_logic_vector(31 downto 0);  -- SCAN_TIMEOUT
    signal s_img_src    : t_cdc_data_array(0 to c_CFG_IMAGE_N_REGS - 1);  -- CFG_IMAGE

    -- =========================================================================
    -- CTL after CDC (i_axis_aclk domain)
    -- =========================================================================
    signal s_ctl1_out   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_ctl3_out   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_ctl4_out   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_ctl21_out  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_img_out    : t_cfg_image := (others => C_ZERO32);

    -- =========================================================================
    -- CTL CDC handshake signals
    -- =========================================================================
    -- CTL1 (BUS_TIMING)
    signal s_src_send_ctl1  : std_logic := '0';
    signal s_src_rcv_ctl1   : std_logic;
    signal s_dest_req_ctl1  : std_logic;
    signal s_ctl1_d1        : std_logic_vector(31 downto 0) := (others => '1');

    -- CTL3 (START_OFF1)
    signal s_src_send_ctl3  : std_logic := '0';
    signal s_src_rcv_ctl3   : std_logic;
    signal s_dest_req_ctl3  : std_logic;
    signal s_ctl3_d1        : std_logic_vector(31 downto 0) := (others => '1');

    -- CTL4 (CFG_REG7)
    signal s_src_send_ctl4  : std_logic := '0';
    signal s_src_rcv_ctl4   : std_logic;
    signal s_dest_req_ctl4  : std_logic;
    signal s_ctl4_d1        : std_logic_vector(31 downto 0) := (others => '1');

    -- CTL21 (SCAN_TIMEOUT)
    signal s_src_send_ctl21 : std_logic := '0';
    signal s_src_rcv_ctl21  : std_logic;
    signal s_dest_req_ctl21 : std_logic;
    signal s_ctl21_d1       : std_logic_vector(31 downto 0) := (others => '1');

    -- cfg_image CDC handshake (CTL5~20, per-register)
    signal s_src_send_img   : std_logic_vector(c_CFG_IMAGE_N_REGS - 1 downto 0) := (others => '0');
    signal s_src_rcv_img    : std_logic_vector(c_CFG_IMAGE_N_REGS - 1 downto 0);
    signal s_dest_req_img   : std_logic_vector(c_CFG_IMAGE_N_REGS - 1 downto 0);
    signal s_img_d1         : t_cdc_data_array(0 to c_CFG_IMAGE_N_REGS - 1) := (others => (others => '1'));

    -- =========================================================================
    -- Per-chip STAT registers (i_axis_aclk domain) and CDC outputs
    -- =========================================================================
    signal s_reg_stat_r : t_stat_array := (others => (others => '0'));
    signal s_stat_out   : t_stat_cdc_array := (others => (others => '0'));

    -- =========================================================================
    -- IRQ CDC (i_axis_aclk -> s_axi_aclk)
    -- =========================================================================
    signal s_irq_pulse : std_logic;

    -- =========================================================================
    -- CDC-idle flag (s_axi_aclk domain)
    -- =========================================================================
    signal s_cdc_all_idle_src : std_logic := '1';

    -- Local 2-FF sync of CDC idle to i_axis_aclk (for reg command gating)
    signal s_cdc_all_idle_ff  : std_logic_vector(1 downto 0) := "11";

    -- =========================================================================
    -- Reg access edge detect (i_axis_aclk domain)
    -- =========================================================================
    signal s_reg_cmd_prev_r   : std_logic_vector(1 downto 0) := (others => '0');
    signal s_reg_cmd_pulse_r  : std_logic_vector(1 downto 0) := (others => '0');

    -- =========================================================================
    -- Bus timing combined constraint intermediates
    -- =========================================================================
    signal s_div_clamped  : unsigned(5 downto 0);
    signal s_ticks_min    : unsigned(2 downto 0);

begin

    -- =========================================================================
    -- [1] tdc_gpx_axil_csr32_chip instantiation (32 CTL, 32 STAT)
    -- =========================================================================
    u_csr : tdc_gpx_axil_csr32_chip
        port map (
            s_axi_csr_aclk    => s_axi_aclk,
            s_axi_csr_aresetn => s_axi_aresetn,
            s_axi_csr_awaddr  => s_axi_awaddr,
            s_axi_csr_awprot  => s_axi_awprot,
            s_axi_csr_awvalid => s_axi_awvalid,
            s_axi_csr_awready => s_axi_awready,
            s_axi_csr_wdata   => s_axi_wdata,
            s_axi_csr_wstrb   => s_axi_wstrb,
            s_axi_csr_wvalid  => s_axi_wvalid,
            s_axi_csr_wready  => s_axi_wready,
            s_axi_csr_bresp   => s_axi_bresp,
            s_axi_csr_bvalid  => s_axi_bvalid,
            s_axi_csr_bready  => s_axi_bready,
            s_axi_csr_araddr  => s_axi_araddr,
            s_axi_csr_arprot  => s_axi_arprot,
            s_axi_csr_arvalid => s_axi_arvalid,
            s_axi_csr_arready => s_axi_arready,
            s_axi_csr_rdata   => s_axi_rdata,
            s_axi_csr_rresp   => s_axi_rresp,
            s_axi_csr_rvalid  => s_axi_rvalid,
            s_axi_csr_rready  => s_axi_rready,
            -- Init values: CTL0/CTL2 unused (owned by csr_pipeline), CTL5~20 cfg_image
            reg0_init_val  => C_ZERO32,             -- CTL0 unused
            reg1_init_val  => c_INIT_BUS_TIMING,    -- CTL1
            reg2_init_val  => C_ZERO32,             -- CTL2 unused
            reg3_init_val  => c_INIT_START_OFF1,    -- CTL3
            reg4_init_val  => c_INIT_CFG_REG7,      -- CTL4
            reg5_init_val  => C_ZERO32,   reg6_init_val  => C_ZERO32,
            reg7_init_val  => C_ZERO32,   reg8_init_val  => C_ZERO32,
            reg9_init_val  => C_ZERO32,   reg10_init_val => C_ZERO32,
            reg11_init_val => C_ZERO32,   reg12_init_val => C_ZERO32,
            reg13_init_val => C_ZERO32,   reg14_init_val => C_ZERO32,
            reg15_init_val => C_ZERO32,   reg16_init_val => C_ZERO32,
            reg17_init_val => C_ZERO32,   reg18_init_val => C_ZERO32,
            reg19_init_val => C_ZERO32,   reg20_init_val => C_ZERO32,
            reg21_init_val => C_ZERO32,   reg22_init_val => C_ZERO32,
            reg23_init_val => C_ZERO32,   reg24_init_val => C_ZERO32,
            reg25_init_val => C_ZERO32,   reg26_init_val => C_ZERO32,
            reg27_init_val => C_ZERO32,   reg28_init_val => C_ZERO32,
            reg29_init_val => C_ZERO32,   reg30_init_val => C_ZERO32,
            reg31_init_val => C_ZERO32,
            -- CTL outputs: only CTL1, CTL3, CTL4, CTL5~20, CTL21 used
            ctl0_out  => open,              -- MAIN_CTRL: owned by csr_pipeline
            ctl1_out  => s_ctl1_src,        -- BUS_TIMING
            ctl2_out  => open,              -- RANGE_COLS: owned by csr_pipeline
            ctl3_out  => s_ctl3_src,        -- START_OFF1
            ctl4_out  => s_ctl4_src,        -- CFG_REG7
            ctl5_out  => s_img_src(0),      -- CFG_IMAGE[0]
            ctl6_out  => s_img_src(1),
            ctl7_out  => s_img_src(2),
            ctl8_out  => s_img_src(3),
            ctl9_out  => s_img_src(4),
            ctl10_out => s_img_src(5),
            ctl11_out => s_img_src(6),
            ctl12_out => s_img_src(7),
            ctl13_out => s_img_src(8),
            ctl14_out => s_img_src(9),
            ctl15_out => s_img_src(10),
            ctl16_out => s_img_src(11),
            ctl17_out => s_img_src(12),
            ctl18_out => s_img_src(13),
            ctl19_out => s_img_src(14),
            ctl20_out => s_img_src(15),     -- CFG_IMAGE[15]
            ctl21_out => s_ctl21_src,       -- SCAN_TIMEOUT
            ctl22_out => open,              -- reserved
            ctl23_out => open,
            ctl24_out => open,
            ctl25_out => open,
            ctl26_out => open,
            ctl27_out => open,
            ctl28_out => open,
            ctl29_out => open,
            ctl30_out => open,
            ctl31_out => open,
            -- STAT inputs: STAT0~3 = per-chip results (CDC'd); rest tied to zero
            stat0_in  => s_stat_out(0),     -- chip0 result
            stat1_in  => s_stat_out(1),     -- chip1 result
            stat2_in  => s_stat_out(2),     -- chip2 result
            stat3_in  => s_stat_out(3),     -- chip3 result
            stat4_in  => C_ZERO32,          -- reserved
            stat5_in  => C_ZERO32,
            stat6_in  => C_ZERO32,
            stat7_in  => C_ZERO32,
            stat8_in  => C_ZERO32,
            stat9_in  => C_ZERO32,
            stat10_in => C_ZERO32,
            stat11_in => C_ZERO32,          -- was REG_RDATA, now in STAT0-3
            stat12_in => C_ZERO32,  stat13_in => C_ZERO32,
            stat14_in => C_ZERO32,  stat15_in => C_ZERO32,
            stat16_in => C_ZERO32,  stat17_in => C_ZERO32,
            stat18_in => C_ZERO32,  stat19_in => C_ZERO32,
            stat20_in => C_ZERO32,  stat21_in => C_ZERO32,
            stat22_in => C_ZERO32,  stat23_in => C_ZERO32,
            stat24_in => C_ZERO32,  stat25_in => C_ZERO32,
            stat26_in => C_ZERO32,  stat27_in => C_ZERO32,
            stat28_in => C_ZERO32,  stat29_in => C_ZERO32,
            stat30_in => C_ZERO32,  stat31_in => C_ZERO32,
            -- Interrupt: done_pulse CDC'd to s_axi_aclk domain
            intrpt_src_in(0) => s_irq_pulse,
            irq              => o_irq
        );

    -- =========================================================================
    -- [2] CTL1 CDC: s_axi_aclk -> i_axis_aclk (BUS_TIMING)
    -- =========================================================================
    u_cdc_ctl1 : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 0,
            SRC_SYNC_FF    => 4,
            WIDTH          => 32
        )
        port map (
            src_clk   => s_axi_aclk,
            src_in    => s_ctl1_src,
            src_send  => s_src_send_ctl1,
            src_rcv   => s_src_rcv_ctl1,
            dest_clk  => i_axis_aclk,
            dest_req  => s_dest_req_ctl1,
            dest_ack  => s_dest_req_ctl1,
            dest_out  => s_ctl1_out
        );

    p_send_ctl1 : process(s_axi_aclk)
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                s_src_send_ctl1 <= '0';
                s_ctl1_d1       <= (others => '1');
            else
                if s_src_send_ctl1 = '0' and s_ctl1_src /= s_ctl1_d1 then
                    s_src_send_ctl1 <= '1';
                    s_ctl1_d1       <= s_ctl1_src;
                elsif s_src_rcv_ctl1 = '1' then
                    s_src_send_ctl1 <= '0';
                end if;
            end if;
        end if;
    end process p_send_ctl1;

    -- =========================================================================
    -- [3] CTL3 CDC: s_axi_aclk -> i_axis_aclk (START_OFF1)
    -- =========================================================================
    u_cdc_ctl3 : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 0,
            SRC_SYNC_FF    => 4,
            WIDTH          => 32
        )
        port map (
            src_clk   => s_axi_aclk,
            src_in    => s_ctl3_src,
            src_send  => s_src_send_ctl3,
            src_rcv   => s_src_rcv_ctl3,
            dest_clk  => i_axis_aclk,
            dest_req  => s_dest_req_ctl3,
            dest_ack  => s_dest_req_ctl3,
            dest_out  => s_ctl3_out
        );

    p_send_ctl3 : process(s_axi_aclk)
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                s_src_send_ctl3 <= '0';
                s_ctl3_d1       <= (others => '1');
            else
                if s_src_send_ctl3 = '0' and s_ctl3_src /= s_ctl3_d1 then
                    s_src_send_ctl3 <= '1';
                    s_ctl3_d1       <= s_ctl3_src;
                elsif s_src_rcv_ctl3 = '1' then
                    s_src_send_ctl3 <= '0';
                end if;
            end if;
        end if;
    end process p_send_ctl3;

    -- =========================================================================
    -- [4] CTL4 CDC: s_axi_aclk -> i_axis_aclk (CFG_REG7)
    -- =========================================================================
    u_cdc_ctl4 : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 0,
            SRC_SYNC_FF    => 4,
            WIDTH          => 32
        )
        port map (
            src_clk   => s_axi_aclk,
            src_in    => s_ctl4_src,
            src_send  => s_src_send_ctl4,
            src_rcv   => s_src_rcv_ctl4,
            dest_clk  => i_axis_aclk,
            dest_req  => s_dest_req_ctl4,
            dest_ack  => s_dest_req_ctl4,
            dest_out  => s_ctl4_out
        );

    p_send_ctl4 : process(s_axi_aclk)
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                s_src_send_ctl4 <= '0';
                s_ctl4_d1       <= (others => '1');
            else
                if s_src_send_ctl4 = '0' and s_ctl4_src /= s_ctl4_d1 then
                    s_src_send_ctl4 <= '1';
                    s_ctl4_d1       <= s_ctl4_src;
                elsif s_src_rcv_ctl4 = '1' then
                    s_src_send_ctl4 <= '0';
                end if;
            end if;
        end if;
    end process p_send_ctl4;

    -- =========================================================================
    -- [5] CTL21 CDC: s_axi_aclk -> i_axis_aclk (SCAN_TIMEOUT + MAX_HITS)
    -- =========================================================================
    u_cdc_ctl21 : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 0,
            SRC_SYNC_FF    => 4,
            WIDTH          => 32
        )
        port map (
            src_clk   => s_axi_aclk,
            src_in    => s_ctl21_src,
            src_send  => s_src_send_ctl21,
            src_rcv   => s_src_rcv_ctl21,
            dest_clk  => i_axis_aclk,
            dest_req  => s_dest_req_ctl21,
            dest_ack  => s_dest_req_ctl21,
            dest_out  => s_ctl21_out
        );

    p_send_ctl21 : process(s_axi_aclk)
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                s_src_send_ctl21 <= '0';
                s_ctl21_d1       <= (others => '1');
            else
                if s_src_send_ctl21 = '0' and s_ctl21_src /= s_ctl21_d1 then
                    s_src_send_ctl21 <= '1';
                    s_ctl21_d1       <= s_ctl21_src;
                elsif s_src_rcv_ctl21 = '1' then
                    s_src_send_ctl21 <= '0';
                end if;
            end if;
        end if;
    end process p_send_ctl21;

    -- =========================================================================
    -- [6] cfg_image CDC: s_axi_aclk -> i_axis_aclk (CTL5~20, per-register)
    -- =========================================================================
    gen_img_cdc : for i in 0 to c_CFG_IMAGE_N_REGS - 1 generate
        u_cdc_img : xpm_cdc_handshake
            generic map (
                DEST_EXT_HSK   => 1,
                DEST_SYNC_FF   => 4,
                INIT_SYNC_FF   => 0,
                SIM_ASSERT_CHK => 0,
                SRC_SYNC_FF    => 4,
                WIDTH          => 32
            )
            port map (
                src_clk   => s_axi_aclk,
                src_in    => s_img_src(i),
                src_send  => s_src_send_img(i),
                src_rcv   => s_src_rcv_img(i),
                dest_clk  => i_axis_aclk,
                dest_req  => s_dest_req_img(i),
                dest_ack  => s_dest_req_img(i),
                dest_out  => s_img_out(i)
            );

        p_send_img : process(s_axi_aclk)
        begin
            if rising_edge(s_axi_aclk) then
                if s_axi_aresetn = '0' then
                    s_src_send_img(i) <= '0';
                    s_img_d1(i)       <= (others => '1');
                else
                    if s_src_send_img(i) = '0' and s_img_src(i) /= s_img_d1(i) then
                        s_src_send_img(i) <= '1';
                        s_img_d1(i)       <= s_img_src(i);
                    elsif s_src_rcv_img(i) = '1' then
                        s_src_send_img(i) <= '0';
                    end if;
                end if;
            end if;
        end process p_send_img;
    end generate gen_img_cdc;

    -- =========================================================================
    -- [7] Per-chip STAT CDC: i_axis_aclk -> s_axi_aclk (STAT0~3)
    --   Generate 4 xpm_cdc_handshake instances, one per chip.
    --   Each transfers s_reg_stat_r(i) to s_stat_out(i).
    -- =========================================================================
    gen_stat_cdc : for i in 0 to c_N_CHIPS - 1 generate
        signal s_src_send_stat : std_logic := '0';
        signal s_src_rcv_stat  : std_logic;
        signal s_dest_req_stat : std_logic;
        signal s_stat_d1       : std_logic_vector(31 downto 0) := (others => '1');
    begin
        u_cdc_stat : xpm_cdc_handshake
            generic map (
                DEST_EXT_HSK   => 1,
                DEST_SYNC_FF   => 4,
                INIT_SYNC_FF   => 0,
                SIM_ASSERT_CHK => 0,
                SRC_SYNC_FF    => 4,
                WIDTH          => 32
            )
            port map (
                src_clk   => i_axis_aclk,
                src_in    => s_reg_stat_r(i),
                src_send  => s_src_send_stat,
                src_rcv   => s_src_rcv_stat,
                dest_clk  => s_axi_aclk,
                dest_req  => s_dest_req_stat,
                dest_ack  => s_dest_req_stat,
                dest_out  => s_stat_out(i)
            );

        p_send_stat : process(i_axis_aclk)
        begin
            if rising_edge(i_axis_aclk) then
                if i_axis_aresetn = '0' then
                    s_src_send_stat <= '0';
                    s_stat_d1       <= (others => '1');
                else
                    if s_src_send_stat = '0' and s_reg_stat_r(i) /= s_stat_d1 then
                        s_src_send_stat <= '1';
                        s_stat_d1       <= s_reg_stat_r(i);
                    elsif s_src_rcv_stat = '1' then
                        s_src_send_stat <= '0';
                    end if;
                end if;
            end if;
        end process p_send_stat;
    end generate gen_stat_cdc;

    -- =========================================================================
    -- [7b] IRQ CDC: i_axis_aclk -> s_axi_aclk (done_pulse)
    -- =========================================================================
    u_cdc_irq : xpm_cdc_pulse
        generic map (
            DEST_SYNC_FF => 4,
            INIT_SYNC_FF => 0,
            REG_OUTPUT   => 1,
            RST_USED     => 1
        )
        port map (
            src_clk    => i_axis_aclk,
            src_rst    => not i_axis_aresetn,
            src_pulse  => i_cmd_reg_done_pulse,
            dest_clk   => s_axi_aclk,
            dest_rst   => not s_axi_aresetn,
            dest_pulse => s_irq_pulse
        );

    -- =========================================================================
    -- [8] CDC-idle flag: all CTL handshakes quiescent (s_axi_aclk domain)
    --   NOR of ctl1, ctl3, ctl4, img x16, ctl21 src_send signals.
    --   Output directly to csr_pipeline via o_cdc_idle.
    --   Note: STAT CDCs (i_axis->s_axi direction) are NOT included.
    -- =========================================================================
    s_cdc_all_idle_src <= '1' when s_src_send_ctl1  = '0'
                                and s_src_send_ctl3  = '0'
                                and s_src_send_ctl4  = '0'
                                and s_src_send_ctl21 = '0'
                                and s_src_send_img = (s_src_send_img'range => '0')
                          else '0';

    o_cdc_idle <= s_cdc_all_idle_src;

    -- Local 2-FF sync to i_axis_aclk for reg command gating
    p_cdc_idle_sync : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_cdc_all_idle_ff <= "11";
            else
                s_cdc_all_idle_ff(0) <= s_cdc_all_idle_src;
                s_cdc_all_idle_ff(1) <= s_cdc_all_idle_ff(0);
            end if;
        end if;
    end process p_cdc_idle_sync;

    -- =========================================================================
    -- [9] Reg access edge detect (i_axis_aclk domain)
    --   CTL1[30] = reg_read trigger, CTL1[31] = reg_write trigger
    --   Uses existing CTL1 CDC path — no additional CDC instance needed.
    -- =========================================================================
    p_reg_cmd_edge : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_reg_cmd_prev_r  <= (others => '0');
                s_reg_cmd_pulse_r <= (others => '0');
            else
                s_reg_cmd_prev_r  <= s_ctl1_out(31 downto 30);
                s_reg_cmd_pulse_r <= s_ctl1_out(31 downto 30) and (not s_reg_cmd_prev_r);
            end if;
        end if;
    end process p_reg_cmd_edge;

    -- Mutual exclusion: if both CTL1[31:30] rise simultaneously, write wins.
    -- CDC idle gating: ensures cfg_image data is stable before reg access.
    o_cmd_reg_read  <= s_reg_cmd_pulse_r(0) and (not s_reg_cmd_pulse_r(1))
                       and s_cdc_all_idle_ff(1);
    o_cmd_reg_write <= s_reg_cmd_pulse_r(1)
                       and s_cdc_all_idle_ff(1);
    o_cmd_reg_addr  <= s_ctl1_out(c_BT_REG_ADDR_HI downto c_BT_REG_ADDR_LO);
    o_cmd_reg_chip  <= unsigned(s_ctl1_out(c_BT_REG_CHIP_HI downto c_BT_REG_CHIP_LO));

    -- Chip address output from CTL1 CDC (i_axis_aclk domain)
    -- Backward compatibility: if chip_address mask is "0000" (SW only set chip_id),
    -- auto-generate one-hot mask from the 2-bit chip_id field.
    o_cmd_reg_chip_address <=
        s_ctl1_out(c_BT_REG_CHIP_ADDR_HI downto c_BT_REG_CHIP_ADDR_LO)
        when s_ctl1_out(c_BT_REG_CHIP_ADDR_HI downto c_BT_REG_CHIP_ADDR_LO) /= "0000"
        else std_logic_vector(shift_left(to_unsigned(1, c_N_CHIPS),
             to_integer(unsigned(s_ctl1_out(c_BT_REG_CHIP_HI downto c_BT_REG_CHIP_LO)))));

    -- =========================================================================
    -- [10] Per-chip reg read data latch (i_axis_aclk domain)
    --   i_cmd_reg_rvalid(i) -> latch i_cmd_reg_rdata_i -> feeds STAT(i) CDC
    --   For write operations, rvalid fires but rdata=0, so STAT[27:0]=0.
    --   STAT[31:28] = addr_done (always present).
    -- =========================================================================
    p_reg_rdata_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_reg_stat_r <= (others => (others => '0'));
            else
                -- Per-chip: latch on individual rvalid
                if i_cmd_reg_rvalid(0) = '1' then
                    s_reg_stat_r(0)(27 downto 0)  <= i_cmd_reg_rdata_0;
                    s_reg_stat_r(0)(31 downto 28) <= i_cmd_reg_addr_done;
                end if;
                if i_cmd_reg_rvalid(1) = '1' then
                    s_reg_stat_r(1)(27 downto 0)  <= i_cmd_reg_rdata_1;
                    s_reg_stat_r(1)(31 downto 28) <= i_cmd_reg_addr_done;
                end if;
                if i_cmd_reg_rvalid(2) = '1' then
                    s_reg_stat_r(2)(27 downto 0)  <= i_cmd_reg_rdata_2;
                    s_reg_stat_r(2)(31 downto 28) <= i_cmd_reg_addr_done;
                end if;
                if i_cmd_reg_rvalid(3) = '1' then
                    s_reg_stat_r(3)(27 downto 0)  <= i_cmd_reg_rdata_3;
                    s_reg_stat_r(3)(31 downto 28) <= i_cmd_reg_addr_done;
                end if;
            end if;
        end if;
    end process p_reg_rdata_latch;

    -- =========================================================================
    -- [11] Configuration output extraction (i_axis_aclk domain)
    -- =========================================================================

    -- Bus timing clamping ---------------------------------------------------
    -- bus_clk_div: clamp >= c_BUS_CLK_DIV_MIN (2)
    s_div_clamped <= unsigned(s_ctl1_out(c_BT_CLK_DIV_HI downto c_BT_CLK_DIV_LO))
                     when unsigned(s_ctl1_out(c_BT_CLK_DIV_HI downto c_BT_CLK_DIV_LO))
                          >= c_BUS_CLK_DIV_MIN
                     else to_unsigned(c_BUS_CLK_DIV_MIN, 6);
    o_bus_clk_div <= s_div_clamped;

    -- bus_ticks: minimum ticks>=4 (div>=2 always, div=1 prohibited)
    s_ticks_min   <= to_unsigned(c_BUS_TICKS_MIN, 3);
    o_bus_ticks   <= unsigned(s_ctl1_out(c_BT_TICKS_HI downto c_BT_TICKS_LO))
                     when unsigned(s_ctl1_out(c_BT_TICKS_HI downto c_BT_TICKS_LO))
                          >= s_ticks_min
                     else s_ticks_min;

    -- START_OFF1 ------------------------------------------------------------
    o_start_off1  <= unsigned(s_ctl3_out(17 downto 0));

    -- CFG_REG7 --------------------------------------------------------------
    o_cfg_reg7    <= s_ctl4_out;

    -- SCAN_TIMEOUT + MAX_HITS -----------------------------------------------
    o_max_scan_clks <= unsigned(s_ctl21_out(c_ST_MAX_SCAN_HI downto c_ST_MAX_SCAN_LO));

    -- max_hits_cfg: 0 -> c_MAX_HITS_PER_STOP (7), 1~7 = literal
    o_max_hits_cfg  <= to_unsigned(c_MAX_HITS_PER_STOP, 3)
                       when unsigned(s_ctl21_out(c_ST_MAX_HITS_HI downto c_ST_MAX_HITS_LO)) = 0
                       else unsigned(s_ctl21_out(c_ST_MAX_HITS_HI downto c_ST_MAX_HITS_LO));

    -- cfg_image output ------------------------------------------------------
    o_cfg_image <= s_img_out;

end architecture rtl;
