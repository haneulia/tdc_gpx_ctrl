-- =============================================================================
-- tdc_gpx_csr.vhd
-- TDC-GPX Controller - AXI4-Lite CSR Wrapper + CDC
-- (based on my_axil_csr32, echo_receiver_csr pattern)
-- =============================================================================
--
-- Function:
--   AXI4-Lite register interface wrapping tdc_gpx_axil_csr32 (32 CTL + 12 STAT)
--   with cross-clock-domain transfer to the TDC processing domain (i_axis_aclk).
--
-- Register map (9-bit address, 32 CTL + 11 STAT):
--   CTL0  (0x00) MAIN_CTRL     packed: [3:0] active_chip_mask, [4] packet_scope,
--                               [6:5] hit_store_mode, [9:7] dist_scale,
--                               [10] drain_mode, [11] pipeline_en,
--                               [14:12] n_faces, [18:15] stops_per_chip,
--                               [22:19] n_drain_cap, [27:23] stopdis_override,
--                               [31:28] COMMAND
--   CTL1  (0x04) BUS_TIMING    [5:0] bus_clk_div, [8:6] bus_ticks
--   CTL2  (0x08) RANGE_COLS    [15:0] max_range_clks, [31:16] cols_per_face
--   CTL3  (0x0C) START_OFF1    [17:0]
--   CTL4  (0x10) CFG_REG7      [31:0]
--   CTL5..20 (0x14~0x50) CFG_IMAGE[0..15] [31:0] each
--   CTL21..31 reserved
--
--   STAT0  (0x80) HW_VERSION       [31:0] (constant)
--   STAT1  (0x84) HW_CONFIG        packed generics (constant)
--   STAT2  (0x88) MAX_ROWS         [15:0] (constant)
--   STAT3  (0x8C) CELL_SIZE        [15:0] (constant)
--   STAT4  (0x90) MAX_HSIZE        [15:0] (constant)
--   STAT5  (0x94) STATUS           [0] busy, [1] overrun, [2] bin_mismatch, [7:4] chip_err
--   STAT6  (0x98) SHOT_SEQ         [15:0]
--   STAT7  (0x9C) FRAME_COUNT      [31:0]
--   STAT8  (0xA0) ERROR_COUNT      [31:0]
--   STAT9  (0xA4) BIN_PS           [15:0]
--   STAT10 (0xA8) K_DIST           [31:0]
--   STAT11 (0xAC) REG_RDATA       [27:0] chip register read data
--
-- CDC structure:
--   CTL: 5 x xpm_cdc_handshake (s_axi_aclk -> i_axis_aclk) for CTL0~4
--        16 x xpm_cdc_handshake (s_axi_aclk -> i_axis_aclk) for cfg_image CTL5~20
--   STAT: 7 x xpm_cdc_handshake (i_axis_aclk -> s_axi_aclk) for STAT5~11
--   Total: 27 CDC instances
--
-- Clock domains:
--   s_axi_aclk : AXI4-Lite domain (PS clock)
--   i_axis_aclk : TDC processing / AXI-Stream domain (200 MHz)
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

library xpm;
use xpm.vcomponents.all;

entity tdc_gpx_csr is
    generic (
        g_HW_VERSION : std_logic_vector(31 downto 0) := x"00010000"
    );
    port (
        -- AXI4-Lite clock / reset
        s_axi_aclk          : in  std_logic;
        s_axi_aresetn       : in  std_logic;

        -- AXI4-Lite Slave (9-bit address)
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

        -- laser_ctrl_result stream input (i_axis_aclk domain)
        -- tdata[15:0] → cols_per_face override (latched on tvalid)
        i_lsr_tvalid        : in  std_logic;
        i_lsr_tdata         : in  std_logic_vector(31 downto 0);

        -- Configuration output (i_axis_aclk domain)
        o_cfg               : out t_tdc_cfg;
        o_cfg_image         : out t_cfg_image;

        -- Command pulses (i_axis_aclk domain, 1-clk, rising-edge detect)
        o_cmd_start         : out std_logic;
        o_cmd_stop          : out std_logic;
        o_cmd_soft_reset    : out std_logic;
        o_cmd_cfg_write     : out std_logic;

        -- Individual TDC-GPX register access (i_axis_aclk domain)
        -- Trigger: edge-detected from CTL1[31:30]
        -- Addr/chip: from CTL1[13:10] / CTL1[15:14]
        -- Write data: from cfg_image[target_addr] (routed in TOP)
        -- Read data: from chip_ctrl response → STAT11
        o_cmd_reg_read      : out std_logic;        -- 1-clk pulse
        o_cmd_reg_write     : out std_logic;         -- 1-clk pulse
        o_cmd_reg_addr      : out std_logic_vector(3 downto 0);
        o_cmd_reg_chip      : out unsigned(1 downto 0);
        i_cmd_reg_rdata     : in  std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        i_cmd_reg_rvalid    : in  std_logic;

        -- Status input (i_axis_aclk domain)
        i_status            : in  t_tdc_status;
        i_bin_resolution_ps : in  unsigned(15 downto 0);
        i_k_dist_fixed      : in  unsigned(31 downto 0);

        -- Interrupt
        o_irq               : out std_logic
    );
end entity tdc_gpx_csr;

architecture rtl of tdc_gpx_csr is

    -- =========================================================================
    -- tdc_gpx_axil_csr32 component (Vivado IP: 32 CTL, 12 STAT, 1 IRQ)
    -- =========================================================================
    component tdc_gpx_axil_csr32 is
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
            reg0_init_val       : in  std_logic_vector(31 downto 0);
            reg1_init_val       : in  std_logic_vector(31 downto 0);
            reg2_init_val       : in  std_logic_vector(31 downto 0);
            reg3_init_val       : in  std_logic_vector(31 downto 0);
            reg4_init_val       : in  std_logic_vector(31 downto 0);
            reg5_init_val       : in  std_logic_vector(31 downto 0);
            reg6_init_val       : in  std_logic_vector(31 downto 0);
            reg7_init_val       : in  std_logic_vector(31 downto 0);
            reg8_init_val       : in  std_logic_vector(31 downto 0);
            reg9_init_val       : in  std_logic_vector(31 downto 0);
            reg10_init_val      : in  std_logic_vector(31 downto 0);
            reg11_init_val      : in  std_logic_vector(31 downto 0);
            reg12_init_val      : in  std_logic_vector(31 downto 0);
            reg13_init_val      : in  std_logic_vector(31 downto 0);
            reg14_init_val      : in  std_logic_vector(31 downto 0);
            reg15_init_val      : in  std_logic_vector(31 downto 0);
            reg16_init_val      : in  std_logic_vector(31 downto 0);
            reg17_init_val      : in  std_logic_vector(31 downto 0);
            reg18_init_val      : in  std_logic_vector(31 downto 0);
            reg19_init_val      : in  std_logic_vector(31 downto 0);
            reg20_init_val      : in  std_logic_vector(31 downto 0);
            reg21_init_val      : in  std_logic_vector(31 downto 0);
            reg22_init_val      : in  std_logic_vector(31 downto 0);
            reg23_init_val      : in  std_logic_vector(31 downto 0);
            reg24_init_val      : in  std_logic_vector(31 downto 0);
            reg25_init_val      : in  std_logic_vector(31 downto 0);
            reg26_init_val      : in  std_logic_vector(31 downto 0);
            reg27_init_val      : in  std_logic_vector(31 downto 0);
            reg28_init_val      : in  std_logic_vector(31 downto 0);
            reg29_init_val      : in  std_logic_vector(31 downto 0);
            reg30_init_val      : in  std_logic_vector(31 downto 0);
            reg31_init_val      : in  std_logic_vector(31 downto 0);
            -- CTL outputs (32)
            ctl0_out            : out std_logic_vector(31 downto 0);
            ctl1_out            : out std_logic_vector(31 downto 0);
            ctl2_out            : out std_logic_vector(31 downto 0);
            ctl3_out            : out std_logic_vector(31 downto 0);
            ctl4_out            : out std_logic_vector(31 downto 0);
            ctl5_out            : out std_logic_vector(31 downto 0);
            ctl6_out            : out std_logic_vector(31 downto 0);
            ctl7_out            : out std_logic_vector(31 downto 0);
            ctl8_out            : out std_logic_vector(31 downto 0);
            ctl9_out            : out std_logic_vector(31 downto 0);
            ctl10_out           : out std_logic_vector(31 downto 0);
            ctl11_out           : out std_logic_vector(31 downto 0);
            ctl12_out           : out std_logic_vector(31 downto 0);
            ctl13_out           : out std_logic_vector(31 downto 0);
            ctl14_out           : out std_logic_vector(31 downto 0);
            ctl15_out           : out std_logic_vector(31 downto 0);
            ctl16_out           : out std_logic_vector(31 downto 0);
            ctl17_out           : out std_logic_vector(31 downto 0);
            ctl18_out           : out std_logic_vector(31 downto 0);
            ctl19_out           : out std_logic_vector(31 downto 0);
            ctl20_out           : out std_logic_vector(31 downto 0);
            ctl21_out           : out std_logic_vector(31 downto 0);
            ctl22_out           : out std_logic_vector(31 downto 0);
            ctl23_out           : out std_logic_vector(31 downto 0);
            ctl24_out           : out std_logic_vector(31 downto 0);
            ctl25_out           : out std_logic_vector(31 downto 0);
            ctl26_out           : out std_logic_vector(31 downto 0);
            ctl27_out           : out std_logic_vector(31 downto 0);
            ctl28_out           : out std_logic_vector(31 downto 0);
            ctl29_out           : out std_logic_vector(31 downto 0);
            ctl30_out           : out std_logic_vector(31 downto 0);
            ctl31_out           : out std_logic_vector(31 downto 0);
            -- STAT inputs (12)
            stat0_in            : in  std_logic_vector(31 downto 0);
            stat1_in            : in  std_logic_vector(31 downto 0);
            stat2_in            : in  std_logic_vector(31 downto 0);
            stat3_in            : in  std_logic_vector(31 downto 0);
            stat4_in            : in  std_logic_vector(31 downto 0);
            stat5_in            : in  std_logic_vector(31 downto 0);
            stat6_in            : in  std_logic_vector(31 downto 0);
            stat7_in            : in  std_logic_vector(31 downto 0);
            stat8_in            : in  std_logic_vector(31 downto 0);
            stat9_in            : in  std_logic_vector(31 downto 0);
            stat10_in           : in  std_logic_vector(31 downto 0);
            stat11_in           : in  std_logic_vector(31 downto 0);
            -- Interrupt
            intrpt_src_in       : in  std_logic_vector(0 downto 0);
            irq                 : out std_logic
        );
    end component tdc_gpx_axil_csr32;

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant C_NUM_STAT_CDC : natural := 7;     -- STAT5~11: live status CDC
    constant C_ZERO32       : std_logic_vector(31 downto 0) := (others => '0');

    -- =========================================================================
    -- Internal types
    -- =========================================================================
    type t_cdc_data_array is array(natural range <>) of std_logic_vector(31 downto 0);

    -- =========================================================================
    -- CTL raw outputs (s_axi_aclk domain)
    -- =========================================================================
    signal s_ctl_src : t_cdc_data_array(0 to c_NUM_CTL_REGS - 1);

    -- CTL after CDC (i_axis_aclk domain) — CTL0~4 (5 active registers)
    signal s_ctl_out : t_cdc_data_array(0 to c_NUM_CTL_CDC - 1) := (others => C_ZERO32);

    -- cfg_image after CDC (i_axis_aclk domain) — CTL5~20 (16 registers)
    signal s_img_out : t_cfg_image := (others => C_ZERO32);

    -- CTL CDC handshake (CTL0~4)
    signal s_src_send_ctl : std_logic_vector(c_NUM_CTL_CDC - 1 downto 0) := (others => '0');
    signal s_src_rcv_ctl  : std_logic_vector(c_NUM_CTL_CDC - 1 downto 0);
    signal s_dest_req_ctl : std_logic_vector(c_NUM_CTL_CDC - 1 downto 0);
    signal s_ctl_d1       : t_cdc_data_array(0 to c_NUM_CTL_CDC - 1) := (others => (others => '1'));

    -- cfg_image CDC handshake (CTL5~20, per-register)
    signal s_src_send_img : std_logic_vector(c_CFG_IMAGE_N_REGS - 1 downto 0) := (others => '0');
    signal s_src_rcv_img  : std_logic_vector(c_CFG_IMAGE_N_REGS - 1 downto 0);
    signal s_dest_req_img : std_logic_vector(c_CFG_IMAGE_N_REGS - 1 downto 0);
    signal s_img_d1       : t_cdc_data_array(0 to c_CFG_IMAGE_N_REGS - 1) := (others => (others => '1'));

    -- STAT source (i_axis_aclk domain, 6 live registers)
    signal s_stat_src : t_cdc_data_array(0 to C_NUM_STAT_CDC - 1);

    -- STAT after CDC (s_axi_aclk domain)
    signal s_stat_out : t_cdc_data_array(0 to C_NUM_STAT_CDC - 1) := (others => C_ZERO32);

    -- STAT CDC handshake
    signal s_src_send_stat : std_logic_vector(C_NUM_STAT_CDC - 1 downto 0) := (others => '0');
    signal s_src_rcv_stat  : std_logic_vector(C_NUM_STAT_CDC - 1 downto 0);
    signal s_dest_req_stat : std_logic_vector(C_NUM_STAT_CDC - 1 downto 0);
    signal s_stat_d1       : t_cdc_data_array(0 to C_NUM_STAT_CDC - 1) := (others => (others => '1'));

    -- HW_CONFIG constant (compile-time)
    signal s_hw_config : std_logic_vector(31 downto 0);

    -- Command edge detect (i_axis_aclk domain)
    signal s_cmd_prev_r  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cmd_pulse_r : std_logic_vector(3 downto 0) := (others => '0');

    -- Reg access edge detect: CTL1[31:30] (i_axis_aclk domain)
    signal s_reg_cmd_prev_r  : std_logic_vector(1 downto 0) := (others => '0');
    signal s_reg_cmd_pulse_r : std_logic_vector(1 downto 0) := (others => '0');

    -- Reg access read data latch (i_axis_aclk domain)
    signal s_reg_rdata_r     : std_logic_vector(31 downto 0) := (others => '0');

    -- laser_ctrl cols_per_face latch (i_axis_aclk domain)
    signal s_lsr_cols_r  : unsigned(15 downto 0) := (others => '0');
    signal s_lsr_valid_r : std_logic := '0';    -- '1' after first tvalid

begin

    -- =========================================================================
    -- [1] HW_CONFIG constant assembly
    -- =========================================================================
    s_hw_config(c_HWCFG_N_CHIPS_HI downto c_HWCFG_N_CHIPS_LO)
        <= std_logic_vector(to_unsigned(c_N_CHIPS, 4));
    s_hw_config(c_HWCFG_MAX_STOPS_HI downto c_HWCFG_MAX_STOPS_LO)
        <= std_logic_vector(to_unsigned(c_MAX_STOPS_PER_CHIP, 4));
    s_hw_config(c_HWCFG_MAX_HITS_HI downto c_HWCFG_MAX_HITS_LO)
        <= std_logic_vector(to_unsigned(c_MAX_HITS_PER_STOP, 4));
    s_hw_config(c_HWCFG_HIT_WIDTH_HI downto c_HWCFG_HIT_WIDTH_LO)
        <= std_logic_vector(to_unsigned(c_HIT_SLOT_DATA_WIDTH, 5));
    s_hw_config(c_HWCFG_TDATA_HI downto c_HWCFG_TDATA_LO)
        <= std_logic_vector(to_unsigned(c_TDATA_WIDTH, 8));
    s_hw_config(c_HWCFG_CELL_FMT_HI downto c_HWCFG_CELL_FMT_LO)
        <= std_logic_vector(to_unsigned(c_CELL_FORMAT, 3));
    s_hw_config(31 downto c_HWCFG_CELL_FMT_HI + 1) <= (others => '0');

    -- =========================================================================
    -- [2] tdc_gpx_axil_csr32 instantiation (32 CTL, 12 STAT)
    -- =========================================================================
    u_csr : tdc_gpx_axil_csr32
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
            -- Init values: CTL0~4 active, CTL5~20 cfg_image (zero), CTL21~31 reserved
            reg0_init_val  => c_INIT_MAIN_CTRL,
            reg1_init_val  => c_INIT_BUS_TIMING,
            reg2_init_val  => c_INIT_RANGE_COLS,
            reg3_init_val  => c_INIT_START_OFF1,
            reg4_init_val  => c_INIT_CFG_REG7,
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
            -- CTL outputs → s_ctl_src array
            ctl0_out  => s_ctl_src(0),    ctl1_out  => s_ctl_src(1),
            ctl2_out  => s_ctl_src(2),    ctl3_out  => s_ctl_src(3),
            ctl4_out  => s_ctl_src(4),    ctl5_out  => s_ctl_src(5),
            ctl6_out  => s_ctl_src(6),    ctl7_out  => s_ctl_src(7),
            ctl8_out  => s_ctl_src(8),    ctl9_out  => s_ctl_src(9),
            ctl10_out => s_ctl_src(10),   ctl11_out => s_ctl_src(11),
            ctl12_out => s_ctl_src(12),   ctl13_out => s_ctl_src(13),
            ctl14_out => s_ctl_src(14),   ctl15_out => s_ctl_src(15),
            ctl16_out => s_ctl_src(16),   ctl17_out => s_ctl_src(17),
            ctl18_out => s_ctl_src(18),   ctl19_out => s_ctl_src(19),
            ctl20_out => s_ctl_src(20),   ctl21_out => s_ctl_src(21),
            ctl22_out => s_ctl_src(22),   ctl23_out => s_ctl_src(23),
            ctl24_out => s_ctl_src(24),   ctl25_out => s_ctl_src(25),
            ctl26_out => s_ctl_src(26),   ctl27_out => s_ctl_src(27),
            ctl28_out => s_ctl_src(28),   ctl29_out => s_ctl_src(29),
            ctl30_out => s_ctl_src(30),   ctl31_out => s_ctl_src(31),
            -- STAT inputs: STAT0~4 = constants (s_axi_aclk domain, no CDC)
            stat0_in  => g_HW_VERSION,
            stat1_in  => s_hw_config,
            stat2_in  => std_logic_vector(to_unsigned(c_MAX_ROWS_PER_FACE, 32)),
            stat3_in  => std_logic_vector(to_unsigned(c_CELL_SIZE_BYTES, 32)),
            stat4_in  => std_logic_vector(to_unsigned(c_HSIZE_MAX, 32)),
            -- STAT5~10 = live status (CDC'd from i_axis_aclk)
            stat5_in  => s_stat_out(0),
            stat6_in  => s_stat_out(1),
            stat7_in  => s_stat_out(2),
            stat8_in  => s_stat_out(3),
            stat9_in  => s_stat_out(4),
            stat10_in => s_stat_out(5),
            stat11_in => s_stat_out(6),  -- REG_RDATA: chip register readback
            -- Interrupt
            intrpt_src_in => "0",
            irq           => o_irq
        );

    -- =========================================================================
    -- [3] STAT source packing (i_axis_aclk domain)
    -- =========================================================================
    -- STAT5 = STATUS word
    s_stat_src(0)(c_STAT_BUSY)          <= i_status.busy;
    s_stat_src(0)(c_STAT_OVERRUN)       <= i_status.pipeline_overrun;
    s_stat_src(0)(c_STAT_BIN_MISMATCH)  <= i_status.bin_mismatch;
    s_stat_src(0)(3) <= '0';
    s_stat_src(0)(c_STAT_CHIP_ERR_HI downto c_STAT_CHIP_ERR_LO) <= i_status.chip_error_mask;
    s_stat_src(0)(c_STAT_DRAIN_TO_HI downto c_STAT_DRAIN_TO_LO) <= i_status.drain_timeout_mask;
    s_stat_src(0)(c_STAT_SEQ_ERR_HI downto c_STAT_SEQ_ERR_LO)   <= i_status.sequence_error_mask;
    s_stat_src(0)(31 downto c_STAT_SEQ_ERR_HI + 1) <= (others => '0');

    -- STAT6 = SHOT_SEQ
    s_stat_src(1)(c_SHOT_SEQ_WIDTH - 1 downto 0)    <= std_logic_vector(i_status.shot_seq_current);
    s_stat_src(1)(31 downto c_SHOT_SEQ_WIDTH)       <= (others => '0');

    -- STAT7 = FRAME_COUNT
    s_stat_src(2) <= std_logic_vector(i_status.vdma_frame_count);

    -- STAT8 = ERROR_COUNT
    s_stat_src(3) <= std_logic_vector(i_status.error_count);

    -- STAT9 = BIN_PS
    s_stat_src(4)(15 downto 0)  <= std_logic_vector(i_bin_resolution_ps);
    s_stat_src(4)(31 downto 16) <= (others => '0');

    -- STAT10 = K_DIST
    s_stat_src(5) <= std_logic_vector(i_k_dist_fixed);

    -- STAT11 = REG_RDATA (chip register read data, latched by p_reg_rdata_latch)
    s_stat_src(6) <= s_reg_rdata_r;

    -- =========================================================================
    -- [4] STAT CDC: i_axis_aclk -> s_axi_aclk (7 live registers)
    -- =========================================================================
    gen_stat_cdc : for i in 0 to C_NUM_STAT_CDC - 1 generate
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
                src_in    => s_stat_src(i),
                src_send  => s_src_send_stat(i),
                src_rcv   => s_src_rcv_stat(i),
                dest_clk  => s_axi_aclk,
                dest_req  => s_dest_req_stat(i),
                dest_ack  => s_dest_req_stat(i),
                dest_out  => s_stat_out(i)
            );

        p_send_stat : process(i_axis_aclk)
        begin
            if rising_edge(i_axis_aclk) then
                if i_axis_aresetn = '0' then
                    s_src_send_stat(i) <= '0';
                    s_stat_d1(i)       <= (others => '1');
                else
                    if s_src_send_stat(i) = '0' and s_stat_src(i) /= s_stat_d1(i) then
                        s_src_send_stat(i) <= '1';
                        s_stat_d1(i)       <= s_stat_src(i);
                    elsif s_src_rcv_stat(i) = '1' then
                        s_src_send_stat(i) <= '0';
                    end if;
                end if;
            end if;
        end process p_send_stat;
    end generate gen_stat_cdc;

    -- =========================================================================
    -- [5] CTL CDC: s_axi_aclk -> i_axis_aclk (CTL0~4: 5 active control registers)
    -- =========================================================================
    gen_ctl_cdc : for i in 0 to c_NUM_CTL_CDC - 1 generate
        u_cdc_ctl : xpm_cdc_handshake
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
                src_in    => s_ctl_src(i),
                src_send  => s_src_send_ctl(i),
                src_rcv   => s_src_rcv_ctl(i),
                dest_clk  => i_axis_aclk,
                dest_req  => s_dest_req_ctl(i),
                dest_ack  => s_dest_req_ctl(i),
                dest_out  => s_ctl_out(i)
            );

        p_send_ctl : process(s_axi_aclk)
        begin
            if rising_edge(s_axi_aclk) then
                if s_axi_aresetn = '0' then
                    s_src_send_ctl(i) <= '0';
                    s_ctl_d1(i)       <= (others => '1');
                else
                    if s_src_send_ctl(i) = '0' and s_ctl_src(i) /= s_ctl_d1(i) then
                        s_src_send_ctl(i) <= '1';
                        s_ctl_d1(i)       <= s_ctl_src(i);
                    elsif s_src_rcv_ctl(i) = '1' then
                        s_src_send_ctl(i) <= '0';
                    end if;
                end if;
            end if;
        end process p_send_ctl;
    end generate gen_ctl_cdc;

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
                src_in    => s_ctl_src(5 + i),      -- CTL5~20
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
                    if s_src_send_img(i) = '0' and s_ctl_src(5 + i) /= s_img_d1(i) then
                        s_src_send_img(i) <= '1';
                        s_img_d1(i)       <= s_ctl_src(5 + i);
                    elsif s_src_rcv_img(i) = '1' then
                        s_src_send_img(i) <= '0';
                    end if;
                end if;
            end if;
        end process p_send_img;
    end generate gen_img_cdc;

    -- =========================================================================
    -- [7] Command edge detect (i_axis_aclk domain)
    --   CTL0[31:28] = {cfg_write, soft_reset, stop, start}
    --   Rising edge in i_axis_aclk domain → 1-clk pulse
    -- =========================================================================
    p_cmd_edge : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_cmd_prev_r  <= (others => '0');
                s_cmd_pulse_r <= (others => '0');
            else
                s_cmd_prev_r  <= s_ctl_out(0)(31 downto 28);
                s_cmd_pulse_r <= s_ctl_out(0)(31 downto 28) and (not s_cmd_prev_r);
            end if;
        end if;
    end process p_cmd_edge;

    o_cmd_start      <= s_cmd_pulse_r(0);   -- CTL0[28]
    o_cmd_stop       <= s_cmd_pulse_r(1);   -- CTL0[29]
    o_cmd_soft_reset <= s_cmd_pulse_r(2);   -- CTL0[30]
    o_cmd_cfg_write  <= s_cmd_pulse_r(3);   -- CTL0[31]

    -- =========================================================================
    -- [7a] Reg access edge detect (i_axis_aclk domain)
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
                s_reg_cmd_prev_r  <= s_ctl_out(1)(31 downto 30);
                s_reg_cmd_pulse_r <= s_ctl_out(1)(31 downto 30) and (not s_reg_cmd_prev_r);
            end if;
        end if;
    end process p_reg_cmd_edge;

    o_cmd_reg_read  <= s_reg_cmd_pulse_r(0);    -- CTL1[30]
    o_cmd_reg_write <= s_reg_cmd_pulse_r(1);    -- CTL1[31]
    o_cmd_reg_addr  <= s_ctl_out(1)(c_BT_REG_ADDR_HI downto c_BT_REG_ADDR_LO);
    o_cmd_reg_chip  <= unsigned(s_ctl_out(1)(c_BT_REG_CHIP_HI downto c_BT_REG_CHIP_LO));

    -- Reg read data latch (i_axis_aclk domain): capture for STAT11 readback.
    -- SW reads 0xAC (STAT11) to get the last chip register read result.
    -- CDC path: s_reg_rdata_r → s_stat_src(6) → xpm_cdc → stat11_in.
    p_reg_rdata_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_reg_rdata_r <= (others => '0');
            else
                if i_cmd_reg_rvalid = '1' then
                    s_reg_rdata_r(c_TDC_BUS_WIDTH - 1 downto 0) <= i_cmd_reg_rdata;
                    s_reg_rdata_r(31 downto c_TDC_BUS_WIDTH)     <= (others => '0');
                end if;
            end if;
        end if;
    end process p_reg_rdata_latch;

    -- =========================================================================
    -- [8] laser_ctrl cols_per_face latch (i_axis_aclk domain)
    --   On tvalid, latch tdata[15:0] as cols_per_face override.
    --   When s_lsr_valid_r='1', overrides CTL2[31:16]; otherwise CSR value.
    -- =========================================================================
    p_lsr_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_lsr_cols_r  <= (others => '0');
                s_lsr_valid_r <= '0';
            else
                if i_lsr_tvalid = '1' then
                    s_lsr_cols_r  <= unsigned(i_lsr_tdata(15 downto 0));
                    s_lsr_valid_r <= '1';
                end if;
            end if;
        end if;
    end process p_lsr_latch;

    -- =========================================================================
    -- [9] CSR output: t_tdc_cfg field extraction (i_axis_aclk domain)
    --
    -- Safety clamping applied at CSR boundary so all downstream modules
    -- receive valid values.  Minimum constraints:
    --   active_chip_mask != 0  (face_assembler deadlocks on zero mask)
    --   stops_per_chip  [2..8] (cell_builder/assembler use low 3 bits)
    --   n_faces         >= 1  (face_seq does n_faces-1)
    --   bus_ticks       >= 3  (bus_phy does ticks-2)
    --   bus_clk_div     >= 2  (200 MHz: div=1 → 5 ns < 6 ns datasheet min)
    --   cols_per_face   >= 1  (header_inserter does cols-1)
    -- =========================================================================
    -- CTL0: MAIN_CTRL packed fields
    -- active_chip_mask: clamp >= "0001" (at least 1 chip must be active,
    -- otherwise face_assembler deadlocks in ST_SCAN)
    o_cfg.active_chip_mask <= s_ctl_out(0)(c_MC_ACTIVE_MASK_HI downto c_MC_ACTIVE_MASK_LO)
                              when s_ctl_out(0)(c_MC_ACTIVE_MASK_HI downto c_MC_ACTIVE_MASK_LO) /= "0000"
                              else "0001";
    o_cfg.packet_scope     <= s_ctl_out(0)(c_MC_PACKET_SCOPE);
    o_cfg.hit_store_mode   <= unsigned(s_ctl_out(0)(c_MC_HIT_STORE_HI downto c_MC_HIT_STORE_LO));
    o_cfg.dist_scale       <= unsigned(s_ctl_out(0)(c_MC_DIST_SCALE_HI downto c_MC_DIST_SCALE_LO));
    o_cfg.drain_mode       <= s_ctl_out(0)(c_MC_DRAIN_MODE);
    o_cfg.pipeline_en      <= s_ctl_out(0)(c_MC_PIPELINE_EN);

    -- n_faces: clamp >= 1 (face_seq does n_faces-1 comparison)
    o_cfg.n_faces          <= unsigned(s_ctl_out(0)(c_MC_N_FACES_HI downto c_MC_N_FACES_LO))
                              when unsigned(s_ctl_out(0)(c_MC_N_FACES_HI downto c_MC_N_FACES_LO)) >= 1
                              else to_unsigned(1, 3);

    -- stops_per_chip: clamp [2..8] (cell_builder/assembler use stops(2:0)-1;
    --   values 9..15 alias to 1..7 after truncation, breaking IFIFO routing)
    o_cfg.stops_per_chip   <= unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO))
                              when unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) >= 2
                                   and unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) <= 8
                              else to_unsigned(8, 4)
                              when unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) > 8
                              else to_unsigned(2, 4);

    o_cfg.n_drain_cap      <= unsigned(s_ctl_out(0)(c_MC_N_DRAIN_CAP_HI downto c_MC_N_DRAIN_CAP_LO));
    o_cfg.stopdis_override <= s_ctl_out(0)(c_MC_STOPDIS_HI downto c_MC_STOPDIS_LO);

    -- CTL1: BUS_TIMING
    -- bus_clk_div: clamp >= 2 (at 200 MHz, div=1 → 5 ns period < 6 ns minimum
    --   strobe width per TDC-GPX datasheet; div=2 → 10 ns is safe)
    o_cfg.bus_clk_div      <= unsigned(s_ctl_out(1)(c_BT_CLK_DIV_HI downto c_BT_CLK_DIV_LO))
                              when unsigned(s_ctl_out(1)(c_BT_CLK_DIV_HI downto c_BT_CLK_DIV_LO)) >= 2
                              else to_unsigned(2, 6);

    -- bus_ticks: clamp >= 3 (bus_phy does ticks-2, needs Phase A+L+H minimum)
    o_cfg.bus_ticks        <= unsigned(s_ctl_out(1)(c_BT_TICKS_HI downto c_BT_TICKS_LO))
                              when unsigned(s_ctl_out(1)(c_BT_TICKS_HI downto c_BT_TICKS_LO)) >= 3
                              else to_unsigned(3, 3);

    -- CTL2: RANGE_COLS (cols_per_face overridden by laser_ctrl when valid)
    o_cfg.max_range_clks   <= unsigned(s_ctl_out(2)(c_RC_MAX_RANGE_HI downto c_RC_MAX_RANGE_LO));

    -- cols_per_face: clamp >= 1 (header_inserter does cols-1, underflows to 65535)
    o_cfg.cols_per_face    <= s_lsr_cols_r
                              when s_lsr_valid_r = '1' and s_lsr_cols_r >= 1
                              else unsigned(s_ctl_out(2)(c_RC_COLS_HI downto c_RC_COLS_LO))
                              when unsigned(s_ctl_out(2)(c_RC_COLS_HI downto c_RC_COLS_LO)) >= 1
                              else to_unsigned(1, 16);

    -- CTL3: START_OFF1
    o_cfg.start_off1       <= unsigned(s_ctl_out(3)(17 downto 0));

    -- CTL4: CFG_REG7
    o_cfg.cfg_reg7         <= s_ctl_out(4);

    -- =========================================================================
    -- [10] CSR output: t_cfg_image (i_axis_aclk domain)
    -- =========================================================================
    o_cfg_image <= s_img_out;

end architecture rtl;
