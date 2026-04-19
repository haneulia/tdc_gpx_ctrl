-- =============================================================================
-- tdc_gpx_csr_pipeline.vhd
-- TDC-GPX Controller - Pipeline / Shot Control CSR
-- =============================================================================
--
-- Purpose:
--   AXI4-Lite register interface for pipeline and shot control.
--   Uses tdc_gpx_axil_csr_pipeline IP (8 CTL + 8 STAT, 7-bit address).
--   CDC transfers control registers to i_axis_aclk domain.
--
-- Register map:
--   CTL0  (0x00) MAIN_CTRL     packed: [3:0] active_chip_mask, [4] packet_scope,
--                               [6:5] hit_store_mode, [9:7] dist_scale,
--                               [10] drain_mode, [11] pipeline_en,
--                               [14:12] n_faces, [18:15] stops_per_chip,
--                               [22:19] n_drain_cap, [27:23] stopdis_override,
--                               [31:28] COMMAND
--   CTL1  (0x04) RANGE_COLS    [15:0] max_range_clks, [31:16] cols_per_face
--   CTL2..7 reserved
--
--   STAT0  (0x40) HW_VERSION   [31:0] (constant)
--   STAT1  (0x44) HW_CONFIG    packed generics (constant)
--   STAT2  (0x48) MAX_ROWS     [15:0] (constant)
--   STAT3  (0x4C) CELL_SIZE    [15:0] (constant)
--   STAT4  (0x50) MAX_HSIZE    [15:0] (constant)
--   STAT5  (0x54) STATUS       [0] busy, [1] overrun, [2] err_fatal,
--                               [7:4] chip_err, [11:8] drain_timeout,
--                               [15:12] seq_err
--   STAT6..7 reserved
--
-- CDC structure:
--   CTL: 2 × xpm_cdc_handshake (s_axi_aclk → i_axis_aclk) for CTL0, CTL1
--   STAT: 1 × xpm_cdc_handshake (i_axis_aclk → s_axi_aclk) for STAT5
--   CDC idle flag: 2-FF sync of NOR(src_send_ctl0, src_send_ctl1)
--
-- Command safety:
--   cmd_start and cmd_cfg_write are gated by:
--     1. local CDC idle (this module's CTL0/CTL1 handshakes)
--     2. external chip CSR CDC idle (i_chip_csr_cdc_idle)
--   Both must be '1' for commands to pass.
--
-- Clock domains:
--   s_axi_aclk   : AXI4-Lite domain (PS clock)
--   i_axis_aclk  : TDC processing / AXI-Stream domain (200 MHz)
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

entity tdc_gpx_csr_pipeline is
    generic (
        g_HW_VERSION   : std_logic_vector(31 downto 0) := x"00010000";
        g_OUTPUT_WIDTH : natural := 32    -- 32 or 64 (for HW_CONFIG TDATA width report)
    );
    port (
        -- AXI4-Lite clock / reset
        s_axi_aclk          : in  std_logic;
        s_axi_aresetn       : in  std_logic;

        -- AXI4-Lite Slave (7-bit address, tdc_gpx_axil_csr_pipeline IP)
        s_axi_awvalid       : in  std_logic;
        s_axi_awready       : out std_logic;
        s_axi_awaddr        : in  std_logic_vector(6 downto 0);
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
        s_axi_araddr        : in  std_logic_vector(6 downto 0);
        s_axi_arprot        : in  std_logic_vector(2 downto 0);
        s_axi_rvalid        : out std_logic;
        s_axi_rready        : in  std_logic;
        s_axi_rdata         : out std_logic_vector(31 downto 0);
        s_axi_rresp         : out std_logic_vector(1 downto 0);

        -- TDC processing clock / reset
        i_axis_aclk         : in  std_logic;
        i_axis_aresetn      : in  std_logic;

        -- laser_ctrl_result stream input (i_axis_aclk domain)
        i_lsr_tvalid        : in  std_logic;
        i_lsr_tdata         : in  std_logic_vector(31 downto 0);

        -- chip CSR CDC idle input (from csr_chip)
        i_chip_csr_cdc_idle : in  std_logic;

        -- Configuration output (i_axis_aclk domain)
        o_cfg               : out t_tdc_cfg;

        -- Command pulses (i_axis_aclk domain, 1-clk)
        o_cmd_start         : out std_logic;
        i_cmd_start_accepted : in  std_logic;
        o_cmd_stop          : out std_logic;
        o_cmd_soft_reset    : out std_logic;
        o_cmd_cfg_write     : out std_logic;

        -- Status input (i_axis_aclk domain)
        i_status            : in  t_tdc_status;

        -- Interrupt
        o_irq               : out std_logic
    );
end entity tdc_gpx_csr_pipeline;

architecture rtl of tdc_gpx_csr_pipeline is

    -- =========================================================================
    -- tdc_gpx_axil_csr_pipeline component (Vivado IP: 8 CTL, 8 STAT, 1 IRQ)
    -- =========================================================================
    component tdc_gpx_axil_csr_pipeline is
        port (
            s_axi_csr_aclk      : in  std_logic;
            s_axi_csr_aresetn   : in  std_logic;
            s_axi_csr_awaddr    : in  std_logic_vector(6 downto 0);
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
            s_axi_csr_araddr    : in  std_logic_vector(6 downto 0);
            s_axi_csr_arprot    : in  std_logic_vector(2 downto 0);
            s_axi_csr_arvalid   : in  std_logic;
            s_axi_csr_arready   : out std_logic;
            s_axi_csr_rdata     : out std_logic_vector(31 downto 0);
            s_axi_csr_rresp     : out std_logic_vector(1 downto 0);
            s_axi_csr_rvalid    : out std_logic;
            s_axi_csr_rready    : in  std_logic;
            reg0_init_val       : in  std_logic_vector(31 downto 0);
            reg1_init_val       : in  std_logic_vector(31 downto 0);
            reg2_init_val       : in  std_logic_vector(31 downto 0);
            reg3_init_val       : in  std_logic_vector(31 downto 0);
            reg4_init_val       : in  std_logic_vector(31 downto 0);
            reg5_init_val       : in  std_logic_vector(31 downto 0);
            reg6_init_val       : in  std_logic_vector(31 downto 0);
            reg7_init_val       : in  std_logic_vector(31 downto 0);
            ctl0_out            : out std_logic_vector(31 downto 0);
            ctl1_out            : out std_logic_vector(31 downto 0);
            ctl2_out            : out std_logic_vector(31 downto 0);
            ctl3_out            : out std_logic_vector(31 downto 0);
            ctl4_out            : out std_logic_vector(31 downto 0);
            ctl5_out            : out std_logic_vector(31 downto 0);
            ctl6_out            : out std_logic_vector(31 downto 0);
            ctl7_out            : out std_logic_vector(31 downto 0);
            stat0_in            : in  std_logic_vector(31 downto 0);
            stat1_in            : in  std_logic_vector(31 downto 0);
            stat2_in            : in  std_logic_vector(31 downto 0);
            stat3_in            : in  std_logic_vector(31 downto 0);
            stat4_in            : in  std_logic_vector(31 downto 0);
            stat5_in            : in  std_logic_vector(31 downto 0);
            stat6_in            : in  std_logic_vector(31 downto 0);
            stat7_in            : in  std_logic_vector(31 downto 0);
            intrpt_src_in       : in  std_logic_vector(0 downto 0);
            irq                 : out std_logic
        );
    end component tdc_gpx_axil_csr_pipeline;

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant C_NUM_CTL_CDC  : natural := 2;   -- CTL0 (MAIN_CTRL), CTL1 (RANGE_COLS)
    constant C_NUM_STAT_CDC : natural := 1;   -- STAT5 (STATUS)
    constant C_ZERO32       : std_logic_vector(31 downto 0) := (others => '0');

    -- =========================================================================
    -- Internal types
    -- =========================================================================
    type t_cdc_data_array is array(natural range <>) of std_logic_vector(31 downto 0);

    -- =========================================================================
    -- CTL raw outputs (s_axi_aclk domain)
    -- =========================================================================
    signal s_ctl_src : t_cdc_data_array(0 to 7);  -- 8 CTL from IP

    -- CTL after CDC (i_axis_aclk domain) — CTL0, CTL1
    signal s_ctl_out : t_cdc_data_array(0 to C_NUM_CTL_CDC - 1) := (others => C_ZERO32);

    -- CTL CDC handshake
    signal s_src_send_ctl : std_logic_vector(C_NUM_CTL_CDC - 1 downto 0) := (others => '0');
    signal s_src_rcv_ctl  : std_logic_vector(C_NUM_CTL_CDC - 1 downto 0);
    signal s_dest_req_ctl : std_logic_vector(C_NUM_CTL_CDC - 1 downto 0);
    signal s_ctl_d1       : t_cdc_data_array(0 to C_NUM_CTL_CDC - 1)
                            := (others => (others => '1'));

    -- STAT source (i_axis_aclk domain, 1 live register)
    signal s_stat_src : std_logic_vector(31 downto 0);

    -- STAT after CDC (s_axi_aclk domain)
    signal s_stat_out : std_logic_vector(31 downto 0) := C_ZERO32;

    -- STAT CDC handshake
    signal s_src_send_stat : std_logic := '0';
    signal s_src_rcv_stat  : std_logic;
    signal s_dest_req_stat : std_logic;
    signal s_stat_d1       : std_logic_vector(31 downto 0) := (others => '1');

    -- STAT6 (Round 5 follow-up): pipeline-wide + per-chip/per-slope stickies.
    -- Same CDC pattern as STAT5; separate handshake to keep payload atomic.
    signal s_stat6_src     : std_logic_vector(31 downto 0);
    signal s_stat6_out     : std_logic_vector(31 downto 0) := C_ZERO32;
    signal s_src_send_stat6 : std_logic := '0';
    signal s_src_rcv_stat6  : std_logic;
    signal s_dest_req_stat6 : std_logic;
    signal s_stat6_d1      : std_logic_vector(31 downto 0) := (others => '1');

    -- STAT7 (Round 11 Category C): reg timeout mask + stop_id error mask +
    -- run timeout cause + per-slope face_start collapsed count.
    signal s_stat7_src     : std_logic_vector(31 downto 0);
    signal s_stat7_out     : std_logic_vector(31 downto 0) := C_ZERO32;
    signal s_src_send_stat7 : std_logic := '0';
    signal s_src_rcv_stat7  : std_logic;
    signal s_dest_req_stat7 : std_logic;
    signal s_stat7_d1      : std_logic_vector(31 downto 0) := (others => '1');

    -- HW_CONFIG constant (compile-time)
    signal s_hw_config : std_logic_vector(31 downto 0);

    -- Command edge detect (i_axis_aclk domain)
    signal s_cmd_prev_r  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cmd_pulse_r : std_logic_vector(3 downto 0) := (others => '0');

    -- Pending latches (i_axis_aclk domain)
    signal s_cfg_write_pending_r : std_logic := '0';
    signal s_start_pending_r     : std_logic := '0';

    -- CDC-idle flag: '1' when local + chip CSR handshakes are quiescent.
    signal s_cdc_local_idle_src : std_logic := '1';
    signal s_cdc_all_idle_ff   : std_logic_vector(1 downto 0) := "11";

    -- laser_ctrl cols_per_face latch (i_axis_aclk domain)
    signal s_lsr_cols_r  : unsigned(15 downto 0) := (others => '0');
    signal s_lsr_valid_r : std_logic := '0';

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
        <= std_logic_vector(to_unsigned(g_OUTPUT_WIDTH, 8));
    s_hw_config(c_HWCFG_CELL_FMT_HI downto c_HWCFG_CELL_FMT_LO)
        <= std_logic_vector(to_unsigned(c_CELL_FORMAT, 3));
    s_hw_config(31 downto c_HWCFG_CELL_FMT_HI + 1) <= (others => '0');

    -- =========================================================================
    -- [2] tdc_gpx_axil_csr_pipeline instantiation (8 CTL, 8 STAT)
    -- =========================================================================
    u_srm : tdc_gpx_axil_csr_pipeline
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
            -- Init values
            reg0_init_val  => c_INIT_MAIN_CTRL,
            reg1_init_val  => c_INIT_RANGE_COLS,
            reg2_init_val  => C_ZERO32,
            reg3_init_val  => C_ZERO32,
            reg4_init_val  => C_ZERO32,
            reg5_init_val  => C_ZERO32,
            reg6_init_val  => C_ZERO32,
            reg7_init_val  => C_ZERO32,
            -- CTL outputs
            ctl0_out => s_ctl_src(0),  -- MAIN_CTRL
            ctl1_out => s_ctl_src(1),  -- RANGE_COLS
            ctl2_out => s_ctl_src(2),  ctl3_out => s_ctl_src(3),
            ctl4_out => s_ctl_src(4),  ctl5_out => s_ctl_src(5),
            ctl6_out => s_ctl_src(6),  ctl7_out => s_ctl_src(7),
            -- STAT inputs: constants + CDC'd status
            stat0_in => g_HW_VERSION,
            stat1_in => s_hw_config,
            stat2_in => std_logic_vector(to_unsigned(c_MAX_ROWS_PER_FACE, 32)),
            stat3_in => std_logic_vector(to_unsigned(c_CELL_SIZE_BYTES, 32)),
            stat4_in => std_logic_vector(to_unsigned(
                (c_MAX_ROWS_PER_FACE * fn_beats_per_cell(g_OUTPUT_WIDTH)
                 + fn_hdr_prefix_beats(g_OUTPUT_WIDTH)) * (g_OUTPUT_WIDTH / 8), 32)),
            stat5_in => s_stat_out,     -- STATUS (CDC'd from i_axis_aclk)
            stat6_in => s_stat6_out,    -- STATUS_EXT (Round 5 follow-up, CDC'd)
            stat7_in => s_stat7_out,    -- STATUS_EXT2 (Round 11 Category C, CDC'd)
            intrpt_src_in => "0",
            irq           => o_irq
        );

    -- =========================================================================
    -- [3] STAT source packing (i_axis_aclk domain) — STAT5 only
    -- =========================================================================
    s_stat_src(c_STAT_BUSY)          <= i_status.busy;
    s_stat_src(c_STAT_OVERRUN)       <= i_status.pipeline_overrun;
    s_stat_src(c_STAT_ERR_FATAL)     <= i_status.err_fatal;
    s_stat_src(3) <= '0';
    s_stat_src(c_STAT_CHIP_ERR_HI downto c_STAT_CHIP_ERR_LO)
        <= i_status.chip_error_mask;
    s_stat_src(c_STAT_DRAIN_TO_HI downto c_STAT_DRAIN_TO_LO)
        <= i_status.drain_timeout_mask;
    s_stat_src(c_STAT_SEQ_ERR_HI downto c_STAT_SEQ_ERR_LO)
        <= i_status.sequence_error_mask;
    s_stat_src(31 downto c_STAT_SEQ_ERR_HI + 1) <= (others => '0');

    -- =========================================================================
    -- [3b] STAT6 source packing (Round 5 follow-up, i_axis_aclk domain)
    --   bit  0    err_read_timeout (pipeline)
    --   bit  1    reg_rejected (pipeline)
    --   bit  2    reg_zero_mask (pipeline)
    --   bit  3    rise_shot_flush_drop
    --   bit  4    fall_shot_flush_drop
    --   bits[15:8]  rise_shot_overrun_count
    --   bits[23:16] fall_shot_overrun_count
    --   bits[27:24] err_reg_overflow_mask (per-chip)
    --   bits[31:28] run_drain_complete_mask (per-chip, sticky from pulses)
    -- =========================================================================
    s_stat6_src(0)            <= i_status.err_read_timeout;
    s_stat6_src(1)            <= i_status.reg_rejected;
    s_stat6_src(2)            <= i_status.reg_zero_mask;
    s_stat6_src(3)            <= i_status.rise_shot_flush_drop;
    s_stat6_src(4)            <= i_status.fall_shot_flush_drop;
    s_stat6_src(7 downto 5)   <= (others => '0');
    s_stat6_src(15 downto 8)  <= std_logic_vector(i_status.rise_shot_overrun_count);
    s_stat6_src(23 downto 16) <= std_logic_vector(i_status.fall_shot_overrun_count);
    s_stat6_src(27 downto 24) <= i_status.err_reg_overflow_mask;
    s_stat6_src(31 downto 28) <= i_status.run_drain_complete_mask;

    -- =========================================================================
    -- [3c] STAT7 source packing (Round 11 Category C)
    --   [3:0]   reg_timeout_mask
    --   [7:4]   stop_id_error_mask
    --   [10:8]  run_timeout_cause_last
    --   [15:11] reserved
    --   [23:16] rise_face_start_collapsed_count
    --   [31:24] fall_face_start_collapsed_count
    -- =========================================================================
    s_stat7_src(3 downto 0)   <= i_status.reg_timeout_mask;
    s_stat7_src(7 downto 4)   <= i_status.stop_id_error_mask;
    s_stat7_src(10 downto 8)  <= i_status.run_timeout_cause_last;
    s_stat7_src(15 downto 11) <= (others => '0');
    s_stat7_src(23 downto 16) <= std_logic_vector(i_status.rise_face_start_collapsed_count);
    s_stat7_src(31 downto 24) <= std_logic_vector(i_status.fall_face_start_collapsed_count);

    -- =========================================================================
    -- [4] STAT CDC: i_axis_aclk → s_axi_aclk (1 live register)
    -- =========================================================================
    u_cdc_stat5 : xpm_cdc_handshake
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
            src_in    => s_stat_src,
            src_send  => s_src_send_stat,
            src_rcv   => s_src_rcv_stat,
            dest_clk  => s_axi_aclk,
            dest_req  => s_dest_req_stat,
            dest_ack  => s_dest_req_stat,
            dest_out  => s_stat_out
        );

    p_send_stat : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_src_send_stat <= '0';
                s_stat_d1       <= (others => '1');
            else
                if s_src_send_stat = '0' and s_stat_src /= s_stat_d1 then
                    s_src_send_stat <= '1';
                    s_stat_d1       <= s_stat_src;
                elsif s_src_rcv_stat = '1' then
                    s_src_send_stat <= '0';
                end if;
            end if;
        end if;
    end process p_send_stat;

    -- STAT6 CDC (Round 5 follow-up): i_axis_aclk -> s_axi_aclk
    u_cdc_stat6 : xpm_cdc_handshake
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
            src_in    => s_stat6_src,
            src_send  => s_src_send_stat6,
            src_rcv   => s_src_rcv_stat6,
            dest_clk  => s_axi_aclk,
            dest_req  => s_dest_req_stat6,
            dest_ack  => s_dest_req_stat6,
            dest_out  => s_stat6_out
        );

    p_send_stat6 : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_src_send_stat6 <= '0';
                s_stat6_d1       <= (others => '1');
            else
                if s_src_send_stat6 = '0' and s_stat6_src /= s_stat6_d1 then
                    s_src_send_stat6 <= '1';
                    s_stat6_d1       <= s_stat6_src;
                elsif s_src_rcv_stat6 = '1' then
                    s_src_send_stat6 <= '0';
                end if;
            end if;
        end if;
    end process p_send_stat6;

    -- STAT7 CDC (Round 11 Category C): i_axis_aclk -> s_axi_aclk
    u_cdc_stat7 : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK => 1, DEST_SYNC_FF => 4, INIT_SYNC_FF => 0,
            SIM_ASSERT_CHK => 0, SRC_SYNC_FF => 4, WIDTH => 32
        )
        port map (
            src_clk  => i_axis_aclk,
            src_in   => s_stat7_src,
            src_send => s_src_send_stat7,
            src_rcv  => s_src_rcv_stat7,
            dest_clk => s_axi_aclk,
            dest_req => s_dest_req_stat7,
            dest_ack => s_dest_req_stat7,
            dest_out => s_stat7_out
        );

    p_send_stat7 : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_src_send_stat7 <= '0';
                s_stat7_d1       <= (others => '1');
            else
                if s_src_send_stat7 = '0' and s_stat7_src /= s_stat7_d1 then
                    s_src_send_stat7 <= '1';
                    s_stat7_d1       <= s_stat7_src;
                elsif s_src_rcv_stat7 = '1' then
                    s_src_send_stat7 <= '0';
                end if;
            end if;
        end if;
    end process p_send_stat7;

    -- =========================================================================
    -- [5] CTL CDC: s_axi_aclk → i_axis_aclk (CTL0, CTL1)
    -- =========================================================================
    gen_ctl_cdc : for i in 0 to C_NUM_CTL_CDC - 1 generate
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
    -- [6] CDC-idle flag: local CTL + external chip CSR both quiescent
    -- =========================================================================
    p_cdc_idle_src : process(s_axi_aclk)
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                s_cdc_local_idle_src <= '1';
            elsif s_src_send_ctl = (s_src_send_ctl'range => '0') then
                s_cdc_local_idle_src <= '1';
            else
                s_cdc_local_idle_src <= '0';
            end if;
        end if;
    end process p_cdc_idle_src;

    -- Combine local idle + chip CSR idle, sync to i_axis_aclk
    p_cdc_idle_sync : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_cdc_all_idle_ff <= "11";
            else
                s_cdc_all_idle_ff(0) <= s_cdc_local_idle_src and i_chip_csr_cdc_idle;
                s_cdc_all_idle_ff(1) <= s_cdc_all_idle_ff(0);
            end if;
        end if;
    end process p_cdc_idle_sync;

    -- =========================================================================
    -- [7] Command edge detect (i_axis_aclk domain)
    --   CTL0[31:28] = {cfg_write, soft_reset, stop, start}
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

    -- start pending latch: hold until BOTH CDC idle AND face_seq accepts.
    -- Two-phase: (1) wait for CDC idle, (2) wait for accepted feedback.
    -- Prevents start loss when face_seq is not yet ready.
    p_start_pending : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0'
               or s_cmd_pulse_r(1) = '1'      -- stop clears
               or s_cmd_pulse_r(2) = '1' then  -- soft_reset clears
                s_start_pending_r <= '0';
            elsif s_cmd_pulse_r(0) = '1' then
                -- New start request: always latch (CDC may or may not be idle)
                s_start_pending_r <= '1';
            elsif s_start_pending_r = '1' and i_cmd_start_accepted = '1' then
                -- face_seq accepted the start → clear pending
                s_start_pending_r <= '0';
            end if;
        end if;
    end process p_start_pending;

    -- Output start pulse: fires every cycle while pending AND CDC idle.
    -- face_seq sees repeated pulses until it accepts (level-ish behavior).
    o_cmd_start      <= s_start_pending_r and s_cdc_all_idle_ff(1);
    o_cmd_stop       <= s_cmd_pulse_r(1);
    o_cmd_soft_reset <= s_cmd_pulse_r(2);

    -- cfg_write pending latch
    p_cfg_write_pending : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0'
               or s_cmd_pulse_r(1) = '1'
               or s_cmd_pulse_r(2) = '1' then
                s_cfg_write_pending_r <= '0';
            elsif s_cmd_pulse_r(3) = '1' and s_cdc_all_idle_ff(1) = '0' then
                s_cfg_write_pending_r <= '1';
            elsif s_cfg_write_pending_r = '1' and s_cdc_all_idle_ff(1) = '1' then
                s_cfg_write_pending_r <= '0';
            end if;
        end if;
    end process p_cfg_write_pending;

    o_cmd_cfg_write <= (s_cmd_pulse_r(3) and s_cdc_all_idle_ff(1))
                    or (s_cfg_write_pending_r and s_cdc_all_idle_ff(1));

    -- =========================================================================
    -- [8] laser_ctrl cols_per_face latch (i_axis_aclk domain)
    -- =========================================================================
    p_lsr_latch : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_lsr_cols_r  <= (others => '0');
                s_lsr_valid_r <= '0';
            else
                if i_cmd_start_accepted = '1' then
                    s_lsr_valid_r <= '0';
                end if;
                if i_lsr_tvalid = '1' then
                    s_lsr_cols_r  <= unsigned(i_lsr_tdata(15 downto 0));
                    s_lsr_valid_r <= '1';
                end if;
            end if;
        end if;
    end process p_lsr_latch;

    -- =========================================================================
    -- [9] CSR output: t_tdc_cfg field extraction (i_axis_aclk domain)
    -- =========================================================================
    -- CTL0: MAIN_CTRL
    o_cfg.active_chip_mask <= s_ctl_out(0)(c_MC_ACTIVE_MASK_HI downto c_MC_ACTIVE_MASK_LO)
                              when s_ctl_out(0)(c_MC_ACTIVE_MASK_HI downto c_MC_ACTIVE_MASK_LO) /= "0000"
                              else "0001";
    o_cfg.packet_scope     <= s_ctl_out(0)(c_MC_PACKET_SCOPE);
    o_cfg.hit_store_mode   <= unsigned(s_ctl_out(0)(c_MC_HIT_STORE_HI downto c_MC_HIT_STORE_LO));
    o_cfg.dist_scale       <= unsigned(s_ctl_out(0)(c_MC_DIST_SCALE_HI downto c_MC_DIST_SCALE_LO));
    o_cfg.drain_mode       <= s_ctl_out(0)(c_MC_DRAIN_MODE);
    o_cfg.pipeline_en      <= s_ctl_out(0)(c_MC_PIPELINE_EN);

    o_cfg.n_faces          <= unsigned(s_ctl_out(0)(c_MC_N_FACES_HI downto c_MC_N_FACES_LO))
                              when unsigned(s_ctl_out(0)(c_MC_N_FACES_HI downto c_MC_N_FACES_LO)) >= 1
                              else to_unsigned(1, 3);

    o_cfg.stops_per_chip   <= unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO))
                              when unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) >= 2
                                   and unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) <= 8
                              else to_unsigned(8, 4)
                              when unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) > 8
                              else to_unsigned(2, 4);

    o_cfg.n_drain_cap      <= unsigned(s_ctl_out(0)(c_MC_N_DRAIN_CAP_HI downto c_MC_N_DRAIN_CAP_LO));
    o_cfg.stopdis_override <= s_ctl_out(0)(c_MC_STOPDIS_HI downto c_MC_STOPDIS_LO);

    -- CTL1: RANGE_COLS
    o_cfg.max_range_clks   <= unsigned(s_ctl_out(1)(c_RC_MAX_RANGE_HI downto c_RC_MAX_RANGE_LO));

    o_cfg.cols_per_face    <= s_lsr_cols_r
                              when s_lsr_valid_r = '1' and s_lsr_cols_r >= 1
                              else unsigned(s_ctl_out(1)(c_RC_COLS_HI downto c_RC_COLS_LO))
                              when unsigned(s_ctl_out(1)(c_RC_COLS_HI downto c_RC_COLS_LO)) >= 1
                              else to_unsigned(1, 16);

    -- Fields owned by csr_chip — drive defaults here, overridden at top level
    -- by connecting csr_chip outputs directly to the downstream modules.
    o_cfg.bus_clk_div      <= to_unsigned(2, 6);     -- default, overridden
    o_cfg.bus_ticks        <= to_unsigned(5, 3);     -- default, overridden
    o_cfg.start_off1       <= (others => '0');       -- default, overridden
    o_cfg.cfg_reg7         <= (others => '0');       -- default, overridden
    o_cfg.max_scan_clks    <= (others => '0');       -- default, overridden
    o_cfg.max_hits_cfg     <= to_unsigned(7, 3);     -- default, overridden

end architecture rtl;
