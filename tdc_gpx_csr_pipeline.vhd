-- =============================================================================
-- tdc_gpx_csr_pipeline.vhd
-- TDC-GPX Controller - Pipeline / Shot Control CSR
-- =============================================================================
--
-- Purpose:
--   AXI4-Lite register interface for pipeline and shot control.
--   Uses the canonical source-level my_axil_csr implementation
--   (8 CTL + 8 STAT, 7-bit address).
--   CDC transfers control registers to i_axis_aclk domain.
--
-- Register map:
--   CTL0  (0x00) MAIN_CTRL     packed: [3:0] active_chip_mask, [4] packet_scope,
--                               [6:5] hit_store_mode, [9:7] dist_scale,
--                               [10] drain_mode, [11] pipeline_en,
--                               [14:12] reserved, [18:15] stops_per_chip,
--                               [22:19] n_drain_cap, [27:23] stopdis_override,
--                               [31:28] COMMAND
--   CTL1  (0x04) RANGE_COLS    [15:0] max_range_5ns_ticks, [31:16] cols_per_face
--                               max_range_5ns_ticks is always encoded in
--                               200 MHz reference units (1 tick = 5 ns).
--   CTL2  (0x08) AUX_CMD       [0] force_reinit (rising edge),
--                               [1] err_soft_clear (rising edge),
--                               [31:2] reserved
--   CTL3..7 reserved
--
--   STAT0  (0x40) HW_VERSION   [31:0] (constant)
--   STAT1  (0x44) HW_CONFIG    packed build profile, [31:29] motor n_faces
--   STAT2  (0x48) MAX_ROWS     [15:0] (constant)
    --   STAT3  (0x4C) CELL_SIZE    packed canonical maximum bytes/cell
    --   STAT4  (0x50) MAX_HSIZE    packed full-mask maximum bytes/line
--   STAT5  (0x54) STATUS       [0] busy, [1] overrun, [2] err_fatal,
--                               [7:4] chip_err, [11:8] drain_timeout,
--                               [15:12] seq_err
    --   STAT6  (0x58) STATUS_EXT   extended sticky/counter status
    --   STAT7  (0x5C) STATUS_EXT2  extended diagnostic status
--
-- CDC structure:
--   CTL: 3 × xpm_cdc_handshake (s_axi_aclk → i_axis_aclk) for CTL0..CTL2
--   STAT: handshake transfers from i_axis_aclk to s_axi_aclk
--   CDC idle flag: 2-FF sync of NOR(src_send_ctl0..src_send_ctl2)
--
-- Command safety:
--   cmd_start and cmd_cfg_write are gated by:
--     1. local CDC idle (this module's CTL0..CTL2 handshakes)
--     2. external chip CSR CDC idle (i_chip_csr_cdc_idle)
--   Both must be '1' for commands to pass.
--
-- Clock domains:
--   s_axi_aclk   : AXI4-Lite domain (PS clock)
--   i_axis_aclk  : AXI-Stream domain selected by top-level g_AXIS_CLK_MHZ
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
        g_HW_VERSION          : std_logic_vector(31 downto 0) := c_DEFAULT_HW_VERSION;
        g_OUTPUT_WIDTH        : natural := c_DEFAULT_OUTPUT_WIDTH;
        g_PRESENT_CHIP_MASK   : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := c_ALL_CHIPS_MASK;
        g_FALL_CHIP_MASK      : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := c_DEFAULT_FALL_CHIP_MASK;
        g_MAX_STOPS_PER_CHIP  : positive range 2 to c_MAX_STOPS_PER_CHIP := c_MAX_STOPS_PER_CHIP;
        g_MAX_HITS_PER_STOP   : positive range 1 to c_MAX_HITS_PER_STOP := c_MAX_HITS_PER_STOP
    );
    port (
        -- AXI4-Lite clock / reset
        s_axi_aclk          : in  std_logic;
        s_axi_aresetn       : in  std_logic;

        -- AXI4-Lite Slave (7-bit address, canonical my_axil_csr)
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

        -- Static mechanical geometry sideband. This must be driven from the
        -- motor decoder's g_N_FACES-derived output and remain constant while
        -- the bitstream is active, so no runtime CDC handshake is required.
        i_n_faces           : in  std_logic_vector(2 downto 0);

        -- chip CSR CDC idle input (from csr_chip)
        i_chip_csr_cdc_idle : in  std_logic;

        -- Configuration output (i_axis_aclk domain)
        o_cfg               : out t_tdc_cfg;

        -- Command pulses (i_axis_aclk domain, 1-clk)
        o_cmd_start         : out std_logic;
        i_cmd_start_accepted : in  std_logic;
        o_cmd_stop          : out std_logic;
        o_cmd_soft_reset    : out std_logic;
        -- Round 12 A1: force-reinit escape command (SW pulse).
        o_cmd_force_reinit  : out std_logic;
        -- Round 13 follow-up: SW-initiated error-history soft clear pulse.
        -- CTL2[1] rising edge emits a single-cycle pulse in i_axis_aclk domain.
        -- Consumed by err_handler, status_agg, chip_reg, config_ctrl sticky
        -- clear logic (SOFT-CLEAR category).
        o_err_soft_clear    : out std_logic;
        o_cmd_cfg_write     : out std_logic;

        -- Status input (i_axis_aclk domain)
        i_status            : in  t_tdc_status;

        -- Interrupt
        o_irq               : out std_logic
    );
end entity tdc_gpx_csr_pipeline;

architecture rtl of tdc_gpx_csr_pipeline is

    function fn_sanitize_n_faces(
        value : std_logic_vector(2 downto 0)
    ) return unsigned is
    begin
        case value is
            when "001" | "010" | "011" | "100" | "101" =>
                return unsigned(value);
            when others =>
                return to_unsigned(1, 3);
        end case;
    end function;

    -- =========================================================================
    -- Constants
    -- =========================================================================
    -- CTL CDC slots (s_axi_aclk → i_axis_aclk). Round 12 A1: added CTL2
    -- for auxiliary recovery commands (currently just force_reinit).
    constant C_NUM_CTL_CDC  : natural := 3;   -- CTL0 (MAIN_CTRL), CTL1 (RANGE_COLS), CTL2 (AUX_CMD)
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

    -- CTL after CDC (i_axis_aclk domain) — CTL0..CTL2
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

    -- Published-to-native CSR address translation (see [2] below).
    signal s_awaddr_ip     : std_logic_vector(6 downto 0);
    signal s_araddr_ip     : std_logic_vector(6 downto 0);
    constant c_UNMAPPED_IP_ADDR : std_logic_vector(6 downto 0) := "1111100"; -- 0x7C

    -- HW_CONFIG constant (compile-time)
    signal s_hw_config : std_logic_vector(31 downto 0);
    signal s_n_faces_effective : unsigned(2 downto 0);
    constant c_BUILD_CHIP_COUNT : natural := fn_count_ones(g_PRESENT_CHIP_MASK);
    constant c_BUILD_MAX_ROWS   : natural := c_BUILD_CHIP_COUNT * g_MAX_STOPS_PER_CHIP;
    constant c_BUILD_CELL_BYTES : natural := fn_canonical_cell_bytes(g_MAX_HITS_PER_STOP);
    constant c_BUILD_VDMA_BYTES : natural :=
        fn_vdma_line_bytes(c_BUILD_MAX_ROWS, g_MAX_HITS_PER_STOP);
    constant c_ZERO_CHIP_MASK : std_logic_vector(c_MAX_CHIPS - 1 downto 0) := (others => '0');
    signal s_requested_chip_mask : std_logic_vector(c_MAX_CHIPS - 1 downto 0);
    signal s_effective_chip_mask : std_logic_vector(c_MAX_CHIPS - 1 downto 0);

    -- Command edge detect (i_axis_aclk domain)
    signal s_cmd_prev_r  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cmd_pulse_r : std_logic_vector(3 downto 0) := (others => '0');
    -- Round 12 A1: force-reinit command via CTL2[0] edge detect
    signal s_force_reinit_prev_r  : std_logic := '0';
    signal s_force_reinit_pulse_r : std_logic := '0';
    -- Round 13 follow-up: err soft-clear command via CTL2[1] edge detect
    signal s_err_soft_clear_prev_r  : std_logic := '0';
    signal s_err_soft_clear_pulse_r : std_logic := '0';

    -- Pending latches (i_axis_aclk domain)
    signal s_cfg_write_pending_r : std_logic := '0';
    signal s_start_pending_r     : std_logic := '0';

    -- CDC-idle flag: '1' when local + chip CSR handshakes are quiescent.
    signal s_cdc_all_idle_src_r : std_logic := '1';
    signal s_cdc_all_idle_ff    : std_logic_vector(1 downto 0) := "11";

    attribute ASYNC_REG : string;
    attribute ASYNC_REG of s_cdc_all_idle_ff : signal is "TRUE";

begin

    s_n_faces_effective <= fn_sanitize_n_faces(i_n_faces);

    -- A generic-derived output reaches this input after the initial VHDL
    -- delta cycle. Validate only after reset release so a legal static
    -- sideband is not rejected at time zero while its driver is still 'U'.
    p_assert_n_faces : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '1' then
                assert i_n_faces = "001" or i_n_faces = "010" or
                       i_n_faces = "011" or i_n_faces = "100" or
                       i_n_faces = "101"
                    report "tdc_gpx_csr_pipeline: i_n_faces must be a static value in 1..5 from motor_decoder"
                    severity failure;
            end if;
        end if;
    end process p_assert_n_faces;

    assert fn_output_width_supported(g_OUTPUT_WIDTH)
        report "tdc_gpx_csr_pipeline: g_OUTPUT_WIDTH must be 32, 64, or 128 for full-keep Phase A"
        severity failure;

    assert c_BUILD_CHIP_COUNT > 0
        report "tdc_gpx_csr_pipeline: g_PRESENT_CHIP_MASK must contain at least one implemented chip"
        severity failure;

    -- =========================================================================
    -- [1] HW_CONFIG constant assembly
    -- =========================================================================
    s_hw_config(c_HWCFG_N_CHIPS_HI downto c_HWCFG_N_CHIPS_LO)
        <= std_logic_vector(to_unsigned(c_BUILD_CHIP_COUNT, 4));
    s_hw_config(c_HWCFG_MAX_STOPS_HI downto c_HWCFG_MAX_STOPS_LO)
        <= std_logic_vector(to_unsigned(g_MAX_STOPS_PER_CHIP, 4));
    s_hw_config(c_HWCFG_MAX_HITS_HI downto c_HWCFG_MAX_HITS_LO)
        <= std_logic_vector(to_unsigned(g_MAX_HITS_PER_STOP, 4));
    s_hw_config(c_HWCFG_HIT_WIDTH_HI downto c_HWCFG_HIT_WIDTH_LO)
        <= std_logic_vector(to_unsigned(c_HIT_SLOT_DATA_WIDTH, 5));
    s_hw_config(c_HWCFG_TDATA_HI downto c_HWCFG_TDATA_LO)
        <= std_logic_vector(to_unsigned(g_OUTPUT_WIDTH, 8));
    s_hw_config(c_HWCFG_CELL_FMT_HI downto c_HWCFG_CELL_FMT_LO)
        <= std_logic_vector(to_unsigned(c_CELL_FORMAT, 3));
    s_hw_config(c_HWCFG_HAS_FALLING) <= '1'
        when fn_count_ones(g_FALL_CHIP_MASK and g_PRESENT_CHIP_MASK) > 0
        else '0';
    s_hw_config(c_HWCFG_N_FACES_HI downto c_HWCFG_N_FACES_LO)
        <= std_logic_vector(s_n_faces_effective);

    -- =========================================================================
    -- [2] Canonical source-level my_axil_csr (8 CTL, 8 STAT)
    --
    -- LATENT-BUG FIX (2026-07-17): STAT read address remap.
    -- The IP was generated with num_ctl_regs=8 / num_stat_regs=8, which
    -- places STAT0..7 at WORD addresses 8..15 (byte 0x20..0x3C). The
    -- published register map — and every TB/SW consumer — reads STAT at
    -- 0x40..0x5C, which decodes OUTSIDE the IP's register range, so every
    -- pipeline STAT read has returned zero since the block was created.
    -- Combined with the t_tdc_status 'U'-source defect (status_agg record
    -- out port) the exact-zero STAT compares in the TBs passed vacuously.
    -- The published 0x40-based contract is kept while the native alias and
    -- generated IRQ window are hidden by the scoped translation below.
    -- =========================================================================
    -- Only the published status window is remapped. Hide the generated IP's
    -- native 0x20 status alias, leave 0x60..0x7F unmapped, and permit writes
    -- only to CTL0..7 so status writes cannot touch native IRQ registers.
    s_araddr_ip <= "01" & s_axi_araddr(4 downto 0)
                   when s_axi_araddr(6 downto 5) = "10" else
                   c_UNMAPPED_IP_ADDR
                   when s_axi_araddr(6 downto 5) = "01" else
                   s_axi_araddr;

    s_awaddr_ip <= s_axi_awaddr
                   when s_axi_awaddr(6 downto 5) = "00" else
                   c_UNMAPPED_IP_ADDR;

    u_srm : entity work.my_axil_csr
        generic map (
            num_data_bits       => 32,
            num_ctl_regs        => 8,
            num_stat_regs       => 8,
            has_interrupt_regs  => true,
            num_intr_regs       => 4,
            num_interrupt_src   => 1
        )
        port map (
            s_axi_csr_aclk    => s_axi_aclk,
            s_axi_csr_aresetn => s_axi_aresetn,
            s_axi_csr_awaddr  => s_awaddr_ip,
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
            s_axi_csr_araddr  => s_araddr_ip,
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
            stat2_in => std_logic_vector(to_unsigned(c_BUILD_MAX_ROWS, 32)),
            stat3_in => std_logic_vector(to_unsigned(c_BUILD_CELL_BYTES, 32)),
            stat4_in => std_logic_vector(to_unsigned(c_BUILD_VDMA_BYTES, 32)),
            stat5_in => s_stat_out,     -- STATUS (CDC'd from i_axis_aclk)
            stat6_in => s_stat6_out,    -- STATUS_EXT (Round 5 follow-up, CDC'd)
            stat7_in => s_stat7_out,    -- STATUS_EXT2 (Round 11 Category C, CDC'd)
            -- C06 contract: pipeline IRQ is reserved/tied-off. SW shall read
            -- STAT5/6/7 for pipeline done/fault visibility instead of waiting
            -- for a dedicated pipeline interrupt source.
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
    -- [3b] STAT6 source packing (Round 5 follow-up + Round 11 items 3/11/15)
    --   bit  0      err_read_timeout (pipeline)
    --   bit  1      reg_rejected (pipeline)
    --   bit  2      reg_zero_mask (pipeline)
    --   bit  3      rise_shot_flush_drop (OR-reduction, any chip)
    --   bit  4      fall_shot_flush_drop (OR-reduction, any chip)
    --   bit  5      rise_hdr_drain_timeout        (Round 11 item 3)
    --   bit  6      fall_hdr_drain_timeout        (Round 11 item 3)
    --   bit  7      err_frame_wait_escape         (Round 11 item 11)
    --   bits[11:8]  rise_shot_overrun_count[3:0]  (reduced 8→4; wrap)
    --   bits[15:12] shot_flush_drop_mask          (Round 11 item 15, per-chip)
    --   bits[19:16] fall_shot_overrun_count[3:0]  (reduced 8→4; wrap)
    --   bits[23:20] cmd_collision_mask            (Round 11 item 18 C, per-chip)
    --   bits[27:24] err_reg_overflow_mask (per-chip)
    --   bits[31:28] run_drain_complete_mask (per-chip)
    --
    -- shot_overrun_count reduced 8→4 bits to free space for
    -- shot_flush_drop_mask. Rationale matches face_start_collapsed_count
    -- reduction: under correct operation these counters stay zero, so a
    -- 4-bit wrap suffices as a "nonzero = bug" indicator. Full 8-bit
    -- magnitude was never functionally required by SW.
    -- =========================================================================
    s_stat6_src(c_STAT6_ERR_READ_TIMEOUT) <= i_status.err_read_timeout;
    s_stat6_src(c_STAT6_REG_REJECTED) <= i_status.reg_rejected;
    s_stat6_src(c_STAT6_REG_ZERO_MASK) <= i_status.reg_zero_mask;
    s_stat6_src(c_STAT6_SHOT_FLUSH_DROP_RISE) <= i_status.rise_shot_flush_drop;
    s_stat6_src(c_STAT6_SHOT_FLUSH_DROP_FALL) <= i_status.fall_shot_flush_drop;
    s_stat6_src(c_STAT6_HDR_DRAIN_TO_RISE) <= i_status.rise_hdr_drain_timeout;
    s_stat6_src(c_STAT6_HDR_DRAIN_TO_FALL) <= i_status.fall_hdr_drain_timeout;
    s_stat6_src(c_STAT6_FRAME_WAIT_ESCAPE) <= i_status.err_frame_wait_escape;
    s_stat6_src(c_STAT6_OVRUN_CNT_RISE_HI downto c_STAT6_OVRUN_CNT_RISE_LO)
        <= std_logic_vector(i_status.rise_shot_overrun_count(3 downto 0));
    s_stat6_src(c_STAT6_SHOT_FLUSH_MASK_HI downto c_STAT6_SHOT_FLUSH_MASK_LO)
        <= i_status.shot_flush_drop_mask;
    s_stat6_src(c_STAT6_OVRUN_CNT_FALL_HI downto c_STAT6_OVRUN_CNT_FALL_LO)
        <= std_logic_vector(i_status.fall_shot_overrun_count(3 downto 0));
    s_stat6_src(c_STAT6_CMD_COLL_HI downto c_STAT6_CMD_COLL_LO)
        <= i_status.cmd_collision_mask;
    s_stat6_src(c_STAT6_ERR_REG_OVR_HI downto c_STAT6_ERR_REG_OVR_LO)
        <= i_status.err_reg_overflow_mask;
    s_stat6_src(c_STAT6_RUN_DRAIN_COMP_HI downto c_STAT6_RUN_DRAIN_COMP_LO)
        <= i_status.run_drain_complete_mask;

    -- =========================================================================
    -- [3c] STAT7 source packing (Round 11 Category C + items 4/14)
    --   [3:0]   reg_timeout_mask
    --   [7:4]   stop_id_error_mask
    --   [10:8]  run_timeout_cause_last
    --   [14:11] quarantine_escape_mask       (Round 11 item 4)
    --   [15]    masked_slope_drop_any        (CHAIN P1, 2026-07-16)
    --   [19:16] rise_face_start_collapsed_count (reduced from 8-bit; wrap-4)
    --   [23:20] reserved, reads as zero
    --   [27:24] fall_face_start_collapsed_count (reduced from 8-bit; wrap-4)
    --   [31:28] init_cfg_coalesced_mask      (Round 11 item 14)
    --
    -- face_start_collapsed_count fields reduced from 8 to 4 bits.
    -- Rationale: with Round 11 item 1 (1-cycle pulse guarantee), these
    -- counters should stay at 0 under correct behavior; a 4-bit wrap is
    -- sufficient to flag "nonzero = bug" without consuming 16 bits that
    -- the new Round 11 masks need. The low nibble of the source counter
    -- is surfaced (so a wrap at 16 is still observable as the bits
    -- changing; full saturation would appear as stuck zeros).
    -- =========================================================================
    s_stat7_src(c_STAT7_REG_TO_HI downto c_STAT7_REG_TO_LO)
        <= i_status.reg_timeout_mask;
    s_stat7_src(c_STAT7_STOP_ID_ERR_HI downto c_STAT7_STOP_ID_ERR_LO)
        <= i_status.stop_id_error_mask;
    s_stat7_src(c_STAT7_RUN_CAUSE_HI downto c_STAT7_RUN_CAUSE_LO)
        <= i_status.run_timeout_cause_last;
    s_stat7_src(c_STAT7_QUARANTINE_HI downto c_STAT7_QUARANTINE_LO)
        <= i_status.quarantine_escape_mask;
    -- CHAIN P1: OR of cell_pipe masked-slope hit-drop stickies (both
    -- slopes, all chips). Nonzero = physical edge misconfiguration.
    s_stat7_src(c_STAT7_MASKED_SLOPE_DROP) <= i_status.masked_slope_drop_any;
    s_stat7_src(c_STAT7_FS_COLL_RISE_HI downto c_STAT7_FS_COLL_RISE_LO)
        <= std_logic_vector(i_status.rise_face_start_collapsed_count(3 downto 0));
    s_stat7_src(c_STAT7_RESERVED_HI downto c_STAT7_RESERVED_LO)
        <= (others => '0');
    s_stat7_src(c_STAT7_FS_COLL_FALL_HI downto c_STAT7_FS_COLL_FALL_LO)
        <= std_logic_vector(i_status.fall_face_start_collapsed_count(3 downto 0));
    s_stat7_src(c_STAT7_INIT_COALESCE_HI downto c_STAT7_INIT_COALESCE_LO)
        <= i_status.init_cfg_coalesced_mask;

    -- synthesis translate_off
    -- Fail before CDC can conceal unresolved record fields as zeros and let
    -- exact-zero status checks pass vacuously.
    p_assert_status_known : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '1' then
                assert not is_x(s_stat_src)
                    report "tdc_gpx_csr_pipeline: STAT5 source contains X/U"
                    severity failure;
                assert not is_x(s_stat6_src)
                    report "tdc_gpx_csr_pipeline: STAT6 source contains X/U"
                    severity failure;
                assert not is_x(s_stat7_src)
                    report "tdc_gpx_csr_pipeline: STAT7 source contains X/U"
                    severity failure;
            end if;
        end if;
    end process p_assert_status_known;
    -- synthesis translate_on

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
    -- [5] CTL CDC: s_axi_aclk → i_axis_aclk (CTL0..CTL2)
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
                s_cdc_all_idle_src_r <= '1';
            elsif s_src_send_ctl = (s_src_send_ctl'range => '0')
              and i_chip_csr_cdc_idle = '1' then
                s_cdc_all_idle_src_r <= '1';
            else
                s_cdc_all_idle_src_r <= '0';
            end if;
        end if;
    end process p_cdc_idle_src;

    -- The combined idle level is registered in the AXI-Lite source domain
    -- before the two-stage synchronizer to avoid combinational CDC glitches.
    p_cdc_idle_sync : process(i_axis_aclk)
    begin
        if rising_edge(i_axis_aclk) then
            if i_axis_aresetn = '0' then
                s_cdc_all_idle_ff <= "11";
            else
                s_cdc_all_idle_ff(0) <= s_cdc_all_idle_src_r;
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
                -- Round 12 A1: force-reinit edge detect
                s_force_reinit_prev_r  <= '0';
                s_force_reinit_pulse_r <= '0';
                -- Round 13 follow-up: err soft-clear edge detect
                s_err_soft_clear_prev_r  <= '0';
                s_err_soft_clear_pulse_r <= '0';
            else
                s_cmd_prev_r  <= s_ctl_out(0)(31 downto 28);
                s_cmd_pulse_r <= s_ctl_out(0)(31 downto 28) and (not s_cmd_prev_r);
                -- Round 12 A1: CTL2[0] rising edge → force_reinit pulse.
                -- SW writes CTL2[0]=1 then CTL2[0]=0 to issue a single
                -- recovery attempt.
                s_force_reinit_prev_r  <= s_ctl_out(2)(0);
                s_force_reinit_pulse_r <= s_ctl_out(2)(0) and (not s_force_reinit_prev_r);
                -- Round 13 follow-up: CTL2[1] rising edge → err_soft_clear pulse.
                -- SW writes CTL2[1]=1 then CTL2[1]=0 to ack per-run error
                -- history (SOFT-CLEAR category). Higher bits of CTL2 reserved.
                s_err_soft_clear_prev_r  <= s_ctl_out(2)(1);
                s_err_soft_clear_pulse_r <= s_ctl_out(2)(1) and (not s_err_soft_clear_prev_r);
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
    o_cmd_force_reinit <= s_force_reinit_pulse_r;  -- Round 12 A1
    o_err_soft_clear   <= s_err_soft_clear_pulse_r;  -- Round 13 follow-up

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
    -- [8] CSR output: t_tdc_cfg field extraction (i_axis_aclk domain)
    -- =========================================================================
    -- CTL0: MAIN_CTRL
    s_requested_chip_mask <=
        s_ctl_out(0)(c_MC_ACTIVE_MASK_HI downto c_MC_ACTIVE_MASK_LO);
    s_effective_chip_mask <= s_requested_chip_mask and g_PRESENT_CHIP_MASK;
    o_cfg.active_chip_mask <= s_effective_chip_mask
                              when s_effective_chip_mask /= c_ZERO_CHIP_MASK
                              else fn_first_one_mask(g_PRESENT_CHIP_MASK);
    o_cfg.packet_scope     <= s_ctl_out(0)(c_MC_PACKET_SCOPE);
    o_cfg.hit_store_mode   <= unsigned(s_ctl_out(0)(c_MC_HIT_STORE_HI downto c_MC_HIT_STORE_LO));
    o_cfg.dist_scale       <= unsigned(s_ctl_out(0)(c_MC_DIST_SCALE_HI downto c_MC_DIST_SCALE_LO));
    o_cfg.drain_mode       <= s_ctl_out(0)(c_MC_DRAIN_MODE);
    o_cfg.pipeline_en      <= s_ctl_out(0)(c_MC_PIPELINE_EN);

    -- Face count is a static optical/mechanical contract owned by
    -- motor_decoder. MAIN_CTRL[14:12] is deliberately ignored so software
    -- cannot create a second, divergent geometry owner.
    o_cfg.n_faces          <= s_n_faces_effective;

    o_cfg.stops_per_chip   <= unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO))
                              when unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) >= 2
                                   and unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) <= g_MAX_STOPS_PER_CHIP
                              else to_unsigned(g_MAX_STOPS_PER_CHIP, 4)
                              when unsigned(s_ctl_out(0)(c_MC_STOPS_HI downto c_MC_STOPS_LO)) > g_MAX_STOPS_PER_CHIP
                              else to_unsigned(2, 4);

    o_cfg.n_drain_cap      <= unsigned(s_ctl_out(0)(c_MC_N_DRAIN_CAP_HI downto c_MC_N_DRAIN_CAP_LO));
    o_cfg.stopdis_override <= s_ctl_out(0)(c_MC_STOPDIS_HI downto c_MC_STOPDIS_LO);

    -- CTL1: RANGE_COLS
    o_cfg.max_range_5ns_ticks <= unsigned(
        s_ctl_out(1)(c_RC_MAX_RANGE_5NS_HI downto c_RC_MAX_RANGE_5NS_LO));

    -- Pipeline CSR is the sole geometry owner. The current laser_ctrl result
    -- stream carries step_idx/remaining, not a columns-per-face value.
    o_cfg.cols_per_face    <= unsigned(s_ctl_out(1)(c_RC_COLS_HI downto c_RC_COLS_LO))
                              when unsigned(s_ctl_out(1)(c_RC_COLS_HI downto c_RC_COLS_LO)) >= 1
                              else to_unsigned(1, 16);

    -- Fields owned by csr_chip — drive defaults here, overridden at top level
    -- by connecting csr_chip outputs directly to the downstream modules.
    o_cfg.bus_clk_div      <= to_unsigned(2, 6);     -- default, overridden
    o_cfg.bus_ticks        <= to_unsigned(5, 3);     -- default, overridden
    o_cfg.start_off1       <= (others => '0');       -- default, overridden
    o_cfg.cfg_reg7         <= (others => '0');       -- default, overridden
    o_cfg.max_scan_5ns_ticks <= (others => '0');     -- default, overridden
    o_cfg.max_hits_cfg     <= to_unsigned(g_MAX_HITS_PER_STOP, 3); -- default, overridden
    o_cfg.falling_enable   <= c_DEFAULT_FALLING_ENABLE; -- default, overridden

end architecture rtl;
