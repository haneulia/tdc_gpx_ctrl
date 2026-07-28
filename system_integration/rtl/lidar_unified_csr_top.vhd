-- =============================================================================
-- LiDAR unified CSR control-plane top
--
-- This block owns the only AXI4-Lite CSR bank in unified mode. It performs no
-- real-time Motor, Laser, Echo, or TDC processing; it only routes registered
-- controls, status snapshots, transaction state, and interrupt causes.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

use work.lidar_unified_csr_pkg.all;

entity lidar_unified_csr_top is
    generic (
        g_VERSION_WORD : std_logic_vector(31 downto 0) :=
            C_UCSR_VERSION_WORD;
        g_CAPABILITY_WORD : std_logic_vector(31 downto 0) :=
            C_UCSR_CAPABILITY_WORD
    );
    port (
        -- AXI4-Lite CSR domain. This clock/reset also drives every child
        -- unified configuration interface in the parent integration top.
        s_axi_csr_aclk    : in  std_logic;
        s_axi_csr_aresetn : in  std_logic;
        s_axi_csr_awaddr  : in  std_logic_vector(8 downto 0);
        s_axi_csr_awprot  : in  std_logic_vector(2 downto 0);
        s_axi_csr_awvalid : in  std_logic;
        s_axi_csr_awready : out std_logic;
        s_axi_csr_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_csr_wstrb   : in  std_logic_vector(3 downto 0);
        s_axi_csr_wvalid  : in  std_logic;
        s_axi_csr_wready  : out std_logic;
        s_axi_csr_bresp   : out std_logic_vector(1 downto 0);
        s_axi_csr_bvalid  : out std_logic;
        s_axi_csr_bready  : in  std_logic;
        s_axi_csr_araddr  : in  std_logic_vector(8 downto 0);
        s_axi_csr_arprot  : in  std_logic_vector(2 downto 0);
        s_axi_csr_arvalid : in  std_logic;
        s_axi_csr_arready : out std_logic;
        s_axi_csr_rdata   : out std_logic_vector(31 downto 0);
        s_axi_csr_rresp   : out std_logic_vector(1 downto 0);
        s_axi_csr_rvalid  : out std_logic;
        s_axi_csr_rready  : in  std_logic;
        o_irq             : out std_logic;

        -- Shared system transaction words.
        o_sys_ctrl      : out std_logic_vector(31 downto 0);
        o_sys_cfg_apply : out std_logic_vector(31 downto 0);

        -- Motor CTL2..CTL7.
        o_motor_cfg            : out std_logic_vector(31 downto 0);
        o_motor_ticks_lo       : out std_logic_vector(31 downto 0);
        o_motor_sched_latency  : out std_logic_vector(31 downto 0);
        o_motor_z_param        : out std_logic_vector(31 downto 0);
        o_motor_face_index     : out std_logic_vector(31 downto 0);
        o_motor_face_geometry  : out std_logic_vector(31 downto 0);

        -- Laser CTL8..CTL14.
        o_laser_fire_cfg  : out std_logic_vector(31 downto 0);
        o_laser_roundtrip : out std_logic_vector(31 downto 0);
        o_laser_tdc_width : out std_logic_vector(31 downto 0);
        o_laser_sim_delay : out std_logic_vector(31 downto 0);
        o_laser_sched0    : out std_logic_vector(31 downto 0);
        o_laser_sched1    : out std_logic_vector(31 downto 0);
        o_laser_sched2    : out std_logic_vector(31 downto 0);

        -- Echo CTL15..CTL16.
        o_echo_delay_cmd  : out std_logic_vector(31 downto 0);
        o_echo_delay_data : out std_logic_vector(31 downto 0);

        -- TDC-GPX CTL17..CTL25.
        o_tdc_bus_timing    : out std_logic_vector(31 downto 0);
        o_tdc_start_offset  : out std_logic_vector(31 downto 0);
        o_tdc_cfg_reg7      : out std_logic_vector(31 downto 0);
        o_tdc_image_cmd     : out std_logic_vector(31 downto 0);
        o_tdc_image_data    : out std_logic_vector(31 downto 0);
        o_tdc_scan_cfg      : out std_logic_vector(31 downto 0);
        o_tdc_pipeline_main : out std_logic_vector(31 downto 0);
        o_tdc_range_cols    : out std_logic_vector(31 downto 0);
        o_tdc_aux_cmd       : out std_logic_vector(31 downto 0);

        -- Build-time/static TDC geometry for STAT3..STAT5.
        i_tdc_max_rows  : in std_logic_vector(31 downto 0) :=
            (others => '0');
        i_tdc_cell_size : in std_logic_vector(31 downto 0) :=
            (others => '0');
        i_tdc_max_hsize : in std_logic_vector(31 downto 0) :=
            (others => '0');

        -- Motor STAT6..STAT11 and IRQ4..IRQ7.
        i_motor_status        : in std_logic_vector(31 downto 0);
        i_motor_face_geometry : in std_logic_vector(31 downto 0);
        i_motor_cfg_status    : in std_logic_vector(31 downto 0);
        i_motor_quad_invalid  : in std_logic_vector(31 downto 0);
        i_motor_axis_drop     : in std_logic_vector(31 downto 0);
        i_motor_rev_period    : in std_logic_vector(31 downto 0);
        i_motor_irq_cause     : in std_logic_vector(3 downto 0);

        -- Laser STAT12..STAT18, transaction sideband, and IRQ8..IRQ10.
        i_laser_status          : in std_logic_vector(31 downto 0);
        i_laser_encoder_to_fire : in std_logic_vector(31 downto 0);
        i_laser_fire_done       : in std_logic_vector(31 downto 0);
        i_laser_fire_to_tdc     : in std_logic_vector(31 downto 0);
        i_laser_metric_flags    : in std_logic_vector(31 downto 0);
        i_laser_frame_count     : in std_logic_vector(31 downto 0);
        i_laser_timeout_count   : in std_logic_vector(31 downto 0);
        i_laser_cfg_epoch_accepted : in std_logic_vector(7 downto 0);
        i_laser_reset_epoch_accepted : in std_logic_vector(7 downto 0);
        i_laser_cfg_busy   : in std_logic;
        i_laser_cfg_reject : in std_logic;
        i_laser_cfg_valid  : in std_logic;
        i_laser_irq_cause  : in std_logic_vector(2 downto 0);

        -- Echo STAT19..STAT22, reset acknowledgement, and IRQ16..IRQ17.
        i_echo_rise_mask      : in std_logic_vector(31 downto 0);
        i_echo_fall_mask      : in std_logic_vector(31 downto 0);
        i_echo_status         : in std_logic_vector(31 downto 0);
        i_echo_delay_readback : in std_logic_vector(31 downto 0);
        i_echo_reset_epoch_accepted : in std_logic_vector(7 downto 0);
        i_echo_irq_cause      : in std_logic_vector(1 downto 0);

        -- TDC STAT23..STAT30, transaction sideband, and IRQ21..IRQ27.
        i_tdc_chip0_result    : in std_logic_vector(31 downto 0);
        i_tdc_chip1_result    : in std_logic_vector(31 downto 0);
        i_tdc_chip2_result    : in std_logic_vector(31 downto 0);
        i_tdc_chip3_result    : in std_logic_vector(31 downto 0);
        i_tdc_pipeline_status : in std_logic_vector(31 downto 0);
        i_tdc_status_ext      : in std_logic_vector(31 downto 0);
        i_tdc_status_ext2     : in std_logic_vector(31 downto 0);
        i_tdc_image_selected_data : in std_logic_vector(31 downto 0);
        i_tdc_cfg_epoch_accepted : in std_logic_vector(7 downto 0);
        i_tdc_reset_epoch_accepted : in std_logic_vector(7 downto 0);
        i_tdc_cfg_busy       : in std_logic;
        i_tdc_cfg_reject     : in std_logic;
        i_tdc_cfg_valid      : in std_logic;
        i_tdc_cmd_busy       : in std_logic;
        i_tdc_command_reject : in std_logic;
        i_tdc_image_reject   : in std_logic;
        i_tdc_irq_cause      : in std_logic_vector(6 downto 0)
    );

    attribute X_INTERFACE_INFO : string;
    attribute X_INTERFACE_PARAMETER : string;
    attribute X_INTERFACE_INFO of o_irq : signal is
        "xilinx.com:signal:interrupt:1.0 irq INTERRUPT";
    attribute X_INTERFACE_PARAMETER of o_irq : signal is
        "SENSITIVITY EDGE_RISING";
    attribute X_INTERFACE_INFO of s_axi_csr_aclk : signal is
        "xilinx.com:signal:clock:1.0 s_axi_csr_aclk CLK";
    attribute X_INTERFACE_PARAMETER of s_axi_csr_aclk : signal is
        "ASSOCIATED_BUSIF s_axi_csr, ASSOCIATED_RESET s_axi_csr_aresetn";
end entity lidar_unified_csr_top;

architecture rtl of lidar_unified_csr_top is
    type t_word_array is array(0 to C_UCSR_CTL_COUNT - 1) of
        std_logic_vector(31 downto 0);

    constant C_ZERO32 : std_logic_vector(31 downto 0) := (others => '0');

    signal s_ctl : t_word_array := (others => C_ZERO32);
    signal s_irq_src : std_logic_vector(31 downto 0);
    signal s_sys_config : std_logic_vector(31 downto 0);
    signal s_sys_adapter_state : std_logic_vector(31 downto 0);

    signal s_motor_cfg_match : std_logic;
    signal s_laser_cfg_match : std_logic;
    signal s_tdc_cfg_match   : std_logic;
    signal s_motor_reset_match : std_logic;
    signal s_laser_reset_match : std_logic;
    signal s_echo_reset_match  : std_logic;
    signal s_tdc_reset_match   : std_logic;
    signal s_motor_busy : std_logic;
    signal s_echo_busy  : std_logic;
    signal s_tdc_busy   : std_logic;
    signal s_motor_reject : std_logic;
    signal s_echo_reject  : std_logic;
    signal s_tdc_reject   : std_logic;
    signal s_motor_valid : std_logic;
    signal s_all_cfg_accepted   : std_logic;
    signal s_all_reset_accepted : std_logic;
    signal s_any_busy   : std_logic;
    signal s_any_reject : std_logic;
begin
    -- Only active words leave this block. CTL26..CTL31 may retain AXI
    -- readback values, but have no processing output and therefore no effect.
    o_sys_ctrl      <= s_ctl(C_CTL_SYS_CTRL);
    o_sys_cfg_apply <= s_ctl(C_CTL_SYS_CFG_APPLY);

    o_motor_cfg           <= s_ctl(C_CTL_MOTOR_CFG);
    o_motor_ticks_lo      <= s_ctl(C_CTL_MOTOR_TICKS_LO);
    o_motor_sched_latency <= s_ctl(C_CTL_MOTOR_SCHED_LATENCY);
    o_motor_z_param       <= s_ctl(C_CTL_MOTOR_Z_PARAM);
    o_motor_face_index    <= s_ctl(C_CTL_MOTOR_FACE_INDEX);
    o_motor_face_geometry <= s_ctl(C_CTL_MOTOR_FACE_GEOMETRY);

    o_laser_fire_cfg  <= s_ctl(C_CTL_LASER_FIRE_CFG);
    o_laser_roundtrip <= s_ctl(C_CTL_LASER_ROUNDTRIP);
    o_laser_tdc_width <= s_ctl(C_CTL_LASER_TDC_WIDTH);
    o_laser_sim_delay <= s_ctl(C_CTL_LASER_SIM_DELAY);
    o_laser_sched0    <= s_ctl(C_CTL_LASER_SCHED0);
    o_laser_sched1    <= s_ctl(C_CTL_LASER_SCHED1);
    o_laser_sched2    <= s_ctl(C_CTL_LASER_SCHED2);

    o_echo_delay_cmd  <= s_ctl(C_CTL_ECHO_DELAY_CMD);
    o_echo_delay_data <= s_ctl(C_CTL_ECHO_DELAY_DATA);

    o_tdc_bus_timing    <= s_ctl(C_CTL_TDC_BUS_TIMING);
    o_tdc_start_offset  <= s_ctl(C_CTL_TDC_START_OFFSET);
    o_tdc_cfg_reg7      <= s_ctl(C_CTL_TDC_CFG_REG7);
    o_tdc_image_cmd     <= s_ctl(C_CTL_TDC_IMAGE_CMD);
    o_tdc_image_data    <= s_ctl(C_CTL_TDC_IMAGE_DATA);
    o_tdc_scan_cfg      <= s_ctl(C_CTL_TDC_SCAN_CFG);
    o_tdc_pipeline_main <= s_ctl(C_CTL_TDC_PIPELINE_MAIN);
    o_tdc_range_cols    <= s_ctl(C_CTL_TDC_RANGE_COLS);
    o_tdc_aux_cmd       <= s_ctl(C_CTL_TDC_AUX_CMD);

    s_motor_valid <= i_motor_cfg_status(C_MOTOR_CFG_STATUS_VALID_BIT);
    s_motor_busy <= i_motor_cfg_status(C_MOTOR_CFG_STATUS_BUSY_BIT);
    s_motor_reject <= i_motor_cfg_status(C_MOTOR_CFG_STATUS_REJECT_BIT);
    s_echo_busy <= i_echo_status(C_ECHO_STATUS_APPLY_PENDING_BIT);
    s_echo_reject <= i_echo_status(C_ECHO_STATUS_CMD_REJECT_BIT);
    s_tdc_busy <= i_tdc_cfg_busy or i_tdc_cmd_busy;
    s_tdc_reject <= i_tdc_cfg_reject or i_tdc_command_reject
        or i_tdc_image_reject;

    s_motor_cfg_match <= '1' when
        s_motor_valid = '1'
        and i_motor_cfg_status(C_MOTOR_CFG_STATUS_CFG_EPOCH_HI downto
                               C_MOTOR_CFG_STATUS_CFG_EPOCH_LO) =
            s_ctl(C_CTL_SYS_CFG_APPLY)(C_SYS_CFG_EPOCH_HI downto
                                       C_SYS_CFG_EPOCH_LO)
        else '0';
    s_laser_cfg_match <= '1' when
        i_laser_cfg_valid = '1'
        and i_laser_cfg_epoch_accepted =
            s_ctl(C_CTL_SYS_CFG_APPLY)(C_SYS_CFG_EPOCH_HI downto
                                       C_SYS_CFG_EPOCH_LO)
        else '0';
    s_tdc_cfg_match <= '1' when
        i_tdc_cfg_valid = '1'
        and i_tdc_cfg_epoch_accepted =
            s_ctl(C_CTL_SYS_CFG_APPLY)(C_SYS_CFG_EPOCH_HI downto
                                       C_SYS_CFG_EPOCH_LO)
        else '0';

    s_motor_reset_match <= '1' when
        i_motor_cfg_status(C_MOTOR_CFG_STATUS_RESET_EPOCH_HI downto
                           C_MOTOR_CFG_STATUS_RESET_EPOCH_LO) =
            s_ctl(C_CTL_SYS_CTRL)(C_SYS_CTRL_RESET_EPOCH_HI downto
                                  C_SYS_CTRL_RESET_EPOCH_LO)
        else '0';
    s_laser_reset_match <= '1' when
        i_laser_reset_epoch_accepted =
            s_ctl(C_CTL_SYS_CTRL)(C_SYS_CTRL_RESET_EPOCH_HI downto
                                  C_SYS_CTRL_RESET_EPOCH_LO)
        else '0';
    s_echo_reset_match <= '1' when
        i_echo_reset_epoch_accepted =
            s_ctl(C_CTL_SYS_CTRL)(C_SYS_CTRL_RESET_EPOCH_HI downto
                                  C_SYS_CTRL_RESET_EPOCH_LO)
        else '0';
    s_tdc_reset_match <= '1' when
        i_tdc_reset_epoch_accepted =
            s_ctl(C_CTL_SYS_CTRL)(C_SYS_CTRL_RESET_EPOCH_HI downto
                                  C_SYS_CTRL_RESET_EPOCH_LO)
        else '0';

    s_all_cfg_accepted <= s_motor_cfg_match and s_laser_cfg_match
        and s_tdc_cfg_match;
    s_all_reset_accepted <= s_motor_reset_match and s_laser_reset_match
        and s_echo_reset_match and s_tdc_reset_match;
    s_any_busy <= s_motor_busy or i_laser_cfg_busy or s_echo_busy
        or s_tdc_busy;
    s_any_reject <= s_motor_reject or i_laser_cfg_reject or s_echo_reject
        or s_tdc_reject;

    s_sys_config <=
        i_tdc_cfg_epoch_accepted
        & i_laser_cfg_epoch_accepted
        & s_ctl(C_CTL_SYS_CTRL)(C_SYS_CTRL_RESET_EPOCH_HI downto
                                C_SYS_CTRL_RESET_EPOCH_LO)
        & s_ctl(C_CTL_SYS_CFG_APPLY)(C_SYS_CFG_EPOCH_HI downto
                                     C_SYS_CFG_EPOCH_LO);

    s_sys_adapter_state <=
        "0000000000"
        & s_any_reject
        & s_any_busy
        & s_all_reset_accepted
        & s_all_cfg_accepted
        & i_tdc_cfg_valid
        & i_laser_cfg_valid
        & s_motor_valid
        & s_tdc_reject
        & s_echo_reject
        & i_laser_cfg_reject
        & s_motor_reject
        & s_tdc_busy
        & s_echo_busy
        & i_laser_cfg_busy
        & s_motor_busy
        & s_tdc_reset_match
        & s_echo_reset_match
        & s_laser_reset_match
        & s_motor_reset_match
        & s_tdc_cfg_match
        & s_laser_cfg_match
        & s_motor_cfg_match;

    -- Global IRQ identity is preserved by fixed slices. System IRQ0..3 and
    -- reserved IRQ28..31 remain zero until a separate source contract exists.
    s_irq_src <=
        "0000"
        & i_tdc_irq_cause
        & "000"
        & i_echo_irq_cause
        & "00000"
        & i_laser_irq_cause
        & i_motor_irq_cause
        & "0000";

    u_csr : entity work.my_axil_csr32_top
        generic map (
            num_data_bits      => 32,
            num_ctl_regs       => C_UCSR_CTL_COUNT,
            num_stat_regs      => C_UCSR_STAT_COUNT,
            has_interrupt_regs => true,
            num_intr_regs      => C_UCSR_INTR_COUNT,
            num_interrupt_src  => C_UCSR_INTR_SOURCE_COUNT
        )
        port map (
            s_axi_csr_aclk    => s_axi_csr_aclk,
            s_axi_csr_aresetn => s_axi_csr_aresetn,
            s_axi_csr_awaddr  => s_axi_csr_awaddr,
            s_axi_csr_awprot  => s_axi_csr_awprot,
            s_axi_csr_awvalid => s_axi_csr_awvalid,
            s_axi_csr_awready => s_axi_csr_awready,
            s_axi_csr_wdata   => s_axi_csr_wdata,
            s_axi_csr_wstrb   => s_axi_csr_wstrb,
            s_axi_csr_wvalid  => s_axi_csr_wvalid,
            s_axi_csr_wready  => s_axi_csr_wready,
            s_axi_csr_bresp   => s_axi_csr_bresp,
            s_axi_csr_bvalid  => s_axi_csr_bvalid,
            s_axi_csr_bready  => s_axi_csr_bready,
            s_axi_csr_araddr  => s_axi_csr_araddr,
            s_axi_csr_arprot  => s_axi_csr_arprot,
            s_axi_csr_arvalid => s_axi_csr_arvalid,
            s_axi_csr_arready => s_axi_csr_arready,
            s_axi_csr_rdata   => s_axi_csr_rdata,
            s_axi_csr_rresp   => s_axi_csr_rresp,
            s_axi_csr_rvalid  => s_axi_csr_rvalid,
            s_axi_csr_rready  => s_axi_csr_rready,

            reg0_init_val  => C_ZERO32, reg1_init_val  => C_ZERO32,
            reg2_init_val  => C_ZERO32, reg3_init_val  => C_ZERO32,
            reg4_init_val  => C_ZERO32, reg5_init_val  => C_ZERO32,
            reg6_init_val  => C_ZERO32, reg7_init_val  => C_ZERO32,
            reg8_init_val  => C_ZERO32, reg9_init_val  => C_ZERO32,
            reg10_init_val => C_ZERO32, reg11_init_val => C_ZERO32,
            reg12_init_val => C_ZERO32, reg13_init_val => C_ZERO32,
            reg14_init_val => C_ZERO32, reg15_init_val => C_ZERO32,
            reg16_init_val => C_ZERO32, reg17_init_val => C_ZERO32,
            reg18_init_val => C_ZERO32, reg19_init_val => C_ZERO32,
            reg20_init_val => C_ZERO32, reg21_init_val => C_ZERO32,
            reg22_init_val => C_ZERO32, reg23_init_val => C_ZERO32,
            reg24_init_val => C_ZERO32, reg25_init_val => C_ZERO32,
            reg26_init_val => C_ZERO32, reg27_init_val => C_ZERO32,
            reg28_init_val => C_ZERO32, reg29_init_val => C_ZERO32,
            reg30_init_val => C_ZERO32, reg31_init_val => C_ZERO32,

            ctl0_out  => s_ctl(0),  ctl1_out  => s_ctl(1),
            ctl2_out  => s_ctl(2),  ctl3_out  => s_ctl(3),
            ctl4_out  => s_ctl(4),  ctl5_out  => s_ctl(5),
            ctl6_out  => s_ctl(6),  ctl7_out  => s_ctl(7),
            ctl8_out  => s_ctl(8),  ctl9_out  => s_ctl(9),
            ctl10_out => s_ctl(10), ctl11_out => s_ctl(11),
            ctl12_out => s_ctl(12), ctl13_out => s_ctl(13),
            ctl14_out => s_ctl(14), ctl15_out => s_ctl(15),
            ctl16_out => s_ctl(16), ctl17_out => s_ctl(17),
            ctl18_out => s_ctl(18), ctl19_out => s_ctl(19),
            ctl20_out => s_ctl(20), ctl21_out => s_ctl(21),
            ctl22_out => s_ctl(22), ctl23_out => s_ctl(23),
            ctl24_out => s_ctl(24), ctl25_out => s_ctl(25),
            ctl26_out => s_ctl(26), ctl27_out => s_ctl(27),
            ctl28_out => s_ctl(28), ctl29_out => s_ctl(29),
            ctl30_out => s_ctl(30), ctl31_out => s_ctl(31),

            stat0_in  => g_VERSION_WORD,
            stat1_in  => g_CAPABILITY_WORD,
            stat2_in  => s_sys_config,
            stat3_in  => i_tdc_max_rows,
            stat4_in  => i_tdc_cell_size,
            stat5_in  => i_tdc_max_hsize,
            stat6_in  => i_motor_status,
            stat7_in  => i_motor_face_geometry,
            stat8_in  => i_motor_cfg_status,
            stat9_in  => i_motor_quad_invalid,
            stat10_in => i_motor_axis_drop,
            stat11_in => i_motor_rev_period,
            stat12_in => i_laser_status,
            stat13_in => i_laser_encoder_to_fire,
            stat14_in => i_laser_fire_done,
            stat15_in => i_laser_fire_to_tdc,
            stat16_in => i_laser_metric_flags,
            stat17_in => i_laser_frame_count,
            stat18_in => i_laser_timeout_count,
            stat19_in => i_echo_rise_mask,
            stat20_in => i_echo_fall_mask,
            stat21_in => i_echo_status,
            stat22_in => i_echo_delay_readback,
            stat23_in => i_tdc_chip0_result,
            stat24_in => i_tdc_chip1_result,
            stat25_in => i_tdc_chip2_result,
            stat26_in => i_tdc_chip3_result,
            stat27_in => i_tdc_pipeline_status,
            stat28_in => i_tdc_status_ext,
            stat29_in => i_tdc_status_ext2,
            stat30_in => i_tdc_image_selected_data,
            stat31_in => s_sys_adapter_state,

            intrpt_src_in => s_irq_src,
            irq           => o_irq
        );
end architecture rtl;
