library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_unified_csr_pkg.all;

entity tb_lidar_unified_csr_top is
end entity tb_lidar_unified_csr_top;

architecture sim of tb_lidar_unified_csr_top is
    constant C_CLK_PERIOD : time := 10 ns;
    type t_word_array is array(natural range <>) of
        std_logic_vector(31 downto 0);

    function fn_tag(index : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(16#10000000# + index, 32));
    end function fn_tag;

    function fn_ctl_tag(index : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(16#20000000# + index, 32));
    end function fn_ctl_tag;

    function fn_status_init return t_word_array is
        variable v_result : t_word_array(0 to 30);
    begin
        for i in v_result'range loop
            v_result(i) := fn_tag(i);
        end loop;
        return v_result;
    end function fn_status_init;

    signal clk    : std_logic := '0';
    signal rst_n  : std_logic := '0';
    signal awaddr : std_logic_vector(8 downto 0) := (others => '0');
    signal awprot : std_logic_vector(2 downto 0) := (others => '0');
    signal awvalid : std_logic := '0';
    signal awready : std_logic;
    signal wdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal wstrb  : std_logic_vector(3 downto 0) := (others => '0');
    signal wvalid : std_logic := '0';
    signal wready : std_logic;
    signal bresp  : std_logic_vector(1 downto 0);
    signal bvalid : std_logic;
    signal bready : std_logic := '0';
    signal araddr : std_logic_vector(8 downto 0) := (others => '0');
    signal arprot : std_logic_vector(2 downto 0) := (others => '0');
    signal arvalid : std_logic := '0';
    signal arready : std_logic;
    signal rdata  : std_logic_vector(31 downto 0);
    signal rresp  : std_logic_vector(1 downto 0);
    signal rvalid : std_logic;
    signal rready : std_logic := '0';
    signal irq    : std_logic;

    signal s_control : t_word_array(0 to C_UCSR_ACTIVE_CTL_COUNT - 1);
    signal s_status  : t_word_array(0 to 30) := fn_status_init;

    signal s_laser_cfg_epoch : std_logic_vector(7 downto 0) :=
        (others => '0');
    signal s_laser_reset_epoch : std_logic_vector(7 downto 0) :=
        (others => '0');
    signal s_laser_busy   : std_logic := '0';
    signal s_laser_reject : std_logic := '0';
    signal s_laser_valid  : std_logic := '0';
    signal s_echo_reset_epoch : std_logic_vector(7 downto 0) :=
        (others => '0');
    signal s_tdc_cfg_epoch : std_logic_vector(7 downto 0) :=
        (others => '0');
    signal s_tdc_reset_epoch : std_logic_vector(7 downto 0) :=
        (others => '0');
    signal s_tdc_cfg_busy   : std_logic := '0';
    signal s_tdc_cfg_reject : std_logic := '0';
    signal s_tdc_cfg_valid  : std_logic := '0';
    signal s_tdc_cmd_busy   : std_logic := '0';
    signal s_tdc_cmd_reject : std_logic := '0';
    signal s_tdc_image_reject : std_logic := '0';

    signal s_motor_irq : std_logic_vector(3 downto 0) := (others => '0');
    signal s_laser_irq : std_logic_vector(2 downto 0) := (others => '0');
    signal s_echo_irq  : std_logic_vector(4 downto 0) := (others => '0');
    signal s_tdc_irq   : std_logic_vector(6 downto 0) := (others => '0');
begin
    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_unified_csr_top
        port map (
            s_axi_csr_aclk    => clk,
            s_axi_csr_aresetn => rst_n,
            s_axi_csr_awaddr  => awaddr,
            s_axi_csr_awprot  => awprot,
            s_axi_csr_awvalid => awvalid,
            s_axi_csr_awready => awready,
            s_axi_csr_wdata   => wdata,
            s_axi_csr_wstrb   => wstrb,
            s_axi_csr_wvalid  => wvalid,
            s_axi_csr_wready  => wready,
            s_axi_csr_bresp   => bresp,
            s_axi_csr_bvalid  => bvalid,
            s_axi_csr_bready  => bready,
            s_axi_csr_araddr  => araddr,
            s_axi_csr_arprot  => arprot,
            s_axi_csr_arvalid => arvalid,
            s_axi_csr_arready => arready,
            s_axi_csr_rdata   => rdata,
            s_axi_csr_rresp   => rresp,
            s_axi_csr_rvalid  => rvalid,
            s_axi_csr_rready  => rready,
            o_irq             => irq,

            o_sys_ctrl      => s_control(0),
            o_sys_cfg_apply => s_control(1),
            o_motor_cfg           => s_control(2),
            o_motor_ticks_lo      => s_control(3),
            o_motor_sched_latency => s_control(4),
            o_motor_z_param       => s_control(5),
            o_motor_face_index    => s_control(6),
            o_motor_face_geometry => s_control(7),
            o_laser_fire_cfg  => s_control(8),
            o_laser_roundtrip => s_control(9),
            o_laser_tdc_width => s_control(10),
            o_laser_sim_delay => s_control(11),
            o_laser_sched0    => s_control(12),
            o_laser_sched1    => s_control(13),
            o_laser_sched2    => s_control(14),
            o_echo_delay_cmd  => s_control(15),
            o_echo_delay_data => s_control(16),
            o_tdc_bus_timing    => s_control(17),
            o_tdc_start_offset  => s_control(18),
            o_tdc_cfg_reg7      => s_control(19),
            o_tdc_image_cmd     => s_control(20),
            o_tdc_image_data    => s_control(21),
            o_tdc_scan_cfg      => s_control(22),
            o_tdc_pipeline_main => s_control(23),
            o_tdc_range_cols    => s_control(24),
            o_tdc_aux_cmd       => s_control(25),

            i_tdc_max_rows  => s_status(3),
            i_tdc_cell_size => s_status(4),
            i_tdc_max_hsize => s_status(5),
            i_motor_status        => s_status(6),
            i_motor_face_geometry => s_status(7),
            i_motor_cfg_status    => s_status(8),
            i_motor_quad_invalid  => s_status(9),
            i_motor_axis_drop     => s_status(10),
            i_motor_rev_period    => s_status(11),
            i_motor_irq_cause     => s_motor_irq,
            i_laser_status          => s_status(12),
            i_laser_encoder_to_fire => s_status(13),
            i_laser_fire_done       => s_status(14),
            i_laser_fire_to_tdc     => s_status(15),
            i_laser_metric_flags    => s_status(16),
            i_laser_frame_count     => s_status(17),
            i_laser_timeout_count   => s_status(18),
            i_laser_cfg_epoch_accepted => s_laser_cfg_epoch,
            i_laser_reset_epoch_accepted => s_laser_reset_epoch,
            i_laser_cfg_busy   => s_laser_busy,
            i_laser_cfg_reject => s_laser_reject,
            i_laser_cfg_valid  => s_laser_valid,
            i_laser_irq_cause  => s_laser_irq,
            i_echo_rise_mask      => s_status(19),
            i_echo_fall_mask      => s_status(20),
            i_echo_status         => s_status(21),
            i_echo_delay_readback => s_status(22),
            i_echo_reset_epoch_accepted => s_echo_reset_epoch,
            i_echo_irq_cause      => s_echo_irq,
            i_tdc_chip0_result    => s_status(23),
            i_tdc_chip1_result    => s_status(24),
            i_tdc_chip2_result    => s_status(25),
            i_tdc_chip3_result    => s_status(26),
            i_tdc_pipeline_status => s_status(27),
            i_tdc_status_ext      => s_status(28),
            i_tdc_status_ext2     => s_status(29),
            i_tdc_image_selected_data => s_status(30),
            i_tdc_cfg_epoch_accepted => s_tdc_cfg_epoch,
            i_tdc_reset_epoch_accepted => s_tdc_reset_epoch,
            i_tdc_cfg_busy       => s_tdc_cfg_busy,
            i_tdc_cfg_reject     => s_tdc_cfg_reject,
            i_tdc_cfg_valid      => s_tdc_cfg_valid,
            i_tdc_cmd_busy       => s_tdc_cmd_busy,
            i_tdc_command_reject => s_tdc_cmd_reject,
            i_tdc_image_reject   => s_tdc_image_reject,
            i_tdc_irq_cause      => s_tdc_irq
        );

    p_stimulus : process
        variable v_passed : natural := 0;
        variable v_motor_cfg_status : std_logic_vector(31 downto 0);

        procedure wait_falling is
        begin
            wait until falling_edge(clk);
        end procedure wait_falling;

        procedure wait_settled is
        begin
            wait until rising_edge(clk);
            wait for 1 ps;
        end procedure wait_settled;

        procedure axi_write(
            constant address : natural;
            constant value   : std_logic_vector(31 downto 0);
            constant strobe  : std_logic_vector(3 downto 0) := "1111"
        ) is
            variable aw_done : boolean := false;
            variable w_done  : boolean := false;
        begin
            wait_falling;
            awaddr  <= std_logic_vector(to_unsigned(address, awaddr'length));
            awvalid <= '1';
            wdata   <= value;
            wstrb   <= strobe;
            wvalid  <= '1';

            while not (aw_done and w_done) loop
                wait until rising_edge(clk);
                if awvalid = '1' and awready = '1' then
                    awvalid <= '0';
                    aw_done := true;
                end if;
                if wvalid = '1' and wready = '1' then
                    wvalid <= '0';
                    w_done := true;
                end if;
            end loop;
            wstrb <= (others => '0');

            while bvalid = '0' loop
                wait_settled;
            end loop;
            assert bresp = "00"
                report "AXI write response was not OKAY"
                severity failure;
            wait_falling;
            bready <= '1';
            wait until rising_edge(clk);
            bready <= '0';
            wait for 1 ps;
        end procedure axi_write;

        procedure axi_read(
            constant address  : natural;
            constant expected : std_logic_vector(31 downto 0)
        ) is
        begin
            wait_falling;
            araddr  <= std_logic_vector(to_unsigned(address, araddr'length));
            arvalid <= '1';
            loop
                wait until rising_edge(clk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';
            while rvalid = '0' loop
                wait_settled;
            end loop;
            assert rresp = "00"
                report "AXI read response was not OKAY"
                severity failure;
            assert rdata = expected
                report "AXI read mismatch at address " & integer'image(address)
                severity failure;
            wait_falling;
            rready <= '1';
            wait until rising_edge(clk);
            rready <= '0';
            wait for 1 ps;
        end procedure axi_read;
    begin
        rst_n <= '0';
        for i in 1 to 5 loop
            wait_settled;
        end loop;
        rst_n <= '1';
        for i in 1 to 3 loop
            wait_settled;
        end loop;

        axi_read(fn_stat_byte_offset(0), C_UCSR_VERSION_WORD);
        axi_read(fn_stat_byte_offset(1), C_UCSR_CAPABILITY_WORD);
        for i in 3 to 30 loop
            axi_read(fn_stat_byte_offset(i), s_status(i));
        end loop;
        v_passed := v_passed + 1;

        for i in 0 to C_UCSR_ACTIVE_CTL_COUNT - 1 loop
            axi_write(fn_ctl_byte_offset(i), fn_ctl_tag(i));
            assert s_control(i) = fn_ctl_tag(i)
                report "Active CTL routing mismatch at index " &
                       integer'image(i)
                severity failure;
        end loop;
        v_passed := v_passed + 1;

        axi_write(fn_ctl_byte_offset(C_CTL_LASER_FIRE_CFG),
                  x"AABBCCDD", "0101");
        assert s_control(C_CTL_LASER_FIRE_CFG) = x"20BB00DD"
            report "Byte-strobe update did not reach Laser FIRE_CFG"
            severity failure;
        v_passed := v_passed + 1;

        for i in C_CTL_RESERVED_FIRST to C_CTL_RESERVED_LAST loop
            axi_write(fn_ctl_byte_offset(i), x"FFFFFFFF");
        end loop;
        assert s_control(C_CTL_SYS_CTRL) = fn_ctl_tag(C_CTL_SYS_CTRL)
            and s_control(C_CTL_TDC_AUX_CMD) = fn_ctl_tag(C_CTL_TDC_AUX_CMD)
            and s_control(C_CTL_LASER_FIRE_CFG) = x"20BB00DD"
            report "Reserved CTL write changed an active routed control"
            severity failure;
        v_passed := v_passed + 1;

        axi_write(fn_ctl_byte_offset(C_CTL_SYS_CTRL), x"00002200");
        axi_write(fn_ctl_byte_offset(C_CTL_SYS_CFG_APPLY), x"00000011");

        v_motor_cfg_status := (others => '0');
        v_motor_cfg_status(C_MOTOR_CFG_STATUS_CFG_EPOCH_HI downto
                           C_MOTOR_CFG_STATUS_CFG_EPOCH_LO) := x"11";
        v_motor_cfg_status(C_MOTOR_CFG_STATUS_RESET_EPOCH_HI downto
                           C_MOTOR_CFG_STATUS_RESET_EPOCH_LO) := x"22";
        v_motor_cfg_status(C_MOTOR_CFG_STATUS_VALID_BIT) := '1';
        s_status(8) <= v_motor_cfg_status;
        s_laser_cfg_epoch <= x"11";
        s_laser_reset_epoch <= x"22";
        s_laser_valid <= '1';
        s_echo_reset_epoch <= x"22";
        s_tdc_cfg_epoch <= x"11";
        s_tdc_reset_epoch <= x"22";
        s_tdc_cfg_valid <= '1';
        wait_settled;

        axi_read(fn_stat_byte_offset(C_STAT_SYS_CONFIG), x"11112211");
        axi_read(fn_stat_byte_offset(C_STAT_SYS_ADAPTER_STATE),
                 x"000F807F");
        v_passed := v_passed + 1;

        v_motor_cfg_status(C_MOTOR_CFG_STATUS_BUSY_BIT) := '1';
        v_motor_cfg_status(C_MOTOR_CFG_STATUS_REJECT_BIT) := '1';
        s_status(8) <= v_motor_cfg_status;
        s_laser_busy <= '1';
        s_laser_reject <= '1';
        s_status(21)(C_ECHO_STATUS_APPLY_PENDING_BIT) <= '1';
        s_status(21)(C_ECHO_STATUS_CMD_REJECT_BIT) <= '1';
        s_tdc_cmd_busy <= '1';
        s_tdc_image_reject <= '1';
        wait_settled;
        axi_read(fn_stat_byte_offset(C_STAT_SYS_ADAPTER_STATE),
                 x"003FFFFF");
        v_passed := v_passed + 1;

        s_motor_irq <= "0001";
        s_laser_irq <= "001";
        s_echo_irq  <= "11111";
        s_tdc_irq   <= "0000001";
        axi_write(fn_intr_byte_offset(0), x"003F0110");
        for i in 1 to 5 loop
            wait_settled;
        end loop;
        s_motor_irq <= (others => '0');
        s_laser_irq <= (others => '0');
        s_echo_irq  <= (others => '0');
        s_tdc_irq   <= (others => '0');

        for i in 1 to 10 loop
            exit when irq = '1';
            wait_settled;
        end loop;
        assert irq = '1'
            report "Unified IRQ did not assert"
            severity failure;
        axi_read(fn_intr_byte_offset(2), x"003F0110");
        axi_write(fn_intr_byte_offset(2), x"003F0110");
        for i in 1 to 3 loop
            wait_settled;
        end loop;
        assert irq = '0'
            report "Unified IRQ did not clear after W1C"
            severity failure;
        axi_read(fn_intr_byte_offset(2), x"00000000");
        v_passed := v_passed + 1;

        report "[SUMMARY] Passed=" & integer'image(v_passed) & " Failed=0"
            severity note;
        report "LIDAR_UNIFIED_CSR_TOP_PASS" severity note;
        stop;
        wait;
    end process p_stimulus;
end architecture sim;
