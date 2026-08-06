library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_csr_map_pkg.all;
use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_lidar_ctrl_v2_k05 is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
end entity tb_tdc_gpx_lidar_ctrl_v2_k05;

architecture sim of tb_tdc_gpx_lidar_ctrl_v2_k05 is

    constant C_CHIPS : positive := 2;
    constant C_STOPS_PER_CHIP : positive := 8;
    constant C_RETURNS_PER_STOP : positive := 7;
    constant C_WORDS_PER_IFIFO : positive :=
        (C_STOPS_PER_CHIP / 2) * C_RETURNS_PER_STOP;
    constant C_WORDS_PER_CHIP : positive := 2 * C_WORDS_PER_IFIFO;

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.num_chips := C_CHIPS;
        result.stops_per_chip := C_STOPS_PER_CHIP;
        result.max_returns_per_stop := C_RETURNS_PER_STOP;
        result.rise_capability_mask := "0011";
        result.fall_capability_mask := "0000";
        result.output_width := 32;
        result.num_faces := 1;
        result.enable_echo_receiver := true;
        result.enable_echo_simulation := true;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;

    function fn_runtime_config return lidar_runtime_config_t is
        variable result : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
    begin
        result.motor.cpr := to_unsigned(8, 16);
        result.motor.decode_mode := DECODE_X4;
        result.motor.direction := DIRECTION_CW;
        result.motor.virtual_ticks_lo := to_unsigned(3, 32);
        result.motor.virtual_hi_count := (others => '0');
        result.motor.simulation_mode := '1';
        result.mirror.face_centers := (others => (others => '0'));
        result.mirror.face_centers(0) :=
            to_unsigned(7, C_POSITION_WIDTH);
        result.mirror.common_half_width :=
            to_unsigned(1, C_POSITION_WIDTH);
        result.laser.face_enable_mask := "00001";
        result.laser.fire_width_5ns_ticks := to_unsigned(2, 16);
        result.laser.fire_done_timeout_5ns_ticks := to_unsigned(16, 16);
        result.laser.target_range_window_5ns := to_unsigned(48, 32);
        result.laser.start_width_5ns_ticks := to_unsigned(2, 16);
        result.laser.stop_width_5ns_ticks := to_unsigned(2, 16);
        result.laser.simulation_start_delay_5ns := to_unsigned(3, 32);
        result.laser.optical_shot_interval_udeg :=
            to_unsigned(45_000_000, angle_udeg_t'length);
        result.echo.channel_0_delay_5ns := to_unsigned(2, 16);
        result.echo.channel_step_5ns := to_unsigned(1, 16);
        result.tdc.active_chip_mask := "0011";
        result.tdc.falling_enable := '0';
        result.tdc.max_hits_per_stop := to_unsigned(7, 3);
        return result;
    end function fn_runtime_config;

    function fn_raw_word(
        chip_index : natural;
        ififo_id   : std_logic;
        word_index : natural
    ) return std_logic_vector is
        variable result : std_logic_vector(27 downto 0) := (others => '0');
        variable hit_value : natural;
    begin
        result(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(word_index mod 4, 2));
        result(c_RAW_STARTNUM_HI downto c_RAW_STARTNUM_LO) :=
            (others => '0');
        result(c_RAW_SLOPE_BIT) := '1';
        hit_value := chip_index * 16#2000# + word_index;
        if ififo_id = '1' then
            hit_value := hit_value + 16#0800#;
        end if;
        result(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, c_RAW_HIT_WIDTH));
        return result;
    end function fn_raw_word;

    constant C_RUNTIME : lidar_runtime_config_t := fn_runtime_config;
    constant C_DERIVED : lidar_derived_config_t :=
        fn_derive_runtime_config(C_BUILD_CONFIG, C_RUNTIME);
    constant C_WORDS : csr_word_array_t :=
        fn_pack_runtime_config(C_RUNTIME);
    constant C_CSR_PERIOD : time := 10 ns;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;

    type natural_array_t is array (0 to C_CHIPS - 1) of
        natural range 0 to 65535;
    type logic_array_t is array (0 to C_CHIPS - 1) of std_logic;

    signal csr_clk : std_logic := '0';
    signal proc_clk : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal csr_rst_n : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';
    signal done : boolean := false;

    signal awaddr : std_logic_vector(8 downto 0) := (others => '0');
    signal awprot : std_logic_vector(2 downto 0) := (others => '0');
    signal awvalid : std_logic := '0';
    signal awready : std_logic;
    signal wdata : std_logic_vector(31 downto 0) := (others => '0');
    signal wstrb : std_logic_vector(3 downto 0) := (others => '0');
    signal wvalid : std_logic := '0';
    signal wready : std_logic;
    signal bresp : std_logic_vector(1 downto 0);
    signal bvalid : std_logic;
    signal bready : std_logic := '0';
    signal araddr : std_logic_vector(8 downto 0) := (others => '0');
    signal arprot : std_logic_vector(2 downto 0) := (others => '0');
    signal arvalid : std_logic := '0';
    signal arready : std_logic;
    signal rdata : std_logic_vector(31 downto 0);
    signal rresp : std_logic_vector(1 downto 0);
    signal rvalid : std_logic;
    signal rready : std_logic := '0';

    signal fire_pulse : std_logic;
    signal start_tdc : std_logic;
    signal stop_tdc : std_logic;
    signal shot_start : std_logic;
    signal shot_face_index : std_logic_vector(2 downto 0);
    signal pd_p : std_logic_vector(15 downto 0) := (others => '0');
    signal pd_n : std_logic_vector(15 downto 0) := (others => '1');
    signal tdc_stop : std_logic_vector(15 downto 0);

    signal tdc_d : std_logic_vector(C_CHIPS * 28 - 1 downto 0) :=
        (others => 'Z');
    signal tdc_adr : std_logic_vector(C_CHIPS * 4 - 1 downto 0);
    signal tdc_csn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_rdn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_wrn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_oen : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_stopdis : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_alutrigger : std_logic_vector(C_CHIPS - 1 downto 0);
    signal tdc_puresn : std_logic_vector(C_CHIPS - 1 downto 0);
    signal ef1 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal ef2 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal lf1 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal lf2 : std_logic_vector(C_CHIPS - 1 downto 0);
    signal irflag : std_logic_vector(C_CHIPS - 1 downto 0) :=
        (others => '0');

    signal chip_d_out : std_logic_vector(C_CHIPS * 28 - 1 downto 0) :=
        (others => '0');
    signal chip_d_oe : std_logic_vector(C_CHIPS - 1 downto 0) :=
        (others => '0');
    signal fifo1_fill : natural_array_t := (others => 0);
    signal fifo2_fill : natural_array_t := (others => 0);
    signal fifo1_read_index : natural_array_t := (others => 0);
    signal fifo2_read_index : natural_array_t := (others => 0);
    signal read_count : natural_array_t := (others => 0);
    signal write_count : natural_array_t := (others => 0);
    signal fifo_load : std_logic := '0';

    signal rise_cfg_valid : std_logic;
    signal rise_cfg_ready : std_logic := '0';
    signal fall_cfg_valid : std_logic;
    signal fall_cfg_ready : std_logic := '0';
    signal previous_shot_r : std_logic := '0';
    signal shot_count : natural := 0;

begin

    csr_clk <= not csr_clk after C_CSR_PERIOD / 2 when not done;
    proc_clk <= not proc_clk after C_PROC_PERIOD / 2 when not done;
    tdc_clk <= not tdc_clk after C_TDC_PERIOD / 2 when not done;

    gen_chip_pins : for index in 0 to C_CHIPS - 1 generate
        constant C_DATA_LO : natural := index * 28;
        constant C_DATA_HI : natural := (index + 1) * 28 - 1;
    begin
        ef1(index) <= '1' when fifo1_fill(index) = 0 else '0';
        ef2(index) <= '1' when fifo2_fill(index) = 0 else '0';
        lf1(index) <= '1' when fifo1_fill(index) >= 2 else '0';
        lf2(index) <= '1' when fifo2_fill(index) >= 2 else '0';
        tdc_d(C_DATA_HI downto C_DATA_LO) <=
            chip_d_out(C_DATA_HI downto C_DATA_LO)
            when chip_d_oe(index) = '1' else (others => 'Z');
    end generate gen_chip_pins;

    u_dut : entity work.tdc_gpx_lidar_ctrl_v2_top
        generic map (
            G_CSR_CLK_MHZ => 100,
            G_PROC_CLK_MHZ => G_PROC_CLK_MHZ,
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_STREAM_CLK_MODE => "ASYNC",
            G_NUM_CHIPS => C_CHIPS,
            G_STOPS_PER_CHIP => C_STOPS_PER_CHIP,
            G_MAX_RETURNS_PER_STOP => C_RETURNS_PER_STOP,
            G_RISE_CAPABILITY_MASK => "0011",
            G_FALL_CAPABILITY_MASK => "0000",
            G_OUTPUT_WIDTH => 32,
            G_NUM_FACES => 1,
            G_ENABLE_ECHO_RECEIVER => true,
            G_ENABLE_ECHO_SIMULATION => true,
            G_PHASE_TIMEOUT_US => 50,
            G_DRAIN_MARGIN_TIME_NS => 6000
        )
        port map (
            s_axi_csr_aclk => csr_clk,
            s_axi_csr_aresetn => csr_rst_n,
            proc_aclk => proc_clk,
            proc_aresetn => proc_rst_n,
            i_tdc_clk => tdc_clk,
            i_tdc_aresetn => tdc_rst_n,
            s_axi_csr_awaddr => awaddr,
            s_axi_csr_awprot => awprot,
            s_axi_csr_awvalid => awvalid,
            s_axi_csr_awready => awready,
            s_axi_csr_wdata => wdata,
            s_axi_csr_wstrb => wstrb,
            s_axi_csr_wvalid => wvalid,
            s_axi_csr_wready => wready,
            s_axi_csr_bresp => bresp,
            s_axi_csr_bvalid => bvalid,
            s_axi_csr_bready => bready,
            s_axi_csr_araddr => araddr,
            s_axi_csr_arprot => arprot,
            s_axi_csr_arvalid => arvalid,
            s_axi_csr_arready => arready,
            s_axi_csr_rdata => rdata,
            s_axi_csr_rresp => rresp,
            s_axi_csr_rvalid => rvalid,
            s_axi_csr_rready => rready,
            o_irq => open,
            i_external_laser_permit => '0',
            i_enc_a => '0',
            i_enc_b => '0',
            i_enc_z => '0',
            i_fire_done => '0',
            o_fire_pulse => fire_pulse,
            o_start_tdc => start_tdc,
            o_stop_tdc => stop_tdc,
            o_shot_start => shot_start,
            o_shot_face_index => shot_face_index,
            o_n_faces => open,
            i_pd_lvds_p => pd_p,
            i_pd_lvds_n => pd_n,
            o_tdc_stop => tdc_stop,
            io_tdc_d => tdc_d,
            o_tdc_adr => tdc_adr,
            o_tdc_csn => tdc_csn,
            o_tdc_rdn => tdc_rdn,
            o_tdc_wrn => tdc_wrn,
            o_tdc_oen => tdc_oen,
            o_tdc_stopdis => tdc_stopdis,
            o_tdc_alutrigger => tdc_alutrigger,
            o_tdc_puresn => tdc_puresn,
            i_tdc_ef1 => ef1,
            i_tdc_ef2 => ef2,
            i_tdc_lf1 => lf1,
            i_tdc_lf2 => lf2,
            i_tdc_irflag => irflag,
            i_tdc_errflag => (others => '0'),
            m_axis_monitor_tdata => open,
            m_axis_monitor_tkeep => open,
            m_axis_monitor_tuser => open,
            m_axis_monitor_tvalid => open,
            m_axis_monitor_tlast => open,
            m_axis_monitor_tready => '1',
            m_axis_rise_tdata => open,
            m_axis_rise_tkeep => open,
            m_axis_rise_tstrb => open,
            m_axis_rise_tuser => open,
            m_axis_rise_tvalid => open,
            m_axis_rise_tlast => open,
            m_axis_rise_tready => '1',
            m_axis_fall_tdata => open,
            m_axis_fall_tkeep => open,
            m_axis_fall_tstrb => open,
            m_axis_fall_tuser => open,
            m_axis_fall_tvalid => open,
            m_axis_fall_tlast => open,
            m_axis_fall_tready => '1',
            o_vdma_rise_cfg_valid => rise_cfg_valid,
            i_vdma_rise_cfg_ready => rise_cfg_ready,
            o_vdma_rise_cfg_enable => open,
            o_vdma_rise_hsize_bytes => open,
            o_vdma_rise_vsize_lines => open,
            o_vdma_rise_stride_bytes => open,
            o_vdma_fall_cfg_valid => fall_cfg_valid,
            i_vdma_fall_cfg_ready => fall_cfg_ready,
            o_vdma_fall_cfg_enable => open,
            o_vdma_fall_hsize_bytes => open,
            o_vdma_fall_vsize_lines => open,
            o_vdma_fall_stride_bytes => open
        );

    p_chip_models : process (tdc_clk)
        variable rdn_previous : logic_array_t := (others => '1');
        variable wrn_previous : logic_array_t := (others => '1');
        variable address_v : std_logic_vector(3 downto 0);
        variable data_v : std_logic_vector(27 downto 0);
    begin
        if rising_edge(tdc_clk) then
            chip_d_oe <= (others => '0');
            if tdc_rst_n = '0' then
                fifo1_fill <= (others => 0);
                fifo2_fill <= (others => 0);
                fifo1_read_index <= (others => 0);
                fifo2_read_index <= (others => 0);
                read_count <= (others => 0);
                write_count <= (others => 0);
                rdn_previous := (others => '1');
                wrn_previous := (others => '1');
            else
                for index in 0 to C_CHIPS - 1 loop
                    address_v := tdc_adr(
                        (index + 1) * 4 - 1 downto index * 4);

                    if fifo_load = '1' then
                        fifo1_fill(index) <= C_WORDS_PER_IFIFO;
                        fifo2_fill(index) <= C_WORDS_PER_IFIFO;
                        fifo1_read_index(index) <= 0;
                        fifo2_read_index(index) <= 0;
                        read_count(index) <= 0;
                    end if;

                    if tdc_oen(index) = '0' and tdc_rdn(index) = '0' then
                        chip_d_oe(index) <= '1';
                        if address_v = c_TDC_REG8_IFIFO1 then
                            data_v := fn_raw_word(index, '0',
                                fifo1_read_index(index));
                        elsif address_v = c_TDC_REG9_IFIFO2 then
                            data_v := fn_raw_word(index, '1',
                                fifo2_read_index(index));
                        else
                            data_v := (others => '0');
                        end if;
                        chip_d_out((index + 1) * 28 - 1 downto index * 28)
                            <= data_v;
                    end if;

                    if tdc_rdn(index) = '1' and
                       rdn_previous(index) = '0' then
                        if address_v = c_TDC_REG8_IFIFO1 and
                           fifo1_fill(index) > 0 then
                            fifo1_fill(index) <= fifo1_fill(index) - 1;
                            fifo1_read_index(index) <=
                                fifo1_read_index(index) + 1;
                            read_count(index) <= read_count(index) + 1;
                        elsif address_v = c_TDC_REG9_IFIFO2 and
                              fifo2_fill(index) > 0 then
                            fifo2_fill(index) <= fifo2_fill(index) - 1;
                            fifo2_read_index(index) <=
                                fifo2_read_index(index) + 1;
                            read_count(index) <= read_count(index) + 1;
                        end if;
                    end if;

                    if tdc_wrn(index) = '0' and
                       wrn_previous(index) = '1' and
                       tdc_csn(index) = '0' then
                        write_count(index) <= write_count(index) + 1;
                    end if;
                    rdn_previous(index) := tdc_rdn(index);
                    wrn_previous(index) := tdc_wrn(index);
                end loop;
            end if;
        end if;
    end process p_chip_models;

    p_shot_observer : process (proc_clk)
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                previous_shot_r <= '0';
                shot_count <= 0;
            else
                if shot_start = '1' and previous_shot_r = '0' then
                    shot_count <= shot_count + 1;
                end if;
                previous_shot_r <= shot_start;
                assert fire_pulse = '0'
                    report "V2-K05-TOP simulation asserted physical fire"
                    severity failure;
            end if;
        end if;
    end process p_shot_observer;

    p_test : process
        variable status_word : std_logic_vector(31 downto 0);

        procedure wait_proc(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(proc_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_proc;

        procedure wait_tdc(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(tdc_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_tdc;

        procedure csr_tick is
        begin
            wait until rising_edge(csr_clk);
            wait for 1 ps;
        end procedure csr_tick;

        procedure axi_write(
            constant address : natural;
            constant value : std_logic_vector(31 downto 0)
        ) is
            variable aw_sent : boolean := false;
            variable w_sent : boolean := false;
        begin
            wait until falling_edge(csr_clk);
            awaddr <= std_logic_vector(to_unsigned(address, awaddr'length));
            awvalid <= '1';
            wdata <= value;
            wstrb <= "1111";
            wvalid <= '1';
            while not (aw_sent and w_sent) loop
                wait until rising_edge(csr_clk);
                if awvalid = '1' and awready = '1' then
                    awvalid <= '0';
                    aw_sent := true;
                end if;
                if wvalid = '1' and wready = '1' then
                    wvalid <= '0';
                    w_sent := true;
                end if;
            end loop;
            wstrb <= (others => '0');
            while bvalid = '0' loop
                csr_tick;
            end loop;
            assert bresp = "00"
                report "V2-K05-TOP AXI write response"
                severity failure;
            bready <= '1';
            csr_tick;
            bready <= '0';
        end procedure axi_write;

        procedure axi_read(
            constant address : natural;
            variable value : out std_logic_vector(31 downto 0)
        ) is
        begin
            wait until falling_edge(csr_clk);
            araddr <= std_logic_vector(to_unsigned(address, araddr'length));
            arvalid <= '1';
            loop
                wait until rising_edge(csr_clk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';
            while rvalid = '0' loop
                csr_tick;
            end loop;
            assert rresp = "00"
                report "V2-K05-TOP AXI read response"
                severity failure;
            value := rdata;
            rready <= '1';
            csr_tick;
            rready <= '0';
        end procedure axi_read;

        procedure command(constant bit_index : natural) is
            variable value : std_logic_vector(31 downto 0) :=
                (others => '0');
        begin
            value(bit_index) := '1';
            axi_write(fn_ctl_byte_offset(C_CTL_COMMAND), value);
        end procedure command;

        procedure wait_operation_bit(
            constant bit_index : natural;
            constant expected : std_logic;
            constant case_name : string
        ) is
        begin
            for cycle in 0 to 200 loop
                axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION),
                    status_word);
                exit when status_word(bit_index) = expected;
            end loop;
            assert status_word(bit_index) = expected
                report case_name severity failure;
        end procedure wait_operation_bit;
    begin
        assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
            report "V2-K05-TOP invalid build"
            severity failure;
        assert fn_validate_runtime_config(C_BUILD_CONFIG, C_RUNTIME) = CFG_OK
            report "V2-K05-TOP invalid runtime"
            severity failure;
        assert C_DERIVED.columns_per_face = 1
            report "V2-K05-TOP expected one planned Shot per Face"
            severity failure;

        wait for 50 ns;
        csr_rst_n <= '1';
        proc_rst_n <= '1';
        tdc_rst_n <= '1';
        wait_proc(8);

        for index in 1 to C_CTL_ECHO_DELAY_PROFILE loop
            axi_write(fn_ctl_byte_offset(index), C_WORDS(index));
        end loop;
        command(C_CMD_COMMIT_BIT);

        for cycle in 0 to 10000 loop
            wait_proc(1);
            exit when rise_cfg_valid = '1' and fall_cfg_valid = '1';
        end loop;
        assert rise_cfg_valid = '1' and fall_cfg_valid = '1'
            report "V2-K05-TOP VDMA profile request timeout"
            severity failure;
        rise_cfg_ready <= '1';
        fall_cfg_ready <= '1';
        wait_proc(1);
        rise_cfg_ready <= '0';
        fall_cfg_ready <= '0';

        for cycle in 0 to 2000 loop
            axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION), status_word);
            exit when status_word(C_TXN_BUSY_BIT) = '0' and
                status_word(C_TXN_DONE_STICKY_BIT) = '1';
        end loop;
        assert status_word(C_TXN_SUCCESS_STICKY_BIT) = '1' and
               status_word(C_TXN_ACTIVE_VALID_BIT) = '1'
            report "V2-K05-TOP deferred GPX configuration failed"
            severity failure;
        for index in 0 to C_CHIPS - 1 loop
            assert write_count(index) > 0
                report "V2-K05-TOP missing physical GPX configuration writes"
                severity failure;
        end loop;

        fifo_load <= '1';
        wait_tdc(1);
        fifo_load <= '0';
        wait_tdc(4);

        command(C_CMD_RUN_BIT);
        wait_operation_bit(C_OP_RUNNING_BIT, '1',
            "V2-K05-TOP RUN timeout");
        command(C_CMD_ARM_BIT);
        wait_operation_bit(C_OP_SIMULATION_ENABLE_BIT, '1',
            "V2-K05-TOP ARM timeout");

        for cycle in 0 to 10000 loop
            wait_proc(1);
            exit when shot_count >= 1 and start_tdc = '1';
        end loop;
        assert shot_count >= 1 and shot_face_index = "000"
            report "V2-K05-TOP first Shot missing"
            severity failure;

        for cycle in 0 to 10000 loop
            wait_proc(1);
            exit when stop_tdc = '1';
        end loop;
        assert stop_tdc = '1'
            report "V2-K05-TOP target-range STOP missing"
            severity failure;

        wait until falling_edge(tdc_clk);
        irflag <= (others => '1');
        wait_tdc(6);
        irflag <= (others => '0');

        for cycle in 0 to 500000 loop
            wait_proc(1);
            exit when read_count(0) = C_WORDS_PER_CHIP and
                read_count(1) = C_WORDS_PER_CHIP and
                fifo1_fill(0) = 0 and fifo2_fill(0) = 0 and
                fifo1_fill(1) = 0 and fifo2_fill(1) = 0;
        end loop;
        for index in 0 to C_CHIPS - 1 loop
            assert read_count(index) = C_WORDS_PER_CHIP and
                   fifo1_fill(index) = 0 and fifo2_fill(index) = 0
                report "V2-K05-TOP IFIFO drain count mismatch"
                severity failure;
        end loop;

        -- A second revolution can produce a Shot only after B5-B8 completes
        -- the first Shot and acknowledges the held Face-close event.
        for cycle in 0 to 500000 loop
            wait_proc(1);
            exit when shot_count >= 2;
        end loop;
        assert shot_count >= 2
            report "V2-K05-TOP B5-B8/Face-close chain did not reopen"
            severity failure;

        report "LIDAR_V2_TOP_K05_GPX_B5_B8_PASS proc_mhz=" &
            positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            positive'image(G_TDC_CLK_MHZ) severity note;
        done <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
