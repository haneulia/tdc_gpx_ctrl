-- ============================================================================
-- 테스트 자산 목적: v2 Top의 Processing/Echo 통합과 physical/simulation 모드를 검증한다.
-- 핵심 검증 계약: K0-4 source 상호배제, fire/start/STOP 경로와 profile barrier이다.
-- 관련 RTL: tdc_gpx_lidar_ctrl_v2_top, Processing subsystem, Echo subsystem.
-- 실행 회귀: scripts/run_v2_k04_integration.ps1
-- 유지보수 주의: simulation mode가 physical fire 또는 LVDS STOP을 구동하지 않게 유지한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_csr_map_pkg.all;

entity tb_tdc_gpx_lidar_ctrl_v2_k04 is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200;
        G_SIMULATION_MODE : boolean := false
    );
end entity tb_tdc_gpx_lidar_ctrl_v2_k04;

architecture sim of tb_tdc_gpx_lidar_ctrl_v2_k04 is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.num_chips := 2;
        result.stops_per_chip := 8;
        result.max_returns_per_stop := 7;
        result.rise_capability_mask := "0011";
        result.fall_capability_mask := "0000";
        result.output_width := 32;
        result.num_faces := 1;
        result.enable_echo_receiver := true;
        result.enable_echo_simulation := G_SIMULATION_MODE;
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
        result.motor.virtual_ticks_lo := to_unsigned(2, 32);
        result.motor.virtual_hi_count := (others => '0');
        result.motor.z_offset := (others => '0');
        result.motor.z_width := (others => '0');
        if G_SIMULATION_MODE then
            result.motor.simulation_mode := '1';
        else
            result.motor.simulation_mode := '0';
        end if;
        result.mirror.face_centers := (others => (others => '0'));
        result.mirror.face_centers(0) :=
            to_unsigned(7, C_POSITION_WIDTH);
        result.mirror.common_half_width :=
            to_unsigned(3, C_POSITION_WIDTH);
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

    constant C_RUNTIME : lidar_runtime_config_t := fn_runtime_config;
    constant C_DERIVED : lidar_derived_config_t :=
        fn_derive_runtime_config(C_BUILD_CONFIG, C_RUNTIME);
    constant C_WORDS : csr_word_array_t :=
        fn_pack_runtime_config(C_RUNTIME);
    constant C_CSR_PERIOD : time := 10 ns;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;

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

    signal external_permit : std_logic := '0';
    signal enc_a : std_logic := '0';
    signal enc_b : std_logic := '0';
    signal enc_z : std_logic := '0';
    signal fire_done : std_logic := '0';
    signal fire_pulse : std_logic;
    signal start_tdc : std_logic;
    signal stop_tdc : std_logic;
    signal shot_start : std_logic;
    signal shot_face_index : std_logic_vector(2 downto 0);

    signal pd_p : std_logic_vector(15 downto 0) := (others => '0');
    signal pd_n : std_logic_vector(15 downto 0) := (others => '1');
    signal tdc_stop : std_logic_vector(15 downto 0);
    signal tdc_d : std_logic_vector(55 downto 0) := (others => 'Z');

    signal rise_cfg_valid : std_logic;
    signal rise_cfg_ready : std_logic := '0';
    signal fall_cfg_valid : std_logic;
    signal fall_cfg_ready : std_logic := '0';

    signal previous_fire_r : std_logic := '0';
    signal previous_start_r : std_logic := '0';
    signal previous_stop_r : std_logic := '0';
    signal previous_shot_r : std_logic := '0';
    signal fire_rise_count : natural := 0;
    signal start_rise_count : natural := 0;
    signal stop_rise_count : natural := 0;
    signal shot_rise_count : natural := 0;
    signal echo_stop_seen : std_logic := '0';

begin

    csr_clk <= not csr_clk after C_CSR_PERIOD / 2 when not done;
    proc_clk <= not proc_clk after C_PROC_PERIOD / 2 when not done;
    tdc_clk <= not tdc_clk after C_TDC_PERIOD / 2 when not done;

    u_dut : entity work.tdc_gpx_lidar_ctrl_v2_top
        generic map (
            G_CSR_CLK_MHZ => 100,
            G_PROC_CLK_MHZ => G_PROC_CLK_MHZ,
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_STREAM_CLK_MODE => "ASYNC",
            G_NUM_CHIPS => 2,
            G_STOPS_PER_CHIP => 8,
            G_MAX_RETURNS_PER_STOP => 7,
            G_RISE_CAPABILITY_MASK => "0011",
            G_FALL_CAPABILITY_MASK => "0000",
            G_OUTPUT_WIDTH => 32,
            G_NUM_FACES => 1,
            G_ENABLE_ECHO_RECEIVER => true,
            G_ENABLE_ECHO_SIMULATION => G_SIMULATION_MODE,
            G_PHASE_TIMEOUT_US => 20
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
            i_external_laser_permit => external_permit,
            i_enc_a => enc_a,
            i_enc_b => enc_b,
            i_enc_z => enc_z,
            i_fire_done => fire_done,
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
            o_tdc_adr => open,
            o_tdc_csn => open,
            o_tdc_rdn => open,
            o_tdc_wrn => open,
            o_tdc_oen => open,
            o_tdc_stopdis => open,
            o_tdc_alutrigger => open,
            o_tdc_puresn => open,
            i_tdc_ef1 => (others => '1'),
            i_tdc_ef2 => (others => '1'),
            i_tdc_lf1 => (others => '0'),
            i_tdc_lf2 => (others => '0'),
            i_tdc_irflag => (others => '0'),
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

    p_observe : process (proc_clk)
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                previous_fire_r <= '0';
                previous_start_r <= '0';
                previous_stop_r <= '0';
                previous_shot_r <= '0';
                fire_rise_count <= 0;
                start_rise_count <= 0;
                stop_rise_count <= 0;
                shot_rise_count <= 0;
                echo_stop_seen <= '0';
            else
                if fire_pulse = '1' and previous_fire_r = '0' then
                    fire_rise_count <= fire_rise_count + 1;
                end if;
                if start_tdc = '1' and previous_start_r = '0' then
                    start_rise_count <= start_rise_count + 1;
                end if;
                if stop_tdc = '1' and previous_stop_r = '0' then
                    stop_rise_count <= stop_rise_count + 1;
                end if;
                if shot_start = '1' and previous_shot_r = '0' then
                    shot_rise_count <= shot_rise_count + 1;
                end if;
                if tdc_stop /= (tdc_stop'range => '0') then
                    echo_stop_seen <= '1';
                end if;
                previous_fire_r <= fire_pulse;
                previous_start_r <= start_tdc;
                previous_stop_r <= stop_tdc;
                previous_shot_r <= shot_start;

                if G_SIMULATION_MODE then
                    assert fire_pulse = '0'
                        report "V2-K04-TOP simulation asserted fire_pulse"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_observe;

    p_test : process
        variable status_word : std_logic_vector(31 downto 0);
        variable phase_v : natural := 0;
        variable start_before_v : natural;

        procedure wait_proc(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(proc_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_proc;

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
                report "V2-K04-TOP AXI write response"
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
                report "V2-K04-TOP AXI read response"
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

        procedure wait_profile_request is
        begin
            for cycle in 0 to 4000 loop
                wait_proc(1);
                exit when rise_cfg_valid = '1' and fall_cfg_valid = '1';
            end loop;
            assert rise_cfg_valid = '1' and fall_cfg_valid = '1'
                report "V2-K04-TOP VDMA profile request timeout"
                severity failure;
        end procedure wait_profile_request;

        procedure wait_commit_done is
        begin
            for cycle in 0 to 600 loop
                axi_read(fn_stat_byte_offset(C_STAT_TRANSACTION),
                    status_word);
                exit when status_word(C_TXN_BUSY_BIT) = '0' and
                    status_word(C_TXN_DONE_STICKY_BIT) = '1';
            end loop;
            assert status_word(C_TXN_BUSY_BIT) = '0' and
                   status_word(C_TXN_SUCCESS_STICKY_BIT) = '1' and
                   status_word(C_TXN_ACTIVE_VALID_BIT) = '1'
                report "V2-K04-TOP commit completion timeout"
                severity failure;
        end procedure wait_commit_done;

        procedure wait_operation_bit(
            constant bit_index : natural;
            constant expected : std_logic;
            constant case_name : string
        ) is
        begin
            for cycle in 0 to 80 loop
                axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION),
                    status_word);
                exit when status_word(bit_index) = expected;
            end loop;
            assert status_word(bit_index) = expected
                report case_name severity failure;
        end procedure wait_operation_bit;

        procedure drive_cw_step is
        begin
            phase_v := (phase_v + 1) mod 4;
            wait until falling_edge(proc_clk);
            case phase_v is
                when 1      => enc_a <= '1'; enc_b <= '0';
                when 2      => enc_a <= '1'; enc_b <= '1';
                when 3      => enc_a <= '0'; enc_b <= '1';
                when others => enc_a <= '0'; enc_b <= '0';
            end case;
            wait_proc(1);
        end procedure drive_cw_step;
    begin
        assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
            report "V2-K04-TOP invalid build"
            severity failure;
        assert fn_validate_runtime_config(C_BUILD_CONFIG, C_RUNTIME) = CFG_OK
            report "V2-K04-TOP invalid runtime"
            severity failure;
        assert C_DERIVED.face_lower(0) = 4 and
               C_DERIVED.face_upper(0) = 10 and
               C_DERIVED.shot_interval_states = 2 and
               C_DERIVED.columns_per_face = 3
            report "V2-K04-TOP geometry mismatch"
            severity failure;

        wait for 50 ns;
        csr_rst_n <= '1';
        proc_rst_n <= '1';
        tdc_rst_n <= '1';
        wait_proc(8);

        -- Physical LVDS remains a direct, channel-preserving STOP path.
        pd_p(5) <= '1';
        pd_n(5) <= '0';
        wait for 100 ps;
        assert tdc_stop = x"0020"
            report "V2-K04-TOP physical Echo channel mapping mismatch"
            severity failure;
        wait for 1400 ps;
        pd_p(5) <= '0';
        pd_n(5) <= '1';
        wait for 100 ps;
        assert tdc_stop = x"0000"
            report "V2-K04-TOP physical Echo did not deassert"
            severity failure;

        for index in 1 to C_CTL_ECHO_DELAY_PROFILE loop
            axi_write(fn_ctl_byte_offset(index), C_WORDS(index));
        end loop;
        command(C_CMD_COMMIT_BIT);
        wait_profile_request;

        rise_cfg_ready <= '1';
        fall_cfg_ready <= '1';
        wait_proc(1);
        rise_cfg_ready <= '0';
        fall_cfg_ready <= '0';
        wait_commit_done;

        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), status_word);
        assert unsigned(status_word(15 downto 0)) = 1 and
               status_word(C_OP_CONFIG_READY_BIT) = '1'
            report "V2-K04-TOP Active Config release mismatch"
            severity failure;

        -- A raw fire_done outside an armed physical request must be inert.
        start_before_v := start_rise_count;
        fire_done <= '1';
        wait for 2 ns;
        fire_done <= '0';
        wait_proc(5);
        assert start_rise_count = start_before_v
            report "V2-K04-TOP unarmed fire_done generated START"
            severity failure;

        if G_SIMULATION_MODE then
            command(C_CMD_RUN_BIT);
            wait_operation_bit(C_OP_RUNNING_BIT, '1',
                "V2-K04-TOP simulation RUN timeout");
            command(C_CMD_ARM_BIT);
            wait_operation_bit(C_OP_SIMULATION_ENABLE_BIT, '1',
                "V2-K04-TOP simulation ARM timeout");

            for cycle in 0 to 240 loop
                wait_proc(1);
                exit when shot_rise_count >= 1 and
                    start_rise_count >= 1 and echo_stop_seen = '1';
            end loop;
            assert shot_rise_count >= 1 and start_rise_count >= 1 and
                   echo_stop_seen = '1' and fire_rise_count = 0 and
                   shot_face_index = "000"
                report "V2-K04-TOP simulation Shot/Echo chain mismatch"
                severity failure;

            start_before_v := start_rise_count;
            fire_done <= '1';
            wait for 2 ns;
            fire_done <= '0';
            wait_proc(5);
            assert start_rise_count = start_before_v
                report "V2-K04-TOP simulation consumed physical fire_done"
                severity failure;
        else
            external_permit <= '1';
            wait_proc(5);
            command(C_CMD_RUN_BIT);
            wait_operation_bit(C_OP_RUNNING_BIT, '1',
                "V2-K04-TOP physical RUN timeout");
            command(C_CMD_ARM_BIT);
            wait_operation_bit(C_OP_PHYSICAL_FIRE_ENABLE_BIT, '1',
                "V2-K04-TOP physical ARM timeout");

            phase_v := 0;
            for step_index in 1 to 4 loop
                drive_cw_step;
            end loop;
            for cycle in 0 to 30 loop
                wait_proc(1);
                exit when fire_pulse = '1';
            end loop;
            assert fire_pulse = '1' and start_rise_count = 0 and
                   shot_rise_count = 0
                report "V2-K04-TOP physical fire ordering mismatch"
                severity failure;

            fire_done <= '1';
            wait for 100 ps;
            assert start_tdc = '1'
                report "V2-K04-TOP raw fire_done low-latency START missing"
                severity failure;
            wait for 1900 ps;
            fire_done <= '0';

            for cycle in 0 to 30 loop
                wait_proc(1);
                exit when shot_rise_count >= 1;
            end loop;
            assert fire_rise_count = 1 and shot_rise_count = 1 and
                   start_rise_count = 1 and shot_face_index = "000"
                report "V2-K04-TOP physical Shot identity mismatch"
                severity failure;

            pd_p(3) <= '1';
            pd_n(3) <= '0';
            wait for 1500 ps;
            pd_p(3) <= '0';
            pd_n(3) <= '1';

            for cycle in 0 to 100 loop
                wait_proc(1);
                exit when stop_rise_count >= 1;
            end loop;
            assert stop_rise_count = 1
                report "V2-K04-TOP physical target-range STOP missing"
                severity failure;
            wait_proc(8);
        end if;

        command(C_CMD_SOFT_RESET_BIT);
        wait_operation_bit(C_OP_RUNNING_BIT, '0',
            "V2-K04-TOP soft reset did not clear RUNNING");
        wait_operation_bit(C_OP_ARMED_BIT, '0',
            "V2-K04-TOP soft reset did not clear ARMED");
        axi_read(fn_stat_byte_offset(C_STAT_ACTIVE_VERSION), status_word);
        assert unsigned(status_word(15 downto 0)) = 1 and
               status_word(C_OP_CONFIG_READY_BIT) = '1'
            report "V2-K04-TOP soft reset destroyed Active Config"
            severity failure;

        if G_SIMULATION_MODE then
            report "LIDAR_V2_TOP_K04_INTEGRATION_PASS mode=simulation proc_mhz=" &
                positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
                positive'image(G_TDC_CLK_MHZ) severity note;
        else
            report "LIDAR_V2_TOP_K04_INTEGRATION_PASS mode=physical proc_mhz=" &
                positive'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
                positive'image(G_TDC_CLK_MHZ) severity note;
        end if;
        done <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
