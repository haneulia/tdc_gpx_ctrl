library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_echo_pkg.all;

entity tb_lidar_echo_subsystem is
    generic (
        G_PROC_CLK_MHZ      : positive := 150;
        G_TDC_CLK_MHZ       : positive := 200;
        G_NUM_CHIPS         : positive := 2;
        G_ENABLE_RECEIVER   : boolean := true;
        G_ENABLE_SIMULATION : boolean := false
    );
end entity tb_lidar_echo_subsystem;

architecture sim of tb_lidar_echo_subsystem is

    constant C_CLK_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz            => G_PROC_CLK_MHZ,
        tdc_clk_mhz             => G_TDC_CLK_MHZ,
        stream_clock_mode       => STREAM_CLOCK_ASYNC,
        num_chips               => G_NUM_CHIPS,
        stops_per_chip          => 8,
        max_returns_per_stop    => 7,
        rise_capability_mask    => fn_present_chip_mask(G_NUM_CHIPS),
        fall_capability_mask    => (others => '0'),
        output_width            => 32,
        num_faces               => 5,
        enable_echo_receiver    => G_ENABLE_RECEIVER,
        enable_echo_simulation  => G_ENABLE_SIMULATION
    );
    constant C_NUM_CHANNELS : positive :=
        fn_echo_channel_count(C_BUILD_CONFIG);

    function fn_echo_active_config(
        version_value : natural;
        channel_0_ticks : natural;
        channel_step_ticks : natural
    ) return lidar_active_config_t is
        variable source_value : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        source_value.echo.channel_0_delay_5ns :=
            to_unsigned(channel_0_ticks, 16);
        source_value.echo.channel_step_5ns :=
            to_unsigned(channel_step_ticks, 16);
        result.version := to_unsigned(version_value, 16);
        result.source  := source_value;
        result.derived := fn_derive_runtime_config(
            C_BUILD_CONFIG,
            source_value);
        return result;
    end function fn_echo_active_config;

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal done  : boolean := false;

    signal active_valid : std_logic := '0';
    signal active_config : lidar_active_config_t :=
        fn_echo_active_config(1, 4, 4);
    signal shot_start  : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_result : shot_result_t := C_SHOT_RESULT_IDLE;

    signal pd_p : std_logic_vector(C_NUM_CHANNELS - 1 downto 0) :=
        (others => '0');
    signal pd_n : std_logic_vector(C_NUM_CHANNELS - 1 downto 0) :=
        (others => '1');
    signal tdc_stop : std_logic_vector(C_NUM_CHANNELS - 1 downto 0);
    signal simulation_active : std_logic;
    signal profile_ready : std_logic;
    signal profile_busy : std_logic;
    signal profile_version : u16_t;
    signal window_active : std_logic;
    signal idle : std_logic;
    signal diagnostics : echo_diagnostics_t;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not done;

    u_dut : entity work.lidar_echo_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_clear_diagnostics => '0',
            i_active_valid      => active_valid,
            i_active_config     => active_config,
            i_shot_start        => shot_start,
            i_shot_result       => shot_result,
            i_pd_lvds_p         => pd_p,
            i_pd_lvds_n         => pd_n,
            o_tdc_stop          => tdc_stop,
            o_simulation_active => simulation_active,
            o_profile_ready     => profile_ready,
            o_profile_busy      => profile_busy,
            o_profile_version   => profile_version,
            o_window_active     => window_active,
            o_idle              => idle,
            o_diagnostics       => diagnostics
        );

    p_stimulus : process
        variable request_value : shot_request_t;
        variable result_value  : shot_result_t;
        variable expected_stop : std_logic_vector(
            C_NUM_CHANNELS - 1 downto 0);
        variable expected_clock : natural;
        variable max_clock : natural;
        variable profile_cycles : natural;

        procedure wait_clocks(count : positive) is
        begin
            for iteration in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure start_shot(
            simulation_mode : std_logic;
            shot_number     : natural
        ) is
        begin
            request_value := C_SHOT_REQUEST_IDLE;
            request_value.valid := '1';
            request_value.source_sim := simulation_mode;
            request_value.face_index := to_unsigned(shot_number mod 5, 3);
            request_value.shot_index := to_unsigned(shot_number, 16);
            request_value.active_version := active_config.version;
            shot_start <= C_SHOT_START_EVENT_IDLE;
            shot_start.valid <= '1';
            shot_start.request <= request_value;
            wait until rising_edge(clk);
            shot_start <= C_SHOT_START_EVENT_IDLE;
            wait for 1 ps;
        end procedure start_shot;

        procedure finish_shot is
        begin
            result_value := C_SHOT_RESULT_IDLE;
            result_value.valid := '1';
            result_value.request := request_value;
            shot_result <= result_value;
            wait until rising_edge(clk);
            shot_result <= C_SHOT_RESULT_IDLE;
            wait until rising_edge(clk);
            wait for 1 ps;
        end procedure finish_shot;

        procedure pulse_all_channels is
        begin
            pd_p <= (others => '1');
            pd_n <= (others => '0');
            wait_clocks(3);
            pd_p <= (others => '0');
            pd_n <= (others => '1');
            wait_clocks(4);
        end procedure pulse_all_channels;

        procedure check_simulated_profile(
            channel_0_ticks : natural;
            channel_step_ticks : natural;
            shot_number : natural
        ) is
            variable channel_ticks : natural;
        begin
            start_shot('1', shot_number);
            assert simulation_active = '1'
                report "Echo simulation source did not latch"
                severity failure;

            channel_ticks := channel_0_ticks +
                (C_NUM_CHANNELS - 1) * channel_step_ticks;
            max_clock := to_integer(fn_echo_ticks_to_proc_clocks(
                to_unsigned(channel_ticks, C_ECHO_DELAY_WIDTH),
                G_PROC_CLK_MHZ));

            for elapsed_clock in 1 to max_clock loop
                wait until rising_edge(clk);
                wait for 1 ps;
                expected_stop := (others => '0');
                for channel in 0 to C_NUM_CHANNELS - 1 loop
                    channel_ticks := channel_0_ticks +
                        channel * channel_step_ticks;
                    expected_clock := to_integer(
                        fn_echo_ticks_to_proc_clocks(
                            to_unsigned(channel_ticks,
                                C_ECHO_DELAY_WIDTH),
                            G_PROC_CLK_MHZ));
                    if expected_clock = elapsed_clock and
                       channel_ticks /= 0 then
                        expected_stop(channel) := '1';
                    end if;
                end loop;
                assert tdc_stop = expected_stop
                    report "synthetic Echo CH0/STEP delay mismatch"
                    severity failure;
            end loop;
            wait_clocks(2);
            finish_shot;
            assert diagnostics.snapshot.valid = '1'
                report "simulation Echo snapshot missing"
                severity failure;
            assert diagnostics.snapshot.total_rise =
                to_unsigned(C_NUM_CHANNELS, 16) and
                diagnostics.snapshot.total_fall = 0
                report "simulation Echo count mismatch"
                severity failure;
        end procedure check_simulated_profile;

    begin
        assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
            report "Echo TB build profile is invalid"
            severity failure;

        wait_clocks(5);
        rst_n <= '1';
        active_valid <= '1';
        wait_clocks(5);

        if not G_ENABLE_RECEIVER then
            pd_p <= (others => '1');
            pd_n <= (others => '0');
            wait for 1 ns;
            assert tdc_stop = (tdc_stop'range => '0')
                report "disabled Echo receiver passed an LVDS input"
                severity failure;
            assert simulation_active = '0' and window_active = '0' and
                   profile_ready = '1' and profile_busy = '0' and idle = '1'
                report "disabled Echo receiver did not remain inert"
                severity failure;
            assert diagnostics = C_ECHO_DIAGNOSTICS_CLEAR
                report "disabled Echo receiver generated diagnostics"
                severity failure;
            report "V2-ECHO-DISABLED PASS channels=" &
                integer'image(C_NUM_CHANNELS)
                severity note;
        else
            -- Every flat channel has one independent physical STOP output.
            -- This 1.5 ns test is independent of the sampled observer.
            for channel in 0 to C_NUM_CHANNELS - 1 loop
                pd_p(channel) <= '1';
                pd_n(channel) <= '0';
                wait for 100 ps;
                expected_stop := (others => '0');
                expected_stop(channel) := '1';
                assert tdc_stop = expected_stop
                    report "B4 physical channel order or assertion failed"
                    severity failure;
                wait for 1400 ps;
                pd_p(channel) <= '0';
                pd_n(channel) <= '1';
                wait for 100 ps;
                assert tdc_stop = (tdc_stop'range => '0')
                    report "B4 physical STOP deassertion failed"
                    severity failure;
            end loop;
            wait_clocks(5);

            if not G_ENABLE_SIMULATION then
                assert profile_ready = '1' and profile_busy = '0'
                    report "production Echo exposed a profile dependency"
                    severity failure;

                -- Return 1..7 are independent pulses per channel in seven
                -- consecutive Shots. These counts remain pre-GPX diagnostics.
                for return_count in 1 to 7 loop
                    start_shot('0', return_count);
                    assert window_active = '1'
                        report "physical Echo window did not open"
                        severity failure;
                    for return_index in 1 to return_count loop
                        pulse_all_channels;
                    end loop;
                    finish_shot;
                    assert diagnostics.snapshot.valid = '1'
                        report "physical Echo snapshot missing"
                        severity failure;
                    assert diagnostics.snapshot.request = request_value
                        report "physical Echo Shot identity changed"
                        severity failure;
                    assert diagnostics.snapshot.total_rise =
                        to_unsigned(return_count * C_NUM_CHANNELS, 16)
                        report "physical Return rise total mismatch"
                        severity failure;
                    assert diagnostics.snapshot.total_fall =
                        to_unsigned(return_count * C_NUM_CHANNELS, 16)
                        report "physical Return fall total mismatch"
                        severity failure;
                    for channel in 0 to C_NUM_CHANNELS - 1 loop
                        assert diagnostics.snapshot.rise_count(channel) =
                            to_unsigned(return_count, C_ECHO_COUNT_WIDTH)
                            report "physical per-channel Rise count mismatch"
                            severity failure;
                        assert diagnostics.snapshot.fall_count(channel) =
                            to_unsigned(return_count, C_ECHO_COUNT_WIDTH)
                            report "physical per-channel Fall count mismatch"
                            severity failure;
                    end loop;
                    wait_clocks(1);
                end loop;

                report "V2-ECHO-PHYSICAL PASS channels=" &
                    integer'image(C_NUM_CHANNELS) & " proc_mhz=" &
                    integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
                    integer'image(G_TDC_CLK_MHZ)
                    severity note;
            else
                profile_cycles := 0;
                while profile_ready /= '1' loop
                    wait_clocks(1);
                    profile_cycles := profile_cycles + 1;
                    assert profile_cycles <= C_NUM_CHANNELS + 2
                        report "initial Echo profile expansion timed out"
                        severity failure;
                end loop;
                assert profile_version = active_config.version
                    report "initial Echo profile version mismatch"
                    severity failure;

                check_simulated_profile(4, 4, 101);

                -- One atomic config change rebuilds all channels with no
                -- indexed CSR writes or table-specific CDC transaction.
                active_config <= fn_echo_active_config(2, 7, 3);
                wait_clocks(1);
                assert profile_ready = '0' and profile_busy = '1'
                    report "Echo profile did not detect active config change"
                    severity failure;
                profile_cycles := 0;
                while profile_ready /= '1' loop
                    wait_clocks(1);
                    profile_cycles := profile_cycles + 1;
                    assert profile_cycles <= C_NUM_CHANNELS + 1
                        report "updated Echo profile expansion timed out"
                        severity failure;
                end loop;
                assert profile_version = to_unsigned(2, 16)
                    report "updated Echo profile version mismatch"
                    severity failure;
                check_simulated_profile(7, 3, 102);

                -- A Shot cannot consume a partially rebuilt profile. The
                -- event is visible in diagnostics instead of creating STOPs.
                active_config <= fn_echo_active_config(3, 9, 2);
                start_shot('1', 103);
                wait_clocks(2);
                assert tdc_stop = (tdc_stop'range => '0')
                    report "not-ready Echo profile generated STOP"
                    severity failure;
                assert diagnostics.profile_not_ready_sticky = '1' and
                       diagnostics.profile_not_ready_count = 1
                    report "not-ready Echo profile was not diagnosed"
                    severity failure;
                finish_shot;

                while profile_ready /= '1' loop
                    wait_clocks(1);
                end loop;

                report "V2-ECHO-SIMULATION PASS channels=" &
                    integer'image(C_NUM_CHANNELS) & " proc_mhz=" &
                    integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
                    integer'image(G_TDC_CLK_MHZ)
                    severity note;
            end if;
        end if;

        report "LIDAR_V2_ECHO_SUBSYSTEM_PASS" severity note;
        done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_echo_physical_16_150_200 is
end entity;

architecture sim of tb_lidar_echo_physical_16_150_200 is
begin
    u_test : entity work.tb_lidar_echo_subsystem
        generic map (
            G_PROC_CLK_MHZ      => 150,
            G_TDC_CLK_MHZ       => 200,
            G_NUM_CHIPS         => 2,
            G_ENABLE_RECEIVER   => true,
            G_ENABLE_SIMULATION => false
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_echo_physical_32_200_150 is
end entity;

architecture sim of tb_lidar_echo_physical_32_200_150 is
begin
    u_test : entity work.tb_lidar_echo_subsystem
        generic map (
            G_PROC_CLK_MHZ      => 200,
            G_TDC_CLK_MHZ       => 150,
            G_NUM_CHIPS         => 4,
            G_ENABLE_RECEIVER   => true,
            G_ENABLE_SIMULATION => false
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_echo_sim_16_150_200 is
end entity;

architecture sim of tb_lidar_echo_sim_16_150_200 is
begin
    u_test : entity work.tb_lidar_echo_subsystem
        generic map (
            G_PROC_CLK_MHZ      => 150,
            G_TDC_CLK_MHZ       => 200,
            G_NUM_CHIPS         => 2,
            G_ENABLE_RECEIVER   => true,
            G_ENABLE_SIMULATION => true
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_echo_sim_16_200_150 is
end entity;

architecture sim of tb_lidar_echo_sim_16_200_150 is
begin
    u_test : entity work.tb_lidar_echo_subsystem
        generic map (
            G_PROC_CLK_MHZ      => 200,
            G_TDC_CLK_MHZ       => 150,
            G_NUM_CHIPS         => 2,
            G_ENABLE_RECEIVER   => true,
            G_ENABLE_SIMULATION => true
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_echo_disabled_16_150_200 is
end entity;

architecture sim of tb_lidar_echo_disabled_16_150_200 is
begin
    u_test : entity work.tb_lidar_echo_subsystem
        generic map (
            G_PROC_CLK_MHZ      => 150,
            G_TDC_CLK_MHZ       => 200,
            G_NUM_CHIPS         => 2,
            G_ENABLE_RECEIVER   => false,
            G_ENABLE_SIMULATION => false
        );
end architecture;
