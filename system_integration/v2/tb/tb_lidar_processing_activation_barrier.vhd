library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_processing_activation_barrier is
end entity tb_lidar_processing_activation_barrier;

architecture sim of tb_lidar_processing_activation_barrier is

    constant C_CLK_PERIOD : time := 5 ns;
    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz             => 200,
        tdc_clk_mhz              => 150,
        stream_clock_mode        => STREAM_CLOCK_ASYNC,
        num_chips                => 2,
        stops_per_chip           => 8,
        max_returns_per_stop     => 7,
        rise_capability_mask     => "0011",
        fall_capability_mask     => "0000",
        output_width             => 32,
        num_faces                => 1,
        enable_echo_receiver     => true,
        enable_echo_simulation   => true
    );

    function fn_active_config(
        version_value : positive
    ) return lidar_active_config_t is
        variable source_v : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        source_v.motor.cpr := to_unsigned(8, 16);
        source_v.motor.decode_mode := DECODE_X4;
        source_v.motor.virtual_ticks_lo := to_unsigned(2, 32);
        source_v.motor.virtual_hi_count := (others => '0');
        source_v.mirror.face_centers := (others => (others => '0'));
        source_v.mirror.face_centers(0) :=
            to_unsigned(7, C_POSITION_WIDTH);
        source_v.mirror.common_half_width :=
            to_unsigned(3, C_POSITION_WIDTH);
        source_v.laser.face_enable_mask := "00001";
        source_v.laser.optical_shot_interval_udeg :=
            to_unsigned(45_000_000, angle_udeg_t'length);
        source_v.tdc.active_chip_mask := "0011";
        source_v.tdc.falling_enable := '0';

        result.version := to_unsigned(version_value, 16);
        result.source := source_v;
        result.derived := fn_derive_runtime_config(
            C_BUILD_CONFIG, source_v);
        return result;
    end function fn_active_config;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal done : boolean := false;
    signal abort_request : std_logic := '0';
    signal activate_start : std_logic := '0';
    signal active_valid : std_logic := '1';
    signal active_config : lidar_active_config_t := fn_active_config(99);
    signal echo_ready : std_logic := '0';
    signal echo_busy : std_logic := '1';
    signal echo_version : u16_t := (others => '0');
    signal rise_valid : std_logic;
    signal rise_ready : std_logic := '0';
    signal fall_valid : std_logic;
    signal fall_ready : std_logic := '0';
    signal complete : std_logic;
    signal fault : std_logic;
    signal busy : std_logic;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not done;

    u_dut : entity work.lidar_processing_activation_barrier
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_request,
            i_activate_start => activate_start,
            i_active_valid => active_valid,
            i_active_config => active_config,
            i_datapath_idle => '1',
            i_echo_profile_ready => echo_ready,
            i_echo_profile_busy => echo_busy,
            i_echo_profile_version => echo_version,
            o_rise_cfg_valid => rise_valid,
            i_rise_cfg_ready => rise_ready,
            o_rise_cfg_enable => open,
            o_rise_hsize_bytes => open,
            o_rise_vsize_lines => open,
            o_rise_stride_bytes => open,
            o_fall_cfg_valid => fall_valid,
            i_fall_cfg_ready => fall_ready,
            o_fall_cfg_enable => open,
            o_fall_hsize_bytes => open,
            o_fall_vsize_lines => open,
            o_fall_stride_bytes => open,
            o_rise_active_profile => open,
            o_fall_active_profile => open,
            o_activate_complete => complete,
            o_activate_fault => fault,
            o_busy => busy
        );

    p_test : process
        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
                wait for 1 ps;
            end loop;
        end procedure wait_clocks;

        procedure pulse_activate is
        begin
            wait until falling_edge(clk);
            activate_start <= '1';
            wait until falling_edge(clk);
            activate_start <= '0';
        end procedure pulse_activate;

        procedure wait_vdma_request is
        begin
            for cycle in 0 to 100 loop
                wait_clocks(1);
                exit when rise_valid = '1' and fall_valid = '1';
            end loop;
            assert rise_valid = '1' and fall_valid = '1'
                report "V2-K04-BARRIER VDMA request timeout"
                severity failure;
        end procedure wait_vdma_request;
    begin
        assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
            report "V2-K04-BARRIER invalid build"
            severity failure;

        wait_clocks(4);
        rst_n <= '1';
        wait_clocks(3);

        pulse_activate;
        wait_vdma_request;
        rise_ready <= '1';
        fall_ready <= '1';
        wait_clocks(1);
        rise_ready <= '0';
        fall_ready <= '0';
        wait_clocks(4);

        assert busy = '1' and complete = '0' and fault = '0'
            report "V2-K04-BARRIER released before Echo readiness"
            severity failure;

        echo_version <= to_unsigned(98, 16);
        echo_ready <= '1';
        echo_busy <= '0';
        wait_clocks(3);
        assert busy = '1' and complete = '0'
            report "V2-K04-BARRIER accepted mismatched Echo version"
            severity failure;

        echo_version <= to_unsigned(99, 16);
        echo_busy <= '1';
        wait_clocks(3);
        assert busy = '1' and complete = '0'
            report "V2-K04-BARRIER accepted busy Echo profile"
            severity failure;

        echo_busy <= '0';
        wait_clocks(1);
        assert complete = '1' and busy = '0' and fault = '0'
            report "V2-K04-BARRIER matching dependencies did not release"
            severity failure;
        wait_clocks(1);
        assert complete = '0'
            report "V2-K04-BARRIER completion was not a pulse"
            severity failure;

        active_config <= fn_active_config(100);
        echo_ready <= '0';
        echo_busy <= '1';
        pulse_activate;
        wait_clocks(1);
        pulse_activate;
        wait for 1 ps;
        assert fault = '1' and busy = '0'
            report "V2-K04-BARRIER overlap did not abort and fault"
            severity failure;

        report "LIDAR_V2_PROCESSING_ACTIVATION_BARRIER_PASS"
            severity note;
        done <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
