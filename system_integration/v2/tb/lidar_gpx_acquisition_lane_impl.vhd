library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

-- OOC implementation harness. Runtime controls and physical pins remain live
-- so synthesis cannot optimize the acquisition lane into a reset-only shell.
entity lidar_gpx_acquisition_lane_impl is
    generic (
        G_TDC_CLK_MHZ : positive := 200
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_run_enable   : in  std_logic;
        i_config_apply : in  std_logic;
        i_shot_valid   : in  std_logic;
        i_stop_tdc     : in  std_logic;
        i_event_ready  : in  std_logic;
        i_d            : in  gpx_bus_data_t;
        i_ef1          : in  std_logic;
        i_ef2          : in  std_logic;
        i_lf1          : in  std_logic;
        i_lf2          : in  std_logic;
        i_irflag       : in  std_logic;
        i_errflag      : in  std_logic;
        o_adr          : out gpx_bus_address_t;
        o_csn          : out std_logic;
        o_rdn          : out std_logic;
        o_wrn          : out std_logic;
        o_oen          : out std_logic;
        o_d            : out gpx_bus_data_t;
        o_d_tri        : out std_logic;
        o_stopdis      : out std_logic;
        o_alutrigger   : out std_logic;
        o_puresn       : out std_logic;
        o_shot_ready   : out std_logic;
        o_event_valid  : out std_logic;
        o_event_kind   : out std_logic_vector(1 downto 0);
        o_event_word   : out gpx_bus_data_t;
        o_fault_any    : out std_logic
    );
end entity lidar_gpx_acquisition_lane_impl;

architecture rtl of lidar_gpx_acquisition_lane_impl is

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz          => 150,
        tdc_clk_mhz           => G_TDC_CLK_MHZ,
        stream_clock_mode     => STREAM_CLOCK_ASYNC,
        num_chips             => 1,
        stops_per_chip        => 8,
        max_returns_per_stop  => 7,
        rise_capability_mask  => "0001",
        fall_capability_mask  => "0000",
        output_width          => 32,
        num_faces             => 5,
        enable_echo_receiver  => false,
        enable_echo_simulation => false
    );

    function fn_active return lidar_active_config_t is
        variable runtime : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        result.version := to_unsigned(1, 16);
        result.source := runtime;
        result.derived := fn_derive_runtime_config(C_BUILD_CONFIG, runtime);
        return result;
    end function fn_active;

    function fn_image return gpx_register_image_t is
        variable result : gpx_register_image_t := (others => (others => '0'));
    begin
        for index in result'range loop
            result(index) := c_GPX_DEFAULT_IMAGE(index);
        end loop;
        return result;
    end function fn_image;

    signal shot_c : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal event_c : gpx_raw_event_t;
    signal status_c : gpx_lane_status_t;
    signal faults_c : gpx_lane_faults_t;

begin

    shot_c.valid <= i_shot_valid;
    shot_c.request.valid <= i_shot_valid;
    shot_c.request.face_index <= (others => '0');
    shot_c.request.position <= (others => '0');
    shot_c.request.direction <= DIRECTION_CW;
    shot_c.request.shot_index <= (others => '0');
    shot_c.request.last_in_face <= '0';
    shot_c.request.source_sim <= '0';
    shot_c.request.source_latency_clks <= (others => '0');
    shot_c.request.source_latency_valid <= '0';
    shot_c.request.active_version <= to_unsigned(1, 16);
    shot_c.fire_to_t0_clks <= (others => '0');

    o_event_valid <= event_c.valid;
    o_event_kind <= std_logic_vector(to_unsigned(
        gpx_raw_event_kind_t'pos(event_c.kind), 2));
    o_event_word <= event_c.raw_word;
    o_fault_any <= faults_c.drain_timeout_pulse or
        faults_c.sequence_pulse or
        faults_c.response_mismatch_sticky or
        faults_c.raw_drop_sticky or
        faults_c.drain_cap_sticky or
        faults_c.register_overflow_sticky or
        faults_c.run_timeout_pulse or
        faults_c.init_cfg_coalesced_sticky or
        faults_c.command_collision_sticky or
        faults_c.bus_fatal_sticky;

    u_lane : entity work.lidar_gpx_acquisition_lane
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_CHIP_INDEX => 0
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_active_valid => '1',
            i_active_config => fn_active,
            i_register_image => fn_image,
            i_config_apply => i_config_apply,
            o_config_ready => open,
            o_config_done => open,
            i_run_enable => i_run_enable,
            i_soft_reset => '0',
            i_force_reinit => '0',
            i_clear_status => '0',
            o_safe => open,
            i_shot => shot_c,
            o_shot_ready => o_shot_ready,
            i_stop_tdc => i_stop_tdc,
            o_event => event_c,
            i_event_ready => i_event_ready,
            o_adr => o_adr,
            o_csn => o_csn,
            o_rdn => o_rdn,
            o_wrn => o_wrn,
            o_oen => o_oen,
            i_d => i_d,
            o_d => o_d,
            o_d_tri => o_d_tri,
            i_ef1 => i_ef1,
            i_ef2 => i_ef2,
            i_lf1 => i_lf1,
            i_lf2 => i_lf2,
            i_irflag => i_irflag,
            i_errflag => i_errflag,
            o_stopdis => o_stopdis,
            o_alutrigger => o_alutrigger,
            o_puresn => o_puresn,
            o_status => status_c,
            o_faults => faults_c
        );

end architecture rtl;
