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

-- OOC implementation harness for all four physical acquisition lanes and the
-- registered event merger. Flat pin vectors model the eventual IP wrapper.
entity lidar_gpx_acquisition_coordinator_impl is
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
        i_d            : in  std_logic_vector(111 downto 0);
        i_ef1          : in  std_logic_vector(3 downto 0);
        i_ef2          : in  std_logic_vector(3 downto 0);
        i_lf1          : in  std_logic_vector(3 downto 0);
        i_lf2          : in  std_logic_vector(3 downto 0);
        i_irflag       : in  std_logic_vector(3 downto 0);
        i_errflag      : in  std_logic_vector(3 downto 0);
        o_adr          : out std_logic_vector(15 downto 0);
        o_csn          : out std_logic_vector(3 downto 0);
        o_rdn          : out std_logic_vector(3 downto 0);
        o_wrn          : out std_logic_vector(3 downto 0);
        o_oen          : out std_logic_vector(3 downto 0);
        o_d            : out std_logic_vector(111 downto 0);
        o_d_tri        : out std_logic_vector(3 downto 0);
        o_stopdis      : out std_logic_vector(3 downto 0);
        o_alutrigger   : out std_logic_vector(3 downto 0);
        o_puresn       : out std_logic_vector(3 downto 0);
        o_config_ready : out std_logic;
        o_config_done  : out std_logic;
        o_safe         : out std_logic;
        o_shot_ready   : out std_logic;
        o_shot_complete : out std_logic;
        o_event_valid  : out std_logic;
        o_event_kind   : out std_logic_vector(1 downto 0);
        o_event_chip   : out std_logic_vector(1 downto 0);
        o_event_word   : out std_logic_vector(27 downto 0);
        o_terminal_mask : out std_logic_vector(3 downto 0);
        o_fault_any    : out std_logic
    );
end entity lidar_gpx_acquisition_coordinator_impl;

architecture rtl of lidar_gpx_acquisition_coordinator_impl is

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => 150,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "0011",
        fall_capability_mask   => "1100",
        output_width           => 32,
        num_faces              => 5,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    function fn_active return lidar_active_config_t is
        variable runtime : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        runtime.tdc.falling_enable := '1';
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
    signal d_in_c : gpx_bus_data_array_t;
    signal d_out_c : gpx_bus_data_array_t;
    signal adr_c : gpx_bus_address_array_t;
    signal status_c : gpx_lane_status_array_t;
    signal faults_c : gpx_lane_faults_array_t;
    signal active_mask_c : chip_mask_t;

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

    gen_flatten : for index in 0 to 3 generate
        d_in_c(index) <= i_d((index + 1) * 28 - 1 downto index * 28);
        o_d((index + 1) * 28 - 1 downto index * 28) <= d_out_c(index);
        o_adr((index + 1) * 4 - 1 downto index * 4) <= adr_c(index);
    end generate gen_flatten;

    o_event_valid <= event_c.valid;
    o_event_kind <= std_logic_vector(to_unsigned(
        gpx_raw_event_kind_t'pos(event_c.kind), 2));
    o_event_chip <= std_logic_vector(event_c.chip_index);
    o_event_word <= event_c.raw_word;

    p_fault_or : process (all)
        variable value : std_logic;
    begin
        value := '0';
        for index in 0 to 3 loop
            value := value or faults_c(index).drain_timeout_pulse or
                faults_c(index).sequence_pulse or
                faults_c(index).response_mismatch_sticky or
                faults_c(index).raw_drop_sticky or
                faults_c(index).drain_cap_sticky or
                faults_c(index).register_overflow_sticky or
                faults_c(index).run_timeout_pulse or
                faults_c(index).init_cfg_coalesced_sticky or
                faults_c(index).command_collision_sticky or
                faults_c(index).bus_fatal_sticky;
        end loop;
        o_fault_any <= value;
    end process p_fault_or;

    u_coordinator : entity work.lidar_gpx_acquisition_coordinator
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_active_valid => '1',
            i_active_config => fn_active,
            i_register_image => fn_image,
            i_config_apply => i_config_apply,
            o_config_ready => o_config_ready,
            o_config_done => o_config_done,
            i_run_enable => i_run_enable,
            i_soft_reset => '0',
            i_force_reinit => '0',
            i_clear_status => '0',
            o_safe => o_safe,
            i_shot => shot_c,
            o_shot_ready => o_shot_ready,
            i_stop_tdc => i_stop_tdc,
            o_shot_complete => o_shot_complete,
            o_event => event_c,
            i_event_ready => i_event_ready,
            o_adr => adr_c,
            o_csn => o_csn,
            o_rdn => o_rdn,
            o_wrn => o_wrn,
            o_oen => o_oen,
            i_d => d_in_c,
            o_d => d_out_c,
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
            o_active_mask => active_mask_c,
            o_terminal_mask => o_terminal_mask,
            o_status => status_c,
            o_faults => faults_c
        );

end architecture rtl;
