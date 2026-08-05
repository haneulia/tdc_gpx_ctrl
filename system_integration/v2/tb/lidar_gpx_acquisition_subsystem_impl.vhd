library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_event_pkg.all;

-- Scalar-port OOC harness for the complete Processing/TDC B5 acquisition
-- assembly. Four chips expose 32 physical STOP lanes. Dedicated 2-rise/2-fall
-- masks represent 16 APD channels observed as 16 rise and 16 fall lanes.
entity lidar_gpx_acquisition_subsystem_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
    port (
        i_proc_clk       : in  std_logic;
        i_proc_rst_n     : in  std_logic;
        i_tdc_clk        : in  std_logic;
        i_tdc_rst_n      : in  std_logic;
        i_run_enable     : in  std_logic;
        i_config_apply   : in  std_logic;
        i_shot_valid     : in  std_logic;
        i_shot_payload   : in  gpx_shot_start_payload_t;
        i_stop_tdc       : in  std_logic;
        i_result_ready   : in  std_logic;
        i_d              : in  std_logic_vector(111 downto 0);
        i_ef1            : in  std_logic_vector(3 downto 0);
        i_ef2            : in  std_logic_vector(3 downto 0);
        i_lf1            : in  std_logic_vector(3 downto 0);
        i_lf2            : in  std_logic_vector(3 downto 0);
        i_irflag         : in  std_logic_vector(3 downto 0);
        i_errflag        : in  std_logic_vector(3 downto 0);
        o_adr            : out std_logic_vector(15 downto 0);
        o_csn            : out std_logic_vector(3 downto 0);
        o_rdn            : out std_logic_vector(3 downto 0);
        o_wrn            : out std_logic_vector(3 downto 0);
        o_oen            : out std_logic_vector(3 downto 0);
        o_d              : out std_logic_vector(111 downto 0);
        o_d_tri          : out std_logic_vector(3 downto 0);
        o_stopdis        : out std_logic_vector(3 downto 0);
        o_alutrigger     : out std_logic_vector(3 downto 0);
        o_puresn         : out std_logic_vector(3 downto 0);
        o_config_ready   : out std_logic;
        o_config_done    : out std_logic;
        o_safe           : out std_logic;
        o_shot_ready     : out std_logic;
        o_shot_complete  : out std_logic;
        o_result_valid   : out std_logic;
        o_result_payload : out gpx_raw_event_payload_t;
        o_cdc_reset_busy : out std_logic;
        o_drop_any       : out std_logic;
        o_fault_any      : out std_logic
    );
end entity lidar_gpx_acquisition_subsystem_impl;

architecture rtl of lidar_gpx_acquisition_subsystem_impl is

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "0011",
        fall_capability_mask   => "1100",
        output_width           => 32,
        num_faces              => 4,
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

    signal shot_c : shot_start_event_t;
    signal result_c : gpx_raw_event_t;
    signal d_in_c : gpx_bus_data_array_t;
    signal d_out_c : gpx_bus_data_array_t;
    signal adr_c : gpx_bus_address_array_t;
    signal status_c : gpx_lane_status_array_t;
    signal faults_c : gpx_lane_faults_array_t;
    signal active_mask_c : chip_mask_t;
    signal terminal_mask_c : chip_mask_t;
    signal shot_drop_c : std_logic;
    signal stop_drop_c : std_logic;

begin

    p_shot : process (all)
        variable value : shot_start_event_t;
    begin
        value := fn_unpack_shot_start(i_shot_payload);
        value.valid := i_shot_valid;
        shot_c <= value;
    end process p_shot;

    gen_flatten : for index in 0 to 3 generate
        d_in_c(index) <= i_d((index + 1) * 28 - 1 downto index * 28);
        o_d((index + 1) * 28 - 1 downto index * 28) <= d_out_c(index);
        o_adr((index + 1) * 4 - 1 downto index * 4) <= adr_c(index);
    end generate gen_flatten;

    o_result_valid <= result_c.valid;
    o_result_payload <= fn_pack_raw_event(result_c);
    o_drop_any <= shot_drop_c or stop_drop_c;

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

    u_subsystem : entity work.lidar_gpx_acquisition_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_proc_clk => i_proc_clk,
            i_proc_rst_n => i_proc_rst_n,
            i_proc_shot => shot_c,
            o_proc_shot_ready => o_shot_ready,
            i_proc_stop_tdc => i_stop_tdc,
            i_proc_clear_status => '0',
            o_proc_result => result_c,
            i_proc_result_ready => i_result_ready,
            o_shot_drop_sticky => shot_drop_c,
            o_stop_drop_sticky => stop_drop_c,
            i_tdc_clk => i_tdc_clk,
            i_tdc_rst_n => i_tdc_rst_n,
            i_tdc_active_valid => '1',
            i_tdc_active_config => fn_active,
            i_tdc_register_image => C_GPX_REGISTER_IMAGE_DEFAULT,
            i_tdc_config_apply => i_config_apply,
            o_tdc_config_ready => o_config_ready,
            o_tdc_config_done => o_config_done,
            i_tdc_run_enable => i_run_enable,
            i_tdc_soft_reset => '0',
            i_tdc_force_reinit => '0',
            i_tdc_clear_status => '0',
            o_tdc_safe => o_safe,
            o_tdc_shot_complete => o_shot_complete,
            o_cdc_reset_busy => o_cdc_reset_busy,
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
            o_terminal_mask => terminal_mask_c,
            o_status => status_c,
            o_faults => faults_c
        );

end architecture rtl;
