library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;

-- Scalar-generic OOC wrapper used only to implement the two routine build
-- profiles. Production RTL continues to use the single build-config record.
entity lidar_processing_subsystem_impl is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        i_active_valid      : in  std_logic;
        i_active_config     : in  lidar_active_config_t;
        i_operation_state   : in  operation_state_t;
        i_enc_a             : in  std_logic;
        i_enc_b             : in  std_logic;
        i_enc_z             : in  std_logic;
        i_fire_done_raw     : in  std_logic;
        i_clear_diagnostics : in  std_logic;
        m_mon_axis_tready   : in  std_logic;
        m_mon_axis_tvalid   : out std_logic;
        m_mon_axis_tdata    : out processing_monitor_tdata_t;
        m_mon_axis_tkeep    : out processing_monitor_tkeep_t;
        m_mon_axis_tuser    : out processing_monitor_tuser_t;
        m_mon_axis_tlast    : out std_logic;
        o_fire_pulse        : out std_logic;
        o_start_tdc         : out std_logic;
        o_stop_tdc          : out std_logic;
        o_shot_start        : out shot_start_event_t;
        o_shot_result       : out shot_result_t;
        o_pipeline_idle     : out std_logic;
        o_diagnostics       : out processing_diagnostics_t
    );
end entity lidar_processing_subsystem_impl;

architecture rtl of lidar_processing_subsystem_impl is

    function fn_profile return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz      := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz       := G_TDC_CLK_MHZ;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        return result;
    end function fn_profile;

    constant C_PROFILE : lidar_build_config_t := fn_profile;

begin

    u_dut : entity work.lidar_processing_subsystem
        generic map (
            G_BUILD_CONFIG => C_PROFILE
        )
        port map (
            i_clk                => i_clk,
            i_rst_n              => i_rst_n,
            i_active_valid       => i_active_valid,
            i_active_config      => i_active_config,
            i_operation_state    => i_operation_state,
            i_enc_a              => i_enc_a,
            i_enc_b              => i_enc_b,
            i_enc_z              => i_enc_z,
            i_fire_done_raw      => i_fire_done_raw,
            i_clear_diagnostics  => i_clear_diagnostics,
            m_mon_axis_tready    => m_mon_axis_tready,
            m_mon_axis_tvalid    => m_mon_axis_tvalid,
            m_mon_axis_tdata     => m_mon_axis_tdata,
            m_mon_axis_tkeep     => m_mon_axis_tkeep,
            m_mon_axis_tuser     => m_mon_axis_tuser,
            m_mon_axis_tlast     => m_mon_axis_tlast,
            o_fire_pulse         => o_fire_pulse,
            o_start_tdc          => o_start_tdc,
            o_stop_tdc           => o_stop_tdc,
            o_position_event     => open,
            o_face_event         => open,
            o_shot_request       => open,
            o_shot_start         => o_shot_start,
            o_shot_result        => o_shot_result,
            o_current_request    => open,
            o_current_position   => open,
            o_current_direction  => open,
            o_executor_ready     => open,
            o_request_accept     => open,
            o_request_drop       => open,
            o_executor_busy      => open,
            o_physical_arm       => open,
            o_rearm_active       => open,
            o_pipeline_idle      => o_pipeline_idle,
            o_virtual_a          => open,
            o_virtual_b          => open,
            o_virtual_z          => open,
            o_b0_to_accept_clks  => open,
            o_physical_to_fire_clks => open,
            o_virtual_to_accept_clks => open,
            o_fire_done_sync_clks => open,
            o_rearm_margin_clks   => open,
            o_diagnostics         => o_diagnostics
        );

end architecture rtl;
