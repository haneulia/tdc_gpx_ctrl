library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- Processing-domain activation owner. A candidate configuration is released
-- only after both VDMA lanes are programmed and, when synthetic Echo exists,
-- the Echo delay table reports the same Active Config version.
entity lidar_processing_activation_barrier is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_activate_start : in std_logic;
        i_active_valid   : in std_logic;
        i_active_config  : in lidar_active_config_t;
        i_datapath_idle  : in std_logic;

        i_echo_profile_ready   : in std_logic;
        i_echo_profile_busy    : in std_logic;
        i_echo_profile_version : in u16_t;

        o_rise_cfg_valid   : out std_logic;
        i_rise_cfg_ready   : in  std_logic;
        o_rise_cfg_enable  : out std_logic;
        o_rise_hsize_bytes : out gpx_vdma_geometry_value_t;
        o_rise_vsize_lines : out gpx_vdma_geometry_value_t;
        o_rise_stride_bytes : out gpx_vdma_geometry_value_t;

        o_fall_cfg_valid   : out std_logic;
        i_fall_cfg_ready   : in  std_logic;
        o_fall_cfg_enable  : out std_logic;
        o_fall_hsize_bytes : out gpx_vdma_geometry_value_t;
        o_fall_vsize_lines : out gpx_vdma_geometry_value_t;
        o_fall_stride_bytes : out gpx_vdma_geometry_value_t;

        o_rise_active_profile : out gpx_vdma_lane_profile_t;
        o_fall_active_profile : out gpx_vdma_lane_profile_t;
        o_activate_complete   : out std_logic;
        o_activate_fault      : out std_logic;
        o_busy                : out std_logic
    );
end entity lidar_processing_activation_barrier;

architecture rtl of lidar_processing_activation_barrier is

    type state_t is (ST_IDLE, ST_WAIT_DEPENDENCIES);

    signal state_r : state_t := ST_IDLE;
    signal expected_version_r : u16_t := (others => '0');
    signal vdma_start_r : std_logic := '0';
    signal vdma_abort_r : std_logic := '0';
    signal vdma_complete_c : std_logic;
    signal vdma_fault_c : std_logic;
    signal vdma_done_r : std_logic := '0';
    signal complete_r : std_logic := '0';
    signal fault_r : std_logic := '0';

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-K0-PROC-ACT-001 invalid build configuration"
        severity failure;

    o_activate_complete <= complete_r;
    o_activate_fault <= fault_r;
    o_busy <= '0' when state_r = ST_IDLE else '1';

    u_vdma_transaction : entity work.lidar_gpx_vdma_profile_transaction
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => i_abort or vdma_abort_r,
            i_activate_start => vdma_start_r,
            i_active_valid => i_active_valid,
            i_active_config => i_active_config,
            i_datapath_idle => i_datapath_idle,
            o_rise_cfg_valid => o_rise_cfg_valid,
            i_rise_cfg_ready => i_rise_cfg_ready,
            o_rise_cfg_enable => o_rise_cfg_enable,
            o_rise_hsize_bytes => o_rise_hsize_bytes,
            o_rise_vsize_lines => o_rise_vsize_lines,
            o_rise_stride_bytes => o_rise_stride_bytes,
            o_fall_cfg_valid => o_fall_cfg_valid,
            i_fall_cfg_ready => i_fall_cfg_ready,
            o_fall_cfg_enable => o_fall_cfg_enable,
            o_fall_hsize_bytes => o_fall_hsize_bytes,
            o_fall_vsize_lines => o_fall_vsize_lines,
            o_fall_stride_bytes => o_fall_stride_bytes,
            o_rise_active_profile => o_rise_active_profile,
            o_fall_active_profile => o_fall_active_profile,
            o_activate_complete => vdma_complete_c,
            o_activate_fault => vdma_fault_c,
            o_busy => open
        );

    p_barrier : process (i_clk)
        variable vdma_done_v : std_logic;
        variable echo_done_v : std_logic;
    begin
        if rising_edge(i_clk) then
            vdma_start_r <= '0';
            vdma_abort_r <= '0';
            complete_r <= '0';
            fault_r <= '0';

            if i_rst_n = '0' or i_abort = '1' then
                state_r <= ST_IDLE;
                expected_version_r <= (others => '0');
                vdma_done_r <= '0';
            elsif i_activate_start = '1' and state_r /= ST_IDLE then
                vdma_abort_r <= '1';
                fault_r <= '1';
                state_r <= ST_IDLE;
                vdma_done_r <= '0';
            else
                case state_r is
                    when ST_IDLE =>
                        vdma_done_r <= '0';
                        if i_activate_start = '1' then
                            if i_active_valid /= '1' then
                                fault_r <= '1';
                            else
                                expected_version_r <= i_active_config.version;
                                vdma_start_r <= '1';
                                state_r <= ST_WAIT_DEPENDENCIES;
                            end if;
                        end if;

                    when ST_WAIT_DEPENDENCIES =>
                        vdma_done_v := vdma_done_r;
                        if vdma_complete_c = '1' then
                            vdma_done_r <= '1';
                            vdma_done_v := '1';
                        end if;

                        if G_BUILD_CONFIG.enable_echo_simulation then
                            if i_echo_profile_ready = '1' and
                               i_echo_profile_busy = '0' and
                               i_echo_profile_version = expected_version_r then
                                echo_done_v := '1';
                            else
                                echo_done_v := '0';
                            end if;
                        else
                            echo_done_v := '1';
                        end if;

                        if vdma_fault_c = '1' then
                            fault_r <= '1';
                            state_r <= ST_IDLE;
                            vdma_done_r <= '0';
                        elsif vdma_done_v = '1' and echo_done_v = '1' then
                            complete_r <= '1';
                            state_r <= ST_IDLE;
                            vdma_done_r <= '0';
                        end if;
                end case;
            end if;
        end if;
    end process p_barrier;

end architecture rtl;
