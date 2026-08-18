library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- Processing-domain owner of one atomic Rise/Fall VDMA profile activation.
-- Both lane profiles are derived from the same Active Config version. The
-- enclosing configuration transaction receives COMPLETE only after both
-- external VDMA programming interfaces acknowledge their new geometry.
entity lidar_gpx_vdma_profile_transaction is
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
end entity lidar_gpx_vdma_profile_transaction;

architecture rtl of lidar_gpx_vdma_profile_transaction is

    type state_t is (
        ST_IDLE,
        ST_REQUEST,
        ST_WAIT_PENDING,
        ST_ACTIVATE,
        ST_WAIT_PROGRAM
    );

    signal state_r : state_t := ST_IDLE;
    signal manager_abort_r : std_logic := '0';
    signal manager_abort_c : std_logic;
    signal complete_r : std_logic := '0';
    signal fault_r : std_logic := '0';

    signal request_rise_mask_r : chip_mask_t := (others => '0');
    signal request_fall_mask_r : chip_mask_t := (others => '0');
    signal request_returns_r : unsigned(2 downto 0) := (others => '0');
    signal request_shots_r : shot_index_t := (others => '0');

    signal rise_request_valid_r : std_logic := '0';
    signal fall_request_valid_r : std_logic := '0';
    signal rise_request_ready : std_logic;
    signal fall_request_ready : std_logic;
    signal rise_request_rejected : std_logic;
    signal fall_request_rejected : std_logic;
    signal rise_request_sent_r : std_logic := '0';
    signal fall_request_sent_r : std_logic := '0';

    signal rise_activate_valid_r : std_logic := '0';
    signal fall_activate_valid_r : std_logic := '0';
    signal rise_activate_ready : std_logic;
    signal fall_activate_ready : std_logic;
    signal rise_activate_sent_r : std_logic := '0';
    signal fall_activate_sent_r : std_logic := '0';
    signal rise_profile_activated : std_logic;
    signal fall_profile_activated : std_logic;
    signal rise_program_done_r : std_logic := '0';
    signal fall_program_done_r : std_logic := '0';
    signal rise_pending : std_logic;
    signal fall_pending : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-K0-VDMA-001 invalid build configuration"
        severity failure;

    manager_abort_c <= i_abort or manager_abort_r;
    o_activate_complete <= complete_r;
    o_activate_fault <= fault_r;
    o_busy <= '0' when state_r = ST_IDLE else '1';

    u_rise_profile : entity work.lidar_gpx_vdma_profile_manager
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => manager_abort_c,
            i_request_valid => rise_request_valid_r,
            o_request_ready => rise_request_ready,
            i_lane_chip_mask => request_rise_mask_r,
            i_visible_returns => request_returns_r,
            i_planned_shots => request_shots_r,
            o_request_rejected => rise_request_rejected,
            i_activate_valid => rise_activate_valid_r,
            o_activate_ready => rise_activate_ready,
            i_datapath_idle => i_datapath_idle,
            o_vdma_cfg_valid => o_rise_cfg_valid,
            i_vdma_cfg_ready => i_rise_cfg_ready,
            o_vdma_cfg_enable => o_rise_cfg_enable,
            o_vdma_hsize_bytes => o_rise_hsize_bytes,
            o_vdma_vsize_lines => o_rise_vsize_lines,
            o_vdma_stride_bytes => o_rise_stride_bytes,
            o_active_profile => o_rise_active_profile,
            o_profile_activated => rise_profile_activated,
            o_pending_valid => rise_pending,
            o_busy => open
        );

    u_fall_profile : entity work.lidar_gpx_vdma_profile_manager
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG,
            G_LANE_RISE    => false
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_abort => manager_abort_c,
            i_request_valid => fall_request_valid_r,
            o_request_ready => fall_request_ready,
            i_lane_chip_mask => request_fall_mask_r,
            i_visible_returns => request_returns_r,
            i_planned_shots => request_shots_r,
            o_request_rejected => fall_request_rejected,
            i_activate_valid => fall_activate_valid_r,
            o_activate_ready => fall_activate_ready,
            i_datapath_idle => i_datapath_idle,
            o_vdma_cfg_valid => o_fall_cfg_valid,
            i_vdma_cfg_ready => i_fall_cfg_ready,
            o_vdma_cfg_enable => o_fall_cfg_enable,
            o_vdma_hsize_bytes => o_fall_hsize_bytes,
            o_vdma_vsize_lines => o_fall_vsize_lines,
            o_vdma_stride_bytes => o_fall_stride_bytes,
            o_active_profile => o_fall_active_profile,
            o_profile_activated => fall_profile_activated,
            o_pending_valid => fall_pending,
            o_busy => open
        );

    p_transaction : process (i_clk)
        variable rise_done_v : std_logic;
        variable fall_done_v : std_logic;
    begin
        if rising_edge(i_clk) then
            complete_r <= '0';
            fault_r <= '0';
            manager_abort_r <= '0';

            if i_rst_n = '0' then
                state_r <= ST_IDLE;
                request_rise_mask_r <= (others => '0');
                request_fall_mask_r <= (others => '0');
                request_returns_r <= (others => '0');
                request_shots_r <= (others => '0');
                rise_request_valid_r <= '0';
                fall_request_valid_r <= '0';
                rise_request_sent_r <= '0';
                fall_request_sent_r <= '0';
                rise_activate_valid_r <= '0';
                fall_activate_valid_r <= '0';
                rise_activate_sent_r <= '0';
                fall_activate_sent_r <= '0';
                rise_program_done_r <= '0';
                fall_program_done_r <= '0';
            elsif i_abort = '1' then
                state_r <= ST_IDLE;
                rise_request_valid_r <= '0';
                fall_request_valid_r <= '0';
                rise_activate_valid_r <= '0';
                fall_activate_valid_r <= '0';
                rise_request_sent_r <= '0';
                fall_request_sent_r <= '0';
                rise_activate_sent_r <= '0';
                fall_activate_sent_r <= '0';
                rise_program_done_r <= '0';
                fall_program_done_r <= '0';
            elsif i_activate_start = '1' and state_r /= ST_IDLE then
                fault_r <= '1';
                manager_abort_r <= '1';
                state_r <= ST_IDLE;
                rise_request_valid_r <= '0';
                fall_request_valid_r <= '0';
                rise_activate_valid_r <= '0';
                fall_activate_valid_r <= '0';
            else
                case state_r is
                    when ST_IDLE =>
                        rise_request_sent_r <= '0';
                        fall_request_sent_r <= '0';
                        rise_activate_sent_r <= '0';
                        fall_activate_sent_r <= '0';
                        rise_program_done_r <= '0';
                        fall_program_done_r <= '0';
                        if i_activate_start = '1' then
                            if i_active_valid /= '1' then
                                fault_r <= '1';
                            else
                                request_rise_mask_r <=
                                    i_active_config.derived.active_rise_mask;
                                request_fall_mask_r <=
                                    i_active_config.derived.active_fall_mask;
                                request_returns_r <=
                                    i_active_config.source.tdc.max_hits_per_stop;
                                request_shots_r <=
                                    i_active_config.derived.columns_per_face;
                                rise_request_valid_r <= '1';
                                fall_request_valid_r <= '1';
                                state_r <= ST_REQUEST;
                            end if;
                        end if;

                    when ST_REQUEST =>
                        rise_done_v := rise_request_sent_r;
                        fall_done_v := fall_request_sent_r;
                        if rise_request_valid_r = '1' and
                           rise_request_ready = '1' then
                            rise_request_valid_r <= '0';
                            rise_request_sent_r <= '1';
                            rise_done_v := '1';
                        end if;
                        if fall_request_valid_r = '1' and
                           fall_request_ready = '1' then
                            fall_request_valid_r <= '0';
                            fall_request_sent_r <= '1';
                            fall_done_v := '1';
                        end if;
                        if rise_done_v = '1' and fall_done_v = '1' then
                            state_r <= ST_WAIT_PENDING;
                        end if;

                    when ST_WAIT_PENDING =>
                        if rise_request_rejected = '1' or
                           fall_request_rejected = '1' then
                            fault_r <= '1';
                            manager_abort_r <= '1';
                            state_r <= ST_IDLE;
                        elsif rise_pending = '1' and fall_pending = '1' then
                            rise_activate_valid_r <= '1';
                            fall_activate_valid_r <= '1';
                            rise_activate_sent_r <= '0';
                            fall_activate_sent_r <= '0';
                            state_r <= ST_ACTIVATE;
                        end if;

                    when ST_ACTIVATE =>
                        rise_done_v := rise_activate_sent_r;
                        fall_done_v := fall_activate_sent_r;
                        if rise_activate_valid_r = '1' and
                           rise_activate_ready = '1' then
                            rise_activate_valid_r <= '0';
                            rise_activate_sent_r <= '1';
                            rise_done_v := '1';
                        end if;
                        if fall_activate_valid_r = '1' and
                           fall_activate_ready = '1' then
                            fall_activate_valid_r <= '0';
                            fall_activate_sent_r <= '1';
                            fall_done_v := '1';
                        end if;
                        if rise_done_v = '1' and fall_done_v = '1' then
                            state_r <= ST_WAIT_PROGRAM;
                        end if;

                    when ST_WAIT_PROGRAM =>
                        rise_done_v := rise_program_done_r;
                        fall_done_v := fall_program_done_r;
                        if rise_profile_activated = '1' then
                            rise_program_done_r <= '1';
                            rise_done_v := '1';
                        end if;
                        if fall_profile_activated = '1' then
                            fall_program_done_r <= '1';
                            fall_done_v := '1';
                        end if;
                        if rise_done_v = '1' and fall_done_v = '1' then
                            complete_r <= '1';
                            state_r <= ST_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_transaction;

end architecture rtl;
