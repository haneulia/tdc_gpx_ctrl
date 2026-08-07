library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.tdc_gpx_pkg.all;

-- Production B5 assembly. Processing events cross only through named
-- gateways; the coordinator and every physical GPX pin remain in the TDC
-- domain. The result FIFO is derived from immutable topology so it can hold
-- one maximum-capacity Shot without adding a runtime sizing variable.
entity lidar_gpx_acquisition_subsystem is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_SHOT_FIFO_DEPTH : positive := 16;
        G_STOP_FIFO_DEPTH : positive := 16;
        G_OEN_MODE : string := "DYNAMIC_CONNECTED";
        G_POWERUP_TIME_NS         : positive := c_DEFAULT_POWERUP_TIME_NS;
        G_RECOVERY_TIME_NS        : positive := c_DEFAULT_RECOVERY_TIME_NS;
        G_ALU_PULSE_TIME_NS       : positive := c_DEFAULT_ALU_PULSE_TIME_NS;
        G_BUS_IDLE_STABLE_TIME_NS : positive :=
            c_DEFAULT_BUS_IDLE_STABLE_TIME_NS;
        G_DRAIN_MARGIN_TIME_NS    : positive := 6000
    );
    port (
        i_proc_clk          : in  std_logic;
        i_proc_rst_n        : in  std_logic;
        i_proc_shot         : in  shot_start_event_t;
        o_proc_shot_ready   : out std_logic;
        i_proc_stop_tdc     : in  std_logic;
        i_proc_clear_status : in  std_logic := '0';
        o_proc_result       : out gpx_raw_event_t;
        i_proc_result_ready : in  std_logic;
        o_shot_drop_sticky  : out std_logic;
        o_stop_drop_sticky  : out std_logic;

        i_tdc_clk           : in  std_logic;
        i_tdc_rst_n         : in  std_logic;
        i_tdc_active_valid  : in  std_logic;
        i_tdc_active_config : in  lidar_active_config_t;
        i_tdc_register_image : in gpx_register_image_t;
        i_tdc_config_apply  : in  std_logic;
        o_tdc_config_ready  : out std_logic;
        o_tdc_config_done   : out std_logic;
        i_tdc_run_enable    : in  std_logic;
        i_tdc_soft_reset    : in  std_logic := '0';
        i_tdc_force_reinit  : in  std_logic := '0';
        i_tdc_clear_status  : in  std_logic := '0';
        o_tdc_safe          : out std_logic;
        o_tdc_shot_complete : out std_logic;
        -- Processing-clock-domain reset busy. Safe for Processing control
        -- decisions; TDC-domain reset busy is folded into o_tdc_safe.
        o_cdc_reset_busy    : out std_logic;

        o_adr        : out gpx_bus_address_array_t;
        o_csn        : out chip_mask_t;
        o_rdn        : out chip_mask_t;
        o_wrn        : out chip_mask_t;
        o_oen        : out chip_mask_t;
        i_d          : in  gpx_bus_data_array_t;
        o_d          : out gpx_bus_data_array_t;
        o_d_tri      : out chip_mask_t;
        i_ef1        : in  chip_mask_t;
        i_ef2        : in  chip_mask_t;
        i_lf1        : in  chip_mask_t;
        i_lf2        : in  chip_mask_t;
        i_irflag     : in  chip_mask_t;
        i_errflag    : in  chip_mask_t;
        o_stopdis    : out chip_mask_t;
        o_alutrigger : out chip_mask_t;
        o_puresn     : out chip_mask_t;

        o_active_mask   : out chip_mask_t;
        o_terminal_mask : out chip_mask_t;
        o_status        : out gpx_lane_status_array_t;
        o_faults        : out gpx_lane_faults_array_t
    );
end entity lidar_gpx_acquisition_subsystem;

architecture rtl of lidar_gpx_acquisition_subsystem is

    constant C_RESULT_FIFO_DEPTH : positive :=
        fn_gpx_result_fifo_depth(G_BUILD_CONFIG);

    signal shot_pending_r : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_source_ready_c : std_logic;
    signal shot_ingress_ready_c : std_logic;
    signal shot_drop_r : std_logic := '0';
    signal tdc_shot_c : shot_start_event_t;
    signal tdc_shot_ready_c : std_logic;
    signal shot_proc_reset_busy_c : std_logic;
    signal shot_tdc_reset_busy_c : std_logic;

    signal tdc_stop_c : std_logic;
    signal stop_drop_c : std_logic;
    signal stop_proc_reset_busy_c : std_logic;
    signal stop_tdc_reset_busy_c : std_logic;

    signal tdc_event_c : gpx_raw_event_t;
    signal tdc_event_ready_c : std_logic;
    signal tdc_event_payload_c : gpx_raw_event_payload_t;
    signal buffered_event_payload_c : gpx_raw_event_payload_t;
    signal buffered_event_valid_c : std_logic;
    signal buffered_event_c : gpx_raw_event_t;
    signal result_gateway_ready_c : std_logic;
    signal result_proc_reset_busy_c : std_logic;
    signal result_tdc_reset_busy_c : std_logic;
    signal proc_reset_busy_c : std_logic;
    signal tdc_reset_busy_c : std_logic;
    signal coordinator_safe_c : std_logic;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-GPX-SUBSYS-001 invalid build configuration"
        severity failure;
    assert C_RESULT_FIFO_DEPTH >=
           fn_gpx_events_per_shot_capacity(G_BUILD_CONFIG)
        report "V2-GPX-SUBSYS-002 result FIFO cannot hold one maximum Shot"
        severity failure;

    shot_ingress_ready_c <= not shot_pending_r.valid or shot_source_ready_c;
    o_proc_shot_ready <= shot_ingress_ready_c;
    o_shot_drop_sticky <= shot_drop_r;
    o_stop_drop_sticky <= stop_drop_c;
    proc_reset_busy_c <= shot_proc_reset_busy_c or
        stop_proc_reset_busy_c or result_proc_reset_busy_c;
    tdc_reset_busy_c <= shot_tdc_reset_busy_c or
        stop_tdc_reset_busy_c or result_tdc_reset_busy_c;
    o_cdc_reset_busy <= proc_reset_busy_c;
    o_tdc_safe <= coordinator_safe_c and not tdc_reset_busy_c;

    p_shot_ingress : process (i_proc_clk, i_proc_rst_n)
    begin
        if i_proc_rst_n = '0' then
            shot_pending_r <= C_SHOT_START_EVENT_IDLE;
            shot_drop_r <= '0';
        elsif rising_edge(i_proc_clk) then
            if shot_pending_r.valid = '1' and shot_source_ready_c = '1' then
                shot_pending_r <= C_SHOT_START_EVENT_IDLE;
            end if;

            if i_proc_clear_status = '1' then
                shot_drop_r <= '0';
            end if;

            if i_proc_shot.valid = '1' then
                if shot_ingress_ready_c = '1' then
                    shot_pending_r <= i_proc_shot;
                else
                    shot_drop_r <= '1';
                end if;
            end if;
        end if;
    end process p_shot_ingress;

    u_shot_gateway : entity work.lidar_gpx_shot_gateway
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG,
            G_FIFO_DEPTH   => G_SHOT_FIFO_DEPTH
        )
        port map (
            i_proc_clk   => i_proc_clk,
            i_proc_rst_n => i_proc_rst_n,
            i_proc_shot  => shot_pending_r,
            o_proc_ready => shot_source_ready_c,
            i_tdc_clk    => i_tdc_clk,
            i_tdc_rst_n  => i_tdc_rst_n,
            o_tdc_shot   => tdc_shot_c,
            i_tdc_ready  => tdc_shot_ready_c,
            o_proc_reset_busy => shot_proc_reset_busy_c,
            o_tdc_reset_busy  => shot_tdc_reset_busy_c,
            o_reset_busy      => open
        );

    u_stop_gateway : entity work.lidar_gpx_stop_gateway
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG,
            G_FIFO_DEPTH   => G_STOP_FIFO_DEPTH
        )
        port map (
            i_proc_clk        => i_proc_clk,
            i_proc_rst_n      => i_proc_rst_n,
            i_proc_stop_tdc   => i_proc_stop_tdc,
            i_clear_status    => i_proc_clear_status,
            o_overflow_sticky => stop_drop_c,
            i_tdc_clk         => i_tdc_clk,
            i_tdc_rst_n       => i_tdc_rst_n,
            o_tdc_stop_tdc    => tdc_stop_c,
            o_proc_reset_busy => stop_proc_reset_busy_c,
            o_tdc_reset_busy  => stop_tdc_reset_busy_c,
            o_reset_busy      => open
        );

    u_coordinator : entity work.lidar_gpx_acquisition_coordinator
        generic map (
            G_BUILD_CONFIG            => G_BUILD_CONFIG,
            G_OEN_MODE                => G_OEN_MODE,
            G_POWERUP_TIME_NS         => G_POWERUP_TIME_NS,
            G_RECOVERY_TIME_NS        => G_RECOVERY_TIME_NS,
            G_ALU_PULSE_TIME_NS       => G_ALU_PULSE_TIME_NS,
            G_BUS_IDLE_STABLE_TIME_NS => G_BUS_IDLE_STABLE_TIME_NS,
            G_DRAIN_MARGIN_TIME_NS    => G_DRAIN_MARGIN_TIME_NS
        )
        port map (
            i_clk            => i_tdc_clk,
            i_rst_n          => i_tdc_rst_n,
            i_active_valid   => i_tdc_active_valid,
            i_active_config  => i_tdc_active_config,
            i_register_image => i_tdc_register_image,
            i_config_apply   => i_tdc_config_apply,
            o_config_ready   => o_tdc_config_ready,
            o_config_done    => o_tdc_config_done,
            i_run_enable     => i_tdc_run_enable,
            i_soft_reset     => i_tdc_soft_reset,
            i_force_reinit   => i_tdc_force_reinit,
            i_clear_status   => i_tdc_clear_status,
            o_safe           => coordinator_safe_c,
            i_shot           => tdc_shot_c,
            o_shot_ready     => tdc_shot_ready_c,
            i_stop_tdc       => tdc_stop_c,
            o_shot_complete  => o_tdc_shot_complete,
            o_event          => tdc_event_c,
            i_event_ready    => tdc_event_ready_c,
            o_adr            => o_adr,
            o_csn            => o_csn,
            o_rdn            => o_rdn,
            o_wrn            => o_wrn,
            o_oen            => o_oen,
            i_d              => i_d,
            o_d              => o_d,
            o_d_tri          => o_d_tri,
            i_ef1            => i_ef1,
            i_ef2            => i_ef2,
            i_lf1            => i_lf1,
            i_lf2            => i_lf2,
            i_irflag         => i_irflag,
            i_errflag        => i_errflag,
            o_stopdis        => o_stopdis,
            o_alutrigger     => o_alutrigger,
            o_puresn         => o_puresn,
            o_active_mask    => o_active_mask,
            o_terminal_mask  => o_terminal_mask,
            o_status         => o_status,
            o_faults         => o_faults
        );

    -- Break the result FIFO full/reset-ready cone before it reaches the
    -- multi-Chip merge. This two-entry registered-ready boundary preserves
    -- one event per TDC clock and absorbs one stale-ready cycle without
    -- changing event ordering or payload identity.
    tdc_event_payload_c <= fn_pack_raw_event(tdc_event_c);

    u_result_ingress_skid : entity work.tdc_gpx_skid_buffer
        generic map (
            g_DATA_WIDTH => C_GPX_RAW_EVENT_PAYLOAD_WIDTH
        )
        port map (
            i_clk     => i_tdc_clk,
            i_rst_n   => i_tdc_rst_n,
            i_flush   => '0',
            i_s_valid => tdc_event_c.valid,
            o_s_ready => tdc_event_ready_c,
            i_s_data  => tdc_event_payload_c,
            o_m_valid => buffered_event_valid_c,
            i_m_ready => result_gateway_ready_c,
            o_m_data  => buffered_event_payload_c
        );

    p_buffered_event : process (all)
        variable result : gpx_raw_event_t;
    begin
        result := C_GPX_RAW_EVENT_IDLE;
        if buffered_event_valid_c = '1' then
            result := fn_unpack_raw_event(buffered_event_payload_c);
            result.valid := '1';
        end if;
        buffered_event_c <= result;
    end process p_buffered_event;

    u_result_gateway : entity work.lidar_gpx_result_gateway
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG,
            G_FIFO_DEPTH   => C_RESULT_FIFO_DEPTH
        )
        port map (
            i_tdc_clk     => i_tdc_clk,
            i_tdc_rst_n   => i_tdc_rst_n,
            i_tdc_result  => buffered_event_c,
            o_tdc_ready   => result_gateway_ready_c,
            i_proc_clk    => i_proc_clk,
            i_proc_rst_n  => i_proc_rst_n,
            o_proc_result => o_proc_result,
            i_proc_ready  => i_proc_result_ready,
            o_tdc_reset_busy  => result_tdc_reset_busy_c,
            o_proc_reset_busy => result_proc_reset_busy_c,
            o_reset_busy      => open
        );

end architecture rtl;
