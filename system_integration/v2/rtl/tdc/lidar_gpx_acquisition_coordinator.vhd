library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.tdc_gpx_pkg.all;

-- Multi-Chip owner for one TDC-domain Shot. The coordinator broadcasts an
-- accepted Shot to every runtime-active lane in one clock, merges all lane
-- events through a registered fair arbiter and completes the Shot only after
-- every active Chip terminal event has crossed the output handshake.
entity lidar_gpx_acquisition_coordinator is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_OEN_MODE     : string := "DYNAMIC_CONNECTED";
        G_POWERUP_TIME_NS         : positive := c_DEFAULT_POWERUP_TIME_NS;
        G_RECOVERY_TIME_NS        : positive := c_DEFAULT_RECOVERY_TIME_NS;
        G_ALU_PULSE_TIME_NS       : positive := c_DEFAULT_ALU_PULSE_TIME_NS;
        G_BUS_IDLE_STABLE_TIME_NS : positive :=
            c_DEFAULT_BUS_IDLE_STABLE_TIME_NS;
        G_DRAIN_MARGIN_TIME_NS    : positive := 6000
    );
    port (
        i_clk              : in  std_logic;
        i_rst_n            : in  std_logic;

        i_active_valid     : in  std_logic;
        i_active_config    : in  lidar_active_config_t;
        i_register_image   : in  gpx_register_image_t;
        i_config_apply     : in  std_logic;
        o_config_ready     : out std_logic;
        o_config_done      : out std_logic;

        i_run_enable       : in  std_logic;
        i_soft_reset       : in  std_logic := '0';
        i_force_reinit     : in  std_logic := '0';
        i_clear_status     : in  std_logic := '0';
        o_safe             : out std_logic;

        i_shot             : in  shot_start_event_t;
        o_shot_ready       : out std_logic;
        i_stop_tdc         : in  std_logic;
        o_shot_complete    : out std_logic;

        o_event            : out gpx_raw_event_t;
        i_event_ready      : in  std_logic;

        o_adr              : out gpx_bus_address_array_t;
        o_csn              : out chip_mask_t;
        o_rdn              : out chip_mask_t;
        o_wrn              : out chip_mask_t;
        o_oen              : out chip_mask_t;
        i_d                : in  gpx_bus_data_array_t;
        o_d                : out gpx_bus_data_array_t;
        o_d_tri            : out chip_mask_t;
        i_ef1              : in  chip_mask_t;
        i_ef2              : in  chip_mask_t;
        i_lf1              : in  chip_mask_t;
        i_lf2              : in  chip_mask_t;
        i_irflag           : in  chip_mask_t;
        i_errflag          : in  chip_mask_t;
        o_stopdis          : out chip_mask_t;
        o_alutrigger       : out chip_mask_t;
        o_puresn           : out chip_mask_t;

        o_active_mask      : out chip_mask_t;
        o_terminal_mask    : out chip_mask_t;
        o_status           : out gpx_lane_status_array_t;
        o_faults           : out gpx_lane_faults_array_t
    );
end entity lidar_gpx_acquisition_coordinator;

architecture rtl of lidar_gpx_acquisition_coordinator is

    constant C_PRESENT_MASK : chip_mask_t := fn_present_chip_mask(
        G_BUILD_CONFIG.num_chips);

    type shot_array_t is array (
        0 to work.lidar_build_pkg.C_MAX_CHIPS - 1) of
        shot_start_event_t;

    signal active_mask_c : chip_mask_t := (others => '0');
    signal lane_run_enable_c : chip_mask_t := (others => '0');
    signal lane_shot_c : shot_array_t :=
        (others => C_SHOT_START_EVENT_IDLE);
    signal lane_shot_ready_c : chip_mask_t := (others => '0');
    signal lane_event_c : gpx_raw_event_array_t :=
        (others => C_GPX_RAW_EVENT_IDLE);
    signal lane_event_ready_c : chip_mask_t := (others => '0');
    signal lane_config_ready_c : chip_mask_t := (others => '0');
    signal lane_config_done_c : chip_mask_t := (others => '0');
    signal lane_safe_c : chip_mask_t := (others => '1');
    signal lane_status_c : gpx_lane_status_array_t :=
        (others => C_GPX_LANE_STATUS_RESET);
    signal lane_faults_c : gpx_lane_faults_array_t :=
        (others => C_GPX_LANE_FAULTS_CLEAR);

    signal adr_c : gpx_bus_address_array_t := (others => (others => '0'));
    signal csn_c : chip_mask_t := (others => '1');
    signal rdn_c : chip_mask_t := (others => '1');
    signal wrn_c : chip_mask_t := (others => '1');
    signal oen_c : chip_mask_t := (others => '1');
    signal d_out_c : gpx_bus_data_array_t := (others => (others => '0'));
    signal d_tri_c : chip_mask_t := (others => '1');
    signal stopdis_c : chip_mask_t := (others => '1');
    signal alutrigger_c : chip_mask_t := (others => '0');
    signal puresn_c : chip_mask_t := (others => '1');

    signal shot_ready_c : std_logic := '0';
    signal shot_accept_c : std_logic := '0';
    signal shot_dispatch_r : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_dispatch_mask_r : chip_mask_t := (others => '0');
    signal merge_outstanding_c : std_logic := '0';
    signal merge_terminal_mask_c : chip_mask_t := (others => '0');

    signal config_ready_c : std_logic := '0';
    signal config_accept_c : std_logic := '0';
    signal config_inflight_r : std_logic := '0';
    signal config_done_mask_r : chip_mask_t := (others => '0');
    signal config_done_r : std_logic := '0';

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-GPX-COORD-001 invalid build configuration"
        severity failure;

    active_mask_c <=
        i_active_config.source.tdc.active_chip_mask and C_PRESENT_MASK
        when i_active_valid = '1' else (others => '0');
    lane_run_enable_c <= active_mask_c when i_run_enable = '1' else
        (others => '0');

    shot_ready_c <= '1' when i_active_valid = '1' and
        i_run_enable = '1' and active_mask_c /= (active_mask_c'range => '0') and
        (lane_shot_ready_c and active_mask_c) = active_mask_c and
        shot_dispatch_r.valid = '0' and
        merge_outstanding_c = '0' else '0';
    shot_accept_c <= i_shot.valid and shot_ready_c;

    config_ready_c <= '1' when config_inflight_r = '0' and
        shot_dispatch_r.valid = '0' and
        merge_outstanding_c = '0' and
        (lane_config_ready_c and C_PRESENT_MASK) = C_PRESENT_MASK else '0';
    config_accept_c <= i_config_apply and config_ready_c;

    o_shot_ready <= shot_ready_c;
    o_config_ready <= config_ready_c;
    o_config_done <= config_done_r;
    o_safe <= '1' when config_inflight_r = '0' and
                      shot_dispatch_r.valid = '0' and
                      merge_outstanding_c = '0' and
                      (lane_safe_c and C_PRESENT_MASK) = C_PRESENT_MASK
              else '0';

    o_adr <= adr_c;
    o_csn <= csn_c;
    o_rdn <= rdn_c;
    o_wrn <= wrn_c;
    o_oen <= oen_c;
    o_d <= d_out_c;
    o_d_tri <= d_tri_c;
    o_stopdis <= stopdis_c;
    o_alutrigger <= alutrigger_c;
    o_puresn <= puresn_c;
    o_active_mask <= active_mask_c;
    o_terminal_mask <= merge_terminal_mask_c;
    o_status <= lane_status_c;
    o_faults <= lane_faults_c;

    p_shot_broadcast : process (all)
        variable value : shot_array_t;
    begin
        value := (others => C_SHOT_START_EVENT_IDLE);
        if shot_dispatch_r.valid = '1' then
            for index in 0 to work.lidar_build_pkg.C_MAX_CHIPS - 1 loop
                if shot_dispatch_mask_r(index) = '1' then
                    value(index) := shot_dispatch_r;
                end if;
            end loop;
        end if;
        lane_shot_c <= value;
    end process p_shot_broadcast;

    -- Capture the accepted cross-domain Shot before broadcasting it. This
    -- removes the all-lane-ready reduction from every lane FSM input cone and
    -- gives all active Chips the same registered Shot context one clock later.
    p_shot_dispatch : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                shot_dispatch_r <= C_SHOT_START_EVENT_IDLE;
                shot_dispatch_mask_r <= (others => '0');
            else
                if shot_dispatch_r.valid = '1' then
                    shot_dispatch_r <= C_SHOT_START_EVENT_IDLE;
                    shot_dispatch_mask_r <= (others => '0');
                end if;

                if shot_accept_c = '1' then
                    shot_dispatch_r <= i_shot;
                    shot_dispatch_r.valid <= '1';
                    shot_dispatch_mask_r <= active_mask_c;
                end if;

                -- synthesis translate_off
                if shot_dispatch_r.valid = '1' then
                    assert (lane_shot_ready_c and shot_dispatch_mask_r) =
                           shot_dispatch_mask_r
                        report "V2-GPX-COORD-005 registered Shot dispatch lost lane readiness"
                        severity failure;
                end if;
                -- synthesis translate_on
            end if;
        end if;
    end process p_shot_dispatch;

    p_config_completion : process (i_clk)
        variable completed : chip_mask_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                config_inflight_r <= '0';
                config_done_mask_r <= (others => '0');
                config_done_r <= '0';
            else
                completed := config_done_mask_r;
                config_done_r <= '0';

                if config_accept_c = '1' then
                    completed := (others => '0');
                    config_inflight_r <= '1';
                end if;

                for index in 0 to work.lidar_build_pkg.C_MAX_CHIPS - 1 loop
                    if lane_config_done_c(index) = '1' then
                        completed(index) := '1';
                    end if;
                end loop;

                if config_inflight_r = '1' and
                   (completed and C_PRESENT_MASK) = C_PRESENT_MASK then
                    config_inflight_r <= '0';
                    config_done_r <= '1';
                end if;
                config_done_mask_r <= completed;

                -- synthesis translate_off
                assert not (i_config_apply = '1' and config_ready_c = '0')
                    report "V2-GPX-COORD-002 config apply while not ready"
                    severity warning;
                assert not (i_shot.valid = '1' and shot_ready_c = '0')
                    report "V2-GPX-COORD-003 Shot valid while not ready"
                    severity warning;
                if shot_accept_c = '1' then
                    assert i_shot.request.valid = '1'
                        report "V2-GPX-COORD-004 accepted Shot has invalid request"
                        severity failure;
                end if;
                -- synthesis translate_on
            end if;
        end if;
    end process p_config_completion;

    gen_lanes : for index in
        0 to work.lidar_build_pkg.C_MAX_CHIPS - 1 generate
        gen_present : if index < G_BUILD_CONFIG.num_chips generate
            u_lane : entity work.lidar_gpx_acquisition_lane
                generic map (
                    G_BUILD_CONFIG => G_BUILD_CONFIG,
                    G_CHIP_INDEX => index,
                    G_OEN_MODE => G_OEN_MODE,
                    G_POWERUP_TIME_NS => G_POWERUP_TIME_NS,
                    G_RECOVERY_TIME_NS => G_RECOVERY_TIME_NS,
                    G_ALU_PULSE_TIME_NS => G_ALU_PULSE_TIME_NS,
                    G_BUS_IDLE_STABLE_TIME_NS =>
                        G_BUS_IDLE_STABLE_TIME_NS,
                    G_DRAIN_MARGIN_TIME_NS => G_DRAIN_MARGIN_TIME_NS
                )
                port map (
                    i_clk => i_clk,
                    i_rst_n => i_rst_n,
                    i_active_valid => i_active_valid,
                    i_active_config => i_active_config,
                    i_register_image => i_register_image,
                    i_config_apply => config_accept_c,
                    o_config_ready => lane_config_ready_c(index),
                    o_config_done => lane_config_done_c(index),
                    i_run_enable => lane_run_enable_c(index),
                    i_soft_reset => i_soft_reset,
                    i_force_reinit => i_force_reinit,
                    i_clear_status => i_clear_status,
                    o_safe => lane_safe_c(index),
                    i_shot => lane_shot_c(index),
                    o_shot_ready => lane_shot_ready_c(index),
                    i_stop_tdc => i_stop_tdc and active_mask_c(index),
                    o_event => lane_event_c(index),
                    i_event_ready => lane_event_ready_c(index),
                    o_adr => adr_c(index),
                    o_csn => csn_c(index),
                    o_rdn => rdn_c(index),
                    o_wrn => wrn_c(index),
                    o_oen => oen_c(index),
                    i_d => i_d(index),
                    o_d => d_out_c(index),
                    o_d_tri => d_tri_c(index),
                    i_ef1 => i_ef1(index),
                    i_ef2 => i_ef2(index),
                    i_lf1 => i_lf1(index),
                    i_lf2 => i_lf2(index),
                    i_irflag => i_irflag(index),
                    i_errflag => i_errflag(index),
                    o_stopdis => stopdis_c(index),
                    o_alutrigger => alutrigger_c(index),
                    o_puresn => puresn_c(index),
                    o_status => lane_status_c(index),
                    o_faults => lane_faults_c(index)
                );
        end generate gen_present;

        gen_absent : if index >= G_BUILD_CONFIG.num_chips generate
            lane_shot_ready_c(index) <= '0';
            lane_event_c(index) <= C_GPX_RAW_EVENT_IDLE;
            lane_config_ready_c(index) <= '0';
            lane_config_done_c(index) <= '0';
            lane_safe_c(index) <= '1';
            lane_status_c(index) <= C_GPX_LANE_STATUS_RESET;
            lane_faults_c(index) <= C_GPX_LANE_FAULTS_CLEAR;
            adr_c(index) <= (others => '0');
            csn_c(index) <= '1';
            rdn_c(index) <= '1';
            wrn_c(index) <= '1';
            oen_c(index) <= '1';
            d_out_c(index) <= (others => '0');
            d_tri_c(index) <= '1';
            stopdis_c(index) <= '1';
            alutrigger_c(index) <= '0';
            puresn_c(index) <= '1';
        end generate gen_absent;
    end generate gen_lanes;

    u_merge : entity work.lidar_gpx_event_merge
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_shot_accept => shot_dispatch_r.valid,
            i_shot_mask => shot_dispatch_mask_r,
            i_lane_event => lane_event_c,
            o_lane_ready => lane_event_ready_c,
            o_event => o_event,
            i_event_ready => i_event_ready,
            o_shot_complete => o_shot_complete,
            o_shot_outstanding => merge_outstanding_c,
            o_terminal_mask => merge_terminal_mask_c
        );

end architecture rtl;
