library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

-- One typed v2 acquisition lane around the frozen v1 Chip controller and the
-- H1 physical-bus wrapper. This module owns no scan geometry and performs no
-- Hit/distance decode. It preserves the complete 28-bit I-Mode word and adds
-- only Chip, IFIFO, Shot-context and terminal-event identity.
entity lidar_gpx_acquisition_lane is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_CHIP_INDEX   : natural range 0 to 3 := 0;
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

        o_event            : out gpx_raw_event_t;
        i_event_ready      : in  std_logic;

        o_adr              : out gpx_bus_address_t;
        o_csn              : out std_logic;
        o_rdn              : out std_logic;
        o_wrn              : out std_logic;
        o_oen              : out std_logic;
        i_d                : in  gpx_bus_data_t;
        o_d                : out gpx_bus_data_t;
        o_d_tri            : out std_logic;
        i_ef1              : in  std_logic;
        i_ef2              : in  std_logic;
        i_lf1              : in  std_logic;
        i_lf2              : in  std_logic;
        i_irflag           : in  std_logic;
        i_errflag          : in  std_logic;
        o_stopdis          : out std_logic;
        o_alutrigger       : out std_logic;
        o_puresn           : out std_logic;

        o_status           : out gpx_lane_status_t;
        o_faults           : out gpx_lane_faults_t
    );
end entity lidar_gpx_acquisition_lane;

architecture rtl of lidar_gpx_acquisition_lane is

    constant C_POWERUP_CLKS : positive := fn_time_ns_to_clks_ceil(
        G_POWERUP_TIME_NS, G_BUILD_CONFIG.tdc_clk_mhz);
    constant C_RECOVERY_CLKS : positive := fn_time_ns_to_clks_ceil(
        G_RECOVERY_TIME_NS, G_BUILD_CONFIG.tdc_clk_mhz);
    constant C_ALU_PULSE_CLKS : positive := fn_time_ns_to_clks_ceil(
        G_ALU_PULSE_TIME_NS, G_BUILD_CONFIG.tdc_clk_mhz);
    constant C_BUS_IDLE_STABLE_CLKS : positive := fn_time_ns_to_clks_ceil(
        G_BUS_IDLE_STABLE_TIME_NS, G_BUILD_CONFIG.tdc_clk_mhz);
    constant C_DRAIN_MARGIN_CLKS : positive := fn_time_ns_to_clks_ceil(
        G_DRAIN_MARGIN_TIME_NS, G_BUILD_CONFIG.tdc_clk_mhz);
    constant C_EF_SYNC_GUARD_CLKS : positive :=
        fn_time_ps_to_clks_ceil(
            c_TDC_EF_DATA_VALID_MAX_PS,
            G_BUILD_CONFIG.tdc_clk_mhz) + c_TDC_STATUS_SYNC_CLKS;
    constant C_DRAIN_CAP_QUADS : positive :=
        fn_gpx_drain_cap_quads(G_BUILD_CONFIG, G_CHIP_INDEX);

    type lane_state_t is (
        LANE_BOOT,
        LANE_IDLE,
        LANE_CONFIG_WAIT,
        LANE_START_WAIT,
        LANE_ARMED,
        LANE_SHOT,
        LANE_STOP_WAIT
    );

    signal state_r : lane_state_t := LANE_BOOT;
    signal initialized_r : std_logic := '0';
    signal config_seen_busy_r : std_logic := '0';
    signal config_done_r : std_logic := '0';
    signal stop_sent_r : std_logic := '0';

    signal cmd_start_r : std_logic := '0';
    signal cmd_stop_r : std_logic := '0';
    signal cmd_cfg_write_r : std_logic := '0';
    signal shot_start_r : std_logic := '0';

    signal shot_context_r : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_seq_start_r : unsigned(15 downto 0) := (others => '0');
    signal terminal_seen_r : std_logic := '0';
    signal sequence_advanced_r : std_logic := '0';
    signal timeout_seen_r : std_logic := '0';
    signal timeout_cause_r : std_logic_vector(2 downto 0) := (others => '0');

    signal legacy_cfg_c : t_tdc_cfg := c_TDC_CFG_INIT;
    signal legacy_image_c : t_cfg_image := c_GPX_DEFAULT_IMAGE;

    signal bus_request_c : gpx_bus_request_t := C_GPX_BUS_REQUEST_IDLE;
    signal bus_response_c : gpx_bus_response_t := C_GPX_BUS_RESPONSE_IDLE;
    signal bus_timing_c : gpx_bus_timing_t := C_GPX_BUS_TIMING_DEFAULT;
    signal pin_status_c : gpx_pin_status_t := C_GPX_PIN_STATUS_RESET;
    signal bus_busy_c : std_logic;
    signal bus_response_pending_c : std_logic;
    signal bus_response_ready_c : std_logic;
    signal effective_ticks_c : unsigned(2 downto 0);
    signal tick_en_c : std_logic;

    signal raw_valid_c : std_logic;
    signal raw_data_c : t_raw_axis_tdata;
    signal raw_user_c : t_raw_axis_tuser;
    signal drain_done_c : std_logic;
    signal run_drain_complete_c : std_logic;
    signal chip_shot_seq_c : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
    signal controller_busy_c : std_logic;

    signal err_drain_timeout_c : std_logic;
    signal err_sequence_c : std_logic;
    signal err_rsp_mismatch_c : std_logic;
    signal err_raw_overflow_c : std_logic;
    signal err_raw_drop_c : std_logic;
    signal err_drain_cap_c : std_logic;
    signal physical_drain_cap_sticky_r : std_logic := '0';
    signal err_reg_overflow_c : std_logic;
    signal run_timeout_c : std_logic;
    signal run_timeout_cause_c : std_logic_vector(2 downto 0);
    signal init_cfg_coalesced_c : std_logic;
    signal err_cmd_collision_c : std_logic;
    signal err_bus_fatal_c : std_logic;
    signal run_active_c : std_logic;
    signal shot_outstanding_c : std_logic;
    signal capture_window_c : unsigned(15 downto 0) := (others => '0');

    function fn_legacy_config(
        active_valid : std_logic;
        active       : lidar_active_config_t;
        image        : gpx_register_image_t
    ) return t_tdc_cfg is
        variable result : t_tdc_cfg := c_TDC_CFG_INIT;
    begin
        result.active_chip_mask := fn_present_chip_mask(
            G_BUILD_CONFIG.num_chips);
        result.stops_per_chip := to_unsigned(
            G_BUILD_CONFIG.stops_per_chip,
            result.stops_per_chip'length);
        -- One cap unit is four raw words per IFIFO. Use the immutable build
        -- capacity so a malformed GPX stream cannot exceed the implemented
        -- STOP/Return storage, while runtime max_hits remains a downstream
        -- cell/formatter policy and never changes physical drain safety.
        result.n_drain_cap := to_unsigned(
            C_DRAIN_CAP_QUADS, result.n_drain_cap'length);
        result.cfg_reg7 := image(7);

        if active_valid = '1' then
            result.active_chip_mask := active.source.tdc.active_chip_mask;
            result.bus_clk_div := active.source.tdc.bus_clk_div;
            result.bus_ticks := active.source.tdc.bus_ticks(2 downto 0);
            result.start_off1 := active.source.tdc.start_offset;
            result.falling_enable := active.source.tdc.falling_enable;
        end if;
        return result;
    end function fn_legacy_config;

    function fn_prepared_image(
        active_valid : std_logic;
        active       : lidar_active_config_t;
        base_image   : gpx_register_image_t
    ) return t_cfg_image is
        variable result : t_cfg_image := (others => (others => '0'));
        variable chip_active : boolean;
        variable rise_enabled : boolean;
        variable fall_enabled : boolean;
        variable default_runtime : lidar_runtime_config_t;
    begin
        for register_index in result'range loop
            result(register_index) := base_image(register_index);
        end loop;

        default_runtime := fn_default_runtime_config(G_BUILD_CONFIG);
        chip_active := default_runtime.tdc.active_chip_mask(G_CHIP_INDEX) = '1';
        rise_enabled := chip_active and
            (default_runtime.tdc.falling_enable = '0' or
             G_BUILD_CONFIG.rise_capability_mask(G_CHIP_INDEX) = '1');
        fall_enabled := chip_active and
            default_runtime.tdc.falling_enable = '1' and
            G_BUILD_CONFIG.fall_capability_mask(G_CHIP_INDEX) = '1';

        if active_valid = '1' then
            chip_active := active.source.tdc.active_chip_mask(G_CHIP_INDEX) = '1';
            rise_enabled := chip_active and
                active.derived.active_rise_mask(G_CHIP_INDEX) = '1';
            fall_enabled := chip_active and
                active.derived.active_fall_mask(G_CHIP_INDEX) = '1';
            result(5)(c_REG5_STARTOFF1_HI downto c_REG5_STARTOFF1_LO) :=
                std_logic_vector(active.source.tdc.start_offset);
        end if;

        result(5)(c_REG5_MASTER_ALU_TRIG) := '1';
        result(5)(c_REG5_PARTIAL_ALU_TRIG) := '0';

        if chip_active then
            result(0)(c_REG0_TSTART_RISE) := '1';
            result(0)(c_REG0_TSTART_FALL) := '0';
        else
            result(0)(c_REG0_TRISEEN_HI downto c_REG0_TRISEEN_LO) :=
                (others => '0');
            result(0)(c_REG0_TFALLEN_HI downto c_REG0_TFALLEN_LO) :=
                (others => '0');
        end if;

        for stop_index in 0 to
            work.lidar_build_pkg.C_MAX_STOPS_PER_CHIP - 1 loop
            if stop_index >= G_BUILD_CONFIG.stops_per_chip or
               not rise_enabled then
                result(0)(c_REG0_TSTOP_RISE_LO + stop_index) := '0';
            end if;
            if stop_index >= G_BUILD_CONFIG.stops_per_chip or
               not fall_enabled then
                result(0)(c_REG0_TSTOP_FALL_LO + stop_index) := '0';
            end if;
        end loop;

        return result;
    end function fn_prepared_image;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-GPX-LANE-001 invalid build configuration"
        severity failure;
    assert G_CHIP_INDEX < G_BUILD_CONFIG.num_chips
        report "V2-GPX-LANE-002 Chip index is outside the build topology"
        severity failure;
    assert C_DRAIN_CAP_QUADS <= 15
        report "V2-GPX-LANE-005 build-derived physical drain cap exceeds GPX field"
        severity failure;
    assert i_active_valid = '0' or
           i_active_config.derived.capture_window_tdc_clks(31 downto 16) = 0
        report "V2-GPX-LANE-003 accepted capture window exceeds 16-bit controller"
        severity failure;
    assert i_active_valid = '0' or
           i_active_config.source.tdc.bus_ticks(5 downto 3) = 0
        report "V2-GPX-LANE-004 accepted BUS_TICKS exceeds physical width"
        severity failure;

    legacy_cfg_c <= fn_legacy_config(
        i_active_valid, i_active_config, i_register_image);
    legacy_image_c <= fn_prepared_image(
        i_active_valid, i_active_config, i_register_image);

    capture_window_c <=
        i_active_config.derived.capture_window_tdc_clks(15 downto 0)
        when i_active_valid = '1' else (others => '0');

    o_config_ready <= '1' when state_r = LANE_IDLE and
                              i_active_valid = '1' and
                              controller_busy_c = '0' and
                              raw_valid_c = '0' else '0';
    o_config_done <= config_done_r;
    o_safe <= '1' when state_r = LANE_IDLE and
                       controller_busy_c = '0' and
                       bus_busy_c = '0' and
                       bus_response_pending_c = '0' and
                       raw_valid_c = '0' else '0';
    o_shot_ready <= '1' when state_r = LANE_ARMED and
                              i_active_valid = '1' and
                              i_run_enable = '1' else '0';

    run_active_c <= '1' when state_r = LANE_START_WAIT or
                             state_r = LANE_ARMED or
                             state_r = LANE_SHOT or
                             state_r = LANE_STOP_WAIT else '0';
    shot_outstanding_c <= '1' when state_r = LANE_SHOT else '0';

    p_event : process (all)
        variable result : gpx_raw_event_t;
        variable timeout_now : std_logic;
    begin
        result := C_GPX_RAW_EVENT_IDLE;
        timeout_now := timeout_seen_r or run_timeout_c;
        if raw_valid_c = '1' then
            result.valid := '1';
            result.chip_index := to_unsigned(
                G_CHIP_INDEX, result.chip_index'length);
            result.ififo_id := raw_user_c(0);
            result.raw_word := raw_data_c(C_GPX_BUS_DATA_WIDTH - 1 downto 0);
            result.faulted := raw_user_c(5);
            result.shot_context := shot_context_r;
            result.chip_shot_seq := shot_seq_start_r;

            if raw_user_c(7) = '0' then
                result.kind := GPX_RAW_DATA;
            elsif raw_user_c(0) = '0' then
                result.kind := GPX_RAW_IFIFO1_DONE;
                result.raw_word := (others => '0');
            elsif timeout_now = '1' then
                result.kind := GPX_RAW_TIMEOUT;
                result.raw_word := (others => '0');
                result.faulted := '1';
                if run_timeout_c = '1' then
                    result.timeout_cause := run_timeout_cause_c;
                else
                    result.timeout_cause := timeout_cause_r;
                end if;
            else
                result.kind := GPX_RAW_DRAIN_DONE;
                result.raw_word := (others => '0');
            end if;
        end if;
        o_event <= result;
    end process p_event;

    p_control : process (i_clk)
        variable terminal_now : boolean;
        variable sequence_now : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r <= LANE_BOOT;
                initialized_r <= '0';
                config_seen_busy_r <= '0';
                config_done_r <= '0';
                stop_sent_r <= '0';
                cmd_start_r <= '0';
                cmd_stop_r <= '0';
                cmd_cfg_write_r <= '0';
                shot_start_r <= '0';
                shot_context_r <= C_SHOT_START_EVENT_IDLE;
                shot_seq_start_r <= (others => '0');
                terminal_seen_r <= '0';
                sequence_advanced_r <= '0';
                timeout_seen_r <= '0';
                timeout_cause_r <= (others => '0');
            else
                config_done_r <= '0';
                cmd_start_r <= '0';
                cmd_stop_r <= '0';
                cmd_cfg_write_r <= '0';
                shot_start_r <= '0';

                terminal_now := raw_valid_c = '1' and
                    i_event_ready = '1' and raw_user_c(7) = '1' and
                    raw_user_c(0) = '1';
                sequence_now := chip_shot_seq_c /= shot_seq_start_r;

                if run_timeout_c = '1' then
                    timeout_seen_r <= '1';
                    timeout_cause_r <= run_timeout_cause_c;
                end if;

                case state_r is
                    when LANE_BOOT =>
                        if controller_busy_c = '0' then
                            initialized_r <= '1';
                            state_r <= LANE_IDLE;
                        end if;

                    when LANE_IDLE =>
                        stop_sent_r <= '0';
                        if i_config_apply = '1' then
                            cmd_cfg_write_r <= '1';
                            config_seen_busy_r <= '0';
                            state_r <= LANE_CONFIG_WAIT;
                        elsif i_run_enable = '1' and i_active_valid = '1' then
                            cmd_start_r <= '1';
                            state_r <= LANE_START_WAIT;
                        end if;

                    when LANE_CONFIG_WAIT =>
                        if controller_busy_c = '1' then
                            config_seen_busy_r <= '1';
                        elsif config_seen_busy_r = '1' then
                            config_done_r <= '1';
                            config_seen_busy_r <= '0';
                            state_r <= LANE_IDLE;
                        end if;

                    when LANE_START_WAIT =>
                        if i_run_enable = '0' then
                            cmd_stop_r <= '1';
                            state_r <= LANE_STOP_WAIT;
                        elsif controller_busy_c = '1' then
                            state_r <= LANE_ARMED;
                        end if;

                    when LANE_ARMED =>
                        if i_run_enable = '0' then
                            cmd_stop_r <= '1';
                            state_r <= LANE_STOP_WAIT;
                        elsif i_shot.valid = '1' then
                            shot_start_r <= '1';
                            shot_context_r <= i_shot;
                            shot_seq_start_r <= chip_shot_seq_c;
                            terminal_seen_r <= '0';
                            sequence_advanced_r <= '0';
                            timeout_seen_r <= '0';
                            timeout_cause_r <= (others => '0');
                            stop_sent_r <= '0';
                            state_r <= LANE_SHOT;
                        end if;

                    when LANE_SHOT =>
                        if terminal_now then
                            terminal_seen_r <= '1';
                        end if;
                        if sequence_now then
                            sequence_advanced_r <= '1';
                        end if;

                        if i_run_enable = '0' and stop_sent_r = '0' then
                            cmd_stop_r <= '1';
                            stop_sent_r <= '1';
                        end if;

                        if i_run_enable = '1' and
                           (terminal_seen_r = '1' or terminal_now) and
                           (sequence_advanced_r = '1' or sequence_now) then
                            terminal_seen_r <= '0';
                            sequence_advanced_r <= '0';
                            state_r <= LANE_ARMED;
                        elsif i_run_enable = '0' and
                              (terminal_seen_r = '1' or terminal_now) and
                              controller_busy_c = '0' then
                            terminal_seen_r <= '0';
                            sequence_advanced_r <= '0';
                            state_r <= LANE_IDLE;
                        end if;

                    when LANE_STOP_WAIT =>
                        if controller_busy_c = '0' then
                            state_r <= LANE_IDLE;
                        end if;
                end case;

                if i_soft_reset = '1' or i_force_reinit = '1' then
                    state_r <= LANE_BOOT;
                    initialized_r <= '0';
                    shot_context_r <= C_SHOT_START_EVENT_IDLE;
                    terminal_seen_r <= '0';
                    sequence_advanced_r <= '0';
                    timeout_seen_r <= '0';
                    stop_sent_r <= '0';
                end if;

                -- synthesis translate_off
                assert not (i_config_apply = '1' and o_config_ready = '0')
                    report "V2-GPX-LANE-006 config apply while lane is not ready"
                    severity warning;
                assert not (raw_valid_c = '1' and
                            shot_context_r.valid = '0')
                    report "V2-GPX-LANE-007 raw event has no Shot context"
                    severity failure;
                -- synthesis translate_on
            end if;
        end if;
    end process p_control;

    -- The legacy controller's o_err_drain_cap reports only the response-bus
    -- quarantine cap. Preserve the physical IFIFO output-cap event as a
    -- sticky as well; otherwise the final faulted terminal beat is the only
    -- evidence after downstream consumption. Event assertion wins a
    -- simultaneous software clear.
    p_physical_drain_cap_status : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                physical_drain_cap_sticky_r <= '0';
            else
                if i_clear_status = '1' then
                    physical_drain_cap_sticky_r <= '0';
                end if;
                if raw_valid_c = '1' and raw_user_c(7) = '1' and
                   raw_user_c(0) = '1' and raw_user_c(5) = '1' then
                    physical_drain_cap_sticky_r <= '1';
                end if;
            end if;
        end if;
    end process p_physical_drain_cap_status;

    u_controller : entity work.tdc_gpx_chip_ctrl
        generic map (
            g_BUS_DATA_WIDTH => C_GPX_BUS_DATA_WIDTH,
            g_CHIP_ID => G_CHIP_INDEX,
            g_POWERUP_CLKS => C_POWERUP_CLKS,
            g_RECOVERY_CLKS => C_RECOVERY_CLKS,
            g_ALU_PULSE_CLKS => C_ALU_PULSE_CLKS,
            g_BUS_IDLE_STABLE_CLKS => C_BUS_IDLE_STABLE_CLKS,
            g_DRAIN_MARGIN_CLKS => C_DRAIN_MARGIN_CLKS,
            g_EF_SYNC_GUARD_CLKS => C_EF_SYNC_GUARD_CLKS
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_cfg => legacy_cfg_c,
            i_cfg_image => legacy_image_c,
            i_cmd_start => cmd_start_r,
            i_cmd_stop => cmd_stop_r,
            i_cmd_soft_reset => i_soft_reset,
            i_cmd_soft_reset_err => '0',
            i_cmd_force_reinit => i_force_reinit,
            i_cmd_cfg_write => cmd_cfg_write_r,
            i_soft_clear => i_clear_status,
            i_cmd_reg_read => '0',
            i_cmd_reg_write => '0',
            i_cmd_reg_addr => (others => '0'),
            i_cmd_reg_wdata => (others => '0'),
            o_cmd_reg_rdata => open,
            o_cmd_reg_rvalid => open,
            o_cmd_reg_done => open,
            i_shot_start => shot_start_r,
            i_max_range_tdc_clks => capture_window_c,
            i_stop_tdc => i_stop_tdc,
            o_bus_req_valid => bus_request_c.valid,
            o_bus_req_rw => bus_request_c.write,
            o_bus_req_addr => bus_request_c.address,
            o_bus_req_wdata => bus_request_c.write_data,
            o_bus_oen_permanent => bus_request_c.oen_permanent,
            o_bus_req_burst => bus_request_c.burst,
            o_bus_clk_div_snap => bus_timing_c.clock_div,
            o_bus_ticks_snap => bus_timing_c.ticks,
            i_s_axis_tvalid => bus_response_c.valid,
            i_s_axis_tdata =>
                "0000" & bus_response_c.read_data,
            i_s_axis_tuser =>
                "000" & bus_response_c.address & bus_response_c.write_ack,
            o_s_axis_tready => bus_response_ready_c,
            i_bus_busy => bus_busy_c,
            i_bus_rsp_pending => bus_response_pending_c,
            i_ef1_sync => pin_status_c.ef1,
            i_ef2_sync => pin_status_c.ef2,
            i_irflag_sync => pin_status_c.irflag,
            i_lf1_sync => pin_status_c.lf1,
            i_lf2_sync => pin_status_c.lf2,
            o_tick_en => tick_en_c,
            o_stopdis => o_stopdis,
            o_alutrigger => o_alutrigger,
            o_puresn => o_puresn,
            o_m_raw_axis_tvalid => raw_valid_c,
            o_m_raw_axis_tdata => raw_data_c,
            o_m_raw_axis_tuser => raw_user_c,
            i_m_raw_axis_tready => i_event_ready,
            o_drain_done => drain_done_c,
            o_run_drain_complete => run_drain_complete_c,
            o_shot_seq => chip_shot_seq_c,
            o_busy => controller_busy_c,
            o_err_drain_timeout => err_drain_timeout_c,
            o_err_sequence => err_sequence_c,
            o_err_rsp_mismatch => err_rsp_mismatch_c,
            o_err_raw_overflow => err_raw_overflow_c,
            o_err_raw_drop => err_raw_drop_c,
            o_err_drain_cap => err_drain_cap_c,
            o_err_reg_overflow => err_reg_overflow_c,
            o_run_timeout => run_timeout_c,
            o_run_timeout_cause => run_timeout_cause_c,
            o_init_cfg_coalesced => init_cfg_coalesced_c,
            o_err_cmd_collision => err_cmd_collision_c,
            o_err_bus_fatal => err_bus_fatal_c
        );

    u_bus : entity work.lidar_gpx_bus_engine
        generic map (
            G_TDC_CLK_MHZ => G_BUILD_CONFIG.tdc_clk_mhz,
            G_OEN_MODE => G_OEN_MODE
        )
        port map (
            i_clk => i_clk,
            i_rst_n => i_rst_n,
            i_tick_en => tick_en_c,
            i_timing => bus_timing_c,
            i_request => bus_request_c,
            o_busy => bus_busy_c,
            o_response_pending => bus_response_pending_c,
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
            o_response => bus_response_c,
            i_response_ready => bus_response_ready_c,
            o_status => pin_status_c,
            o_effective_ticks => effective_ticks_c
        );

    o_status <= (
        initialized      => initialized_r,
        run_active       => run_active_c,
        shot_outstanding => shot_outstanding_c,
        controller_busy  => controller_busy_c,
        bus_busy         => bus_busy_c,
        response_pending => bus_response_pending_c,
        pin_status       => pin_status_c,
        effective_ticks  => effective_ticks_c,
        chip_shot_seq    => chip_shot_seq_c
    );

    o_faults <= (
        drain_timeout_pulse       => err_drain_timeout_c,
        sequence_pulse            => err_sequence_c,
        response_mismatch_sticky  => err_rsp_mismatch_c,
        raw_drop_sticky           => err_raw_drop_c,
        -- Summary of either immutable physical output-cap truncation or the
        -- legacy response-bus quarantine cap.
        drain_cap_sticky          => err_drain_cap_c or
                                     physical_drain_cap_sticky_r,
        register_overflow_sticky  => err_reg_overflow_c,
        run_timeout_pulse         => run_timeout_c,
        run_timeout_cause         => run_timeout_cause_c,
        init_cfg_coalesced_sticky => init_cfg_coalesced_c,
        command_collision_sticky  => err_cmd_collision_c,
        bus_fatal_sticky          => err_bus_fatal_c
    );

end architecture rtl;
