library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_status_pkg.all;

-- TDC-domain owner for GPX lane status. Pulse-only controller faults are
-- retained locally until CLEAR_STATUS so both indexed software reads and the
-- CSR-domain interrupt synchronizer cannot miss a one-clock event.
entity lidar_tdc_status_source is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_clear_status : in  std_logic;

        i_request_valid : in  std_logic;
        o_request_ready : out std_logic;
        i_request_index : in  lidar_diag_index_t;
        o_response_valid : out std_logic;
        i_response_ready : in  std_logic;
        o_response       : out lidar_diag_response_t;

        i_active_mask   : in chip_mask_t;
        i_terminal_mask : in chip_mask_t;
        i_lane_status   : in gpx_lane_status_array_t;
        i_lane_faults   : in gpx_lane_faults_array_t;
        i_tdc_safe       : in std_logic;
        i_run_enable     : in std_logic;
        i_active_valid   : in std_logic;
        i_config_ready   : in std_logic;

        -- CTL23의 11CCAAAA 요청을 실제 GPX bus read로 변환한다. pause는
        -- acquisition RUN만 잠시 내리며 Processing/CSR clock에는 쓰지 않는다.
        o_register_service_pause : out std_logic;
        o_register_read : out gpx_register_read_request_t;
        i_register_read_ready : in std_logic := '0';
        i_register_read_response : in gpx_register_read_response_t :=
            C_GPX_REGISTER_READ_RESPONSE_IDLE;
        o_register_read_response_ready : out std_logic;

        o_irq_gpx_transport : out std_logic
    );
end entity lidar_tdc_status_source;

architecture rtl of lidar_tdc_status_source is

    constant C_REGISTER_SERVICE_TIMEOUT_CLKS : positive :=
        work.tdc_gpx_pkg.fn_time_ns_to_clks_ceil(
            C_GPX_REGISTER_SERVICE_TIMEOUT_NS,
            G_BUILD_CONFIG.tdc_clk_mhz);

    type source_state_t is (
        SOURCE_IDLE,
        SOURCE_BUILD,
        SOURCE_REGISTER_WAIT_SAFE,
        SOURCE_REGISTER_REQUEST,
        SOURCE_REGISTER_RESPONSE,
        SOURCE_RESPONSE
    );
    type timeout_cause_array_t is array (0 to C_MAX_CHIPS - 1) of
        std_logic_vector(2 downto 0);
    signal state_r : source_state_t := SOURCE_IDLE;
    signal index_r : lidar_diag_index_t := (others => '0');
    signal response_r : lidar_diag_response_t := (others => '0');
    signal register_request_c : gpx_register_read_request_t :=
        C_GPX_REGISTER_READ_REQUEST_IDLE;
    signal register_service_pause_r : std_logic := '0';
    signal register_service_count_r : natural range 0 to
        C_REGISTER_SERVICE_TIMEOUT_CLKS := 0;
    signal register_service_error_seen_r : std_logic := '0';

    signal drain_timeout_seen_r : chip_mask_t := (others => '0');
    signal sequence_seen_r      : chip_mask_t := (others => '0');
    signal run_timeout_seen_r   : chip_mask_t := (others => '0');
    signal run_timeout_cause_r  : timeout_cause_array_t :=
        (others => (others => '0'));
    signal irq_transport_r      : std_logic := '0';

begin

    o_request_ready <= '1' when state_r = SOURCE_IDLE else '0';
    o_response_valid <= '1' when state_r = SOURCE_RESPONSE else '0';
    o_response <= response_r;
    o_irq_gpx_transport <= irq_transport_r;
    o_register_service_pause <= register_service_pause_r;
    o_register_read <= register_request_c;
    o_register_read_response_ready <= '1' when
        state_r = SOURCE_REGISTER_RESPONSE else '0';

    p_register_request : process (all)
        variable result : gpx_register_read_request_t;
    begin
        result := C_GPX_REGISTER_READ_REQUEST_IDLE;
        result.chip := fn_diag_gpx_register_chip(index_r);
        result.address := fn_diag_gpx_register_address(index_r);
        if state_r = SOURCE_REGISTER_REQUEST then
            result.valid := '1';
        end if;
        register_request_c <= result;
    end process p_register_request;

    p_source : process (i_clk, i_rst_n)
        variable v_index : natural;
        variable v_lane  : natural;
        variable v_data  : lidar_diag_word_t;
        variable v_error : std_logic;
        variable v_transport_fault : std_logic;
    begin
        if i_rst_n = '0' then
            state_r <= SOURCE_IDLE;
            index_r <= (others => '0');
            response_r <= (others => '0');
            register_service_pause_r <= '0';
            register_service_count_r <= 0;
            register_service_error_seen_r <= '0';
            drain_timeout_seen_r <= (others => '0');
            sequence_seen_r <= (others => '0');
            run_timeout_seen_r <= (others => '0');
            run_timeout_cause_r <= (others => (others => '0'));
            irq_transport_r <= '0';
        elsif rising_edge(i_clk) then
            v_transport_fault := '0';
            -- CLEAR_STATUS는 이력만 지우며 진행 중 portal transaction은
            -- 유지한다. Pulse형 새 사건은 같은 clock의 clear보다 우선한다.
            if i_clear_status = '1' then
                drain_timeout_seen_r <= (others => '0');
                sequence_seen_r <= (others => '0');
                run_timeout_seen_r <= (others => '0');
                run_timeout_cause_r <= (others => (others => '0'));
                register_service_error_seen_r <= '0';
                irq_transport_r <= '0';
            else
                v_transport_fault := register_service_error_seen_r;
                for lane in 0 to G_BUILD_CONFIG.num_chips - 1 loop
                    v_transport_fault := v_transport_fault or
                        i_lane_faults(lane).response_mismatch_sticky or
                        i_lane_faults(lane).raw_drop_sticky or
                        i_lane_faults(lane).drain_cap_sticky or
                        i_lane_faults(lane).register_overflow_sticky or
                        i_lane_faults(lane).init_cfg_coalesced_sticky or
                        i_lane_faults(lane).command_collision_sticky or
                        i_lane_faults(lane).bus_fatal_sticky or
                        drain_timeout_seen_r(lane) or
                        sequence_seen_r(lane) or
                        run_timeout_seen_r(lane);
                end loop;
            end if;

            -- Pulse event는 clear와 겹쳐도 보존한다. 반면 level sticky는
            -- owner가 같은 edge에서 clear되기 전의 이전 값을 보이므로,
            -- clear clock에는 위 old-level 집계에서 의도적으로 제외한다.
            for lane in 0 to G_BUILD_CONFIG.num_chips - 1 loop
                if i_lane_faults(lane).drain_timeout_pulse = '1' then
                    drain_timeout_seen_r(lane) <= '1';
                end if;
                if i_lane_faults(lane).sequence_pulse = '1' then
                    sequence_seen_r(lane) <= '1';
                end if;
                if i_lane_faults(lane).run_timeout_pulse = '1' then
                    run_timeout_seen_r(lane) <= '1';
                    run_timeout_cause_r(lane) <=
                        i_lane_faults(lane).run_timeout_cause;
                end if;
                if i_lane_faults(lane).drain_timeout_pulse = '1' or
                   i_lane_faults(lane).sequence_pulse = '1' or
                   i_lane_faults(lane).run_timeout_pulse = '1' then
                    v_transport_fault := '1';
                end if;
            end loop;

            if v_transport_fault = '1' then
                irq_transport_r <= '1';
            end if;

            case state_r is
                when SOURCE_IDLE =>
                    if i_request_valid = '1' then
                        index_r <= i_request_index;
                        if fn_diag_is_gpx_register_read(i_request_index) then
                            if to_integer(fn_diag_gpx_register_chip(
                                    i_request_index)) <
                                    G_BUILD_CONFIG.num_chips then
                                -- DISARM 뒤 새 Shot이 없는 상태에서 GPX
                                -- acquisition만 정지시키고 bus idle을 기다린다.
                                register_service_pause_r <= '1';
                                register_service_count_r <= 0;
                                state_r <= SOURCE_REGISTER_WAIT_SAFE;
                            else
                                v_data := fn_pack_gpx_register_read_word(
                                    fn_diag_gpx_register_address(
                                        i_request_index),
                                    (others => '0'));
                                response_r <= fn_pack_diag_response(
                                    v_data, '1');
                                state_r <= SOURCE_RESPONSE;
                            end if;
                        else
                            state_r <= SOURCE_BUILD;
                        end if;
                    end if;

                when SOURCE_BUILD =>
                    v_index := to_integer(unsigned(index_r));
                    v_data := (others => '0');
                    v_error := '0';

                    case v_index is
                        when C_DIAG_TDC_SUMMARY =>
                            v_data(C_TDC_SUMMARY_ACTIVE_MASK_MSB downto
                                C_TDC_SUMMARY_ACTIVE_MASK_LSB) :=
                                i_active_mask;
                            v_data(C_TDC_SUMMARY_TERMINAL_MASK_MSB downto
                                C_TDC_SUMMARY_TERMINAL_MASK_LSB) :=
                                i_terminal_mask;
                            v_data(C_TDC_SUMMARY_SAFE_BIT) := i_tdc_safe;
                            v_data(C_TDC_SUMMARY_RUN_ENABLE_BIT) :=
                                i_run_enable;
                            v_data(C_TDC_SUMMARY_ACTIVE_VALID_BIT) :=
                                i_active_valid;
                            v_data(C_TDC_SUMMARY_CONFIG_READY_BIT) :=
                                i_config_ready;
                            v_data(C_TDC_SUMMARY_REGISTER_READ_ERROR_BIT) :=
                                register_service_error_seen_r;

                        when C_DIAG_TDC_LANE_STATUS_0 to
                             C_DIAG_TDC_LANE_STATUS_3 =>
                            v_lane := v_index - C_DIAG_TDC_LANE_STATUS_0;
                            if v_lane < G_BUILD_CONFIG.num_chips then
                                v_data(0) :=
                                    i_lane_status(v_lane).initialized;
                                v_data(1) :=
                                    i_lane_status(v_lane).run_active;
                                v_data(2) :=
                                    i_lane_status(v_lane).shot_outstanding;
                                v_data(3) :=
                                    i_lane_status(v_lane).controller_busy;
                                v_data(4) :=
                                    i_lane_status(v_lane).bus_busy;
                                v_data(5) :=
                                    i_lane_status(v_lane).response_pending;
                                v_data(6) :=
                                    i_lane_status(v_lane).pin_status.ef1;
                                v_data(7) :=
                                    i_lane_status(v_lane).pin_status.ef2;
                                v_data(8) :=
                                    i_lane_status(v_lane).pin_status.lf1;
                                v_data(9) :=
                                    i_lane_status(v_lane).pin_status.lf2;
                                v_data(10) :=
                                    i_lane_status(v_lane).pin_status.irflag;
                                v_data(11) :=
                                    i_lane_status(v_lane).pin_status.errflag;
                                v_data(14 downto 12) := std_logic_vector(
                                    i_lane_status(v_lane).effective_ticks);
                                v_data(31 downto 16) := std_logic_vector(
                                    i_lane_status(v_lane).chip_shot_seq);
                            end if;

                        when C_DIAG_TDC_LANE_FAULT_0 to
                             C_DIAG_TDC_LANE_FAULT_3 =>
                            v_lane := v_index - C_DIAG_TDC_LANE_FAULT_0;
                            if v_lane < G_BUILD_CONFIG.num_chips then
                                v_data(0) := drain_timeout_seen_r(v_lane);
                                v_data(1) := sequence_seen_r(v_lane);
                                v_data(2) := i_lane_faults(v_lane).
                                    response_mismatch_sticky;
                                v_data(3) := i_lane_faults(v_lane).
                                    raw_drop_sticky;
                                v_data(4) := i_lane_faults(v_lane).
                                    drain_cap_sticky;
                                v_data(5) := i_lane_faults(v_lane).
                                    register_overflow_sticky;
                                v_data(6) := run_timeout_seen_r(v_lane);
                                v_data(9 downto 7) :=
                                    run_timeout_cause_r(v_lane);
                                v_data(10) := i_lane_faults(v_lane).
                                    init_cfg_coalesced_sticky;
                                v_data(11) := i_lane_faults(v_lane).
                                    command_collision_sticky;
                                v_data(12) := i_lane_faults(v_lane).
                                    bus_fatal_sticky;
                            end if;

                        when others =>
                            v_error := '1';
                    end case;

                    response_r <= fn_pack_diag_response(v_data, v_error);
                    state_r <= SOURCE_RESPONSE;

                when SOURCE_REGISTER_WAIT_SAFE =>
                    if i_tdc_safe = '1' then
                        register_service_count_r <= 0;
                        state_r <= SOURCE_REGISTER_REQUEST;
                    elsif register_service_count_r =
                          C_REGISTER_SERVICE_TIMEOUT_CLKS - 1 then
                        v_data := fn_pack_gpx_register_read_word(
                            fn_diag_gpx_register_address(index_r),
                            (others => '0'));
                        response_r <= fn_pack_diag_response(v_data, '1');
                        register_service_pause_r <= '0';
                        register_service_error_seen_r <= '1';
                        irq_transport_r <= '1';
                        state_r <= SOURCE_RESPONSE;
                    else
                        register_service_count_r <=
                            register_service_count_r + 1;
                    end if;

                when SOURCE_REGISTER_REQUEST =>
                    if i_register_read_ready = '1' then
                        register_service_count_r <= 0;
                        state_r <= SOURCE_REGISTER_RESPONSE;
                    elsif register_service_count_r =
                          C_REGISTER_SERVICE_TIMEOUT_CLKS - 1 then
                        v_data := fn_pack_gpx_register_read_word(
                            fn_diag_gpx_register_address(index_r),
                            (others => '0'));
                        response_r <= fn_pack_diag_response(v_data, '1');
                        register_service_pause_r <= '0';
                        register_service_error_seen_r <= '1';
                        irq_transport_r <= '1';
                        state_r <= SOURCE_RESPONSE;
                    else
                        register_service_count_r <=
                            register_service_count_r + 1;
                    end if;

                when SOURCE_REGISTER_RESPONSE =>
                    if i_register_read_response.valid = '1' then
                        v_error := i_register_read_response.error;
                        if i_register_read_response.chip /=
                               fn_diag_gpx_register_chip(index_r) or
                           i_register_read_response.address /=
                               fn_diag_gpx_register_address(index_r) then
                            v_error := '1';
                        end if;
                        v_data := fn_pack_gpx_register_read_word(
                            fn_diag_gpx_register_address(index_r),
                            i_register_read_response.read_data);
                        response_r <= fn_pack_diag_response(v_data, v_error);
                        register_service_pause_r <= '0';
                        if v_error = '1' then
                            register_service_error_seen_r <= '1';
                            irq_transport_r <= '1';
                        end if;
                        state_r <= SOURCE_RESPONSE;
                    elsif register_service_count_r =
                          C_REGISTER_SERVICE_TIMEOUT_CLKS - 1 then
                        v_data := fn_pack_gpx_register_read_word(
                            fn_diag_gpx_register_address(index_r),
                            (others => '0'));
                        response_r <= fn_pack_diag_response(v_data, '1');
                        register_service_pause_r <= '0';
                        register_service_error_seen_r <= '1';
                        irq_transport_r <= '1';
                        state_r <= SOURCE_RESPONSE;
                    else
                        register_service_count_r <=
                            register_service_count_r + 1;
                    end if;

                when SOURCE_RESPONSE =>
                    if i_response_ready = '1' then
                        state_r <= SOURCE_IDLE;
                    end if;
            end case;
        end if;
    end process p_source;

end architecture rtl;
