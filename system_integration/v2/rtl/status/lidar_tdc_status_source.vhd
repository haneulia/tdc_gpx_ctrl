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

        o_irq_gpx_transport : out std_logic
    );
end entity lidar_tdc_status_source;

architecture rtl of lidar_tdc_status_source is

    type source_state_t is (SOURCE_IDLE, SOURCE_BUILD, SOURCE_RESPONSE);
    type timeout_cause_array_t is array (0 to C_MAX_CHIPS - 1) of
        std_logic_vector(2 downto 0);
    signal state_r : source_state_t := SOURCE_IDLE;
    signal index_r : lidar_diag_index_t := (others => '0');
    signal response_r : lidar_diag_response_t := (others => '0');

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
            drain_timeout_seen_r <= (others => '0');
            sequence_seen_r <= (others => '0');
            run_timeout_seen_r <= (others => '0');
            run_timeout_cause_r <= (others => (others => '0'));
            irq_transport_r <= '0';
        elsif rising_edge(i_clk) then
            if i_clear_status = '1' then
                drain_timeout_seen_r <= (others => '0');
                sequence_seen_r <= (others => '0');
                run_timeout_seen_r <= (others => '0');
                run_timeout_cause_r <= (others => (others => '0'));
                irq_transport_r <= '0';
            else
                v_transport_fault := '0';
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
                    v_transport_fault := v_transport_fault or
                        i_lane_faults(lane).drain_timeout_pulse or
                        i_lane_faults(lane).sequence_pulse or
                        i_lane_faults(lane).response_mismatch_sticky or
                        i_lane_faults(lane).raw_drop_sticky or
                        i_lane_faults(lane).drain_cap_sticky or
                        i_lane_faults(lane).register_overflow_sticky or
                        i_lane_faults(lane).run_timeout_pulse or
                        i_lane_faults(lane).init_cfg_coalesced_sticky or
                        i_lane_faults(lane).command_collision_sticky or
                        i_lane_faults(lane).bus_fatal_sticky or
                        drain_timeout_seen_r(lane) or
                        sequence_seen_r(lane) or
                        run_timeout_seen_r(lane);
                end loop;
                if v_transport_fault = '1' then
                    irq_transport_r <= '1';
                end if;
            end if;

            case state_r is
                when SOURCE_IDLE =>
                    if i_request_valid = '1' then
                        index_r <= i_request_index;
                        state_r <= SOURCE_BUILD;
                    end if;

                when SOURCE_BUILD =>
                    v_index := to_integer(unsigned(index_r));
                    v_data := (others => '0');
                    v_error := '0';

                    case v_index is
                        when C_DIAG_TDC_SUMMARY =>
                            v_data(3 downto 0) := i_active_mask;
                            v_data(7 downto 4) := i_terminal_mask;
                            v_data(10) := i_tdc_safe;
                            v_data(12) := i_run_enable;
                            v_data(13) := i_active_valid;
                            v_data(14) := i_config_ready;

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

                when SOURCE_RESPONSE =>
                    if i_response_ready = '1' then
                        state_r <= SOURCE_IDLE;
                    end if;
            end case;
        end if;
    end process p_source;

end architecture rtl;
