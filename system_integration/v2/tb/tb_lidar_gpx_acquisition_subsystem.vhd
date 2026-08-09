-- ============================================================================
-- 테스트 자산 목적: 외부 GPX pin부터 merged B5 raw stream까지 acquisition 전체를 검증한다.
-- 핵심 검증 계약: 32 physical STOP lane, Chip/IFIFO/raw28/Shot identity와 fault 진단이다.
-- 관련 RTL: lidar_gpx_acquisition_subsystem과 bus/lane/coordinator/gateway 계층.
-- 실행 회귀: scripts/run_v2_gpx_acquisition_subsystem.ps1
-- 유지보수 주의: 150/200·200/150 CDC와 정상/timeout/cap/backpressure 시나리오를 유지한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.tdc_gpx_pkg.all;

entity tb_lidar_gpx_acquisition_subsystem is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200
    );
end entity tb_lidar_gpx_acquisition_subsystem;

architecture sim of tb_lidar_gpx_acquisition_subsystem is

    constant C_CHIPS : positive := 4;
    constant C_STOPS_PER_CHIP : positive := 8;
    constant C_STOPS_PER_IFIFO : positive := 4;
    constant C_RETURNS_PER_STOP : positive := 7;
    constant C_WORDS_PER_IFIFO : positive :=
        C_STOPS_PER_IFIFO * C_RETURNS_PER_STOP;
    constant C_NORMAL_EVENTS_PER_CHIP : positive :=
        2 * C_WORDS_PER_IFIFO + 2;
    constant C_NORMAL_TOTAL_EVENTS : positive :=
        C_CHIPS * C_NORMAL_EVENTS_PER_CHIP;
    constant C_TIMEOUT_TOTAL_EVENTS : positive := C_CHIPS;
    constant C_CAP_EVENTS_PER_CHIP : positive :=
        C_WORDS_PER_IFIFO + 3;
    constant C_CAP_TOTAL_EVENTS : positive :=
        C_CHIPS * C_CAP_EVENTS_PER_CHIP;

    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => C_CHIPS,
        stops_per_chip         => C_STOPS_PER_CHIP,
        max_returns_per_stop   => C_RETURNS_PER_STOP,
        rise_capability_mask   => "0011",
        fall_capability_mask   => "1100",
        output_width           => 32,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    constant C_DUAL_EDGE_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => C_CHIPS,
        stops_per_chip         => C_STOPS_PER_CHIP,
        max_returns_per_stop   => C_RETURNS_PER_STOP,
        rise_capability_mask   => "1111",
        fall_capability_mask   => "1111",
        output_width           => 32,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    type natural_array_t is array (0 to C_CHIPS - 1) of
        natural range 0 to 65535;
    type logic_array_t is array (0 to C_CHIPS - 1) of std_logic;

    function fn_active_config return lidar_active_config_t is
        variable runtime : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        runtime.tdc.active_chip_mask := "1111";
        runtime.tdc.falling_enable := '1';
        result.version := to_unsigned(21, 16);
        result.source := runtime;
        result.derived := fn_derive_runtime_config(C_BUILD_CONFIG, runtime);
        return result;
    end function fn_active_config;

    function fn_shot(scenario : natural) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(2, 3);
        result.request.position := to_unsigned(
            3000 + scenario * 100, C_POSITION_WIDTH);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(100 + scenario, 16);
        result.request.last_in_face := '0';
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(8, 8);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(21, 16);
        result.fire_to_t0_clks := to_unsigned(4, 32);
        return result;
    end function fn_shot;

    function fn_raw_word(
        chip_index : natural;
        ififo_id   : std_logic;
        word_index : natural
    ) return gpx_bus_data_t is
        variable result : gpx_bus_data_t := (others => '0');
        variable hit_value : natural;
    begin
        result(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(word_index mod 4, 2));
        result(c_RAW_STARTNUM_HI downto c_RAW_STARTNUM_LO) :=
            (others => '0');
        if chip_index < 2 then
            result(c_RAW_SLOPE_BIT) := '1';
        else
            result(c_RAW_SLOPE_BIT) := '0';
        end if;
        hit_value := chip_index * 16#2000# + word_index;
        if ififo_id = '1' then
            hit_value := hit_value + 16#0800#;
        end if;
        result(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, c_RAW_HIT_WIDTH));
        return result;
    end function fn_raw_word;

    function fn_expected_chip_shot_seq(
        scenario : natural
    ) return natural is
    begin
        -- The timeout scenario terminates through run stop and therefore
        -- does not advance the controller's clean-completion sequence.
        if scenario = 2 then
            return 1;
        end if;
        return scenario;
    end function fn_expected_chip_shot_seq;

    function fn_all_initialized(
        value : gpx_lane_status_array_t
    ) return boolean is
    begin
        for index in 0 to C_CHIPS - 1 loop
            if value(index).initialized /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_all_initialized;

    function fn_all_running(
        value : gpx_lane_status_array_t
    ) return boolean is
    begin
        for index in 0 to C_CHIPS - 1 loop
            if value(index).run_active /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_all_running;

    function fn_all_shot_outstanding(
        value : gpx_lane_status_array_t
    ) return boolean is
    begin
        for index in 0 to C_CHIPS - 1 loop
            if value(index).shot_outstanding /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_all_shot_outstanding;

    signal proc_clk : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_clk : std_logic := '0';
    signal tdc_rst_n : std_logic := '0';
    signal simulation_done : boolean := false;

    signal proc_shot : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal proc_shot_ready : std_logic;
    signal proc_stop_tdc : std_logic := '0';
    signal proc_clear_status : std_logic := '0';
    signal proc_result : gpx_raw_event_t;
    signal proc_result_ready : std_logic := '1';
    signal shot_drop_sticky : std_logic;
    signal stop_drop_sticky : std_logic;

    signal active_config : lidar_active_config_t := fn_active_config;
    signal config_apply : std_logic := '0';
    signal config_ready : std_logic;
    signal config_done : std_logic;
    signal run_enable : std_logic := '0';
    signal tdc_clear_status : std_logic := '0';
    signal safe : std_logic;
    signal shot_complete : std_logic;
    signal cdc_reset_busy : std_logic;

    signal adr : gpx_bus_address_array_t;
    signal csn : chip_mask_t;
    signal rdn : chip_mask_t;
    signal wrn : chip_mask_t;
    signal oen : chip_mask_t;
    signal d_from_dut : gpx_bus_data_array_t;
    signal d_tri : gpx_bus_data_array_t;
    signal d_bus : gpx_bus_data_array_t := (others => (others => 'Z'));
    signal chip_d_out : gpx_bus_data_array_t := (others => (others => '0'));
    signal chip_d_oe : chip_mask_t := (others => '0');
    signal ef1 : chip_mask_t;
    signal ef2 : chip_mask_t;
    signal lf1 : chip_mask_t;
    signal lf2 : chip_mask_t;
    signal irflag : chip_mask_t := (others => '0');
    signal errflag : chip_mask_t := (others => '0');
    signal stopdis : chip_mask_t;
    signal alutrigger : chip_mask_t;
    signal puresn : chip_mask_t;
    signal active_mask : chip_mask_t;
    signal terminal_mask : chip_mask_t;
    signal status : gpx_lane_status_array_t;
    signal faults : gpx_lane_faults_array_t;

    signal fifo1_fill : natural_array_t := (others => 0);
    signal fifo2_fill : natural_array_t := (others => 0);
    signal fifo1_read_index : natural_array_t := (others => 0);
    signal fifo2_read_index : natural_array_t := (others => 0);
    signal fifo_load : std_logic := '0';
    signal fifo_load_1 : natural range 0 to 64 := 0;
    signal fifo_load_2 : natural range 0 to 64 := 0;
    signal empty_read_count : natural_array_t := (others => 0);

    signal score_scenario : natural range 0 to 2 := 0;
    signal score_reset : std_logic := '0';
    signal event_count : natural_array_t := (others => 0);
    signal total_event_count : natural := 0;
    signal completion_count : natural := 0;
    signal sequence_seen : std_logic := '0';
    signal monitor_clear : std_logic := '0';

begin

    gen_capacity_contract : for chip_index in 0 to C_CHIPS - 1 generate
    begin
        assert fn_gpx_drain_cap_quads(C_BUILD_CONFIG, chip_index) = 7
            report "V2-GPX-H3-TB build-derived drain cap mismatch"
            severity failure;
        assert fn_gpx_drain_cap_quads(
                   C_DUAL_EDGE_BUILD_CONFIG, chip_index) = 14
            report "V2-GPX-H3-TB dual-edge drain cap mismatch"
            severity failure;
    end generate gen_capacity_contract;
    assert fn_gpx_events_per_shot_capacity(C_BUILD_CONFIG) =
           C_NORMAL_TOTAL_EVENTS
        report "V2-GPX-H3-TB event-capacity mismatch"
        severity failure;
    assert fn_gpx_result_fifo_depth(C_BUILD_CONFIG) = 256
        report "V2-GPX-H3-TB result FIFO depth mismatch"
        severity failure;
    assert fn_gpx_events_per_shot_capacity(C_DUAL_EDGE_BUILD_CONFIG) = 456
        report "V2-GPX-H3-TB dual-edge event-capacity mismatch"
        severity failure;
    assert fn_gpx_result_fifo_depth(C_DUAL_EDGE_BUILD_CONFIG) = 512
        report "V2-GPX-H3-TB dual-edge result FIFO depth mismatch"
        severity failure;

    proc_clk <= not proc_clk after C_PROC_PERIOD / 2
        when not simulation_done;
    tdc_clk <= not tdc_clk after C_TDC_PERIOD / 2
        when not simulation_done;

    gen_pin_models : for index in 0 to C_CHIPS - 1 generate
        ef1(index) <= '1' when fifo1_fill(index) = 0 else '0';
        ef2(index) <= '1' when fifo2_fill(index) = 0 else '0';
        lf1(index) <= '1' when fifo1_fill(index) >= 2 else '0';
        lf2(index) <= '1' when fifo2_fill(index) >= 2 else '0';

        d_bus(index) <= d_from_dut(index)
            when d_tri(index)(0) = '0' else (others => 'Z');
        d_bus(index) <= chip_d_out(index)
            when chip_d_oe(index) = '1' else (others => 'Z');
    end generate gen_pin_models;

    u_dut : entity work.lidar_gpx_acquisition_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_OEN_MODE => "DYNAMIC_CONNECTED",
            G_DRAIN_MARGIN_TIME_NS => 100
        )
        port map (
            i_proc_clk => proc_clk,
            i_proc_rst_n => proc_rst_n,
            i_proc_shot => proc_shot,
            o_proc_shot_ready => proc_shot_ready,
            i_proc_stop_tdc => proc_stop_tdc,
            i_proc_clear_status => proc_clear_status,
            o_proc_result => proc_result,
            i_proc_result_ready => proc_result_ready,
            o_shot_drop_sticky => shot_drop_sticky,
            o_stop_drop_sticky => stop_drop_sticky,
            i_tdc_clk => tdc_clk,
            i_tdc_rst_n => tdc_rst_n,
            i_tdc_active_valid => '1',
            i_tdc_active_config => active_config,
            i_tdc_register_image => C_GPX_REGISTER_IMAGE_DEFAULT,
            i_tdc_config_apply => config_apply,
            o_tdc_config_ready => config_ready,
            o_tdc_config_done => config_done,
            i_tdc_run_enable => run_enable,
            i_tdc_soft_reset => '0',
            i_tdc_force_reinit => '0',
            i_tdc_clear_status => tdc_clear_status,
            o_tdc_safe => safe,
            o_tdc_shot_complete => shot_complete,
            i_tdc_register_read => C_GPX_REGISTER_READ_REQUEST_IDLE,
            o_tdc_register_read_ready => open,
            o_tdc_register_read_response => open,
            i_tdc_register_read_response_ready => '1',
            o_cdc_reset_busy => cdc_reset_busy,
            o_adr => adr,
            o_csn => csn,
            o_rdn => rdn,
            o_wrn => wrn,
            o_oen => oen,
            i_d => d_bus,
            o_d => d_from_dut,
            o_d_tri => d_tri,
            i_ef1 => ef1,
            i_ef2 => ef2,
            i_lf1 => lf1,
            i_lf2 => lf2,
            i_irflag => irflag,
            i_errflag => errflag,
            o_stopdis => stopdis,
            o_alutrigger => alutrigger,
            o_puresn => puresn,
            o_active_mask => active_mask,
            o_terminal_mask => terminal_mask,
            o_status => status,
            o_faults => faults
        );

    p_chip_models : process (tdc_clk)
        variable rdn_previous : logic_array_t := (others => '1');
        variable wrn_previous : logic_array_t := (others => '1');
    begin
        if rising_edge(tdc_clk) then
            chip_d_oe <= (others => '0');

            if tdc_rst_n = '0' then
                fifo1_fill <= (others => 0);
                fifo2_fill <= (others => 0);
                fifo1_read_index <= (others => 0);
                fifo2_read_index <= (others => 0);
                empty_read_count <= (others => 0);
                rdn_previous := (others => '1');
                wrn_previous := (others => '1');
            else
                for index in 0 to C_CHIPS - 1 loop
                    if fifo_load = '1' then
                        fifo1_fill(index) <= fifo_load_1;
                        fifo2_fill(index) <= fifo_load_2;
                        fifo1_read_index(index) <= 0;
                        fifo2_read_index(index) <= 0;
                    end if;

                    if oen(index) = '0' and rdn(index) = '0' then
                        chip_d_oe(index) <= '1';
                        if adr(index) = c_TDC_REG8_IFIFO1 then
                            chip_d_out(index) <= fn_raw_word(
                                index, '0', fifo1_read_index(index));
                        elsif adr(index) = c_TDC_REG9_IFIFO2 then
                            chip_d_out(index) <= fn_raw_word(
                                index, '1', fifo2_read_index(index));
                        else
                            chip_d_out(index) <= (others => '0');
                        end if;
                    end if;

                    if rdn(index) = '0' and rdn_previous(index) = '1' then
                        if adr(index) = c_TDC_REG8_IFIFO1 and
                           fifo1_fill(index) = 0 then
                            empty_read_count(index) <=
                                empty_read_count(index) + 1;
                        elsif adr(index) = c_TDC_REG9_IFIFO2 and
                              fifo2_fill(index) = 0 then
                            empty_read_count(index) <=
                                empty_read_count(index) + 1;
                        end if;
                    end if;

                    if rdn(index) = '1' and rdn_previous(index) = '0' then
                        if adr(index) = c_TDC_REG8_IFIFO1 and
                           fifo1_fill(index) > 0 then
                            fifo1_fill(index) <= fifo1_fill(index) - 1;
                            fifo1_read_index(index) <=
                                fifo1_read_index(index) + 1;
                        elsif adr(index) = c_TDC_REG9_IFIFO2 and
                              fifo2_fill(index) > 0 then
                            fifo2_fill(index) <= fifo2_fill(index) - 1;
                            fifo2_read_index(index) <=
                                fifo2_read_index(index) + 1;
                        end if;
                    end if;
                    rdn_previous(index) := rdn(index);
                    wrn_previous(index) := wrn(index);
                end loop;
            end if;
        end if;
    end process p_chip_models;

    p_scoreboard : process (proc_clk)
        variable chip : natural range 0 to C_CHIPS - 1;
        variable local_index : natural;
        variable expected_word : gpx_bus_data_t;
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' or score_reset = '1' then
                event_count <= (others => 0);
                total_event_count <= 0;
            elsif proc_result.valid = '1' and proc_result_ready = '1' then
                chip := to_integer(proc_result.chip_index);
                local_index := event_count(chip);

                assert chip = total_event_count mod C_CHIPS
                    report "V2-GPX-H3-TB deterministic merge order mismatch"
                    severity failure;
                assert proc_result.shot_context = fn_shot(score_scenario)
                    report "V2-GPX-H3-TB Shot context mismatch"
                    severity failure;
                assert proc_result.chip_shot_seq =
                       fn_expected_chip_shot_seq(score_scenario)
                    report "V2-GPX-H3-TB Chip Shot sequence mismatch: actual=" &
                        integer'image(to_integer(proc_result.chip_shot_seq)) &
                        " expected=" & integer'image(
                            fn_expected_chip_shot_seq(score_scenario)) &
                        " scenario=" & integer'image(score_scenario)
                    severity failure;

                case score_scenario is
                    when 0 =>
                        case local_index is
                            when 0 to C_WORDS_PER_IFIFO - 1 =>
                                expected_word := fn_raw_word(
                                    chip, '0', local_index);
                                assert proc_result.kind = GPX_RAW_DATA and
                                       proc_result.ififo_id = '0' and
                                       proc_result.raw_word = expected_word
                                    report "V2-GPX-H3-TB normal IFIFO1 mismatch"
                                    severity failure;
                            when C_WORDS_PER_IFIFO =>
                                assert proc_result.kind =
                                           GPX_RAW_IFIFO1_DONE and
                                       proc_result.ififo_id = '0'
                                    report "V2-GPX-H3-TB normal IFIFO1_DONE mismatch"
                                    severity failure;
                            when C_WORDS_PER_IFIFO + 1 to
                                 2 * C_WORDS_PER_IFIFO =>
                                expected_word := fn_raw_word(
                                    chip, '1',
                                    local_index - C_WORDS_PER_IFIFO - 1);
                                assert proc_result.kind = GPX_RAW_DATA and
                                       proc_result.ififo_id = '1' and
                                       proc_result.raw_word = expected_word
                                    report "V2-GPX-H3-TB normal IFIFO2 mismatch"
                                    severity failure;
                            when C_NORMAL_EVENTS_PER_CHIP - 1 =>
                                assert proc_result.kind = GPX_RAW_DRAIN_DONE and
                                       proc_result.faulted = '0'
                                    report "V2-GPX-H3-TB normal terminal mismatch"
                                    severity failure;
                            when others =>
                                assert false
                                    report "V2-GPX-H3-TB extra normal event"
                                    severity failure;
                        end case;

                    when 1 =>
                        assert local_index = 0 and
                               proc_result.kind = GPX_RAW_TIMEOUT and
                               proc_result.faulted = '1' and
                               proc_result.timeout_cause = "111"
                            report "V2-GPX-H3-TB timeout event mismatch"
                            severity failure;

                    when 2 =>
                        case local_index is
                            when 0 to C_WORDS_PER_IFIFO - 1 =>
                                expected_word := fn_raw_word(
                                    chip, '0', local_index);
                                assert proc_result.kind = GPX_RAW_DATA and
                                       proc_result.ififo_id = '0' and
                                       proc_result.raw_word = expected_word
                                    report "V2-GPX-H3-TB capped IFIFO1 mismatch"
                                    severity failure;
                            when C_WORDS_PER_IFIFO =>
                                assert proc_result.kind =
                                           GPX_RAW_IFIFO1_DONE and
                                       proc_result.ififo_id = '0'
                                    report "V2-GPX-H3-TB capped IFIFO1_DONE mismatch"
                                    severity failure;
                            when C_WORDS_PER_IFIFO + 1 =>
                                expected_word := fn_raw_word(chip, '1', 0);
                                assert proc_result.kind = GPX_RAW_DATA and
                                       proc_result.ififo_id = '1' and
                                       proc_result.raw_word = expected_word
                                    report "V2-GPX-H3-TB capped IFIFO2 mismatch"
                                    severity failure;
                            when C_CAP_EVENTS_PER_CHIP - 1 =>
                                assert proc_result.kind = GPX_RAW_DRAIN_DONE and
                                       proc_result.faulted = '1'
                                    report "V2-GPX-H3-TB capped terminal mismatch"
                                    severity failure;
                            when others =>
                                assert false
                                    report "V2-GPX-H3-TB extra capped event"
                                    severity failure;
                        end case;

                    when others =>
                        assert false report "V2-GPX-H3-TB invalid scenario"
                            severity failure;
                end case;

                event_count(chip) <= event_count(chip) + 1;
                total_event_count <= total_event_count + 1;
            end if;
        end if;
    end process p_scoreboard;

    p_tdc_monitors : process (tdc_clk)
    begin
        if rising_edge(tdc_clk) then
            if tdc_rst_n = '0' then
                completion_count <= 0;
                sequence_seen <= '0';
            else
                if shot_complete = '1' then
                    completion_count <= completion_count + 1;
                end if;
                if monitor_clear = '1' then
                    sequence_seen <= '0';
                end if;
                for index in 0 to C_CHIPS - 1 loop
                    if faults(index).sequence_pulse = '1' then
                        sequence_seen <= '1';
                    end if;
                end loop;
            end if;
        end if;
    end process p_tdc_monitors;

    p_stimulus : process
        procedure wait_proc_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(proc_clk);
            end loop;
            wait for 1 ps;
        end procedure wait_proc_clocks;

        procedure wait_tdc_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(tdc_clk);
            end loop;
            wait for 1 ps;
        end procedure wait_tdc_clocks;

        procedure wait_tdc_high(
            signal value : in std_logic;
            constant message_text : in string;
            constant timeout_clocks : in positive := 200000
        ) is
        begin
            for timeout in 0 to timeout_clocks loop
                wait until rising_edge(tdc_clk);
                wait for 1 ps;
                if value = '1' then
                    return;
                end if;
            end loop;
            assert false report message_text severity failure;
        end procedure wait_tdc_high;

        procedure pulse_score_reset(scenario : natural) is
        begin
            score_scenario <= scenario;
            score_reset <= '1';
            wait_proc_clocks(1);
            score_reset <= '0';
        end procedure pulse_score_reset;

        procedure load_fifos(
            ififo1_words : natural;
            ififo2_words : natural
        ) is
        begin
            fifo_load_1 <= ififo1_words;
            fifo_load_2 <= ififo2_words;
            fifo_load <= '1';
            wait_tdc_clocks(1);
            fifo_load <= '0';
            wait_tdc_clocks(4);
        end procedure load_fifos;

        procedure send_shot(scenario : natural) is
        begin
            for timeout in 0 to 1000 loop
                exit when proc_shot_ready = '1';
                wait_proc_clocks(1);
            end loop;
            assert proc_shot_ready = '1'
                report "V2-GPX-H3-TB Shot ingress not ready"
                severity failure;
            proc_shot <= fn_shot(scenario);
            wait_proc_clocks(1);
            proc_shot <= C_SHOT_START_EVENT_IDLE;
        end procedure send_shot;

        variable held_result : gpx_raw_event_t;
    begin
        proc_rst_n <= '0';
        tdc_rst_n <= '0';
        wait_proc_clocks(8);
        wait_tdc_clocks(8);
        proc_rst_n <= '1';
        tdc_rst_n <= '1';

        for timeout in 0 to 100000 loop
            wait_tdc_clocks(1);
            exit when fn_all_initialized(status) and cdc_reset_busy = '0';
        end loop;
        assert fn_all_initialized(status) and cdc_reset_busy = '0'
            report "V2-GPX-H3-TB initialization/reset-busy timeout"
            severity failure;

        wait_tdc_high(config_ready,
            "V2-GPX-H3-TB configuration ready timeout");
        config_apply <= '1';
        wait_tdc_clocks(1);
        config_apply <= '0';
        wait_tdc_high(config_done,
            "V2-GPX-H3-TB configuration completion timeout");

        run_enable <= '1';
        for timeout in 0 to 100000 loop
            wait_tdc_clocks(1);
            exit when fn_all_running(status);
        end loop;
        assert fn_all_running(status)
            report "V2-GPX-H3-TB run arm timeout"
            severity failure;

        -- H3-NORMAL: 32 physical STOP lanes. Dedicated masks model 16 APD
        -- channels as 16 rising and 16 falling observations. Hold the
        -- Processing consumer stopped until the complete Shot is buffered.
        pulse_score_reset(0);
        load_fifos(C_WORDS_PER_IFIFO, C_WORDS_PER_IFIFO);
        proc_result_ready <= '0';
        send_shot(0);
        for timeout in 0 to 10000 loop
            wait_tdc_clocks(1);
            exit when fn_all_shot_outstanding(status);
        end loop;
        assert fn_all_shot_outstanding(status)
            report "V2-GPX-H3-TB normal Shot was not broadcast"
            severity failure;
        irflag <= (others => '1');
        wait_tdc_clocks(6);
        irflag <= (others => '0');
        proc_stop_tdc <= '1';
        wait_proc_clocks(2);
        proc_stop_tdc <= '0';

        for timeout in 0 to 300000 loop
            wait_tdc_clocks(1);
            exit when completion_count = 1;
        end loop;
        assert completion_count = 1
            report "V2-GPX-H3-TB normal Shot did not complete into result FIFO"
            severity failure;
        assert proc_result.valid = '1'
            report "V2-GPX-H3-TB buffered result not visible"
            severity failure;
        held_result := proc_result;
        wait_proc_clocks(8);
        assert proc_result = held_result
            report "V2-GPX-H3-TB Processing result changed under backpressure"
            severity failure;
        proc_result_ready <= '1';

        for timeout in 0 to 300000 loop
            wait_proc_clocks(1);
            exit when total_event_count = C_NORMAL_TOTAL_EVENTS;
        end loop;
        assert total_event_count = C_NORMAL_TOTAL_EVENTS
            report "V2-GPX-H3-TB normal event count mismatch"
            severity failure;
        for index in 0 to C_CHIPS - 1 loop
            assert event_count(index) = C_NORMAL_EVENTS_PER_CHIP and
                   fifo1_fill(index) = 0 and fifo2_fill(index) = 0 and
                   faults(index) = C_GPX_LANE_FAULTS_CLEAR
                report "V2-GPX-H3-TB normal per-Chip closure mismatch"
                severity failure;
        end loop;

        -- H3-TIMEOUT: STOP crosses Processing->TDC before IrFlag. The
        -- operation owner then stops the run, exercising the bounded legacy
        -- capture-stop fallback and one terminal timeout per Chip.
        pulse_score_reset(1);
        monitor_clear <= '1';
        wait_tdc_clocks(1);
        monitor_clear <= '0';
        load_fifos(0, 0);
        send_shot(1);
        for timeout in 0 to 10000 loop
            wait_tdc_clocks(1);
            exit when fn_all_shot_outstanding(status);
        end loop;
        assert fn_all_shot_outstanding(status)
            report "V2-GPX-H3-TB timeout Shot was not broadcast"
            severity failure;
        proc_stop_tdc <= '1';
        wait_proc_clocks(2);
        proc_stop_tdc <= '0';
        for timeout in 0 to 10000 loop
            wait_tdc_clocks(1);
            exit when sequence_seen = '1';
        end loop;
        assert sequence_seen = '1'
            report "V2-GPX-H3-TB STOP event did not cross into TDC domain"
            severity failure;
        run_enable <= '0';

        for timeout in 0 to 300000 loop
            wait_proc_clocks(1);
            exit when total_event_count = C_TIMEOUT_TOTAL_EVENTS;
        end loop;
        assert total_event_count = C_TIMEOUT_TOTAL_EVENTS
            report "V2-GPX-H3-TB timeout terminal count mismatch"
            severity failure;
        for timeout in 0 to 300000 loop
            wait_tdc_clocks(1);
            exit when completion_count = 2 and safe = '1';
        end loop;
        assert completion_count = 2 and safe = '1'
            report "V2-GPX-H3-TB timeout recovery did not become safe"
            severity failure;
        for index in 0 to C_CHIPS - 1 loop
            assert faults(index).drain_cap_sticky = '0'
                report "V2-GPX-H3-TB timeout aliased into drain-cap sticky chip=" &
                    integer'image(index)
                severity failure;
        end loop;

        proc_clear_status <= '1';
        tdc_clear_status <= '1';
        wait_proc_clocks(1);
        proc_clear_status <= '0';
        wait_tdc_clocks(1);
        tdc_clear_status <= '0';

        -- H3-CAP: load above the immutable 28-word/IFIFO build capacity.
        -- Exactly 28 IFIFO1 words and one IFIFO2 word remain observable;
        -- physical tails are purged and final completion is faulted.
        run_enable <= '1';
        for timeout in 0 to 100000 loop
            wait_tdc_clocks(1);
            exit when fn_all_running(status);
        end loop;
        assert fn_all_running(status)
            report "V2-GPX-H3-TB cap scenario re-arm timeout"
            severity failure;
        pulse_score_reset(2);
        load_fifos(C_WORDS_PER_IFIFO + 4, 1);
        send_shot(2);
        for timeout in 0 to 10000 loop
            wait_tdc_clocks(1);
            exit when fn_all_shot_outstanding(status);
        end loop;
        assert fn_all_shot_outstanding(status)
            report "V2-GPX-H3-TB cap Shot was not broadcast"
            severity failure;
        irflag <= (others => '1');
        wait_tdc_clocks(6);
        irflag <= (others => '0');

        for timeout in 0 to 300000 loop
            wait_proc_clocks(1);
            exit when total_event_count = C_CAP_TOTAL_EVENTS;
        end loop;
        assert total_event_count = C_CAP_TOTAL_EVENTS
            report "V2-GPX-H3-TB cap event count mismatch"
            severity failure;
        for timeout in 0 to 300000 loop
            wait_tdc_clocks(1);
            exit when completion_count = 3;
        end loop;
        assert completion_count = 3
            report "V2-GPX-H3-TB cap Shot completion missing"
            severity failure;
        for index in 0 to C_CHIPS - 1 loop
            assert event_count(index) = C_CAP_EVENTS_PER_CHIP and
                   fifo1_fill(index) = 0 and fifo2_fill(index) = 0 and
                   faults(index).drain_cap_sticky = '1' and
                   empty_read_count(index) = 0
                report "V2-GPX-H3-TB cap/purge closure mismatch chip=" &
                    integer'image(index) &
                    " events=" & integer'image(event_count(index)) &
                    " fifo1=" & integer'image(fifo1_fill(index)) &
                    " fifo2=" & integer'image(fifo2_fill(index)) &
                    " cap=" & std_logic'image(
                        faults(index).drain_cap_sticky) &
                    " empty_reads=" & integer'image(
                        empty_read_count(index))
                severity failure;
        end loop;

        assert shot_drop_sticky = '0' and stop_drop_sticky = '0'
            report "V2-GPX-H3-TB ingress event was dropped"
            severity failure;
        assert active_mask = "1111" and terminal_mask = "1111"
            report "V2-GPX-H3-TB final mask mismatch"
            severity failure;

        report "LIDAR_V2_GPX_ACQUISITION_SUBSYSTEM_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
            severity note;
        simulation_done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_acquisition_subsystem_150_200 is
end entity tb_lidar_gpx_acquisition_subsystem_150_200;

architecture sim of tb_lidar_gpx_acquisition_subsystem_150_200 is
begin
    u_test : entity work.tb_lidar_gpx_acquisition_subsystem
        generic map (
            G_PROC_CLK_MHZ => 150,
            G_TDC_CLK_MHZ  => 200
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_acquisition_subsystem_200_150 is
end entity tb_lidar_gpx_acquisition_subsystem_200_150;

architecture sim of tb_lidar_gpx_acquisition_subsystem_200_150 is
begin
    u_test : entity work.tb_lidar_gpx_acquisition_subsystem
        generic map (
            G_PROC_CLK_MHZ => 200,
            G_TDC_CLK_MHZ  => 150
        );
end architecture sim;
