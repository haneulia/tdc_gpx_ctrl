library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;

entity tb_lidar_processing_subsystem is
    generic (
        G_PROC_CLK_MHZ        : positive := 150;
        G_PROC_HALF_PERIOD_PS : positive := 3333;
        G_TDC_CLK_MHZ         : positive := 200;
        G_TDC_HALF_PERIOD_PS  : positive := 2500
    );
end entity tb_lidar_processing_subsystem;

architecture sim of tb_lidar_processing_subsystem is

    function fn_test_build return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz      := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz       := G_TDC_CLK_MHZ;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        return result;
    end function fn_test_build;

    constant C_TEST_BUILD : lidar_build_config_t := fn_test_build;

    function fn_test_config(
        simulation_mode : std_logic;
        version_value   : positive
    ) return lidar_active_config_t is
        variable result   : lidar_active_config_t;
        variable source_v : lidar_runtime_config_t;
    begin
        source_v := fn_default_runtime_config(C_TEST_BUILD);
        source_v.motor.cpr              := to_unsigned(8, 16);
        source_v.motor.decode_mode      := DECODE_X4;
        source_v.motor.direction        := DIRECTION_CW;
        source_v.motor.virtual_ticks_lo := to_unsigned(2, 32);
        source_v.motor.virtual_hi_count := (others => '0');
        source_v.motor.z_offset         := (others => '0');
        source_v.motor.z_width          := (others => '0');
        source_v.motor.simulation_mode  := simulation_mode;
        source_v.mirror.face_centers    := (others => (others => '0'));
        source_v.mirror.face_centers(0) := to_unsigned(7, C_POSITION_WIDTH);
        source_v.mirror.common_half_width :=
            to_unsigned(3, C_POSITION_WIDTH);
        source_v.laser.face_enable_mask := "00001";
        source_v.laser.optical_shot_interval_udeg :=
            to_unsigned(45_000_000, angle_udeg_t'length);

        result.version := to_unsigned(version_value, result.version'length);
        result.source  := source_v;
        result.derived := fn_derive_runtime_config(C_TEST_BUILD, source_v);

        -- Keep shot timing short and identical in both clock profiles. The
        -- calculator boundary already verifies the 5 ns conversion itself.
        result.derived.fire_width_proc_clks := to_unsigned(3, 32);
        result.derived.fire_done_timeout_proc_clks := to_unsigned(16, 32);
        result.derived.target_range_proc_clks := to_unsigned(5, 32);
        result.derived.start_width_proc_clks := to_unsigned(2, 32);
        result.derived.stop_width_proc_clks := to_unsigned(2, 32);
        result.derived.simulation_start_delay_proc_clks := to_unsigned(2, 32);
        return result;
    end function fn_test_config;

    constant C_PHYSICAL_CONFIG : lidar_active_config_t :=
        fn_test_config('0', 601);
    constant C_SIMULATION_CONFIG : lidar_active_config_t :=
        fn_test_config('1', 602);

    signal proc_clk   : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_clk    : std_logic := '0';
    signal stop_clock : boolean := false;
    signal proc_cycle : natural := 0;

    signal command_valid : std_logic := '0';
    signal command       : operation_command_t := OP_COMMAND_NONE;
    signal source_online : std_logic := '1';
    signal laser_permit  : std_logic := '0';
    signal config_enable : std_logic := '1';
    signal active_valid  : std_logic := '1';
    signal active_config : lidar_active_config_t := C_PHYSICAL_CONFIG;
    signal operation_state  : operation_state_t;
    signal command_accepted : std_logic;
    signal command_rejected : std_logic;
    signal permit_trip      : std_logic;
    signal safe_to_prepare  : std_logic;

    signal enc_a : std_logic := '0';
    signal enc_b : std_logic := '0';
    signal enc_z : std_logic := '0';
    signal fire_done_raw : std_logic := '0';
    signal auto_fire_done : std_logic := '0';
    signal clear_diagnostics : std_logic := '0';

    signal mon_ready : std_logic := '1';
    signal mon_valid : std_logic;
    signal mon_data  : processing_monitor_tdata_t;
    signal mon_keep  : processing_monitor_tkeep_t;
    signal mon_user  : processing_monitor_tuser_t;
    signal mon_last  : std_logic;

    signal fire_pulse : std_logic;
    signal start_tdc  : std_logic;
    signal stop_tdc   : std_logic;
    signal position_event : position_event_t;
    signal face_event     : face_event_t;
    signal shot_request   : shot_request_t;
    signal shot_start     : shot_start_event_t;
    signal shot_result    : shot_result_t;
    signal current_request : shot_request_t;
    signal current_position : position_t;
    signal current_direction : direction_t;
    signal executor_ready : std_logic;
    signal request_accept : std_logic;
    signal request_drop   : std_logic;
    signal executor_busy  : std_logic;
    signal physical_arm   : std_logic;
    signal rearm_active   : std_logic;
    signal pipeline_idle  : std_logic;
    signal virtual_a      : std_logic;
    signal virtual_b      : std_logic;
    signal virtual_z      : std_logic;
    signal b0_to_accept_clks : processing_latency_t;
    signal physical_to_fire_clks : processing_latency_t;
    signal virtual_to_accept_clks : processing_latency_t;
    signal fire_done_sync_clks : unsigned(15 downto 0);
    signal rearm_margin_clks   : unsigned(15 downto 0);
    signal diagnostics : processing_diagnostics_t;
    signal face_close_event : face_close_event_t;
    signal face_close_overflow_sticky : std_logic;

begin

    u_operation : entity work.lidar_operation_manager
        generic map (
            G_BUILD_CONFIG => C_TEST_BUILD
        )
        port map (
            i_clk                   => proc_clk,
            i_rst_n                 => proc_rst_n,
            i_command_valid         => command_valid,
            i_command               => command,
            i_command_source_online => source_online,
            i_external_laser_permit => laser_permit,
            i_config_enable         => config_enable,
            i_active_valid          => active_valid,
            i_active_config         => active_config,
            i_pipeline_idle         => pipeline_idle,
            o_state                 => operation_state,
            o_command_accepted      => command_accepted,
            o_command_rejected      => command_rejected,
            o_permit_trip           => permit_trip,
            o_safe_to_prepare       => safe_to_prepare
        );

    u_dut : entity work.lidar_processing_subsystem
        generic map (
            G_BUILD_CONFIG => C_TEST_BUILD
        )
        port map (
            i_clk                => proc_clk,
            i_rst_n              => proc_rst_n,
            i_active_valid       => active_valid,
            i_active_config      => active_config,
            i_operation_state    => operation_state,
            i_enc_a              => enc_a,
            i_enc_b              => enc_b,
            i_enc_z              => enc_z,
            i_fire_done_raw      => fire_done_raw,
            i_clear_diagnostics  => clear_diagnostics,
            i_face_close_ready   => '1',
            o_face_close_event   => face_close_event,
            o_face_close_overflow_sticky =>
                face_close_overflow_sticky,
            m_mon_axis_tready    => mon_ready,
            m_mon_axis_tvalid    => mon_valid,
            m_mon_axis_tdata     => mon_data,
            m_mon_axis_tkeep     => mon_keep,
            m_mon_axis_tuser     => mon_user,
            m_mon_axis_tlast     => mon_last,
            o_fire_pulse         => fire_pulse,
            o_start_tdc          => start_tdc,
            o_stop_tdc           => stop_tdc,
            o_position_event     => position_event,
            o_face_event         => face_event,
            o_shot_request       => shot_request,
            o_shot_start         => shot_start,
            o_shot_result        => shot_result,
            o_current_request    => current_request,
            o_current_position   => current_position,
            o_current_direction  => current_direction,
            o_executor_ready     => executor_ready,
            o_request_accept     => request_accept,
            o_request_drop       => request_drop,
            o_executor_busy      => executor_busy,
            o_physical_arm       => physical_arm,
            o_rearm_active       => rearm_active,
            o_pipeline_idle      => pipeline_idle,
            o_virtual_a          => virtual_a,
            o_virtual_b          => virtual_b,
            o_virtual_z          => virtual_z,
            o_b0_to_accept_clks  => b0_to_accept_clks,
            o_physical_to_fire_clks => physical_to_fire_clks,
            o_virtual_to_accept_clks => virtual_to_accept_clks,
            o_fire_done_sync_clks => fire_done_sync_clks,
            o_rearm_margin_clks   => rearm_margin_clks,
            o_diagnostics         => diagnostics
        );

    p_proc_clock : process
    begin
        while not stop_clock loop
            proc_clk <= '0';
            wait for G_PROC_HALF_PERIOD_PS * 1 ps;
            proc_clk <= '1';
            wait for G_PROC_HALF_PERIOD_PS * 1 ps;
        end loop;
        wait;
    end process p_proc_clock;

    p_tdc_clock : process
    begin
        wait for 731 ps;
        while not stop_clock loop
            tdc_clk <= '0';
            wait for G_TDC_HALF_PERIOD_PS * 1 ps;
            tdc_clk <= '1';
            wait for G_TDC_HALF_PERIOD_PS * 1 ps;
        end loop;
        wait;
    end process p_tdc_clock;

    p_cycle_counter : process (proc_clk)
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                proc_cycle <= 0;
            else
                proc_cycle <= proc_cycle + 1;
            end if;
        end if;
    end process p_cycle_counter;

    -- The external laser response is intentionally generated on the TDC
    -- profile clock so raw fire_done has no fixed phase relationship to the
    -- Processing clock in either 150/200 or 200/150 profile.
    p_fire_done_model : process
    begin
        fire_done_raw <= '0';
        loop
            wait until fire_pulse'event and fire_pulse = '1';
            if auto_fire_done = '1' then
                wait until rising_edge(tdc_clk);
                wait until rising_edge(tdc_clk);
                fire_done_raw <= '1';
                wait for 1 ps;
                assert start_tdc = '1'
                    report "V2-PROC-P50 raw fire_done did not assert START"
                    severity failure;
                wait until rising_edge(tdc_clk);
                fire_done_raw <= '0';
            end if;
        end loop;
    end process p_fire_done_model;

    p_test : process
        procedure check(
            condition    : boolean;
            message_text : string
        ) is
        begin
            assert condition report message_text severity failure;
        end procedure check;

        procedure wait_proc(count_value : positive) is
        begin
            for index in 1 to count_value loop
                wait until rising_edge(proc_clk);
                wait for 1 ps;
            end loop;
        end procedure wait_proc;

        procedure reset_system(
            constant config_value : in lidar_active_config_t
        ) is
        begin
            proc_rst_n       <= '0';
            command_valid    <= '0';
            command          <= OP_COMMAND_NONE;
            source_online    <= '1';
            laser_permit     <= '0';
            config_enable    <= '1';
            active_valid     <= '1';
            active_config    <= config_value;
            enc_a            <= '0';
            enc_b            <= '0';
            enc_z            <= '0';
            auto_fire_done   <= '0';
            mon_ready        <= '1';
            clear_diagnostics <= '0';
            wait_proc(5);
            proc_rst_n <= '1';
            wait_proc(6);
            check(pipeline_idle = '1',
                "V2-PROC reset did not reach idle");
        end procedure reset_system;

        procedure send_command(
            constant command_value : in operation_command_t;
            constant case_name     : in string
        ) is
        begin
            wait until falling_edge(proc_clk);
            command       <= command_value;
            command_valid <= '1';
            wait until rising_edge(proc_clk);
            wait for 1 ps;
            check(command_accepted = '1' and command_rejected = '0',
                case_name & " command rejected");
            command_valid <= '0';
            command       <= OP_COMMAND_NONE;
        end procedure send_command;

        procedure drive_cw_step(
            variable phase_value : inout natural;
            variable sample_cycle_value : out natural
        ) is
        begin
            phase_value := (phase_value + 1) mod 4;
            wait until falling_edge(proc_clk);
            case phase_value is
                when 1      => enc_a <= '1'; enc_b <= '0';
                when 2      => enc_a <= '1'; enc_b <= '1';
                when 3      => enc_a <= '0'; enc_b <= '1';
                when others => enc_a <= '0'; enc_b <= '0';
            end case;
            wait until rising_edge(proc_clk);
            wait for 1 ps;
            sample_cycle_value := proc_cycle;
        end procedure drive_cw_step;

        variable phase_v              : natural := 0;
        variable sample_cycle_v       : natural := 0;
        variable due_sample_cycle_v   : natural := 0;
        variable fire_cycle_v         : natural := 0;
        variable virtual_change_cycle_v : natural := 0;
        variable virtual_changes_v    : natural := 0;
        variable wait_count_v         : natural := 0;
        variable saw_start_v          : boolean := false;
        variable saw_stop_v           : boolean := false;
        variable held_data_v          : processing_monitor_tdata_t;
        variable held_user_v          : processing_monitor_tuser_t;
        variable held_last_v          : std_logic;
        variable previous_virtual_v   : std_logic_vector(1 downto 0);
    begin
        check(fn_validate_build_config(C_TEST_BUILD) = CFG_OK,
            "V2-PROC test build is invalid");
        check(C_PHYSICAL_CONFIG.derived.total_states = 32 and
              C_PHYSICAL_CONFIG.derived.face_lower(0) = 4 and
              C_PHYSICAL_CONFIG.derived.face_upper(0) = 10 and
              C_PHYSICAL_CONFIG.derived.shot_interval_states = 2 and
              C_PHYSICAL_CONFIG.derived.columns_per_face = 3,
            "V2-PROC test geometry derivation mismatch");

        -- P50/P51/P52: physical pin through B0..B3, monitor isolation and
        -- integrated safe point while an observation beat remains stalled.
        reset_system(C_PHYSICAL_CONFIG);
        laser_permit   <= '1';
        auto_fire_done <= '1';
        mon_ready      <= '0';
        wait_proc(4);
        send_command(OP_COMMAND_RUN, "V2-PROC-P50 RUN");
        send_command(OP_COMMAND_ARM, "V2-PROC-P50 ARM");
        check(operation_state.physical_fire_enable = '1' and
              operation_state.simulation_enable = '0',
            "V2-PROC-P50 physical gate mismatch");
        check(to_integer(b0_to_accept_clks) = 4 and
              to_integer(physical_to_fire_clks) = 8 and
              to_integer(virtual_to_accept_clks) = 5 and
              to_integer(fire_done_sync_clks) = 3 and
              to_integer(rearm_margin_clks) = 2,
            "V2-PROC-P50 read-only latency contract mismatch");

        phase_v := 0;
        for step_index in 1 to 4 loop
            drive_cw_step(phase_v, sample_cycle_v);
            if step_index = 4 then
                due_sample_cycle_v := sample_cycle_v;
            end if;
        end loop;

        wait_count_v := 0;
        while fire_pulse /= '1' loop
            wait_proc(1);
            wait_count_v := wait_count_v + 1;
            check(wait_count_v < 20, "V2-PROC-P50 physical fire timeout");
        end loop;
        fire_cycle_v := proc_cycle;
        check(fire_cycle_v - due_sample_cycle_v =
              to_integer(physical_to_fire_clks),
            "V2-PROC-P50 physical pin-to-fire latency mismatch");
        check(request_accept = '1' and request_drop = '0' and
              current_request.position = 4 and
              current_request.shot_index = 0 and
              current_request.source_sim = '0' and
              current_request.active_version = C_PHYSICAL_CONFIG.version,
            "V2-PROC-P50 accepted request identity mismatch");

        check(mon_valid = '1' and
              unsigned(mon_data(C_MON_POSITION_MSB downto
                  C_MON_POSITION_LSB)) = 1 and
              mon_keep = (mon_keep'range => '1'),
            "V2-PROC-P51 first stalled monitor beat mismatch");
        held_data_v := mon_data;
        held_user_v := mon_user;
        held_last_v := mon_last;

        wait_count_v := 0;
        saw_start_v := false;
        while shot_start.valid /= '1' loop
            wait_proc(1);
            wait_count_v := wait_count_v + 1;
            if start_tdc = '1' then
                saw_start_v := true;
            end if;
            check(mon_valid = '1' and mon_data = held_data_v and
                  mon_user = held_user_v and mon_last = held_last_v,
                "V2-PROC-P51 stalled AXIS payload changed");
            check(wait_count_v < 12, "V2-PROC-P50 shot-start timeout");
        end loop;
        check(shot_start.request = current_request and
              shot_start.request.position = 4 and saw_start_v,
            "V2-PROC-P50 shot-start identity mismatch");

        send_command(OP_COMMAND_STOP, "V2-PROC-P52 STOP");
        check(pipeline_idle = '0' and safe_to_prepare = '0',
            "V2-PROC-P52 safe point rose before shot drain");

        wait_count_v := 0;
        saw_stop_v := false;
        while shot_result.valid /= '1' loop
            wait_proc(1);
            wait_count_v := wait_count_v + 1;
            if stop_tdc = '1' then
                saw_stop_v := true;
            end if;
            check(wait_count_v < 20, "V2-PROC-P50 shot result timeout");
        end loop;
        if stop_tdc = '1' then
            saw_stop_v := true;
        end if;
        check(saw_stop_v and shot_result.timeout = '0' and
              shot_result.aborted = '0' and
              shot_result.request = current_request,
            "V2-PROC-P50 physical completion mismatch");

        wait_count_v := 0;
        while pipeline_idle /= '1' loop
            wait_proc(1);
            wait_count_v := wait_count_v + 1;
            check(wait_count_v < 12, "V2-PROC-P52 drain did not become idle");
        end loop;
        wait for 1 ps;
        check(mon_valid = '1' and safe_to_prepare = '1',
            "V2-PROC-P52 monitor stall blocked functional safe point");
        check(diagnostics.monitor_drop_sticky = '1' and
              diagnostics.monitor_drop_count >= 3 and
              diagnostics.schedule_overrun_count = 0 and
              diagnostics.laser.fire_done_timeout_count = 0,
            "V2-PROC-P51 isolation diagnostics mismatch");

        mon_ready <= '1';
        wait_proc(1);
        check(mon_valid = '0',
            "V2-PROC-P51 accepted monitor beat did not retire");
        clear_diagnostics <= '1';
        wait_proc(1);
        clear_diagnostics <= '0';
        check(diagnostics.monitor_drop_sticky = '0' and
              diagnostics.monitor_drop_count = 0,
            "V2-PROC-P51 monitor diagnostic clear failed");
        report "V2-PROC-P50 physical B0..B3 chain PASS" severity note;
        report "V2-PROC-P51 AXIS monitor isolation PASS" severity note;
        report "V2-PROC-P52 integrated safe point PASS" severity note;

        -- P53: the virtual source follows the same B0/B1/B2 identity path,
        -- reaches executor accept in five clocks and never fires the laser.
        reset_system(C_SIMULATION_CONFIG);
        laser_permit   <= '0';
        auto_fire_done <= '0';
        mon_ready      <= '1';
        send_command(OP_COMMAND_RUN, "V2-PROC-P53 RUN");
        send_command(OP_COMMAND_ARM, "V2-PROC-P53 ARM");
        check(operation_state.simulation_enable = '1' and
              operation_state.physical_fire_enable = '0',
            "V2-PROC-P53 simulation gate mismatch");

        previous_virtual_v := virtual_a & virtual_b;
        virtual_changes_v := 0;
        wait_count_v := 0;
        while request_accept /= '1' loop
            wait_proc(1);
            wait_count_v := wait_count_v + 1;
            check(fire_pulse = '0' and physical_arm = '0',
                "V2-PROC-P53 simulation enabled physical path");
            if (virtual_a & virtual_b) /= previous_virtual_v then
                previous_virtual_v := virtual_a & virtual_b;
                virtual_changes_v := virtual_changes_v + 1;
                if virtual_changes_v = 4 then
                    virtual_change_cycle_v := proc_cycle;
                end if;
            end if;
            check(wait_count_v < 80,
                "V2-PROC-P53 simulation request timeout");
        end loop;
        check(virtual_changes_v >= 4 and
              proc_cycle - virtual_change_cycle_v =
                  to_integer(virtual_to_accept_clks),
            "V2-PROC-P53 virtual transition-to-accept latency mismatch");
        check(current_request.position = 4 and
              current_request.source_sim = '1' and
              current_request.active_version = C_SIMULATION_CONFIG.version,
            "V2-PROC-P53 simulation request identity mismatch");

        wait_count_v := 0;
        while shot_start.valid /= '1' loop
            wait_proc(1);
            wait_count_v := wait_count_v + 1;
            check(fire_pulse = '0',
                "V2-PROC-P53 simulation produced fire pulse");
            check(wait_count_v < 8,
                "V2-PROC-P53 simulation START timeout");
        end loop;
        check(start_tdc = '1' and shot_start.request = current_request and
              shot_start.request.source_sim = '1',
            "V2-PROC-P53 simulation START identity mismatch");

        wait_count_v := 0;
        saw_stop_v := false;
        while shot_result.valid /= '1' loop
            wait_proc(1);
            wait_count_v := wait_count_v + 1;
            if stop_tdc = '1' then
                saw_stop_v := true;
            end if;
            check(fire_pulse = '0',
                "V2-PROC-P53 late physical fire pulse");
            check(wait_count_v < 16,
                "V2-PROC-P53 simulation result timeout");
        end loop;
        if stop_tdc = '1' then
            saw_stop_v := true;
        end if;
        check(saw_stop_v and shot_result.timeout = '0' and
              shot_result.aborted = '0' and
              diagnostics.monitor_drop_count = 0,
            "V2-PROC-P53 simulation completion mismatch");
        report "V2-PROC-P53 virtual B0..B3 chain PASS" severity note;

        report "LIDAR_V2_PROCESSING_SUBSYSTEM_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
            severity note;
        stop_clock <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
