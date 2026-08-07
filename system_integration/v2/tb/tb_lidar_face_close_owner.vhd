library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;

entity tb_lidar_face_close_owner is
end entity tb_lidar_face_close_owner;

architecture sim of tb_lidar_face_close_owner is

    constant C_PERIOD : time := 5 ns;
    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => 200,
        tdc_clk_mhz            => 150,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 8,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "0011",
        fall_capability_mask   => "1100",
        output_width           => 32,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    function fn_active return lidar_active_config_t is
        variable runtime : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        result.version := to_unsigned(9, 16);
        result.source := runtime;
        result.derived := fn_derive_runtime_config(C_BUILD_CONFIG, runtime);
        result.derived.columns_per_face := to_unsigned(37, 16);
        return result;
    end function fn_active;

    function fn_face_event(
        face_number : natural;
        enter_value : std_logic;
        exit_value  : std_logic;
        direction_value : direction_t;
        source_value : std_logic
    ) return face_event_t is
        variable result : face_event_t := C_FACE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.inside := enter_value;
        result.enter_event := enter_value;
        result.exit_event := exit_value;
        result.face_index := to_unsigned(face_number, 3);
        result.position := to_unsigned(1000 + face_number, C_POSITION_WIDTH);
        result.direction := direction_value;
        result.source_sim := source_value;
        result.active_version := to_unsigned(9, 16);
        return result;
    end function fn_face_event;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal done : boolean := false;
    signal enable : std_logic := '0';
    signal active_config : lidar_active_config_t := fn_active;
    signal face_event : face_event_t := C_FACE_EVENT_IDLE;
    signal executor_ready : std_logic := '1';
    signal close_event : face_close_event_t;
    signal close_ready : std_logic := '0';
    signal scheduler_block : std_logic;
    signal idle : std_logic;
    signal overflow_sticky : std_logic;
    signal clear_diagnostics : std_logic := '0';

begin

    clk <= not clk after C_PERIOD / 2 when not done;

    u_dut : entity work.lidar_face_close_owner
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_enable => enable,
            i_active_valid => '1',
            i_active_config => active_config,
            i_face_event => face_event,
            i_executor_ready => executor_ready,
            i_clear_diagnostics => clear_diagnostics,
            o_close_event => close_event,
            i_close_ready => close_ready,
            o_scheduler_block => scheduler_block,
            o_idle => idle,
            o_overflow_sticky => overflow_sticky
        );

    p_test : process
        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure pulse_face(value : face_event_t) is
        begin
            face_event <= value;
            wait_clocks(1);
            face_event <= C_FACE_EVENT_IDLE;
        end procedure pulse_face;

        variable held_close : face_close_event_t;
    begin
        rst_n <= '0';
        enable <= '0';
        wait_clocks(4);
        rst_n <= '1';
        wait_clocks(2);
        assert idle = '1' and scheduler_block = '0'
            report "V2-FCLOSE-TB reset/config snapshot mismatch"
            severity failure;

        enable <= '1';
        executor_ready <= '0';
        pulse_face(fn_face_event(1, '1', '0', DIRECTION_CCW, '0'));
        assert idle = '0' and scheduler_block = '0'
            report "V2-FCLOSE-TB traversal did not open"
            severity failure;
        pulse_face(fn_face_event(1, '0', '1', DIRECTION_CCW, '0'));
        assert scheduler_block = '1' and close_event.valid = '0'
            report "V2-FCLOSE-TB executor wait did not block scheduler"
            severity failure;
        wait_clocks(5);
        assert scheduler_block = '1' and close_event.valid = '0'
            report "V2-FCLOSE-TB close escaped while executor busy"
            severity failure;

        executor_ready <= '1';
        for timeout in 0 to 8 loop
            wait_clocks(1);
            exit when close_event.valid = '1';
        end loop;
        assert close_event.valid = '1' and
               close_event.face_frame_id = to_unsigned(0, 32) and
               close_event.face_index = to_unsigned(1, 3) and
               close_event.direction = DIRECTION_CCW and
               close_event.source_sim = '0' and
               close_event.active_version = to_unsigned(9, 16) and
               close_event.columns_per_face = to_unsigned(37, 16)
            report "V2-FCLOSE-TB close identity mismatch"
            severity failure;
        held_close := close_event;
        wait_clocks(6);
        assert close_event = held_close and scheduler_block = '1'
            report "V2-FCLOSE-TB close changed under backpressure"
            severity failure;

        close_ready <= '1';
        wait_clocks(1);
        close_ready <= '0';
        wait_clocks(1);
        assert close_event.valid = '0' and scheduler_block = '0' and
               idle = '1'
            report "V2-FCLOSE-TB close retirement mismatch"
            severity failure;

        -- No Shot observation is required to close an admitted traversal.
        -- This is the explicit all-hole Face boundary consumed by B8.
        pulse_face(fn_face_event(2, '1', '0', DIRECTION_CW, '0'));
        pulse_face(fn_face_event(2, '0', '1', DIRECTION_CW, '0'));
        for timeout in 0 to 8 loop
            wait_clocks(1);
            exit when close_event.valid = '1';
        end loop;
        assert close_event.valid = '1' and
               close_event.face_frame_id = to_unsigned(1, 32) and
               close_event.face_index = to_unsigned(2, 3)
            report "V2-FCLOSE-TB all-hole traversal close missing"
            severity failure;
        close_ready <= '1';
        wait_clocks(1);
        close_ready <= '0';
        wait_clocks(1);

        -- A zero-gap Face change reports exit and enter together. The old
        -- close is ordered first and the overlapping new traversal is skipped
        -- so a pending output cannot create an endless all-hole Face chain.
        pulse_face(fn_face_event(0, '1', '0', DIRECTION_CW, '0'));
        face_event <= fn_face_event(1, '1', '1', DIRECTION_CCW, '0');
        wait for 1 ps;
        assert scheduler_block = '1'
            report "V2-FCLOSE-TB direct transition was not blocked immediately"
            severity failure;
        wait_clocks(1);
        face_event <= C_FACE_EVENT_IDLE;
        for timeout in 0 to 8 loop
            wait_clocks(1);
            exit when close_event.valid = '1';
        end loop;
        assert close_event.valid = '1' and
               close_event.face_frame_id = to_unsigned(2, 32) and
               close_event.face_index = to_unsigned(0, 3) and
               close_event.direction = DIRECTION_CW
            report "V2-FCLOSE-TB old Face close lost on direct transition"
            severity failure;
        close_ready <= '1';
        wait_clocks(1);
        close_ready <= '0';
        wait_clocks(1);
        assert scheduler_block = '0' and idle = '1' and
               overflow_sticky = '1'
            report "V2-FCLOSE-TB busy-boundary traversal was not skipped"
            severity failure;

        pulse_face(fn_face_event(1, '1', '0', DIRECTION_CCW, '0'));
        pulse_face(fn_face_event(1, '0', '1', DIRECTION_CCW, '0'));
        for timeout in 0 to 8 loop
            wait_clocks(1);
            exit when close_event.valid = '1';
        end loop;
        assert close_event.valid = '1' and
               close_event.face_frame_id = to_unsigned(3, 32) and
               close_event.face_index = to_unsigned(1, 3) and
               close_event.direction = DIRECTION_CCW
            report "V2-FCLOSE-TB recovery Face close missing"
            severity failure;
        close_ready <= '1';
        wait_clocks(1);
        close_ready <= '0';
        wait_clocks(1);

        assert overflow_sticky = '1' and idle = '1'
            report "V2-FCLOSE-TB overload diagnostic missing"
            severity failure;
        clear_diagnostics <= '1';
        wait_clocks(1);
        clear_diagnostics <= '0';
        wait_clocks(1);
        assert overflow_sticky = '0'
            report "V2-FCLOSE-TB overload diagnostic did not clear"
            severity failure;
        report "LIDAR_V2_FACE_CLOSE_OWNER_PASS" severity note;
        done <= true;
        stop;
        wait;
    end process p_test;

end architecture sim;
