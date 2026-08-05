library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;

entity tb_lidar_gpx_event_merge is
end entity tb_lidar_gpx_event_merge;

architecture sim of tb_lidar_gpx_event_merge is

    constant C_CLK_PERIOD : time := 5 ns;
    constant C_FIRST_MASK : chip_mask_t := "1011";

    type natural_array_t is array (0 to C_MAX_CHIPS - 1) of natural;
    type chip_sequence_t is array (natural range <>) of natural;
    constant C_EXPECTED_CHIPS : chip_sequence_t(0 to 5) :=
        (0, 1, 3, 0, 1, 3);

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal simulation_done : boolean := false;

    signal shot_accept : std_logic := '0';
    signal shot_mask : chip_mask_t := (others => '0');
    signal source_start : std_logic := '0';
    signal lane_event : gpx_raw_event_array_t :=
        (others => C_GPX_RAW_EVENT_IDLE);
    signal lane_ready : chip_mask_t;
    signal event_out : gpx_raw_event_t;
    signal event_ready : std_logic := '1';
    signal shot_complete : std_logic;
    signal shot_outstanding : std_logic;
    signal terminal_mask : chip_mask_t;

    signal source_stage : natural_array_t := (others => 0);
    signal receive_count : natural := 0;
    signal completion_count : natural := 0;

    function fn_event(
        chip_index : natural;
        kind       : gpx_raw_event_kind_t
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := kind;
        result.chip_index := to_unsigned(
            chip_index, result.chip_index'length);
        result.raw_word := std_logic_vector(to_unsigned(
            16#100# + chip_index, result.raw_word'length));
        result.shot_context.valid := '1';
        result.shot_context.request.valid := '1';
        result.shot_context.request.shot_index := to_unsigned(
            9, result.shot_context.request.shot_index'length);
        return result;
    end function fn_event;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not simulation_done;

    u_dut : entity work.lidar_gpx_event_merge
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_shot_accept => shot_accept,
            i_shot_mask => shot_mask,
            i_lane_event => lane_event,
            o_lane_ready => lane_ready,
            o_event => event_out,
            i_event_ready => event_ready,
            o_shot_complete => shot_complete,
            o_shot_outstanding => shot_outstanding,
            o_terminal_mask => terminal_mask
        );

    p_sources : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                lane_event <= (others => C_GPX_RAW_EVENT_IDLE);
                source_stage <= (others => 0);
            else
                if source_start = '1' then
                    for index in 0 to C_MAX_CHIPS - 1 loop
                        if shot_mask(index) = '1' then
                            lane_event(index) <= fn_event(
                                index, GPX_RAW_DATA);
                            source_stage(index) <= 1;
                        end if;
                    end loop;
                end if;

                for index in 0 to C_MAX_CHIPS - 1 loop
                    if lane_event(index).valid = '1' and
                       lane_ready(index) = '1' then
                        if source_stage(index) = 1 then
                            lane_event(index) <= fn_event(
                                index, GPX_RAW_DRAIN_DONE);
                            source_stage(index) <= 2;
                        else
                            lane_event(index) <= C_GPX_RAW_EVENT_IDLE;
                            source_stage(index) <= 3;
                        end if;
                    end if;
                end loop;
            end if;
        end if;
    end process p_sources;

    p_scoreboard : process (clk)
        variable expected_kind : gpx_raw_event_kind_t;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                receive_count <= 0;
                completion_count <= 0;
            else
                if event_out.valid = '1' and event_ready = '1' then
                    assert receive_count <= C_EXPECTED_CHIPS'high
                        report "V2-GPX-MERGE-TB unexpected extra event"
                        severity failure;
                    assert to_integer(event_out.chip_index) =
                           C_EXPECTED_CHIPS(receive_count)
                        report "V2-GPX-MERGE-TB round-robin order mismatch"
                        severity failure;
                    if receive_count < 3 then
                        expected_kind := GPX_RAW_DATA;
                    else
                        expected_kind := GPX_RAW_DRAIN_DONE;
                    end if;
                    assert event_out.kind = expected_kind
                        report "V2-GPX-MERGE-TB event kind mismatch"
                        severity failure;
                    receive_count <= receive_count + 1;
                end if;

                if shot_complete = '1' then
                    completion_count <= completion_count + 1;
                end if;
            end if;
        end if;
    end process p_scoreboard;

    p_stimulus : process
        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        variable held_event : gpx_raw_event_t;
    begin
        rst_n <= '0';
        wait_clocks(6);
        rst_n <= '1';
        wait_clocks(2);

        shot_mask <= C_FIRST_MASK;
        shot_accept <= '1';
        wait_clocks(1);
        shot_accept <= '0';
        source_start <= '1';
        wait_clocks(1);
        source_start <= '0';

        wait until event_out.valid = '1';
        wait for 1 ps;
        event_ready <= '0';
        held_event := event_out;
        wait_clocks(4);
        assert event_out = held_event
            report "V2-GPX-MERGE-TB output changed under backpressure"
            severity failure;
        event_ready <= '1';

        for timeout in 0 to 100 loop
            wait_clocks(1);
            exit when completion_count = 1;
        end loop;
        assert receive_count = 6 and completion_count = 1
            report "V2-GPX-MERGE-TB first Shot completion mismatch"
            severity failure;
        assert terminal_mask = C_FIRST_MASK and shot_outstanding = '0'
            report "V2-GPX-MERGE-TB terminal mask mismatch"
            severity failure;

        report "LIDAR_V2_GPX_EVENT_MERGE_PASS" severity note;
        simulation_done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;
