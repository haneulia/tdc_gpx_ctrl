library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_frame_lane_assembler is
    generic (
        G_CLK_MHZ  : positive := 150;
        G_SCENARIO : natural range 0 to 4 := 0
    );
end entity tb_lidar_gpx_frame_lane_assembler;

architecture sim of tb_lidar_gpx_frame_lane_assembler is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz  := 200;
        result.output_width := 32;
        result.num_faces    := 4;
        result.enable_echo_receiver   := false;
        result.enable_echo_simulation := false;
        if G_SCENARIO = 1 then
            result.num_chips            := 1;
            result.rise_capability_mask := "0001";
            result.fall_capability_mask := "0001";
        elsif G_SCENARIO = 4 then
            result.num_chips            := 4;
            result.rise_capability_mask := "1111";
            result.fall_capability_mask := "1111";
        else
            result.num_chips            := 4;
            result.rise_capability_mask := "0011";
            result.fall_capability_mask := "1100";
        end if;
        return result;
    end function fn_build_config;

    function fn_rise_mask return chip_mask_t is
    begin
        case G_SCENARIO is
            when 0 | 3 => return "0011";
            when 1     => return "0001";
            when 2 | 4 => return "1111";
        end case;
    end function fn_rise_mask;

    function fn_fall_mask return chip_mask_t is
    begin
        case G_SCENARIO is
            when 0 | 3 => return "1100";
            when 1     => return "0001";
            when 2     => return "0000";
            when 4     => return "1111";
        end case;
    end function fn_fall_mask;

    function fn_columns return natural is
    begin
        if G_SCENARIO = 0 then
            return 3;
        end if;
        return 1;
    end function fn_columns;

    function fn_expected_hit(
        shot_index  : natural;
        chip_index  : natural;
        stop_index  : natural;
        slope_value : gpx_slope_t
    ) return natural is
        variable slope_offset : natural := 0;
    begin
        if slope_value = GPX_SLOPE_FALL then
            slope_offset := 100;
        end if;
        return shot_index * 10000 + chip_index * 1000 +
            slope_offset + stop_index + 1;
    end function fn_expected_hit;

    function fn_chip_for_slot(
        mask_value : chip_mask_t;
        slot_value : natural
    ) return natural is
        variable rank_value : natural :=
            slot_value / C_DEFAULT_BUILD_CONFIG.stops_per_chip;
    begin
        for chip_index in 0 to C_MAX_CHIPS - 1 loop
            if mask_value(chip_index) = '1' then
                if rank_value = 0 then
                    return chip_index;
                end if;
                rank_value := rank_value - 1;
            end if;
        end loop;
        return 0;
    end function fn_chip_for_slot;

    function fn_context(
        shot_index : natural;
        columns    : natural
    ) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(1, 3);
        result.request.position := to_unsigned(100 + shot_index, 15);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(shot_index, 16);
        if shot_index + 1 >= columns then
            result.request.last_in_face := '1';
        end if;
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(
            3, result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(9, 16);
        result.fire_to_t0_clks := to_unsigned(7, 32);
        return result;
    end function fn_context;

    function fn_data_cell(
        shot_index  : natural;
        columns     : natural;
        chip_index  : natural;
        stop_index  : natural;
        slope_value : gpx_slope_t
    ) return gpx_cell_event_t is
        variable result : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_CELL_DATA;
        result.chip_index := to_unsigned(chip_index, 2);
        result.stop_index := to_unsigned(stop_index, 3);
        if stop_index >= 4 then
            result.ififo_id := '1';
        end if;
        result.slope := slope_value;
        result.hit_count := to_unsigned(1, 3);
        result.max_hits := to_unsigned(2, 3);
        result.hits(0) := to_unsigned(fn_expected_hit(
            shot_index, chip_index, stop_index, slope_value),
            C_GPX_HIT_WIDTH);
        result.shot_context := fn_context(shot_index, columns);
        result.chip_shot_seq := to_unsigned(10 + shot_index, 16);
        return result;
    end function fn_data_cell;

    function fn_terminal(
        shot_index : natural;
        columns    : natural;
        chip_index : natural
    ) return gpx_cell_event_t is
        variable result : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := GPX_CELL_DRAIN_DONE;
        result.chip_index := to_unsigned(chip_index, 2);
        result.max_hits := to_unsigned(2, 3);
        result.shot_context := fn_context(shot_index, columns);
        result.chip_shot_seq := to_unsigned(10 + shot_index, 16);
        return result;
    end function fn_terminal;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_RISE_MASK : chip_mask_t := fn_rise_mask;
    constant C_FALL_MASK : chip_mask_t := fn_fall_mask;
    constant C_COLUMNS : natural := fn_columns;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal abort_event  : std_logic := '0';
    signal clear_sticky : std_logic := '0';
    signal cell_event   : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    signal cell_ready   : std_logic;
    signal rise_event   : gpx_frame_cell_event_t;
    signal fall_event   : gpx_frame_cell_event_t;
    signal rise_ready   : std_logic := '0';
    signal fall_ready   : std_logic := '0';
    signal rise_line_done : std_logic;
    signal fall_line_done : std_logic;
    signal shot_done      : std_logic;
    signal idle            : std_logic;
    signal fault_pulse  : gpx_frame_assembler_faults_t;
    signal fault_sticky : gpx_frame_assembler_faults_t;

    signal ready_counter : natural := 0;
    signal rise_cell_count : natural := 0;
    signal fall_cell_count : natural := 0;
    signal rise_line_count : natural := 0;
    signal fall_line_count : natural := 0;
    signal shot_done_count : natural := 0;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_frame_lane_assembler
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk              => clk,
            i_rst_n            => rst_n,
            i_abort            => abort_event,
            i_clear_sticky     => clear_sticky,
            i_active_version   => to_unsigned(9, 16),
            i_active_rise_mask => C_RISE_MASK,
            i_active_fall_mask => C_FALL_MASK,
            i_columns_per_face => to_unsigned(C_COLUMNS, 16),
            i_cell_event       => cell_event,
            o_cell_ready       => cell_ready,
            o_rise_event       => rise_event,
            i_rise_ready       => rise_ready,
            o_fall_event       => fall_event,
            i_fall_ready       => fall_ready,
            o_rise_line_done   => rise_line_done,
            o_fall_line_done   => fall_line_done,
            o_shot_done        => shot_done,
            o_idle             => idle,
            o_fault_pulse      => fault_pulse,
            o_fault_sticky     => fault_sticky
        );

    p_ready : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                ready_counter <= 0;
                rise_ready <= '0';
                fall_ready <= '0';
            else
                ready_counter <= ready_counter + 1;
                if G_SCENARIO = 3 then
                    if ready_counter mod 5 = 1 or
                       ready_counter mod 5 = 2 then
                        rise_ready <= '0';
                    else
                        rise_ready <= '1';
                    end if;
                    if ready_counter mod 7 = 3 or
                       ready_counter mod 7 = 4 or
                       ready_counter mod 7 = 5 then
                        fall_ready <= '0';
                    else
                        fall_ready <= '1';
                    end if;
                else
                    rise_ready <= '1';
                    fall_ready <= '1';
                end if;
            end if;
        end if;
    end process p_ready;

    p_rise_monitor : process (clk)
        variable slot_value : natural;
        variable chip_value : natural;
        variable stop_value : natural;
        variable shot_value : natural;
        variable expected_blank : std_logic;
        variable held_valid : boolean := false;
        variable held_event : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                rise_cell_count <= 0;
                rise_line_count <= 0;
                held_valid := false;
            else
                if held_valid then
                    assert rise_event = held_event
                        report "V2-B8-TB Rise event changed under backpressure"
                        severity failure;
                end if;
                held_valid := rise_event.valid = '1' and rise_ready = '0';
                if held_valid then
                    held_event := rise_event;
                end if;

                if rise_event.valid = '1' and rise_ready = '1' then
                    slot_value := to_integer(rise_event.slot_index);
                    chip_value := fn_chip_for_slot(C_RISE_MASK, slot_value);
                    stop_value := slot_value mod
                        C_BUILD_CONFIG.stops_per_chip;
                    if G_SCENARIO = 0 and rise_line_count = 1 then
                        shot_value := 2;
                    else
                        shot_value := 0;
                    end if;
                    expected_blank := '0';
                    if G_SCENARIO = 3 and chip_value = 1 and
                       stop_value = 7 then
                        expected_blank := '1';
                    end if;

                    assert rise_event.cell.slope = GPX_SLOPE_RISE and
                           to_integer(rise_event.cell.chip_index) = chip_value and
                           to_integer(rise_event.cell.stop_index) = stop_value and
                           to_integer(rise_event.slot_count) =
                               fn_popcount(C_RISE_MASK) *
                               C_BUILD_CONFIG.stops_per_chip and
                           rise_event.slot_blank = expected_blank
                        report "V2-B8-TB Rise ordering mismatch"
                        severity failure;
                    assert to_integer(rise_event.cell.shot_context.request.
                               shot_index) = shot_value
                        report "V2-B8-TB Rise Shot index mismatch"
                        severity failure;
                    if expected_blank = '0' then
                        assert to_integer(rise_event.cell.hits(0)) =
                            fn_expected_hit(shot_value, chip_value,
                                stop_value, GPX_SLOPE_RISE)
                            report "V2-B8-TB Rise payload mismatch"
                            severity failure;
                    else
                        assert rise_event.cell.hit_count = 0 and
                               rise_event.cell.error_fill = '1' and
                               rise_event.cell.faulted = '1'
                            report "V2-B8-TB missing Rise Cell not blank-filled"
                            severity failure;
                    end if;

                    if slot_value = 0 then
                        assert rise_event.line_start = '1'
                            report "V2-B8-TB Rise line_start missing"
                            severity failure;
                        if G_SCENARIO = 0 and rise_line_count = 1 then
                            assert rise_event.gap_before = 1
                                report "V2-B8-TB Rise gap metadata mismatch"
                                severity failure;
                        else
                            assert rise_event.gap_before = 0
                                report "V2-B8-TB unexpected Rise gap"
                                severity failure;
                        end if;
                    else
                        assert rise_event.line_start = '0' and
                               rise_event.gap_before = 0
                            report "V2-B8-TB Rise non-first metadata mismatch"
                            severity failure;
                    end if;

                    if slot_value + 1 = to_integer(rise_event.slot_count) then
                        assert rise_event.line_end = '1'
                            report "V2-B8-TB Rise line_end missing"
                            severity failure;
                        rise_line_count <= rise_line_count + 1;
                    else
                        assert rise_event.line_end = '0'
                            report "V2-B8-TB early Rise line_end"
                            severity failure;
                    end if;
                    if G_SCENARIO = 3 then
                        assert rise_event.line_faulted = '1'
                            report "V2-B8-TB faulted Rise line not marked"
                            severity failure;
                    else
                        assert rise_event.line_faulted = '0'
                            report "V2-B8-TB clean Rise line marked faulted"
                            severity failure;
                    end if;
                    rise_cell_count <= rise_cell_count + 1;
                end if;
            end if;
        end if;
    end process p_rise_monitor;

    p_fall_monitor : process (clk)
        variable slot_value : natural;
        variable chip_value : natural;
        variable stop_value : natural;
        variable shot_value : natural;
        variable held_valid : boolean := false;
        variable held_event : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                fall_cell_count <= 0;
                fall_line_count <= 0;
                held_valid := false;
            else
                if held_valid then
                    assert fall_event = held_event
                        report "V2-B8-TB Fall event changed under backpressure"
                        severity failure;
                end if;
                held_valid := fall_event.valid = '1' and fall_ready = '0';
                if held_valid then
                    held_event := fall_event;
                end if;

                if fall_event.valid = '1' and fall_ready = '1' then
                    slot_value := to_integer(fall_event.slot_index);
                    chip_value := fn_chip_for_slot(C_FALL_MASK, slot_value);
                    stop_value := slot_value mod
                        C_BUILD_CONFIG.stops_per_chip;
                    if G_SCENARIO = 0 and fall_line_count = 1 then
                        shot_value := 2;
                    else
                        shot_value := 0;
                    end if;

                    assert fall_event.cell.slope = GPX_SLOPE_FALL and
                           to_integer(fall_event.cell.chip_index) = chip_value and
                           to_integer(fall_event.cell.stop_index) = stop_value and
                           to_integer(fall_event.slot_count) =
                               fn_popcount(C_FALL_MASK) *
                               C_BUILD_CONFIG.stops_per_chip and
                           fall_event.slot_blank = '0' and
                           to_integer(fall_event.cell.hits(0)) =
                               fn_expected_hit(shot_value, chip_value,
                                   stop_value, GPX_SLOPE_FALL)
                        report "V2-B8-TB Fall ordering/payload mismatch"
                        severity failure;
                    if slot_value = 0 then
                        assert fall_event.line_start = '1'
                            report "V2-B8-TB Fall line_start missing"
                            severity failure;
                        if G_SCENARIO = 0 and fall_line_count = 1 then
                            assert fall_event.gap_before = 1
                                report "V2-B8-TB Fall gap metadata mismatch"
                                severity failure;
                        else
                            assert fall_event.gap_before = 0
                                report "V2-B8-TB unexpected Fall gap"
                                severity failure;
                        end if;
                    end if;
                    if slot_value + 1 = to_integer(fall_event.slot_count) then
                        assert fall_event.line_end = '1'
                            report "V2-B8-TB Fall line_end missing"
                            severity failure;
                        fall_line_count <= fall_line_count + 1;
                    end if;
                    if G_SCENARIO = 3 then
                        assert fall_event.line_faulted = '1'
                            report "V2-B8-TB faulted Fall line not marked"
                            severity failure;
                    else
                        assert fall_event.line_faulted = '0'
                            report "V2-B8-TB clean Fall line marked faulted"
                            severity failure;
                    end if;
                    fall_cell_count <= fall_cell_count + 1;
                end if;
            end if;
        end if;
    end process p_fall_monitor;

    p_done_monitor : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                shot_done_count <= 0;
            elsif shot_done = '1' then
                shot_done_count <= shot_done_count + 1;
            end if;
        end if;
    end process p_done_monitor;

    p_stimulus : process
        procedure wait_clocks(count : natural) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
        end procedure wait_clocks;

        procedure send_event(value : gpx_cell_event_t) is
        begin
            loop
                wait until falling_edge(clk);
                exit when cell_ready = '1';
            end loop;
            cell_event <= value;
            wait until rising_edge(clk);
            cell_event <= C_GPX_CELL_EVENT_IDLE;
        end procedure send_event;

        procedure send_shot(
            shot_index : natural;
            inject_faults : boolean := false
        ) is
            variable expected_mask : chip_mask_t;
            variable value : gpx_cell_event_t;
        begin
            for chip_index in C_MAX_CHIPS - 1 downto 0 loop
                if C_RISE_MASK(chip_index) = '1' then
                    for stop_index in C_BUILD_CONFIG.stops_per_chip - 1
                        downto 0 loop
                        if not (inject_faults and chip_index = 1 and
                                stop_index = 7) then
                            send_event(fn_data_cell(
                                shot_index, C_COLUMNS, chip_index,
                                stop_index, GPX_SLOPE_RISE));
                        end if;
                    end loop;
                end if;
                if C_FALL_MASK(chip_index) = '1' then
                    for stop_index in C_BUILD_CONFIG.stops_per_chip - 1
                        downto 0 loop
                        send_event(fn_data_cell(
                            shot_index, C_COLUMNS, chip_index,
                            stop_index, GPX_SLOPE_FALL));
                    end loop;
                end if;
            end loop;

            if inject_faults then
                value := fn_data_cell(
                    shot_index, C_COLUMNS, 0, 0, GPX_SLOPE_RISE);
                send_event(value);
                value := fn_data_cell(
                    shot_index, C_COLUMNS, 0, 0, GPX_SLOPE_FALL);
                send_event(value);
            end if;

            expected_mask := C_RISE_MASK or C_FALL_MASK;
            for chip_index in C_MAX_CHIPS - 1 downto 0 loop
                if expected_mask(chip_index) = '1' then
                    send_event(fn_terminal(
                        shot_index, C_COLUMNS, chip_index));
                end if;
            end loop;
        end procedure send_shot;

        variable expected_rise_lines : natural;
        variable expected_fall_lines : natural;
        variable expected_shots : natural;
    begin
        rst_n <= '0';
        wait_clocks(5);
        rst_n <= '1';
        wait_clocks(3);

        send_shot(0, G_SCENARIO = 3);
        wait until rising_edge(clk) and shot_done = '1';

        if G_SCENARIO = 0 then
            send_shot(2);
            wait until rising_edge(clk) and shot_done = '1';
            expected_shots := 2;
            expected_rise_lines := 2;
            expected_fall_lines := 2;
        else
            expected_shots := 1;
            expected_rise_lines := 1;
            if C_FALL_MASK = "0000" then
                expected_fall_lines := 0;
            else
                expected_fall_lines := 1;
            end if;
        end if;
        wait_clocks(5);

        assert shot_done_count = expected_shots and
               rise_line_count = expected_rise_lines and
               fall_line_count = expected_fall_lines and
               idle = '1'
            report "V2-B8-TB completion count mismatch"
            severity failure;

        if G_SCENARIO = 0 then
            assert fault_sticky.column_gap = '1' and
                   fault_sticky.context_mismatch = '0' and
                   fault_sticky.unexpected_cell = '0' and
                   fault_sticky.duplicate_cell = '0' and
                   fault_sticky.duplicate_terminal = '0' and
                   fault_sticky.missing_cell = '0' and
                   fault_sticky.geometry_error = '0' and
                   fault_sticky.masked_payload_drop = '0'
                report "V2-B8-TB gap diagnostics mismatch"
                severity failure;
        elsif G_SCENARIO = 3 then
            assert fault_sticky.duplicate_cell = '1' and
                   fault_sticky.missing_cell = '1' and
                   fault_sticky.masked_payload_drop = '1' and
                   fault_sticky.context_mismatch = '0' and
                   fault_sticky.unexpected_cell = '0' and
                   fault_sticky.duplicate_terminal = '0' and
                   fault_sticky.geometry_error = '0'
                report "V2-B8-TB fault diagnostics mismatch"
                severity failure;
        else
            assert fault_sticky = C_GPX_FRAME_ASSEMBLER_FAULTS_CLEAR
                report "V2-B8-TB clean scenario raised a fault"
                severity failure;
        end if;

        report "LIDAR_V2_GPX_FRAME_LANE_ASSEMBLER_PASS"
            severity note;
        finish;
        wait;
    end process p_stimulus;

end architecture sim;

entity tb_lidar_gpx_frame_lane_dedicated_150 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_dedicated_150 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 0);
end architecture;

entity tb_lidar_gpx_frame_lane_dedicated_200 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_dedicated_200 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 0);
end architecture;

entity tb_lidar_gpx_frame_lane_dual_150 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_dual_150 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 1);
end architecture;

entity tb_lidar_gpx_frame_lane_dual_200 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_dual_200 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 1);
end architecture;

entity tb_lidar_gpx_frame_lane_fall_off_150 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_fall_off_150 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 2);
end architecture;

entity tb_lidar_gpx_frame_lane_fall_off_200 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_fall_off_200 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 2);
end architecture;

entity tb_lidar_gpx_frame_lane_faults_150 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_faults_150 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 3);
end architecture;

entity tb_lidar_gpx_frame_lane_faults_200 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_faults_200 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 3);
end architecture;

entity tb_lidar_gpx_frame_lane_dual4_150 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_dual4_150 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 150, G_SCENARIO => 4);
end architecture;

entity tb_lidar_gpx_frame_lane_dual4_200 is end entity;
architecture sim of tb_lidar_gpx_frame_lane_dual4_200 is
begin
    u_tb : entity work.tb_lidar_gpx_frame_lane_assembler
        generic map (G_CLK_MHZ => 200, G_SCENARIO => 4);
end architecture;
