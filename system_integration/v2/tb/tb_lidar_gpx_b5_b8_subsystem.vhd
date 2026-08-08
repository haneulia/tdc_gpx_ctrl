-- ============================================================================
-- 테스트 자산 목적: physical B5 stream부터 B8 Rise/Fall Cell lane까지 end-to-end 검증한다.
-- 핵심 검증 계약: raw28/Hit17/Cell identity, Face close, dedicated/all-dual과 CDC stall이다.
-- 관련 RTL: lidar_gpx_b5_b8_subsystem과 acquisition/hit/cell/frame 계층.
-- 실행 회귀: scripts/run_v2_gpx_b5_b8_subsystem.ps1
-- 유지보수 주의: 하위 단위 TB를 통과한 뒤 150/200·200/150 전체 순서를 비교한다.
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
use work.lidar_gpx_data_pkg.all;
use work.tdc_gpx_pkg.all;

entity tb_lidar_gpx_b5_b8_subsystem is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200;
        G_ALL_DUAL     : boolean := false
    );
end entity tb_lidar_gpx_b5_b8_subsystem;

architecture sim of tb_lidar_gpx_b5_b8_subsystem is

    constant C_CHIPS : positive := 4;
    constant C_STOPS_PER_CHIP : positive := 8;
    constant C_RETURNS_PER_STOP : positive := 7;

    function fn_slopes_per_chip return positive is
    begin
        if G_ALL_DUAL then
            return 2;
        end if;
        return 1;
    end function fn_slopes_per_chip;

    function fn_slots_per_lane return positive is
    begin
        if G_ALL_DUAL then
            return C_CHIPS * C_STOPS_PER_CHIP;
        end if;
        return 2 * C_STOPS_PER_CHIP;
    end function fn_slots_per_lane;

    function fn_rise_capability return chip_mask_t is
    begin
        if G_ALL_DUAL then
            return "1111";
        end if;
        return "0011";
    end function fn_rise_capability;

    function fn_fall_capability return chip_mask_t is
    begin
        if G_ALL_DUAL then
            return "1111";
        end if;
        return "1100";
    end function fn_fall_capability;

    constant C_WORDS_PER_IFIFO : positive :=
        4 * C_RETURNS_PER_STOP * fn_slopes_per_chip;
    constant C_SLOTS_PER_LANE : positive := fn_slots_per_lane;
    constant C_COLUMNS_PER_FACE : positive := 4;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD : time := 1 us / G_TDC_CLK_MHZ;

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => G_PROC_CLK_MHZ,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => C_CHIPS,
        stops_per_chip         => C_STOPS_PER_CHIP,
        max_returns_per_stop   => C_RETURNS_PER_STOP,
        rise_capability_mask   => fn_rise_capability,
        fall_capability_mask   => fn_fall_capability,
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
        runtime.tdc.max_hits_per_stop := to_unsigned(7, 3);
        result.version := to_unsigned(21, 16);
        result.source := runtime;
        result.derived := fn_derive_runtime_config(C_BUILD_CONFIG, runtime);
        result.derived.columns_per_face := to_unsigned(
            C_COLUMNS_PER_FACE, 16);
        return result;
    end function fn_active_config;

    function fn_shot(column_index : natural) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(2, 3);
        result.request.position := to_unsigned(
            3000 + column_index, C_POSITION_WIDTH);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(column_index, 16);
        result.request.last_in_face := '0';
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(8, 8);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(21, 16);
        result.fire_to_t0_clks := to_unsigned(4, 32);
        return result;
    end function fn_shot;

    function fn_close(face_number : natural) return face_close_event_t is
        variable result : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := to_unsigned(100 + face_number, 32);
        result.face_index := to_unsigned(face_number, 3);
        result.direction := DIRECTION_CW;
        result.source_sim := '0';
        result.active_version := to_unsigned(21, 16);
        result.columns_per_face := to_unsigned(C_COLUMNS_PER_FACE, 16);
        return result;
    end function fn_close;

    function fn_raw_word(
        chip_index : natural;
        ififo_id   : std_logic;
        word_index : natural
    ) return gpx_bus_data_t is
        variable result : gpx_bus_data_t := (others => '0');
        variable hit_value : natural;
        variable channel_index : natural;
    begin
        if G_ALL_DUAL then
            channel_index := (word_index / 2) mod 4;
        else
            channel_index := word_index mod 4;
        end if;
        result(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(channel_index, 2));
        result(c_RAW_STARTNUM_HI downto c_RAW_STARTNUM_LO) :=
            (others => '0');
        if G_ALL_DUAL and word_index mod 2 = 0 then
            result(c_RAW_SLOPE_BIT) := '1';
        elsif G_ALL_DUAL then
            result(c_RAW_SLOPE_BIT) := '0';
        elsif chip_index < 2 then
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

    function fn_expected_hit(
        chip_index   : natural;
        stop_index   : natural;
        return_index : natural;
        slope_value  : gpx_slope_t
    ) return gpx_hit_value_t is
        variable word_index : natural;
        variable hit_value  : natural;
        variable local_stop : natural;
    begin
        if stop_index < 4 then
            local_stop := stop_index;
            if G_ALL_DUAL then
                word_index := return_index * 8 + local_stop * 2;
                if slope_value = GPX_SLOPE_FALL then
                    word_index := word_index + 1;
                end if;
            else
                word_index := return_index * 4 + local_stop;
            end if;
            hit_value := chip_index * 16#2000# + word_index;
        else
            local_stop := stop_index - 4;
            if G_ALL_DUAL then
                word_index := return_index * 8 + local_stop * 2;
                if slope_value = GPX_SLOPE_FALL then
                    word_index := word_index + 1;
                end if;
            else
                word_index := return_index * 4 + local_stop;
            end if;
            hit_value := chip_index * 16#2000# + 16#0800# + word_index;
        end if;
        return to_unsigned(hit_value, C_GPX_HIT_WIDTH);
    end function fn_expected_hit;

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

    signal active_config : lidar_active_config_t := fn_active_config;
    signal proc_shot : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal proc_shot_ready : std_logic;
    signal proc_stop_tdc : std_logic := '0';
    signal face_close : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal face_close_ready : std_logic;
    signal rise_event : gpx_frame_cell_event_t;
    signal fall_event : gpx_frame_cell_event_t;
    signal frame_close_event : gpx_frame_close_event_t;
    signal rise_ready : std_logic := '1';
    signal fall_ready : std_logic := '1';
    signal frame_close_ready : std_logic := '1';
    signal rise_line_done : std_logic;
    signal fall_line_done : std_logic;
    signal shot_done : std_logic;
    signal shot_done_context : shot_start_event_t;
    signal proc_idle : std_logic;
    signal outstanding_shots : unsigned(15 downto 0);
    signal context_fault_sticky : std_logic;

    signal config_apply : std_logic := '0';
    signal config_ready : std_logic;
    signal config_done : std_logic;
    signal run_enable : std_logic := '0';
    signal safe : std_logic;
    signal tdc_shot_complete : std_logic;
    signal cdc_reset_busy : std_logic;

    signal adr : gpx_bus_address_array_t;
    signal csn : chip_mask_t;
    signal rdn : chip_mask_t;
    signal wrn : chip_mask_t;
    signal oen : chip_mask_t;
    signal d_from_dut : gpx_bus_data_array_t;
    signal d_tri : chip_mask_t;
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
    signal tdc_status : gpx_lane_status_array_t;
    signal tdc_faults : gpx_lane_faults_array_t;
    signal shot_drop_sticky : std_logic;
    signal stop_drop_sticky : std_logic;
    signal hit_fault_pulse : gpx_hit_decoder_faults_t;
    signal hit_fault_sticky : gpx_hit_decoder_faults_t;
    signal cell_fault_pulse : gpx_cell_collector_faults_t;
    signal cell_fault_sticky : gpx_cell_collector_faults_t;
    signal frame_fault_pulse : gpx_frame_assembler_faults_t;
    signal frame_fault_sticky : gpx_frame_assembler_faults_t;

    signal fifo1_fill : natural_array_t := (others => 0);
    signal fifo2_fill : natural_array_t := (others => 0);
    signal fifo1_read_index : natural_array_t := (others => 0);
    signal fifo2_read_index : natural_array_t := (others => 0);
    signal fifo_load : std_logic := '0';
    signal rise_count : natural := 0;
    signal fall_count : natural := 0;
    signal shot_done_count : natural := 0;
    signal close_count : natural := 0;

begin

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
            when d_tri(index) = '0' else (others => 'Z');
        d_bus(index) <= chip_d_out(index)
            when chip_d_oe(index) = '1' else (others => 'Z');
    end generate gen_pin_models;

    u_dut : entity work.lidar_gpx_b5_b8_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_OEN_MODE => "DYNAMIC_CONNECTED",
            G_DRAIN_MARGIN_TIME_NS => 100
        )
        port map (
            i_proc_clk => proc_clk,
            i_proc_rst_n => proc_rst_n,
            i_proc_abort => '0',
            i_proc_clear_status => '0',
            i_proc_active_valid => '1',
            i_proc_active_config => active_config,
            i_proc_shot => proc_shot,
            o_proc_shot_ready => proc_shot_ready,
            i_proc_stop_tdc => proc_stop_tdc,
            i_face_close_event => face_close,
            o_face_close_ready => face_close_ready,
            o_rise_event => rise_event,
            i_rise_ready => rise_ready,
            o_fall_event => fall_event,
            i_fall_ready => fall_ready,
            o_frame_close_event => frame_close_event,
            i_frame_close_ready => frame_close_ready,
            o_rise_line_done => rise_line_done,
            o_fall_line_done => fall_line_done,
            o_shot_done => shot_done,
            o_shot_done_context => shot_done_context,
            o_proc_idle => proc_idle,
            o_outstanding_shots => outstanding_shots,
            o_context_fault_sticky => context_fault_sticky,
            i_tdc_clk => tdc_clk,
            i_tdc_rst_n => tdc_rst_n,
            i_tdc_active_valid => '1',
            i_tdc_active_config => active_config,
            i_tdc_register_image => C_GPX_REGISTER_IMAGE_DEFAULT,
            i_tdc_config_apply => config_apply,
            o_tdc_config_ready => config_ready,
            o_tdc_config_done => config_done,
            i_tdc_run_enable => run_enable,
            o_tdc_safe => safe,
            o_tdc_shot_complete => tdc_shot_complete,
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
            o_tdc_status => tdc_status,
            o_tdc_faults => tdc_faults,
            o_shot_drop_sticky => shot_drop_sticky,
            o_stop_drop_sticky => stop_drop_sticky,
            o_hit_fault_pulse => hit_fault_pulse,
            o_hit_fault_sticky => hit_fault_sticky,
            o_cell_fault_pulse => cell_fault_pulse,
            o_cell_fault_sticky => cell_fault_sticky,
            o_frame_fault_pulse => frame_fault_pulse,
            o_frame_fault_sticky => frame_fault_sticky
        );

    p_chip_models : process (tdc_clk)
        variable rdn_previous : logic_array_t := (others => '1');
    begin
        if rising_edge(tdc_clk) then
            chip_d_oe <= (others => '0');
            if tdc_rst_n = '0' then
                fifo1_fill <= (others => 0);
                fifo2_fill <= (others => 0);
                fifo1_read_index <= (others => 0);
                fifo2_read_index <= (others => 0);
                rdn_previous := (others => '1');
            else
                for index in 0 to C_CHIPS - 1 loop
                    if fifo_load = '1' then
                        fifo1_fill(index) <= C_WORDS_PER_IFIFO;
                        fifo2_fill(index) <= C_WORDS_PER_IFIFO;
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
                end loop;
            end if;
        end if;
    end process p_chip_models;

    p_scoreboard : process (proc_clk)
        variable slot : natural;
        variable chip : natural;
        variable stop_index : natural;
    begin
        if rising_edge(proc_clk) then
            if proc_rst_n = '0' then
                rise_count <= 0;
                fall_count <= 0;
                shot_done_count <= 0;
                close_count <= 0;
            else
                if rise_event.valid = '1' and rise_ready = '1' then
                    slot := rise_count;
                    chip := slot / C_STOPS_PER_CHIP;
                    stop_index := slot mod C_STOPS_PER_CHIP;
                    assert rise_event.slot_index = to_unsigned(slot, 6) and
                           rise_event.slot_count =
                               to_unsigned(C_SLOTS_PER_LANE, 6) and
                           to_integer(rise_event.cell.chip_index) = chip and
                           to_integer(rise_event.cell.stop_index) = stop_index and
                           rise_event.cell.slope = GPX_SLOPE_RISE and
                           rise_event.cell.hit_count = to_unsigned(7, 3) and
                           rise_event.cell.max_hits = to_unsigned(7, 3) and
                           rise_event.slot_blank = '0' and
                           rise_event.line_faulted = '0' and
                           rise_event.cell.shot_context = fn_shot(1)
                        report "V2-I4-TB Rise Cell metadata mismatch"
                        severity failure;
                    for hit_index in 0 to C_RETURNS_PER_STOP - 1 loop
                        assert rise_event.cell.hits(hit_index) =
                               fn_expected_hit(
                                   chip, stop_index, hit_index,
                                   GPX_SLOPE_RISE)
                            report "V2-I4-TB Rise hit payload mismatch"
                            severity failure;
                    end loop;
                    if slot = 0 then
                        assert rise_event.line_start = '1' and
                               rise_event.gap_before = to_unsigned(1, 16)
                            report "V2-I4-TB Rise leading gap mismatch"
                            severity failure;
                    end if;
                    if slot = C_SLOTS_PER_LANE - 1 then
                        assert rise_event.line_end = '1'
                            report "V2-I4-TB Rise line end missing"
                            severity failure;
                    end if;
                    rise_count <= rise_count + 1;
                end if;

                if fall_event.valid = '1' and fall_ready = '1' then
                    slot := fall_count;
                    if G_ALL_DUAL then
                        chip := slot / C_STOPS_PER_CHIP;
                    else
                        chip := 2 + slot / C_STOPS_PER_CHIP;
                    end if;
                    stop_index := slot mod C_STOPS_PER_CHIP;
                    assert fall_event.slot_index = to_unsigned(slot, 6) and
                           fall_event.slot_count =
                               to_unsigned(C_SLOTS_PER_LANE, 6) and
                           to_integer(fall_event.cell.chip_index) = chip and
                           to_integer(fall_event.cell.stop_index) = stop_index and
                           fall_event.cell.slope = GPX_SLOPE_FALL and
                           fall_event.cell.hit_count = to_unsigned(7, 3) and
                           fall_event.cell.max_hits = to_unsigned(7, 3) and
                           fall_event.slot_blank = '0' and
                           fall_event.line_faulted = '0' and
                           fall_event.cell.shot_context = fn_shot(1)
                        report "V2-I4-TB Fall Cell metadata mismatch"
                        severity failure;
                    for hit_index in 0 to C_RETURNS_PER_STOP - 1 loop
                        assert fall_event.cell.hits(hit_index) =
                               fn_expected_hit(
                                   chip, stop_index, hit_index,
                                   GPX_SLOPE_FALL)
                            report "V2-I4-TB Fall hit payload mismatch"
                            severity failure;
                    end loop;
                    if slot = 0 then
                        assert fall_event.line_start = '1' and
                               fall_event.gap_before = to_unsigned(1, 16)
                            report "V2-I4-TB Fall leading gap mismatch"
                            severity failure;
                    end if;
                    if slot = C_SLOTS_PER_LANE - 1 then
                        assert fall_event.line_end = '1'
                            report "V2-I4-TB Fall line end missing"
                            severity failure;
                    end if;
                    fall_count <= fall_count + 1;
                end if;

                if shot_done = '1' then
                    assert shot_done_context = fn_shot(1)
                        report "V2-I4-TB Shot-done identity mismatch"
                        severity failure;
                    shot_done_count <= shot_done_count + 1;
                end if;

                if frame_close_event.valid = '1' and
                   frame_close_ready = '1' then
                    if close_count = 0 then
                        assert frame_close_event.face_index =
                                   to_unsigned(2, 3) and
                               frame_close_event.face_frame_id =
                                   to_unsigned(102, 32) and
                               frame_close_event.trailing_gap =
                                   to_unsigned(2, 16) and
                               frame_close_event.all_hole = '0' and
                               frame_close_event.face_faulted = '0'
                            report "V2-I4-TB trailing Face-close mismatch"
                            severity failure;
                    elsif close_count = 1 then
                        assert frame_close_event.face_index =
                                   to_unsigned(3, 3) and
                               frame_close_event.face_frame_id =
                                   to_unsigned(103, 32) and
                               frame_close_event.trailing_gap =
                                   to_unsigned(C_COLUMNS_PER_FACE, 16) and
                               frame_close_event.all_hole = '1' and
                               frame_close_event.face_faulted = '0'
                            report "V2-I4-TB all-hole Face-close mismatch"
                            severity failure;
                    else
                        assert false report "V2-I4-TB extra Face-close"
                            severity failure;
                    end if;
                    close_count <= close_count + 1;
                end if;
            end if;
        end if;
    end process p_scoreboard;

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

        procedure send_close(face_number : natural) is
        begin
            face_close <= fn_close(face_number);
            for timeout in 0 to 300000 loop
                wait_proc_clocks(1);
                exit when face_close_ready = '1';
            end loop;
            assert face_close_ready = '1'
                report "V2-I4-TB Face-close acknowledgement timeout"
                severity failure;
            wait_proc_clocks(1);
            face_close <= C_FACE_CLOSE_EVENT_IDLE;
            wait_proc_clocks(2);
        end procedure send_close;

        variable held_close : gpx_frame_close_event_t;
    begin
        proc_rst_n <= '0';
        tdc_rst_n <= '0';
        wait_proc_clocks(8);
        wait_tdc_clocks(8);
        proc_rst_n <= '1';
        tdc_rst_n <= '1';

        for timeout in 0 to 100000 loop
            wait_tdc_clocks(1);
            exit when fn_all_initialized(tdc_status) and
                cdc_reset_busy = '0';
        end loop;
        assert fn_all_initialized(tdc_status) and cdc_reset_busy = '0'
            report "V2-I4-TB initialization timeout"
            severity failure;

        for timeout in 0 to 10000 loop
            wait_tdc_clocks(1);
            exit when config_ready = '1';
        end loop;
        assert config_ready = '1'
            report "V2-I4-TB config-ready timeout"
            severity failure;
        config_apply <= '1';
        wait_tdc_clocks(1);
        config_apply <= '0';
        for timeout in 0 to 10000 loop
            wait_tdc_clocks(1);
            exit when config_done = '1';
        end loop;
        assert config_done = '1'
            report "V2-I4-TB config-done timeout"
            severity failure;

        run_enable <= '1';
        for timeout in 0 to 100000 loop
            wait_tdc_clocks(1);
            exit when fn_all_running(tdc_status);
        end loop;
        assert fn_all_running(tdc_status)
            report "V2-I4-TB run arm timeout"
            severity failure;

        fifo_load <= '1';
        wait_tdc_clocks(1);
        fifo_load <= '0';
        wait_tdc_clocks(4);

        for timeout in 0 to 1000 loop
            exit when proc_shot_ready = '1';
            wait_proc_clocks(1);
        end loop;
        assert proc_shot_ready = '1'
            report "V2-I4-TB Shot ingress timeout"
            severity failure;
        proc_shot <= fn_shot(1);
        wait_proc_clocks(1);
        proc_shot <= C_SHOT_START_EVENT_IDLE;

        for timeout in 0 to 10000 loop
            wait_tdc_clocks(1);
            exit when fn_all_shot_outstanding(tdc_status);
        end loop;
        assert fn_all_shot_outstanding(tdc_status)
            report "V2-I4-TB Shot was not broadcast to all Chips"
            severity failure;

        irflag <= (others => '1');
        wait_tdc_clocks(6);
        irflag <= (others => '0');
        proc_stop_tdc <= '1';
        wait_proc_clocks(2);
        proc_stop_tdc <= '0';

        for timeout in 0 to 500000 loop
            wait_proc_clocks(1);
            exit when shot_done_count = 1 and
                rise_count = C_SLOTS_PER_LANE and
                fall_count = C_SLOTS_PER_LANE;
        end loop;
        assert shot_done_count = 1 and
               rise_count = C_SLOTS_PER_LANE and
               fall_count = C_SLOTS_PER_LANE and
               outstanding_shots = 0
            report "V2-I4-TB B5-B8 Shot closure timeout: shot_done=" &
                integer'image(shot_done_count) & " rise=" &
                integer'image(rise_count) & " fall=" &
                integer'image(fall_count) & " outstanding=" &
                integer'image(to_integer(outstanding_shots))
            severity failure;

        -- Hold the downstream Face-close channel stopped. The Processing
        -- owner must remain unacknowledged and the registered payload stable.
        frame_close_ready <= '0';
        face_close <= fn_close(2);
        for timeout in 0 to 1000 loop
            wait_proc_clocks(1);
            exit when frame_close_event.valid = '1';
        end loop;
        assert frame_close_event.valid = '1' and
               face_close_ready = '0'
            report "V2-I4-TB close backpressure setup failed"
            severity failure;
        held_close := frame_close_event;
        wait_proc_clocks(8);
        assert frame_close_event = held_close and face_close_ready = '0'
            report "V2-I4-TB close payload changed under backpressure"
            severity failure;
        frame_close_ready <= '1';
        for timeout in 0 to 1000 loop
            wait_proc_clocks(1);
            exit when face_close_ready = '1';
        end loop;
        assert face_close_ready = '1'
            report "V2-I4-TB close did not acknowledge after drain"
            severity failure;
        wait_proc_clocks(1);
        face_close <= C_FACE_CLOSE_EVENT_IDLE;
        wait_proc_clocks(2);

        send_close(3);
        for timeout in 0 to 1000 loop
            wait_proc_clocks(1);
            exit when close_count = 2 and proc_idle = '1';
        end loop;

        assert close_count = 2 and proc_idle = '1'
            report "V2-I4-TB all-hole close did not complete"
            severity failure;
        assert shot_drop_sticky = '0' and stop_drop_sticky = '0' and
               context_fault_sticky = '0'
            report "V2-I4-TB integration ordering/drop fault"
            severity failure;
        assert hit_fault_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR and
               cell_fault_sticky = C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
            report "V2-I4-TB Hit/Cell fault mismatch"
            severity failure;
        assert frame_fault_sticky.context_mismatch = '0' and
               frame_fault_sticky.unexpected_cell = '0' and
               frame_fault_sticky.duplicate_cell = '0' and
               frame_fault_sticky.duplicate_terminal = '0' and
               frame_fault_sticky.missing_cell = '0' and
               frame_fault_sticky.geometry_error = '0' and
               frame_fault_sticky.column_gap = '1' and
               frame_fault_sticky.masked_payload_drop = '0'
            report "V2-I4-TB Frame diagnostic mismatch"
            severity failure;
        assert active_mask = "1111" and terminal_mask = "1111"
            report "V2-I4-TB terminal masks mismatch"
            severity failure;

        report "LIDAR_V2_GPX_B5_B8_SUBSYSTEM_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ) & " all_dual=" &
            boolean'image(G_ALL_DUAL)
            severity note;
        simulation_done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_b5_b8_subsystem_150_200_all_dual is
end entity tb_lidar_gpx_b5_b8_subsystem_150_200_all_dual;

architecture sim of tb_lidar_gpx_b5_b8_subsystem_150_200_all_dual is
begin
    u_test : entity work.tb_lidar_gpx_b5_b8_subsystem
        generic map (
            G_PROC_CLK_MHZ => 150,
            G_TDC_CLK_MHZ  => 200,
            G_ALL_DUAL     => true
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_b5_b8_subsystem_200_150_all_dual is
end entity tb_lidar_gpx_b5_b8_subsystem_200_150_all_dual;

architecture sim of tb_lidar_gpx_b5_b8_subsystem_200_150_all_dual is
begin
    u_test : entity work.tb_lidar_gpx_b5_b8_subsystem
        generic map (
            G_PROC_CLK_MHZ => 200,
            G_TDC_CLK_MHZ  => 150,
            G_ALL_DUAL     => true
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_b5_b8_subsystem_150_200 is
end entity tb_lidar_gpx_b5_b8_subsystem_150_200;

architecture sim of tb_lidar_gpx_b5_b8_subsystem_150_200 is
begin
    u_test : entity work.tb_lidar_gpx_b5_b8_subsystem
        generic map (
            G_PROC_CLK_MHZ => 150,
            G_TDC_CLK_MHZ  => 200
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_b5_b8_subsystem_200_150 is
end entity tb_lidar_gpx_b5_b8_subsystem_200_150;

architecture sim of tb_lidar_gpx_b5_b8_subsystem_200_150 is
begin
    u_test : entity work.tb_lidar_gpx_b5_b8_subsystem
        generic map (
            G_PROC_CLK_MHZ => 200,
            G_TDC_CLK_MHZ  => 150
        );
end architecture sim;
