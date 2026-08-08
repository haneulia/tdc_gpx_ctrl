-- ============================================================================
-- 테스트 자산 목적: Rise/Fall Shot/Hole/Footer/width-packer 전체 AXIS 출력 체인을 검증한다.
-- 핵심 검증 계약: dual-lane 독립 stall, SOF/EOL/TKEEP/TLAST, geometry와 Frame 완료이다.
-- 관련 RTL: lidar_gpx_axis_output_subsystem과 Stage J 출력 하위 블록.
-- 실행 회귀: scripts/run_v2_k06_axis_dual_lane.ps1
-- 유지보수 주의: 한 lane stall이 다른 lane 데이터나 Footer 소유권을 오염시키지 않아야 한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_gpx_axis_output_subsystem is
    generic (
        G_PROC_MHZ          : positive := 150;
        G_OUTPUT_WIDTH      : positive := 32;
        -- 0: two dedicated Rise plus two dedicated Fall chips
        -- 1: one chip contributes to both slopes
        -- 2: all four chips contribute to both slopes
        G_TOPOLOGY          : natural range 0 to 2 := 0;
        G_FOOTER_STALL_CLKS : positive := 11
    );
end entity tb_lidar_gpx_axis_output_subsystem;

architecture sim of tb_lidar_gpx_axis_output_subsystem is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.stops_per_chip := 1;
        result.max_returns_per_stop := 7;
        result.output_width := G_OUTPUT_WIDTH;
        result.num_faces := 1;
        case G_TOPOLOGY is
            when 0 =>
                result.num_chips := 4;
                result.rise_capability_mask := "0011";
                result.fall_capability_mask := "1100";
            when 1 =>
                result.num_chips := 1;
                result.rise_capability_mask := "0001";
                result.fall_capability_mask := "0001";
            when others =>
                result.num_chips := 4;
                result.rise_capability_mask := "1111";
                result.fall_capability_mask := "1111";
        end case;
        return result;
    end function fn_build_config;

    function fn_slot_count return positive is
    begin
        case G_TOPOLOGY is
            when 0      => return 2;
            when 1      => return 1;
            when others => return 4;
        end case;
    end function fn_slot_count;

    function fn_chip_index(
        rise_lane : boolean;
        slot      : natural
    ) return natural is
    begin
        if G_TOPOLOGY = 0 and not rise_lane then
            return slot + 2;
        end if;
        return slot;
    end function fn_chip_index;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_SLOT_COUNT : positive := fn_slot_count;
    constant C_CLK_PERIOD : time := 1 us / G_PROC_MHZ;
    constant C_BEAT_BYTES : positive := G_OUTPUT_WIDTH / 8;
    constant C_WORDS_PER_BEAT : positive := G_OUTPUT_WIDTH / 32;
    constant C_CELL_WORDS : positive := fn_gpx_vdma_cell_word_count(1);
    constant C_RAW_LINE_WORDS : positive :=
        C_GPX_VDMA_SHOT_META_WORDS + C_SLOT_COUNT * C_CELL_WORDS;
    constant C_HSIZE_BYTES : positive := fn_gpx_vdma_shot_hsize_bytes(
        C_SLOT_COUNT, 1, G_OUTPUT_WIDTH);
    constant C_FOOTER_LINES : positive := fn_gpx_vdma_footer_lines(
        C_HSIZE_BYTES);
    constant C_VSIZE_LINES : positive := 1 + C_FOOTER_LINES;
    constant C_BEATS_PER_LINE : positive := C_HSIZE_BYTES / C_BEAT_BYTES;
    constant C_EXPECTED_BEATS : positive :=
        C_BEATS_PER_LINE * C_VSIZE_LINES;

    function fn_profile return gpx_vdma_lane_profile_t is
        variable result : gpx_vdma_lane_profile_t :=
            C_GPX_VDMA_LANE_PROFILE_IDLE;
    begin
        result.valid := '1';
        result.enabled := '1';
        result.slot_count := to_unsigned(
            C_SLOT_COUNT, result.slot_count'length);
        result.visible_returns := to_unsigned(
            1, result.visible_returns'length);
        result.cell_word_count := to_unsigned(
            C_CELL_WORDS, result.cell_word_count'length);
        result.planned_shots := to_unsigned(
            1, result.planned_shots'length);
        result.raw_line_words := to_unsigned(
            C_RAW_LINE_WORDS, result.raw_line_words'length);
        result.hsize_bytes := to_unsigned(
            C_HSIZE_BYTES, result.hsize_bytes'length);
        result.hsize_words := to_unsigned(
            C_HSIZE_BYTES / 4, result.hsize_words'length);
        result.footer_lines := to_unsigned(
            C_FOOTER_LINES, result.footer_lines'length);
        result.vsize_lines := to_unsigned(
            C_VSIZE_LINES, result.vsize_lines'length);
        result.stride_bytes := to_unsigned(
            fn_gpx_vdma_stride_bytes(C_SLOT_COUNT, 7, G_OUTPUT_WIDTH),
            result.stride_bytes'length);
        return result;
    end function fn_profile;

    function fn_cell_event(
        rise_lane : boolean;
        slot      : natural
    ) return gpx_frame_cell_event_t is
        variable result : gpx_frame_cell_event_t :=
            C_GPX_FRAME_CELL_EVENT_IDLE;
        variable chip : natural;
        variable hit_value : natural;
    begin
        chip := fn_chip_index(rise_lane, slot);
        if rise_lane then
            hit_value := 16#10001# + slot;
        else
            hit_value := 16#18001# + slot;
        end if;

        result.valid := '1';
        result.cell.valid := '1';
        result.cell.kind := GPX_CELL_DATA;
        result.cell.chip_index := to_unsigned(
            chip, result.cell.chip_index'length);
        result.cell.stop_index := (others => '0');
        if rise_lane then
            result.cell.slope := GPX_SLOPE_RISE;
        else
            result.cell.slope := GPX_SLOPE_FALL;
        end if;
        result.cell.hit_count := to_unsigned(
            1, result.cell.hit_count'length);
        result.cell.max_hits := to_unsigned(
            1, result.cell.max_hits'length);
        result.cell.hits(0) := to_unsigned(
            hit_value, result.cell.hits(0)'length);
        result.cell.shot_context.valid := '1';
        result.cell.shot_context.request.valid := '1';
        result.cell.shot_context.request.face_index := (others => '0');
        result.cell.shot_context.request.position := to_unsigned(
            16#0123#, result.cell.shot_context.request.position'length);
        result.cell.shot_context.request.direction := DIRECTION_CW;
        result.cell.shot_context.request.shot_index := (others => '0');
        result.cell.shot_context.request.last_in_face := '1';
        result.cell.shot_context.request.source_sim := '1';
        result.cell.shot_context.request.source_latency_clks :=
            to_unsigned(2, C_POSITION_LATENCY_WIDTH);
        result.cell.shot_context.request.source_latency_valid := '1';
        result.cell.shot_context.request.active_version := x"0042";
        result.cell.shot_context.fire_to_t0_clks := to_unsigned(3, 32);
        result.cell.shot_context.t0_timestamp_ticks :=
            to_unsigned(16#76543210#, C_T0_TIMESTAMP_WIDTH);
        result.cell.shot_context.t0_timestamp_valid := '1';
        result.cell.shot_context.t0_time_sync_valid := '1';
        result.cell.chip_shot_seq := to_unsigned(
            1, result.cell.chip_shot_seq'length);
        result.slot_index := to_unsigned(slot, result.slot_index'length);
        result.slot_count := to_unsigned(
            C_SLOT_COUNT, result.slot_count'length);
        if slot = 0 then
            result.line_start := '1';
        end if;
        if slot + 1 = C_SLOT_COUNT then
            result.line_end := '1';
        end if;
        result.first_column := '1';
        result.last_column := '1';
        return result;
    end function fn_cell_event;

    function fn_close_event return gpx_frame_close_event_t is
        variable result : gpx_frame_close_event_t :=
            C_GPX_FRAME_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := x"01020304";
        result.face_index := (others => '0');
        result.direction := DIRECTION_CW;
        result.source_sim := '1';
        result.active_version := x"0042";
        result.columns_per_face := to_unsigned(
            1, result.columns_per_face'length);
        return result;
    end function fn_close_event;

    constant C_PROFILE : gpx_vdma_lane_profile_t := fn_profile;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_run : std_logic := '0';

    signal rise_event : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal rise_ready : std_logic;
    signal fall_event : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal fall_ready : std_logic;
    signal close_event : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal close_ready : std_logic;
    signal frame_output_done : std_logic;

    signal rise_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal rise_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal rise_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal rise_tuser : std_logic_vector(0 downto 0);
    signal rise_tvalid : std_logic;
    signal rise_tlast : std_logic;
    signal rise_tready : std_logic := '0';
    signal fall_tdata : std_logic_vector(G_OUTPUT_WIDTH - 1 downto 0);
    signal fall_tkeep : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal fall_tstrb : std_logic_vector(G_OUTPUT_WIDTH / 8 - 1 downto 0);
    signal fall_tuser : std_logic_vector(0 downto 0);
    signal fall_tvalid : std_logic;
    signal fall_tlast : std_logic;
    signal fall_tready : std_logic := '0';

    signal rise_line_done : std_logic;
    signal fall_line_done : std_logic;
    signal rise_frame_done : std_logic;
    signal fall_frame_done : std_logic;
    signal output_idle : std_logic;

    signal fall_stall_started_r : std_logic := '0';
    signal fall_stall_remaining_r : natural range 0 to
        G_FOOTER_STALL_CLKS := 0;
    signal fall_stall_observed_r : natural := 0;
    signal rise_beats_r : natural := 0;
    signal fall_beats_r : natural := 0;
    signal rise_lines_r : natural := 0;
    signal fall_lines_r : natural := 0;
    signal rise_sof_r : natural := 0;
    signal fall_sof_r : natural := 0;
    signal rise_magic_r : natural := 0;
    signal fall_magic_r : natural := 0;
    signal rise_commit_r : natural := 0;
    signal fall_commit_r : natural := 0;
    signal rise_frame_seen_r : std_logic := '0';
    signal fall_frame_seen_r : std_logic := '0';
    signal global_done_seen_r : std_logic := '0';

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_axis_output_subsystem
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_abort => abort_run,
            i_rise_active_profile => C_PROFILE,
            i_fall_active_profile => C_PROFILE,
            i_rise_event => rise_event,
            o_rise_ready => rise_ready,
            i_fall_event => fall_event,
            o_fall_ready => fall_ready,
            i_frame_close_event => close_event,
            o_frame_close_ready => close_ready,
            o_frame_output_done => frame_output_done,
            o_rise_tdata => rise_tdata,
            o_rise_tkeep => rise_tkeep,
            o_rise_tstrb => rise_tstrb,
            o_rise_tuser => rise_tuser,
            o_rise_tvalid => rise_tvalid,
            o_rise_tlast => rise_tlast,
            i_rise_tready => rise_tready,
            o_fall_tdata => fall_tdata,
            o_fall_tkeep => fall_tkeep,
            o_fall_tstrb => fall_tstrb,
            o_fall_tuser => fall_tuser,
            o_fall_tvalid => fall_tvalid,
            o_fall_tlast => fall_tlast,
            i_fall_tready => fall_tready,
            o_rise_line_done => rise_line_done,
            o_fall_line_done => fall_line_done,
            o_rise_frame_done => rise_frame_done,
            o_fall_frame_done => fall_frame_done,
            o_idle => output_idle
        );

    rise_tready <= rst_n;

    p_fall_backpressure : process (clk)
    begin
        -- Testbench-only half-cycle control lets TREADY drop before the
        -- rising-edge handshake of the first Footer Beat at every width.
        if falling_edge(clk) then
            if rst_n = '0' then
                fall_tready <= '0';
                fall_stall_started_r <= '0';
                fall_stall_remaining_r <= 0;
            else
                fall_tready <= '1';
                if fall_stall_remaining_r > 0 then
                    fall_tready <= '0';
                    fall_stall_remaining_r <= fall_stall_remaining_r - 1;
                elsif fall_tvalid = '1' and fall_tready = '1' and
                      fall_tdata(31 downto 0) = C_GPX_VDMA_FOOTER_MAGIC and
                      fall_stall_started_r = '0' then
                    fall_tready <= '0';
                    fall_stall_started_r <= '1';
                    fall_stall_remaining_r <= G_FOOTER_STALL_CLKS - 1;
                end if;
            end if;
        end if;
    end process p_fall_backpressure;

    p_monitor : process (clk)
        variable held_valid_v : boolean := false;
        variable held_data_v : std_logic_vector(
            G_OUTPUT_WIDTH - 1 downto 0) := (others => '0');
        variable held_keep_v : std_logic_vector(
            G_OUTPUT_WIDTH / 8 - 1 downto 0) := (others => '0');
        variable held_last_v : std_logic := '0';
        variable held_user_v : std_logic := '0';
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                rise_beats_r <= 0;
                fall_beats_r <= 0;
                rise_lines_r <= 0;
                fall_lines_r <= 0;
                rise_sof_r <= 0;
                fall_sof_r <= 0;
                rise_magic_r <= 0;
                fall_magic_r <= 0;
                rise_commit_r <= 0;
                fall_commit_r <= 0;
                fall_stall_observed_r <= 0;
                rise_frame_seen_r <= '0';
                fall_frame_seen_r <= '0';
                global_done_seen_r <= '0';
                held_valid_v := false;
            else
                if rise_tvalid = '1' and rise_tready = '1' then
                    rise_beats_r <= rise_beats_r + 1;
                    assert rise_tkeep = (rise_tkeep'range => '1') and
                           rise_tstrb = (rise_tstrb'range => '1')
                        report "V2-K06-DUAL-TB Rise TKEEP/TSTRB is not full"
                        severity failure;
                    if rise_tlast = '1' then
                        rise_lines_r <= rise_lines_r + 1;
                    end if;
                    if rise_tuser(0) = '1' then
                        rise_sof_r <= rise_sof_r + 1;
                    end if;
                    for word_index in 0 to C_WORDS_PER_BEAT - 1 loop
                        if rise_tdata((word_index + 1) * 32 - 1 downto
                                     word_index * 32) =
                           C_GPX_VDMA_FOOTER_MAGIC then
                            rise_magic_r <= rise_magic_r + 1;
                        end if;
                        if rise_tdata((word_index + 1) * 32 - 1 downto
                                     word_index * 32) =
                           C_GPX_VDMA_FOOTER_COMMIT then
                            rise_commit_r <= rise_commit_r + 1;
                        end if;
                    end loop;
                end if;

                if fall_tvalid = '1' and fall_tready = '1' then
                    fall_beats_r <= fall_beats_r + 1;
                    assert fall_tkeep = (fall_tkeep'range => '1') and
                           fall_tstrb = (fall_tstrb'range => '1')
                        report "V2-K06-DUAL-TB Fall TKEEP/TSTRB is not full"
                        severity failure;
                    if fall_tlast = '1' then
                        fall_lines_r <= fall_lines_r + 1;
                    end if;
                    if fall_tuser(0) = '1' then
                        fall_sof_r <= fall_sof_r + 1;
                    end if;
                    for word_index in 0 to C_WORDS_PER_BEAT - 1 loop
                        if fall_tdata((word_index + 1) * 32 - 1 downto
                                     word_index * 32) =
                           C_GPX_VDMA_FOOTER_MAGIC then
                            fall_magic_r <= fall_magic_r + 1;
                        end if;
                        if fall_tdata((word_index + 1) * 32 - 1 downto
                                     word_index * 32) =
                           C_GPX_VDMA_FOOTER_COMMIT then
                            fall_commit_r <= fall_commit_r + 1;
                        end if;
                    end loop;
                end if;

                if fall_tvalid = '1' and fall_tready = '0' then
                    fall_stall_observed_r <= fall_stall_observed_r + 1;
                    assert frame_output_done = '0'
                        report "V2-K06-DUAL-TB global done asserted during Fall stall"
                        severity failure;
                    if held_valid_v then
                        assert fall_tdata = held_data_v and
                               fall_tkeep = held_keep_v and
                               fall_tlast = held_last_v and
                               fall_tuser(0) = held_user_v
                            report "V2-K06-DUAL-TB Fall AXIS changed under stall"
                            severity failure;
                    end if;
                    held_valid_v := true;
                    held_data_v := fall_tdata;
                    held_keep_v := fall_tkeep;
                    held_last_v := fall_tlast;
                    held_user_v := fall_tuser(0);
                else
                    held_valid_v := false;
                end if;

                if rise_frame_done = '1' then
                    rise_frame_seen_r <= '1';
                end if;
                if fall_frame_done = '1' then
                    fall_frame_seen_r <= '1';
                end if;
                if frame_output_done = '1' then
                    assert rise_frame_seen_r = '1' and
                           (fall_frame_seen_r = '1' or fall_frame_done = '1')
                        report "V2-K06-DUAL-TB global completion preceded a lane"
                        severity failure;
                    global_done_seen_r <= '1';
                end if;
            end if;
        end if;
    end process p_monitor;

    p_test : process
        procedure send_cell(
            signal event_value : out gpx_frame_cell_event_t;
            signal ready_value : in std_logic;
            constant value     : in gpx_frame_cell_event_t
        ) is
        begin
            wait until falling_edge(clk);
            event_value <= value;
            loop
                wait until rising_edge(clk);
                exit when ready_value = '1';
            end loop;
            wait until falling_edge(clk);
            event_value <= C_GPX_FRAME_CELL_EVENT_IDLE;
        end procedure send_cell;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until falling_edge(clk);
        rst_n <= '1';

        for slot in 0 to C_SLOT_COUNT - 1 loop
            send_cell(rise_event, rise_ready, fn_cell_event(true, slot));
        end loop;
        for slot in 0 to C_SLOT_COUNT - 1 loop
            send_cell(fall_event, fall_ready, fn_cell_event(false, slot));
        end loop;

        wait until falling_edge(clk);
        close_event <= fn_close_event;
        loop
            wait until rising_edge(clk);
            exit when close_ready = '1';
        end loop;
        wait until falling_edge(clk);
        close_event <= C_GPX_FRAME_CLOSE_EVENT_IDLE;

        for timeout_index in 0 to 10000 loop
            wait until falling_edge(clk);
            exit when global_done_seen_r = '1';
            assert timeout_index < 10000
                report "V2-K06-DUAL-TB timed out waiting for completion"
                severity failure;
        end loop;
        wait until rising_edge(clk);
        wait until falling_edge(clk);

        assert fall_stall_started_r = '1' and
               fall_stall_observed_r >= G_FOOTER_STALL_CLKS - 1
            report "V2-K06-DUAL-TB did not exercise Fall Footer stall"
            severity failure;
        assert rise_beats_r = C_EXPECTED_BEATS and
               fall_beats_r = C_EXPECTED_BEATS
            report "V2-K06-DUAL-TB accepted Beat count mismatch rise=" &
                integer'image(rise_beats_r) & " fall=" &
                integer'image(fall_beats_r) & " expected=" &
                integer'image(C_EXPECTED_BEATS) & " rise_lines=" &
                integer'image(rise_lines_r) & " fall_lines=" &
                integer'image(fall_lines_r) & " rise_magic=" &
                integer'image(rise_magic_r) & " fall_magic=" &
                integer'image(fall_magic_r) & " rise_commit=" &
                integer'image(rise_commit_r) & " fall_commit=" &
                integer'image(fall_commit_r)
            severity failure;
        assert rise_lines_r = C_VSIZE_LINES and
               fall_lines_r = C_VSIZE_LINES
            report "V2-K06-DUAL-TB Line/TLAST count mismatch"
            severity failure;
        assert rise_sof_r = 1 and fall_sof_r = 1
            report "V2-K06-DUAL-TB SOF count mismatch"
            severity failure;
        assert rise_magic_r = 1 and fall_magic_r = 1 and
               rise_commit_r = 1 and fall_commit_r = 1
            report "V2-K06-DUAL-TB Footer Magic/Commit mismatch"
            severity failure;
        assert rise_frame_seen_r = '1' and fall_frame_seen_r = '1' and
               global_done_seen_r = '1'
            report "V2-K06-DUAL-TB completion pulse mismatch"
            severity failure;
        assert output_idle = '1' and close_ready = '1'
            report "V2-K06-DUAL-TB output owner did not return idle"
            severity failure;

        report "LIDAR_V2_GPX_AXIS_OUTPUT_DUAL_PASS proc_mhz=" &
            integer'image(G_PROC_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH) & " topology=" &
            integer'image(G_TOPOLOGY) severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;
