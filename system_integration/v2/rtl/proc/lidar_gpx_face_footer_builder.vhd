library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- J7 passes complete Shot/Hole Lines and appends the fixed 8-Word Face Footer.
-- Runtime geometry is precomputed at a Face boundary and arrives as one
-- registered profile. The streaming path contains counters and comparisons
-- only; it performs no variable divide, modulo, or multiply operation.
entity lidar_gpx_face_footer_builder is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_LANE_RISE    : boolean := true
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_active_profile : in gpx_vdma_lane_profile_t;

        i_line_word       : in  gpx_vdma_line_word_event_t;
        o_line_word_ready : out std_logic;

        i_frame_close_event : in  gpx_frame_close_event_t;
        o_frame_close_ready : out std_logic;

        o_line_word       : out gpx_vdma_line_word_event_t;
        i_line_word_ready : in  std_logic;

        o_active_hsize_bytes : out gpx_vdma_geometry_value_t;
        o_active_vsize_lines : out gpx_vdma_geometry_value_t;
        o_stride_bytes       : out gpx_vdma_geometry_value_t;
        o_footer_emitted     : out std_logic;
        o_idle               : out std_logic
    );
end entity lidar_gpx_face_footer_builder;

architecture rtl of lidar_gpx_face_footer_builder is

    function fn_lane_slope return gpx_slope_t is
    begin
        if G_LANE_RISE then
            return GPX_SLOPE_RISE;
        end if;
        return GPX_SLOPE_FALL;
    end function fn_lane_slope;

    type state_t is (ST_PASS, ST_FOOTER, ST_FOOTER_LAST_WAIT);

    signal state_r : state_t := ST_PASS;
    signal line_word_r : gpx_vdma_line_word_event_t :=
        C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
    signal output_ready_c : std_logic;
    signal input_line_active_r : std_logic := '0';

    signal completed_shots_r : shot_index_t := (others => '0');
    signal any_line_fault_r  : std_logic := '0';
    signal any_hole_r        : std_logic := '0';
    signal any_timeout_r     : std_logic := '0';
    signal any_aborted_r     : std_logic := '0';

    signal footer_context_r : gpx_vdma_footer_context_t :=
        C_GPX_VDMA_FOOTER_CONTEXT_IDLE;
    signal footer_profile_r : gpx_vdma_lane_profile_t :=
        C_GPX_VDMA_LANE_PROFILE_IDLE;
    signal footer_logical_word_r : natural range 0 to
        C_GPX_VDMA_FOOTER_WORDS := 0;
    signal footer_line_word_r : gpx_vdma_line_word_index_t :=
        (others => '0');
    signal footer_lines_remaining_r : unsigned(1 downto 0) :=
        (others => '0');
    signal footer_emitted_r : std_logic := '0';

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-B9-J7-001 invalid build configuration"
        severity failure;

    output_ready_c <= '1' when line_word_r.valid = '0' or
        i_line_word_ready = '1' else '0';

    o_line_word_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
        state_r = ST_PASS and output_ready_c = '1' and
        i_frame_close_event.valid = '0' else '0';

    o_frame_close_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
        state_r = ST_PASS and input_line_active_r = '0' and
        line_word_r.valid = '0' and i_line_word.valid = '0' and
        i_active_profile.valid = '1' and
        i_active_profile.enabled = '1' else '0';

    o_line_word <= line_word_r;
    o_active_hsize_bytes <= i_active_profile.hsize_bytes;
    o_active_vsize_lines <= i_active_profile.vsize_lines;
    o_stride_bytes <= i_active_profile.stride_bytes;
    o_footer_emitted <= footer_emitted_r;
    o_idle <= '1' when state_r = ST_PASS and input_line_active_r = '0' and
        line_word_r.valid = '0' and completed_shots_r = 0 else '0';

    p_build : process (i_clk)
        variable next_word_v : gpx_vdma_line_word_event_t;
        variable context_v : gpx_vdma_footer_context_t;
        variable line_last_v : boolean;
        variable frame_last_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                state_r <= ST_PASS;
                line_word_r <= C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
                input_line_active_r <= '0';
                completed_shots_r <= (others => '0');
                any_line_fault_r <= '0';
                any_hole_r <= '0';
                any_timeout_r <= '0';
                any_aborted_r <= '0';
                footer_context_r <= C_GPX_VDMA_FOOTER_CONTEXT_IDLE;
                footer_profile_r <= C_GPX_VDMA_LANE_PROFILE_IDLE;
                footer_logical_word_r <= 0;
                footer_line_word_r <= (others => '0');
                footer_lines_remaining_r <= (others => '0');
                footer_emitted_r <= '0';
            else
                footer_emitted_r <= '0';
                if line_word_r.valid = '1' and i_line_word_ready = '1' then
                    line_word_r.valid <= '0';
                end if;

                case state_r is
                    when ST_PASS =>
                        if i_line_word.valid = '1' and
                           o_line_word_ready = '1' then
                            next_word_v := i_line_word;
                            next_word_v.frame_end := '0';
                            line_word_r <= next_word_v;

                            if i_line_word.line_start = '1' then
                                input_line_active_r <=
                                    not i_line_word.line_end;
                            elsif i_line_word.line_end = '1' then
                                input_line_active_r <= '0';
                            end if;

                            if i_line_word.line_hole = '1' then
                                any_hole_r <= '1';
                            end if;
                            if i_line_word.line_faulted = '1' then
                                any_line_fault_r <= '1';
                            end if;
                            if i_line_word.kind =
                                   GPX_VDMA_LINE_SHOT_METADATA and
                               i_line_word.word_index =
                                   C_GPX_VDMA_SHOT_META_WORDS - 1 then
                                if i_line_word.data(
                                       C_GPX_SHOT_META_TIMEOUT) = '1' then
                                    any_timeout_r <= '1';
                                end if;
                                if i_line_word.data(
                                       C_GPX_SHOT_META_ABORTED) = '1' then
                                    any_aborted_r <= '1';
                                end if;
                            end if;
                            if i_line_word.line_end = '1' then
                                completed_shots_r <= completed_shots_r + 1;
                            end if;

                        elsif i_frame_close_event.valid = '1' and
                              o_frame_close_ready = '1' then
                            context_v := C_GPX_VDMA_FOOTER_CONTEXT_IDLE;
                            context_v.frame_close := i_frame_close_event;
                            context_v.slope := fn_lane_slope;
                            context_v.output_width_code :=
                                fn_gpx_vdma_output_width_code(
                                    G_BUILD_CONFIG.output_width);
                            context_v.slot_count :=
                                i_active_profile.slot_count;
                            context_v.visible_returns :=
                                i_active_profile.visible_returns;
                            context_v.hsize_bytes :=
                                i_active_profile.hsize_bytes;
                            context_v.vsize_lines :=
                                i_active_profile.vsize_lines;
                            context_v.completed_shots := completed_shots_r;
                            if completed_shots_r /=
                               i_frame_close_event.columns_per_face then
                                context_v.count_mismatch := '1';
                            end if;
                            context_v.any_line_fault := any_line_fault_r;
                            context_v.any_hole := any_hole_r;
                            context_v.any_timeout := any_timeout_r;
                            context_v.any_aborted := any_aborted_r;

                            footer_context_r <= context_v;
                            footer_profile_r <= i_active_profile;
                            footer_logical_word_r <= 0;
                            footer_line_word_r <= (others => '0');
                            footer_lines_remaining_r <=
                                i_active_profile.footer_lines;
                            state_r <= ST_FOOTER;
                        end if;

                    when ST_FOOTER =>
                        if output_ready_c = '1' then
                            line_last_v := footer_line_word_r + 1 =
                                footer_profile_r.hsize_words;
                            frame_last_v := line_last_v and
                                footer_lines_remaining_r = 1;

                            next_word_v := C_GPX_VDMA_LINE_WORD_EVENT_IDLE;
                            next_word_v.valid := '1';
                            next_word_v.kind := GPX_VDMA_LINE_FACE_FOOTER;
                            if footer_logical_word_r <
                               C_GPX_VDMA_FOOTER_WORDS then
                                next_word_v.data := fn_gpx_vdma_footer_word(
                                    footer_context_r,
                                    footer_logical_word_r);
                            end if;
                            next_word_v.word_index := footer_line_word_r;
                            next_word_v.line_word_count :=
                                footer_profile_r.hsize_words;
                            if footer_line_word_r = 0 then
                                next_word_v.line_start := '1';
                            end if;
                            if line_last_v then
                                next_word_v.line_end := '1';
                            end if;
                            if frame_last_v then
                                next_word_v.frame_end := '1';
                            end if;
                            next_word_v.slot_count :=
                                footer_profile_r.slot_count;
                            next_word_v.cell_word_count :=
                                footer_profile_r.cell_word_count;
                            line_word_r <= next_word_v;

                            if footer_logical_word_r <
                               C_GPX_VDMA_FOOTER_WORDS then
                                footer_logical_word_r <=
                                    footer_logical_word_r + 1;
                            end if;

                            if frame_last_v then
                                state_r <= ST_FOOTER_LAST_WAIT;
                            elsif line_last_v then
                                footer_line_word_r <= (others => '0');
                                footer_lines_remaining_r <=
                                    footer_lines_remaining_r - 1;
                            else
                                footer_line_word_r <=
                                    footer_line_word_r + 1;
                            end if;
                        end if;

                    when ST_FOOTER_LAST_WAIT =>
                        if line_word_r.valid = '1' and
                           i_line_word_ready = '1' then
                            footer_emitted_r <= '1';
                            completed_shots_r <= (others => '0');
                            any_line_fault_r <= '0';
                            any_hole_r <= '0';
                            any_timeout_r <= '0';
                            any_aborted_r <= '0';
                            footer_context_r <=
                                C_GPX_VDMA_FOOTER_CONTEXT_IDLE;
                            footer_profile_r <=
                                C_GPX_VDMA_LANE_PROFILE_IDLE;
                            state_r <= ST_PASS;
                        end if;
                end case;
            end if;
        end if;
    end process p_build;

    -- synthesis translate_off
    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if i_line_word.valid = '1' and
                   o_line_word_ready = '1' then
                    assert i_active_profile.valid = '1' and
                           i_active_profile.enabled = '1'
                        report "V2-B9-J7-003 Line arrived without active profile"
                        severity failure;
                    assert i_line_word.slot_count =
                               i_active_profile.slot_count and
                           i_line_word.cell_word_count =
                               i_active_profile.cell_word_count and
                           i_line_word.line_word_count =
                               i_active_profile.raw_line_words
                        report "V2-B9-J7-005 active Line geometry mismatch"
                        severity failure;
                    assert i_line_word.frame_end = '0'
                        report "V2-B9-J7-006 upstream asserted Frame end"
                        severity failure;
                end if;

                if i_frame_close_event.valid = '1' and
                   o_frame_close_ready = '1' then
                    assert i_frame_close_event.columns_per_face /= 0 and
                           i_frame_close_event.columns_per_face =
                               i_active_profile.planned_shots
                        report "V2-B9-J7-007 planned Shot/profile mismatch"
                        severity failure;
                    assert i_active_profile.footer_lines >= 1 and
                           i_active_profile.hsize_words >= 1 and
                           i_active_profile.hsize_bytes <=
                               i_active_profile.stride_bytes
                        report "V2-B9-J7-008 invalid active VDMA profile"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;
    -- synthesis translate_on

end architecture rtl;
