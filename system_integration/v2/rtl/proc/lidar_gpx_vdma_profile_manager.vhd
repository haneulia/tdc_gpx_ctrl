library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

-- J8 owns one Rise or Fall VDMA geometry profile. It derives geometry over
-- multiple clocks, waits for a safe Face boundary, and changes Active only
-- after the external VDMA configuration owner acknowledges programming.
entity lidar_gpx_vdma_profile_manager is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_LANE_RISE    : boolean := true
    );
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_request_valid   : in  std_logic;
        o_request_ready   : out std_logic;
        i_lane_chip_mask  : in  chip_mask_t;
        i_visible_returns : in  unsigned(2 downto 0);
        i_planned_shots   : in  shot_index_t;
        o_request_rejected : out std_logic;

        i_activate_valid : in  std_logic;
        o_activate_ready : out std_logic;
        i_datapath_idle  : in  std_logic;

        o_vdma_cfg_valid   : out std_logic;
        i_vdma_cfg_ready   : in  std_logic;
        o_vdma_cfg_enable  : out std_logic;
        o_vdma_hsize_bytes : out gpx_vdma_geometry_value_t;
        o_vdma_vsize_lines : out gpx_vdma_geometry_value_t;
        o_vdma_stride_bytes : out gpx_vdma_geometry_value_t;

        o_active_profile   : out gpx_vdma_lane_profile_t;
        o_profile_activated : out std_logic;
        o_pending_valid     : out std_logic;
        o_busy              : out std_logic
    );
end entity lidar_gpx_vdma_profile_manager;

architecture rtl of lidar_gpx_vdma_profile_manager is

    function fn_lane_max_slots return natural is
        variable present_mask : chip_mask_t;
    begin
        present_mask := fn_present_chip_mask(G_BUILD_CONFIG.num_chips);
        if G_LANE_RISE then
            return G_BUILD_CONFIG.num_chips *
                G_BUILD_CONFIG.stops_per_chip;
        end if;
        return fn_popcount(
            G_BUILD_CONFIG.fall_capability_mask and present_mask) *
            G_BUILD_CONFIG.stops_per_chip;
    end function fn_lane_max_slots;

    function fn_allowed_mask return chip_mask_t is
    begin
        if G_LANE_RISE then
            return fn_present_chip_mask(G_BUILD_CONFIG.num_chips);
        end if;
        return G_BUILD_CONFIG.fall_capability_mask and
            fn_present_chip_mask(G_BUILD_CONFIG.num_chips);
    end function fn_allowed_mask;

    function fn_cell_words(
        return_count : unsigned(2 downto 0)
    ) return gpx_vdma_word_count_t is
        variable result : natural := 0;
    begin
        case to_integer(return_count) is
            when 1 | 2 => result := 2;
            when 3 | 4 => result := 3;
            when 5 | 6 => result := 4;
            when 7     => result := 5;
            when others => result := 0;
        end case;
        return to_unsigned(result, gpx_vdma_word_count_t'length);
    end function fn_cell_words;

    function fn_align_words(
        raw_words : gpx_vdma_line_word_count_t
    ) return gpx_vdma_line_word_count_t is
        variable result : gpx_vdma_line_word_count_t := raw_words;
    begin
        case G_BUILD_CONFIG.output_width is
            when 32 =>
                null;
            when 64 =>
                if raw_words(0) = '1' then
                    result := raw_words + 1;
                end if;
            when 128 =>
                case raw_words(1 downto 0) is
                    when "01" => result := raw_words + 3;
                    when "10" => result := raw_words + 2;
                    when "11" => result := raw_words + 1;
                    when others => null;
                end case;
            when others =>
                null;
        end case;
        return result;
    end function fn_align_words;

    constant C_MAX_SLOT_COUNT : natural := fn_lane_max_slots;
    constant C_ALLOWED_MASK : chip_mask_t := fn_allowed_mask;
    constant C_STRIDE_BYTES : natural := fn_gpx_vdma_stride_bytes(
        C_MAX_SLOT_COUNT,
        G_BUILD_CONFIG.max_returns_per_stop,
        G_BUILD_CONFIG.output_width);

    type state_t is (
        ST_IDLE,
        ST_COUNT_CHIPS,
        ST_ACCUMULATE_CELLS,
        ST_FINALIZE,
        ST_WAIT_ACTIVATE,
        ST_PROGRAM
    );

    signal state_r : state_t := ST_IDLE;
    signal request_rejected_r : std_logic := '0';
    signal profile_activated_r : std_logic := '0';

    signal request_mask_r : chip_mask_t := (others => '0');
    signal request_returns_r : unsigned(2 downto 0) := (others => '0');
    signal request_shots_r : shot_index_t := (others => '0');
    signal cell_words_r : gpx_vdma_word_count_t := (others => '0');

    signal chip_index_r : natural range 0 to C_MAX_CHIPS := 0;
    signal slot_count_r : natural range 0 to C_GPX_VDMA_MAX_LINE_SLOTS := 0;
    signal slots_remaining_r : natural range 0 to
        C_GPX_VDMA_MAX_LINE_SLOTS := 0;
    signal raw_line_words_r : gpx_vdma_line_word_count_t :=
        (others => '0');

    signal pending_profile_r : gpx_vdma_lane_profile_t :=
        C_GPX_VDMA_LANE_PROFILE_IDLE;
    signal active_profile_r : gpx_vdma_lane_profile_t :=
        C_GPX_VDMA_LANE_PROFILE_IDLE;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-B9-J8-001 invalid build configuration"
        severity failure;
    assert C_STRIDE_BYTES <= 65_535
        report "V2-B9-J8-002 fixed STRIDE exceeds 16-bit geometry"
        severity failure;

    o_request_ready <= '1' when state_r = ST_IDLE else '0';
    o_activate_ready <= '1' when state_r = ST_WAIT_ACTIVATE and
        i_datapath_idle = '1' else '0';

    o_vdma_cfg_valid <= '1' when state_r = ST_PROGRAM else '0';
    o_vdma_cfg_enable <= pending_profile_r.enabled;
    o_vdma_hsize_bytes <= pending_profile_r.hsize_bytes;
    o_vdma_vsize_lines <= pending_profile_r.vsize_lines;
    o_vdma_stride_bytes <= pending_profile_r.stride_bytes;

    o_active_profile <= active_profile_r;
    o_profile_activated <= profile_activated_r;
    o_request_rejected <= request_rejected_r;
    o_pending_valid <= '1' when state_r = ST_WAIT_ACTIVATE or
        state_r = ST_PROGRAM else '0';
    o_busy <= '0' when state_r = ST_IDLE else '1';

    p_profile : process (i_clk)
        variable next_slot_count_v : natural range 0 to
            C_GPX_VDMA_MAX_LINE_SLOTS;
        variable aligned_words_v : gpx_vdma_line_word_count_t;
        variable footer_lines_v : unsigned(1 downto 0);
        variable vsize_ext_v : unsigned(16 downto 0);
        variable profile_v : gpx_vdma_lane_profile_t;
    begin
        if rising_edge(i_clk) then
            request_rejected_r <= '0';
            profile_activated_r <= '0';

            if i_rst_n = '0' then
                state_r <= ST_IDLE;
                request_mask_r <= (others => '0');
                request_returns_r <= (others => '0');
                request_shots_r <= (others => '0');
                cell_words_r <= (others => '0');
                chip_index_r <= 0;
                slot_count_r <= 0;
                slots_remaining_r <= 0;
                raw_line_words_r <= (others => '0');
                pending_profile_r <= C_GPX_VDMA_LANE_PROFILE_IDLE;
                active_profile_r <= C_GPX_VDMA_LANE_PROFILE_IDLE;
            elsif i_abort = '1' then
                state_r <= ST_IDLE;
                pending_profile_r <= C_GPX_VDMA_LANE_PROFILE_IDLE;
                chip_index_r <= 0;
                slot_count_r <= 0;
                slots_remaining_r <= 0;
                raw_line_words_r <= (others => '0');
            else
                case state_r is
                    when ST_IDLE =>
                        if i_request_valid = '1' then
                            if (i_lane_chip_mask and not C_ALLOWED_MASK) /=
                                   chip_mask_t'(others => '0') or
                               to_integer(i_visible_returns) < 1 or
                               to_integer(i_visible_returns) >
                                   G_BUILD_CONFIG.max_returns_per_stop or
                               i_planned_shots = 0 then
                                request_rejected_r <= '1';
                            else
                                request_mask_r <= i_lane_chip_mask;
                                request_returns_r <= i_visible_returns;
                                request_shots_r <= i_planned_shots;
                                cell_words_r <= fn_cell_words(
                                    i_visible_returns);
                                chip_index_r <= 0;
                                slot_count_r <= 0;
                                state_r <= ST_COUNT_CHIPS;
                            end if;
                        end if;

                    when ST_COUNT_CHIPS =>
                        next_slot_count_v := slot_count_r;
                        if request_mask_r(chip_index_r) = '1' then
                            next_slot_count_v := slot_count_r +
                                G_BUILD_CONFIG.stops_per_chip;
                        end if;
                        slot_count_r <= next_slot_count_v;

                        if chip_index_r = C_MAX_CHIPS - 1 then
                            if next_slot_count_v = 0 then
                                profile_v := C_GPX_VDMA_LANE_PROFILE_IDLE;
                                profile_v.valid := '1';
                                profile_v.enabled := '0';
                                profile_v.visible_returns :=
                                    request_returns_r;
                                profile_v.planned_shots := request_shots_r;
                                profile_v.stride_bytes := to_unsigned(
                                    C_STRIDE_BYTES,
                                    profile_v.stride_bytes'length);
                                pending_profile_r <= profile_v;
                                state_r <= ST_WAIT_ACTIVATE;
                            else
                                slots_remaining_r <= next_slot_count_v;
                                raw_line_words_r <= to_unsigned(
                                    C_GPX_VDMA_SHOT_META_WORDS,
                                    raw_line_words_r'length);
                                state_r <= ST_ACCUMULATE_CELLS;
                            end if;
                        else
                            chip_index_r <= chip_index_r + 1;
                        end if;

                    when ST_ACCUMULATE_CELLS =>
                        if slots_remaining_r = 0 then
                            state_r <= ST_FINALIZE;
                        else
                            raw_line_words_r <= raw_line_words_r +
                                resize(cell_words_r,
                                    raw_line_words_r'length);
                            slots_remaining_r <= slots_remaining_r - 1;
                        end if;

                    when ST_FINALIZE =>
                        aligned_words_v := fn_align_words(raw_line_words_r);
                        if aligned_words_v >= C_GPX_VDMA_FOOTER_WORDS then
                            footer_lines_v := to_unsigned(1, 2);
                        else
                            footer_lines_v := to_unsigned(2, 2);
                        end if;
                        vsize_ext_v := resize(request_shots_r, 17) +
                            resize(footer_lines_v, 17);

                        if vsize_ext_v(16) = '1' or
                           slot_count_r > C_MAX_SLOT_COUNT then
                            request_rejected_r <= '1';
                            pending_profile_r <=
                                C_GPX_VDMA_LANE_PROFILE_IDLE;
                            state_r <= ST_IDLE;
                        else
                            profile_v := C_GPX_VDMA_LANE_PROFILE_IDLE;
                            profile_v.valid := '1';
                            profile_v.enabled := '1';
                            profile_v.slot_count := to_unsigned(
                                slot_count_r, profile_v.slot_count'length);
                            profile_v.visible_returns := request_returns_r;
                            profile_v.cell_word_count := cell_words_r;
                            profile_v.planned_shots := request_shots_r;
                            profile_v.raw_line_words := raw_line_words_r;
                            profile_v.hsize_words := aligned_words_v;
                            profile_v.hsize_bytes := shift_left(
                                resize(aligned_words_v,
                                    profile_v.hsize_bytes'length), 2);
                            profile_v.footer_lines := footer_lines_v;
                            profile_v.vsize_lines := vsize_ext_v(15 downto 0);
                            profile_v.stride_bytes := to_unsigned(
                                C_STRIDE_BYTES,
                                profile_v.stride_bytes'length);
                            pending_profile_r <= profile_v;
                            state_r <= ST_WAIT_ACTIVATE;
                        end if;

                    when ST_WAIT_ACTIVATE =>
                        if i_activate_valid = '1' and
                           o_activate_ready = '1' then
                            state_r <= ST_PROGRAM;
                        end if;

                    when ST_PROGRAM =>
                        if i_vdma_cfg_ready = '1' then
                            active_profile_r <= pending_profile_r;
                            pending_profile_r <=
                                C_GPX_VDMA_LANE_PROFILE_IDLE;
                            profile_activated_r <= '1';
                            state_r <= ST_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_profile;

    -- synthesis translate_off
    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) and i_rst_n = '1' and i_abort = '0' then
            if state_r = ST_PROGRAM and i_vdma_cfg_ready = '0' then
                assert pending_profile_r.valid = '1'
                    report "V2-B9-J8-003 VDMA request lost pending profile"
                    severity failure;
            end if;
            if active_profile_r.valid = '1' and
               active_profile_r.enabled = '1' then
                assert active_profile_r.hsize_bytes <=
                           active_profile_r.stride_bytes and
                       active_profile_r.hsize_words /= 0 and
                       active_profile_r.footer_lines >= 1
                    report "V2-B9-J8-004 invalid Active profile"
                    severity failure;
            end if;
        end if;
    end process p_contract;
    -- synthesis translate_on

end architecture rtl;
