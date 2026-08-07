library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- B1 Processing boundary. Geometry is direction-independent and inclusive.
-- Stage 1 registers membership/priority results; Stage 2 registers traversal
-- events, keeping the control path free of AXIS backpressure.
entity face_tracker is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                       : in  std_logic;
        i_rst_n                     : in  std_logic;
        i_enable                    : in  std_logic;
        i_active_valid              : in  std_logic;
        i_active_config             : in  lidar_active_config_t;
        i_position_event            : in  position_event_t;
        i_clear_diagnostics         : in  std_logic;
        o_face_event                : out face_event_t;
        o_overlap_sticky            : out std_logic;
        o_overlap_count             : out u32_t
    );
end entity face_tracker;

architecture rtl of face_tracker is

    function fn_is_inside(
        position_value : position_t;
        lower_value    : position_t;
        upper_value    : position_t
    ) return std_logic is
    begin
        if lower_value <= upper_value then
            if position_value >= lower_value and
               position_value <= upper_value then
                return '1';
            end if;
        elsif position_value >= lower_value or
              position_value <= upper_value then
            return '1';
        end if;
        return '0';
    end function fn_is_inside;

    constant C_BUILD_FACE_MASK : face_mask_t :=
        fn_active_face_mask(G_BUILD_CONFIG.num_faces);

    signal face_lower_r : face_position_array_t :=
        (others => (others => '0'));
    signal face_upper_r : face_position_array_t :=
        (others => (others => '0'));
    signal face_mask_r : face_mask_t := (others => '0');
    signal active_version_r : u16_t := (others => '0');

    signal membership_valid_r    : std_logic := '0';
    signal membership_inside_r   : std_logic := '0';
    signal membership_overlap_r  : std_logic := '0';
    signal membership_face_r     : face_index_t := (others => '0');
    signal membership_context_r  : position_event_t :=
        C_POSITION_EVENT_IDLE;

    signal previous_valid_r     : std_logic := '0';
    signal previous_inside_r    : std_logic := '0';
    signal previous_face_r      : face_index_t := (others => '0');
    signal previous_direction_r : direction_t := DIRECTION_CW;

    signal face_event_r      : face_event_t := C_FACE_EVENT_IDLE;
    signal overlap_sticky_r  : std_logic := '0';
    signal overlap_count_r   : u32_t := (others => '0');

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-FACE-001 invalid build configuration"
        severity failure;

    o_face_event     <= face_event_r;
    o_overlap_sticky <= overlap_sticky_r;
    o_overlap_count  <= overlap_count_r;

    -- Active geometry is copied only while the Processing pipeline is
    -- quiescent. A build mask is applied locally so inactive entity capacity
    -- cannot be enabled by malformed runtime state.
    p_config : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                face_lower_r     <= (others => (others => '0'));
                face_upper_r     <= (others => (others => '0'));
                face_mask_r      <= (others => '0');
                active_version_r <= (others => '0');
            elsif i_enable = '0' and i_active_valid = '1' then
                face_lower_r <= i_active_config.derived.face_lower;
                face_upper_r <= i_active_config.derived.face_upper;
                face_mask_r <= i_active_config.source.laser.face_enable_mask
                    and C_BUILD_FACE_MASK;
                active_version_r <= i_active_config.version;
            end if;
        end if;
    end process p_config;

    -- Stage 1: five bounded comparisons plus a lowest-index priority select.
    p_membership : process (i_clk)
        variable match_v          : face_mask_t;
        variable match_count_v    : natural range 0 to C_MAX_FACES;
        variable selected_v       : face_index_t;
        variable selected_found_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_enable = '0' or i_active_valid = '0' then
                membership_valid_r   <= '0';
                membership_inside_r  <= '0';
                membership_overlap_r <= '0';
                membership_face_r    <= (others => '0');
                membership_context_r <= C_POSITION_EVENT_IDLE;
            else
                membership_valid_r <= i_position_event.valid;
                if i_position_event.valid = '1' then
                    match_v          := (others => '0');
                    match_count_v    := 0;
                    selected_v       := (others => '0');
                    selected_found_v := false;

                    for face_index in 0 to G_BUILD_CONFIG.num_faces - 1 loop
                        if face_mask_r(face_index) = '1' then
                            match_v(face_index) := fn_is_inside(
                                i_position_event.position,
                                face_lower_r(face_index),
                                face_upper_r(face_index));
                            if match_v(face_index) = '1' then
                                match_count_v := match_count_v + 1;
                                if not selected_found_v then
                                    selected_v := to_unsigned(
                                        face_index, selected_v'length);
                                    selected_found_v := true;
                                end if;
                            end if;
                        end if;
                    end loop;

                    membership_context_r <= i_position_event;
                    membership_face_r    <= selected_v;
                    if match_count_v = 0 then
                        membership_inside_r <= '0';
                    else
                        membership_inside_r <= '1';
                    end if;
                    if match_count_v > 1 then
                        membership_overlap_r <= '1';
                    else
                        membership_overlap_r <= '0';
                    end if;
                else
                    membership_inside_r  <= '0';
                    membership_overlap_r <= '0';
                    membership_face_r    <= (others => '0');
                    membership_context_r <= C_POSITION_EVENT_IDLE;
                end if;
            end if;
        end if;
    end process p_membership;

    -- Stage 2: traversal state and registered B1 event. A direct Face change
    -- and an in-Face direction reversal both close the old traversal and open
    -- the new traversal in the same event.
    p_traversal : process (i_clk)
        variable event_v             : face_event_t;
        variable direction_changed_v : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_enable = '0' or i_active_valid = '0' then
                previous_valid_r     <= '0';
                previous_inside_r    <= '0';
                previous_face_r      <= (others => '0');
                previous_direction_r <= DIRECTION_CW;
                face_event_r         <= C_FACE_EVENT_IDLE;
            else
                event_v := C_FACE_EVENT_IDLE;
                if membership_valid_r = '1' then
                    event_v.valid                := '1';
                    event_v.inside               := membership_inside_r;
                    event_v.overlap              := membership_overlap_r;
                    event_v.position             :=
                        membership_context_r.position;
                    event_v.direction            :=
                        membership_context_r.direction;
                    event_v.source_sim           :=
                        membership_context_r.source_sim;
                    event_v.source_latency_clks  :=
                        membership_context_r.source_latency_clks;
                    event_v.source_latency_valid :=
                        membership_context_r.source_latency_valid;
                    event_v.z_event              :=
                        membership_context_r.z_event;
                    event_v.active_version       :=
                        membership_context_r.active_version;

                    direction_changed_v := previous_valid_r = '1' and
                        membership_context_r.direction /=
                            previous_direction_r;

                    if direction_changed_v then
                        event_v.exit_event  := previous_inside_r;
                        event_v.enter_event := membership_inside_r;
                    else
                        if membership_inside_r = '1' and
                           (previous_valid_r = '0' or
                            previous_inside_r = '0' or
                            membership_face_r /= previous_face_r) then
                            event_v.enter_event := '1';
                        end if;
                        if previous_valid_r = '1' and
                           previous_inside_r = '1' and
                           (membership_inside_r = '0' or
                            membership_face_r /= previous_face_r) then
                            event_v.exit_event := '1';
                        end if;
                    end if;

                    if membership_inside_r = '1' then
                        event_v.face_index := membership_face_r;
                    elsif event_v.exit_event = '1' then
                        event_v.face_index := previous_face_r;
                    end if;

                    previous_valid_r     <= '1';
                    previous_inside_r    <= membership_inside_r;
                    previous_direction_r <=
                        membership_context_r.direction;
                    if membership_inside_r = '1' then
                        previous_face_r <= membership_face_r;
                    end if;
                end if;
                face_event_r <= event_v;
            end if;
        end if;
    end process p_traversal;

    p_diagnostics : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                overlap_sticky_r <= '0';
                overlap_count_r  <= (others => '0');
            else
                if i_clear_diagnostics = '1' then
                    overlap_sticky_r <= '0';
                    overlap_count_r  <= (others => '0');
                end if;
                if i_enable = '1' and i_active_valid = '1' and
                   membership_valid_r = '1' and
                   membership_overlap_r = '1' then
                    overlap_sticky_r <= '1';
                    if i_clear_diagnostics = '1' then
                        overlap_count_r <= to_unsigned(
                            1, overlap_count_r'length);
                    else
                        overlap_count_r <= overlap_count_r + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_diagnostics;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_enable = '1' then
                assert i_active_valid = '1'
                    report "V2-FACE-004 enable without active configuration"
                    severity failure;
                assert i_active_config.version = active_version_r
                    report "V2-FACE-005 active version changed while enabled"
                    severity failure;
                if i_position_event.valid = '1' then
                    assert i_position_event.active_version =
                           active_version_r
                        report "V2-FACE-006 position/config version mismatch"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
