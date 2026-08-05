library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- B7 width-independent Hit-to-Cell collector.
--
-- A Cell is one Shot x Chip x STOP x slope. Hit payload remains 17 bit here;
-- AXIS width, byte packing and VDMA geometry belong to later boundaries.
entity lidar_gpx_cell_collector is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk          : in  std_logic;
        i_rst_n        : in  std_logic;
        i_abort        : in  std_logic;
        i_clear_sticky : in  std_logic;

        i_active_version     : in unsigned(15 downto 0);
        i_max_hits_per_stop  : in unsigned(2 downto 0);

        i_hit_event : in  gpx_hit_event_t;
        o_hit_ready : out std_logic;

        o_cell_event : out gpx_cell_event_t;
        i_cell_ready : in  std_logic;

        o_fault_pulse  : out gpx_cell_collector_faults_t;
        o_fault_sticky : out gpx_cell_collector_faults_t
    );
end entity lidar_gpx_cell_collector;

architecture rtl of lidar_gpx_cell_collector is

    constant C_CELL_ADDRESS_COUNT : positive :=
        C_MAX_CHIPS * 2 * C_MAX_STOPS_PER_CHIP;

    type collector_state_t is (
        S_COLLECT,
        S_HIT_READ,
        S_HIT_APPLY,
        S_CELL_READ,
        S_CELL_LOAD,
        S_CELL_WAIT,
        S_CONTROL_LOAD,
        S_CONTROL_WAIT
    );

    type hit_bank_t is array (0 to C_CELL_ADDRESS_COUNT - 1) of
        gpx_hit_value_t;
    type count_memory_t is array (0 to C_CELL_ADDRESS_COUNT - 1) of
        unsigned(2 downto 0);
    type context_by_chip_t is array (0 to C_MAX_CHIPS - 1) of
        shot_start_event_t;
    type sequence_by_chip_t is array (0 to C_MAX_CHIPS - 1) of
        unsigned(15 downto 0);
    type max_hits_by_chip_t is array (0 to C_MAX_CHIPS - 1) of
        unsigned(2 downto 0);

    signal state_r : collector_state_t := S_COLLECT;

    -- Payload RAM is intentionally not reset. Per-cell visible count masks
    -- every stale location after reset, abort and terminal cleanup.
    signal hit_bank_0_r : hit_bank_t;
    signal hit_bank_1_r : hit_bank_t;
    signal hit_bank_2_r : hit_bank_t;
    signal hit_bank_3_r : hit_bank_t;
    signal hit_bank_4_r : hit_bank_t;
    signal hit_bank_5_r : hit_bank_t;
    signal hit_bank_6_r : hit_bank_t;
    attribute ram_style : string;
    attribute ram_style of hit_bank_0_r : signal is "distributed";
    attribute ram_style of hit_bank_1_r : signal is "distributed";
    attribute ram_style of hit_bank_2_r : signal is "distributed";
    attribute ram_style of hit_bank_3_r : signal is "distributed";
    attribute ram_style of hit_bank_4_r : signal is "distributed";
    attribute ram_style of hit_bank_5_r : signal is "distributed";
    attribute ram_style of hit_bank_6_r : signal is "distributed";

    signal seen_count_r    : count_memory_t := (others => (others => '0'));
    signal visible_count_r : count_memory_t := (others => (others => '0'));
    signal hit_dropped_r   : std_logic_vector(
        C_CELL_ADDRESS_COUNT - 1 downto 0) := (others => '0');
    signal cell_error_r    : std_logic_vector(
        C_CELL_ADDRESS_COUNT - 1 downto 0) := (others => '0');

    signal shot_active_r  : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal shot_context_r : context_by_chip_t :=
        (others => C_SHOT_START_EVENT_IDLE);
    signal chip_sequence_r : sequence_by_chip_t :=
        (others => (others => '0'));
    signal max_hits_r : max_hits_by_chip_t :=
        (others => (others => '0'));
    signal shot_fault_r : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal lower_emitted_r : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');

    signal emit_chip_r          : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal emit_stop_r          : natural range 0 to C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal emit_stop_first_r    : natural range 0 to C_MAX_STOPS_PER_CHIP := 0;
    signal emit_stop_last_r     : natural range 0 to C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal emit_slope_r         : gpx_slope_t := GPX_SLOPE_RISE;
    signal emit_control_kind_r  : gpx_cell_event_kind_t := GPX_CELL_DRAIN_DONE;
    signal emit_ififo_r         : std_logic := '0';
    signal emit_error_fill_r    : std_logic := '0';
    signal emit_timeout_cause_r : std_logic_vector(2 downto 0) :=
        (others => '0');

    signal read_hits_r      : gpx_hit_value_array_t :=
        (others => (others => '0'));
    signal read_hit_count_r : unsigned(2 downto 0) := (others => '0');
    signal read_dropped_r   : std_logic := '0';
    signal read_error_r     : std_logic := '0';

    -- Hit metadata uses an explicit read/apply pipeline. This prevents the
    -- 64-address count arrays from becoming a cross-address read/modify/write
    -- mux in one Processing-clock cycle.
    signal pending_address_r : natural range 0 to
        C_CELL_ADDRESS_COUNT - 1 := 0;
    attribute max_fanout : integer;
    attribute max_fanout of pending_address_r : signal is 8;
    signal pending_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal pending_return_r : unsigned(2 downto 0) := (others => '0');
    signal pending_hit_r : gpx_hit_value_t := (others => '0');
    signal pending_start_nonzero_r : std_logic := '0';
    signal pending_store_limit_r : unsigned(2 downto 0) :=
        to_unsigned(1, 3);
    signal selected_seen_r    : unsigned(2 downto 0) := (others => '0');
    signal selected_visible_r : unsigned(2 downto 0) := (others => '0');

    signal cell_event_r   : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    signal fault_pulse_r  : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
    signal fault_sticky_r : gpx_cell_collector_faults_t :=
        C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;

    function fn_cell_address(
        chip_index : natural;
        slope      : gpx_slope_t;
        stop_index : natural
    ) return natural is
        variable slope_offset : natural := 0;
    begin
        if slope = GPX_SLOPE_FALL then
            slope_offset := C_MAX_STOPS_PER_CHIP;
        end if;
        return chip_index * (2 * C_MAX_STOPS_PER_CHIP) +
               slope_offset + stop_index;
    end function fn_cell_address;

    function fn_slope_supported(
        chip_index : natural;
        slope      : gpx_slope_t
    ) return boolean is
    begin
        if slope = GPX_SLOPE_RISE then
            return G_BUILD_CONFIG.rise_capability_mask(chip_index) = '1';
        end if;
        return G_BUILD_CONFIG.fall_capability_mask(chip_index) = '1';
    end function fn_slope_supported;

    function fn_first_slope(
        chip_index : natural
    ) return gpx_slope_t is
    begin
        if fn_slope_supported(chip_index, GPX_SLOPE_RISE) then
            return GPX_SLOPE_RISE;
        end if;
        return GPX_SLOPE_FALL;
    end function fn_first_slope;

    function fn_has_emit_cells(
        chip_index : natural;
        first_stop : natural
    ) return boolean is
    begin
        return chip_index < G_BUILD_CONFIG.num_chips and
               first_stop < G_BUILD_CONFIG.stops_per_chip and
               (fn_slope_supported(chip_index, GPX_SLOPE_RISE) or
                fn_slope_supported(chip_index, GPX_SLOPE_FALL));
    end function fn_has_emit_cells;

    function fn_effective_max_hits(
        value : unsigned(2 downto 0)
    ) return unsigned is
    begin
        if value = 0 then
            return to_unsigned(1, value'length);
        elsif to_integer(value) > G_BUILD_CONFIG.max_returns_per_stop then
            return to_unsigned(
                G_BUILD_CONFIG.max_returns_per_stop, value'length);
        end if;
        return value;
    end function fn_effective_max_hits;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-B7-001 illegal build configuration"
        severity failure;

    assert G_BUILD_CONFIG.max_returns_per_stop <=
           2 ** i_max_hits_per_stop'length - 1
        report "V2-B7-002 max Return capacity exceeds Cell counter width"
        severity failure;

    o_hit_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
                            state_r = S_COLLECT and
                            cell_event_r.valid = '0' else
                   '0';

    o_cell_event   <= cell_event_r;
    o_fault_pulse  <= fault_pulse_r;
    o_fault_sticky <= fault_sticky_r;

    p_collect : process (i_clk)
        variable chip_index     : natural range 0 to C_MAX_CHIPS - 1;
        variable stop_index     : natural range 0 to C_MAX_STOPS_PER_CHIP - 1;
        variable address_value  : natural range 0 to C_CELL_ADDRESS_COUNT - 1;
        variable return_index   : natural range 0 to C_MAX_RETURNS_PER_STOP;
        variable visible_count  : natural range 0 to C_MAX_RETURNS_PER_STOP;
        variable store_limit    : natural range 1 to C_MAX_RETURNS_PER_STOP;
        variable effective_max  : unsigned(2 downto 0);
        variable first_stop     : natural range 0 to C_MAX_STOPS_PER_CHIP;
        variable last_stop      : natural range 0 to C_MAX_STOPS_PER_CHIP - 1;
        variable next_exists    : boolean;
        variable result         : gpx_cell_event_t;
        variable context_fault  : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r            <= S_COLLECT;
                seen_count_r       <= (others => (others => '0'));
                visible_count_r    <= (others => (others => '0'));
                hit_dropped_r      <= (others => '0');
                cell_error_r       <= (others => '0');
                shot_active_r      <= (others => '0');
                shot_context_r     <= (others => C_SHOT_START_EVENT_IDLE);
                chip_sequence_r    <= (others => (others => '0'));
                max_hits_r         <= (others => (others => '0'));
                shot_fault_r       <= (others => '0');
                lower_emitted_r    <= (others => '0');
                emit_chip_r        <= 0;
                emit_stop_r        <= 0;
                emit_stop_first_r  <= 0;
                emit_stop_last_r   <= 0;
                emit_slope_r       <= GPX_SLOPE_RISE;
                emit_control_kind_r <= GPX_CELL_DRAIN_DONE;
                emit_ififo_r       <= '0';
                emit_error_fill_r  <= '0';
                emit_timeout_cause_r <= (others => '0');
                read_hits_r        <= (others => (others => '0'));
                read_hit_count_r   <= (others => '0');
                read_dropped_r     <= '0';
                read_error_r       <= '0';
                pending_address_r  <= 0;
                pending_chip_r     <= 0;
                pending_return_r   <= (others => '0');
                pending_hit_r      <= (others => '0');
                pending_start_nonzero_r <= '0';
                pending_store_limit_r <= to_unsigned(1, 3);
                selected_seen_r    <= (others => '0');
                selected_visible_r <= (others => '0');
                cell_event_r       <= C_GPX_CELL_EVENT_IDLE;
                fault_pulse_r      <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                fault_sticky_r     <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
            else
                fault_pulse_r <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;

                if i_clear_sticky = '1' then
                    fault_sticky_r <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                end if;

                if i_abort = '1' then
                    state_r         <= S_COLLECT;
                    seen_count_r    <= (others => (others => '0'));
                    visible_count_r <= (others => (others => '0'));
                    hit_dropped_r   <= (others => '0');
                    cell_error_r    <= (others => '0');
                    shot_active_r   <= (others => '0');
                    shot_fault_r    <= (others => '0');
                    lower_emitted_r <= (others => '0');
                    cell_event_r    <= C_GPX_CELL_EVENT_IDLE;
                else
                    case state_r is
                        when S_COLLECT =>
                            if i_hit_event.valid = '1' then
                                chip_index := to_integer(i_hit_event.chip_index);

                                if chip_index >= G_BUILD_CONFIG.num_chips then
                                    fault_pulse_r.context_mismatch  <= '1';
                                    fault_sticky_r.context_mismatch <= '1';
                                else
                                    effective_max := fn_effective_max_hits(
                                        i_max_hits_per_stop);
                                    context_fault := false;

                                    if shot_active_r(chip_index) = '0' then
                                        shot_active_r(chip_index) <= '1';
                                        shot_context_r(chip_index) <=
                                            i_hit_event.shot_context;
                                        chip_sequence_r(chip_index) <=
                                            i_hit_event.chip_shot_seq;
                                        max_hits_r(chip_index) <= effective_max;
                                        shot_fault_r(chip_index) <=
                                            i_hit_event.faulted;
                                        lower_emitted_r(chip_index) <= '0';

                                        if i_hit_event.shot_context.request.
                                                active_version /=
                                           i_active_version or
                                           i_max_hits_per_stop = 0 or
                                           to_integer(i_max_hits_per_stop) >
                                               G_BUILD_CONFIG.
                                                   max_returns_per_stop then
                                            context_fault := true;
                                            shot_fault_r(chip_index) <= '1';
                                        end if;
                                    else
                                        if chip_sequence_r(chip_index) /=
                                               i_hit_event.chip_shot_seq or
                                           shot_context_r(chip_index).request.
                                               active_version /=
                                               i_hit_event.shot_context.request.
                                                   active_version or
                                           shot_context_r(chip_index).request.
                                               shot_index /=
                                               i_hit_event.shot_context.request.
                                                   shot_index or
                                           shot_context_r(chip_index).request.
                                               face_index /=
                                               i_hit_event.shot_context.request.
                                                   face_index or
                                           shot_context_r(chip_index).request.
                                               source_sim /=
                                               i_hit_event.shot_context.request.
                                                   source_sim or
                                           max_hits_r(chip_index) /=
                                               effective_max or
                                           i_hit_event.shot_context.request.
                                               active_version /=
                                               i_active_version then
                                            context_fault := true;
                                            shot_fault_r(chip_index) <= '1';
                                        end if;

                                        if i_hit_event.faulted = '1' then
                                            shot_fault_r(chip_index) <= '1';
                                        end if;
                                    end if;

                                    if context_fault then
                                        fault_pulse_r.context_mismatch  <= '1';
                                        fault_sticky_r.context_mismatch <= '1';
                                    end if;

                                    if i_hit_event.kind = GPX_HIT_DATA then
                                        stop_index := to_integer(
                                            i_hit_event.stop_index);
                                        pending_address_r <= fn_cell_address(
                                            chip_index,
                                            i_hit_event.slope,
                                            stop_index);
                                        pending_chip_r   <= chip_index;
                                        pending_return_r <=
                                            i_hit_event.return_index;
                                        pending_hit_r <= i_hit_event.hit;
                                        if i_hit_event.start_number /= 0 then
                                            pending_start_nonzero_r <= '1';
                                        else
                                            pending_start_nonzero_r <= '0';
                                        end if;

                                        if shot_active_r(chip_index) = '0' then
                                            pending_store_limit_r <= effective_max;
                                        else
                                            pending_store_limit_r <=
                                                max_hits_r(chip_index);
                                        end if;
                                        state_r <= S_HIT_READ;
                                    else
                                        emit_chip_r <= chip_index;
                                        emit_ififo_r <= i_hit_event.ififo_id;
                                        emit_timeout_cause_r <=
                                            i_hit_event.timeout_cause;

                                        case i_hit_event.kind is
                                            when GPX_HIT_IFIFO1_DONE =>
                                                emit_control_kind_r <=
                                                    GPX_CELL_IFIFO1_DONE;
                                                first_stop := 0;
                                                if G_BUILD_CONFIG.stops_per_chip >
                                                   4 then
                                                    last_stop := 3;
                                                else
                                                    last_stop :=
                                                        G_BUILD_CONFIG.
                                                            stops_per_chip - 1;
                                                end if;
                                                emit_error_fill_r <=
                                                    i_hit_event.faulted;

                                            when GPX_HIT_DRAIN_DONE =>
                                                emit_control_kind_r <=
                                                    GPX_CELL_DRAIN_DONE;
                                                if lower_emitted_r(
                                                        chip_index) = '1' then
                                                    first_stop := 4;
                                                else
                                                    first_stop := 0;
                                                end if;
                                                last_stop :=
                                                    G_BUILD_CONFIG.
                                                        stops_per_chip - 1;
                                                emit_error_fill_r <=
                                                    i_hit_event.faulted;

                                            when GPX_HIT_TIMEOUT =>
                                                emit_control_kind_r <=
                                                    GPX_CELL_TIMEOUT;
                                                if lower_emitted_r(
                                                        chip_index) = '1' then
                                                    first_stop := 4;
                                                else
                                                    first_stop := 0;
                                                end if;
                                                last_stop :=
                                                    G_BUILD_CONFIG.
                                                        stops_per_chip - 1;
                                                emit_error_fill_r <= '1';
                                                shot_fault_r(chip_index) <= '1';

                                            when GPX_HIT_DATA =>
                                                first_stop := 0;
                                                last_stop := 0;
                                        end case;

                                        emit_stop_first_r <= first_stop;
                                        emit_stop_last_r  <= last_stop;
                                        emit_stop_r       <= first_stop;
                                        emit_slope_r      <= fn_first_slope(
                                            chip_index);

                                        if fn_has_emit_cells(
                                                chip_index, first_stop) then
                                            state_r <= S_CELL_READ;
                                        else
                                            state_r <= S_CONTROL_LOAD;
                                        end if;
                                    end if;
                                end if;
                            end if;

                        when S_HIT_READ =>
                            selected_seen_r <=
                                seen_count_r(pending_address_r);
                            selected_visible_r <=
                                visible_count_r(pending_address_r);
                            state_r <= S_HIT_APPLY;

                        when S_HIT_APPLY =>
                            address_value := pending_address_r;
                            return_index := to_integer(pending_return_r);

                            if pending_start_nonzero_r = '1' then
                                cell_error_r(address_value) <= '1';
                                shot_fault_r(pending_chip_r) <= '1';
                                fault_pulse_r.start_number_nonzero <= '1';
                                fault_sticky_r.start_number_nonzero <= '1';
                            end if;

                            if return_index /= to_integer(selected_seen_r) then
                                cell_error_r(address_value) <= '1';
                                shot_fault_r(pending_chip_r) <= '1';
                                fault_pulse_r.return_sequence_error <= '1';
                                fault_sticky_r.return_sequence_error <= '1';
                            else
                                if selected_seen_r < to_unsigned(
                                        C_MAX_RETURNS_PER_STOP,
                                        selected_seen_r'length) then
                                    seen_count_r(address_value) <=
                                        selected_seen_r + 1;
                                end if;

                                visible_count := to_integer(
                                    selected_visible_r);
                                store_limit := to_integer(
                                    pending_store_limit_r);
                                if visible_count < store_limit then
                                    case visible_count is
                                        when 0 =>
                                            hit_bank_0_r(address_value) <=
                                                pending_hit_r;
                                        when 1 =>
                                            hit_bank_1_r(address_value) <=
                                                pending_hit_r;
                                        when 2 =>
                                            hit_bank_2_r(address_value) <=
                                                pending_hit_r;
                                        when 3 =>
                                            hit_bank_3_r(address_value) <=
                                                pending_hit_r;
                                        when 4 =>
                                            hit_bank_4_r(address_value) <=
                                                pending_hit_r;
                                        when 5 =>
                                            hit_bank_5_r(address_value) <=
                                                pending_hit_r;
                                        when 6 =>
                                            hit_bank_6_r(address_value) <=
                                                pending_hit_r;
                                        when others =>
                                            null;
                                    end case;
                                    visible_count_r(address_value) <=
                                        selected_visible_r + 1;
                                else
                                    hit_dropped_r(address_value) <= '1';
                                    fault_pulse_r.hit_capacity_drop <= '1';
                                    fault_sticky_r.hit_capacity_drop <= '1';
                                end if;
                            end if;
                            state_r <= S_COLLECT;

                        when S_CELL_READ =>
                            address_value := fn_cell_address(
                                emit_chip_r, emit_slope_r, emit_stop_r);
                            if visible_count_r(address_value) > 0 then
                                read_hits_r(0) <= hit_bank_0_r(address_value);
                            else
                                read_hits_r(0) <= (others => '0');
                            end if;
                            if visible_count_r(address_value) > 1 then
                                read_hits_r(1) <= hit_bank_1_r(address_value);
                            else
                                read_hits_r(1) <= (others => '0');
                            end if;
                            if visible_count_r(address_value) > 2 then
                                read_hits_r(2) <= hit_bank_2_r(address_value);
                            else
                                read_hits_r(2) <= (others => '0');
                            end if;
                            if visible_count_r(address_value) > 3 then
                                read_hits_r(3) <= hit_bank_3_r(address_value);
                            else
                                read_hits_r(3) <= (others => '0');
                            end if;
                            if visible_count_r(address_value) > 4 then
                                read_hits_r(4) <= hit_bank_4_r(address_value);
                            else
                                read_hits_r(4) <= (others => '0');
                            end if;
                            if visible_count_r(address_value) > 5 then
                                read_hits_r(5) <= hit_bank_5_r(address_value);
                            else
                                read_hits_r(5) <= (others => '0');
                            end if;
                            if visible_count_r(address_value) > 6 then
                                read_hits_r(6) <= hit_bank_6_r(address_value);
                            else
                                read_hits_r(6) <= (others => '0');
                            end if;
                            read_hit_count_r <= visible_count_r(address_value);
                            read_dropped_r   <= hit_dropped_r(address_value);
                            read_error_r     <= cell_error_r(address_value);
                            state_r          <= S_CELL_LOAD;

                        when S_CELL_LOAD =>
                            result := C_GPX_CELL_EVENT_IDLE;
                            result.valid      := '1';
                            result.kind       := GPX_CELL_DATA;
                            result.chip_index := to_unsigned(
                                emit_chip_r, result.chip_index'length);
                            if emit_stop_r >= 4 then
                                result.ififo_id := '1';
                            end if;
                            result.stop_index := to_unsigned(
                                emit_stop_r, result.stop_index'length);
                            result.slope     := emit_slope_r;
                            result.hit_count := read_hit_count_r;
                            result.max_hits  := max_hits_r(emit_chip_r);
                            result.hits      := read_hits_r;
                            result.hit_dropped := read_dropped_r;
                            result.error_fill  := emit_error_fill_r;
                            result.faulted := read_error_r or
                                shot_fault_r(emit_chip_r) or
                                emit_error_fill_r;
                            result.timeout_cause := emit_timeout_cause_r;
                            result.shot_context  :=
                                shot_context_r(emit_chip_r);
                            result.chip_shot_seq :=
                                chip_sequence_r(emit_chip_r);
                            cell_event_r <= result;
                            state_r      <= S_CELL_WAIT;

                        when S_CELL_WAIT =>
                            if cell_event_r.valid = '1' and
                               i_cell_ready = '1' then
                                address_value := fn_cell_address(
                                    emit_chip_r, emit_slope_r, emit_stop_r);
                                seen_count_r(address_value)    <= (others => '0');
                                visible_count_r(address_value) <= (others => '0');
                                hit_dropped_r(address_value)   <= '0';
                                cell_error_r(address_value)    <= '0';
                                cell_event_r.valid <= '0';

                                next_exists := false;
                                if emit_stop_r < emit_stop_last_r then
                                    emit_stop_r <= emit_stop_r + 1;
                                    next_exists := true;
                                elsif emit_slope_r = GPX_SLOPE_RISE and
                                      fn_slope_supported(
                                          emit_chip_r, GPX_SLOPE_FALL) then
                                    emit_slope_r <= GPX_SLOPE_FALL;
                                    emit_stop_r  <= emit_stop_first_r;
                                    next_exists := true;
                                end if;

                                if next_exists then
                                    state_r <= S_CELL_READ;
                                else
                                    state_r <= S_CONTROL_LOAD;
                                end if;
                            end if;

                        when S_CONTROL_LOAD =>
                            result := C_GPX_CELL_EVENT_IDLE;
                            result.valid         := '1';
                            result.kind          := emit_control_kind_r;
                            result.chip_index    := to_unsigned(
                                emit_chip_r, result.chip_index'length);
                            result.ififo_id      := emit_ififo_r;
                            result.max_hits      := max_hits_r(emit_chip_r);
                            result.error_fill    := emit_error_fill_r;
                            result.faulted       :=
                                shot_fault_r(emit_chip_r) or
                                emit_error_fill_r;
                            result.timeout_cause := emit_timeout_cause_r;
                            result.shot_context  :=
                                shot_context_r(emit_chip_r);
                            result.chip_shot_seq :=
                                chip_sequence_r(emit_chip_r);
                            cell_event_r <= result;

                            if emit_control_kind_r =
                               GPX_CELL_IFIFO1_DONE then
                                lower_emitted_r(emit_chip_r) <= '1';
                            end if;
                            state_r <= S_CONTROL_WAIT;

                        when S_CONTROL_WAIT =>
                            if cell_event_r.valid = '1' and
                               i_cell_ready = '1' then
                                cell_event_r.valid <= '0';

                                if emit_control_kind_r /=
                                   GPX_CELL_IFIFO1_DONE then
                                    for slope_value in gpx_slope_t loop
                                        for stop_value in 0 to
                                                C_MAX_STOPS_PER_CHIP - 1 loop
                                            address_value := fn_cell_address(
                                                emit_chip_r,
                                                slope_value,
                                                stop_value);
                                            seen_count_r(address_value) <=
                                                (others => '0');
                                            visible_count_r(address_value) <=
                                                (others => '0');
                                            hit_dropped_r(address_value) <= '0';
                                            cell_error_r(address_value) <= '0';
                                        end loop;
                                    end loop;
                                    shot_active_r(emit_chip_r)   <= '0';
                                    shot_fault_r(emit_chip_r)    <= '0';
                                    lower_emitted_r(emit_chip_r) <= '0';
                                end if;
                                state_r <= S_COLLECT;
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process p_collect;

end architecture rtl;
