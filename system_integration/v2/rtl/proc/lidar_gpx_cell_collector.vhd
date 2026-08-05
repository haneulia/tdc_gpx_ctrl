library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

-- B7 width-independent Hit-to-Cell collector.
--
-- A Cell is one Shot x Chip x STOP x slope. B7 is the single owner of the
-- physical Return order inside that Cell. Hit payload remains 17 bit here;
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

        i_active_version    : in unsigned(15 downto 0);
        i_max_hits_per_stop : in unsigned(2 downto 0);
        i_active_rise_mask  : in chip_mask_t :=
            G_BUILD_CONFIG.rise_capability_mask;
        i_active_fall_mask  : in chip_mask_t :=
            G_BUILD_CONFIG.fall_capability_mask;

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
    constant C_CELLS_PER_CHIP : positive := 2 * C_MAX_STOPS_PER_CHIP;

    constant C_META_COUNT_LO : natural := 0;
    constant C_META_COUNT_HI : natural := 2;
    constant C_META_DROPPED  : natural := 3;
    constant C_META_ERROR    : natural := 4;

    type collector_state_t is (
        S_COLLECT,
        S_EVENT_SELECT,
        S_SCRUB_META,
        S_EVENT_CHECK,
        S_EVENT_ROUTE,
        S_HIT_META_READ,
        S_HIT_APPLY,
        S_CELL_META_READ,
        S_CELL_HIT_READ,
        S_CELL_LOAD,
        S_CELL_WAIT,
        S_CONTROL_LOAD,
        S_CONTROL_WAIT
    );

    type hit_bank_t is array (0 to C_CELL_ADDRESS_COUNT - 1) of
        gpx_hit_value_t;
    subtype cell_meta_t is std_logic_vector(C_META_ERROR downto 0);
    type cell_meta_memory_t is array (0 to C_CELL_ADDRESS_COUNT - 1) of
        cell_meta_t;
    subtype shot_identity_t is std_logic_vector(51 downto 0);
    type context_by_chip_t is array (0 to C_MAX_CHIPS - 1) of
        shot_start_event_t;
    type identity_by_chip_t is array (0 to C_MAX_CHIPS - 1) of
        shot_identity_t;
    type sequence_by_chip_t is array (0 to C_MAX_CHIPS - 1) of
        unsigned(15 downto 0);
    type max_hits_by_chip_t is array (0 to C_MAX_CHIPS - 1) of
        unsigned(2 downto 0);

    signal state_r : collector_state_t := S_COLLECT;

    -- Payload and metadata RAMs are intentionally not reset. The first event
    -- of each Chip Shot is held while its 16 metadata addresses are scrubbed.
    -- A zero metadata count makes every stale Hit payload location invisible.
    signal hit_bank_0_r : hit_bank_t;
    signal hit_bank_1_r : hit_bank_t;
    signal hit_bank_2_r : hit_bank_t;
    signal hit_bank_3_r : hit_bank_t;
    signal hit_bank_4_r : hit_bank_t;
    signal hit_bank_5_r : hit_bank_t;
    signal hit_bank_6_r : hit_bank_t;
    signal cell_meta_r  : cell_meta_memory_t;

    attribute ram_style : string;
    attribute ram_style of hit_bank_0_r : signal is "distributed";
    attribute ram_style of hit_bank_1_r : signal is "distributed";
    attribute ram_style of hit_bank_2_r : signal is "distributed";
    attribute ram_style of hit_bank_3_r : signal is "distributed";
    attribute ram_style of hit_bank_4_r : signal is "distributed";
    attribute ram_style of hit_bank_5_r : signal is "distributed";
    attribute ram_style of hit_bank_6_r : signal is "distributed";
    attribute ram_style of cell_meta_r  : signal is "distributed";

    signal shot_active_r : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal shot_context_r : context_by_chip_t :=
        (others => C_SHOT_START_EVENT_IDLE);
    signal shot_identity_r : identity_by_chip_t := (others => (others => '0'));
    signal chip_sequence_r : sequence_by_chip_t :=
        (others => (others => '0'));
    signal max_hits_r : max_hits_by_chip_t :=
        (others => (others => '0'));
    signal shot_fault_r : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal lower_emitted_r : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal rise_enabled_r : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal fall_enabled_r : std_logic_vector(C_MAX_CHIPS - 1 downto 0) :=
        (others => '0');

    -- The input event and its selected per-Chip owner context are registered
    -- before any wide identity comparison. This keeps the upstream ready path
    -- independent of metadata RAM and context comparison logic.
    signal pending_event_r : gpx_hit_event_t := C_GPX_HIT_EVENT_IDLE;
    signal pending_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal pending_identity_r : shot_identity_t := (others => '0');
    signal pending_expected_identity_r : shot_identity_t := (others => '0');
    signal pending_owner_context_r : shot_start_event_t :=
        C_SHOT_START_EVENT_IDLE;
    signal pending_owner_sequence_r : unsigned(15 downto 0) :=
        (others => '0');
    signal pending_owner_max_hits_r : unsigned(2 downto 0) :=
        to_unsigned(1, 3);
    signal pending_requested_max_hits_r : unsigned(2 downto 0) :=
        to_unsigned(1, 3);
    signal pending_owner_fault_r : std_logic := '0';
    signal pending_owner_rise_r : std_logic := '0';
    signal pending_owner_fall_r : std_logic := '0';
    signal pending_cfg_valid_r : std_logic := '0';
    signal pending_compare_fault_r : std_logic_vector(4 downto 0) :=
        (others => '0');
    signal pending_address_r : natural range 0 to
        C_CELL_ADDRESS_COUNT - 1 := 0;
    signal scrub_index_r : natural range 0 to C_CELLS_PER_CHIP - 1 := 0;

    signal meta_read_r : cell_meta_t := (others => '0');
    signal cell_address_r : natural range 0 to
        C_CELL_ADDRESS_COUNT - 1 := 0;
    signal cell_error_read_r : std_logic := '0';

    signal emit_chip_r : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal emit_stop_r : natural range 0 to C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal emit_stop_first_r : natural range 0 to C_MAX_STOPS_PER_CHIP := 0;
    signal emit_stop_last_r : natural range 0 to
        C_MAX_STOPS_PER_CHIP - 1 := 0;
    signal emit_slope_r : gpx_slope_t := GPX_SLOPE_RISE;
    signal emit_control_kind_r : gpx_cell_event_kind_t :=
        GPX_CELL_DRAIN_DONE;
    signal emit_ififo_r : std_logic := '0';
    signal emit_error_fill_r : std_logic := '0';
    signal emit_timeout_cause_r : std_logic_vector(2 downto 0) :=
        (others => '0');
    signal emit_context_r : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal emit_sequence_r : unsigned(15 downto 0) := (others => '0');
    signal emit_max_hits_r : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal emit_faulted_r : std_logic := '0';
    signal emit_rise_enabled_r : std_logic := '0';
    signal emit_fall_enabled_r : std_logic := '0';

    signal cell_event_r : gpx_cell_event_t := C_GPX_CELL_EVENT_IDLE;
    signal fault_pulse_r : gpx_cell_collector_faults_t :=
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
        return chip_index * C_CELLS_PER_CHIP + slope_offset + stop_index;
    end function fn_cell_address;

    function fn_shot_identity(
        event_value : gpx_hit_event_t
    ) return shot_identity_t is
    begin
        return std_logic_vector(event_value.chip_shot_seq) &
               std_logic_vector(
                   event_value.shot_context.request.active_version) &
               std_logic_vector(
                   event_value.shot_context.request.shot_index) &
               std_logic_vector(
                   event_value.shot_context.request.face_index) &
               event_value.shot_context.request.source_sim;
    end function fn_shot_identity;

    function fn_slope_supported(
        slope        : gpx_slope_t;
        rise_enabled : std_logic;
        fall_enabled : std_logic
    ) return boolean is
    begin
        if slope = GPX_SLOPE_RISE then
            return rise_enabled = '1';
        end if;
        return fall_enabled = '1';
    end function fn_slope_supported;

    function fn_first_slope(
        rise_enabled : std_logic
    ) return gpx_slope_t is
    begin
        if rise_enabled = '1' then
            return GPX_SLOPE_RISE;
        end if;
        return GPX_SLOPE_FALL;
    end function fn_first_slope;

    function fn_has_emit_cells(
        chip_index : natural;
        first_stop : natural;
        rise_enabled : std_logic;
        fall_enabled : std_logic
    ) return boolean is
    begin
        return chip_index < G_BUILD_CONFIG.num_chips and
               first_stop < G_BUILD_CONFIG.stops_per_chip and
               (rise_enabled = '1' or fall_enabled = '1');
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
        variable chip_index    : natural range 0 to C_MAX_CHIPS - 1;
        variable stop_index    : natural range 0 to C_MAX_STOPS_PER_CHIP - 1;
        variable address_value : natural range 0 to C_CELL_ADDRESS_COUNT - 1;
        variable first_stop    : natural range 0 to C_MAX_STOPS_PER_CHIP;
        variable last_stop     : natural range 0 to
            C_MAX_STOPS_PER_CHIP - 1;
        variable seen_count    : natural range 0 to C_MAX_RETURNS_PER_STOP;
        variable visible_count : natural range 0 to C_MAX_RETURNS_PER_STOP;
        variable effective_max : unsigned(2 downto 0);
        variable meta_value    : cell_meta_t;
        variable context_fault : boolean;
        variable event_faulted : std_logic;
        variable initial_fault : std_logic;
        variable next_exists   : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r            <= S_COLLECT;
                shot_active_r      <= (others => '0');
                shot_context_r     <= (others => C_SHOT_START_EVENT_IDLE);
                shot_identity_r    <= (others => (others => '0'));
                chip_sequence_r    <= (others => (others => '0'));
                max_hits_r         <= (others => (others => '0'));
                shot_fault_r       <= (others => '0');
                lower_emitted_r    <= (others => '0');
                rise_enabled_r     <= (others => '0');
                fall_enabled_r     <= (others => '0');
                pending_event_r    <= C_GPX_HIT_EVENT_IDLE;
                pending_chip_r     <= 0;
                pending_identity_r <= (others => '0');
                pending_expected_identity_r <= (others => '0');
                pending_owner_context_r <= C_SHOT_START_EVENT_IDLE;
                pending_owner_sequence_r <= (others => '0');
                pending_owner_max_hits_r <= to_unsigned(1, 3);
                pending_requested_max_hits_r <= to_unsigned(1, 3);
                pending_owner_fault_r <= '0';
                pending_owner_rise_r <= '0';
                pending_owner_fall_r <= '0';
                pending_cfg_valid_r <= '0';
                pending_compare_fault_r <= (others => '0');
                pending_address_r <= 0;
                scrub_index_r <= 0;
                meta_read_r <= (others => '0');
                cell_address_r <= 0;
                cell_error_read_r <= '0';
                emit_chip_r <= 0;
                emit_stop_r <= 0;
                emit_stop_first_r <= 0;
                emit_stop_last_r <= 0;
                emit_slope_r <= GPX_SLOPE_RISE;
                emit_control_kind_r <= GPX_CELL_DRAIN_DONE;
                emit_ififo_r <= '0';
                emit_error_fill_r <= '0';
                emit_timeout_cause_r <= (others => '0');
                emit_context_r <= C_SHOT_START_EVENT_IDLE;
                emit_sequence_r <= (others => '0');
                emit_max_hits_r <= to_unsigned(1, 3);
                emit_faulted_r <= '0';
                emit_rise_enabled_r <= '0';
                emit_fall_enabled_r <= '0';
                cell_event_r <= C_GPX_CELL_EVENT_IDLE;
                fault_pulse_r <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                fault_sticky_r <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
            else
                fault_pulse_r <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;

                if i_clear_sticky = '1' then
                    fault_sticky_r <= C_GPX_CELL_COLLECTOR_FAULTS_CLEAR;
                end if;

                if i_abort = '1' then
                    state_r         <= S_COLLECT;
                    shot_active_r   <= (others => '0');
                    shot_fault_r    <= (others => '0');
                    lower_emitted_r <= (others => '0');
                    rise_enabled_r  <= (others => '0');
                    fall_enabled_r  <= (others => '0');
                    cell_event_r    <= C_GPX_CELL_EVENT_IDLE;
                else
                    case state_r is
                        when S_COLLECT =>
                            if i_hit_event.valid = '1' then
                                chip_index := to_integer(i_hit_event.chip_index);

                                if chip_index >= G_BUILD_CONFIG.num_chips then
                                    fault_pulse_r.context_mismatch <= '1';
                                    fault_sticky_r.context_mismatch <= '1';
                                else
                                    effective_max := fn_effective_max_hits(
                                        i_max_hits_per_stop);

                                    pending_event_r <= i_hit_event;
                                    pending_chip_r <= chip_index;
                                    pending_requested_max_hits_r <=
                                        effective_max;
                                    pending_identity_r <=
                                        fn_shot_identity(i_hit_event);

                                    if i_hit_event.shot_context.request.
                                            active_version = i_active_version and
                                       i_max_hits_per_stop /= 0 and
                                       to_integer(i_max_hits_per_stop) <=
                                           G_BUILD_CONFIG.
                                               max_returns_per_stop then
                                        pending_cfg_valid_r <= '1';
                                    else
                                        pending_cfg_valid_r <= '0';
                                    end if;
                                    state_r <= S_EVENT_SELECT;
                                end if;
                            end if;

                        when S_EVENT_SELECT =>
                            chip_index := pending_chip_r;
                            initial_fault := pending_event_r.faulted;
                            if pending_cfg_valid_r = '0' then
                                initial_fault := '1';
                            end if;

                            if shot_active_r(chip_index) = '0' then
                                pending_expected_identity_r <=
                                    pending_identity_r;
                                pending_owner_context_r <=
                                    pending_event_r.shot_context;
                                pending_owner_sequence_r <=
                                    pending_event_r.chip_shot_seq;
                                pending_owner_max_hits_r <=
                                    pending_requested_max_hits_r;
                                pending_owner_fault_r <= initial_fault;
                                pending_owner_rise_r <=
                                    i_active_rise_mask(chip_index);
                                pending_owner_fall_r <=
                                    i_active_fall_mask(chip_index);

                                shot_active_r(chip_index) <= '1';
                                shot_context_r(chip_index) <=
                                    pending_event_r.shot_context;
                                shot_identity_r(chip_index) <=
                                    pending_identity_r;
                                chip_sequence_r(chip_index) <=
                                    pending_event_r.chip_shot_seq;
                                max_hits_r(chip_index) <=
                                    pending_requested_max_hits_r;
                                shot_fault_r(chip_index) <= initial_fault;
                                lower_emitted_r(chip_index) <= '0';
                                rise_enabled_r(chip_index) <=
                                    i_active_rise_mask(chip_index);
                                fall_enabled_r(chip_index) <=
                                    i_active_fall_mask(chip_index);

                                scrub_index_r <= 0;
                                state_r <= S_SCRUB_META;
                            else
                                pending_expected_identity_r <=
                                    shot_identity_r(chip_index);
                                pending_owner_context_r <=
                                    shot_context_r(chip_index);
                                pending_owner_sequence_r <=
                                    chip_sequence_r(chip_index);
                                pending_owner_max_hits_r <=
                                    max_hits_r(chip_index);
                                pending_owner_fault_r <=
                                    shot_fault_r(chip_index);
                                pending_owner_rise_r <=
                                    rise_enabled_r(chip_index);
                                pending_owner_fall_r <=
                                    fall_enabled_r(chip_index);
                                state_r <= S_EVENT_CHECK;
                            end if;

                        when S_SCRUB_META =>
                            address_value := pending_chip_r *
                                C_CELLS_PER_CHIP + scrub_index_r;
                            cell_meta_r(address_value) <= (others => '0');

                            if scrub_index_r = C_CELLS_PER_CHIP - 1 then
                                state_r <= S_EVENT_CHECK;
                            else
                                scrub_index_r <= scrub_index_r + 1;
                            end if;

                        when S_EVENT_CHECK =>
                            if pending_identity_r(51 downto 36) /=
                               pending_expected_identity_r(51 downto 36) then
                                pending_compare_fault_r(0) <= '1';
                            else
                                pending_compare_fault_r(0) <= '0';
                            end if;
                            if pending_identity_r(35 downto 20) /=
                               pending_expected_identity_r(35 downto 20) then
                                pending_compare_fault_r(1) <= '1';
                            else
                                pending_compare_fault_r(1) <= '0';
                            end if;
                            if pending_identity_r(19 downto 4) /=
                               pending_expected_identity_r(19 downto 4) then
                                pending_compare_fault_r(2) <= '1';
                            else
                                pending_compare_fault_r(2) <= '0';
                            end if;
                            if pending_identity_r(3 downto 0) /=
                               pending_expected_identity_r(3 downto 0) then
                                pending_compare_fault_r(3) <= '1';
                            else
                                pending_compare_fault_r(3) <= '0';
                            end if;
                            if pending_owner_max_hits_r /=
                                   pending_requested_max_hits_r or
                               pending_cfg_valid_r = '0' then
                                pending_compare_fault_r(4) <= '1';
                            else
                                pending_compare_fault_r(4) <= '0';
                            end if;
                            state_r <= S_EVENT_ROUTE;

                        when S_EVENT_ROUTE =>
                            context_fault := pending_compare_fault_r /=
                                "00000";

                            event_faulted := pending_owner_fault_r or
                                pending_event_r.faulted;
                            if context_fault then
                                event_faulted := '1';
                                shot_fault_r(pending_chip_r) <= '1';
                                fault_pulse_r.context_mismatch <= '1';
                                fault_sticky_r.context_mismatch <= '1';
                            elsif pending_event_r.faulted = '1' then
                                shot_fault_r(pending_chip_r) <= '1';
                            end if;

                            if pending_event_r.kind = GPX_HIT_DATA and
                               fn_slope_supported(
                                   pending_event_r.slope,
                                   pending_owner_rise_r,
                                   pending_owner_fall_r) then
                                stop_index := to_integer(
                                    pending_event_r.stop_index);
                                pending_address_r <= fn_cell_address(
                                    pending_chip_r,
                                    pending_event_r.slope,
                                    stop_index);
                                state_r <= S_HIT_META_READ;
                            elsif pending_event_r.kind = GPX_HIT_DATA then
                                shot_fault_r(pending_chip_r) <= '1';
                                fault_pulse_r.context_mismatch <= '1';
                                fault_sticky_r.context_mismatch <= '1';
                                state_r <= S_COLLECT;
                            else
                                emit_chip_r <= pending_chip_r;
                                emit_ififo_r <= pending_event_r.ififo_id;
                                emit_timeout_cause_r <=
                                    pending_event_r.timeout_cause;
                                emit_context_r <= pending_owner_context_r;
                                emit_sequence_r <= pending_owner_sequence_r;
                                emit_max_hits_r <= pending_owner_max_hits_r;
                                emit_faulted_r <= event_faulted;
                                emit_rise_enabled_r <= pending_owner_rise_r;
                                emit_fall_enabled_r <= pending_owner_fall_r;

                                case pending_event_r.kind is
                                    when GPX_HIT_IFIFO1_DONE =>
                                        emit_control_kind_r <=
                                            GPX_CELL_IFIFO1_DONE;
                                        first_stop := 0;
                                        if G_BUILD_CONFIG.stops_per_chip > 4 then
                                            last_stop := 3;
                                        else
                                            last_stop := G_BUILD_CONFIG.
                                                stops_per_chip - 1;
                                        end if;
                                        emit_error_fill_r <=
                                            pending_event_r.faulted;

                                    when GPX_HIT_DRAIN_DONE =>
                                        emit_control_kind_r <=
                                            GPX_CELL_DRAIN_DONE;
                                        if lower_emitted_r(
                                                pending_chip_r) = '1' then
                                            first_stop := 4;
                                        else
                                            first_stop := 0;
                                        end if;
                                        last_stop := G_BUILD_CONFIG.
                                            stops_per_chip - 1;
                                        emit_error_fill_r <=
                                            pending_event_r.faulted;

                                    when GPX_HIT_TIMEOUT =>
                                        emit_control_kind_r <= GPX_CELL_TIMEOUT;
                                        if lower_emitted_r(
                                                pending_chip_r) = '1' then
                                            first_stop := 4;
                                        else
                                            first_stop := 0;
                                        end if;
                                        last_stop := G_BUILD_CONFIG.
                                            stops_per_chip - 1;
                                        emit_error_fill_r <= '1';
                                        emit_faulted_r <= '1';
                                        shot_fault_r(pending_chip_r) <= '1';

                                    when GPX_HIT_DATA =>
                                        first_stop := 0;
                                        last_stop := 0;
                                end case;

                                emit_stop_first_r <= first_stop;
                                emit_stop_last_r <= last_stop;
                                emit_stop_r <= first_stop;
                                emit_slope_r <= fn_first_slope(
                                    pending_owner_rise_r);

                                if fn_has_emit_cells(
                                        pending_chip_r, first_stop,
                                        pending_owner_rise_r,
                                        pending_owner_fall_r) then
                                    state_r <= S_CELL_META_READ;
                                else
                                    state_r <= S_CONTROL_LOAD;
                                end if;
                            end if;

                        when S_HIT_META_READ =>
                            meta_read_r <= cell_meta_r(pending_address_r);
                            state_r <= S_HIT_APPLY;

                        when S_HIT_APPLY =>
                            address_value := pending_address_r;
                            meta_value := meta_read_r;
                            seen_count := to_integer(unsigned(meta_value(
                                C_META_COUNT_HI downto C_META_COUNT_LO)));

                            if pending_event_r.start_number /= 0 then
                                meta_value(C_META_ERROR) := '1';
                                shot_fault_r(pending_chip_r) <= '1';
                                fault_pulse_r.start_number_nonzero <= '1';
                                fault_sticky_r.start_number_nonzero <= '1';
                            end if;

                            if seen_count <
                               G_BUILD_CONFIG.max_returns_per_stop then
                                if seen_count < to_integer(
                                        pending_owner_max_hits_r) then
                                    case seen_count is
                                        when 0 =>
                                            hit_bank_0_r(address_value) <=
                                                pending_event_r.hit;
                                        when 1 =>
                                            hit_bank_1_r(address_value) <=
                                                pending_event_r.hit;
                                        when 2 =>
                                            hit_bank_2_r(address_value) <=
                                                pending_event_r.hit;
                                        when 3 =>
                                            hit_bank_3_r(address_value) <=
                                                pending_event_r.hit;
                                        when 4 =>
                                            hit_bank_4_r(address_value) <=
                                                pending_event_r.hit;
                                        when 5 =>
                                            hit_bank_5_r(address_value) <=
                                                pending_event_r.hit;
                                        when 6 =>
                                            hit_bank_6_r(address_value) <=
                                                pending_event_r.hit;
                                        when others =>
                                            null;
                                    end case;
                                else
                                    meta_value(C_META_DROPPED) := '1';
                                    fault_pulse_r.hit_capacity_drop <= '1';
                                    fault_sticky_r.hit_capacity_drop <= '1';
                                end if;

                                meta_value(
                                    C_META_COUNT_HI downto C_META_COUNT_LO) :=
                                    std_logic_vector(to_unsigned(
                                        seen_count + 1, 3));
                            else
                                meta_value(C_META_DROPPED) := '1';
                                meta_value(C_META_ERROR) := '1';
                                shot_fault_r(pending_chip_r) <= '1';
                                fault_pulse_r.return_overflow <= '1';
                                fault_sticky_r.return_overflow <= '1';
                                fault_pulse_r.hit_capacity_drop <= '1';
                                fault_sticky_r.hit_capacity_drop <= '1';
                            end if;

                            cell_meta_r(address_value) <= meta_value;
                            state_r <= S_COLLECT;

                        when S_CELL_META_READ =>
                            address_value := fn_cell_address(
                                emit_chip_r, emit_slope_r, emit_stop_r);
                            cell_address_r <= address_value;
                            meta_read_r <= cell_meta_r(address_value);
                            state_r <= S_CELL_HIT_READ;

                        when S_CELL_HIT_READ =>
                            seen_count := to_integer(unsigned(meta_read_r(
                                C_META_COUNT_HI downto C_META_COUNT_LO)));
                            if seen_count > to_integer(emit_max_hits_r) then
                                visible_count := to_integer(emit_max_hits_r);
                            else
                                visible_count := seen_count;
                            end if;

                            cell_event_r.hit_count <= to_unsigned(
                                visible_count, cell_event_r.hit_count'length);
                            cell_event_r.hit_dropped <=
                                meta_read_r(C_META_DROPPED);
                            cell_error_read_r <= meta_read_r(C_META_ERROR);

                            if visible_count > 0 then
                                cell_event_r.hits(0) <=
                                    hit_bank_0_r(cell_address_r);
                            else
                                cell_event_r.hits(0) <= (others => '0');
                            end if;
                            if visible_count > 1 then
                                cell_event_r.hits(1) <=
                                    hit_bank_1_r(cell_address_r);
                            else
                                cell_event_r.hits(1) <= (others => '0');
                            end if;
                            if visible_count > 2 then
                                cell_event_r.hits(2) <=
                                    hit_bank_2_r(cell_address_r);
                            else
                                cell_event_r.hits(2) <= (others => '0');
                            end if;
                            if visible_count > 3 then
                                cell_event_r.hits(3) <=
                                    hit_bank_3_r(cell_address_r);
                            else
                                cell_event_r.hits(3) <= (others => '0');
                            end if;
                            if visible_count > 4 then
                                cell_event_r.hits(4) <=
                                    hit_bank_4_r(cell_address_r);
                            else
                                cell_event_r.hits(4) <= (others => '0');
                            end if;
                            if visible_count > 5 then
                                cell_event_r.hits(5) <=
                                    hit_bank_5_r(cell_address_r);
                            else
                                cell_event_r.hits(5) <= (others => '0');
                            end if;
                            if visible_count > 6 then
                                cell_event_r.hits(6) <=
                                    hit_bank_6_r(cell_address_r);
                            else
                                cell_event_r.hits(6) <= (others => '0');
                            end if;
                            state_r <= S_CELL_LOAD;

                        when S_CELL_LOAD =>
                            cell_event_r.valid <= '1';
                            cell_event_r.kind <= GPX_CELL_DATA;
                            cell_event_r.chip_index <= to_unsigned(
                                emit_chip_r, cell_event_r.chip_index'length);
                            if emit_stop_r >= 4 then
                                cell_event_r.ififo_id <= '1';
                            else
                                cell_event_r.ififo_id <= '0';
                            end if;
                            cell_event_r.stop_index <= to_unsigned(
                                emit_stop_r, cell_event_r.stop_index'length);
                            cell_event_r.slope <= emit_slope_r;
                            cell_event_r.max_hits <= emit_max_hits_r;
                            cell_event_r.error_fill <= emit_error_fill_r;
                            cell_event_r.faulted <= cell_error_read_r or
                                emit_faulted_r or emit_error_fill_r;
                            cell_event_r.timeout_cause <=
                                emit_timeout_cause_r;
                            cell_event_r.shot_context <= emit_context_r;
                            cell_event_r.chip_shot_seq <= emit_sequence_r;
                            state_r <= S_CELL_WAIT;

                        when S_CELL_WAIT =>
                            if cell_event_r.valid = '1' and
                               i_cell_ready = '1' then
                                cell_event_r.valid <= '0';

                                next_exists := false;
                                if emit_stop_r < emit_stop_last_r then
                                    emit_stop_r <= emit_stop_r + 1;
                                    next_exists := true;
                                elsif emit_slope_r = GPX_SLOPE_RISE and
                                      emit_fall_enabled_r = '1' then
                                    emit_slope_r <= GPX_SLOPE_FALL;
                                    emit_stop_r <= emit_stop_first_r;
                                    next_exists := true;
                                end if;

                                if next_exists then
                                    state_r <= S_CELL_META_READ;
                                else
                                    state_r <= S_CONTROL_LOAD;
                                end if;
                            end if;

                        when S_CONTROL_LOAD =>
                            cell_event_r.valid <= '1';
                            cell_event_r.kind <= emit_control_kind_r;
                            cell_event_r.chip_index <= to_unsigned(
                                emit_chip_r, cell_event_r.chip_index'length);
                            cell_event_r.ififo_id <= emit_ififo_r;
                            cell_event_r.stop_index <= (others => '0');
                            cell_event_r.slope <= fn_first_slope(
                                emit_rise_enabled_r);
                            cell_event_r.hit_count <= (others => '0');
                            cell_event_r.max_hits <= emit_max_hits_r;
                            cell_event_r.hits <= (others => (others => '0'));
                            cell_event_r.hit_dropped <= '0';
                            cell_event_r.error_fill <= emit_error_fill_r;
                            cell_event_r.faulted <=
                                emit_faulted_r or emit_error_fill_r;
                            cell_event_r.timeout_cause <=
                                emit_timeout_cause_r;
                            cell_event_r.shot_context <= emit_context_r;
                            cell_event_r.chip_shot_seq <= emit_sequence_r;

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
                                    shot_active_r(emit_chip_r) <= '0';
                                    shot_fault_r(emit_chip_r) <= '0';
                                    lower_emitted_r(emit_chip_r) <= '0';
                                    rise_enabled_r(emit_chip_r) <= '0';
                                    fall_enabled_r(emit_chip_r) <= '0';
                                end if;
                                state_r <= S_COLLECT;
                            end if;
                    end case;
                end if;
            end if;
        end if;
    end process p_collect;

end architecture rtl;
