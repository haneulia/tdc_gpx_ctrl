library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- B2 Processing boundary. A Face entry anchors a fixed angular lattice.
-- Busy due points are consumed and diagnosed; they are never retried at a
-- later off-grid position. The block owns no RUN, ARM or permit state and
-- consumes only the F3a scheduler-enable decision.
entity shot_scheduler is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                       : in  std_logic;
        i_rst_n                     : in  std_logic;
        i_enable                    : in  std_logic;
        i_active_valid              : in  std_logic;
        i_active_config             : in  lidar_active_config_t;
        i_face_event                : in  face_event_t;
        i_executor_ready            : in  std_logic;
        i_request_accept            : in  std_logic;
        i_request_drop              : in  std_logic;
        i_clear_diagnostics         : in  std_logic;
        o_shot_request              : out shot_request_t;
        o_schedule_overrun_pulse    : out std_logic;
        o_schedule_overrun_sticky   : out std_logic;
        o_schedule_overrun_count    : out u32_t;
        o_idle                      : out std_logic
    );
end entity shot_scheduler;

architecture rtl of shot_scheduler is

    constant C_BUILD_FACE_MASK : face_mask_t :=
        fn_active_face_mask(G_BUILD_CONFIG.num_faces);
    -- face_tracker has two registered stages. After scheduler enable rises,
    -- discard events already resident in those stages so a pre-ARM entry can
    -- never become the first physical shot of a newly armed session.
    constant C_ENABLE_EVENT_QUARANTINE_CLKS : natural := 2;

    function fn_context_is_valid(
        event_value       : face_event_t;
        interval_value    : u16_t;
        columns_value     : u16_t;
        face_mask_value   : face_mask_t;
        simulation_value  : std_logic;
        version_value     : u16_t
    ) return boolean is
        variable face_number : natural range 0 to 7;
    begin
        face_number := to_integer(event_value.face_index);
        if face_number >= G_BUILD_CONFIG.num_faces then
            return false;
        end if;
        return interval_value /= 0 and columns_value /= 0 and
            face_mask_value(face_number) = '1' and
            event_value.overlap = '0' and
            event_value.active_version = version_value and
            event_value.source_sim = simulation_value;
    end function fn_context_is_valid;

    function fn_make_request(
        event_value  : face_event_t;
        column_value : shot_index_t;
        columns_value: u16_t
    ) return shot_request_t is
        variable result : shot_request_t := C_SHOT_REQUEST_IDLE;
    begin
        result.valid                := '1';
        result.face_index           := event_value.face_index;
        result.position             := event_value.position;
        result.direction            := event_value.direction;
        result.shot_index           := column_value;
        result.source_sim           := event_value.source_sim;
        result.source_latency_clks  := event_value.source_latency_clks;
        result.source_latency_valid := event_value.source_latency_valid;
        result.active_version       := event_value.active_version;
        if column_value + 1 >= columns_value then
            result.last_in_face := '1';
        end if;
        return result;
    end function fn_make_request;

    signal shot_interval_r : u16_t := (others => '0');
    signal columns_r       : u16_t := (others => '0');
    signal face_mask_r     : face_mask_t := (others => '0');
    signal simulation_mode_r : std_logic := '0';
    signal active_version_r  : u16_t := (others => '0');

    signal session_active_r : std_logic := '0';
    signal enable_age_r     : natural range 0 to
        C_ENABLE_EVENT_QUARANTINE_CLKS := 0;
    signal session_face_r   : face_index_t := (others => '0');
    signal grid_countdown_r : u16_t := (others => '0');
    signal next_column_r    : shot_index_t := (others => '0');
    signal inflight_r       : std_logic := '0';

    signal shot_request_r   : shot_request_t := C_SHOT_REQUEST_IDLE;
    signal overrun_pulse_r  : std_logic := '0';
    signal overrun_sticky_r : std_logic := '0';
    signal overrun_count_r  : u32_t := (others => '0');

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-SCHED-001 invalid build configuration"
        severity failure;

    o_shot_request            <= shot_request_r;
    o_schedule_overrun_pulse  <= overrun_pulse_r;
    o_schedule_overrun_sticky <= overrun_sticky_r;
    o_schedule_overrun_count  <= overrun_count_r;
    o_idle                    <= not inflight_r;

    -- Commit-time arithmetic owns interval and column derivation. B2 copies
    -- the result only while F3a has disabled scheduling, keeping the real-time
    -- path to compare/decrement/increment operations.
    p_config : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                shot_interval_r   <= (others => '0');
                columns_r         <= (others => '0');
                face_mask_r       <= (others => '0');
                simulation_mode_r <= '0';
                active_version_r  <= (others => '0');
            elsif i_enable = '0' and i_active_valid = '1' then
                shot_interval_r <=
                    i_active_config.derived.shot_interval_states;
                columns_r <= i_active_config.derived.columns_per_face;
                face_mask_r <= i_active_config.source.laser.face_enable_mask
                    and C_BUILD_FACE_MASK;
                simulation_mode_r <=
                    i_active_config.source.motor.simulation_mode;
                active_version_r <= i_active_config.version;
            end if;
        end if;
    end process p_config;

    p_scheduler : process (i_clk)
        variable request_v       : shot_request_t;
        variable session_v       : std_logic;
        variable enable_age_v    : natural range 0 to
            C_ENABLE_EVENT_QUARANTINE_CLKS;
        variable session_face_v  : face_index_t;
        variable countdown_v     : u16_t;
        variable column_v        : shot_index_t;
        variable inflight_v      : std_logic;
        variable context_valid_v : boolean;
        variable due_v           : boolean;
        variable blocked_v       : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                session_active_r <= '0';
                enable_age_r     <= 0;
                session_face_r   <= (others => '0');
                grid_countdown_r <= (others => '0');
                next_column_r    <= (others => '0');
                inflight_r       <= '0';
                shot_request_r   <= C_SHOT_REQUEST_IDLE;
                overrun_pulse_r  <= '0';
            else
                request_v      := shot_request_r;
                request_v.valid := '0';
                session_v      := session_active_r;
                enable_age_v   := enable_age_r;
                session_face_v := session_face_r;
                countdown_v    := grid_countdown_r;
                column_v       := next_column_r;
                inflight_v     := inflight_r;
                blocked_v      := false;
                if inflight_v = '1' and
                   (i_request_accept = '1' or i_request_drop = '1') then
                    inflight_v := '0';
                end if;
                if i_active_valid /= '1' then
                    session_v   := '0';
                    enable_age_v := 0;
                    countdown_v := (others => '0');
                    column_v    := (others => '0');
                    inflight_v  := '0';
                elsif i_enable /= '1' then
                    session_v   := '0';
                    enable_age_v := 0;
                    countdown_v := (others => '0');
                    column_v    := (others => '0');
                elsif enable_age_v < C_ENABLE_EVENT_QUARANTINE_CLKS then
                    enable_age_v := enable_age_v + 1;
                    session_v    := '0';
                    countdown_v  := (others => '0');
                    column_v     := (others => '0');
                elsif i_face_event.valid = '1' then
                    if i_face_event.inside /= '1' then
                        session_v   := '0';
                        countdown_v := (others => '0');
                        column_v    := (others => '0');
                    else
                        if i_face_event.enter_event = '1' then
                            session_v      := '1';
                            session_face_v := i_face_event.face_index;
                            countdown_v    := (others => '0');
                            column_v       := (others => '0');
                        elsif session_v = '1' and
                              i_face_event.face_index /= session_face_v then
                            session_v := '0';
                        end if;
                        if i_face_event.overlap = '1' then
                            session_v := '0';
                        end if;
                        due_v := session_v = '1' and
                            i_face_event.face_index = session_face_v and
                            countdown_v = 0 and column_v < columns_r;

                        if due_v then
                            context_valid_v := fn_context_is_valid(
                                i_face_event, shot_interval_r, columns_r,
                                face_mask_r, simulation_mode_r,
                                active_version_r);
                            if context_valid_v and i_executor_ready = '1' and
                               inflight_v = '0' then
                                request_v := fn_make_request(
                                    i_face_event, column_v, columns_r);
                                inflight_v := '1';
                            elsif context_valid_v then
                                blocked_v := true;
                            end if;
                            column_v := column_v + 1;
                            if shot_interval_r > 1 then
                                countdown_v := shot_interval_r - 1;
                            else
                                countdown_v := (others => '0');
                            end if;
                            if column_v >= columns_r then
                                session_v := '0';
                            end if;
                        elsif session_v = '1' and countdown_v /= 0 then
                            countdown_v := countdown_v - 1;
                        end if;
                    end if;
                end if;
                if blocked_v then
                    overrun_pulse_r <= '1';
                else
                    overrun_pulse_r <= '0';
                end if;
                shot_request_r   <= request_v;
                session_active_r <= session_v;
                enable_age_r     <= enable_age_v;
                session_face_r   <= session_face_v;
                grid_countdown_r <= countdown_v;
                next_column_r    <= column_v;
                inflight_r       <= inflight_v;
            end if;
        end if;
    end process p_scheduler;

    p_diagnostics : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                overrun_sticky_r <= '0';
                overrun_count_r  <= (others => '0');
            else
                if i_clear_diagnostics = '1' then
                    overrun_sticky_r <= '0';
                    overrun_count_r  <= (others => '0');
                end if;
                if overrun_pulse_r = '1' then
                    overrun_sticky_r <= '1';
                    if i_clear_diagnostics = '1' then
                        overrun_count_r <= to_unsigned(
                            1, overrun_count_r'length);
                    else
                        overrun_count_r <= overrun_count_r + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_diagnostics;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                assert not (i_request_accept = '1' and
                            i_request_drop = '1')
                    report "V2-SCHED-002 accept and drop overlap"
                    severity failure;
                if i_request_accept = '1' or i_request_drop = '1' then
                    assert inflight_r = '1'
                        report "V2-SCHED-009 resolve without in-flight request"
                        severity failure;
                end if;
                if i_enable = '1' then
                    assert i_active_valid = '1'
                        report "V2-SCHED-003 enable without active configuration"
                        severity failure;
                    assert i_active_config.version = active_version_r
                        report "V2-SCHED-004 active version changed while enabled"
                        severity failure;
                    assert shot_interval_r /= 0 and columns_r /= 0
                        report "V2-SCHED-005 invalid derived shot geometry"
                        severity failure;
                    if i_face_event.valid = '1' then
                        assert i_face_event.active_version = active_version_r
                            report "V2-SCHED-006 Face/config version mismatch"
                            severity failure;
                        assert i_face_event.source_sim = simulation_mode_r
                            report "V2-SCHED-007 Face/source mode mismatch"
                            severity failure;
                    end if;
                end if;
                if shot_request_r.valid = '1' then
                    assert inflight_r = '1'
                        report "V2-SCHED-008 request without in-flight ownership"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
