library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_echo_pkg.all;

-- Observation-only Processing-domain Echo counter and Shot snapshot owner.
-- These values describe edges seen before GPX acceptance and never control
-- STOP, acquisition completion or IFIFO drain bounds.
entity echo_diagnostics is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        i_clear             : in  std_logic;
        i_simulation_active : in  std_logic;
        i_profile_not_ready : in  std_logic;
        i_shot_start        : in  shot_start_event_t;
        i_shot_result       : in  shot_result_t;
        i_rise_event        : in  std_logic_vector(
            fn_echo_channel_count(G_BUILD_CONFIG) - 1 downto 0);
        i_fall_event        : in  std_logic_vector(
            fn_echo_channel_count(G_BUILD_CONFIG) - 1 downto 0);
        o_window_active     : out std_logic;
        o_diagnostics       : out echo_diagnostics_t
    );
end entity echo_diagnostics;

architecture rtl of echo_diagnostics is

    constant C_NUM_CHANNELS : positive :=
        fn_echo_channel_count(G_BUILD_CONFIG);
    constant C_COUNT_MAX : echo_count_t := (others => '1');

    subtype event_count_3_t is unsigned(2 downto 0);
    subtype event_count_4_t is unsigned(3 downto 0);
    subtype event_count_5_t is unsigned(4 downto 0);
    subtype event_count_6_t is unsigned(5 downto 0);
    type event_count_3_array_t is array (0 to 7) of event_count_3_t;
    type event_count_4_array_t is array (0 to 3) of event_count_4_t;
    type event_count_5_array_t is array (0 to 1) of event_count_5_t;

    -- Explicitly balanced 32-bit popcount. A flat variable accumulator creates
    -- a 32-deep carry chain at maximum topology and fails the 200 MHz observer
    -- timing even though the physical STOP path remains direct.
    function fn_event_popcount(
        events : std_logic_vector
    ) return event_count_6_t is
        variable padded : std_logic_vector(31 downto 0) := (others => '0');
        variable level_0 : event_count_3_array_t :=
            (others => (others => '0'));
        variable level_1 : event_count_4_array_t :=
            (others => (others => '0'));
        variable level_2 : event_count_5_array_t :=
            (others => (others => '0'));
        variable level_3 : event_count_6_t := (others => '0');
    begin
        padded(events'length - 1 downto 0) := events;

        for group_index in 0 to 7 loop
            for bit_index in 0 to 3 loop
                if padded(4 * group_index + bit_index) = '1' then
                    level_0(group_index) := level_0(group_index) + 1;
                end if;
            end loop;
        end loop;
        for group_index in 0 to 3 loop
            level_1(group_index) :=
                resize(level_0(2 * group_index), 4) +
                resize(level_0(2 * group_index + 1), 4);
        end loop;
        for group_index in 0 to 1 loop
            level_2(group_index) :=
                resize(level_1(2 * group_index), 5) +
                resize(level_1(2 * group_index + 1), 5);
        end loop;
        level_3 := resize(level_2(0), 6) + resize(level_2(1), 6);
        return level_3;
    end function fn_event_popcount;

    signal window_active_r : std_logic := '0';
    signal rise_event_stage_r : std_logic_vector(
        C_NUM_CHANNELS - 1 downto 0) := (others => '0');
    signal fall_event_stage_r : std_logic_vector(
        C_NUM_CHANNELS - 1 downto 0) := (others => '0');
    signal any_event_stage_r : std_logic := '0';
    signal shot_start_stage_r : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_result_stage_r : shot_result_t := C_SHOT_RESULT_IDLE;
    signal snapshot_capture_stage_r : std_logic := '0';
    signal profile_not_ready_stage_r : std_logic := '0';
    signal request_r : shot_request_t := C_SHOT_REQUEST_IDLE;
    signal rise_mask_r : echo_channel_mask_t := (others => '0');
    signal fall_mask_r : echo_channel_mask_t := (others => '0');
    signal rise_count_r : echo_count_array_t := C_ECHO_COUNTS_CLEAR;
    signal fall_count_r : echo_count_array_t := C_ECHO_COUNTS_CLEAR;
    signal total_rise_r : echo_total_count_t := (others => '0');
    signal total_fall_r : echo_total_count_t := (others => '0');
    signal rise_increment_r : event_count_6_t := (others => '0');
    signal fall_increment_r : event_count_6_t := (others => '0');

    signal outside_window_pulse_r  : std_logic := '0';
    signal outside_window_sticky_r : std_logic := '0';
    signal outside_window_count_r  : unsigned(31 downto 0) :=
        (others => '0');
    signal overlap_pulse_r  : std_logic := '0';
    signal overlap_sticky_r : std_logic := '0';
    signal overlap_count_r  : unsigned(31 downto 0) := (others => '0');
    signal profile_not_ready_sticky_r : std_logic := '0';
    signal profile_not_ready_count_r : unsigned(31 downto 0) :=
        (others => '0');
    signal snapshot_r : echo_shot_snapshot_t :=
        C_ECHO_SHOT_SNAPSHOT_CLEAR;
    signal snapshot_pending_r : echo_shot_snapshot_t :=
        C_ECHO_SHOT_SNAPSHOT_CLEAR;
    -- [0]: control/request, [1]: masks/totals, [2..9]: four channels/bank.
    -- Equivalent registers are intentionally preserved so each physical bank
    -- owns a local CE source instead of one global high-fanout pulse.
    signal snapshot_finalize_bank_r : std_logic_vector(9 downto 0) :=
        (others => '0');
    -- [0]: window/request, [1]: totals, [2..9]: four channels/bank.
    -- shot_start를 각 register bank 가까이에서 복제하여 32채널 count
    -- 초기화의 전역 fanout을 막는다. 이 경계는 관측/진단 전용이며
    -- 실제 LVDS STOP 전달이나 GPX 수집 시작 시각에는 영향을 주지 않는다.
    signal shot_start_clear_bank_r : std_logic_vector(9 downto 0) :=
        (others => '0');

    -- capture pulse는 staged Shot result와 같은 순차 경계에 있다.
    -- fanout을 제한하면 implementation이 32-channel count bank 가까이에
    -- local register copy를 배치할 수 있다. Finalize는 아래 KEEP bank가
    -- RTL에서 직접 분할하므로 max_fanout 도구 힌트에 의존하지 않는다.
    attribute max_fanout : integer;
    attribute max_fanout of snapshot_capture_stage_r : signal is 48;
    attribute keep : string;
    attribute keep of snapshot_finalize_bank_r : signal is "true";
    attribute keep of shot_start_clear_bank_r : signal is "true";

begin

    o_window_active <= window_active_r;
    o_diagnostics <= (
        window_active         => window_active_r,
        simulation_active     => i_simulation_active,
        outside_window_pulse  => outside_window_pulse_r,
        outside_window_sticky => outside_window_sticky_r,
        outside_window_count  => outside_window_count_r,
        overlap_pulse         => overlap_pulse_r,
        overlap_sticky        => overlap_sticky_r,
        overlap_count         => overlap_count_r,
        profile_not_ready_pulse  => profile_not_ready_stage_r,
        profile_not_ready_sticky => profile_not_ready_sticky_r,
        profile_not_ready_count  => profile_not_ready_count_r,
        snapshot              => snapshot_r
    );

    -- Diagnostics are intentionally one Processing clock behind the physical
    -- STOP path. Register events and the closing Shot-result boundary so the
    -- 32-channel popcount/counters never sit on the frontend synchronizer
    -- route and a final Echo cannot cross that delayed closing boundary. Shot
    -- start도 event와 같은 관측 경계에 등록한다. 같은 원시 clock에서
    -- 들어온 start/event는 아래 bank clear와 event 허용을 함께 적용하여
    -- 첫 Echo를 잃지 않는다. 이 추가 clock은 진단 표시 지연일 뿐이다.
    p_observation_input : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                rise_event_stage_r <= (others => '0');
                fall_event_stage_r <= (others => '0');
                any_event_stage_r <= '0';
                shot_start_stage_r <= C_SHOT_START_EVENT_IDLE;
                shot_start_clear_bank_r <= (others => '0');
                shot_result_stage_r <= C_SHOT_RESULT_IDLE;
                snapshot_capture_stage_r <= '0';
                profile_not_ready_stage_r <= '0';
            else
                rise_event_stage_r <= i_rise_event;
                fall_event_stage_r <= i_fall_event;
                if i_rise_event /= (i_rise_event'range => '0') or
                   i_fall_event /= (i_fall_event'range => '0') then
                    any_event_stage_r <= '1';
                else
                    any_event_stage_r <= '0';
                end if;
                shot_start_stage_r <= i_shot_start;
                shot_start_clear_bank_r <=
                    (others => i_shot_start.valid);
                shot_result_stage_r <= i_shot_result;
                snapshot_capture_stage_r <= i_shot_result.valid;
                profile_not_ready_stage_r <= i_profile_not_ready;
            end if;
        end if;
    end process p_observation_input;

    p_diagnostics : process (i_clk)
        variable rise_count_v : echo_count_array_t;
        variable fall_count_v : echo_count_array_t;
        variable rise_mask_v  : echo_channel_mask_t;
        variable fall_mask_v  : echo_channel_mask_t;
        variable total_rise_v : echo_total_count_t;
        variable total_fall_v : echo_total_count_t;
        variable rise_events_v : event_count_6_t;
        variable fall_events_v : event_count_6_t;
        variable any_event_v  : std_logic;
        variable window_accept_v : std_logic;
        variable snapshot_v   : echo_shot_snapshot_t;
    begin
        if rising_edge(i_clk) then
            outside_window_pulse_r <= '0';
            overlap_pulse_r        <= '0';
            snapshot_r.valid       <= '0';

            if i_rst_n = '0' then
                window_active_r <= '0';
                request_r       <= C_SHOT_REQUEST_IDLE;
                rise_mask_r     <= (others => '0');
                fall_mask_r     <= (others => '0');
                rise_count_r    <= C_ECHO_COUNTS_CLEAR;
                fall_count_r    <= C_ECHO_COUNTS_CLEAR;
                total_rise_r    <= (others => '0');
                total_fall_r    <= (others => '0');
                rise_increment_r <= (others => '0');
                fall_increment_r <= (others => '0');
                outside_window_sticky_r <= '0';
                outside_window_count_r  <= (others => '0');
                overlap_sticky_r <= '0';
                overlap_count_r  <= (others => '0');
                profile_not_ready_sticky_r <= '0';
                profile_not_ready_count_r  <= (others => '0');
                snapshot_r       <= C_ECHO_SHOT_SNAPSHOT_CLEAR;
                snapshot_pending_r <= C_ECHO_SHOT_SNAPSHOT_CLEAR;
                snapshot_finalize_bank_r <= (others => '0');
            else
                snapshot_finalize_bank_r <= (others => '0');
                if i_clear = '1' then
                    outside_window_sticky_r <= '0';
                    outside_window_count_r  <= (others => '0');
                    overlap_sticky_r        <= '0';
                    overlap_count_r         <= (others => '0');
                    profile_not_ready_sticky_r <= '0';
                    profile_not_ready_count_r  <= (others => '0');
                end if;

                if profile_not_ready_stage_r = '1' then
                    profile_not_ready_sticky_r <= '1';
                    profile_not_ready_count_r <=
                        profile_not_ready_count_r + 1;
                end if;

                rise_count_v := rise_count_r;
                fall_count_v := fall_count_r;
                rise_mask_v  := rise_mask_r;
                fall_mask_v  := fall_mask_r;
                total_rise_v := total_rise_r +
                    resize(rise_increment_r, total_rise_r'length);
                total_fall_v := total_fall_r +
                    resize(fall_increment_r, total_fall_r'length);
                rise_events_v := fn_event_popcount(rise_event_stage_r);
                fall_events_v := fn_event_popcount(fall_event_stage_r);
                -- 32채널 OR 결과는 event vector와 같은 입력 단계에서
                -- 등록되어 있다. 채널별 count는 아래 loop가 처리하고,
                -- window 밖 사건 카운터는 이 1-bit 결과만 사용하므로
                -- 32채널 MUX/OR가 32-bit 누적 counter 제어로 이어지지 않는다.
                any_event_v  := any_event_stage_r;
                -- 새 Shot과 같은 관측 clock의 Echo도 첫 Return으로 센다.
                -- 각 count bank는 아래의 로컬 clear bit를 사용한다.
                window_accept_v := window_active_r or
                    shot_start_clear_bank_r(0);
                rise_increment_r <= (others => '0');
                fall_increment_r <= (others => '0');

                for channel in 0 to C_NUM_CHANNELS - 1 loop
                    if shot_start_clear_bank_r(channel / 4 + 2) = '1' then
                        rise_mask_v(channel) := '0';
                        fall_mask_v(channel) := '0';
                        rise_count_v(channel) := (others => '0');
                        fall_count_v(channel) := (others => '0');
                    end if;
                    if rise_event_stage_r(channel) = '1' then
                        if window_accept_v = '1' then
                            rise_mask_v(channel) := '1';
                            if rise_count_v(channel) /= C_COUNT_MAX then
                                rise_count_v(channel) :=
                                    rise_count_v(channel) + 1;
                            end if;
                        end if;
                    end if;
                    if fall_event_stage_r(channel) = '1' then
                        if window_accept_v = '1' then
                            fall_mask_v(channel) := '1';
                            if fall_count_v(channel) /= C_COUNT_MAX then
                                fall_count_v(channel) :=
                                    fall_count_v(channel) + 1;
                            end if;
                        end if;
                    end if;
                end loop;

                if shot_start_clear_bank_r(1) = '1' then
                    total_rise_v := (others => '0');
                    total_fall_v := (others => '0');
                end if;

                if window_accept_v = '1' then
                    rise_increment_r <= rise_events_v;
                    fall_increment_r <= fall_events_v;
                end if;

                if any_event_v = '1' and window_accept_v = '0' then
                    outside_window_pulse_r  <= '1';
                    outside_window_sticky_r <= '1';
                    outside_window_count_r  <=
                        outside_window_count_r + 1;
                end if;

                if shot_start_clear_bank_r(0) = '1' then
                    if window_active_r = '1' then
                        overlap_pulse_r  <= '1';
                        overlap_sticky_r <= '1';
                        overlap_count_r  <= overlap_count_r + 1;
                    end if;
                    window_active_r <= '1';
                    request_r       <= shot_start_stage_r.request;
                end if;
                rise_mask_r  <= rise_mask_v;
                fall_mask_r  <= fall_mask_v;
                rise_count_r <= rise_count_v;
                fall_count_r <= fall_count_v;
                total_rise_r <= total_rise_v;
                total_fall_r <= total_fall_v;

                snapshot_v := C_ECHO_SHOT_SNAPSHOT_CLEAR;
                snapshot_v.valid      := '1';
                snapshot_v.request    := request_r;
                snapshot_v.timeout    := shot_result_stage_r.timeout;
                snapshot_v.aborted    := shot_result_stage_r.aborted;
                snapshot_v.rise_mask  := rise_mask_v;
                snapshot_v.fall_mask  := fall_mask_v;
                snapshot_v.rise_count := rise_count_v;
                snapshot_v.fall_count := fall_count_v;
                snapshot_v.total_rise := (others => '0');
                snapshot_v.total_fall := (others => '0');
                if snapshot_capture_stage_r = '1' then
                    snapshot_pending_r  <= snapshot_v;
                end if;
                if shot_result_stage_r.valid = '1' then
                    snapshot_finalize_bank_r <= (others => '1');
                    window_active_r     <= '0';
                end if;

                if snapshot_finalize_bank_r(0) = '1' then
                    snapshot_r.valid <= '1';
                    snapshot_r.request <= snapshot_pending_r.request;
                    snapshot_r.timeout <= snapshot_pending_r.timeout;
                    snapshot_r.aborted <= snapshot_pending_r.aborted;
                end if;
                if snapshot_finalize_bank_r(1) = '1' then
                    snapshot_r.rise_mask <= snapshot_pending_r.rise_mask;
                    snapshot_r.fall_mask <= snapshot_pending_r.fall_mask;
                    snapshot_r.total_rise <= total_rise_r +
                        resize(rise_increment_r, total_rise_r'length);
                    snapshot_r.total_fall <= total_fall_r +
                        resize(fall_increment_r, total_fall_r'length);
                end if;
                for bank in 0 to 7 loop
                    if snapshot_finalize_bank_r(bank + 2) = '1' then
                        for channel in 4 * bank to 4 * bank + 3 loop
                            snapshot_r.rise_count(channel) <=
                                snapshot_pending_r.rise_count(channel);
                            snapshot_r.fall_count(channel) <=
                                snapshot_pending_r.fall_count(channel);
                        end loop;
                    end if;
                end loop;
            end if;
        end if;
    end process p_diagnostics;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and shot_result_stage_r.valid = '1' then
                assert snapshot_capture_stage_r = '1'
                    report "V2-ECHO-010 snapshot capture/result misalignment"
                    severity failure;
                assert window_active_r = '1' or
                    shot_result_stage_r.timeout = '1' or
                    shot_result_stage_r.aborted = '1'
                    report "V2-ECHO-008 normal result without Echo window"
                    severity failure;
                if window_active_r = '1' then
                    assert shot_result_stage_r.request = request_r
                        report "V2-ECHO-009 Echo Shot identity mismatch"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
