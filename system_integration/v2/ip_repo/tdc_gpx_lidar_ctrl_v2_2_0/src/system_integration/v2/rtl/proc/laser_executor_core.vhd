library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- Sequential owner of one accepted-shot lifecycle. Physical CDC and pulse
-- stretching are explicit boundaries outside this core, so this FSM contains
-- only request resolution, timeout, simulation delay, range and re-arm state.
-- 실제 수신 기준 시점(T0)은 물리 모드의 fire_done 동기 이벤트 또는
-- 시뮬레이션 모드의 동등한 시작 이벤트이다. start_tdc/Shot timestamp와
-- 목표 왕복시간 카운트는 모두 이 실제 수신 기준 시점(T0)에서 시작한다.
entity laser_executor_core is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                    : in  std_logic;
        i_rst_n                  : in  std_logic;
        i_config_ready           : in  std_logic;
        i_active_version         : in  u16_t;
        i_simulation_mode        : in  std_logic;
        i_fire_done_timeout_clks : in  u32_t;
        i_target_range_clks      : in  u32_t;
        i_sim_start_delay_clks   : in  u32_t;
        i_physical_fire_enable   : in  std_logic;
        i_simulation_enable      : in  std_logic;
        i_shot_request           : in  shot_request_t;
        i_request_context_valid  : in  std_logic;
        i_bridge_ready           : in  std_logic;
        i_t0_event               : in  std_logic;
        i_timestamp_ticks        : in  t0_timestamp_t;
        i_physical_t0_timestamp_ticks : in t0_timestamp_t;
        i_physical_t0_timestamp_valid : in std_logic;
        i_physical_start_busy    : in  std_logic;
        i_fire_busy              : in  std_logic;
        i_sim_start_busy         : in  std_logic;
        i_stop_busy              : in  std_logic;

        o_executor_ready         : out std_logic;
        o_request_accept         : out std_logic;
        o_request_drop           : out std_logic;
        o_fire_trigger           : out std_logic;
        o_sim_start_trigger      : out std_logic;
        o_stop_trigger           : out std_logic;
        o_physical_arm           : out std_logic;
        o_shot_start             : out shot_start_event_t;
        o_shot_result            : out shot_result_t;
        o_current_request        : out shot_request_t;
        o_busy                   : out std_logic;
        o_rearm_active           : out std_logic;
        o_fire_done_sync_clks    : out unsigned(15 downto 0);
        o_rearm_margin_clks      : out unsigned(15 downto 0)
    );
end entity laser_executor_core;

architecture rtl of laser_executor_core is

    type executor_state_t is (
        EXEC_IDLE,
        EXEC_WAIT_PHYSICAL_T0,
        EXEC_WAIT_SIMULATION_T0,
        EXEC_TIMEOUT_RESOLVE,
        EXEC_ABORT_RESOLVE,
        EXEC_RANGE_WINDOW,
        EXEC_REARM
    );

    constant C_FIRE_DONE_SYNC_CLKS : positive := 3;
    constant C_REARM_MARGIN_CLKS   : positive := 2;
    constant C_REARM_POST_BUSY_CLKS : positive :=
        C_REARM_MARGIN_CLKS - 1;

    function fn_sat_increment(value : unsigned) return unsigned is
    begin
        if value = (value'range => '1') then
            return value;
        end if;
        return value + 1;
    end function fn_sat_increment;

    signal state_r             : executor_state_t := EXEC_IDLE;
    signal current_request_r   : shot_request_t := C_SHOT_REQUEST_IDLE;
    signal request_accept_r    : std_logic := '0';
    signal request_drop_r      : std_logic := '0';
    signal shot_start_r        : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_result_r       : shot_result_t := C_SHOT_RESULT_IDLE;
    signal physical_arm_r      : std_logic := '0';

    signal fire_timeout_r      : unsigned(31 downto 0) := (others => '0');
    signal target_range_r      : unsigned(31 downto 0) := (others => '0');
    signal simulation_delay_r  : unsigned(31 downto 0) := (others => '0');
    signal simulation_delay_last_r : std_logic := '0';
    signal range_count_r       : unsigned(31 downto 0) := (others => '0');
    signal fire_to_t0_count_r  : unsigned(31 downto 0) := (others => '0');
    signal fire_to_t0_value_r  : unsigned(31 downto 0) := (others => '0');
    signal resolve_count_r     : natural range 0 to C_FIRE_DONE_SYNC_CLKS := 0;
    signal rearm_count_r       : natural range 0 to C_REARM_MARGIN_CLKS := 0;
    signal mode_ready_c            : std_logic;
    signal executor_ready_c        : std_logic;
    signal request_context_valid_c : std_logic;
    signal request_accept_c        : std_logic;
    signal fire_trigger_c          : std_logic;
    signal simulation_t0_c         : std_logic;
    signal range_end_c             : std_logic;
    signal physical_start_busy_r   : std_logic := '0';
    signal fire_busy_r             : std_logic := '0';
    signal simulation_start_busy_r : std_logic := '0';
    signal stop_busy_r             : std_logic := '0';

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-LASER-001 invalid build configuration"
        severity failure;

    mode_ready_c <= '1' when
        (i_simulation_mode = '1' and
         i_simulation_enable = '1') or
        (i_simulation_mode = '0' and
         i_physical_fire_enable = '1' and
         i_bridge_ready = '1') else '0';
    executor_ready_c <= '1' when state_r = EXEC_IDLE and
        i_config_ready = '1' and mode_ready_c = '1' else '0';
    -- The wrapper registers this qualification with the request. Keeping the
    -- 16-bit version comparison outside the lifecycle core prevents it from
    -- becoming the enable cone of every Shot-start/timestamp register.
    request_context_valid_c <= i_shot_request.valid and
        i_request_context_valid;
    request_accept_c <= executor_ready_c and request_context_valid_c;
    fire_trigger_c <= request_accept_c and not i_shot_request.source_sim;
    simulation_t0_c <= '1' when
        (request_accept_c = '1' and i_shot_request.source_sim = '1' and
         i_sim_start_delay_clks = 0) or
        (state_r = EXEC_WAIT_SIMULATION_T0 and
         i_simulation_enable = '1' and
         simulation_delay_last_r = '1') else '0';
    range_end_c <= '1' when state_r = EXEC_RANGE_WINDOW and
        range_count_r <= 1 else '0';

    o_executor_ready      <= executor_ready_c;
    o_request_accept      <= request_accept_r;
    o_request_drop        <= request_drop_r;
    o_fire_trigger        <= fire_trigger_c;
    o_sim_start_trigger   <= simulation_t0_c;
    o_stop_trigger        <= range_end_c;
    o_physical_arm        <= physical_arm_r;
    o_shot_start          <= shot_start_r;
    o_shot_result         <= shot_result_r;
    o_current_request     <= current_request_r;
    o_busy                <= '0' when state_r = EXEC_IDLE else '1';
    o_rearm_active        <= '1' when state_r = EXEC_REARM else '0';
    o_fire_done_sync_clks <= to_unsigned(
        C_FIRE_DONE_SYNC_CLKS, o_fire_done_sync_clks'length);
    o_rearm_margin_clks   <= to_unsigned(
        C_REARM_MARGIN_CLKS, o_rearm_margin_clks'length);

    -- Busy status is observation feedback from separately registered pulse
    -- owners. Pipeline it once before the lifecycle FSM so pulse-width counter
    -- logic cannot become part of the 200 MHz state/control cone. The added
    -- cycle only extends the conservative re-arm interval.
    p_busy_pipeline : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                physical_start_busy_r   <= '0';
                fire_busy_r             <= '0';
                simulation_start_busy_r <= '0';
                stop_busy_r             <= '0';
            else
                physical_start_busy_r   <= i_physical_start_busy;
                fire_busy_r             <= i_fire_busy;
                simulation_start_busy_r <= i_sim_start_busy;
                stop_busy_r             <= i_stop_busy;
            end if;
        end if;
    end process p_busy_pipeline;

    p_executor : process (i_clk)
        variable wait_value_v : unsigned(31 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r             <= EXEC_IDLE;
                current_request_r   <= C_SHOT_REQUEST_IDLE;
                request_accept_r    <= '0';
                request_drop_r      <= '0';
                shot_start_r        <= C_SHOT_START_EVENT_IDLE;
                shot_result_r       <= C_SHOT_RESULT_IDLE;
                physical_arm_r      <= '0';
                fire_timeout_r      <= (others => '0');
                target_range_r      <= (others => '0');
                simulation_delay_r  <= (others => '0');
                simulation_delay_last_r <= '0';
                range_count_r       <= (others => '0');
                fire_to_t0_count_r  <= (others => '0');
                fire_to_t0_value_r  <= (others => '0');
                resolve_count_r     <= 0;
                rearm_count_r       <= 0;
            else
                request_accept_r   <= '0';
                request_drop_r     <= '0';
                shot_start_r.valid <= '0';
                shot_result_r.valid <= '0';

                -- A producer is expected to observe ready, but every presented
                -- one-cycle request is still resolved defensively. A request
                -- arriving while this single-entry executor is busy is dropped
                -- explicitly rather than leaving B2 ownership unresolved.
                if i_shot_request.valid = '1' and state_r /= EXEC_IDLE then
                    request_drop_r <= '1';
                end if;

                case state_r is
                    when EXEC_IDLE =>
                        physical_arm_r <= '0';
                        if i_shot_request.valid = '1' and
                           request_accept_c /= '1' then
                            request_drop_r <= '1';
                        elsif request_accept_c = '1' then
                            current_request_r  <= i_shot_request;
                            request_accept_r   <= '1';
                            fire_timeout_r     <= i_fire_done_timeout_clks;
                            target_range_r     <= i_target_range_clks;
                            simulation_delay_r <= i_sim_start_delay_clks;
                            if i_sim_start_delay_clks <= 1 then
                                simulation_delay_last_r <= '1';
                            else
                                simulation_delay_last_r <= '0';
                            end if;
                            fire_to_t0_count_r <= (others => '0');
                            fire_to_t0_value_r <= (others => '0');
                            resolve_count_r    <= 0;
                            rearm_count_r      <= C_REARM_MARGIN_CLKS;

                            if i_shot_request.source_sim = '0' then
                                physical_arm_r <= '1';
                                state_r <= EXEC_WAIT_PHYSICAL_T0;
                            elsif i_sim_start_delay_clks = 0 then
                                simulation_delay_last_r <= '0';
                                shot_start_r.valid <= '1';
                                shot_start_r.request <= i_shot_request;
                                shot_start_r.fire_to_t0_clks <= (others => '0');
                                shot_start_r.t0_timestamp_ticks <=
                                    i_timestamp_ticks;
                                shot_start_r.t0_timestamp_valid <= '1';
                                shot_start_r.t0_time_sync_valid <= '0';
                                range_count_r <= i_target_range_clks;
                                state_r <= EXEC_RANGE_WINDOW;
                            else
                                state_r <= EXEC_WAIT_SIMULATION_T0;
                            end if;
                        end if;

                    when EXEC_WAIT_PHYSICAL_T0 =>
                        wait_value_v := fn_sat_increment(fire_to_t0_count_r);
                        fire_to_t0_count_r <= wait_value_v;
                        if i_t0_event = '1' then
                            physical_arm_r <= '0';
                            fire_to_t0_value_r <= wait_value_v;
                            shot_start_r.valid <= '1';
                            shot_start_r.request <= current_request_r;
                            shot_start_r.fire_to_t0_clks <= wait_value_v;
                            shot_start_r.t0_timestamp_ticks <=
                                i_physical_t0_timestamp_ticks;
                            shot_start_r.t0_timestamp_valid <=
                                i_physical_t0_timestamp_valid;
                            shot_start_r.t0_time_sync_valid <= '0';
                            range_count_r <= target_range_r;
                            state_r <= EXEC_RANGE_WINDOW;
                        elsif i_physical_fire_enable /= '1' then
                            physical_arm_r  <= '0';
                            resolve_count_r <= C_FIRE_DONE_SYNC_CLKS;
                            state_r <= EXEC_ABORT_RESOLVE;
                        elsif fire_timeout_r > 1 then
                            fire_timeout_r <= fire_timeout_r - 1;
                        else
                            physical_arm_r  <= '0';
                            resolve_count_r <= C_FIRE_DONE_SYNC_CLKS;
                            state_r <= EXEC_TIMEOUT_RESOLVE;
                        end if;

                    when EXEC_WAIT_SIMULATION_T0 =>
                        wait_value_v := fn_sat_increment(fire_to_t0_count_r);
                        fire_to_t0_count_r <= wait_value_v;
                        if i_simulation_enable /= '1' then
                            simulation_delay_last_r <= '0';
                            shot_result_r.valid   <= '1';
                            shot_result_r.timeout <= '0';
                            shot_result_r.aborted <= '1';
                            shot_result_r.request <= current_request_r;
                            shot_result_r.fire_to_t0_clks <= wait_value_v;
                            rearm_count_r <= C_REARM_MARGIN_CLKS;
                            state_r <= EXEC_REARM;
                        elsif simulation_delay_last_r = '0' then
                            simulation_delay_r <= simulation_delay_r - 1;
                            if simulation_delay_r = 2 then
                                simulation_delay_last_r <= '1';
                            end if;
                        else
                            simulation_delay_r <= (others => '0');
                            simulation_delay_last_r <= '0';
                            fire_to_t0_value_r <= wait_value_v;
                            shot_start_r.valid <= '1';
                            shot_start_r.request <= current_request_r;
                            shot_start_r.fire_to_t0_clks <= wait_value_v;
                            shot_start_r.t0_timestamp_ticks <=
                                i_timestamp_ticks;
                            shot_start_r.t0_timestamp_valid <= '1';
                            shot_start_r.t0_time_sync_valid <= '0';
                            range_count_r <= target_range_r;
                            state_r <= EXEC_RANGE_WINDOW;
                        end if;

                    when EXEC_TIMEOUT_RESOLVE | EXEC_ABORT_RESOLVE =>
                        if i_t0_event = '1' then
                            fire_to_t0_value_r <= fire_to_t0_count_r;
                            shot_start_r.valid <= '1';
                            shot_start_r.request <= current_request_r;
                            shot_start_r.fire_to_t0_clks <= fire_to_t0_count_r;
                            shot_start_r.t0_timestamp_ticks <=
                                i_physical_t0_timestamp_ticks;
                            shot_start_r.t0_timestamp_valid <=
                                i_physical_t0_timestamp_valid;
                            shot_start_r.t0_time_sync_valid <= '0';
                            range_count_r <= target_range_r;
                            state_r <= EXEC_RANGE_WINDOW;
                        elsif resolve_count_r > 1 then
                            resolve_count_r <= resolve_count_r - 1;
                        else
                            shot_result_r.valid   <= '1';
                            shot_result_r.timeout <= '0';
                            shot_result_r.aborted <= '0';
                            if state_r = EXEC_TIMEOUT_RESOLVE then
                                shot_result_r.timeout <= '1';
                            else
                                shot_result_r.aborted <= '1';
                            end if;
                            shot_result_r.request <= current_request_r;
                            shot_result_r.fire_to_t0_clks <= fire_to_t0_count_r;
                            rearm_count_r <= C_REARM_MARGIN_CLKS;
                            state_r <= EXEC_REARM;
                        end if;

                    when EXEC_RANGE_WINDOW =>
                        -- 이 값은 CTL12를 GPX Reg7.MTimer의 25 ns 단위로
                        -- 올림한 동일 시간이다. 따라서 stop_tdc가 물리 GPX
                        -- TimerFlag보다 먼저 발생하지 않는다.
                        if range_count_r > 1 then
                            range_count_r <= range_count_r - 1;
                        else
                            range_count_r <= (others => '0');
                            shot_result_r.valid   <= '1';
                            shot_result_r.timeout <= '0';
                            shot_result_r.aborted <= '0';
                            shot_result_r.request <= current_request_r;
                            shot_result_r.fire_to_t0_clks <= fire_to_t0_value_r;
                            rearm_count_r <= C_REARM_MARGIN_CLKS;
                            state_r <= EXEC_REARM;
                        end if;

                    when EXEC_REARM =>
                        if fire_busy_r = '1' or
                           simulation_start_busy_r = '1' or
                           stop_busy_r = '1' or
                           physical_start_busy_r = '1' then
                            -- One clock of the published two-clock margin is
                            -- already supplied by p_busy_pipeline.
                            rearm_count_r <= C_REARM_POST_BUSY_CLKS;
                        elsif rearm_count_r > 1 then
                            rearm_count_r <= rearm_count_r - 1;
                        else
                            rearm_count_r <= 0;
                            state_r <= EXEC_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_executor;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                assert not (request_accept_r = '1' and request_drop_r = '1')
                    report "V2-LASER-002 accept and drop overlap"
                    severity failure;
                assert not (i_physical_fire_enable = '1' and
                            i_simulation_enable = '1')
                    report "V2-LASER-003 physical and simulation enable overlap"
                    severity failure;
                if fire_trigger_c = '1' then
                    assert i_physical_fire_enable = '1' and
                        i_shot_request.source_sim = '0'
                        report "V2-LASER-005 physical fire without permission"
                        severity failure;
                end if;
                if state_r /= EXEC_IDLE then
                    assert current_request_r.active_version =
                        i_active_version
                        report "V2-LASER-007 active version changed while busy"
                        severity failure;
                end if;
                if shot_result_r.valid = '1' then
                    assert not (shot_result_r.timeout = '1' and
                                shot_result_r.aborted = '1')
                        report "V2-LASER-008 timeout and abort overlap"
                        severity failure;
                end if;
                if shot_start_r.valid = '1' then
                    assert shot_start_r.t0_timestamp_valid = '1'
                        report "V2-LASER-010 Shot START without T0 timestamp"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
