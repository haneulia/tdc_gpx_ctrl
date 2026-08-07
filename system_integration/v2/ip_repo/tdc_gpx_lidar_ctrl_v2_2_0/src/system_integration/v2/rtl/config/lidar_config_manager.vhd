library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;

-- CSR-domain owner of one atomic configuration transaction.
entity lidar_config_manager is
    generic (
        G_BUILD_CONFIG       : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
        G_CSR_CLK_MHZ        : positive := 100;
        G_PHASE_TIMEOUT_US   : positive := 1000
    );
    port (
        i_clk              : in  std_logic;
        i_rst_n            : in  std_logic;
        i_commit           : in  std_logic;
        i_shadow           : in  lidar_runtime_config_t;
        i_proc_prepare_ack : in  std_logic;
        i_proc_activate_ack : in std_logic;
        i_proc_release_ack : in  std_logic;
        i_proc_fault       : in  std_logic;
        i_tdc_prepare_ack  : in  std_logic;
        i_tdc_activate_ack : in  std_logic;
        i_tdc_release_ack  : in  std_logic;
        i_tdc_fault        : in  std_logic;
        o_prepare_req      : out std_logic;
        o_activate_req     : out std_logic;
        o_release_req      : out std_logic;
        o_candidate        : out lidar_active_config_t;
        o_busy             : out std_logic;
        o_done             : out std_logic;
        o_commit_rejected  : out std_logic;
        o_reject_error     : out lidar_cfg_error_t;
        o_error            : out lidar_cfg_error_t;
        o_recovery_required : out std_logic;
        o_active_valid     : out std_logic;
        o_active           : out lidar_active_config_t
    );
end entity lidar_config_manager;

architecture rtl of lidar_config_manager is

    constant C_PHASE_TIMEOUT_CLKS : positive :=
        G_CSR_CLK_MHZ * G_PHASE_TIMEOUT_US;

    type state_t is (
        S_IDLE,
        S_WAIT_CALCULATOR,
        S_STAGE_CANDIDATE,
        S_WAIT_PREPARE,
        S_WAIT_ACTIVATE,
        S_WAIT_RELEASE,
        S_CLEAR_RELEASE,
        S_CLEAR_ACTIVATE,
        S_CLEAR_PREPARE,
        S_ABORT_WAIT_CLEAR,
        S_LOCKED
    );

    signal state_r : state_t := S_IDLE;
    signal busy_r  : std_logic := '0';
    signal done_r  : std_logic := '0';
    signal rejected_r : std_logic := '0';
    signal error_r : lidar_cfg_error_t := CFG_OK;
    signal pending_error_r : lidar_cfg_error_t := CFG_OK;
    signal recovery_required_r : std_logic := '0';
    signal active_valid_r : std_logic := '0';
    signal prepare_req_r  : std_logic := '0';
    signal activate_req_r : std_logic := '0';
    signal release_req_r  : std_logic := '0';
    signal timeout_count_r : natural range 0 to
        C_PHASE_TIMEOUT_CLKS - 1 := 0;

    signal source_snapshot_r : lidar_runtime_config_t;
    signal candidate_r       : lidar_active_config_t;
    signal active_r          : lidar_active_config_t;

    signal calculator_start_r : std_logic := '0';
    signal calculator_done    : std_logic;
    signal calculator_error   : lidar_cfg_error_t;
    signal calculator_derived : lidar_derived_config_t;

begin

    assert fn_is_legal_clock_mhz(G_CSR_CLK_MHZ)
        report "V2-MGR-001 illegal CSR clock frequency"
        severity failure;

    o_prepare_req       <= prepare_req_r;
    o_activate_req      <= activate_req_r;
    o_release_req       <= release_req_r;
    o_candidate         <= candidate_r;
    o_busy              <= busy_r;
    o_done              <= done_r;
    o_commit_rejected   <= rejected_r;
    o_reject_error      <= CFG_TRANSACTION_BUSY;
    o_error             <= error_r;
    o_recovery_required <= recovery_required_r;
    o_active_valid      <= active_valid_r;
    o_active            <= active_r;

    u_calculator : entity work.lidar_commit_calculator
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk            => i_clk,
            i_rst_n          => i_rst_n,
            i_start          => calculator_start_r,
            i_source         => source_snapshot_r,
            o_busy           => open,
            o_done           => calculator_done,
            o_start_rejected => open,
            o_error          => calculator_error,
            o_derived        => calculator_derived
        );

    p_manager : process (i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            state_r             <= S_IDLE;
            busy_r              <= '0';
            done_r              <= '0';
            rejected_r          <= '0';
            error_r             <= CFG_OK;
            pending_error_r     <= CFG_OK;
            recovery_required_r <= '0';
            active_valid_r      <= '0';
            prepare_req_r       <= '0';
            activate_req_r      <= '0';
            release_req_r       <= '0';
            timeout_count_r     <= 0;
            calculator_start_r  <= '0';
        elsif rising_edge(i_clk) then
            done_r             <= '0';
            rejected_r         <= '0';
            calculator_start_r <= '0';

            if i_commit = '1' and state_r /= S_IDLE then
                rejected_r <= '1';
            end if;

            case state_r is
                when S_IDLE =>
                    if i_commit = '1' then
                        -- COMMIT 시점의 Shadow 전체를 한 번만 캡처한다.
                        -- 이후 소프트웨어가 CSR를 다시 써도 진행 중인
                        -- GPX/Processing 설정에는 섞이지 않는다.
                        source_snapshot_r <= i_shadow;
                        calculator_start_r <= '1';
                        busy_r  <= '1';
                        error_r <= CFG_OK;
                        state_r <= S_WAIT_CALCULATOR;
                    end if;

                when S_WAIT_CALCULATOR =>
                    if calculator_done = '1' then
                        if calculator_error /= CFG_OK then
                            error_r <= calculator_error;
                            busy_r  <= '0';
                            done_r  <= '1';
                            state_r <= S_IDLE;
                        else
                            if active_valid_r = '1' then
                                candidate_r.version <= active_r.version + 1;
                            else
                                candidate_r.version <= to_unsigned(1, 16);
                            end if;
                            candidate_r.source  <= source_snapshot_r;
                            candidate_r.derived <= calculator_derived;
                            state_r <= S_STAGE_CANDIDATE;
                        end if;
                    end if;

                when S_STAGE_CANDIDATE =>
                    -- Prepare 동안 scheduler_enable이 내려가고 기존 Shot이
                    -- Drain될 때까지 기다린다. RUN/ARM 상태 자체는 보존되며
                    -- 새 설정은 두 클럭 도메인이 함께 승인한 뒤만 활성화된다.
                    prepare_req_r   <= '1';
                    timeout_count_r <= 0;
                    state_r <= S_WAIT_PREPARE;

                when S_WAIT_PREPARE =>
                    if i_proc_fault = '1' or i_tdc_fault = '1' then
                        prepare_req_r   <= '0';
                        pending_error_r <= CFG_TRANSACTION_GATEWAY_PROTOCOL;
                        timeout_count_r <= 0;
                        state_r <= S_ABORT_WAIT_CLEAR;
                    elsif i_proc_prepare_ack = '1'
                          and i_tdc_prepare_ack = '1' then
                        activate_req_r <= '1';
                        timeout_count_r <= 0;
                        state_r <= S_WAIT_ACTIVATE;
                    elsif timeout_count_r = C_PHASE_TIMEOUT_CLKS - 1 then
                        prepare_req_r   <= '0';
                        pending_error_r <= CFG_TRANSACTION_PREPARE_TIMEOUT;
                        timeout_count_r <= 0;
                        state_r <= S_ABORT_WAIT_CLEAR;
                    else
                        timeout_count_r <= timeout_count_r + 1;
                    end if;

                when S_WAIT_ACTIVATE =>
                    if i_proc_fault = '1' or i_tdc_fault = '1' then
                        error_r <= CFG_TRANSACTION_GATEWAY_PROTOCOL;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    elsif i_proc_activate_ack = '1'
                          and i_tdc_activate_ack = '1' then
                        release_req_r <= '1';
                        timeout_count_r <= 0;
                        state_r <= S_WAIT_RELEASE;
                    elsif timeout_count_r = C_PHASE_TIMEOUT_CLKS - 1 then
                        error_r <= CFG_TRANSACTION_ACTIVATE_TIMEOUT;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    else
                        timeout_count_r <= timeout_count_r + 1;
                    end if;

                when S_WAIT_RELEASE =>
                    if i_proc_fault = '1' or i_tdc_fault = '1' then
                        error_r <= CFG_TRANSACTION_GATEWAY_PROTOCOL;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    elsif i_proc_release_ack = '1'
                          and i_tdc_release_ack = '1' then
                        active_r         <= candidate_r;
                        active_valid_r   <= '1';
                        release_req_r    <= '0';
                        timeout_count_r <= 0;
                        state_r <= S_CLEAR_RELEASE;
                    elsif timeout_count_r = C_PHASE_TIMEOUT_CLKS - 1 then
                        error_r <= CFG_TRANSACTION_RELEASE_TIMEOUT;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    else
                        timeout_count_r <= timeout_count_r + 1;
                    end if;

                when S_CLEAR_RELEASE =>
                    if i_proc_fault = '1' or i_tdc_fault = '1' then
                        error_r <= CFG_TRANSACTION_GATEWAY_PROTOCOL;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    elsif i_proc_release_ack = '0'
                          and i_tdc_release_ack = '0' then
                        activate_req_r  <= '0';
                        timeout_count_r <= 0;
                        state_r <= S_CLEAR_ACTIVATE;
                    elsif timeout_count_r = C_PHASE_TIMEOUT_CLKS - 1 then
                        error_r <= CFG_TRANSACTION_CLEAR_TIMEOUT;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    else
                        timeout_count_r <= timeout_count_r + 1;
                    end if;

                when S_CLEAR_ACTIVATE =>
                    if i_proc_fault = '1' or i_tdc_fault = '1' then
                        error_r <= CFG_TRANSACTION_GATEWAY_PROTOCOL;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    elsif i_proc_activate_ack = '0'
                          and i_tdc_activate_ack = '0' then
                        prepare_req_r   <= '0';
                        timeout_count_r <= 0;
                        state_r <= S_CLEAR_PREPARE;
                    elsif timeout_count_r = C_PHASE_TIMEOUT_CLKS - 1 then
                        error_r <= CFG_TRANSACTION_CLEAR_TIMEOUT;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    else
                        timeout_count_r <= timeout_count_r + 1;
                    end if;

                when S_CLEAR_PREPARE =>
                    if i_proc_fault = '1' or i_tdc_fault = '1' then
                        error_r <= CFG_TRANSACTION_GATEWAY_PROTOCOL;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    elsif i_proc_prepare_ack = '0'
                          and i_tdc_prepare_ack = '0' then
                        busy_r  <= '0';
                        done_r  <= '1';
                        error_r <= CFG_OK;
                        state_r <= S_IDLE;
                    elsif timeout_count_r = C_PHASE_TIMEOUT_CLKS - 1 then
                        error_r <= CFG_TRANSACTION_CLEAR_TIMEOUT;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    else
                        timeout_count_r <= timeout_count_r + 1;
                    end if;

                when S_ABORT_WAIT_CLEAR =>
                    activate_req_r <= '0';
                    release_req_r  <= '0';
                    if i_proc_prepare_ack = '0'
                       and i_proc_activate_ack = '0'
                       and i_proc_release_ack = '0'
                       and i_tdc_prepare_ack = '0'
                       and i_tdc_activate_ack = '0'
                       and i_tdc_release_ack = '0' then
                        error_r <= pending_error_r;
                        busy_r  <= '0';
                        done_r  <= '1';
                        state_r <= S_IDLE;
                    elsif timeout_count_r = C_PHASE_TIMEOUT_CLKS - 1 then
                        error_r <= CFG_TRANSACTION_CLEAR_TIMEOUT;
                        recovery_required_r <= '1';
                        busy_r <= '0';
                        done_r <= '1';
                        state_r <= S_LOCKED;
                    else
                        timeout_count_r <= timeout_count_r + 1;
                    end if;

                when S_LOCKED =>
                    if i_commit = '1' then
                        rejected_r <= '1';
                    end if;
            end case;
        end if;
    end process p_manager;

end architecture rtl;
