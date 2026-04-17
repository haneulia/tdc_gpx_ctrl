-- =============================================================================
-- tdc_gpx_chip_run.vhd
-- TDC-GPX Controller - Measurement Cycle Sub-FSM
-- =============================================================================
--
-- Purpose:
--   Handles the complete measurement cycle: armed → capture → drain → ALU.
--   Extracted from tdc_gpx_chip_ctrl to reduce FSM complexity.
--
-- FSM States (12):
--   ST_OFF → (start) → ST_ARMED → ST_CAPTURE → ST_DRAIN_LATCH →
--   ST_DRAIN_CHECK → ST_DRAIN_EF1/EF2/BURST/FLUSH → ST_DRAIN_SETTLE →
--   ST_ALU_PULSE → ST_ALU_RECOVERY → (done/armed)
--   ST_OVERRUN_FLUSH (overrun recovery path)
--
-- Features:
--   - EF1-first round-robin IFIFO drain
--   - LF-based burst read optimization
--   - Per-IFIFO early drain_done (ififo1_done intermediate beat)
--   - Shot overrun detection and purge recovery
--   - cmd_stop deferred handling (via stop_pending)
--   - drain_mode/n_drain_cap/bus_clk_div already snapshotted by coordinator
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_chip_run is
    generic (
        g_BUS_DATA_WIDTH : natural := c_TDC_BUS_WIDTH;
        g_RECOVERY_CLKS  : positive := 8;
        g_ALU_PULSE_CLKS : positive := 4
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- Control from coordinator
        i_start             : in  std_logic;        -- enter ARMED
        i_cmd_stop          : in  std_logic;
        o_done              : out std_logic;         -- 1-clk: returned to idle
        o_armed             : out std_logic;         -- '1' while in ST_ARMED

        -- Snapshot config (latched by coordinator at cmd_start)
        i_drain_mode        : in  std_logic;
        i_n_drain_cap       : in  unsigned(3 downto 0);
        i_cfg_image         : in  t_cfg_image;       -- for Fill (Reg6)

        -- Shot trigger
        i_shot_start        : in  std_logic;

        -- Expected IFIFO counts (from echo_receiver)
        i_expected_ififo1   : in  unsigned(7 downto 0);
        i_expected_ififo2   : in  unsigned(7 downto 0);

        -- Bus request (to coordinator mux)
        o_bus_req_valid     : out std_logic;
        o_bus_req_rw        : out std_logic;
        o_bus_req_addr      : out std_logic_vector(3 downto 0);
        o_bus_req_wdata     : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_bus_oen_permanent : out std_logic;
        o_bus_req_burst     : out std_logic;

        -- Bus response (from coordinator, gated)
        i_bus_rsp_valid     : in  std_logic;
        i_bus_rsp_rdata     : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        i_bus_busy          : in  std_logic;

        -- Sync status pins
        i_ef1_sync          : in  std_logic;
        i_ef2_sync          : in  std_logic;
        i_irflag_sync       : in  std_logic;
        i_lf1_sync          : in  std_logic;
        i_lf2_sync          : in  std_logic;

        -- Raw word output (to coordinator passthrough → decoder_i_mode)
        o_raw_word          : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_raw_valid         : out std_logic;
        o_ififo_id          : out std_logic;
        o_drain_done        : out std_logic;
        o_ififo1_done_beat  : out std_logic;

        -- Backpressure from coordinator hold register
        i_raw_busy          : in  std_logic;    -- '1' = hold register full, stall drain

        -- Pin outputs
        o_stopdis           : out std_logic;
        o_alutrigger        : out std_logic;
        o_busy              : out std_logic;

        -- Shot sequence
        o_shot_seq          : out unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0)
    );
end entity tdc_gpx_chip_run;

architecture rtl of tdc_gpx_chip_run is

    type t_run_state is (
        ST_OFF,
        ST_ARMED,
        ST_CAPTURE,
        ST_DRAIN_LATCH,
        ST_DRAIN_CHECK,
        ST_DRAIN_EF1,
        ST_DRAIN_EF2,
        ST_DRAIN_BURST,
        ST_DRAIN_FLUSH,
        ST_DRAIN_SETTLE,
        ST_ALU_PULSE,
        ST_ALU_RECOVERY,
        ST_OVERRUN_FLUSH
    );

    signal s_state_r : t_run_state := ST_OFF;

    constant c_RECOVERY_LAST  : unsigned(15 downto 0) := to_unsigned(g_RECOVERY_CLKS - 1, 16);
    constant c_ALU_PULSE_LAST : unsigned(15 downto 0) := to_unsigned(g_ALU_PULSE_CLKS - 1, 16);
    constant c_FLAG_SETTLE_LAST : unsigned(15 downto 0) := to_unsigned(2, 16);

    signal s_wait_cnt_r        : unsigned(15 downto 0) := (others => '0');
    signal s_req_valid_r       : std_logic := '0';
    signal s_req_rw_r          : std_logic := '0';
    signal s_req_addr_r        : std_logic_vector(3 downto 0) := (others => '0');
    signal s_req_wdata_r       : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_req_burst_r       : std_logic := '0';
    signal s_oen_permanent_r   : std_logic := '0';

    signal s_stopdis_r         : std_logic := '1';
    signal s_alutrigger_r      : std_logic := '0';
    signal s_busy_r            : std_logic := '0';
    signal s_done_r            : std_logic := '0';

    signal s_raw_word_r        : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_raw_valid_r       : std_logic := '0';
    signal s_ififo_id_r        : std_logic := '0';
    signal s_drain_done_r      : std_logic := '0';
    signal s_ififo1_done_beat_r : std_logic := '0';
    signal s_ififo1_done_sent_r : std_logic := '0';

    signal s_irflag_prev_r     : std_logic := '0';
    signal s_shot_seq_r        : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0) := (others => '0');

    signal s_expected_ififo1_r : unsigned(7 downto 0) := (others => '0');
    signal s_expected_ififo2_r : unsigned(7 downto 0) := (others => '0');
    signal s_drain_cnt_ififo1_r : unsigned(7 downto 0) := (others => '0');
    signal s_drain_cnt_ififo2_r : unsigned(7 downto 0) := (others => '0');

    signal s_fill_r            : unsigned(7 downto 0) := (others => '0');
    signal s_burst_cnt_r       : unsigned(7 downto 0) := (others => '0');
    signal s_burst_limit_r     : unsigned(7 downto 0) := (others => '0');

    signal s_range_active_r    : std_logic := '0';
    signal s_purge_mode_r      : std_logic := '0';
    signal s_stop_pending_r    : std_logic := '0';
    signal s_overrun_deferred_r : std_logic := '0';

begin

    p_irflag_edge : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_irflag_prev_r <= '0';
            else
                s_irflag_prev_r <= i_irflag_sync;
            end if;
        end if;
    end process;

    p_fsm : process(i_clk)
        variable v_cap             : unsigned(7 downto 0);
        variable v_ififo1_done     : boolean;
        variable v_ififo2_done     : boolean;
        variable v_ififo1_can_read : boolean;
        variable v_ififo2_can_read : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r           <= ST_OFF;
                s_wait_cnt_r        <= (others => '0');
                s_req_valid_r       <= '0';
                s_req_burst_r       <= '0';
                s_oen_permanent_r   <= '0';
                s_stopdis_r         <= '1';
                s_alutrigger_r      <= '0';
                s_busy_r            <= '0';
                s_done_r            <= '0';
                s_raw_word_r        <= (others => '0');
                s_raw_valid_r       <= '0';
                s_ififo_id_r        <= '0';
                s_drain_done_r      <= '0';
                s_ififo1_done_beat_r <= '0';
                s_ififo1_done_sent_r <= '0';
                s_shot_seq_r        <= (others => '0');
                s_expected_ififo1_r <= (others => '0');
                s_expected_ififo2_r <= (others => '0');
                s_drain_cnt_ififo1_r <= (others => '0');
                s_drain_cnt_ififo2_r <= (others => '0');
                s_range_active_r    <= '0';
                s_purge_mode_r      <= '0';
                s_stop_pending_r    <= '0';
                s_overrun_deferred_r <= '0';
            else
                s_raw_valid_r        <= '0';
                s_drain_done_r       <= '0';
                s_ififo1_done_beat_r <= '0';
                s_done_r             <= '0';

                -- Deferred stop latch
                if i_cmd_stop = '1' then
                    case s_state_r is
                        when ST_DRAIN_LATCH | ST_DRAIN_CHECK
                           | ST_DRAIN_EF1   | ST_DRAIN_EF2
                           | ST_DRAIN_BURST | ST_DRAIN_FLUSH
                           | ST_DRAIN_SETTLE
                           | ST_ALU_PULSE   | ST_ALU_RECOVERY =>
                            s_stop_pending_r <= '1';
                        when others =>
                            null;
                    end case;
                end if;

                case s_state_r is

                    when ST_OFF =>
                        s_busy_r <= '0';
                        if i_start = '1' then
                            s_stopdis_r          <= '0';
                            s_stop_pending_r     <= '0';
                            s_ififo1_done_sent_r <= '0';
                            s_state_r            <= ST_ARMED;
                        end if;

                    when ST_ARMED =>
                        if i_cmd_stop = '1' then
                            s_stopdis_r <= '1';
                            s_done_r    <= '1';
                            s_state_r   <= ST_OFF;
                        elsif i_shot_start = '1' then
                            s_busy_r             <= '1';
                            s_range_active_r     <= '1';
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_expected_ififo1_r  <= (others => '0');
                            s_expected_ififo2_r  <= (others => '0');
                            s_ififo1_done_sent_r <= '0';
                            s_state_r            <= ST_CAPTURE;
                        end if;

                    when ST_CAPTURE =>
                        if i_cmd_stop = '1' then
                            s_stopdis_r          <= '1';
                            s_stop_pending_r     <= '1';
                            s_range_active_r     <= '0';
                            s_raw_valid_r        <= '0';
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_expected_ififo1_r  <= (others => '0');
                            s_expected_ififo2_r  <= (others => '0');
                            s_drain_done_r       <= '0';
                            s_purge_mode_r       <= '1';
                            if i_drain_mode = '1' then
                                s_oen_permanent_r <= '1';
                            else
                                s_oen_permanent_r <= '0';
                            end if;
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_DRAIN_SETTLE;
                        elsif i_irflag_sync = '1' and s_irflag_prev_r = '0' then
                            s_fill_r <= unsigned(i_cfg_image(6)(
                                c_REG6_LF_THRESH_HI downto c_REG6_LF_THRESH_LO));
                            if i_drain_mode = '1' then
                                s_oen_permanent_r <= '1';
                            end if;
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_expected_ififo1_r  <= (others => '0');
                            s_expected_ififo2_r  <= (others => '0');
                            s_state_r <= ST_DRAIN_LATCH;
                        end if;

                    when ST_DRAIN_LATCH =>
                        s_expected_ififo1_r <= i_expected_ififo1;
                        s_expected_ififo2_r <= i_expected_ififo2;
                        s_state_r           <= ST_DRAIN_CHECK;

                    when ST_DRAIN_CHECK =>
                      if i_raw_busy = '0' then
                        s_wait_cnt_r <= (others => '0');  -- clear raw_busy watchdog
                        -- n_drain_cap × 4: each cap unit = 4 IFIFO words (burst quantum)
                        v_cap := shift_left(resize(i_n_drain_cap, 8), 2);  -- ×4

                        v_ififo1_done := (i_ef1_sync = '1')
                            or (s_purge_mode_r = '0' and i_n_drain_cap /= "0000"
                                and s_drain_cnt_ififo1_r >= v_cap);
                        v_ififo2_done := (i_ef2_sync = '1')
                            or (s_purge_mode_r = '0' and i_n_drain_cap /= "0000"
                                and s_drain_cnt_ififo2_r >= v_cap);

                        v_ififo1_can_read := not v_ififo1_done and (i_ef1_sync = '0');
                        v_ififo2_can_read := not v_ififo2_done and (i_ef2_sync = '0');

                        -- Early IFIFO1 done beat
                        if v_ififo1_done and not v_ififo2_done
                           and s_ififo1_done_sent_r = '0'
                           and s_purge_mode_r = '0' then
                            s_ififo1_done_beat_r <= '1';
                            s_ififo_id_r         <= '0';
                            s_ififo1_done_sent_r <= '1';
                        end if;

                        -- Completion
                        if v_ififo1_done and v_ififo2_done then
                            s_oen_permanent_r <= '0';
                            s_range_active_r  <= '0';
                            s_wait_cnt_r      <= (others => '0');
                            if s_purge_mode_r = '1' then
                                s_purge_mode_r <= '0';
                            end if;
                            -- Always emit final drain_done (normal + purge)
                            s_drain_done_r <= '1';
                            s_ififo_id_r   <= '1';
                            s_state_r      <= ST_ALU_PULSE;

                        -- Burst IFIFO1
                        elsif s_purge_mode_r = '0'
                              and i_drain_mode = '1'
                              and s_fill_r >= 2
                              and i_ef1_sync = '0' and i_lf1_sync = '1'
                              and v_ififo1_can_read
                              and s_expected_ififo1_r >= s_drain_cnt_ififo1_r + 2 then
                            if s_fill_r <= s_expected_ififo1_r - s_drain_cnt_ififo1_r then
                                s_burst_limit_r <= s_fill_r - 1;
                            else
                                s_burst_limit_r <= s_expected_ififo1_r - s_drain_cnt_ififo1_r - 1;
                            end if;
                            s_burst_cnt_r <= (others => '0');
                            s_req_valid_r <= '1';
                            s_req_burst_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= c_TDC_REG8_IFIFO1;
                            s_ififo_id_r  <= '0';
                            s_state_r     <= ST_DRAIN_BURST;

                        -- Burst IFIFO2
                        elsif s_purge_mode_r = '0'
                              and i_drain_mode = '1'
                              and s_fill_r >= 2
                              and i_ef2_sync = '0' and i_lf2_sync = '1'
                              and v_ififo2_can_read
                              and s_expected_ififo2_r >= s_drain_cnt_ififo2_r + 2 then
                            if s_fill_r <= s_expected_ififo2_r - s_drain_cnt_ififo2_r then
                                s_burst_limit_r <= s_fill_r - 1;
                            else
                                s_burst_limit_r <= s_expected_ififo2_r - s_drain_cnt_ififo2_r - 1;
                            end if;
                            s_burst_cnt_r <= (others => '0');
                            s_req_valid_r <= '1';
                            s_req_burst_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= c_TDC_REG9_IFIFO2;
                            s_ififo_id_r  <= '1';
                            s_state_r     <= ST_DRAIN_BURST;

                        -- EF single IFIFO1
                        elsif v_ififo1_can_read then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= c_TDC_REG8_IFIFO1;
                            s_ififo_id_r  <= '0';
                            s_state_r     <= ST_DRAIN_EF1;

                        -- EF single IFIFO2
                        elsif v_ififo2_can_read then
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '0';
                            s_req_addr_r  <= c_TDC_REG9_IFIFO2;
                            s_ififo_id_r  <= '1';
                            s_state_r     <= ST_DRAIN_EF2;

                        else
                            -- Fallback completion
                            s_oen_permanent_r <= '0';
                            s_range_active_r  <= '0';
                            s_wait_cnt_r      <= (others => '0');
                            if s_purge_mode_r = '1' then
                                s_purge_mode_r <= '0';
                            end if;
                            -- Always emit final drain_done (normal + purge)
                            s_drain_done_r <= '1';
                            s_ififo_id_r   <= '1';
                            s_state_r      <= ST_ALU_PULSE;
                        end if;
                      else
                        -- raw_busy watchdog: abort drain if stalled too long
                        s_wait_cnt_r <= s_wait_cnt_r + 1;
                        if s_wait_cnt_r = x"FFFF" then
                            s_oen_permanent_r <= '0';  -- cleanup
                            s_drain_done_r <= '1';
                            s_ififo_id_r   <= '1';
                            s_state_r      <= ST_ALU_PULSE;
                        end if;
                      end if; -- i_raw_busy = '0'

                    when ST_DRAIN_EF1 =>
                        if i_bus_rsp_valid = '1' then
                            s_req_valid_r        <= '0';
                            s_raw_word_r         <= i_bus_rsp_rdata;
                            if s_purge_mode_r = '0' then
                                s_raw_valid_r    <= '1';
                            end if;
                            s_drain_cnt_ififo1_r <= s_drain_cnt_ififo1_r + 1;
                            s_wait_cnt_r         <= (others => '0');
                            s_state_r            <= ST_DRAIN_SETTLE;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            if s_wait_cnt_r = x"FFFF" then
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_state_r         <= ST_ALU_PULSE;
                            end if;
                        end if;

                    when ST_DRAIN_EF2 =>
                        if i_bus_rsp_valid = '1' then
                            s_req_valid_r        <= '0';
                            s_raw_word_r         <= i_bus_rsp_rdata;
                            if s_purge_mode_r = '0' then
                                s_raw_valid_r    <= '1';
                            end if;
                            s_drain_cnt_ififo2_r <= s_drain_cnt_ififo2_r + 1;
                            s_wait_cnt_r         <= (others => '0');
                            s_state_r            <= ST_DRAIN_SETTLE;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            if s_wait_cnt_r = x"FFFF" then
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_state_r         <= ST_ALU_PULSE;
                            end if;
                        end if;

                    when ST_DRAIN_BURST =>
                        if i_bus_rsp_valid = '1' then
                            s_raw_word_r  <= i_bus_rsp_rdata;
                            s_raw_valid_r <= '1';
                            s_burst_cnt_r <= s_burst_cnt_r + 1;
                            s_wait_cnt_r  <= (others => '0');
                            if s_ififo_id_r = '0' then
                                s_drain_cnt_ififo1_r <= s_drain_cnt_ififo1_r + 1;
                            else
                                s_drain_cnt_ififo2_r <= s_drain_cnt_ififo2_r + 1;
                            end if;
                            if (s_burst_cnt_r + 1) >= s_burst_limit_r then
                                s_req_burst_r <= '0';
                                s_req_valid_r <= '0';
                                s_state_r     <= ST_DRAIN_FLUSH;
                            end if;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                            if s_wait_cnt_r = x"FFFF" then
                                s_req_burst_r     <= '0';
                                s_req_valid_r     <= '0';
                                s_oen_permanent_r <= '0';
                                s_drain_done_r    <= '1';
                                s_ififo_id_r      <= '1';
                                s_state_r         <= ST_ALU_PULSE;
                            end if;
                        end if;

                    when ST_DRAIN_FLUSH =>
                        if i_bus_rsp_valid = '1' then
                            s_raw_word_r  <= i_bus_rsp_rdata;
                            s_raw_valid_r <= '1';
                            s_wait_cnt_r  <= (others => '0');
                            if s_ififo_id_r = '0' then
                                s_drain_cnt_ififo1_r <= s_drain_cnt_ififo1_r + 1;
                            else
                                s_drain_cnt_ififo2_r <= s_drain_cnt_ififo2_r + 1;
                            end if;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;
                        if i_bus_busy = '0' and i_bus_rsp_valid = '0' then
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_DRAIN_SETTLE;
                        elsif s_wait_cnt_r = x"FFFF" then
                            -- Bus hung during flush: force completion
                            s_oen_permanent_r <= '0';
                            s_drain_done_r    <= '1';
                            s_ififo_id_r      <= '1';
                            s_state_r         <= ST_ALU_PULSE;
                        end if;

                    when ST_DRAIN_SETTLE =>
                        if s_wait_cnt_r = c_FLAG_SETTLE_LAST then
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_DRAIN_CHECK;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_ALU_PULSE =>
                        s_alutrigger_r <= '1';
                        if s_wait_cnt_r = c_ALU_PULSE_LAST then
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_ALU_RECOVERY;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_ALU_RECOVERY =>
                        s_alutrigger_r <= '0';
                        if s_wait_cnt_r = c_RECOVERY_LAST then
                            s_wait_cnt_r <= (others => '0');
                            if s_overrun_deferred_r = '1' then
                                s_overrun_deferred_r <= '0';
                                s_raw_valid_r        <= '0';
                                s_range_active_r     <= '1';
                                s_drain_cnt_ififo1_r <= (others => '0');
                                s_drain_cnt_ififo2_r <= (others => '0');
                                s_expected_ififo1_r  <= (others => '0');
                                s_expected_ififo2_r  <= (others => '0');
                                s_drain_done_r       <= '0';
                                s_purge_mode_r       <= '1';
                                if i_drain_mode = '1' then
                                    s_oen_permanent_r <= '1';
                                else
                                    s_oen_permanent_r <= '0';
                                end if;
                                s_state_r <= ST_DRAIN_SETTLE;
                            elsif s_stop_pending_r = '1' then
                                s_stop_pending_r <= '0';
                                s_stopdis_r      <= '1';
                                s_busy_r         <= '0';
                                s_done_r         <= '1';
                                s_state_r        <= ST_OFF;
                            else
                                s_shot_seq_r <= s_shot_seq_r + 1;
                                s_busy_r     <= '0';
                                s_ififo1_done_sent_r <= '0';
                                s_state_r    <= ST_ARMED;
                            end if;
                        else
                            s_wait_cnt_r <= s_wait_cnt_r + 1;
                        end if;

                    when ST_OVERRUN_FLUSH =>
                        s_raw_valid_r <= '0';
                        if i_bus_busy = '0' and i_bus_rsp_valid = '0' then
                            s_purge_mode_r       <= '1';
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_expected_ififo1_r  <= (others => '0');
                            s_expected_ififo2_r  <= (others => '0');
                            if i_drain_mode = '1' then
                                s_oen_permanent_r <= '1';
                            end if;
                            s_wait_cnt_r <= (others => '0');
                            s_state_r    <= ST_DRAIN_SETTLE;
                        end if;

                end case;

                -- Shot overrun handler
                if i_shot_start = '1' and s_state_r /= ST_OFF
                   and s_state_r /= ST_ARMED then
                    case s_state_r is
                        when ST_CAPTURE | ST_DRAIN_LATCH
                           | ST_DRAIN_CHECK | ST_DRAIN_SETTLE =>
                            s_req_valid_r        <= '0';
                            s_req_burst_r        <= '0';
                            s_raw_valid_r        <= '0';
                            s_range_active_r     <= '1';
                            s_drain_cnt_ififo1_r <= (others => '0');
                            s_drain_cnt_ififo2_r <= (others => '0');
                            s_overrun_deferred_r <= '0';
                            s_state_r            <= ST_OVERRUN_FLUSH;
                        when ST_DRAIN_EF1 | ST_DRAIN_EF2
                           | ST_DRAIN_BURST | ST_DRAIN_FLUSH =>
                            s_raw_valid_r        <= '0';
                            s_req_burst_r        <= '0';
                            s_req_valid_r        <= '0';
                            s_oen_permanent_r    <= '0';
                            s_overrun_deferred_r <= '0';
                            s_state_r            <= ST_OVERRUN_FLUSH;
                        when ST_ALU_PULSE | ST_ALU_RECOVERY =>
                            s_overrun_deferred_r <= '1';
                        when others =>
                            null;
                    end case;
                end if;

            end if;
        end if;
    end process p_fsm;

    o_bus_req_valid     <= s_req_valid_r;
    o_bus_req_rw        <= s_req_rw_r;
    o_bus_req_addr      <= s_req_addr_r;
    o_bus_req_wdata     <= s_req_wdata_r;
    o_bus_oen_permanent <= s_oen_permanent_r;
    o_bus_req_burst     <= s_req_burst_r;
    o_raw_word          <= s_raw_word_r;
    o_raw_valid         <= s_raw_valid_r;
    o_ififo_id          <= s_ififo_id_r;
    o_drain_done        <= s_drain_done_r;
    o_ififo1_done_beat  <= s_ififo1_done_beat_r;
    o_stopdis           <= s_stopdis_r;
    o_alutrigger        <= s_alutrigger_r;
    o_busy              <= s_busy_r;
    o_shot_seq          <= s_shot_seq_r;
    o_done              <= s_done_r;
    o_armed             <= '1' when s_state_r = ST_ARMED else '0';

end architecture rtl;
