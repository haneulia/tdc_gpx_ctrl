-- =============================================================================
-- tb_tdc_gpx_cell_pipe_lane_mask.vhd
-- Testbench for tdc_gpx_cell_pipe -- CHAIN P1 slope lane-mask gating
-- =============================================================================
--
-- Purpose:
--   Proves the DEDICATED_2X2 wrong-slope builder fix:
--
--   Scenario L (legacy, masks "1111"):
--     Documents the pre-fix mechanism. drain_done control beats broadcast to
--     both slopes make EVERY (chip, slope) builder emit a slice per shot --
--     including all-blank slices on the slope a chip does not serve. With
--     all-ones masks the new gating code takes the identical branch as the
--     pre-fix RTL, so this scenario is also the legacy-compatibility check.
--
--   Scenario D (dedicated, rise "0011" / fall "1100", 3 shots):
--     Wrong-slope lanes must stay completely silent (0 beats), right-slope
--     lanes deliver one slice per shot, and no builder starves (no
--     shot_dropped pulses, sticky masks clean).
--
--   Scenario M (masked-slope hit drop):
--     A hit beat addressed to a masked slope is consumed (stream does not
--     stall) and flagged in o_masked_slope_drop_*; the chip's own lane
--     still completes its slice.
--
--   Scenario C (sticky lifecycle):
--     The masked-slope evidence survives abort/cmd_stop, clears only on the
--     explicit diagnostic soft-clear, and stays clear through a nominal run.
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_cell_pipe_lane_mask is
end entity;

architecture sim of tb_tdc_gpx_cell_pipe_lane_mask is

    constant CLK_PERIOD   : time    := 5 ns;   -- 200 MHz
    constant OUTPUT_WIDTH : natural := 32;
    constant C_STOPS      : natural := 8;
    constant C_MAX_HITS   : natural := 1;
    -- Slice length: stops x beats/cell (runtime max_hits=1 @32b -> 2)
    constant C_SLICE_BEATS : natural :=
        C_STOPS * fn_beats_per_cell_rt(C_MAX_HITS, OUTPUT_WIDTH);

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal done  : boolean   := false;

    -- DUT inputs
    signal evt_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');
    signal evt_tdata  : t_evt_axis_tdata_array := (others => (others => '0'));
    signal evt_tuser  : t_evt_axis_tuser_array := (others => (others => '0'));
    signal evt_tready : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal shot_start : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');
    signal rise_mask  : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');
    signal fall_mask  : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');
    signal abort      : std_logic := '0';
    signal sticky_clear : std_logic := '0';
    signal face_stops : unsigned(3 downto 0) := to_unsigned(C_STOPS, 4);
    signal max_hits   : unsigned(2 downto 0) := to_unsigned(C_MAX_HITS, 3);

    -- DUT outputs
    signal cell_rise_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_rise_tlast  : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_rise_tready : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');
    signal cell_fall_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_fall_tlast  : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_fall_tready : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');
    signal shot_dropped      : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal shot_fall_dropped : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal masked_drop_rise  : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal masked_drop_fall  : std_logic_vector(c_N_CHIPS-1 downto 0);

    -- Monitor counters (per lane)
    type t_nat_arr is array (0 to c_N_CHIPS-1) of natural;
    signal beat_rise  : t_nat_arr := (others => 0);
    signal beat_fall  : t_nat_arr := (others => 0);
    signal tlast_rise : t_nat_arr := (others => 0);
    signal tlast_fall : t_nat_arr := (others => 0);
    signal saw_shot_drop : boolean := false;

begin

    clk <= not clk after CLK_PERIOD / 2 when not done else '0';

    u_dut : entity work.tdc_gpx_cell_pipe
        generic map (
            g_OUTPUT_WIDTH => OUTPUT_WIDTH,
            g_AXIS_CLK_MHZ => 200
        )
        port map (
            i_clk                 => clk,
            i_rst_n               => rst_n,
            i_evt_sk_tvalid       => evt_tvalid,
            i_evt_sk_tdata        => evt_tdata,
            i_evt_sk_tuser        => evt_tuser,
            o_evt_sk_tready       => evt_tready,
            i_shot_start_per_chip => shot_start,
            i_rise_chip_mask      => rise_mask,
            i_fall_chip_mask      => fall_mask,
            i_abort               => abort,
            i_sticky_clear        => sticky_clear,
            i_face_stops_per_chip => face_stops,
            i_max_hits_cfg        => max_hits,
            i_max_range_5ns_ticks => (others => '0'),
            o_cell_rise_tdata_0   => open,
            o_cell_rise_tdata_1   => open,
            o_cell_rise_tdata_2   => open,
            o_cell_rise_tdata_3   => open,
            o_cell_rise_tvalid    => cell_rise_tvalid,
            o_cell_rise_tlast     => cell_rise_tlast,
            i_cell_rise_tready    => cell_rise_tready,
            o_cell_fall_tdata_0   => open,
            o_cell_fall_tdata_1   => open,
            o_cell_fall_tdata_2   => open,
            o_cell_fall_tdata_3   => open,
            o_cell_fall_tvalid    => cell_fall_tvalid,
            o_cell_fall_tlast     => cell_fall_tlast,
            i_cell_fall_tready    => cell_fall_tready,
            o_hit_dropped         => open,
            o_hit_fall_dropped    => open,
            o_shot_dropped        => shot_dropped,
            o_shot_fall_dropped   => shot_fall_dropped,
            o_slice_timeout       => open,
            o_slice_fall_timeout  => open,
            o_stop_id_error       => open,
            o_stop_id_fall_error  => open,
            o_quarantine_escape_rise => open,
            o_quarantine_escape_fall => open,
            o_cell_rise_tuser     => open,
            o_cell_fall_tuser     => open,
            o_masked_slope_drop_rise => masked_drop_rise,
            o_masked_slope_drop_fall => masked_drop_fall
        );

    -- =========================================================================
    -- Monitors
    -- =========================================================================
    p_monitor : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to c_N_CHIPS-1 loop
                if cell_rise_tvalid(i) = '1' and cell_rise_tready(i) = '1' then
                    beat_rise(i) <= beat_rise(i) + 1;
                    if cell_rise_tlast(i) = '1' then
                        tlast_rise(i) <= tlast_rise(i) + 1;
                    end if;
                end if;
                if cell_fall_tvalid(i) = '1' and cell_fall_tready(i) = '1' then
                    beat_fall(i) <= beat_fall(i) + 1;
                    if cell_fall_tlast(i) = '1' then
                        tlast_fall(i) <= tlast_fall(i) + 1;
                    end if;
                end if;
            end loop;
            if shot_dropped /= "0000" or shot_fall_dropped /= "0000" then
                saw_shot_drop <= true;
            end if;
        end if;
    end process p_monitor;

    -- =========================================================================
    -- Stimulus
    -- =========================================================================
    p_stim : process
        function fn_tuser_data(slope, chip_id, stop_id, ififo_id : natural)
            return std_logic_vector is
            variable v : std_logic_vector(15 downto 0) := (others => '0');
        begin
            v(0)          := std_logic(to_unsigned(slope, 1)(0));
            v(2 downto 1) := std_logic_vector(to_unsigned(chip_id, 2));
            v(5 downto 3) := std_logic_vector(to_unsigned(stop_id, 3));
            v(6)          := std_logic(to_unsigned(ififo_id, 1)(0));
            v(7)          := '0';
            return v;
        end function;

        function fn_tuser_drain(ififo_id : natural)
            return std_logic_vector is
            variable v : std_logic_vector(15 downto 0) := (others => '0');
        begin
            v(7) := '1';
            v(6) := std_logic(to_unsigned(ififo_id, 1)(0));
            return v;
        end function;

        -- Send one beat on chip ch and wait for acceptance
        procedure send_beat(ch : natural;
                            data : std_logic_vector(31 downto 0);
                            usr  : std_logic_vector(15 downto 0)) is
        begin
            evt_tdata(ch)  <= data;
            evt_tuser(ch)  <= usr;
            evt_tvalid(ch) <= '1';
            -- Handshake-exact: proceed on the clock edge where tready was
            -- high AT the edge (pre-edge value), matching what the DUT samples.
            wait until rising_edge(clk) and evt_tready(ch) = '1';
            evt_tvalid(ch) <= '0';
            wait until rising_edge(clk);
        end procedure;

        -- One full shot: pulse shot_start for all chips, then per chip send
        -- one hit (its physical slope) + ififo1_done + final drain_done.
        procedure run_shot is
            variable v_slope : natural;
        begin
            shot_start <= (others => '1');
            wait until rising_edge(clk);
            shot_start <= (others => '0');
            wait for 5 * CLK_PERIOD;
            wait until rising_edge(clk);
            for ch in 0 to c_N_CHIPS-1 loop
                if ch < 2 then v_slope := 1; else v_slope := 0; end if;
                send_beat(ch, std_logic_vector(to_unsigned(ch + 1, 32)),
                          fn_tuser_data(v_slope, ch, 0, 0));
                send_beat(ch, x"00000000", fn_tuser_drain(0));
                send_beat(ch, x"00000000", fn_tuser_drain(1));
            end loop;
        end procedure;

        variable v_base_beat_rise  : t_nat_arr;
        variable v_base_beat_fall  : t_nat_arr;
        variable v_base_tlast_rise : t_nat_arr;
        variable v_base_tlast_fall : t_nat_arr;
        variable v_ok : boolean;
    begin
        rst_n <= '0';
        wait for 100 ns;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);

        -- =====================================================================
        -- SCENARIO L: legacy masks "1111" -- every (chip, slope) builder
        -- emits a slice (wrong-slope = all-blank). Pre-fix behavior evidence
        -- AND default-compatibility check.
        -- =====================================================================
        report "===== LANE_MASK SCENARIO L: legacy masks 1111 =====" severity note;
        rise_mask <= (others => '1');
        fall_mask <= (others => '1');
        wait until rising_edge(clk);

        run_shot;
        wait for 4 us;
        wait until rising_edge(clk);

        v_ok := true;
        for i in 0 to c_N_CHIPS-1 loop
            report "L: chip" & integer'image(i)
                   & " rise beats=" & integer'image(beat_rise(i))
                   & " tlast=" & integer'image(tlast_rise(i))
                   & " / fall beats=" & integer'image(beat_fall(i))
                   & " tlast=" & integer'image(tlast_fall(i)) severity note;
            if tlast_rise(i) /= 1 or tlast_fall(i) /= 1
               or beat_rise(i) /= C_SLICE_BEATS
               or beat_fall(i) /= C_SLICE_BEATS then
                v_ok := false;
            end if;
        end loop;
        if v_ok and masked_drop_rise = "0000" and masked_drop_fall = "0000" then
            report "*** LANE_MASK SCENARIO L PASS (legacy: all 8 lanes emit"
                   & " a slice per shot; wrong-slope churn confirmed) ***"
                severity note;
        else
            report "*** LANE_MASK SCENARIO L FAIL ***" severity failure;
        end if;

        -- =====================================================================
        -- SCENARIO D: dedicated masks rise 0011 / fall 1100, 3 shots.
        -- Wrong-slope lanes must be completely silent; no builder starvation.
        -- =====================================================================
        report "===== LANE_MASK SCENARIO D: dedicated masks, 3 shots =====" severity note;
        rise_mask <= "0011";
        fall_mask <= "1100";
        wait until rising_edge(clk);

        v_base_beat_rise  := beat_rise;
        v_base_beat_fall  := beat_fall;
        v_base_tlast_rise := tlast_rise;
        v_base_tlast_fall := tlast_fall;

        for shot in 1 to 3 loop
            run_shot;
            wait for 3 us;
            wait until rising_edge(clk);
        end loop;

        v_ok := true;
        for i in 0 to c_N_CHIPS-1 loop
            report "D: chip" & integer'image(i)
                   & " rise beats+=" & integer'image(beat_rise(i) - v_base_beat_rise(i))
                   & " tlast+=" & integer'image(tlast_rise(i) - v_base_tlast_rise(i))
                   & " / fall beats+=" & integer'image(beat_fall(i) - v_base_beat_fall(i))
                   & " tlast+=" & integer'image(tlast_fall(i) - v_base_tlast_fall(i))
                severity note;
        end loop;
        for i in 0 to 1 loop
            if (tlast_rise(i) - v_base_tlast_rise(i)) /= 3
               or (beat_rise(i) - v_base_beat_rise(i)) /= 3 * C_SLICE_BEATS
               or (beat_fall(i) - v_base_beat_fall(i)) /= 0 then
                v_ok := false;
            end if;
        end loop;
        for i in 2 to 3 loop
            if (tlast_fall(i) - v_base_tlast_fall(i)) /= 3
               or (beat_fall(i) - v_base_beat_fall(i)) /= 3 * C_SLICE_BEATS
               or (beat_rise(i) - v_base_beat_rise(i)) /= 0 then
                v_ok := false;
            end if;
        end loop;
        if v_ok and not saw_shot_drop
           and masked_drop_rise = "0000" and masked_drop_fall = "0000" then
            report "*** LANE_MASK SCENARIO D PASS (wrong-slope lanes silent,"
                   & " no shot drops across 3 shots) ***" severity note;
        else
            report "*** LANE_MASK SCENARIO D FAIL ***" severity failure;
        end if;

        -- =====================================================================
        -- SCENARIO M: hit beat addressed to a masked slope (chip 0, fall).
        -- Must be consumed (no stall), flagged sticky, chip 0 rise slice OK.
        -- =====================================================================
        report "===== LANE_MASK SCENARIO M: masked-slope hit drop =====" severity note;
        v_base_tlast_rise := tlast_rise;
        v_base_beat_fall  := beat_fall;

        shot_start <= (others => '1');
        wait until rising_edge(clk);
        shot_start <= (others => '0');
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);

        -- Wrong-slope hit on chip 0 (fall while fall_mask(0)=0)
        send_beat(0, x"000000AA", fn_tuser_data(0, 0, 0, 0));
        -- Normal rise hit + drains so the shot completes
        send_beat(0, x"00000001", fn_tuser_data(1, 0, 0, 0));
        send_beat(0, x"00000000", fn_tuser_drain(0));
        send_beat(0, x"00000000", fn_tuser_drain(1));
        -- Other chips: drains only (blank slices on their own lanes)
        for ch in 1 to c_N_CHIPS-1 loop
            send_beat(ch, x"00000000", fn_tuser_drain(0));
            send_beat(ch, x"00000000", fn_tuser_drain(1));
        end loop;
        wait for 4 us;
        wait until rising_edge(clk);

        if masked_drop_fall(0) = '1'
           and masked_drop_rise = "0000"
           and masked_drop_fall(3 downto 1) = "000"
           and (tlast_rise(0) - v_base_tlast_rise(0)) = 1
           and (beat_fall(0) - v_base_beat_fall(0)) = 0 then
            report "*** LANE_MASK SCENARIO M PASS (masked-slope hit dropped"
                   & " with sticky, stream alive) ***" severity note;
        else
            report "LANE_MASK M: sticky_fall=" & std_logic'image(masked_drop_fall(0))
                   & " rise_tlast_delta=" & integer'image(tlast_rise(0) - v_base_tlast_rise(0))
                   & " fall_beat_delta=" & integer'image(beat_fall(0) - v_base_beat_fall(0))
                severity note;
            report "*** LANE_MASK SCENARIO M FAIL ***" severity failure;
        end if;

        -- =====================================================================
        -- SCENARIO C: sticky lifecycle across abort and explicit soft clear.
        -- =====================================================================
        report "===== LANE_MASK SCENARIO C: sticky lifecycle =====" severity note;
        abort <= '1';
        wait until rising_edge(clk);
        abort <= '0';
        wait until rising_edge(clk);

        assert masked_drop_fall(0) = '1'
            report "LANE_MASK C: abort erased post-run masked-slope evidence"
            severity failure;

        sticky_clear <= '1';
        wait until rising_edge(clk);
        sticky_clear <= '0';
        wait until rising_edge(clk);

        assert masked_drop_rise = "0000" and masked_drop_fall = "0000"
            report "LANE_MASK C: explicit sticky clear did not clear both masks"
            severity failure;

        run_shot;
        wait for 4 us;
        wait until rising_edge(clk);
        assert masked_drop_rise = "0000" and masked_drop_fall = "0000"
            report "LANE_MASK C: nominal run re-contaminated masked-slope history"
            severity failure;
        report "*** LANE_MASK SCENARIO C PASS (abort retain, soft clear, clean next run) ***"
            severity note;

        report "*** TB_LANE_MASK ALL PASS ***" severity note;
        done <= true;
        wait for 10 * CLK_PERIOD;
        std.env.finish;
    end process p_stim;

end architecture sim;
