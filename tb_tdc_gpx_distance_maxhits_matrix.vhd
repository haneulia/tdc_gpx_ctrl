-- ============================================================================
-- Testbench: tb_tdc_gpx_distance_maxhits_matrix
--
-- Purpose:
--   Analytical C04 timing-capacity check.  It maps operating distance to the
--   start_tdc -> stop_tdc round-trip time, then records whether a full C04
--   output line can drain at the downstream output clock for each max_hits_cfg
--   and output width.
--
-- Notes:
--   * Distance starts at 150 m and increases by 50 m only after a FAIL.
--   * After a FAIL, the same max_hits_cfg is retried at the next distance.
--   * Full line load is 4 chips x 8 stops.
--   * Output drain clock is modeled as 150 MHz.
--   * Primary judgement is the conservative post-stop model:
--       start_tdc = 0
--       stop_tdc  = round_trip(distance)
--       next start_tdc = 1.5 * round_trip(distance)
--       C04 drain is budgeted only after stop_tdc.
--   * The previous full-round-trip/overlap check is still reported as a
--     reference field, but it is not used to advance the sweep.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_distance_maxhits_matrix is
    generic (
        g_START_DISTANCE_M : natural := 150;
        g_DISTANCE_STEP_M  : natural := 50
    );
end entity;

architecture sim of tb_tdc_gpx_distance_maxhits_matrix is

    constant C_ACTIVE_CHIPS        : natural := 4;
    constant C_STOPS_PER_CHIP      : natural := 8;
    constant C_START_DISTANCE_M    : natural := g_START_DISTANCE_M;
    constant C_DISTANCE_STEP_M     : natural := g_DISTANCE_STEP_M;
    constant C_RT_PS_PER_M         : natural := 6671;
    constant C_OUTPUT_PERIOD_PS    : natural := 6667;  -- 150 MHz
    constant C_SHOT_PERIOD_NUM     : natural := 3;
    constant C_SHOT_PERIOD_DEN     : natural := 2;
    constant C_MAX_DISTANCE_M      : natural := 5000;

    function fn_line_beats(
        max_hits       : natural;
        tdata_width    : natural
    ) return natural is
    begin
        return fn_hdr_prefix_beats(tdata_width) +
               C_ACTIVE_CHIPS * C_STOPS_PER_CHIP *
               fn_beats_per_cell_rt(max_hits, tdata_width);
    end function;

    function fn_round_trip_ps(distance_m : natural) return natural is
    begin
        return distance_m * C_RT_PS_PER_M;
    end function;

    function fn_shot_period_ps(round_trip_ps : natural) return natural is
    begin
        return (round_trip_ps * C_SHOT_PERIOD_NUM) / C_SHOT_PERIOD_DEN;
    end function;

    function fn_post_stop_budget_ps(round_trip_ps : natural) return natural is
    begin
        return fn_shot_period_ps(round_trip_ps) - round_trip_ps;
    end function;

    function fn_drain_ps(line_beats : natural) return natural is
    begin
        return line_beats * C_OUTPUT_PERIOD_PS;
    end function;

    procedure run_width_sweep(
        constant width_bits : in natural
    ) is
        variable v_distance_m : natural;
        variable v_cfg        : natural;
        variable v_line_beats : natural;
        variable v_drain_ps   : natural;
        variable v_round_trip_ps : natural;
        variable v_shot_period_ps : natural;
        variable v_post_stop_budget_ps : natural;
        variable v_ideal_margin_ps : integer;
        variable v_post_stop_margin_ps : integer;
        variable v_ideal_result : string(1 to 4);
        variable v_post_stop_result : string(1 to 4);
    begin
        report "DIST_SWEEP WIDTH=" & integer'image(width_bits)
               & " START distance_m=" & integer'image(C_START_DISTANCE_M)
               & " step_m=" & integer'image(C_DISTANCE_STEP_M)
               & " output_period_ps=" & integer'image(C_OUTPUT_PERIOD_PS)
               & " shot_period_ratio="
               & integer'image(C_SHOT_PERIOD_NUM) & "/"
               & integer'image(C_SHOT_PERIOD_DEN)
               severity note;

        v_distance_m := C_START_DISTANCE_M;
        v_cfg        := 1;

        while v_cfg <= 7 loop
            assert v_distance_m <= C_MAX_DISTANCE_M
                report "DIST_SWEEP WIDTH=" & integer'image(width_bits)
                       & " exceeded max distance while checking cfg="
                       & integer'image(v_cfg)
                severity failure;

            v_line_beats := fn_line_beats(v_cfg, width_bits);
            v_drain_ps   := fn_drain_ps(v_line_beats);
            v_round_trip_ps := fn_round_trip_ps(v_distance_m);
            v_shot_period_ps := fn_shot_period_ps(v_round_trip_ps);
            v_post_stop_budget_ps := fn_post_stop_budget_ps(v_round_trip_ps);
            v_ideal_margin_ps := integer(v_round_trip_ps) - integer(v_drain_ps);
            v_post_stop_margin_ps := integer(v_post_stop_budget_ps) - integer(v_drain_ps);

            if v_drain_ps <= v_round_trip_ps then
                v_ideal_result := "PASS";
            else
                v_ideal_result := "FAIL";
            end if;

            if v_drain_ps <= v_post_stop_budget_ps then
                v_post_stop_result := "PASS";
            else
                v_post_stop_result := "FAIL";
            end if;

            if v_drain_ps <= v_post_stop_budget_ps then
                report "DIST_SWEEP WIDTH=" & integer'image(width_bits)
                       & " DIST_M=" & integer'image(v_distance_m)
                       & " CFG=" & integer'image(v_cfg)
                       & " LINE_BEATS=" & integer'image(v_line_beats)
                       & " DRAIN_PS=" & integer'image(v_drain_ps)
                       & " START_TDC_PS=0"
                       & " STOP_TDC_PS=" & integer'image(v_round_trip_ps)
                       & " SHOT_PERIOD_PS=" & integer'image(v_shot_period_ps)
                       & " POST_STOP_BUDGET_PS=" & integer'image(v_post_stop_budget_ps)
                       & " DRAIN_END_IF_POST_STOP_PS="
                       & integer'image(v_round_trip_ps + v_drain_ps)
                       & " IDEAL_RT_RESULT=" & v_ideal_result
                       & " IDEAL_RT_MARGIN_PS=" & integer'image(v_ideal_margin_ps)
                       & " POST_STOP_RESULT=" & v_post_stop_result
                       & " POST_STOP_MARGIN_PS="
                       & integer'image(v_post_stop_margin_ps)
                       & " PASS"
                       severity note;
                v_cfg := v_cfg + 1;
            else
                report "DIST_SWEEP WIDTH=" & integer'image(width_bits)
                       & " DIST_M=" & integer'image(v_distance_m)
                       & " CFG=" & integer'image(v_cfg)
                       & " LINE_BEATS=" & integer'image(v_line_beats)
                       & " DRAIN_PS=" & integer'image(v_drain_ps)
                       & " START_TDC_PS=0"
                       & " STOP_TDC_PS=" & integer'image(v_round_trip_ps)
                       & " SHOT_PERIOD_PS=" & integer'image(v_shot_period_ps)
                       & " POST_STOP_BUDGET_PS=" & integer'image(v_post_stop_budget_ps)
                       & " DRAIN_END_IF_POST_STOP_PS="
                       & integer'image(v_round_trip_ps + v_drain_ps)
                       & " IDEAL_RT_RESULT=" & v_ideal_result
                       & " IDEAL_RT_MARGIN_PS=" & integer'image(v_ideal_margin_ps)
                       & " POST_STOP_RESULT=" & v_post_stop_result
                       & " POST_STOP_MARGIN_PS="
                       & integer'image(v_post_stop_margin_ps)
                       & " FAIL_RETRY_NEXT_DISTANCE"
                       severity note;
                v_distance_m := v_distance_m + C_DISTANCE_STEP_M;
            end if;
        end loop;

        report "DIST_SWEEP WIDTH=" & integer'image(width_bits)
               & " COMPLETE FINAL_DISTANCE_M=" & integer'image(v_distance_m)
               severity note;
    end procedure;

begin

    p_main : process
    begin
        report "tb_tdc_gpx_distance_maxhits_matrix: start" severity note;
        report "DIST_SWEEP MODEL active_chips="
               & integer'image(C_ACTIVE_CHIPS)
               & " stops_per_chip=" & integer'image(C_STOPS_PER_CHIP)
               & " round_trip_ps_per_m=" & integer'image(C_RT_PS_PER_M)
               & " shot_period_ratio="
               & integer'image(C_SHOT_PERIOD_NUM) & "/"
               & integer'image(C_SHOT_PERIOD_DEN)
               severity note;

        run_width_sweep(32);
        run_width_sweep(64);
        run_width_sweep(128);

        report "*** tb_tdc_gpx_distance_maxhits_matrix PASS ***" severity note;
        finish;
        wait;
    end process;

end architecture;
