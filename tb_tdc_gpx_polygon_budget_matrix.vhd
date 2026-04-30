-- ============================================================================
-- Testbench: tb_tdc_gpx_polygon_budget_matrix
--
-- Purpose:
--   Analytical C04 timing-capacity check for the polygon mirror timing budget.
--
-- Model:
--   * Polygon start_tdc interval is 13.888889 us:
--       10 Hz rotation, 5 faces, 60 deg used out of a 72 deg facet,
--       0.05 deg point spacing.
--   * VDMA + PS processing + Ethernet transfer reserve is 8.000000 us.
--   * Remaining budget before time-of-flight is 5.888889 us.
--   * For each distance, available C04 budget is:
--       5.888889 us - round_trip(distance)
--   * For each width and distance, report the largest max_hits_cfg that
--     fits within the available budget.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_polygon_budget_matrix is
end entity;

architecture sim of tb_tdc_gpx_polygon_budget_matrix is

    constant C_ACTIVE_CHIPS        : natural := 4;
    constant C_STOPS_PER_CHIP      : natural := 8;
    constant C_START_DISTANCE_M    : natural := 10;
    constant C_DISTANCE_STEP_M     : natural := 10;
    constant C_MAX_DISTANCE_M      : natural := 900;
    constant C_RT_PS_PER_M         : natural := 6671;
    constant C_OUTPUT_PERIOD_PS    : natural := 6667;      -- 150 MHz
    constant C_START_INTERVAL_PS   : integer := 13888889;  -- 13.888889 us
    constant C_SYSTEM_RESERVED_PS  : integer := 8000000;   -- 8 us
    constant C_PRE_TOF_BUDGET_PS   : integer :=
        C_START_INTERVAL_PS - C_SYSTEM_RESERVED_PS;

    function fn_line_beats(
        max_hits    : natural;
        tdata_width : natural
    ) return natural is
    begin
        return fn_hdr_prefix_beats(tdata_width) +
               C_ACTIVE_CHIPS * C_STOPS_PER_CHIP *
               fn_beats_per_cell_rt(max_hits, tdata_width);
    end function;

    function fn_round_trip_ps(distance_m : natural) return integer is
    begin
        return integer(distance_m * C_RT_PS_PER_M);
    end function;

    function fn_budget_ps(distance_m : natural) return integer is
    begin
        return C_PRE_TOF_BUDGET_PS - fn_round_trip_ps(distance_m);
    end function;

    function fn_drain_ps(
        max_hits    : natural;
        tdata_width : natural
    ) return integer is
    begin
        return integer(fn_line_beats(max_hits, tdata_width) * C_OUTPUT_PERIOD_PS);
    end function;

    procedure run_width_sweep(
        constant width_bits : in natural
    ) is
        variable v_distance_m     : natural;
        variable v_budget_ps      : integer;
        variable v_round_trip_ps  : integer;
        variable v_max_pass_cfg   : integer;
        variable v_prev_max_cfg   : integer;
        variable v_cfg1_margin_ps : integer;
        variable v_cfg6_margin_ps : integer;
        variable v_cfg7_margin_ps : integer;
        variable v_seen_pass      : boolean;
    begin
        report "POLY_SWEEP WIDTH=" & integer'image(width_bits)
               & " START_DISTANCE_M=" & integer'image(C_START_DISTANCE_M)
               & " STEP_M=" & integer'image(C_DISTANCE_STEP_M)
               & " START_INTERVAL_PS=" & integer'image(C_START_INTERVAL_PS)
               & " SYSTEM_RESERVED_PS=" & integer'image(C_SYSTEM_RESERVED_PS)
               & " PRE_TOF_BUDGET_PS=" & integer'image(C_PRE_TOF_BUDGET_PS)
               & " OUTPUT_PERIOD_PS=" & integer'image(C_OUTPUT_PERIOD_PS)
               severity note;

        v_distance_m   := C_START_DISTANCE_M;
        v_prev_max_cfg := -1;
        v_seen_pass    := false;

        while v_distance_m <= C_MAX_DISTANCE_M loop
            v_round_trip_ps := fn_round_trip_ps(v_distance_m);
            v_budget_ps     := fn_budget_ps(v_distance_m);
            v_max_pass_cfg  := 0;

            for cfg in 1 to 7 loop
                if v_budget_ps >= fn_drain_ps(cfg, width_bits) then
                    v_max_pass_cfg := cfg;
                end if;
            end loop;

            if v_max_pass_cfg > 0 then
                v_seen_pass := true;
            end if;

            v_cfg1_margin_ps := v_budget_ps - fn_drain_ps(1, width_bits);
            v_cfg6_margin_ps := v_budget_ps - fn_drain_ps(6, width_bits);
            v_cfg7_margin_ps := v_budget_ps - fn_drain_ps(7, width_bits);

            report "POLY_SWEEP WIDTH=" & integer'image(width_bits)
                   & " DIST_M=" & integer'image(v_distance_m)
                   & " ROUND_TRIP_PS=" & integer'image(v_round_trip_ps)
                   & " START_INTERVAL_PS=" & integer'image(C_START_INTERVAL_PS)
                   & " SYSTEM_RESERVED_PS=" & integer'image(C_SYSTEM_RESERVED_PS)
                   & " BUDGET_PS=" & integer'image(v_budget_ps)
                   & " MAX_PASS_CFG=" & integer'image(v_max_pass_cfg)
                   & " CFG1_MARGIN_PS=" & integer'image(v_cfg1_margin_ps)
                   & " CFG6_MARGIN_PS=" & integer'image(v_cfg6_margin_ps)
                   & " CFG7_MARGIN_PS=" & integer'image(v_cfg7_margin_ps)
                   severity note;

            if v_max_pass_cfg /= v_prev_max_cfg then
                report "POLY_BOUNDARY WIDTH=" & integer'image(width_bits)
                       & " DIST_M=" & integer'image(v_distance_m)
                       & " MAX_PASS_CFG=" & integer'image(v_max_pass_cfg)
                       & " BUDGET_PS=" & integer'image(v_budget_ps)
                       & " CFG1_DRAIN_PS=" & integer'image(fn_drain_ps(1, width_bits))
                       & " CFG6_DRAIN_PS=" & integer'image(fn_drain_ps(6, width_bits))
                       & " CFG7_DRAIN_PS=" & integer'image(fn_drain_ps(7, width_bits))
                       severity note;
                v_prev_max_cfg := v_max_pass_cfg;
            end if;

            if v_seen_pass and v_max_pass_cfg = 0 then
                exit;
            end if;

            v_distance_m := v_distance_m + C_DISTANCE_STEP_M;
        end loop;

        report "POLY_SWEEP WIDTH=" & integer'image(width_bits)
               & " COMPLETE LAST_DISTANCE_M=" & integer'image(v_distance_m)
               & " LAST_MAX_PASS_CFG=" & integer'image(v_prev_max_cfg)
               severity note;
    end procedure;

begin

    p_main : process
    begin
        report "tb_tdc_gpx_polygon_budget_matrix: start" severity note;
        report "POLY_MODEL active_chips="
               & integer'image(C_ACTIVE_CHIPS)
               & " stops_per_chip=" & integer'image(C_STOPS_PER_CHIP)
               & " round_trip_ps_per_m=" & integer'image(C_RT_PS_PER_M)
               & " point_interval_ps=" & integer'image(C_START_INTERVAL_PS)
               & " system_reserved_ps=" & integer'image(C_SYSTEM_RESERVED_PS)
               & " pre_tof_budget_ps=" & integer'image(C_PRE_TOF_BUDGET_PS)
               severity note;

        run_width_sweep(32);
        run_width_sweep(64);
        run_width_sweep(128);

        report "*** tb_tdc_gpx_polygon_budget_matrix PASS ***" severity note;
        finish;
        wait;
    end process;

end architecture;
