-- =============================================================================
-- tb_tdc_gpx_range_ticks.vhd
-- Unit test for runtime 5 ns ticks and elaboration-time physical timing.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_range_ticks is
end entity tb_tdc_gpx_range_ticks;

architecture sim of tb_tdc_gpx_range_ticks is
begin
    p_test : process
        procedure check_count(
            ticks_5ns : natural;
            clk_mhz   : positive;
            expected  : natural
        ) is
            variable v_actual : unsigned(15 downto 0);
        begin
            v_actual := fn_range_5ns_ticks_to_clks(
                to_unsigned(ticks_5ns, 16), clk_mhz);
            assert to_integer(v_actual) = expected
                report "range conversion mismatch: ticks="
                    & integer'image(ticks_5ns)
                    & " clk_mhz=" & integer'image(clk_mhz)
                    & " expected=" & integer'image(expected)
                    & " actual=" & integer'image(to_integer(v_actual))
                severity failure;
        end procedure;

        procedure check_time_ns(
            time_ns  : natural;
            clk_mhz  : positive;
            expected : natural
        ) is
            variable v_actual : natural;
        begin
            v_actual := fn_time_ns_to_clks_ceil(time_ns, clk_mhz);
            assert v_actual = expected
                report "ns conversion mismatch: time_ns="
                    & integer'image(time_ns)
                    & " clk_mhz=" & integer'image(clk_mhz)
                    & " expected=" & integer'image(expected)
                    & " actual=" & integer'image(v_actual)
                severity failure;
        end procedure;

        procedure check_time_ps(
            time_ps  : natural;
            clk_mhz  : positive;
            expected : natural
        ) is
            variable v_actual : natural;
        begin
            v_actual := fn_time_ps_to_clks_ceil(time_ps, clk_mhz);
            assert v_actual = expected
                report "ps conversion mismatch: time_ps="
                    & integer'image(time_ps)
                    & " clk_mhz=" & integer'image(clk_mhz)
                    & " expected=" & integer'image(expected)
                    & " actual=" & integer'image(v_actual)
                severity failure;
        end procedure;
    begin
        assert c_RANGE_REF_CLK_MHZ = 200 and c_RANGE_REF_TICK_NS = 5
            report "range reference timebase must remain 200 MHz / 5 ns"
            severity failure;

        assert fn_range_clk_mhz_supported(50)
            and fn_range_clk_mhz_supported(100)
            and fn_range_clk_mhz_supported(125)
            and fn_range_clk_mhz_supported(150)
            and fn_range_clk_mhz_supported(200)
            and not fn_range_clk_mhz_supported(75)
            report "supported range-clock set mismatch"
            severity failure;

        for clk_mhz in 50 to 200 loop
            if fn_range_clk_mhz_supported(clk_mhz) then
                check_count(0, clk_mhz, 0);
            end if;
        end loop;

        -- 267 ticks: legacy default (~200 m).
        check_count(267,  50,  67);
        check_count(267, 100, 134);
        check_count(267, 125, 167);
        check_count(267, 150, 201);
        check_count(267, 200, 267);

        -- 401 ticks: C08 representative 300 m contract.
        check_count(401,  50, 101);
        check_count(401, 100, 201);
        check_count(401, 125, 251);
        check_count(401, 150, 301);
        check_count(401, 200, 401);

        -- Maximum CSR value also proves intermediate arithmetic width.
        check_count(65535,  50, 16384);
        check_count(65535, 100, 32768);
        check_count(65535, 125, 40960);
        check_count(65535, 150, 49152);
        check_count(65535, 200, 65535);

        -- Static timing conversion preserves physical duration as the clock
        -- changes. Representative defaults also prove upward rounding.
        check_time_ns(c_DEFAULT_POWERUP_TIME_NS,  50, 12);
        check_time_ns(c_DEFAULT_POWERUP_TIME_NS, 100, 24);
        check_time_ns(c_DEFAULT_POWERUP_TIME_NS, 125, 30);
        check_time_ns(c_DEFAULT_POWERUP_TIME_NS, 150, 36);
        check_time_ns(c_DEFAULT_POWERUP_TIME_NS, 200, 48);

        check_time_ns(c_DEFAULT_BUS_READ_PERIOD_MIN_TIME_NS,  50, 2);
        check_time_ns(c_DEFAULT_BUS_READ_PERIOD_MIN_TIME_NS, 100, 3);
        check_time_ns(c_DEFAULT_BUS_READ_PERIOD_MIN_TIME_NS, 125, 4);
        check_time_ns(c_DEFAULT_BUS_READ_PERIOD_MIN_TIME_NS, 150, 4);
        check_time_ns(c_DEFAULT_BUS_READ_PERIOD_MIN_TIME_NS, 200, 5);

        check_time_ns(c_DEFAULT_CELL_QUARANTINE_MARGIN_TIME_NS, 150, 512);
        check_time_ns(c_DEFAULT_CELL_IFIFO2_MARGIN_TIME_NS, 150, 256);
        check_time_ps(c_TDC_EF_DATA_VALID_MAX_PS,  50, 1);
        check_time_ps(c_TDC_EF_DATA_VALID_MAX_PS, 100, 2);
        check_time_ps(c_TDC_EF_DATA_VALID_MAX_PS, 125, 2);
        check_time_ps(c_TDC_EF_DATA_VALID_MAX_PS, 150, 2);
        check_time_ps(c_TDC_EF_DATA_VALID_MAX_PS, 200, 3);

        assert c_DEFAULT_EF_SYNC_GUARD_CLKS = 5
            report "default EF guard must include data-valid time plus 2-FF latency"
            severity failure;

        report "5 ns range tick conversion matrix - PASS" severity note;
        finish;
        wait;
    end process;
end architecture sim;
