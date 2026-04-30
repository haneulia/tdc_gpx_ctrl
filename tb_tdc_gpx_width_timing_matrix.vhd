-- ============================================================================
-- Testbench: tb_tdc_gpx_width_timing_matrix
--
-- Purpose:
--   Closes C02 Data Flow / Timing Pipeline verification gaps for output widths
--   32, 64, and 128 bit. This TB checks the package-level contracts used by
--   cell_builder, output_stage, and header_inserter:
--     * supported output widths
--     * AXI keep/header beat widths
--     * runtime max_hits_cfg cell beat matrix
--     * line transfer beat model for C=4 chips and N=8 stops
--
-- Notes:
--   i_max_hits_cfg=000 is handled in tdc_gpx_cell_builder by aliasing to 7.
--   This TB checks the effective 1..7 values used by the selected helper.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_width_timing_matrix is
end entity;

architecture sim of tb_tdc_gpx_width_timing_matrix is

    function fn_line_beats(
        max_hits       : natural;
        tdata_width    : natural;
        active_chips   : natural;
        stops_per_chip : natural
    ) return natural is
    begin
        return fn_hdr_prefix_beats(tdata_width) +
               active_chips * stops_per_chip *
               fn_beats_per_cell_rt(max_hits, tdata_width);
    end function;

    procedure check_nat(
        constant name_s   : in string;
        constant observed : in natural;
        constant expected : in natural
    ) is
    begin
        assert observed = expected
            report name_s & " mismatch: observed=" & integer'image(observed) &
                   " expected=" & integer'image(expected)
            severity failure;
        report name_s & " = " & integer'image(observed) & " PASS"
            severity note;
    end procedure;

begin

    p_main : process
        constant C_ACTIVE_CHIPS   : natural := 4;
        constant C_STOPS_PER_CHIP : natural := 8;
    begin
        report "tb_tdc_gpx_width_timing_matrix: start" severity note;

        assert fn_output_width_supported(32)
            report "32-bit output width must be supported"
            severity failure;
        assert fn_output_width_supported(64)
            report "64-bit output width must be supported"
            severity failure;
        assert fn_output_width_supported(128)
            report "128-bit output width must be supported"
            severity failure;
        assert not fn_output_width_supported(256)
            report "256-bit output width is intentionally out of C02 scope"
            severity failure;

        check_nat("hit slot data width", c_HIT_SLOT_DATA_WIDTH, 16);

        check_nat("keep width 32", fn_axis_keep_width(32), 4);
        check_nat("keep width 64", fn_axis_keep_width(64), 8);
        check_nat("keep width 128", fn_axis_keep_width(128), 16);

        check_nat("header beats 32", fn_hdr_prefix_beats(32), 12);
        check_nat("header beats 64", fn_hdr_prefix_beats(64), 6);
        check_nat("header beats 128", fn_hdr_prefix_beats(128), 3);

        check_nat("cell beats max1 width32", fn_beats_per_cell_rt(1, 32), 2);
        check_nat("cell beats max1 width64", fn_beats_per_cell_rt(1, 64), 2);
        check_nat("cell beats max1 width128", fn_beats_per_cell_rt(1, 128), 2);

        check_nat("cell beats max2 width32", fn_beats_per_cell_rt(2, 32), 2);
        check_nat("cell beats max2 width64", fn_beats_per_cell_rt(2, 64), 2);
        check_nat("cell beats max2 width128", fn_beats_per_cell_rt(2, 128), 2);

        check_nat("cell beats max3 width32", fn_beats_per_cell_rt(3, 32), 3);
        check_nat("cell beats max3 width64", fn_beats_per_cell_rt(3, 64), 2);
        check_nat("cell beats max3 width128", fn_beats_per_cell_rt(3, 128), 2);

        check_nat("cell beats max4 width32", fn_beats_per_cell_rt(4, 32), 3);
        check_nat("cell beats max4 width64", fn_beats_per_cell_rt(4, 64), 2);
        check_nat("cell beats max4 width128", fn_beats_per_cell_rt(4, 128), 2);

        check_nat("cell beats max5 width32", fn_beats_per_cell_rt(5, 32), 4);
        check_nat("cell beats max5 width64", fn_beats_per_cell_rt(5, 64), 3);
        check_nat("cell beats max5 width128", fn_beats_per_cell_rt(5, 128), 2);

        check_nat("cell beats max6 width32", fn_beats_per_cell_rt(6, 32), 4);
        check_nat("cell beats max6 width64", fn_beats_per_cell_rt(6, 64), 3);
        check_nat("cell beats max6 width128", fn_beats_per_cell_rt(6, 128), 2);

        check_nat("cell beats max7 width32", fn_beats_per_cell_rt(7, 32), 5);
        check_nat("cell beats max7 width64", fn_beats_per_cell_rt(7, 64), 3);
        check_nat("cell beats max7 width128", fn_beats_per_cell_rt(7, 128), 2);

        check_nat("line beats max1 width32", fn_line_beats(1, 32, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 76);
        check_nat("line beats max1 width64", fn_line_beats(1, 64, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 70);
        check_nat("line beats max1 width128", fn_line_beats(1, 128, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 67);

        check_nat("line beats max3 width32", fn_line_beats(3, 32, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 108);
        check_nat("line beats max3 width64", fn_line_beats(3, 64, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 70);
        check_nat("line beats max3 width128", fn_line_beats(3, 128, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 67);

        check_nat("line beats max5 width32", fn_line_beats(5, 32, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 140);
        check_nat("line beats max5 width64", fn_line_beats(5, 64, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 102);
        check_nat("line beats max5 width128", fn_line_beats(5, 128, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 67);

        check_nat("line beats max7 width32", fn_line_beats(7, 32, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 172);
        check_nat("line beats max7 width64", fn_line_beats(7, 64, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 102);
        check_nat("line beats max7 width128", fn_line_beats(7, 128, C_ACTIVE_CHIPS, C_STOPS_PER_CHIP), 67);

        report "*** tb_tdc_gpx_width_timing_matrix PASS ***" severity note;
        finish;
        wait;
    end process;

end architecture;
