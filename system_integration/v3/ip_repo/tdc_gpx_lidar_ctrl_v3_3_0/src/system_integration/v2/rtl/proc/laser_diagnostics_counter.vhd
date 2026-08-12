library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_event_types_pkg.all;

-- Observation-only B3 diagnostic owner. It cannot gate or delay a shot.
entity laser_diagnostics_counter is
    port (
        i_clk                   : in  std_logic;
        i_rst_n                 : in  std_logic;
        i_clear                 : in  std_logic;
        i_request_drop          : in  std_logic;
        i_fire_done_timeout     : in  std_logic;
        i_operation_abort       : in  std_logic;
        i_unexpected_done       : in  std_logic;
        o_diagnostics           : out laser_diagnostics_t
    );
end entity laser_diagnostics_counter;

architecture rtl of laser_diagnostics_counter is

    signal diagnostics_r : laser_diagnostics_t :=
        C_LASER_DIAGNOSTICS_CLEAR;

begin

    o_diagnostics <= diagnostics_r;

    p_diagnostics : process (i_clk)
        variable next_v : laser_diagnostics_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                diagnostics_r <= C_LASER_DIAGNOSTICS_CLEAR;
            else
                next_v := diagnostics_r;
                next_v.request_drop_pulse       := '0';
                next_v.fire_done_timeout_pulse  := '0';
                next_v.operation_abort_pulse    := '0';
                next_v.unexpected_done_pulse    := '0';

                if i_clear = '1' then
                    next_v := C_LASER_DIAGNOSTICS_CLEAR;
                end if;

                if i_request_drop = '1' then
                    next_v.request_drop_pulse  := '1';
                    next_v.request_drop_sticky := '1';
                    next_v.request_drop_count  :=
                        next_v.request_drop_count + 1;
                end if;
                if i_fire_done_timeout = '1' then
                    next_v.fire_done_timeout_pulse  := '1';
                    next_v.fire_done_timeout_sticky := '1';
                    next_v.fire_done_timeout_count  :=
                        next_v.fire_done_timeout_count + 1;
                end if;
                if i_operation_abort = '1' then
                    next_v.operation_abort_pulse  := '1';
                    next_v.operation_abort_sticky := '1';
                    next_v.operation_abort_count  :=
                        next_v.operation_abort_count + 1;
                end if;
                if i_unexpected_done = '1' then
                    next_v.unexpected_done_pulse  := '1';
                    next_v.unexpected_done_sticky := '1';
                    next_v.unexpected_done_count  :=
                        next_v.unexpected_done_count + 1;
                end if;

                diagnostics_r <= next_v;
            end if;
        end if;
    end process p_diagnostics;

end architecture rtl;
