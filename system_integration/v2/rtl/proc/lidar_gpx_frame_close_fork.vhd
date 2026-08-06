library ieee;
use ieee.std_logic_1164.all;

use work.lidar_gpx_data_pkg.all;

-- Registered one-to-two fork for the ordered Face-close event. Disabled lanes
-- acknowledge no event and therefore cannot hold the active lane pipeline.
entity lidar_gpx_frame_close_fork is
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;
        i_abort : in std_logic;

        i_rise_enable : in std_logic;
        i_fall_enable : in std_logic;

        i_close_event : in  gpx_frame_close_event_t;
        o_close_ready : out std_logic;

        o_rise_close_event : out gpx_frame_close_event_t;
        i_rise_close_ready : in  std_logic;
        o_fall_close_event : out gpx_frame_close_event_t;
        i_fall_close_ready : in  std_logic;

        o_idle : out std_logic
    );
end entity lidar_gpx_frame_close_fork;

architecture rtl of lidar_gpx_frame_close_fork is

    signal rise_event_r : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal fall_event_r : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;

begin

    o_close_ready <= '1' when i_rst_n = '1' and i_abort = '0' and
        rise_event_r.valid = '0' and fall_event_r.valid = '0' else '0';
    o_rise_close_event <= rise_event_r;
    o_fall_close_event <= fall_event_r;
    o_idle <= '1' when rise_event_r.valid = '0' and
        fall_event_r.valid = '0' else '0';

    p_fork : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                rise_event_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
                fall_event_r <= C_GPX_FRAME_CLOSE_EVENT_IDLE;
            else
                if rise_event_r.valid = '1' and
                   i_rise_close_ready = '1' then
                    rise_event_r.valid <= '0';
                end if;
                if fall_event_r.valid = '1' and
                   i_fall_close_ready = '1' then
                    fall_event_r.valid <= '0';
                end if;

                if i_close_event.valid = '1' and o_close_ready = '1' then
                    if i_rise_enable = '1' then
                        rise_event_r <= i_close_event;
                        rise_event_r.valid <= '1';
                    end if;
                    if i_fall_enable = '1' then
                        fall_event_r <= i_close_event;
                        fall_event_r.valid <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_fork;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' then
                if rise_event_r.valid = '1' then
                    assert rise_event_r = i_close_event or
                           i_close_event.valid = '0' or
                           o_close_ready = '0'
                        report "V2-B9-J7-FORK-001 Rise close changed"
                        severity failure;
                end if;
                if fall_event_r.valid = '1' then
                    assert fall_event_r = i_close_event or
                           i_close_event.valid = '0' or
                           o_close_ready = '0'
                        report "V2-B9-J7-FORK-002 Fall close changed"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
