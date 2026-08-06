library ieee;
use ieee.std_logic_1164.all;

-- Processing RUN is a persistent level, not an event pulse. This named CDC
-- boundary synchronizes that level into the TDC bus domain. Shot and STOP
-- events continue to use their acknowledged/FIFO gateways.
entity lidar_gpx_run_enable_cdc is
    port (
        i_tdc_clk         : in  std_logic;
        i_tdc_rst_n       : in  std_logic;
        i_proc_run_enable : in  std_logic;
        o_tdc_run_enable  : out std_logic
    );
end entity lidar_gpx_run_enable_cdc;

architecture rtl of lidar_gpx_run_enable_cdc is

    signal run_meta_r : std_logic := '0';
    signal run_sync_r : std_logic := '0';

    attribute ASYNC_REG : string;
    attribute SHREG_EXTRACT : string;
    attribute ASYNC_REG of run_meta_r : signal is "TRUE";
    attribute ASYNC_REG of run_sync_r : signal is "TRUE";
    attribute SHREG_EXTRACT of run_meta_r : signal is "NO";
    attribute SHREG_EXTRACT of run_sync_r : signal is "NO";

begin

    o_tdc_run_enable <= run_sync_r;

    p_sync : process (i_tdc_clk, i_tdc_rst_n)
    begin
        if i_tdc_rst_n = '0' then
            run_meta_r <= '0';
            run_sync_r <= '0';
        elsif rising_edge(i_tdc_clk) then
            run_meta_r <= i_proc_run_enable;
            run_sync_r <= run_meta_r;
        end if;
    end process p_sync;

end architecture rtl;
