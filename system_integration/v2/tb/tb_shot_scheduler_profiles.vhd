library ieee;
use ieee.std_logic_1164.all;

entity tb_shot_scheduler_150 is
end entity tb_shot_scheduler_150;

architecture sim of tb_shot_scheduler_150 is
begin
    u_tb : entity work.tb_shot_scheduler
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_PROC_HALF_PERIOD_PS => 3333
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_shot_scheduler_200 is
end entity tb_shot_scheduler_200;

architecture sim of tb_shot_scheduler_200 is
begin
    u_tb : entity work.tb_shot_scheduler
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_PROC_HALF_PERIOD_PS => 2500
        );
end architecture sim;
