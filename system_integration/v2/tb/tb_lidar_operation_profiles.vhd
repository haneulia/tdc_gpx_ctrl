library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_operation_50 is
end entity tb_lidar_operation_50;

architecture sim of tb_lidar_operation_50 is
begin
    u_test : entity work.tb_lidar_operation_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 50,
            G_PROC_HALF_PERIOD_PS => 10000
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_operation_150 is
end entity tb_lidar_operation_150;

architecture sim of tb_lidar_operation_150 is
begin
    u_test : entity work.tb_lidar_operation_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_PROC_HALF_PERIOD_PS => 3333
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_operation_200 is
end entity tb_lidar_operation_200;

architecture sim of tb_lidar_operation_200 is
begin
    u_test : entity work.tb_lidar_operation_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_PROC_HALF_PERIOD_PS => 2500
        );
end architecture sim;
