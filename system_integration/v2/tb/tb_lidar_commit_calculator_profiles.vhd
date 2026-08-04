entity tb_lidar_commit_calculator_150 is
end entity tb_lidar_commit_calculator_150;

architecture sim of tb_lidar_commit_calculator_150 is
begin
    u_profile : entity work.tb_lidar_commit_calculator
        generic map (
            G_HALF_PERIOD_PS => 3333
        );
end architecture sim;

entity tb_lidar_commit_calculator_200 is
end entity tb_lidar_commit_calculator_200;

architecture sim of tb_lidar_commit_calculator_200 is
begin
    u_profile : entity work.tb_lidar_commit_calculator
        generic map (
            G_HALF_PERIOD_PS => 2500
        );
end architecture sim;
