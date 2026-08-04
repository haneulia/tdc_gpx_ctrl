entity tb_motor_position_core_150 is
end entity tb_motor_position_core_150;

architecture sim of tb_motor_position_core_150 is
begin
    u_profile : entity work.tb_motor_position_core
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_PROC_HALF_PERIOD_PS => 3333
        );
end architecture sim;

entity tb_motor_position_core_200 is
end entity tb_motor_position_core_200;

architecture sim of tb_motor_position_core_200 is
begin
    u_profile : entity work.tb_motor_position_core
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_PROC_HALF_PERIOD_PS => 2500
        );
end architecture sim;
