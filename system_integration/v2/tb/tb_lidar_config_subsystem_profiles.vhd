-- ============================================================================
-- 테스트 자산 목적: config subsystem을 150/200 및 200/150 MHz CDC 관계로 실행한다.
-- 핵심 검증 계약: 어느 domain이 빠르더라도 같은 snapshot과 Active version이 적용된다.
-- 관련 RTL/TB: tb_lidar_config_subsystem과 configuration manager 계층.
-- 실행 회귀: scripts/run_v2_config_manager.ps1
-- 유지보수 주의: 이 wrapper의 clock 조합은 routine CDC sign-off 조합과 맞춘다.
-- ============================================================================
entity tb_lidar_config_subsystem_150_200 is
end entity tb_lidar_config_subsystem_150_200;

architecture sim of tb_lidar_config_subsystem_150_200 is
begin
    u_profile : entity work.tb_lidar_config_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_TDC_CLK_MHZ         => 200,
            G_PROC_HALF_PERIOD_PS => 3333,
            G_TDC_HALF_PERIOD_PS  => 2500
        );
end architecture sim;

entity tb_lidar_config_subsystem_200_150 is
end entity tb_lidar_config_subsystem_200_150;

architecture sim of tb_lidar_config_subsystem_200_150 is
begin
    u_profile : entity work.tb_lidar_config_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_TDC_CLK_MHZ         => 150,
            G_PROC_HALF_PERIOD_PS => 2500,
            G_TDC_HALF_PERIOD_PS  => 3333
        );
end architecture sim;
