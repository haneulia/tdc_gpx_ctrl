-- ============================================================================
-- 테스트 자산 목적: Processing subsystem을 150/200 및 200/150 MHz 관계로 실행한다.
-- 핵심 검증 계약: TDC clock 관계와 무관하게 Processing 사건 및 안전 출력이 동일하다.
-- 관련 RTL/TB: tb_lidar_processing_subsystem, lidar_processing_subsystem.
-- 실행 회귀: scripts/run_v2_processing_subsystem.ps1
-- 유지보수 주의: routine regression의 두 clock 관계를 삭제하거나 완화하지 않는다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_processing_subsystem_150_200 is
end entity tb_lidar_processing_subsystem_150_200;

architecture sim of tb_lidar_processing_subsystem_150_200 is
begin
    u_test : entity work.tb_lidar_processing_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_PROC_HALF_PERIOD_PS => 3333,
            G_TDC_CLK_MHZ         => 200,
            G_TDC_HALF_PERIOD_PS  => 2500
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_processing_subsystem_200_150 is
end entity tb_lidar_processing_subsystem_200_150;

architecture sim of tb_lidar_processing_subsystem_200_150 is
begin
    u_test : entity work.tb_lidar_processing_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_PROC_HALF_PERIOD_PS => 2500,
            G_TDC_CLK_MHZ         => 150,
            G_TDC_HALF_PERIOD_PS  => 3333
        );
end architecture sim;
