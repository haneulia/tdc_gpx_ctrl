-- ============================================================================
-- 테스트 자산 목적: operation 안전 TB를 50/150/200 MHz에서 반복한다.
-- 핵심 검증 계약: 시간 환산과 관계없이 명령 우선순위와 fail-safe 출력이 동일하다.
-- 관련 RTL/TB: tb_lidar_operation_subsystem, operation manager 계층.
-- 실행 회귀: scripts/run_v2_operation.ps1
-- 유지보수 주의: 지원 Processing clock 목록 변경 시 이 profile 표도 함께 갱신한다.
-- ============================================================================
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
