-- ============================================================================
-- 테스트 자산 목적: laser-executor 공통 TB를 150/200 MHz profile로 실행한다.
-- 핵심 검증 계약: 물리 시간 설정이 주파수별 clock 폭으로 바뀌어도 사건 순서는 같다.
-- 관련 RTL/TB: tb_laser_executor, laser_executor.
-- 실행 회귀: scripts/run_v2_laser_executor.ps1
-- 유지보수 주의: profile wrapper에 기능 판단을 넣지 말고 공통 TB assertion을 확장한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;

entity tb_laser_executor_150 is
end entity tb_laser_executor_150;

architecture sim of tb_laser_executor_150 is
begin
    u_tb : entity work.tb_laser_executor
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_PROC_HALF_PERIOD_PS => 3333
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_laser_executor_200 is
end entity tb_laser_executor_200;

architecture sim of tb_laser_executor_200 is
begin
    u_tb : entity work.tb_laser_executor
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_PROC_HALF_PERIOD_PS => 2500
        );
end architecture sim;
