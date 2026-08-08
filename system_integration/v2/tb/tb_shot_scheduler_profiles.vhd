-- ============================================================================
-- 테스트 자산 목적: shot-scheduler 공통 TB를 150/200 MHz로 구체화한다.
-- 핵심 검증 계약: 주파수별 환산 clock 수가 달라도 같은 Shot 위치 계약을 만족한다.
-- 관련 RTL/TB: tb_shot_scheduler, shot_scheduler.
-- 실행 회귀: scripts/run_v2_shot_scheduler.ps1
-- 유지보수 주의: 기능 시나리오는 공통 TB에서 관리하고 여기서는 profile만 관리한다.
-- ============================================================================
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
