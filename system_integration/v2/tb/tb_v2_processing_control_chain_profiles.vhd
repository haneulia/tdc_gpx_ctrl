-- ============================================================================
-- 테스트 자산 목적: Processing control chain을 150/200 MHz에서 반복 검증한다.
-- 핵심 검증 계약: clock profile과 무관하게 B0~B2 end-to-end 사건 identity가 같다.
-- 관련 RTL/TB: tb_v2_processing_control_chain과 operation/Face/scheduler 계층.
-- 실행 회귀: scripts/run_v2_shot_scheduler.ps1
-- 유지보수 주의: profile만 소유하며 end-to-end 기대값은 공통 TB에서 관리한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;

entity tb_v2_processing_control_chain_150 is
end entity tb_v2_processing_control_chain_150;

architecture sim of tb_v2_processing_control_chain_150 is
begin
    u_tb : entity work.tb_v2_processing_control_chain
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_PROC_HALF_PERIOD_PS => 3333
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_v2_processing_control_chain_200 is
end entity tb_v2_processing_control_chain_200;

architecture sim of tb_v2_processing_control_chain_200 is
begin
    u_tb : entity work.tb_v2_processing_control_chain
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_PROC_HALF_PERIOD_PS => 2500
        );
end architecture sim;
