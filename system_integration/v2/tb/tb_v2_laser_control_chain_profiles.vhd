-- ============================================================================
-- 테스트 자산 목적: 전체 laser control chain을 150/200 MHz로 반복 검증한다.
-- 핵심 검증 계약: 주파수에 따른 clock 수 차이 외에 B0~B3 물리 사건 의미는 동일하다.
-- 관련 RTL/TB: tb_v2_laser_control_chain과 Processing laser 제어 계층.
-- 실행 회귀: scripts/run_v2_laser_executor.ps1
-- 유지보수 주의: 기능 assertion은 공통 TB에 두고 이 파일은 profile만 소유한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;

entity tb_v2_laser_control_chain_150 is
end entity tb_v2_laser_control_chain_150;

architecture sim of tb_v2_laser_control_chain_150 is
begin
    u_tb : entity work.tb_v2_laser_control_chain
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_PROC_HALF_PERIOD_PS => 3333
        );
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_v2_laser_control_chain_200 is
end entity tb_v2_laser_control_chain_200;

architecture sim of tb_v2_laser_control_chain_200 is
begin
    u_tb : entity work.tb_v2_laser_control_chain
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_PROC_HALF_PERIOD_PS => 2500
        );
end architecture sim;
