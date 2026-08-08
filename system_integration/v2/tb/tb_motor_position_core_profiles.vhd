-- ============================================================================
-- 테스트 자산 목적: motor-position 공통 TB를 150/200 MHz Processing profile로 실행한다.
-- 핵심 검증 계약: 주파수가 달라도 위치·방향·source identity와 물리 latency 계약이 같다.
-- 관련 RTL/TB: tb_motor_position_core, motor_position_core.
-- 실행 회귀: scripts/run_v2_motor_position.ps1
-- 유지보수 주의: profile 값만 소유하며 기능 assertion은 공통 TB에 유지한다.
-- ============================================================================
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
