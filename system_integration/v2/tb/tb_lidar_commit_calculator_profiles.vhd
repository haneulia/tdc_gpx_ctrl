-- ============================================================================
-- 테스트 자산 목적: commit-calculator 공통 TB를 150/200 MHz profile로 구체화한다.
-- 핵심 검증 계약: 같은 물리 시간 설정이 Processing 주파수별 clock 수로 정확히 변환된다.
-- 관련 RTL/TB: tb_lidar_commit_calculator, lidar_commit_calculator.
-- 실행 회귀: scripts/run_v2_commit_calculator.ps1
-- 유지보수 주의: 이 파일은 profile wrapper이므로 assertion은 공통 TB에서 관리한다.
-- ============================================================================
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
