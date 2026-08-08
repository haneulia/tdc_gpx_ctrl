-- ============================================================================
-- 테스트 자산 목적: CSR-to-Active 통합 TB를 150/200 및 200/150 MHz로 구체화한다.
-- 핵심 검증 계약: clock 관계가 뒤집혀도 CSR transaction과 Active snapshot이 동일하다.
-- 관련 RTL/TB: tb_lidar_csr_config_subsystem과 unified CSR 계층.
-- 실행 회귀: scripts/run_v2_unified_csr.ps1
-- 유지보수 주의: assertion은 공통 TB에 두고 이 파일은 clock profile만 소유한다.
-- ============================================================================
entity tb_lidar_csr_config_150_200 is
end entity tb_lidar_csr_config_150_200;

architecture sim of tb_lidar_csr_config_150_200 is
begin
    u_profile : entity work.tb_lidar_csr_config_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 150,
            G_TDC_CLK_MHZ         => 200,
            G_PROC_HALF_PERIOD_PS => 3333,
            G_TDC_HALF_PERIOD_PS  => 2500
        );
end architecture sim;

entity tb_lidar_csr_config_200_150 is
end entity tb_lidar_csr_config_200_150;

architecture sim of tb_lidar_csr_config_200_150 is
begin
    u_profile : entity work.tb_lidar_csr_config_subsystem
        generic map (
            G_PROC_CLK_MHZ        => 200,
            G_TDC_CLK_MHZ         => 150,
            G_PROC_HALF_PERIOD_PS => 2500,
            G_TDC_HALF_PERIOD_PS  => 3333
        );
end architecture sim;
