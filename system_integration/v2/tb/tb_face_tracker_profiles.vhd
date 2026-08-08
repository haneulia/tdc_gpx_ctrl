-- ============================================================================
-- 테스트 자산 목적: face-tracker 공통 TB를 150/200 MHz와 Face 1~5 조합으로 펼친다.
-- 핵심 검증 계약: build-time Face 수마다 center/boundary/wrap 계산이 동일 규칙을 따른다.
-- 관련 RTL/TB: tb_face_tracker, face_tracker.
-- 실행 회귀: scripts/run_v2_face_tracker.ps1
-- 유지보수 주의: g_N_FACES 범위가 바뀌면 wrapper와 XGUI 허용 범위를 함께 바꾼다.
-- ============================================================================
entity tb_face_tracker_150_f1 is end entity;
architecture sim of tb_face_tracker_150_f1 is begin
    u : entity work.tb_face_tracker generic map (150, 3333, 1);
end architecture;

entity tb_face_tracker_150_f2 is end entity;
architecture sim of tb_face_tracker_150_f2 is begin
    u : entity work.tb_face_tracker generic map (150, 3333, 2);
end architecture;

entity tb_face_tracker_150_f3 is end entity;
architecture sim of tb_face_tracker_150_f3 is begin
    u : entity work.tb_face_tracker generic map (150, 3333, 3);
end architecture;

entity tb_face_tracker_150_f4 is end entity;
architecture sim of tb_face_tracker_150_f4 is begin
    u : entity work.tb_face_tracker generic map (150, 3333, 4);
end architecture;

entity tb_face_tracker_150_f5 is end entity;
architecture sim of tb_face_tracker_150_f5 is begin
    u : entity work.tb_face_tracker generic map (150, 3333, 5);
end architecture;

entity tb_face_tracker_200_f1 is end entity;
architecture sim of tb_face_tracker_200_f1 is begin
    u : entity work.tb_face_tracker generic map (200, 2500, 1);
end architecture;

entity tb_face_tracker_200_f2 is end entity;
architecture sim of tb_face_tracker_200_f2 is begin
    u : entity work.tb_face_tracker generic map (200, 2500, 2);
end architecture;

entity tb_face_tracker_200_f3 is end entity;
architecture sim of tb_face_tracker_200_f3 is begin
    u : entity work.tb_face_tracker generic map (200, 2500, 3);
end architecture;

entity tb_face_tracker_200_f4 is end entity;
architecture sim of tb_face_tracker_200_f4 is begin
    u : entity work.tb_face_tracker generic map (200, 2500, 4);
end architecture;

entity tb_face_tracker_200_f5 is end entity;
architecture sim of tb_face_tracker_200_f5 is begin
    u : entity work.tb_face_tracker generic map (200, 2500, 5);
end architecture;
