-- ============================================================================
-- 테스트 자산 목적: 하나의 Face-close를 Rise/Fall Footer 경로에 손실 없이 복제한다.
-- 핵심 검증 계약: lane별 독립 backpressure, exactly-once 전달과 global 완료 ACK이다.
-- 관련 RTL: lidar_gpx_frame_close_fork.
-- 실행 회귀: scripts/run_v2_gpx_face_footer.ps1
-- 유지보수 주의: 한 lane disable 시에도 다른 lane 완료와 source ready가 교착되지 않아야 한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_event_types_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_frame_close_fork is
    generic (
        G_CLK_MHZ : positive := 150
    );
end entity tb_lidar_gpx_frame_close_fork;

architecture sim of tb_lidar_gpx_frame_close_fork is

    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_run : std_logic := '0';
    signal rise_enable : std_logic := '0';
    signal fall_enable : std_logic := '0';
    signal close_in : gpx_frame_close_event_t :=
        C_GPX_FRAME_CLOSE_EVENT_IDLE;
    signal close_ready : std_logic;
    signal rise_close : gpx_frame_close_event_t;
    signal rise_ready : std_logic := '0';
    signal fall_close : gpx_frame_close_event_t;
    signal fall_ready : std_logic := '0';
    signal idle : std_logic;

    function fn_close(value : natural) return gpx_frame_close_event_t is
        variable result : gpx_frame_close_event_t :=
            C_GPX_FRAME_CLOSE_EVENT_IDLE;
    begin
        result.valid := '1';
        result.face_frame_id := to_unsigned(value, 32);
        result.face_index := to_unsigned(value mod 5, 3);
        result.active_version := to_unsigned(7, 16);
        result.columns_per_face := to_unsigned(18, 16);
        return result;
    end function fn_close;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_dut : entity work.lidar_gpx_frame_close_fork
        port map (
            i_clk              => clk,
            i_rst_n            => rst_n,
            i_abort            => abort_run,
            i_rise_enable      => rise_enable,
            i_fall_enable      => fall_enable,
            i_close_event      => close_in,
            o_close_ready      => close_ready,
            o_rise_close_event => rise_close,
            i_rise_close_ready => rise_ready,
            o_fall_close_event => fall_close,
            i_fall_close_ready => fall_ready,
            o_idle             => idle
        );

    p_test : process
        procedure send_close(
            constant value : in gpx_frame_close_event_t
        ) is
        begin
            close_in <= value;
            loop
                wait until rising_edge(clk);
                exit when close_ready = '1';
            end loop;
            close_in.valid <= '0';
        end procedure send_close;
    begin
        rst_n <= '0';
        wait for 6 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';

        rise_enable <= '1';
        fall_enable <= '1';
        send_close(fn_close(11));
        wait until falling_edge(clk);
        assert rise_close.valid = '1' and fall_close.valid = '1' and
               rise_close.face_frame_id = to_unsigned(11, 32) and
               fall_close = rise_close and close_ready = '0'
            report "V2-B9-J7-FORK-TB dual-lane capture mismatch"
            severity failure;

        rise_ready <= '1';
        wait until rising_edge(clk);
        rise_ready <= '0';
        wait until falling_edge(clk);
        assert rise_close.valid = '0' and fall_close.valid = '1' and
               fall_close.face_frame_id = to_unsigned(11, 32) and
               close_ready = '0'
            report "V2-B9-J7-FORK-TB independent retirement mismatch"
            severity failure;

        fall_ready <= '1';
        wait until rising_edge(clk);
        fall_ready <= '0';
        wait until falling_edge(clk);
        assert idle = '1' and close_ready = '1'
            report "V2-B9-J7-FORK-TB fork did not return idle"
            severity failure;

        fall_enable <= '0';
        send_close(fn_close(12));
        wait until falling_edge(clk);
        assert rise_close.valid = '1' and fall_close.valid = '0'
            report "V2-B9-J7-FORK-TB disabled Fall lane received close"
            severity failure;
        rise_ready <= '1';
        wait until rising_edge(clk);
        rise_ready <= '0';

        rise_enable <= '0';
        send_close(fn_close(13));
        wait until falling_edge(clk);
        assert idle = '1' and rise_close.valid = '0' and
               fall_close.valid = '0'
            report "V2-B9-J7-FORK-TB disabled lanes retained close"
            severity failure;

        rise_enable <= '1';
        fall_enable <= '1';
        send_close(fn_close(14));
        wait until falling_edge(clk);
        assert idle = '0'
            report "V2-B9-J7-FORK-TB abort setup did not capture"
            severity failure;
        abort_run <= '1';
        wait until rising_edge(clk);
        abort_run <= '0';
        wait until falling_edge(clk);
        assert idle = '1' and rise_close.valid = '0' and
               fall_close.valid = '0'
            report "V2-B9-J7-FORK-TB abort did not clear pending closes"
            severity failure;

        report "LIDAR_V2_GPX_FRAME_CLOSE_FORK_PASS proc_mhz=" &
            integer'image(G_CLK_MHZ) severity note;
        finish;
        wait;
    end process p_test;

end architecture sim;

entity tb_lidar_gpx_frame_close_fork_150 is end entity;
architecture sim of tb_lidar_gpx_frame_close_fork_150 is begin
    u : entity work.tb_lidar_gpx_frame_close_fork
        generic map (G_CLK_MHZ => 150);
end architecture;

entity tb_lidar_gpx_frame_close_fork_200 is end entity;
architecture sim of tb_lidar_gpx_frame_close_fork_200 is begin
    u : entity work.tb_lidar_gpx_frame_close_fork
        generic map (G_CLK_MHZ => 200);
end architecture;
