-- ============================================================================
-- 테스트 자산 목적: Processing↔TDC typed command/result gateway의 SYNC/ASYNC 경로를 검증한다.
-- 핵심 검증 계약: ready/valid 보존, 순서, reset 복구, routine/release clock 전달이다.
-- 관련 RTL: lidar_gpx_shot_gateway, lidar_gpx_result_gateway.
-- 실행 회귀: scripts/run_v2_gpx_event_gateway.ps1
-- 유지보수 주의: payload 필드 추가 시 stall 중 안정성과 양 clock 관계를 함께 검사한다.
--                  release gate는 150/150 shared, 200/50, 50/200, 150/100 MHz다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_event_pkg.all;

entity tb_lidar_gpx_event_gateway is
    generic (
        G_PROC_CLK_MHZ : positive := 150;
        G_TDC_CLK_MHZ  : positive := 200;
        G_CLOCK_MODE   : stream_clock_mode_t := STREAM_CLOCK_ASYNC
    );
end entity tb_lidar_gpx_event_gateway;

architecture sim of tb_lidar_gpx_event_gateway is

    constant C_TRANSFER_COUNT : positive := 64;
    constant C_PROC_PERIOD : time := 1 us / G_PROC_CLK_MHZ;
    constant C_TDC_PERIOD  : time := 1 us / G_TDC_CLK_MHZ;

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_PROC_CLK_MHZ;
        result.tdc_clk_mhz := G_TDC_CLK_MHZ;
        result.stream_clock_mode := G_CLOCK_MODE;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;

    signal proc_clk : std_logic := '0';
    signal tdc_clk  : std_logic := '0';
    signal sync_clk : std_logic := '0';
    signal proc_rst_n : std_logic := '0';
    signal tdc_rst_n  : std_logic := '0';
    signal done : boolean := false;

    signal proc_shot : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal proc_shot_ready : std_logic;
    signal tdc_shot : shot_start_event_t;
    signal tdc_shot_ready : std_logic := '0';
    signal shot_reset_busy : std_logic;

    signal tdc_result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal tdc_result_ready : std_logic;
    signal proc_result : gpx_raw_event_t;
    signal proc_result_ready : std_logic := '0';
    signal result_reset_busy : std_logic;

    signal shot_sent : natural range 0 to C_TRANSFER_COUNT := 0;
    signal shot_received : natural range 0 to C_TRANSFER_COUNT := 0;
    signal result_sent : natural range 0 to C_TRANSFER_COUNT := 0;
    signal result_received : natural range 0 to C_TRANSFER_COUNT := 0;

    function fn_make_shot(index : natural) return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(
            index mod C_BUILD_CONFIG.num_faces,
            result.request.face_index'length);
        result.request.position := to_unsigned(
            (index * 37 + 11) mod (2 ** C_POSITION_WIDTH),
            result.request.position'length);
        if index mod 2 = 0 then
            result.request.direction := DIRECTION_CW;
        else
            result.request.direction := DIRECTION_CCW;
        end if;
        result.request.shot_index := to_unsigned(
            index * 3 + 1, result.request.shot_index'length);
        if index mod 9 = 8 then
            result.request.last_in_face := '1';
        end if;
        if index mod 3 = 1 then
            result.request.source_sim := '1';
        end if;
        result.request.source_latency_clks := to_unsigned(
            index mod 251, result.request.source_latency_clks'length);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(
            index + 16, result.request.active_version'length);
        result.fire_to_t0_clks := to_unsigned(
            index * 13 + 7, result.fire_to_t0_clks'length);
        result.t0_timestamp_ticks := x"1234567800000000";
        result.t0_timestamp_ticks := result.t0_timestamp_ticks +
            to_unsigned(index * 257 + 19,
                result.t0_timestamp_ticks'length);
        result.t0_timestamp_valid := '1';
        if index mod 5 = 0 then
            result.t0_time_sync_valid := '1';
        end if;
        return result;
    end function fn_make_shot;

    function fn_make_result(index : natural) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := gpx_raw_event_kind_t'val(index mod 4);
        result.chip_index := to_unsigned(
            index mod C_BUILD_CONFIG.num_chips,
            result.chip_index'length);
        if index mod 2 = 1 then
            result.ififo_id := '1';
        end if;
        result.raw_word := std_logic_vector(to_unsigned(
            index * 65537 + 16#1234#, result.raw_word'length));
        if result.kind = GPX_RAW_DRAIN_DONE and index mod 8 = 2 then
            result.faulted := '1';
        end if;
        result.timeout_cause := std_logic_vector(to_unsigned(
            index mod 8, result.timeout_cause'length));
        result.shot_context := fn_make_shot(index);
        result.chip_shot_seq := to_unsigned(
            index + 1, result.chip_shot_seq'length);
        return result;
    end function fn_make_result;

begin

    assert fn_validate_build_config(C_BUILD_CONFIG) = CFG_OK
        report "V2-GPX-CDC-TB invalid profile"
        severity failure;

    gen_sync_clock : if G_CLOCK_MODE = STREAM_CLOCK_SYNC generate
    begin
        sync_clk <= not sync_clk after C_PROC_PERIOD / 2 when not done;
        proc_clk <= sync_clk;
        tdc_clk  <= sync_clk;
    end generate gen_sync_clock;

    gen_async_clocks : if G_CLOCK_MODE = STREAM_CLOCK_ASYNC generate
    begin
        proc_clk <= not proc_clk after C_PROC_PERIOD / 2 when not done;
        tdc_clk  <= not tdc_clk after C_TDC_PERIOD / 2 when not done;
    end generate gen_async_clocks;

    u_shot_gateway : entity work.lidar_gpx_shot_gateway
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_FIFO_DEPTH   => 16
        )
        port map (
            i_proc_clk   => proc_clk,
            i_proc_rst_n => proc_rst_n,
            i_proc_shot  => proc_shot,
            o_proc_ready => proc_shot_ready,
            i_tdc_clk    => tdc_clk,
            i_tdc_rst_n  => tdc_rst_n,
            o_tdc_shot   => tdc_shot,
            i_tdc_ready  => tdc_shot_ready,
            o_proc_reset_busy => open,
            o_tdc_reset_busy  => open,
            o_reset_busy => shot_reset_busy
        );

    u_result_gateway : entity work.lidar_gpx_result_gateway
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_FIFO_DEPTH   => 16
        )
        port map (
            i_tdc_clk     => tdc_clk,
            i_tdc_rst_n   => tdc_rst_n,
            i_tdc_result  => tdc_result,
            o_tdc_ready   => tdc_result_ready,
            i_proc_clk    => proc_clk,
            i_proc_rst_n  => proc_rst_n,
            o_proc_result => proc_result,
            i_proc_ready  => proc_result_ready,
            o_tdc_reset_busy  => open,
            o_proc_reset_busy => open,
            o_reset_busy  => result_reset_busy
        );

    p_reset : process
    begin
        proc_rst_n <= '0';
        tdc_rst_n <= '0';
        wait for 120 ns;
        proc_rst_n <= '1';
        tdc_rst_n <= '1';
        wait;
    end process p_reset;

    p_shot_source : process (proc_clk, proc_rst_n)
    begin
        if proc_rst_n = '0' then
            proc_shot <= C_SHOT_START_EVENT_IDLE;
            shot_sent <= 0;
        elsif rising_edge(proc_clk) then
            if shot_reset_busy = '1' then
                proc_shot <= C_SHOT_START_EVENT_IDLE;
            elsif proc_shot.valid = '0' then
                if shot_sent < C_TRANSFER_COUNT then
                    proc_shot <= fn_make_shot(shot_sent);
                end if;
            elsif proc_shot_ready = '1' then
                if shot_sent + 1 < C_TRANSFER_COUNT then
                    proc_shot <= fn_make_shot(shot_sent + 1);
                else
                    proc_shot <= C_SHOT_START_EVENT_IDLE;
                end if;
                shot_sent <= shot_sent + 1;
            end if;
        end if;
    end process p_shot_source;

    p_result_source : process (tdc_clk, tdc_rst_n)
    begin
        if tdc_rst_n = '0' then
            tdc_result <= C_GPX_RAW_EVENT_IDLE;
            result_sent <= 0;
        elsif rising_edge(tdc_clk) then
            if result_reset_busy = '1' then
                tdc_result <= C_GPX_RAW_EVENT_IDLE;
            elsif tdc_result.valid = '0' then
                if result_sent < C_TRANSFER_COUNT then
                    tdc_result <= fn_make_result(result_sent);
                end if;
            elsif tdc_result_ready = '1' then
                if result_sent + 1 < C_TRANSFER_COUNT then
                    tdc_result <= fn_make_result(result_sent + 1);
                else
                    tdc_result <= C_GPX_RAW_EVENT_IDLE;
                end if;
                result_sent <= result_sent + 1;
            end if;
        end if;
    end process p_result_source;

    p_tdc_ready : process (tdc_clk, tdc_rst_n)
        variable cycle : natural := 0;
    begin
        if tdc_rst_n = '0' then
            cycle := 0;
            tdc_shot_ready <= '0';
        elsif rising_edge(tdc_clk) then
            cycle := cycle + 1;
            if shot_reset_busy = '0' and cycle mod 7 /= 2
               and cycle mod 7 /= 3 then
                tdc_shot_ready <= '1';
            else
                tdc_shot_ready <= '0';
            end if;
        end if;
    end process p_tdc_ready;

    p_proc_ready : process (proc_clk, proc_rst_n)
        variable cycle : natural := 0;
    begin
        if proc_rst_n = '0' then
            cycle := 0;
            proc_result_ready <= '0';
        elsif rising_edge(proc_clk) then
            cycle := cycle + 1;
            if result_reset_busy = '0' and cycle mod 11 /= 4
               and cycle mod 11 /= 5 and cycle mod 11 /= 6 then
                proc_result_ready <= '1';
            else
                proc_result_ready <= '0';
            end if;
        end if;
    end process p_proc_ready;

    p_shot_scoreboard : process (tdc_clk, tdc_rst_n)
        variable expected : shot_start_event_t;
    begin
        if tdc_rst_n = '0' then
            shot_received <= 0;
        elsif rising_edge(tdc_clk) then
            if tdc_shot.valid = '1' and tdc_shot_ready = '1' then
                expected := fn_make_shot(shot_received);
                assert tdc_shot = expected
                    report "V2-GPX-CDC shot payload/order mismatch"
                    severity failure;
                shot_received <= shot_received + 1;
            end if;
        end if;
    end process p_shot_scoreboard;

    p_result_scoreboard : process (proc_clk, proc_rst_n)
        variable expected : gpx_raw_event_t;
    begin
        if proc_rst_n = '0' then
            result_received <= 0;
        elsif rising_edge(proc_clk) then
            if proc_result.valid = '1' and proc_result_ready = '1' then
                expected := fn_make_result(result_received);
                assert proc_result = expected
                    report "V2-GPX-CDC result payload/order mismatch"
                    severity failure;
                result_received <= result_received + 1;
            end if;
        end if;
    end process p_result_scoreboard;

    p_finish : process
    begin
        wait until shot_received = C_TRANSFER_COUNT and
            result_received = C_TRANSFER_COUNT;
        wait for 20 * C_PROC_PERIOD;
        assert shot_sent = C_TRANSFER_COUNT and
               result_sent = C_TRANSFER_COUNT
            report "V2-GPX-CDC source transfer count mismatch"
            severity failure;
        report "LIDAR_V2_GPX_EVENT_GATEWAY_PASS proc_mhz=" &
            integer'image(G_PROC_CLK_MHZ) & " tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ) & " mode=" &
            stream_clock_mode_t'image(G_CLOCK_MODE)
            severity note;
        done <= true;
        stop;
        wait;
    end process p_finish;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_build_pkg.all;

entity tb_lidar_gpx_event_gateway_async_150_200 is
end entity;

architecture sim of tb_lidar_gpx_event_gateway_async_150_200 is
begin
    u_test : entity work.tb_lidar_gpx_event_gateway
        generic map (
            G_PROC_CLK_MHZ => 150,
            G_TDC_CLK_MHZ  => 200,
            G_CLOCK_MODE   => STREAM_CLOCK_ASYNC
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_build_pkg.all;

entity tb_lidar_gpx_event_gateway_async_200_150 is
end entity;

architecture sim of tb_lidar_gpx_event_gateway_async_200_150 is
begin
    u_test : entity work.tb_lidar_gpx_event_gateway
        generic map (
            G_PROC_CLK_MHZ => 200,
            G_TDC_CLK_MHZ  => 150,
            G_CLOCK_MODE   => STREAM_CLOCK_ASYNC
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_build_pkg.all;

entity tb_lidar_gpx_event_gateway_sync_150 is
end entity;

architecture sim of tb_lidar_gpx_event_gateway_sync_150 is
begin
    u_test : entity work.tb_lidar_gpx_event_gateway
        generic map (
            G_PROC_CLK_MHZ => 150,
            G_TDC_CLK_MHZ  => 150,
            G_CLOCK_MODE   => STREAM_CLOCK_SYNC
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_build_pkg.all;

entity tb_lidar_gpx_event_gateway_async_200_50 is
end entity;

architecture sim of tb_lidar_gpx_event_gateway_async_200_50 is
begin
    u_test : entity work.tb_lidar_gpx_event_gateway
        generic map (
            G_PROC_CLK_MHZ => 200,
            G_TDC_CLK_MHZ  => 50,
            G_CLOCK_MODE   => STREAM_CLOCK_ASYNC
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_build_pkg.all;

entity tb_lidar_gpx_event_gateway_async_50_200 is
end entity;

architecture sim of tb_lidar_gpx_event_gateway_async_50_200 is
begin
    u_test : entity work.tb_lidar_gpx_event_gateway
        generic map (
            G_PROC_CLK_MHZ => 50,
            G_TDC_CLK_MHZ  => 200,
            G_CLOCK_MODE   => STREAM_CLOCK_ASYNC
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use work.lidar_build_pkg.all;

entity tb_lidar_gpx_event_gateway_async_150_100 is
end entity;

architecture sim of tb_lidar_gpx_event_gateway_async_150_100 is
begin
    u_test : entity work.tb_lidar_gpx_event_gateway
        generic map (
            G_PROC_CLK_MHZ => 150,
            G_TDC_CLK_MHZ  => 100,
            G_CLOCK_MODE   => STREAM_CLOCK_ASYNC
        );
end architecture;
