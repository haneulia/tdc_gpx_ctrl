-- ============================================================================
-- 테스트 자산 목적: typed GPX bus wrapper와 검증된 v1 물리 bus FSM의 등가성을 검증한다.
-- 핵심 검증 계약: pin/phase/28-bit data, BUS_TICKS, timeout과 response identity가 일치한다.
-- 관련 RTL: lidar_gpx_bus_engine, tdc_gpx_chip_ctrl 및 물리 bus 하위 RTL.
-- 실행 회귀: scripts/run_v2_gpx_bus.ps1
-- 유지보수 주의: 물리 FSM 변경 시 이전 oracle과 cycle/pin 비교를 먼저 갱신한다.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_pkg.all;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_lidar_gpx_bus_engine is
    generic (
        G_TDC_CLK_MHZ : positive := 200;
        G_OEN_MODE    : string := "DYNAMIC_CONNECTED"
    );
end entity tb_lidar_gpx_bus_engine;

architecture sim of tb_lidar_gpx_bus_engine is

    constant C_CLK_PERIOD : time := 1 us / G_TDC_CLK_MHZ;
    constant C_READ_MIN_CLKS : positive := fn_time_ns_to_clks_ceil(
        c_DEFAULT_BUS_READ_PERIOD_MIN_TIME_NS,
        G_TDC_CLK_MHZ);
    constant C_READ_WORD : gpx_bus_data_t :=
        std_logic_vector(to_unsigned(16#5A12345#, C_GPX_BUS_DATA_WIDTH));

    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal done  : boolean := false;
    signal tick_en : std_logic := '0';
    signal timing : gpx_bus_timing_t := C_GPX_BUS_TIMING_DEFAULT;
    signal request : gpx_bus_request_t := C_GPX_BUS_REQUEST_IDLE;
    signal response_ready : std_logic := '1';

    signal ef1_pin : std_logic := '1';
    signal ef2_pin : std_logic := '1';
    signal lf1_pin : std_logic := '0';
    signal lf2_pin : std_logic := '0';
    signal irflag_pin : std_logic := '0';
    signal errflag_pin : std_logic := '0';

    signal dut_busy : std_logic;
    signal dut_pending : std_logic;
    signal dut_adr : gpx_bus_address_t;
    signal dut_csn : std_logic;
    signal dut_rdn : std_logic;
    signal dut_wrn : std_logic;
    signal dut_oen : std_logic;
    signal dut_d_in : gpx_bus_data_t;
    signal dut_d_out : gpx_bus_data_t;
    signal dut_d_tri : std_logic;
    signal dut_response : gpx_bus_response_t;
    signal dut_status : gpx_pin_status_t;
    signal dut_effective_ticks : unsigned(2 downto 0);

    signal ref_busy : std_logic;
    signal ref_pending : std_logic;
    signal ref_adr : gpx_bus_address_t;
    signal ref_csn : std_logic;
    signal ref_rdn : std_logic;
    signal ref_wrn : std_logic;
    signal ref_oen : std_logic;
    signal ref_d_in : gpx_bus_data_t;
    signal ref_d_out : gpx_bus_data_t;
    signal ref_d_tri : std_logic;
    signal ref_response_valid : std_logic;
    signal ref_response_data : t_bus_rsp_tdata;
    signal ref_response_keep : std_logic_vector(3 downto 0);
    signal ref_response_user : t_bus_rsp_tuser;
    signal ref_ef1 : std_logic;
    signal ref_ef2 : std_logic;
    signal ref_lf1 : std_logic;
    signal ref_lf2 : std_logic;
    signal ref_irflag : std_logic;
    signal ref_errflag : std_logic;

    signal response_count : natural := 0;

    function fn_expected_ticks(
        timing_value : gpx_bus_timing_t
    ) return unsigned is
        variable requested : natural;
        variable minimum   : natural;
    begin
        requested := to_integer(timing_value.ticks);
        minimum := fn_bus_min_ticks_for_capture(
            to_integer(timing_value.clock_div),
            C_READ_MIN_CLKS);
        if requested < minimum then
            return to_unsigned(minimum, 3);
        else
            return timing_value.ticks;
        end if;
    end function fn_expected_ticks;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not done;

    dut_d_in <= C_READ_WORD when dut_csn = '0' and dut_rdn = '0'
        else (others => '0');
    ref_d_in <= C_READ_WORD when ref_csn = '0' and ref_rdn = '0'
        else (others => '0');

    p_tick : process (clk)
        variable count_v : natural range 0 to 63 := 0;
        variable div_v   : natural;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                count_v := 0;
                tick_en <= '0';
            else
                div_v := to_integer(timing.clock_div);
                if div_v <= 1 then
                    count_v := 0;
                    tick_en <= '1';
                elsif count_v = div_v - 1 then
                    count_v := 0;
                    tick_en <= '1';
                else
                    count_v := count_v + 1;
                    tick_en <= '0';
                end if;
            end if;
        end if;
    end process p_tick;

    p_count : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                response_count <= 0;
            elsif dut_response.valid = '1' and response_ready = '1' then
                response_count <= response_count + 1;
            end if;
        end if;
    end process p_count;

    p_lockstep : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '1' then
                assert dut_busy = ref_busy and dut_pending = ref_pending
                    report "V2-GPX-TB busy/pending mismatch"
                    severity failure;
                assert dut_adr = ref_adr and dut_csn = ref_csn and
                       dut_rdn = ref_rdn and dut_wrn = ref_wrn and
                       dut_oen = ref_oen and dut_d_out = ref_d_out and
                       dut_d_tri = ref_d_tri
                    report "V2-GPX-TB physical pin mismatch"
                    severity failure;
                assert dut_response.valid = ref_response_valid and
                       dut_response.read_data =
                           ref_response_data(C_GPX_BUS_DATA_WIDTH - 1 downto 0) and
                       dut_response.write_ack = ref_response_user(0) and
                       dut_response.address = ref_response_user(4 downto 1)
                    report "V2-GPX-TB response mismatch"
                    severity failure;
                assert dut_status.ef1 = ref_ef1 and
                       dut_status.ef2 = ref_ef2 and
                       dut_status.lf1 = ref_lf1 and
                       dut_status.lf2 = ref_lf2 and
                       dut_status.irflag = ref_irflag and
                       dut_status.errflag = ref_errflag
                    report "V2-GPX-TB synchronized status mismatch"
                    severity failure;
                assert dut_effective_ticks = fn_expected_ticks(timing)
                    report "V2-GPX-TB effective tick diagnostic mismatch"
                    severity failure;
            end if;
        end if;
    end process p_lockstep;

    u_dut : entity work.lidar_gpx_bus_engine
        generic map (
            G_TDC_CLK_MHZ => G_TDC_CLK_MHZ,
            G_OEN_MODE    => G_OEN_MODE
        )
        port map (
            i_clk              => clk,
            i_rst_n            => rst_n,
            i_tick_en          => tick_en,
            i_timing           => timing,
            i_request          => request,
            o_busy             => dut_busy,
            o_response_pending => dut_pending,
            o_adr              => dut_adr,
            o_csn              => dut_csn,
            o_rdn              => dut_rdn,
            o_wrn              => dut_wrn,
            o_oen              => dut_oen,
            i_d                => dut_d_in,
            o_d                => dut_d_out,
            o_d_tri            => dut_d_tri,
            i_ef1              => ef1_pin,
            i_ef2              => ef2_pin,
            i_lf1              => lf1_pin,
            i_lf2              => lf2_pin,
            i_irflag           => irflag_pin,
            i_errflag          => errflag_pin,
            o_response         => dut_response,
            i_response_ready   => response_ready,
            o_status           => dut_status,
            o_effective_ticks  => dut_effective_ticks
        );

    u_reference : entity work.tdc_gpx_bus_phy
        generic map (
            g_BUS_DATA_WIDTH           => C_GPX_BUS_DATA_WIDTH,
            g_OEN_MODE                 => G_OEN_MODE,
            g_BUS_READ_PERIOD_MIN_CLKS => C_READ_MIN_CLKS
        )
        port map (
            i_clk           => clk,
            i_rst_n         => rst_n,
            i_tick_en       => tick_en,
            i_bus_ticks     => timing.ticks,
            i_bus_clk_div   => timing.clock_div,
            i_req_valid     => request.valid,
            i_req_rw        => request.write,
            i_req_addr      => request.address,
            i_req_wdata     => request.write_data,
            i_oen_permanent => request.oen_permanent,
            i_req_burst     => request.burst,
            o_busy          => ref_busy,
            o_rsp_pending   => ref_pending,
            o_adr           => ref_adr,
            o_csn           => ref_csn,
            o_rdn           => ref_rdn,
            o_wrn           => ref_wrn,
            o_oen           => ref_oen,
            i_d             => ref_d_in,
            o_d             => ref_d_out,
            o_d_tri         => ref_d_tri,
            i_ef1_pin       => ef1_pin,
            i_ef2_pin       => ef2_pin,
            i_lf1_pin       => lf1_pin,
            i_lf2_pin       => lf2_pin,
            i_irflag_pin    => irflag_pin,
            i_errflag_pin   => errflag_pin,
            o_m_axis_tvalid => ref_response_valid,
            o_m_axis_tdata  => ref_response_data,
            o_m_axis_tkeep  => ref_response_keep,
            o_m_axis_tuser  => ref_response_user,
            i_m_axis_tready => response_ready,
            o_ef1_sync      => ref_ef1,
            o_ef2_sync      => ref_ef2,
            o_lf1_sync      => ref_lf1,
            o_lf2_sync      => ref_lf2,
            o_irflag_sync   => ref_irflag,
            o_errflag_sync  => ref_errflag
        );

    p_stimulus : process
        procedure wait_clocks(count : positive) is
        begin
            for iteration in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure wait_response is
        begin
            for timeout in 0 to 500 loop
                wait until rising_edge(clk);
                wait for 1 ps;
                if dut_response.valid = '1' then
                    return;
                end if;
            end loop;
            assert false report "V2-GPX-TB response timeout" severity failure;
        end procedure wait_response;

        procedure wait_idle is
        begin
            for timeout in 0 to 500 loop
                wait until rising_edge(clk);
                wait for 1 ps;
                if dut_busy = '0' and dut_pending = '0' then
                    return;
                end if;
            end loop;
            assert false report "V2-GPX-TB idle timeout" severity failure;
        end procedure wait_idle;

        variable count_before : natural;
        variable held_response : gpx_bus_response_t;
    begin
        rst_n <= '0';
        wait_clocks(6);
        rst_n <= '1';
        wait_clocks(4);

        ef1_pin <= '0';
        ef2_pin <= '0';
        lf1_pin <= '1';
        lf2_pin <= '1';
        irflag_pin <= '1';
        errflag_pin <= '1';
        wait_clocks(3);
        assert dut_status = (
            ef1 => '0', ef2 => '0', lf1 => '1', lf2 => '1',
            irflag => '1', errflag => '1')
            report "V2-GPX-TB status synchronizer value mismatch"
            severity failure;

        request <= C_GPX_BUS_REQUEST_IDLE;
        request.valid   <= '1';
        request.address <= c_TDC_REG8_IFIFO1;
        wait_response;
        assert dut_response.read_data = C_READ_WORD and
               dut_response.write_ack = '0' and
               dut_response.address = c_TDC_REG8_IFIFO1
            report "V2-GPX-TB typed READ response mismatch"
            severity failure;
        request <= C_GPX_BUS_REQUEST_IDLE;
        wait_idle;

        count_before := response_count;
        request <= C_GPX_BUS_REQUEST_IDLE;
        request.valid      <= '1';
        request.write      <= '1';
        request.address    <= x"3";
        request.write_data <= std_logic_vector(
            to_unsigned(16#13579BD#, C_GPX_BUS_DATA_WIDTH));
        wait_response;
        wait_clocks(20);
        assert response_count = count_before + 1
            report "V2-GPX-TB held valid replayed a transaction"
            severity failure;
        request <= C_GPX_BUS_REQUEST_IDLE;
        wait_idle;

        response_ready <= '0';
        request <= C_GPX_BUS_REQUEST_IDLE;
        request.valid   <= '1';
        request.address <= c_TDC_REG9_IFIFO2;
        wait_response;
        held_response := dut_response;
        wait_clocks(6);
        assert dut_response = held_response
            report "V2-GPX-TB response changed while stalled"
            severity failure;
        response_ready <= '1';
        wait_clocks(1);
        request <= C_GPX_BUS_REQUEST_IDLE;
        wait_idle;

        timing.clock_div <= to_unsigned(1, 6);
        timing.ticks <= to_unsigned(4, 3);
        wait_clocks(3);
        assert dut_effective_ticks = fn_expected_ticks(timing)
            report "V2-GPX-TB clamp result mismatch"
            severity failure;
        request <= C_GPX_BUS_REQUEST_IDLE;
        request.valid   <= '1';
        request.address <= c_TDC_REG8_IFIFO1;
        wait_response;
        request <= C_GPX_BUS_REQUEST_IDLE;
        wait_idle;

        timing <= C_GPX_BUS_TIMING_DEFAULT;
        wait_clocks(3);
        count_before := response_count;
        request <= C_GPX_BUS_REQUEST_IDLE;
        request.valid         <= '1';
        request.address       <= c_TDC_REG8_IFIFO1;
        request.oen_permanent <= '1';
        request.burst         <= '1';
        for timeout in 0 to 1000 loop
            wait until rising_edge(clk);
            wait for 1 ps;
            exit when response_count >= count_before + 3;
        end loop;
        assert response_count >= count_before + 3
            report "V2-GPX-TB burst did not produce three responses"
            severity failure;
        request.burst <= '0';
        request.valid <= '0';
        wait_idle;
        assert response_count <= count_before + 4
            report "V2-GPX-TB burst produced more than one in-flight tail"
            severity failure;

        report "LIDAR_V2_GPX_BUS_ENGINE_PASS tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ) & " oen_mode=" & G_OEN_MODE
            severity note;
        done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_bus_engine_200 is
end entity;

architecture sim of tb_lidar_gpx_bus_engine_200 is
begin
    u_test : entity work.tb_lidar_gpx_bus_engine
        generic map (
            G_TDC_CLK_MHZ => 200
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_bus_engine_200_pullup is
end entity;

architecture sim of tb_lidar_gpx_bus_engine_200_pullup is
begin
    u_test : entity work.tb_lidar_gpx_bus_engine
        generic map (
            G_TDC_CLK_MHZ => 200,
            G_OEN_MODE    => "PULLUP_OR_NOT_CONNECTED"
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_bus_engine_150_pullup is
end entity;

architecture sim of tb_lidar_gpx_bus_engine_150_pullup is
begin
    u_test : entity work.tb_lidar_gpx_bus_engine
        generic map (
            G_TDC_CLK_MHZ => 150,
            G_OEN_MODE    => "PULLUP_OR_NOT_CONNECTED"
        );
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_bus_engine_150 is
end entity;

architecture sim of tb_lidar_gpx_bus_engine_150 is
begin
    u_test : entity work.tb_lidar_gpx_bus_engine
        generic map (
            G_TDC_CLK_MHZ => 150
        );
end architecture;
