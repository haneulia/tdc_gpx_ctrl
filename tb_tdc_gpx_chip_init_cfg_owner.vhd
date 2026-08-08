-- =============================================================================
-- tb_tdc_gpx_chip_init_cfg_owner.vhd
-- Verifies direct use of the coordinator-owned stable configuration snapshot.
-- =============================================================================
-- 테스트 자산 목적: GPX 초기화 image 소유권과 cfg-write coalesced 진단을 검증한다.
-- 핵심 검증 계약: deferred write 보존, CLEAR_STATUS, 같은 clock 새 fault 우선이다.
-- 관련 RTL: tdc_gpx_chip_init.
-- 실행 회귀: system_integration/v2/scripts/run_v2_gpx_clear_status.ps1
-- 유지보수 주의: 진단 clear가 진행 중 init/pending 요청을 취소하면 안 된다.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_chip_init_cfg_owner is
end entity tb_tdc_gpx_chip_init_cfg_owner;

architecture sim of tb_tdc_gpx_chip_init_cfg_owner is
    constant c_CLK_PERIOD : time := 10 ns;

    type t_reg_seq is array(natural range <>) of natural range 0 to 15;
    constant c_CFG_SEQ : t_reg_seq := (0, 1, 2, 3, 4, 5, 6, 7, 11, 12, 14);

    function fn_image return t_cfg_image is
        variable v : t_cfg_image := (others => (others => '0'));
    begin
        for i in v'range loop
            v(i) := std_logic_vector(to_unsigned(16#120000# + i * 16#101#, 32));
        end loop;
        v(4)(22) := '0';
        v(14)(4) := '1';
        return v;
    end function;

    constant c_IMAGE : t_cfg_image := fn_image;

    signal s_clk           : std_logic := '0';
    signal s_rst_n         : std_logic := '0';
    signal s_start         : std_logic := '0';
    signal s_cfg_write_req : std_logic := '0';
    signal s_soft_clear    : std_logic := '0';
    signal s_rsp_valid     : std_logic := '0';
    signal s_req_valid     : std_logic;
    signal s_req_rw        : std_logic;
    signal s_req_addr      : std_logic_vector(3 downto 0);
    signal s_req_wdata     : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
    signal s_done          : std_logic;
    signal s_timeout       : std_logic;
    signal s_busy          : std_logic;
    signal s_coalesced     : std_logic;
begin
    s_clk <= not s_clk after c_CLK_PERIOD / 2;

    u_dut : entity work.tdc_gpx_chip_init
        generic map (
            g_BUS_DATA_WIDTH => c_TDC_BUS_WIDTH,
            g_POWERUP_CLKS   => 2,
            g_RECOVERY_CLKS  => 2
        )
        port map (
            i_clk                 => s_clk,
            i_rst_n               => s_rst_n,
            i_start               => s_start,
            i_cfg_write_req       => s_cfg_write_req,
            i_soft_clear          => s_soft_clear,
            i_cfg_image           => c_IMAGE,
            o_done                => s_done,
            o_timeout             => s_timeout,
            o_bus_req_valid       => s_req_valid,
            o_bus_req_rw          => s_req_rw,
            o_bus_req_addr        => s_req_addr,
            o_bus_req_wdata       => s_req_wdata,
            i_bus_rsp_valid       => s_rsp_valid,
            o_puresn              => open,
            o_stopdis             => open,
            o_busy                => s_busy,
            o_cfg_write_coalesced => s_coalesced
        );

    p_stimulus : process
        procedure pulse_requests(
            constant start_value : in std_logic;
            constant cfg_value   : in std_logic
        ) is
        begin
            wait until falling_edge(s_clk);
            s_start         <= start_value;
            s_cfg_write_req <= cfg_value;
            wait until falling_edge(s_clk);
            s_start         <= '0';
            s_cfg_write_req <= '0';
        end procedure;

        procedure acknowledge_write(
            constant reg_num      : in natural;
            constant master_reset : in boolean := false
        ) is
            variable v_expected : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0);
        begin
            wait until s_req_valid = '1';
            wait for 1 ns;
            v_expected := c_IMAGE(reg_num)(c_TDC_BUS_WIDTH - 1 downto 0);
            if reg_num = 14 then
                v_expected(4) := '0';
            end if;
            if master_reset then
                v_expected(22) := '1';
            end if;
            assert s_req_rw = '1'
                report "chip_init emitted a read during configuration"
                severity failure;
            assert s_req_addr = std_logic_vector(to_unsigned(reg_num, 4))
                report "chip_init register order mismatch"
                severity failure;
            assert s_req_wdata = v_expected
                report "chip_init configuration data mismatch"
                severity failure;

            wait until falling_edge(s_clk);
            s_rsp_valid <= '1';
            wait until falling_edge(s_clk);
            s_rsp_valid <= '0';
            wait for 1 ns;
            assert s_req_valid = '0'
                report "chip_init request did not retire after response"
                severity failure;
        end procedure;

        procedure acknowledge_cfg_sequence is
        begin
            for i in c_CFG_SEQ'range loop
                acknowledge_write(c_CFG_SEQ(i));
            end loop;
        end procedure;

        procedure wait_done is
        begin
            if s_done /= '1' then
                wait until s_done = '1';
            end if;
            wait for 1 ns;
            assert s_timeout = '0'
                report "chip_init unexpectedly timed out"
                severity failure;
        end procedure;
    begin
        wait for 3 * c_CLK_PERIOD;
        wait until falling_edge(s_clk);
        s_rst_n <= '1';

        -- Runtime configuration reads all data directly from the held image.
        pulse_requests('0', '1');
        acknowledge_cfg_sequence;
        wait_done;

        -- Full initialization also reads Reg4 for the master-reset command.
        wait until falling_edge(s_clk);
        s_rst_n <= '0';
        wait for 2 * c_CLK_PERIOD;
        wait until falling_edge(s_clk);
        s_rst_n <= '1';
        pulse_requests('1', '0');
        acknowledge_cfg_sequence;
        acknowledge_write(4, true);
        wait_done;

        -- Start+cfg queues one deferred write. A further busy-window request
        -- is coalesced, but both operations keep using the same owner snapshot.
        wait until falling_edge(s_clk);
        s_rst_n <= '0';
        wait for 2 * c_CLK_PERIOD;
        wait until falling_edge(s_clk);
        s_rst_n <= '1';
        pulse_requests('1', '1');
        wait for 1 ns;
        assert s_busy = '1'
            report "chip_init did not enter the full-init busy state"
            severity failure;
        pulse_requests('0', '1');
        wait for 1 ns;
        assert s_coalesced = '1'
            report "chip_init did not flag a coalesced busy-window request"
            severity failure;

        -- CLEAR_STATUS는 진단 이력만 지우고 진행 중 init과 pending
        -- cfg-write를 그대로 유지해야 한다.
        wait until falling_edge(s_clk);
        s_soft_clear <= '1';
        wait until rising_edge(s_clk);
        wait for 1 ns;
        assert s_coalesced = '0' and s_busy = '1'
            report "chip_init CLEAR_STATUS changed busy state or missed clear"
            severity failure;
        wait until falling_edge(s_clk);
        s_soft_clear <= '0';

        -- clear와 새 coalesced 사건이 같은 clock이면 새 사건이 우선한다.
        s_soft_clear    <= '1';
        s_cfg_write_req <= '1';
        wait until rising_edge(s_clk);
        wait for 1 ns;
        assert s_coalesced = '1'
            report "chip_init same-cycle coalesced event lost to CLEAR_STATUS"
            severity failure;
        wait until falling_edge(s_clk);
        s_soft_clear    <= '0';
        s_cfg_write_req <= '0';

        acknowledge_cfg_sequence;
        acknowledge_write(4, true);
        wait_done;
        acknowledge_cfg_sequence;
        wait_done;

        report "CHIP_INIT_COORDINATOR_CFG_OWNER PASS" severity note;
        report "LIDAR_V2_K11_INIT_STICKY_CLEAR_PASS" severity note;
        std.env.stop;
        wait;
    end process p_stimulus;
end architecture sim;
