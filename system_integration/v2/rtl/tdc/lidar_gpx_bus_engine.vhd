library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_pkg.all;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

-- Typed v2 boundary around the proven v1 physical GPX bus FSM.
--
-- tdc_gpx_bus_phy remains the sole owner of ADR/D/CSN/RDN/WRN/OEN timing,
-- turnaround and response holding. This wrapper only packs/unpacks records;
-- it must remain cycle- and pin-equivalent to a direct v1 instantiation.
entity lidar_gpx_bus_engine is
    generic (
        G_TDC_CLK_MHZ : positive := 200;
        G_OEN_MODE : string := "DYNAMIC_CONNECTED"
    );
    port (
        i_clk       : in  std_logic;
        i_rst_n     : in  std_logic;
        i_tick_en   : in  std_logic;
        i_timing    : in  gpx_bus_timing_t;
        i_request   : in  gpx_bus_request_t;
        o_busy      : out std_logic;
        o_response_pending : out std_logic;

        o_adr       : out gpx_bus_address_t;
        o_csn       : out std_logic;
        o_rdn       : out std_logic;
        o_wrn       : out std_logic;
        o_oen       : out std_logic;
        i_d         : in  gpx_bus_data_t;
        o_d         : out gpx_bus_data_t;
        o_d_tri     : out std_logic;

        i_ef1       : in  std_logic;
        i_ef2       : in  std_logic;
        i_lf1       : in  std_logic;
        i_lf2       : in  std_logic;
        i_irflag    : in  std_logic;
        i_errflag   : in  std_logic;

        o_response  : out gpx_bus_response_t;
        i_response_ready : in std_logic;
        o_status    : out gpx_pin_status_t;
        o_effective_ticks : out unsigned(2 downto 0)
    );
end entity lidar_gpx_bus_engine;

architecture rtl of lidar_gpx_bus_engine is

    constant C_READ_PERIOD_MIN_CLKS : positive := fn_time_ns_to_clks_ceil(
        c_DEFAULT_BUS_READ_PERIOD_MIN_TIME_NS,
        G_TDC_CLK_MHZ);

    signal response_valid_c : std_logic;
    signal response_data_c : t_bus_rsp_tdata;
    signal response_keep_c : std_logic_vector(
        c_BUS_RSP_TKEEP_WIDTH - 1 downto 0);
    signal response_user_c : t_bus_rsp_tuser;
    signal status_c : gpx_pin_status_t := C_GPX_PIN_STATUS_RESET;

    function fn_effective_ticks(
        timing : gpx_bus_timing_t
    ) return unsigned is
        variable requested_ticks : natural;
        variable minimum_ticks   : natural;
    begin
        requested_ticks := to_integer(timing.ticks);
        minimum_ticks := fn_bus_min_ticks_for_capture(
            to_integer(timing.clock_div),
            C_READ_PERIOD_MIN_CLKS);
        if requested_ticks < minimum_ticks then
            return to_unsigned(minimum_ticks, 3);
        else
            return timing.ticks;
        end if;
    end function fn_effective_ticks;

begin

    assert fn_is_legal_clock_mhz(G_TDC_CLK_MHZ)
        report "V2-GPX-001 unsupported TDC clock"
        severity failure;
    assert G_OEN_MODE = "DYNAMIC_CONNECTED" or
           G_OEN_MODE = "PULLUP_OR_NOT_CONNECTED"
        report "V2-GPX-004 unsupported OEN mode"
        severity failure;

    o_response <= (
        valid     => response_valid_c,
        write_ack => response_user_c(0),
        address   => response_user_c(4 downto 1),
        read_data => response_data_c(C_GPX_BUS_DATA_WIDTH - 1 downto 0)
    );
    o_status <= status_c;
    o_effective_ticks <= fn_effective_ticks(i_timing);

    u_proven_bus_phy : entity work.tdc_gpx_bus_phy
        generic map (
            g_BUS_DATA_WIDTH           => C_GPX_BUS_DATA_WIDTH,
            g_OEN_MODE                 => G_OEN_MODE,
            g_BUS_READ_PERIOD_MIN_CLKS => C_READ_PERIOD_MIN_CLKS
        )
        port map (
            i_clk           => i_clk,
            i_rst_n         => i_rst_n,
            i_tick_en       => i_tick_en,
            i_bus_ticks     => i_timing.ticks,
            i_bus_clk_div   => i_timing.clock_div,
            i_req_valid     => i_request.valid,
            i_req_rw        => i_request.write,
            i_req_addr      => i_request.address,
            i_req_wdata     => i_request.write_data,
            i_oen_permanent => i_request.oen_permanent,
            i_req_burst     => i_request.burst,
            o_busy          => o_busy,
            o_rsp_pending   => o_response_pending,
            o_adr           => o_adr,
            o_csn           => o_csn,
            o_rdn           => o_rdn,
            o_wrn           => o_wrn,
            o_oen           => o_oen,
            i_d             => i_d,
            o_d             => o_d,
            o_d_tri         => o_d_tri,
            i_ef1_pin       => i_ef1,
            i_ef2_pin       => i_ef2,
            i_lf1_pin       => i_lf1,
            i_lf2_pin       => i_lf2,
            i_irflag_pin    => i_irflag,
            i_errflag_pin   => i_errflag,
            o_m_axis_tvalid => response_valid_c,
            o_m_axis_tdata  => response_data_c,
            o_m_axis_tkeep  => response_keep_c,
            o_m_axis_tuser  => response_user_c,
            i_m_axis_tready => i_response_ready,
            o_ef1_sync      => status_c.ef1,
            o_ef2_sync      => status_c.ef2,
            o_lf1_sync      => status_c.lf1,
            o_lf2_sync      => status_c.lf2,
            o_irflag_sync   => status_c.irflag,
            o_errflag_sync  => status_c.errflag
        );

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' then
                assert response_keep_c = "1111"
                    report "V2-GPX-002 proven bus response TKEEP changed"
                    severity failure;
                if i_request.valid = '1' then
                    assert i_timing.clock_div /= 0 and i_timing.ticks /= 0
                        report "V2-GPX-003 invalid active bus timing"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
