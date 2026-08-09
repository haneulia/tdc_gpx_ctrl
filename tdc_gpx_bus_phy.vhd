-- =============================================================================
-- tdc_gpx_bus_phy.vhd
-- TDC-GPX Controller - Physical Bus Interface FSM
-- =============================================================================
--
-- Purpose:
--   Converts chip_ctrl request interface to TDC-GPX 28-bit async parallel bus.
--   Manages bus timing (Tick-Phase), IOBUF, 2-FF synchronizers, turnaround gaps.
--
-- Response interface:
--   AXI-Stream master (32b tdata, 4b tkeep, 8b tuser) — sole response path.
--   tdata[27:0]  = 28-bit read data (zero for writes), tdata[31:28] = 0
--   tuser[0]     = '0' READ response, '1' WRITE ack
--   tuser[4:1]   = target register address
--   tkeep        = "1111" (all bytes valid)
--   Handshake: tvalid held until tready='1'. One beat per transaction.
--   chip_ctrl receives this via AXI-Stream slave (through skid buffer in top).
--
-- Request interface contract:
--   i_req_valid is level-held until the response path accepts the request.
--   A non-burst request is accepted only once while i_req_valid remains high;
--   the requester must drive i_req_valid low for at least one i_clk edge before
--   presenting the next independent request. Burst reads remain continuous
--   inside ST_READ and use the live i_req_burst input to terminate the burst.
--
-- Bus Timing (per deep_analysis section 12.3):
--   Once Phase A launches, 1 transaction = locally clamped i_bus_ticks ticks.
--   A newly accepted request first crosses one registered-request phase; burst
--   reads restart inside ST_READ and therefore do not repeat that overhead.
--   C01 contract: csr_chip and bus_phy receive the same clock-derived
--   minimum capture window. At 200 MHz the 25 ns default requires div=1,
--   ticks=7 or div=2, ticks=5; every runtime combination is clamped by the
--   same package function used by both CSR ownership modes.
--   Phase A (1 tick):                address setup, strobe high
--   Phase L (i_bus_ticks - 2 ticks): strobe low (RDN or WRN)
--   Phase H (1 tick):                strobe high, transaction complete
--
--   READ:  tick 0=ADR setup, tick 1..N-2=RDN low,
--          sample at tick N-2, tick N-1=RDN high + rsp_valid
--   WRITE: tick 0 plus guarded setup phases=ADR+DATA setup,
--          tick 1..N-2=WRN low, tick N-1=WRN high,
--          guarded hold phases=DATA hold + rsp_valid
--
--   The IDLE->ST_READ/ST_WRITE_SETUP transition on tick_en IS tick 0.
--   READ starts its internal counter at 1. WRITE deliberately starts at 0
--   to add one setup interval before WRN can fall.
--
-- Read sample timing analysis (200 MHz, T_clk = 5 ns):
--   RDN low at tick 1 (clock C_k).
--   sample_en at tick N-2 (clock C_k + (N-3)*div clocks).
--   IOB FF capture at clock C_k + (N-3)*div + 1.
--   => Delay from RDN low to capture = ((ticks-3)*div + 1) * T_clk
--
--   tV-DR constraint (data valid <= 11.8 ns after RDN low):
--     ticks=3: 5 ns  => VIOLATION (sample_en coincides with RDN low)
--     ticks=4, div=1: 10 ns < 11.8 ns => VIOLATION
--     ticks=4, div=2: 15 ns => device-only timing passes, board policy fails
--     ticks=5, div=1: 15 ns => device-only timing passes, board policy fails
--     ticks=5, div=2: 25 ns => OK (default)
--
--   Burst tPW-RH (RDN high between back-to-back reads):
--     Burst restarts at tick 0 (Phase A gap), so RDN high = 2 ticks.
--     div=1: 2*5 = 10 ns >= 6 ns OK.
--
--   Board-safe constraint:
--     ((ticks-3)*div + 1) >= g_BUS_READ_PERIOD_MIN_CLKS.
--   See tdc_gpx_cfg_pkg for legal combination table.
--
-- OEN mode contract:
--   g_OEN_MODE="DYNAMIC_CONNECTED"
--     FPGA controls GPX OEN. READ drives OEN low, WRITE drives OEN high,
--     and drain burst may use i_oen_permanent.
--   g_OEN_MODE="PULLUP_OR_NOT_CONNECTED"
--     FPGA keeps o_oen high. READ relies on the datasheet OEN-high/RDN
--     strobed output behavior, and i_oen_permanent is not a dependency.
--
-- Invariants (bus safety, see 01_chip_acquisition §6):
--   INV-1: WRITE => OEN = '1' (prevent bus contention)
--   INV-2: READ  => D-bus Hi-Z (FPGA does not drive)
--   INV-3: IDLE  => D-bus Hi-Z, OEN = '1' except dynamic drain hold
--   INV-5: WRITE->READ turnaround gap (1 tick minimum)
--   INV-6: READ->WRITE OEN='1' leading (1 tick minimum)
--   INV-7: dynamic oen_permanent='1' => WRITE forbidden
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tdc_gpx_bus_phy is
    generic (
        g_BUS_DATA_WIDTH             : natural  := c_TDC_BUS_WIDTH;      -- 28
        g_OEN_MODE                   : string   := c_DEFAULT_OEN_MODE;
        g_BUS_READ_PERIOD_MIN_CLKS   : positive := c_DEFAULT_BUS_READ_PERIOD_MIN_CLKS
    );
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;

        -- Tick enable (1 clk pulse per BUS_CLK_DIV period)
        i_tick_en       : in  std_logic;

        -- Bus timing config (latched internally at transaction entry)
        i_bus_ticks     : in  unsigned(2 downto 0);         -- 3..7
        i_bus_clk_div   : in  unsigned(5 downto 0);         -- 1..63, used for legality checks

        -- Request interface (from chip_ctrl)
        i_req_valid     : in  std_logic;
        i_req_rw        : in  std_logic;                    -- '0'=READ, '1'=WRITE
        i_req_addr      : in  std_logic_vector(3 downto 0);
        i_req_wdata     : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        i_oen_permanent : in  std_logic;                    -- '1' = drain burst
        i_req_burst     : in  std_logic;                    -- '1' = back-to-back read

        -- Response: busy flag (active during transaction)
        o_busy          : out std_logic;
        o_rsp_pending   : out std_logic;  -- response pending or AXI tvalid held

        -- TDC-GPX physical-pin signals. The owning integration layer places
        -- the IOBUF so sparse logical chip slots can be compacted onto only
        -- the physical lanes selected by g_PRESENT_CHIP_MASK.
        o_adr           : out std_logic_vector(3 downto 0);
        o_csn           : out std_logic;
        o_rdn           : out std_logic;
        o_wrn           : out std_logic;
        o_oen           : out std_logic;
        i_d             : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_d             : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_d_tri         : out std_logic;

        -- Async status pins (from TDC-GPX, active HIGH)
        i_ef1_pin       : in  std_logic;
        i_ef2_pin       : in  std_logic;
        i_lf1_pin       : in  std_logic;
        i_lf2_pin       : in  std_logic;
        i_irflag_pin    : in  std_logic;
        i_errflag_pin   : in  std_logic;

        -- AXI-Stream master: bus response mirror (read data + write ack)
        -- ---------------------------------------------------------------
        -- Standard AXI4-Stream, 32-bit tdata, 8-bit tuser.
        --
        -- tdata[27:0]  = bus read data (28-bit TDC-GPX data, valid for reads)
        -- tdata[31:28] = 0 (reserved, zero-padded to 32-bit boundary)
        -- tuser[0]     = '0' READ response, '1' WRITE ack
        -- tuser[4:1]   = target register address [3:0]
        -- tuser[7:5]   = 0 (reserved)
        -- tkeep         = "1111" (all 4 bytes always valid)
        --
        -- Handshake: tvalid asserted on transaction completion, held
        --            until tready='1'. One beat per bus transaction.
        -- ---------------------------------------------------------------
        o_m_axis_tvalid : out std_logic;
        o_m_axis_tdata  : out t_bus_rsp_tdata;
        o_m_axis_tkeep  : out std_logic_vector(c_BUS_RSP_TKEEP_WIDTH - 1 downto 0);
        o_m_axis_tuser  : out t_bus_rsp_tuser;
        i_m_axis_tready : in  std_logic;

        -- Synchronized outputs (2-FF, active HIGH)
        o_ef1_sync      : out std_logic;
        o_ef2_sync      : out std_logic;
        o_lf1_sync      : out std_logic;
        o_lf2_sync      : out std_logic;
        o_irflag_sync   : out std_logic;
        o_errflag_sync  : out std_logic
    );
end entity tdc_gpx_bus_phy;

architecture rtl of tdc_gpx_bus_phy is

    constant c_OEN_DYNAMIC_CONNECTED : boolean := g_OEN_MODE = "DYNAMIC_CONNECTED";
    constant c_OEN_PULLUP_OR_NC      : boolean := g_OEN_MODE = "PULLUP_OR_NOT_CONNECTED";

    -- OEN이 PCB에 실제 연결된 구성에서만 출력 FF의 IOB 배치를 요구한다.
    -- Pull-up/미연결 구성은 Parent에서 OEN 포트를 외부화하지 않으므로 TRUE를
    -- 강제하면 Vivado REQP-1618 경고가 발생한다. 두 구성의 물리 계약을 이
    -- 속성 값으로 구분하되 OEN 상태기계 자체의 동작은 바꾸지 않는다.
    function fn_oen_iob_attribute(mode : string) return string is
    begin
        if mode = "DYNAMIC_CONNECTED" then
            return "TRUE";
        end if;
        return "FALSE";
    end function fn_oen_iob_attribute;

    constant c_OEN_IOB_ATTRIBUTE : string :=
        fn_oen_iob_attribute(g_OEN_MODE);

    function fn_min_ticks_for_div(div_value : unsigned(5 downto 0)) return unsigned is
        variable v_min_ticks : natural;
    begin
        v_min_ticks := fn_bus_min_ticks_for_capture(
            to_integer(div_value), g_BUS_READ_PERIOD_MIN_CLKS);

        return to_unsigned(v_min_ticks, 3);
    end function fn_min_ticks_for_div;

    -- =========================================================================
    -- FSM states
    -- =========================================================================
    type t_bus_state is (
        ST_IDLE,            -- bus idle, waiting for request
        ST_REQUEST,         -- locally registered request, awaiting launch
        ST_READ,            -- read transaction (tick counter drives phases)
        ST_WRITE_SETUP,     -- extend ADR/DATA setup before WRN falls
        ST_WRITE,           -- write transaction (tick counter drives phases)
        ST_WRITE_HOLD,      -- keep DATA driven after WRN rises (tH-DW)
        ST_TURNAROUND       -- 1-tick gap for direction change (INV-5, INV-6)
    );

    signal s_state_r        : t_bus_state := ST_IDLE;

    -- =========================================================================
    -- Tick counter
    -- In ST_READ: runs 1 .. i_bus_ticks-1. ST_WRITE starts at 0 after its
    -- setup guard. Tick 0 (Phase A) is consumed at request launch.
    -- =========================================================================
    signal s_tick_r          : unsigned(2 downto 0) := (others => '0');

    -- =========================================================================
    -- Pin control registers
    -- =========================================================================
    signal s_adr_r           : std_logic_vector(3 downto 0) := (others => '0');
    signal s_csn_r           : std_logic := '1';
    signal s_rdn_r           : std_logic := '1';
    signal s_wrn_r           : std_logic := '1';
    signal s_oen_r           : std_logic := '1';

    -- D-bus IOBUF control
    signal s_d_out_r         : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_d_tri_r         : std_logic := '1';    -- '1'=Hi-Z, '0'=drive

    -- IOB FF for read data capture
    signal s_d_in_ff_r       : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_sample_en       : std_logic := '0';

    attribute IOB : string;
    attribute IOB of s_d_in_ff_r : signal is "TRUE";
    attribute IOB of s_adr_r      : signal is "TRUE";
    attribute IOB of s_csn_r      : signal is "TRUE";
    attribute IOB of s_rdn_r      : signal is "TRUE";
    attribute IOB of s_wrn_r      : signal is "TRUE";
    attribute IOB of s_oen_r      : signal is c_OEN_IOB_ATTRIBUTE;
    attribute IOB of s_d_out_r    : signal is "TRUE";

    -- =========================================================================
    -- Response
    -- =========================================================================
    signal s_rsp_valid_r     : std_logic := '0';
    signal s_rsp_rdata_r     : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_rsp_pending_r   : std_logic := '0';    -- read response deferred by 1 tick
    signal s_rsp_pending_out_r : std_logic := '0';  -- registered module-boundary output
    signal s_read_phase_h_done_r : std_logic := '0'; -- suppress held-burst response replay
    signal s_busy_r          : std_logic := '0';

    -- AXI-Stream master: bus response mirror (32-bit tdata, 8-bit tuser)
    signal s_axis_tvalid_r   : std_logic := '0';
    signal s_axis_tdata_r    : t_bus_rsp_tdata := (others => '0');
    signal s_axis_tuser_r    : t_bus_rsp_tuser := (others => '0');
    signal s_axis_rw_r       : std_logic := '0';    -- '0'=read, '1'=write (latched at txn entry)
    signal s_axis_addr_r     : std_logic_vector(3 downto 0) := (others => '0');  -- latched addr

    -- Latched bus_ticks: snapshot at transaction entry to prevent
    -- mid-transaction changes from corrupting tick counter comparisons.
    signal s_bus_ticks_r      : unsigned(2 downto 0) := "101";  -- default 5
    signal s_req_burst_r      : std_logic := '0';  -- latched at accept
    signal s_oen_perm_r       : std_logic := '0';  -- latched at accept
    signal s_req_seen_r       : std_logic := '0';  -- rearm only after req_valid goes low

    -- Direction tracking for turnaround
    signal s_last_was_write_r : std_logic := '0';
    signal s_last_was_read_r  : std_logic := '0';

    -- Turnaround: target direction and latched request
    signal s_turn_to_write_r : std_logic := '0';
    signal s_req_addr_r      : std_logic_vector(3 downto 0) := (others => '0');
    signal s_req_wdata_r     : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- 2-FF synchronizer signals
    -- =========================================================================
    signal s_ef1_meta_r      : std_logic := '1';  -- EF defaults HIGH (empty)
    signal s_ef1_sync_r      : std_logic := '1';
    signal s_ef2_meta_r      : std_logic := '1';
    signal s_ef2_sync_r      : std_logic := '1';
    signal s_lf1_meta_r      : std_logic := '0';
    signal s_lf1_sync_r      : std_logic := '0';
    signal s_lf2_meta_r      : std_logic := '0';
    signal s_lf2_sync_r      : std_logic := '0';
    signal s_irflag_meta_r   : std_logic := '0';
    signal s_irflag_sync_r   : std_logic := '0';
    signal s_errflag_meta_r  : std_logic := '0';
    signal s_errflag_sync_r  : std_logic := '0';

    attribute ASYNC_REG : string;
    attribute ASYNC_REG of s_ef1_meta_r     : signal is "TRUE";
    attribute ASYNC_REG of s_ef1_sync_r     : signal is "TRUE";
    attribute ASYNC_REG of s_ef2_meta_r     : signal is "TRUE";
    attribute ASYNC_REG of s_ef2_sync_r     : signal is "TRUE";
    attribute ASYNC_REG of s_lf1_meta_r     : signal is "TRUE";
    attribute ASYNC_REG of s_lf1_sync_r     : signal is "TRUE";
    attribute ASYNC_REG of s_lf2_meta_r     : signal is "TRUE";
    attribute ASYNC_REG of s_lf2_sync_r     : signal is "TRUE";
    attribute ASYNC_REG of s_irflag_meta_r  : signal is "TRUE";
    attribute ASYNC_REG of s_irflag_sync_r  : signal is "TRUE";
    attribute ASYNC_REG of s_errflag_meta_r : signal is "TRUE";
    attribute ASYNC_REG of s_errflag_sync_r : signal is "TRUE";

begin

    assert g_BUS_READ_PERIOD_MIN_CLKS <= c_BUS_CAPTURE_MAX_CLKS
        report "bus_phy: capture minimum exceeds div=63/ticks=7 capacity"
        severity failure;

    o_d     <= s_d_out_r;
    o_d_tri <= s_d_tri_r;

    -- =========================================================================
    -- IOB FF: read data capture
    -- Captures one i_clk after s_sample_en is asserted by FSM.
    -- This coincides with RDN physically transitioning high (Phase H start).
    -- Dynamic OEN mode holds OEN low during READ; pull-up/NC mode relies on
    -- the datasheet OEN-high/RDN-strobed read behavior.
    -- =========================================================================
    p_iob_ff : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if s_sample_en = '1' then
                s_d_in_ff_r <= i_d;
            end if;
        end if;
    end process p_iob_ff;

    -- =========================================================================
    -- Main FSM
    --
    -- Transaction tick assignment:
    --   ST_IDLE captures a complete request into local registers.
    --   ST_REQUEST/TURNAROUND launch tick 0 (Phase A: ADR setup).
    --   ST_READ ticks = 1 .. i_bus_ticks-1
    --   ST_WRITE_SETUP holds two ticks and ST_WRITE tick 0 adds the final
    --   guarded setup phase;
    --   ST_WRITE then runs through tick i_bus_ticks-1 and enters
    --   ST_WRITE_HOLD for three guarded hold phases.
    --
    -- Phase A (tick 0):   ADR valid, strobe high, CSN low
    -- Phase L (tick 1..N-2): strobe low, sample_en at tick N-2
    -- Phase H (tick N-1): strobe high, IOB FF captures, rsp_pending
    -- Phase H+1 (tick N): rsp_valid + rsp_rdata (READ only)
    --
    -- Pin control within ST_READ/ST_WRITE uses independent if-statements
    -- (not elsif) so conditions can overlap for small BUS_TICKS values.
    -- =========================================================================
    p_fsm : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r           <= ST_IDLE;
                s_tick_r            <= (others => '0');
                s_adr_r             <= (others => '0');
                s_csn_r             <= '1';
                s_rdn_r             <= '1';
                s_wrn_r             <= '1';
                s_oen_r             <= '1';
                s_d_out_r           <= (others => '0');
                s_d_tri_r           <= '1';             -- Hi-Z [INV-3]
                s_sample_en         <= '0';
                s_rsp_valid_r       <= '0';
                s_rsp_rdata_r       <= (others => '0');
                s_rsp_pending_r     <= '0';
                s_rsp_pending_out_r <= '0';
                s_read_phase_h_done_r <= '0';
                s_busy_r            <= '0';
                s_last_was_write_r  <= '0';
                s_last_was_read_r   <= '0';
                s_req_addr_r        <= (others => '0');
                s_req_wdata_r       <= (others => '0');
                s_turn_to_write_r   <= '0';
                s_bus_ticks_r       <= "101";            -- default 5
                s_axis_tvalid_r     <= '0';
                s_axis_tdata_r      <= (others => '0');
                s_axis_tuser_r      <= (others => '0');
                s_axis_rw_r         <= '0';
                s_axis_addr_r       <= (others => '0');
                s_req_seen_r        <= '0';
            else
                -- Default: clear single-cycle pulses
                s_rsp_valid_r <= '0';
                s_sample_en   <= '0';

                -- The request channel has no READY signal. Treat each
                -- contiguous req_valid-high interval as one request so a
                -- response/skid latency cannot replay the same transaction
                -- when the FSM returns to IDLE.
                if i_req_valid = '0' then
                    s_req_seen_r <= '0';
                end if;

                -- AXI-Stream handshake: clear tvalid when tready accepted
                if s_axis_tvalid_r = '1' and i_m_axis_tready = '1' then
                    s_axis_tvalid_r <= '0';
                    if s_rsp_pending_r = '0' then
                        s_rsp_pending_out_r <= '0';
                    end if;
                end if;

                -- ==========================================================
                -- Deferred read response: emit rsp_valid one tick after
                -- Phase H, so that s_d_in_ff_r (IOB FF) has settled.
                --
                -- Timeline:
                --   tick N-2: s_sample_en = '1'
                --   tick N-1: IOB FF captures s_d_in -> s_d_in_ff_r
                --             FSM sets s_rsp_pending_r = '1'
                --   tick N  : s_d_in_ff_r is valid; emit rsp_valid + rdata
                -- ==========================================================
                -- The deferred register is the second response slot. Do not
                -- overwrite a stalled AXIS output; transfer only when that
                -- slot is empty or is being consumed on this clock.
                if s_rsp_pending_r = '1'
                   and (s_axis_tvalid_r = '0' or i_m_axis_tready = '1') then
                    s_rsp_valid_r   <= '1';
                    s_rsp_rdata_r   <= s_d_in_ff_r;
                    s_rsp_pending_r <= '0';
                    s_rsp_pending_out_r <= '1';
                    -- AXI-Stream: read response (zero-pad 28→32 bit)
                    s_axis_tvalid_r             <= '1';
                    s_axis_tdata_r(27 downto 0) <= s_d_in_ff_r;
                    s_axis_tdata_r(c_BUS_RSP_TDATA_WIDTH - 1 downto c_TDC_BUS_WIDTH) <= (others => '0');
                    s_axis_tuser_r              <= "000" & s_axis_addr_r & '0';
                end if;

                case s_state_r is

                    -- ---------------------------------------------------------
                    -- IDLE: wait for and register a complete request.
                    -- No IOB-facing pin changes are launched in this state.
                    -- ---------------------------------------------------------
                    when ST_IDLE =>
                        s_read_phase_h_done_r <= '0';
                        s_csn_r   <= '1';
                        s_rdn_r   <= '1';
                        s_wrn_r   <= '1';
                        s_d_tri_r <= '1';               -- Hi-Z [INV-3]
                        s_busy_r  <= '0';

                        -- OEN: dynamic mode may hold OEN low during burst.
                        -- Pull-up/NC mode keeps the FPGA OEN output high and
                        -- relies on RDN to gate normal reads.
                        if c_OEN_DYNAMIC_CONNECTED and i_oen_permanent = '1' then
                            s_oen_r <= '0';
                        else
                            s_oen_r <= '1';
                        end if;

                        -- Guard: block acceptance when a response is pending
                        -- or being emitted this cycle.
                        -- s_rsp_pending_r check is critical: without it, the
                        -- deferred-read path sets s_rsp_valid_r='1' above, but
                        -- VHDL signal semantics mean the ST_IDLE check below
                        -- still sees the OLD value ('0'), causing spurious
                        -- re-acceptance of the same request.
                        if i_req_valid = '1' and s_req_seen_r = '0'
                           and i_tick_en = '1'
                           and s_rsp_valid_r = '0'
                           and s_rsp_pending_r = '0'
                           and s_axis_tvalid_r = '0' then  -- AXI-Stream response fully consumed
                            -- Mark even a policy-rejected request as consumed.
                            -- A retry is a new request and therefore requires
                            -- req_valid to return low first.
                            s_req_seen_r <= '1';
                            -- synthesis translate_off
                            assert c_OEN_DYNAMIC_CONNECTED or c_OEN_PULLUP_OR_NC
                                report "bus_phy: unsupported g_OEN_MODE"
                                severity failure;
                            assert to_integer(i_bus_ticks) >= to_integer(fn_min_ticks_for_div(i_bus_clk_div))
                                report "bus_phy: bus timing clamped (div=" &
                                       integer'image(to_integer(i_bus_clk_div)) &
                                       ", ticks=" & integer'image(to_integer(i_bus_ticks)) & ")"
                                severity warning;
                            assert to_integer(i_bus_clk_div) >=
                                   fn_bus_min_div_for_capture(g_BUS_READ_PERIOD_MIN_CLKS)
                                report "bus_phy: bus divider cannot satisfy configured capture window"
                                severity warning;
                            -- synthesis translate_on
                            if i_req_rw = '1' and c_OEN_DYNAMIC_CONNECTED
                               and i_oen_permanent = '1' then
                                -- [INV-7] WRITE forbidden during oen_permanent.
                                -- synthesis translate_off
                                assert false
                                    report "bus_phy: write request ignored (oen_permanent='1')"
                                    severity warning;
                                -- synthesis translate_on
                            else
                                -- Register the complete request before any
                                -- IOB-facing pin changes. This removes the
                                -- chip-controller-to-IOB combinational path.
                                s_req_addr_r   <= i_req_addr;
                                s_req_wdata_r  <= i_req_wdata;
                                s_req_burst_r  <= i_req_burst;
                                s_oen_perm_r   <= i_oen_permanent;
                                s_axis_rw_r    <= i_req_rw;
                                s_axis_addr_r  <= i_req_addr;
                                if i_bus_ticks >= fn_min_ticks_for_div(i_bus_clk_div) then
                                    s_bus_ticks_r <= i_bus_ticks;
                                else
                                    s_bus_ticks_r <= fn_min_ticks_for_div(i_bus_clk_div);
                                end if;
                                s_busy_r  <= '1';
                                s_state_r <= ST_REQUEST;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    -- Registered request launch
                    --
                    -- Requests enter this state through a local register bank.
                    -- IOB-facing outputs therefore depend only on local state
                    -- and registered request fields at the 200 MHz boundary.
                    -- ---------------------------------------------------------
                    when ST_REQUEST =>
                        s_csn_r   <= '1';
                        s_rdn_r   <= '1';
                        s_wrn_r   <= '1';
                        s_d_tri_r <= '1';
                        s_oen_r   <= '1';
                        s_busy_r  <= '1';

                        if i_tick_en = '1' then
                            if s_axis_rw_r = '1' then
                                if s_last_was_read_r = '1' then
                                    s_turn_to_write_r <= '1';
                                    s_state_r         <= ST_TURNAROUND;
                                else
                                    s_adr_r       <= s_req_addr_r;
                                    s_csn_r       <= '0';
                                    s_d_out_r     <= s_req_wdata_r;
                                    s_d_tri_r     <= '0';
                                    s_tick_r      <= to_unsigned(0, 3);
                                    s_state_r     <= ST_WRITE_SETUP;
                                end if;
                            else
                                if s_last_was_write_r = '1' then
                                    s_turn_to_write_r <= '0';
                                    s_state_r         <= ST_TURNAROUND;
                                else
                                    s_adr_r   <= s_req_addr_r;
                                    s_csn_r   <= '0';
                                    if c_OEN_DYNAMIC_CONNECTED then
                                        s_oen_r <= '0';
                                    end if;
                                    s_tick_r  <= to_unsigned(1, 3);
                                    s_state_r <= ST_READ;
                                end if;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    -- TURNAROUND: 1-tick gap for direction change [INV-5, INV-6]
                    -- All strobes high, D-bus Hi-Z, OEN high.
                    -- The turnaround tick is extra overhead (not counted in
                    -- BUS_TICKS). On tick_en, enters next transaction Phase A.
                    -- ---------------------------------------------------------
                    when ST_TURNAROUND =>
                        s_read_phase_h_done_r <= '0';
                        s_csn_r   <= '1';
                        s_rdn_r   <= '1';
                        s_wrn_r   <= '1';
                        s_d_tri_r <= '1';               -- Hi-Z
                        s_oen_r   <= '1';               -- OEN high during gap

                        if i_tick_en = '1' then
                            if s_turn_to_write_r = '1' then
                                -- Enter WRITE Phase A (tick 0)
                                s_adr_r   <= s_req_addr_r;
                                s_csn_r   <= '0';
                                s_oen_r   <= '1';       -- [INV-1]
                                s_d_out_r <= s_req_wdata_r;
                                s_d_tri_r <= '0';       -- drive
                                s_wrn_r   <= '1';
                                -- Same guarded setup path as direct entry.
                                s_tick_r  <= to_unsigned(0, 3);
                                s_state_r <= ST_WRITE_SETUP;
                            else
                                -- Enter READ Phase A (tick 0)
                                s_adr_r   <= s_req_addr_r;
                                s_csn_r   <= '0';
                                if c_OEN_DYNAMIC_CONNECTED then
                                    s_oen_r <= '0';     -- chip drives
                                else
                                    s_oen_r <= '1';     -- pull-up/NC: RDN gates output
                                end if;
                                s_d_tri_r <= '1';       -- Hi-Z [INV-2]
                                s_rdn_r   <= '1';
                                s_tick_r  <= to_unsigned(1, 3);
                                s_state_r <= ST_READ;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    -- WRITE setup guard
                    --
                    -- Phase A pins are already registered. Hold them for two
                    -- additional phase intervals before ST_WRITE tick 0.
                    -- WRN can therefore fall only after four phase intervals
                    -- (20 ns at 200 MHz, div=1).
                    -- ---------------------------------------------------------
                    when ST_WRITE_SETUP =>
                        s_wrn_r   <= '1';
                        s_oen_r   <= '1';
                        s_d_tri_r <= '0';
                        s_busy_r  <= '1';

                        if i_tick_en = '1' then
                            if s_tick_r >= to_unsigned(1, s_tick_r'length) then
                                s_tick_r  <= to_unsigned(0, s_tick_r'length);
                                s_state_r <= ST_WRITE;
                            else
                                s_tick_r <= s_tick_r + 1;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    -- READ transaction
                    --
                    -- Entered with Phase A pins already set (tick 0 consumed
                    -- at ST_REQUEST/TURNAROUND). s_tick_r starts at 1.
                    --
                    -- Pin control uses independent if-statements so conditions
                    -- can overlap for small BUS_TICKS (e.g., BUS_TICKS=3:
                    -- tick 1 = Phase L start AND sample simultaneously).
                    --
                    -- D-bus = Hi-Z throughout [INV-2].
                    -- OEN is mode-specific: low in dynamic mode, high in
                    -- pull-up/NC mode where RDN gates the GPX output.
                    -- ---------------------------------------------------------
                    when ST_READ =>
                        if i_tick_en = '1' then

                            -- RDN: low during Phase L (ticks 1 .. N-2)
                            if s_tick_r >= 1 and s_tick_r <= s_bus_ticks_r - 2 then
                                s_rdn_r <= '0';
                            end if;

                            -- Sample IOB FF at Phase L last tick (tick N-2)
                            if s_tick_r = s_bus_ticks_r - 2 then
                                s_sample_en <= '1';
                            end if;

                            -- Phase H: transaction complete (tick N-1)
                            -- RDN='1' assignment here overrides the range check
                            -- above (last sequential assignment wins in VHDL).
                            -- rsp_valid is deferred by 1 tick (s_rsp_pending_r)
                            -- so that s_d_in_ff_r has settled from the IOB FF
                            -- capture that happens at this same clock edge.
                            if s_tick_r = s_bus_ticks_r - 1 then
                                -- Phase H can be held while the two response
                                -- slots (AXIS output + deferred response) are
                                -- full. Publish this physical read only on the
                                -- first Phase H clock; replaying the side
                                -- effects would duplicate the same GPX word.
                                if s_read_phase_h_done_r = '0' then
                                    s_rdn_r                <= '1';
                                    s_rsp_pending_r        <= '1';
                                    s_rsp_pending_out_r    <= '1';
                                    s_last_was_write_r     <= '0';
                                    s_last_was_read_r      <= '1';
                                    s_read_phase_h_done_r  <= '1';
                                end if;

                                -- Burst: back-to-back read (no IDLE)
                                -- Restart at tick 0 (Phase A gap) so that
                                -- RDN high = 2 ticks (Phase H + Phase A),
                                -- satisfying tPW-RH >= 6 ns even at div=1
                                -- (2 × 5 ns = 10 ns >= 6 ns).
                                -- CSN, OEN, ADR unchanged throughout burst.
                                -- Asymmetric by design (#23):
                                --   - s_oen_perm_r : LATCHED at burst start so a
                                --     transient drop of i_req_oen_permanent mid-
                                --     burst cannot unlatch output enable.
                                --   - i_req_burst  : LIVE, so chip_run can abort
                                --     a burst cleanly by lowering it one cycle
                                --     before the final beat.
                                -- Do NOT latch i_req_burst — doing so removes
                                -- chip_run's mid-burst-abort capability and has
                                -- no functional benefit. Concern raised in #23
                                -- was module independence, not correctness.
                                if ((c_OEN_DYNAMIC_CONNECTED and s_oen_perm_r = '1') or c_OEN_PULLUP_OR_NC)
                                   and i_req_burst = '1' then
                                    -- Burst mode: continue only if response was consumed
                                    if s_axis_tvalid_r = '0' or i_m_axis_tready = '1' then
                                        s_tick_r                <= to_unsigned(0, 3);
                                        s_read_phase_h_done_r   <= '0';
                                        -- stay in ST_READ, busy remains '1'
                                    end if;
                                    -- else: hold at max tick until tready clears response
                                else
                                    -- Normal completion: return to IDLE
                                    -- busy is NOT cleared here: rsp_pending
                                    -- is still '1', so the deferred rsp_valid
                                    -- has not fired yet. ST_IDLE's default
                                    -- s_busy_r <= '0' clears it on the next
                                    -- cycle, coinciding with rsp_valid='1'.
                                    -- This prevents chip_ctrl from seeing
                                    -- busy=0 + rsp_valid=0 (premature exit).
                                    s_csn_r   <= '1';
                                    if (not c_OEN_DYNAMIC_CONNECTED) or s_oen_perm_r = '0' then  -- use latched, not live
                                        s_oen_r <= '1';
                                    end if;
                                    s_tick_r  <= (others => '0');
                                    s_read_phase_h_done_r <= '0';
                                    s_state_r <= ST_IDLE;
                                end if;
                            else
                                s_tick_r <= s_tick_r + 1;
                            end if;

                        end if;

                    -- ---------------------------------------------------------
                    -- WRITE transaction
                    --
                    -- Entered with Phase A pins already set at
                    -- ST_REQUEST/TURNAROUND. s_tick_r starts at 0 so the first
                    -- tick_en keeps WRN high. Together with ST_WRITE_SETUP,
                    -- Phase L starts after four complete phase intervals.
                    --
                    -- D-bus = drive throughout (FPGA outputs write data)
                    -- OEN = '1' throughout [INV-1]
                    -- ---------------------------------------------------------
                    when ST_WRITE =>
                        if i_tick_en = '1' then

                            -- WRN: low during Phase L (ticks 1 .. N-2)
                            if s_tick_r >= 1 and s_tick_r <= s_bus_ticks_r - 2 then
                                s_wrn_r <= '0';
                            end if;

                            -- Phase H: transaction complete (tick N-1)
                            if s_tick_r = s_bus_ticks_r - 1 then
                                s_wrn_r            <= '1';
                                s_last_was_write_r <= '1';
                                s_last_was_read_r  <= '0';
                                s_csn_r            <= '1';
                                -- Keep the D-bus driven after WRN rises.
                                -- ST_WRITE_HOLD releases it on the next
                                -- guarded phase counter below, providing
                                -- 15 ns before pad skew at 200 MHz, div=1.
                                s_d_tri_r          <= '0';
                                s_tick_r           <= (others => '0');
                                s_state_r          <= ST_WRITE_HOLD;
                            else
                                s_tick_r <= s_tick_r + 1;
                            end if;

                        end if;

                    -- ---------------------------------------------------------
                    -- WRITE data-hold phase
                    --
                    -- WRN is already high and CSN may be released. Keep DATA
                    -- actively driven for three phase intervals, then publish
                    -- the write acknowledgement and return IDLE.
                    -- ---------------------------------------------------------
                    when ST_WRITE_HOLD =>
                        s_wrn_r   <= '1';
                        s_oen_r   <= '1';
                        s_d_tri_r <= '0';
                        s_busy_r  <= '1';

                        if i_tick_en = '1' then
                            if s_tick_r >= to_unsigned(2, s_tick_r'length) then
                                s_d_tri_r           <= '1';
                                s_busy_r            <= '0';
                                s_state_r           <= ST_IDLE;
                                s_rsp_valid_r       <= '1';
                                s_axis_tvalid_r     <= '1';
                                s_axis_tdata_r      <= (others => '0');
                                s_axis_tuser_r      <= "000" & s_axis_addr_r & '1';
                                s_rsp_pending_out_r <= '1';
                                s_tick_r            <= (others => '0');
                            else
                                s_tick_r <= s_tick_r + 1;
                            end if;
                        end if;

                    when others =>
                        s_state_r <= ST_IDLE;

                end case;
            end if;
        end if;
    end process p_fsm;

    -- =========================================================================
    -- Output assignments (directly from registers — no combinational paths)
    -- =========================================================================
    o_adr       <= s_adr_r;
    o_csn       <= s_csn_r;
    o_rdn       <= s_rdn_r;
    o_wrn       <= s_wrn_r;
    o_oen       <= s_oen_r;

    o_busy        <= s_busy_r;
    o_rsp_pending <= s_rsp_pending_out_r;

    o_m_axis_tvalid <= s_axis_tvalid_r;
    o_m_axis_tdata  <= s_axis_tdata_r;
    o_m_axis_tkeep  <= (others => '1');
    o_m_axis_tuser  <= s_axis_tuser_r;

    -- =========================================================================
    -- 2-FF Synchronizers (free-running, not gated by tick_en)
    -- =========================================================================
    p_sync : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_ef1_meta_r     <= '1';    -- EF defaults HIGH (empty)
                s_ef1_sync_r     <= '1';
                s_ef2_meta_r     <= '1';
                s_ef2_sync_r     <= '1';
                s_lf1_meta_r     <= '0';
                s_lf1_sync_r     <= '0';
                s_lf2_meta_r     <= '0';
                s_lf2_sync_r     <= '0';
                s_irflag_meta_r  <= '0';
                s_irflag_sync_r  <= '0';
                s_errflag_meta_r <= '0';
                s_errflag_sync_r <= '0';
            else
                -- Stage 1 (meta)
                s_ef1_meta_r     <= i_ef1_pin;
                s_ef2_meta_r     <= i_ef2_pin;
                s_lf1_meta_r     <= i_lf1_pin;
                s_lf2_meta_r     <= i_lf2_pin;
                s_irflag_meta_r  <= i_irflag_pin;
                s_errflag_meta_r <= i_errflag_pin;
                -- Stage 2 (sync)
                s_ef1_sync_r     <= s_ef1_meta_r;
                s_ef2_sync_r     <= s_ef2_meta_r;
                s_lf1_sync_r     <= s_lf1_meta_r;
                s_lf2_sync_r     <= s_lf2_meta_r;
                s_irflag_sync_r  <= s_irflag_meta_r;
                s_errflag_sync_r <= s_errflag_meta_r;
            end if;
        end if;
    end process p_sync;

    o_ef1_sync     <= s_ef1_sync_r;
    o_ef2_sync     <= s_ef2_sync_r;
    o_lf1_sync     <= s_lf1_sync_r;
    o_lf2_sync     <= s_lf2_sync_r;
    o_irflag_sync  <= s_irflag_sync_r;
    o_errflag_sync <= s_errflag_sync_r;

end architecture rtl;
