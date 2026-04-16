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
-- Bus Timing (per deep_analysis section 12.3):
--   1 transaction = i_bus_ticks ticks (min 4, default 5)
--   Phase A (1 tick):                address setup, strobe high
--   Phase L (i_bus_ticks - 2 ticks): strobe low (RDN or WRN)
--   Phase H (1 tick):                strobe high, transaction complete
--
--   READ:  tick 0=ADR setup, tick 1..N-2=RDN low,
--          sample at tick N-2, tick N-1=RDN high + rsp_valid
--   WRITE: tick 0=ADR+DATA setup, tick 1..N-2=WRN low,
--          tick N-1=WRN high + rsp_valid
--
--   The IDLE->ST_READ/ST_WRITE transition on tick_en IS tick 0 (Phase A).
--   The first tick_en inside ST_READ/ST_WRITE is tick 1.
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
--     ticks=4, div=2: 15 ns => OK (3.2 ns margin)
--     ticks=5, div=1: 15 ns => OK (3.2 ns margin, fastest: 40 MHz)
--     ticks=5, div=2: 25 ns => OK (default)
--
--   Burst tPW-RH (RDN high between back-to-back reads):
--     Burst restarts at tick 0 (Phase A gap), so RDN high = 2 ticks.
--     div=1: 2*5 = 10 ns >= 6 ns OK.
--
--   CSR combined constraint: (ticks-3)*div >= 2
--     div=1 => ticks >= 5;  div >= 2 => ticks >= 4.
--   See tdc_gpx_cfg_pkg for legal combination table.
--
-- Invariants (bus safety, see 01_chip_acquisition §6):
--   INV-1: WRITE => OEN = '1' (prevent bus contention)
--   INV-2: READ  => D-bus Hi-Z (FPGA does not drive)
--   INV-3: IDLE  => D-bus Hi-Z, OEN = '1' (oen_permanent exception)
--   INV-5: WRITE->READ turnaround gap (1 tick minimum)
--   INV-6: READ->WRITE OEN='1' leading (1 tick minimum)
--   INV-7: oen_permanent='1' => WRITE forbidden
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_bus_phy is
    generic (
        g_BUS_DATA_WIDTH : natural := c_TDC_BUS_WIDTH      -- 28
    );
    port (
        i_clk           : in  std_logic;
        i_rst_n         : in  std_logic;

        -- Tick enable (1 clk pulse per BUS_CLK_DIV period)
        i_tick_en       : in  std_logic;

        -- Bus timing config (latched internally at transaction entry)
        i_bus_ticks     : in  unsigned(2 downto 0);         -- 3..7

        -- Request interface (from chip_ctrl)
        i_req_valid     : in  std_logic;
        i_req_rw        : in  std_logic;                    -- '0'=READ, '1'=WRITE
        i_req_addr      : in  std_logic_vector(3 downto 0);
        i_req_wdata     : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        i_oen_permanent : in  std_logic;                    -- '1' = drain burst
        i_req_burst     : in  std_logic;                    -- '1' = back-to-back read

        -- Response: busy flag (active during transaction)
        o_busy          : out std_logic;

        -- TDC-GPX physical pins
        o_adr           : out std_logic_vector(3 downto 0);
        o_csn           : out std_logic;
        o_rdn           : out std_logic;
        o_wrn           : out std_logic;
        o_oen           : out std_logic;
        io_d            : inout std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);

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
        o_m_axis_tdata  : out std_logic_vector(31 downto 0);
        o_m_axis_tkeep  : out std_logic_vector(3 downto 0);
        o_m_axis_tuser  : out std_logic_vector(7 downto 0);
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

    -- =========================================================================
    -- FSM states
    -- =========================================================================
    type t_bus_state is (
        ST_IDLE,            -- bus idle, waiting for request
        ST_READ,            -- read transaction (tick counter drives phases)
        ST_WRITE,           -- write transaction (tick counter drives phases)
        ST_TURNAROUND       -- 1-tick gap for direction change (INV-5, INV-6)
    );

    signal s_state_r        : t_bus_state := ST_IDLE;

    -- =========================================================================
    -- Tick counter
    -- In ST_READ/ST_WRITE: runs 1 .. i_bus_ticks-1.
    -- Tick 0 (Phase A) is consumed at the IDLE->ST_READ/ST_WRITE transition.
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
    signal s_d_in            : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    signal s_d_tri_r         : std_logic := '1';    -- '1'=Hi-Z, '0'=drive

    -- IOB FF for read data capture
    signal s_d_in_ff_r       : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_sample_en       : std_logic := '0';

    attribute IOB : string;
    attribute IOB of s_d_in_ff_r : signal is "TRUE";

    -- =========================================================================
    -- Response
    -- =========================================================================
    signal s_rsp_valid_r     : std_logic := '0';
    signal s_rsp_rdata_r     : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_rsp_pending_r   : std_logic := '0';    -- read response deferred by 1 tick
    signal s_busy_r          : std_logic := '0';

    -- AXI-Stream master: bus response mirror (32-bit tdata, 8-bit tuser)
    signal s_axis_tvalid_r   : std_logic := '0';
    signal s_axis_tdata_r    : std_logic_vector(31 downto 0) := (others => '0');
    signal s_axis_tuser_r    : std_logic_vector(7 downto 0) := (others => '0');
    signal s_axis_rw_r       : std_logic := '0';    -- '0'=read, '1'=write (latched at txn entry)
    signal s_axis_addr_r     : std_logic_vector(3 downto 0) := (others => '0');  -- latched addr

    -- Latched bus_ticks: snapshot at transaction entry to prevent
    -- mid-transaction changes from corrupting tick counter comparisons.
    signal s_bus_ticks_r      : unsigned(2 downto 0) := "101";  -- default 5

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

    -- =========================================================================
    -- IOBUF instantiation (bidirectional D-bus)
    -- =========================================================================
    gen_iobuf : for i in 0 to g_BUS_DATA_WIDTH - 1 generate
        u_iobuf : IOBUF
            port map (
                IO => io_d(i),          -- physical pin (bidirectional)
                I  => s_d_out_r(i),     -- FPGA -> chip (write data)
                O  => s_d_in(i),        -- chip -> FPGA (read data)
                T  => s_d_tri_r         -- '1' = Hi-Z, '0' = drive
            );
    end generate gen_iobuf;

    -- =========================================================================
    -- IOB FF: read data capture
    -- Captures one i_clk after s_sample_en is asserted by FSM.
    -- This coincides with RDN physically transitioning high (Phase H start),
    -- which is safe because tH-DR >= 0ns when OEN is held low.
    -- =========================================================================
    p_iob_ff : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if s_sample_en = '1' then
                s_d_in_ff_r <= s_d_in;
            end if;
        end if;
    end process p_iob_ff;

    -- =========================================================================
    -- Main FSM
    --
    -- Transaction tick assignment:
    --   IDLE/TURNAROUND acceptance on tick_en = tick 0 (Phase A: ADR setup)
    --   ST_READ/ST_WRITE ticks = 1 .. i_bus_ticks-1
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
            else
                -- Default: clear single-cycle pulses
                s_rsp_valid_r <= '0';
                s_sample_en   <= '0';

                -- AXI-Stream handshake: clear tvalid when tready accepted
                if s_axis_tvalid_r = '1' and i_m_axis_tready = '1' then
                    s_axis_tvalid_r <= '0';
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
                if s_rsp_pending_r = '1' then
                    s_rsp_valid_r   <= '1';
                    s_rsp_rdata_r   <= s_d_in_ff_r;
                    s_rsp_pending_r <= '0';
                    -- AXI-Stream: read response (zero-pad 28→32 bit)
                    s_axis_tvalid_r             <= '1';
                    s_axis_tdata_r(27 downto 0) <= s_d_in_ff_r;
                    s_axis_tdata_r(31 downto 28)<= (others => '0');
                    s_axis_tuser_r              <= "000" & s_axis_addr_r & '0';
                end if;

                case s_state_r is

                    -- ---------------------------------------------------------
                    -- IDLE: wait for request
                    -- Acceptance on tick_en IS tick 0 (Phase A).
                    -- ---------------------------------------------------------
                    when ST_IDLE =>
                        s_csn_r   <= '1';
                        s_rdn_r   <= '1';
                        s_wrn_r   <= '1';
                        s_d_tri_r <= '1';               -- Hi-Z [INV-3]
                        s_busy_r  <= '0';

                        -- OEN: respect oen_permanent [INV-3 exception]
                        if i_oen_permanent = '1' then
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
                        if i_req_valid = '1' and i_tick_en = '1'
                           and s_rsp_valid_r = '0'
                           and s_rsp_pending_r = '0'
                           and s_axis_tvalid_r = '0' then  -- AXI-Stream response fully consumed
                            s_busy_r <= '1';

                            if i_req_rw = '1' then
                                -- ===== WRITE request =====
                                if i_oen_permanent = '1' then
                                    -- [INV-7] WRITE forbidden during oen_permanent
                                    s_busy_r <= '0';

                                elsif s_last_was_read_r = '1' then
                                    -- [INV-6] Need turnaround: OEN='1' first
                                    s_req_addr_r      <= i_req_addr;
                                    s_req_wdata_r     <= i_req_wdata;
                                    s_oen_r           <= '1';
                                    s_turn_to_write_r <= '1';
                                    s_bus_ticks_r     <= i_bus_ticks;
                                    s_axis_rw_r       <= '1';
                                    s_axis_addr_r     <= i_req_addr;
                                    s_state_r         <= ST_TURNAROUND;

                                else
                                    -- Direct entry: Phase A (tick 0)
                                    s_adr_r       <= i_req_addr;
                                    s_csn_r       <= '0';
                                    s_oen_r       <= '1';           -- [INV-1]
                                    s_d_out_r     <= i_req_wdata;
                                    s_d_tri_r     <= '0';           -- drive D-bus
                                    s_wrn_r       <= '1';           -- high during Phase A
                                    s_bus_ticks_r <= i_bus_ticks;   -- latch config
                                    s_axis_rw_r   <= '1';           -- WRITE
                                    s_axis_addr_r <= i_req_addr;
                                    s_tick_r      <= to_unsigned(1, 3);
                                    s_state_r     <= ST_WRITE;
                                end if;

                            else
                                -- ===== READ request =====
                                if s_last_was_write_r = '1' then
                                    -- [INV-5] Need turnaround
                                    s_req_addr_r      <= i_req_addr;
                                    s_turn_to_write_r <= '0';
                                    s_bus_ticks_r     <= i_bus_ticks;
                                    s_axis_rw_r       <= '0';
                                    s_axis_addr_r     <= i_req_addr;
                                    s_state_r         <= ST_TURNAROUND;

                                else
                                    -- Direct entry: Phase A (tick 0)
                                    s_adr_r       <= i_req_addr;
                                    s_csn_r       <= '0';
                                    s_oen_r       <= '0';           -- chip drives D-bus
                                    s_d_tri_r     <= '1';           -- FPGA Hi-Z [INV-2]
                                    s_rdn_r       <= '1';           -- high during Phase A
                                    s_bus_ticks_r <= i_bus_ticks;   -- latch config
                                    s_axis_rw_r   <= '0';           -- READ
                                    s_axis_addr_r <= i_req_addr;
                                    s_tick_r      <= to_unsigned(1, 3);
                                    s_state_r     <= ST_READ;
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
                                s_tick_r  <= to_unsigned(1, 3);
                                s_state_r <= ST_WRITE;
                            else
                                -- Enter READ Phase A (tick 0)
                                s_adr_r   <= s_req_addr_r;
                                s_csn_r   <= '0';
                                s_oen_r   <= '0';       -- chip drives
                                s_d_tri_r <= '1';       -- Hi-Z [INV-2]
                                s_rdn_r   <= '1';
                                s_tick_r  <= to_unsigned(1, 3);
                                s_state_r <= ST_READ;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    -- READ transaction
                    --
                    -- Entered with Phase A pins already set (tick 0 consumed
                    -- at IDLE/TURNAROUND). s_tick_r starts at 1.
                    --
                    -- Pin control uses independent if-statements so conditions
                    -- can overlap for small BUS_TICKS (e.g., BUS_TICKS=3:
                    -- tick 1 = Phase L start AND sample simultaneously).
                    --
                    -- D-bus = Hi-Z throughout [INV-2]
                    -- OEN = '0' throughout (chip drives D-bus)
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
                                s_rdn_r            <= '1';
                                s_rsp_pending_r    <= '1';
                                s_last_was_write_r <= '0';
                                s_last_was_read_r  <= '1';

                                -- Burst: back-to-back read (no IDLE)
                                -- Restart at tick 0 (Phase A gap) so that
                                -- RDN high = 2 ticks (Phase H + Phase A),
                                -- satisfying tPW-RH >= 6 ns even at div=1
                                -- (2 × 5 ns = 10 ns >= 6 ns).
                                -- CSN, OEN, ADR unchanged throughout burst.
                                if i_oen_permanent = '1'
                                   and i_req_burst = '1' then
                                    -- Burst mode: continue only if response was consumed
                                    if s_axis_tvalid_r = '0' or i_m_axis_tready = '1' then
                                        s_tick_r <= to_unsigned(0, 3);
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
                                    if i_oen_permanent = '0' then
                                        s_oen_r <= '1';
                                    end if;
                                    s_tick_r  <= (others => '0');
                                    s_state_r <= ST_IDLE;
                                end if;
                            else
                                s_tick_r <= s_tick_r + 1;
                            end if;

                        end if;

                    -- ---------------------------------------------------------
                    -- WRITE transaction
                    --
                    -- Entered with Phase A pins already set (tick 0 consumed
                    -- at IDLE/TURNAROUND). s_tick_r starts at 1.
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
                                s_rsp_valid_r      <= '1';
                                s_last_was_write_r <= '1';
                                s_last_was_read_r  <= '0';
                                s_csn_r            <= '1';
                                s_d_tri_r          <= '1';  -- release D-bus
                                s_busy_r           <= '0';
                                s_tick_r           <= (others => '0');
                                s_state_r          <= ST_IDLE;
                                -- AXI-Stream: write ack (tdata=0)
                                s_axis_tvalid_r    <= '1';
                                s_axis_tdata_r     <= (others => '0');
                                s_axis_tuser_r     <= "000" & s_axis_addr_r & '1';
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

    o_busy      <= s_busy_r;

    o_m_axis_tvalid <= s_axis_tvalid_r;
    o_m_axis_tdata  <= s_axis_tdata_r;
    o_m_axis_tkeep  <= "1111";
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
