-- =============================================================================
-- tdc_gpx_chip_ctrl.vhd
-- TDC-GPX Controller - SINGLE_SHOT Chip Control FSM
-- =============================================================================
--
-- Purpose:
--   Per-chip SINGLE_SHOT FSM that manages the complete measurement cycle:
--     Powerup -> Init (StopDis, cfg_write, master_reset) -> Idle
--     Idle -> Armed -> Capture -> Drain -> AluTrigger -> Armed (repeat)
--
--   Responsibilities:
--     - Tick enable generation (clock divider from BUS_CLK_DIV)
--     - PuResN powerup sequence
--     - StopDis pin control
--     - cfg_image write sequence (Reg0~7, Reg11, Reg12, Reg14)
--     - Master reset via Reg4 bit22
--     - IrFlag rising-edge detection (MTimer expiry)
--     - EF1-first round-robin IFIFO drain
--     - oen_permanent control: drain_mode='1' → OEN held low during drain
--       (bus_phy INV-7 blocks writes while oen_permanent='1')
--     - n_drain_cap enforcement: limits drain reads to n_drain_cap × 8
--       (0 = unlimited, prevents runaway drain on stuck IFIFO)
--     - AluTrigger pulse generation
--     - Shot sequence counter
--
--   Non-responsibilities:
--     - Bus timing, IOBUF, 2-FF sync   -> bus_phy
--     - Raw word decode                  -> decode_i
--     - Cell building, face assembly     -> cell_builder, face_assembler
--
-- Invariants enforced:
--   [INV-4] EF=1 (empty) -> never issue read to IFIFO
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_chip_ctrl is
    generic (
        g_BUS_DATA_WIDTH    : natural := c_TDC_BUS_WIDTH;   -- 28
        g_CHIP_ID           : natural := 0;                  -- 0..3
        g_POWERUP_CLKS      : natural := 48;     -- PuResN low duration (>200ns, ~240ns @ 200MHz)
        g_RECOVERY_CLKS     : natural := 8;       -- Reset/ALU recovery (~40ns @ 200MHz)
        g_ALU_PULSE_CLKS    : natural := 4        -- AluTrigger pulse width (~20ns @ 200MHz)
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;

        -- CSR configuration (latched at packet_start, stable during frame)
        i_cfg               : in  t_tdc_cfg;
        i_cfg_image         : in  t_cfg_image;

        -- CSR commands (active-high, 1-clk pulses from CSR)
        i_cmd_start         : in  std_logic;        -- IDLE -> ARMED
        i_cmd_stop          : in  std_logic;         -- ARMED/CAPTURE -> IDLE
        i_cmd_soft_reset    : in  std_logic;         -- any -> POWERUP
        i_cmd_cfg_write     : in  std_logic;         -- IDLE -> CFG_WRITE

        -- Shot start (from laser_ctrl, 1-clk pulse)
        i_shot_start        : in  std_logic;

        -- bus_phy request interface
        o_bus_req_valid     : out std_logic;
        o_bus_req_rw        : out std_logic;          -- '0'=READ, '1'=WRITE
        o_bus_req_addr      : out std_logic_vector(3 downto 0);
        o_bus_req_wdata     : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_bus_oen_permanent : out std_logic;
        o_bus_req_burst     : out std_logic;          -- '1' = back-to-back burst read

        -- bus_phy response interface
        i_bus_rsp_valid     : in  std_logic;
        i_bus_rsp_rdata     : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        i_bus_busy          : in  std_logic;

        -- bus_phy synchronized status inputs
        i_ef1_sync          : in  std_logic;          -- '1' = IFIFO1 empty
        i_ef2_sync          : in  std_logic;          -- '1' = IFIFO2 empty
        i_irflag_sync       : in  std_logic;          -- '1' = IrFlag active
        i_lf1_sync          : in  std_logic;          -- '1' = IFIFO1 loaded (reserved)
        i_lf2_sync          : in  std_logic;          -- '1' = IFIFO2 loaded (reserved)

        -- Tick enable output (to bus_phy)
        o_tick_en           : out std_logic;

        -- Physical pin control (directly to TDC-GPX chip)
        o_stopdis           : out std_logic;          -- '1' = stops disabled
        o_alutrigger        : out std_logic;
        o_puresn            : out std_logic;          -- '0' = chip in reset

        -- Data outputs (to decode_i, next pipeline stage)
        o_raw_word          : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_raw_word_valid    : out std_logic;           -- 1-clk pulse per word
        o_ififo_id          : out std_logic;           -- '0'=IFIFO1, '1'=IFIFO2
        o_drain_done        : out std_logic;           -- 1-clk pulse at drain end

        -- Status
        o_shot_seq          : out unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0);
        o_busy              : out std_logic
    );
end entity tdc_gpx_chip_ctrl;

architecture rtl of tdc_gpx_chip_ctrl is

    -- =========================================================================
    -- FSM states
    -- =========================================================================
    type t_chip_state is (
        ST_POWERUP,         -- PuResN low, wait for power stabilization
        ST_PU_RELEASE,      -- PuResN high, wait for chip ready
        ST_STOPDIS_HIGH,    -- StopDis='1', prepare for cfg write
        ST_CFG_WRITE,       -- Set up cfg_image register write request
        ST_CFG_WR_WAIT,     -- Wait for bus_phy write response
        ST_MASTER_RESET,    -- Set up Reg4 master reset write
        ST_MR_WAIT,         -- Wait for master reset response
        ST_RECOVERY,        -- Wait for reset recovery
        ST_STOPDIS_LOW,     -- StopDis='0', enable stops
        ST_IDLE,            -- Ready, waiting for start command
        ST_ARMED,           -- Waiting for shot_start pulse
        ST_CAPTURE,         -- Measurement in progress, waiting for IrFlag
        ST_DRAIN_CHECK,     -- Check EF1/EF2 to decide next read
        ST_DRAIN_EF1,       -- Reading IFIFO1 (Reg8), waiting for response
        ST_DRAIN_EF2,       -- Reading IFIFO2 (Reg9), waiting for response
        ST_DRAIN_BURST,     -- Burst reading: back-to-back reads (LF-gated)
        ST_DRAIN_FLUSH,     -- Post-burst: capture remaining response, wait bus idle
        ST_ALU_PULSE,       -- AluTrigger high for g_ALU_PULSE_CLKS
        ST_ALU_RECOVERY     -- Post-AluTrigger recovery wait
    );

    signal s_state_r        : t_chip_state := ST_POWERUP;

    -- =========================================================================
    -- Timing limit constants (pre-computed from generics, avoids repeated -1)
    -- =========================================================================
    constant c_POWERUP_LAST   : unsigned(15 downto 0) := to_unsigned(g_POWERUP_CLKS - 1, 16);
    constant c_RECOVERY_LAST  : unsigned(15 downto 0) := to_unsigned(g_RECOVERY_CLKS - 1, 16);
    constant c_ALU_PULSE_LAST : unsigned(15 downto 0) := to_unsigned(g_ALU_PULSE_CLKS - 1, 16);

    -- =========================================================================
    -- Wait counter (shared across timed states)
    -- =========================================================================
    signal s_wait_cnt_r     : unsigned(15 downto 0) := (others => '0');

    -- =========================================================================
    -- Cfg write sequence
    -- Register indices to write: Reg0..7, Reg11, Reg12, Reg14
    -- =========================================================================
    type t_reg_idx_array is array(0 to 10) of natural range 0 to 15;
    constant c_CFG_WRITE_SEQ  : t_reg_idx_array := (0, 1, 2, 3, 4, 5, 6, 7, 11, 12, 14);
    constant c_CFG_WRITE_LAST : natural := 10;

    signal s_cfg_idx_r      : unsigned(3 downto 0) := (others => '0');

    -- Init mode flag: '1' during powerup sequence, '0' for runtime cfg_write
    -- Powerup path: cfg_write -> MASTER_RESET -> RECOVERY -> STOPDIS_LOW
    -- Runtime path: cfg_write -> IDLE (no master reset)
    signal s_init_mode_r    : std_logic := '1';

    -- =========================================================================
    -- Bus request registers
    -- =========================================================================
    signal s_req_valid_r    : std_logic := '0';
    signal s_req_rw_r       : std_logic := '0';
    signal s_req_addr_r     : std_logic_vector(3 downto 0) := (others => '0');
    signal s_req_wdata_r    : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Burst read control
    -- =========================================================================
    signal s_req_burst_r    : std_logic := '0';

    -- =========================================================================
    -- Pin control registers
    -- =========================================================================
    signal s_stopdis_r      : std_logic := '1';     -- high at reset (stops disabled)
    signal s_alutrigger_r   : std_logic := '0';
    signal s_puresn_r       : std_logic := '0';     -- low at reset (chip in reset)

    -- =========================================================================
    -- IrFlag rising-edge detection
    -- =========================================================================
    signal s_irflag_prev_r  : std_logic := '0';

    -- =========================================================================
    -- Shot sequence counter
    -- =========================================================================
    signal s_shot_seq_r     : unsigned(c_SHOT_SEQ_WIDTH - 1 downto 0) := (others => '0');

    -- =========================================================================
    -- Data output registers
    -- =========================================================================
    signal s_raw_word_r     : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_raw_valid_r    : std_logic := '0';
    signal s_ififo_id_r     : std_logic := '0';
    signal s_drain_done_r   : std_logic := '0';

    -- =========================================================================
    -- Tick enable generation (BUS_CLK_DIV clock divider)
    -- =========================================================================
    signal s_div_cnt_r      : unsigned(7 downto 0) := (others => '0');
    signal s_tick_en_r      : std_logic := '0';

    -- =========================================================================
    -- OEN permanent control (burst drain mode)
    -- drain_mode='1': asserted during drain, deasserted on drain complete
    -- drain_mode='0': always '0' (legacy SourceGating mode)
    -- =========================================================================
    signal s_oen_permanent_r : std_logic := '0';

    -- =========================================================================
    -- Drain read counter (for n_drain_cap enforcement)
    -- Counts actual IFIFO reads per drain cycle. Reset on drain start.
    -- n_drain_cap=0: unlimited, 1~15: cap at n_drain_cap × 8 reads.
    -- Max drain per chip = 4 stops/IFIFO × 8 hits × 2 IFIFOs = 64 reads.
    -- n_drain_cap=8 → cap at 64 (exact full drain).
    -- =========================================================================
    signal s_drain_cnt_r    : unsigned(6 downto 0) := (others => '0');

    -- =========================================================================
    -- Busy flag
    -- =========================================================================
    signal s_busy_r         : std_logic := '0';

begin

    -- =========================================================================
    -- Generic sanity checks (elaboration-only, ignored by synthesis)
    -- =========================================================================
    assert g_POWERUP_CLKS >= 1
        report "g_POWERUP_CLKS must be >= 1" severity failure;
    assert g_RECOVERY_CLKS >= 1
        report "g_RECOVERY_CLKS must be >= 1" severity failure;
    assert g_ALU_PULSE_CLKS >= 1
        report "g_ALU_PULSE_CLKS must be >= 1" severity failure;

    -- =========================================================================
    -- Tick enable generation
    -- Produces a 1-clk pulse every BUS_CLK_DIV sys_clk cycles.
    -- BUS_CLK_DIV=1: tick_en every cycle. BUS_CLK_DIV=0: clamped to 1.
    -- =========================================================================
    p_tick_en : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_div_cnt_r <= (others => '0');
                s_tick_en_r <= '0';
            else
                if i_cfg.bus_clk_div = 0
                   or s_div_cnt_r = i_cfg.bus_clk_div - 1 then
                    s_div_cnt_r <= (others => '0');
                    s_tick_en_r <= '1';
                else
                    s_div_cnt_r <= s_div_cnt_r + 1;
                    s_tick_en_r <= '0';
                end if;
            end if;
        end if;
    end process p_tick_en;

    o_tick_en <= s_tick_en_r;

    -- =========================================================================
    -- IrFlag edge detection (free-running, not gated by FSM state)
    -- =========================================================================
    p_irflag_edge : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_irflag_prev_r <= '0';
            else
                s_irflag_prev_r <= i_irflag_sync;
            end if;
        end if;
    end process p_irflag_edge;

    -- =========================================================================
    -- OEN permanent output: FSM-controlled during drain phase
    --   drain_mode='1': asserted at drain start, released at drain end
    --   drain_mode='0': always '0' (legacy)
    --   bus_phy enforces INV-7: oen_permanent='1' blocks WRITE requests
    -- =========================================================================
    o_bus_oen_permanent <= s_oen_permanent_r;

    -- =========================================================================
    -- StopDis output: FSM-controlled, with CSR override for debug
    -- stopdis_override[4] = override enable, [g_CHIP_ID] = override value
    -- override enable=0: FSM이 제어 (init 시 '1', 측정 시 '0')
    -- override enable=1: SW가 chip별로 StopDis 강제 설정 (디버그용)
    -- =========================================================================
    o_stopdis <= i_cfg.stopdis_override(g_CHIP_ID)      -- 이 chip의 bit 선택
                 when i_cfg.stopdis_override(4) = '1'   -- override enable
                 else s_stopdis_r;                      -- FSM 제어

    -- =========================================================================
    -- Main FSM
    --
    -- Bus transaction pattern:
    --   SETUP state:  assert s_req_valid_r='1' with addr/rw/wdata,
    --                 transition to WAIT state
    --   WAIT state:   hold request until i_bus_rsp_valid='1',
    --                 clear s_req_valid_r, advance
    --
    -- req_valid is NOT default-cleared (holds its value across clocks).
    -- raw_valid and drain_done ARE default-cleared (1-clk pulses).
    -- =========================================================================
    p_fsm : process(i_clk)
        variable v_reg_num : natural range 0 to 15;
        variable v_wr_data : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_state_r       <= ST_POWERUP;
                s_wait_cnt_r    <= (others => '0');
                s_cfg_idx_r     <= (others => '0');
                s_req_valid_r   <= '0';
                s_req_rw_r      <= '0';
                s_req_addr_r    <= (others => '0');
                s_req_wdata_r   <= (others => '0');
                s_stopdis_r     <= '1';
                s_alutrigger_r  <= '0';
                s_puresn_r      <= '0';
                s_shot_seq_r    <= (others => '0');
                s_raw_word_r    <= (others => '0');
                s_raw_valid_r   <= '0';
                s_ififo_id_r    <= '0';
                s_drain_done_r  <= '0';
                s_busy_r        <= '0';
                s_init_mode_r   <= '1';
                s_oen_permanent_r <= '0';
                s_drain_cnt_r   <= (others => '0');
                s_req_burst_r   <= '0';
            else
                -- Default: clear single-cycle pulses
                s_raw_valid_r  <= '0';
                s_drain_done_r <= '0';

                -- =========================================================
                -- Soft reset: highest priority, returns to POWERUP
                -- =========================================================
                if i_cmd_soft_reset = '1' then
                    s_state_r      <= ST_POWERUP;
                    s_puresn_r     <= '0';
                    s_stopdis_r    <= '1';
                    s_alutrigger_r <= '0';
                    s_req_valid_r  <= '0';
                    s_busy_r       <= '0';
                    s_oen_permanent_r <= '0';
                    s_drain_cnt_r  <= (others => '0');
                    s_req_burst_r  <= '0';
                    s_wait_cnt_r   <= (others => '0');
                    s_cfg_idx_r    <= (others => '0');
                    s_init_mode_r  <= '1';
                else
                    case s_state_r is

                        -- =================================================
                        -- Powerup sequence
                        -- =================================================

                        when ST_POWERUP =>
                            s_puresn_r  <= '0';         -- chip in reset
                            s_stopdis_r <= '1';         -- stops disabled
                            s_busy_r    <= '1';
                            if s_wait_cnt_r = c_POWERUP_LAST then
                                s_wait_cnt_r <= (others => '0');
                                s_state_r    <= ST_PU_RELEASE;
                            else
                                s_wait_cnt_r <= s_wait_cnt_r + 1;
                            end if;

                        when ST_PU_RELEASE =>
                            s_puresn_r <= '1';          -- release chip reset
                            if s_wait_cnt_r = c_RECOVERY_LAST then
                                s_wait_cnt_r <= (others => '0');
                                s_state_r    <= ST_STOPDIS_HIGH;
                            else
                                s_wait_cnt_r <= s_wait_cnt_r + 1;
                            end if;

                        when ST_STOPDIS_HIGH =>
                            s_stopdis_r <= '1';         -- all stops disabled
                            s_cfg_idx_r <= (others => '0');
                            s_state_r   <= ST_CFG_WRITE;

                        -- =================================================
                        -- Cfg write sequence
                        -- Writes Reg0..7, Reg11, Reg12, Reg14 from cfg_image.
                        -- Each register: SETUP -> WAIT -> next or done.
                        -- =================================================

                        when ST_CFG_WRITE =>
                            v_reg_num := c_CFG_WRITE_SEQ(to_integer(s_cfg_idx_r));
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '1';       -- WRITE
                            s_req_addr_r  <= std_logic_vector(to_unsigned(v_reg_num, 4));
                            s_req_wdata_r <= i_cfg_image(v_reg_num)(g_BUS_DATA_WIDTH - 1 downto 0);
                            s_state_r     <= ST_CFG_WR_WAIT;

                        when ST_CFG_WR_WAIT =>
                            if i_bus_rsp_valid = '1' then
                                s_req_valid_r <= '0';
                                if s_cfg_idx_r = c_CFG_WRITE_LAST then
                                    -- Init: full sequence (master reset + recovery)
                                    -- Runtime: just return to IDLE
                                    if s_init_mode_r = '1' then
                                        s_state_r <= ST_MASTER_RESET;
                                    else
                                        s_busy_r  <= '0';
                                        s_state_r <= ST_IDLE;
                                    end if;
                                else
                                    s_cfg_idx_r <= s_cfg_idx_r + 1;
                                    s_state_r   <= ST_CFG_WRITE;
                                end if;
                            end if;

                        -- =================================================
                        -- Master reset: Reg4 with bit 22 forced high
                        -- =================================================

                        when ST_MASTER_RESET =>
                            v_wr_data     := i_cfg_image(4)(g_BUS_DATA_WIDTH - 1 downto 0);
                            v_wr_data(22) := '1';       -- MasterReset bit
                            s_req_valid_r <= '1';
                            s_req_rw_r    <= '1';       -- WRITE
                            s_req_addr_r  <= c_TDC_REG4;
                            s_req_wdata_r <= v_wr_data;
                            s_state_r     <= ST_MR_WAIT;

                        when ST_MR_WAIT =>
                            if i_bus_rsp_valid = '1' then
                                s_req_valid_r <= '0';
                                s_wait_cnt_r  <= (others => '0');
                                s_state_r     <= ST_RECOVERY;
                            end if;

                        when ST_RECOVERY =>
                            if s_wait_cnt_r = c_RECOVERY_LAST then
                                s_wait_cnt_r <= (others => '0');
                                s_state_r    <= ST_STOPDIS_LOW;
                            else
                                s_wait_cnt_r <= s_wait_cnt_r + 1;
                            end if;

                        when ST_STOPDIS_LOW =>
                            s_stopdis_r <= '0';         -- enable stops
                            s_busy_r    <= '0';
                            s_state_r   <= ST_IDLE;

                        -- =================================================
                        -- Idle: ready for start or cfg_write re-trigger
                        -- =================================================

                        when ST_IDLE =>
                            s_busy_r <= '0';
                            if i_cmd_start = '1' then
                                s_stopdis_r <= '0';     -- clear stop-disable for new cycle
                                s_state_r   <= ST_ARMED;
                            elsif i_cmd_cfg_write = '1' then
                                s_cfg_idx_r   <= (others => '0');
                                s_init_mode_r <= '0';   -- runtime: no master reset
                                s_busy_r      <= '1';
                                s_state_r     <= ST_CFG_WRITE;
                            end if;

                        -- =================================================
                        -- Measurement cycle (repeats per shot)
                        -- =================================================

                        when ST_ARMED =>
                            if i_cmd_stop = '1' then
                                s_stopdis_r <= '1';
                                s_state_r   <= ST_IDLE;
                            elsif i_shot_start = '1' then
                                s_busy_r  <= '1';
                                s_state_r <= ST_CAPTURE;
                            end if;

                        when ST_CAPTURE =>
                            -- Wait for IrFlag rising edge (MTimer expired)
                            if i_cmd_stop = '1' then
                                s_stopdis_r <= '1';
                                s_busy_r    <= '0';
                                s_state_r   <= ST_IDLE;
                            elsif i_irflag_sync = '1' and s_irflag_prev_r = '0' then
                                -- Enter drain phase
                                if i_cfg.drain_mode = '1' then
                                    s_oen_permanent_r <= '1';   -- burst: OEN stays low
                                end if;
                                s_drain_cnt_r <= (others => '0');
                                s_state_r     <= ST_DRAIN_CHECK;
                            end if;

                        -- =================================================
                        -- Drain: EF1-first round-robin (Phase 1)
                        -- [INV-4] Only read IFIFO when EF='0' (not empty).
                        -- EF1 priority: stop 0~3 first, then stop 4~7.
                        -- =================================================

                        when ST_DRAIN_CHECK =>
                            -- n_drain_cap enforcement (0 = unlimited)
                            -- Cap threshold = n_drain_cap × 8 reads
                            -- (full drain = 4 stops × 8 hits × 2 IFIFOs = 64 → cap=8)
                            if i_cfg.n_drain_cap /= "0000"
                               and s_drain_cnt_r >= shift_left(
                                   resize(i_cfg.n_drain_cap, 7), 3) then
                                -- Cap reached: force drain complete
                                s_oen_permanent_r <= '0';
                                s_drain_done_r    <= '1';
                                s_wait_cnt_r      <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;

                            -- Burst path: drain_mode='1' AND LF='1' (loaded)
                            -- LF threshold provides safety margin for 2-FF sync
                            -- latency of EF — prevents reading empty FIFO.
                            elsif i_cfg.drain_mode = '1'
                                  and i_ef1_sync = '0' and i_lf1_sync = '1' then
                                -- IFIFO1 loaded: burst read Reg8
                                s_req_valid_r <= '1';
                                s_req_burst_r <= '1';
                                s_req_rw_r    <= '0';   -- READ
                                s_req_addr_r  <= c_TDC_REG8_IFIFO1;
                                s_ififo_id_r  <= '0';
                                s_state_r     <= ST_DRAIN_BURST;
                            elsif i_cfg.drain_mode = '1'
                                  and i_ef2_sync = '0' and i_lf2_sync = '1' then
                                -- IFIFO2 loaded: burst read Reg9
                                s_req_valid_r <= '1';
                                s_req_burst_r <= '1';
                                s_req_rw_r    <= '0';   -- READ
                                s_req_addr_r  <= c_TDC_REG9_IFIFO2;
                                s_ififo_id_r  <= '1';
                                s_state_r     <= ST_DRAIN_BURST;

                            -- Individual read path (legacy or LF='0')
                            elsif i_ef1_sync = '0' then
                                -- IFIFO1 has data -> read Reg8
                                s_req_valid_r <= '1';
                                s_req_rw_r    <= '0';   -- READ
                                s_req_addr_r  <= c_TDC_REG8_IFIFO1;
                                s_ififo_id_r  <= '0';
                                s_state_r     <= ST_DRAIN_EF1;
                            elsif i_ef2_sync = '0' then
                                -- IFIFO2 has data -> read Reg9
                                s_req_valid_r <= '1';
                                s_req_rw_r    <= '0';   -- READ
                                s_req_addr_r  <= c_TDC_REG9_IFIFO2;
                                s_ififo_id_r  <= '1';
                                s_state_r     <= ST_DRAIN_EF2;
                            else
                                -- Both FIFOs empty -> drain complete
                                s_oen_permanent_r <= '0';
                                s_drain_done_r    <= '1';
                                s_wait_cnt_r      <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;
                            end if;

                        when ST_DRAIN_EF1 =>
                            if i_bus_rsp_valid = '1' then
                                s_req_valid_r <= '0';
                                s_raw_word_r  <= i_bus_rsp_rdata;
                                s_raw_valid_r <= '1';
                                s_drain_cnt_r <= s_drain_cnt_r + 1;
                                s_state_r     <= ST_DRAIN_CHECK;
                            end if;

                        when ST_DRAIN_EF2 =>
                            if i_bus_rsp_valid = '1' then
                                s_req_valid_r <= '0';
                                s_raw_word_r  <= i_bus_rsp_rdata;
                                s_raw_valid_r <= '1';
                                s_drain_cnt_r <= s_drain_cnt_r + 1;
                                s_state_r     <= ST_DRAIN_CHECK;
                            end if;

                        -- =================================================
                        -- Burst drain: back-to-back reads (LF-gated)
                        --
                        -- bus_phy stays in ST_READ as long as req_burst='1'.
                        -- Each rsp_valid delivers one word.
                        --
                        -- Stop burst when:
                        --   (a) LF drops (FIFO near-empty, 2-FF sync safety)
                        --   (b) n_drain_cap reached
                        --
                        -- On burst stop: deassert req_burst → bus_phy will
                        -- complete the in-flight read (1 extra response) and
                        -- return to IDLE. ST_DRAIN_FLUSH captures this.
                        -- =================================================

                        when ST_DRAIN_BURST =>
                            if i_bus_rsp_valid = '1' then
                                -- Capture read data
                                s_raw_word_r  <= i_bus_rsp_rdata;
                                s_raw_valid_r <= '1';
                                s_drain_cnt_r <= s_drain_cnt_r + 1;

                                -- Check burst stop conditions on NEXT count
                                -- (s_drain_cnt_r+1 hasn't taken effect yet,
                                --  so compare with current+1)
                                if (s_ififo_id_r = '0' and i_lf1_sync = '0')
                                   or (s_ififo_id_r = '1' and i_lf2_sync = '0') then
                                    -- LF dropped: FIFO near-empty, stop burst
                                    s_req_burst_r <= '0';
                                    s_req_valid_r <= '0';
                                    s_state_r     <= ST_DRAIN_FLUSH;
                                elsif i_cfg.n_drain_cap /= "0000"
                                      and (s_drain_cnt_r + 1) >= shift_left(
                                          resize(i_cfg.n_drain_cap, 7), 3) then
                                    -- Drain cap reached: stop burst
                                    s_req_burst_r <= '0';
                                    s_req_valid_r <= '0';
                                    s_state_r     <= ST_DRAIN_FLUSH;
                                end if;
                                -- else: keep bursting (req_burst stays '1')
                            end if;

                        -- =================================================
                        -- Post-burst flush: capture remaining in-flight
                        -- response (bus_phy completes the last started read),
                        -- then return to ST_DRAIN_CHECK for re-evaluation.
                        -- =================================================

                        when ST_DRAIN_FLUSH =>
                            -- Capture any remaining response
                            if i_bus_rsp_valid = '1' then
                                s_raw_word_r  <= i_bus_rsp_rdata;
                                s_raw_valid_r <= '1';
                                s_drain_cnt_r <= s_drain_cnt_r + 1;
                            end if;
                            -- When bus_phy returns to IDLE: re-evaluate drain
                            if i_bus_busy = '0' and i_bus_rsp_valid = '0' then
                                s_state_r <= ST_DRAIN_CHECK;
                            end if;

                        -- =================================================
                        -- AluTrigger pulse + recovery -> next shot
                        -- =================================================

                        when ST_ALU_PULSE =>
                            s_alutrigger_r <= '1';
                            if s_wait_cnt_r = c_ALU_PULSE_LAST then
                                s_alutrigger_r <= '0';
                                s_wait_cnt_r   <= (others => '0');
                                s_state_r      <= ST_ALU_RECOVERY;
                            else
                                s_wait_cnt_r <= s_wait_cnt_r + 1;
                            end if;

                        when ST_ALU_RECOVERY =>
                            s_alutrigger_r <= '0';
                            if s_wait_cnt_r = c_RECOVERY_LAST then
                                s_wait_cnt_r <= (others => '0');
                                s_shot_seq_r <= s_shot_seq_r + 1;
                                s_busy_r     <= '0';
                                s_state_r    <= ST_ARMED;
                            else
                                s_wait_cnt_r <= s_wait_cnt_r + 1;
                            end if;

                        when others =>
                            s_state_r <= ST_IDLE;

                    end case;
                end if;  -- soft_reset
            end if;  -- rst_n
        end if;  -- rising_edge
    end process p_fsm;

    -- =========================================================================
    -- Output assignments (directly from registers)
    -- =========================================================================
    o_bus_req_valid  <= s_req_valid_r;
    o_bus_req_rw     <= s_req_rw_r;
    o_bus_req_addr   <= s_req_addr_r;
    o_bus_req_wdata  <= s_req_wdata_r;
    o_bus_req_burst  <= s_req_burst_r;

    o_alutrigger     <= s_alutrigger_r;
    o_puresn         <= s_puresn_r;

    o_raw_word       <= s_raw_word_r;
    o_raw_word_valid <= s_raw_valid_r;
    o_ififo_id       <= s_ififo_id_r;
    o_drain_done     <= s_drain_done_r;

    o_shot_seq       <= s_shot_seq_r;
    o_busy           <= s_busy_r;

end architecture rtl;
