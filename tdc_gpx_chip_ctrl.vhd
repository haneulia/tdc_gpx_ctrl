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
--     - Individual register read/write (from IDLE, for runtime access)
--     - Master reset via Reg4 bit22
--     - IrFlag rising-edge detection (MTimer expiry)
--     - EF1-first round-robin IFIFO drain
--     - Count-based burst drain: LF='1' → read exactly Fill times
--       (Fill = cfg_image[6][7:0], TDC-GPX Register 6 LF threshold)
--       No LF/EF check during burst — deterministic count-based exit.
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

        -- Individual register access (from CSR, 1-clk pulses, IDLE only)
        i_cmd_reg_read      : in  std_logic;
        i_cmd_reg_write     : in  std_logic;
        i_cmd_reg_addr      : in  std_logic_vector(3 downto 0);
        i_cmd_reg_wdata     : in  std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_cmd_reg_rdata     : out std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0);
        o_cmd_reg_rvalid    : out std_logic;         -- 1-clk pulse with read data

        -- Shot start (from laser_ctrl, 1-clk pulse)
        i_shot_start        : in  std_logic;

        -- Max range clock budget (from cfg, latched at shot_start)
        i_max_range_clks    : in  unsigned(15 downto 0);

        -- External stop signal (from laser_ctrl, 1-clk pulse, i_axis_aclk domain)
        -- Used for ERROR DETECTION ONLY — NOT a drain trigger.
        -- If stop_tdc rises before IrFlag in ST_CAPTURE → err_sequence.
        -- Rationale: stop_tdc means "max range reached, no more returns expected,"
        -- but IrFlag (MTimer expiry) must ALWAYS arrive first to guarantee
        -- IFIFO writes are settled. If stop_tdc arrives first, the MTimer
        -- setting is misconfigured relative to the actual measurement window.
        i_stop_tdc          : in  std_logic;

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
        o_busy              : out std_logic;

        -- Error flags (1-clk pulses)
        o_err_drain_timeout : out std_logic;    -- max_range_clks expired before drain_done
        o_err_sequence      : out std_logic     -- IrFlag expected but not yet received
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
        ST_DRAIN_BURST,     -- Burst reading: count-based (Fill reads, no LF/EF check)
        ST_DRAIN_FLUSH,     -- Post-burst: capture remaining response, wait bus idle
        ST_DRAIN_SETTLE,    -- EF/LF flag settling wait (tS-EF ≤ 11.8ns → 3 clk @ 200MHz)
        ST_REG_ACCESS,      -- Individual register read/write (from IDLE)
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

    -- EF/LF flag settling guard after last read.
    -- TDC-GPX datasheet: tS-EF (Empty Flag Set Time) ≤ 11.8 ns max.
    -- LF has NO timing spec — datasheet states:
    --   "The load-level flag is valid only if it is not read from this FIFO."
    -- After bus idle (no reads), 3 clk × 5ns = 15ns > 11.8ns → EF/LF stable.
    -- Plus 2-FF synchronizer adds 2 clk on top → total margin ≈ 25ns.
    constant c_FLAG_SETTLE_LAST : unsigned(15 downto 0) := to_unsigned(2, 16);  -- 0,1,2 = 3 clocks

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
    -- stop_tdc rising-edge detection (for err_sequence only)
    -- =========================================================================
    signal s_stop_tdc_prev_r : std_logic := '0';

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
    -- Burst read counter (Fill-based: count reads within one burst session)
    -- Fill = cfg_image[6][7:0] = TDC-GPX Register 6 LF threshold.
    -- LF='1' guarantees >= Fill entries in FIFO.
    -- s_fill_r: latched at drain entry (FF boundary for 200MHz timing).
    -- s_burst_limit_r: copied from s_fill_r at burst entry.
    -- s_burst_cnt_r: counts responses within one burst session.
    -- =========================================================================
    signal s_fill_r         : unsigned(7 downto 0) := (others => '0');
    signal s_burst_cnt_r    : unsigned(7 downto 0) := (others => '0');
    signal s_burst_limit_r  : unsigned(7 downto 0) := (others => '0');

    -- =========================================================================
    -- Individual register access (read/write single TDC-GPX register)
    -- =========================================================================
    signal s_reg_rdata_r    : std_logic_vector(g_BUS_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal s_reg_rvalid_r   : std_logic := '0';

    -- =========================================================================
    -- Shot range counter: counts clocks from shot_start.
    -- Used to detect drain timeout (max_range_clks reached before drain_done).
    -- =========================================================================
    signal s_range_cnt_r        : unsigned(15 downto 0) := (others => '0');
    signal s_range_active_r     : std_logic := '0';     -- '1' during capture+drain (p_fsm)
    signal s_range_active_prev_r : std_logic := '0';    -- edge detect for counter reset
    signal s_err_drain_timeout_r : std_logic := '0';
    signal s_err_sequence_r     : std_logic := '0';

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
                s_irflag_prev_r  <= '0';
                s_stop_tdc_prev_r <= '0';
            else
                s_irflag_prev_r  <= i_irflag_sync;
                s_stop_tdc_prev_r <= i_stop_tdc;
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
                s_fill_r        <= (others => '0');
                s_burst_cnt_r   <= (others => '0');
                s_burst_limit_r <= (others => '0');
                s_reg_rdata_r   <= (others => '0');
                s_reg_rvalid_r  <= '0';
                s_range_active_r <= '0';
            else
                -- Default: clear single-cycle pulses
                s_raw_valid_r  <= '0';
                s_drain_done_r <= '0';
                s_reg_rvalid_r <= '0';

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
                    s_fill_r       <= (others => '0');
                    s_burst_cnt_r  <= (others => '0');
                    s_burst_limit_r <= (others => '0');
                    s_range_active_r <= '0';
                    s_wait_cnt_r   <= (others => '0');
                    s_cfg_idx_r    <= (others => '0');
                    s_init_mode_r  <= '1';
                    s_shot_seq_r   <= (others => '0');
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
                            elsif i_cmd_reg_read = '1' then
                                -- Individual register read
                                s_req_valid_r <= '1';
                                s_req_rw_r    <= '0';   -- READ
                                s_req_addr_r  <= i_cmd_reg_addr;
                                s_busy_r      <= '1';
                                s_state_r     <= ST_REG_ACCESS;
                            elsif i_cmd_reg_write = '1' then
                                -- Individual register write
                                s_req_valid_r <= '1';
                                s_req_rw_r    <= '1';   -- WRITE
                                s_req_addr_r  <= i_cmd_reg_addr;
                                s_req_wdata_r <= i_cmd_reg_wdata;
                                s_busy_r      <= '1';
                                s_state_r     <= ST_REG_ACCESS;
                            end if;

                        -- =================================================
                        -- Measurement cycle (repeats per shot)
                        -- =================================================

                        when ST_ARMED =>
                            if i_cmd_stop = '1' then
                                s_stopdis_r <= '1';
                                s_state_r   <= ST_IDLE;
                            elsif i_shot_start = '1' then
                                s_busy_r         <= '1';
                                s_range_active_r <= '1';
                                s_state_r        <= ST_CAPTURE;
                            end if;

                        when ST_CAPTURE =>
                            -- ==============================================
                            -- DESIGN POINT: IrFlag is the ONLY drain trigger
                            -- ==============================================
                            --
                            -- IrFlag = TDC-GPX internal MTimer expiry.
                            -- It guarantees that ALL pending stop events have
                            -- been written to the IFIFOs before it asserts.
                            --
                            -- External signals (e.g. stop_tdc from laser_ctrl)
                            -- must NOT bypass IrFlag as a drain trigger because:
                            --   1. External timing has no visibility into TDC-GPX
                            --      internal IFIFO write pipeline latency.
                            --   2. Draining before IFIFO writes complete causes
                            --      DATA LOSS — EF may read '1' (empty) while
                            --      data is still being written internally.
                            --   3. EF can transition 0→1→0 multiple times as
                            --      stop events arrive asynchronously during the
                            --      measurement window — EF='1' before IrFlag
                            --      does NOT mean "no more data coming."
                            --
                            -- The safe sequence is:
                            --   shot_start → stops arrive → MTimer expires →
                            --   IrFlag↑ (IFIFO settled) → drain → drain_done
                            --
                            -- Error detection:
                            --   max_range_clks budget covers capture + drain.
                            --   If drain_done has not fired by max_range_clks,
                            --   the shot has overrun → err_drain_timeout.
                            --
                            if i_cmd_stop = '1' then
                                s_stopdis_r      <= '1';
                                s_busy_r         <= '0';
                                s_range_active_r <= '0';
                                s_state_r        <= ST_IDLE;

                            elsif i_irflag_sync = '1' and s_irflag_prev_r = '0' then
                                -- MTimer expired: IFIFO settled, safe to drain.
                                -- Latch Fill value at drain entry (FF boundary
                                -- for 200MHz timing — no live cfg_image in FSM body)
                                s_fill_r      <= unsigned(i_cfg_image(6)(7 downto 0));
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
                                s_range_active_r  <= '0';
                                s_wait_cnt_r      <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;

                            -- Burst path: drain_mode='1' AND LF='1' (loaded)
                            -- s_fill_r = latched Fill value (registered at drain entry).
                            -- LF='1' guarantees >= Fill entries in FIFO.
                            -- Burst exactly Fill times (count-based, no LF/EF check
                            -- during burst). Fill < 2 → skip burst, use EF path.
                            elsif i_cfg.drain_mode = '1'
                                  and s_fill_r >= 2
                                  and i_ef1_sync = '0' and i_lf1_sync = '1' then
                                -- IFIFO1 loaded: burst read Reg8
                                s_burst_limit_r <= s_fill_r;
                                s_burst_cnt_r   <= (others => '0');
                                s_req_valid_r   <= '1';
                                s_req_burst_r   <= '1';
                                s_req_rw_r      <= '0';   -- READ
                                s_req_addr_r    <= c_TDC_REG8_IFIFO1;
                                s_ififo_id_r    <= '0';
                                s_state_r       <= ST_DRAIN_BURST;
                            elsif i_cfg.drain_mode = '1'
                                  and s_fill_r >= 2
                                  and i_ef2_sync = '0' and i_lf2_sync = '1' then
                                -- IFIFO2 loaded: burst read Reg9
                                s_burst_limit_r <= s_fill_r;
                                s_burst_cnt_r   <= (others => '0');
                                s_req_valid_r   <= '1';
                                s_req_burst_r   <= '1';
                                s_req_rw_r      <= '0';   -- READ
                                s_req_addr_r    <= c_TDC_REG9_IFIFO2;
                                s_ififo_id_r    <= '1';
                                s_state_r       <= ST_DRAIN_BURST;

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
                                s_range_active_r  <= '0';
                                s_wait_cnt_r      <= (others => '0');
                                s_state_r         <= ST_ALU_PULSE;
                            end if;

                        when ST_DRAIN_EF1 =>
                            if i_bus_rsp_valid = '1' then
                                s_req_valid_r <= '0';
                                s_raw_word_r  <= i_bus_rsp_rdata;
                                s_raw_valid_r <= '1';
                                s_drain_cnt_r <= s_drain_cnt_r + 1;
                                -- Settle before re-checking EF/LF
                                -- (tS-EF + 2-FF sync must complete)
                                s_wait_cnt_r  <= (others => '0');
                                s_state_r     <= ST_DRAIN_SETTLE;
                            end if;

                        when ST_DRAIN_EF2 =>
                            if i_bus_rsp_valid = '1' then
                                s_req_valid_r <= '0';
                                s_raw_word_r  <= i_bus_rsp_rdata;
                                s_raw_valid_r <= '1';
                                s_drain_cnt_r <= s_drain_cnt_r + 1;
                                -- Settle before re-checking EF/LF
                                s_wait_cnt_r  <= (others => '0');
                                s_state_r     <= ST_DRAIN_SETTLE;
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
                                s_burst_cnt_r <= s_burst_cnt_r + 1;

                                -- Count-based burst stop (deterministic):
                                -- LF='1' at entry guaranteed >= Fill entries.
                                -- Read exactly Fill times, then re-evaluate via
                                -- ST_DRAIN_FLUSH -> ST_DRAIN_CHECK (EF stable).
                                if (s_burst_cnt_r + 1) >= s_burst_limit_r then
                                    -- Fill count reached: stop burst
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
                            -- When bus_phy returns to IDLE: enter settling wait
                            -- (EF/LF need time to stabilize after last read)
                            if i_bus_busy = '0' and i_bus_rsp_valid = '0' then
                                s_wait_cnt_r <= (others => '0');
                                s_state_r    <= ST_DRAIN_SETTLE;
                            end if;

                        -- =================================================
                        -- Flag settling guard (tS-EF ≤ 11.8ns + 2-FF sync)
                        --
                        -- After the last RDN↑, TDC-GPX updates EF/LF pins
                        -- with propagation delay tS-EF (max 11.8ns).
                        -- Then 2-FF synchronizer adds 2 clk (10ns @ 200MHz).
                        -- This state waits 3 clk (15ns) after bus idle,
                        -- ensuring EF/LF sync outputs are stable before
                        -- ST_DRAIN_CHECK re-evaluates them.
                        --
                        -- Total settling budget:
                        --   bus_phy hold phase (≥1 tick) + 3 clk + 2-FF
                        --   = well over 11.8ns + 10ns requirement.
                        -- =================================================

                        when ST_DRAIN_SETTLE =>
                            if s_wait_cnt_r = c_FLAG_SETTLE_LAST then
                                s_wait_cnt_r <= (others => '0');
                                s_state_r    <= ST_DRAIN_CHECK;
                            else
                                s_wait_cnt_r <= s_wait_cnt_r + 1;
                            end if;

                        -- =================================================
                        -- Individual register access (from IDLE)
                        -- Waits for bus response, captures read data,
                        -- returns to IDLE.
                        -- =================================================

                        when ST_REG_ACCESS =>
                            if i_bus_rsp_valid = '1' then
                                s_req_valid_r  <= '0';
                                s_reg_rdata_r  <= i_bus_rsp_rdata;
                                s_reg_rvalid_r <= '1';  -- 1-clk pulse (also for writes as ack)
                                s_busy_r       <= '0';
                                s_state_r      <= ST_IDLE;
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
    -- Range counter: counts clocks from shot_start through drain_done.
    -- Detects two error conditions:
    --   (1) err_drain_timeout: max_range_clks reached before drain_done.
    --       Means the shot took too long (IrFlag late, or drain stalled).
    --   (2) err_sequence: stop_tdc rising edge detected in ST_CAPTURE
    --       before IrFlag rising edge. This means the external max-range
    --       signal arrived before TDC-GPX MTimer expired — indicates
    --       MTimer is misconfigured (too long) relative to actual range.
    -- Both are 1-clk pulses, cleared each cycle.
    -- =========================================================================
    p_range_cnt : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                s_range_cnt_r         <= (others => '0');
                s_range_active_prev_r <= '0';
                s_err_drain_timeout_r <= '0';
                s_err_sequence_r      <= '0';
            else
                -- Default: clear 1-clk pulses
                s_err_drain_timeout_r <= '0';
                s_err_sequence_r      <= '0';

                -- Edge detect on range_active (driven by p_fsm)
                s_range_active_prev_r <= s_range_active_r;

                -- (1) Range counter → err_drain_timeout
                if s_range_active_r = '1' and s_range_active_prev_r = '0' then
                    -- Rising edge: new shot started, reset counter
                    s_range_cnt_r <= (others => '0');
                elsif s_range_active_r = '1' then
                    if s_range_cnt_r >= i_max_range_clks then
                        -- Budget exhausted before drain_done
                        s_err_drain_timeout_r <= '1';
                        -- Don't clear range_active here — FSM will clear it
                        -- when drain_done eventually fires (or on soft_reset).
                    else
                        s_range_cnt_r <= s_range_cnt_r + 1;
                    end if;
                end if;

                -- (2) Sequence error: stop_tdc↑ before IrFlag↑ in ST_CAPTURE
                -- Normal sequence: shot_start → IrFlag↑ → drain → stop_tdc↑
                -- Error: stop_tdc↑ arrives while still in ST_CAPTURE
                --        (i.e., IrFlag has NOT yet risen)
                if s_state_r = ST_CAPTURE then
                    if i_stop_tdc = '1' and s_stop_tdc_prev_r = '0' then
                        s_err_sequence_r <= '1';
                    end if;
                end if;
            end if;
        end if;
    end process p_range_cnt;

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

    o_cmd_reg_rdata  <= s_reg_rdata_r;
    o_cmd_reg_rvalid <= s_reg_rvalid_r;

    o_err_drain_timeout <= s_err_drain_timeout_r;
    o_err_sequence      <= s_err_sequence_r;

end architecture rtl;
