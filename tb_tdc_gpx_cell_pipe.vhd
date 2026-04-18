-- =============================================================================
-- tb_tdc_gpx_cell_pipe.vhd
-- Testbench for Cluster 3 wrapper (tdc_gpx_cell_pipe)
-- =============================================================================
--
-- Verifies that the slope demux routes rising events to the rising
-- cell_builder and that cell output appears on the rising output port.
--
-- Stimulus: single rising hit on chip 0, stop 0, hit 0, then drain_done
-- sequence (ififo1_done + final_done).
--
-- Standard: VHDL-2008
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_cell_pipe is
end entity;

architecture sim of tb_tdc_gpx_cell_pipe is

    -- =========================================================================
    -- Constants
    -- =========================================================================
    constant CLK_PERIOD    : time    := 5 ns;      -- 200 MHz
    constant OUTPUT_WIDTH  : natural := 64;
    constant WATCHDOG_TIME : time    := 10 us;

    -- =========================================================================
    -- Signals: clock / reset / control
    -- =========================================================================
    signal clk    : std_logic := '0';
    signal rst_n  : std_logic := '0';
    signal done   : boolean   := false;

    -- =========================================================================
    -- DUT input ports
    -- =========================================================================
    signal evt_tvalid         : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');
    signal evt_tdata          : t_slv32_array := (others => (others => '0'));
    signal evt_tuser          : t_slv16_array := (others => (others => '0'));
    signal shot_start         : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');
    signal face_stops         : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal max_hits           : unsigned(2 downto 0) := to_unsigned(7, 3);

    -- =========================================================================
    -- DUT rising output ports
    -- =========================================================================
    signal cell_rise_tdata_0  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_rise_tdata_1  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_rise_tdata_2  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_rise_tdata_3  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_rise_tvalid   : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_rise_tlast    : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_rise_tready   : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');

    -- =========================================================================
    -- DUT falling output ports
    -- =========================================================================
    signal cell_fall_tdata_0  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_fall_tdata_1  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_fall_tdata_2  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_fall_tdata_3  : std_logic_vector(OUTPUT_WIDTH-1 downto 0);
    signal cell_fall_tvalid   : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_fall_tlast    : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal cell_fall_tready   : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');

    -- =========================================================================
    -- DUT status ports
    -- =========================================================================
    signal hit_dropped        : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal hit_fall_dropped   : std_logic_vector(c_N_CHIPS-1 downto 0);

begin

    -- =========================================================================
    -- Clock generation (200 MHz, 5 ns period)
    -- =========================================================================
    clk <= not clk after CLK_PERIOD / 2 when not done else '0';

    -- =========================================================================
    -- DUT instantiation
    -- =========================================================================
    u_dut : entity work.tdc_gpx_cell_pipe
        generic map (
            g_OUTPUT_WIDTH => OUTPUT_WIDTH
        )
        port map (
            i_clk                 => clk,
            i_rst_n               => rst_n,

            i_evt_sk_tvalid       => evt_tvalid,
            i_evt_sk_tdata        => evt_tdata,
            i_evt_sk_tuser        => evt_tuser,

            i_shot_start_per_chip => shot_start,
            i_abort               => '0',
            i_face_stops_per_chip => face_stops,
            i_max_hits_cfg        => max_hits,

            o_cell_rise_tdata_0   => cell_rise_tdata_0,
            o_cell_rise_tdata_1   => cell_rise_tdata_1,
            o_cell_rise_tdata_2   => cell_rise_tdata_2,
            o_cell_rise_tdata_3   => cell_rise_tdata_3,
            o_cell_rise_tvalid    => cell_rise_tvalid,
            o_cell_rise_tlast     => cell_rise_tlast,
            i_cell_rise_tready    => cell_rise_tready,

            o_cell_fall_tdata_0   => cell_fall_tdata_0,
            o_cell_fall_tdata_1   => cell_fall_tdata_1,
            o_cell_fall_tdata_2   => cell_fall_tdata_2,
            o_cell_fall_tdata_3   => cell_fall_tdata_3,
            o_cell_fall_tvalid    => cell_fall_tvalid,
            o_cell_fall_tlast     => cell_fall_tlast,
            i_cell_fall_tready    => cell_fall_tready,

            o_hit_dropped         => hit_dropped,
            o_hit_fall_dropped    => hit_fall_dropped,
            o_shot_dropped        => open,
            o_shot_fall_dropped   => open,
            o_slice_timeout       => open,
            o_slice_fall_timeout  => open
        );

    -- =========================================================================
    -- Stimulus process
    -- =========================================================================
    p_stim : process
        --
        -- tuser encoding for cell_builder input (post-decode):
        --   [0]   = slope       (1=rising, 0=falling)
        --   [2:1] = chip_id
        --   [5:3] = stop_id_local (0..7)
        --   [6]   = ififo_id
        --   [7]   = drain_done  (1=control beat, no data)
        --

        -- Helper: build tuser for a data beat
        function fn_tuser_data(slope, chip_id, stop_id, ififo_id : natural)
            return std_logic_vector is
            variable v : std_logic_vector(15 downto 0) := (others => '0');
        begin
            v(0)          := std_logic(to_unsigned(slope, 1)(0));
            v(2 downto 1) := std_logic_vector(to_unsigned(chip_id, 2));
            v(5 downto 3) := std_logic_vector(to_unsigned(stop_id, 3));
            v(6)          := std_logic(to_unsigned(ififo_id, 1)(0));
            v(7)          := '0';  -- not drain_done
            return v;
        end function;

        -- Helper: build tuser for a drain_done control beat
        function fn_tuser_drain(ififo_id : natural)
            return std_logic_vector is
            variable v : std_logic_vector(15 downto 0) := (others => '0');
        begin
            v(7) := '1';  -- drain_done
            v(6) := std_logic(to_unsigned(ififo_id, 1)(0));
            return v;
        end function;

    begin
        -- -----------------------------------------------------------------
        -- Reset phase (100 ns)
        -- -----------------------------------------------------------------
        rst_n <= '0';
        wait for 100 ns;
        wait until rising_edge(clk);
        rst_n <= '1';
        wait until rising_edge(clk);

        -- -----------------------------------------------------------------
        -- Config is held constant by signal initialisers:
        --   face_stops = 8, max_hits = 7
        -- Chips 1-3: tvalid stays '0', shot_start stays '0'
        -- -----------------------------------------------------------------

        -- -----------------------------------------------------------------
        -- Step 1: shot_start pulse for chip 0
        -- -----------------------------------------------------------------
        shot_start(0) <= '1';
        wait until rising_edge(clk);
        shot_start(0) <= '0';
        wait until rising_edge(clk);

        -- -----------------------------------------------------------------
        -- Step 2: Send one rising hit on chip 0, stop 0, hit 0
        --   tdata = 0x00000001  (hit value = 1)
        --   tuser = slope=1, chip_id=0, stop_id=0, ififo_id=0
        -- -----------------------------------------------------------------
        evt_tdata(0)  <= x"00000001";
        evt_tuser(0)  <= fn_tuser_data(slope => 1, chip_id => 0,
                                       stop_id => 0, ififo_id => 0);
        evt_tvalid(0) <= '1';
        wait until rising_edge(clk);
        evt_tvalid(0) <= '0';
        wait until rising_edge(clk);

        -- -----------------------------------------------------------------
        -- Step 3: Send drain_done -- ififo1_done (tuser[7]=1, tuser[6]=0)
        -- This triggers output of stops 0~3
        -- -----------------------------------------------------------------
        evt_tdata(0)  <= (others => '0');
        evt_tuser(0)  <= fn_tuser_drain(ififo_id => 0);
        evt_tvalid(0) <= '1';
        wait until rising_edge(clk);
        evt_tvalid(0) <= '0';
        wait until rising_edge(clk);

        -- -----------------------------------------------------------------
        -- Step 4: Send drain_done -- final done (tuser[7]=1, tuser[6]=1)
        -- This triggers output of stops 4~7 and buffer swap
        -- -----------------------------------------------------------------
        evt_tuser(0)  <= fn_tuser_drain(ififo_id => 1);
        evt_tvalid(0) <= '1';
        wait until rising_edge(clk);
        evt_tvalid(0) <= '0';

        -- -----------------------------------------------------------------
        -- Wait for cell output to appear (generous margin)
        -- -----------------------------------------------------------------
        wait for 2 us;
        wait until rising_edge(clk);

        -- Stimulus complete -- checker process handles PASS/FAIL
        wait;
    end process p_stim;

    -- =========================================================================
    -- Monitor / checker process
    -- =========================================================================
    p_check : process
        variable v_saw_tvalid : boolean := false;
        variable v_saw_tlast  : boolean := false;
        variable v_tdata_ok   : boolean := false;
    begin
        -- Wait until reset is released
        wait until rst_n = '1';

        -- Monitor rising cell output on chip 0
        lp_monitor : loop
            wait until rising_edge(clk);

            if cell_rise_tvalid(0) = '1' then
                v_saw_tvalid := true;

                -- Check that tdata is non-zero (contains cell data)
                if cell_rise_tdata_0 /= (cell_rise_tdata_0'range => '0') then
                    v_tdata_ok := true;
                end if;

                if cell_rise_tlast(0) = '1' then
                    v_saw_tlast := true;
                    exit lp_monitor;
                end if;
            end if;

            -- Timeout guard inside the loop
            if now > WATCHDOG_TIME then
                exit lp_monitor;
            end if;
        end loop;

        -- -----------------------------------------------------------------
        -- Report results
        -- -----------------------------------------------------------------
        if v_saw_tvalid and v_saw_tlast and v_tdata_ok then
            report "PASS: Rising cell output appeared with valid tdata and tlast."
                severity note;
        else
            if not v_saw_tvalid then
                report "FAIL: o_cell_rise_tvalid(0) never went high."
                    severity error;
            end if;
            if not v_tdata_ok then
                report "FAIL: o_cell_rise_tdata_0 was all-zero (expected cell data)."
                    severity error;
            end if;
            if not v_saw_tlast then
                report "FAIL: o_cell_rise_tlast(0) never fired."
                    severity error;
            end if;
        end if;

        done <= true;
        wait;
    end process p_check;

    -- =========================================================================
    -- Watchdog timeout (10 us)
    -- =========================================================================
    p_watchdog : process
    begin
        wait for WATCHDOG_TIME;
        if not done then
            report "FAIL: Watchdog timeout reached (10 us) -- simulation hung."
                severity failure;
        end if;
        wait;
    end process p_watchdog;

end architecture sim;
