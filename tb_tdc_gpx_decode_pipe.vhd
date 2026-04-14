--------------------------------------------------------------------------------
-- tb_tdc_gpx_decode_pipe.vhd
--
-- VHDL-2008 testbench for tdc_gpx_decode_pipe (Cluster 2 wrapper).
-- Verifies that raw AXI-Stream data passes through the decode pipeline
-- and that chip_id enrichment is applied correctly on the output tuser.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tb_tdc_gpx_decode_pipe is
end entity tb_tdc_gpx_decode_pipe;

architecture sim of tb_tdc_gpx_decode_pipe is

    -- Clock period: 200 MHz => 5 ns
    constant C_CLK_PERIOD : time := 5 ns;
    constant C_WATCHDOG   : time := 10 us;

    -- DUT ports
    signal clk   : std_logic := '0';
    signal rst_n : std_logic := '0';

    signal raw_sk_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');
    signal raw_sk_tdata  : t_slv32_array := (others => (others => '0'));
    signal raw_sk_tuser  : t_slv8_array  := (others => (others => '0'));
    signal raw_sk_tready : std_logic_vector(c_N_CHIPS-1 downto 0);

    signal chip_shot_seq      : t_shot_seq_array := (others => (others => '0'));
    signal face_stops_per_chip : unsigned(3 downto 0) := to_unsigned(8, 4);

    signal evt_sk_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal evt_sk_tdata  : t_slv32_array;
    signal evt_sk_tuser  : t_slv16_array;

    signal stop_id_error : std_logic_vector(c_N_CHIPS-1 downto 0);

    -- Testbench control
    signal sim_done : boolean := false;

begin

    --------------------------------------------------------------------------
    -- Clock generation
    --------------------------------------------------------------------------
    clk <= not clk after C_CLK_PERIOD / 2 when not sim_done else '0';

    --------------------------------------------------------------------------
    -- DUT instantiation
    --------------------------------------------------------------------------
    u_dut : entity work.tdc_gpx_decode_pipe
        port map (
            i_clk                 => clk,
            i_rst_n               => rst_n,
            i_raw_sk_tvalid       => raw_sk_tvalid,
            i_raw_sk_tdata        => raw_sk_tdata,
            i_raw_sk_tuser        => raw_sk_tuser,
            o_raw_sk_tready       => raw_sk_tready,
            i_chip_shot_seq       => chip_shot_seq,
            i_face_stops_per_chip => face_stops_per_chip,
            o_evt_sk_tvalid       => evt_sk_tvalid,
            o_evt_sk_tdata        => evt_sk_tdata,
            o_evt_sk_tuser        => evt_sk_tuser,
            o_stop_id_error       => stop_id_error
        );

    --------------------------------------------------------------------------
    -- Watchdog: abort if simulation runs too long
    --------------------------------------------------------------------------
    p_watchdog : process
    begin
        wait for C_WATCHDOG;
        if not sim_done then
            report "WATCHDOG TIMEOUT after " & time'image(C_WATCHDOG)
                severity failure;
        end if;
        wait;
    end process p_watchdog;

    --------------------------------------------------------------------------
    -- Stimulus process
    --------------------------------------------------------------------------
    p_stim : process
        variable v_pass_count : natural := 0;
        variable v_fail_count : natural := 0;

        -- Helper: wait one rising clock edge
        procedure clk_edge is
        begin
            wait until rising_edge(clk);
        end procedure;

        -- Helper: drive one AXI-S beat on chip 0 input
        procedure drive_beat_chip0(
            tdata : std_logic_vector(31 downto 0);
            tuser : std_logic_vector(7 downto 0)
        ) is
        begin
            raw_sk_tvalid(0) <= '1';
            raw_sk_tdata(0)  <= tdata;
            raw_sk_tuser(0)  <= tuser;
            clk_edge;
            -- Wait for ready handshake
            while raw_sk_tready(0) /= '1' loop
                clk_edge;
            end loop;
            -- Beat accepted on this edge; deassert on next
            raw_sk_tvalid(0) <= '0';
            raw_sk_tdata(0)  <= (others => '0');
            raw_sk_tuser(0)  <= (others => '0');
        end procedure;

        -- Helper: wait for a valid output beat on chip 0 with timeout
        procedure wait_output_chip0(
            timeout_cycles : natural := 200
        ) is
            variable cnt : natural := 0;
        begin
            loop
                clk_edge;
                exit when evt_sk_tvalid(0) = '1';
                cnt := cnt + 1;
                assert cnt < timeout_cycles
                    report "FAIL: Timed out waiting for output tvalid on chip 0"
                    severity failure;
            end loop;
        end procedure;

    begin
        -- ---------------------------------------------------------------
        -- Reset phase
        -- ---------------------------------------------------------------
        rst_n <= '0';
        raw_sk_tvalid <= (others => '0');
        raw_sk_tdata  <= (others => (others => '0'));
        raw_sk_tuser  <= (others => (others => '0'));
        chip_shot_seq <= (others => (others => '0'));
        face_stops_per_chip <= to_unsigned(8, 4);

        wait for 100 ns;
        clk_edge;
        rst_n <= '1';
        -- Let reset propagate
        for i in 1 to 4 loop
            clk_edge;
        end loop;

        report "INFO: Reset released, starting stimulus";

        -- ---------------------------------------------------------------
        -- Send hit beat 0 on chip 0
        -- tdata[16:0] = hit data (arbitrary value 0x0001_2345)
        -- tuser: slope=0 [0], cha_code="01" [2:1], stop_id="001" [5:3],
        --        ififo_id=0 [6], drain_done=0 [7]
        -- tuser = 0b_0_0_001_01_0 = 0x0A
        -- ---------------------------------------------------------------
        report "INFO: Driving hit beat 0 on chip 0";
        drive_beat_chip0(
            tdata => x"0001_2345",
            tuser => "00001010"  -- ififo=0, stop=001, cha=01, slope=0
        );

        -- ---------------------------------------------------------------
        -- Send hit beat 1 on chip 0
        -- tdata[16:0] = 0x0000_6789
        -- tuser: slope=1, cha_code="10", stop_id="010", ififo_id=0
        -- tuser = 0b_0_0_010_10_1 = 0x15
        -- ---------------------------------------------------------------
        report "INFO: Driving hit beat 1 on chip 0";
        drive_beat_chip0(
            tdata => x"0000_6789",
            tuser => "00010101"  -- ififo=0, stop=010, cha=10, slope=1
        );

        -- ---------------------------------------------------------------
        -- Send drain_done for ififo0: tuser[7]=1, tuser[6]=0
        -- ---------------------------------------------------------------
        report "INFO: Driving drain_done (ififo0) on chip 0";
        drive_beat_chip0(
            tdata => x"0000_0000",
            tuser => "10000000"  -- drain_done=1, ififo_id=0
        );

        -- ---------------------------------------------------------------
        -- Send drain_done for ififo1 (all done): tuser[7]=1, tuser[6]=1
        -- ---------------------------------------------------------------
        report "INFO: Driving drain_done (all done) on chip 0";
        drive_beat_chip0(
            tdata => x"0000_0000",
            tuser => "11000000"  -- drain_done=1, ififo_id=1
        );

        -- ---------------------------------------------------------------
        -- Monitor outputs: collect beats from chip 0
        -- Pipeline latency is several cycles; wait for output beats.
        -- ---------------------------------------------------------------
        report "INFO: Waiting for output beats on chip 0";

        -- Beat 0: first hit
        wait_output_chip0(timeout_cycles => 300);
        -- Check chip_id in tuser[2:1] = "00" (chip 0)
        if evt_sk_tuser(0)(2 downto 1) = "00" then
            report "PASS: Beat 0 chip_id = 00 (correct for chip 0)";
            v_pass_count := v_pass_count + 1;
        else
            report "FAIL: Beat 0 chip_id = " &
                   to_string(evt_sk_tuser(0)(2 downto 1)) &
                   ", expected 00"
                severity error;
            v_fail_count := v_fail_count + 1;
        end if;

        -- Beat 1: second hit
        wait_output_chip0(timeout_cycles => 200);
        if evt_sk_tuser(0)(2 downto 1) = "00" then
            report "PASS: Beat 1 chip_id = 00 (correct for chip 0)";
            v_pass_count := v_pass_count + 1;
        else
            report "FAIL: Beat 1 chip_id = " &
                   to_string(evt_sk_tuser(0)(2 downto 1)) &
                   ", expected 00"
                severity error;
            v_fail_count := v_fail_count + 1;
        end if;

        -- Beat 2: drain_done (ififo0)
        wait_output_chip0(timeout_cycles => 200);
        if evt_sk_tuser(0)(7) = '1' then
            report "PASS: Beat 2 drain_done flag set";
            v_pass_count := v_pass_count + 1;
        else
            report "FAIL: Beat 2 drain_done flag not set"
                severity error;
            v_fail_count := v_fail_count + 1;
        end if;

        -- Beat 3: drain_done (all done)
        wait_output_chip0(timeout_cycles => 200);
        if evt_sk_tuser(0)(7) = '1' then
            report "PASS: Beat 3 drain_done (all done) flag set";
            v_pass_count := v_pass_count + 1;
        else
            report "FAIL: Beat 3 drain_done (all done) flag not set"
                severity error;
            v_fail_count := v_fail_count + 1;
        end if;

        -- ---------------------------------------------------------------
        -- Check that other chip outputs stayed quiet
        -- ---------------------------------------------------------------
        clk_edge;
        if evt_sk_tvalid(1) = '0' and
           evt_sk_tvalid(2) = '0' and
           evt_sk_tvalid(3) = '0' then
            report "PASS: Chips 1-3 outputs remain idle";
            v_pass_count := v_pass_count + 1;
        else
            report "FAIL: Unexpected tvalid on chips 1-3"
                severity error;
            v_fail_count := v_fail_count + 1;
        end if;

        -- ---------------------------------------------------------------
        -- Summary
        -- ---------------------------------------------------------------
        report "============================================";
        if v_fail_count = 0 then
            report "ALL TESTS PASSED (" &
                   integer'image(v_pass_count) & " checks)";
        else
            report "TESTS COMPLETED: " &
                   integer'image(v_pass_count) & " passed, " &
                   integer'image(v_fail_count) & " FAILED"
                severity error;
        end if;
        report "============================================";

        sim_done <= true;
        wait;
    end process p_stim;

end architecture sim;
