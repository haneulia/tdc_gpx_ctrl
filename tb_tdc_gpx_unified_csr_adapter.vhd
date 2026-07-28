-- =============================================================================
-- tb_tdc_gpx_unified_csr_adapter.vhd
-- Focused unified TDC CSR transaction/CDC/status/IRQ regression
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

library work;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_tdc_gpx_unified_csr_adapter is
end entity tb_tdc_gpx_unified_csr_adapter;

architecture sim of tb_tdc_gpx_unified_csr_adapter is
    constant C_CFG_PERIOD  : time := 10 ns;
    constant C_AXIS_PERIOD : time := 20 ns / 3;

    type t_count_array is array(0 to 5) of natural;
    type t_irq_count_array is array(0 to 6) of natural;

    signal s_cfg_clk    : std_logic := '0';
    signal s_cfg_rst_n  : std_logic := '0';
    signal s_axis_clk   : std_logic := '0';
    signal s_axis_rst_n : std_logic := '0';
    signal s_done       : boolean := false;

    signal s_sys_ctrl    : std_logic_vector(31 downto 0) := (others => '0');
    signal s_sys_apply   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bus         : std_logic_vector(31 downto 0) := c_INIT_BUS_TIMING;
    signal s_start       : std_logic_vector(31 downto 0) := c_INIT_START_OFF1;
    signal s_reg7        : std_logic_vector(31 downto 0) := c_INIT_CFG_REG7;
    signal s_image_cmd   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_image_data  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_scan        : std_logic_vector(31 downto 0) := c_INIT_SCAN_TIMEOUT;
    signal s_main        : std_logic_vector(31 downto 0) := c_INIT_MAIN_CTRL;
    signal s_range       : std_logic_vector(31 downto 0) := c_INIT_RANGE_COLS;
    signal s_aux         : std_logic_vector(31 downto 0) := (others => '0');
    signal s_n_faces     : std_logic_vector(2 downto 0) := "100";

    signal s_cfg_ready : std_logic := '1';
    signal s_cmd_ready : std_logic := '1';
    signal s_cfg_out   : t_tdc_cfg;
    signal s_image_out : t_cfg_image;

    signal s_cmd_start        : std_logic;
    signal s_cmd_stop         : std_logic;
    signal s_cmd_soft_reset   : std_logic;
    signal s_cmd_force_reinit : std_logic;
    signal s_err_soft_clear   : std_logic;
    signal s_cmd_cfg_write    : std_logic;
    signal s_cmd_reg_read     : std_logic;
    signal s_cmd_reg_write    : std_logic;
    signal s_cmd_reg_addr     : std_logic_vector(3 downto 0);
    signal s_cmd_reg_chip     : unsigned(1 downto 0);
    signal s_cmd_reg_mask     : std_logic_vector(c_MAX_CHIPS - 1 downto 0);

    signal s_rdata0 : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) :=
        (others => '0');
    signal s_rdata1 : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) :=
        (others => '0');
    signal s_rdata2 : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) :=
        (others => '0');
    signal s_rdata3 : std_logic_vector(c_TDC_BUS_WIDTH - 1 downto 0) :=
        (others => '0');
    signal s_rvalid    : std_logic_vector(c_MAX_CHIPS - 1 downto 0) :=
        (others => '0');
    signal s_reg_done  : std_logic := '0';
    signal s_addr_done : std_logic_vector(3 downto 0) := (others => '0');
    signal s_status    : t_tdc_status := c_TDC_STATUS_INIT;

    signal s_chip0 : std_logic_vector(31 downto 0);
    signal s_chip1 : std_logic_vector(31 downto 0);
    signal s_chip2 : std_logic_vector(31 downto 0);
    signal s_chip3 : std_logic_vector(31 downto 0);
    signal s_pipe_status : std_logic_vector(31 downto 0);
    signal s_status_ext  : std_logic_vector(31 downto 0);
    signal s_status_ext2 : std_logic_vector(31 downto 0);
    signal s_cfg_accepted   : std_logic_vector(7 downto 0);
    signal s_reset_accepted : std_logic_vector(7 downto 0);
    signal s_cfg_busy       : std_logic;
    signal s_cfg_reject     : std_logic;
    signal s_cfg_valid      : std_logic;
    signal s_cmd_accepted   : std_logic_vector(7 downto 0);
    signal s_cmd_busy       : std_logic;
    signal s_cmd_reject     : std_logic;
    signal s_image_accepted : std_logic_vector(7 downto 0);
    signal s_image_reject   : std_logic;
    signal s_image_selected : std_logic_vector(31 downto 0);
    signal s_irq_cause      : std_logic_vector(6 downto 0);

    signal s_cfg_write_count : natural := 0;
    signal s_reset_count     : natural := 0;
    signal s_cmd_count       : t_count_array := (others => 0);
    signal s_irq_count       : t_irq_count_array := (others => 0);

    function fn_image_cmd(index : natural; epoch : natural)
        return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v_word(4 downto 0) := std_logic_vector(to_unsigned(index, 5));
        v_word(15 downto 8) := std_logic_vector(to_unsigned(epoch, 8));
        return v_word;
    end function fn_image_cmd;

    function fn_aux(opcode : natural; epoch : natural)
        return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v_word(2 downto 0) := std_logic_vector(to_unsigned(opcode, 3));
        v_word(15 downto 8) := std_logic_vector(to_unsigned(epoch, 8));
        return v_word;
    end function fn_aux;

    function fn_bus(
        clk_div : natural;
        ticks   : natural;
        address : natural;
        chip    : natural;
        mask    : natural
    ) return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v_word(5 downto 0) := std_logic_vector(to_unsigned(clk_div, 6));
        v_word(8 downto 6) := std_logic_vector(to_unsigned(ticks, 3));
        v_word(13 downto 10) := std_logic_vector(to_unsigned(address, 4));
        v_word(15 downto 14) := std_logic_vector(to_unsigned(chip, 2));
        v_word(19 downto 16) := std_logic_vector(to_unsigned(mask, 4));
        return v_word;
    end function fn_bus;

    function fn_main(mask : natural; stops : natural)
        return std_logic_vector is
        variable v_word : std_logic_vector(31 downto 0) := (others => '0');
    begin
        v_word(3 downto 0) := std_logic_vector(to_unsigned(mask, 4));
        v_word(c_MC_PACKET_SCOPE) := '1';
        v_word(c_MC_HIT_STORE_HI downto c_MC_HIT_STORE_LO) := "10";
        v_word(c_MC_DIST_SCALE_HI downto c_MC_DIST_SCALE_LO) := "011";
        v_word(c_MC_DRAIN_MODE) := '1';
        v_word(c_MC_PIPELINE_EN) := '1';
        v_word(c_MC_STOPS_HI downto c_MC_STOPS_LO) :=
            std_logic_vector(to_unsigned(stops, 4));
        v_word(c_MC_N_DRAIN_CAP_HI downto c_MC_N_DRAIN_CAP_LO) := "0101";
        v_word(c_MC_STOPDIS_HI downto c_MC_STOPDIS_LO) := "10101";
        return v_word;
    end function fn_main;
begin
    p_cfg_clock : process
    begin
        while not s_done loop
            s_cfg_clk <= '0';
            wait for C_CFG_PERIOD / 2;
            s_cfg_clk <= '1';
            wait for C_CFG_PERIOD / 2;
        end loop;
        wait;
    end process p_cfg_clock;

    p_axis_clock : process
    begin
        while not s_done loop
            s_axis_clk <= '0';
            wait for C_AXIS_PERIOD / 2;
            s_axis_clk <= '1';
            wait for C_AXIS_PERIOD / 2;
        end loop;
        wait;
    end process p_axis_clock;

    u_dut : entity work.tdc_gpx_unified_csr_adapter
        generic map (
            g_PRESENT_CHIP_MASK => "0111",
            g_MAX_STOPS_PER_CHIP => 8,
            g_MAX_HITS_PER_STOP => 7,
            g_BUS_READ_PERIOD_MIN_CLKS => 5
        )
        port map (
            i_cfg_clk => s_cfg_clk,
            i_cfg_rst_n => s_cfg_rst_n,
            i_sys_ctrl => s_sys_ctrl,
            i_sys_cfg_apply => s_sys_apply,
            i_tdc_bus_timing => s_bus,
            i_tdc_start_offset => s_start,
            i_tdc_cfg_reg7 => s_reg7,
            i_tdc_image_cmd => s_image_cmd,
            i_tdc_image_data => s_image_data,
            i_tdc_scan_cfg => s_scan,
            i_tdc_pipeline_main => s_main,
            i_tdc_range_cols => s_range,
            i_tdc_aux_cmd => s_aux,
            i_n_faces => s_n_faces,
            i_axis_clk => s_axis_clk,
            i_axis_rst_n => s_axis_rst_n,
            i_cfg_ready => s_cfg_ready,
            i_cmd_ready => s_cmd_ready,
            o_cfg => s_cfg_out,
            o_cfg_image => s_image_out,
            o_cmd_start => s_cmd_start,
            o_cmd_stop => s_cmd_stop,
            o_cmd_soft_reset => s_cmd_soft_reset,
            o_cmd_force_reinit => s_cmd_force_reinit,
            o_err_soft_clear => s_err_soft_clear,
            o_cmd_cfg_write => s_cmd_cfg_write,
            o_cmd_reg_read => s_cmd_reg_read,
            o_cmd_reg_write => s_cmd_reg_write,
            o_cmd_reg_addr => s_cmd_reg_addr,
            o_cmd_reg_chip => s_cmd_reg_chip,
            o_cmd_reg_chip_address => s_cmd_reg_mask,
            i_cmd_reg_rdata_0 => s_rdata0,
            i_cmd_reg_rdata_1 => s_rdata1,
            i_cmd_reg_rdata_2 => s_rdata2,
            i_cmd_reg_rdata_3 => s_rdata3,
            i_cmd_reg_rvalid => s_rvalid,
            i_cmd_reg_done_pulse => s_reg_done,
            i_cmd_reg_addr_done => s_addr_done,
            i_status => s_status,
            o_chip0_result => s_chip0,
            o_chip1_result => s_chip1,
            o_chip2_result => s_chip2,
            o_chip3_result => s_chip3,
            o_pipeline_status => s_pipe_status,
            o_status_ext => s_status_ext,
            o_status_ext2 => s_status_ext2,
            o_cfg_epoch_accepted => s_cfg_accepted,
            o_reset_epoch_accepted => s_reset_accepted,
            o_cfg_busy => s_cfg_busy,
            o_cfg_reject => s_cfg_reject,
            o_cfg_valid => s_cfg_valid,
            o_cmd_epoch_accepted => s_cmd_accepted,
            o_cmd_busy => s_cmd_busy,
            o_command_reject => s_cmd_reject,
            o_image_write_epoch_accepted => s_image_accepted,
            o_image_reject => s_image_reject,
            o_image_selected_data => s_image_selected,
            o_irq_cause => s_irq_cause
        );

    p_axis_monitor : process(s_axis_clk)
    begin
        if rising_edge(s_axis_clk) then
            if s_cmd_cfg_write = '1' then
                s_cfg_write_count <= s_cfg_write_count + 1;
            end if;
            if s_cmd_soft_reset = '1' then
                s_reset_count <= s_reset_count + 1;
            end if;
            if s_cmd_start = '1' then
                s_cmd_count(0) <= s_cmd_count(0) + 1;
            end if;
            if s_cmd_stop = '1' then
                s_cmd_count(1) <= s_cmd_count(1) + 1;
            end if;
            if s_cmd_force_reinit = '1' then
                s_cmd_count(2) <= s_cmd_count(2) + 1;
            end if;
            if s_err_soft_clear = '1' then
                s_cmd_count(3) <= s_cmd_count(3) + 1;
            end if;
            if s_cmd_reg_read = '1' then
                s_cmd_count(4) <= s_cmd_count(4) + 1;
            end if;
            if s_cmd_reg_write = '1' then
                s_cmd_count(5) <= s_cmd_count(5) + 1;
            end if;
        end if;
    end process p_axis_monitor;

    p_irq_monitor : process(s_cfg_clk)
    begin
        if rising_edge(s_cfg_clk) then
            for i in 0 to 6 loop
                if s_irq_cause(i) = '1' then
                    s_irq_count(i) <= s_irq_count(i) + 1;
                end if;
            end loop;
        end if;
    end process p_irq_monitor;

    p_stimulus : process
        procedure wait_image_ack(
            constant epoch : in natural;
            constant rejected : in std_logic;
            constant message_text : in string
        ) is
        begin
            for i in 0 to 1000 loop
                wait until rising_edge(s_cfg_clk);
                exit when unsigned(s_image_accepted) = epoch
                    and s_image_reject = rejected;
            end loop;
            assert unsigned(s_image_accepted) = epoch
                and s_image_reject = rejected
                report message_text severity failure;
        end procedure wait_image_ack;

        procedure issue_command(
            constant opcode : in natural;
            constant epoch  : in natural;
            constant count_index : in natural
        ) is
            variable v_before : natural;
        begin
            v_before := s_cmd_count(count_index);
            wait until falling_edge(s_cfg_clk);
            s_aux <= fn_aux(opcode, epoch);
            for i in 0 to 2000 loop
                wait until rising_edge(s_cfg_clk);
                exit when unsigned(s_cmd_accepted) = epoch;
            end loop;
            assert unsigned(s_cmd_accepted) = epoch
                and s_cmd_count(count_index) = v_before + 1
                report "serialized TDC command was lost or duplicated"
                severity failure;
        end procedure issue_command;
    begin
        wait for 50 ns;
        wait until falling_edge(s_cfg_clk);
        s_cfg_rst_n <= '1';
        wait until falling_edge(s_axis_clk);
        s_axis_rst_n <= '1';

        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cfg_valid = '1' and s_cfg_out.n_faces = 4;
        end loop;
        assert s_cfg_valid = '1' and s_cfg_out.n_faces = 4
            and s_cfg_out.active_chip_mask = "0111"
            and s_cfg_write_count = 0 and s_reset_count = 0
            report "TDC adapter reset defaults or static geometry are incorrect"
            severity failure;

        -- Stage all 16 GPX image words through one indexed window.
        for index in 0 to 15 loop
            wait until falling_edge(s_cfg_clk);
            s_image_data <= std_logic_vector(to_unsigned(16#1000# + index, 32));
            s_image_cmd <= fn_image_cmd(index, index + 1);
            wait_image_ack(index + 1, '0', "valid GPX image word was not accepted");
            assert s_image_selected =
                std_logic_vector(to_unsigned(16#1000# + index, 32))
                report "selected GPX staging readback is incorrect"
                severity failure;
        end loop;
        for index in 0 to 15 loop
            assert s_image_out(index) = x"00000000"
                report "staged GPX image leaked before CFG_EPOCH"
                severity failure;
        end loop;

        -- Invalid index is consumed and diagnosed; a following valid write
        -- clears rejection and updates the final word.
        wait until falling_edge(s_cfg_clk);
        s_image_data <= x"BAD00010";
        s_image_cmd <= fn_image_cmd(16, 17);
        wait_image_ack(17, '1', "invalid GPX image index was not rejected");
        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_irq_count(5) = 1;
        end loop;
        assert s_irq_count(5) = 1
            report "image rejection did not raise unified TDC IRQ[26]"
            severity failure;

        wait until falling_edge(s_cfg_clk);
        s_image_data <= x"DEAD000F";
        s_image_cmd <= fn_image_cmd(15, 18);
        wait_image_ack(18, '0', "valid image retry did not clear rejection");

        -- Configure deliberately out-of-range values to prove compatibility
        -- clamps: absent chip mask -> first present, 9 stops -> build max,
        -- div1/ticks4 -> ticks5, zero columns -> one.
        wait until falling_edge(s_cfg_clk);
        s_bus   <= fn_bus(1, 4, 0, 0, 0);
        s_start <= x"00012345";
        s_reg7  <= x"89ABCDEF";
        s_scan  <= x"00080064";
        s_main  <= fn_main(8, 9);
        s_range <= x"000001F4";
        s_cfg_ready <= '0';
        s_sys_apply(7 downto 0) <= x"01";
        wait for 2 us;
        assert s_cfg_busy = '1' and s_cfg_write_count = 0
            and s_cfg_out.cfg_reg7 = c_INIT_CFG_REG7
            report "TDC config did not defer while destination was busy"
            severity failure;

        wait until falling_edge(s_axis_clk);
        s_cfg_ready <= '1';
        for i in 0 to 2000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cfg_accepted = x"01" and s_cfg_busy = '0';
        end loop;
        assert s_cfg_accepted = x"01" and s_cfg_busy = '0'
            and s_cfg_write_count = 1
            report "atomic TDC configuration was not applied exactly once"
            severity failure;
        assert s_cfg_out.active_chip_mask = "0001"
            and s_cfg_out.bus_clk_div = 1
            and s_cfg_out.bus_ticks = 5
            and s_cfg_out.stops_per_chip = 8
            and s_cfg_out.cols_per_face = 1
            and s_cfg_out.max_range_5ns_ticks = 500
            and s_cfg_out.start_off1 = to_unsigned(16#12345#, 18)
            and s_cfg_out.cfg_reg7 = x"89ABCDEF"
            and s_cfg_out.max_scan_5ns_ticks = 100
            and s_cfg_out.max_hits_cfg = 7
            and s_cfg_out.falling_enable = '1'
            report "unified TDC configuration packing/clamping changed"
            severity failure;
        for index in 0 to 14 loop
            assert s_image_out(index) =
                std_logic_vector(to_unsigned(16#1000# + index, 32))
                report "committed GPX image word mismatch"
                severity failure;
        end loop;
        assert s_image_out(15) = x"DEAD000F"
            report "final committed GPX image word mismatch"
            severity failure;

        -- RESET_EPOCH is the sole soft-reset owner and preserves active config.
        wait until falling_edge(s_cfg_clk);
        s_sys_ctrl(15 downto 8) <= x"01";
        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_reset_accepted = x"01" and s_reset_count = 1;
        end loop;
        assert s_reset_accepted = x"01" and s_reset_count = 1
            and s_cfg_out.cfg_reg7 = x"89ABCDEF"
            report "TDC RESET_EPOCH lifecycle changed active configuration"
            severity failure;

        -- A serialized command waits while command arbitration is busy.
        s_cmd_ready <= '0';
        wait until falling_edge(s_cfg_clk);
        s_aux <= fn_aux(1, 1);
        wait for 2 us;
        assert s_cmd_busy = '1' and s_cmd_count(0) = 0
            report "pending TDC START did not wait for command readiness"
            severity failure;
        wait until falling_edge(s_axis_clk);
        s_cmd_ready <= '1';
        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cmd_accepted = x"01";
        end loop;
        assert s_cmd_count(0) = 1 and s_cmd_busy = '0'
            report "pending TDC START was lost or duplicated"
            severity failure;

        issue_command(2, 2, 1);
        issue_command(3, 3, 2);
        issue_command(4, 4, 3);

        -- Register commands sample target fields from the same CDC snapshot.
        wait until falling_edge(s_cfg_clk);
        s_bus <= fn_bus(1, 4, 10, 2, 0);
        s_aux <= fn_aux(5, 5);
        for i in 0 to 2000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cmd_accepted = x"05";
        end loop;
        assert s_cmd_count(4) = 1 and s_cmd_reg_addr = x"A"
            and s_cmd_reg_chip = 2 and s_cmd_reg_mask = "0100"
            report "serialized GPX register-read target was torn"
            severity failure;

        wait until falling_edge(s_cfg_clk);
        s_bus <= fn_bus(1, 4, 11, 1, 3);
        s_aux <= fn_aux(6, 6);
        for i in 0 to 2000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cmd_accepted = x"06";
        end loop;
        assert s_cmd_count(5) = 1 and s_cmd_reg_addr = x"B"
            and s_cmd_reg_chip = 1 and s_cmd_reg_mask = "0011"
            report "serialized GPX register-write target was torn"
            severity failure;

        -- Opcode 7 is reserved and must never emit a downstream pulse.
        wait until falling_edge(s_cfg_clk);
        s_aux <= fn_aux(7, 7);
        for i in 0 to 2000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_cmd_reject = '1' and s_cmd_busy = '0';
        end loop;
        assert s_cmd_reject = '1' and s_cmd_accepted = x"06"
            report "reserved TDC opcode was not rejected"
            severity failure;
        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_irq_count(6) = 1;
        end loop;
        assert s_irq_count(6) = 1
            report "command rejection did not raise unified TDC IRQ[27]"
            severity failure;

        -- Preserve four independent address+28-bit register results.
        wait until falling_edge(s_axis_clk);
        s_addr_done <= x"A";
        s_rdata0 <= x"0000001";
        s_rdata1 <= x"0000002";
        s_rdata2 <= x"0000003";
        s_rdata3 <= x"0000004";
        s_rvalid <= "1111";
        s_reg_done <= '1';
        wait until falling_edge(s_axis_clk);
        s_rvalid <= (others => '0');
        s_reg_done <= '0';
        for i in 0 to 2000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_chip3 = x"A0000004";
        end loop;
        assert s_chip0 = x"A0000001" and s_chip1 = x"A0000002"
            and s_chip2 = x"A0000003" and s_chip3 = x"A0000004"
            report "per-chip GPX register results were merged or torn"
            severity failure;
        for i in 0 to 1000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_irq_count(0) = 1;
        end loop;
        assert s_irq_count(0) = 1
            report "register done did not raise unified TDC IRQ[21]"
            severity failure;

        -- Exercise the five diagnostic IRQ groups independently.
        wait until falling_edge(s_axis_clk);
        s_status.pipeline_overrun <= '1';
        wait until falling_edge(s_axis_clk);
        s_status.chip_error_mask <= "0011";
        wait until falling_edge(s_axis_clk);
        s_status.drain_timeout_mask <= "0101";
        wait until falling_edge(s_axis_clk);
        s_status.sequence_error_mask <= "1001";
        for i in 0 to 2000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_irq_count(1) = 1 and s_irq_count(2) = 1
                and s_irq_count(3) = 1 and s_irq_count(4) = 1;
        end loop;
        assert s_irq_count(1) = 1 and s_irq_count(2) = 1
            and s_irq_count(3) = 1 and s_irq_count(4) = 1
            report "unified TDC diagnostic IRQ identity changed"
            severity failure;

        -- Exact legacy STAT6/7 packing remains intact.
        wait until falling_edge(s_axis_clk);
        s_status.busy <= '1';
        s_status.err_fatal <= '1';
        s_status.err_read_timeout <= '1';
        s_status.reg_rejected <= '1';
        s_status.reg_zero_mask <= '1';
        s_status.rise_shot_flush_drop <= '1';
        s_status.fall_shot_flush_drop <= '1';
        s_status.rise_hdr_drain_timeout <= '1';
        s_status.fall_hdr_drain_timeout <= '1';
        s_status.err_frame_wait_escape <= '1';
        s_status.rise_shot_overrun_count <= x"0A";
        s_status.fall_shot_overrun_count <= x"0B";
        s_status.shot_flush_drop_mask <= "1100";
        s_status.cmd_collision_mask <= "0011";
        s_status.err_reg_overflow_mask <= "0101";
        s_status.run_drain_complete_mask <= "1010";
        s_status.reg_timeout_mask <= "0001";
        s_status.stop_id_error_mask <= "0010";
        s_status.run_timeout_cause_last <= "101";
        s_status.quarantine_escape_mask <= "0100";
        s_status.masked_slope_drop_any <= '1';
        s_status.rise_face_start_collapsed_count <= x"0C";
        s_status.fall_face_start_collapsed_count <= x"0D";
        s_status.init_cfg_coalesced_mask <= "1110";

        for i in 0 to 2000 loop
            wait until rising_edge(s_cfg_clk);
            exit when s_status_ext(31 downto 28) = "1010"
                and s_status_ext2(31 downto 28) = "1110";
        end loop;
        assert s_pipe_status(15 downto 0) = x"9537"
            and s_pipe_status(23 downto 16) = x"06"
            and s_pipe_status(31 downto 24) = x"12"
            report "TDC pipeline status/ack packing changed"
            severity failure;
        assert s_status_ext(7 downto 0) = x"FF"
            and s_status_ext(11 downto 8) = x"A"
            and s_status_ext(15 downto 12) = x"C"
            and s_status_ext(19 downto 16) = x"B"
            and s_status_ext(23 downto 20) = x"3"
            and s_status_ext(27 downto 24) = x"5"
            and s_status_ext(31 downto 28) = x"A"
            report "legacy TDC STATUS_EXT packing changed"
            severity failure;
        assert s_status_ext2(3 downto 0) = x"1"
            and s_status_ext2(7 downto 4) = x"2"
            and s_status_ext2(10 downto 8) = "101"
            and s_status_ext2(14 downto 11) = "0100"
            and s_status_ext2(15) = '1'
            and s_status_ext2(19 downto 16) = x"C"
            and s_status_ext2(23 downto 20) = x"0"
            and s_status_ext2(27 downto 24) = x"D"
            and s_status_ext2(31 downto 28) = x"E"
            report "legacy TDC STATUS_EXT2 packing changed"
            severity failure;

        report "TDC_GPX_UNIFIED_CSR_ADAPTER_PASS" severity note;
        s_done <= true;
        stop;
        wait;
    end process p_stimulus;
end architecture sim;
