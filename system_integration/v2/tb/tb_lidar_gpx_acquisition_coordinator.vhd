library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_config_reference_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

entity tb_lidar_gpx_acquisition_coordinator is
    generic (
        G_TDC_CLK_MHZ : positive := 200
    );
end entity tb_lidar_gpx_acquisition_coordinator;

architecture sim of tb_lidar_gpx_acquisition_coordinator is

    constant C_CHIPS : positive := work.lidar_build_pkg.C_MAX_CHIPS;
    constant C_CLK_PERIOD : time := 1 us / G_TDC_CLK_MHZ;
    constant C_ACTIVE_MASK : chip_mask_t := "1011";
    constant C_IFIFO1_WORDS : natural := 2;
    constant C_IFIFO2_WORDS : natural := 1;
    constant C_EVENTS_PER_CHIP : natural :=
        C_IFIFO1_WORDS + C_IFIFO2_WORDS + 2;
    constant C_TOTAL_EVENTS : natural := 3 * C_EVENTS_PER_CHIP;

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz           => 150,
        tdc_clk_mhz            => G_TDC_CLK_MHZ,
        stream_clock_mode      => STREAM_CLOCK_ASYNC,
        num_chips              => 4,
        stops_per_chip         => 4,
        max_returns_per_stop   => 7,
        rise_capability_mask   => "1111",
        fall_capability_mask   => "0000",
        output_width           => 32,
        num_faces              => 4,
        enable_echo_receiver   => false,
        enable_echo_simulation => false
    );

    type natural_array_t is array (0 to C_CHIPS - 1) of
        natural range 0 to 65535;
    type logic_array_t is array (0 to C_CHIPS - 1) of std_logic;
    type register_capture_t is array (
        0 to C_CHIPS - 1, 0 to 15) of gpx_bus_data_t;

    function fn_active_config return lidar_active_config_t is
        variable runtime : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        runtime.tdc.active_chip_mask := C_ACTIVE_MASK;
        runtime.tdc.start_offset := to_unsigned(321, 18);
        result.version := to_unsigned(12, 16);
        result.source := runtime;
        result.derived := fn_derive_runtime_config(C_BUILD_CONFIG, runtime);
        return result;
    end function fn_active_config;

    function fn_register_image return gpx_register_image_t is
        variable result : gpx_register_image_t := (others => (others => '0'));
    begin
        for index in result'range loop
            result(index) := c_GPX_DEFAULT_IMAGE(index);
        end loop;
        return result;
    end function fn_register_image;

    function fn_shot return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(3, 3);
        result.request.position := to_unsigned(4040, C_POSITION_WIDTH);
        result.request.direction := DIRECTION_CW;
        result.request.shot_index := to_unsigned(51, 16);
        result.request.last_in_face := '0';
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(7, 8);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(12, 16);
        result.fire_to_t0_clks := to_unsigned(4, 32);
        return result;
    end function fn_shot;

    function fn_raw_word(
        chip_index : natural;
        ififo_id   : std_logic;
        word_index : natural
    ) return gpx_bus_data_t is
        variable result : gpx_bus_data_t := (others => '0');
        variable hit_value : natural;
    begin
        result(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(word_index mod 4, 2));
        result(c_RAW_SLOPE_BIT) := ififo_id;
        hit_value := chip_index * 16#1000# + word_index;
        if ififo_id = '1' then
            hit_value := hit_value + 16#100#;
        end if;
        result(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, c_RAW_HIT_WIDTH));
        return result;
    end function fn_raw_word;

    function fn_all_initialized(
        value : gpx_lane_status_array_t
    ) return boolean is
    begin
        for index in 0 to C_CHIPS - 1 loop
            if value(index).initialized /= '1' then
                return false;
            end if;
        end loop;
        return true;
    end function fn_all_initialized;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal simulation_done : boolean := false;

    signal active_config : lidar_active_config_t := fn_active_config;
    signal register_image : gpx_register_image_t := fn_register_image;
    signal applied_register_image : gpx_register_image_t;
    signal config_activate_start : std_logic := '0';
    signal config_activate_complete : std_logic;
    signal config_activate_fault : std_logic;
    signal config_apply : std_logic;
    signal config_ready : std_logic;
    signal config_done : std_logic;
    signal run_enable : std_logic := '0';
    signal safe : std_logic;
    signal shot : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_ready : std_logic;
    signal stop_tdc : std_logic := '0';
    signal shot_complete : std_logic;
    signal event_out : gpx_raw_event_t;
    signal event_ready : std_logic := '1';

    signal adr : gpx_bus_address_array_t;
    signal csn : chip_mask_t;
    signal rdn : chip_mask_t;
    signal wrn : chip_mask_t;
    signal oen : chip_mask_t;
    signal d_from_coordinator : gpx_bus_data_array_t;
    signal d_tri : chip_mask_t;
    signal d_bus : gpx_bus_data_array_t := (others => (others => 'Z'));
    signal chip_d_out : gpx_bus_data_array_t := (others => (others => '0'));
    signal chip_d_oe : chip_mask_t := (others => '0');
    signal ef1 : chip_mask_t;
    signal ef2 : chip_mask_t;
    signal lf1 : chip_mask_t;
    signal lf2 : chip_mask_t;
    signal irflag : chip_mask_t := (others => '0');
    signal errflag : chip_mask_t := (others => '0');
    signal stopdis : chip_mask_t;
    signal alutrigger : chip_mask_t;
    signal puresn : chip_mask_t;
    signal active_mask : chip_mask_t;
    signal terminal_mask : chip_mask_t;
    signal status : gpx_lane_status_array_t;
    signal faults : gpx_lane_faults_array_t;

    signal fifo1_fill : natural_array_t := (others => 0);
    signal fifo2_fill : natural_array_t := (others => 0);
    signal fifo1_read_index : natural_array_t := (others => 0);
    signal fifo2_read_index : natural_array_t := (others => 0);
    signal fifo_load : std_logic := '0';
    signal write_capture : register_capture_t :=
        (others => (others => (others => '0')));
    signal write_count : natural_array_t := (others => 0);
    signal read_count : natural_array_t := (others => 0);
    signal empty_read_count : natural_array_t := (others => 0);
    signal event_count : natural_array_t := (others => 0);
    signal total_event_count : natural := 0;
    signal completion_count : natural := 0;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not simulation_done;

    gen_pin_models : for index in 0 to C_CHIPS - 1 generate
        ef1(index) <= '1' when fifo1_fill(index) = 0 else '0';
        ef2(index) <= '1' when fifo2_fill(index) = 0 else '0';
        lf1(index) <= '1' when fifo1_fill(index) >= 2 else '0';
        lf2(index) <= '1' when fifo2_fill(index) >= 2 else '0';

        d_bus(index) <= d_from_coordinator(index)
            when d_tri(index) = '0' else (others => 'Z');
        d_bus(index) <= chip_d_out(index)
            when chip_d_oe(index) = '1' else (others => 'Z');
    end generate gen_pin_models;

    u_config_activation : entity work.lidar_gpx_config_activation
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_activate_start    => config_activate_start,
            i_candidate_image   => register_image,
            i_apply_ready       => config_ready,
            i_apply_done        => config_done,
            i_apply_fault       => '0',
            o_register_image    => applied_register_image,
            o_apply             => config_apply,
            o_activate_complete => config_activate_complete,
            o_activate_fault    => config_activate_fault,
            o_busy              => open
        );

    u_dut : entity work.lidar_gpx_acquisition_coordinator
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_OEN_MODE => "DYNAMIC_CONNECTED"
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_active_valid => '1',
            i_active_config => active_config,
            i_register_image => applied_register_image,
            i_config_apply => config_apply,
            o_config_ready => config_ready,
            o_config_done => config_done,
            i_run_enable => run_enable,
            i_soft_reset => '0',
            i_force_reinit => '0',
            i_clear_status => '0',
            o_safe => safe,
            i_shot => shot,
            o_shot_ready => shot_ready,
            i_stop_tdc => stop_tdc,
            o_shot_complete => shot_complete,
            o_event => event_out,
            i_event_ready => event_ready,
            o_adr => adr,
            o_csn => csn,
            o_rdn => rdn,
            o_wrn => wrn,
            o_oen => oen,
            i_d => d_bus,
            o_d => d_from_coordinator,
            o_d_tri => d_tri,
            i_ef1 => ef1,
            i_ef2 => ef2,
            i_lf1 => lf1,
            i_lf2 => lf2,
            i_irflag => irflag,
            i_errflag => errflag,
            o_stopdis => stopdis,
            o_alutrigger => alutrigger,
            o_puresn => puresn,
            o_active_mask => active_mask,
            o_terminal_mask => terminal_mask,
            o_status => status,
            o_faults => faults
        );

    p_chip_models : process (clk)
        variable rdn_previous : logic_array_t := (others => '1');
        variable wrn_previous : logic_array_t := (others => '1');
        variable write_data_hold : gpx_bus_data_array_t :=
            (others => (others => '0'));
        variable write_address_hold : gpx_bus_address_array_t :=
            (others => (others => '0'));
        variable address_value : natural;
    begin
        if rising_edge(clk) then
            chip_d_oe <= (others => '0');

            if rst_n = '0' then
                fifo1_fill <= (others => 0);
                fifo2_fill <= (others => 0);
                fifo1_read_index <= (others => 0);
                fifo2_read_index <= (others => 0);
                write_capture <= (others => (others => (others => '0')));
                write_count <= (others => 0);
                read_count <= (others => 0);
                empty_read_count <= (others => 0);
                rdn_previous := (others => '1');
                wrn_previous := (others => '1');
            else
                for index in 0 to C_CHIPS - 1 loop
                    if fifo_load = '1' then
                        fifo1_fill(index) <= C_IFIFO1_WORDS;
                        fifo2_fill(index) <= C_IFIFO2_WORDS;
                        fifo1_read_index(index) <= 0;
                        fifo2_read_index(index) <= 0;
                    end if;

                    if oen(index) = '0' and rdn(index) = '0' then
                        chip_d_oe(index) <= '1';
                        if adr(index) = c_TDC_REG8_IFIFO1 then
                            chip_d_out(index) <= fn_raw_word(
                                index, '0', fifo1_read_index(index));
                        elsif adr(index) = c_TDC_REG9_IFIFO2 then
                            chip_d_out(index) <= fn_raw_word(
                                index, '1', fifo2_read_index(index));
                        else
                            chip_d_out(index) <= (others => '0');
                        end if;
                    end if;

                    if rdn(index) = '0' and rdn_previous(index) = '1' then
                        read_count(index) <= read_count(index) + 1;
                        if adr(index) = c_TDC_REG8_IFIFO1 and
                           fifo1_fill(index) = 0 then
                            empty_read_count(index) <=
                                empty_read_count(index) + 1;
                        elsif adr(index) = c_TDC_REG9_IFIFO2 and
                              fifo2_fill(index) = 0 then
                            empty_read_count(index) <=
                                empty_read_count(index) + 1;
                        end if;
                    end if;

                    if rdn(index) = '1' and rdn_previous(index) = '0' then
                        if adr(index) = c_TDC_REG8_IFIFO1 and
                           fifo1_fill(index) > 0 then
                            fifo1_fill(index) <= fifo1_fill(index) - 1;
                            fifo1_read_index(index) <=
                                fifo1_read_index(index) + 1;
                        elsif adr(index) = c_TDC_REG9_IFIFO2 and
                              fifo2_fill(index) > 0 then
                            fifo2_fill(index) <= fifo2_fill(index) - 1;
                            fifo2_read_index(index) <=
                                fifo2_read_index(index) + 1;
                        end if;
                    end if;
                    rdn_previous(index) := rdn(index);

                    if wrn(index) = '0' then
                        write_data_hold(index) := d_bus(index);
                        write_address_hold(index) := adr(index);
                    end if;
                    if wrn(index) = '1' and wrn_previous(index) = '0' then
                        address_value := to_integer(unsigned(
                            write_address_hold(index)));
                        write_capture(index, address_value) <=
                            write_data_hold(index);
                        write_count(index) <= write_count(index) + 1;
                    end if;
                    wrn_previous(index) := wrn(index);
                end loop;
            end if;
        end if;
    end process p_chip_models;

    p_scoreboard : process (clk)
        variable chip : natural range 0 to C_CHIPS - 1;
        variable local_index : natural;
        variable expected_word : gpx_bus_data_t;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                event_count <= (others => 0);
                total_event_count <= 0;
                completion_count <= 0;
            else
                if event_out.valid = '1' and event_ready = '1' then
                    chip := to_integer(event_out.chip_index);
                    assert C_ACTIVE_MASK(chip) = '1'
                        report "V2-GPX-COORD-TB event from inactive Chip"
                        severity failure;
                    assert event_out.shot_context = fn_shot
                        report "V2-GPX-COORD-TB Shot context mismatch"
                        severity failure;
                    assert event_out.chip_shot_seq = 0
                        report "V2-GPX-COORD-TB Chip sequence mismatch"
                        severity failure;

                    local_index := event_count(chip);
                    case local_index is
                        when 0 to C_IFIFO1_WORDS - 1 =>
                            expected_word := fn_raw_word(
                                chip, '0', local_index);
                            assert event_out.kind = GPX_RAW_DATA and
                                   event_out.ififo_id = '0' and
                                   event_out.raw_word = expected_word
                                report "V2-GPX-COORD-TB IFIFO1 mismatch"
                                severity failure;
                        when C_IFIFO1_WORDS =>
                            assert event_out.kind = GPX_RAW_IFIFO1_DONE and
                                   event_out.ififo_id = '0'
                                report "V2-GPX-COORD-TB IFIFO1_DONE mismatch"
                                severity failure;
                        when C_IFIFO1_WORDS + 1 =>
                            expected_word := fn_raw_word(chip, '1', 0);
                            assert event_out.kind = GPX_RAW_DATA and
                                   event_out.ififo_id = '1' and
                                   event_out.raw_word = expected_word
                                report "V2-GPX-COORD-TB IFIFO2 mismatch"
                                severity failure;
                        when C_EVENTS_PER_CHIP - 1 =>
                            assert event_out.kind = GPX_RAW_DRAIN_DONE and
                                   event_out.faulted = '0'
                                report "V2-GPX-COORD-TB terminal mismatch"
                                severity failure;
                        when others =>
                            assert false
                                report "V2-GPX-COORD-TB extra per-Chip event"
                                severity failure;
                    end case;
                    event_count(chip) <= event_count(chip) + 1;
                    total_event_count <= total_event_count + 1;
                end if;

                if shot_complete = '1' then
                    completion_count <= completion_count + 1;
                end if;
            end if;
        end if;
    end process p_scoreboard;

    p_stimulus : process
        procedure wait_clocks(count : positive) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
            wait for 1 ps;
        end procedure wait_clocks;

        procedure wait_until_high(
            signal value : in std_logic;
            constant message_text : in string;
            constant timeout_clocks : in positive := 40000
        ) is
        begin
            for timeout in 0 to timeout_clocks loop
                wait until rising_edge(clk);
                wait for 1 ps;
                if value = '1' then
                    return;
                end if;
            end loop;
            assert false report message_text severity failure;
        end procedure wait_until_high;

        variable held_event : gpx_raw_event_t;
        variable updated_config : lidar_active_config_t;
    begin
        rst_n <= '0';
        wait_clocks(8);
        rst_n <= '1';

        for timeout in 0 to 40000 loop
            wait_clocks(1);
            exit when fn_all_initialized(status);
        end loop;
        assert fn_all_initialized(status)
            report "V2-GPX-COORD-TB initialization timeout"
            severity failure;
        assert active_mask = C_ACTIVE_MASK
            report "V2-GPX-COORD-TB active mask mismatch"
            severity failure;

        for index in 0 to C_CHIPS - 1 loop
            assert write_count(index) >= 12
                report "V2-GPX-COORD-TB incomplete initialization"
                severity failure;
            if C_ACTIVE_MASK(index) = '1' then
                assert write_capture(index, 0)(c_REG0_TSTART_RISE) = '1' and
                       write_capture(index, 0)(c_REG0_TSTOP_RISE_LO + 3
                           downto c_REG0_TSTOP_RISE_LO) = "1111"
                    report "V2-GPX-COORD-TB active Reg0 mismatch"
                    severity failure;
            else
                assert write_capture(index, 0)(c_REG0_TRISEEN_HI
                           downto c_REG0_TRISEEN_LO) = (8 downto 0 => '0') and
                       write_capture(index, 0)(c_REG0_TFALLEN_HI
                           downto c_REG0_TFALLEN_LO) = (8 downto 0 => '0')
                    report "V2-GPX-COORD-TB inactive Reg0 not disabled"
                    severity failure;
            end if;
        end loop;

        run_enable <= '1';
        wait_until_high(shot_ready,
            "V2-GPX-COORD-TB all-active-lane arm timeout");

        fifo_load <= '1';
        wait_clocks(1);
        fifo_load <= '0';
        wait_clocks(4);

        shot <= fn_shot;
        wait until rising_edge(clk) and shot_ready = '1';
        wait for 1 ps;
        shot <= C_SHOT_START_EVENT_IDLE;
        wait_clocks(1);
        assert status(0).shot_outstanding = '1' and
               status(1).shot_outstanding = '1' and
               status(3).shot_outstanding = '1' and
               status(2).shot_outstanding = '0'
            report "V2-GPX-COORD-TB Shot was not atomically masked"
            severity failure;

        event_ready <= '0';
        wait_clocks(5);
        irflag <= C_ACTIVE_MASK;
        wait_clocks(4);
        irflag <= (others => '0');

        wait_until_high(event_out.valid,
            "V2-GPX-COORD-TB first merged event timeout");
        held_event := event_out;
        wait_clocks(5);
        assert event_out = held_event
            report "V2-GPX-COORD-TB merged event changed under backpressure"
            severity failure;
        event_ready <= '1';

        for timeout in 0 to 40000 loop
            wait_clocks(1);
            exit when completion_count = 1;
        end loop;
        assert total_event_count = C_TOTAL_EVENTS and
               completion_count = 1 and terminal_mask = C_ACTIVE_MASK
            report "V2-GPX-COORD-TB Shot completion aggregation mismatch"
            severity failure;
        wait_until_high(shot_ready,
            "V2-GPX-COORD-TB coordinator did not re-arm");

        for index in 0 to C_CHIPS - 1 loop
            assert empty_read_count(index) = 0
                report "V2-GPX-COORD-TB empty IFIFO read"
                severity failure;
            assert faults(index) = C_GPX_LANE_FAULTS_CLEAR
                report "V2-GPX-COORD-TB unexpected lane fault"
                severity failure;
        end loop;
        assert read_count(2) = 0 and status(2).chip_shot_seq = 0 and
               status(2).run_active = '0'
            report "V2-GPX-COORD-TB inactive Chip performed acquisition"
            severity failure;

        run_enable <= '0';
        wait_until_high(safe, "V2-GPX-COORD-TB stop-to-safe timeout");
        wait_until_high(config_ready,
            "V2-GPX-COORD-TB config-ready timeout");

        updated_config := active_config;
        updated_config.version := to_unsigned(13, 16);
        updated_config.source.tdc.start_offset := to_unsigned(88, 18);
        updated_config.derived := fn_derive_runtime_config(
            C_BUILD_CONFIG, updated_config.source);
        active_config <= updated_config;
        wait_clocks(2);
        config_activate_start <= '1';
        wait_clocks(1);
        config_activate_start <= '0';
        wait_until_high(config_activate_complete,
            "V2-GPX-COORD-TB all-Chip activation completion timeout");
        assert config_activate_fault = '0'
            report "V2-GPX-COORD-TB unexpected activation fault"
            severity failure;
        for index in 0 to C_CHIPS - 1 loop
            assert write_capture(index, 5)(c_REG5_STARTOFF1_HI downto
                       c_REG5_STARTOFF1_LO) =
                   std_logic_vector(to_unsigned(88, 18))
                report "V2-GPX-COORD-TB config did not reach every Chip"
                severity failure;
        end loop;

        report "LIDAR_V2_GPX_ACQUISITION_COORDINATOR_PASS tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
            severity note;
        simulation_done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_acquisition_coordinator_150 is
end entity tb_lidar_gpx_acquisition_coordinator_150;

architecture sim of tb_lidar_gpx_acquisition_coordinator_150 is
begin
    u_test : entity work.tb_lidar_gpx_acquisition_coordinator
        generic map (G_TDC_CLK_MHZ => 150);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_acquisition_coordinator_200 is
end entity tb_lidar_gpx_acquisition_coordinator_200;

architecture sim of tb_lidar_gpx_acquisition_coordinator_200 is
begin
    u_test : entity work.tb_lidar_gpx_acquisition_coordinator
        generic map (G_TDC_CLK_MHZ => 200);
end architecture sim;
