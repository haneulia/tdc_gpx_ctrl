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

entity tb_lidar_gpx_acquisition_lane is
    generic (
        G_TDC_CLK_MHZ : positive := 200
    );
end entity tb_lidar_gpx_acquisition_lane;

architecture sim of tb_lidar_gpx_acquisition_lane is

    constant C_CLK_PERIOD : time := 1 us / G_TDC_CLK_MHZ;
    constant C_IFIFO1_WORDS : natural := 4;
    constant C_IFIFO2_WORDS : natural := 3;

    constant C_BUILD_CONFIG : lidar_build_config_t := (
        proc_clk_mhz          => 150,
        tdc_clk_mhz           => G_TDC_CLK_MHZ,
        stream_clock_mode     => STREAM_CLOCK_ASYNC,
        num_chips             => 1,
        stops_per_chip        => 4,
        max_returns_per_stop  => 7,
        rise_capability_mask  => "0001",
        fall_capability_mask  => "0000",
        output_width          => 32,
        num_faces             => 4,
        enable_echo_receiver  => false,
        enable_echo_simulation => false
    );

    function fn_active_config return lidar_active_config_t is
        variable runtime : lidar_runtime_config_t :=
            fn_default_runtime_config(C_BUILD_CONFIG);
        variable result : lidar_active_config_t;
    begin
        runtime.tdc.start_offset := to_unsigned(1234, 18);
        result.version := to_unsigned(7, 16);
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
        result.request.face_index := to_unsigned(2, 3);
        result.request.position := to_unsigned(2222, C_POSITION_WIDTH);
        result.request.direction := DIRECTION_CCW;
        result.request.shot_index := to_unsigned(37, 16);
        result.request.last_in_face := '0';
        result.request.source_sim := '0';
        result.request.source_latency_clks := to_unsigned(8, 8);
        result.request.source_latency_valid := '1';
        result.request.active_version := to_unsigned(7, 16);
        result.fire_to_t0_clks := to_unsigned(3, 32);
        return result;
    end function fn_shot;

    function fn_raw_word(
        ififo_id : std_logic;
        index    : natural
    ) return gpx_bus_data_t is
        variable result : gpx_bus_data_t := (others => '0');
        variable hit_value : natural;
    begin
        result(c_RAW_CHACODE_HI downto c_RAW_CHACODE_LO) :=
            std_logic_vector(to_unsigned(index mod 4, 2));
        result(c_RAW_SLOPE_BIT) := ififo_id;
        if ififo_id = '0' then
            hit_value := 16#100# + index;
        else
            hit_value := 16#200# + index;
        end if;
        result(c_RAW_HIT_HI downto c_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, c_RAW_HIT_WIDTH));
        return result;
    end function fn_raw_word;

    type register_capture_t is array (0 to 15) of gpx_bus_data_t;

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal simulation_done : boolean := false;

    signal active_valid : std_logic := '1';
    signal active_config : lidar_active_config_t := fn_active_config;
    signal register_image : gpx_register_image_t := fn_register_image;
    signal config_apply : std_logic := '0';
    signal config_ready : std_logic;
    signal config_done : std_logic;
    signal run_enable : std_logic := '0';
    signal safe : std_logic;

    signal shot : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    signal shot_ready : std_logic;
    signal stop_tdc : std_logic := '0';
    signal event_out : gpx_raw_event_t;
    signal event_ready : std_logic := '1';

    signal adr : gpx_bus_address_t;
    signal csn : std_logic;
    signal rdn : std_logic;
    signal wrn : std_logic;
    signal oen : std_logic;
    signal d_from_lane : gpx_bus_data_t;
    signal d_tri : std_logic;
    signal d_bus : gpx_bus_data_t := (others => 'Z');
    signal chip_d_out : gpx_bus_data_t := (others => '0');
    signal chip_d_oe : std_logic := '0';

    signal fifo1_fill : natural range 0 to 64 := 0;
    signal fifo2_fill : natural range 0 to 64 := 0;
    signal fifo1_read_index : natural range 0 to 64 := 0;
    signal fifo2_read_index : natural range 0 to 64 := 0;
    signal fifo_load : std_logic := '0';
    signal fifo_load_1 : natural range 0 to 64 := 0;
    signal fifo_load_2 : natural range 0 to 64 := 0;

    signal ef1 : std_logic;
    signal ef2 : std_logic;
    signal lf1 : std_logic;
    signal lf2 : std_logic;
    signal irflag : std_logic := '0';
    signal errflag : std_logic := '0';
    signal stopdis : std_logic;
    signal alutrigger : std_logic;
    signal puresn : std_logic;
    signal status : gpx_lane_status_t;
    signal faults : gpx_lane_faults_t;

    signal write_capture : register_capture_t := (others => (others => '0'));
    signal write_count : natural := 0;
    signal empty_read_count : natural := 0;
    signal event_count : natural := 0;

begin

    clk <= not clk after C_CLK_PERIOD / 2 when not simulation_done;

    ef1 <= '1' when fifo1_fill = 0 else '0';
    ef2 <= '1' when fifo2_fill = 0 else '0';
    lf1 <= '1' when fifo1_fill >= 2 else '0';
    lf2 <= '1' when fifo2_fill >= 2 else '0';

    d_bus <= d_from_lane when d_tri = '0' else (others => 'Z');
    d_bus <= chip_d_out when chip_d_oe = '1' else (others => 'Z');

    u_dut : entity work.lidar_gpx_acquisition_lane
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_CHIP_INDEX => 0,
            G_OEN_MODE => "DYNAMIC_CONNECTED"
        )
        port map (
            i_clk => clk,
            i_rst_n => rst_n,
            i_active_valid => active_valid,
            i_active_config => active_config,
            i_register_image => register_image,
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
            o_event => event_out,
            i_event_ready => event_ready,
            o_adr => adr,
            o_csn => csn,
            o_rdn => rdn,
            o_wrn => wrn,
            o_oen => oen,
            i_d => d_bus,
            o_d => d_from_lane,
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
            o_status => status,
            o_faults => faults
        );

    p_chip_model : process (clk)
        variable rdn_previous : std_logic := '1';
        variable wrn_previous : std_logic := '1';
        variable load_previous : std_logic := '0';
        variable write_data_hold : gpx_bus_data_t := (others => '0');
        variable write_address_hold : gpx_bus_address_t := (others => '0');
    begin
        if rising_edge(clk) then
            chip_d_oe <= '0';

            if rst_n = '0' then
                fifo1_fill <= 0;
                fifo2_fill <= 0;
                fifo1_read_index <= 0;
                fifo2_read_index <= 0;
                write_capture <= (others => (others => '0'));
                write_count <= 0;
                empty_read_count <= 0;
                rdn_previous := '1';
                wrn_previous := '1';
                load_previous := '0';
            else
                if fifo_load = '1' and load_previous = '0' then
                    fifo1_fill <= fifo_load_1;
                    fifo2_fill <= fifo_load_2;
                    fifo1_read_index <= 0;
                    fifo2_read_index <= 0;
                end if;
                load_previous := fifo_load;

                if oen = '0' and rdn = '0' then
                    chip_d_oe <= '1';
                    if adr = c_TDC_REG8_IFIFO1 then
                        chip_d_out <= fn_raw_word('0', fifo1_read_index);
                    elsif adr = c_TDC_REG9_IFIFO2 then
                        chip_d_out <= fn_raw_word('1', fifo2_read_index);
                    else
                        chip_d_out <= (others => '0');
                    end if;
                end if;

                if rdn = '0' and rdn_previous = '1' then
                    if adr = c_TDC_REG8_IFIFO1 and fifo1_fill = 0 then
                        empty_read_count <= empty_read_count + 1;
                    elsif adr = c_TDC_REG9_IFIFO2 and fifo2_fill = 0 then
                        empty_read_count <= empty_read_count + 1;
                    end if;
                end if;

                if rdn = '1' and rdn_previous = '0' then
                    if adr = c_TDC_REG8_IFIFO1 and fifo1_fill > 0 then
                        fifo1_fill <= fifo1_fill - 1;
                        fifo1_read_index <= fifo1_read_index + 1;
                    elsif adr = c_TDC_REG9_IFIFO2 and fifo2_fill > 0 then
                        fifo2_fill <= fifo2_fill - 1;
                        fifo2_read_index <= fifo2_read_index + 1;
                    end if;
                end if;
                rdn_previous := rdn;

                if wrn = '0' then
                    write_data_hold := d_bus;
                    write_address_hold := adr;
                end if;
                if wrn = '1' and wrn_previous = '0' then
                    write_capture(to_integer(unsigned(write_address_hold))) <=
                        write_data_hold;
                    write_count <= write_count + 1;
                end if;
                wrn_previous := wrn;
            end if;
        end if;
    end process p_chip_model;

    p_scoreboard : process (clk)
        variable expected_word : gpx_bus_data_t;
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                event_count <= 0;
            elsif event_out.valid = '1' and event_ready = '1' then
                assert event_out.chip_index = 0
                    report "V2-GPX-LANE-TB Chip identity mismatch"
                    severity failure;
                assert event_out.shot_context = fn_shot
                    report "V2-GPX-LANE-TB Shot context mismatch"
                    severity failure;
                assert event_out.chip_shot_seq = 0
                    report "V2-GPX-LANE-TB Shot sequence mismatch"
                    severity failure;

                case event_count is
                    when 0 to C_IFIFO1_WORDS - 1 =>
                        expected_word := fn_raw_word('0', event_count);
                        assert event_out.kind = GPX_RAW_DATA and
                               event_out.ififo_id = '0' and
                               event_out.raw_word = expected_word
                            report "V2-GPX-LANE-TB IFIFO1 data mismatch"
                            severity failure;
                    when C_IFIFO1_WORDS =>
                        assert event_out.kind = GPX_RAW_IFIFO1_DONE and
                               event_out.ififo_id = '0'
                            report "V2-GPX-LANE-TB IFIFO1 completion mismatch"
                            severity failure;
                    when C_IFIFO1_WORDS + 1 to
                         C_IFIFO1_WORDS + C_IFIFO2_WORDS =>
                        expected_word := fn_raw_word(
                            '1', event_count - C_IFIFO1_WORDS - 1);
                        assert event_out.kind = GPX_RAW_DATA and
                               event_out.ififo_id = '1' and
                               event_out.raw_word = expected_word
                            report "V2-GPX-LANE-TB IFIFO2 data mismatch"
                            severity failure;
                    when C_IFIFO1_WORDS + C_IFIFO2_WORDS + 1 =>
                        assert event_out.kind = GPX_RAW_DRAIN_DONE and
                               event_out.ififo_id = '1' and
                               event_out.faulted = '0'
                            report "V2-GPX-LANE-TB final completion mismatch"
                            severity failure;
                    when others =>
                        assert false
                            report "V2-GPX-LANE-TB unexpected extra event"
                            severity failure;
                end case;
                event_count <= event_count + 1;
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
            constant timeout_clocks : in positive := 20000
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

        wait_until_high(status.initialized,
            "V2-GPX-LANE-TB initialization timeout");
        -- The proven initializer writes its 11-register mandatory sequence
        -- and then issues one Reg4 master-reset write; reserved image slots
        -- are intentionally not bus transactions.
        assert write_count >= 12
            report "V2-GPX-LANE-TB incomplete GPX initialization sequence"
            severity failure;
        assert write_capture(0)(c_REG0_TSTART_RISE) = '1' and
               write_capture(0)(c_REG0_TSTART_FALL) = '0' and
               write_capture(0)(c_REG0_TSTOP_RISE_LO + 3 downto
                                 c_REG0_TSTOP_RISE_LO) = "1111" and
               write_capture(0)(c_REG0_TSTOP_RISE_HI downto
                                 c_REG0_TSTOP_RISE_LO + 4) = "0000" and
               write_capture(0)(c_REG0_TSTOP_FALL_HI downto
                                 c_REG0_TSTOP_FALL_LO) = x"00"
            report "V2-GPX-LANE-TB Reg0 topology normalization mismatch"
            severity failure;

        run_enable <= '1';
        wait_until_high(shot_ready,
            "V2-GPX-LANE-TB run arm timeout");

        fifo_load_1 <= C_IFIFO1_WORDS;
        fifo_load_2 <= C_IFIFO2_WORDS;
        fifo_load <= '1';
        wait_clocks(1);
        fifo_load <= '0';
        wait_clocks(4);

        shot <= fn_shot;
        wait until rising_edge(clk) and shot_ready = '1';
        wait for 1 ps;
        shot <= C_SHOT_START_EVENT_IDLE;

        event_ready <= '0';
        wait_clocks(5);
        irflag <= '1';
        wait_clocks(4);
        irflag <= '0';

        wait_until_high(event_out.valid,
            "V2-GPX-LANE-TB first event timeout");
        held_event := event_out;
        wait_clocks(5);
        assert event_out = held_event
            report "V2-GPX-LANE-TB event changed under backpressure"
            severity failure;
        event_ready <= '1';

        for timeout in 0 to 20000 loop
            wait until rising_edge(clk);
            wait for 1 ps;
            exit when event_count = C_IFIFO1_WORDS + C_IFIFO2_WORDS + 2;
        end loop;
        assert event_count = C_IFIFO1_WORDS + C_IFIFO2_WORDS + 2
            report "V2-GPX-LANE-TB ordered drain did not complete"
            severity failure;
        wait_until_high(shot_ready,
            "V2-GPX-LANE-TB did not re-arm after terminal event");
        assert status.chip_shot_seq = 1 and empty_read_count = 0
            report "V2-GPX-LANE-TB sequence or empty-read contract failed"
            severity failure;
        assert faults = C_GPX_LANE_FAULTS_CLEAR
            report "V2-GPX-LANE-TB unexpected lane fault"
            severity failure;

        run_enable <= '0';
        wait_until_high(safe, "V2-GPX-LANE-TB stop-to-safe timeout");

        updated_config := active_config;
        updated_config.version := to_unsigned(8, 16);
        updated_config.source.tdc.start_offset := to_unsigned(77, 18);
        active_config <= updated_config;
        wait_clocks(2);
        wait_until_high(config_ready,
            "V2-GPX-LANE-TB config ready timeout");
        config_apply <= '1';
        wait_clocks(1);
        config_apply <= '0';
        wait_until_high(config_done,
            "V2-GPX-LANE-TB config apply timeout");
        assert write_capture(5)(c_REG5_STARTOFF1_HI downto
                                c_REG5_STARTOFF1_LO) =
               std_logic_vector(to_unsigned(77, 18))
            report "V2-GPX-LANE-TB StartOff1 image update mismatch"
            severity failure;

        report "LIDAR_V2_GPX_ACQUISITION_LANE_PASS tdc_mhz=" &
            integer'image(G_TDC_CLK_MHZ)
            severity note;
        simulation_done <= true;
        stop;
        wait;
    end process p_stimulus;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_acquisition_lane_150 is
end entity tb_lidar_gpx_acquisition_lane_150;

architecture sim of tb_lidar_gpx_acquisition_lane_150 is
begin
    u_test : entity work.tb_lidar_gpx_acquisition_lane
        generic map (G_TDC_CLK_MHZ => 150);
end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_acquisition_lane_200 is
end entity tb_lidar_gpx_acquisition_lane_200;

architecture sim of tb_lidar_gpx_acquisition_lane_200 is
begin
    u_test : entity work.tb_lidar_gpx_acquisition_lane
        generic map (G_TDC_CLK_MHZ => 200);
end architecture sim;
