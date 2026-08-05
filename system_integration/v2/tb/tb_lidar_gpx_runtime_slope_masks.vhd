library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_event_pkg.all;
use work.lidar_gpx_data_pkg.all;

entity tb_lidar_gpx_runtime_slope_masks is
    generic (
        G_CLK_MHZ  : positive := 150;
        G_ALL_DUAL : boolean := false
    );
end entity tb_lidar_gpx_runtime_slope_masks;

architecture sim of tb_lidar_gpx_runtime_slope_masks is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz := 200;
        result.num_chips := 4;
        if G_ALL_DUAL then
            result.rise_capability_mask := "1111";
            result.fall_capability_mask := "1111";
        else
            result.rise_capability_mask := "0011";
            result.fall_capability_mask := "1100";
        end if;
        result.num_faces := 4;
        result.enable_echo_receiver := false;
        result.enable_echo_simulation := false;
        return result;
    end function fn_build_config;

    function fn_context return shot_start_event_t is
        variable result : shot_start_event_t := C_SHOT_START_EVENT_IDLE;
    begin
        result.valid := '1';
        result.request.valid := '1';
        result.request.face_index := to_unsigned(0, 3);
        result.request.position := to_unsigned(100, 15);
        result.request.shot_index := to_unsigned(0, 16);
        result.request.last_in_face := '1';
        result.request.active_version := to_unsigned(7, 16);
        return result;
    end function fn_context;

    function fn_raw_data(
        chip_index : natural;
        slope_value : gpx_slope_t
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
        variable hit_value : natural := 100 + chip_index;
    begin
        result.valid := '1';
        result.kind := GPX_RAW_DATA;
        result.chip_index := to_unsigned(chip_index, 2);
        result.ififo_id := '0';
        result.raw_word(C_GPX_RAW_CHACODE_HI downto
            C_GPX_RAW_CHACODE_LO) := "00";
        if slope_value = GPX_SLOPE_RISE then
            result.raw_word(C_GPX_RAW_SLOPE_BIT) := '1';
        else
            result.raw_word(C_GPX_RAW_SLOPE_BIT) := '0';
            hit_value := 200 + chip_index;
        end if;
        result.raw_word(C_GPX_RAW_HIT_HI downto C_GPX_RAW_HIT_LO) :=
            std_logic_vector(to_unsigned(hit_value, 17));
        result.shot_context := fn_context;
        result.chip_shot_seq := to_unsigned(1, 16);
        return result;
    end function fn_raw_data;

    function fn_raw_control(
        kind_value : gpx_raw_event_kind_t;
        chip_index : natural
    ) return gpx_raw_event_t is
        variable result : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    begin
        result.valid := '1';
        result.kind := kind_value;
        result.chip_index := to_unsigned(chip_index, 2);
        if kind_value = GPX_RAW_DRAIN_DONE then
            result.ififo_id := '1';
        end if;
        result.shot_context := fn_context;
        result.chip_shot_seq := to_unsigned(1, 16);
        return result;
    end function fn_raw_control;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_ACTIVE_RISE_MASK : chip_mask_t := "1111";
    constant C_ACTIVE_FALL_MASK : chip_mask_t :=
        "1111" when G_ALL_DUAL else "0000";

    signal clk          : std_logic := '0';
    signal rst_n        : std_logic := '0';
    signal raw_event    : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal raw_ready    : std_logic;
    signal hit_event    : gpx_hit_event_t;
    signal hit_ready    : std_logic;
    signal cell_event   : gpx_cell_event_t;
    signal cell_ready   : std_logic := '1';
    signal decoder_pulse  : gpx_hit_decoder_faults_t;
    signal decoder_sticky : gpx_hit_decoder_faults_t;
    signal collector_pulse  : gpx_cell_collector_faults_t;
    signal collector_sticky : gpx_cell_collector_faults_t;
    signal data_count    : natural := 0;
    signal control_count : natural := 0;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_decoder : entity work.lidar_gpx_hit_decoder
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk              => clk,
            i_rst_n            => rst_n,
            i_abort            => '0',
            i_clear_sticky     => '0',
            i_active_rise_mask => C_ACTIVE_RISE_MASK,
            i_active_fall_mask => C_ACTIVE_FALL_MASK,
            i_raw_event        => raw_event,
            o_raw_ready        => raw_ready,
            o_hit_event        => hit_event,
            i_hit_ready        => hit_ready,
            o_fault_pulse      => decoder_pulse,
            o_fault_sticky     => decoder_sticky
        );

    u_collector : entity work.lidar_gpx_cell_collector
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG
        )
        port map (
            i_clk               => clk,
            i_rst_n             => rst_n,
            i_abort             => '0',
            i_clear_sticky      => '0',
            i_active_version    => to_unsigned(7, 16),
            i_max_hits_per_stop => to_unsigned(2, 3),
            i_active_rise_mask  => C_ACTIVE_RISE_MASK,
            i_active_fall_mask  => C_ACTIVE_FALL_MASK,
            i_hit_event         => hit_event,
            o_hit_ready         => hit_ready,
            o_cell_event        => cell_event,
            i_cell_ready        => cell_ready,
            o_fault_pulse       => collector_pulse,
            o_fault_sticky      => collector_sticky
        );

    p_monitor : process (clk)
    begin
        if rising_edge(clk) then
            if rst_n = '0' then
                data_count <= 0;
                control_count <= 0;
            elsif cell_event.valid = '1' and cell_ready = '1' then
                if cell_event.kind = GPX_CELL_DATA then
                    if not G_ALL_DUAL then
                        assert cell_event.slope = GPX_SLOPE_RISE
                            report "V2-SLOPE-TB Fall Cell emitted while Fall disabled"
                            severity failure;
                    end if;
                    if cell_event.stop_index = 0 then
                        assert cell_event.hit_count = 1
                            report "V2-SLOPE-TB runtime Rise payload lost"
                            severity failure;
                        if cell_event.slope = GPX_SLOPE_RISE then
                            assert to_integer(cell_event.hits(0)) =
                                    100 + to_integer(cell_event.chip_index)
                                report "V2-SLOPE-TB runtime Rise payload mismatch"
                                severity failure;
                        else
                            assert G_ALL_DUAL and
                                   to_integer(cell_event.hits(0)) =
                                    200 + to_integer(cell_event.chip_index)
                                report "V2-SLOPE-TB runtime Fall payload mismatch"
                                severity failure;
                        end if;
                    else
                        assert cell_event.hit_count = 0
                            report "V2-SLOPE-TB unexpected nonzero Cell"
                            severity failure;
                    end if;
                    data_count <= data_count + 1;
                else
                    control_count <= control_count + 1;
                end if;
            end if;
        end if;
    end process p_monitor;

    p_stimulus : process
        procedure wait_clocks(count : natural) is
        begin
            for index in 1 to count loop
                wait until rising_edge(clk);
            end loop;
        end procedure wait_clocks;

        procedure send_raw(value : gpx_raw_event_t) is
        begin
            loop
                wait until falling_edge(clk);
                exit when raw_ready = '1';
            end loop;
            raw_event <= value;
            wait until rising_edge(clk);
            raw_event <= C_GPX_RAW_EVENT_IDLE;
        end procedure send_raw;
    begin
        rst_n <= '0';
        wait_clocks(5);
        rst_n <= '1';
        wait_clocks(3);

        for chip_index in 0 to C_MAX_CHIPS - 1 loop
            send_raw(fn_raw_data(chip_index, GPX_SLOPE_RISE));
            if G_ALL_DUAL then
                send_raw(fn_raw_data(chip_index, GPX_SLOPE_FALL));
            end if;
            send_raw(fn_raw_control(GPX_RAW_IFIFO1_DONE, chip_index));
            send_raw(fn_raw_control(GPX_RAW_DRAIN_DONE, chip_index));
        end loop;

        while data_count < C_MAX_CHIPS * C_MAX_STOPS_PER_CHIP *
                (1 + boolean'pos(G_ALL_DUAL)) or
              control_count < C_MAX_CHIPS * 2 loop
            wait until rising_edge(clk);
        end loop;
        wait_clocks(3);

        assert decoder_sticky = C_GPX_HIT_DECODER_FAULTS_CLEAR and
               collector_sticky = C_GPX_CELL_COLLECTOR_FAULTS_CLEAR
            report "V2-SLOPE-TB runtime mask alignment raised a fault"
            severity failure;

        report "LIDAR_V2_GPX_RUNTIME_SLOPE_MASKS_PASS"
            severity note;
        finish;
        wait;
    end process p_stimulus;

end architecture sim;

entity tb_lidar_gpx_runtime_slope_masks_150 is end entity;
architecture sim of tb_lidar_gpx_runtime_slope_masks_150 is
begin
    u_tb : entity work.tb_lidar_gpx_runtime_slope_masks
        generic map (G_CLK_MHZ => 150);
end architecture;

entity tb_lidar_gpx_runtime_slope_masks_200 is end entity;
architecture sim of tb_lidar_gpx_runtime_slope_masks_200 is
begin
    u_tb : entity work.tb_lidar_gpx_runtime_slope_masks
        generic map (G_CLK_MHZ => 200);
end architecture;

entity tb_lidar_gpx_runtime_slope_dual4_150 is end entity;
architecture sim of tb_lidar_gpx_runtime_slope_dual4_150 is
begin
    u_tb : entity work.tb_lidar_gpx_runtime_slope_masks
        generic map (G_CLK_MHZ => 150, G_ALL_DUAL => true);
end architecture;

entity tb_lidar_gpx_runtime_slope_dual4_200 is end entity;
architecture sim of tb_lidar_gpx_runtime_slope_dual4_200 is
begin
    u_tb : entity work.tb_lidar_gpx_runtime_slope_masks
        generic map (G_CLK_MHZ => 200, G_ALL_DUAL => true);
end architecture;
