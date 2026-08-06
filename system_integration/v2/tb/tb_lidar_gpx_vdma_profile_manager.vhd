library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.env.all;

use work.lidar_build_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity tb_lidar_gpx_vdma_profile_manager is
    generic (
        G_CLK_MHZ      : positive := 150;
        G_OUTPUT_WIDTH : positive := 32
    );
end entity tb_lidar_gpx_vdma_profile_manager;

architecture sim of tb_lidar_gpx_vdma_profile_manager is

    function fn_build_config return lidar_build_config_t is
        variable result : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG;
    begin
        result.proc_clk_mhz := G_CLK_MHZ;
        result.tdc_clk_mhz := 200;
        result.stream_clock_mode := STREAM_CLOCK_ASYNC;
        result.stops_per_chip := 1;
        result.output_width := G_OUTPUT_WIDTH;
        return result;
    end function fn_build_config;

    constant C_BUILD_CONFIG : lidar_build_config_t := fn_build_config;
    constant C_CLK_PERIOD : time := 1 us / G_CLK_MHZ;
    constant C_RISE_STRIDE : natural := fn_gpx_vdma_stride_bytes(
        4, 7, G_OUTPUT_WIDTH);
    constant C_FALL_STRIDE : natural := fn_gpx_vdma_stride_bytes(
        2, 7, G_OUTPUT_WIDTH);

    signal clk : std_logic := '0';
    signal rst_n : std_logic := '0';
    signal abort_run : std_logic := '0';

    signal rise_request_valid : std_logic := '0';
    signal rise_request_ready : std_logic;
    signal rise_mask : chip_mask_t := (others => '0');
    signal rise_returns : unsigned(2 downto 0) := (others => '0');
    signal rise_shots : shot_index_t := (others => '0');
    signal rise_rejected : std_logic;
    signal rise_activate_valid : std_logic := '0';
    signal rise_activate_ready : std_logic;
    signal rise_idle : std_logic := '0';
    signal rise_cfg_valid : std_logic;
    signal rise_cfg_ready : std_logic := '0';
    signal rise_cfg_enable : std_logic;
    signal rise_cfg_hsize : gpx_vdma_geometry_value_t;
    signal rise_cfg_vsize : gpx_vdma_geometry_value_t;
    signal rise_cfg_stride : gpx_vdma_geometry_value_t;
    signal rise_active : gpx_vdma_lane_profile_t;
    signal rise_activated : std_logic;
    signal rise_pending : std_logic;
    signal rise_busy : std_logic;

    signal fall_request_valid : std_logic := '0';
    signal fall_request_ready : std_logic;
    signal fall_mask : chip_mask_t := (others => '0');
    signal fall_returns : unsigned(2 downto 0) := (others => '0');
    signal fall_shots : shot_index_t := (others => '0');
    signal fall_rejected : std_logic;
    signal fall_activate_valid : std_logic := '0';
    signal fall_activate_ready : std_logic;
    signal fall_idle : std_logic := '0';
    signal fall_cfg_valid : std_logic;
    signal fall_cfg_ready : std_logic := '0';
    signal fall_cfg_enable : std_logic;
    signal fall_cfg_hsize : gpx_vdma_geometry_value_t;
    signal fall_cfg_vsize : gpx_vdma_geometry_value_t;
    signal fall_cfg_stride : gpx_vdma_geometry_value_t;
    signal fall_active : gpx_vdma_lane_profile_t;
    signal fall_activated : std_logic;
    signal fall_pending : std_logic;
    signal fall_busy : std_logic;

begin

    clk <= not clk after C_CLK_PERIOD / 2;

    u_rise : entity work.lidar_gpx_vdma_profile_manager
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => true
        )
        port map (
            i_clk                  => clk,
            i_rst_n                => rst_n,
            i_abort                => abort_run,
            i_request_valid        => rise_request_valid,
            o_request_ready        => rise_request_ready,
            i_lane_chip_mask       => rise_mask,
            i_visible_returns      => rise_returns,
            i_planned_shots        => rise_shots,
            o_request_rejected     => rise_rejected,
            i_activate_valid       => rise_activate_valid,
            o_activate_ready       => rise_activate_ready,
            i_datapath_idle        => rise_idle,
            o_vdma_cfg_valid       => rise_cfg_valid,
            i_vdma_cfg_ready       => rise_cfg_ready,
            o_vdma_cfg_enable      => rise_cfg_enable,
            o_vdma_hsize_bytes     => rise_cfg_hsize,
            o_vdma_vsize_lines     => rise_cfg_vsize,
            o_vdma_stride_bytes    => rise_cfg_stride,
            o_active_profile       => rise_active,
            o_profile_activated    => rise_activated,
            o_pending_valid        => rise_pending,
            o_busy                 => rise_busy
        );

    u_fall : entity work.lidar_gpx_vdma_profile_manager
        generic map (
            G_BUILD_CONFIG => C_BUILD_CONFIG,
            G_LANE_RISE    => false
        )
        port map (
            i_clk                  => clk,
            i_rst_n                => rst_n,
            i_abort                => abort_run,
            i_request_valid        => fall_request_valid,
            o_request_ready        => fall_request_ready,
            i_lane_chip_mask       => fall_mask,
            i_visible_returns      => fall_returns,
            i_planned_shots        => fall_shots,
            o_request_rejected     => fall_rejected,
            i_activate_valid       => fall_activate_valid,
            o_activate_ready       => fall_activate_ready,
            i_datapath_idle        => fall_idle,
            o_vdma_cfg_valid       => fall_cfg_valid,
            i_vdma_cfg_ready       => fall_cfg_ready,
            o_vdma_cfg_enable      => fall_cfg_enable,
            o_vdma_hsize_bytes     => fall_cfg_hsize,
            o_vdma_vsize_lines     => fall_cfg_vsize,
            o_vdma_stride_bytes    => fall_cfg_stride,
            o_active_profile       => fall_active,
            o_profile_activated    => fall_activated,
            o_pending_valid        => fall_pending,
            o_busy                 => fall_busy
        );

    p_test : process
        procedure check_profile(
            constant value : in gpx_vdma_lane_profile_t;
            constant expected_enabled : in boolean;
            constant expected_slots : in natural;
            constant expected_returns : in positive;
            constant expected_shots : in positive;
            constant expected_stride : in natural
        ) is
            variable expected_cell_words : positive;
            variable expected_raw_words : natural;
            variable expected_hsize : natural;
            variable expected_footer_lines : natural;
        begin
            assert value.valid = '1'
                report "V2-B9-J8-TB profile is not valid"
                severity failure;
            if expected_enabled then
                expected_cell_words := fn_gpx_vdma_cell_word_count(
                    expected_returns);
                expected_raw_words := C_GPX_VDMA_SHOT_META_WORDS +
                    expected_slots * expected_cell_words;
                expected_hsize := fn_gpx_vdma_shot_hsize_bytes(
                    expected_slots, expected_returns, G_OUTPUT_WIDTH);
                expected_footer_lines := fn_gpx_vdma_footer_lines(
                    expected_hsize);
                assert value.enabled = '1' and
                       to_integer(value.slot_count) = expected_slots and
                       to_integer(value.visible_returns) = expected_returns and
                       to_integer(value.cell_word_count) = expected_cell_words and
                       to_integer(value.planned_shots) = expected_shots and
                       to_integer(value.raw_line_words) = expected_raw_words and
                       to_integer(value.hsize_bytes) = expected_hsize and
                       to_integer(value.hsize_words) = expected_hsize / 4 and
                       to_integer(value.footer_lines) = expected_footer_lines and
                       to_integer(value.vsize_lines) =
                           expected_shots + expected_footer_lines
                    report "V2-B9-J8-TB enabled profile geometry mismatch"
                    severity failure;
            else
                assert value.enabled = '0' and
                       to_integer(value.hsize_bytes) = 0 and
                       to_integer(value.vsize_lines) = 0
                    report "V2-B9-J8-TB disabled profile is not zero geometry"
                    severity failure;
            end if;
            assert to_integer(value.stride_bytes) = expected_stride
                report "V2-B9-J8-TB fixed STRIDE mismatch"
                severity failure;
        end procedure check_profile;

        procedure wait_rise_pending is
        begin
            for cycle in 0 to 100 loop
                wait until rising_edge(clk);
                exit when rise_pending = '1';
            end loop;
            assert rise_pending = '1'
                report "V2-B9-J8-TB Rise profile calculation timeout"
                severity failure;
        end procedure wait_rise_pending;

        procedure wait_fall_pending is
        begin
            for cycle in 0 to 100 loop
                wait until rising_edge(clk);
                exit when fall_pending = '1';
            end loop;
            assert fall_pending = '1'
                report "V2-B9-J8-TB Fall profile calculation timeout"
                severity failure;
        end procedure wait_fall_pending;

        variable preserved_rise : gpx_vdma_lane_profile_t;
    begin
        rst_n <= '0';
        wait for 8 * C_CLK_PERIOD;
        wait until rising_edge(clk);
        rst_n <= '1';

        -- Invalid Return count is rejected without disturbing Active.
        rise_mask <= "0001";
        rise_returns <= to_unsigned(0, 3);
        rise_shots <= to_unsigned(2, 16);
        rise_request_valid <= '1';
        wait until rising_edge(clk) and rise_request_ready = '1';
        rise_request_valid <= '0';
        wait until falling_edge(clk);
        assert rise_rejected = '1' and rise_active.valid = '0'
            report "V2-B9-J8-TB invalid request was not rejected"
            severity failure;

        -- Minimum one-Cell profile: activation waits for a safe boundary and
        -- the VDMA acknowledgement owns the exact Active transition.
        wait until rising_edge(clk);
        rise_mask <= "0001";
        rise_returns <= to_unsigned(1, 3);
        rise_shots <= to_unsigned(2, 16);
        rise_request_valid <= '1';
        wait until rising_edge(clk) and rise_request_ready = '1';
        rise_request_valid <= '0';
        wait_rise_pending;
        assert rise_active.valid = '0'
            report "V2-B9-J8-TB Active changed before activation"
            severity failure;

        rise_activate_valid <= '1';
        rise_idle <= '0';
        for cycle in 0 to 2 loop
            wait until rising_edge(clk);
            assert rise_activate_ready = '0' and rise_cfg_valid = '0'
                report "V2-B9-J8-TB unsafe boundary was accepted"
                severity failure;
        end loop;
        rise_idle <= '1';
        wait until rising_edge(clk) and rise_activate_ready = '1';
        rise_activate_valid <= '0';
        wait until falling_edge(clk);
        assert rise_cfg_valid = '1' and rise_cfg_enable = '1' and
               to_integer(rise_cfg_hsize) =
                   fn_gpx_vdma_shot_hsize_bytes(1, 1, G_OUTPUT_WIDTH) and
               to_integer(rise_cfg_vsize) = 2 +
                   fn_gpx_vdma_footer_lines(
                       fn_gpx_vdma_shot_hsize_bytes(
                           1, 1, G_OUTPUT_WIDTH)) and
               to_integer(rise_cfg_stride) = C_RISE_STRIDE
            report "V2-B9-J8-TB VDMA minimum request mismatch"
            severity failure;
        for cycle in 0 to 2 loop
            wait until rising_edge(clk);
            assert rise_cfg_valid = '1' and rise_active.valid = '0'
                report "V2-B9-J8-TB VDMA request was not held"
                severity failure;
        end loop;
        rise_cfg_ready <= '1';
        wait until rising_edge(clk);
        rise_cfg_ready <= '0';
        wait until falling_edge(clk);
        assert rise_activated = '1'
            report "V2-B9-J8-TB Rise activation pulse missing"
            severity failure;
        check_profile(rise_active, true, 1, 1, 2, C_RISE_STRIDE);

        -- Maximum Rise topology and Return count replace Active only after the
        -- second VDMA acknowledgement.
        rise_mask <= "1111";
        rise_returns <= to_unsigned(7, 3);
        rise_shots <= to_unsigned(1800, 16);
        rise_request_valid <= '1';
        wait until rising_edge(clk) and rise_request_ready = '1';
        rise_request_valid <= '0';
        wait_rise_pending;
        check_profile(rise_active, true, 1, 1, 2, C_RISE_STRIDE);
        rise_activate_valid <= '1';
        wait until rising_edge(clk) and rise_activate_ready = '1';
        rise_activate_valid <= '0';
        rise_cfg_ready <= '1';
        wait until rising_edge(clk) and rise_cfg_valid = '1';
        rise_cfg_ready <= '0';
        wait until falling_edge(clk);
        check_profile(rise_active, true, 4, 7, 1800, C_RISE_STRIDE);
        preserved_rise := rise_active;

        -- Abort cancels Pending but deliberately preserves the last programmed
        -- Active profile for the next run.
        rise_mask <= "0011";
        rise_returns <= to_unsigned(3, 3);
        rise_shots <= to_unsigned(100, 16);
        rise_request_valid <= '1';
        wait until rising_edge(clk) and rise_request_ready = '1';
        rise_request_valid <= '0';
        wait_rise_pending;
        abort_run <= '1';
        wait until rising_edge(clk);
        abort_run <= '0';
        wait until falling_edge(clk);
        assert rise_pending = '0' and rise_busy = '0' and
               rise_active = preserved_rise
            report "V2-B9-J8-TB abort did not preserve Active"
            severity failure;

        -- Dedicated Fall rejects a Rise-only mask, then accepts chips 2/3.
        fall_mask <= "0001";
        fall_returns <= to_unsigned(1, 3);
        fall_shots <= to_unsigned(10, 16);
        fall_request_valid <= '1';
        wait until rising_edge(clk) and fall_request_ready = '1';
        fall_request_valid <= '0';
        wait until falling_edge(clk);
        assert fall_rejected = '1'
            report "V2-B9-J8-TB invalid Fall mask was not rejected"
            severity failure;

        wait until rising_edge(clk);
        fall_mask <= "1100";
        fall_returns <= to_unsigned(2, 3);
        fall_shots <= to_unsigned(10, 16);
        fall_request_valid <= '1';
        wait until rising_edge(clk) and fall_request_ready = '1';
        fall_request_valid <= '0';
        wait_fall_pending;
        fall_idle <= '1';
        fall_activate_valid <= '1';
        wait until rising_edge(clk) and fall_activate_ready = '1';
        fall_activate_valid <= '0';
        fall_cfg_ready <= '1';
        wait until rising_edge(clk) and fall_cfg_valid = '1';
        fall_cfg_ready <= '0';
        wait until falling_edge(clk);
        check_profile(fall_active, true, 2, 2, 10, C_FALL_STRIDE);

        -- A zero-chip Fall profile is a legal lane-disable transaction.
        fall_mask <= "0000";
        fall_returns <= to_unsigned(1, 3);
        fall_shots <= to_unsigned(10, 16);
        fall_request_valid <= '1';
        wait until rising_edge(clk) and fall_request_ready = '1';
        fall_request_valid <= '0';
        wait_fall_pending;
        fall_activate_valid <= '1';
        wait until rising_edge(clk) and fall_activate_ready = '1';
        fall_activate_valid <= '0';
        wait until falling_edge(clk);
        assert fall_cfg_valid = '1' and fall_cfg_enable = '0' and
               fall_cfg_hsize = 0 and fall_cfg_vsize = 0
            report "V2-B9-J8-TB lane-disable VDMA request mismatch"
            severity failure;
        fall_cfg_ready <= '1';
        wait until rising_edge(clk);
        fall_cfg_ready <= '0';
        wait until falling_edge(clk);
        check_profile(fall_active, false, 0, 1, 10, C_FALL_STRIDE);

        report "LIDAR_V2_GPX_VDMA_PROFILE_MANAGER_PASS proc_mhz=" &
            integer'image(G_CLK_MHZ) & " width=" &
            integer'image(G_OUTPUT_WIDTH)
            severity note;
        finish;
    end process p_test;

end architecture sim;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_vdma_profile_manager_150_32 is end entity;
architecture sim of tb_lidar_gpx_vdma_profile_manager_150_32 is begin
    u : entity work.tb_lidar_gpx_vdma_profile_manager
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 32);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_vdma_profile_manager_150_64 is end entity;
architecture sim of tb_lidar_gpx_vdma_profile_manager_150_64 is begin
    u : entity work.tb_lidar_gpx_vdma_profile_manager
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 64);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_vdma_profile_manager_150_128 is end entity;
architecture sim of tb_lidar_gpx_vdma_profile_manager_150_128 is begin
    u : entity work.tb_lidar_gpx_vdma_profile_manager
        generic map (G_CLK_MHZ => 150, G_OUTPUT_WIDTH => 128);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_vdma_profile_manager_200_32 is end entity;
architecture sim of tb_lidar_gpx_vdma_profile_manager_200_32 is begin
    u : entity work.tb_lidar_gpx_vdma_profile_manager
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 32);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_vdma_profile_manager_200_64 is end entity;
architecture sim of tb_lidar_gpx_vdma_profile_manager_200_64 is begin
    u : entity work.tb_lidar_gpx_vdma_profile_manager
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 64);
end architecture;

library ieee;
use ieee.std_logic_1164.all;

entity tb_lidar_gpx_vdma_profile_manager_200_128 is end entity;
architecture sim of tb_lidar_gpx_vdma_profile_manager_200_128 is begin
    u : entity work.tb_lidar_gpx_vdma_profile_manager
        generic map (G_CLK_MHZ => 200, G_OUTPUT_WIDTH => 128);
end architecture;
