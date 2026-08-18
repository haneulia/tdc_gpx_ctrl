library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;

entity lidar_config_deriver_seq is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;
        i_start              : in  std_logic;
        i_virtual_ticks_lo   : in  u32_t;
        i_face_centers       : in  face_position_array_t;
        i_common_half_width  : in  position_t;
        i_fire_width_5ns     : in  u16_t;
        i_fire_timeout_5ns   : in  u16_t;
        i_gpx_mtimer_ref_ticks : in gpx_mtimer_t;
        i_effective_target_range_5ns : in u32_t;
        i_start_width_5ns    : in  u16_t;
        i_stop_width_5ns     : in  u16_t;
        i_sim_start_delay_5ns : in u32_t;
        i_scan_timeout_5ns   : in  u32_t;
        i_active_chip_mask   : in  chip_mask_t;
        i_falling_enable     : in  std_logic;
        i_total_states       : in  u16_t;
        i_angle_product      : in  unsigned(63 downto 0);
        i_capture_window_5ns : in  u32_t;
        o_busy               : out std_logic;
        o_done               : out std_logic;
        o_fault              : out std_logic;
        o_derived            : out lidar_derived_config_t;
        o_mul_start          : out std_logic;
        o_mul_left           : out u32_t;
        o_mul_right          : out u16_t;
        i_mul_done           : in  std_logic;
        i_mul_product        : in  unsigned(63 downto 0);
        o_div_start          : out std_logic;
        o_div_numerator      : out unsigned(63 downto 0);
        o_div_denominator    : out u32_t;
        i_div_done           : in  std_logic;
        i_div_zero           : in  std_logic;
        i_div_quotient       : in  unsigned(63 downto 0);
        i_div_remainder      : in  u32_t
    );
end entity lidar_config_deriver_seq;

architecture rtl of lidar_config_deriver_seq is

    type state_t is (
        S_IDLE,
        S_INITIALIZE,
        S_LOAD_FACE,
        S_DERIVE_FACE_LOWER,
        S_DERIVE_FACE_UPPER,
        S_STORE_FACE,
        S_START_MECHANICAL_DIV,
        S_WAIT_MECHANICAL_DIV,
        S_START_OPTICAL_DIV,
        S_WAIT_OPTICAL_DIV,
        S_START_SHOT_DIV,
        S_WAIT_SHOT_DIV,
        S_WAIT_COLUMN_DIV,
        S_LOAD_TIME_FIELD,
        S_START_TIME_MULTIPLY,
        S_WAIT_TIME_MULTIPLY,
        S_START_TIME_DIVIDE,
        S_WAIT_TIME_DIVIDE,
        S_STORE_TIME_FIELD,
        S_FINISH,
        S_FINISH_FAULT
    );

    type time_field_t is (
        TIME_FIRE_WIDTH_PROC,
        TIME_FIRE_TIMEOUT_PROC,
        TIME_TARGET_RANGE_PROC,
        TIME_START_WIDTH_PROC,
        TIME_STOP_WIDTH_PROC,
        TIME_SIM_START_DELAY_PROC,
        TIME_CAPTURE_WINDOW_TDC,
        TIME_SCAN_TIMEOUT_TDC
    );

    signal r_state           : state_t := S_IDLE;
    signal r_busy            : std_logic := '0';
    signal r_done            : std_logic := '0';
    signal r_fault           : std_logic := '0';
    signal r_derived         : lidar_derived_config_t;
    signal r_face_index      : natural range 0 to C_MAX_FACES - 1 := 0;
    signal r_face_active     : std_logic := '0';
    signal r_face_center     : position_t;
    signal r_face_lower      : position_t;
    signal r_face_upper      : position_t;
    signal r_mul_start       : std_logic := '0';
    signal r_mul_left        : u32_t;
    signal r_mul_right       : u16_t;
    signal r_div_start       : std_logic := '0';
    signal r_div_numerator   : unsigned(63 downto 0);
    signal r_div_denominator : u32_t;
    signal r_tick_product    : unsigned(63 downto 0);
    signal r_time_field      : time_field_t := TIME_FIRE_WIDTH_PROC;
    signal r_time_clocks     : u32_t;

begin

    o_busy            <= r_busy;
    o_done            <= r_done;
    o_fault           <= r_fault;
    o_derived         <= r_derived;
    o_mul_start       <= r_mul_start;
    o_mul_left        <= r_mul_left;
    o_mul_right       <= r_mul_right;
    o_div_start       <= r_div_start;
    o_div_numerator   <= r_div_numerator;
    o_div_denominator <= r_div_denominator;

    p_derive : process (i_clk)
        variable v_bound        : u16_t;
        variable v_intervals    : u16_t;
        variable v_quotient     : unsigned(63 downto 0);
        variable v_rise_mask    : chip_mask_t;
        variable v_fall_mask    : chip_mask_t;
    begin
        if rising_edge(i_clk) then
            r_done      <= '0';
            r_mul_start <= '0';
            r_div_start <= '0';

            if i_rst_n = '0' then
                r_state      <= S_IDLE;
                r_busy       <= '0';
                r_done       <= '0';
                r_fault      <= '0';
                r_face_index <= 0;
            else
                case r_state is
                    when S_IDLE =>
                        if i_start = '1' then
                            r_busy  <= '1';
                            r_fault <= '0';
                            r_state <= S_INITIALIZE;
                        end if;

                    when S_INITIALIZE =>
                        v_intervals := shift_left(
                            resize(i_common_half_width, 16), 1);
                        r_derived.total_states <= i_total_states;
                        r_derived.virtual_ticks_hi <=
                            i_virtual_ticks_lo + 1;
                        r_derived.face_active_positions <= v_intervals + 1;
                        r_derived.face_angular_intervals <= v_intervals;
                        r_derived.present_chip_mask <=
                            fn_present_chip_mask(G_BUILD_CONFIG.num_chips);
                        r_derived.gpx_mtimer_ref_ticks <=
                            i_gpx_mtimer_ref_ticks;
                        r_derived.effective_target_range_5ns <=
                            i_effective_target_range_5ns;
                        r_derived.capture_window_5ns <= i_capture_window_5ns;

                        if i_falling_enable = '1' then
                            v_rise_mask := i_active_chip_mask
                                and G_BUILD_CONFIG.rise_capability_mask;
                            v_fall_mask := i_active_chip_mask
                                and G_BUILD_CONFIG.fall_capability_mask;
                        else
                            v_rise_mask := i_active_chip_mask;
                            v_fall_mask := (others => '0');
                        end if;
                        r_derived.active_rise_mask <= v_rise_mask;
                        r_derived.active_fall_mask <= v_fall_mask;
                        r_face_index <= 0;
                        r_state <= S_LOAD_FACE;

                    when S_LOAD_FACE =>
                        if r_face_index < G_BUILD_CONFIG.num_faces then
                            r_face_active <= '1';
                            case r_face_index is
                                when 0 =>
                                    r_face_center <=
                                        i_face_centers(0);
                                when 1 =>
                                    r_face_center <=
                                        i_face_centers(1);
                                when 2 =>
                                    r_face_center <=
                                        i_face_centers(2);
                                when 3 =>
                                    r_face_center <=
                                        i_face_centers(3);
                                when 4 =>
                                    r_face_center <=
                                        i_face_centers(4);
                            end case;
                        else
                            r_face_active <= '0';
                            r_face_center <= (others => '0');
                        end if;
                        r_state <= S_DERIVE_FACE_LOWER;

                    when S_DERIVE_FACE_LOWER =>
                        if r_face_active = '1' then
                            if resize(r_face_center, 16) >= resize(
                               i_common_half_width, 16) then
                                v_bound := resize(r_face_center, 16) - resize(
                                    i_common_half_width, 16);
                            else
                                v_bound := i_total_states - (
                                    resize(i_common_half_width, 16)
                                    - resize(r_face_center, 16));
                            end if;
                            r_face_lower <= v_bound(C_POSITION_WIDTH - 1 downto 0);
                        else
                            r_face_lower <= (others => '0');
                        end if;
                        r_state <= S_DERIVE_FACE_UPPER;

                    when S_DERIVE_FACE_UPPER =>
                        if r_face_active = '1' then
                            v_bound := resize(r_face_center, 16) + resize(
                                i_common_half_width, 16);
                            if v_bound >= i_total_states then
                                v_bound := v_bound - i_total_states;
                            end if;
                            r_face_upper <= v_bound(C_POSITION_WIDTH - 1 downto 0);
                        else
                            r_face_upper <= (others => '0');
                        end if;
                        r_state <= S_STORE_FACE;

                    when S_STORE_FACE =>
                        case r_face_index is
                            when 0 =>
                                r_derived.face_lower(0) <= r_face_lower;
                                r_derived.face_upper(0) <= r_face_upper;
                            when 1 =>
                                r_derived.face_lower(1) <= r_face_lower;
                                r_derived.face_upper(1) <= r_face_upper;
                            when 2 =>
                                r_derived.face_lower(2) <= r_face_lower;
                                r_derived.face_upper(2) <= r_face_upper;
                            when 3 =>
                                r_derived.face_lower(3) <= r_face_lower;
                                r_derived.face_upper(3) <= r_face_upper;
                            when 4 =>
                                r_derived.face_lower(4) <= r_face_lower;
                                r_derived.face_upper(4) <= r_face_upper;
                        end case;
                        if r_face_index = C_MAX_FACES - 1 then
                            r_state <= S_START_MECHANICAL_DIV;
                        else
                            r_face_index <= r_face_index + 1;
                            r_state <= S_LOAD_FACE;
                        end if;

                    when S_START_MECHANICAL_DIV =>
                        r_div_numerator <= to_unsigned(
                            C_FULL_MECHANICAL_UDEG, 64);
                        r_div_denominator <= resize(i_total_states, 32);
                        r_div_start <= '1';
                        r_state <= S_WAIT_MECHANICAL_DIV;

                    when S_WAIT_MECHANICAL_DIV =>
                        if i_div_done = '1' then
                            if i_div_zero = '1' then
                                r_state <= S_FINISH_FAULT;
                            else
                                r_derived.mechanical_angle_per_state_udeg <=
                                    i_div_quotient(29 downto 0);
                                r_state <= S_START_OPTICAL_DIV;
                            end if;
                        end if;

                    when S_START_OPTICAL_DIV =>
                        r_div_numerator <= to_unsigned(
                            C_FULL_OPTICAL_SCALE_UDEG, 64);
                        r_div_denominator <= resize(i_total_states, 32);
                        r_div_start <= '1';
                        r_state <= S_WAIT_OPTICAL_DIV;

                    when S_WAIT_OPTICAL_DIV =>
                        if i_div_done = '1' then
                            if i_div_zero = '1' then
                                r_state <= S_FINISH_FAULT;
                            else
                                r_derived.optical_angle_per_state_udeg <=
                                    i_div_quotient(29 downto 0);
                                r_state <= S_START_SHOT_DIV;
                            end if;
                        end if;

                    when S_START_SHOT_DIV =>
                        r_div_numerator <= i_angle_product;
                        r_div_denominator <= to_unsigned(
                            C_FULL_OPTICAL_SCALE_UDEG, 32);
                        r_div_start <= '1';
                        r_state <= S_WAIT_SHOT_DIV;

                    when S_WAIT_SHOT_DIV =>
                        if i_div_done = '1' then
                            if i_div_zero = '1' then
                                r_state <= S_FINISH_FAULT;
                            else
                                v_quotient := i_div_quotient;
                                if i_div_remainder /= 0 then
                                    v_quotient := v_quotient + 1;
                                end if;
                                r_derived.shot_interval_states <=
                                    v_quotient(15 downto 0);
                                r_div_numerator <= resize(
                                    r_derived.face_angular_intervals, 64);
                                r_div_denominator <= v_quotient(31 downto 0);
                                r_div_start <= '1';
                                r_state <= S_WAIT_COLUMN_DIV;
                            end if;
                        end if;

                    when S_WAIT_COLUMN_DIV =>
                        if i_div_done = '1' then
                            if i_div_zero = '1' then
                                r_state <= S_FINISH_FAULT;
                            else
                                v_quotient := i_div_quotient;
                                if i_div_remainder /= 0 then
                                    v_quotient := v_quotient + 1;
                                end if;
                                r_derived.columns_per_face <=
                                    v_quotient(15 downto 0);
                                r_time_field <= TIME_FIRE_WIDTH_PROC;
                                r_state <= S_LOAD_TIME_FIELD;
                            end if;
                        end if;

                    when S_LOAD_TIME_FIELD =>
                        case r_time_field is
                            when TIME_FIRE_WIDTH_PROC =>
                                r_mul_left <= resize(i_fire_width_5ns, 32);
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.proc_clk_mhz, 16);
                            when TIME_FIRE_TIMEOUT_PROC =>
                                r_mul_left <= resize(i_fire_timeout_5ns, 32);
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.proc_clk_mhz, 16);
                            when TIME_TARGET_RANGE_PROC =>
                                -- stop_tdc가 GPX Reg7.MTimer 종료보다 먼저
                                -- 발생하지 않도록 물리 GPX에 기록하는 것과
                                -- 같은 25 ns 올림 실효 왕복시간을 사용한다.
                                r_mul_left <= i_effective_target_range_5ns;
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.proc_clk_mhz, 16);
                            when TIME_START_WIDTH_PROC =>
                                r_mul_left <= resize(i_start_width_5ns, 32);
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.proc_clk_mhz, 16);
                            when TIME_STOP_WIDTH_PROC =>
                                r_mul_left <= resize(i_stop_width_5ns, 32);
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.proc_clk_mhz, 16);
                            when TIME_SIM_START_DELAY_PROC =>
                                r_mul_left <= i_sim_start_delay_5ns;
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.proc_clk_mhz, 16);
                            when TIME_CAPTURE_WINDOW_TDC =>
                                r_mul_left <= i_capture_window_5ns;
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.tdc_clk_mhz, 16);
                            when TIME_SCAN_TIMEOUT_TDC =>
                                r_mul_left <= i_scan_timeout_5ns;
                                r_mul_right <= to_unsigned(
                                    G_BUILD_CONFIG.tdc_clk_mhz, 16);
                        end case;
                        r_state <= S_START_TIME_MULTIPLY;

                    when S_START_TIME_MULTIPLY =>
                        r_mul_start <= '1';
                        r_state <= S_WAIT_TIME_MULTIPLY;

                    when S_WAIT_TIME_MULTIPLY =>
                        if i_mul_done = '1' then
                            r_tick_product <= i_mul_product;
                            r_state <= S_START_TIME_DIVIDE;
                        end if;

                    when S_START_TIME_DIVIDE =>
                        r_div_numerator <= r_tick_product;
                        r_div_denominator <= to_unsigned(
                            C_5NS_TICK_RATE_MHZ, 32);
                        r_div_start <= '1';
                        r_state <= S_WAIT_TIME_DIVIDE;

                    when S_WAIT_TIME_DIVIDE =>
                        if i_div_done = '1' then
                            if i_div_zero = '1' then
                                r_state <= S_FINISH_FAULT;
                            else
                                v_quotient := i_div_quotient;
                                if i_div_remainder /= 0 then
                                    v_quotient := v_quotient + 1;
                                end if;
                                r_time_clocks <= v_quotient(31 downto 0);
                                r_state <= S_STORE_TIME_FIELD;
                            end if;
                        end if;

                    when S_STORE_TIME_FIELD =>
                        case r_time_field is
                            when TIME_FIRE_WIDTH_PROC =>
                                r_derived.fire_width_proc_clks <= r_time_clocks;
                                r_time_field <= TIME_FIRE_TIMEOUT_PROC;
                                r_state <= S_LOAD_TIME_FIELD;
                            when TIME_FIRE_TIMEOUT_PROC =>
                                r_derived.fire_done_timeout_proc_clks <=
                                    r_time_clocks;
                                r_time_field <= TIME_TARGET_RANGE_PROC;
                                r_state <= S_LOAD_TIME_FIELD;
                            when TIME_TARGET_RANGE_PROC =>
                                r_derived.target_range_proc_clks <= r_time_clocks;
                                r_time_field <= TIME_START_WIDTH_PROC;
                                r_state <= S_LOAD_TIME_FIELD;
                            when TIME_START_WIDTH_PROC =>
                                r_derived.start_width_proc_clks <= r_time_clocks;
                                r_time_field <= TIME_STOP_WIDTH_PROC;
                                r_state <= S_LOAD_TIME_FIELD;
                            when TIME_STOP_WIDTH_PROC =>
                                r_derived.stop_width_proc_clks <= r_time_clocks;
                                r_time_field <= TIME_SIM_START_DELAY_PROC;
                                r_state <= S_LOAD_TIME_FIELD;
                            when TIME_SIM_START_DELAY_PROC =>
                                r_derived.simulation_start_delay_proc_clks <=
                                    r_time_clocks;
                                r_time_field <= TIME_CAPTURE_WINDOW_TDC;
                                r_state <= S_LOAD_TIME_FIELD;
                            when TIME_CAPTURE_WINDOW_TDC =>
                                r_derived.capture_window_tdc_clks <= r_time_clocks;
                                r_time_field <= TIME_SCAN_TIMEOUT_TDC;
                                r_state <= S_LOAD_TIME_FIELD;
                            when TIME_SCAN_TIMEOUT_TDC =>
                                r_derived.scan_timeout_tdc_clks <= r_time_clocks;
                                r_state <= S_FINISH;
                        end case;

                    when S_FINISH =>
                        r_busy  <= '0';
                        r_done  <= '1';
                        r_fault <= '0';
                        r_state <= S_IDLE;

                    when S_FINISH_FAULT =>
                        r_busy  <= '0';
                        r_done  <= '1';
                        r_fault <= '1';
                        r_state <= S_IDLE;
                end case;
            end if;
        end if;
    end process p_derive;

end architecture rtl;
