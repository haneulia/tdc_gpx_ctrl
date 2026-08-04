library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;

entity lidar_config_validator_seq is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk                : in  std_logic;
        i_rst_n              : in  std_logic;
        i_start              : in  std_logic;
        i_source             : in  lidar_runtime_config_t;
        o_busy               : out std_logic;
        o_done               : out std_logic;
        o_error              : out lidar_cfg_error_t;
        o_total_states       : out u16_t;
        o_angle_product      : out unsigned(63 downto 0);
        o_capture_window_5ns : out u32_t;
        o_mul_start          : out std_logic;
        o_mul_left           : out u32_t;
        o_mul_right          : out u16_t;
        i_mul_done           : in  std_logic;
        i_mul_product        : in  unsigned(63 downto 0)
    );
end entity lidar_config_validator_seq;

architecture rtl of lidar_config_validator_seq is

    type state_t is (
        S_IDLE,
        S_CHECK_BUILD,
        S_CHECK_CPR,
        S_CHECK_MOTOR,
        S_CHECK_FACE_HALF,
        S_LOAD_FACE_CENTER,
        S_CHECK_FACE_CENTER,
        S_LOAD_FACE_PAIR,
        S_CALC_DIRECT_DISTANCE,
        S_CALC_CIRCULAR_DISTANCE,
        S_CHECK_FACE_OVERLAP,
        S_ADVANCE_FACE_PAIR,
        S_CHECK_FACE_MASK,
        S_START_ANGLE_PRODUCT,
        S_WAIT_ANGLE_PRODUCT,
        S_CHECK_PULSE_AND_RANGE,
        S_COMPARE_FIRE_TIMEOUT,
        S_CHECK_FIRE_TIMEOUT,
        S_CALC_CAPTURE_WINDOW,
        S_CHECK_CAPTURE_WINDOW,
        S_CHECK_ACTIVE_MASK,
        S_CHECK_SLOPE_BALANCE,
        S_CHECK_TDC_LIMITS,
        S_FINISH_OK,
        S_FINISH_ERROR
    );

    constant C_BUILD_ERROR : lidar_cfg_error_t :=
        fn_validate_build_config(G_BUILD_CONFIG);
    constant C_PRESENT_MASK : chip_mask_t :=
        fn_present_chip_mask(G_BUILD_CONFIG.num_chips);
    constant C_ALLOWED_FACE_MASK : face_mask_t :=
        fn_active_face_mask(G_BUILD_CONFIG.num_faces);
    constant C_ZERO_CHIP_MASK : chip_mask_t := (others => '0');
    constant C_ZERO_FACE_MASK : face_mask_t := (others => '0');
    constant C_U32_MAX : u32_t := (others => '1');

    signal r_state          : state_t := S_IDLE;
    signal r_busy           : std_logic := '0';
    signal r_done           : std_logic := '0';
    signal r_error          : lidar_cfg_error_t := CFG_OK;
    signal r_total_states   : u16_t;
    signal r_angle_product  : unsigned(63 downto 0);
    signal r_capture_window : u32_t;
    signal r_half_twice     : u16_t;
    signal r_center_left    : u16_t;
    signal r_center_right   : u16_t;
    signal r_direct_distance : u16_t;
    signal r_circular_distance : u16_t;
    signal r_capture_sum     : signed(33 downto 0);
    signal r_fire_timeout_invalid : std_logic := '0';
    signal r_face_left      : natural range 0 to C_MAX_FACES - 1 := 0;
    signal r_face_right     : natural range 0 to C_MAX_FACES - 1 := 1;
    signal r_mul_start      : std_logic := '0';
    signal r_mul_left       : u32_t;
    signal r_mul_right      : u16_t;

begin

    o_busy               <= r_busy;
    o_done               <= r_done;
    o_error              <= r_error;
    o_total_states       <= r_total_states;
    o_angle_product      <= r_angle_product;
    o_capture_window_5ns <= r_capture_window;
    o_mul_start          <= r_mul_start;
    o_mul_left           <= r_mul_left;
    o_mul_right          <= r_mul_right;

    p_validate : process (i_clk)
        variable v_total_states : u16_t;
        variable v_half_twice   : u16_t;
        variable v_range_ext    : signed(33 downto 0);
        variable v_adjust_ext   : signed(33 downto 0);
        variable v_rise_mask    : chip_mask_t;
        variable v_fall_mask    : chip_mask_t;
    begin
        if rising_edge(i_clk) then
            r_done      <= '0';
            r_mul_start <= '0';

            if i_rst_n = '0' then
                r_state     <= S_IDLE;
                r_busy      <= '0';
                r_done      <= '0';
                r_error     <= CFG_OK;
                r_fire_timeout_invalid <= '0';
                r_face_left <= 0;
                r_face_right <= 1;
            else
                case r_state is
                    when S_IDLE =>
                        if i_start = '1' then
                            r_busy  <= '1';
                            r_error <= CFG_OK;
                            r_state <= S_CHECK_BUILD;
                        end if;

                    when S_CHECK_BUILD =>
                        if C_BUILD_ERROR /= CFG_OK then
                            r_error <= C_BUILD_ERROR;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_CHECK_CPR;
                        end if;

                    when S_CHECK_CPR =>
                        if i_source.motor.cpr = 0
                           or i_source.motor.cpr > to_unsigned(C_MAX_CPR, 16) then
                            r_error <= CFG_RUNTIME_CPR;
                            r_state <= S_FINISH_ERROR;
                        else
                            v_total_states := resize(i_source.motor.cpr, 16);
                            case i_source.motor.decode_mode is
                                when DECODE_X1 => null;
                                when DECODE_X2 =>
                                    v_total_states := shift_left(v_total_states, 1);
                                when DECODE_X4 =>
                                    v_total_states := shift_left(v_total_states, 2);
                            end case;
                            r_total_states <= v_total_states;
                            r_state <= S_CHECK_MOTOR;
                        end if;

                    when S_CHECK_MOTOR =>
                        if r_total_states < 2
                           or r_total_states > to_unsigned((2 ** C_POSITION_WIDTH) - 1, 16) then
                            r_error <= CFG_RUNTIME_TOTAL_STATES;
                            r_state <= S_FINISH_ERROR;
                        elsif i_source.motor.virtual_ticks_lo = 0
                              or i_source.motor.virtual_ticks_lo = C_U32_MAX then
                            r_error <= CFG_RUNTIME_VIRTUAL_TICKS;
                            r_state <= S_FINISH_ERROR;
                        elsif resize(i_source.motor.virtual_hi_count, 16)
                              > r_total_states then
                            r_error <= CFG_RUNTIME_VIRTUAL_HI_COUNT;
                            r_state <= S_FINISH_ERROR;
                        elsif resize(i_source.motor.z_offset, 16) >= r_total_states
                              or resize(i_source.motor.z_width, 16) >= r_total_states then
                            r_error <= CFG_RUNTIME_Z_PARAM;
                            r_state <= S_FINISH_ERROR;
                        elsif i_source.motor.simulation_mode /= '0'
                              and i_source.motor.simulation_mode /= '1' then
                            r_error <= CFG_RUNTIME_SOURCE_MODE;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_CHECK_FACE_HALF;
                        end if;

                    when S_CHECK_FACE_HALF =>
                        v_half_twice := shift_left(
                            resize(i_source.mirror.common_half_width, 16), 1);
                        if i_source.mirror.common_half_width = 0
                           or v_half_twice + 1 >= r_total_states then
                            r_error <= CFG_RUNTIME_FACE_HALF_WIDTH;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_half_twice <= v_half_twice;
                            r_face_left <= 0;
                            r_state <= S_LOAD_FACE_CENTER;
                        end if;

                    when S_LOAD_FACE_CENTER =>
                        r_center_left <= resize(
                            i_source.mirror.face_centers(r_face_left), 16);
                        r_state <= S_CHECK_FACE_CENTER;

                    when S_CHECK_FACE_CENTER =>
                        if r_center_left >= r_total_states then
                            r_error <= CFG_RUNTIME_FACE_CENTER;
                            r_state <= S_FINISH_ERROR;
                        elsif r_face_left + 1 >= G_BUILD_CONFIG.num_faces then
                            if G_BUILD_CONFIG.num_faces > 1 then
                                r_face_left  <= 0;
                                r_face_right <= 1;
                                r_state <= S_LOAD_FACE_PAIR;
                            else
                                r_state <= S_CHECK_FACE_MASK;
                            end if;
                        else
                            r_face_left <= r_face_left + 1;
                            r_state <= S_LOAD_FACE_CENTER;
                        end if;

                    when S_LOAD_FACE_PAIR =>
                        r_center_left <= resize(
                            i_source.mirror.face_centers(r_face_left), 16);
                        r_center_right <= resize(
                            i_source.mirror.face_centers(r_face_right), 16);
                        r_state <= S_CALC_DIRECT_DISTANCE;

                    when S_CALC_DIRECT_DISTANCE =>
                        if r_center_left >= r_center_right then
                            r_direct_distance <=
                                r_center_left - r_center_right;
                        else
                            r_direct_distance <=
                                r_center_right - r_center_left;
                        end if;
                        r_state <= S_CALC_CIRCULAR_DISTANCE;

                    when S_CALC_CIRCULAR_DISTANCE =>
                        if r_direct_distance
                           > r_total_states - r_direct_distance then
                            r_circular_distance <=
                                r_total_states - r_direct_distance;
                        else
                            r_circular_distance <= r_direct_distance;
                        end if;
                        r_state <= S_CHECK_FACE_OVERLAP;

                    when S_CHECK_FACE_OVERLAP =>
                        if r_circular_distance <= r_half_twice then
                            r_error <= CFG_RUNTIME_FACE_OVERLAP;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_ADVANCE_FACE_PAIR;
                        end if;

                    when S_ADVANCE_FACE_PAIR =>
                        if r_face_right + 1 < G_BUILD_CONFIG.num_faces then
                            r_face_right <= r_face_right + 1;
                            r_state <= S_LOAD_FACE_PAIR;
                        elsif r_face_left + 2 < G_BUILD_CONFIG.num_faces then
                            r_face_left  <= r_face_left + 1;
                            r_face_right <= r_face_left + 2;
                            r_state <= S_LOAD_FACE_PAIR;
                        else
                            r_state <= S_CHECK_FACE_MASK;
                        end if;

                    when S_CHECK_FACE_MASK =>
                        if not fn_is_binary(i_source.laser.face_enable_mask)
                           or (i_source.laser.face_enable_mask
                               and not C_ALLOWED_FACE_MASK) /= C_ZERO_FACE_MASK
                           or (i_source.laser.face_enable_mask
                               and C_ALLOWED_FACE_MASK) = C_ZERO_FACE_MASK then
                            r_error <= CFG_RUNTIME_FACE_ENABLE_MASK;
                            r_state <= S_FINISH_ERROR;
                        elsif i_source.laser.optical_shot_interval_udeg = 0
                              or i_source.laser.optical_shot_interval_udeg
                                 > to_unsigned(C_FULL_MECHANICAL_UDEG, 30) then
                            r_error <= CFG_RUNTIME_SHOT_ANGLE;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_START_ANGLE_PRODUCT;
                        end if;

                    when S_START_ANGLE_PRODUCT =>
                        r_mul_left <= resize(
                            i_source.laser.optical_shot_interval_udeg, 32);
                        r_mul_right <= r_total_states;
                        r_mul_start <= '1';
                        r_state <= S_WAIT_ANGLE_PRODUCT;

                    when S_WAIT_ANGLE_PRODUCT =>
                        if i_mul_done = '1' then
                            r_angle_product <= i_mul_product;
                            if i_mul_product < to_unsigned(
                               C_FULL_OPTICAL_SCALE_UDEG, 64) then
                                r_error <= CFG_RUNTIME_SHOT_BELOW_ONE_STATE;
                                r_state <= S_FINISH_ERROR;
                            else
                                r_state <= S_CHECK_PULSE_AND_RANGE;
                            end if;
                        end if;

                    when S_CHECK_PULSE_AND_RANGE =>
                        if i_source.laser.fire_width_5ns_ticks = 0
                           or i_source.laser.start_width_5ns_ticks = 0
                           or i_source.laser.stop_width_5ns_ticks = 0 then
                            r_error <= CFG_RUNTIME_FIRE_WIDTH;
                            r_state <= S_FINISH_ERROR;
                        elsif i_source.laser.target_range_window_5ns = 0 then
                            r_error <= CFG_RUNTIME_RANGE_WINDOW;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_COMPARE_FIRE_TIMEOUT;
                        end if;

                    when S_COMPARE_FIRE_TIMEOUT =>
                        if i_source.laser.fire_done_timeout_5ns_ticks = 0
                           or resize(
                               i_source.laser.fire_done_timeout_5ns_ticks, 32)
                              > i_source.laser.target_range_window_5ns then
                            r_fire_timeout_invalid <= '1';
                        else
                            r_fire_timeout_invalid <= '0';
                        end if;
                        r_state <= S_CHECK_FIRE_TIMEOUT;

                    when S_CHECK_FIRE_TIMEOUT =>
                        if r_fire_timeout_invalid = '1' then
                            r_error <= CFG_RUNTIME_FIRE_TIMEOUT;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_CALC_CAPTURE_WINDOW;
                        end if;

                    when S_CALC_CAPTURE_WINDOW =>
                        v_range_ext := signed(resize(
                            i_source.laser.target_range_window_5ns, 34));
                        v_adjust_ext := resize(
                            i_source.tdc.capture_adjust_5ns, 34);
                        r_capture_sum <= v_range_ext + v_adjust_ext;
                        r_state <= S_CHECK_CAPTURE_WINDOW;

                    when S_CHECK_CAPTURE_WINDOW =>
                        if r_capture_sum <= to_signed(0, 34)
                           or r_capture_sum(33 downto 32) /= "00" then
                            r_error <= CFG_RUNTIME_CAPTURE_WINDOW;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_capture_window <= unsigned(
                                r_capture_sum(31 downto 0));
                            r_state <= S_CHECK_ACTIVE_MASK;
                        end if;

                    when S_CHECK_ACTIVE_MASK =>
                        if not fn_is_binary(i_source.tdc.active_chip_mask)
                           or (i_source.tdc.falling_enable /= '0'
                               and i_source.tdc.falling_enable /= '1')
                           or i_source.tdc.active_chip_mask = C_ZERO_CHIP_MASK
                           or (i_source.tdc.active_chip_mask
                               and not C_PRESENT_MASK) /= C_ZERO_CHIP_MASK then
                            r_error <= CFG_RUNTIME_ACTIVE_CHIP_MASK;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_CHECK_SLOPE_BALANCE;
                        end if;

                    when S_CHECK_SLOPE_BALANCE =>
                        if i_source.tdc.falling_enable = '1' then
                            v_rise_mask := i_source.tdc.active_chip_mask
                                and G_BUILD_CONFIG.rise_capability_mask;
                            v_fall_mask := i_source.tdc.active_chip_mask
                                and G_BUILD_CONFIG.fall_capability_mask;
                            if fn_popcount(v_rise_mask) < fn_popcount(v_fall_mask) then
                                r_error <= CFG_RUNTIME_ACTIVE_CHIP_MASK;
                                r_state <= S_FINISH_ERROR;
                            else
                                r_state <= S_CHECK_TDC_LIMITS;
                            end if;
                        else
                            r_state <= S_CHECK_TDC_LIMITS;
                        end if;

                    when S_CHECK_TDC_LIMITS =>
                        if i_source.tdc.max_hits_per_stop = 0
                           or to_integer(i_source.tdc.max_hits_per_stop)
                              > G_BUILD_CONFIG.max_returns_per_stop then
                            r_error <= CFG_RUNTIME_MAX_HITS;
                            r_state <= S_FINISH_ERROR;
                        elsif i_source.tdc.bus_clk_div = 0
                              or i_source.tdc.bus_ticks = 0
                              or i_source.tdc.bus_ticks > 7 then
                            r_error <= CFG_RUNTIME_BUS_TIMING;
                            r_state <= S_FINISH_ERROR;
                        else
                            r_state <= S_FINISH_OK;
                        end if;

                    when S_FINISH_OK =>
                        r_busy  <= '0';
                        r_done  <= '1';
                        r_error <= CFG_OK;
                        r_state <= S_IDLE;

                    when S_FINISH_ERROR =>
                        r_busy  <= '0';
                        r_done  <= '1';
                        r_state <= S_IDLE;
                end case;
            end if;
        end if;
    end process p_validate;

end architecture rtl;
