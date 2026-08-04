library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;

entity lidar_commit_calculator is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk            : in  std_logic;
        i_rst_n          : in  std_logic;
        i_start          : in  std_logic;
        i_source         : in  lidar_runtime_config_t;
        o_busy           : out std_logic;
        o_done           : out std_logic;
        o_start_rejected : out std_logic;
        o_error          : out lidar_cfg_error_t;
        o_derived        : out lidar_derived_config_t
    );
end entity lidar_commit_calculator;

architecture rtl of lidar_commit_calculator is

    type phase_t is (P_IDLE, P_VALIDATE, P_DERIVE);

    signal r_phase          : phase_t := P_IDLE;
    signal r_busy           : std_logic := '0';
    signal r_done           : std_logic := '0';
    signal r_start_rejected : std_logic := '0';
    signal r_error          : lidar_cfg_error_t := CFG_OK;
    signal r_source         : lidar_runtime_config_t;
    signal r_result         : lidar_derived_config_t;
    signal r_validator_start : std_logic := '0';
    signal r_deriver_start   : std_logic := '0';

    signal w_validator_done    : std_logic;
    signal w_validator_error   : lidar_cfg_error_t;
    signal w_total_states      : u16_t;
    signal w_angle_product     : unsigned(63 downto 0);
    signal w_capture_window    : u32_t;
    signal w_val_mul_start     : std_logic;
    signal w_val_mul_left      : u32_t;
    signal w_val_mul_right     : u16_t;

    signal w_deriver_done      : std_logic;
    signal w_deriver_fault     : std_logic;
    signal w_work_derived      : lidar_derived_config_t;
    signal w_der_mul_start     : std_logic;
    signal w_der_mul_left      : u32_t;
    signal w_der_mul_right     : u16_t;
    signal w_der_div_start     : std_logic;
    signal w_der_div_numerator : unsigned(63 downto 0);
    signal w_der_div_denominator : u32_t;

    signal w_mul_start   : std_logic;
    signal w_mul_left    : u32_t;
    signal w_mul_right   : u16_t;
    signal w_mul_done    : std_logic;
    signal w_mul_product : unsigned(63 downto 0);

    signal w_div_done      : std_logic;
    signal w_div_zero      : std_logic;
    signal w_div_quotient  : unsigned(63 downto 0);
    signal w_div_remainder : u32_t;

begin

    o_busy           <= r_busy;
    o_done           <= r_done;
    o_start_rejected <= r_start_rejected;
    o_error          <= r_error;
    o_derived        <= r_result;

    w_mul_start <= w_val_mul_start when r_phase = P_VALIDATE
        else w_der_mul_start when r_phase = P_DERIVE
        else '0';
    w_mul_left <= w_val_mul_left when r_phase = P_VALIDATE
        else w_der_mul_left;
    w_mul_right <= w_val_mul_right when r_phase = P_VALIDATE
        else w_der_mul_right;

    u_multiplier : entity work.lidar_u32_u16_multiplier_seq
        port map (
            i_clk     => i_clk,
            i_rst_n   => i_rst_n,
            i_start   => w_mul_start,
            i_left    => w_mul_left,
            i_right   => w_mul_right,
            o_busy    => open,
            o_done    => w_mul_done,
            o_product => w_mul_product
        );

    u_divider : entity work.lidar_u64_u32_divider_seq
        port map (
            i_clk         => i_clk,
            i_rst_n       => i_rst_n,
            i_start       => w_der_div_start,
            i_numerator   => w_der_div_numerator,
            i_denominator => w_der_div_denominator,
            o_busy        => open,
            o_done        => w_div_done,
            o_div_zero    => w_div_zero,
            o_quotient    => w_div_quotient,
            o_remainder   => w_div_remainder
        );

    u_validator : entity work.lidar_config_validator_seq
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                => i_clk,
            i_rst_n              => i_rst_n,
            i_start              => r_validator_start,
            i_source             => r_source,
            o_busy               => open,
            o_done               => w_validator_done,
            o_error              => w_validator_error,
            o_total_states       => w_total_states,
            o_angle_product      => w_angle_product,
            o_capture_window_5ns => w_capture_window,
            o_mul_start          => w_val_mul_start,
            o_mul_left           => w_val_mul_left,
            o_mul_right          => w_val_mul_right,
            i_mul_done           => w_mul_done,
            i_mul_product        => w_mul_product
        );

    u_deriver : entity work.lidar_config_deriver_seq
        generic map (
            G_BUILD_CONFIG => G_BUILD_CONFIG
        )
        port map (
            i_clk                => i_clk,
            i_rst_n              => i_rst_n,
            i_start              => r_deriver_start,
            i_virtual_ticks_lo   => r_source.motor.virtual_ticks_lo,
            i_face_centers       => r_source.mirror.face_centers,
            i_common_half_width  => r_source.mirror.common_half_width,
            i_target_range_5ns   => r_source.laser.target_range_window_5ns,
            i_active_chip_mask   => r_source.tdc.active_chip_mask,
            i_falling_enable     => r_source.tdc.falling_enable,
            i_total_states       => w_total_states,
            i_angle_product      => w_angle_product,
            i_capture_window_5ns => w_capture_window,
            o_busy               => open,
            o_done               => w_deriver_done,
            o_fault              => w_deriver_fault,
            o_derived            => w_work_derived,
            o_mul_start          => w_der_mul_start,
            o_mul_left           => w_der_mul_left,
            o_mul_right          => w_der_mul_right,
            i_mul_done           => w_mul_done,
            i_mul_product        => w_mul_product,
            o_div_start          => w_der_div_start,
            o_div_numerator      => w_der_div_numerator,
            o_div_denominator    => w_der_div_denominator,
            i_div_done           => w_div_done,
            i_div_zero           => w_div_zero,
            i_div_quotient       => w_div_quotient,
            i_div_remainder      => w_div_remainder
        );

    p_control : process (i_clk)
    begin
        if rising_edge(i_clk) then
            r_done            <= '0';
            r_start_rejected  <= '0';
            r_validator_start <= '0';
            r_deriver_start   <= '0';

            if i_rst_n = '0' then
                r_phase          <= P_IDLE;
                r_busy           <= '0';
                r_done           <= '0';
                r_start_rejected <= '0';
                r_error          <= CFG_OK;
            else
                if i_start = '1' and r_busy = '1' then
                    r_start_rejected <= '1';
                end if;

                case r_phase is
                    when P_IDLE =>
                        if i_start = '1' then
                            r_source <= i_source;
                            r_busy <= '1';
                            r_error <= CFG_OK;
                            r_validator_start <= '1';
                            r_phase <= P_VALIDATE;
                        end if;

                    when P_VALIDATE =>
                        if w_validator_done = '1' then
                            if w_validator_error /= CFG_OK then
                                r_error <= w_validator_error;
                                r_busy  <= '0';
                                r_done  <= '1';
                                r_phase <= P_IDLE;
                            else
                                r_deriver_start <= '1';
                                r_phase <= P_DERIVE;
                            end if;
                        end if;

                    when P_DERIVE =>
                        if w_deriver_done = '1' then
                            if w_deriver_fault = '1' then
                                r_error <= CFG_INTERNAL_ARITHMETIC;
                            else
                                r_result <= w_work_derived;
                                r_error <= CFG_OK;
                            end if;
                            r_busy  <= '0';
                            r_done  <= '1';
                            r_phase <= P_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_control;

end architecture rtl;
