library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_status_pkg.all;

package lidar_csr_map_pkg is

    constant C_LIDAR_CSR_ABI_MAJOR : natural := 2;
    constant C_LIDAR_CSR_ABI_MINOR : natural := 5;

    constant C_LIDAR_CTL_COUNT  : positive := 32;
    constant C_LIDAR_STAT_COUNT : positive := 32;
    constant C_LIDAR_IRQ_COUNT  : positive := 4;

    subtype csr_word_t is std_logic_vector(31 downto 0);
    type csr_word_array_t is array (0 to C_LIDAR_CTL_COUNT - 1) of csr_word_t;

    constant C_CTL_COMMAND             : natural := 0;
    constant C_CTL_MOTOR_PROFILE       : natural := 1;
    constant C_CTL_VIRTUAL_TICKS_LO    : natural := 2;
    constant C_CTL_VIRTUAL_HI_COUNT    : natural := 3;
    constant C_CTL_Z_PROFILE           : natural := 4;
    constant C_CTL_FACE_CENTER_0       : natural := 5;
    constant C_CTL_FACE_CENTER_1       : natural := 6;
    constant C_CTL_FACE_CENTER_2       : natural := 7;
    constant C_CTL_FACE_CENTER_3       : natural := 8;
    constant C_CTL_FACE_CENTER_4       : natural := 9;
    constant C_CTL_FACE_PROFILE        : natural := 10;
    constant C_CTL_LASER_FIRE_PROFILE  : natural := 11;
    constant C_CTL_TARGET_RANGE        : natural := 12;
    constant C_CTL_TDC_PULSE_WIDTHS    : natural := 13;
    constant C_CTL_SIM_START_DELAY     : natural := 14;
    constant C_CTL_SHOT_INTERVAL       : natural := 15;
    constant C_CTL_TDC_BUS_PROFILE     : natural := 16;
    constant C_CTL_TDC_START_OFFSET    : natural := 17;
    constant C_CTL_TDC_SCAN_TIMEOUT    : natural := 18;
    constant C_CTL_TDC_CAPTURE_ADJUST  : natural := 19;
    constant C_CTL_ECHO_DELAY_PROFILE  : natural := 20;
    constant C_CTL_GPX_IMAGE_INDEX     : natural := 21;
    constant C_CTL_GPX_IMAGE_DATA      : natural := 22;
    constant C_CTL_DIAG_INDEX          : natural := 23;
    constant C_CTL_DIAG_DATA           : natural := 24;
    constant C_CTL_RESERVED_FIRST      : natural := 25;
    constant C_CTL_RESERVED_LAST       : natural := 31;

    constant C_GPX_IMAGE_INDEX_MSB       : natural := 3;
    constant C_GPX_IMAGE_INDEX_LSB       : natural := 0;
    constant C_GPX_IMAGE_VIEW_ACTIVE_BIT : natural := 8;
    constant C_GPX_IMAGE_INDEX_VALID_MASK : csr_word_t := x"0000010F";

    -- CTL23 write: INDEX[7:0] plus CAPTURE W1S[8]. CTL23 read: the same
    -- selected index, BUSY[8], VALID[9], ERROR[10], SEQUENCE[31:16]. CTL24 is
    -- the last atomic 32-bit response and rejects every write.
    constant C_DIAG_INDEX_MSB       : natural := 7;
    constant C_DIAG_INDEX_LSB       : natural := 0;
    constant C_DIAG_CAPTURE_BIT     : natural := 8;
    constant C_DIAG_BUSY_BIT        : natural := 8;
    constant C_DIAG_VALID_BIT       : natural := 9;
    constant C_DIAG_ERROR_BIT       : natural := 10;
    constant C_DIAG_SEQUENCE_MSB    : natural := 31;
    constant C_DIAG_SEQUENCE_LSB    : natural := 16;
    constant C_DIAG_INDEX_WRITE_VALID_MASK : csr_word_t := x"000001FF";

    constant C_CMD_COMMIT_BIT       : natural := 0;
    constant C_CMD_CLEAR_STATUS_BIT : natural := 1;
    constant C_CMD_SOFT_RESET_BIT   : natural := 2;
    constant C_CMD_RUN_BIT          : natural := 3;
    constant C_CMD_STOP_BIT         : natural := 4;
    constant C_CMD_ARM_BIT          : natural := 5;
    constant C_CMD_DISARM_BIT       : natural := 6;
    constant C_COMMAND_VALID_MASK   : csr_word_t := x"0000007F";

    constant C_STAT_CORE_INFO         : natural := 0;
    constant C_STAT_BUILD_INFO        : natural := 1;
    constant C_STAT_TRANSACTION       : natural := 2;
    constant C_STAT_ACTIVE_VERSION    : natural := 3;
    constant C_STAT_ACTIVE_SOURCE_BASE : natural := 4;
    constant C_STAT_DERIVED_GEOMETRY  : natural := 23;
    constant C_STAT_DERIVED_FACE      : natural := 24;
    constant C_STAT_FACE_BOUNDS_0     : natural := 25;
    constant C_STAT_FACE_BOUNDS_1     : natural := 26;
    constant C_STAT_FACE_BOUNDS_2     : natural := 27;
    constant C_STAT_FACE_BOUNDS_3     : natural := 28;
    constant C_STAT_FACE_BOUNDS_4     : natural := 29;
    constant C_STAT_CAPTURE_TDC_CLKS  : natural := 30;
    constant C_STAT_DERIVED_MASKS     : natural := 31;

    constant C_TXN_BUSY_BIT              : natural := 0;
    constant C_TXN_DONE_STICKY_BIT       : natural := 1;
    constant C_TXN_SUCCESS_STICKY_BIT    : natural := 2;
    constant C_TXN_ERROR_STICKY_BIT      : natural := 3;
    constant C_TXN_REJECTED_STICKY_BIT   : natural := 4;
    constant C_TXN_RECOVERY_REQUIRED_BIT : natural := 5;
    constant C_TXN_ACTIVE_VALID_BIT      : natural := 6;
    constant C_TXN_ACCESS_ERROR_BIT      : natural := 7;
    constant C_TXN_SHADOW_DIRTY_BIT      : natural := 8;

    -- STAT3 keeps ACTIVE_VERSION in [15:0]. The upper bits are synchronized
    -- read-only operation state; they are not a second state owner.
    constant C_OP_RUNNING_BIT              : natural := 16;
    constant C_OP_ARMED_BIT                : natural := 17;
    constant C_OP_EXTERNAL_PERMIT_BIT      : natural := 18;
    constant C_OP_CONFIG_READY_BIT         : natural := 19;
    constant C_OP_PROCESSING_ENABLE_BIT    : natural := 20;
    constant C_OP_SCHEDULER_ENABLE_BIT     : natural := 21;
    constant C_OP_PHYSICAL_FIRE_ENABLE_BIT : natural := 22;
    constant C_OP_SIMULATION_ENABLE_BIT    : natural := 23;
    constant C_OP_COMMAND_READY_BIT        : natural := 24;
    constant C_OP_COMMAND_BUSY_BIT         : natural := 25;

    constant C_IRQ_COMMIT_SUCCESS   : natural := 0;
    constant C_IRQ_COMMIT_ERROR     : natural := 1;
    constant C_IRQ_COMMIT_REJECTED  : natural := 2;
    constant C_IRQ_RECOVERY_REQUIRED : natural := 3;
    constant C_IRQ_ACCESS_ERROR     : natural := 4;
    constant C_IRQ_PROCESSING_WARNING : natural := 5;
    constant C_IRQ_LASER_TIMEOUT      : natural := 6;
    constant C_IRQ_ECHO_DIAGNOSTIC    : natural := 7;
    constant C_IRQ_GPX_TRANSPORT      : natural := 8;
    constant C_IRQ_GPX_DATA           : natural := 9;
    constant C_LIDAR_IRQ_SOURCES    : positive := 10;

    function fn_ctl_byte_offset(index : natural) return natural;
    function fn_stat_byte_offset(index : natural) return natural;
    function fn_irq_byte_offset(index : natural) return natural;
    function fn_decode_mode_bits(mode : decode_mode_t)
        return std_logic_vector;
    function fn_direction_bit(direction : direction_t) return std_logic;
    function fn_pack_runtime_config(
        config : lidar_runtime_config_t
    ) return csr_word_array_t;
    function fn_unpack_runtime_config(
        words : csr_word_array_t
    ) return lidar_runtime_config_t;
    function fn_ctl_word_encoding_valid(
        index : natural;
        value : csr_word_t
    ) return boolean;

end package lidar_csr_map_pkg;

package body lidar_csr_map_pkg is

    function fn_ctl_byte_offset(index : natural) return natural is
    begin
        return 4 * index;
    end function fn_ctl_byte_offset;

    function fn_stat_byte_offset(index : natural) return natural is
    begin
        return 4 * (C_LIDAR_CTL_COUNT + index);
    end function fn_stat_byte_offset;

    function fn_irq_byte_offset(index : natural) return natural is
    begin
        return 4 * (C_LIDAR_CTL_COUNT + C_LIDAR_STAT_COUNT + index);
    end function fn_irq_byte_offset;

    function fn_decode_mode_bits(mode : decode_mode_t)
        return std_logic_vector is
    begin
        case mode is
            when DECODE_X1 => return "00";
            when DECODE_X2 => return "01";
            when DECODE_X4 => return "10";
        end case;
    end function fn_decode_mode_bits;

    function fn_direction_bit(direction : direction_t) return std_logic is
    begin
        case direction is
            when DIRECTION_CW  => return '0';
            when DIRECTION_CCW => return '1';
        end case;
    end function fn_direction_bit;

    function fn_pack_runtime_config(
        config : lidar_runtime_config_t
    ) return csr_word_array_t is
        variable result : csr_word_array_t := (others => (others => '0'));
    begin
        result(C_CTL_MOTOR_PROFILE)(15 downto 0) :=
            std_logic_vector(config.motor.cpr);
        result(C_CTL_MOTOR_PROFILE)(17 downto 16) :=
            fn_decode_mode_bits(config.motor.decode_mode);
        result(C_CTL_MOTOR_PROFILE)(18) :=
            fn_direction_bit(config.motor.direction);
        result(C_CTL_MOTOR_PROFILE)(19) := config.motor.z_early;
        result(C_CTL_MOTOR_PROFILE)(20) := config.motor.simulation_mode;

        result(C_CTL_VIRTUAL_TICKS_LO) :=
            std_logic_vector(config.motor.virtual_ticks_lo);
        result(C_CTL_VIRTUAL_HI_COUNT)(14 downto 0) :=
            std_logic_vector(config.motor.virtual_hi_count);
        result(C_CTL_Z_PROFILE)(14 downto 0) :=
            std_logic_vector(config.motor.z_offset);
        result(C_CTL_Z_PROFILE)(30 downto 16) :=
            std_logic_vector(config.motor.z_width);

        for face_index in 0 to C_MAX_FACES - 1 loop
            result(C_CTL_FACE_CENTER_0 + face_index)(14 downto 0) :=
                std_logic_vector(config.mirror.face_centers(face_index));
        end loop;
        result(C_CTL_FACE_PROFILE)(14 downto 0) :=
            std_logic_vector(config.mirror.common_half_width);
        result(C_CTL_FACE_PROFILE)(20 downto 16) :=
            config.laser.face_enable_mask;

        result(C_CTL_LASER_FIRE_PROFILE)(15 downto 0) :=
            std_logic_vector(config.laser.fire_width_5ns_ticks);
        result(C_CTL_LASER_FIRE_PROFILE)(31 downto 16) :=
            std_logic_vector(config.laser.fire_done_timeout_5ns_ticks);
        result(C_CTL_TARGET_RANGE) :=
            std_logic_vector(config.laser.target_range_window_5ns);
        result(C_CTL_TDC_PULSE_WIDTHS)(15 downto 0) :=
            std_logic_vector(config.laser.start_width_5ns_ticks);
        result(C_CTL_TDC_PULSE_WIDTHS)(31 downto 16) :=
            std_logic_vector(config.laser.stop_width_5ns_ticks);
        result(C_CTL_SIM_START_DELAY) :=
            std_logic_vector(config.laser.simulation_start_delay_5ns);
        result(C_CTL_SHOT_INTERVAL)(29 downto 0) :=
            std_logic_vector(config.laser.optical_shot_interval_udeg);

        result(C_CTL_TDC_BUS_PROFILE)(5 downto 0) :=
            std_logic_vector(config.tdc.bus_clk_div);
        result(C_CTL_TDC_BUS_PROFILE)(11 downto 6) :=
            std_logic_vector(config.tdc.bus_ticks);
        result(C_CTL_TDC_BUS_PROFILE)(15 downto 12) :=
            config.tdc.active_chip_mask;
        result(C_CTL_TDC_BUS_PROFILE)(16) := config.tdc.falling_enable;
        result(C_CTL_TDC_BUS_PROFILE)(19 downto 17) :=
            std_logic_vector(config.tdc.max_hits_per_stop);
        result(C_CTL_TDC_START_OFFSET)(17 downto 0) :=
            std_logic_vector(config.tdc.start_offset);
        result(C_CTL_TDC_SCAN_TIMEOUT) :=
            std_logic_vector(config.tdc.scan_timeout_5ns);
        result(C_CTL_TDC_CAPTURE_ADJUST)(16 downto 0) :=
            std_logic_vector(config.tdc.capture_adjust_5ns);
        result(C_CTL_ECHO_DELAY_PROFILE)(15 downto 0) :=
            std_logic_vector(config.echo.channel_0_delay_5ns);
        result(C_CTL_ECHO_DELAY_PROFILE)(31 downto 16) :=
            std_logic_vector(config.echo.channel_step_5ns);

        return result;
    end function fn_pack_runtime_config;

    function fn_unpack_runtime_config(
        words : csr_word_array_t
    ) return lidar_runtime_config_t is
        variable result : lidar_runtime_config_t := C_DEFAULT_RUNTIME_CONFIG;
    begin
        result.motor.cpr := unsigned(
            words(C_CTL_MOTOR_PROFILE)(15 downto 0));
        case words(C_CTL_MOTOR_PROFILE)(17 downto 16) is
            when "00"   => result.motor.decode_mode := DECODE_X1;
            when "01"   => result.motor.decode_mode := DECODE_X2;
            when others => result.motor.decode_mode := DECODE_X4;
        end case;
        if words(C_CTL_MOTOR_PROFILE)(18) = '1' then
            result.motor.direction := DIRECTION_CCW;
        else
            result.motor.direction := DIRECTION_CW;
        end if;
        result.motor.z_early := words(C_CTL_MOTOR_PROFILE)(19);
        result.motor.simulation_mode := words(C_CTL_MOTOR_PROFILE)(20);
        result.motor.virtual_ticks_lo := unsigned(
            words(C_CTL_VIRTUAL_TICKS_LO));
        result.motor.virtual_hi_count := unsigned(
            words(C_CTL_VIRTUAL_HI_COUNT)(14 downto 0));
        result.motor.z_offset := unsigned(
            words(C_CTL_Z_PROFILE)(14 downto 0));
        result.motor.z_width := unsigned(
            words(C_CTL_Z_PROFILE)(30 downto 16));

        for face_index in 0 to C_MAX_FACES - 1 loop
            result.mirror.face_centers(face_index) := unsigned(
                words(C_CTL_FACE_CENTER_0 + face_index)(14 downto 0));
        end loop;
        result.mirror.common_half_width := unsigned(
            words(C_CTL_FACE_PROFILE)(14 downto 0));
        result.laser.face_enable_mask :=
            words(C_CTL_FACE_PROFILE)(20 downto 16);

        result.laser.fire_width_5ns_ticks := unsigned(
            words(C_CTL_LASER_FIRE_PROFILE)(15 downto 0));
        result.laser.fire_done_timeout_5ns_ticks := unsigned(
            words(C_CTL_LASER_FIRE_PROFILE)(31 downto 16));
        result.laser.target_range_window_5ns := unsigned(
            words(C_CTL_TARGET_RANGE));
        result.laser.start_width_5ns_ticks := unsigned(
            words(C_CTL_TDC_PULSE_WIDTHS)(15 downto 0));
        result.laser.stop_width_5ns_ticks := unsigned(
            words(C_CTL_TDC_PULSE_WIDTHS)(31 downto 16));
        result.laser.simulation_start_delay_5ns := unsigned(
            words(C_CTL_SIM_START_DELAY));
        result.laser.optical_shot_interval_udeg := unsigned(
            words(C_CTL_SHOT_INTERVAL)(29 downto 0));

        result.tdc.bus_clk_div := unsigned(
            words(C_CTL_TDC_BUS_PROFILE)(5 downto 0));
        result.tdc.bus_ticks := unsigned(
            words(C_CTL_TDC_BUS_PROFILE)(11 downto 6));
        result.tdc.active_chip_mask :=
            words(C_CTL_TDC_BUS_PROFILE)(15 downto 12);
        result.tdc.falling_enable :=
            words(C_CTL_TDC_BUS_PROFILE)(16);
        result.tdc.max_hits_per_stop := unsigned(
            words(C_CTL_TDC_BUS_PROFILE)(19 downto 17));
        result.tdc.start_offset := unsigned(
            words(C_CTL_TDC_START_OFFSET)(17 downto 0));
        result.tdc.scan_timeout_5ns := unsigned(
            words(C_CTL_TDC_SCAN_TIMEOUT));
        result.tdc.capture_adjust_5ns := signed(
            words(C_CTL_TDC_CAPTURE_ADJUST)(16 downto 0));
        result.echo.channel_0_delay_5ns := unsigned(
            words(C_CTL_ECHO_DELAY_PROFILE)(15 downto 0));
        result.echo.channel_step_5ns := unsigned(
            words(C_CTL_ECHO_DELAY_PROFILE)(31 downto 16));

        return result;
    end function fn_unpack_runtime_config;

    function fn_ctl_word_encoding_valid(
        index : natural;
        value : csr_word_t
    ) return boolean is
        variable zero_word : csr_word_t := (others => '0');
    begin
        case index is
            when C_CTL_MOTOR_PROFILE =>
                return value(17 downto 16) /= "11"
                    and (value(20) = '0' or value(20) = '1')
                    and value(31 downto 21) = zero_word(31 downto 21);
            when C_CTL_VIRTUAL_TICKS_LO |
                 C_CTL_LASER_FIRE_PROFILE |
                 C_CTL_TARGET_RANGE |
                 C_CTL_TDC_PULSE_WIDTHS |
                 C_CTL_SIM_START_DELAY |
                 C_CTL_TDC_SCAN_TIMEOUT =>
                return true;
            when C_CTL_VIRTUAL_HI_COUNT =>
                return value(31 downto 15) = zero_word(31 downto 15);
            when C_CTL_Z_PROFILE =>
                return value(31) = '0' and value(15) = '0';
            when C_CTL_FACE_CENTER_0 to C_CTL_FACE_CENTER_4 =>
                return value(31 downto 15) = zero_word(31 downto 15);
            when C_CTL_FACE_PROFILE =>
                return value(31 downto 21) = zero_word(31 downto 21)
                    and value(15) = '0';
            when C_CTL_SHOT_INTERVAL =>
                return value(31 downto 30) = "00";
            when C_CTL_TDC_BUS_PROFILE =>
                return value(31 downto 20) = zero_word(31 downto 20);
            when C_CTL_TDC_START_OFFSET =>
                return value(31 downto 18) = zero_word(31 downto 18);
            when C_CTL_TDC_CAPTURE_ADJUST =>
                return value(31 downto 17) = zero_word(31 downto 17);
            when C_CTL_ECHO_DELAY_PROFILE =>
                return true;
            when C_CTL_GPX_IMAGE_INDEX =>
                return (value and not C_GPX_IMAGE_INDEX_VALID_MASK) =
                    zero_word;
            when C_CTL_GPX_IMAGE_DATA =>
                -- The external GPX bus is 28 bits. Reject rather than
                -- silently truncating image data above bit 27.
                return value(31 downto 28) = "0000";
            when C_CTL_DIAG_INDEX =>
                return (value and not C_DIAG_INDEX_WRITE_VALID_MASK) =
                    zero_word;
            when C_CTL_DIAG_DATA =>
                return false;
            when others =>
                return false;
        end case;
    end function fn_ctl_word_encoding_valid;

end package body lidar_csr_map_pkg;
