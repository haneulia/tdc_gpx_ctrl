library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_unified_csr_pkg.all;

entity tb_lidar_unified_csr_pkg is
end entity tb_lidar_unified_csr_pkg;

architecture sim of tb_lidar_unified_csr_pkg is
begin
    p_contract : process
    begin
        assert C_UCSR_CTL_COUNT = 32 and C_UCSR_STAT_COUNT = 32
            report "unified CSR must retain 32 CTL and 32 STAT slots"
            severity failure;
        assert C_UCSR_INTR_COUNT = 4 and C_UCSR_INTR_SOURCE_COUNT = 32
            report "unified CSR interrupt geometry changed"
            severity failure;

        assert C_UCSR_VERSION_WORD(31 downto 24) = x"4C"
            and C_UCSR_CAPABILITY_WORD(C_SYS_CAP_MOTOR_PRESENT_BIT) = '1'
            and C_UCSR_CAPABILITY_WORD(C_SYS_CAP_LASER_PRESENT_BIT) = '1'
            and C_UCSR_CAPABILITY_WORD(C_SYS_CAP_ECHO_PRESENT_BIT) = '1'
            and C_UCSR_CAPABILITY_WORD(C_SYS_CAP_TDC_PRESENT_BIT) = '1'
            and unsigned(C_UCSR_CAPABILITY_WORD(
                C_SYS_CAP_ACTIVE_CTL_HI downto C_SYS_CAP_ACTIVE_CTL_LO)) =
                C_UCSR_ACTIVE_CTL_COUNT
            and unsigned(C_UCSR_CAPABILITY_WORD(
                C_SYS_CAP_ACTIVE_STAT_HI downto C_SYS_CAP_ACTIVE_STAT_LO)) =
                C_UCSR_ACTIVE_STAT_COUNT
            and unsigned(C_UCSR_CAPABILITY_WORD(
                C_SYS_CAP_IRQ_COUNT_HI downto C_SYS_CAP_IRQ_COUNT_LO)) =
                C_UCSR_INTR_SOURCE_COUNT
            report "unified identity/capability word changed"
            severity failure;

        assert fn_ctl_byte_offset(0) = 16#000#
            and fn_ctl_byte_offset(31) = 16#07C#
            and fn_stat_byte_offset(0) = 16#080#
            and fn_stat_byte_offset(31) = 16#0FC#
            and fn_intr_byte_offset(0) = 16#100#
            and fn_intr_byte_offset(3) = C_UCSR_LAST_ADDR
            report "unified CSR byte-address geometry changed"
            severity failure;

        assert C_CTL_SYS_CTRL = 0
            and C_CTL_MOTOR_CFG = 2
            and C_CTL_LASER_FIRE_CFG = 8
            and C_CTL_ECHO_DELAY_CMD = 15
            and C_CTL_TDC_BUS_TIMING = 17
            and C_CTL_TDC_AUX_CMD + 1 = C_CTL_RESERVED_FIRST
            report "unified CTL ownership ranges overlap or contain a gap"
            severity failure;

        assert C_STAT_SYS_VERSION = 0
            and C_STAT_MOTOR_STATUS = 6
            and C_STAT_LASER_STATUS = 12
            and C_STAT_ECHO_RISE_MASK = 19
            and C_STAT_TDC_CHIP0_RESULT = 23
            and C_STAT_TDC_PIPELINE_STATUS = 27
            and C_STAT_TDC_STATUS_EXT2 + 1 =
                C_STAT_TDC_IMAGE_SELECTED_DATA
            and C_STAT_TDC_IMAGE_SELECTED_DATA + 1 =
                C_STAT_SYS_ADAPTER_STATE
            and C_STAT_SYS_ADAPTER_STATE = C_UCSR_STAT_COUNT - 1
            report "unified STAT ownership ranges overlap or contain a gap"
            severity failure;

        assert C_IRQ_SYS_FIRST = 0
            and C_IRQ_SYS_LAST + 1 = C_IRQ_MOTOR_FIRST
            and C_IRQ_MOTOR_LAST + 1 = C_IRQ_LASER_FIRST
            and C_IRQ_LASER_LAST + 1 = C_IRQ_ECHO_FIRST
            and C_IRQ_ECHO_LAST + 1 = C_IRQ_TDC_FIRST
            and C_IRQ_TDC_LAST + 1 = C_IRQ_RESERVED_FIRST
            and C_IRQ_RESERVED_LAST = C_UCSR_INTR_SOURCE_COUNT - 1
            report "unified interrupt-source ownership ranges overlap"
            severity failure;

        assert C_UCSR_ACTIVE_WORD_COUNT = 62
            report "unified active-register count changed"
            severity failure;

        assert C_SYS_CTRL_RESET_EPOCH_HI - C_SYS_CTRL_RESET_EPOCH_LO + 1 = 8
            and C_SYS_CFG_EPOCH_HI - C_SYS_CFG_EPOCH_LO + 1 = 8
            report "system transaction epochs must remain 8 bits"
            severity failure;

        assert C_SYS_CONFIG_CFG_REQUEST_HI
                   < C_SYS_CONFIG_RESET_REQUEST_LO
            and C_SYS_CONFIG_RESET_REQUEST_HI
                   < C_SYS_CONFIG_LASER_CFG_ACCEPTED_LO
            and C_SYS_CONFIG_LASER_CFG_ACCEPTED_HI
                   < C_SYS_CONFIG_TDC_CFG_ACCEPTED_LO
            and C_SYS_CONFIG_TDC_CFG_ACCEPTED_HI = 31
            and C_SYS_STATE_ANY_REJECT_BIT < 32
            report "system transaction status packing changed"
            severity failure;

        assert C_MOTOR_CFG_CPR_HI < C_MOTOR_CFG_DIR_BIT
            and C_MOTOR_CFG_DIR_BIT < C_MOTOR_CFG_DEC_MODE_LO
            and C_MOTOR_CFG_DEC_MODE_HI < C_MOTOR_CFG_Z_EARLY_BIT
            and C_MOTOR_CFG_Z_EARLY_BIT < C_MOTOR_CFG_VALID_HOLD_LO
            and C_MOTOR_CFG_VALID_HOLD_HI < 32
            report "MOTOR_CFG fields overlap or exceed one word"
            severity failure;

        assert C_MOTOR_SCHED_HI_COUNT_HI < C_MOTOR_SCHED_PHYS_LAT_LO
            and C_MOTOR_SCHED_PHYS_LAT_HI < C_MOTOR_SCHED_VIRT_LAT_LO
            and C_MOTOR_SCHED_VIRT_LAT_HI < 32
            report "MOTOR_SCHED_LATENCY fields overlap or exceed one word"
            severity failure;

        assert C_MOTOR_Z_OFFSET_HI < C_MOTOR_Z_WIDTH_LO
            and C_MOTOR_Z_WIDTH_HI < 32
            and C_MOTOR_FACE_CENTER_HI < C_MOTOR_FACE_HALF_LO
            and C_MOTOR_FACE_HALF_HI < C_MOTOR_FACE_VALID_BIT
            and C_MOTOR_FACE_VALID_BIT < 32
            report "Motor 15-bit pair packing changed"
            severity failure;

        assert C_MOTOR_FACE_WRITE_INDEX_HI < C_MOTOR_FACE_READ_INDEX_LO
            and C_MOTOR_FACE_READ_INDEX_HI < C_MOTOR_FACE_WRITE_EPOCH_LO
            and C_MOTOR_FACE_WRITE_EPOCH_HI < 32
            report "MOTOR_FACE_INDEX fields overlap"
            severity failure;

        assert C_MOTOR_CFG_STATUS_CFG_EPOCH_HI
                   < C_MOTOR_CFG_STATUS_FACE_EPOCH_LO
            and C_MOTOR_CFG_STATUS_FACE_EPOCH_HI
                   < C_MOTOR_CFG_STATUS_READ_INDEX_LO
            and C_MOTOR_CFG_STATUS_READ_INDEX_HI
                   < C_MOTOR_CFG_STATUS_GEOM_VALID_BIT
            and C_MOTOR_CFG_STATUS_VALID_BIT
                   < C_MOTOR_CFG_STATUS_RESET_EPOCH_LO
            and C_MOTOR_CFG_STATUS_RESET_EPOCH_HI = 31
            report "MOTOR_CFG_STATUS fields overlap or leave the word"
            severity failure;

        assert C_ECHO_DELAY_INDEX_HI < C_ECHO_DELAY_WRITE_TOGGLE_BIT
            and C_ECHO_DELAY_WRITE_TOGGLE_BIT
                   < C_ECHO_DELAY_APPLY_TOGGLE_BIT
            and C_ECHO_DELAY_DATA_HI < 32
            report "Echo indexed delay command fields changed"
            severity failure;

        assert C_IRQ_MOTOR_ACTIVE_ENTER = C_IRQ_MOTOR_FIRST
            and C_IRQ_MOTOR_DIAGNOSTIC = C_IRQ_MOTOR_LAST
            and C_IRQ_LASER_BLOCKING_CFG = C_IRQ_LASER_FIRST
            and C_IRQ_LASER_FRAME_OVERFLOW <= C_IRQ_LASER_LAST
            report "Stage 3 interrupt ownership changed"
            severity failure;

        assert C_IRQ_ECHO_DIAGNOSTIC = C_IRQ_ECHO_FIRST
            and C_IRQ_ECHO_CMD_REJECT = C_IRQ_ECHO_FIRST + 1
            and C_IRQ_ECHO_CMD_REJECT < C_IRQ_ECHO_LAST
            report "Echo interrupt ownership changed"
            severity failure;

        assert C_TDC_IMAGE_INDEX_HI < C_TDC_IMAGE_WRITE_EPOCH_LO
            and C_TDC_IMAGE_WRITE_EPOCH_HI < 32
            and C_TDC_CMD_OPCODE_HI < C_TDC_CMD_EPOCH_LO
            and C_TDC_CMD_EPOCH_HI < 32
            report "TDC indexed image/command fields overlap"
            severity failure;

        assert C_TDC_CMD_NONE = 0
            and C_TDC_CMD_START = 1
            and C_TDC_CMD_STOP = 2
            and C_TDC_CMD_FORCE_REINIT = 3
            and C_TDC_CMD_ERROR_CLEAR = 4
            and C_TDC_CMD_REG_READ = 5
            and C_TDC_CMD_REG_WRITE = 6
            report "TDC serialized command opcodes changed"
            severity failure;

        assert C_TDC_STATUS_CMD_EPOCH_HI < C_TDC_STATUS_IMAGE_EPOCH_LO
            and C_TDC_STATUS_IMAGE_EPOCH_HI = 31
            and C_IRQ_TDC_REG_DONE = C_IRQ_TDC_FIRST
            and C_IRQ_TDC_COMMAND_REJECT = C_IRQ_TDC_LAST
            report "TDC status acknowledgement or IRQ ownership changed"
            severity failure;

        report "UNIFIED_CSR_CONTRACT_PASS" severity note;
        wait;
    end process p_contract;
end architecture sim;
