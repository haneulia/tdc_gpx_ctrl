-- =============================================================================
-- LiDAR unified CSR address contract
--
-- The software-visible map is deliberately fixed at 32 CTL, 32 STAT, and
-- four interrupt registers.  Individual IP adapters may use fewer slots, but
-- adding a feature must not move an existing register.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;

package lidar_unified_csr_pkg is

    constant C_UCSR_WORD_BYTES       : positive := 4;
    constant C_UCSR_CTL_COUNT        : positive := 32;
    constant C_UCSR_STAT_COUNT       : positive := 32;
    constant C_UCSR_INTR_COUNT       : positive := 4;
    constant C_UCSR_INTR_SOURCE_COUNT : positive := 32;

    constant C_UCSR_CTL_BASE  : natural := 16#000#;
    constant C_UCSR_STAT_BASE : natural := 16#080#;
    constant C_UCSR_INTR_BASE : natural := 16#100#;
    constant C_UCSR_LAST_ADDR : natural := 16#10C#;

    -- CTL0..1: system ownership and transaction delimiters.
    constant C_CTL_SYS_CTRL      : natural := 0;
    constant C_CTL_SYS_CFG_APPLY : natural := 1;

    -- CTL2..7: Motor Decoder and embedded Virtual Encoder.
    constant C_CTL_MOTOR_CFG          : natural := 2;
    constant C_CTL_MOTOR_TICKS_LO     : natural := 3;
    constant C_CTL_MOTOR_SCHED_LATENCY : natural := 4;
    constant C_CTL_MOTOR_Z_PARAM      : natural := 5;
    constant C_CTL_MOTOR_FACE_INDEX   : natural := 6;
    constant C_CTL_MOTOR_FACE_GEOMETRY : natural := 7;

    -- CTL8..14: Laser Controller.
    constant C_CTL_LASER_FIRE_CFG  : natural := 8;
    constant C_CTL_LASER_ROUNDTRIP : natural := 9;
    constant C_CTL_LASER_TDC_WIDTH : natural := 10;
    constant C_CTL_LASER_SIM_DELAY : natural := 11;
    constant C_CTL_LASER_SCHED0    : natural := 12;
    constant C_CTL_LASER_SCHED1    : natural := 13;
    constant C_CTL_LASER_SCHED2    : natural := 14;

    -- CTL15..16: Echo Receiver indexed simulation-delay window.
    constant C_CTL_ECHO_DELAY_CMD  : natural := 15;
    constant C_CTL_ECHO_DELAY_DATA : natural := 16;

    -- CTL17..25: TDC-GPX chip and processing pipeline.
    constant C_CTL_TDC_BUS_TIMING   : natural := 17;
    constant C_CTL_TDC_START_OFFSET : natural := 18;
    constant C_CTL_TDC_CFG_REG7     : natural := 19;
    constant C_CTL_TDC_IMAGE_CMD    : natural := 20;
    constant C_CTL_TDC_IMAGE_DATA   : natural := 21;
    constant C_CTL_TDC_SCAN_CFG     : natural := 22;
    constant C_CTL_TDC_PIPELINE_MAIN : natural := 23;
    constant C_CTL_TDC_RANGE_COLS   : natural := 24;
    constant C_CTL_TDC_AUX_CMD      : natural := 25;

    constant C_CTL_RESERVED_FIRST : natural := 26;
    constant C_CTL_RESERVED_LAST  : natural := 31;
    constant C_UCSR_ACTIVE_CTL_COUNT : positive := 26;

    -- STAT0..5: system identity, capability, and derived geometry.
    constant C_STAT_SYS_VERSION    : natural := 0;
    constant C_STAT_SYS_CAPABILITY : natural := 1;
    constant C_STAT_SYS_CONFIG     : natural := 2;
    constant C_STAT_TDC_MAX_ROWS   : natural := 3;
    constant C_STAT_TDC_CELL_SIZE  : natural := 4;
    constant C_STAT_TDC_MAX_HSIZE  : natural := 5;

    -- STAT6..11: Motor Decoder.
    constant C_STAT_MOTOR_STATUS       : natural := 6;
    constant C_STAT_MOTOR_FACE_GEOMETRY : natural := 7;
    constant C_STAT_MOTOR_CFG_STATUS   : natural := 8;
    constant C_STAT_MOTOR_QUAD_INVALID : natural := 9;
    constant C_STAT_MOTOR_AXIS_DROP    : natural := 10;
    constant C_STAT_MOTOR_REV_PERIOD   : natural := 11;

    -- STAT12..18: Laser Controller.
    constant C_STAT_LASER_STATUS          : natural := 12;
    constant C_STAT_LASER_ENCODER_TO_FIRE : natural := 13;
    constant C_STAT_LASER_FIRE_DONE       : natural := 14;
    constant C_STAT_LASER_FIRE_TO_TDC     : natural := 15;
    constant C_STAT_LASER_METRIC_FLAGS    : natural := 16;
    constant C_STAT_LASER_FRAME_COUNT     : natural := 17;
    constant C_STAT_LASER_TIMEOUT_COUNT   : natural := 18;

    -- STAT19..22: Echo Receiver.  The OR mask and the transient 32-channel
    -- countdown mask are intentionally absent; see the migration contract.
    constant C_STAT_ECHO_RISE_MASK      : natural := 19;
    constant C_STAT_ECHO_FALL_MASK      : natural := 20;
    constant C_STAT_ECHO_STATUS         : natural := 21;
    constant C_STAT_ECHO_DELAY_READBACK : natural := 22;

    -- STAT23..26: direct GPX register-read results.
    constant C_STAT_TDC_CHIP0_RESULT : natural := 23;
    constant C_STAT_TDC_CHIP1_RESULT : natural := 24;
    constant C_STAT_TDC_CHIP2_RESULT : natural := 25;
    constant C_STAT_TDC_CHIP3_RESULT : natural := 26;

    -- STAT27..29: live/sticky GPX pipeline diagnostics.
    constant C_STAT_TDC_PIPELINE_STATUS : natural := 27;
    constant C_STAT_TDC_STATUS_EXT      : natural := 28;
    constant C_STAT_TDC_STATUS_EXT2     : natural := 29;

    constant C_STAT_RESERVED_FIRST : natural := 30;
    constant C_STAT_RESERVED_LAST  : natural := 31;
    constant C_UCSR_ACTIVE_STAT_COUNT : positive := 30;

    -- Interrupt source ownership is reserved by group in phase 1.  Individual
    -- source meanings are frozen when each adapter reaches its unit-test gate.
    constant C_IRQ_SYS_FIRST      : natural := 0;
    constant C_IRQ_SYS_LAST       : natural := 3;
    constant C_IRQ_MOTOR_FIRST    : natural := 4;
    constant C_IRQ_MOTOR_LAST     : natural := 7;
    constant C_IRQ_LASER_FIRST    : natural := 8;
    constant C_IRQ_LASER_LAST     : natural := 15;
    constant C_IRQ_ECHO_FIRST     : natural := 16;
    constant C_IRQ_ECHO_LAST      : natural := 20;
    constant C_IRQ_TDC_FIRST      : natural := 21;
    constant C_IRQ_TDC_LAST       : natural := 27;
    constant C_IRQ_RESERVED_FIRST : natural := 28;
    constant C_IRQ_RESERVED_LAST  : natural := 31;

    constant C_UCSR_ACTIVE_WORD_COUNT : positive :=
        C_UCSR_ACTIVE_CTL_COUNT + C_UCSR_ACTIVE_STAT_COUNT
        + C_UCSR_INTR_COUNT;

    -- ---------------------------------------------------------------------
    -- Shared control transaction fields
    -- ---------------------------------------------------------------------
    -- SYS_CTRL carries live mode/enable bits plus an 8-bit reset epoch.
    -- Software requests reset by writing a value different from the last
    -- accepted epoch. A level bit is deliberately not used for reset.
    constant C_SYS_CTRL_MOTOR_SIM_EN_BIT   : natural := 0;
    constant C_SYS_CTRL_LASER_EN_BIT       : natural := 1;
    constant C_SYS_CTRL_LASER_STREAM_EN_BIT : natural := 2;
    constant C_SYS_CTRL_ECHO_SIM_EN_BIT    : natural := 3;
    constant C_SYS_CTRL_RESET_EPOCH_LO     : natural := 8;
    constant C_SYS_CTRL_RESET_EPOCH_HI     : natural := 15;

    -- A changed configuration epoch captures every adapter-owned staging
    -- word as one transaction. Each adapter returns its accepted epoch.
    constant C_SYS_CFG_EPOCH_LO : natural := 0;
    constant C_SYS_CFG_EPOCH_HI : natural := 7;

    -- ---------------------------------------------------------------------
    -- Motor Decoder CTL2..7 field contract
    -- ---------------------------------------------------------------------
    constant C_MOTOR_CFG_CPR_LO        : natural := 0;
    constant C_MOTOR_CFG_CPR_HI        : natural := 15;
    constant C_MOTOR_CFG_DIR_BIT       : natural := 16;
    constant C_MOTOR_CFG_DEC_MODE_LO   : natural := 17;
    constant C_MOTOR_CFG_DEC_MODE_HI   : natural := 18;
    constant C_MOTOR_CFG_Z_EARLY_BIT   : natural := 19;
    constant C_MOTOR_CFG_VALID_HOLD_LO : natural := 20;
    constant C_MOTOR_CFG_VALID_HOLD_HI : natural := 27;

    constant C_MOTOR_SCHED_HI_COUNT_LO : natural := 0;
    constant C_MOTOR_SCHED_HI_COUNT_HI : natural := 14;
    constant C_MOTOR_SCHED_PHYS_LAT_LO : natural := 15;
    constant C_MOTOR_SCHED_PHYS_LAT_HI : natural := 20;
    constant C_MOTOR_SCHED_VIRT_LAT_LO : natural := 21;
    constant C_MOTOR_SCHED_VIRT_LAT_HI : natural := 26;

    constant C_MOTOR_Z_OFFSET_LO : natural := 0;
    constant C_MOTOR_Z_OFFSET_HI : natural := 14;
    constant C_MOTOR_Z_WIDTH_LO  : natural := 15;
    constant C_MOTOR_Z_WIDTH_HI  : natural := 29;

    constant C_MOTOR_FACE_WRITE_INDEX_LO : natural := 0;
    constant C_MOTOR_FACE_WRITE_INDEX_HI : natural := 2;
    constant C_MOTOR_FACE_READ_INDEX_LO  : natural := 3;
    constant C_MOTOR_FACE_READ_INDEX_HI  : natural := 5;
    constant C_MOTOR_FACE_WRITE_EPOCH_LO : natural := 8;
    constant C_MOTOR_FACE_WRITE_EPOCH_HI : natural := 15;

    -- CTL7 and STAT7 use the same compact 30-bit geometry representation.
    constant C_MOTOR_FACE_CENTER_LO : natural := 0;
    constant C_MOTOR_FACE_CENTER_HI : natural := 14;
    constant C_MOTOR_FACE_HALF_LO   : natural := 15;
    constant C_MOTOR_FACE_HALF_HI   : natural := 29;
    constant C_MOTOR_FACE_VALID_BIT : natural := 30;

    -- Motor status words. MOTOR_STATUS is live operating state;
    -- MOTOR_CFG_STATUS owns transaction acknowledgement and selection state.
    constant C_MOTOR_STATUS_CUR_FACE_LO       : natural := 0;
    constant C_MOTOR_STATUS_CUR_FACE_HI       : natural := 2;
    constant C_MOTOR_STATUS_ACTIVE_BIT        : natural := 3;
    constant C_MOTOR_STATUS_SIM_RUNNING_BIT   : natural := 4;
    constant C_MOTOR_STATUS_CFG_BUSY_BIT      : natural := 5;
    constant C_MOTOR_STATUS_Z_FAULT_LO        : natural := 6;
    constant C_MOTOR_STATUS_Z_FAULT_HI        : natural := 7;
    constant C_MOTOR_STATUS_POS_OVERFLOW_BIT  : natural := 8;
    constant C_MOTOR_STATUS_QUAD_STICKY_BIT   : natural := 9;
    constant C_MOTOR_STATUS_AXIS_DROP_BIT     : natural := 10;
    constant C_MOTOR_STATUS_DEC_MODE_LO       : natural := 11;
    constant C_MOTOR_STATUS_DEC_MODE_HI       : natural := 12;
    constant C_MOTOR_STATUS_N_FACES_LO        : natural := 13;
    constant C_MOTOR_STATUS_N_FACES_HI        : natural := 15;
    constant C_MOTOR_STATUS_DIR_BIT            : natural := 16;

    constant C_MOTOR_CFG_STATUS_CFG_EPOCH_LO   : natural := 0;
    constant C_MOTOR_CFG_STATUS_CFG_EPOCH_HI   : natural := 7;
    constant C_MOTOR_CFG_STATUS_FACE_EPOCH_LO  : natural := 8;
    constant C_MOTOR_CFG_STATUS_FACE_EPOCH_HI  : natural := 15;
    constant C_MOTOR_CFG_STATUS_READ_INDEX_LO  : natural := 16;
    constant C_MOTOR_CFG_STATUS_READ_INDEX_HI  : natural := 18;
    constant C_MOTOR_CFG_STATUS_GEOM_VALID_BIT : natural := 19;
    constant C_MOTOR_CFG_STATUS_BUSY_BIT       : natural := 20;
    constant C_MOTOR_CFG_STATUS_APPLY_TRACK_BIT : natural := 21;
    constant C_MOTOR_CFG_STATUS_REJECT_BIT     : natural := 22;
    constant C_MOTOR_CFG_STATUS_VALID_BIT      : natural := 23;
    constant C_MOTOR_CFG_STATUS_RESET_EPOCH_LO : natural := 24;
    constant C_MOTOR_CFG_STATUS_RESET_EPOCH_HI : natural := 31;

    -- ---------------------------------------------------------------------
    -- Echo Receiver CTL15..16 indexed command contract
    -- ---------------------------------------------------------------------
    constant C_ECHO_DELAY_INDEX_LO        : natural := 0;
    constant C_ECHO_DELAY_INDEX_HI        : natural := 4;
    constant C_ECHO_DELAY_WRITE_TOGGLE_BIT : natural := 8;
    constant C_ECHO_DELAY_APPLY_TOGGLE_BIT : natural := 9;
    constant C_ECHO_DELAY_DATA_LO         : natural := 0;
    constant C_ECHO_DELAY_DATA_HI         : natural := 15;

    -- Frozen interrupt source meanings for adapters closed in Stage 2/3.
    constant C_IRQ_MOTOR_ACTIVE_ENTER : natural := 4;
    constant C_IRQ_MOTOR_ACTIVE_EXIT  : natural := 5;
    constant C_IRQ_MOTOR_REVOLUTION   : natural := 6;
    constant C_IRQ_MOTOR_DIAGNOSTIC   : natural := 7;

    constant C_IRQ_LASER_BLOCKING_CFG    : natural := 8;
    constant C_IRQ_LASER_TIMEOUT_OVERFLOW : natural := 9;
    constant C_IRQ_LASER_FRAME_OVERFLOW   : natural := 10;

    function fn_ctl_byte_offset(index : natural) return natural;
    function fn_stat_byte_offset(index : natural) return natural;
    function fn_intr_byte_offset(index : natural) return natural;

end package lidar_unified_csr_pkg;

package body lidar_unified_csr_pkg is

    function fn_ctl_byte_offset(index : natural) return natural is
    begin
        return C_UCSR_CTL_BASE + index * C_UCSR_WORD_BYTES;
    end function;

    function fn_stat_byte_offset(index : natural) return natural is
    begin
        return C_UCSR_STAT_BASE + index * C_UCSR_WORD_BYTES;
    end function;

    function fn_intr_byte_offset(index : natural) return natural is
    begin
        return C_UCSR_INTR_BASE + index * C_UCSR_WORD_BYTES;
    end function;

end package body lidar_unified_csr_pkg;
