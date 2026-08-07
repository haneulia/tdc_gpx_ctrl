library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_csr_map_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.lidar_status_pkg.all;

-- One AXI4-Lite owner for the v2 LiDAR configuration ABI.
--
-- CTL1..20 are shadow storage. CTL21/22 are an indexed 16-entry GPX setting
-- image portal, while CTL23/24 are the read-only runtime-diagnostic portal.
-- CTL23 INDEX=11CCAAAA additionally requests one actual physical GPX read;
-- CTL24 then returns {requested_address[3:0], read_data[27:0]}. CTL0 is
-- write-one-set command space and never stores a command
-- level. Active readback is sourced only from the atomic transaction owner,
-- so software can distinguish edited and applied data.
entity lidar_csr_bank is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk    : in  std_logic;
        i_rst_n  : in  std_logic;

        s_axi_awaddr  : in  std_logic_vector(8 downto 0);
        s_axi_awprot  : in  std_logic_vector(2 downto 0);
        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic;
        s_axi_wdata   : in  std_logic_vector(31 downto 0);
        s_axi_wstrb   : in  std_logic_vector(3 downto 0);
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic;
        s_axi_bresp   : out std_logic_vector(1 downto 0);
        s_axi_bvalid  : out std_logic;
        s_axi_bready  : in  std_logic;
        s_axi_araddr  : in  std_logic_vector(8 downto 0);
        s_axi_arprot  : in  std_logic_vector(2 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;
        s_axi_rdata   : out std_logic_vector(31 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic;

        i_cfg_busy              : in  std_logic;
        i_cfg_done              : in  std_logic;
        i_cfg_commit_rejected   : in  std_logic;
        i_cfg_reject_error      : in  lidar_cfg_error_t;
        i_cfg_error             : in  lidar_cfg_error_t;
        i_cfg_recovery_required : in  std_logic;
        i_cfg_active_valid      : in  std_logic;
        i_cfg_active            : in  lidar_active_config_t;
        i_cfg_active_gpx_image  : in  gpx_register_image_t;
        i_operation_status      : in  operation_state_t;
        i_operation_command_ready : in std_logic;
        i_operation_command_busy  : in std_logic;
        i_operation_command_rejected : in std_logic;
        i_system_command_ready    : in std_logic := '1';
        i_system_command_rejected : in std_logic := '0';
        i_runtime_irq : in lidar_runtime_irq_t := C_RUNTIME_IRQ_CLEAR;

        o_diag_request_valid : out std_logic;
        i_diag_request_ready : in  std_logic := '0';
        o_diag_request_index : out lidar_diag_index_t;
        i_diag_response_valid : in  std_logic := '0';
        o_diag_response_ready : out std_logic;
        i_diag_response       : in  lidar_diag_response_t :=
            (others => '0');

        o_shadow            : out lidar_runtime_config_t;
        o_gpx_image_shadow  : out gpx_register_image_t;
        o_commit            : out std_logic;
        o_clear_status      : out std_logic;
        o_soft_reset_request : out std_logic;
        o_operation_command_valid : out std_logic;
        o_operation_command       : out operation_command_t;
        o_irq               : out std_logic
    );
end entity lidar_csr_bank;

architecture rtl of lidar_csr_bank is

    constant C_DEFAULT_SHADOW : lidar_runtime_config_t :=
        fn_default_runtime_config(G_BUILD_CONFIG);
    constant C_DEFAULT_WORDS : csr_word_array_t :=
        fn_pack_runtime_config(C_DEFAULT_SHADOW);

    signal r_shadow_words : csr_word_array_t := C_DEFAULT_WORDS;
    signal r_gpx_image_staging : gpx_register_image_t :=
        C_GPX_REGISTER_IMAGE_DEFAULT;
    signal r_gpx_image_index : natural range 0 to
        C_GPX_REGISTER_COUNT - 1 := 0;
    signal r_gpx_image_view_active : std_logic := '0';
    signal r_diag_index : lidar_diag_index_t := (others => '0');
    signal r_diag_request_valid : std_logic := '0';
    signal r_diag_busy  : std_logic := '0';
    signal r_diag_valid : std_logic := '0';
    signal r_diag_error : std_logic := '0';
    signal r_diag_data  : lidar_diag_word_t := (others => '0');
    signal r_diag_sequence : unsigned(15 downto 0) := (others => '0');
    signal w_status_words : csr_word_array_t;

    signal w_addr : std_logic_vector(8 downto 0);
    signal w_data : csr_word_t;
    signal w_strb : std_logic_vector(3 downto 0);
    signal w_we   : std_logic;
    signal r_addr : std_logic_vector(8 downto 0);
    signal w_read_data : csr_word_t;

    signal w_word_addr : integer range 0 to 127;
    signal r_word_addr : integer range 0 to 127;

    signal r_commit_pulse      : std_logic := '0';
    signal r_clear_pulse       : std_logic := '0';
    signal r_soft_reset_pulse  : std_logic := '0';
    signal r_operation_command_valid : std_logic := '0';
    signal r_operation_command : operation_command_t := OP_COMMAND_NONE;
    signal r_access_error_event : std_logic := '0';

    signal r_done_sticky      : std_logic := '0';
    signal r_success_sticky   : std_logic := '0';
    signal r_error_sticky     : std_logic := '0';
    signal r_rejected_sticky  : std_logic := '0';
    signal r_access_error_sticky : std_logic := '0';
    signal r_shadow_dirty     : std_logic := '1';
    signal r_last_error_code  : std_logic_vector(7 downto 0) := x"00";
    signal r_last_reject_code : std_logic_vector(7 downto 0) := x"00";
    signal r_completion_count : unsigned(15 downto 0) := (others => '0');
    signal r_shadow_revision  : unsigned(31 downto 0) := (others => '0');
    signal r_commit_revision  : unsigned(31 downto 0) := (others => '0');

    signal w_intr_sources : std_logic_vector(C_LIDAR_IRQ_SOURCES - 1 downto 0);
    signal w_intr_read_data : csr_word_t;
    signal w_intr_read_hit  : std_logic;

    function fn_output_width_code(width_value : natural)
        return std_logic_vector is
    begin
        case width_value is
            when 32     => return "00";
            when 64     => return "01";
            when others => return "10";
        end case;
    end function fn_output_width_code;

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-CSR-001 illegal build configuration"
        severity failure;

    o_shadow             <= fn_unpack_runtime_config(r_shadow_words);
    o_gpx_image_shadow   <= r_gpx_image_staging;
    o_commit             <= r_commit_pulse;
    o_clear_status       <= r_clear_pulse;
    o_soft_reset_request <= r_soft_reset_pulse;
    o_operation_command_valid <= r_operation_command_valid;
    o_operation_command       <= r_operation_command;
    o_diag_request_valid <= r_diag_request_valid;
    o_diag_request_index <= r_diag_index;
    o_diag_response_ready <= r_diag_busy;

    w_word_addr <= to_integer(unsigned(w_addr(8 downto 2)));
    r_word_addr <= to_integer(unsigned(r_addr(8 downto 2)));

    u_axil_fsm : entity work.axil_fsm_32
        generic map (
            num_ctl_regs  => C_LIDAR_CTL_COUNT,
            num_stat_regs => C_LIDAR_STAT_COUNT,
            num_data_bits => 32,
            en_secure_chk => '0',
            en_priv_chk   => '0'
        )
        port map (
            aclk    => i_clk,
            aresetn => i_rst_n,
            awaddr  => s_axi_awaddr,
            awprot  => s_axi_awprot,
            awvalid => s_axi_awvalid,
            awready => s_axi_awready,
            wdata   => s_axi_wdata,
            wstrb   => s_axi_wstrb,
            wvalid  => s_axi_wvalid,
            wready  => s_axi_wready,
            bresp   => s_axi_bresp,
            bvalid  => s_axi_bvalid,
            bready  => s_axi_bready,
            araddr  => s_axi_araddr,
            arprot  => s_axi_arprot,
            arvalid => s_axi_arvalid,
            arready => s_axi_arready,
            rdata   => s_axi_rdata,
            rresp   => s_axi_rresp,
            rvalid  => s_axi_rvalid,
            rready  => s_axi_rready,
            w_addr  => w_addr,
            w_data  => w_data,
            w_strb  => w_strb,
            w_we    => w_we,
            r_addr  => r_addr,
            rd_data => w_read_data
        );

    w_intr_sources(C_IRQ_COMMIT_SUCCESS) <= '1' when
        i_cfg_done = '1' and i_cfg_error = CFG_OK else '0';
    w_intr_sources(C_IRQ_COMMIT_ERROR) <= '1' when
        i_cfg_done = '1' and i_cfg_error /= CFG_OK else '0';
    w_intr_sources(C_IRQ_COMMIT_REJECTED) <= i_cfg_commit_rejected;
    w_intr_sources(C_IRQ_RECOVERY_REQUIRED) <= i_cfg_recovery_required;
    w_intr_sources(C_IRQ_ACCESS_ERROR) <= r_access_error_event;
    w_intr_sources(C_IRQ_PROCESSING_WARNING) <=
        i_runtime_irq(C_RUNTIME_IRQ_PROCESSING_WARNING);
    w_intr_sources(C_IRQ_LASER_TIMEOUT) <=
        i_runtime_irq(C_RUNTIME_IRQ_LASER_TIMEOUT);
    w_intr_sources(C_IRQ_ECHO_DIAGNOSTIC) <=
        i_runtime_irq(C_RUNTIME_IRQ_ECHO_DIAGNOSTIC);
    w_intr_sources(C_IRQ_GPX_TRANSPORT) <=
        i_runtime_irq(C_RUNTIME_IRQ_GPX_TRANSPORT);
    w_intr_sources(C_IRQ_GPX_DATA) <=
        i_runtime_irq(C_RUNTIME_IRQ_GPX_DATA);

    u_interrupts : entity work.axil_intr_32
        generic map (
            num_ctl_regs      => C_LIDAR_CTL_COUNT,
            num_stat_regs     => C_LIDAR_STAT_COUNT,
            num_intr_regs     => C_LIDAR_IRQ_REGISTER_COUNT,
            num_interrupt_src => C_LIDAR_IRQ_SOURCES,
            num_data_bits     => 32
        )
        port map (
            aclk          => i_clk,
            aresetn       => i_rst_n,
            w_addr_num    => w_word_addr,
            w_data        => w_data,
            w_strb        => w_strb,
            w_we          => w_we,
            intrpt_src_in => w_intr_sources,
            r_addr_num    => r_word_addr,
            rd_data       => w_intr_read_data,
            rd_hit        => w_intr_read_hit,
            irq           => o_irq
        );

    p_registers : process (i_clk, i_rst_n)
        variable v_effective_command : csr_word_t;
        variable v_merged_word       : csr_word_t;
        variable v_command_count     : natural;
        variable v_write_valid       : boolean;
        variable v_shadow_changed    : boolean;
        variable v_portal_control    : csr_word_t;
        variable v_image_word        : csr_word_t;
        variable v_diag_control      : csr_word_t;
    begin
        if i_rst_n = '0' then
            r_shadow_words       <= C_DEFAULT_WORDS;
            r_gpx_image_staging  <= C_GPX_REGISTER_IMAGE_DEFAULT;
            r_gpx_image_index    <= 0;
            r_gpx_image_view_active <= '0';
            r_diag_index         <= (others => '0');
            r_diag_request_valid <= '0';
            r_diag_busy          <= '0';
            r_diag_valid         <= '0';
            r_diag_error         <= '0';
            r_diag_data          <= (others => '0');
            r_diag_sequence      <= (others => '0');
            r_commit_pulse       <= '0';
            r_clear_pulse        <= '0';
            r_soft_reset_pulse   <= '0';
            r_operation_command_valid <= '0';
            r_operation_command <= OP_COMMAND_NONE;
            r_access_error_event <= '0';

            r_done_sticky        <= '0';
            r_success_sticky     <= '0';
            r_error_sticky       <= '0';
            r_rejected_sticky    <= '0';
            r_access_error_sticky <= '0';
            r_shadow_dirty       <= '1';
            r_last_error_code    <= x"00";
            r_last_reject_code   <= x"00";
            r_completion_count   <= (others => '0');
            r_shadow_revision    <= (others => '0');
            r_commit_revision    <= (others => '0');
        elsif rising_edge(i_clk) then
            v_shadow_changed := false;
            r_commit_pulse       <= '0';
            r_clear_pulse        <= '0';
            r_soft_reset_pulse   <= '0';
            r_operation_command_valid <= '0';
            r_operation_command <= OP_COMMAND_NONE;
            r_access_error_event <= '0';

            if r_diag_request_valid = '1' and
                    i_diag_request_ready = '1' then
                r_diag_request_valid <= '0';
            end if;

            if i_diag_response_valid = '1' and r_diag_busy = '1' then
                r_diag_busy <= '0';
                r_diag_valid <= '1';
                r_diag_error <= i_diag_response(
                    C_DIAG_RESPONSE_ERROR_BIT);
                r_diag_data <= i_diag_response(
                    C_DIAG_RESPONSE_DATA_HI downto
                    C_DIAG_RESPONSE_DATA_LO);
                r_diag_sequence <= r_diag_sequence + 1;
                if i_diag_response(C_DIAG_RESPONSE_ERROR_BIT) = '1' then
                    r_access_error_sticky <= '1';
                    r_access_error_event <= '1';
                end if;
            end if;

            -- r_commit_pulse is observed here one clock after the AXI write,
            -- on the same edge where the manager accepts or rejects it.  This
            -- also covers a new COMMIT coincident with prior completion.
            if r_commit_pulse = '1' and i_cfg_busy = '0' then
                r_commit_revision <= r_shadow_revision;
            end if;

            if w_we = '1' then
                if w_addr(1 downto 0) /= "00" then
                    r_access_error_sticky <= '1';
                    r_access_error_event  <= '1';
                elsif w_word_addr = C_CTL_COMMAND then
                    v_effective_command := (others => '0');
                    for byte_index in 0 to 3 loop
                        if w_strb(byte_index) = '1' then
                            v_effective_command(
                                8 * byte_index + 7 downto 8 * byte_index) :=
                                w_data(8 * byte_index + 7 downto
                                    8 * byte_index);
                        end if;
                    end loop;
                    v_command_count := fn_popcount(
                        v_effective_command and C_COMMAND_VALID_MASK);

                    if (v_effective_command and not C_COMMAND_VALID_MASK) /=
                            x"00000000"
                       or v_command_count > 1 then
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    elsif v_command_count = 0 then
                        null;
                    elsif v_effective_command(C_CMD_COMMIT_BIT) = '1' and
                          r_diag_busy = '1' and
                          fn_diag_is_gpx_register_read(r_diag_index) then
                        -- GPX image 적용과 실제 Register read는 같은 물리
                        -- bus를 사용한다. 둘을 겹치지 않고 다시 요청하게 한다.
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    elsif v_effective_command(C_CMD_COMMIT_BIT) = '1' then
                        r_commit_pulse <= '1';
                    elsif v_effective_command(C_CMD_CLEAR_STATUS_BIT) = '1'
                          and i_system_command_ready /= '1' then
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    elsif v_effective_command(C_CMD_CLEAR_STATUS_BIT) = '1' then
                        r_clear_pulse         <= '1';
                        r_done_sticky         <= '0';
                        r_success_sticky      <= '0';
                        r_error_sticky        <= '0';
                        r_rejected_sticky     <= '0';
                        r_access_error_sticky <= '0';
                        r_last_error_code     <= x"00";
                        r_last_reject_code    <= x"00";
                        r_diag_valid          <= '0';
                        r_diag_error          <= '0';
                        r_diag_data           <= (others => '0');
                    elsif v_effective_command(C_CMD_SOFT_RESET_BIT) = '1'
                          and i_system_command_ready /= '1' then
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    elsif v_effective_command(C_CMD_SOFT_RESET_BIT) = '1' then
                        r_soft_reset_pulse <= '1';
                    elsif (v_effective_command(C_CMD_RUN_BIT) = '1' or
                           v_effective_command(C_CMD_ARM_BIT) = '1') and
                          r_diag_busy = '1' and
                          fn_diag_is_gpx_register_read(r_diag_index) then
                        -- 물리 read 완료 전 RUN/ARM을 허용하면 pause 해제와
                        -- 동시에 새 Shot이 들어올 수 있으므로 명시적으로 거부한다.
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    elsif i_operation_command_ready /= '1' then
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    else
                        r_operation_command_valid <= '1';
                        if v_effective_command(C_CMD_RUN_BIT) = '1' then
                            r_operation_command <= OP_COMMAND_RUN;
                        elsif v_effective_command(C_CMD_STOP_BIT) = '1' then
                            r_operation_command <= OP_COMMAND_STOP;
                        elsif v_effective_command(C_CMD_ARM_BIT) = '1' then
                            r_operation_command <= OP_COMMAND_ARM;
                        else
                            r_operation_command <= OP_COMMAND_DISARM;
                        end if;
                    end if;
                elsif w_word_addr >= C_CTL_MOTOR_PROFILE and
                      w_word_addr <= C_CTL_ECHO_DELAY_PROFILE then
                    -- Keep the common shadow-register path ahead of the two
                    -- indexed portals. This removes unrelated portal decode
                    -- levels from every ordinary configuration write.
                    v_merged_word := r_shadow_words(w_word_addr);
                    for byte_index in 0 to 3 loop
                        if w_strb(byte_index) = '1' then
                            v_merged_word(
                                8 * byte_index + 7 downto 8 * byte_index) :=
                                w_data(8 * byte_index + 7 downto
                                    8 * byte_index);
                        end if;
                    end loop;
                    v_write_valid := fn_ctl_word_encoding_valid(
                        w_word_addr, v_merged_word);
                    if v_write_valid then
                        if v_merged_word /= r_shadow_words(w_word_addr) then
                            r_shadow_words(w_word_addr) <= v_merged_word;
                            r_shadow_revision <= r_shadow_revision + 1;
                            v_shadow_changed := true;
                        end if;
                    else
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    end if;
                elsif w_word_addr = C_CTL_GPX_IMAGE_INDEX then
                    v_portal_control := (others => '0');
                    v_portal_control(
                        C_GPX_IMAGE_INDEX_MSB downto
                        C_GPX_IMAGE_INDEX_LSB) := std_logic_vector(
                            to_unsigned(r_gpx_image_index, 4));
                    v_portal_control(C_GPX_IMAGE_VIEW_ACTIVE_BIT) :=
                        r_gpx_image_view_active;
                    for byte_index in 0 to 3 loop
                        if w_strb(byte_index) = '1' then
                            v_portal_control(
                                8 * byte_index + 7 downto 8 * byte_index) :=
                                w_data(8 * byte_index + 7 downto
                                    8 * byte_index);
                        end if;
                    end loop;
                    if fn_ctl_word_encoding_valid(
                            C_CTL_GPX_IMAGE_INDEX, v_portal_control) then
                        r_gpx_image_index <= to_integer(unsigned(
                            v_portal_control(
                                C_GPX_IMAGE_INDEX_MSB downto
                                C_GPX_IMAGE_INDEX_LSB)));
                        r_gpx_image_view_active <= v_portal_control(
                            C_GPX_IMAGE_VIEW_ACTIVE_BIT);
                    else
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    end if;
                elsif w_word_addr = C_CTL_GPX_IMAGE_DATA then
                    if r_gpx_image_view_active = '1' then
                        -- Active image is observation-only. Software must
                        -- select staging view before editing an entry.
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    else
                        v_image_word := r_gpx_image_staging(
                            r_gpx_image_index);
                        for byte_index in 0 to 3 loop
                            if w_strb(byte_index) = '1' then
                                v_image_word(
                                    8 * byte_index + 7 downto
                                    8 * byte_index) := w_data(
                                        8 * byte_index + 7 downto
                                        8 * byte_index);
                            end if;
                        end loop;
                        if fn_ctl_word_encoding_valid(
                                C_CTL_GPX_IMAGE_DATA, v_image_word) then
                            if v_image_word /= r_gpx_image_staging(
                                    r_gpx_image_index) then
                                r_gpx_image_staging(r_gpx_image_index) <=
                                    v_image_word;
                                r_shadow_revision <= r_shadow_revision + 1;
                                v_shadow_changed := true;
                            end if;
                        else
                            r_access_error_sticky <= '1';
                            r_access_error_event  <= '1';
                        end if;
                    end if;
                elsif w_word_addr = C_CTL_DIAG_INDEX then
                    if r_diag_busy = '1' then
                        -- The selected index is the in-flight request payload;
                        -- changing it under backpressure would violate the
                        -- ready/valid contract.
                        r_access_error_sticky <= '1';
                        r_access_error_event  <= '1';
                    else
                        v_diag_control := (others => '0');
                        v_diag_control(C_DIAG_INDEX_MSB downto
                            C_DIAG_INDEX_LSB) := r_diag_index;
                        for byte_index in 0 to 3 loop
                            if w_strb(byte_index) = '1' then
                                v_diag_control(
                                    8 * byte_index + 7 downto
                                    8 * byte_index) := w_data(
                                        8 * byte_index + 7 downto
                                        8 * byte_index);
                            end if;
                        end loop;
                        if fn_ctl_word_encoding_valid(
                                C_CTL_DIAG_INDEX, v_diag_control) then
                            r_diag_index <= v_diag_control(
                                C_DIAG_INDEX_MSB downto C_DIAG_INDEX_LSB);
                            r_diag_valid <= '0';
                            r_diag_error <= '0';
                            r_diag_data <= (others => '0');
                            if v_diag_control(C_DIAG_CAPTURE_BIT) = '1' then
                                if fn_diag_is_gpx_register_read(
                                        v_diag_control(
                                            C_DIAG_INDEX_MSB downto
                                            C_DIAG_INDEX_LSB)) and
                                   (i_operation_status.scheduler_enable = '1' or
                                    i_cfg_busy = '1') then
                                    -- DISARM 완료와 config transaction 종료를
                                    -- 확인하지 않은 물리 read는 bus에 내보내지 않는다.
                                    r_diag_valid <= '1';
                                    r_diag_error <= '1';
                                    r_diag_data <=
                                        fn_pack_gpx_register_read_word(
                                            fn_diag_gpx_register_address(
                                                v_diag_control(
                                                    C_DIAG_INDEX_MSB downto
                                                    C_DIAG_INDEX_LSB)),
                                            (others => '0'));
                                    r_diag_sequence <= r_diag_sequence + 1;
                                    r_access_error_sticky <= '1';
                                    r_access_error_event <= '1';
                                else
                                    r_diag_request_valid <= '1';
                                    r_diag_busy <= '1';
                                end if;
                            end if;
                        else
                            r_access_error_sticky <= '1';
                            r_access_error_event  <= '1';
                        end if;
                    end if;
                elsif w_word_addr = C_CTL_DIAG_DATA then
                    r_access_error_sticky <= '1';
                    r_access_error_event  <= '1';
                elsif w_word_addr = C_LIDAR_CTL_COUNT +
                        C_LIDAR_STAT_COUNT
                      or w_word_addr = C_LIDAR_CTL_COUNT +
                        C_LIDAR_STAT_COUNT + 2
                      or w_word_addr = C_LIDAR_CTL_COUNT +
                        C_LIDAR_STAT_COUNT + 3 then
                    null;
                else
                    r_access_error_sticky <= '1';
                    r_access_error_event  <= '1';
                end if;
            end if;

            -- A race in which READY closes after the CSR accepted the W1S
            -- write is still diagnosed; the mailbox never silently overwrites
            -- the earlier command.
            if i_operation_command_rejected = '1' then
                r_access_error_sticky <= '1';
                r_access_error_event  <= '1';
            end if;
            if i_system_command_rejected = '1' then
                r_access_error_sticky <= '1';
                r_access_error_event  <= '1';
            end if;

            if i_cfg_done = '1' then
                r_done_sticky      <= '1';
                r_completion_count <= r_completion_count + 1;
                if i_cfg_error = CFG_OK then
                    r_success_sticky <= '1';
                    if r_shadow_revision = r_commit_revision then
                        r_shadow_dirty <= '0';
                    else
                        r_shadow_dirty <= '1';
                    end if;
                else
                    r_error_sticky    <= '1';
                    r_last_error_code <= fn_cfg_error_code(i_cfg_error);
                end if;
            end if;

            if i_cfg_commit_rejected = '1' then
                r_rejected_sticky    <= '1';
                r_last_reject_code   <= fn_cfg_error_code(i_cfg_reject_error);
            end if;

            -- A write concurrent with successful activation belongs to the
            -- next transaction and therefore keeps the shadow dirty.
            if v_shadow_changed then
                r_shadow_dirty <= '1';
            end if;
        end if;
    end process p_registers;

    p_status : process (
        i_cfg_busy,
        i_cfg_error,
        i_cfg_recovery_required,
        i_cfg_active_valid,
        i_cfg_active,
        i_operation_status,
        i_operation_command_ready,
        i_operation_command_busy,
        r_done_sticky,
        r_success_sticky,
        r_error_sticky,
        r_rejected_sticky,
        r_access_error_sticky,
        r_shadow_dirty,
        r_last_error_code,
        r_last_reject_code,
        r_completion_count
    )
        variable v_status : csr_word_array_t;
        variable v_active : csr_word_array_t;
    begin
        v_status := (others => (others => '0'));

        v_status(C_STAT_CORE_INFO)(7 downto 0) := std_logic_vector(
            to_unsigned(C_LIDAR_CSR_ABI_MINOR, 8));
        v_status(C_STAT_CORE_INFO)(15 downto 8) := std_logic_vector(
            to_unsigned(C_LIDAR_CSR_ABI_MAJOR, 8));
        v_status(C_STAT_CORE_INFO)(18 downto 16) := std_logic_vector(
            to_unsigned(G_BUILD_CONFIG.num_faces, 3));
        v_status(C_STAT_CORE_INFO)(21 downto 19) := std_logic_vector(
            to_unsigned(G_BUILD_CONFIG.num_chips, 3));
        v_status(C_STAT_CORE_INFO)(25 downto 22) := std_logic_vector(
            to_unsigned(G_BUILD_CONFIG.stops_per_chip, 4));
        v_status(C_STAT_CORE_INFO)(28 downto 26) := std_logic_vector(
            to_unsigned(G_BUILD_CONFIG.max_returns_per_stop, 3));
        if G_BUILD_CONFIG.enable_echo_receiver then
            v_status(C_STAT_CORE_INFO)(29) := '1';
        end if;
        if G_BUILD_CONFIG.enable_echo_simulation then
            v_status(C_STAT_CORE_INFO)(30) := '1';
        end if;
        if G_BUILD_CONFIG.stream_clock_mode = STREAM_CLOCK_SYNC then
            v_status(C_STAT_CORE_INFO)(31) := '1';
        end if;

        v_status(C_STAT_BUILD_INFO)(7 downto 0) := std_logic_vector(
            to_unsigned(G_BUILD_CONFIG.proc_clk_mhz, 8));
        v_status(C_STAT_BUILD_INFO)(15 downto 8) := std_logic_vector(
            to_unsigned(G_BUILD_CONFIG.tdc_clk_mhz, 8));
        v_status(C_STAT_BUILD_INFO)(17 downto 16) :=
            fn_output_width_code(G_BUILD_CONFIG.output_width);
        v_status(C_STAT_BUILD_INFO)(23 downto 20) :=
            G_BUILD_CONFIG.rise_capability_mask;
        v_status(C_STAT_BUILD_INFO)(27 downto 24) :=
            G_BUILD_CONFIG.fall_capability_mask;

        v_status(C_STAT_TRANSACTION)(C_TXN_BUSY_BIT) := i_cfg_busy;
        v_status(C_STAT_TRANSACTION)(C_TXN_DONE_STICKY_BIT) :=
            r_done_sticky;
        v_status(C_STAT_TRANSACTION)(C_TXN_SUCCESS_STICKY_BIT) :=
            r_success_sticky;
        v_status(C_STAT_TRANSACTION)(C_TXN_ERROR_STICKY_BIT) :=
            r_error_sticky;
        v_status(C_STAT_TRANSACTION)(C_TXN_REJECTED_STICKY_BIT) :=
            r_rejected_sticky;
        v_status(C_STAT_TRANSACTION)(C_TXN_RECOVERY_REQUIRED_BIT) :=
            i_cfg_recovery_required;
        v_status(C_STAT_TRANSACTION)(C_TXN_ACTIVE_VALID_BIT) :=
            i_cfg_active_valid;
        v_status(C_STAT_TRANSACTION)(C_TXN_ACCESS_ERROR_BIT) :=
            r_access_error_sticky;
        v_status(C_STAT_TRANSACTION)(C_TXN_SHADOW_DIRTY_BIT) :=
            r_shadow_dirty;
        v_status(C_STAT_TRANSACTION)(23 downto 16) := r_last_error_code;
        v_status(C_STAT_TRANSACTION)(31 downto 24) := r_last_reject_code;

        if i_cfg_active_valid = '1' then
            v_status(C_STAT_ACTIVE_VERSION)(15 downto 0) :=
                std_logic_vector(i_cfg_active.version);
            v_active := fn_pack_runtime_config(i_cfg_active.source);
            for source_index in 1 to C_CTL_TDC_CAPTURE_ADJUST loop
                v_status(C_STAT_ACTIVE_SOURCE_BASE + source_index - 1) :=
                    v_active(source_index);
            end loop;

            v_status(C_STAT_DERIVED_GEOMETRY)(15 downto 0) :=
                std_logic_vector(i_cfg_active.derived.total_states);
            v_status(C_STAT_DERIVED_GEOMETRY)(31 downto 16) :=
                std_logic_vector(i_cfg_active.derived.shot_interval_states);
            v_status(C_STAT_DERIVED_FACE)(15 downto 0) :=
                std_logic_vector(i_cfg_active.derived.face_active_positions);
            v_status(C_STAT_DERIVED_FACE)(31 downto 16) :=
                std_logic_vector(i_cfg_active.derived.columns_per_face);
            for face_index in 0 to C_MAX_FACES - 1 loop
                v_status(C_STAT_FACE_BOUNDS_0 + face_index)(14 downto 0) :=
                    std_logic_vector(i_cfg_active.derived.face_lower(face_index));
                v_status(C_STAT_FACE_BOUNDS_0 + face_index)(30 downto 16) :=
                    std_logic_vector(i_cfg_active.derived.face_upper(face_index));
            end loop;
            v_status(C_STAT_CAPTURE_TDC_CLKS) := std_logic_vector(
                i_cfg_active.derived.capture_window_tdc_clks);
            v_status(C_STAT_DERIVED_MASKS)(3 downto 0) :=
                i_cfg_active.derived.present_chip_mask;
            v_status(C_STAT_DERIVED_MASKS)(7 downto 4) :=
                i_cfg_active.derived.active_rise_mask;
            v_status(C_STAT_DERIVED_MASKS)(11 downto 8) :=
                i_cfg_active.derived.active_fall_mask;
            v_status(C_STAT_DERIVED_MASKS)(31 downto 16) :=
                std_logic_vector(r_completion_count);
        end if;

        if i_cfg_active_valid = '0' then
            v_status(C_STAT_DERIVED_MASKS)(31 downto 16) :=
                std_logic_vector(r_completion_count);
        end if;

        v_status(C_STAT_ACTIVE_VERSION)(C_OP_RUNNING_BIT) :=
            i_operation_status.running;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_ARMED_BIT) :=
            i_operation_status.armed;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_EXTERNAL_PERMIT_BIT) :=
            i_operation_status.external_permit;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_CONFIG_READY_BIT) :=
            i_operation_status.config_ready;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_PROCESSING_ENABLE_BIT) :=
            i_operation_status.processing_enable;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_SCHEDULER_ENABLE_BIT) :=
            i_operation_status.scheduler_enable;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_PHYSICAL_FIRE_ENABLE_BIT) :=
            i_operation_status.physical_fire_enable;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_SIMULATION_ENABLE_BIT) :=
            i_operation_status.simulation_enable;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_COMMAND_READY_BIT) :=
            i_operation_command_ready;
        v_status(C_STAT_ACTIVE_VERSION)(C_OP_COMMAND_BUSY_BIT) :=
            i_operation_command_busy;

        w_status_words <= v_status;
    end process p_status;

    p_read_mux : process (
        r_addr,
        r_word_addr,
        r_shadow_words,
        r_gpx_image_staging,
        r_gpx_image_index,
        r_gpx_image_view_active,
        r_diag_index,
        r_diag_busy,
        r_diag_valid,
        r_diag_error,
        r_diag_data,
        r_diag_sequence,
        i_cfg_active_valid,
        i_cfg_active_gpx_image,
        w_status_words,
        w_intr_read_data,
        w_intr_read_hit
    )
    begin
        w_read_data <= (others => '0');
        if r_addr(1 downto 0) = "00" then
            if r_word_addr < C_LIDAR_CTL_COUNT then
                if r_word_addr = C_CTL_GPX_IMAGE_INDEX then
                    w_read_data(
                        C_GPX_IMAGE_INDEX_MSB downto
                        C_GPX_IMAGE_INDEX_LSB) <= std_logic_vector(
                            to_unsigned(r_gpx_image_index, 4));
                    w_read_data(C_GPX_IMAGE_VIEW_ACTIVE_BIT) <=
                        r_gpx_image_view_active;
                elsif r_word_addr = C_CTL_GPX_IMAGE_DATA then
                    if r_gpx_image_view_active = '1' then
                        if i_cfg_active_valid = '1' then
                            w_read_data <= i_cfg_active_gpx_image(
                                r_gpx_image_index);
                        end if;
                    else
                        w_read_data <= r_gpx_image_staging(
                            r_gpx_image_index);
                    end if;
                elsif r_word_addr = C_CTL_DIAG_INDEX then
                    w_read_data(C_DIAG_INDEX_MSB downto
                        C_DIAG_INDEX_LSB) <= r_diag_index;
                    w_read_data(C_DIAG_BUSY_BIT) <= r_diag_busy;
                    w_read_data(C_DIAG_VALID_BIT) <= r_diag_valid;
                    w_read_data(C_DIAG_ERROR_BIT) <= r_diag_error;
                    w_read_data(C_DIAG_SEQUENCE_MSB downto
                        C_DIAG_SEQUENCE_LSB) <= std_logic_vector(
                            r_diag_sequence);
                elsif r_word_addr = C_CTL_DIAG_DATA then
                    w_read_data <= r_diag_data;
                elsif r_word_addr /= C_CTL_COMMAND then
                    w_read_data <= r_shadow_words(r_word_addr);
                end if;
            elsif r_word_addr < C_LIDAR_CTL_COUNT + C_LIDAR_STAT_COUNT then
                w_read_data <= w_status_words(
                    r_word_addr - C_LIDAR_CTL_COUNT);
            elsif w_intr_read_hit = '1' then
                w_read_data <= w_intr_read_data;
            end if;
        end if;
    end process p_read_mux;

end architecture rtl;
