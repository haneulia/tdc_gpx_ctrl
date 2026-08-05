library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;

-- Processing-domain owner of the explicit Face-close boundary.
--
-- face_tracker reports physical traversal, while shot_scheduler owns accepted
-- firing points. This block admits the same configured Face context, captures
-- its immutable geometry at entry, and waits for the executor to become ready
-- before releasing close. Therefore every Shot-start belonging to the Face has
-- already been observable by the B5-B8 path before close can be accepted.
entity lidar_face_close_owner is
    generic (
        G_BUILD_CONFIG : lidar_build_config_t := C_DEFAULT_BUILD_CONFIG
    );
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        i_enable            : in  std_logic;
        i_active_valid      : in  std_logic;
        i_active_config     : in  lidar_active_config_t;
        i_face_event        : in  face_event_t;
        i_executor_ready    : in  std_logic;
        i_clear_diagnostics : in  std_logic;

        o_close_event       : out face_close_event_t;
        i_close_ready       : in  std_logic;
        o_scheduler_block   : out std_logic;
        o_idle              : out std_logic;
        o_overflow_sticky   : out std_logic
    );
end entity lidar_face_close_owner;

architecture rtl of lidar_face_close_owner is

    constant C_BUILD_FACE_MASK : face_mask_t :=
        fn_active_face_mask(G_BUILD_CONFIG.num_faces);

    signal face_mask_r       : face_mask_t := (others => '0');
    signal columns_r         : shot_index_t := (others => '0');
    signal simulation_mode_r : std_logic := '0';
    signal active_version_r  : unsigned(15 downto 0) := (others => '0');

    signal traversal_open_r     : std_logic := '0';
    signal traversal_face_r     : face_index_t := (others => '0');
    signal traversal_direction_r: direction_t := DIRECTION_CW;
    signal traversal_source_r   : std_logic := '0';
    signal traversal_version_r  : unsigned(15 downto 0) := (others => '0');
    signal traversal_columns_r  : shot_index_t := (others => '0');

    signal wait_event_r  : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal close_event_r : face_close_event_t := C_FACE_CLOSE_EVENT_IDLE;
    signal overflow_sticky_r : std_logic := '0';

begin

    assert fn_validate_build_config(G_BUILD_CONFIG) = CFG_OK
        report "V2-FCLOSE-001 invalid build configuration"
        severity failure;

    o_close_event <= close_event_r;
    -- enter_event and exit_event may share one registered B1 event. Block the
    -- scheduler in that same cycle so a new-Face Shot cannot overtake the
    -- close event registered at the edge.
    o_scheduler_block <= wait_event_r.valid or close_event_r.valid or
        (i_face_event.valid and i_face_event.exit_event);
    o_idle <= not traversal_open_r and not wait_event_r.valid and
        not close_event_r.valid;
    o_overflow_sticky <= overflow_sticky_r;

    -- Configuration changes are legal only while scheduling is disabled. The
    -- real-time traversal path uses this registered snapshot exclusively.
    p_config : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                face_mask_r       <= (others => '0');
                columns_r         <= (others => '0');
                simulation_mode_r <= '0';
                active_version_r  <= (others => '0');
            elsif i_enable = '0' and i_active_valid = '1' then
                face_mask_r <= i_active_config.source.laser.face_enable_mask
                    and C_BUILD_FACE_MASK;
                columns_r <= i_active_config.derived.columns_per_face;
                simulation_mode_r <=
                    i_active_config.source.motor.simulation_mode;
                active_version_r <= i_active_config.version;
            end if;
        end if;
    end process p_config;

    p_owner : process (i_clk)
        variable face_number_v : natural range 0 to 7;
        variable context_valid_v : boolean;
        variable close_v : face_close_event_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                traversal_open_r      <= '0';
                traversal_face_r      <= (others => '0');
                traversal_direction_r <= DIRECTION_CW;
                traversal_source_r    <= '0';
                traversal_version_r   <= (others => '0');
                traversal_columns_r   <= (others => '0');
                wait_event_r           <= C_FACE_CLOSE_EVENT_IDLE;
                close_event_r          <= C_FACE_CLOSE_EVENT_IDLE;
                overflow_sticky_r      <= '0';
            else
                if i_clear_diagnostics = '1' then
                    overflow_sticky_r <= '0';
                end if;

                if close_event_r.valid = '1' and i_close_ready = '1' then
                    close_event_r.valid <= '0';
                end if;

                -- No same-cycle refill is used. The registered bubble keeps
                -- downstream ready from becoming a traversal control path.
                if wait_event_r.valid = '1' and i_executor_ready = '1' and
                   close_event_r.valid = '0' then
                    close_event_r <= wait_event_r;
                    close_event_r.valid <= '1';
                    wait_event_r.valid <= '0';
                end if;

                if i_enable = '0' or i_active_valid = '0' then
                    traversal_open_r <= '0';
                    wait_event_r <= C_FACE_CLOSE_EVENT_IDLE;
                    close_event_r <= C_FACE_CLOSE_EVENT_IDLE;
                elsif i_face_event.valid = '1' then
                    if i_face_event.exit_event = '1' and
                       traversal_open_r = '1' then
                        close_v := C_FACE_CLOSE_EVENT_IDLE;
                        close_v.valid := '1';
                        close_v.face_index := traversal_face_r;
                        close_v.direction := traversal_direction_r;
                        close_v.source_sim := traversal_source_r;
                        close_v.active_version := traversal_version_r;
                        close_v.columns_per_face := traversal_columns_r;

                        if wait_event_r.valid = '0' then
                            wait_event_r <= close_v;
                        else
                            overflow_sticky_r <= '1';
                        end if;
                        traversal_open_r <= '0';
                    end if;

                    -- Keep the new traversal context even when the same event
                    -- closes the previous Face. The scheduler remains blocked
                    -- by the pending close, so the context can advance without
                    -- allowing a new-Face Shot to overtake that close.
                    if i_face_event.enter_event = '1' and
                       i_face_event.inside = '1' then
                        face_number_v := to_integer(i_face_event.face_index);
                        context_valid_v :=
                            face_number_v < G_BUILD_CONFIG.num_faces and
                            face_mask_r(face_number_v) = '1' and
                            i_face_event.overlap = '0' and
                            i_face_event.active_version = active_version_r and
                            i_face_event.source_sim = simulation_mode_r and
                            columns_r /= 0;

                        if context_valid_v then
                            traversal_open_r <= '1';
                            traversal_face_r <= i_face_event.face_index;
                            traversal_direction_r <= i_face_event.direction;
                            traversal_source_r <= i_face_event.source_sim;
                            traversal_version_r <= i_face_event.active_version;
                            traversal_columns_r <= columns_r;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process p_owner;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_enable = '1' then
                assert i_active_valid = '1'
                    report "V2-FCLOSE-002 enable without active configuration"
                    severity failure;
                if close_event_r.valid = '1' then
                    assert close_event_r.columns_per_face /= 0
                        report "V2-FCLOSE-003 zero-column close event"
                        severity failure;
                end if;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
