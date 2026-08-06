library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_event_pkg.all;

-- Registered, fair merge for the per-Chip acquisition streams. A dedicated
-- one-entry ingress slot per lane breaks every cross-lane ready path before
-- arbitration. The output slot then makes every field stable under downstream
-- backpressure.
entity lidar_gpx_event_merge is
    port (
        i_clk            : in  std_logic;
        i_rst_n          : in  std_logic;

        i_shot_accept    : in  std_logic;
        i_shot_mask      : in  chip_mask_t;

        i_lane_event     : in  gpx_raw_event_array_t;
        o_lane_ready     : out chip_mask_t;

        o_event          : out gpx_raw_event_t;
        i_event_ready    : in  std_logic;

        o_shot_complete  : out std_logic;
        o_shot_outstanding : out std_logic;
        o_terminal_mask  : out chip_mask_t
    );
end entity lidar_gpx_event_merge;

architecture rtl of lidar_gpx_event_merge is

    signal shot_mask_r : chip_mask_t := (others => '0');
    signal terminal_mask_r : chip_mask_t := (others => '0');
    signal shot_outstanding_r : std_logic := '0';
    signal shot_complete_r : std_logic := '0';

    signal event_r : gpx_raw_event_t := C_GPX_RAW_EVENT_IDLE;
    signal ingress_r : gpx_raw_event_array_t :=
        (others => C_GPX_RAW_EVENT_IDLE);
    signal round_robin_r : natural range 0 to C_MAX_CHIPS - 1 := 0;

    signal selected_valid_c : std_logic := '0';
    signal selected_index_c : natural range 0 to C_MAX_CHIPS - 1 := 0;
    signal lane_ready_c : chip_mask_t := (others => '0');
    signal ingress_take_c : chip_mask_t := (others => '0');
    signal slot_available_c : std_logic;

    function fn_is_terminal(kind : gpx_raw_event_kind_t) return boolean is
    begin
        return kind = GPX_RAW_DRAIN_DONE or kind = GPX_RAW_TIMEOUT;
    end function fn_is_terminal;

begin

    o_event <= event_r;
    o_lane_ready <= lane_ready_c;
    o_shot_complete <= shot_complete_r;
    o_shot_outstanding <= shot_outstanding_r;
    o_terminal_mask <= terminal_mask_r;

    slot_available_c <= '1' when event_r.valid = '0' or
                                i_event_ready = '1' else '0';

    -- Deliberately do not add same-cycle refill. Qualify each ingress lane with
    -- the registered Shot mask here, before central arbitration. This keeps an
    -- inactive lane from handshaking and removes the wide Shot-mask-to-event
    -- mux cone from the 200 MHz output register path.
    gen_lane_ready : for index in 0 to C_MAX_CHIPS - 1 generate
        lane_ready_c(index) <= not ingress_r(index).valid
            when shot_outstanding_r = '1' and shot_mask_r(index) = '1'
            else '0';
    end generate gen_lane_ready;

    p_select : process (all)
        variable candidate : natural range 0 to C_MAX_CHIPS - 1;
        variable found : boolean;
        variable take_value : chip_mask_t;
    begin
        selected_valid_c <= '0';
        selected_index_c <= round_robin_r;
        take_value := (others => '0');
        found := false;

        if slot_available_c = '1' and shot_outstanding_r = '1' then
            for offset in 0 to C_MAX_CHIPS - 1 loop
                candidate := (round_robin_r + offset) mod C_MAX_CHIPS;
                if not found and ingress_r(candidate).valid = '1' then
                    selected_valid_c <= '1';
                    selected_index_c <= candidate;
                    take_value(candidate) := '1';
                    found := true;
                end if;
            end loop;
        end if;

        ingress_take_c <= take_value;
    end process p_select;

    p_ingress : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                ingress_r <= (others => C_GPX_RAW_EVENT_IDLE);
            else
                for index in 0 to C_MAX_CHIPS - 1 loop
                    if ingress_r(index).valid = '0' then
                        if i_lane_event(index).valid = '1' then
                            ingress_r(index) <= i_lane_event(index);
                            ingress_r(index).valid <= '1';
                        end if;
                    elsif ingress_take_c(index) = '1' then
                        ingress_r(index) <= C_GPX_RAW_EVENT_IDLE;
                    end if;
                end loop;
            end if;
        end if;
    end process p_ingress;

    p_merge : process (i_clk)
        variable completed_mask : chip_mask_t;
        variable accepted_chip : natural range 0 to C_MAX_CHIPS - 1;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                shot_mask_r <= (others => '0');
                terminal_mask_r <= (others => '0');
                shot_outstanding_r <= '0';
                shot_complete_r <= '0';
                event_r <= C_GPX_RAW_EVENT_IDLE;
                round_robin_r <= 0;
            else
                shot_complete_r <= '0';

                if i_shot_accept = '1' then
                    shot_mask_r <= i_shot_mask;
                    terminal_mask_r <= (others => '0');
                    shot_outstanding_r <= '1';
                    round_robin_r <= 0;
                end if;

                if event_r.valid = '1' and i_event_ready = '1' and
                   fn_is_terminal(event_r.kind) then
                    completed_mask := terminal_mask_r;
                    accepted_chip := to_integer(event_r.chip_index);
                    completed_mask(accepted_chip) := '1';
                    terminal_mask_r <= completed_mask;

                    if (completed_mask and shot_mask_r) = shot_mask_r then
                        shot_outstanding_r <= '0';
                        shot_complete_r <= '1';
                    end if;
                end if;

                if slot_available_c = '1' then
                    if selected_valid_c = '1' then
                        event_r <= ingress_r(selected_index_c);
                        event_r.valid <= '1';
                        if selected_index_c = C_MAX_CHIPS - 1 then
                            round_robin_r <= 0;
                        else
                            round_robin_r <= selected_index_c + 1;
                        end if;
                    else
                        -- Payload is don't-care while valid is low. Holding it
                        -- avoids broadcasting downstream ready into every bit
                        -- of the wide Shot context as a synchronous clear.
                        event_r.valid <= '0';
                    end if;
                end if;

                -- synthesis translate_off
                assert not (i_shot_accept = '1' and
                            (shot_outstanding_r = '1' or event_r.valid = '1'))
                    report "V2-GPX-MERGE-001 overlapping Shot acceptance"
                    severity failure;
                for index in 0 to C_MAX_CHIPS - 1 loop
                    if lane_ready_c(index) = '1' and
                       i_lane_event(index).valid = '1' then
                        assert to_integer(i_lane_event(index).chip_index) = index
                            report "V2-GPX-MERGE-002 lane/Chip identity mismatch"
                            severity failure;
                    end if;
                end loop;
                if event_r.valid = '1' and i_event_ready = '1' and
                   fn_is_terminal(event_r.kind) then
                    assert shot_mask_r(to_integer(event_r.chip_index)) = '1'
                        report "V2-GPX-MERGE-003 terminal from inactive Chip"
                        severity failure;
                    assert terminal_mask_r(to_integer(event_r.chip_index)) = '0'
                        report "V2-GPX-MERGE-004 duplicate terminal event"
                        severity failure;
                end if;
                -- synthesis translate_on
            end if;
        end if;
    end process p_merge;

end architecture rtl;
