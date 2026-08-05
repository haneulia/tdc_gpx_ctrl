library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_gpx_data_pkg.all;
use work.lidar_gpx_vdma_pkg.all;

entity lidar_gpx_cell_word_serializer is
    port (
        i_clk        : in  std_logic;
        i_rst_n      : in  std_logic;
        i_abort      : in  std_logic;

        i_cell_event : in  gpx_frame_cell_event_t;
        o_cell_ready : out std_logic;

        o_word_event : out gpx_vdma_word_event_t;
        i_word_ready : in  std_logic;

        o_idle       : out std_logic
    );
end entity lidar_gpx_cell_word_serializer;

architecture rtl of lidar_gpx_cell_word_serializer is

    signal cell_event_r : gpx_frame_cell_event_t :=
        C_GPX_FRAME_CELL_EVENT_IDLE;
    signal cell_valid_r : std_logic := '0';
    signal word_index_r : natural range 0 to
        C_GPX_VDMA_MAX_CELL_WORDS - 1 := 0;
    signal word_count_r : natural range 2 to
        C_GPX_VDMA_MAX_CELL_WORDS := 2;

    signal word_event_r : gpx_vdma_word_event_t :=
        C_GPX_VDMA_WORD_EVENT_IDLE;
    signal output_ready_c : std_logic;
    signal current_last_c : std_logic;

begin

    output_ready_c <= '1' when word_event_r.valid = '0' or
        i_word_ready = '1' else '0';
    current_last_c <= '1' when cell_valid_r = '1' and
        word_index_r + 1 = word_count_r else '0';

    o_cell_ready <= '1' when cell_valid_r = '0' or
        (output_ready_c = '1' and current_last_c = '1') else '0';
    o_word_event <= word_event_r;
    o_idle <= '1' when cell_valid_r = '0' and
        word_event_r.valid = '0' else '0';

    p_serialize : process (i_clk)
        variable max_hits : positive range 1 to C_MAX_RETURNS_PER_STOP;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                cell_valid_r <= '0';
                word_index_r <= 0;
                word_count_r <= 2;
                word_event_r.valid <= '0';
            else
                if output_ready_c = '1' then
                    if cell_valid_r = '1' then
                        word_event_r <= fn_gpx_vdma_make_word_event(
                            cell_event_r, word_index_r);
                        if current_last_c = '1' then
                            cell_valid_r <= '0';
                            word_index_r <= 0;
                        else
                            word_index_r <= word_index_r + 1;
                        end if;
                    else
                        word_event_r.valid <= '0';
                    end if;
                end if;

                if i_cell_event.valid = '1' and o_cell_ready = '1' then
                    max_hits := fn_gpx_vdma_effective_max_hits(
                        i_cell_event.cell.max_hits);
                    cell_event_r <= i_cell_event;
                    cell_valid_r <= '1';
                    word_index_r <= 0;
                    word_count_r <= fn_gpx_vdma_cell_word_count(max_hits);
                end if;
            end if;
        end if;
    end process p_serialize;

    p_contract : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '1' and i_abort = '0' and
               i_cell_event.valid = '1' and o_cell_ready = '1' then
                assert i_cell_event.cell.kind = GPX_CELL_DATA
                    report "V2-B9-J1-001 serializer accepted non-data Cell"
                    severity failure;
                assert to_integer(i_cell_event.cell.hit_count) <=
                       fn_gpx_vdma_effective_max_hits(
                           i_cell_event.cell.max_hits)
                    report "V2-B9-J1-002 hit_count exceeds max_hits"
                    severity failure;
            end if;
        end if;
    end process p_contract;

end architecture rtl;
