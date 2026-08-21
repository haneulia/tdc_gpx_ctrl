library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_config_types_pkg.all;
use work.lidar_event_types_pkg.all;
use work.lidar_processing_pkg.all;

-- One-beat, observation-only AXI4-Stream tap of the registered B1 event.
-- Encoder and shot control cannot stop, so a candidate arriving while the
-- output beat is stalled is counted and dropped. The retained AXIS beat stays
-- stable until accepted, and TREADY has no output back into the control path.
entity lidar_processing_axis_monitor is
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        i_clear_diagnostics : in  std_logic;
        i_face_event        : in  face_event_t;

        m_axis_tready       : in  std_logic;
        m_axis_tvalid       : out std_logic;
        m_axis_tdata        : out processing_monitor_tdata_t;
        m_axis_tkeep        : out processing_monitor_tkeep_t;
        m_axis_tuser        : out processing_monitor_tuser_t;
        m_axis_tlast        : out std_logic;

        o_drop_pulse        : out std_logic;
        o_drop_sticky       : out std_logic;
        o_drop_count        : out u32_t
    );
end entity lidar_processing_axis_monitor;

architecture rtl of lidar_processing_axis_monitor is

    signal axis_valid_r  : std_logic := '0';
    signal axis_data_r   : processing_monitor_tdata_t := (others => '0');
    signal axis_user_r   : processing_monitor_tuser_t := (others => '0');
    signal axis_last_r   : std_logic := '0';
    signal drop_pulse_r  : std_logic := '0';
    signal drop_sticky_r : std_logic := '0';
    signal drop_count_r  : u32_t := (others => '0');

begin

    m_axis_tvalid <= axis_valid_r;
    m_axis_tdata  <= axis_data_r;
    m_axis_tkeep  <= (others => '1');
    m_axis_tuser  <= axis_user_r;
    m_axis_tlast  <= axis_last_r;

    o_drop_pulse  <= drop_pulse_r;
    o_drop_sticky <= drop_sticky_r;
    o_drop_count  <= drop_count_r;

    p_monitor : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                axis_valid_r  <= '0';
                axis_data_r   <= (others => '0');
                axis_user_r   <= (others => '0');
                axis_last_r   <= '0';
                drop_pulse_r  <= '0';
                drop_sticky_r <= '0';
                drop_count_r  <= (others => '0');
            else
                drop_pulse_r <= '0';
                if i_clear_diagnostics = '1' then
                    drop_sticky_r <= '0';
                    drop_count_r  <= (others => '0');
                end if;

                -- Empty or accepted storage can take the newest candidate.
                if axis_valid_r = '0' or m_axis_tready = '1' then
                    axis_valid_r <= i_face_event.valid;
                    if i_face_event.valid = '1' then
                        axis_data_r <= fn_pack_processing_monitor_data(
                            i_face_event);
                        axis_user_r <= fn_pack_processing_monitor_user(
                            i_face_event);
                        axis_last_r <= i_face_event.exit_event;
                    else
                        axis_data_r <= (others => '0');
                        axis_user_r <= (others => '0');
                        axis_last_r <= '0';
                    end if;
                elsif i_face_event.valid = '1' then
                    drop_pulse_r  <= '1';
                    drop_sticky_r <= '1';
                    if i_clear_diagnostics = '1' then
                        drop_count_r <= to_unsigned(1, drop_count_r'length);
                    else
                        drop_count_r <= drop_count_r + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_monitor;

end architecture rtl;
