--------------------------------------------------------------------------------
-- tdc_gpx_decode_pipe.vhd
--
-- Cluster 2: Decode pipeline wrapper.
-- Per-chip pipeline: decoder_i_mode -> skid_dec -> raw_event_builder -> skid_evt.
-- Four identical lanes generated via gen_chip.
--
-- Architecture rules:
--   - Pure structural: instances + concurrent assignments only, NO processes
--   - KEEP_HIERARCHY = "yes"
--   - All sub-modules use direct entity instantiation (entity work.xxx)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_decode_pipe is
    port (
        i_clk   : in std_logic;
        i_rst_n : in std_logic;

        -- Input from Cluster 1 (AXI-Stream x4, from chip_ctrl skid outputs)
        i_raw_sk_tvalid : in  std_logic_vector(c_N_CHIPS-1 downto 0);
        i_raw_sk_tdata  : in  t_slv32_array;
        i_raw_sk_tuser  : in  t_slv8_array;
        o_raw_sk_tready : out std_logic_vector(c_N_CHIPS-1 downto 0);

        -- Context from Cluster 1 (per-chip shot sequence)
        i_chip_shot_seq : in t_shot_seq_array;

        -- Config (latched at packet_start by caller)
        i_face_stops_per_chip : in unsigned(3 downto 0);

        -- Output to Cluster 3 (AXI-Stream x4, skid buffer outputs)
        o_evt_sk_tvalid : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_evt_sk_tdata  : out t_slv32_array;
        o_evt_sk_tuser  : out t_slv16_array;
        i_evt_sk_tready : in  std_logic_vector(c_N_CHIPS-1 downto 0);  -- backpressure from cell_pipe

        -- Status
        o_stop_id_error : out std_logic_vector(c_N_CHIPS-1 downto 0)
    );
end entity tdc_gpx_decode_pipe;

architecture rtl of tdc_gpx_decode_pipe is

    attribute KEEP_HIERARCHY : string;
    attribute KEEP_HIERARCHY of rtl : architecture is "yes";

    ---------------------------------------------------------------------------
    -- decoder_i_mode output -> skid_dec input
    ---------------------------------------------------------------------------
    signal s_dec_axis_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_dec_axis_tdata  : t_slv32_array;
    signal s_dec_axis_tuser  : t_slv8_array;
    signal s_dec_axis_tready : std_logic_vector(c_N_CHIPS-1 downto 0);

    ---------------------------------------------------------------------------
    -- skid_dec output -> event_bld input
    ---------------------------------------------------------------------------
    signal s_dec_sk_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_dec_sk_tdata  : t_slv32_array;
    signal s_dec_sk_tuser  : t_slv8_array;
    signal s_dec_sk_tready : std_logic_vector(c_N_CHIPS-1 downto 0);

    ---------------------------------------------------------------------------
    -- event_bld output -> skid_evt input
    ---------------------------------------------------------------------------
    signal s_evt_axis_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_evt_axis_tdata  : t_slv32_array;
    signal s_evt_axis_tuser  : t_slv16_array;
    signal s_evt_axis_tready : std_logic_vector(c_N_CHIPS-1 downto 0);

begin

    ---------------------------------------------------------------------------
    -- Per-chip decode pipeline (4 instances of each stage)
    ---------------------------------------------------------------------------
    gen_chip : for i in 0 to c_N_CHIPS-1 generate

        -- Stage 1: I-Mode 28-bit field extraction
        u_decode : entity work.tdc_gpx_decoder_i_mode
            port map (
                i_clk           => i_clk,
                i_rst_n         => i_rst_n,
                i_s_axis_tvalid => i_raw_sk_tvalid(i),
                i_s_axis_tdata  => i_raw_sk_tdata(i),
                i_s_axis_tuser  => i_raw_sk_tuser(i),
                o_s_axis_tready => o_raw_sk_tready(i),
                o_m_axis_tvalid => s_dec_axis_tvalid(i),
                o_m_axis_tdata  => s_dec_axis_tdata(i),
                o_m_axis_tuser  => s_dec_axis_tuser(i),
                i_m_axis_tready => s_dec_axis_tready(i)
            );

        -- Stage 2: Skid buffer decode -> event_bld (40b: 32b tdata & 8b tuser)
        u_sk_dec : entity work.tdc_gpx_skid_buffer
            generic map (
                g_DATA_WIDTH => 40
            )
            port map (
                i_clk                  => i_clk,
                i_rst_n                => i_rst_n,
                i_flush                => '0',
                i_s_valid              => s_dec_axis_tvalid(i),
                o_s_ready              => s_dec_axis_tready(i),
                i_s_data               => s_dec_axis_tdata(i) & s_dec_axis_tuser(i),
                o_m_valid              => s_dec_sk_tvalid(i),
                i_m_ready              => s_dec_sk_tready(i),
                o_m_data(39 downto 8)  => s_dec_sk_tdata(i),
                o_m_data(7 downto 0)   => s_dec_sk_tuser(i)
            );

        -- Stage 3: Raw event builder (hit enrichment)
        u_event_bld : entity work.tdc_gpx_raw_event_builder
            port map (
                i_clk            => i_clk,
                i_rst_n          => i_rst_n,
                i_s_axis_tvalid  => s_dec_sk_tvalid(i),
                i_s_axis_tdata   => s_dec_sk_tdata(i),
                i_s_axis_tuser   => s_dec_sk_tuser(i),
                o_s_axis_tready  => s_dec_sk_tready(i),
                i_chip_id        => to_unsigned(i, 2),
                i_shot_seq       => i_chip_shot_seq(i),
                i_stops_per_chip => i_face_stops_per_chip,
                o_m_axis_tvalid  => s_evt_axis_tvalid(i),
                o_m_axis_tdata   => s_evt_axis_tdata(i),
                o_m_axis_tuser   => s_evt_axis_tuser(i),
                i_m_axis_tready  => s_evt_axis_tready(i),
                o_stop_id_error  => o_stop_id_error(i)
            );

        -- Stage 4: Skid buffer event_bld -> cell_pipe (48b: 32b tdata & 16b tuser)
        u_sk_evt : entity work.tdc_gpx_skid_buffer
            generic map (
                g_DATA_WIDTH => 48
            )
            port map (
                i_clk                   => i_clk,
                i_rst_n                 => i_rst_n,
                i_flush                 => '0',
                i_s_valid               => s_evt_axis_tvalid(i),
                o_s_ready               => s_evt_axis_tready(i),
                i_s_data                => s_evt_axis_tdata(i) & s_evt_axis_tuser(i),
                o_m_valid               => o_evt_sk_tvalid(i),
                i_m_ready               => i_evt_sk_tready(i),  -- backpressure from cell_pipe
                o_m_data(47 downto 16)  => o_evt_sk_tdata(i),
                o_m_data(15 downto 0)   => o_evt_sk_tuser(i)
            );

    end generate gen_chip;

end architecture rtl;
