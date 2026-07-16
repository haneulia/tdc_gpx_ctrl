--------------------------------------------------------------------------------
-- tdc_gpx_cell_pipe.vhd
--
-- Cluster 3: Cell pipeline wrapper.
-- Splits incoming event stream by slope (registered demux), then feeds
-- four rising and four falling cell_builder instances.
--
-- Architecture rules:
--   - KEEP_HIERARCHY = "yes"
--   - All sub-modules use direct entity instantiation (entity work.xxx)
--   - Single registered process for slope demux (g_AXIS_CLK_MHZ timing)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_cell_pipe is
    generic (
        g_OUTPUT_WIDTH : natural := 32;  -- 32, 64, or 128
        g_AXIS_CLK_MHZ : positive := 150
    );
    port (
        -- Clock / Reset
        i_clk                   : in  std_logic;
        i_rst_n                 : in  std_logic;

        -- Event input from Cluster 2 (AXI-Stream x4 chips)
        i_evt_sk_tvalid         : in  std_logic_vector(c_N_CHIPS-1 downto 0);
        i_evt_sk_tdata          : in  t_evt_axis_tdata_array;
        i_evt_sk_tuser          : in  t_evt_axis_tuser_array;
        o_evt_sk_tready         : out std_logic_vector(c_N_CHIPS-1 downto 0);

        -- Control / Config
        i_shot_start_per_chip   : in  std_logic_vector(c_N_CHIPS-1 downto 0);
        -- CHAIN P1 (2026-07-16): per-slope lane chip masks (face snapshot).
        -- A chip's builder participates in a slope lane only when its bit
        -- is set: shot_start is gated per (chip, slope) and drain_done
        -- control beats are forwarded only to enabled slopes. Without this,
        -- DEDICATED_2X2 topology made every wrong-slope builder emit a full
        -- blank slice per shot into a face_assembler input FIFO that is
        -- never read (lane-masked chip), polluting the shot_flush_drop
        -- sticky/mask on every single shot. Hit beats addressed to a masked
        -- slope are consumed and dropped with a per-chip sticky (physical
        -- edge misconfiguration visibility). Defaults keep the legacy
        -- both-slopes-active behavior for existing instantiations.
        i_rise_chip_mask        : in  std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');
        i_fall_chip_mask        : in  std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '1');
        i_abort                 : in  std_logic;   -- legacy global abort (default)
        -- #22 Sprint 2: per-slope abort ports. Tie to i_abort if caller wants
        -- legacy coupling; drive separately when slope-independence is
        -- activated (Sprint 3). Defaults to i_abort via '0'-default so
        -- existing instantiations keep compiling.
        i_abort_rise            : in  std_logic := '0';
        i_abort_fall            : in  std_logic := '0';
        i_face_stops_per_chip   : in  unsigned(3 downto 0);
        i_max_hits_cfg          : in  unsigned(2 downto 0);
        -- Physical max-range window from CSR in fixed 5 ns reference ticks.
        -- This module converts it once to AXIS clocks, then every cell_builder
        -- takes its existing per-buffer/drop/output snapshot from that value.
        i_max_range_5ns_ticks   : in  unsigned(15 downto 0);

        -- Rising cell output to Cluster 4 (AXI-Stream x4)
        o_cell_rise_tdata_0     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_rise_tdata_1     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_rise_tdata_2     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_rise_tdata_3     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_rise_tvalid      : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_cell_rise_tlast       : out std_logic_vector(c_N_CHIPS-1 downto 0);
        i_cell_rise_tready      : in  std_logic_vector(c_N_CHIPS-1 downto 0);

        -- Falling cell output to Cluster 4 (AXI-Stream x4)
        o_cell_fall_tdata_0     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_fall_tdata_1     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_fall_tdata_2     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_fall_tdata_3     : out std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);
        o_cell_fall_tvalid      : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_cell_fall_tlast       : out std_logic_vector(c_N_CHIPS-1 downto 0);
        i_cell_fall_tready      : in  std_logic_vector(c_N_CHIPS-1 downto 0);

        -- Status
        o_hit_dropped           : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_hit_fall_dropped      : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_shot_dropped          : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_shot_fall_dropped     : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_slice_timeout         : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_slice_fall_timeout    : out std_logic_vector(c_N_CHIPS-1 downto 0);
        -- Round 11 C: distinct stop_id error cause (separate from hit overflow)
        o_stop_id_error         : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_stop_id_fall_error    : out std_logic_vector(c_N_CHIPS-1 downto 0);
        -- Round 11 item 4: per-chip cell_builder QUARANTINE escalation sticky.
        o_quarantine_escape_rise : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_quarantine_escape_fall : out std_logic_vector(c_N_CHIPS-1 downto 0);
        -- Round 13 follow-up P1 (audit 4번): per-chip tuser(0) = faulted,
        -- co-incident with each chip's cell-stream tlast (non-tlast beats
        -- always 0). Travels with the cell data through the downstream
        -- xpm_fifo_axis so face_assembler can tag degraded rows without
        -- a cross-shot race. Replaces the earlier slice_done_faulted
        -- side-channel which could mis-align across FIFO latency.
        o_cell_rise_tuser       : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_cell_fall_tuser       : out std_logic_vector(c_N_CHIPS-1 downto 0);
        -- CHAIN P1: sticky per chip -- a hit beat addressed a masked slope
        -- and was dropped at the slope demux. Cleared on reset/global abort.
        o_masked_slope_drop_rise : out std_logic_vector(c_N_CHIPS-1 downto 0);
        o_masked_slope_drop_fall : out std_logic_vector(c_N_CHIPS-1 downto 0)
    );
end entity tdc_gpx_cell_pipe;

architecture rtl of tdc_gpx_cell_pipe is

    attribute KEEP_HIERARCHY : string;
    attribute KEEP_HIERARCHY of rtl : architecture is "yes";

    ---------------------------------------------------------------------------
    -- Internal tdata array type (maps to individual output ports)
    ---------------------------------------------------------------------------
    type t_out_tdata_array is array(0 to c_N_CHIPS-1)
        of std_logic_vector(g_OUTPUT_WIDTH-1 downto 0);

    ---------------------------------------------------------------------------
    -- Input skid output (Cluster 2 -> Cluster 3 boundary)
    ---------------------------------------------------------------------------
    signal s_evt_skid_tvalid : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_evt_skid_tready : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_evt_skid_tdata  : t_evt_axis_tdata_array;
    signal s_evt_skid_tuser  : t_evt_axis_tuser_array;
    signal s_max_range_axis_clks : unsigned(15 downto 0);

    ---------------------------------------------------------------------------
    -- Slope-demux registered outputs
    ---------------------------------------------------------------------------
    signal s_rise_valid_r : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_fall_valid_r : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_rise_tdata_r : t_evt_axis_tdata_array;
    signal s_fall_tdata_r : t_evt_axis_tdata_array;
    signal s_rise_tuser_r : t_evt_axis_tuser_array;
    signal s_fall_tuser_r : t_evt_axis_tuser_array;

    ---------------------------------------------------------------------------
    -- Cell builder output tdata arrays
    ---------------------------------------------------------------------------
    signal s_cell_rise_tdata : t_out_tdata_array;
    signal s_cell_fall_tdata : t_out_tdata_array;

    -- cell_builder tready (architecture scope for cross-generate visibility)
    signal s_rise_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_fall_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- Round 13 follow-up P1: per-chip cell_builder m_axis_tuser (1-bit slv).
    -- Each bit i holds chip i's tuser(0) = faulted flag on tlast beat.
    type t_cell_tuser_array is array(0 to c_N_CHIPS - 1)
        of std_logic_vector(0 downto 0);
    signal s_cell_rise_tuser_int : t_cell_tuser_array;
    signal s_cell_fall_tuser_int : t_cell_tuser_array;

    -- #22 Sprint 2: effective abort per slope.
    -- Global i_abort always forces abort on both slopes; additionally
    -- i_abort_rise / i_abort_fall allow slope-independent abort once the
    -- upstream wiring starts driving them (Sprint 3).
    signal s_abort_rise  : std_logic;
    signal s_abort_fall  : std_logic;

    -- CHAIN P1: lane-gated shot_start per (chip, slope) and masked-slope
    -- hit-drop stickies (written by p_slope_demux, single driver).
    signal s_shot_start_rise    : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_shot_start_fall    : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_masked_drop_rise_r : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');
    signal s_masked_drop_fall_r : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');

begin

    assert fn_output_width_supported(g_OUTPUT_WIDTH)
        report "tdc_gpx_cell_pipe: g_OUTPUT_WIDTH must be 32, 64, or 128 for full-keep Phase A"
        severity failure;

    assert fn_range_clk_mhz_supported(g_AXIS_CLK_MHZ)
        report "tdc_gpx_cell_pipe: g_AXIS_CLK_MHZ must be 50, 100, 125, 150, or 200"
        severity failure;

    s_max_range_axis_clks <= fn_range_5ns_ticks_to_clks(
        i_max_range_5ns_ticks, g_AXIS_CLK_MHZ);

    ---------------------------------------------------------------------------
    -- Slope demux (REGISTERED, with tready backpressure)
    -- slope = tuser(0): 1 = rising, 0 = falling
    -- drain_done = tuser(7): control beat passed to both slopes
    --
    -- AXI-Stream handshake: upstream valid held until downstream ready.
    -- Per-chip tready: for drain_done beats, both rise+fall must be ready.
    -- For hit beats, only the target slope's builder must be ready.
    ---------------------------------------------------------------------------

    -- Effective per-slope abort (Sprint 2): global takes priority, additive OR
    s_abort_rise <= i_abort or i_abort_rise;
    s_abort_fall <= i_abort or i_abort_fall;

    -- CHAIN P1: a builder only opens a shot for its own lane. Masks are
    -- face-snapshot values (stable at the registered shot_start pulse).
    s_shot_start_rise <= i_shot_start_per_chip and i_rise_chip_mask;
    s_shot_start_fall <= i_shot_start_per_chip and i_fall_chip_mask;

    o_masked_slope_drop_rise <= s_masked_drop_rise_r;
    o_masked_slope_drop_fall <= s_masked_drop_fall_r;

    -- Per-chip: can this chip's demux accept new input?
    -- drain_done goes to both → need both ready
    -- hit goes to one → need that one ready
    gen_input_skid : for i in 0 to c_N_CHIPS - 1 generate
    begin
        u_evt_in_skid : entity work.tdc_gpx_skid_buffer
            generic map (
                g_DATA_WIDTH => c_EVT_AXIS_PACK_WIDTH
            )
            port map (
                i_clk                   => i_clk,
                i_rst_n                 => i_rst_n,
                i_flush                 => i_abort,
                i_s_valid               => i_evt_sk_tvalid(i),
                o_s_ready               => o_evt_sk_tready(i),
                i_s_data                => i_evt_sk_tdata(i) & i_evt_sk_tuser(i),
                o_m_valid               => s_evt_skid_tvalid(i),
                i_m_ready               => s_evt_skid_tready(i),
                o_m_data(c_EVT_AXIS_PACK_WIDTH - 1 downto c_EVT_AXIS_TUSER_WIDTH) => s_evt_skid_tdata(i),
                o_m_data(c_EVT_AXIS_TUSER_WIDTH - 1 downto 0) => s_evt_skid_tuser(i)
            );
    end generate gen_input_skid;

    gen_demux_ready : for i in 0 to c_N_CHIPS - 1 generate
        signal s_rise_free : std_logic;
        signal s_fall_free : std_logic;
    begin
        s_rise_free <= '1' when s_abort_rise = '1'
                              or s_rise_valid_r(i) = '0'
                              or s_rise_tready(i) = '1' else '0';
        s_fall_free <= '1' when s_abort_fall = '1'
                              or s_fall_valid_r(i) = '0'
                              or s_fall_tready(i) = '1' else '0';

        s_evt_skid_tready(i) <= '1' when s_evt_skid_tvalid(i) = '0'
                       else s_rise_free and s_fall_free when s_evt_skid_tuser(i)(7) = '1'
                       else s_rise_free when s_evt_skid_tuser(i)(0) = '1'
                       else s_fall_free;
    end generate gen_demux_ready;

    p_slope_demux : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                s_rise_valid_r <= (others => '0');
                s_fall_valid_r <= (others => '0');
                s_masked_drop_rise_r <= (others => '0');
                s_masked_drop_fall_r <= (others => '0');
            else
                for i in 0 to c_N_CHIPS - 1 loop
                    -- Clear valid on downstream handshake or matching slope abort.
                    if s_abort_rise = '1' then
                        s_rise_valid_r(i) <= '0';
                    elsif s_rise_valid_r(i) = '1' and s_rise_tready(i) = '1' then
                        s_rise_valid_r(i) <= '0';
                    end if;
                    if s_abort_fall = '1' then
                        s_fall_valid_r(i) <= '0';
                    elsif s_fall_valid_r(i) = '1' and s_fall_tready(i) = '1' then
                        s_fall_valid_r(i) <= '0';
                    end if;

                    -- Load only beats popped from the input skid. Aborted-slope
                    -- beats are consumed but intentionally not re-issued.
                    -- CHAIN P1: lane-masked slopes are treated the same way --
                    -- drain_done control beats simply skip a masked slope
                    -- (broadcast by design, not an error), while a HIT beat
                    -- addressed to a masked slope is dropped WITH a sticky
                    -- (it indicates a physical edge misconfiguration).
                    if s_evt_skid_tvalid(i) = '1' and s_evt_skid_tready(i) = '1' then
                        if s_evt_skid_tuser(i)(7) = '1' then
                            if s_abort_rise = '0' and i_rise_chip_mask(i) = '1' then
                                s_rise_valid_r(i) <= '1';
                                s_rise_tdata_r(i) <= s_evt_skid_tdata(i);
                                s_rise_tuser_r(i) <= s_evt_skid_tuser(i);
                            end if;
                            if s_abort_fall = '0' and i_fall_chip_mask(i) = '1' then
                                s_fall_valid_r(i) <= '1';
                                s_fall_tdata_r(i) <= s_evt_skid_tdata(i);
                                s_fall_tuser_r(i) <= s_evt_skid_tuser(i);
                            end if;
                        elsif s_evt_skid_tuser(i)(0) = '1' then
                            if i_rise_chip_mask(i) = '0' then
                                s_masked_drop_rise_r(i) <= '1';
                            elsif s_abort_rise = '0' then
                                s_rise_valid_r(i) <= '1';
                                s_rise_tdata_r(i) <= s_evt_skid_tdata(i);
                                s_rise_tuser_r(i) <= s_evt_skid_tuser(i);
                            end if;
                        else
                            if i_fall_chip_mask(i) = '0' then
                                s_masked_drop_fall_r(i) <= '1';
                            elsif s_abort_fall = '0' then
                                s_fall_valid_r(i) <= '1';
                                s_fall_tdata_r(i) <= s_evt_skid_tdata(i);
                                s_fall_tuser_r(i) <= s_evt_skid_tuser(i);
                            end if;
                        end if;
                    end if;
                end loop;
            end if;
        end if;
    end process p_slope_demux;

    ---------------------------------------------------------------------------
    -- Cell builders (rising + falling) -- one generate for all 4 chips
    ---------------------------------------------------------------------------
    -- Round 11 item 8: the pre-Round-11 comment claimed cell_builder's
    -- o_s_axis_tready was monitored but NOT used for upstream backpressure.
    -- That was incorrect — the backpressure chain IS wired through:
    --
    --   cell_builder.o_s_axis_tready → s_rise_tready / s_fall_tready
    --     → s_rise_free / s_fall_free (= slot-empty OR downstream-ready)
    --     → s_evt_skid_tready (gated by skid-valid + beat-type)
    --     → gen_input_skid registered o_evt_sk_tready boundary
    --     → o_evt_sk_tready (actual upstream backpressure)
    --
    -- gen_input_skid absorbs the registered-ready latency at the module
    -- boundary. The slope holding registers then provide the final
    -- registered handoff to each cell_builder.
    --
    -- The warning assert below is retained as a DIAGNOSTIC: it fires
    -- whenever a beat waits for cell_builder (e.g. arrived before
    -- shot_start, or during a DROP→QUARANTINE transition). Not an error.
    ---------------------------------------------------------------------------
    gen_chip : for i in 0 to c_N_CHIPS-1 generate
    begin

        -- Simulation-only timing observability. Fires when a beat is held
        -- in the cell_pipe skid waiting for cell_builder readiness. This
        -- is the intended behavior of the first-beat absorb register and
        -- not a data-loss condition (upstream backpressure kicks in via
        -- the input skid, so the slot cannot be overrun).
        -- synthesis translate_off
        p_drop_assert : process(i_clk)
        begin
            if rising_edge(i_clk) then
                assert not (s_rise_valid_r(i) = '1' and s_rise_tready(i) = '0')
                    report "INFO: cell_pipe rise(" & integer'image(i) & ") skid holding beat for cell_builder"
                    severity note;
                assert not (s_fall_valid_r(i) = '1' and s_fall_tready(i) = '0')
                    report "INFO: cell_pipe fall(" & integer'image(i) & ") skid holding beat for cell_builder"
                    severity note;
            end if;
        end process p_drop_assert;
        -- synthesis translate_on

        -- Rising-slope cell builder
        u_cell_bld_rise : entity work.tdc_gpx_cell_builder
            generic map (
                g_CHIP_ID     => i,
                g_TDATA_WIDTH => g_OUTPUT_WIDTH
            )
            port map (
                i_clk               => i_clk,
                i_rst_n             => i_rst_n,
                i_s_axis_tvalid     => s_rise_valid_r(i),
                i_s_axis_tdata      => s_rise_tdata_r(i),
                i_s_axis_tuser      => s_rise_tuser_r(i),
                o_s_axis_tready     => s_rise_tready(i),
                i_shot_start        => s_shot_start_rise(i),
                i_abort             => s_abort_rise,
                i_stops_per_chip    => i_face_stops_per_chip,
                i_max_hits_cfg      => i_max_hits_cfg,
                i_max_range_axis_clks => s_max_range_axis_clks,
                o_m_axis_tdata      => s_cell_rise_tdata(i),
                o_m_axis_tvalid     => o_cell_rise_tvalid(i),
                o_m_axis_tlast      => o_cell_rise_tlast(i),
                o_m_axis_tuser      => s_cell_rise_tuser_int(i),  -- P1 rework
                i_m_axis_tready     => i_cell_rise_tready(i),
                o_slice_done        => open,
                o_hit_dropped_any   => o_hit_dropped(i),
                o_shot_dropped      => o_shot_dropped(i),
                o_slice_timeout     => o_slice_timeout(i),
                o_stop_id_error     => o_stop_id_error(i),
                o_quarantine_escape_sticky => o_quarantine_escape_rise(i)
            );

        -- Falling-slope cell builder
        u_cell_bld_fall : entity work.tdc_gpx_cell_builder
            generic map (
                g_CHIP_ID     => i,
                g_TDATA_WIDTH => g_OUTPUT_WIDTH
            )
            port map (
                i_clk               => i_clk,
                i_rst_n             => i_rst_n,
                i_s_axis_tvalid     => s_fall_valid_r(i),
                i_s_axis_tdata      => s_fall_tdata_r(i),
                i_s_axis_tuser      => s_fall_tuser_r(i),
                o_s_axis_tready     => s_fall_tready(i),
                i_shot_start        => s_shot_start_fall(i),
                i_abort             => s_abort_fall,
                i_stops_per_chip    => i_face_stops_per_chip,
                i_max_hits_cfg      => i_max_hits_cfg,
                i_max_range_axis_clks => s_max_range_axis_clks,
                o_m_axis_tdata      => s_cell_fall_tdata(i),
                o_m_axis_tvalid     => o_cell_fall_tvalid(i),
                o_m_axis_tlast      => o_cell_fall_tlast(i),
                o_m_axis_tuser      => s_cell_fall_tuser_int(i),  -- P1 rework
                i_m_axis_tready     => i_cell_fall_tready(i),
                o_slice_done        => open,
                o_hit_dropped_any   => o_hit_fall_dropped(i),
                o_shot_dropped      => o_shot_fall_dropped(i),
                o_slice_timeout     => o_slice_fall_timeout(i),
                o_stop_id_error     => o_stop_id_fall_error(i),
                o_quarantine_escape_sticky => o_quarantine_escape_fall(i)
            );

    end generate gen_chip;

    ---------------------------------------------------------------------------
    -- tdata output mapping: internal array -> individual ports
    ---------------------------------------------------------------------------
    o_cell_rise_tdata_0 <= s_cell_rise_tdata(0);
    o_cell_rise_tdata_1 <= s_cell_rise_tdata(1);
    o_cell_rise_tdata_2 <= s_cell_rise_tdata(2);
    o_cell_rise_tdata_3 <= s_cell_rise_tdata(3);

    o_cell_fall_tdata_0 <= s_cell_fall_tdata(0);
    o_cell_fall_tdata_1 <= s_cell_fall_tdata(1);
    o_cell_fall_tdata_2 <= s_cell_fall_tdata(2);
    o_cell_fall_tdata_3 <= s_cell_fall_tdata(3);

    -- Round 13 follow-up P1: pack per-chip tuser(0) bit into output vector
    gen_tuser_map : for i in 0 to c_N_CHIPS - 1 generate
        o_cell_rise_tuser(i) <= s_cell_rise_tuser_int(i)(0);
        o_cell_fall_tuser(i) <= s_cell_fall_tuser_int(i)(0);
    end generate gen_tuser_map;

end architecture rtl;
