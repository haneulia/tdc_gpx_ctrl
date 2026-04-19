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
--   - Single registered process for slope demux (200 MHz timing)
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.tdc_gpx_pkg.all;

entity tdc_gpx_cell_pipe is
    generic (
        g_OUTPUT_WIDTH : natural := 32
    );
    port (
        -- Clock / Reset
        i_clk                   : in  std_logic;
        i_rst_n                 : in  std_logic;

        -- Event input from Cluster 2 (AXI-Stream x4 chips)
        i_evt_sk_tvalid         : in  std_logic_vector(c_N_CHIPS-1 downto 0);
        i_evt_sk_tdata          : in  t_slv32_array;
        i_evt_sk_tuser          : in  t_slv16_array;
        o_evt_sk_tready         : out std_logic_vector(c_N_CHIPS-1 downto 0);

        -- Control / Config
        i_shot_start_per_chip   : in  std_logic_vector(c_N_CHIPS-1 downto 0);
        i_abort                 : in  std_logic;   -- legacy global abort (default)
        -- #22 Sprint 2: per-slope abort ports. Tie to i_abort if caller wants
        -- legacy coupling; drive separately when slope-independence is
        -- activated (Sprint 3). Defaults to i_abort via '0'-default so
        -- existing instantiations keep compiling.
        i_abort_rise            : in  std_logic := '0';
        i_abort_fall            : in  std_logic := '0';
        i_face_stops_per_chip   : in  unsigned(3 downto 0);
        i_max_hits_cfg          : in  unsigned(2 downto 0);

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
        o_quarantine_escape_fall : out std_logic_vector(c_N_CHIPS-1 downto 0)
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
    -- Slope-demux registered outputs
    ---------------------------------------------------------------------------
    signal s_rise_valid_r : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_fall_valid_r : std_logic_vector(c_N_CHIPS-1 downto 0);
    signal s_rise_tdata_r : t_slv32_array;
    signal s_fall_tdata_r : t_slv32_array;
    signal s_rise_tuser_r : t_slv16_array;
    signal s_fall_tuser_r : t_slv16_array;

    ---------------------------------------------------------------------------
    -- Cell builder output tdata arrays
    ---------------------------------------------------------------------------
    signal s_cell_rise_tdata : t_out_tdata_array;
    signal s_cell_fall_tdata : t_out_tdata_array;

    -- cell_builder tready (architecture scope for cross-generate visibility)
    signal s_rise_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);
    signal s_fall_tready : std_logic_vector(c_N_CHIPS - 1 downto 0);

    -- #22 Sprint 2: effective abort per slope.
    -- Global i_abort always forces abort on both slopes; additionally
    -- i_abort_rise / i_abort_fall allow slope-independent abort once the
    -- upstream wiring starts driving them (Sprint 3).
    signal s_abort_rise  : std_logic;
    signal s_abort_fall  : std_logic;

    -- Registered tready for 200MHz timing closure.
    -- Upstream skid buffer (u_sk_evt in decode_pipe) absorbs 1-cycle latency.
    signal s_can_accept_r : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

begin

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

    -- Per-chip: can this chip's demux accept new input?
    -- drain_done goes to both → need both ready
    -- hit goes to one → need that one ready
    gen_tready : for i in 0 to c_N_CHIPS - 1 generate
        signal s_rise_free      : std_logic;
        signal s_fall_free      : std_logic;
        signal s_can_accept_comb : std_logic;
    begin
        s_rise_free <= '1' when s_rise_valid_r(i) = '0' or s_rise_tready(i) = '1' else '0';
        s_fall_free <= '1' when s_fall_valid_r(i) = '0' or s_fall_tready(i) = '1' else '0';

        s_can_accept_comb <= '1' when s_rise_free = '1' and s_fall_free = '1'
                              and (i_evt_sk_tvalid(i) = '0' or i_evt_sk_tuser(i)(7) = '1')
                   else s_rise_free when i_evt_sk_tvalid(i) = '1' and i_evt_sk_tuser(i)(7) = '0'
                                         and i_evt_sk_tuser(i)(0) = '1'
                   else s_fall_free when i_evt_sk_tvalid(i) = '1' and i_evt_sk_tuser(i)(7) = '0'
                                         and i_evt_sk_tuser(i)(0) = '0'
                   else s_rise_free and s_fall_free;

        -- Register tready for timing (breaks combinational chain to upstream)
        p_tready_reg : process(i_clk)
        begin
            if rising_edge(i_clk) then
                if i_rst_n = '0' or i_abort = '1' then
                    s_can_accept_r(i) <= '0';
                else
                    s_can_accept_r(i) <= s_can_accept_comb;
                end if;
            end if;
        end process;

        o_evt_sk_tready(i) <= s_can_accept_r(i);
    end generate gen_tready;

    p_slope_demux : process(i_clk)
        variable v_can_load : boolean;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' or i_abort = '1' then
                s_rise_valid_r <= (others => '0');
                s_fall_valid_r <= (others => '0');
            else
                for i in 0 to c_N_CHIPS - 1 loop
                    -- Clear valid on downstream handshake
                    if s_rise_valid_r(i) = '1' and s_rise_tready(i) = '1' then
                        s_rise_valid_r(i) <= '0';
                    end if;
                    if s_fall_valid_r(i) = '1' and s_fall_tready(i) = '1' then
                        s_fall_valid_r(i) <= '0';
                    end if;

                    -- Load new data: beat-type-aware backpressure
                    if i_evt_sk_tuser(i)(7) = '1' then
                        -- drain_done: goes to both slopes, need both free
                        v_can_load := (s_rise_valid_r(i) = '0' or s_rise_tready(i) = '1')
                                  and (s_fall_valid_r(i) = '0' or s_fall_tready(i) = '1');
                    elsif i_evt_sk_tuser(i)(0) = '1' then
                        -- rising hit: only need rise slot free
                        v_can_load := (s_rise_valid_r(i) = '0' or s_rise_tready(i) = '1');
                    else
                        -- falling hit: only need fall slot free
                        v_can_load := (s_fall_valid_r(i) = '0' or s_fall_tready(i) = '1');
                    end if;

                    if v_can_load and i_evt_sk_tvalid(i) = '1' then
                        s_rise_valid_r(i) <= i_evt_sk_tuser(i)(0) or i_evt_sk_tuser(i)(7);
                        s_fall_valid_r(i) <= (not i_evt_sk_tuser(i)(0)) or i_evt_sk_tuser(i)(7);
                        s_rise_tdata_r(i) <= i_evt_sk_tdata(i);
                        s_fall_tdata_r(i) <= i_evt_sk_tdata(i);
                        s_rise_tuser_r(i) <= i_evt_sk_tuser(i);
                        s_fall_tuser_r(i) <= i_evt_sk_tuser(i);
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
    --     → s_can_accept_comb (gated by incoming-valid + beat-type)
    --     → s_can_accept_r  (1-cycle registered for timing closure)
    --     → o_evt_sk_tready (actual upstream backpressure)
    --
    -- The 1-cycle register is tolerated by the 1-slot holding register
    -- (s_rise_valid_r / s_fall_valid_r) inside p_slope_demux, which acts
    -- as the first-beat absorb skid recommended by the review:
    --   - If cell_builder is in ST_C_IDLE and a beat arrives before
    --     shot_start, it is latched here and held until cell_builder
    --     enters ST_C_ACTIVE. No data loss.
    --   - Upstream is throttled via o_evt_sk_tready while the slot is
    --     full, so subsequent beats cannot overwrite the held one.
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
        -- s_can_accept_r, so the slot cannot be overrun).
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
                i_shot_start        => i_shot_start_per_chip(i),
                i_abort             => s_abort_rise,
                i_stops_per_chip    => i_face_stops_per_chip,
                i_max_hits_cfg      => i_max_hits_cfg,
                o_m_axis_tdata      => s_cell_rise_tdata(i),
                o_m_axis_tvalid     => o_cell_rise_tvalid(i),
                o_m_axis_tlast      => o_cell_rise_tlast(i),
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
                i_shot_start        => i_shot_start_per_chip(i),
                i_abort             => s_abort_fall,
                i_stops_per_chip    => i_face_stops_per_chip,
                i_max_hits_cfg      => i_max_hits_cfg,
                o_m_axis_tdata      => s_cell_fall_tdata(i),
                o_m_axis_tvalid     => o_cell_fall_tvalid(i),
                o_m_axis_tlast      => o_cell_fall_tlast(i),
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

end architecture rtl;
