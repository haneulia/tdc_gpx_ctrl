library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library xpm;
use xpm.vcomponents.all;

use work.lidar_build_pkg.all;

-- Ready/valid payload gateway with one explicit implementation per clock mode.
-- SYNC requires both ports to use the same physical clock net. ASYNC uses an
-- XPM asynchronous FIFO and never samples a multi-bit payload independently.
entity lidar_stream_gateway is
    generic (
        G_WIDTH      : positive := 32;
        G_FIFO_DEPTH : positive := 16;
        G_CLOCK_MODE : stream_clock_mode_t := STREAM_CLOCK_ASYNC
    );
    port (
        i_source_clk       : in  std_logic;
        i_source_rst_n     : in  std_logic;
        i_source_valid     : in  std_logic;
        o_source_ready     : out std_logic;
        i_source_data      : in  std_logic_vector(G_WIDTH - 1 downto 0);

        i_destination_clk   : in  std_logic;
        i_destination_rst_n : in  std_logic;
        o_destination_valid : out std_logic;
        i_destination_ready : in  std_logic;
        o_destination_data  : out std_logic_vector(G_WIDTH - 1 downto 0);

        o_source_reset_busy      : out std_logic;
        o_destination_reset_busy : out std_logic;
        -- Diagnostic aggregate only. Do not consume this signal in either
        -- clock domain; use the matching domain-specific output above.
        o_reset_busy              : out std_logic
    );
end entity lidar_stream_gateway;

architecture rtl of lidar_stream_gateway is

    signal source_ready_c : std_logic;
    signal destination_valid_c : std_logic;
    signal destination_data_c : std_logic_vector(G_WIDTH - 1 downto 0);
    signal source_reset_busy_c : std_logic;
    signal destination_reset_busy_c : std_logic;
    signal reset_busy_c : std_logic;

    function fn_is_power_of_two(value : positive) return boolean is
        variable working : positive := value;
    begin
        while working > 1 loop
            if working mod 2 /= 0 then
                return false;
            end if;
            working := working / 2;
        end loop;
        return true;
    end function fn_is_power_of_two;

begin

    o_source_ready     <= source_ready_c;
    o_destination_valid <= destination_valid_c;
    o_destination_data <= destination_data_c;
    o_source_reset_busy <= source_reset_busy_c;
    o_destination_reset_busy <= destination_reset_busy_c;
    o_reset_busy       <= reset_busy_c;

    assert G_CLOCK_MODE = STREAM_CLOCK_SYNC or
           (G_FIFO_DEPTH >= 16 and fn_is_power_of_two(G_FIFO_DEPTH))
        report "V2-CDC-001 async FIFO depth must be a power of two >= 16"
        severity failure;

    gen_sync : if G_CLOCK_MODE = STREAM_CLOCK_SYNC generate
        signal valid_r : std_logic := '0';
        signal data_r  : std_logic_vector(G_WIDTH - 1 downto 0) :=
            (others => '0');
    begin
        -- A single registered elastic slot. The only combinational path is the
        -- local ready term; payload is always registered before consumption.
        source_ready_c <= (not valid_r or i_destination_ready)
            and i_source_rst_n and i_destination_rst_n;
        destination_valid_c <= valid_r and i_source_rst_n
            and i_destination_rst_n;
        destination_data_c <= data_r;
        source_reset_busy_c <= not i_source_rst_n or not i_destination_rst_n;
        destination_reset_busy_c <= source_reset_busy_c;
        reset_busy_c <= source_reset_busy_c;

        p_sync_slot : process (
            i_source_clk, i_source_rst_n, i_destination_rst_n)
        begin
            if i_source_rst_n = '0' or i_destination_rst_n = '0' then
                valid_r <= '0';
                data_r  <= (others => '0');
            elsif rising_edge(i_source_clk) then
                if source_ready_c = '1' then
                    valid_r <= i_source_valid;
                    if i_source_valid = '1' then
                        data_r <= i_source_data;
                    end if;
                end if;
            end if;
        end process p_sync_slot;
    end generate gen_sync;

    gen_async : if G_CLOCK_MODE = STREAM_CLOCK_ASYNC generate
        constant C_DEST_RESET_SYNC_STAGES : positive := 4;
        constant C_RESET_HOLD_CLKS : positive := 8;
        signal fifo_full : std_logic;
        signal fifo_empty : std_logic;
        signal fifo_dout : std_logic_vector(G_WIDTH - 1 downto 0);
        signal fifo_rst : std_logic;
        signal fifo_rst_r : std_logic := '1';
        signal wr_rst_busy : std_logic;
        signal rd_rst_busy : std_logic;
        signal destination_reset_n_sync_s : std_logic := '0';
        signal reset_count_r : natural range 0 to C_RESET_HOLD_CLKS :=
            C_RESET_HOLD_CLKS;
    begin
        source_ready_c <= not fifo_full and not wr_rst_busy and not fifo_rst;
        destination_valid_c <= not fifo_empty and not rd_rst_busy
            and i_destination_rst_n;
        destination_data_c <= fifo_dout;
        source_reset_busy_c <= fifo_rst or wr_rst_busy;
        destination_reset_busy_c <= rd_rst_busy or not i_destination_rst_n;
        reset_busy_c <= source_reset_busy_c or destination_reset_busy_c;

        -- 목적지 리셋은 데이터처럼 4단 XPM 동기화기를 거쳐 소스 클럭
        -- 도메인으로 전달한다. 리셋 입력은 최소 4개의 소스 클럭 주기 동안
        -- LOW를 유지해야 하며, 시스템의 proc_sys_reset은 이 조건을 만족한다.
        -- 목적지 출력 valid는 원래 리셋으로 즉시 차단되므로 동기화 지연 중
        -- 오래된 payload가 소비되지 않는다.
        u_destination_reset_sync : xpm_cdc_single
            generic map (
                DEST_SYNC_FF   => C_DEST_RESET_SYNC_STAGES,
                INIT_SYNC_FF   => 1,
                SIM_ASSERT_CHK => 1,
                -- 같은 목적지 리셋이 여러 Gateway로 분기되더라도 각 XPM이
                -- 원래 클럭에서 먼저 등록한 단일 launch 경로만 교차시킨다.
                SRC_INPUT_REG  => 1
            )
            port map (
                src_clk  => i_destination_clk,
                src_in   => i_destination_rst_n,
                dest_clk => i_source_clk,
                dest_out => destination_reset_n_sync_s
            );

        -- xpm_fifo_async.rst는 wr_clk에 동기여야 한다. 따라서 FIFO 리셋과
        -- 유지 카운터는 소스 클럭 순차회로만으로 구동한다. 목적지 또는
        -- 소스 리셋을 감지하면 기존 payload를 폐기하고 8클럭을 더 유지한
        -- 뒤 새 전송을 허용한다.
        p_fifo_reset : process (i_source_clk)
        begin
            if rising_edge(i_source_clk) then
                if i_source_rst_n = '0' or
                   destination_reset_n_sync_s = '0' then
                    fifo_rst_r <= '1';
                    reset_count_r <= C_RESET_HOLD_CLKS;
                elsif reset_count_r > 1 then
                    fifo_rst_r <= '1';
                    reset_count_r <= reset_count_r - 1;
                elsif reset_count_r = 1 then
                    fifo_rst_r <= '0';
                    reset_count_r <= 0;
                else
                    fifo_rst_r <= '0';
                end if;
            end if;
        end process p_fifo_reset;

        fifo_rst <= fifo_rst_r;

        u_async_fifo : xpm_fifo_async
            generic map (
                CDC_SYNC_STAGES   => 2,
                -- Let XPM keep small control FIFOs in distributed RAM and
                -- move topology-sized result FIFOs into BRAM. This avoids a
                -- user-facing memory-selection generic and follows capacity.
                FIFO_MEMORY_TYPE  => "auto",
                FIFO_READ_LATENCY => 0,
                FIFO_WRITE_DEPTH  => G_FIFO_DEPTH,
                READ_DATA_WIDTH   => G_WIDTH,
                READ_MODE         => "fwft",
                RELATED_CLOCKS    => 0,
                SIM_ASSERT_CHK    => 1,
                WRITE_DATA_WIDTH  => G_WIDTH
            )
            port map (
                wr_clk        => i_source_clk,
                wr_en         => i_source_valid and source_ready_c,
                din           => i_source_data,
                full          => fifo_full,
                rd_clk        => i_destination_clk,
                rd_en         => destination_valid_c and i_destination_ready,
                dout          => fifo_dout,
                empty         => fifo_empty,
                rst           => fifo_rst,
                wr_rst_busy   => wr_rst_busy,
                rd_rst_busy   => rd_rst_busy,
                almost_empty  => open,
                almost_full   => open,
                data_valid    => open,
                dbiterr       => open,
                overflow      => open,
                prog_empty    => open,
                prog_full     => open,
                rd_data_count => open,
                sbiterr       => open,
                underflow     => open,
                wr_ack        => open,
                wr_data_count => open,
                sleep         => '0',
                injectdbiterr => '0',
                injectsbiterr => '0'
            );
    end generate gen_async;

end architecture rtl;
