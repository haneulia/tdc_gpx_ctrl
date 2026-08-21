library ieee;
use ieee.std_logic_1164.all;

--------------------------------------------------------------------------------
-- AXI4-Lite single-outstanding transaction engine for the 9-bit CSR32 map.
--
-- AW and W are buffered independently, so either channel may arrive first.
-- Read and write channels are independent and may progress concurrently.
-- B and R payloads remain stable until the corresponding READY handshake.
--------------------------------------------------------------------------------
entity axil_fsm_32 is
    generic (
        num_ctl_regs  : integer := 32;
        num_stat_regs : integer := 32;
        num_data_bits : integer := 32;
        en_secure_chk : std_logic := '0';
        en_priv_chk   : std_logic := '0'
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        awaddr  : in  std_logic_vector(8 downto 0);
        awprot  : in  std_logic_vector(2 downto 0);
        awvalid : in  std_logic;
        awready : out std_logic;

        wdata   : in  std_logic_vector(num_data_bits-1 downto 0);
        wstrb   : in  std_logic_vector((num_data_bits/8)-1 downto 0);
        wvalid  : in  std_logic;
        wready  : out std_logic;

        bresp   : out std_logic_vector(1 downto 0);
        bvalid  : out std_logic;
        bready  : in  std_logic;

        araddr  : in  std_logic_vector(8 downto 0);
        arprot  : in  std_logic_vector(2 downto 0);
        arvalid : in  std_logic;
        arready : out std_logic;

        rdata   : out std_logic_vector(num_data_bits-1 downto 0);
        rresp   : out std_logic_vector(1 downto 0);
        rvalid  : out std_logic;
        rready  : in  std_logic;

        w_addr  : out std_logic_vector(8 downto 0);
        w_data  : out std_logic_vector(num_data_bits-1 downto 0);
        w_strb  : out std_logic_vector((num_data_bits/8)-1 downto 0);
        w_we    : out std_logic;

        r_addr  : out std_logic_vector(8 downto 0);
        rd_data : in  std_logic_vector(num_data_bits-1 downto 0)
    );
end entity axil_fsm_32;

architecture Behavioral of axil_fsm_32 is
    type t_read_state is (RD_IDLE, RD_CAPTURE);

    signal s_aw_valid_r : std_logic := '0';
    signal s_aw_addr_r  : std_logic_vector(8 downto 0) := (others => '0');
    signal s_aw_prot_r  : std_logic_vector(2 downto 0) := (others => '0');

    signal s_w_valid_r : std_logic := '0';
    signal s_w_data_r  : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');
    signal s_w_strb_r  : std_logic_vector((num_data_bits/8)-1 downto 0) :=
        (others => '0');

    signal s_bvalid_r : std_logic := '0';
    signal s_bresp_r  : std_logic_vector(1 downto 0) := (others => '0');

    signal s_read_state_r : t_read_state := RD_IDLE;
    signal s_ar_addr_r    : std_logic_vector(8 downto 0) := (others => '0');
    signal s_ar_prot_r    : std_logic_vector(2 downto 0) := (others => '0');
    signal s_rvalid_r     : std_logic := '0';
    signal s_rdata_r      : std_logic_vector(num_data_bits-1 downto 0) :=
        (others => '0');
    signal s_rresp_r      : std_logic_vector(1 downto 0) := (others => '0');

    signal s_awready        : std_logic;
    signal s_wready         : std_logic;
    signal s_arready        : std_logic;
    signal s_write_commit   : std_logic;
    signal s_write_prot_err : std_logic;
    signal s_read_prot_err  : std_logic;
begin
    assert num_data_bits = 32
        report "axil_fsm_32 supports a 32-bit AXI4-Lite data bus"
        severity failure;
    assert num_ctl_regs >= 0 and num_stat_regs >= 0
        report "axil_fsm_32 register counts cannot be negative"
        severity failure;

    s_awready <= '1' when
        aresetn = '1' and s_aw_valid_r = '0' and s_bvalid_r = '0' else '0';
    s_wready <= '1' when
        aresetn = '1' and s_w_valid_r = '0' and s_bvalid_r = '0' else '0';
    s_arready <= '1' when
        aresetn = '1' and s_read_state_r = RD_IDLE and s_rvalid_r = '0' else '0';

    awready <= s_awready;
    wready  <= s_wready;
    arready <= s_arready;

    bvalid <= s_bvalid_r;
    bresp  <= s_bresp_r;
    rvalid <= s_rvalid_r;
    rdata  <= s_rdata_r;
    rresp  <= s_rresp_r;

    w_addr <= s_aw_addr_r;
    w_data <= s_w_data_r;
    w_strb <= s_w_strb_r;
    r_addr <= s_ar_addr_r;

    s_write_prot_err <= '1' when
        (en_secure_chk = '1' and s_aw_prot_r(1) = '1') or
        (en_priv_chk = '1' and s_aw_prot_r(0) = '0') else '0';

    s_read_prot_err <= '1' when
        (en_secure_chk = '1' and s_ar_prot_r(1) = '1') or
        (en_priv_chk = '1' and s_ar_prot_r(0) = '0') else '0';

    s_write_commit <= '1' when
        aresetn = '1' and s_aw_valid_r = '1' and s_w_valid_r = '1' and
        s_bvalid_r = '0' else '0';
    w_we <= s_write_commit and not s_write_prot_err;

    p_write : process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s_aw_valid_r <= '0';
                s_aw_addr_r  <= (others => '0');
                s_aw_prot_r  <= (others => '0');
                s_w_valid_r  <= '0';
                s_w_data_r   <= (others => '0');
                s_w_strb_r   <= (others => '0');
                s_bvalid_r   <= '0';
                s_bresp_r    <= (others => '0');
            else
                if s_bvalid_r = '1' then
                    if bready = '1' then
                        s_bvalid_r <= '0';
                    end if;
                elsif s_write_commit = '1' then
                    s_aw_valid_r <= '0';
                    s_w_valid_r  <= '0';
                    s_bvalid_r   <= '1';
                    if s_write_prot_err = '1' then
                        s_bresp_r <= "10";
                    else
                        s_bresp_r <= "00";
                    end if;
                end if;

                if s_awready = '1' and awvalid = '1' then
                    s_aw_valid_r <= '1';
                    s_aw_addr_r  <= awaddr;
                    s_aw_prot_r  <= awprot;
                end if;

                if s_wready = '1' and wvalid = '1' then
                    s_w_valid_r <= '1';
                    s_w_data_r  <= wdata;
                    s_w_strb_r  <= wstrb;
                end if;
            end if;
        end if;
    end process p_write;

    p_read : process(aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                s_read_state_r <= RD_IDLE;
                s_ar_addr_r    <= (others => '0');
                s_ar_prot_r    <= (others => '0');
                s_rvalid_r     <= '0';
                s_rdata_r      <= (others => '0');
                s_rresp_r      <= (others => '0');
            else
                if s_rvalid_r = '1' then
                    if rready = '1' then
                        s_rvalid_r <= '0';
                    end if;
                else
                    case s_read_state_r is
                        when RD_IDLE =>
                            if s_arready = '1' and arvalid = '1' then
                                s_ar_addr_r    <= araddr;
                                s_ar_prot_r    <= arprot;
                                s_read_state_r <= RD_CAPTURE;
                            end if;

                        when RD_CAPTURE =>
                            if s_read_prot_err = '1' then
                                s_rdata_r <= (others => '0');
                                s_rresp_r <= "10";
                            else
                                s_rdata_r <= rd_data;
                                s_rresp_r <= "00";
                            end if;
                            s_rvalid_r     <= '1';
                            s_read_state_r <= RD_IDLE;
                    end case;
                end if;
            end if;
        end if;
    end process p_read;
end architecture Behavioral;
