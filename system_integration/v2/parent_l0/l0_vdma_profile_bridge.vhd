library ieee;
use ieee.std_logic_1164.all;

library xpm;
use xpm.vcomponents.all;

-- Processing domain에서 계산한 VDMA Face geometry를 CSR domain의 PS가
-- 원자적으로 읽고 승인하도록 연결하는 부모 프로젝트 전용 CDC bridge.
--
-- Source 계약:
--   * i_proc_cfg_valid가 High인 동안 49-bit profile은 변하지 않는다.
--   * PS가 해당 VDMA를 설정하고 ACK한 뒤 o_proc_cfg_ready를 1 clock pulse로
--     반환한다. 이 pulse를 받은 v2 core만 pending profile을 Active로 바꾼다.
-- Destination 계약:
--   * o_csr_cfg_valid가 High일 때 enable/HSIZE/VSIZE/STRIDE는 한 snapshot이다.
--   * 이전 ACK가 High로 남아 있으면 승인하지 않는다. ACK Low를 확인한 뒤
--     새 High가 들어와야 이번 profile을 승인한다.
entity l0_vdma_profile_bridge is
    port (
        i_proc_clk          : in  std_logic;
        i_proc_rst_n        : in  std_logic;
        i_proc_cfg_valid    : in  std_logic;
        i_proc_cfg_enable   : in  std_logic;
        i_proc_hsize_bytes  : in  std_logic_vector(15 downto 0);
        i_proc_vsize_lines  : in  std_logic_vector(15 downto 0);
        i_proc_stride_bytes : in  std_logic_vector(15 downto 0);
        o_proc_cfg_ready    : out std_logic;

        i_csr_clk          : in  std_logic;
        i_csr_rst_n        : in  std_logic;
        o_csr_cfg_valid    : out std_logic;
        o_csr_cfg_enable   : out std_logic;
        o_csr_hsize_bytes  : out std_logic_vector(15 downto 0);
        o_csr_vsize_lines  : out std_logic_vector(15 downto 0);
        o_csr_stride_bytes : out std_logic_vector(15 downto 0);
        i_csr_cfg_ack      : in  std_logic
    );
end entity l0_vdma_profile_bridge;

architecture rtl of l0_vdma_profile_bridge is

    constant C_PROFILE_WIDTH : positive := 49;

    type proc_state_t is (
        PROC_IDLE,
        PROC_WAIT_ACK,
        PROC_WAIT_VALID_LOW,
        PROC_WAIT_CDC_RELEASE
    );

    signal proc_state_r   : proc_state_t := PROC_IDLE;
    signal proc_payload_r : std_logic_vector(C_PROFILE_WIDTH - 1 downto 0) :=
        (others => '0');
    signal proc_send_r    : std_logic := '0';
    signal proc_received  : std_logic;
    signal proc_ready_r   : std_logic := '0';

    signal csr_payload    : std_logic_vector(C_PROFILE_WIDTH - 1 downto 0);
    signal csr_request    : std_logic;
    signal csr_ack_r      : std_logic := '0';
    signal csr_ack_armed_r : std_logic := '0';

begin

    o_proc_cfg_ready <= proc_ready_r;

    o_csr_cfg_valid    <= csr_request;
    o_csr_cfg_enable   <= csr_payload(48);
    o_csr_hsize_bytes  <= csr_payload(47 downto 32);
    o_csr_vsize_lines  <= csr_payload(31 downto 16);
    o_csr_stride_bytes <= csr_payload(15 downto 0);

    u_profile_cdc : xpm_cdc_handshake
        generic map (
            DEST_EXT_HSK   => 1,
            DEST_SYNC_FF   => 4,
            INIT_SYNC_FF   => 0,
            SIM_ASSERT_CHK => 1,
            SRC_SYNC_FF    => 4,
            WIDTH          => C_PROFILE_WIDTH
        )
        port map (
            src_clk  => i_proc_clk,
            src_in   => proc_payload_r,
            src_send => proc_send_r,
            src_rcv  => proc_received,
            dest_clk => i_csr_clk,
            dest_req => csr_request,
            dest_ack => csr_ack_r,
            dest_out => csr_payload
        );

    p_proc_request : process (i_proc_clk)
    begin
        if rising_edge(i_proc_clk) then
            proc_ready_r <= '0';

            if i_proc_rst_n = '0' then
                proc_state_r <= PROC_IDLE;
                proc_payload_r <= (others => '0');
                proc_send_r <= '0';
            else
                case proc_state_r is
                    when PROC_IDLE =>
                        proc_send_r <= '0';
                        if i_proc_cfg_valid = '1' and
                           proc_received = '0' then
                            proc_payload_r <= i_proc_cfg_enable &
                                i_proc_hsize_bytes &
                                i_proc_vsize_lines &
                                i_proc_stride_bytes;
                            proc_send_r <= '1';
                            proc_state_r <= PROC_WAIT_ACK;
                        end if;

                    when PROC_WAIT_ACK =>
                        if proc_received = '1' then
                            -- v2 profile manager가 이 pulse를 샘플한 뒤에만
                            -- pending profile을 Active profile로 승격한다.
                            proc_ready_r <= '1';
                            proc_send_r <= '0';
                            proc_state_r <= PROC_WAIT_VALID_LOW;
                        end if;

                    when PROC_WAIT_VALID_LOW =>
                        -- 원본 valid의 Low를 먼저 확인하여 동일 요청의
                        -- 중복 전송을 막는다. src_rcv 해제 순서와 무관하다.
                        if i_proc_cfg_valid = '0' then
                            if proc_received = '0' then
                                proc_state_r <= PROC_IDLE;
                            else
                                proc_state_r <= PROC_WAIT_CDC_RELEASE;
                            end if;
                        end if;

                    when PROC_WAIT_CDC_RELEASE =>
                        -- CDC 내부 acknowledgement가 늦게 해제되는 경우를
                        -- 별도로 기다린다. 다음 valid가 이미 올라와도
                        -- IDLE 진입 후 새 49-bit snapshot으로 다시 잡는다.
                        if proc_received = '0' then
                            proc_state_r <= PROC_IDLE;
                        end if;
                end case;
            end if;
        end if;
    end process p_proc_request;

    p_csr_ack : process (i_csr_clk)
    begin
        if rising_edge(i_csr_clk) then
            if i_csr_rst_n = '0' then
                csr_ack_r <= '0';
                csr_ack_armed_r <= '0';
            elsif csr_request = '0' then
                csr_ack_r <= '0';
                -- 이전 ACK가 Low로 복귀한 것이 확인되어야 다음 요청을
                -- 승인할 수 있다. PS가 ACK bit를 High로 방치한 상태는
                -- 새로운 VDMA programming 완료로 인정하지 않는다.
                if i_csr_cfg_ack = '0' then
                    csr_ack_armed_r <= '1';
                else
                    csr_ack_armed_r <= '0';
                end if;
            elsif csr_ack_r = '0' then
                if i_csr_cfg_ack = '0' then
                    csr_ack_armed_r <= '1';
                elsif csr_ack_armed_r = '1' then
                    csr_ack_r <= '1';
                    csr_ack_armed_r <= '0';
                end if;
            end if;
        end if;
    end process p_csr_ack;

end architecture rtl;
