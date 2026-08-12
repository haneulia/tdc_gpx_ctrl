library ieee;
use ieee.std_logic_1164.all;

use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;

-- TDC-domain bridge between one configuration ACTIVATE request and the
-- multi-Chip GPX programming handshake. i_candidate_image is a coherently
-- held mailbox payload; the gateway's synchronized activate-start event is
-- the capture strobe.
entity lidar_gpx_config_activation is
    port (
        i_clk               : in  std_logic;
        i_rst_n             : in  std_logic;
        i_activate_start    : in  std_logic;
        i_candidate_image   : in  gpx_register_image_t;
        i_apply_ready       : in  std_logic;
        i_apply_done        : in  std_logic;
        i_apply_fault       : in  std_logic;
        o_register_image    : out gpx_register_image_t;
        o_apply             : out std_logic;
        o_activate_complete : out std_logic;
        o_activate_fault    : out std_logic;
        o_busy              : out std_logic
    );
end entity lidar_gpx_config_activation;

architecture rtl of lidar_gpx_config_activation is

    type state_t is (IDLE, WAIT_READY, WAIT_DONE, FAULTED);

    signal state_r : state_t := IDLE;
    signal image_r : gpx_register_image_t :=
        C_GPX_REGISTER_IMAGE_DEFAULT;
    signal apply_r : std_logic := '0';
    signal complete_r : std_logic := '0';

begin

    o_register_image <= image_r;
    o_apply <= apply_r;
    o_activate_complete <= complete_r;
    o_activate_fault <= '1' when state_r = FAULTED else '0';
    o_busy <= '1' when state_r = WAIT_READY or state_r = WAIT_DONE else '0';

    p_activation : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                state_r <= IDLE;
                image_r <= C_GPX_REGISTER_IMAGE_DEFAULT;
                apply_r <= '0';
                complete_r <= '0';
            else
                apply_r <= '0';
                complete_r <= '0';

                case state_r is
                    when IDLE =>
                        if i_activate_start = '1' then
                            image_r <= i_candidate_image;
                            state_r <= WAIT_READY;
                        end if;

                    when WAIT_READY =>
                        if i_apply_fault = '1' then
                            state_r <= FAULTED;
                        elsif i_apply_ready = '1' then
                            apply_r <= '1';
                            state_r <= WAIT_DONE;
                        end if;

                    when WAIT_DONE =>
                        if i_apply_fault = '1' then
                            state_r <= FAULTED;
                        elsif i_apply_done = '1' then
                            complete_r <= '1';
                            state_r <= IDLE;
                        end if;

                    when FAULTED =>
                        null;
                end case;

                -- synthesis translate_off
                assert not (i_activate_start = '1' and state_r /= IDLE)
                    report "V2-GPX-ACT-001 overlapping activation"
                    severity failure;
                assert not (i_apply_done = '1' and state_r /= WAIT_DONE)
                    report "V2-GPX-ACT-002 completion without active apply"
                    severity warning;
                -- synthesis translate_on
            end if;
        end if;
    end process p_activation;

end architecture rtl;
