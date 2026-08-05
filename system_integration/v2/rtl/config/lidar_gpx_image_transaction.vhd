library ieee;
use ieee.std_logic_1164.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;

-- CSR-domain companion to the central configuration transaction. The image
-- is not part of Processing configuration arithmetic, but it is snapshotted
-- on the same accepted COMMIT and becomes active only after the same manager
-- transaction succeeds.
entity lidar_gpx_image_transaction is
    port (
        i_clk             : in  std_logic;
        i_rst_n           : in  std_logic;
        i_commit          : in  std_logic;
        i_cfg_busy        : in  std_logic;
        i_cfg_done        : in  std_logic;
        i_cfg_error       : in  lidar_cfg_error_t;
        i_shadow_image    : in  gpx_register_image_t;
        o_candidate_image : out gpx_register_image_t;
        o_active_image    : out gpx_register_image_t
    );
end entity lidar_gpx_image_transaction;

architecture rtl of lidar_gpx_image_transaction is

    signal candidate_image_r : gpx_register_image_t :=
        C_GPX_REGISTER_IMAGE_DEFAULT;
    signal active_image_r : gpx_register_image_t :=
        C_GPX_REGISTER_IMAGE_DEFAULT;

begin

    o_candidate_image <= candidate_image_r;
    o_active_image <= active_image_r;

    p_transaction : process (i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                candidate_image_r <= C_GPX_REGISTER_IMAGE_DEFAULT;
                active_image_r <= C_GPX_REGISTER_IMAGE_DEFAULT;
            else
                if i_commit = '1' and i_cfg_busy = '0' then
                    candidate_image_r <= i_shadow_image;
                end if;

                if i_cfg_done = '1' and i_cfg_error = CFG_OK then
                    active_image_r <= candidate_image_r;
                end if;
            end if;
        end if;
    end process p_transaction;

end architecture rtl;
