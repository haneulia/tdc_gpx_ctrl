library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lidar_build_pkg.all;
use work.lidar_config_types_pkg.all;
use work.lidar_gpx_pkg.all;
use work.lidar_gpx_image_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

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
        i_prepare         : in  std_logic;
        i_candidate       : in  lidar_active_config_t;
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
        variable v_effective_image : gpx_register_image_t;
    begin
        if rising_edge(i_clk) then
            if i_rst_n = '0' then
                candidate_image_r <= C_GPX_REGISTER_IMAGE_DEFAULT;
                active_image_r <= C_GPX_REGISTER_IMAGE_DEFAULT;
            else
                if i_commit = '1' and i_cfg_busy = '0' then
                    candidate_image_r <= i_shadow_image;
                end if;

                if i_prepare = '1' then
                    v_effective_image := candidate_image_r;
                    -- CTL22에서 입력한 Reg7의 다른 보정 필드는 보존하되,
                    -- MTimer만 CTL12 목표 왕복시간에서 자동 계산한 값으로
                    -- 덮어쓴다. 따라서 소프트웨어가 두 시간을 따로 맞출
                    -- 필요가 없고 Active-image readback도 실제 칩 값과 같다.
                    v_effective_image(7)(
                        c_REG7_MTIMER_HI downto c_REG7_MTIMER_LO) :=
                        std_logic_vector(
                            i_candidate.derived.gpx_mtimer_ref_ticks);
                    candidate_image_r <= v_effective_image;
                end if;

                if i_cfg_done = '1' and i_cfg_error = CFG_OK then
                    active_image_r <= candidate_image_r;
                end if;
            end if;
        end if;
    end process p_transaction;

end architecture rtl;
