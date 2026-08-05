library ieee;
use ieee.std_logic_1164.all;

use work.lidar_gpx_pkg.all;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

-- Single v2 adapter for the board-proven v1 GPX reset image. The register
-- values remain owned by tdc_gpx_cfg_pkg; v2 does not copy their literals.
package lidar_gpx_image_pkg is

    constant C_GPX_REGISTER_IMAGE_DEFAULT : gpx_register_image_t;

end package lidar_gpx_image_pkg;

package body lidar_gpx_image_pkg is

    function fn_default_image return gpx_register_image_t is
        variable result : gpx_register_image_t :=
            (others => (others => '0'));
    begin
        for index in result'range loop
            result(index) := c_GPX_DEFAULT_IMAGE(index);
        end loop;
        return result;
    end function fn_default_image;

    constant C_GPX_REGISTER_IMAGE_DEFAULT : gpx_register_image_t :=
        fn_default_image;

end package body lidar_gpx_image_pkg;
