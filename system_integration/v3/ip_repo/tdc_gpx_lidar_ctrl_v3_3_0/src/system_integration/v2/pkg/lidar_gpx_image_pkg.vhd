library ieee;
use ieee.std_logic_1164.all;

use work.lidar_gpx_pkg.all;
use work.tdc_gpx_pkg.all;
use work.tdc_gpx_cfg_pkg.all;

-- 보드에서 검증된 v1 GPX 초기 Register image를 v2 형식으로 연결하는 단일
-- 어댑터다. 실제 16개 Register 기본값의 소유자는 tdc_gpx_cfg_pkg이며,
-- 이 패키지는 값을 복제하지 않고 28-bit gpx_register_image_t로만 변환한다.
package lidar_gpx_image_pkg is

    constant C_GPX_REGISTER_IMAGE_DEFAULT : gpx_register_image_t;

end package lidar_gpx_image_pkg;

package body lidar_gpx_image_pkg is

    -- v1 배열의 Register 0..15를 같은 주소 순서로 복사한다. 이 함수는
    -- elaboration 때 상수 한 번을 만들기 위한 것이며 Runtime 회로가 아니다.
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
