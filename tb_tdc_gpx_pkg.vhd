-- =============================================================================
-- tb_tdc_gpx_pkg.vhd
-- TDC-GPX Controller - Testbench Helper Package
-- =============================================================================
-- TB에서 공유하는 유틸리티 프로시저 모음
--   모든 프로시저는 signal 파라미터를 명시적으로 받아 재사용 가능
--
-- 포함 항목:
--   - Console helpers: pr_sep, pr_print, pr_pass, pr_fail, pr_info
--   - Clock/wait: tb_wait_clk, tb_wait_sig_value
--   - Natural / hex image: nat_img, hex_img
--
-- Standard: VHDL-93 compatible
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

package tb_tdc_gpx_pkg is

    -- =========================================================================
    -- Console helpers
    -- =========================================================================
    procedure pr_sep;
    procedure pr_print(msg : string);
    procedure pr_pass(msg : string);
    procedure pr_fail(msg : string; cnt : inout natural);
    procedure pr_info(msg : string);

    -- =========================================================================
    -- Clock / wait utilities
    -- =========================================================================
    procedure tb_wait_clk(
        signal clk : in std_logic;
        n          : natural
    );

    -- Wait for a signal to reach target value, with timeout (in clocks).
    -- Sets 'found' to true if the target value was observed.
    procedure tb_wait_sig_value(
        signal clk    : in std_logic;
        signal sig    : in std_logic;
        target        : std_logic;
        timeout_clk   : natural;
        found         : out boolean
    );

    -- =========================================================================
    -- String conversion helpers (VHDL-93)
    -- =========================================================================
    function nat_img(n : natural) return string;

    -- Hex image for std_logic_vector (7 nibbles = 28 bits)
    function hex_img(v : std_logic_vector) return string;

end package tb_tdc_gpx_pkg;

package body tb_tdc_gpx_pkg is

    -- =========================================================================
    -- Console helpers
    -- =========================================================================
    procedure pr_sep is
        variable vl : line;
    begin
        write(vl, string'("------------------------------------------------------------"));
        writeline(output, vl);
    end procedure;

    procedure pr_print(msg : string) is
        variable vl : line;
    begin
        write(vl, msg);
        writeline(output, vl);
    end procedure;

    procedure pr_pass(msg : string) is
    begin
        pr_print("  PASS: " & msg);
    end procedure;

    procedure pr_fail(msg : string; cnt : inout natural) is
        variable vl : line;
    begin
        write(vl, string'("  FAIL: ") & msg);
        writeline(output, vl);
        cnt := cnt + 1;
    end procedure;

    procedure pr_info(msg : string) is
    begin
        pr_print("  INFO: " & msg);
    end procedure;

    -- =========================================================================
    -- Clock / wait utilities
    -- =========================================================================
    procedure tb_wait_clk(
        signal clk : in std_logic;
        n          : natural
    ) is
    begin
        for i in 0 to n - 1 loop
            wait until rising_edge(clk);
        end loop;
    end procedure;

    procedure tb_wait_sig_value(
        signal clk    : in std_logic;
        signal sig    : in std_logic;
        target        : std_logic;
        timeout_clk   : natural;
        found         : out boolean
    ) is
    begin
        found := false;
        for i in 0 to timeout_clk - 1 loop
            wait until rising_edge(clk);
            if sig = target then
                found := true;
                return;
            end if;
        end loop;
    end procedure;

    -- =========================================================================
    -- String conversion helpers
    -- =========================================================================
    function nat_img(n : natural) return string is
    begin
        return integer'image(n);
    end function;

    function hex_img(v : std_logic_vector) return string is
        -- Convert slv to hex string (4 bits per char, MSB first)
        constant c_LEN    : natural := (v'length + 3) / 4;  -- number of hex chars
        variable v_padded : std_logic_vector(c_LEN * 4 - 1 downto 0) := (others => '0');
        variable v_nib    : unsigned(3 downto 0);
        variable v_result : string(1 to c_LEN);
        constant c_HEX    : string(1 to 16) := "0123456789ABCDEF";
    begin
        v_padded(v'length - 1 downto 0) := v;
        for i in 0 to c_LEN - 1 loop
            v_nib := unsigned(v_padded(c_LEN * 4 - 1 - i * 4 downto c_LEN * 4 - 4 - i * 4));
            v_result(i + 1) := c_HEX(to_integer(v_nib) + 1);
        end loop;
        return v_result;
    end function;

end package body tb_tdc_gpx_pkg;
