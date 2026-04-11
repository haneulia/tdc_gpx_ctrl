--------------------------------------------------------------------------------
-- Pentek Utility Package
--------------------------------------------------------------------------------
-- Create Date: 04/17/2015 07:00:43 PM
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 

--------------------------------------------------------------------------------
-- Libraries
--------------------------------------------------------------------------------
use std.textio.all;

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

use ieee.numeric_std.ALL;

--------------------------------------------------------------------------------
-- Package
--------------------------------------------------------------------------------      
package px_utility_pkg is

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Functions
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------  
  -- Converts boolean to std_logic
  function px_boolean_to_std_logic(l: boolean) return std_logic;
  -- Converts boolean to integer 0 or 1
  function px_boolean_to_int(l: boolean) return integer;
  -- Converts std_logic to boolean
  function px_std_logic_to_boolean(ivalue : std_logic) return boolean;
  -- Convert std_logic_vector to Hex String
  function px_std_logic_vector_to_hstring (value : std_logic_vector) return string;
  -- Reduces a vector to std_logic by 'OR' of bits.
  function px_or_reduce(l : std_logic_vector) return std_logic;
  -- Reduces a vector to std_logic by 'AND' of bits.
  function px_and_reduce(l : std_logic_vector) return std_logic;
  -- Calculate the Log2 of a Unsigned Number
  function px_log2_unsigned( x : natural ) return natural;
   -- Log2 Rounded Up
   -- Combine the ceil and log2 functions.  ceil_log2(x) then gives the minimum number
   -- of bits required to represent 'x'.  ceil_log2(4) = 2, ceil_log2(5) = 3, etc.
   -----------------------------------------------------------------------------------
   -- Can be used to determine the number of bits required to represent a number
   function px_ceil_log2 (Arg : positive) return natural;

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Procedures
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
  -- Purpose: Skips white space
  procedure px_skip_whitespace (l : inout line);
  -- Hex Character to quad bit vector
  procedure px_char2quadbits (c           :     CHARACTER;
                             result      : out std_logic_vector(3 downto 0);
                             good        : out boolean;
                             issue_error : in  boolean);
  -- Hex Value Read to std_logic_vector
  procedure px_hread (l    : inout line; value : out std_logic_vector; good : out   boolean);
  -- AXI4-Lite Master Writer
  -- Used to test Write sequences to an AXI-Lite Slave
  procedure px_axi_lite_writer(
      constant addr: in std_logic_vector;
      constant val : in std_logic_vector(31 downto 0);
      signal axi_aclk    : in std_logic;
      signal axi_awaddr  : out  std_logic_vector;
      signal axi_awprot  : out  std_logic_vector(2 downto 0);
      signal axi_awvalid : out  std_logic;
      signal axi_awready : in   std_logic;
      signal axi_wdata   : out  std_logic_vector(31 downto 0);
      signal axi_wstrb   : out  std_logic_vector(3 downto 0);
      signal axi_wvalid  : out  std_logic;
      signal axi_wready  : in   std_logic;
      signal axi_bresp   : in   std_logic_vector(1 downto 0);
      signal axi_bvalid  : in   std_logic;
      signal axi_bready  : out  std_logic
      );
   -- AXI4-Lite Master Reader
   -----------------------------------------------------------------------------
   -- Used to test Read sequences from an AXI-Lite Slave
   procedure px_axi_lite_reader(
       constant addr: in std_logic_vector;
       constant val : in std_logic_vector(31 downto 0); -- comparision value
       constant comp : in std_logic; -- '1' = allow compare
       constant fail_on_error : in std_logic; -- '1' = allow failure on error    
       signal axi_aclk          : in std_logic; 
       signal axi_araddr        : out std_logic_vector;
       signal axi_arprot        : out std_logic_vector(2 downto 0);
       signal axi_arvalid       : out std_logic;
       signal axi_arready       : in std_logic;
       signal axi_rdata         : in std_logic_vector(31 downto 0);
       signal axi_rresp         : in std_logic_vector(1 downto 0);
       signal axi_rvalid        : in std_logic;
       signal axi_rready        : out std_logic
       );  

-------------------------------------------------------------------------------- 
  
end;

--------------------------------------------------------------------------------
-- Package Body
--------------------------------------------------------------------------------      
package body px_utility_pkg is

--------------------------------------------------------------------------------
-- Constants
--------------------------------------------------------------------------------
constant NBSP : CHARACTER      := CHARACTER'val(160);  -- space character
constant NUS  : string(2 to 1) := (others => ' ');     -- null STRING

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Functions
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
-- Converts boolean to std_logic
--------------------------------------------------------------------------------   
   function px_boolean_to_std_logic(l: boolean) return std_logic is
   begin
       if l then
          return('1');
       else
          return('0');
       end if;
   end function px_boolean_to_std_logic;
   
--------------------------------------------------------------------------------
-- Converts boolean to std_logic
--------------------------------------------------------------------------------   
   function px_boolean_to_int(l: boolean) return integer is
   begin
       if l then
          return(1);
       else
          return(0);
       end if;
   end function px_boolean_to_int;
   
--------------------------------------------------------------------------------
-- Converts std_logic to boolean
--------------------------------------------------------------------------------   
   function px_std_logic_to_boolean(ivalue : std_logic) return boolean is
   begin
      if ivalue = '0' then
         return false;
      else
         return true;
      end if;    
   end function px_std_logic_to_boolean;

--------------------------------------------------------------------------------   
-- Reduces a vector to std_logic by 'OR' of bits.  
-------------------------------------------------------------------------------- 
   function px_or_reduce(l : std_logic_vector) return std_logic is
   variable result : std_logic := '0';
   begin
      for i in l'reverse_range loop
         result := (l(i) or result);
      end loop;
      return result;
   end function px_or_reduce;

--------------------------------------------------------------------------------   
-- Reduces a vector to std_logic by 'AND' of bits.  
-------------------------------------------------------------------------------- 
   function px_and_reduce(l : std_logic_vector) return std_logic is
   variable result : std_logic := '1';
   begin
      for i in l'reverse_range loop
         result := (l(i) and result);
      end loop;
      return result;
   end function px_and_reduce;
   
-------------------------------------------------------------------------------- 
-- Convert std_logic_vector to Hex String
-------------------------------------------------------------------------------- 
  function px_std_logic_vector_to_hstring (value : std_logic_vector) return string is
  constant ne     : integer := (value'length+3)/4;
  variable pad    : std_logic_vector(0 to (ne*4 - value'length) - 1);
  variable ivalue : std_logic_vector(0 to ne*4 - 1);
  variable result : string(1 to ne);
  variable quad   : std_logic_vector(0 to 3);
  begin
    if value'length < 1 then
      return NUS;
    else
      if value (value'left) = 'Z' then
        pad := (others => 'Z');
      else
        pad := (others => '0');
      end if;
      ivalue := pad & value;
      for i in 0 to ne-1 loop
        quad := To_X01Z(ivalue(4*i to 4*i+3));
        case quad is
          when x"0"   => result(i+1) := '0';
          when x"1"   => result(i+1) := '1';
          when x"2"   => result(i+1) := '2';
          when x"3"   => result(i+1) := '3';
          when x"4"   => result(i+1) := '4';
          when x"5"   => result(i+1) := '5';
          when x"6"   => result(i+1) := '6';
          when x"7"   => result(i+1) := '7';
          when x"8"   => result(i+1) := '8';
          when x"9"   => result(i+1) := '9';
          when x"A"   => result(i+1) := 'A';
          when x"B"   => result(i+1) := 'B';
          when x"C"   => result(i+1) := 'C';
          when x"D"   => result(i+1) := 'D';
          when x"E"   => result(i+1) := 'E';
          when x"F"   => result(i+1) := 'F';
          when "ZZZZ" => result(i+1) := 'Z';
          when others => result(i+1) := 'X';
        end case;
      end loop;
      return result;
    end if;
  end function px_std_logic_vector_to_hstring;
  
  ------------------------------------------------------------------------------
  -- Calculate the Log2 of a Unsigned Number
  ------------------------------------------------------------------------------
  function px_log2_unsigned ( x : natural ) return natural is
  variable temp : natural := x ;
  variable n : natural := 0 ;
  begin
      while temp > 1 loop
         temp := temp / 2 ;
         n := n + 1 ;
      end loop ;
      return n ;
   end function px_log2_unsigned ;
  
   -----------------------------------------------------------------------------------
   -- Log2 Rounded Up
   -- Combine the ceil and log2 functions.  ceil_log2(x) then gives the minimum number
   -- of bits required to represent 'x'.  ceil_log2(4) = 2, ceil_log2(5) = 3, etc.
   -----------------------------------------------------------------------------------
   -- Can be used to determine the number of bits required to represent a number
   function px_ceil_log2 (Arg : positive) return natural is
       variable RetVal:    natural;
   begin
       RetVal := px_log2_unsigned(Arg);
       if (Arg > (2**RetVal)) then
           return(RetVal + 1); -- RetVal is too small, so bump it up by 1 and return
       else
           return(RetVal); -- Just right
       end if;
   end function px_ceil_log2;

--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
-- Procedures
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
  -- Purpose: Skips white space
--------------------------------------------------------------------------------  
  procedure px_skip_whitespace (l : inout line) is
  variable readok : boolean;
  variable c : CHARACTER;
  begin
    while l /= null and l.all'length /= 0 loop
      if (l.all(1) = ' ' or l.all(1) = NBSP or l.all(1) = HT) then
        read (l, c, readok);
      else
        exit;
      end if;
    end loop;
  end procedure px_skip_whitespace;

---------------------------------------------------------------------------------
-- Hex Character to quad bit vector
---------------------------------------------------------------------------------
  procedure px_char2quadbits (c           :     CHARACTER;
                           result      : out std_logic_vector(3 downto 0);
                           good        : out boolean;
                           issue_error : in  boolean) is
  begin
    case c is
      when '0'       => result := x"0"; good := true;
      when '1'       => result := x"1"; good := true;
      when '2'       => result := x"2"; good := true;
      when '3'       => result := x"3"; good := true;
      when '4'       => result := x"4"; good := true;
      when '5'       => result := x"5"; good := true;
      when '6'       => result := x"6"; good := true;
      when '7'       => result := x"7"; good := true;
      when '8'       => result := x"8"; good := true;
      when '9'       => result := x"9"; good := true;
      when 'A' | 'a' => result := x"A"; good := true;
      when 'B' | 'b' => result := x"B"; good := true;
      when 'C' | 'c' => result := x"C"; good := true;
      when 'D' | 'd' => result := x"D"; good := true;
      when 'E' | 'e' => result := x"E"; good := true;
      when 'F' | 'f' => result := x"F"; good := true;
      when 'Z'       => result := "ZZZZ"; good := true;
      when 'X'       => result := "XXXX"; good := true;
      when others =>
        assert not issue_error
          report
          "STD_LOGIC_1164.HREAD Read a '" & c &
          "', expected a Hex character (0-F)."
          severity error;
        good := false;
    end case;
  end procedure px_char2quadbits;
  
--------------------------------------------------------------------------------
-- Hex Value Read to std_logic_vector
--------------------------------------------------------------------------------
   procedure px_hread (l : inout line; value : out std_logic_vector; good : out   boolean) is
   variable ok     : boolean;
   variable c      : CHARACTER;
   constant ne     : integer := (value'length+3)/4;
   constant pad    : integer := ne*4 - VALUE'length;
   variable sv     : std_logic_vector(0 to ne*4 - 1);
   variable i      : integer;
   variable lastu  : boolean := false;       -- last character was an "_"
   begin
    value := (value'range => 'U'); -- initialize to a "U"
    px_skip_whitespace (l);
    if value'length > 0 then
      read (l, c, ok);
      i := 0;
      while i < ne loop
        -- Bail out if there was a bad read
        if not ok then
          good := false;
          return;
        elsif c = '_' then
          if i = 0 then
            good := false;                -- Begins with an "_"
            return;
          elsif lastu then
            good := false;                -- "__" detected
            return;
          else
            lastu := true;
          end if;
        else
          px_char2quadbits(c, sv(4*i to 4*i+3), ok, false);
          if not ok then
            good := false;
            return;
          end if;
          i := i + 1;
          lastu := false;
        end if;
        if i < ne then
          read(L, c, ok);
        end if;
      end loop;
      if px_or_reduce(sv (0 to pad-1)) = '1' then  -- %%% replace with "or"
        good := false;                           -- vector was truncated.
      else
        good  := true;
        value := sv (pad to sv'high);
      end if;
    else
      good := true;                     -- Null input string, skips whitespace
    end if;
  end procedure px_hread;

--------------------------------------------------------------------------------
-- AXI4-Lite Master Writer
--------------------------------------------------------------------------------
-- Used to test Write sequences to an AXI-Lite Slave
procedure px_axi_lite_writer(
    constant addr: in std_logic_vector;
    constant val : in std_logic_vector(31 downto 0);
    signal axi_aclk    : in std_logic;
    signal axi_awaddr  : out  std_logic_vector;
    signal axi_awprot  : out  std_logic_vector(2 downto 0);
    signal axi_awvalid : out  std_logic;
    signal axi_awready : in   std_logic;
    signal axi_wdata   : out  std_logic_vector(31 downto 0);
    signal axi_wstrb   : out  std_logic_vector(3 downto 0);
    signal axi_wvalid  : out  std_logic;
    signal axi_wready  : in   std_logic;
    signal axi_bresp   : in   std_logic_vector(1 downto 0);
    signal axi_bvalid  : in   std_logic;
    signal axi_bready  : out  std_logic
    ) is
    begin
        assert false
        report "Writing x" & (px_std_logic_vector_to_hstring(val)) & " to Address: x" &  (px_std_logic_vector_to_hstring(addr))
        severity NOTE;
        wait until rising_edge(axi_aclk);
        wait for 1 ns;
        axi_awaddr           <= addr; -- Byte Address
        axi_awprot           <= "000";
        axi_awvalid          <= '1';
        axi_wvalid           <= '1';
        axi_wdata            <= val;
        axi_wstrb            <= "1111";
        axi_bready           <= '1';
        while ((axi_awready = '0') and (axi_wready = '0')) loop
            wait until rising_edge(axi_aclk);
            wait for 1 ns;
        end loop;
        if (axi_awready = '1') and (axi_wready = '1') then
            wait until rising_edge(axi_aclk);
            wait for 1 ns; 
            axi_awvalid         <= '0';
            axi_wvalid          <= '0';
            axi_wstrb               <= "0000";
        elsif (axi_awready = '1') then
            wait until rising_edge(axi_aclk);
            wait for 1 ns; 
            axi_awvalid          <= '0';
            while (axi_wready = '0') loop
                wait until rising_edge(axi_aclk);
                wait for 1 ns;
            end loop;
            wait until rising_edge(axi_aclk);
            wait for 1 ns; 
            axi_wvalid          <= '0';
            axi_wstrb               <= "0000";
        else
            wait until rising_edge(axi_aclk);
            wait for 1 ns; 
            axi_wvalid          <= '0';
            axi_wstrb               <= "0000";
            while (axi_awready = '0') loop
                wait until rising_edge(axi_aclk);
                wait for 1 ns;
            end loop;
            wait until rising_edge(axi_aclk);
            wait for 1 ns; 
            axi_awvalid         <= '0';
        end if;
        while (axi_bvalid = '0') loop
            wait until rising_edge(axi_aclk);
            wait for 1 ns;
        end loop;
        wait until rising_edge(axi_aclk);
        wait for 1 ns;             
        axi_bready           <= '0';
        if axi_bresp = "00" then
            assert false
            report "Write completed at time = " & time'image(now)
            severity NOTE;
        else
            assert false
            report "Write completed with error at time = " & time'image(now)
            severity NOTE;
        end if;          
end procedure px_axi_lite_writer;

--------------------------------------------------------------------------------
-- AXI4-Lite Master Reader
--------------------------------------------------------------------------------
-- Used to test Read sequences from an AXI-Lite Slave
procedure px_axi_lite_reader(
    constant addr: in std_logic_vector;
    constant val : in std_logic_vector(31 downto 0); -- comparision value
    constant comp : in std_logic; -- '1' = allow compare
    constant fail_on_error : in std_logic; -- '1' = allow failure on error    
    signal axi_aclk          : in std_logic; 
    signal axi_araddr        : out std_logic_vector;
    signal axi_arprot        : out std_logic_vector(2 downto 0);
    signal axi_arvalid       : out std_logic;
    signal axi_arready       : in std_logic;
    signal axi_rdata         : in std_logic_vector(31 downto 0);
    signal axi_rresp         : in std_logic_vector(1 downto 0);
    signal axi_rvalid        : in std_logic;
    signal axi_rready        : out std_logic
    ) is
    begin
        assert false
        report "Reading Address: x" &  (px_std_logic_vector_to_hstring(addr))
        severity NOTE;
        wait until rising_edge(axi_aclk);
        wait for 1 ns;
        axi_araddr           <= addr; -- Byte Address
        axi_arprot           <= "000";
        axi_arvalid          <= '1';
        while (axi_arready = '0') loop
            wait until rising_edge(axi_aclk);
            wait for 1 ns;
        end loop;
        wait until rising_edge(axi_aclk);
        wait for 1 ns; 
        axi_arvalid          <= '0';
        axi_rready           <= '1';
        while (axi_rvalid = '0') loop
           wait until rising_edge(axi_aclk);
        end loop;
        wait for 1 ns; 
        if (comp = '1') then
            assert false
            report "Value is: x" &  (px_std_logic_vector_to_hstring(axi_rdata)) & " Should be: x" &  (px_std_logic_vector_to_hstring(val))
            severity NOTE;
        else
            assert false
            report "Value is: x" &  (px_std_logic_vector_to_hstring(axi_rdata))
            severity NOTE;
        end if;    
    
        if (val = axi_rdata) or (comp = '0') then
            assert false
            report "Read completed successfully at time = " & time'image(now)
            severity NOTE;
        else
            if (fail_on_error = '1') then
                assert false
                report "Read error at time = " & time'image(now)
                severity FAILURE;            
            else
                assert false
                report "Read error at time = " & time'image(now)
                severity NOTE;               
            end if;
        end if; 
        wait until rising_edge(axi_aclk);
        axi_rready           <= '0';       
end procedure px_axi_lite_reader;


end package body;
