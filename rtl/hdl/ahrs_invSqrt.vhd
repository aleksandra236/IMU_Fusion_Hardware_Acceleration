-- =============================================================================
-- ahrs_invSqrt.vhd
-- Fast Inverse Square Root IP - IEEE 754 single precision
--
-- Algorithm: Quake III bit trick + 1x Newton-Raphson iteration
--   initial:  y = reinterpret(0x5F1F1412 - (reinterpret_int(x) >> 1))
--   refine:   y = y * (1.69000231 - 0.714158168 * x * y * y)
--
-- Implementation: bit manipulation + VHDL real (no ieee.float_pkg)
-- Latency: combinational
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ahrs_invSqrt is
    port (
        x_i : in  std_logic_vector(31 downto 0);
        y_o : out std_logic_vector(31 downto 0)
    );
end entity ahrs_invSqrt;

architecture Behavioral of ahrs_invSqrt is

    constant C_MAGIC : integer := 16#5F1F1412#;
    constant C_A     : real    := 1.69000231;
    constant C_B     : real    := 0.714158168;

    -- Decode IEEE 754 float32 bits to VHDL real
    function f32_to_real(slv : std_logic_vector(31 downto 0)) return real is
        variable expo : integer;
        variable mant : real;
        variable sign : real;
    begin
        if slv(30 downto 0) = (30 downto 0 => '0') then return 0.0; end if;
        if slv(31) = '1' then sign := -1.0; else sign := 1.0; end if;
        expo := to_integer(unsigned(slv(30 downto 23))) - 127;
        mant := 1.0 + real(to_integer(unsigned(slv(22 downto 0)))) / 8388608.0;
        return sign * mant * (2.0 ** expo);
    end function;

    -- Encode VHDL real to IEEE 754 float32 bits (positive normalized values only)
    function real_to_f32(r : real) return std_logic_vector is
        variable sign_bit : std_logic;
        variable av       : real;
        variable expo     : integer;
        variable mant     : real;
        variable mant_int : integer;
        variable result   : std_logic_vector(31 downto 0);
    begin
        if r = 0.0 then return (31 downto 0 => '0'); end if;
        if r < 0.0 then sign_bit := '1'; av := -r;
        else             sign_bit := '0'; av :=  r; end if;
        expo := 0;
        while av >= 2.0 loop av := av / 2.0; expo := expo + 1; end loop;
        while av < 1.0 loop av := av * 2.0; expo := expo - 1; end loop;
        mant := av - 1.0;
        mant_int := integer(mant * 8388608.0);
        if mant_int >= 8388608 then  -- mantissa overflow -> increment exponent
            expo     := expo + 1;
            mant_int := 0;
        end if;
        result(31)           := sign_bit;
        result(30 downto 23) := std_logic_vector(to_unsigned(expo + 127, 8));
        result(22 downto 0)  := std_logic_vector(to_unsigned(mant_int, 23));
        return result;
    end function;

begin

    process(x_i)
        variable xi : integer;
        variable yi : integer;
        variable xr : real;
        variable yr : real;
    begin
        xi := to_integer(signed(x_i));
        yi := C_MAGIC - (xi / 2);  -- Quake bit trick (x is always positive float)

        xr := f32_to_real(x_i);
        yr := f32_to_real(std_logic_vector(to_signed(yi, 32)));

        yr := yr * (C_A - C_B * xr * yr * yr);  -- Newton-Raphson

        y_o <= real_to_f32(yr);
    end process;

end architecture Behavioral;
