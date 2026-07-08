-- =============================================================================
-- ahrs_invSqrt_synth.vhd
-- Fast Inverse Square Root IP - IEEE 754 single precision -- SYNTHESIS VERSION
--
-- Algorithm: Quake III bit trick + 1x Newton-Raphson iteration
--   initial:  y = reinterpret(0x5F1F1412 - (reinterpret_int(x) >> 1))
--   refine:   y = y * (1.69000231 - 0.714158168 * x * y * y)
--
-- Koristi ieee.float_pkg (VHDL 2008) - sintetizabilan u Vivado 2022.2
-- Za simulaciju koristi ahrs_invSqrt.vhd (real verzija)
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.float_pkg.all;

entity ahrs_invSqrt is
    port (
        x_i : in  std_logic_vector(31 downto 0);
        y_o : out std_logic_vector(31 downto 0)
    );
end entity ahrs_invSqrt;

architecture Behavioral of ahrs_invSqrt is

    constant C_MAGIC : signed(31 downto 0) := to_signed(16#5F1F1412#, 32);
    constant C_NR_A  : float32 := to_float(1.69000231,  8, 23);
    constant C_NR_B  : float32 := to_float(0.714158168, 8, 23);

begin

    process(x_i)
        variable xi : signed(31 downto 0);
        variable yi : signed(31 downto 0);
        variable xr : float32;
        variable yr : float32;
    begin
        xi := signed(x_i);
        yi := C_MAGIC - shift_right(xi, 1);  -- Quake bit trick

        xr := to_float(x_i,                        8, 23);
        yr := to_float(std_logic_vector(yi),        8, 23);

        yr := yr * (C_NR_A - C_NR_B * xr * yr * yr);  -- Newton-Raphson

        y_o <= to_slv(yr);
    end process;

end architecture Behavioral;
