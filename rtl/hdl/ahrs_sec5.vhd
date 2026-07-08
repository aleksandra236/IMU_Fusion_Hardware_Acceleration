-- =============================================================================
-- ahrs_sec5.vhd
-- AHRS Section 5: Accelerometer Feedback
--
-- Algorithm (matches C fast_inv_sqrt + cross/dot logic):
--   if accel != 0:
--     normAccel = accel * invSqrt(|accel|^2)       <- inv_sqrt_f32 function
--     cross     = normAccel x halfGravity
--     if dot(normAccel, halfGravity) < 0:
--       cross   = cross * invSqrt(|cross|^2)        <- inv_sqrt_f32 function
--     hfb = cross
--
-- Implementation: VHDL real arithmetic (no ieee.float_pkg)
-- Format : IEEE 754 float inputs, Q2.24 signed 26-bit outputs
-- Latency: combinational
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ahrs_sec5 is
    port (
        clk_i   : in  std_logic;
        rst_i   : in  std_logic;
        start_i : in  std_logic;
        ax_i    : in  std_logic_vector(31 downto 0);
        ay_i    : in  std_logic_vector(31 downto 0);
        az_i    : in  std_logic_vector(31 downto 0);
        hgx_i   : in  std_logic_vector(31 downto 0);
        hgy_i   : in  std_logic_vector(31 downto 0);
        hgz_i   : in  std_logic_vector(31 downto 0);
        hfbx_o  : out signed(25 downto 0);
        hfby_o  : out signed(25 downto 0);
        hfbz_o  : out signed(25 downto 0);
        done_o  : out std_logic
    );
end entity ahrs_sec5;

architecture Behavioral of ahrs_sec5 is

    constant C_MAGIC : integer := 16#5F1F1412#;
    constant C_NR_A  : real    := 1.69000231;
    constant C_NR_B  : real    := 0.714158168;

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

    -- Fast inverse square root (Quake + 1x Newton-Raphson), input as real
    function inv_sqrt_f32(x : real) return real is
        variable xi : integer;
        variable yi : integer;
        variable xr : real;
        variable yr : real;
        variable x_slv : std_logic_vector(31 downto 0);
        variable y_slv : std_logic_vector(31 downto 0);
        variable sign_b : std_logic;
        variable expo   : integer;
        variable mant   : real;
        variable mant_i : integer;
    begin
        -- Encode x as float32 to get the integer bit pattern for the bit trick
        sign_b := '0';
        expo   := 0;
        mant   := x;
        while mant >= 2.0 loop mant := mant / 2.0; expo := expo + 1; end loop;
        while mant < 1.0 and mant > 0.0 loop mant := mant * 2.0; expo := expo - 1; end loop;
        mant   := mant - 1.0;
        mant_i := integer(mant * 8388608.0);
        if mant_i >= 8388608 then expo := expo + 1; mant_i := 0; end if;
        x_slv(31)           := sign_b;
        x_slv(30 downto 23) := std_logic_vector(to_unsigned(expo + 127, 8));
        x_slv(22 downto 0)  := std_logic_vector(to_unsigned(mant_i, 23));

        -- Quake bit trick
        xi := to_integer(signed(x_slv));
        yi := C_MAGIC - (xi / 2);

        -- Decode yi as float32
        y_slv := std_logic_vector(to_signed(yi, 32));
        expo  := to_integer(unsigned(y_slv(30 downto 23))) - 127;
        mant  := 1.0 + real(to_integer(unsigned(y_slv(22 downto 0)))) / 8388608.0;
        yr    := mant * (2.0 ** expo);

        -- Newton-Raphson
        yr := yr * (C_NR_A - C_NR_B * x * yr * yr);
        return yr;
    end function;

begin

    done_o <= '1';  -- kombinaciona verzija: uvek spremna

    process(ax_i, ay_i, az_i, hgx_i, hgy_i, hgz_i)
        variable ax, ay, az     : real;
        variable hgx, hgy, hgz : real;
        variable accelMagSq     : real;
        variable accelInvMag    : real;
        variable nax, nay, naz  : real;
        variable cx, cy, cz     : real;
        variable dot            : real;
        variable crossMagSq     : real;
        variable crossInvMag    : real;
        variable hfbX, hfbY, hfbZ : real;
        variable hfbX_i, hfbY_i, hfbZ_i : integer;
    begin
        ax  := f32_to_real(ax_i);
        ay  := f32_to_real(ay_i);
        az  := f32_to_real(az_i);
        hgx := f32_to_real(hgx_i);
        hgy := f32_to_real(hgy_i);
        hgz := f32_to_real(hgz_i);

        if ax = 0.0 and ay = 0.0 and az = 0.0 then
            hfbX := 0.0;
            hfbY := 0.0;
            hfbZ := 0.0;
        else
            accelMagSq  := ax*ax + ay*ay + az*az;
            accelInvMag := inv_sqrt_f32(accelMagSq);

            nax := ax * accelInvMag;
            nay := ay * accelInvMag;
            naz := az * accelInvMag;

            cx  := nay * hgz - naz * hgy;
            cy  := naz * hgx - nax * hgz;
            cz  := nax * hgy - nay * hgx;

            dot := nax * hgx + nay * hgy + naz * hgz;

            if dot < 0.0 then
                crossMagSq  := cx*cx + cy*cy + cz*cz;
                crossInvMag := inv_sqrt_f32(crossMagSq);
                cx := cx * crossInvMag;
                cy := cy * crossInvMag;
                cz := cz * crossInvMag;
            end if;

            hfbX := cx;
            hfbY := cy;
            hfbZ := cz;
        end if;

        -- Convert to Q2.24 (truncate toward zero, matches C (int) cast)
        hfbX_i := integer(hfbX * 16777216.0);
        hfbY_i := integer(hfbY * 16777216.0);
        hfbZ_i := integer(hfbZ * 16777216.0);

        hfbx_o <= to_signed(hfbX_i, 26);
        hfby_o <= to_signed(hfbY_i, 26);
        hfbz_o <= to_signed(hfbZ_i, 26);
    end process;

end architecture Behavioral;
