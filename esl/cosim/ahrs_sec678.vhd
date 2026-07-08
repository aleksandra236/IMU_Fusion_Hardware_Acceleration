-- =============================================================================
-- ahrs_sec678.vhd
-- AHRS Hardware IP - Sections 6, 7, 8
--
-- Section 6: Gyroscope conversion  gx [deg/s] -> halfGyro [rad/s * 0.5]
-- Section 7: Apply accel feedback  adjHalfGyro = halfGyro + hfb * gain
-- Section 8: Quaternion integration q_new = q + (q x adjHalfGyro) * dt
--
-- Fixed-point formats:
--   gx_i   : signed Q9.7  (16-bit)
--   hfb*_i : signed Q2.24 (26-bit)
--   gain_i : unsigned Q4.16 (20-bit)
--   q*_i   : signed Q2.16 (18-bit)
--   dt_i   : unsigned Q0.20 (20-bit)
--   q*_o   : signed Q2.16 (18-bit)
--
-- Pipeline: 5 stage (sec6 | sec7 | sec8a_mul | sec8a_acc | sec8b+out)
-- Latency:  5 clock cycles  (bio 4, +1 zbog razbijanja Stage 3)
--
-- OPTIMIZACIJA vs originala:
--   Originalni Stage 3 je radio 12x mnozenje + 4x akumulacija u jednom ciklusu.
--   Razbijen je na:
--     Stage 3a - samo 12 mnozenja, registruje sve produkte
--     Stage 3b - samo 4 akumulacije + slicing
--   Kriticni put se smanjuje ~50% u ovom delu, sto omogucava visu frekvenciju.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ahrs_sec678 is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
        start   : in  std_logic;

        gx_i    : in  signed(15 downto 0);
        gy_i    : in  signed(15 downto 0);
        gz_i    : in  signed(15 downto 0);

        hfbx_i  : in  signed(25 downto 0);
        hfby_i  : in  signed(25 downto 0);
        hfbz_i  : in  signed(25 downto 0);

        gain_i  : in  unsigned(19 downto 0);

        qw_i    : in  signed(17 downto 0);
        qx_i    : in  signed(17 downto 0);
        qy_i    : in  signed(17 downto 0);
        qz_i    : in  signed(17 downto 0);

        dt_i    : in  unsigned(19 downto 0);

        ready   : out std_logic;
        qw_o    : out signed(17 downto 0);
        qx_o    : out signed(17 downto 0);
        qy_o    : out signed(17 downto 0);
        qz_o    : out signed(17 downto 0);

        -- DEBUG: interni signali za lokalizaciju buga
        dbg_s1_hgx_o   : out signed(21 downto 0);  -- sec6 izlaz (Q2.20)
        dbg_s2_adjgx_o : out signed(25 downto 0);  -- sec7 izlaz (Q5.21)
        dbg_s3b_dqx_o  : out signed(26 downto 0);  -- sec8a izlaz (Q5.22)
        dbg_s4_shx_o   : out signed(31 downto 0)   -- Stage 4: px >> 26 pre resize
    );
end entity ahrs_sec678;

architecture Behavioral of ahrs_sec678 is

    attribute use_dsp : string;
    attribute use_dsp of Behavioral : architecture is "yes";

    constant C_PI360 : signed(20 downto 0) := to_signed(9150, 21);

    -- -------------------------------------------------------------------------
    -- Stage 1 registers (izlaz sec6)
    -- -------------------------------------------------------------------------
    signal s1_hgx   : signed(21 downto 0) := (others => '0');
    signal s1_hgy   : signed(21 downto 0) := (others => '0');
    signal s1_hgz   : signed(21 downto 0) := (others => '0');

    -- KRITICNO: bez keep/dont_touch, Vivado absorbuje s1_hgx u DSP48E1 i ignorise
    -- slice (28 downto 7) - rezultat je 64x veca vrednost u Stage 2.
    -- Sa keep attributom, sinteza mora napraviti pravi flip-flop sa slice logikom.
    attribute keep       : string;
    attribute dont_touch : string;
    attribute keep       of s1_hgx : signal is "true";
    attribute keep       of s1_hgy : signal is "true";
    attribute keep       of s1_hgz : signal is "true";
    attribute dont_touch of s1_hgx : signal is "true";
    attribute dont_touch of s1_hgy : signal is "true";
    attribute dont_touch of s1_hgz : signal is "true";

    signal s1_hfbx  : signed(25 downto 0) := (others => '0');
    signal s1_hfby  : signed(25 downto 0) := (others => '0');
    signal s1_hfbz  : signed(25 downto 0) := (others => '0');
    signal s1_gain  : unsigned(19 downto 0) := (others => '0');
    signal s1_qw    : signed(17 downto 0) := (others => '0');
    signal s1_qx    : signed(17 downto 0) := (others => '0');
    signal s1_qy    : signed(17 downto 0) := (others => '0');
    signal s1_qz    : signed(17 downto 0) := (others => '0');
    signal s1_dt    : unsigned(19 downto 0) := (others => '0');
    signal s1_valid : std_logic := '0';

    -- -------------------------------------------------------------------------
    -- Stage 2 registers (izlaz sec7)
    -- -------------------------------------------------------------------------
    signal s2_adjgx : signed(25 downto 0) := (others => '0');
    signal s2_adjgy : signed(25 downto 0) := (others => '0');
    signal s2_adjgz : signed(25 downto 0) := (others => '0');
    signal s2_qw    : signed(17 downto 0) := (others => '0');
    signal s2_qx    : signed(17 downto 0) := (others => '0');
    signal s2_qy    : signed(17 downto 0) := (others => '0');
    signal s2_qz    : signed(17 downto 0) := (others => '0');
    signal s2_dt    : unsigned(19 downto 0) := (others => '0');
    signal s2_valid : std_logic := '0';

    -- -------------------------------------------------------------------------
    -- Stage 3a registers (izlaz sec8a - samo produkti, 12x mnozenje)
    -- q Q2.16 (18-bit) x adjhg Q5.21 (26-bit) -> Q7.37 (44-bit)
    -- -------------------------------------------------------------------------
    signal s3a_p00  : signed(43 downto 0) := (others => '0'); -- qw * adjgx
    signal s3a_p01  : signed(43 downto 0) := (others => '0'); -- qw * adjgy
    signal s3a_p02  : signed(43 downto 0) := (others => '0'); -- qw * adjgz
    signal s3a_p10  : signed(43 downto 0) := (others => '0'); -- qx * adjgx
    signal s3a_p11  : signed(43 downto 0) := (others => '0'); -- qx * adjgy
    signal s3a_p12  : signed(43 downto 0) := (others => '0'); -- qx * adjgz
    signal s3a_p20  : signed(43 downto 0) := (others => '0'); -- qy * adjgx
    signal s3a_p21  : signed(43 downto 0) := (others => '0'); -- qy * adjgy
    signal s3a_p22  : signed(43 downto 0) := (others => '0'); -- qy * adjgz
    signal s3a_p30  : signed(43 downto 0) := (others => '0'); -- qz * adjgx
    signal s3a_p31  : signed(43 downto 0) := (others => '0'); -- qz * adjgy
    signal s3a_p32  : signed(43 downto 0) := (others => '0'); -- qz * adjgz
    -- propagacija
    signal s3a_qw   : signed(17 downto 0) := (others => '0');
    signal s3a_qx   : signed(17 downto 0) := (others => '0');
    signal s3a_qy   : signed(17 downto 0) := (others => '0');
    signal s3a_qz   : signed(17 downto 0) := (others => '0');
    signal s3a_dt   : unsigned(19 downto 0) := (others => '0');
    signal s3a_valid : std_logic := '0';

    -- -------------------------------------------------------------------------
    -- Stage 3b registers (izlaz sec8a - akumulacije + slicing)
    -- dq Q5.22 (27-bit)
    -- -------------------------------------------------------------------------
    signal s3b_dqw  : signed(26 downto 0) := (others => '0');
    signal s3b_dqx  : signed(26 downto 0) := (others => '0');
    signal s3b_dqy  : signed(26 downto 0) := (others => '0');
    signal s3b_dqz  : signed(26 downto 0) := (others => '0');
    signal s3b_qw   : signed(17 downto 0) := (others => '0');
    signal s3b_qx   : signed(17 downto 0) := (others => '0');
    signal s3b_qy   : signed(17 downto 0) := (others => '0');
    signal s3b_qz   : signed(17 downto 0) := (others => '0');
    signal s3b_dt   : unsigned(19 downto 0) := (others => '0');
    signal s3b_valid : std_logic := '0';

begin

    -- =========================================================================
    -- STAGE 1: Section 6 - halfGyro = gx * (pi/360)
    -- gx Q9.7 x C_PI360 Q1.20 -> Q10.27, uzmi [28:7] = Q2.20 (22-bit)
    -- =========================================================================
    process(clk)
        variable p_x, p_y, p_z : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            if reset = '1' then
                s1_hgx   <= (others => '0');
                s1_hgy   <= (others => '0');
                s1_hgz   <= (others => '0');
                s1_hfbx  <= (others => '0');
                s1_hfby  <= (others => '0');
                s1_hfbz  <= (others => '0');
                s1_gain  <= (others => '0');
                s1_qw    <= (others => '0');
                s1_qx    <= (others => '0');
                s1_qy    <= (others => '0');
                s1_qz    <= (others => '0');
                s1_dt    <= (others => '0');
                s1_valid <= '0';
            else
                p_x := gx_i * C_PI360;
                p_y := gy_i * C_PI360;
                p_z := gz_i * C_PI360;
                s1_hgx   <= p_x(28 downto 7);
                s1_hgy   <= p_y(28 downto 7);
                s1_hgz   <= p_z(28 downto 7);
                s1_hfbx  <= hfbx_i;
                s1_hfby  <= hfby_i;
                s1_hfbz  <= hfbz_i;
                s1_gain  <= gain_i;
                s1_qw    <= qw_i;
                s1_qx    <= qx_i;
                s1_qy    <= qy_i;
                s1_qz    <= qz_i;
                s1_dt    <= dt_i;
                s1_valid <= start;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- STAGE 2: Section 7 - adjHalfGyro = halfGyro + hfb * gain
    -- hfb Q2.24 x gain Q4.16 -> Q6.40, uzmi [44:19] = Q5.21 (26-bit)
    -- hg Q2.20 -> Q5.21: shift left 1
    -- =========================================================================
    process(clk)
        variable gain_s                : signed(20 downto 0);
        variable p_x, p_y, p_z        : signed(46 downto 0);
        variable cx, cy, cz            : signed(25 downto 0);
        variable hgx_a, hgy_a, hgz_a  : signed(25 downto 0);
    begin
        if rising_edge(clk) then
            if reset = '1' then
                s2_adjgx <= (others => '0');
                s2_adjgy <= (others => '0');
                s2_adjgz <= (others => '0');
                s2_qw    <= (others => '0');
                s2_qx    <= (others => '0');
                s2_qy    <= (others => '0');
                s2_qz    <= (others => '0');
                s2_dt    <= (others => '0');
                s2_valid <= '0';
            else
                gain_s := signed('0' & s1_gain);
                p_x := s1_hfbx * gain_s;
                p_y := s1_hfby * gain_s;
                p_z := s1_hfbz * gain_s;
                cx := resize(shift_right(p_x, 19), 26);   -- bilo 25
                cy := resize(shift_right(p_y, 19), 26);
                cz := resize(shift_right(p_z, 19), 26);
                hgx_a := shift_left(resize(s1_hgx, 26), 1);
                hgy_a := shift_left(resize(s1_hgy, 26), 1);
                hgz_a := shift_left(resize(s1_hgz, 26), 1);
                s2_adjgx <= hgx_a + cx;
                s2_adjgy <= hgy_a + cy;
                s2_adjgz <= hgz_a + cz;
                s2_qw    <= s1_qw;
                s2_qx    <= s1_qx;
                s2_qy    <= s1_qy;
                s2_qz    <= s1_qz;
                s2_dt    <= s1_dt;
                s2_valid <= s1_valid;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- STAGE 3a: Section 8a (deo 1) - samo 12 mnozenja, registruj produkte
    -- q Q2.16 (18-bit) x adjhg Q5.21 (26-bit) -> Q7.37 (44-bit)
    --
    -- Kriticni put ovde: samo jedan 18x26 DSP48 mnozac (~4-5ns na Zybo)
    -- =========================================================================
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                s3a_p00  <= (others => '0');
                s3a_p01  <= (others => '0');
                s3a_p02  <= (others => '0');
                s3a_p10  <= (others => '0');
                s3a_p11  <= (others => '0');
                s3a_p12  <= (others => '0');
                s3a_p20  <= (others => '0');
                s3a_p21  <= (others => '0');
                s3a_p22  <= (others => '0');
                s3a_p30  <= (others => '0');
                s3a_p31  <= (others => '0');
                s3a_p32  <= (others => '0');
                s3a_qw   <= (others => '0');
                s3a_qx   <= (others => '0');
                s3a_qy   <= (others => '0');
                s3a_qz   <= (others => '0');
                s3a_dt   <= (others => '0');
                s3a_valid <= '0';
            else
                -- Sva mnozenja su nezavisna - Vivado ih mapira na 12 paralelnih DSP48
                s3a_p00 <= s2_qw * s2_adjgx;
                s3a_p01 <= s2_qw * s2_adjgy;
                s3a_p02 <= s2_qw * s2_adjgz;
                s3a_p10 <= s2_qx * s2_adjgx;
                s3a_p11 <= s2_qx * s2_adjgy;
                s3a_p12 <= s2_qx * s2_adjgz;
                s3a_p20 <= s2_qy * s2_adjgx;
                s3a_p21 <= s2_qy * s2_adjgy;
                s3a_p22 <= s2_qy * s2_adjgz;
                s3a_p30 <= s2_qz * s2_adjgx;
                s3a_p31 <= s2_qz * s2_adjgy;
                s3a_p32 <= s2_qz * s2_adjgz;
                -- propagacija
                s3a_qw   <= s2_qw;
                s3a_qx   <= s2_qx;
                s3a_qy   <= s2_qy;
                s3a_qz   <= s2_qz;
                s3a_dt   <= s2_dt;
                s3a_valid <= s2_valid;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- STAGE 3b: Section 8a (deo 2) - samo akumulacije + slicing
    -- Ulazi su vec registrovani produkti iz 3a, nema mnozenja ovde.
    -- Kriticni put: 3-operand sabiranje 44-bit vrednosti (~3ns carry chain)
    --
    -- sw = -(p10 + p21 + p32)         uzmi [41:15] -> 27-bit
    -- sx =   p00 + p22 - p31          uzmi [41:15] -> 27-bit
    -- sy =   p01 - p12 + p30          uzmi [41:15] -> 27-bit
    -- sz =   p02 + p11 - p20          uzmi [41:15] -> 27-bit
    -- =========================================================================
    process(clk)
        variable sw, sx, sy, sz : signed(45 downto 0);
    begin
        if rising_edge(clk) then
            if reset = '1' then
                s3b_dqw  <= (others => '0');
                s3b_dqx  <= (others => '0');
                s3b_dqy  <= (others => '0');
                s3b_dqz  <= (others => '0');
                s3b_qw   <= (others => '0');
                s3b_qx   <= (others => '0');
                s3b_qy   <= (others => '0');
                s3b_qz   <= (others => '0');
                s3b_dt   <= (others => '0');
                s3b_valid <= '0';
            else
                sw := -(resize(s3a_p10, 46) + resize(s3a_p21, 46) + resize(s3a_p32, 46));
                sx :=   resize(s3a_p00, 46) + resize(s3a_p22, 46) - resize(s3a_p31, 46);
                sy :=   resize(s3a_p01, 46) - resize(s3a_p12, 46) + resize(s3a_p30, 46);
                sz :=   resize(s3a_p02, 46) + resize(s3a_p11, 46) - resize(s3a_p20, 46);
                s3b_dqw  <= sw(41 downto 15);
                s3b_dqx  <= sx(41 downto 15);
                s3b_dqy  <= sy(41 downto 15);
                s3b_dqz  <= sz(41 downto 15);
                s3b_qw   <= s3a_qw;
                s3b_qx   <= s3a_qx;
                s3b_qy   <= s3a_qy;
                s3b_qz   <= s3a_qz;
                s3b_dt   <= s3a_dt;
                s3b_valid <= s3a_valid;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- STAGE 4 (bio STAGE 4, sada Stage 5): Section 8b - Euler integration
    -- dq Q5.22 (27-bit) x dt Q0.20 (20-bit) -> Q5.42 (48-bit)
    -- uzmi [43:26] = Q2.16 (18-bit)
    -- =========================================================================
    process(clk)
        variable dt_s              : signed(20 downto 0);
        variable pw, px, py, pz    : signed(47 downto 0);
    begin
        if rising_edge(clk) then
            if reset = '1' then
                ready <= '0';
                qw_o  <= (others => '0');
                qx_o  <= (others => '0');
                qy_o  <= (others => '0');
                qz_o  <= (others => '0');
            else
                dt_s := signed('0' & s3b_dt);
                pw := s3b_dqw * dt_s;
                px := s3b_dqx * dt_s;
                py := s3b_dqy * dt_s;
                pz := s3b_dqz * dt_s;
                qw_o <= s3b_qw + resize(shift_right(pw, 26), 18);   -- bilo 32
                qx_o <= s3b_qx + resize(shift_right(px, 26), 18);
                qy_o <= s3b_qy + resize(shift_right(py, 26), 18);
                qz_o <= s3b_qz + resize(shift_right(pz, 26), 18);
                ready <= s3b_valid;   
            end if;
        end if;
    end process;

    -- =========================================================================
    -- DEBUG: izvedi interne signale na izlazne portove
    -- =========================================================================
    dbg_s1_hgx_o   <= s1_hgx;
    dbg_s2_adjgx_o <= s2_adjgx;
    dbg_s3b_dqx_o  <= s3b_dqx;

    -- Za Stage 4 pre-resize signal: izracunaj kombinaciono iz s3b_dqx i s3b_dt.
    -- (Stage 4 px je variable, pa moramo ponoviti racun za debug izlaz)
    process(s3b_dqx, s3b_dt)
        variable dt_s : signed(20 downto 0);
        variable px   : signed(47 downto 0);
        variable sh   : signed(47 downto 0);
    begin
        dt_s := signed('0' & s3b_dt);
        px := s3b_dqx * dt_s;
        sh := shift_right(px, 26);
        dbg_s4_shx_o <= sh(31 downto 0);
    end process;

end architecture Behavioral;