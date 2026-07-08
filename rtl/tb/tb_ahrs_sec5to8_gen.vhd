-- =============================================================================
-- tb_ahrs_sec5to8_gen.vhd
-- Testbench za TOP entitet ahrs_sec5to8 sa STVARNIM test vektorima.
--
-- Struktura:
--   * proces stim prolazi kroz NUM_TV test vektora
--   * za svaki: postavlja ulaze, pulsira start, ceka na ready sa timeout guard
--   * uporedjuje qw/qx/qy/qz izlaz sa ocekivanim vrednostima (Q2.16 signed)
--   * broji PASS/FAIL i na kraju izbacuje sumarni izvestaj
--
-- Test vektori:
--   TV0: bring-up test (ax=0, ay=0, az=1g, mirovanje, hg=(0,0,0.5),
--                       q=identitet, mala ugaona brzina po sve tri ose)
--        - isti ulazi kao u top_zybo.vhd standalone testu na Zybo ploci
--        - HFB = 0 (accel i halfGravity su paralelni, dot >= 0)
--        - test proverava sec678 pipeline sa nultim feedback-om
--
--   TV1: rotacija oko X-ose (mirovanje sa gx = 39 deg/s)
--        - HFB = 0, test proverava kvaternionsku integraciju samo za X osu
--
--   TV2: STVARNI podaci iz senzorskog dataset-a (hw_orientation2506v1.csv, s0)
--        - ax=+0.0016, ay=-0.108, az=-1.005 (mobilni u pocetnom polozaju)
--        - hg=(0,0,0.5), q=identitet
--        - HFB != 0 (znacajno pomeranje po X i Y osi)
--
--   TV3: STVARNI podaci sample 1 (kvaternion je izlaz TV2)
--        - hg izracunat iz kvaterniona TV2 (softverska Sec4)
--
--   TV4: obrnuta orijentacija (kvaternion (0,1,0,0), rotacija 180deg oko X)
--        - proverava sec8 kvaternionsku integraciju u neutralnoj tacki
--
-- Ocekivane vrednosti izracunate SOFTVERSKI (Python skripta gen_tv.py) preko
-- referentnog modela ahrs_sec5to8 u float32 aritmetici. RTL izlaz koristi
-- IEEE 754 float32 kroz Xilinx FP IP jezgra + Q2.16 na kraju, sto uvodi malu
-- razliku (typicno < 500 LSB Q2.16 za slucajeve sa netrivijalnim HFB-om).
--
-- Tolerancija TOL=500 LSB Q2.16 (~0.76% od 1.0) - realno za razlike izmedju
-- float32 IP jezgara i cistog Python float64 modela.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_ahrs_sec5to8_gen is
end entity tb_ahrs_sec5to8_gen;

architecture sim of tb_ahrs_sec5to8_gen is

    component ahrs_sec5to8 is
        port (
            clk     : in  std_logic;
            reset   : in  std_logic;
            start   : in  std_logic;
            ax_i    : in  std_logic_vector(31 downto 0);
            ay_i    : in  std_logic_vector(31 downto 0);
            az_i    : in  std_logic_vector(31 downto 0);
            hgx_i   : in  std_logic_vector(31 downto 0);
            hgy_i   : in  std_logic_vector(31 downto 0);
            hgz_i   : in  std_logic_vector(31 downto 0);
            gx_i    : in  signed(15 downto 0);
            gy_i    : in  signed(15 downto 0);
            gz_i    : in  signed(15 downto 0);
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
            hfbx_dbg : out signed(25 downto 0);
            hfby_dbg : out signed(25 downto 0);
            hfbz_dbg : out signed(25 downto 0);
            dbg_s1_hgx_o   : out signed(21 downto 0);
            dbg_s2_adjgx_o : out signed(25 downto 0);
            dbg_s3b_dqx_o  : out signed(26 downto 0);
            dbg_s4_shx_o   : out signed(31 downto 0)
        );
    end component;

    constant NUM_TV     : integer := 5;
    constant TOL        : integer := 500;    -- LSB Q2.16 (~0.76% od 1.0)
    constant MAX_LAT    : integer := 200;    -- timeout guard u taktovima
    constant CLK_PERIOD : time    := 14 ns;  -- 71.4 MHz (kao u sintezi)

    type slv32_arr is array(0 to NUM_TV-1) of std_logic_vector(31 downto 0);
    type int_arr   is array(0 to NUM_TV-1) of integer;

    -- =========================================================================
    -- TEST VEKTORI (izracunati Python skriptom gen_tv.py)
    --   TV0-TV1: sinteticki (bring-up, jednostavne rotacije)
    --   TV2-TV3: STVARNI podaci iz hw_orientation2506v1.csv (sample 0, 1)
    --   TV4:     rotacija 180deg oko X (mirovanje, obrnuta orijentacija)
    -- =========================================================================

    -- Float32 ulazi (IEEE 754 bit pattern):
    -- TV0: ax=0,          ay=0,           az= 1.0
    -- TV1: ax=0,          ay=0,           az= 1.0
    -- TV2: ax=+0.001648,  ay=-0.107910,   az=-1.005417   (stvarni podaci)
    -- TV3: ax=-0.006607,  ay=-0.104721,   az=-1.008926   (stvarni podaci)
    -- TV4: ax=0,          ay=0,           az=-1.0
    constant TV_AX  : slv32_arr := (x"00000000", x"00000000", x"3AD801B4",
                                    x"BBD87F88", x"00000000");
    constant TV_AY  : slv32_arr := (x"00000000", x"00000000", x"BDDCFFEB",
                                    x"BDD677F7", x"00000000");
    constant TV_AZ  : slv32_arr := (x"3F800000", x"3F800000", x"BF80B181",
                                    x"BF81247D", x"BF800000");

    -- halfGravity (iz softverske Sec4):
    -- TV0-TV2: hg = (0, 0, 0.5)      (pocetni kvaternion je (1,0,0,0))
    -- TV3:     hg izracunat iz kvaterniona TV2 rezultata
    -- TV4:     hg = (0, 0, -0.5)     (kvaternion (0,1,0,0), obrnuta orijentacija)
    constant TV_HGX : slv32_arr := (x"00000000", x"00000000", x"00000000",
                                    x"3B1D7BA6", x"00000000");
    constant TV_HGY : slv32_arr := (x"00000000", x"00000000", x"00000000",
                                    x"BDCB7C35", x"00000000");
    constant TV_HGZ : slv32_arr := (x"3F000000", x"3F000000", x"3F000000",
                                    x"3EFB20DA", x"BF000000");

    -- Fixed-point ulazi:
    -- gx/gy/gz u Q9.7 (128 = 1 deg/s), gain u Q4.16, dt u Q0.20 (10486 = 0.01s),
    -- q_in u Q2.16 (65536 = 1.0)
    constant TV_GX   : int_arr := (-260, 5000, -260, -300, 0);
    constant TV_GY   : int_arr := ( 132,    0,  132,  100, 0);
    constant TV_GZ   : int_arr := (-118,    0, -118,  -80, 0);
    constant TV_GAIN : int_arr := (655360, 655360, 655360, 655360, 32768);
    constant TV_DT   : int_arr := (10486,  10486,  10486,  10486,  10486);

    -- Ulazni kvaternion (Q2.16):
    -- TV0-TV2: identitet (1,0,0,0)
    -- TV3:     izlaz TV2 (izracunat, ~identitet sa malim odstupanjem)
    -- TV4:     (0,1,0,0) obrnuta orijentacija
    constant TV_QW_I : int_arr := (65536, 65536, 65536, 65224, 0);
    constant TV_QX_I : int_arr := (0,     0,     0,    -6543, 65536);
    constant TV_QY_I : int_arr := (0,     0,     0,    -157,  0);
    constant TV_QZ_I : int_arr := (0,     0,     0,    -16,   0);

    -- =========================================================================
    -- OCEKIVANI Q2.16 IZLAZI (izracunato Python skriptom gen_tv.py preko
    -- referentnog algoritma sec5-8 u float32; tolerancija TOL=500 LSB)
    -- =========================================================================
    constant TV_EQW : int_arr := (65536,  65536, 65536, 64569, 0);
    constant TV_EQX : int_arr := (-12,    223,   -6562, -13075, 65536);
    constant TV_EQY : int_arr := (6,      0,     -94,   -112,   0);
    constant TV_EQZ : int_arr := (-5,     0,     -5,    0,      0);

    -- Signali DUT-a
    signal clk_s    : std_logic := '0';
    signal reset_s  : std_logic := '1';
    signal start_s  : std_logic := '0';
    signal ax_s, ay_s, az_s     : std_logic_vector(31 downto 0) := (others => '0');
    signal hgx_s, hgy_s, hgz_s  : std_logic_vector(31 downto 0) := (others => '0');
    signal gx_s, gy_s, gz_s     : signed(15 downto 0) := (others => '0');
    signal gain_s   : unsigned(19 downto 0) := (others => '0');
    signal qw_in_s, qx_in_s, qy_in_s, qz_in_s : signed(17 downto 0) := (others => '0');
    signal dt_s     : unsigned(19 downto 0) := (others => '0');
    signal ready_s  : std_logic;
    signal qw_out_s, qx_out_s, qy_out_s, qz_out_s : signed(17 downto 0);
    signal hfbx_dbg_s, hfby_dbg_s, hfbz_dbg_s     : signed(25 downto 0);
    signal dbg_s1_hgx_s   : signed(21 downto 0);
    signal dbg_s2_adjgx_s : signed(25 downto 0);
    signal dbg_s3b_dqx_s  : signed(26 downto 0);
    signal dbg_s4_shx_s   : signed(31 downto 0);

begin

    DUT : ahrs_sec5to8
        port map (
            clk    => clk_s,     reset  => reset_s,  start  => start_s,
            ax_i   => ax_s,      ay_i   => ay_s,     az_i   => az_s,
            hgx_i  => hgx_s,     hgy_i  => hgy_s,    hgz_i  => hgz_s,
            gx_i   => gx_s,      gy_i   => gy_s,     gz_i   => gz_s,
            gain_i => gain_s,
            qw_i   => qw_in_s,   qx_i   => qx_in_s,
            qy_i   => qy_in_s,   qz_i   => qz_in_s,
            dt_i   => dt_s,      ready  => ready_s,
            qw_o   => qw_out_s,  qx_o   => qx_out_s,
            qy_o   => qy_out_s,  qz_o   => qz_out_s,
            hfbx_dbg => hfbx_dbg_s, hfby_dbg => hfby_dbg_s, hfbz_dbg => hfbz_dbg_s,
            dbg_s1_hgx_o   => dbg_s1_hgx_s,
            dbg_s2_adjgx_o => dbg_s2_adjgx_s,
            dbg_s3b_dqx_o  => dbg_s3b_dqx_s,
            dbg_s4_shx_o   => dbg_s4_shx_s
        );

    clk_s <= not clk_s after CLK_PERIOD / 2;

    -- =========================================================================
    -- STIMULUS + CHECK proces
    -- =========================================================================
    stim : process
        variable pass_cnt : integer := 0;
        variable fail_cnt : integer := 0;
        variable timeout  : integer;
    begin
        report "===============================================";
        report "tb_ahrs_sec5to8_gen: " & integer'image(NUM_TV) & " test vektora"
             & ", TOL = " & integer'image(TOL) & " LSB Q2.16";
        report "===============================================";

        reset_s <= '1';
        start_s <= '0';
        wait for 5 * CLK_PERIOD;
        reset_s <= '0';
        wait for CLK_PERIOD;

        for i in 0 to NUM_TV-1 loop
            -- Postavi ulaze
            ax_s    <= TV_AX(i);
            ay_s    <= TV_AY(i);
            az_s    <= TV_AZ(i);
            hgx_s   <= TV_HGX(i);
            hgy_s   <= TV_HGY(i);
            hgz_s   <= TV_HGZ(i);
            gx_s    <= to_signed(TV_GX(i),   16);
            gy_s    <= to_signed(TV_GY(i),   16);
            gz_s    <= to_signed(TV_GZ(i),   16);
            gain_s  <= to_unsigned(TV_GAIN(i), 20);
            qw_in_s <= to_signed(TV_QW_I(i), 18);
            qx_in_s <= to_signed(TV_QX_I(i), 18);
            qy_in_s <= to_signed(TV_QY_I(i), 18);
            qz_in_s <= to_signed(TV_QZ_I(i), 18);
            dt_s    <= to_unsigned(TV_DT(i), 20);

            -- Pulse start jedan takt
            wait until rising_edge(clk_s);
            start_s <= '1';
            wait until rising_edge(clk_s);
            start_s <= '0';

            -- Cekaj na ready sa timeout-om
            timeout := MAX_LAT;
            while ready_s = '0' and timeout > 0 loop
                wait until rising_edge(clk_s);
                timeout := timeout - 1;
            end loop;

            if timeout = 0 then
                report "TV" & integer'image(i)
                    & ": FAIL - TIMEOUT (ready se nije podigao u "
                    & integer'image(MAX_LAT) & " taktova)"
                    severity error;
                fail_cnt := fail_cnt + 1;
            else
                wait for 1 ns;  -- propagacija do stabilnog izlaza

                if abs(to_integer(qw_out_s) - TV_EQW(i)) <= TOL and
                   abs(to_integer(qx_out_s) - TV_EQX(i)) <= TOL and
                   abs(to_integer(qy_out_s) - TV_EQY(i)) <= TOL and
                   abs(to_integer(qz_out_s) - TV_EQZ(i)) <= TOL then
                    report "TV" & integer'image(i) & ": PASS  qw="
                         & integer'image(to_integer(qw_out_s))
                         & " qx=" & integer'image(to_integer(qx_out_s))
                         & " qy=" & integer'image(to_integer(qy_out_s))
                         & " qz=" & integer'image(to_integer(qz_out_s));
                    pass_cnt := pass_cnt + 1;
                else
                    report "TV" & integer'image(i) & ": FAIL"
                         & "  qw=" & integer'image(to_integer(qw_out_s))
                         & "(exp " & integer'image(TV_EQW(i)) & ")"
                         & "  qx=" & integer'image(to_integer(qx_out_s))
                         & "(exp " & integer'image(TV_EQX(i)) & ")"
                         & "  qy=" & integer'image(to_integer(qy_out_s))
                         & "(exp " & integer'image(TV_EQY(i)) & ")"
                         & "  qz=" & integer'image(to_integer(qz_out_s))
                         & "(exp " & integer'image(TV_EQZ(i)) & ")"
                        severity error;
                    fail_cnt := fail_cnt + 1;
                end if;
            end if;

            wait for 2 * CLK_PERIOD;
        end loop;

        report "===============================================";
        report "REZULTAT: " & integer'image(pass_cnt) & "/"
            & integer'image(NUM_TV) & " PASS, "
            & integer'image(fail_cnt) & " FAIL";
        report "===============================================";

        if fail_cnt = 0 then
            report "SVI TESTOVI PROSLI - TB PASSED" severity note;
        else
            report integer'image(fail_cnt) & " test(ov)a nije proslo - TB FAILED"
                severity failure;
        end if;

        wait;
    end process;

end architecture sim;
