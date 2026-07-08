-- =============================================================================
-- tb_ahrs_sec678.vhd
-- Testbench za ahrs_sec678 - Sections 6, 7, 8
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_ahrs_sec678 is
end entity tb_ahrs_sec678;

architecture sim of tb_ahrs_sec678 is

    component ahrs_sec678 is
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
            qz_o    : out signed(17 downto 0)
        );
    end component;

    signal clk_s    : std_logic := '0';
    signal reset_s  : std_logic := '1';
    signal start_s  : std_logic := '0';

    signal gx_s     : signed(15 downto 0) := (others => '0');
    signal gy_s     : signed(15 downto 0) := (others => '0');
    signal gz_s     : signed(15 downto 0) := (others => '0');
    signal hfbx_s   : signed(25 downto 0) := (others => '0');
    signal hfby_s   : signed(25 downto 0) := (others => '0');
    signal hfbz_s   : signed(25 downto 0) := (others => '0');
    signal gain_s   : unsigned(19 downto 0) := (others => '0');
    signal qw_in_s  : signed(17 downto 0) := (others => '0');
    signal qx_in_s  : signed(17 downto 0) := (others => '0');
    signal qy_in_s  : signed(17 downto 0) := (others => '0');
    signal qz_in_s  : signed(17 downto 0) := (others => '0');
    signal dt_s     : unsigned(19 downto 0) := (others => '0');

    signal ready_s  : std_logic;
    signal qw_out_s : signed(17 downto 0);
    signal qx_out_s : signed(17 downto 0);
    signal qy_out_s : signed(17 downto 0);
    signal qz_out_s : signed(17 downto 0);

    -- TEST 1 expected (realni sample 1 iz C reference, float->int truncation)
    -- Tolerancija +-1 LSB prihvatljiva zbog float vs fixed-point kvantizacije
    constant EXP_QW : signed(17 downto 0) := to_signed( 65536, 18);
    constant EXP_QX : signed(17 downto 0) := to_signed(-6573,  18);
    constant EXP_QY : signed(17 downto 0) := to_signed(-160,   18);
    constant EXP_QZ : signed(17 downto 0) := to_signed( 21,    18);

    -- TEST 2 expected (realni sample 2 iz C reference)
    constant EXP2_QW : signed(17 downto 0) := to_signed( 64570, 18);
    constant EXP2_QX : signed(17 downto 0) := to_signed(-13060, 18);
    constant EXP2_QY : signed(17 downto 0) := to_signed(-167,   18);
    constant EXP2_QZ : signed(17 downto 0) := to_signed( 31,    18);

    constant TOL : integer := 1;

    constant CLK_PERIOD : time := 10 ns;

begin

    DUT : ahrs_sec678
        port map (
            clk     => clk_s,
            reset   => reset_s,
            start   => start_s,
            gx_i    => gx_s,
            gy_i    => gy_s,
            gz_i    => gz_s,
            hfbx_i  => hfbx_s,
            hfby_i  => hfby_s,
            hfbz_i  => hfbz_s,
            gain_i  => gain_s,
            qw_i    => qw_in_s,
            qx_i    => qx_in_s,
            qy_i    => qy_in_s,
            qz_i    => qz_in_s,
            dt_i    => dt_s,
            ready   => ready_s,
            qw_o    => qw_out_s,
            qx_o    => qx_out_s,
            qy_o    => qy_out_s,
            qz_o    => qz_out_s
        );

    clk_s <= not clk_s after CLK_PERIOD / 2;

    stim : process
    begin
        reset_s <= '1';
        start_s <= '0';
        wait for 3 * CLK_PERIOD;
        reset_s <= '0';
        wait for CLK_PERIOD;

        -- =====================================================================
        -- TEST 1: realni sample 1 iz sensor_data, q=identitet
        -- C reference: gx=-7.766 gy=-10.635 gz=3.608 deg/s, gain=9.968, dt=0.01s
        -- Ocekivano: qw=65536, qx=-6573, qy=-160, qz=21
        -- =====================================================================
        report "=== TEST 1: Sec678, realni sample 1 ===";

        gx_s    <= to_signed(-994,       16);  -- -7.766 deg/s Q9.7
        gy_s    <= to_signed(-1361,      16);  -- -10.635 deg/s Q9.7
        gz_s    <= to_signed(461,        16);  -- 3.608 deg/s Q9.7
        hfbx_s  <= to_signed(-16769862,  26);  -- -0.99956 Q2.24
        hfby_s  <= to_signed(-256100,    26);  -- -0.01526 Q2.24
        hfbz_s  <= to_signed(0,          26);
        gain_s  <= to_unsigned(653284,   20);  -- 9.968 Q4.16
        qw_in_s <= to_signed(65536,      18);  -- 1.0 Q2.16
        qx_in_s <= to_signed(0,          18);
        qy_in_s <= to_signed(0,          18);
        qz_in_s <= to_signed(0,          18);
        dt_s    <= to_unsigned(10485,    20);  -- 0.01s Q0.20
        start_s <= '1';

        wait for CLK_PERIOD;
        start_s <= '0';

        -- Latencija = 1 ciklus
        wait for CLK_PERIOD;
        wait for 1 ns;

        report "Izlaz qw_o = " & integer'image(to_integer(qw_out_s));
        report "Izlaz qx_o = " & integer'image(to_integer(qx_out_s));
        report "Izlaz qy_o = " & integer'image(to_integer(qy_out_s));
        report "Izlaz qz_o = " & integer'image(to_integer(qz_out_s));

        if abs(to_integer(qw_out_s) - to_integer(EXP_QW)) <= TOL then report "qw: PASS";
        else report "qw: FAIL - ocekivano " & integer'image(to_integer(EXP_QW))
                    & " dobijeno " & integer'image(to_integer(qw_out_s)) severity error;
        end if;

        if abs(to_integer(qx_out_s) - to_integer(EXP_QX)) <= TOL then report "qx: PASS";
        else report "qx: FAIL - ocekivano " & integer'image(to_integer(EXP_QX))
                    & " dobijeno " & integer'image(to_integer(qx_out_s)) severity error;
        end if;

        if abs(to_integer(qy_out_s) - to_integer(EXP_QY)) <= TOL then report "qy: PASS";
        else report "qy: FAIL - ocekivano " & integer'image(to_integer(EXP_QY))
                    & " dobijeno " & integer'image(to_integer(qy_out_s)) severity error;
        end if;

        if abs(to_integer(qz_out_s) - to_integer(EXP_QZ)) <= TOL then report "qz: PASS";
        else report "qz: FAIL - ocekivano " & integer'image(to_integer(EXP_QZ))
                    & " dobijeno " & integer'image(to_integer(qz_out_s)) severity error;
        end if;

        -- =====================================================================
        -- TEST 2: realni sample 2, q=normalizovani izlaz iz main.c posle sample 1
        -- C reference: gx=-7.084 gy=-9.529 gz=3.754 deg/s, gain=9.937, dt=0.01s
        -- Ocekivano: qw=64570, qx=-13060, qy=-167, qz=31
        -- =====================================================================
        report "=== TEST 2: Sec678, realni sample 2 ===";

        gx_s    <= to_signed(-906,       16);  -- -7.084 deg/s Q9.7
        gy_s    <= to_signed(-1219,      16);  -- -9.529 deg/s Q9.7
        gz_s    <= to_signed(480,        16);  -- 3.754 deg/s Q9.7
        hfbx_s  <= to_signed(-16766826,  26);  -- -0.99938 Q2.24
        hfby_s  <= to_signed(91143,      26);  -- 0.00543 Q2.24
        hfbz_s  <= to_signed(100339,     26);  -- 0.00598 Q2.24
        gain_s  <= to_unsigned(651209,   20);  -- 9.937 Q4.16
        qw_in_s <= to_signed(65223,      18);  -- normalizovani q iz sample 1
        qx_in_s <= to_signed(-6542,      18);
        qy_in_s <= to_signed(-155,       18);
        qz_in_s <= to_signed(-15,        18);
        dt_s    <= to_unsigned(10485,    20);
        start_s <= '1';

        wait for CLK_PERIOD;
        start_s <= '0';

        wait for CLK_PERIOD;
        wait for 1 ns;

        report "Izlaz qw_o = " & integer'image(to_integer(qw_out_s));
        report "Izlaz qx_o = " & integer'image(to_integer(qx_out_s));
        report "Izlaz qy_o = " & integer'image(to_integer(qy_out_s));
        report "Izlaz qz_o = " & integer'image(to_integer(qz_out_s));

        if abs(to_integer(qw_out_s) - to_integer(EXP2_QW)) <= TOL then report "qw: PASS";
        else report "qw: FAIL - ocekivano " & integer'image(to_integer(EXP2_QW))
                    & " dobijeno " & integer'image(to_integer(qw_out_s)) severity error;
        end if;

        if abs(to_integer(qx_out_s) - to_integer(EXP2_QX)) <= TOL then report "qx: PASS";
        else report "qx: FAIL - ocekivano " & integer'image(to_integer(EXP2_QX))
                    & " dobijeno " & integer'image(to_integer(qx_out_s)) severity error;
        end if;

        if abs(to_integer(qy_out_s) - to_integer(EXP2_QY)) <= TOL then report "qy: PASS";
        else report "qy: FAIL - ocekivano " & integer'image(to_integer(EXP2_QY))
                    & " dobijeno " & integer'image(to_integer(qy_out_s)) severity error;
        end if;

        if abs(to_integer(qz_out_s) - to_integer(EXP2_QZ)) <= TOL then report "qz: PASS";
        else report "qz: FAIL - ocekivano " & integer'image(to_integer(EXP2_QZ))
                    & " dobijeno " & integer'image(to_integer(qz_out_s)) severity error;
        end if;

        report "=== Simulacija zavrsena ===";
        wait;
    end process;

end architecture sim;
