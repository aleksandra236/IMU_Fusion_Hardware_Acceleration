-- =============================================================================
-- top_zybo.vhd
-- Top-level wrapper za Zybo Z7-10 (xc7z010clg400-1)
-- Sa Clocking Wizard: 125 MHz -> 66 MHz
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity top_zybo is
    port (
        clk_i  : in  std_logic;
        btn_i  : in  std_logic_vector(3 downto 0);
        led_o  : out std_logic_vector(3 downto 0)
    );
end entity top_zybo;

architecture Behavioral of top_zybo is

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
            qz_o    : out signed(17 downto 0)
        );
    end component;

    component clk_wiz_0 is
        port (
            clk_in1  : in  std_logic;
            clk_out1 : out std_logic;
            locked   : out std_logic
        );
    end component;

    signal clk_66     : std_logic;
    signal pll_locked : std_logic;
    signal reset_s    : std_logic;
    signal ready_s    : std_logic;
    signal qw_s       : signed(17 downto 0);
    signal qx_s       : signed(17 downto 0);
    signal qy_s       : signed(17 downto 0);
    signal qz_s       : signed(17 downto 0);

    -- Heartbeat ~1 Hz na 66 MHz (33_000_000 ciklusa po polu)
    signal hb_cnt     : unsigned(25 downto 0) := (others => '0');
    signal hb_led     : std_logic := '0';

    signal start_s    : std_logic := '0';
    signal started    : std_logic := '0';

    constant C_AX   : std_logic_vector(31 downto 0) := X"00000000";
    constant C_AY   : std_logic_vector(31 downto 0) := X"00000000";
    constant C_AZ   : std_logic_vector(31 downto 0) := X"3F800000";
    constant C_HGX  : std_logic_vector(31 downto 0) := X"00000000";
    constant C_HGY  : std_logic_vector(31 downto 0) := X"00000000";
    constant C_HGZ  : std_logic_vector(31 downto 0) := X"3F000000";
    constant C_GX   : signed(15 downto 0) := to_signed(-260, 16);
    constant C_GY   : signed(15 downto 0) := to_signed( 132, 16);
    constant C_GZ   : signed(15 downto 0) := to_signed(-118, 16);
    constant C_GAIN : unsigned(19 downto 0) := to_unsigned(655360, 20);
    constant C_QW   : signed(17 downto 0) := to_signed(65536, 18);
    constant C_QX   : signed(17 downto 0) := to_signed(0, 18);
    constant C_QY   : signed(17 downto 0) := to_signed(0, 18);
    constant C_QZ   : signed(17 downto 0) := to_signed(0, 18);
    constant C_DT   : unsigned(19 downto 0) := to_unsigned(10486, 20);

begin

    u_clkwiz : clk_wiz_0
        port map (
            clk_in1  => clk_i,
            clk_out1 => clk_66,
            locked   => pll_locked
        );

    reset_s <= (not pll_locked) or btn_i(0);

    process(clk_66)
    begin
        if rising_edge(clk_66) then
            if reset_s = '1' then
                start_s <= '0';
                started <= '0';
            else
                if started = '0' then
                    start_s <= '1';
                    started <= '1';
                else
                    start_s <= '0';
                end if;
            end if;
        end if;
    end process;

    u_ahrs : ahrs_sec5to8
        port map (
            clk    => clk_66,  reset  => reset_s,  start  => start_s,
            ax_i   => C_AX,    ay_i   => C_AY,     az_i   => C_AZ,
            hgx_i  => C_HGX,   hgy_i  => C_HGY,    hgz_i  => C_HGZ,
            gx_i   => C_GX,    gy_i   => C_GY,     gz_i   => C_GZ,
            gain_i => C_GAIN,
            qw_i   => C_QW,    qx_i   => C_QX,
            qy_i   => C_QY,    qz_i   => C_QZ,
            dt_i   => C_DT,    ready  => ready_s,
            qw_o   => qw_s,    qx_o   => qx_s,
            qy_o   => qy_s,    qz_o   => qz_s
        );

    process(clk_66)
    begin
        if rising_edge(clk_66) then
            if reset_s = '1' then
                hb_cnt <= (others => '0');
                hb_led <= '0';
            else
                if hb_cnt = to_unsigned(33_000_000, 26) then
                    hb_cnt <= (others => '0');
                    hb_led <= not hb_led;
                else
                    hb_cnt <= hb_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    led_o(0) <= ready_s;
    led_o(1) <= not qw_s(17);
    led_o(2) <= qx_s(17);
    led_o(3) <= hb_led;

end architecture Behavioral;