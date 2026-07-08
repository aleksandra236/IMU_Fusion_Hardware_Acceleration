-- =============================================================================
-- ahrs_sec5to8.vhd
-- Top-level AHRS IP: Sections 5 through 8
--
-- Structural: ahrs_sec5 (float accel feedback) -> ahrs_sec678 (fixed-point)
-- Simulacija: sec5 je kombinaciona, done_o='1' uvek -> sec678_start = start AND done = start (puls)
-- Sinteza:    sec5 je state machine (~110 ciklusa), sec678 starta kada sec5 zavrsi
--             start_pending registar cuva start signal dok sec5 ne zavrsi
-- =============================================================================
-- =============================================================================
-- ahrs_sec5to8.vhd
-- Top-level AHRS IP: Sections 5 through 8
--
-- Structural: ahrs_sec5 (float accel feedback) -> ahrs_sec678 (fixed-point)
-- Simulacija: sec5 je kombinaciona, done_o='1' uvek -> sec678_start = start AND done = start (puls)
-- Sinteza:    sec5 je state machine (~110 ciklusa), sec678 starta kada sec5 zavrsi
--             start_pending registar cuva start signal dok sec5 ne zavrsi
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ahrs_sec5to8 is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
        start   : in  std_logic;

        -- Sec5: accelerometer (IEEE 754 float, g units)
        ax_i    : in  std_logic_vector(31 downto 0);
        ay_i    : in  std_logic_vector(31 downto 0);
        az_i    : in  std_logic_vector(31 downto 0);

        -- Sec5: half gravity from SW Section 4 (IEEE 754 float)
        hgx_i   : in  std_logic_vector(31 downto 0);
        hgy_i   : in  std_logic_vector(31 downto 0);
        hgz_i   : in  std_logic_vector(31 downto 0);

        -- Sec6: gyroscope Q9.7 signed 16-bit
        gx_i    : in  signed(15 downto 0);
        gy_i    : in  signed(15 downto 0);
        gz_i    : in  signed(15 downto 0);

        -- Sec7: ramped gain Q4.16 unsigned 20-bit
        gain_i  : in  unsigned(19 downto 0);

        -- Sec8: current quaternion Q2.16 signed 18-bit
        qw_i    : in  signed(17 downto 0);
        qx_i    : in  signed(17 downto 0);
        qy_i    : in  signed(17 downto 0);
        qz_i    : in  signed(17 downto 0);

        -- Sec8: delta time Q0.20 unsigned 20-bit
        dt_i    : in  unsigned(19 downto 0);

        -- Output: unnormalized quaternion Q2.16 signed 18-bit
        ready   : out std_logic;
        qw_o    : out signed(17 downto 0);
        qx_o    : out signed(17 downto 0);
        qy_o    : out signed(17 downto 0);
        qz_o    : out signed(17 downto 0);
        
        -- DEBUG: izlaz hfb iz sec5 za dijagnostiku
        hfbx_dbg : out signed(25 downto 0);
        hfby_dbg : out signed(25 downto 0);
        hfbz_dbg : out signed(25 downto 0);

        -- DEBUG: interni signali sec678 za lokalizaciju buga
        dbg_s1_hgx_o   : out signed(21 downto 0);
        dbg_s2_adjgx_o : out signed(25 downto 0);
        dbg_s3b_dqx_o  : out signed(26 downto 0);
        dbg_s4_shx_o   : out signed(31 downto 0)
    );
end entity ahrs_sec5to8;

architecture Structural of ahrs_sec5to8 is

    component ahrs_sec5 is
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
    end component;

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
            qz_o    : out signed(17 downto 0);
            dbg_s1_hgx_o   : out signed(21 downto 0);
            dbg_s2_adjgx_o : out signed(25 downto 0);
            dbg_s3b_dqx_o  : out signed(26 downto 0);
            dbg_s4_shx_o   : out signed(31 downto 0)
        );
    end component;

    signal hfbx_s       : signed(25 downto 0);
    signal hfby_s       : signed(25 downto 0);
    signal hfbz_s       : signed(25 downto 0);
    signal sec5_done    : std_logic;
    signal sec678_start : std_logic;
    signal start_pending: std_logic := '0';

    -- ============================================================
    -- ILA DEBUG: mark hfb signals for chipscope debugging
    -- ============================================================
    attribute mark_debug : string;
    attribute keep       : string;
    
    attribute mark_debug of hfbx_s       : signal is "true";
    attribute keep       of hfbx_s       : signal is "true";
    
    attribute mark_debug of hfby_s       : signal is "true";
    attribute keep       of hfby_s       : signal is "true";
    
    attribute mark_debug of hfbz_s       : signal is "true";
    attribute keep       of hfbz_s       : signal is "true";
    
    attribute mark_debug of sec5_done    : signal is "true";
    attribute keep       of sec5_done    : signal is "true";
    
    attribute mark_debug of sec678_start : signal is "true";
    attribute keep       of sec678_start : signal is "true";
    
    attribute mark_debug of start_pending: signal is "true";
    attribute keep       of start_pending: signal is "true";


begin

    -- Registar koji cuva start signal dok sec5 ne zavrsi
    -- U simulaciji: sec5_done='1' odmah -> start_pending='1' samo jedan ciklus -> sec678_start puls
    -- U sintezi:    start_pending='1' ~110 ciklusa, sec5_done pulsuje -> sec678_start puls
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                start_pending <= '0';
            elsif start = '1' then
                start_pending <= '1';
            elsif sec5_done = '1' then
                start_pending <= '0';
            end if;
        end if;
    end process;

    sec678_start <= sec5_done and start_pending;

    sec5 : ahrs_sec5
        port map (
            clk_i   => clk,
            rst_i   => reset,
            start_i => start,
            ax_i    => ax_i,   ay_i   => ay_i,   az_i   => az_i,
            hgx_i   => hgx_i,  hgy_i  => hgy_i,  hgz_i  => hgz_i,
            hfbx_o  => hfbx_s, hfby_o => hfby_s, hfbz_o => hfbz_s,
            done_o  => sec5_done
        );

    sec678 : ahrs_sec678
        port map (
            clk    => clk,    reset  => reset,
            start  => sec678_start,
            gx_i   => gx_i,   gy_i   => gy_i,   gz_i   => gz_i,
            hfbx_i => hfbx_s, hfby_i => hfby_s, hfbz_i => hfbz_s,
            gain_i => gain_i,
            qw_i   => qw_i,   qx_i   => qx_i,
            qy_i   => qy_i,   qz_i   => qz_i,
            dt_i   => dt_i,   ready  => ready,
            qw_o   => qw_o,   qx_o   => qx_o,
            qy_o   => qy_o,   qz_o   => qz_o,
            dbg_s1_hgx_o   => dbg_s1_hgx_o,
            dbg_s2_adjgx_o => dbg_s2_adjgx_o,
            dbg_s3b_dqx_o  => dbg_s3b_dqx_o,
            dbg_s4_shx_o   => dbg_s4_shx_o
        );
        -- DEBUG: izlaz internih hfb signala
    hfbx_dbg <= hfbx_s;
    hfby_dbg <= hfby_s;
    hfbz_dbg <= hfbz_s;

end architecture Structural;
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ahrs_sec5to8 is
    port (
        clk     : in  std_logic;
        reset   : in  std_logic;
        start   : in  std_logic;

        -- Sec5: accelerometer (IEEE 754 float, g units)
        ax_i    : in  std_logic_vector(31 downto 0);
        ay_i    : in  std_logic_vector(31 downto 0);
        az_i    : in  std_logic_vector(31 downto 0);

        -- Sec5: half gravity from SW Section 4 (IEEE 754 float)
        hgx_i   : in  std_logic_vector(31 downto 0);
        hgy_i   : in  std_logic_vector(31 downto 0);
        hgz_i   : in  std_logic_vector(31 downto 0);

        -- Sec6: gyroscope Q9.7 signed 16-bit
        gx_i    : in  signed(15 downto 0);
        gy_i    : in  signed(15 downto 0);
        gz_i    : in  signed(15 downto 0);

        -- Sec7: ramped gain Q4.16 unsigned 20-bit
        gain_i  : in  unsigned(19 downto 0);

        -- Sec8: current quaternion Q2.16 signed 18-bit
        qw_i    : in  signed(17 downto 0);
        qx_i    : in  signed(17 downto 0);
        qy_i    : in  signed(17 downto 0);
        qz_i    : in  signed(17 downto 0);

        -- Sec8: delta time Q0.20 unsigned 20-bit
        dt_i    : in  unsigned(19 downto 0);

        -- Output: unnormalized quaternion Q2.16 signed 18-bit
        ready   : out std_logic;
        qw_o    : out signed(17 downto 0);
        qx_o    : out signed(17 downto 0);
        qy_o    : out signed(17 downto 0);
        qz_o    : out signed(17 downto 0);
        
        -- DEBUG: izlaz hfb iz sec5 za dijagnostiku
        hfbx_dbg : out signed(25 downto 0);
        hfby_dbg : out signed(25 downto 0);
        hfbz_dbg : out signed(25 downto 0);

        -- DEBUG: interni signali sec678 za lokalizaciju buga
        dbg_s1_hgx_o   : out signed(21 downto 0);
        dbg_s2_adjgx_o : out signed(25 downto 0);
        dbg_s3b_dqx_o  : out signed(26 downto 0);
        dbg_s4_shx_o   : out signed(31 downto 0)
    );
end entity ahrs_sec5to8;

architecture Structural of ahrs_sec5to8 is

    component ahrs_sec5 is
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
    end component;

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
            qz_o    : out signed(17 downto 0);
            dbg_s1_hgx_o   : out signed(21 downto 0);
            dbg_s2_adjgx_o : out signed(25 downto 0);
            dbg_s3b_dqx_o  : out signed(26 downto 0);
            dbg_s4_shx_o   : out signed(31 downto 0)
        );
    end component;

    signal hfbx_s       : signed(25 downto 0);
    signal hfby_s       : signed(25 downto 0);
    signal hfbz_s       : signed(25 downto 0);
    signal sec5_done    : std_logic;
    signal sec678_start : std_logic;
    signal start_pending: std_logic := '0';

    -- ============================================================
    -- ILA DEBUG: mark hfb signals for chipscope debugging
    -- ============================================================
    attribute mark_debug : string;
    attribute keep       : string;
    
    attribute mark_debug of hfbx_s       : signal is "true";
    attribute keep       of hfbx_s       : signal is "true";
    
    attribute mark_debug of hfby_s       : signal is "true";
    attribute keep       of hfby_s       : signal is "true";
    
    attribute mark_debug of hfbz_s       : signal is "true";
    attribute keep       of hfbz_s       : signal is "true";
    
    attribute mark_debug of sec5_done    : signal is "true";
    attribute keep       of sec5_done    : signal is "true";
    
    attribute mark_debug of sec678_start : signal is "true";
    attribute keep       of sec678_start : signal is "true";
    
    attribute mark_debug of start_pending: signal is "true";
    attribute keep       of start_pending: signal is "true";


begin

    -- Registar koji cuva start signal dok sec5 ne zavrsi
    -- U simulaciji: sec5_done='1' odmah -> start_pending='1' samo jedan ciklus -> sec678_start puls
    -- U sintezi:    start_pending='1' ~110 ciklusa, sec5_done pulsuje -> sec678_start puls
    process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                start_pending <= '0';
            elsif start = '1' then
                start_pending <= '1';
            elsif sec5_done = '1' then
                start_pending <= '0';
            end if;
        end if;
    end process;

    sec678_start <= sec5_done and start_pending;

    sec5 : ahrs_sec5
        port map (
            clk_i   => clk,
            rst_i   => reset,
            start_i => start,
            ax_i    => ax_i,   ay_i   => ay_i,   az_i   => az_i,
            hgx_i   => hgx_i,  hgy_i  => hgy_i,  hgz_i  => hgz_i,
            hfbx_o  => hfbx_s, hfby_o => hfby_s, hfbz_o => hfbz_s,
            done_o  => sec5_done
        );

    sec678 : ahrs_sec678
        port map (
            clk    => clk,    reset  => reset,
            start  => sec678_start,
            gx_i   => gx_i,   gy_i   => gy_i,   gz_i   => gz_i,
            hfbx_i => hfbx_s, hfby_i => hfby_s, hfbz_i => hfbz_s,
            gain_i => gain_i,
            qw_i   => qw_i,   qx_i   => qx_i,
            qy_i   => qy_i,   qz_i   => qz_i,
            dt_i   => dt_i,   ready  => ready,
            qw_o   => qw_o,   qx_o   => qx_o,
            qy_o   => qy_o,   qz_o   => qz_o,
            dbg_s1_hgx_o   => dbg_s1_hgx_o,
            dbg_s2_adjgx_o => dbg_s2_adjgx_o,
            dbg_s3b_dqx_o  => dbg_s3b_dqx_o,
            dbg_s4_shx_o   => dbg_s4_shx_o
        );
        -- DEBUG: izlaz internih hfb signala
    hfbx_dbg <= hfbx_s;
    hfby_dbg <= hfby_s;
    hfbz_dbg <= hfbz_s;

end architecture Structural;
