-- =============================================================================
-- ahrs_ip_v3_0.vhd
-- Top-level AHRS IP wrapper
--
-- Interfejsi:
--   1. AXI Lite Slave  (S00_AXI) - CPU pise START, cita STATUS
--   2. Native BRAM port B        - IP direktno cita ulaze i pise rezultate
--   3. irq_o                     - interrupt prema Zynq PS kada je DONE
--
-- BRAM word offsets (iz dokumentacije VP, Tabela 4):
--   Ulazi  (15 rijeci): BRAM_INPUT_OFFSET  = 0x5C = 92  (word index)
--   Izlazi  (4 rijeci): BRAM_OUTPUT_OFFSET = 0xCA = 202 (word index)
--
-- BRAM adresa = word_index, jer native interfejs koristi word adrese.
-- BRAM sirina adrese: 10 bita pokriva 512 rijeci (2KB BRAM).
--
-- Redosljed rijeci u INPUT regionu (odgovara cpu.cpp):
--   0:gx  1:gy  2:gz  3:ax  4:ay  5:az
--   6:halfGravityX  7:halfGravityY  8:halfGravityZ
--   9:deltaTime  10:rampedGain
--   11:qw  12:qx  13:qy  14:qz
--
-- Redosljed rijeci u OUTPUT regionu:
--   0:qw_out  1:qx_out  2:qy_out  3:qz_out
--
-- Konverzija float->fixed:
--   gx/gy/gz (Q9.7 s16):  donji 16 bita IEEE 754 float reprezentacije
--   gain (Q4.16 u20):     donji 20 bita
--   dt   (Q0.20 u20):     donji 20 bita
--   qw/qx/qy/qz (Q2.16 s18): donji 18 bita
--   ax/ay/az, hgx/hgy/hgz: direktno kao std_logic_vector(31 downto 0)
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ahrs_ip_v3_0 is
    generic (
        C_S00_AXI_DATA_WIDTH : integer := 32;
        C_S00_AXI_ADDR_WIDTH : integer := 4;
        -- Word offsets u BRAM-u
        BRAM_INPUT_OFFSET    : integer := 92;   -- 0x5C
        BRAM_OUTPUT_OFFSET   : integer := 202  -- 0xCA
    );
    port (
        -- AXI Lite Slave
        s00_axi_aclk    : in  std_logic;
        s00_axi_aresetn : in  std_logic;
        s00_axi_awaddr  : in  std_logic_vector(C_S00_AXI_ADDR_WIDTH-1 downto 0);
        s00_axi_awprot  : in  std_logic_vector(2 downto 0);
        s00_axi_awvalid : in  std_logic;
        s00_axi_awready : out std_logic;
        s00_axi_wdata   : in  std_logic_vector(C_S00_AXI_DATA_WIDTH-1 downto 0);
        s00_axi_wstrb   : in  std_logic_vector((C_S00_AXI_DATA_WIDTH/8)-1 downto 0);
        s00_axi_wvalid  : in  std_logic;
        s00_axi_wready  : out std_logic;
        s00_axi_bresp   : out std_logic_vector(1 downto 0);
        s00_axi_bvalid  : out std_logic;
        s00_axi_bready  : in  std_logic;
        s00_axi_araddr  : in  std_logic_vector(C_S00_AXI_ADDR_WIDTH-1 downto 0);
        s00_axi_arprot  : in  std_logic_vector(2 downto 0);
        s00_axi_arvalid : in  std_logic;
        s00_axi_arready : out std_logic;
        s00_axi_rdata   : out std_logic_vector(C_S00_AXI_DATA_WIDTH-1 downto 0);
        s00_axi_rresp   : out std_logic_vector(1 downto 0);
        s00_axi_rvalid  : out std_logic;
        s00_axi_rready  : in  std_logic;

        -- Native BRAM port B (IP je master ovog porta)
        bram_clk  : out std_logic;
        bram_en   : out std_logic;
        bram_we   : out std_logic_vector(3 downto 0);
        bram_addr : out std_logic_vector(31 downto 0);
        bram_din  : out std_logic_vector(31 downto 0);
        bram_dout : in  std_logic_vector(31 downto 0);

        -- Interrupt prema Zynq PS
        irq_o : out std_logic
    );
end entity ahrs_ip_v3_0;

architecture arch_imp of ahrs_ip_v3_0 is

    -- -------------------------------------------------------------------------
    -- Komponente
    -- -------------------------------------------------------------------------
    component ahrs_ip_v3_0_S00_AXI is
        generic (
            C_S_AXI_DATA_WIDTH : integer := 32;
            C_S_AXI_ADDR_WIDTH : integer := 4
        );
        port (
            S_AXI_ACLK    : in  std_logic;
            S_AXI_ARESETN : in  std_logic;
            S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
            S_AXI_AWPROT  : in  std_logic_vector(2 downto 0);
            S_AXI_AWVALID : in  std_logic;
            S_AXI_AWREADY : out std_logic;
            S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
            S_AXI_WSTRB   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
            S_AXI_WVALID  : in  std_logic;
            S_AXI_WREADY  : out std_logic;
            S_AXI_BRESP   : out std_logic_vector(1 downto 0);
            S_AXI_BVALID  : out std_logic;
            S_AXI_BREADY  : in  std_logic;
            S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
            S_AXI_ARPROT  : in  std_logic_vector(2 downto 0);
            S_AXI_ARVALID : in  std_logic;
            S_AXI_ARREADY : out std_logic;
            S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
            S_AXI_RRESP   : out std_logic_vector(1 downto 0);
            S_AXI_RVALID  : out std_logic;
            S_AXI_RREADY  : in  std_logic;
            start_o  : out std_logic;
            busy_i   : in  std_logic;
            done_i   : in  std_logic
        );
    end component;

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

    -- -------------------------------------------------------------------------
    -- Interni signali
    -- -------------------------------------------------------------------------
    signal clk    : std_logic;
    signal resetn : std_logic;
    signal reset  : std_logic;

    -- AXI Lite kontrolni signali
    signal start_s : std_logic;
    signal busy_s  : std_logic;
    signal done_s  : std_logic;

    -- AHRS input registri (ucitani iz BRAM-a)
    signal reg_gx   : std_logic_vector(31 downto 0);
    signal reg_gy   : std_logic_vector(31 downto 0);
    signal reg_gz   : std_logic_vector(31 downto 0);
    signal reg_ax   : std_logic_vector(31 downto 0);
    signal reg_ay   : std_logic_vector(31 downto 0);
    signal reg_az   : std_logic_vector(31 downto 0);
    signal reg_hgx  : std_logic_vector(31 downto 0);
    signal reg_hgy  : std_logic_vector(31 downto 0);
    signal reg_hgz  : std_logic_vector(31 downto 0);
    signal reg_dt   : std_logic_vector(31 downto 0);
    signal reg_gain : std_logic_vector(31 downto 0);
    signal reg_qw   : std_logic_vector(31 downto 0);
    signal reg_qx   : std_logic_vector(31 downto 0);
    signal reg_qy   : std_logic_vector(31 downto 0);
    signal reg_qz   : std_logic_vector(31 downto 0);

    -- Fixed-point konverzija (slice donjih bita)
    signal gx_fixed   : signed(15 downto 0);
    signal gy_fixed   : signed(15 downto 0);
    signal gz_fixed   : signed(15 downto 0);
    signal gain_fixed : unsigned(19 downto 0);
    signal dt_fixed   : unsigned(19 downto 0);
    signal qw_fixed   : signed(17 downto 0);
    signal qx_fixed   : signed(17 downto 0);
    signal qy_fixed   : signed(17 downto 0);
    signal qz_fixed   : signed(17 downto 0);

    -- AHRS izlazi
    signal ahrs_ready : std_logic;
    signal ahrs_start : std_logic;
    signal ahrs_reset : std_logic;
    signal ahrs_qw_o  : signed(17 downto 0);
    signal ahrs_qx_o  : signed(17 downto 0);
    signal ahrs_qy_o  : signed(17 downto 0);
    signal ahrs_qz_o  : signed(17 downto 0);
    signal ahrs_hfbx  : signed(25 downto 0);
    signal ahrs_hfby  : signed(25 downto 0);
    signal ahrs_hfbz  : signed(25 downto 0);

    -- DEBUG signali za lokalizaciju buga
    signal ahrs_dbg_s1_hgx   : signed(21 downto 0);
    signal ahrs_dbg_s2_adjgx : signed(25 downto 0);
    signal ahrs_dbg_s3b_dqx  : signed(26 downto 0);
    signal ahrs_dbg_s4_shx   : signed(31 downto 0);

    -- BRAM interni signali
    signal bram_en_r   : std_logic;
    signal bram_we_r   : std_logic_vector(3 downto 0);
    signal bram_addr_r : std_logic_vector(32-1 downto 0);
    signal bram_din_r  : std_logic_vector(31 downto 0);

    -- State masina
    type state_t is (
        S_IDLE,
        S_READ_ISSUE,   -- postavi adresu na BRAM
        S_READ_WAIT,    -- cekaj jedan ciklus na dout
        S_READ_LATCH,   -- uhvati podatak, inkrementuj brojac
        S_AHRS_START,   -- puls start prema ahrs_sec5to8
        S_AHRS_WAIT,    -- cekaj ready od ahrs_sec5to8
        S_WRITE_ISSUE,  -- upisi jednu rijec rezultata u BRAM
        S_WRITE_WAIT,   -- cekaj jedan ciklus (WE puls)
        S_DONE          -- postavi DONE, generiši interrupt
    );
    signal state : state_t;

    -- Brojaci za petlje citanja/pisanja
    signal rd_cnt : integer range 0 to 14;  -- 0..14 = 15 rijeci
    signal wr_cnt : integer range 0 to 10;  -- 0..10 = 11 rijeci (7 originalnih + 4 debug)

begin

    clk    <= s00_axi_aclk;
    resetn <= s00_axi_aresetn;
    reset  <= not resetn;

    -- BRAM port B izlazi
    bram_clk  <= clk;
    bram_en   <= bram_en_r;
    bram_we   <= bram_we_r;
    bram_addr <= bram_addr_r;
    bram_din  <= bram_din_r;

    -- Interrupt - aktivan dok je DONE=1
    irq_o <= done_s;

    -- Fixed-point slice
    gx_fixed   <= signed(reg_gx(15 downto 0));
    gy_fixed   <= signed(reg_gy(15 downto 0));
    gz_fixed   <= signed(reg_gz(15 downto 0));
    gain_fixed <= unsigned(reg_gain(19 downto 0));
    dt_fixed   <= unsigned(reg_dt(19 downto 0));
    qw_fixed   <= signed(reg_qw(17 downto 0));
    qx_fixed   <= signed(reg_qx(17 downto 0));
    qy_fixed   <= signed(reg_qy(17 downto 0));
    qz_fixed   <= signed(reg_qz(17 downto 0));

    -- -------------------------------------------------------------------------
    -- AXI Lite Slave instanca
    -- -------------------------------------------------------------------------
    u_s00_axi : ahrs_ip_v3_0_S00_AXI
        generic map (
            C_S_AXI_DATA_WIDTH => C_S00_AXI_DATA_WIDTH,
            C_S_AXI_ADDR_WIDTH => C_S00_AXI_ADDR_WIDTH
        )
        port map (
            S_AXI_ACLK    => s00_axi_aclk,
            S_AXI_ARESETN => s00_axi_aresetn,
            S_AXI_AWADDR  => s00_axi_awaddr,
            S_AXI_AWPROT  => s00_axi_awprot,
            S_AXI_AWVALID => s00_axi_awvalid,
            S_AXI_AWREADY => s00_axi_awready,
            S_AXI_WDATA   => s00_axi_wdata,
            S_AXI_WSTRB   => s00_axi_wstrb,
            S_AXI_WVALID  => s00_axi_wvalid,
            S_AXI_WREADY  => s00_axi_wready,
            S_AXI_BRESP   => s00_axi_bresp,
            S_AXI_BVALID  => s00_axi_bvalid,
            S_AXI_BREADY  => s00_axi_bready,
            S_AXI_ARADDR  => s00_axi_araddr,
            S_AXI_ARPROT  => s00_axi_arprot,
            S_AXI_ARVALID => s00_axi_arvalid,
            S_AXI_ARREADY => s00_axi_arready,
            S_AXI_RDATA   => s00_axi_rdata,
            S_AXI_RRESP   => s00_axi_rresp,
            S_AXI_RVALID  => s00_axi_rvalid,
            S_AXI_RREADY  => s00_axi_rready,
            start_o       => start_s,
            busy_i        => busy_s,
            done_i        => done_s
        );

    -- -------------------------------------------------------------------------
    -- AHRS jezgro
    -- -------------------------------------------------------------------------
    u_ahrs : ahrs_sec5to8
        port map (
            clk    => clk,
            reset  => ahrs_reset,
            start  => ahrs_start,
            ax_i   => reg_ax,
            ay_i   => reg_ay,
            az_i   => reg_az,
            hgx_i  => reg_hgx,
            hgy_i  => reg_hgy,
            hgz_i  => reg_hgz,
            gx_i   => gx_fixed,
            gy_i   => gy_fixed,
            gz_i   => gz_fixed,
            gain_i => gain_fixed,
            qw_i   => qw_fixed,
            qx_i   => qx_fixed,
            qy_i   => qy_fixed,
            qz_i   => qz_fixed,
            dt_i   => dt_fixed,
            ready  => ahrs_ready,
            qw_o   => ahrs_qw_o,
            qx_o   => ahrs_qx_o,
            qy_o   => ahrs_qy_o,
            qz_o   => ahrs_qz_o,
            hfbx_dbg => ahrs_hfbx,
            hfby_dbg => ahrs_hfby,
            hfbz_dbg => ahrs_hfbz,
            dbg_s1_hgx_o   => ahrs_dbg_s1_hgx,
            dbg_s2_adjgx_o => ahrs_dbg_s2_adjgx,
            dbg_s3b_dqx_o  => ahrs_dbg_s3b_dqx,
            dbg_s4_shx_o   => ahrs_dbg_s4_shx
        );

    -- -------------------------------------------------------------------------
    -- Glavna state masina
    -- -------------------------------------------------------------------------
    process(clk)
        variable word_data : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            if resetn = '0' then
                state      <= S_IDLE;
                busy_s     <= '0';
                done_s     <= '0';
                ahrs_start <= '0';
                ahrs_reset <= '1';
                bram_en_r  <= '0';
                bram_we_r  <= (others => '0');
                bram_addr_r<= (others => '0');
                bram_din_r <= (others => '0');
                rd_cnt     <= 0;
                wr_cnt     <= 0;
            else
                -- Default
                ahrs_start <= '0';
                ahrs_reset <= '0';
                bram_en_r  <= '0';
                bram_we_r  <= (others => '0');

                case state is

                    -- ---------------------------------------------------------
                    when S_IDLE =>
                        -- done_s ostaje '1' dok CPU ne posalje novi START
                        if start_s = '1' then
                            busy_s  <= '1';
                            done_s  <= '0';    -- ocisti done samo na novom STARTu
                            rd_cnt  <= 0;
                            state   <= S_READ_ISSUE;
                    end if;

                    -- ---------------------------------------------------------
                    -- Citanje 15 rijeci iz BRAM-a, jedna po jedna
                    -- BRAM latencija = 1 ciklus (registered output)
                    -- S_READ_ISSUE: postavi adresu i en
                    -- S_READ_WAIT:  sacekaj 1 ciklus
                    -- S_READ_LATCH: uzmi dout, sacuvaj u registar
                    -- ---------------------------------------------------------
                    when S_READ_ISSUE =>
                        bram_en_r   <= '1';
                        bram_we_r   <= (others => '0');
                        bram_addr_r <= std_logic_vector(
                            to_unsigned((BRAM_INPUT_OFFSET + rd_cnt) * 4, 32));
                        state <= S_READ_WAIT;

                    when S_READ_WAIT =>
                        -- adresa je stabilna, cekaj jedan ciklus na izlaz
                        state <= S_READ_LATCH;

                    when S_READ_LATCH =>
                        word_data := bram_dout;
                        case rd_cnt is
                            when 0  => reg_gx   <= word_data;
                            when 1  => reg_gy   <= word_data;
                            when 2  => reg_gz   <= word_data;
                            when 3  => reg_ax   <= word_data;
                            when 4  => reg_ay   <= word_data;
                            when 5  => reg_az   <= word_data;
                            when 6  => reg_hgx  <= word_data;
                            when 7  => reg_hgy  <= word_data;
                            when 8  => reg_hgz  <= word_data;
                            when 9  => reg_dt   <= word_data;
                            when 10 => reg_gain <= word_data;
                            when 11 => reg_qw   <= word_data;
                            when 12 => reg_qx   <= word_data;
                            when 13 => reg_qy   <= word_data;
                            when 14 => reg_qz   <= word_data;
                            when others => null;
                        end case;

                        if rd_cnt = 14 then
                            -- sve rijeci procitane, startuj AHRS
                            state <= S_AHRS_START;
                        else
                            rd_cnt <= rd_cnt + 1;
                            state  <= S_READ_ISSUE;
                        end if;

                    -- ---------------------------------------------------------
                    when S_AHRS_START =>
                        ahrs_start <= '1';
                        state      <= S_AHRS_WAIT;

                    when S_AHRS_WAIT =>
                        if ahrs_ready = '1' then
                            wr_cnt <= 0;
                            state  <= S_WRITE_ISSUE;
                        end if;

                    -- ---------------------------------------------------------
                    -- Pisanje 4 rijeci rezultata u BRAM
                    -- S_WRITE_ISSUE: postavi adresu, din, we
                    -- S_WRITE_WAIT:  jedan ciklus WE puls, pa sledeca rijec ili DONE
                    -- ---------------------------------------------------------
                    when S_WRITE_ISSUE =>
                        bram_en_r   <= '1';
                        bram_we_r   <= "1111";
                        bram_addr_r <= std_logic_vector(
                            to_unsigned((BRAM_OUTPUT_OFFSET + wr_cnt) *4, 32));
                        case wr_cnt is
                            when 0 => bram_din_r <=
                                (31 downto 18 => ahrs_qw_o(17)) & std_logic_vector(ahrs_qw_o);
                            when 1 => bram_din_r <=
                                (31 downto 18 => ahrs_qx_o(17)) & std_logic_vector(ahrs_qx_o);
                            when 2 => bram_din_r <=
                                (31 downto 18 => ahrs_qy_o(17)) & std_logic_vector(ahrs_qy_o);
                            when 3 => bram_din_r <=
                                (31 downto 18 => ahrs_qz_o(17)) & std_logic_vector(ahrs_qz_o);
                            when 4 => bram_din_r <=
                                (31 downto 26 => ahrs_hfbx(25)) & std_logic_vector(ahrs_hfbx);
                            when 5 => bram_din_r <=
                                (31 downto 26 => ahrs_hfby(25)) & std_logic_vector(ahrs_hfby);
                            when 6 => bram_din_r <=
                                (31 downto 26 => ahrs_hfbz(25)) & std_logic_vector(ahrs_hfbz);
                            when 7 => bram_din_r <=
                                (31 downto 22 => ahrs_dbg_s1_hgx(21)) & std_logic_vector(ahrs_dbg_s1_hgx);
                            when 8 => bram_din_r <=
                                (31 downto 26 => ahrs_dbg_s2_adjgx(25)) & std_logic_vector(ahrs_dbg_s2_adjgx);
                            when 9 => bram_din_r <=
                                (31 downto 27 => ahrs_dbg_s3b_dqx(26)) & std_logic_vector(ahrs_dbg_s3b_dqx);
                            when 10 => bram_din_r <=
                                std_logic_vector(ahrs_dbg_s4_shx);
                            when others => null;
                        end case;
                        state <= S_WRITE_WAIT;

                    when S_WRITE_WAIT =>
                        -- WE puls gotov
                        bram_we_r <= (others => '0');
                        if wr_cnt = 10 then
                            state <= S_DONE;
                        else
                            wr_cnt <= wr_cnt + 1;
                            state  <= S_WRITE_ISSUE;
                        end if;

                    -- ---------------------------------------------------------
                    when S_DONE =>
                        busy_s <= '0';
                        done_s <= '1';
                        -- done_s ce se ocistiti na sledeci START (u S_IDLE)
                        state  <= S_IDLE;

                    when others =>
                        state <= S_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture arch_imp;
