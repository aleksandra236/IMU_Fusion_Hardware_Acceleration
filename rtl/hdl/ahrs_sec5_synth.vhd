-- =============================================================================
-- ahrs_sec5_synth.vhd
-- AHRS Section 5: Accelerometer Feedback -- SYNTHESIS VERSION (PARALELIZOVANA)
--
-- Koristi 3x Xilinx Floating Point v7.1 IP core-a za mnozenje (latency 3):
--   fp_mul  : multiply 1
--   fp_mul2 : multiply 2
--   fp_mul3 : multiply 3
-- I jedan add (floating_point_0) i jedan sub (fp_sub).
--
-- Paralelizacija:
--   ax*ax, ay*ay, az*az -> istovremeno (3x fp_mul)
--   cx*cx, cy*cy, cz*cz -> istovremeno (3x fp_mul)
--   cx*inv, cy*inv, cz*inv -> istovremeno (3x fp_mul)
--   cx*sc, cy*sc, cz*sc -> istovremeno (3x fp_mul)
--
-- Latencija: ~100 ciklusa (vs ~180 sekvencijalno)
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

    constant C_FP_LAT   : integer := 4;
    constant C_MAGIC    : integer := 16#5F1F1412#;
    constant C_F32_ZERO : std_logic_vector(31 downto 0) := X"00000000";
    constant C_NR_A     : std_logic_vector(31 downto 0) := X"3FD851EC"; -- 1.69000231
    constant C_NR_B     : std_logic_vector(31 downto 0) := X"3F36D24D"; -- 0.714158168
    constant C_SCALE    : std_logic_vector(31 downto 0) := X"4B800000"; -- 2^24
    signal r_hfbx : signed(25 downto 0) := (others => '0');
    signal r_hfby : signed(25 downto 0) := (others => '0');
    signal r_hfbz : signed(25 downto 0) := (others => '0');
    signal r_cycle_cnt : unsigned(25 downto 0) := (others => '0');
    signal r_running   : std_logic := '0';

    component fp_mul
        port (
            aclk                 : in  std_logic;
            s_axis_a_tvalid      : in  std_logic;
            s_axis_a_tdata       : in  std_logic_vector(31 downto 0);
            s_axis_b_tvalid      : in  std_logic;
            s_axis_b_tdata       : in  std_logic_vector(31 downto 0);
            m_axis_result_tvalid : out std_logic;
            m_axis_result_tdata  : out std_logic_vector(31 downto 0)
        );
    end component;

    component fp_mul2
        port (
            aclk                 : in  std_logic;
            s_axis_a_tvalid      : in  std_logic;
            s_axis_a_tdata       : in  std_logic_vector(31 downto 0);
            s_axis_b_tvalid      : in  std_logic;
            s_axis_b_tdata       : in  std_logic_vector(31 downto 0);
            m_axis_result_tvalid : out std_logic;
            m_axis_result_tdata  : out std_logic_vector(31 downto 0)
        );
    end component;

    component fp_mul3
        port (
            aclk                 : in  std_logic;
            s_axis_a_tvalid      : in  std_logic;
            s_axis_a_tdata       : in  std_logic_vector(31 downto 0);
            s_axis_b_tvalid      : in  std_logic;
            s_axis_b_tdata       : in  std_logic_vector(31 downto 0);
            m_axis_result_tvalid : out std_logic;
            m_axis_result_tdata  : out std_logic_vector(31 downto 0)
        );
    end component;

    component floating_point_0
        port (
            aclk                 : in  std_logic;
            s_axis_a_tvalid      : in  std_logic;
            s_axis_a_tdata       : in  std_logic_vector(31 downto 0);
            s_axis_b_tvalid      : in  std_logic;
            s_axis_b_tdata       : in  std_logic_vector(31 downto 0);
            m_axis_result_tvalid : out std_logic;
            m_axis_result_tdata  : out std_logic_vector(31 downto 0)
        );
    end component;

    component fp_sub
        port (
            aclk                 : in  std_logic;
            s_axis_a_tvalid      : in  std_logic;
            s_axis_a_tdata       : in  std_logic_vector(31 downto 0);
            s_axis_b_tvalid      : in  std_logic;
            s_axis_b_tdata       : in  std_logic_vector(31 downto 0);
            m_axis_result_tvalid : out std_logic;
            m_axis_result_tdata  : out std_logic_vector(31 downto 0)
        );
    end component;

    -- FP IP input registers
    signal mul1_a_r, mul1_b_r : std_logic_vector(31 downto 0) := (others => '0');
    signal mul2_a_r, mul2_b_r : std_logic_vector(31 downto 0) := (others => '0');
    signal mul3_a_r, mul3_b_r : std_logic_vector(31 downto 0) := (others => '0');
    signal add_a_r,  add_b_r  : std_logic_vector(31 downto 0) := (others => '0');
    signal sub_a_r,  sub_b_r  : std_logic_vector(31 downto 0) := (others => '0');
    signal mul1_tv_r, mul2_tv_r, mul3_tv_r : std_logic := '0';
    signal add_tv_r, sub_tv_r              : std_logic := '0';

    -- FP IP outputs
    signal mul1_res_s, mul2_res_s, mul3_res_s : std_logic_vector(31 downto 0);
    signal add_res_s,  sub_res_s              : std_logic_vector(31 downto 0);
    signal mul1_rv_s,  mul2_rv_s,  mul3_rv_s  : std_logic;
    signal add_rv_s,   sub_rv_s               : std_logic;

    -- State machine
    type state_t is (S_IDLE, S_RUN);
    signal state   : state_t := S_IDLE;
    signal step    : integer range 0 to 39 := 0;
    signal lat_cnt : integer range 0 to C_FP_LAT := 0;

    -- Data registers
    signal r_ax, r_ay, r_az       : std_logic_vector(31 downto 0);
    signal r_hgx, r_hgy, r_hgz   : std_logic_vector(31 downto 0);
    signal r_nax, r_nay, r_naz   : std_logic_vector(31 downto 0);
    signal r_cx, r_cy, r_cz       : std_logic_vector(31 downto 0);
    signal r_dot, r_magSq         : std_logic_vector(31 downto 0);
    signal r_accelInvMag          : std_logic_vector(31 downto 0);
    signal r_crossInvMag          : std_logic_vector(31 downto 0);
    signal r_yr, r_tmp1, r_tmp2   : std_logic_vector(31 downto 0);
    signal r_tmp3                 : std_logic_vector(31 downto 0);

        -- Combinational konverzija f32 -> s26 (3 odvojene instance)
    signal r_hfbx_comb : signed(25 downto 0);
    signal r_hfby_comb : signed(25 downto 0);
    signal r_hfbz_comb : signed(25 downto 0);

begin

    U_MUL : fp_mul
        port map (
            aclk                 => clk_i,
            s_axis_a_tvalid      => mul1_tv_r,
            s_axis_a_tdata       => mul1_a_r,
            s_axis_b_tvalid      => mul1_tv_r,
            s_axis_b_tdata       => mul1_b_r,
            m_axis_result_tvalid => mul1_rv_s,
            m_axis_result_tdata  => mul1_res_s
        );

    U_MUL2 : fp_mul
        port map (
            aclk                 => clk_i,
            s_axis_a_tvalid      => mul2_tv_r,
            s_axis_a_tdata       => mul2_a_r,
            s_axis_b_tvalid      => mul2_tv_r,
            s_axis_b_tdata       => mul2_b_r,
            m_axis_result_tvalid => mul2_rv_s,
            m_axis_result_tdata  => mul2_res_s
        );

    U_MUL3 : fp_mul
        port map (
            aclk                 => clk_i,
            s_axis_a_tvalid      => mul3_tv_r,
            s_axis_a_tdata       => mul3_a_r,
            s_axis_b_tvalid      => mul3_tv_r,
            s_axis_b_tdata       => mul3_b_r,
            m_axis_result_tvalid => mul3_rv_s,
            m_axis_result_tdata  => mul3_res_s
        );

    U_ADD : floating_point_0
        port map (
            aclk                 => clk_i,
            s_axis_a_tvalid      => add_tv_r,
            s_axis_a_tdata       => add_a_r,
            s_axis_b_tvalid      => add_tv_r,
            s_axis_b_tdata       => add_b_r,
            m_axis_result_tvalid => add_rv_s,
            m_axis_result_tdata  => add_res_s
        );

    U_SUB : fp_sub
        port map (
            aclk                 => clk_i,
            s_axis_a_tvalid      => sub_tv_r,
            s_axis_a_tdata       => sub_a_r,
            s_axis_b_tvalid      => sub_tv_r,
            s_axis_b_tdata       => sub_b_r,
            m_axis_result_tvalid => sub_rv_s,
            m_axis_result_tdata  => sub_res_s
        );
        
    -- INSTANCE 1 - konverzija r_tmp1 -> r_hfbx_comb
    process(r_tmp1)
        variable be1  : integer;
        variable mx1  : unsigned(25 downto 0);
        variable sh1  : integer;
        variable res1 : signed(25 downto 0);
    begin
        if unsigned(r_tmp1(30 downto 0)) = 0 then
            r_hfbx_comb <= to_signed(0, 26);
        else
            be1 := to_integer(unsigned(r_tmp1(30 downto 23)));
            mx1 := "00" & unsigned('1' & r_tmp1(22 downto 0));
            sh1 := be1 - 150;
            if sh1 > 2 then sh1 := 2; end if;
            if sh1 >= 0 then
                res1 := signed(shift_left(mx1, sh1));
            elsif sh1 >= -23 then
                res1 := signed(shift_right(mx1, -sh1));
            else
                res1 := to_signed(0, 26);
            end if;
            if r_tmp1(31) = '1' then res1 := -res1; end if;
            r_hfbx_comb <= res1;
        end if;
    end process;

    -- INSTANCE 2 - konverzija r_tmp2 -> r_hfby_comb
    process(r_tmp2)
        variable be2  : integer;
        variable mx2  : unsigned(25 downto 0);
        variable sh2  : integer;
        variable res2 : signed(25 downto 0);
    begin
        if unsigned(r_tmp2(30 downto 0)) = 0 then
            r_hfby_comb <= to_signed(0, 26);
        else
            be2 := to_integer(unsigned(r_tmp2(30 downto 23)));
            mx2 := "00" & unsigned('1' & r_tmp2(22 downto 0));
            sh2 := be2 - 150;
            if sh2 > 2 then sh2 := 2; end if;
            if sh2 >= 0 then
                res2 := signed(shift_left(mx2, sh2));
            elsif sh2 >= -23 then
                res2 := signed(shift_right(mx2, -sh2));
            else
                res2 := to_signed(0, 26);
            end if;
            if r_tmp2(31) = '1' then res2 := -res2; end if;
            r_hfby_comb <= res2;
        end if;
    end process;

    -- INSTANCE 3 - konverzija r_tmp3 -> r_hfbz_comb
    process(r_tmp3)
        variable be3  : integer;
        variable mx3  : unsigned(25 downto 0);
        variable sh3  : integer;
        variable res3 : signed(25 downto 0);
    begin
        if unsigned(r_tmp3(30 downto 0)) = 0 then
            r_hfbz_comb <= to_signed(0, 26);
        else
            be3 := to_integer(unsigned(r_tmp3(30 downto 23)));
            mx3 := "00" & unsigned('1' & r_tmp3(22 downto 0));
            sh3 := be3 - 150;
            if sh3 > 2 then sh3 := 2; end if;
            if sh3 >= 0 then
                res3 := signed(shift_left(mx3, sh3));
            elsif sh3 >= -23 then
                res3 := signed(shift_right(mx3, -sh3));
            else
                res3 := to_signed(0, 26);
            end if;
            if r_tmp3(31) = '1' then res3 := -res3; end if;
            r_hfbz_comb <= res3;
        end if;
    end process;

    process(clk_i)
        variable xi_v : signed(31 downto 0);
        variable yi_v : signed(31 downto 0);
    begin
        if rising_edge(clk_i) then
            mul1_tv_r <= '0'; mul2_tv_r <= '0'; mul3_tv_r <= '0';
            add_tv_r  <= '0'; sub_tv_r  <= '0';
            done_o    <= '0';

            if rst_i = '1' then
                state <= S_IDLE; step <= 0;
                r_cycle_cnt <= (others => '0');
                r_running <= '0';
            else
                 if r_running = '1' then
                    r_cycle_cnt <= r_cycle_cnt + 1;
                end if;
                case state is
                    when S_IDLE =>
                        if start_i = '1' then
                            r_cycle_cnt <= (others => '0');
                            r_running <= '1';
                            r_ax  <= ax_i;  r_ay  <= ay_i;  r_az  <= az_i;
                            r_hgx <= hgx_i; r_hgy <= hgy_i; r_hgz <= hgz_i;
                            state <= S_RUN; step <= 0; lat_cnt <= 0;
                        end if;


                    when S_RUN =>
                        case step is

                            -- STEP 0: ax*ax, ay*ay, az*az PARALELNO
                            when 0 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_ax; mul1_b_r <= r_ax; mul1_tv_r <= '1';
                                    mul2_a_r <= r_ay; mul2_b_r <= r_ay; mul2_tv_r <= '1';
                                    mul3_a_r <= r_az; mul3_b_r <= r_az; mul3_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; -- ax^2
                                    r_tmp2 <= mul2_res_s; -- ay^2
                                    r_tmp3 <= mul3_res_s; -- az^2
                                    step <= 1; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 1: ax^2 + ay^2
                            when 1 =>
                                if lat_cnt = 0 then
                                    add_a_r <= r_tmp1; add_b_r <= r_tmp2; add_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= add_res_s; step <= 2; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 2: (ax^2+ay^2) + az^2 = accelMagSq
                            when 2 =>
                                if lat_cnt = 0 then
                                    add_a_r <= r_tmp1; add_b_r <= r_tmp3; add_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_magSq <= add_res_s; step <= 3; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 3: Quake inv sqrt na accelMagSq
                            when 3 =>
                                if lat_cnt = 0 then
                                    lat_cnt <= 1;
                                else
                                    lat_cnt <= 0;
                                    xi_v := signed(r_magSq);
                                    yi_v := to_signed(C_MAGIC, 32) - shift_right(xi_v, 1);
                                    r_yr <= std_logic_vector(yi_v);
                                    step <= 4;
                                end if;

                            -- STEP 4: yr*yr
                            when 4 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_yr; mul1_b_r <= r_yr; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; step <= 5; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 5: C_NR_B * magSq
                            when 5 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= C_NR_B; mul1_b_r <= r_magSq; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp2 <= mul1_res_s; step <= 6; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 6: (C_NR_B*magSq) * yr^2
                            when 6 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_tmp2; mul1_b_r <= r_tmp1; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; step <= 7; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 7: C_NR_A - tmp1
                            when 7 =>
                                if lat_cnt = 0 then
                                    sub_a_r <= C_NR_A; sub_b_r <= r_tmp1; sub_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= sub_res_s; step <= 8; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 8: yr * tmp1 = accelInvMag
                            when 8 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_yr; mul1_b_r <= r_tmp1; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_accelInvMag <= mul1_res_s; step <= 9; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 9: ax*inv, ay*inv, az*inv PARALELNO -> nax, nay, naz
                            when 9 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_ax; mul1_b_r <= r_accelInvMag; mul1_tv_r <= '1';
                                    mul2_a_r <= r_ay; mul2_b_r <= r_accelInvMag; mul2_tv_r <= '1';
                                    mul3_a_r <= r_az; mul3_b_r <= r_accelInvMag; mul3_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_nax <= mul1_res_s;
                                    r_nay <= mul2_res_s;
                                    r_naz <= mul3_res_s;
                                    step <= 10; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 10: cross product cx = nay*hgz - naz*hgy
                            -- Paralelno: nay*hgz i naz*hgy
                            when 10 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_nay; mul1_b_r <= r_hgz; mul1_tv_r <= '1';
                                    mul2_a_r <= r_naz; mul2_b_r <= r_hgy; mul2_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; -- nay*hgz
                                    r_tmp2 <= mul2_res_s; -- naz*hgy
                                    step <= 11; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 11: cx = nay*hgz - naz*hgy
                            when 11 =>
                                if lat_cnt = 0 then
                                    sub_a_r <= r_tmp1; sub_b_r <= r_tmp2; sub_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_cx <= sub_res_s; step <= 12; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 12: cy = naz*hgx - nax*hgz
                            -- Paralelno: naz*hgx i nax*hgz
                            when 12 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_naz; mul1_b_r <= r_hgx; mul1_tv_r <= '1';
                                    mul2_a_r <= r_nax; mul2_b_r <= r_hgz; mul2_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; -- naz*hgx
                                    r_tmp2 <= mul2_res_s; -- nax*hgz
                                    step <= 13; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 13: cy = naz*hgx - nax*hgz
                            when 13 =>
                                if lat_cnt = 0 then
                                    sub_a_r <= r_tmp1; sub_b_r <= r_tmp2; sub_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_cy <= sub_res_s; step <= 14; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 14: cz = nax*hgy - nay*hgx
                            -- Paralelno: nax*hgy i nay*hgx
                            when 14 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_nax; mul1_b_r <= r_hgy; mul1_tv_r <= '1';
                                    mul2_a_r <= r_nay; mul2_b_r <= r_hgx; mul2_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; -- nax*hgy
                                    r_tmp2 <= mul2_res_s; -- nay*hgx
                                    step <= 15; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 15: cz = nax*hgy - nay*hgx
                            when 15 =>
                                if lat_cnt = 0 then
                                    sub_a_r <= r_tmp1; sub_b_r <= r_tmp2; sub_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_cz <= sub_res_s; step <= 16; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 16: dot = nax*hgx + nay*hgy + naz*hgz
                            -- Paralelno: nax*hgx, nay*hgy, naz*hgz
                            when 16 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_nax; mul1_b_r <= r_hgx; mul1_tv_r <= '1';
                                    mul2_a_r <= r_nay; mul2_b_r <= r_hgy; mul2_tv_r <= '1';
                                    mul3_a_r <= r_naz; mul3_b_r <= r_hgz; mul3_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s;
                                    r_tmp2 <= mul2_res_s;
                                    r_tmp3 <= mul3_res_s;
                                    step <= 17; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 17: nax*hgx + nay*hgy
                            when 17 =>
                                if lat_cnt = 0 then
                                    add_a_r <= r_tmp1; add_b_r <= r_tmp2; add_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= add_res_s; step <= 18; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 18: + naz*hgz = dot
                            when 18 =>
                                if lat_cnt = 0 then
                                    add_a_r <= r_tmp1; add_b_r <= r_tmp3; add_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_dot <= add_res_s; step <= 19; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 19: provjeri dot < 0
                            -- STEP 19: provjeri dot < 0 (sačekaj 1 ciklus da r_dot bude vidljiv)
                            when 19 =>
                                if lat_cnt = 0 then
                                    lat_cnt <= 1;          -- ← 1 cycle delay
                                else
                                    lat_cnt <= 0;
                                    if r_dot(31) = '0' then
                                        step <= 30;        -- dot >= 0, skip normalizaciju
                                    else
                                        step <= 20;
                                    end if;
                                end if;

                            -- STEP 20: cx*cx, cy*cy, cz*cz PARALELNO
                            when 20 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_cx; mul1_b_r <= r_cx; mul1_tv_r <= '1';
                                    mul2_a_r <= r_cy; mul2_b_r <= r_cy; mul2_tv_r <= '1';
                                    mul3_a_r <= r_cz; mul3_b_r <= r_cz; mul3_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s;
                                    r_tmp2 <= mul2_res_s;
                                    r_tmp3 <= mul3_res_s;
                                    step <= 21; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 21: cx^2 + cy^2
                            when 21 =>
                                if lat_cnt = 0 then
                                    add_a_r <= r_tmp1; add_b_r <= r_tmp2; add_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= add_res_s; step <= 22; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 22: + cz^2 = crossMagSq
                            when 22 =>
                                if lat_cnt = 0 then
                                    add_a_r <= r_tmp1; add_b_r <= r_tmp3; add_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_magSq <= add_res_s; step <= 23; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 23: Quake inv sqrt na crossMagSq
                            when 23 =>
                                if lat_cnt = 0 then
                                    lat_cnt <= 1;
                                else
                                    lat_cnt <= 0;
                                    xi_v := signed(r_magSq);
                                    yi_v := to_signed(C_MAGIC, 32) - shift_right(xi_v, 1);
                                    r_yr <= std_logic_vector(yi_v);
                                    step <= 24;
                                end if;

                            -- STEP 24: yr*yr
                            when 24 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_yr; mul1_b_r <= r_yr; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; step <= 25; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 25: C_NR_B * magSq
                            when 25 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= C_NR_B; mul1_b_r <= r_magSq; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp2 <= mul1_res_s; step <= 26; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 26: tmp2 * yr^2
                            when 26 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_tmp2; mul1_b_r <= r_tmp1; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s; step <= 27; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 27: C_NR_A - tmp1
                            when 27 =>
                                if lat_cnt = 0 then
                                    sub_a_r <= C_NR_A; sub_b_r <= r_tmp1; sub_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= sub_res_s; step <= 28; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 28: yr * tmp1 = crossInvMag
                            when 28 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_yr; mul1_b_r <= r_tmp1; mul1_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_crossInvMag <= mul1_res_s; step <= 29; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 29: cx*inv, cy*inv, cz*inv PARALELNO
                            when 29 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_cx; mul1_b_r <= r_crossInvMag; mul1_tv_r <= '1';
                                    mul2_a_r <= r_cy; mul2_b_r <= r_crossInvMag; mul2_tv_r <= '1';
                                    mul3_a_r <= r_cz; mul3_b_r <= r_crossInvMag; mul3_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_cx <= mul1_res_s;
                                    r_cy <= mul2_res_s;
                                    r_cz <= mul3_res_s;
                                    step <= 30; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 30: scale cx*2^24, cy*2^24, cz*2^24 PARALELNO
                            when 30 =>
                                if lat_cnt = 0 then
                                    mul1_a_r <= r_cx; mul1_b_r <= C_SCALE; mul1_tv_r <= '1';
                                    mul2_a_r <= r_cy; mul2_b_r <= C_SCALE; mul2_tv_r <= '1';
                                    mul3_a_r <= r_cz; mul3_b_r <= C_SCALE; mul3_tv_r <= '1';
                                    lat_cnt <= C_FP_LAT;
                                elsif lat_cnt = 1 then
                                    r_tmp1 <= mul1_res_s;
                                    r_tmp2 <= mul2_res_s;
                                    r_tmp3 <= mul3_res_s;
                                    step <= 31; lat_cnt <= 0;
                                else lat_cnt <= lat_cnt - 1; end if;

                            -- STEP 31 - DIAGNOSTIC: vidi šta su sirovi FP rezultati u r_tmp

                            when 31 =>
                                if lat_cnt = 0 then
                                    lat_cnt <= 1;
                                else
                                    lat_cnt <= 0;
                                    r_hfbx <= r_hfbx_comb;
                                    r_hfby <= r_hfby_comb;
                                    r_hfbz <= r_hfbz_comb;
                                    step <= 32;
                                end if;
                                
                                -- STEP 32: izlaz + done
                            -- STEP 32: izlaz + done
                            when 32 =>
                                hfbx_o <= r_hfbx;       -- PRAVI rezultat iz step 31
                                hfby_o <= r_hfby;       -- (bilo: to_signed(32, 26))
                                hfbz_o <= r_hfbz;       -- (bilo: to_signed(1, 26))
                                done_o <= '1';
                                state  <= S_IDLE;
                                r_running <= '0';
                            when others =>
                                state <= S_IDLE;

                        end case;
                end case;
            end if;
        end if;
    end process;

end architecture Behavioral;