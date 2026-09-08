library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity clarke_inverse is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        i_vld       : in std_logic;
        i_alpha     : in signed(17 downto 0);
        i_beta      : in signed(17 downto 0);

        o_vld       : out std_logic;
        o_phases_A  : out signed(17 downto 0);
        o_phases_B  : out signed(17 downto 0);
        o_phases_C  : out signed(17 downto 0)
    );
end entity clarke_inverse;

architecture rtl of clarke_inverse is

    -- Voltage: Q5.12, coefficient: Q2.20.
    subtype t_voltage_q12 is signed(17 downto 0);
    subtype t_coeff_q20   is signed(22 downto 0);
    subtype t_product_q32 is signed(40 downto 0);
    subtype t_sum_q32     is signed(41 downto 0);

    constant C_NEG_HALF : t_coeff_q20 :=
        to_signed(-524288, t_coeff_q20'length);
    constant C_POS_SQRT3_HALF : t_coeff_q20 :=
        to_signed(908093, t_coeff_q20'length);
    constant C_NEG_SQRT3_HALF : t_coeff_q20 :=
        to_signed(-908093, t_coeff_q20'length);

    -- Stage 0: inputs.
    signal r_alpha : t_voltage_q12 := (others => '0');
    signal r_beta  : t_voltage_q12 := (others => '0');
    signal r_vld_in : std_logic := '0';

    -- Stage 1: products plus alpha pass-through for phase A.
    signal r_alpha_neg_half : t_product_q32 := (others => '0');
    signal r_beta_pos       : t_product_q32 := (others => '0');
    signal r_beta_neg       : t_product_q32 := (others => '0');
    signal r_alpha_prod_d   : t_voltage_q12 := (others => '0');
    signal r_vld_prod       : std_logic := '0';

    -- Stage 2: B/C sums plus aligned alpha.
    signal r_phase_b_sum  : t_sum_q32 := (others => '0');
    signal r_phase_c_sum  : t_sum_q32 := (others => '0');
    signal r_alpha_sum_d  : t_voltage_q12 := (others => '0');
    signal r_vld_sum      : std_logic := '0';

    function sat_q32_to_q12(x : t_sum_q32) return t_voltage_q12 is
        variable v : t_sum_q32;
    begin
        v := shift_right(x, 20);

        if v > to_signed(131071, v'length) then
            return to_signed(131071, t_voltage_q12'length);
        elsif v < to_signed(-131072, v'length) then
            return to_signed(-131072, t_voltage_q12'length);
        else
            return resize(v, t_voltage_q12'length);
        end if;
    end function;

begin

    pipeline : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_alpha          <= (others => '0');
                r_beta           <= (others => '0');
                r_alpha_neg_half <= (others => '0');
                r_beta_pos       <= (others => '0');
                r_beta_neg       <= (others => '0');
                r_alpha_prod_d   <= (others => '0');
                r_phase_b_sum    <= (others => '0');
                r_phase_c_sum    <= (others => '0');
                r_alpha_sum_d    <= (others => '0');
                r_vld_in         <= '0';
                r_vld_prod       <= '0';
                r_vld_sum        <= '0';
                o_phases_A       <= (others => '0');
                o_phases_B       <= (others => '0');
                o_phases_C       <= (others => '0');
                o_vld            <= '0';
            else
                ----------------------------------------------------------------
                -- Stage 0: sample alpha/beta.
                ----------------------------------------------------------------
                r_vld_in <= i_vld;
                if i_vld = '1' then
                    r_alpha <= i_alpha;
                    r_beta  <= i_beta;
                end if;

                ----------------------------------------------------------------
                -- Stage 1: multiplications.  These registers form a clean DSP
                -- boundary before the wide B/C adders.
                ----------------------------------------------------------------
                r_vld_prod <= r_vld_in;
                if r_vld_in = '1' then
                    r_alpha_neg_half <= r_alpha * C_NEG_HALF;
                    r_beta_pos       <= r_beta  * C_POS_SQRT3_HALF;
                    r_beta_neg       <= r_beta  * C_NEG_SQRT3_HALF;
                    r_alpha_prod_d   <= r_alpha;
                end if;

                ----------------------------------------------------------------
                -- Stage 2: inverse-Clarke sums.
                ----------------------------------------------------------------
                r_vld_sum <= r_vld_prod;
                if r_vld_prod = '1' then
                    r_phase_b_sum <= resize(r_alpha_neg_half, r_phase_b_sum'length) +
                                     resize(r_beta_pos,       r_phase_b_sum'length);
                    r_phase_c_sum <= resize(r_alpha_neg_half, r_phase_c_sum'length) +
                                     resize(r_beta_neg,       r_phase_c_sum'length);
                    r_alpha_sum_d <= r_alpha_prod_d;
                end if;

                ----------------------------------------------------------------
                -- Stage 3: saturation/output register.
                ----------------------------------------------------------------
                o_vld <= r_vld_sum;
                if r_vld_sum = '1' then
                    o_phases_A <= r_alpha_sum_d;
                    o_phases_B <= sat_q32_to_q12(r_phase_b_sum);
                    o_phases_C <= sat_q32_to_q12(r_phase_c_sum);
                end if;
            end if;
        end if;
    end process pipeline;

end architecture rtl;
