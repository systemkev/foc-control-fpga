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

    -- Voltage: Q5.12, coefficient: Q2.20
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

    signal r_alpha : t_voltage_q12 := (others => '0');
    signal r_beta  : t_voltage_q12 := (others => '0');
    signal r_vld   : std_logic := '0';

    function sat_q32_to_q12(
        x : t_sum_q32
    ) return t_voltage_q12 is
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

    sample_input : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_alpha <= (others => '0');
                r_beta  <= (others => '0');
                r_vld   <= '0';
            else
                if i_vld = '1' then
                    r_alpha <= i_alpha;
                    r_beta  <= i_beta;
                    r_vld   <= '1';
                else
                    r_vld <= '0';
                end if;
            end if;
        end if;
    end process;

    calc_inverse_clarke : process(i_clk)
        variable v_alpha_neg_half : t_product_q32;
        variable v_beta_pos       : t_product_q32;
        variable v_beta_neg       : t_product_q32;

        variable v_phase_b_sum    : t_sum_q32;
        variable v_phase_c_sum    : t_sum_q32;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_phases_A <= (others => '0');
                o_phases_B <= (others => '0');
                o_phases_C <= (others => '0');
                o_vld      <= '0';

            elsif r_vld = '1' then
                v_alpha_neg_half := r_alpha * C_NEG_HALF;
                v_beta_pos       := r_beta  * C_POS_SQRT3_HALF;
                v_beta_neg       := r_beta  * C_NEG_SQRT3_HALF;

                v_phase_b_sum :=
                    resize(v_alpha_neg_half, t_sum_q32'length) +
                    resize(v_beta_pos, t_sum_q32'length);

                v_phase_c_sum :=
                    resize(v_alpha_neg_half, t_sum_q32'length) +
                    resize(v_beta_neg, t_sum_q32'length);

                o_phases_A <= r_alpha;
                o_phases_B <= sat_q32_to_q12(v_phase_b_sum);
                o_phases_C <= sat_q32_to_q12(v_phase_c_sum);

                o_vld <= '1';
            else
                o_vld <= '0';
            end if;
        end if;
    end process;

end architecture rtl;