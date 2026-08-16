library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;

library work;
use work.cloop_pkg.all;

entity clarke_inverse is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- Input currents for the Clarke transformation
        -- Valid range [-5A, 5A], Q3.12
        i_alpha_beta    : in t_ab_phase;

        -- Output from the Clarke transformation
        -- Valid range [-5A, 5A], Q3.12
        o_inv_clarke    : out t_abc_phase
    );
end entity clarke_inverse;

architecture rtl of clarke_inverse is
    constant C_CLARKE_INV_MTRX_00   : sfixed(1 downto -14)  := to_sfixed(C_CLARKE_INV_MX_00, 1, -14);
    constant C_CLARKE_INV_MTRX_01   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_INV_MX_01, 0, -15);
    constant C_CLARKE_INV_MTRX_10   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_INV_MX_10, 0, -15);
    constant C_CLARKE_INV_MTRX_11   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_INV_MX_11, 0, -15);
    constant C_CLARKE_INV_MTRX_20   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_INV_MX_20, 0, -15);
    constant C_CLARKE_INV_MTRX_21   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_INV_MX_21, 0, -15);
    
    signal r_input_alpha_beta       : t_ab_phase;
begin
    sample_input : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_input_alpha_beta.alpha    <= to_sfixed(0.0, r_input_alpha_beta.alpha);
                r_input_alpha_beta.beta     <= to_sfixed(0.0, r_input_alpha_beta.beta);
                r_input_alpha_beta.vld      <= '0';
            else
                if i_alpha_beta.vld = '1' then 
                    r_input_alpha_beta      <= i_alpha_beta;
                else 
                    r_input_alpha_beta.vld  <= '0';
                end if;
            end if;
        end if;
    end process sample_input;

    calc_clarke : process(i_clk)
        variable v_mtrx_00          : sfixed(4 downto -27);
        variable v_mtrx_01          : sfixed(4 downto -27);
        variable v_mtrx_10          : sfixed(4 downto -27);
        variable v_mtrx_11          : sfixed(4 downto -27);
        variable v_mtrx_20          : sfixed(4 downto -27);
        variable v_mtrx_21          : sfixed(4 downto -27);

        variable v_ph_raw_a         : sfixed(5 downto -27);
        variable v_ph_raw_b         : sfixed(5 downto -27);
        variable v_ph_raw_c         : sfixed(5 downto -27);

        variable v_ph_a             : sfixed(3 downto -12);
        variable v_ph_b             : sfixed(3 downto -12);
        variable v_ph_c             : sfixed(3 downto -12);
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_inv_clarke.A   <= to_sfixed(0.0, o_inv_clarke.A);
                o_inv_clarke.B   <= to_sfixed(0.0, o_inv_clarke.B);
                o_inv_clarke.C   <= to_sfixed(0.0, o_inv_clarke.C);
                o_inv_clarke.vld <= '0';
            elsif r_input_alpha_beta.vld = '1' then
                v_mtrx_00   := resize(C_CLARKE_INV_MTRX_00 * r_input_alpha_beta.alpha, v_mtrx_00);
                v_mtrx_01   := resize(C_CLARKE_INV_MTRX_01 * r_input_alpha_beta.beta,  v_mtrx_01);
                v_mtrx_10   := resize(C_CLARKE_INV_MTRX_10 * r_input_alpha_beta.alpha, v_mtrx_10);
                v_mtrx_11   := resize(C_CLARKE_INV_MTRX_11 * r_input_alpha_beta.beta,  v_mtrx_11);
                v_mtrx_20   := resize(C_CLARKE_INV_MTRX_20 * r_input_alpha_beta.alpha, v_mtrx_20);
                v_mtrx_21   := resize(C_CLARKE_INV_MTRX_21 * r_input_alpha_beta.beta,  v_mtrx_21);

                v_ph_raw_a  := resize(v_mtrx_00 + v_mtrx_01, v_ph_raw_a);
                v_ph_raw_b  := resize(v_mtrx_10 + v_mtrx_11, v_ph_raw_b);
                v_ph_raw_c  := resize(v_mtrx_20 + v_mtrx_21, v_ph_raw_c);

                if v_ph_raw_a > 5.0 then 
                    v_ph_a  := to_sfixed(5.0, v_ph_a);
                elsif v_ph_raw_a < -5.0 then 
                    v_ph_a  := to_sfixed(-5.0, v_ph_a);
                else
                    v_ph_a  := resize(v_ph_raw_a, v_ph_a);
                end if;

                if v_ph_raw_b > 5.0 then 
                    v_ph_b  := to_sfixed(5.0, v_ph_b);
                elsif v_ph_raw_b < -5.0 then 
                    v_ph_b  := to_sfixed(-5.0, v_ph_b);
                else
                    v_ph_b  := resize(v_ph_raw_b, v_ph_b);
                end if;

                if v_ph_raw_c > 5.0 then 
                    v_ph_c  := to_sfixed(5.0, v_ph_c);
                elsif v_ph_raw_c < -5.0 then 
                    v_ph_c  := to_sfixed(-5.0, v_ph_c);
                else
                    v_ph_c  := resize(v_ph_raw_c, v_ph_c);
                end if;

                o_inv_clarke.A <= v_ph_a; 
                o_inv_clarke.B <= v_ph_b; 
                o_inv_clarke.C <= v_ph_c; 
                o_inv_clarke.vld <= '1';
            else
                o_inv_clarke.vld <= '0';
            end if;
        end if;
    end process calc_clarke;
end architecture rtl;