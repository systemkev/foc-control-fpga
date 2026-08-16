library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;

library work;
use work.cloop_pkg.all;

entity clarke is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- Input currents for the Clarke transformation
        -- Valid range [-5A, 5A], Q3.12
        i_phases    : in t_abc_phase;

        -- Output from the Clarke transformation
        -- Valid range [-5A, 5A], Q3.12
        o_clarke    : out t_ab_phase
    );
end entity clarke;

architecture rtl of clarke is
    constant C_CLARKE_MTRX_00   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_MATRIX_00, 0, -15);
    constant C_CLARKE_MTRX_01   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_MATRIX_01, 0, -15);
    constant C_CLARKE_MTRX_02   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_MATRIX_02, 0, -15);
    constant C_CLARKE_MTRX_10   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_MATRIX_10, 0, -15);
    constant C_CLARKE_MTRX_11   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_MATRIX_11, 0, -15);
    constant C_CLARKE_MTRX_12   : sfixed(0 downto -15)  := to_sfixed(C_CLARKE_MATRIX_12, 0, -15);
    
    signal r_input_phases       : t_abc_phase;
    signal r_output_clarke      : t_ab_phase;
begin
    o_clarke <= r_output_clarke;

    sample_input : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_input_phases.A    <= to_sfixed(0.0, r_input_phases.A);
                r_input_phases.B    <= to_sfixed(0.0, r_input_phases.B);
                r_input_phases.C    <= to_sfixed(0.0, r_input_phases.C);
                r_input_phases.vld  <= '0';
            else
                if i_phases.vld = '1' then 
                    r_input_phases      <= i_phases;
                else 
                    r_input_phases.vld  <= '0';
                end if;
            end if;
        end if;
    end process sample_input;

    calc_clarke : process(i_clk)
        variable v_ph_A_mult_row0   : sfixed(4 downto -27);
        variable v_ph_B_mult_row0   : sfixed(4 downto -27);
        variable v_ph_C_mult_row0   : sfixed(4 downto -27);
        variable v_ph_A_mult_row1   : sfixed(4 downto -27);
        variable v_ph_B_mult_row1   : sfixed(4 downto -27);
        variable v_ph_C_mult_row1   : sfixed(4 downto -27);
        variable v_alpha_raw        : sfixed(5 downto -12);
        variable v_beta_raw         : sfixed(5 downto -12);
        variable v_alpha            : sfixed(3 downto -12);
        variable v_beta             : sfixed(3 downto -12);
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_output_clarke.alpha   <= to_sfixed(0.0, r_output_clarke.alpha);
                r_output_clarke.beta    <= to_sfixed(0.0, r_output_clarke.beta);
                r_output_clarke.vld     <= '0';
            elsif r_input_phases.vld = '1' then
                v_ph_A_mult_row0    := r_input_phases.A * C_CLARKE_MTRX_00;
                v_ph_B_mult_row0    := r_input_phases.B * C_CLARKE_MTRX_01;
                v_ph_C_mult_row0    := r_input_phases.C * C_CLARKE_MTRX_02;
                v_ph_A_mult_row1    := r_input_phases.A * C_CLARKE_MTRX_10;
                v_ph_B_mult_row1    := r_input_phases.B * C_CLARKE_MTRX_11;
                v_ph_C_mult_row1    := r_input_phases.C * C_CLARKE_MTRX_12;

                v_alpha_raw := resize(v_ph_A_mult_row0 + v_ph_B_mult_row0 + v_ph_C_mult_row0, v_alpha_raw);
                v_beta_raw  := resize(v_ph_A_mult_row1 + v_ph_B_mult_row1 + v_ph_C_mult_row1, v_beta_raw);

                if v_alpha_raw > 5.0 then
                    v_alpha := to_sfixed(5.0, v_alpha);
                elsif v_alpha_raw < -5.0 then
                    v_alpha := to_sfixed(-5.0, v_alpha);
                else
                    v_alpha := resize(v_alpha_raw, v_alpha);
                end if;

                if v_beta_raw > 5.0 then
                    v_beta  := to_sfixed(5.0, v_beta);
                elsif v_beta_raw < -5.0 then
                    v_beta  := to_sfixed(-5.0, v_beta);
                else
                    v_beta  := resize(v_beta_raw, v_beta);
                end if;

                r_output_clarke.alpha   <= v_alpha;
                r_output_clarke.beta    <= v_beta;
                r_output_clarke.vld     <= '1';
            else
                r_output_clarke.vld     <= '0';
            end if;
        end if;
    end process calc_clarke;
end architecture rtl;