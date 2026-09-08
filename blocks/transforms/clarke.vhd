library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity clarke is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- Input phase currents, signed Q3.12.
        i_vld       : in std_logic;
        i_phases_A  : in signed(15 downto 0);
        i_phases_B  : in signed(15 downto 0);
        i_phases_C  : in signed(15 downto 0);

        -- Clarke outputs, signed Q3.12, saturated to +/-5.0 A.
        o_vld          : out std_logic;
        o_clarke_alpha : out signed(15 downto 0);
        o_clarke_beta  : out signed(15 downto 0)
    );
end entity clarke;

architecture rtl of clarke is

    subtype t_phase_q12   is signed(15 downto 0);
    subtype t_coeff_q20   is signed(22 downto 0);
    subtype t_product_q32 is signed(38 downto 0);

    -- Clarke coefficients, Q2.20.
    constant C_CLARKE_MTRX_00 : t_coeff_q20 :=
        to_signed(699051, t_coeff_q20'length);   -- +2/3
    constant C_CLARKE_MTRX_01 : t_coeff_q20 :=
        to_signed(-349525, t_coeff_q20'length); -- -1/3
    constant C_CLARKE_MTRX_02 : t_coeff_q20 :=
        to_signed(-349525, t_coeff_q20'length); -- -1/3
    constant C_CLARKE_MTRX_11 : t_coeff_q20 :=
        to_signed(605396, t_coeff_q20'length);  -- +1/sqrt(3)
    constant C_CLARKE_MTRX_12 : t_coeff_q20 :=
        to_signed(-605396, t_coeff_q20'length); -- -1/sqrt(3)

    constant C_OUTPUT_MAX_Q12  : integer :=  20480;
    constant C_OUTPUT_MIN_Q12  : integer := -20480;
    constant C_COEFF_FRAC_BITS : natural := 20;

    -- Stage 0: input registers.
    signal r_phase_a : t_phase_q12 := (others => '0');
    signal r_phase_b : t_phase_q12 := (others => '0');
    signal r_phase_c : t_phase_q12 := (others => '0');
    signal r_vld_in  : std_logic := '0';

    -- Stage 1: multiplier outputs.  Registering every multiplier result lets
    -- Vivado use the DSP output registers and removes multiplier delay from the
    -- following adder network.
    signal r_a_row0 : t_product_q32 := (others => '0');
    signal r_b_row0 : t_product_q32 := (others => '0');
    signal r_c_row0 : t_product_q32 := (others => '0');
    signal r_b_row1 : t_product_q32 := (others => '0');
    signal r_c_row1 : t_product_q32 := (others => '0');
    signal r_vld_prod : std_logic := '0';

    -- Stage 2: first adder level.
    signal r_alpha_ab  : t_product_q32 := (others => '0');
    signal r_alpha_c   : t_product_q32 := (others => '0');
    signal r_beta_sum  : t_product_q32 := (others => '0');
    signal r_vld_add1  : std_logic := '0';

    -- Stage 3: final alpha add.  Beta is simply delayed to remain aligned.
    signal r_alpha_sum : t_product_q32 := (others => '0');
    signal r_beta_sum_d : t_product_q32 := (others => '0');
    signal r_vld_sum   : std_logic := '0';

    function saturate_q32_to_q12(x : t_product_q32) return t_phase_q12 is
        variable v_q12 : t_product_q32;
    begin
        -- Q?.32 -> Q3.12.  shift_right on SIGNED is arithmetic.
        v_q12 := shift_right(x, C_COEFF_FRAC_BITS);

        if v_q12 > to_signed(C_OUTPUT_MAX_Q12, v_q12'length) then
            return to_signed(C_OUTPUT_MAX_Q12, t_phase_q12'length);
        elsif v_q12 < to_signed(C_OUTPUT_MIN_Q12, v_q12'length) then
            return to_signed(C_OUTPUT_MIN_Q12, t_phase_q12'length);
        else
            return resize(v_q12, t_phase_q12'length);
        end if;
    end function saturate_q32_to_q12;

begin

    pipeline : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_phase_a     <= (others => '0');
                r_phase_b     <= (others => '0');
                r_phase_c     <= (others => '0');
                r_a_row0      <= (others => '0');
                r_b_row0      <= (others => '0');
                r_c_row0      <= (others => '0');
                r_b_row1      <= (others => '0');
                r_c_row1      <= (others => '0');
                r_alpha_ab    <= (others => '0');
                r_alpha_c     <= (others => '0');
                r_beta_sum    <= (others => '0');
                r_alpha_sum   <= (others => '0');
                r_beta_sum_d  <= (others => '0');
                r_vld_in      <= '0';
                r_vld_prod    <= '0';
                r_vld_add1    <= '0';
                r_vld_sum     <= '0';
                o_clarke_alpha <= (others => '0');
                o_clarke_beta  <= (others => '0');
                o_vld          <= '0';
            else
                ----------------------------------------------------------------
                -- Stage 0: sample one complete phase-current transaction.
                ----------------------------------------------------------------
                r_vld_in <= i_vld;
                if i_vld = '1' then
                    r_phase_a <= i_phases_A;
                    r_phase_b <= i_phases_B;
                    r_phase_c <= i_phases_C;
                end if;

                ----------------------------------------------------------------
                -- Stage 1: matrix multiplications.
                ----------------------------------------------------------------
                r_vld_prod <= r_vld_in;
                if r_vld_in = '1' then
                    r_a_row0 <= r_phase_a * C_CLARKE_MTRX_00;
                    r_b_row0 <= r_phase_b * C_CLARKE_MTRX_01;
                    r_c_row0 <= r_phase_c * C_CLARKE_MTRX_02;
                    r_b_row1 <= r_phase_b * C_CLARKE_MTRX_11;
                    r_c_row1 <= r_phase_c * C_CLARKE_MTRX_12;
                end if;

                ----------------------------------------------------------------
                -- Stage 2: one adder level only.
                ----------------------------------------------------------------
                r_vld_add1 <= r_vld_prod;
                if r_vld_prod = '1' then
                    r_alpha_ab <= r_a_row0 + r_b_row0;
                    r_alpha_c  <= r_c_row0;
                    r_beta_sum <= r_b_row1 + r_c_row1;
                end if;

                ----------------------------------------------------------------
                -- Stage 3: final alpha add and beta alignment register.
                ----------------------------------------------------------------
                r_vld_sum <= r_vld_add1;
                if r_vld_add1 = '1' then
                    r_alpha_sum  <= r_alpha_ab + r_alpha_c;
                    r_beta_sum_d <= r_beta_sum;
                end if;

                ----------------------------------------------------------------
                -- Stage 4: fixed-point downshift and saturation.
                ----------------------------------------------------------------
                o_vld <= r_vld_sum;
                if r_vld_sum = '1' then
                    o_clarke_alpha <= saturate_q32_to_q12(r_alpha_sum);
                    o_clarke_beta  <= saturate_q32_to_q12(r_beta_sum_d);
                end if;
            end if;
        end if;
    end process pipeline;

end architecture rtl;
