library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity clarke is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- Input phase currents.
        --
        -- 16-bit signed Q3.12:
        --   real_value = signed_value / 4096
        --
        -- Expected application range: [-5.0, +5.0] A
        i_vld       : in std_logic;
        i_phases_A  : in signed(15 downto 0);
        i_phases_B  : in signed(15 downto 0);
        i_phases_C  : in signed(15 downto 0);

        -- Clarke outputs.
        --
        -- 16-bit signed Q3.12:
        --   real_value = signed_value / 4096
        --
        -- Saturated to [-5.0, +5.0]
        o_vld          : out std_logic;
        o_clarke_alpha : out signed(15 downto 0);
        o_clarke_beta  : out signed(15 downto 0)
    );
end entity clarke;


architecture rtl of clarke is

    ---------------------------------------------------------------------------
    -- Fixed-point formats
    ---------------------------------------------------------------------------
    --
    -- Phase current:
    --
    --     signed(15 downto 0)
    --     12 fractional bits
    --
    -- Coefficients:
    --
    --     signed(22 downto 0)
    --     20 fractional bits
    --
    -- This matches the bit precision previously provided by:
    --
    --     sfixed(2 downto -20)
    --
    -- Multiplication:
    --
    --     16 bits * 23 bits = 39 bits
    --
    -- Fractional bits:
    --
    --     12 + 20 = 32 fractional bits
    --
    -- Therefore:
    --
    --     product >> 20
    --
    -- returns the result to 12 fractional bits.
    ---------------------------------------------------------------------------

    subtype t_phase_q12 is signed(15 downto 0);
    subtype t_coeff_q20 is signed(22 downto 0);
    subtype t_product_q32 is signed(38 downto 0);


    ---------------------------------------------------------------------------
    -- Clarke matrix coefficients
    ---------------------------------------------------------------------------
    --
    -- Clarke transform:
    --
    -- alpha = (2/3) * A
    --       - (1/3) * B
    --       - (1/3) * C
    --
    -- beta  = (1/sqrt(3)) * B
    --       - (1/sqrt(3)) * C
    --
    --
    -- Coefficients are scaled by 2^20.
    --
    -- 2/3:
    --
    --     0.6666666667 * 2^20
    --       = 699050.666...
    --       -> 699051
    --
    -- -1/3:
    --
    --    -0.3333333333 * 2^20
    --       = -349525.333...
    --       -> -349525
    --
    -- 1/sqrt(3):
    --
    --     0.5773502692 * 2^20
    --       = 605395.636...
    --       -> 605396
    ---------------------------------------------------------------------------

    constant C_CLARKE_MTRX_00 : t_coeff_q20 :=
        to_signed(699051, t_coeff_q20'length);

    constant C_CLARKE_MTRX_01 : t_coeff_q20 :=
        to_signed(-349525, t_coeff_q20'length);

    constant C_CLARKE_MTRX_02 : t_coeff_q20 :=
        to_signed(-349525, t_coeff_q20'length);


    -- Row 1 coefficient for phase A is exactly zero, so no multiplier is
    -- required for it.

    constant C_CLARKE_MTRX_11 : t_coeff_q20 :=
        to_signed(605396, t_coeff_q20'length);

    constant C_CLARKE_MTRX_12 : t_coeff_q20 :=
        to_signed(-605396, t_coeff_q20'length);


    ---------------------------------------------------------------------------
    -- Output limits
    ---------------------------------------------------------------------------
    --
    -- Q3.12 representation of 5.0:
    --
    --     5.0 * 4096 = 20480
    ---------------------------------------------------------------------------

    constant C_OUTPUT_MAX_Q12 : integer :=  20480;
    constant C_OUTPUT_MIN_Q12 : integer := -20480;

    constant C_COEFF_FRAC_BITS : natural := 20;


    ---------------------------------------------------------------------------
    -- Registered input stage
    ---------------------------------------------------------------------------

    signal r_phase_A : t_phase_q12 := (others => '0');
    signal r_phase_B : t_phase_q12 := (others => '0');
    signal r_phase_C : t_phase_q12 := (others => '0');

    signal r_vld : std_logic := '0';


    ---------------------------------------------------------------------------
    -- Convert accumulated Q?.32 result back to saturated Q3.12
    ---------------------------------------------------------------------------
    --
    -- Input has 32 fractional bits.
    --
    -- Shift right by 20:
    --
    --     32 frac bits - 20 = 12 frac bits
    --
    -- Then saturate against +/-5.0 in raw Q3.12 representation.
    ---------------------------------------------------------------------------

    function saturate_q32_to_q12 (
        x : t_product_q32
    ) return t_phase_q12 is

        variable v_q12 : t_product_q32;

    begin

        -----------------------------------------------------------------------
        -- Convert from 32 fractional bits to 12 fractional bits.
        --
        -- numeric_std.shift_right() on SIGNED performs an arithmetic shift,
        -- preserving the sign.
        -----------------------------------------------------------------------

        v_q12 := shift_right(x, C_COEFF_FRAC_BITS);


        -----------------------------------------------------------------------
        -- Positive saturation
        -----------------------------------------------------------------------

        if v_q12 > to_signed(
            C_OUTPUT_MAX_Q12,
            t_product_q32'length
        ) then

            return to_signed(
                C_OUTPUT_MAX_Q12,
                t_phase_q12'length
            );


        -----------------------------------------------------------------------
        -- Negative saturation
        -----------------------------------------------------------------------

        elsif v_q12 < to_signed(
            C_OUTPUT_MIN_Q12,
            t_product_q32'length
        ) then

            return to_signed(
                C_OUTPUT_MIN_Q12,
                t_phase_q12'length
            );


        -----------------------------------------------------------------------
        -- Value fits in the desired output range.
        -----------------------------------------------------------------------

        else

            return resize(
                v_q12,
                t_phase_q12'length
            );

        end if;

    end function saturate_q32_to_q12;


begin


    ---------------------------------------------------------------------------
    -- Input register
    ---------------------------------------------------------------------------

    sample_input : process(i_clk)
    begin

        if rising_edge(i_clk) then

            if i_rst = '1' then

                r_phase_A <= (others => '0');
                r_phase_B <= (others => '0');
                r_phase_C <= (others => '0');

                r_vld <= '0';

            else

                if i_vld = '1' then

                    r_phase_A <= i_phases_A;
                    r_phase_B <= i_phases_B;
                    r_phase_C <= i_phases_C;

                    r_vld <= '1';

                else

                    r_vld <= '0';

                end if;

            end if;

        end if;

    end process sample_input;


    ---------------------------------------------------------------------------
    -- Clarke transform
    ---------------------------------------------------------------------------

    calc_clarke : process(i_clk)

        -----------------------------------------------------------------------
        -- Individual matrix products.
        --
        -- Q3.12 * coefficient-with-20-frac-bits
        --
        -- gives a signed 39-bit number with 32 fractional bits.
        -----------------------------------------------------------------------

        variable v_A_row0 : t_product_q32;
        variable v_B_row0 : t_product_q32;
        variable v_C_row0 : t_product_q32;

        variable v_B_row1 : t_product_q32;
        variable v_C_row1 : t_product_q32;


        -----------------------------------------------------------------------
        -- Accumulators.
        --
        -- 39 bits is comfortably large enough for the Clarke sums over the
        -- full signed 16-bit input range.
        -----------------------------------------------------------------------

        variable v_alpha_sum : t_product_q32;
        variable v_beta_sum  : t_product_q32;

    begin

        if rising_edge(i_clk) then

            if i_rst = '1' then

                o_clarke_alpha <= (others => '0');
                o_clarke_beta  <= (others => '0');

                o_vld <= '0';


            elsif r_vld = '1' then

                ----------------------------------------------------------------
                -- Alpha row
                --
                -- alpha =
                --
                --     (2/3) A
                --   - (1/3) B
                --   - (1/3) C
                ----------------------------------------------------------------

                v_A_row0 := r_phase_A * C_CLARKE_MTRX_00;
                v_B_row0 := r_phase_B * C_CLARKE_MTRX_01;
                v_C_row0 := r_phase_C * C_CLARKE_MTRX_02;

                v_alpha_sum :=
                    v_A_row0 +
                    v_B_row0 +
                    v_C_row0;


                ----------------------------------------------------------------
                -- Beta row
                --
                -- beta =
                --
                --      (1/sqrt(3)) B
                --    - (1/sqrt(3)) C
                --
                -- Phase A has a coefficient of exactly zero and therefore
                -- doesn't need to be multiplied.
                ----------------------------------------------------------------

                v_B_row1 := r_phase_B * C_CLARKE_MTRX_11;
                v_C_row1 := r_phase_C * C_CLARKE_MTRX_12;

                v_beta_sum :=
                    v_B_row1 +
                    v_C_row1;


                ----------------------------------------------------------------
                -- Return to Q3.12 and saturate to +/-5.0.
                ----------------------------------------------------------------

                o_clarke_alpha <= saturate_q32_to_q12(v_alpha_sum);
                o_clarke_beta  <= saturate_q32_to_q12(v_beta_sum);

                o_vld <= '1';


            else

                ----------------------------------------------------------------
                -- Outputs intentionally hold their previous values when no
                -- input sample is valid.
                ----------------------------------------------------------------

                o_vld <= '0';

            end if;

        end if;

    end process calc_clarke;


end architecture rtl;