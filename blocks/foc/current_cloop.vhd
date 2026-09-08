library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity current_cloop is
    port (
        i_clk               : in std_logic;
        i_rst               : in std_logic;

        i_vld               : in std_logic;
        i_target_curr_d     : in signed(15 downto 0); -- Q3.12
        i_target_curr_q     : in signed(15 downto 0); -- Q3.12
        i_actual_curr_d     : in signed(15 downto 0); -- Q3.12
        i_actual_curr_q     : in signed(15 downto 0); -- Q3.12

        o_vld               : out std_logic;
        o_ph_voltage_d      : out signed(17 downto 0); -- Q5.12
        o_ph_voltage_q      : out signed(17 downto 0); -- Q5.12

        REG_FOC_KP_GAIN     : in signed(31 downto 0); -- Q7.24
        REG_FOC_KI_GAIN     : in signed(31 downto 0); -- Q3.28
        REG_FOC_WINDUP_MAX  : in signed(31 downto 0); -- Q11.20
        REG_FOC_VOLTAGE_MAX : in signed(31 downto 0)  -- Q11.20
    );
end entity current_cloop;

architecture rtl of current_cloop is

    subtype t_current_error_q12 is signed(16 downto 0); -- Q4.12
    subtype t_voltage_q12       is signed(17 downto 0); -- Q5.12
    subtype t_voltage_guard_q12 is signed(18 downto 0); -- one guard bit

    function sat_to_voltage_q12(x : signed) return t_voltage_q12 is
        constant C_MAX : t_voltage_q12 := to_signed(2**17 - 1, t_voltage_q12'length);
        constant C_MIN : t_voltage_q12 := to_signed(-(2**17), t_voltage_q12'length);
    begin
        if x > resize(C_MAX, x'length) then
            return C_MAX;
        elsif x < resize(C_MIN, x'length) then
            return C_MIN;
        else
            return resize(x, t_voltage_q12'length);
        end if;
    end function;

    signal r_vld            : std_logic_vector(3 downto 0);

    signal r_target_curr_d  : signed(15 downto 0); -- Q3.12
    signal r_target_curr_q  : signed(15 downto 0); -- Q3.12
    signal r_actual_curr_d  : signed(15 downto 0); -- Q3.12
    signal r_actual_curr_q  : signed(15 downto 0); -- Q3.12

    -- One extra integer/sign bit is required because target - actual can span
    -- twice the range of either 16-bit input.
    signal r_error_curr_d   : t_current_error_q12;
    signal r_error_curr_q   : t_current_error_q12;

    signal r_gain_prop_d    : t_voltage_q12;
    signal r_gain_prop_q    : t_voltage_q12;
    signal r_gain_intg_d    : t_voltage_q12;
    signal r_gain_intg_q    : t_voltage_q12;
    signal r_intg_accum_d   : t_voltage_q12;
    signal r_intg_accum_q   : t_voltage_q12;

begin

    pipeline_ctrl : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_vld <= (others => '0');
            else
                r_vld <= r_vld(2 downto 0) & i_vld;
            end if;
        end if;
    end process pipeline_ctrl;

    latch : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_target_curr_d <= (others => '0');
                r_target_curr_q <= (others => '0');
                r_actual_curr_d <= (others => '0');
                r_actual_curr_q <= (others => '0');
            elsif i_vld = '1' then
                r_target_curr_d <= i_target_curr_d;
                r_target_curr_q <= i_target_curr_q;
                r_actual_curr_d <= i_actual_curr_d;
                r_actual_curr_q <= i_actual_curr_q;
            end if;
        end if;
    end process latch;

    error : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_error_curr_d <= (others => '0');
                r_error_curr_q <= (others => '0');
            elsif r_vld(0) = '1' then
                r_error_curr_d <= resize(r_target_curr_d, r_error_curr_d'length)
                                  - resize(r_actual_curr_d, r_error_curr_d'length);
                r_error_curr_q <= resize(r_target_curr_q, r_error_curr_q'length)
                                  - resize(r_actual_curr_q, r_error_curr_q'length);
            end if;
        end if;
    end process error;

    gains : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_gain_prop_d <= (others => '0');
                r_gain_prop_q <= (others => '0');
                r_gain_intg_d <= (others => '0');
                r_gain_intg_q <= (others => '0');
            elsif r_vld(1) = '1' then
                -- Saturate after fixed-point rescaling instead of blindly
                -- truncating a wide product into Q5.12.
                r_gain_prop_d <= sat_to_voltage_q12(
                    shift_right(r_error_curr_d * REG_FOC_KP_GAIN, 24)
                );
                r_gain_prop_q <= sat_to_voltage_q12(
                    shift_right(r_error_curr_q * REG_FOC_KP_GAIN, 24)
                );
                r_gain_intg_d <= sat_to_voltage_q12(
                    shift_right(r_error_curr_d * REG_FOC_KI_GAIN, 28)
                );
                r_gain_intg_q <= sat_to_voltage_q12(
                    shift_right(r_error_curr_q * REG_FOC_KI_GAIN, 28)
                );
            end if;
        end if;
    end process gains;

    accumulator : process(i_clk)
        variable v_intg_accum_d : t_voltage_guard_q12;
        variable v_intg_accum_q : t_voltage_guard_q12;
        variable v_windup_max   : t_voltage_q12;
        variable v_windup_guard : t_voltage_guard_q12;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_intg_accum_d <= (others => '0');
                r_intg_accum_q <= (others => '0');
            elsif r_vld(2) = '1' then
                -- The extra guard bit prevents overflow before anti-windup
                -- saturation is applied.
                v_intg_accum_d := resize(r_gain_intg_d, v_intg_accum_d'length)
                                  + resize(r_intg_accum_d, v_intg_accum_d'length);
                v_intg_accum_q := resize(r_gain_intg_q, v_intg_accum_q'length)
                                  + resize(r_intg_accum_q, v_intg_accum_q'length);

                v_windup_max   := sat_to_voltage_q12(shift_right(REG_FOC_WINDUP_MAX, 8));
                v_windup_guard := resize(v_windup_max, v_windup_guard'length);

                if v_intg_accum_d > v_windup_guard then
                    r_intg_accum_d <= v_windup_max;
                elsif v_intg_accum_d < -v_windup_guard then
                    r_intg_accum_d <= -v_windup_max;
                else
                    r_intg_accum_d <= resize(v_intg_accum_d, r_intg_accum_d'length);
                end if;

                if v_intg_accum_q > v_windup_guard then
                    r_intg_accum_q <= v_windup_max;
                elsif v_intg_accum_q < -v_windup_guard then
                    r_intg_accum_q <= -v_windup_max;
                else
                    r_intg_accum_q <= resize(v_intg_accum_q, r_intg_accum_q'length);
                end if;
            end if;
        end if;
    end process accumulator;

    output : process(i_clk)
        variable v_output_volt_d : t_voltage_guard_q12;
        variable v_output_volt_q : t_voltage_guard_q12;
        variable v_voltage_max   : t_voltage_q12;
        variable v_voltage_guard : t_voltage_guard_q12;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_ph_voltage_d <= (others => '0');
                o_ph_voltage_q <= (others => '0');
                o_vld          <= '0';
            else
                o_vld <= r_vld(3);

                if r_vld(3) = '1' then
                    -- P + I also needs a guard bit so the sum cannot wrap
                    -- before the configured voltage clamp is applied.
                    v_output_volt_d := resize(r_intg_accum_d, v_output_volt_d'length)
                                       + resize(r_gain_prop_d, v_output_volt_d'length);
                    v_output_volt_q := resize(r_intg_accum_q, v_output_volt_q'length)
                                       + resize(r_gain_prop_q, v_output_volt_q'length);

                    v_voltage_max   := sat_to_voltage_q12(shift_right(REG_FOC_VOLTAGE_MAX, 8));
                    v_voltage_guard := resize(v_voltage_max, v_voltage_guard'length);

                    if v_output_volt_d > v_voltage_guard then
                        o_ph_voltage_d <= v_voltage_max;
                    elsif v_output_volt_d < -v_voltage_guard then
                        o_ph_voltage_d <= -v_voltage_max;
                    else
                        o_ph_voltage_d <= resize(v_output_volt_d, o_ph_voltage_d'length);
                    end if;

                    if v_output_volt_q > v_voltage_guard then
                        o_ph_voltage_q <= v_voltage_max;
                    elsif v_output_volt_q < -v_voltage_guard then
                        o_ph_voltage_q <= -v_voltage_max;
                    else
                        o_ph_voltage_q <= resize(v_output_volt_q, o_ph_voltage_q'length);
                    end if;
                end if;
            end if;
        end if;
    end process output;

end architecture rtl;
