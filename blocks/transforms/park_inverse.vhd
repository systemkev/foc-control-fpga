library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity park_inverse is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        i_vld       : in std_logic;
        i_d         : in signed(17 downto 0);
        i_q         : in signed(17 downto 0);
        i_cos       : in signed(15 downto 0);
        i_sin       : in signed(15 downto 0);

        o_vld       : out std_logic;
        o_alpha     : out signed(17 downto 0);
        o_beta      : out signed(17 downto 0)
    );
end entity park_inverse;

architecture rtl of park_inverse is

    -- Voltage: Q5.12, sin/cos: Q1.14
    subtype t_voltage_q12 is signed(17 downto 0);
    subtype t_trig_q14    is signed(15 downto 0);
    subtype t_product_q26 is signed(33 downto 0);
    subtype t_sum_q26     is signed(34 downto 0);

    signal r_d   : t_voltage_q12 := (others => '0');
    signal r_q   : t_voltage_q12 := (others => '0');
    signal r_cos : t_trig_q14    := (others => '0');
    signal r_sin : t_trig_q14    := (others => '0');
    signal r_vld : std_logic := '0';

    function sat_q26_to_q12(
        x : t_sum_q26
    ) return t_voltage_q12 is
        variable v : t_sum_q26;
    begin
        v := shift_right(x, 14);

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
                r_d   <= (others => '0');
                r_q   <= (others => '0');
                r_cos <= (others => '0');
                r_sin <= (others => '0');
                r_vld <= '0';
            else
                if i_vld = '1' then
                    r_d   <= i_d;
                    r_q   <= i_q;
                    r_cos <= i_cos;
                    r_sin <= i_sin;
                    r_vld <= '1';
                else
                    r_vld <= '0';
                end if;
            end if;
        end if;
    end process;

    calc_inverse_park : process(i_clk)
        variable v_d_cos : t_product_q26;
        variable v_q_sin : t_product_q26;
        variable v_d_sin : t_product_q26;
        variable v_q_cos : t_product_q26;

        variable v_alpha_sum : t_sum_q26;
        variable v_beta_sum  : t_sum_q26;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_alpha <= (others => '0');
                o_beta  <= (others => '0');
                o_vld   <= '0';

            elsif r_vld = '1' then
                v_d_cos := r_d * r_cos;
                v_q_sin := r_q * r_sin;
                v_d_sin := r_d * r_sin;
                v_q_cos := r_q * r_cos;

                v_alpha_sum :=
                    resize(v_d_cos, t_sum_q26'length) -
                    resize(v_q_sin, t_sum_q26'length);

                v_beta_sum :=
                    resize(v_d_sin, t_sum_q26'length) +
                    resize(v_q_cos, t_sum_q26'length);

                o_alpha <= sat_q26_to_q12(v_alpha_sum);
                o_beta  <= sat_q26_to_q12(v_beta_sum);

                o_vld <= '1';
            else
                o_vld <= '0';
            end if;
        end if;
    end process;

end architecture rtl;