library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity park is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        i_vld       : in std_logic;
        i_alpha     : in signed(15 downto 0);
        i_beta      : in signed(15 downto 0);
        i_cos       : in signed(15 downto 0);
        i_sin       : in signed(15 downto 0);

        o_vld       : out std_logic;
        o_d         : out signed(15 downto 0);
        o_q         : out signed(15 downto 0)
    );
end entity park;

architecture rtl of park is

    -- Current: Q3.12, sin/cos: Q1.14
    subtype t_current_q12 is signed(15 downto 0);
    subtype t_trig_q14    is signed(15 downto 0);
    subtype t_product_q26 is signed(31 downto 0);
    subtype t_sum_q26     is signed(32 downto 0);

    signal r_alpha : t_current_q12 := (others => '0');
    signal r_beta  : t_current_q12 := (others => '0');
    signal r_cos   : t_trig_q14    := (others => '0');
    signal r_sin   : t_trig_q14    := (others => '0');
    signal r_vld   : std_logic := '0';

    function sat_q26_to_q12(
        x : t_sum_q26
    ) return t_current_q12 is
        variable v : t_sum_q26;
    begin
        v := shift_right(x, 14);

        if v > to_signed(32767, v'length) then
            return to_signed(32767, t_current_q12'length);
        elsif v < to_signed(-32768, v'length) then
            return to_signed(-32768, t_current_q12'length);
        else
            return resize(v, t_current_q12'length);
        end if;
    end function;

begin

    sample_input : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_alpha <= (others => '0');
                r_beta  <= (others => '0');
                r_cos   <= (others => '0');
                r_sin   <= (others => '0');
                r_vld   <= '0';
            else
                if i_vld = '1' then
                    r_alpha <= i_alpha;
                    r_beta  <= i_beta;
                    r_cos   <= i_cos;
                    r_sin   <= i_sin;
                    r_vld   <= '1';
                else
                    r_vld <= '0';
                end if;
            end if;
        end if;
    end process;

    calc_park : process(i_clk)
        variable v_alpha_cos : t_product_q26;
        variable v_beta_sin  : t_product_q26;
        variable v_alpha_sin : t_product_q26;
        variable v_beta_cos  : t_product_q26;

        variable v_d_sum     : t_sum_q26;
        variable v_q_sum     : t_sum_q26;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_d   <= (others => '0');
                o_q   <= (others => '0');
                o_vld <= '0';

            elsif r_vld = '1' then
                v_alpha_cos := r_alpha * r_cos;
                v_beta_sin  := r_beta  * r_sin;
                v_alpha_sin := r_alpha * r_sin;
                v_beta_cos  := r_beta  * r_cos;

                v_d_sum :=
                    resize(v_alpha_cos, t_sum_q26'length) +
                    resize(v_beta_sin, t_sum_q26'length);

                v_q_sum :=
                    resize(v_beta_cos, t_sum_q26'length) -
                    resize(v_alpha_sin, t_sum_q26'length);

                o_d <= sat_q26_to_q12(v_d_sum);
                o_q <= sat_q26_to_q12(v_q_sum);

                o_vld <= '1';
            else
                o_vld <= '0';
            end if;
        end if;
    end process;

end architecture rtl;