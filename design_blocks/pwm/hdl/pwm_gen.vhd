library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity pwm_gen is
    port (
        i_clk           : in std_logic;
        i_rst           : in std_logic;
        i_duty          : in unsigned; 

        o_pwm           : out std_logic;    
        o_pwm_start     : out std_logic     -- Goes high on a new PWM cycle for the first clock cycle
    );
end entity pwm_gen;

architecture rtl of pwm_gen is 
    constant C_CLKS_IN_PWM_PERIOD   : natural   := integer(C_CLK_FREQ) / integer(C_PWM_FREQ);
    constant C_CLKS_HALF_PWM_PRD    : natural   := C_CLKS_IN_PWM_PERIOD / 2;

    signal r_pwm_counter            : natural range 0 to C_CLKS_HALF_PWM_PRD;
    signal r_rising_cycle           : std_logic;
begin 
    -- Combinational assignments
    o_pwm_start     <= r_pwm_start;    

    -- Counter for PWM cycle
    counter : process (i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_pwm_counter       <= 0;
                r_rising_cycle      <= '1';
                o_pwm               <= '0';
                o_pwm_start         <= '0';
            else 
                -- Counter logic
                o_pwm_start <= '0';
                if r_rising_cycle = '1' then
                    if r_pwm_counter < C_CLKS_HALF_PWM_PRD then 
                        r_pwm_counter   <= r_pwm_counter+1; 
                        if r_pwm_counter = 0 then 
                            o_pwm_start <= '1';
                        else
                            o_pwm_start <= '0';
                        end if;
                    else 
                        o_pwm_start <= '0';
                        r_pwm_counter   <= r_pwm_counter-1;
                        r_rising_cycle  <= '0';
                    end if;
                else
                    if r_pwm_counter > 0 then 
                        r_pwm_counter   <= r_pwm_counter-1;
                    else
                        r_pwm_counter   <= r_pwm_counter+1;
                        r_rising_cycle  <= '1';
                    end if;
                end if;

                -- Drive PWM
                if i_duty = 0 then
                    o_pwm   <= '0';
                elsif i_duty >= C_CLKS_HALF_PWM_PRD then 
                    o_pwm   <= '1';
                elsif r_pwm_counter < i_duty then 
                    o_pwm   <= '1';
                else
                    o_pwm   <= '0';
                end if;
            end if;
        end if;
    end process counter;

end architecture rtl;