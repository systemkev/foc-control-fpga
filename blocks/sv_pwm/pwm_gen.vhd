library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library work;
use work.common_pkg.all;
use work.math_pkg.all;

entity pwm_gen is
    port (
        i_clk           : in std_logic;
        i_rst           : in std_logic;
        i_duty          : in unsigned(log_2_ceil(C_CLKS_IN_PWM_PRD+1)-1 downto 0); 

        o_pwm                   : out std_logic;    
        o_pwm_is_first_cycle    : out std_logic     -- Goes high on a new PWM cycle for the first clock cycle
    );
end entity pwm_gen;

architecture rtl of pwm_gen is
    constant C_PWM_BITS             : integer := log_2_ceil(C_CLKS_IN_PWM_PRD+1); 
    constant C_CLKS_IN_PWM_PERIOD   : unsigned(C_PWM_BITS-1 downto 0)   
                                        := to_unsigned(C_CLKS_IN_PWM_PRD, C_PWM_BITS);    -- Number of clock ticks in half a PWM period 

    signal r_pwm_start              : unsigned(C_PWM_BITS-1 downto 0);
    signal r_pwm_end                : unsigned(C_PWM_BITS-1 downto 0);
    signal r_pwm_counter            : unsigned(C_PWM_BITS-1 downto 0);
begin 
    padding : process (i_clk)
        variable diff       : unsigned(C_PWM_BITS-1 downto 0);
        variable diff_div_2 : unsigned(C_PWM_BITS-1 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                r_pwm_start <= (others => '0');
                r_pwm_end   <= (others => '0');
            -- Only update i_duty on the last clock cycle of the last PWM cycle
            elsif r_pwm_counter = C_CLKS_IN_PWM_PERIOD-1 then
                if i_duty <= C_CLKS_IN_PWM_PRD then
                    diff    := C_CLKS_IN_PWM_PERIOD - i_duty;
                else
                    diff    := (others => '0');
                end if;

                diff_div_2  := '0' & diff(C_PWM_BITS-1 downto 1);

                r_pwm_start <= diff_div_2;
                r_pwm_end   <= diff_div_2 + i_duty;
            end if;
        end if;
    end process padding;

    -- Counter for PWM cycle
    counter : process (i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_pwm_counter       <= (others => '0');
            else 
                if r_pwm_counter = C_CLKS_IN_PWM_PERIOD - 1 then
                    r_pwm_counter   <= (others => '0');
                else
                    r_pwm_counter   <= r_pwm_counter + 1;
                end if;
            end if;
        end if;
    end process counter;

    drive_pwm : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                o_pwm                   <= '0';
                o_pwm_is_first_cycle    <= '0';
            else
                if r_pwm_counter = 0 then
                    o_pwm_is_first_cycle <= '1';
                else
                    o_pwm_is_first_cycle <= '0';
                end if;

                if r_pwm_counter < r_pwm_start then
                    o_pwm <= '0';
                elsif r_pwm_counter >= r_pwm_end then 
                    o_pwm <= '0';
                else
                    o_pwm <= '1';
                end if;
            end if;
        end if;
    end process drive_pwm;
end architecture rtl;