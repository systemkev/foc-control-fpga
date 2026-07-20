library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

package common_pkg is 
    constant C_CLK_FREQ             : real      := 50.0e6;      -- 50 Mhz clock 
    constant C_PWM_FREQ             : real      := 20.0e3;      -- 20 kHz PWM frequency
    constant C_CLKS_IN_PWM_PRD      : natural   := integer(C_CLK_FREQ / C_PWM_FREQ);
end package common_pkg;