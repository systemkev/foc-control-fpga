library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

package common_pkg is 
    -- System constants
    constant C_CLK_FREQ             : real      := 100.0e6;     -- 100 Mhz clock 
    constant C_PWM_FREQ             : real      := 20.0e3;      -- 20 kHz PWM frequency
    constant C_CLKS_IN_PWM_PRD      : natural   := integer(C_CLK_FREQ / C_PWM_FREQ);
    constant C_VBUS                 : real      := 12.0;        -- 24V bus voltage
    constant C_MAX_VOLT             : real      := C_VBUS / sqrt(3.0);
    constant C_NUM_POLE_PAIRS       : integer   := 7;

    -- Q3.12
    subtype t_phase_current is signed(15 downto 0);   -- Max value: +7.9997 Amps, Min value: -8.0000 Amps
    -- Q5.12
    subtype t_phase_voltage is signed(17 downto 0);   -- Max value: +31.9997 Volts, Min value: -32.0000 Volts
    -- Q8.20
    subtype t_angl_velocity is signed(28 downto 0);
    subtype t_angl_position is signed(31 downto 0);
    subtype t_angl_raw_unwr is unsigned(13 downto 0);
    subtype t_cordic_inputs is signed(15 downto 0);
    -- Q1.14
    subtype t_sin_cos_outpt is signed(15 downto 0);

end package common_pkg;

