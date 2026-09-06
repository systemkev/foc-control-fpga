library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use IEEE.math_real.all;

package common_pkg is 
    -- System constants
    constant C_CLK_FREQ             : real      := 100.0e6;     -- 50 Mhz clock 
    constant C_PWM_FREQ             : real      := 20.0e3;      -- 20 kHz PWM frequency
    constant C_CLKS_IN_PWM_PRD      : natural   := integer(C_CLK_FREQ / C_PWM_FREQ);
    constant C_VBUS                 : real      := 24.0;        -- 24V bus voltage
    constant C_MAX_VOLT             : real      := C_VBUS / sqrt(3.0);
    constant C_NUM_POLE_PAIRS       : integer   := 7;

    subtype t_phase_current is sfixed(3 downto -12);   -- Max value: +7.9997 Amps, Min value: -8.0000 Amps
    subtype t_phase_voltage is sfixed(5 downto -12);   -- Max value: +31.9997 Volts, Min value: -32.0000 Volts
    subtype t_angl_velocity is sfixed(8 downto -20);
    subtype t_angl_position is signed(31 downto 0);
    subtype t_angl_raw_unwr is unsigned(13 downto 0);
    subtype t_cordic_inputs is signed(15 downto 0);
    subtype t_sin_cos_outpt is sfixed(1 downto -14);

    type t_pwm_cnt_phase is record 
        A           : integer range 0 to C_CLKS_IN_PWM_PRD;
        B           : integer range 0 to C_CLKS_IN_PWM_PRD;
        C           : integer range 0 to C_CLKS_IN_PWM_PRD;
    end record t_pwm_cnt_phase;

    type t_abc_volt is record 
        A           : t_phase_voltage;
        B           : t_phase_voltage;
        C           : t_phase_voltage;
    end record t_abc_volt; 

    type t_ab_volt is record 
        alpha       : t_phase_voltage;
        beta        : t_phase_voltage;
    end record t_ab_volt;

    type t_dq_volt is record 
        d           : t_phase_voltage;
        q           : t_phase_voltage;
    end record t_dq_volt;

    -- Interface records
    type t_abc_phase is record 
        A           : t_phase_current;
        B           : t_phase_current;
        C           : t_phase_current;
    end record t_abc_phase;

    type t_ab_phase is record 
        alpha       : t_phase_current;
        beta        : t_phase_current;
    end record t_ab_phase;

    type t_dq_phase is record
        d           : t_phase_current;
        q           : t_phase_current;
    end record t_dq_phase;
end package common_pkg;