library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

package common_pkg is 
    -- System constants
    constant C_CLK_FREQ             : real      := 50.0e6;      -- 50 Mhz clock 
    constant C_PWM_FREQ             : real      := 20.0e3;      -- 20 kHz PWM frequency
    constant C_CLKS_IN_PWM_PRD      : natural   := integer(C_CLK_FREQ / C_PWM_FREQ);
    constant C_VBUS                 : real      := 24.0;        -- 24V bus voltage
    constant C_MAX_VOLT             : real      := C_VBUS / sqrt(3.0);

    type t_phase_current is sfixed(3 downto -12);   -- Max value: +7.9997 Amps, Min value: -8.0000 Amps
    type t_phase_voltage is sfixed(5 downto -12);   -- Max value: +31.9997 Volts, Min value: -32.0000 Volts

    type t_ph_volt_record is record 
        vld         : std_logic;
        A           : t_phase_voltage;
        B           : t_phase_voltage;
        C           : t_phase_voltage;
    end record t_ph_volt_record; 

    type t_clarke_volt_record is record 
        vld         : std_logic;
        alpha       : t_phase_voltage;
        beta        : t_phase_voltage;
    end record t_clarke_volt_record;

    type t_park_volt_record is record 
        vld         : std_logic;
        d           : t_phase_voltage;
        q           : t_phase_voltage;

    -- Interface records
    type t_phase_record is record 
        vld         : std_logic;
        A           : t_phase_current;
        B           : t_phase_current;
        C           : t_phase_current;
    end record t_phase_record;

    type t_clarke_record is record 
        vld         : std_logic;
        alpha       : t_phase_current;
        beta        : t_phase_current;
    end record t_clarke_record;

    type t_park_record is record
        vld         : std_logic;
        d           : t_phase_current;
        q           : t_phase_current;
    end record t_park_record;
end package common_pkg;