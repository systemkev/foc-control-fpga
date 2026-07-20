library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use IEEE.fixed_pkg.all;

package cloop_pkg is 
    -- Interface records
    type t_phase_record is record 
        vld         : std_logic;
        A           : sfixed(3 downto -12);
        B           : sfixed(3 downto -12);
        C           : sfixed(3 downto -12);
    end record t_phase_record;

    type t_clarke_record is record 
        vld         : std_logic;
        alpha       : sfixed(3 downto -12);
        beta        : sfixed(3 downto -12);
    end record t_clarke_record;

    type t_park_record is record
        vld         : std_logic;
        d           : sfixed(3 downto -12);
        q           : sfixed(3 downto -12);
    end record t_park_record;

    -- Clarke transform constants
    constant C_CLARKE_K         : real := 2.0 / 3.0;
    constant C_CLARKE_MATRIX_00 : real := C_CLARKE_K * (1.0);
    constant C_CLARKE_MATRIX_01 : real := C_CLARKE_K * (-0.5);
    constant C_CLARKE_MATRIX_02 : real := C_CLARKE_K * (-0.5);
    constant C_CLARKE_MATRIX_10 : real := C_CLARKE_K * (0.0);
    constant C_CLARKE_MATRIX_11 : real := C_CLARKE_K * (sqrt(3.0)/2.0);
    constant C_CLARKE_MATRIX_12 : real := C_CLARKE_K * (-sqrt(3.0)/2.0);

end package cloop_pkg;