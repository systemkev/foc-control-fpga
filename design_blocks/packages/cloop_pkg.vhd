library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use IEEE.fixed_pkg.all;

package cloop_pkg is 
    -- Clarke transform constants
    constant C_CLARKE_K         : real := 2.0 / 3.0;
    constant C_CLARKE_MATRIX_00 : real := C_CLARKE_K * (1.0);
    constant C_CLARKE_MATRIX_01 : real := C_CLARKE_K * (-0.5);
    constant C_CLARKE_MATRIX_02 : real := C_CLARKE_K * (-0.5);
    constant C_CLARKE_MATRIX_10 : real := C_CLARKE_K * (0.0);
    constant C_CLARKE_MATRIX_11 : real := C_CLARKE_K * (sqrt(3.0)/2.0);
    constant C_CLARKE_MATRIX_12 : real := C_CLARKE_K * (-sqrt(3.0)/2.0);

    -- Inverse Clarke transform constants
    constant C_CLARKE_INV_MX_00 : real := 1.0;
    constant C_CLARKE_INV_MX_01 : real := 0.0;
    constant C_CLARKE_INV_MX_10 : real := -1.0 / 2.0;
    constant C_CLARKE_INV_MX_11 : real := sqrt(3.0)/2.0;
    constant C_CLARKE_INV_MX_20 : real := -1.0 / 2.0;
    constant C_CLARKE_INV_MX_21 : real := -sqrt(3.0)/2.0;
end package cloop_pkg;