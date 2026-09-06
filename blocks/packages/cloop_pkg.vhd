library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use IEEE.fixed_pkg.all;

package cloop_pkg is 

    -- Clarke transform constants
    constant C_CLARKE_K         : real := 2.0 / 3.0;
    constant C_CLARKE_MATRIX_00 : sfixed(2 downto -20) := to_sfixed(C_CLARKE_K * (1.0), 2, -20);
    constant C_CLARKE_MATRIX_01 : sfixed(2 downto -20) := to_sfixed(C_CLARKE_K * (-0.5), 2, -20);
    constant C_CLARKE_MATRIX_02 : sfixed(2 downto -20) := to_sfixed(C_CLARKE_K * (-0.5), 2, -20);
    constant C_CLARKE_MATRIX_10 : sfixed(2 downto -20) := to_sfixed(C_CLARKE_K * (0.0), 2, -20);
    constant C_CLARKE_MATRIX_11 : sfixed(2 downto -20) := to_sfixed(C_CLARKE_K * (sqrt(3.0)/2.0), 2, -20);
    constant C_CLARKE_MATRIX_12 : sfixed(2 downto -20) := to_sfixed(C_CLARKE_K * (-sqrt(3.0)/2.0), 2, -20);

    -- Inverse Clarke transform constants
    constant C_CLARKE_INV_MX_00 : sfixed(2 downto -20) := to_sfixed(1.0, 2, -20);
    constant C_CLARKE_INV_MX_01 : sfixed(2 downto -20) := to_sfixed(0.0, 2, -20);
    constant C_CLARKE_INV_MX_10 : sfixed(2 downto -20) := to_sfixed(-1.0 / 2.0, 2, -20);
    constant C_CLARKE_INV_MX_11 : sfixed(2 downto -20) := to_sfixed(sqrt(3.0)/2.0, 2, -20);
    constant C_CLARKE_INV_MX_20 : sfixed(2 downto -20) := to_sfixed(-1.0 / 2.0, 2, -20);
    constant C_CLARKE_INV_MX_21 : sfixed(2 downto -20) := to_sfixed(-sqrt(3.0)/2.0, 2, -20);
    
end package cloop_pkg;