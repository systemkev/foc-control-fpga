library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

package cloop_pkg is 

    -- Clarke transform constants
    constant C_CLARKE_K         : real := 2.0 / 3.0;
    -- Q2.20
    constant C_CLARKE_MATRIX_00 : signed(22 downto 0) := to_signed(integer(round(C_CLARKE_K * (1.0) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_MATRIX_01 : signed(22 downto 0) := to_signed(integer(round(C_CLARKE_K * (-0.5) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_MATRIX_02 : signed(22 downto 0) := to_signed(integer(round(C_CLARKE_K * (-0.5) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_MATRIX_10 : signed(22 downto 0) := to_signed(integer(round(C_CLARKE_K * (0.0) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_MATRIX_11 : signed(22 downto 0) := to_signed(integer(round(C_CLARKE_K * (sqrt(3.0)/2.0) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_MATRIX_12 : signed(22 downto 0) := to_signed(integer(round(C_CLARKE_K * (-sqrt(3.0)/2.0) * (2.0 ** 20))), 23);

    -- Inverse Clarke transform constants
    -- Q2.20
    constant C_CLARKE_INV_MX_00 : signed(22 downto 0) := to_signed(integer(round(1.0 * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_INV_MX_01 : signed(22 downto 0) := to_signed(integer(round(0.0 * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_INV_MX_10 : signed(22 downto 0) := to_signed(integer(round((-1.0 / 2.0) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_INV_MX_11 : signed(22 downto 0) := to_signed(integer(round((sqrt(3.0)/2.0) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_INV_MX_20 : signed(22 downto 0) := to_signed(integer(round((-1.0 / 2.0) * (2.0 ** 20))), 23);
    -- Q2.20
    constant C_CLARKE_INV_MX_21 : signed(22 downto 0) := to_signed(integer(round((-sqrt(3.0)/2.0) * (2.0 ** 20))), 23);
    
end package cloop_pkg;
