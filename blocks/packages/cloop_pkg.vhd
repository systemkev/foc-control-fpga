library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use IEEE.fixed_pkg.all;

package cloop_pkg is 

    -- Clarke transform constants
    constant C_CLARKE_K         : real := 2.0 / 3.0;
    constant C_CLARKE_MATRIX_00 : sfixed() := C_CLARKE_K * (1.0);
    constant C_CLARKE_MATRIX_01 : sfixed() := C_CLARKE_K * (-0.5);
    constant C_CLARKE_MATRIX_02 : sfixed() := C_CLARKE_K * (-0.5);
    constant C_CLARKE_MATRIX_10 : sfixed() := C_CLARKE_K * (0.0);
    constant C_CLARKE_MATRIX_11 : sfixed() := C_CLARKE_K * (sqrt(3.0)/2.0);
    constant C_CLARKE_MATRIX_12 : sfixed() := C_CLARKE_K * (-sqrt(3.0)/2.0);

    -- Inverse Clarke transform constants
    constant C_CLARKE_INV_MX_00 : sfixed() := 1.0;
    constant C_CLARKE_INV_MX_01 : sfixed() := 0.0;
    constant C_CLARKE_INV_MX_10 : sfixed() := -1.0 / 2.0;
    constant C_CLARKE_INV_MX_11 : sfixed() := sqrt(3.0)/2.0;
    constant C_CLARKE_INV_MX_20 : sfixed() := -1.0 / 2.0;
    constant C_CLARKE_INV_MX_21 : sfixed() := -sqrt(3.0)/2.0;

    -- Current loop controller gains 
    constant C_KP_PROP_GAIN     : sfixed() := 0.0;
    constant C_KI_INTG_GAIN     : sfixed() := 0.0;

    -- Max I[n] for integral anti-windup
    constant C_WINDUP_MAX       : sfixed(5 downto -12) := to_sfixed(C_MAX_VOLT, 5, -12);

    -- Position loop controller gains
    constant C_POS_KP_GAIN      : sfixed() := ;
    
    constant C_MAX_VELOCITY     : sfixed(8 downto -20) := ;
    constant C_MIN_VELOCITY     : sfixed(8 downto -20) := ;
    constant C_DEADBAND_TICKS   : natural := ;

    -- Velocity loop controller gains
    constant C_VEL_KP_GAIN      : sfixed() := ;
    constant C_VEL_KI_GAIN      : sfixed() := ;

    constant C_MAX_IQ_CURRENT   : t_angl_velocity := ;
    constant C_MAX_CURRENT      : t_phase_current := ;
    
end package cloop_pkg;