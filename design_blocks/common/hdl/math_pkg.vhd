library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

package math_pkg is 
    function log_2_ceil_real (n: real) return integer;
    function log_2_ceil (n: integer) return integer;
end package math_pkg;

package body math_pkg is
    function log_2_ceil_real (n: real) return integer is 
    begin 
        -- Guard against 0 or negative inputs
        if n <= 0.0 then
            return 0;
        else
            -- ceil returns a real, so we safely convert it to an integer
            return integer(ceil(log2(n)));
        end if;
    end function log_2_ceil_real;

    function log_2_ceil (n: integer) return integer is
    begin
        if n <= 0 then
            return 0;
        else
            return integer(ceil(log2(real(n))));
        end if;
    end function log_2_ceil;
end package body math_pkg;