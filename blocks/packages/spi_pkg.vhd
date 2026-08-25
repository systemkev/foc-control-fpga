library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

use work.common_pkg.all;

package spi_pkg is
    
    constant C_READ_ANGLE_CMD   : std_logic_vector(15 downto 0) := x"AAAA";
    constant C_CLR_ERR_FL_CMD   : std_logic_vector(15 downto 0) := x"4001";

    constant C_SCLK_FREQ        : real      := 10.0e6;
    constant C_SCLK_SWTCH_CNT   : natural   := integer(C_CLK_FREQ / C_SCLK_FREQ / 2.0); 
    constant C_WAIT_MAX         : natural   := integer(350.0e-9 / (1.0 / C_CLK_FREQ) + 0.5); -- 350ns wait period needed (+0.5 added for rounding up)
    
    type t_spi_frame is array (0 to 1) of std_logic_vector(15 downto 0);

    type t_spi_fsm is (
        ST_IDLE,
        ST_WAIT,
        ST_SPI,
        ST_CHECK,
        ST_DONE
    );

end package spi_pkg;