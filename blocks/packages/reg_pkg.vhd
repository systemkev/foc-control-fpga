library IEEE;
use IEEE.std_logic_1164.all;

package reg_pkg is
    
    constant C_NUM_REGISTERS    : natural := 11;

    -- Register File Addresses
    constant C_FOC_KP_GAIN_ADDR : std_logic_vector(7 downto 0) := x"00";    -- Current loop KP gain 
    constant C_FOC_KI_GAIN_ADDR : std_logic_vector(7 downto 0) := x"01";    -- Current loop KI gain 
    constant C_FOC_WINDUP_MAX   : std_logic_vector(7 downto 0) := x"02";    -- Current loop windup max
    constant C_FOC_VOLTAGE_MAX  : std_logic_vector(7 downto 0) := x"03";    -- Current loop max output voltage
    constant C_VEL_KP_GAIN_ADDR : std_logic_vector(7 downto 0) := x"04";    -- Velocity loop KP gain 
    constant C_VEL_KI_GAIN_ADDR : std_logic_vector(7 downto 0) := x"05";    -- Velocity loop KI gain 
    constant C_VEL_WINDUP_MAX   : std_logic_vector(7 downto 0) := x"06";    -- Velocity loop windup max
    constant C_VEL_CURRENT_MAX  : std_logic_vector(7 downto 0) := x"07";    -- Velocity loop max output current
    constant C_POS_KP_GAIN_ADDR : std_logic_vector(7 downto 0) := x"08";    -- Postion loop KP gain 
    constant C_POS_MAX_VELOCITY : std_logic_vector(7 downto 0) := x"09";    -- Postion loop max output velocity
    constant C_POS_DEADBAND_TCK : std_logic_vector(7 downto 0) := x"0A";    -- Position loop number of deadband ticks

    constant C_LAST_ADDR        : std_logic_vector(7 downto 0) := x"0A";
    
end package reg_pkg;