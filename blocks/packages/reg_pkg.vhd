library ieee;
use ieee.std_logic_1164.all;

package reg_pkg is
    -- Keep the original tuning addresses stable.
    constant C_FOC_KP_GAIN_ADDR : std_logic_vector(7 downto 0) := x"00";
    constant C_FOC_KI_GAIN_ADDR : std_logic_vector(7 downto 0) := x"01";
    constant C_FOC_WINDUP_MAX   : std_logic_vector(7 downto 0) := x"02";
    constant C_FOC_VOLTAGE_MAX  : std_logic_vector(7 downto 0) := x"03";
    constant C_VEL_KP_GAIN_ADDR : std_logic_vector(7 downto 0) := x"04";
    constant C_VEL_KI_GAIN_ADDR : std_logic_vector(7 downto 0) := x"05";
    constant C_VEL_WINDUP_MAX   : std_logic_vector(7 downto 0) := x"06";
    constant C_VEL_CURRENT_MAX  : std_logic_vector(7 downto 0) := x"07";
    constant C_POS_KP_GAIN_ADDR : std_logic_vector(7 downto 0) := x"08";
    constant C_POS_MAX_VELOCITY : std_logic_vector(7 downto 0) := x"09";
    constant C_POS_DEADBAND_TCK : std_logic_vector(7 downto 0) := x"0A";

    -- Controller command/configuration registers.
    constant C_CONTROL_ADDR      : std_logic_vector(7 downto 0) := x"0B";
    constant C_TARGET_DQ_ADDR    : std_logic_vector(7 downto 0) := x"0C";
    constant C_TARGET_VEL_ADDR   : std_logic_vector(7 downto 0) := x"0D";
    constant C_TARGET_POS_ADDR   : std_logic_vector(7 downto 0) := x"0E";
    constant C_MANUAL_ELEC_ADDR  : std_logic_vector(7 downto 0) := x"0F";

    -- Read-only telemetry/status registers.
    constant C_STATUS_ADDR       : std_logic_vector(7 downto 0) := x"10";
    constant C_ACTUAL_POS_ADDR   : std_logic_vector(7 downto 0) := x"11";
    constant C_ACTUAL_VEL_ADDR   : std_logic_vector(7 downto 0) := x"12";
    constant C_PHASE_AB_ADDR     : std_logic_vector(7 downto 0) := x"13";
    constant C_PHASE_C_ID_ADDR   : std_logic_vector(7 downto 0) := x"14";
    constant C_IQ_ELEC_ADDR      : std_logic_vector(7 downto 0) := x"15";
    constant C_CAL_OFF_AB_ADDR   : std_logic_vector(7 downto 0) := x"16";
    constant C_CAL_OFF_C_ADDR    : std_logic_vector(7 downto 0) := x"17";

    constant C_LAST_ADDR         : std_logic_vector(7 downto 0) := x"17";
    constant C_NUM_REGISTERS     : natural := 24;

    -- CONTROL register bit definitions.
    constant C_CTRL_ENABLE_BIT       : natural := 0;
    constant C_CTRL_MODE_LSB         : natural := 1; -- [2:1]
    constant C_CTRL_MODE_MSB         : natural := 2;
    constant C_CTRL_CALIBRATE_BIT    : natural := 3; -- write-one pulse
    constant C_CTRL_CLEAR_FAULT_BIT  : natural := 4; -- write-one pulse
    constant C_CTRL_MANUAL_ELEC_BIT  : natural := 5;
    constant C_CTRL_SENSOR_INV_BIT   : natural := 6;
    constant C_CTRL_WATCHDOG_BIT     : natural := 7;
end package reg_pkg;
