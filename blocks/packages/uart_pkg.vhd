library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use work.common_pkg.all;

package uart_pkg is
    
    constant C_BAUD_RATE        : real      := 921_600.0;
    constant C_OVERSAMPLE_MULT  : integer   := 16;
    constant C_ACC_WIDTH        : integer   := 32;
    constant C_OVERSAMP_RATE    : real      := C_BAUD_RATE * real(C_OVERSAMPLE_MULT);
    constant C_STEP_SIZE        : integer   := integer(ceil(C_OVERSAMP_RATE / C_CLK_FREQ * (2.0**C_ACC_WIDTH)));    -- oversample step size
    constant C_STEP_SIZE_STD    : integer   := integer(ceil(C_BAUD_RATE / C_CLK_FREQ * (2.0**C_ACC_WIDTH)));        -- regular step size (no oversampling)
    constant C_CMD_NUM_BYTES    : integer   := 8;
    constant C_CMD_TOP_BIT      : integer   := C_CMD_NUM_BYTES * 8;
    
    constant C_SYNC_BYTE        : std_logic_vector(7 downto 0) := x"AA";

    constant C_READ_CMD         : std_logic_vector(7 downto 0) := x"01";
    constant C_WRITE_CMD        : std_logic_vector(7 downto 0) := x"02";
    
    type t_uart_parser_states is (ST_IDLE, ST_CMD, ST_ADDR, ST_DATA, ST_CHECK, ST_EXE);
    type t_uart_fsm_states    is (ST_IDLE, ST_START, ST_RUN, ST_STOP, ST_DONE);
    type t_uart_serial_states is (ST_IDLE, ST_SERIALIZE, ST_DONE);
    subtype t_cmd_frame is std_logic_vector(C_CMD_TOP_BIT - 1 downto 0);

    function calc_crc8_56bit(data_in : std_logic_vector(55 downto 0)) return std_logic_vector;

end package uart_pkg;

package body uart_pkg is
    
    function calc_crc8_56bit(data_in : std_logic_vector(55 downto 0)) return std_logic_vector is
        variable v_crc  : std_logic_vector(7 downto 0) := x"00"; -- Initial value
        variable v_poly : std_logic_vector(7 downto 0) := x"07"; -- Polynomial
        variable v_inv  : std_logic;
    begin
        -- Iterate through all 56 bits, starting from the MSB down to the LSB
        for i in data_in'high downto data_in'low loop
            
            -- Check if the MSB of the current CRC XOR'd with the incoming bit is 1
            v_inv := v_crc(7) xor data_in(i);
            
            -- Shift the CRC register left by 1
            v_crc(7 downto 1) := v_crc(6 downto 0);
            v_crc(0) := '0';
            
            -- If the XOR result was 1, apply the polynomial mask
            if v_inv = '1' then
                v_crc := v_crc xor v_poly;
            end if;
            
        end loop;
        
        return v_crc;
    end function calc_crc8_56bit;
    
end package body uart_pkg;