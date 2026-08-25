library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.reg_pkg.all;

entity reg_file is
    port (
        -- System
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- input from UART parser 
        i_vld       : in std_logic;
        i_rw        : in std_logic;
        i_data      : in std_logic_vector(31 downto 0);
        i_addr      : in std_logic_vector(7 downto 0);

        -- output to UART transmitter
        o_vld       : out std_logic;
        o_data      : out std_logic_vector(31 downto 0);
        o_addr      : out std_logic_vector(7 downto 0)
    );
end entity reg_file;

architecture rtl of reg_file is 
    type t_reg_file is array(0 to C_NUM_REGISTERS-1) of std_logic_vector(31 downto 0);
    
    signal r_regs : t_reg_file;

    signal w_addr : integer range 0 to 255;
begin 

    w_addr <= to_integer(unsigned(i_addr));

    reg : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                o_vld       <= '0';
                o_data      <= (others => '0');
                o_addr      <= (others => '0');
                r_regs      <= (others => (others => '0')); 
                
            elsif i_vld = '1' then 
                
                -- prevent out-of-bounds access
                if w_addr < C_NUM_REGISTERS then 
                    
                    if i_rw = '1' then 
                        -- write operation
                        r_regs(w_addr) <= i_data;
                        o_vld          <= '0'; 
                    else 
                        -- read operation
                        o_vld   <= '1';
                        o_data  <= r_regs(w_addr);
                        o_addr  <= i_addr;
                    end if;
                    
                else
                    -- safe fallback if the UART requests an invalid address
                    o_vld  <= '1'; 
                    o_data <= (others => '0'); 
                    o_addr <= i_addr;
                end if;
                
            else 
                -- clear outputs when no valid transaction is occurring
                o_vld   <= '0';
                o_data  <= (others => '0');
                o_addr  <= (others => '0');
            end if;
        end if;
    end process reg;

end architecture rtl;