entity uart_tx is
    port (
        -- System
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- Input from the register file 
        i_vld       : in std_logic;
        i_data      : in std_logic_vector(31 downto 0);
        i_addr      : in std_logic_vector(7 downto 0);

        -- Output 
        o_tx        : out std_logic;
    );
end entity uart_tx;

architecture rtl of uart_tx is
    
begin

    
    
end architecture rtl;