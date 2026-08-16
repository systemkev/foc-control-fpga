entity ff_sync is
    generic (
        C_NUM_STAGES    : integer := 2;
        C_NUM_BITS      : integer := 1
    );
    port (
        i_bit_unsync    : in  std_logic_vector(C_NUM_BITS-1 downto 0);   
        i_rst           : in  std_logic;
        i_dest_clk      : in  std_logic; -- destination clock (domain we are synchronizing to)   

        o_bit_synced    : out std_logic_vector(C_NUM_BITS-1 downto 0)
    );
end entity ff_sync;

architecture rtl of ff_sync is
    type t_pipeline is array(0 to C_NUM_STAGES-1) of std_logic_vector(C_NUM_BITS-1 downto 0);
    signal r_pipeline : t_pipeline := (others => (others => '0'));
begin
    o_bit_synced <= r_pipeline(C_NUM_STAGES-1);

    sync : process(i_dest_clk) 
    begin 
        if rising_edge(i_dest_clk) then 
            if i_rst = '1' then 
                r_pipeline  <= (others => (others => '0'));
            else 
                r_pipeline <= i_bit_unsync & r_pipeline(0 to C_NUM_STAGES-2);
            end if;
        end if;
    end process sync;
    
end architecture rtl;