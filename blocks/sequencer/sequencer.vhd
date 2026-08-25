library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity sequencer is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        
    );
end entity sequencer;

architecture rtl of sequencer is
    
    type t_seq_state is (
        ST_IDLE,
        
    );

begin
    
    
    
end architecture rtl;