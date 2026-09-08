library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.uart_pkg.all;

entity serializer is
    port (
        -- System
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- Register file
        i_vld       : in std_logic;
        i_data      : in std_logic_vector(31 downto 0);
        i_addr      : in std_logic_vector(7 downto 0);

        -- Transmitter 
        i_rdy       : in std_logic;
        o_vld       : out std_logic;
        o_byte      : out std_logic_vector(7 downto 0)
    );
end entity serializer;

architecture rtl of serializer is

    -- FSM
    signal r_cur_state  : t_uart_serial_states;
    signal w_nxt_state  : t_uart_serial_states;

    -- Counter 
    signal r_byte_cnt   : natural range 0 to 8;

    -- Latched inputs
    signal r_data   : std_logic_vector(31 downto 0);
    signal r_addr   : std_logic_vector(7 downto 0);

    -- Frame to send
    type t_frame is array(0 to 7) of std_logic_vector(7 downto 0);
    signal r_frame  : t_frame;

begin
    
    fsm : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                r_cur_state <= ST_IDLE;
            else
                r_cur_state <= w_nxt_state;
            end if;
        end if;
    end process fsm;

    fsm_nxt : process(all)
    begin 
        w_nxt_state <= r_cur_state;

        case r_cur_state is 
            when ST_IDLE => 
                if i_vld = '1' then 
                    w_nxt_state <= ST_SERIALIZE;
                end if;

            when ST_SERIALIZE =>
                if r_byte_cnt = 8 then 
                    w_nxt_state <= ST_DONE;
                end if;

            when ST_DONE => 
                w_nxt_state <= ST_IDLE;

            when others => 
                w_nxt_state <= ST_IDLE;
        end case; 
    end process fsm_nxt;

    serialize : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                o_vld <= '0';
            elsif r_cur_state = ST_SERIALIZE then 
                o_vld <= '0';
                
                if i_rdy = '1' and r_byte_cnt < 8 then
                    o_vld      <= '1';
                    o_byte     <= r_frame(r_byte_cnt);
                    r_byte_cnt <= r_byte_cnt + 1;
                end if;
            elsif r_cur_state = ST_IDLE and i_vld = '1' then 
                r_data      <= i_data;
                r_addr      <= i_addr;
                r_byte_cnt  <= 0;
                r_frame     <= (0 => C_SYNC_BYTE,
                                1 => x"00",
                                2 => i_addr,
                                3 => i_data(31 downto 24),
                                4 => i_data(23 downto 16),
                                5 => i_data(15 downto 8),
                                6 => i_data(7 downto 0),
                                7 => calc_crc8_56bit(C_SYNC_BYTE & x"00" & i_addr & i_data));
            end if;
        end if;
    end process serialize;
    
end architecture rtl;