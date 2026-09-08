library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.common_pkg.all;
use work.uart_pkg.all;
use work.reg_pkg.all;

entity parser is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- valid data from UART receiver
        i_vld       : in std_logic;     
        i_byte      : in std_logic_vector(7 downto 0);

        o_vld       : out std_logic;
        o_rw        : out std_logic;
        o_data      : out std_logic_vector(31 downto 0);
        o_addr      : out std_logic_vector(7 downto 0)
    );
end entity parser;

architecture rtl of parser is
    signal r_cur_state : t_uart_parser_states;
    signal w_nxt_state : t_uart_parser_states;

    signal r_ptr  : integer range 0 to 5;
    signal r_addr : std_logic_vector(7 downto 0);
    signal r_cmd  : std_logic_vector(7 downto 0);
    signal r_data : std_logic_vector(39 downto 0);

    signal w_crc_vld : std_logic;

begin

    w_crc_vld <= '1' when calc_crc8_56bit(C_SYNC_BYTE & r_cmd & r_addr & r_data(39 downto 8)) = r_data(7 downto 0) else '0';

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

    fsm_comb : process(all)
    begin
        w_nxt_state <= r_cur_state; 

        case r_cur_state is
            when ST_IDLE => 
                if i_vld = '1' and i_byte = C_SYNC_BYTE then 
                    w_nxt_state <= ST_CMD;
                end if;

            when ST_CMD => 
                if i_vld = '1' then 
                    -- Check validity directly against i_byte 
                    if i_byte = C_READ_CMD or i_byte = C_WRITE_CMD then 
                        w_nxt_state <= ST_ADDR;
                    else 
                        w_nxt_state <= ST_IDLE; 
                    end if;
                end if;

            when ST_ADDR => 
                if i_vld = '1' then 
                    if unsigned(i_byte) <= unsigned(C_LAST_ADDR) then 
                        w_nxt_state <= ST_DATA;
                    else 
                        w_nxt_state <= ST_IDLE;
                    end if;
                end if;

            when ST_DATA =>
                if i_vld = '1' and r_ptr = 4 then 
                    w_nxt_state <= ST_CHECK;
                end if;

            when ST_CHECK => 
                if w_crc_vld = '1' then 
                    w_nxt_state <= ST_EXE;
                else 
                    w_nxt_state <= ST_IDLE;
                end if;

            when ST_EXE =>
                w_nxt_state <= ST_IDLE;
            
            when others => 
                w_nxt_state <= ST_IDLE;
        end case;
    end process fsm_comb;

    cmd : process(i_clk)
    begin
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_cmd      <= (others => '0');
            elsif r_cur_state = ST_CMD and i_vld = '1' then 
                if i_byte = C_READ_CMD then
                    r_cmd <= C_READ_CMD;
                elsif i_byte = C_WRITE_CMD then
                    r_cmd <= C_WRITE_CMD;
                end if;
            elsif r_cur_state = ST_IDLE then 
                r_cmd       <= (others => '0');
            end if;
        end if;
    end process cmd;

    addr : process(i_clk)
    begin
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_addr      <= (others => '0');
            elsif r_cur_state = ST_ADDR and i_vld = '1' then 
                if unsigned(i_byte) <= unsigned(C_LAST_ADDR) then 
                    r_addr  <= i_byte;
                end if;
            elsif r_cur_state = ST_IDLE then 
                r_addr      <= (others => '0');
            end if;
        end if;
    end process addr;

    data : process(i_clk)
    begin
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_ptr       <= 0;
                r_data      <= (others => '0');
            elsif r_cur_state = ST_DATA then 
                if i_vld = '1' then
                    r_ptr   <= r_ptr + 1;
                    r_data  <= r_data(31 downto 0) & i_byte;
                end if;
            elsif r_cur_state = ST_IDLE then 
                r_ptr       <= 0;
                r_data      <= (others => '0');
            end if;
        end if;
    end process data;

    execute : process(i_clk)
    begin
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                o_vld    <= '0';
            elsif r_cur_state = ST_EXE then 
                o_vld    <= '1';
                
                if r_cmd = C_READ_CMD then 
                    o_rw <= '0';
                else 
                    o_rw <= '1';
                end if;

                o_data   <= r_data(39 downto 8);
                o_addr   <= r_addr;
            else 
                o_vld    <= '0';
            end if;
        end if;
    end process execute; 

end architecture rtl;