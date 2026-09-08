library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.uart_pkg.all;

entity uart_tx is
    port (
        -- System
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        -- Input from the register file 
        i_vld       : in std_logic;
        i_byte      : in std_logic_vector(7 downto 0);

        -- Output 
        o_tx        : out std_logic;
        o_done      : out std_logic;
        o_rdy       : out std_logic
    );
end entity uart_tx;

architecture rtl of uart_tx is

    constant C_STEP_INC : unsigned(C_ACC_WIDTH-1 downto 0) := to_unsigned(C_STEP_SIZE_STD, C_ACC_WIDTH);

    -- Baud rate generation 
    signal r_rx_acc     : unsigned(C_ACC_WIDTH downto 0);
    signal r_ovf_flag   : std_logic;
    signal r_baud_tick  : std_logic;

    -- Counters
    signal r_bit_cnt    : natural range 0 to 10;

    -- Serialize data 
    signal r_frame      : std_logic_vector(9 downto 0);

    -- Latched input
    signal r_vld        : std_logic;
    signal r_byte       : std_logic_vector(7 downto 0);

    -- FSM
    signal r_cur_state  : t_uart_fsm_states;
    signal w_nxt_state  : t_uart_fsm_states;

begin

    o_done  <= '1' when r_cur_state = ST_DONE else '0';
    o_rdy <= '1' when r_cur_state = ST_IDLE and
                      r_vld = '0' and
                      i_vld = '0'
                else '0';
    
    fsm : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                r_cur_state <= ST_IDLE;
            else
                r_cur_state <= w_nxt_state;

                if r_cur_state = ST_IDLE then 
                    r_frame <= '1' & r_byte & '0'; -- stop bit (1) + [7:0] + start bit (0)
                end if;
            end if;
        end if;
    end process fsm;

    fsm_nxt : process(all)
    begin 
        w_nxt_state <= r_cur_state;

        case r_cur_state is 
            when ST_IDLE => 
                if r_vld = '1' then 
                    w_nxt_state <= ST_RUN;
                end if;

            when ST_RUN => 
                if r_bit_cnt = 10 then 
                    w_nxt_state <= ST_DONE;
                end if;
            
            when ST_DONE =>
                w_nxt_state <= ST_IDLE;

            when others => 
                w_nxt_state <= ST_IDLE;
        end case; 
    end process fsm_nxt;

    baud_rate_gen : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                r_rx_acc        <= (others => '0');
                r_ovf_flag      <= '0';
                r_baud_tick     <= '0';
            elsif r_cur_state = ST_RUN then 
                r_rx_acc        <= r_rx_acc + C_STEP_INC;
                r_ovf_flag      <= r_rx_acc(32);

                if (r_ovf_flag = '1' and r_rx_acc(32) = '0') or 
                   (r_ovf_flag = '0' and r_rx_acc(32) = '1') then 
                    r_baud_tick <= '1';
                else
                    r_baud_tick <= '0';
                end if;
            else 
                r_rx_acc    <= (others => '0');
                r_ovf_flag  <= '0';
                r_baud_tick <= '0';
            end if;
        end if;
    end process baud_rate_gen;

    serialize : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                o_tx      <= '1';
                r_bit_cnt <= 0;
            elsif r_cur_state = ST_RUN then 
                if r_baud_tick = '1' then 
                    o_tx      <= r_frame(r_bit_cnt);
                    r_bit_cnt <= r_bit_cnt + 1;
                end if;
            else 
                o_tx      <= '1';
                r_bit_cnt <= 0;
            end if;
        end if;
    end process serialize;

    latch : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_vld   <= '0';
            elsif i_vld = '1' then 
                r_vld   <= '1';
                r_byte  <= i_byte;
            else 
                r_vld   <= '0';
            end if;
        end if;
    end process latch; 
    
end architecture rtl;