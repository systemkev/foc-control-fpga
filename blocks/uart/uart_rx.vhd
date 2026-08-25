library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.uart_pkg.all;

entity uart_rx is
    port (
        i_clk       : in  std_logic;
        i_rst       : in  std_logic;
        i_rx        : in  std_logic;

        o_data      : out std_logic_vector(7 downto 0);
        o_vld       : out std_logic;

        -- Debug port
        o_baud      : out std_logic;
        o_bit       : out integer
    );
end entity uart_rx; 

architecture rtl of uart_rx is

    constant C_STEP_INC     : unsigned(C_ACC_WIDTH-1 downto 0) := to_unsigned(C_STEP_SIZE, C_ACC_WIDTH);

    signal r_rx_shft_reg    : std_logic_vector(7 downto 0);
    signal r_rx_bit_cnt     : natural range 0 to 10  := 0;  
    signal r_rx_tick_cnt    : natural range 0 to 16 := 0;   
    signal r_last_rx        : std_logic;

    -- Baud rate generation 
    signal r_rx_acc         : unsigned(C_ACC_WIDTH downto 0);
    signal r_ovf_flag       : std_logic;
    signal r_baud_tick      : std_logic;

    -- FSM
    signal r_cur_state      : t_uart_fsm_states;
    signal w_nxt_state      : t_uart_fsm_states;

    -- Status signals
    signal r_start_pass     : std_logic;
    signal r_stop_pass      : std_logic;

    --Synchronization 
    signal r_rx_ff          : std_logic;
    signal r_rx_sync        : std_logic;    
begin 

    o_baud <= r_baud_tick;
    o_bit  <= r_rx_bit_cnt;

    sync_proc : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_rx_ff   <= '1';
                r_rx_sync <= '1';
            else
                r_rx_ff   <= i_rx;
                r_rx_sync <= r_rx_ff;
            end if;
        end if;
    end process sync_proc;

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
                if r_rx_sync = '0' then 
                    w_nxt_state <= ST_START;
                end if;

            when ST_START => 
                -- Wait for the baud tick to evaluate the end of the phase
                if r_baud_tick = '1' and r_rx_tick_cnt = 15 then 
                    if r_start_pass = '1' then 
                        w_nxt_state <= ST_RUN;
                    else
                        w_nxt_state <= ST_IDLE;
                    end if;
                end if;
            
            when ST_RUN => 
                if r_baud_tick = '1' and r_rx_tick_cnt = 15 and r_rx_bit_cnt = 8 then 
                    w_nxt_state <= ST_STOP;
                end if;

            when ST_STOP =>
                -- Now it will actually wait for tick 15 of the STOP state!
                if r_baud_tick = '1' and r_rx_tick_cnt = 15 then 
                    if r_stop_pass = '1' then 
                        w_nxt_state <= ST_DONE;
                    else
                        w_nxt_state <= ST_IDLE;
                    end if;
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
                r_last_rx       <= '1';
                r_ovf_flag      <= '0';
                r_baud_tick     <= '0';
            elsif r_cur_state = ST_START or r_cur_state = ST_RUN or r_cur_state = ST_STOP then 
                r_rx_acc        <= r_rx_acc + C_STEP_INC;
                r_ovf_flag      <= r_rx_acc(32);
                r_last_rx       <= r_rx_sync;    -- used to detect falling edge (start of frame)

                if (r_ovf_flag = '1' and r_rx_acc(32) = '0') or 
                   (r_ovf_flag = '0' and r_rx_acc(32) = '1') then 
                    r_baud_tick <= '1';
                else
                    r_baud_tick <= '0';
                end if;
            else 
                r_rx_acc        <= (others => '0');
            end if;
        end if;
    end process baud_rate_gen;

    receive : process(i_clk)
        variable v_sample   : std_logic_vector(2 downto 0);
        variable v_majority : std_logic; 
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_rx_shft_reg <= (others => '0');
                r_rx_bit_cnt  <= 0;
                r_rx_tick_cnt <= 0;
                r_start_pass  <= '0';
                r_stop_pass   <= '0';
            elsif r_cur_state = ST_START or r_cur_state = ST_RUN or r_cur_state = ST_STOP then  
                if r_baud_tick = '1' then 
                    if r_rx_tick_cnt = 15 then 
                        r_rx_tick_cnt <= 0;
                        r_rx_bit_cnt  <= r_rx_bit_cnt + 1;
                    else 
                        r_rx_tick_cnt <= r_rx_tick_cnt + 1;
                    end if;

                    if r_rx_tick_cnt = 7 then 
                        v_sample(0) := r_rx_sync;
                    elsif r_rx_tick_cnt = 8 then 
                        v_sample(1) := r_rx_sync;
                    elsif r_rx_tick_cnt = 9 then 
                        v_sample(2) := r_rx_sync;
                    elsif r_rx_tick_cnt = 10 then  
                        v_majority  := (v_sample(0) and v_sample(1)) or
                                       (v_sample(1) and v_sample(2)) or
                                       (v_sample(0) and v_sample(2));
                    end if;

                    if r_cur_state = ST_START then 
                        if v_majority = '0' and r_rx_tick_cnt = 10 then 
                            r_start_pass <= '1';
                        end if;
                    elsif r_cur_state = ST_RUN then 
                        if r_rx_tick_cnt = 15 and r_rx_bit_cnt >= 1 and r_rx_bit_cnt <= 8 then 
                            r_rx_shft_reg(r_rx_bit_cnt-1) <= v_majority;
                        end if;
                    elsif r_cur_state = ST_STOP then   
                        if v_majority = '1' and r_rx_tick_cnt = 10 then 
                            r_stop_pass <= '1';
                        end if;
                    end if;
                end if;
            elsif r_cur_state = ST_IDLE then 
                r_rx_shft_reg <= (others => '0');
                r_rx_bit_cnt  <= 0;
                r_rx_tick_cnt <= 0;
                r_start_pass  <= '0';
                r_stop_pass   <= '0';
            end if;
        end if;
    end process receive;

    output : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_vld  <= '0';
                o_data <= (others => '0');
            elsif r_cur_state = ST_DONE then 
                o_vld  <= '1';
                o_data <= r_rx_shft_reg;
            else
                o_vld  <= '0';
            end if;
        end if;
    end process output;

end architecture rtl;