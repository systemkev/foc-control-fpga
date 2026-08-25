library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

entity i2c_master is
    generic (
        C_SYS_CLK_FREQ : integer := 100_000_000; -- System clock frequency in Hz
        C_I2C_CLK_FREQ : integer := 100_000      -- Target I2C bus frequency in Hz
    );
    port (
        -- System Clock and Reset
        i_clk       : in    std_logic;
        i_rst       : in    std_logic;

        -- User Logic Interface
        i_en        : in    std_logic;                    -- Pulse high to start transaction
        i_addr      : in    std_logic_vector(6 downto 0); -- 7-bit slave address
        i_rw        : in    std_logic;                    -- '0' for write, '1' for read
        i_data_in   : in    std_logic_vector(7 downto 0); -- Data to transmit (write)
        o_data_out  : out   std_logic_vector(7 downto 0); -- Data received (read)
        o_busy      : out   std_logic;                    -- High while transaction is active
        o_ack_error : out   std_logic;                    -- High if slave NACKs the address/data

        -- I2C Physical Bus Interface
        io_sda      : inout std_logic;                    -- Serial Data
        io_scl      : inout std_logic                     -- Serial Clock
    );
end entity i2c_master;

architecture rtl of i2c_master is
    type t_i2c_fsm is (
        ST_IDLE,
        ST_START,
        ST_ADDRESS,
        ST_ACK1,
        ST_DATA,
        ST_ACK2, 
        ST_STOP
    );

    signal r_cur_state  : t_i2c_fsm;
    signal w_nxt_state  : t_i2c_fsm;

    signal r_addr    : std_logic_vector(6 downto 0);
    signal r_rw      : std_logic;
    signal r_data_in : std_logic_vector(7 downto 0);

    signal r_bit_cnt : natural range 0 to 7;

    signal r_sda : std_logic;
    signal r_scl : std_logic;

    signal r_ack1 : std_logic;
    signal r_ack2 : std_logic;

    signal r_tx_frame : std_logic_vector(7 downto 0);
    signal r_wr_frame : std_logic_vector(7 downto 0);
    signal r_rd_frame : std_logic_vector(7 downto 0);
begin
    
    io_sda <= 'Z' when r_sda = '1' else '0';
    io_scl <= 'Z' when r_scl = '1' else '0';

    fsm : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_cur_state <= ST_IDLE;
            else 
                r_cur_state <= w_nxt_state;

                if r_cur_state = ST_IDLE then 
                    r_tx_frame <= i_addr & i_rw;
                    r_wr_frame <= i_data_in;
                end if;
            end if;
        end if;
    end process fsm; 

    fsm_nxt : process(all)
    begin 
        w_nxt_state <= r_cur_state;

        case r_cur_state is 
            when ST_IDLE => 
                if i_en = '1' then 
                    w_nxt_state <= ST_START;
                end if;

            when ST_START => 
                if i2c_tick = 3 then 
                    w_nxt_state <= ST_ADDRESS;
                end if;

            when ST_ADDRESS => 
                if r_bit_cnt = 7 then 
                    w_nxt_state <= ST_ACK1;
                end if;

            when ST_ACK1 => 
                if r_ack1 = '1' then 
                    w_nxt_state <= ST_DATA;
                end if;

            when ST_DATA => 


            when ST_ACK2 => 

            when ST_STOP => 
                w_nxt_state <= ST_IDLE;

            when others => 
                w_nxt_state <= ST_IDLE;
        end case; 
    end process fsm_nxt;

    drive : process(i_clk) 
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                --
            else 
                case r_cur_state is 
                    when ST_START => 
                        if i2c_tick = 0 then 
                            r_sda     <= '0';
                            r_bit_cnt <= 0;
                            r_frame   <= 
                        elsif i2c_tick = 2 then 
                            r_scl     <= '0';
                        end if;
                    
                    when ST_ADDRESS => 
                        if i2c_tick = 0 then 
                            r_sda     <= r_tx_frame(7-r_bit_cnt);
                        elsif i2c_tick = 1 then 
                            r_scl     <= '1';
                        elsif i2c_tick = 2 then 
                            r_bit_cnt <= r_bit_cnt + 1;
                        elsif i2c_tick = 3 then 
                            r_scl     <= '0';
                        end if;

                    when ST_ACK1 => 
                        if i2c_tick = 2 then 
                            r_bit_cnt   <= 0;
                            if io_sda = '1' then 
                                r_ack1  <= '1';
                            end if;
                        end if;

                    when ST_DATA => 
                        -- Read 
                        if r_rw = '1' then 
                            
                        -- Write
                        else 
                        end if;

                    when ST_ACK2 => 

                    when ST_STOP => 
                        w_nxt_state <= ST_IDLE;

                    when others => 
                        w_nxt_state <= ST_IDLE;
                end case; 
            end if;
        end if;
    end process drive;


    
end architecture rtl;