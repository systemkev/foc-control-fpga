library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

use work.spi_pkg.all; 

entity adc_spi_master is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;
        i_start     : in std_logic;

        -- Serial SPI comm
        i_miso      : in std_logic;
        o_mosi      : out std_logic;
        o_n_cs      : out std_logic;
        o_sclk      : out std_logic;

        -- Output
        o_vld       : out std_logic;
        o_raw_angle : out unsigned(13 downto 0)
    );
end entity adc_spi_master;

architecture rtl of adc_spi_master is

    -- Sequence Tracking
    type t_spi_step is (CMD_PHASE, READ_PHASE, CLEAR_PHASE);
    signal r_step : t_spi_step;

    -- FSM
    signal r_cur_state  : t_spi_fsm; 
    signal w_nxt_state  : t_spi_fsm;

    -- Status signals
    signal w_spi_done   : std_logic;
    signal r_wait_done  : std_logic;

    -- Frame 
    signal r_miso_shft_reg  : std_logic_vector(15 downto 0);
    signal r_mosi_shft_reg  : std_logic_vector(15 downto 0);

    -- Counter
    signal r_timer_cnt  : natural range 0 to C_SCLK_SWTCH_CNT;
    signal r_wait_cnt   : natural range 0 to C_WAIT_MAX;
    signal r_sclk       : std_logic;
    signal r_rise_edge  : std_logic;
    signal r_fall_edge  : std_logic;
    signal r_bit_cnt    : natural range 0 to 15;
begin
    
    o_sclk  <= r_sclk;
    o_mosi  <= r_mosi_shft_reg(15);
    
    -- Assign CSn: Drives low during SPI transaction, high otherwise
    o_n_cs  <= '0' when r_cur_state = ST_SPI else '1';

    w_spi_done  <= '1' when r_cur_state = ST_SPI and r_bit_cnt = 16 and r_timer_cnt = 1 else '0';

    fsm : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_cur_state <= ST_IDLE;
                r_step      <= CMD_PHASE;
            else 
                r_cur_state <= w_nxt_state;
                
                -- Track which frame of the sequence we are currently executing
                if r_cur_state = ST_IDLE and i_start = '1' then
                    r_step <= CMD_PHASE;
                elsif r_cur_state = ST_CHECK then
                    if r_step = CMD_PHASE then
                        r_step <= READ_PHASE;
                    elsif r_step = READ_PHASE and r_miso_shft_reg(14) = '1' then
                        r_step <= CLEAR_PHASE;
                    elsif r_step = CLEAR_PHASE then
                        r_step <= CMD_PHASE;
                    end if;
                end if;
            end if;
        end if;
    end process fsm;

    fsm_nxt : process(all)
    begin 
        w_nxt_state <= r_cur_state;

        case r_cur_state is 
            when ST_IDLE => 
                if i_start = '1' then 
                    w_nxt_state <= ST_WAIT;
                end if;

            when ST_WAIT => 
                if r_wait_done = '1' then 
                    w_nxt_state <= ST_SPI;
                end if;

            when ST_SPI => 
                if w_spi_done = '1' then 
                    w_nxt_state <= ST_CHECK;
                end if;
                
            when ST_CHECK =>
                -- Exit loop only if we completed a read phase with no errors
                if r_step = READ_PHASE and r_miso_shft_reg(14) = '0' then
                    w_nxt_state <= ST_DONE;
                else
                    w_nxt_state <= ST_WAIT; 
                end if;

            when ST_DONE => 
                w_nxt_state     <= ST_IDLE;

            when others =>
                w_nxt_state     <= ST_IDLE;
        end case;
    end process fsm_nxt;
    
    sclk : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            r_rise_edge <= '0';
            r_fall_edge <= '0';

            if r_cur_state = ST_SPI then 
                if r_timer_cnt = 0 then 
                    r_timer_cnt <= C_SCLK_SWTCH_CNT - 1;
                    r_sclk <= not r_sclk;
                    if r_sclk = '0' then 
                        r_rise_edge <= '1'; 
                    else
                        r_fall_edge <= '1'; 
                    end if;
                else 
                    r_timer_cnt <= r_timer_cnt - 1;
                end if;
            else 
                r_sclk      <= '0'; 
                r_timer_cnt <= C_SCLK_SWTCH_CNT - 1;
            end if;
        end if; 
    end process sclk;

    wait_timer : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' or r_cur_state /= ST_WAIT then
                r_wait_cnt  <= C_WAIT_MAX;
                r_wait_done <= '0';
            elsif r_cur_state = ST_WAIT then
                if r_wait_cnt = 0 then
                    r_wait_done <= '1';
                else
                    r_wait_cnt <= r_wait_cnt - 1;
                end if;
            else 
                r_wait_cnt <= C_WAIT_MAX;
            end if;
        end if;
    end process wait_timer;

    miso : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_miso_shft_reg <= (others => '0');
            else 
                if r_cur_state = ST_SPI then 
                    if r_rise_edge = '1' then 
                        r_miso_shft_reg <= r_miso_shft_reg(14 downto 0) & i_miso;
                    end if; 
                end if;
            end if;
        end if;
    end process miso;

    mosi : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_mosi_shft_reg <= C_READ_ANGLE_CMD;
                r_bit_cnt       <= 0;
            else 
                if r_cur_state = ST_SPI then 
                    if r_fall_edge = '1' then 
                        r_mosi_shft_reg <= r_mosi_shft_reg(14 downto 0) & '0';
                        r_bit_cnt       <= r_bit_cnt + 1;
                    end if; 
                elsif r_cur_state = ST_IDLE then
                    r_mosi_shft_reg <= C_READ_ANGLE_CMD;
                    r_bit_cnt       <= 0;
                elsif r_cur_state = ST_CHECK then 
                    r_bit_cnt <= 0;
                    -- Pre-load the command for the upcoming frame
                    if r_step = CMD_PHASE then
                        r_mosi_shft_reg <= C_READ_ANGLE_CMD;
                    elsif r_step = READ_PHASE then
                        if r_miso_shft_reg(14) = '1' then
                            r_mosi_shft_reg <= C_CLR_ERR_FL_CMD;
                        else
                            r_mosi_shft_reg <= C_READ_ANGLE_CMD;
                        end if;
                    elsif r_step = CLEAR_PHASE then
                        r_mosi_shft_reg <= C_READ_ANGLE_CMD;
                    end if;
                end if;
            end if;
        end if;
    end process mosi;

    -- Output capture process
    out_regs : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_vld <= '0';
                o_raw_angle <= (others => '0');
            else
                if r_cur_state = ST_DONE then
                    o_vld <= '1';
                    o_raw_angle <= unsigned(r_miso_shft_reg(13 downto 0));
                else
                    o_vld <= '0';
                end if;
            end if;
        end if;
    end process out_regs;

end architecture rtl;