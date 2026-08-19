--------------------------------------------------------------------------------
-- File:        adc_spi_master.vhd
-- Author:      Kevin Toledo Fernandez
-- Github:      systemkev
-- Website:     systemkev.github.io
-- Date:        8/16/2026
-- Description: 
--------------------------------------------------------------------------------

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

    w_spi_done  <= '1' when r_cur_state = ST_SPI and r_bit_cnt = 16 and r_timer_cnt = 1 else '0';

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
                if i_start = '1' then 
                    w_nxt_state <= ST_WAIT;
                end if;

            when ST_WAIT => 
                if r_wait_done = '1' then 
                    w_nxt_state <= ST_SPI;
                end if;

            when ST_SPI => 
                if w_spi_done = '1' then 
                    w_nxt_state <= ST_DONE;
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
            -- Default assignments (auto-clears edge flags after 1 clock cycle)
            r_rise_edge <= '0';
            r_fall_edge <= '0';

            if r_cur_state = ST_SPI then 
                
                if r_timer_cnt = 0 then 
                    -- reload the counter
                    r_timer_cnt <= C_SCLK_SWTCH_CNT - 1;
                    
                    -- toggle the clock
                    r_sclk <= not r_sclk;
                    
                    -- set the appropriate edge flag based on the current state of r_sclk
                    if r_sclk = '0' then 
                        r_rise_edge <= '1'; -- transitioning 0 -> 1
                    else
                        r_fall_edge <= '1'; -- transitioning 1 -> 0
                    end if;
                    
                else 
                    -- count down
                    r_timer_cnt <= r_timer_cnt - 1;
                end if;
                
            else 
                -- idle
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
            else 
                if r_cur_state = ST_SPI then 
                    if r_fall_edge = '1' then 
                        r_mosi_shft_reg <= r_mosi_shft_reg(14 downto 0) & '0';
                        r_bit_cnt       <= r_bit_cnt + 1;
                    end if; 
                elsif w_nxt_state = ST_WAIT then 
                    r_mosi_shft_reg <= C_READ_ANGLE_CMD;
                else 
                    r_bit_cnt <= 0;
                end if;
            end if;
        end if;
    end process mosi;

end architecture rtl;