library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;

entity svpwm is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        -- Input from inverse Park transformation (volts in alpha/beta direction)
        i_vld           : in std_logic;
        i_ab_volt       : in t_ab_volt;

        -- Output PWMs 
        o_vld           : out std_logic;
        o_abc_cnts      : out t_pwm_cnt_phase
    );
end entity svpwm;

architecture rtl of svpwm is
    type t_svpwm_state is (
        ST_IDLE,
        ST_XYZ,
        ST_
    );

    signal r_state      : t_svpwm_state;
    signal w_nxt_state  : t_svpwm_state;

    signal r_x_ph_vlt   : t_phase_voltage;
    signal r_y_ph_vlt   : t_phase_voltage;
    signal r_z_ph_vlt   : t_phase_voltage;
begin
    fsm : process(i_clk)
    begin 
        if i_rst = '1' then 
            r_state <= ST_IDLE;
        else
            r_state <= w_nxt_state;
        end if;
    end process fsm;

    fsm_advance : process(all)
    begin 

    end process fsm_advance;
    
end architecture rtl;