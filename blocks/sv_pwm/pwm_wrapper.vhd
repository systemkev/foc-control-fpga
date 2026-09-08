--------------------------------------------------------------------------------
-- File:        pwm_3ph_driver.vhd
-- Description: 3-Phase PWM wrapper for SimpleFOC Shield v2.0.4.
--              Converts integer PWM counts to independent PWM signals and
--              provides a master enable/disable gate. No dead-time injection 
--              is included, as the L6234D driver handles it hardware-side.
--------------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library work;
use work.common_pkg.all;
use work.math_pkg.all;

entity pwm_3ph_driver is
    port (
        -- System
        i_clk       : in std_logic;
        i_rst       : in std_logic;
        
        i_en        : in std_logic;

        -- duty cycle counts from SVPWM module
        i_abc_cnts_A : in integer range 0 to C_CLKS_IN_PWM_PRD;
        i_abc_cnts_B : in integer range 0 to C_CLKS_IN_PWM_PRD;
        i_abc_cnts_C : in integer range 0 to C_CLKS_IN_PWM_PRD;

        -- physical outputs to SimpleFOC Shield
        o_pwm_a     : out std_logic;
        o_pwm_b     : out std_logic;
        o_pwm_c     : out std_logic;
        o_en        : out std_logic;

        -- Synchronization pulse for ADC triggering (goes high at counter = 0)
        o_sync_tick : out std_logic
    );
end entity pwm_3ph_driver;

architecture rtl of pwm_3ph_driver is

    -- determine the bit width needed for the pwm_gen modules
    constant C_PWM_BITS : integer := log_2_ceil(C_CLKS_IN_PWM_PRD + 1);

    -- internal unsigned duty cycle signals
    signal w_duty_a : unsigned(C_PWM_BITS-1 downto 0);
    signal w_duty_b : unsigned(C_PWM_BITS-1 downto 0);
    signal w_duty_c : unsigned(C_PWM_BITS-1 downto 0);

    -- internal raw PWM signals from the generators
    signal w_pwm_a : std_logic;
    signal w_pwm_b : std_logic;
    signal w_pwm_c : std_logic;

    -- sync tick from Phase A
    signal w_sync_tick : std_logic;

begin

    w_duty_a <= to_unsigned(i_abc_cnts_A, C_PWM_BITS);
    w_duty_b <= to_unsigned(i_abc_cnts_B, C_PWM_BITS);
    w_duty_c <= to_unsigned(i_abc_cnts_C, C_PWM_BITS);

    ----------------------------------------------------------------------------
    -- PWM Generators
    ----------------------------------------------------------------------------
    gen_pwm_a : entity work.pwm_gen
        port map (
            i_clk                => i_clk,
            i_rst                => i_rst,
            i_duty               => w_duty_a,
            o_pwm                => w_pwm_a,
            o_pwm_is_first_cycle => w_sync_tick 
        );

    gen_pwm_b : entity work.pwm_gen
        port map (
            i_clk                => i_clk,
            i_rst                => i_rst,
            i_duty               => w_duty_b,
            o_pwm                => w_pwm_b,
            o_pwm_is_first_cycle => open -- Ignored, perfectly synced with Phase A
        );

    gen_pwm_c : entity work.pwm_gen
        port map (
            i_clk                => i_clk,
            i_rst                => i_rst,
            i_duty               => w_duty_c,
            o_pwm                => w_pwm_c,
            o_pwm_is_first_cycle => open -- Ignored, perfectly synced with Phase A
        );

    ----------------------------------------------------------------------------
    -- Output Gating & Assignment
    ----------------------------------------------------------------------------
    
    o_en <= i_en;
    
    o_sync_tick <= w_sync_tick;

    o_pwm_a <= w_pwm_a when i_en = '1' else '0';
    o_pwm_b <= w_pwm_b when i_en = '1' else '0';
    o_pwm_c <= w_pwm_c when i_en = '1' else '0';

end architecture rtl;
