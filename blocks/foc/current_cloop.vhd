library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use work.common_pkg.all;
use work.cloop_pkg.all;

entity current_cloop is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        -- Inputs
        i_vld           : in std_logic;
        i_target_curr   : in t_dq_phase;
        i_actual_curr   : in t_dq_phase;

        -- Outputs 
        o_vld           : out std_logic;
        o_ph_voltage    : out t_dq_volt
    );
end entity current_cloop;

architecture rtl of current_cloop is
    signal r_vld           : std_logic_vector(3 downto 0);

    signal r_target_curr   : t_dq_phase;
    signal r_actual_curr   : t_dq_phase;
    signal r_error_curr    : t_dq_phase;
    
    signal r_gain_prop     : t_dq_volt;
    signal r_gain_intg     : t_dq_volt;
    signal r_intg_accum    : t_dq_volt;
begin
    
    pipeline_ctrl : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_vld <= (others => '0');
            else
                r_vld <= r_vld(2 downto 0) & i_vld;
            end if;
        end if;
    end process pipeline_ctrl;

    latch : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_target_curr <= (d => (others => '0'), q => (others => '0'));
                r_actual_curr <= (d => (others => '0'), q => (others => '0'));
            elsif i_vld = '1' then 
                r_target_curr <= i_target_curr;
                r_actual_curr <= i_actual_curr;
            end if;
        end if;
    end process latch;

    error : process(i_clk) 
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_error_curr <= (d => (others => '0'), q => (others => '0'));
            elsif r_vld(0) = '1' then 
                r_error_curr.d <= resize(r_target_curr.d - r_actual_curr.d, r_error_curr.d);
                r_error_curr.q <= resize(r_target_curr.q - r_actual_curr.q, r_error_curr.q);
            end if;
        end if;
    end process error;

    gains : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_gain_prop <= (d => (others => '0'), q => (others => '0'));
                r_gain_intg <= (d => (others => '0'), q => (others => '0'));
            elsif r_vld(1) = '1' then 
                r_gain_prop.d <= resize(r_error_curr.d * C_KP_PROP_GAIN, r_gain_prop.d);
                r_gain_prop.q <= resize(r_error_curr.q * C_KP_PROP_GAIN, r_gain_prop.q);
                r_gain_intg.d <= resize(r_error_curr.d * C_KI_INTG_GAIN, r_gain_intg.d);
                r_gain_intg.q <= resize(r_error_curr.q * C_KI_INTG_GAIN, r_gain_intg.q);
            end if;
        end if;
    end process gains;

    accumulator : process(i_clk)
        variable v_intg_accum : t_dq_volt;
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_intg_accum <= (d => (others => '0'), q => (others => '0'));
            elsif r_vld(2) = '1' then 
                v_intg_accum.d := resize(r_gain_intg.d + r_intg_accum.d, v_intg_accum.d);
                v_intg_accum.q := resize(r_gain_intg.q + r_intg_accum.q, v_intg_accum.q);

                if v_intg_accum.d > C_WINDUP_MAX then 
                    r_intg_accum.d <= C_WINDUP_MAX;
                elsif v_intg_accum.d < -C_WINDUP_MAX then 
                    r_intg_accum.d <= resize(-C_WINDUP_MAX, r_intg_accum.d);
                else
                    r_intg_accum.d <= v_intg_accum.d;
                end if;

                if v_intg_accum.q > C_WINDUP_MAX then 
                    r_intg_accum.q <= C_WINDUP_MAX;
                elsif v_intg_accum.q < -C_WINDUP_MAX then 
                    r_intg_accum.q <= resize(-C_WINDUP_MAX, r_intg_accum.q);
                else
                    r_intg_accum.q <= v_intg_accum.q;
                end if;
            end if;
        end if;
    end process accumulator;

    output : process(i_clk)
        variable v_output_volt : t_dq_volt;
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                o_ph_voltage <= (d => (others => '0'), q => (others => '0'));
                o_vld <= '0';
            else 
                o_vld <= r_vld(3);
                
                if r_vld(3) = '1' then 
                    v_output_volt.d := r_intg_accum.d + r_gain_prop.d;
                    v_output_volt.q := r_intg_accum.q + r_gain_prop.q;

                    if v_output_volt.d > C_WINDUP_MAX then 
                        o_ph_voltage.d <= C_WINDUP_MAX;
                    elsif v_output_volt.d < -C_WINDUP_MAX then 
                        o_ph_voltage.d <= resize(-C_WINDUP_MAX, o_ph_voltage.d);
                    else
                        o_ph_voltage.d <= v_output_volt.d;
                    end if;

                    if v_output_volt.q > C_WINDUP_MAX then 
                        o_ph_voltage.q <= C_WINDUP_MAX;
                    elsif v_output_volt.q < -C_WINDUP_MAX then 
                        o_ph_voltage.q <= resize(-C_WINDUP_MAX, o_ph_voltage.q);
                    else
                        o_ph_voltage.q <= v_output_volt.q;
                    end if;
                end if;
            end if;
        end if;
    end process output;

end architecture rtl;