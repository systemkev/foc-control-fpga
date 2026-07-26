library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use work.common_pkg.all;
use work.cloop_pkg.all;

entity pi_control is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;
        
        -- Input from Park transformation 
        i_vld           : in std_logic;
        i_target_curr   : in t_park_record;

        -- Inputs from encoder reading of current
        i_actual_curr   : in t_park_record;

        -- Output to SVPWM
        o_vld           : out std_logic;
        o_ph_voltage    : out t_park_volt_record;

        -- Ready for new sample 
        o_rdy           : out std_logic
    );
end entity pi_control;

architecture rtl of pi_control is
    type t_pi_controller_states is (
        ST_IDLE,
        ST_ERROR,
        ST_GAIN,
        ST_ACCUM,
        ST_OUTPUT
    );
    
    signal r_state          : t_pi_controller_states;
    signal w_nx_state       : t_pi_controller_states;

    signal r_target_curr    : t_park_record;
    signal r_actual_curr    : t_park_record;
    signal r_error_curr     : t_park_record;
    signal r_gain_prop      : t_park_volt_record;
    signal r_gain_intg      : t_park_volt_record;
    signal r_intg_accum     : t_park_volt_record;
    signal r_volt_next      : t_park_volt_record;
begin
    o_rdy <= '1' when r_state = ST_IDLE else '0';

    fsm : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_state <= ST_IDLE;
            else
                r_state <= w_nx_state;
            end if;
        end if;
    end process fsm;

    fsm_advance : process(all)
    begin 
        w_nx_state <= r_state;

        case r_state is
            when ST_IDLE => 
                if i_vld = '1' then 
                    w_nx_state <= ST_ERROR;
                end if;

            when ST_ERROR => 
                w_nx_state <= ST_GAIN;

            when ST_GAIN => 
                w_nx_state <= ST_ACCUM;

            when ST_ACCUM => 
                w_nx_state <= ST_OUTPUT;

            when ST_OUTPUT => 
                w_nx_state <= ST_IDLE;

            when others =>
                w_nx_state <= ST_IDLE;
        end case;
    end process fsm_advance;

    latch : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_target_curr       <= (others => (others => '0'));
                r_actual_curr       <= (others => (others => '0'));
            elsif r_state = ST_IDLE and i_vld = '1' then 
                r_target_curr       <= i_target_curr;
                r_actual_curr       <= i_actual_curr;
            end if;
        end if;
    end process latch;

    error : process(i_clk) 
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_error_curr        <= (others => (others => '0'));
            elsif r_state = ST_ERROR then 
                r_error_curr.d      <= resize(r_target_curr.d - r_actual_curr.d, r_error_curr.d);
                r_error_curr.q      <= resize(r_target_curr.q - r_actual_curr.q, r_error_curr.q);
            end if;
        end if;
    end process error;

    gains : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_gain_prop         <= (others => (others => '0'));
                r_gain_intg         <= (others => (others => '0'));
            elsif r_state = ST_GAIN then 
                r_gain_prop.d       <= resize(r_error_curr.d * C_KP_PROP_GAIN, r_gain_prop.d);
                r_gain_prop.q       <= resize(r_error_curr.q * C_KP_PROP_GAIN, r_gain_prop.q);

                r_gain_intg.d       <= resize(r_error_curr.d * C_KI_INTG_GAIN, r_gain_intg.d);
                r_gain_intg.q       <= resize(r_error_curr.q * C_KI_INTG_GAIN, r_gain_intg.q);
            end if;
        end if;
    end process gains;

    accumulator : process(i_clk)
        variable v_intg_accum : t_park_volt_record;
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_intg_accum        <= (others => (others => '0'));
            elsif r_state = ST_ACCUM then 
                v_intg_accum.d      := r_gain_intg.d + r_intg_accum.d;
                v_intg_accum.q      := r_gain_intg.q + r_intg_accum.q;

                -- Clamp direct axis integral accumulator 
                if v_intg_accum.d > C_WINDUP_MAX then 
                    r_intg_accum.d  <= C_WINDUP_MAX;
                elsif v_intg_accum.d < -C_WINDUP_MAX then 
                    r_intg_accum.d  <= resize(-C_WINDUP_MAX, r_intg_accum.d);
                else
                    r_intg_accum.d  <= v_intg_accum.d;
                end if;

                -- Clamp quadrature axis integral accumulator 
                if v_intg_accum.q > C_WINDUP_MAX then 
                    r_intg_accum.q  <= C_WINDUP_MAX;
                elsif v_intg_accum.q < -C_WINDUP_MAX then 
                    r_intg_accum.q  <= resize(-C_WINDUP_MAX, r_intg_accum.q);
                else
                    r_intg_accum.q  <= v_intg_accum.q;
                end if;
            end if;
        end if;
    end process accumulator;

    output : process(i_clk)
        variable v_output_volt : t_park_volt_record;
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                o_ph_voltage        <= (others => (others => '0'));
            elsif r_state = ST_OUTPUT then 
                v_output_volt.d     := r_intg_accum.d + r_gain_prop.d;
                v_output_volt.q     := r_intg_accum.q + r_gain_prop.q;

                -- Clamp direct axis voltage output
                if v_output_volt.d > C_WINDUP_MAX then 
                    o_ph_voltage.d  <= C_WINDUP_MAX;
                elsif v_output_volt.d < -C_WINDUP_MAX then 
                    o_ph_voltage.d  <= resize(-C_WINDUP_MAX, o_ph_voltage.d);
                else
                    o_ph_voltage.d  <= v_output_volt.d;
                end if;

                -- Clamp quadrature axis voltage output
                if v_output_volt.q > C_WINDUP_MAX then 
                    o_ph_voltage.q  <= C_WINDUP_MAX;
                elsif v_output_volt.q < -C_WINDUP_MAX then 
                    o_ph_voltage.q  <= resize(-C_WINDUP_MAX, o_ph_voltage.q);
                else
                    o_ph_voltage.q  <= v_output_volt.q;
                end if;

                o_vld <= '1';
            else 
                o_vld <= '0';
            end if;
        end if;
    end process output;
end architecture rtl;