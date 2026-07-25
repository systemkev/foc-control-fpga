entity pi_control is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;
        
        -- Input from Park transformation 
        i_rdy           : in std_logic;
        i_target_curr   : in t_phase_record;

        -- Inputs from target and actual current
        i_actual_curr   : in t_phase_record;

        -- Output to SVPWM
        o_ph_voltage    : out t_clarke_volt_record
    );
end entity pi_control;

architecture rtl of pi_control is
    type t_pi_controller_states is (
        ST_IDLE,
        ST_MATH,
        ST_GAIN,
        ST_WINDUP
    );
    
    signal r_state      : t_pi_controller_states;
    signal w_nx_state   : t_pi_controller_states;

    signal r_target_curr    : t_phase_record;
    signal r_actual_curr    : t_phase_record;
    signal r_error_curr     : t_phase_record;
    signal r_gain_prop      : t_phase_record;
    signal r_gain_intg      : t_phase_record;
begin
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
                if i_rdy = '1' then 
                    w_nx_state <= ST_MATH;
                end if;

            when ST_ERROR => 
                w_nx_state <= ST_GAIN;

            when ST_GAIN => 
                w_nx_state <= ST_WINDUP;

            when ST_WINDUP => 
                when w_nx_state <= ST_IDLE;

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
            elsif r_state = ST_IDLE and i_rdy = '1' then 
                r_target_curr       <= i_target_curr;
                r_actual_curr       <= i_actual_curr;
            end if;
        end if;
    end process latch;

    error : process(i_clk) 
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_error_curr.vld    <= (0 => '0', others => (others => '0'));
            elsif r_state = ST_IDLE and i_rdy = '1' then 
                r_error_curr.vld    <= '1';
                r_error_curr.A      <= resize(r_target_curr.A - r_actual_curr.A, r_error_curr.A);
                r_error_curr.B      <= resize(r_target_curr.B - r_actual_curr.B, r_error_curr.B);
                r_error_curr.C      <= resize(r_target_curr.C - r_actual_curr.C, r_error_curr.C);
            end if;
        end if;
    end process error;

    gains : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_gain_mult         <= (0 => '0', others => (others => '0'));
                r_gain_intg         <= (0 => '0', others => (others => '0'));
            else 
                r_gain_mult.vld     <= '1';
                r_gain_mult.A       <= resize(r_error_curr.A * C_KP_PROP_GAIN, r_gain_mult.A);
                r_gain_mult.B       <= resize(r_error_curr.B * C_KP_PROP_GAIN, r_gain_mult.B);
                r_gain_mult.C       <= resize(r_error_curr.C * C_KP_PROP_GAIN, r_gain_mult.C);

                r_gain_intg.vld     <= '1';
                r_gain_intg.A       <= resize(r_error_curr.A * C_KI_INTG_GAIN, r_gain_intg.A);
                r_gain_intg.B       <= resize(r_error_curr.B * C_KI_INTG_GAIN, r_gain_intg.B);
                r_gain_intg.C       <= resize(r_error_curr.C * C_KI_INTG_GAIN, r_gain_intg.C);
            end if;
        end if;
    end process gains;

    

end architecture rtl;