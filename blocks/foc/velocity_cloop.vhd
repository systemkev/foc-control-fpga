library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use work.common_pkg.all;
use work.cloop_pkg.all;

entity velocity_cloop is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        -- Inputs 
        i_vld           : in std_logic;
        i_target_vel    : in t_angl_velocity;
        i_actual_vel    : in t_angl_velocity;

        -- Outputs
        o_vld           : out std_logic;
        o_current       : out t_dq_phase;

        -- Register file inputs 
        REG_VEL_KP_GAIN     : in sfixed(11 downto -20);
        REG_VEL_KI_GAIN     : in sfixed(7 downto -24);
        REG_VEL_WINDUP_MAX  : in sfixed(9 downto -22);
        REG_VEL_CURRENT_MAX : in sfixed(9 downto -22)
    );
end entity velocity_cloop;

architecture rtl of velocity_cloop is
    signal r_vld_pipe   : std_logic_vector(3 downto 0);

    signal r_target_vel : t_angl_velocity;
    signal r_actual_vel : t_angl_velocity;

    signal r_error_vel  : t_angl_velocity;

    signal r_gain_prop  : t_angl_velocity;
    signal r_gain_intg  : t_angl_velocity;

    signal r_intg_accum : t_angl_velocity;
begin
    
    pipeline_ctrl : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_vld_pipe <= (others => '0');
            else
                r_vld_pipe <= r_vld_pipe(2 downto 0) & i_vld;
            end if;
        end if;
    end process pipeline_ctrl;

    latch : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_target_vel <= (others => '0');
                r_actual_vel <= (others => '0');
            elsif i_vld = '1' then 
                r_target_vel <= i_target_vel;
                r_actual_vel <= i_actual_vel;
            end if;
        end if;
    end process latch;

    error : process(i_clk) 
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_error_vel <= (others =>'0');
            elsif r_vld_pipe(0) = '1' then 
                r_error_vel <= resize(r_target_vel - r_actual_vel, r_error_vel);
            end if;
        end if;
    end process error;
    
    gains : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_gain_prop <= (others => '0');
                r_gain_intg <= (others => '0');
            elsif r_vld_pipe(1) = '1' then 
                r_gain_prop <= resize(r_error_vel * REG_VEL_KP_GAIN, r_gain_prop);
                r_gain_intg <= resize(r_error_vel * REG_VEL_KI_GAIN, r_gain_intg);
            end if;
        end if;
    end process gains;

    accumulator : process(i_clk)
        variable v_intg_accum : t_angl_velocity;
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_intg_accum <= (others => '0');
            elsif r_vld_pipe(2) = '1' then 
                v_intg_accum := resize(r_gain_intg + r_intg_accum, v_intg_accum);

                if v_intg_accum > REG_VEL_WINDUP_MAX then 
                    r_intg_accum <= REG_VEL_WINDUP_MAX;
                elsif v_intg_accum < -REG_VEL_WINDUP_MAX then 
                    r_intg_accum <= resize(-REG_VEL_WINDUP_MAX, r_intg_accum);
                else
                    r_intg_accum <= v_intg_accum;
                end if;

            end if;
        end if;
    end process accumulator;

    output : process(i_clk)
        variable v_output_curr : t_dq_phase;
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                o_vld     <= '0';
                o_current <= (d => (others => '0'), q => (others => '0'));
            else 
                o_vld <= r_vld_pipe(3);
                
                if r_vld_pipe(3) = '1' then 
                    v_output_curr.q := resize(r_intg_accum + r_gain_prop, v_output_curr.q);

                    o_current.d <= (others => '0');

                    if v_output_curr.q > REG_VEL_CURRENT_MAX then 
                        o_current.q <= REG_VEL_CURRENT_MAX;
                    elsif v_output_curr.q < -REG_VEL_CURRENT_MAX then 
                        o_current.q <= resize(-REG_VEL_CURRENT_MAX, o_current.q);
                    else
                        o_current.q <= v_output_curr.q;
                    end if;
                end if;
            end if;
        end if;
    end process output;

end architecture rtl;