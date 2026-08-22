library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use work.common_pkg.all;
use work.cloop_pkg.all;

entity position_cloop is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        -- Inputs 
        i_vld           : in std_logic;
        i_target_pos    : in t_angl_position;    -- Unwrapped position cmd 
        i_actual_pos    : in t_angl_position;    -- Wrapped encoder input

        -- Outputs 
        o_vld           : out std_logic;
        o_omega         : out t_angl_velocity    -- Commanded 

    );
end entity position_cloop;

architecture rtl of position_cloop is
    
    signal r_last_angle : t_angl_raw_unwr;
    signal r_num_wraps  : integer; 
    
begin

    gains : process(i_clk)
        variable v_pos_err   : t_angl_position;
        variable v_nxt_omega : signed(31 downto -20);
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                o_vld <= '0';
            elsif i_vld = '1' then 

                -- error[n] = target - current
                v_pos_err := i_target_pos - r_curr_pos;

                if abs(v_pos_err) <= C_DEADBAND_TICKS then
                    v_pos_err := (others => '0');
                end if;

                -- w[n] = Kp * (error[n])
                v_nxt_omega := resize(C_POS_KP_GAIN * to_sfixed(v_pos_err, 31, -20) v_nxt_omega);

                -- clamp to min/max values
                if v_nxt_omega > C_MAX_VELOCITY then 
                    v_nxt_omega := C_MAX_VELOCITY;
                elsif v_nxt_omega < C_MIN_VELOCITY then 
                    v_nxt_omega := C_MIN_VELOCITY;
                end if;

                o_vld   <= '1';
                o_omega <= resize(v_nxt_omega, o_omega);
                
            else 
                o_vld   <= '0';
            end if;
        end if;
    end process gains;
    
end architecture rtl;