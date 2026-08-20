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
        i_target_pos    : in signed(31 downto 0);   -- Unwrapped position cmd 
        i_actual_pos    : in unsigned(13 downto 0); -- Wrapped encoder input

        -- Outputs 
        o_vld           : out std_logic;
        o_omega         : out t_angl_velocity       -- Commanded 

    );
end entity position_cloop;

architecture rtl of position_cloop is
    constant C_POS_HALF : signed(14 downto 0) := to_signed(2.0 ** 13, 14);
    constant C_NEG_HALF : signed(14 downto 0) := to_signed(-2.0 ** 13, 14);

    signal r_vld_pipe   : std_logic_vector(1 downto 0);
    
    signal r_last_pos   : signed(31 downto 0);
    signal r_curr_pos   : signed(31 downto 0);

    signal r_last_angle : unsigned(13 downto 0);
begin

    pipe : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                r_vld_pipe <= (others => '0');
            else
                r_vld_pipe <= r_vld_pipe(0) & i_vld;
            end if;
        end if;
    end process pipe;

    wrap: process(i_clk)
        variable v_num_wraps : integer;
        variable v_ang_diff  : signed(14 downto 0);
    begin
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                r_num_wraps   <= 0;
                r_last_angle  <= (others => '0');
                r_curr_pos    <= (others => '0');
            elsif i_vld = '1' then 
                v_ang_diff  := resize(signed('0' & i_actual_pos) - signed('0' & r_last_angle), v_ang_diff);
                v_num_wraps := r_num_wraps;

                if v_ang_diff >= C_POS_HALF then 
                    v_num_wraps := v_num_wraps - 1;
                elsif v_ang_diff <= C_NEG_HALF then 
                    v_num_wraps := v_num_wraps + 1;
                end if;

                r_num_wraps  <= v_num_wraps;
                r_last_angle <= i_actual_pos;
                r_curr_pos   <= resize(shift_left(v_num_wraps, 14) + v_ang_diff, r_curr_pos);
            end if;
        end if;
    end process wrap;

    gains : process(i_clk)
        variable v_pos_err   : signed(31 downto 0);
        variable v_nxt_omega : signed(31 downto -20);
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                o_vld <= '0';
            elsif r_vld_pipe(0) = '1' then 

                -- error[n] = target - current
                v_pos_err := i_target_pos - r_pos_cur;

                if abs(v_pos_err) <= C_DEADBAND_TICKS then
                    v_pos_err := (others => '0');
                end if;

                -- w[n] = Kp * (error[n])
                v_nxt_omega := C_POS_KP_GAIN * to_sfixed(v_pos_err, 31, -20);

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