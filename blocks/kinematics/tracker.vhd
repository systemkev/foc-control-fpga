library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;

entity tracker is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        -- Inputs 
        i_vld           : in std_logic;
        i_actual_pos    : in t_angl_raw_unwr;   -- Wrapped encoder input

        -- Outputs 
        o_vld           : out std_logic;
        o_actual_pos    : out t_angl_position;
        o_actual_vel    : out t_angl_velocity
    );
end entity tracker;

architecture rtl of tracker is

    constant C_ALPHA_IIR : sfixed(0 downto -4) := to_sfixed(2.0e-4, 0, -4);
    constant C_POS_HALF  : signed(14 downto 0) := to_signed(2.0 ** 13, 14);
    constant C_NEG_HALF  : signed(14 downto 0) := to_signed(-2.0 ** 13, 14);

    signal r_last_angle : t_angl_raw_unwr;
    signal r_num_wraps  : integer; 
    signal r_actual_pos : t_angl_position;
    signal r_last_pos   : t_angl_position;
    signal r_omega_curr : t_angl_velocity;
    signal r_wrap_done  : std_logic;

begin
    wrap: process(i_clk)
        variable v_num_wraps : integer;
        variable v_ang_diff  : signed(14 downto 0);
    begin
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                r_num_wraps   <= 0;
                r_last_angle  <= (others => '0');
                r_actual_pos  <= (others => '0');
                r_last_pos    <= (others => '0');
                r_wrap_done   <= '0';
            elsif i_vld = '1' then 
                v_ang_diff  := resize(signed('0' & i_actual_pos) - signed('0' & r_last_angle), v_ang_diff);
                v_num_wraps := r_num_wraps;

                if v_ang_diff >= C_POS_HALF then 
                    v_num_wraps := v_num_wraps - 1;
                elsif v_ang_diff <= C_NEG_HALF then 
                    v_num_wraps := v_num_wraps + 1;
                end if;

                r_wrap_done  <= '1';

                r_num_wraps  <= v_num_wraps;
                r_last_angle <= i_actual_pos;

                r_last_pos   <= r_actual_pos;
                r_actual_pos <= resize(v_num_wraps * 16384 + i_actual_pos, o_actual_pos);
            else 
                r_wrap_done  <= '0';
            end if;
        end if;
    end process wrap;
    
    filter: process(i_clk)
        variable v_vel_cur : t_angl_velocity;
    begin
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                r_omega_curr <= (others => '0');
                o_vld        <= '0';
            elsif r_wrap_done = '1' then 
                v_vel_cur    := resize(r_omega_curr + C_ALPHA_IIR * (r_actual_pos - r_last_pos - r_omega_curr), v_vel_cur);
                r_omega_curr <= v_vel_cur;
                o_actual_vel <= v_vel_cur;
                o_actual_pos <= r_actual_pos;
                
                o_vld        <= '1';
            else 
                o_vld        <= '0';
            end if;
        end if;
    end process filter;
    
end architecture rtl;