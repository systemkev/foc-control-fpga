library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use work.common_pkg.all;
use work.cloop_pkg.all;

entity position_cloop is
    port (
        -- System
        i_clk            : in std_logic;
        i_rst            : in std_logic;

        -- Inputs 
        i_vld            : in std_logic;
        i_target_pos     : in t_angl_position;    -- Unwrapped position cmd 
        i_actual_pos     : in t_angl_position;    -- Wrapped encoder input

        -- Outputs 
        o_vld            : out std_logic;
        o_omega          : out t_angl_velocity;   -- Commanded 

        -- Register file inputs
        REG_POS_KP_GAIN      : in sfixed(7 downto -24);
        REG_POS_MAX_VELOCITY : in sfixed(15 downto -16);
        REG_POS_DEADBAND_TCK : in unsigned(31 downto 0)
    );
end entity position_cloop;

architecture rtl of position_cloop is
    
    -- Pipeline shift register
    signal r_vld_pipe   : std_logic_vector(2 downto 0);
    
    -- Latch stage
    signal r_target_pos : t_angl_position;
    signal r_actual_pos : t_angl_position;
    
    -- Error stage
    signal r_pos_err    : t_angl_position;
    
    -- Proportional Output stage
    signal r_omega_raw  : sfixed(31 downto -20);
    
begin

    pipeline_ctrl : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_vld_pipe <= (others => '0');
            else
                r_vld_pipe <= r_vld_pipe(1 downto 0) & i_vld;
            end if;
        end if;
    end process pipeline_ctrl;

    latch : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                r_target_pos <= (others => '0');
                r_actual_pos <= (others => '0');
            elsif i_vld = '1' then
                r_target_pos <= i_target_pos;
                r_actual_pos <= i_actual_pos;
            end if;
        end if;
    end process latch;

    error : process(i_clk)
        variable v_err : t_angl_position;
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then
                r_pos_err <= (others => '0');
            elsif r_vld_pipe(0) = '1' then
                v_err := r_target_pos - r_actual_pos;
                
                -- Apply deadband safely 
                if unsigned(abs(v_err)) <= REG_POS_DEADBAND_TCK then
                    r_pos_err <= (others => '0');
                else
                    r_pos_err <= v_err;
                end if;
            end if;
        end if;
    end process error;

    gains : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_omega_raw <= (others => '0');
            elsif r_vld_pipe(1) = '1' then
                r_omega_raw <= resize(REG_POS_KP_GAIN * to_sfixed(r_pos_err, 31, 0), r_omega_raw);
            end if;
        end if;
    end process gains;

    output : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_vld   <= '0';
                o_omega <= (others => '0');
            else
                o_vld <= r_vld_pipe(2);
                
                if r_vld_pipe(2) = '1' then
                    if r_omega_raw > REG_POS_MAX_VELOCITY then
                        o_omega <= resize(REG_POS_MAX_VELOCITY, o_omega);
                    elsif r_omega_raw < -REG_POS_MAX_VELOCITY then
                        o_omega <= resize(-REG_POS_MAX_VELOCITY, o_omega);
                    else
                        o_omega <= resize(r_omega_raw, o_omega);
                    end if;
                end if;
            end if;
        end if;
    end process output;
    
end architecture rtl;