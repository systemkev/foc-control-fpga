library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
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
        o_current_d     : out t_phase_current;
        o_current_q     : out t_phase_current;

        -- Register file inputs
        -- Q11.20
        REG_VEL_KP_GAIN     : in signed(31 downto 0);
        -- Q7.24
        REG_VEL_KI_GAIN     : in signed(31 downto 0);
        -- Q9.22
        REG_VEL_WINDUP_MAX  : in signed(31 downto 0);
        -- Q9.22
        REG_VEL_CURRENT_MAX : in signed(31 downto 0)
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
                r_error_vel <= (others => '0');
            elsif r_vld_pipe(0) = '1' then
                -- Q8.20
                r_error_vel <= resize(r_target_vel - r_actual_vel, r_error_vel'length);
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
                -- Q8.20
                r_gain_prop <= resize(shift_right(r_error_vel * REG_VEL_KP_GAIN, 20), r_gain_prop'length);
                -- Q8.20
                r_gain_intg <= resize(shift_right(r_error_vel * REG_VEL_KI_GAIN, 24), r_gain_intg'length);
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
                -- Q8.20
                v_intg_accum := resize(r_gain_intg + r_intg_accum, v_intg_accum'length);

                -- Q8.20
                if v_intg_accum > resize(shift_right(REG_VEL_WINDUP_MAX, 2), v_intg_accum'length) then
                    -- Q8.20
                    r_intg_accum <= resize(shift_right(REG_VEL_WINDUP_MAX, 2), r_intg_accum'length);
                -- Q8.20
                elsif v_intg_accum < -resize(shift_right(REG_VEL_WINDUP_MAX, 2), v_intg_accum'length) then
                    -- Q8.20
                    r_intg_accum <= -resize(shift_right(REG_VEL_WINDUP_MAX, 2), r_intg_accum'length);
                else
                    r_intg_accum <= v_intg_accum;
                end if;
            end if;
        end if;
    end process accumulator;

    output : process(i_clk)
        variable v_output_curr_q : t_phase_current;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_vld       <= '0';
                o_current_d <= (others => '0');
                o_current_q <= (others => '0');
            else
                o_vld <= r_vld_pipe(3);

                if r_vld_pipe(3) = '1' then
                    -- Q3.12
                    v_output_curr_q := resize(shift_right(r_intg_accum + r_gain_prop, 8), v_output_curr_q'length);

                    o_current_d <= (others => '0');

                    -- Q3.12
                    if v_output_curr_q > resize(shift_right(REG_VEL_CURRENT_MAX, 10), v_output_curr_q'length) then
                        -- Q3.12
                        o_current_q <= resize(shift_right(REG_VEL_CURRENT_MAX, 10), o_current_q'length);
                    -- Q3.12
                    elsif v_output_curr_q < -resize(shift_right(REG_VEL_CURRENT_MAX, 10), v_output_curr_q'length) then
                        -- Q3.12
                        o_current_q <= -resize(shift_right(REG_VEL_CURRENT_MAX, 10), o_current_q'length);
                    else
                        o_current_q <= v_output_curr_q;
                    end if;
                end if;
            end if;
        end if;
    end process output;

end architecture rtl;
