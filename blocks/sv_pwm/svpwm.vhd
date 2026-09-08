library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.common_pkg.all;

entity svpwm is
    port (
        -- system
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        -- input from inverse Clarke transformation (volts in A/B/C phases)
        i_vld           : in std_logic;
        i_abc_volt_A    : in t_phase_voltage;
        i_abc_volt_B    : in t_phase_voltage;
        i_abc_volt_C    : in t_phase_voltage;

        -- output PWMs 
        o_vld           : out std_logic;
        o_abc_cnts_A    : out integer range 0 to C_CLKS_IN_PWM_PRD;
        o_abc_cnts_B    : out integer range 0 to C_CLKS_IN_PWM_PRD;
        o_abc_cnts_C    : out integer range 0 to C_CLKS_IN_PWM_PRD
    );
end entity svpwm;

architecture rtl of svpwm is

    constant C_HALF_PERIOD : integer := C_CLKS_IN_PWM_PRD / 2;
    
    -- scales voltage to PWM ticks
    -- Q11.12
    constant C_V_TO_CNT    : signed(23 downto 0) := to_signed(integer((real(C_CLKS_IN_PWM_PRD) / C_VBUS) * (2.0 ** 12)), 24);


    -- =========================================================================
    -- pipeline registers
    -- =========================================================================
    -- stage 1
    signal r_vld_stg1   : std_logic;
    signal r_v_max_stg1 : t_phase_voltage;
    signal r_v_min_stg1 : t_phase_voltage;
    signal r_v_a_stg1   : t_phase_voltage;
    signal r_v_b_stg1   : t_phase_voltage;
    signal r_v_c_stg1   : t_phase_voltage;

    -- stage 2
    signal r_vld_stg2      : std_logic;
    signal r_v_offset_stg2 : t_phase_voltage;
    signal r_v_a_stg2      : t_phase_voltage;
    signal r_v_b_stg2      : t_phase_voltage;
    signal r_v_c_stg2      : t_phase_voltage;

    -- stage 3
    signal r_vld_stg3      : std_logic;
    signal r_v_a_star_stg3 : t_phase_voltage;
    signal r_v_b_star_stg3 : t_phase_voltage;
    signal r_v_c_star_stg3 : t_phase_voltage;

begin

    process(i_clk)
        variable v_a, v_b, v_c : t_phase_voltage;
        variable v_max, v_min  : t_phase_voltage;
        
        variable v_cnt_a       : integer;
        variable v_cnt_b       : integer;
        variable v_cnt_c       : integer;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_vld_stg1 <= '0';
                r_vld_stg2 <= '0';
                r_vld_stg3 <= '0';
                o_vld      <= '0';
                
                o_abc_cnts_A <= C_HALF_PERIOD;
                o_abc_cnts_B <= C_HALF_PERIOD;
                o_abc_cnts_C <= C_HALF_PERIOD;
            else
                -- =============================================================
                -- stage 1: find extrema
                -- =============================================================
                r_vld_stg1 <= i_vld;
                
                if i_vld = '1' then
                    v_a := i_abc_volt_A;
                    v_b := i_abc_volt_B;
                    v_c := i_abc_volt_C;
                    
                    -- Pass through raw values
                    r_v_a_stg1 <= v_a;
                    r_v_b_stg1 <= v_b;
                    r_v_c_stg1 <= v_c;
                    
                    -- Find Maximum
                    if (v_a >= v_b) and (v_a >= v_c) then
                        v_max := v_a;
                    elsif (v_b >= v_a) and (v_b >= v_c) then
                        v_max := v_b;
                    else
                        v_max := v_c;
                    end if;
                    r_v_max_stg1 <= v_max;
                    
                    -- Find Minimum
                    if (v_a <= v_b) and (v_a <= v_c) then
                        v_min := v_a;
                    elsif (v_b <= v_a) and (v_b <= v_c) then
                        v_min := v_b;
                    else
                        v_min := v_c;
                    end if;
                    r_v_min_stg1 <= v_min;
                end if;

                -- =============================================================
                -- stage 2: calculate zero-sequence offset
                -- =============================================================
                r_vld_stg2 <= r_vld_stg1;
                
                if r_vld_stg1 = '1' then
                    r_v_a_stg2 <= r_v_a_stg1;
                    r_v_b_stg2 <= r_v_b_stg1;
                    r_v_c_stg2 <= r_v_c_stg1;
                    
                    -- V_offset = -(V_max + V_min) * 0.5
                    -- mult by 0.5 is equivalent to shift right by 1
                    -- Q5.12
                    r_v_offset_stg2 <= resize(shift_right(-(r_v_max_stg1 + r_v_min_stg1), 1), r_v_offset_stg2'length);
                end if;

                -- =============================================================
                -- stage 3: inject offset
                -- =============================================================
                r_vld_stg3 <= r_vld_stg2;
                
                if r_vld_stg2 = '1' then
                    -- V* = V + V_offset
                    -- Q5.12
                    r_v_a_star_stg3 <= resize(r_v_a_stg2 + r_v_offset_stg2, r_v_a_star_stg3'length);
                    -- Q5.12
                    r_v_b_star_stg3 <= resize(r_v_b_stg2 + r_v_offset_stg2, r_v_b_star_stg3'length);
                    -- Q5.12
                    r_v_c_star_stg3 <= resize(r_v_c_stg2 + r_v_offset_stg2, r_v_c_star_stg3'length);
                end if;

                -- =============================================================
                -- stage 4: scale to PWM integers and saturate
                -- =============================================================
                o_vld <= r_vld_stg3;
                
                if r_vld_stg3 = '1' then
                    -- multiply the voltage by our conversion constant
                    -- convert from fixed-point to standard integer
                    -- add C_HALF_PERIOD so 0V centers perfectly at 50% duty cycle
                    -- Q17.24
                    v_cnt_a := to_integer(shift_right(r_v_a_star_stg3 * C_V_TO_CNT, 24)) + C_HALF_PERIOD;
                    -- Q17.24
                    v_cnt_b := to_integer(shift_right(r_v_b_star_stg3 * C_V_TO_CNT, 24)) + C_HALF_PERIOD;
                    -- Q17.24
                    v_cnt_c := to_integer(shift_right(r_v_c_star_stg3 * C_V_TO_CNT, 24)) + C_HALF_PERIOD;
                    
                    -- phase A saturation limits
                    if v_cnt_a > C_CLKS_IN_PWM_PRD then
                        o_abc_cnts_A <= C_CLKS_IN_PWM_PRD;
                    elsif v_cnt_a < 0 then
                        o_abc_cnts_A <= 0;
                    else
                        o_abc_cnts_A <= v_cnt_a;
                    end if;
                    
                    -- phase B saturation limits
                    if v_cnt_b > C_CLKS_IN_PWM_PRD then
                        o_abc_cnts_B <= C_CLKS_IN_PWM_PRD;
                    elsif v_cnt_b < 0 then
                        o_abc_cnts_B <= 0;
                    else
                        o_abc_cnts_B <= v_cnt_b;
                    end if;

                    -- Phase C saturation limits
                    if v_cnt_c > C_CLKS_IN_PWM_PRD then
                        o_abc_cnts_C <= C_CLKS_IN_PWM_PRD;
                    elsif v_cnt_c < 0 then
                        o_abc_cnts_C <= 0;
                    else
                        o_abc_cnts_C <= v_cnt_c;
                    end if;
                    
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
