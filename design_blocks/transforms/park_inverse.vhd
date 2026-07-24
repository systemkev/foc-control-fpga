library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use work.math_pkg.all;

entity park_inverse is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        i_angle     : in signed(15 downto 0);
        i_park      : in t_park_record;

        o_rdy       : out std_logic;
        o_inv_park  : out t_clarke_record
    );
end entity park_inverse;

architecture rtl of park_inverse is
    -- The Inverse Park transform implements the following math:
    --      V_alpha = V_d * cos(theta) - V_q * sin(theta)
    --      V_beta  = V_d * sin(theta) + V_q * cos(theta)

    signal r_input_park         : t_park_record;
    signal r_input_angle        : signed(15 downto 0);

    signal w_cordic_start       : std_logic;
    signal w_cordic_vld         : std_logic;
    signal w_cordic_rdy         : std_logic;
    signal w_cordic_cos         : sfixed(1 downto -14);
    signal w_cordic_sin         : sfixed(1 downto -14);

    signal r_cordic_cos         : sfixed(1 downto -14);
    signal r_cordic_sin         : sfixed(1 downto -14);

    -- Corrected Inverse Park transformation intermediates
    signal r_d_cos_term         : sfixed(5 downto -26);
    signal r_q_sin_term         : sfixed(5 downto -26);
    signal r_d_sin_term         : sfixed(5 downto -26);
    signal r_q_cos_term         : sfixed(5 downto -26);

    signal r_alpha_inv_park     : sfixed(3 downto -12);
    signal r_beta_inv_park      : sfixed(3 downto -12);

    -- FSM states
    type t_park_states is (
        ST_IDLE,   
        ST_FETCH,  
        ST_MULT,
        ST_ADD,
        ST_OUTPUT
    );

    signal r_current_state      : t_park_states;
    signal w_next_state         : t_park_states;
begin
    u_cordic : entity work.cordic
        port map (
            i_clk       => i_clk,
            i_rst       => i_rst,
            i_vld       => w_cordic_start,
            o_rdy       => w_cordic_rdy,
            i_angle     => r_input_angle,
            o_vld       => w_cordic_vld,
            o_cos       => w_cordic_cos,
            o_sin       => w_cordic_sin        
        );

    o_rdy <= '1' when r_current_state = ST_IDLE else '0';
    w_cordic_start <= '1' when (r_current_state = ST_IDLE 
                            and i_park.vld = '1' 
                            and w_cordic_rdy = '1') 
                        else '0';

    fsm : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                r_current_state <= ST_IDLE;
            else
                r_current_state <= w_next_state;
            end if;
        end if;
    end process fsm;

    fsm_advance : process(all)
    begin 
        w_next_state    <= r_current_state;

        case r_current_state is
            when ST_IDLE => 
                -- Fixed: Look at i_park.vld
                if i_park.vld = '1' and w_cordic_rdy = '1' then 
                    w_next_state <= ST_FETCH;
                end if;
            when ST_FETCH => 
                if w_cordic_vld = '1' then 
                    w_next_state <= ST_MULT;
                end if;
            when ST_MULT => 
                w_next_state <= ST_ADD;
            when ST_ADD => 
                w_next_state <= ST_OUTPUT;
            when ST_OUTPUT =>
                w_next_state <= ST_IDLE;
            when others => 
                w_next_state <= ST_IDLE;
        end case;
    end process fsm_advance;
    
    register_input : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                -- Fixed: t_park_record uses d and q
                r_input_park        <= (vld => '0', 
                                        d   => (others => '0'), 
                                        q   => (others => '0'));
                r_input_angle       <= (others => '0');
            elsif i_park.vld = '1' and r_current_state = ST_IDLE and w_cordic_rdy = '1' then 
                r_input_park        <= i_park;
                r_input_angle       <= i_angle;
            end if;
        end if;
    end process register_input;

    register_cordic : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_cordic_cos        <= (others => '0');
                r_cordic_sin        <= (others => '0');
            elsif r_current_state = ST_FETCH then
                if w_cordic_vld = '1' then
                    r_cordic_cos    <= w_cordic_cos;
                    r_cordic_sin    <= w_cordic_sin;
                end if;
            end if;
        end if;
    end process register_cordic;

    -- Execute multiplication phase
    calc_intermediate : process(i_clk) 
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_d_cos_term    <= (others => '0');
                r_q_sin_term    <= (others => '0');
                r_d_sin_term    <= (others => '0');
                r_q_cos_term    <= (others => '0');
            elsif r_current_state = ST_MULT then 
                r_d_cos_term    <= resize(r_input_park.d * r_cordic_cos, r_d_cos_term);
                r_q_sin_term    <= resize(r_input_park.q * r_cordic_sin, r_q_sin_term);
                r_d_sin_term    <= resize(r_input_park.d * r_cordic_sin, r_d_sin_term);
                r_q_cos_term    <= resize(r_input_park.q * r_cordic_cos, r_q_cos_term);
            end if;
        end if;
    end process calc_intermediate;

    -- execute addition/subtraction phase
    add_intermediates : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_alpha_inv_park    <= (others => '0');
                r_beta_inv_park     <= (others => '0');
            elsif r_current_state = ST_ADD then
                -- Fixed: Resize targets refer to the actual target signals
                r_alpha_inv_park    <= resize(r_d_cos_term - r_q_sin_term, r_alpha_inv_park);
                r_beta_inv_park     <= resize(r_d_sin_term + r_q_cos_term, r_beta_inv_park);
            end if;
        end if;
    end process add_intermediates;

    -- Drive Output Port
    drive_output : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                -- Fixed: Target o_inv_park and use clarke struct members
                o_inv_park  <= (vld   => '0', 
                                alpha => (others => '0'), 
                                beta  => (others => '0'));
            elsif r_current_state = ST_OUTPUT then
                o_inv_park  <= (vld   => '1', 
                                alpha => r_alpha_inv_park, 
                                beta  => r_beta_inv_park);
            else 
                o_inv_park  <= (vld   => '0', 
                                alpha => (others => '0'), 
                                beta  => (others => '0'));
            end if;
        end if;
    end process drive_output;
end architecture rtl;