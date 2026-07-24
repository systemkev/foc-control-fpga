library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.fixed_pkg.all;
use work.math_pkg.all;

entity park is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;

        i_clarke    : in t_clarke_record;
        i_angle     : in signed(15 downto 0);

        o_rdy       : out std_logic;
        o_park      : out t_park_record
    );
end entity park;

architecture rtl of park is
    -- The Park transform implements the following math:
    --      i_d = i_a cos(phi) + i_b sin(phi)
    --      i_q = i_b cos(phi) - i_a sin(phi)
    --          where a = alpha, b = beta
    signal r_input_clarke       : t_clarke_record;
    signal r_input_angle        : signed(15 downto 0);

    signal w_cordic_start       : std_logic;
    signal w_cordic_vld         : std_logic;
    signal w_cordic_cos         : sfixed(1 downto -14);
    signal w_cordic_sin         : sfixed(1 downto -14);
    signal w_cordic_rdy         : std_logic;

    signal r_cordic_vld         : std_logic;
    signal r_cordic_cos         : sfixed(1 downto -14);
    signal r_cordic_sin         : sfixed(1 downto -14);
    signal r_cordic_rdy         : std_logic;

    -- Park transformation intermediates
    signal r_id_ia_term         : sfixed(5 downto -26);
    signal r_id_ib_term         : sfixed(5 downto -26);
    signal r_iq_ia_term         : sfixed(5 downto -26);
    signal r_iq_ib_term         : sfixed(5 downto -26);

    signal r_d_park             : sfixed(3 downto -12);
    signal r_q_park             : sfixed(3 downto -12);

    -- FSM states
    type t_park_states is (
        ST_IDLE,   -- Wait for inputs to be valid 
        ST_FETCH,  -- Call the CORDIC (22 cycles)
        ST_MULT,
        ST_ADD,
        ST_OUTPUT
    );

    signal r_current_state      : t_park_states;
    signal w_next_state         : t_park_states;
begin
    o_rdy <= '1' when r_current_state = ST_IDLE else '0';

    w_cordic_start <= '1' when (r_current_state = ST_IDLE 
                            and i_clarke.vld = '1' 
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
                if i_clarke.vld = '1' and w_cordic_rdy = '1' then 
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

    u_cordic : entity work.cordic
        port map (
            i_clk       => i_clk,
            i_rst       => i_rst,
            i_angle     => r_input_angle,
            i_vld       => w_cordic_start,
            o_vld       => w_cordic_vld,
            o_cos       => w_cordic_cos,
            o_sin       => w_cordic_sin,
            o_rdy       => w_cordic_rdy
        );
    
    register_input : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_input_clarke      <= (vld   => '0', 
                                        alpha => (others => '0'), 
                                        beta  => (others => '0'));
                r_input_angle       <= (others => '0');
            elsif i_clarke.vld = '1' and r_current_state = ST_IDLE and w_cordic_rdy = '1' then 
                r_input_clarke      <= i_clarke;
                r_input_angle       <= i_angle;
            end if;
        end if;
    end process register_input;

    register_cordic : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_cordic_vld        <= '0';
                r_cordic_cos        <= (others => '0');
                r_cordic_sin        <= (others => '0');
                r_cordic_rdy        <= '0';
            elsif r_current_state = ST_FETCH then
                if w_cordic_vld = '1' then
                    r_cordic_vld    <= '1';
                    r_cordic_cos    <= w_cordic_cos;
                    r_cordic_sin    <= w_cordic_sin;
                    r_cordic_rdy    <= w_cordic_rdy;
                end if;
            end if;
        end if;
    end process register_cordic;

    -- The Park transform implements the following math:
    --      i_d = i_a cos(phi) + i_b sin(phi)
    --      i_q = i_b cos(phi) - i_a sin(phi)
    calc_intermediate : process(i_clk) 
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_id_ia_term    <= (others => '0');
                r_id_ib_term    <= (others => '0');
                r_iq_ia_term    <= (others => '0');
                r_iq_ib_term    <= (others => '0');
            elsif r_current_state = ST_MULT then 
                r_id_ia_term    <= resize(r_input_clarke.alpha  * r_cordic_cos, r_id_ia_term);
                r_id_ib_term    <= resize(r_input_clarke.beta   * r_cordic_sin, r_id_ib_term);
                r_iq_ib_term    <= resize(r_input_clarke.beta   * r_cordic_cos, r_iq_ib_term);
                r_iq_ia_term    <= resize(r_input_clarke.alpha  * r_cordic_sin, r_iq_ia_term);
            end if;
        end if;
    end process calc_intermediate;

    add_intermediates : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_d_park        <= (others => '0');
                r_q_park        <= (others => '0');
            elsif r_current_state = ST_ADD then
                r_d_park        <= resize(r_id_ia_term + r_id_ib_term, r_d_park);
                r_q_park        <= resize(r_iq_ib_term - r_iq_ia_term, r_q_park);
            end if;
        end if;
    end process add_intermediates;

    drive_output : process(i_clk)
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_park  <= (vld => '0', 
                            d => (others => '0'), 
                            q => (others => '0'));
            elsif r_current_state = ST_OUTPUT then
                o_park  <= (vld => '1', 
                            d => r_d_park, 
                            q => r_q_park);
            else 
                o_park.vld <= '0';
            end if;
        end if;
    end process drive_output;
end architecture rtl;