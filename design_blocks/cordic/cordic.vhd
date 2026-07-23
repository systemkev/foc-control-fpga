library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use IEEE.fixed_pkg.all;

library work;
use work.math_pkg.all;

entity cordic is
    port (
        i_clk           : in std_logic;
        i_rst           : in std_logic;
        
        -- 16-bit angle: -32768 = -180 deg, +32767 = +179.99 deg.
        i_angle         : in signed(15 downto 0);             
        i_vld           : in std_logic;                         
        
        o_vld           : out std_logic;                        
        o_cos           : out sfixed(1 downto -14);             -- Q1.14 cos output
        o_sin           : out sfixed(1 downto -14);             -- Q1.14 sin output
        o_rdy           : out std_logic                         
    );
end entity cordic;

architecture rtl of cordic is 
    type t_arctan_table is array(0 to C_CORDIC_NUM_OF_ITERS-1) of signed(15 downto 0);

    function init_arctan_table return t_arctan_table is
        variable v_arctan_table : t_arctan_table;
    begin 
        for i in v_arctan_table'range loop
            v_arctan_table(i) := to_signed(integer(round(ARCTAN(2.0 ** (-i)) / MATH_PI * (2.0**15))), 16);
        end loop;
        return v_arctan_table;
    end function init_arctan_table;

    constant C_ARCTAN_TABLE : t_arctan_table := init_arctan_table;

    type t_state is (S_IDLE, S_ITERATE, S_DONE);

    signal r_state          : t_state := S_IDLE;
    signal w_next_state     : t_state;

    signal r_counter        : integer range 0 to C_CORDIC_NUM_OF_ITERS-1;
    signal w_next_counter   : integer range 0 to C_CORDIC_NUM_OF_ITERS-1;

    signal r_x_rotater      : sfixed(1 downto -19);
    signal r_y_rotater      : sfixed(1 downto -19);

    signal r_angle_error    : signed(15 downto 0);
    signal w_angl_err_start : signed(15 downto 0);

    signal w_inv_start      : std_logic;
    signal r_inv_output     : std_logic;
begin 
    o_rdy <= '1' when r_state = S_IDLE else '0';

    fsm : process(i_clk)
    begin 
        if rising_edge(i_clk) then 
            if i_rst = '1' then 
                r_state     <= S_IDLE;
                r_counter   <= 0;
            else 
                r_state     <= w_next_state;
                r_counter   <= w_next_counter;
            end if;
        end if;
    end process fsm;

    fsm_advance : process(all)
    begin 
        w_next_state        <= r_state;
        w_next_counter      <= r_counter;

        case r_state is 
            when S_IDLE =>
                w_next_counter  <= 0;
                if i_vld = '1' then 
                    w_next_state    <= S_ITERATE;
                else 
                    w_next_state    <= S_IDLE;
                end if;
            when S_ITERATE => 
                if r_counter = C_CORDIC_NUM_OF_ITERS-1 then
                    w_next_state    <= S_DONE;
                else 
                    w_next_counter  <= r_counter+1;
                end if;
            when others =>          -- This include S_DONE
                w_next_state        <= S_IDLE;
        end case;
    end process fsm_advance;

    preprocess_angle : process(all)
    begin
        -- XOR evaluates to '1' when the bits are different (Q2 or Q3)
        w_inv_start <= i_angle(15) xor i_angle(14);
            
        if (i_angle(15) xor i_angle(14)) = '1' then   
            -- We are in Q2 or Q3. 
            -- Throw away bit 15 and sign-extend bit 14 to map it back to Q1/Q4
            w_angl_err_start <= resize(i_angle(14 downto 0), 16);
        else
            -- We are in Q1 or Q4. The angle is already valid
            w_angl_err_start <= i_angle;
        end if;
    end process preprocess_angle;

    cordic_iteration : process(i_clk)
        variable v_x_shift : sfixed(1 downto -19);
        variable v_y_shift : sfixed(1 downto -19);
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                r_x_rotater         <= (others => '0');
                r_y_rotater         <= (others => '0');
                r_angle_error       <= (others => '0');
                r_inv_output        <= '0';
            else 
                v_x_shift := shift_right(r_x_rotater, r_counter);
                v_y_shift := shift_right(r_y_rotater, r_counter);

                case r_state is 
                    when S_IDLE => 
                        r_x_rotater     <= to_sfixed(0.607253, r_x_rotater);
                        r_y_rotater     <= to_sfixed(0.0, r_y_rotater);
                        r_angle_error   <= w_angl_err_start;
                        r_inv_output    <= w_inv_start;
                    when S_ITERATE => 
                        if r_angle_error >= 0 then
                            r_x_rotater     <= resize(r_x_rotater - v_y_shift, r_x_rotater);
                            r_y_rotater     <= resize(r_y_rotater + v_x_shift, r_y_rotater);
                            r_angle_error   <= r_angle_error - C_ARCTAN_TABLE(r_counter);
                        else 
                            r_x_rotater     <= resize(r_x_rotater + v_y_shift, r_x_rotater);
                            r_y_rotater     <= resize(r_y_rotater - v_x_shift, r_y_rotater);
                            r_angle_error   <= r_angle_error + C_ARCTAN_TABLE(r_counter);
                        end if;
                
                    when others => null;

                end case;
            end if;
        end if;
    end process cordic_iteration;

    drive_output : process(i_clk) 
    begin 
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                o_vld   <= '0';
                o_cos   <= (others => '0');
                o_sin   <= (others => '0');
            else
                o_vld   <= '0';

                if r_state = S_DONE then
                    o_vld   <= '1';

                    -- Truncate to Q1.14 and apply quadrant inversion if needed
                    if r_inv_output = '1' then
                        o_cos <= resize(-r_x_rotater, o_cos);
                        o_sin <= resize(-r_y_rotater, o_sin);
                    else
                        o_cos <= resize(r_x_rotater, o_cos);
                        o_sin <= resize(r_y_rotater, o_sin);
                    end if;
                end if;
            end if;
        end if;
    end process drive_output;
end architecture rtl;