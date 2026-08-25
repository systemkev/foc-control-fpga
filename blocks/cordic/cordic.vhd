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
        
        -- Handshake signals
        i_vld           : in std_logic;                         
        o_rdy           : out std_logic;                        
        
        -- 16-bit angle: -32768 = -180 deg, +32767 = +179.99 deg.
        i_angle         : in t_cordic_inputs;             
        
        -- Outputs
        o_vld           : out std_logic;                        
        o_cos           : out sfixed(1 downto -14);             -- Q1.14 cos output
        o_sin           : out sfixed(1 downto -14)              -- Q1.14 sin output    
    );
end entity cordic;

architecture rtl of cordic is 
    -- Pre-computed Arctan Table
    type t_arctan_table is array(0 to C_CORDIC_NUM_OF_ITERS-1) of t_cordic_inputs;

    function init_arctan_table return t_arctan_table is
        variable v_arctan_table : t_arctan_table;
    begin 
        for i in v_arctan_table'range loop
            v_arctan_table(i) := to_signed(integer(round(ARCTAN(2.0 ** (-i)) / MATH_PI * (2.0**15))), 16);
        end loop;
        return v_arctan_table;
    end function init_arctan_table;

    constant C_ARCTAN_TABLE : t_arctan_table := init_arctan_table;
    constant C_CORDIC_GAIN  : real := 0.607253;

    type t_state is (ST_IDLE, ST_ITERATE, ST_DONE);
    signal r_state, w_next_state : t_state;

    -- Status from Datapath to FSM
    signal w_iter_done  : std_logic;

    signal r_iter       : integer range 0 to C_CORDIC_NUM_OF_ITERS;
    signal r_x          : sfixed(1 downto -19);
    signal r_y          : sfixed(1 downto -19);
    signal r_z          : t_cordic_inputs;
    signal r_inv        : std_logic;

    signal w_inv_start      : std_logic;
    signal w_angl_err_start : t_cordic_inputs;

begin 
    -- Combinational quadrant mapping (folds Q2 and Q3 into Q1 and Q4)
    w_inv_start         <= i_angle(15) xor i_angle(14);
    w_angl_err_start    <= resize(i_angle(14 downto 0), 16) when w_inv_start = '1' else i_angle;

    w_iter_done         <= '1' when r_iter = C_CORDIC_NUM_OF_ITERS-1 else '0';

    fsm : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then 
                r_state <= ST_IDLE;
            else
                r_state <= w_next_state;
            end if;
        end if;
    end process fsm;

    fsm_advance : process(all)
    begin
        -- Default assignments to prevent latches
        w_next_state <= r_state;
        o_rdy        <= '0';

        case r_state is 
            when ST_IDLE =>
                o_rdy <= '1';
                if i_vld = '1' then
                    w_next_state <= ST_ITERATE;
                end if;

            when ST_ITERATE =>
                if w_iter_done = '1' then
                    w_next_state <= ST_DONE;
                end if;

            when ST_DONE =>
                w_next_state <= ST_IDLE;
        end case;
    end process fsm_advance;

    datapath : process(i_clk)
        variable v_x_shift  : sfixed(1 downto -19);
        variable v_y_shift  : sfixed(1 downto -19);
    begin
        if rising_edge(i_clk) then
            o_vld       <= '0';

            if r_state = ST_IDLE and w_next_state = ST_ITERATE then 
                r_x     <= to_sfixed(C_CORDIC_GAIN, r_x);
                r_y     <= to_sfixed(0.0, r_y);
                r_z     <= w_angl_err_start;
                r_inv   <= w_inv_start;
                r_iter  <= 0;

            elsif r_state = ST_ITERATE then
                -- Calculation phase
                v_x_shift := shift_right(r_x, r_iter);
                v_y_shift := shift_right(r_y, r_iter);

                if r_z >= 0 then 
                    r_x <= resize(r_x - v_y_shift, r_x);
                    r_y <= resize(r_y + v_x_shift, r_y);
                    r_z <= resize(r_z - C_ARCTAN_TABLE(r_iter), r_z);
                else 
                    r_x <= resize(r_x + v_y_shift, r_x);
                    r_y <= resize(r_y - v_x_shift, r_y);
                    r_z <= resize(r_z + C_ARCTAN_TABLE(r_iter), r_z);
                end if;
                r_iter <= r_iter + 1;

            elsif r_state = ST_DONE then
                -- Output phase with quadrant correction
                o_vld <= '1';

                if r_inv = '1' then
                    o_cos <= resize(-r_x, o_cos);
                    o_sin <= resize(-r_y, o_sin);
                else
                    o_cos <= resize(r_x, o_cos);
                    o_sin <= resize(r_y, o_sin);
                end if;
            end if;
        end if;
    end process datapath;
end architecture rtl;