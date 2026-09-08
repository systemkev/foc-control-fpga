library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

library work;
use work.math_pkg.all;
use work.common_pkg.all;

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
        o_cos           : out t_sin_cos_outpt; -- Q1.14 cos output
        o_sin           : out t_sin_cos_outpt  -- Q1.14 sin output
    );
end entity cordic;

architecture rtl of cordic is
    type t_arctan_table is array(0 to C_CORDIC_NUM_OF_ITERS-1) of t_cordic_inputs;

    function init_arctan_table return t_arctan_table is
        variable v_arctan_table : t_arctan_table;
    begin
        for i in v_arctan_table'range loop
            v_arctan_table(i) := to_signed(
                integer(round(ARCTAN(2.0 ** (-i)) / MATH_PI * (2.0**15))),
                16
            );
        end loop;
        return v_arctan_table;
    end function init_arctan_table;

    constant C_ARCTAN_TABLE : t_arctan_table := init_arctan_table;
    constant C_CORDIC_GAIN  : real := 0.607253;

    type t_state is (ST_IDLE, ST_ITERATE, ST_DONE);
    signal r_state, w_next_state : t_state;

    signal w_iter_done : std_logic;

    signal r_iter : integer range 0 to C_CORDIC_NUM_OF_ITERS;
    signal r_x    : signed(20 downto 0); -- Q1.19
    signal r_y    : signed(20 downto 0); -- Q1.19
    signal r_z    : t_cordic_inputs;
    signal r_inv  : std_logic;

    signal w_inv_start      : std_logic;
    signal w_angl_err_start : t_cordic_inputs;
begin
    -- Combinational quadrant mapping (folds Q2 and Q3 into Q1 and Q4)
    w_inv_start      <= i_angle(15) xor i_angle(14);
    w_angl_err_start <= resize(i_angle(14 downto 0), 16)
                        when w_inv_start = '1' else i_angle;

    w_iter_done <= '1' when r_iter = C_CORDIC_NUM_OF_ITERS-1 else '0';

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
        variable v_x_shift : signed(20 downto 0); -- Q1.19
        variable v_y_shift : signed(20 downto 0); -- Q1.19
    begin
        if rising_edge(i_clk) then
            -- IMPORTANT: reset the datapath as well as the FSM. Without this
            -- branch, an old r_state=ST_DONE can still assert o_vld on the
            -- same edge that the FSM synchronously resets to ST_IDLE.
            if i_rst = '1' then
                o_vld  <= '0';
                o_cos  <= (others => '0');
                o_sin  <= (others => '0');
                r_x    <= (others => '0');
                r_y    <= (others => '0');
                r_z    <= (others => '0');
                r_inv  <= '0';
                r_iter <= 0;

            else
                o_vld <= '0';

                if r_state = ST_IDLE and w_next_state = ST_ITERATE then
                    r_x <= to_signed(
                        integer(round(C_CORDIC_GAIN * (2.0 ** 19))),
                        r_x'length
                    );
                    r_y    <= (others => '0');
                    r_z    <= w_angl_err_start;
                    r_inv  <= w_inv_start;
                    r_iter <= 0;

                elsif r_state = ST_ITERATE then
                    v_x_shift := shift_right(r_x, r_iter);
                    v_y_shift := shift_right(r_y, r_iter);

                    if r_z >= 0 then
                        r_x <= resize(r_x - v_y_shift, r_x'length);
                        r_y <= resize(r_y + v_x_shift, r_y'length);
                        r_z <= resize(r_z - C_ARCTAN_TABLE(r_iter), r_z'length);
                    else
                        r_x <= resize(r_x + v_y_shift, r_x'length);
                        r_y <= resize(r_y - v_x_shift, r_y'length);
                        r_z <= resize(r_z + C_ARCTAN_TABLE(r_iter), r_z'length);
                    end if;
                    r_iter <= r_iter + 1;

                elsif r_state = ST_DONE then
                    o_vld <= '1';

                    if r_inv = '1' then
                        o_cos <= resize(shift_right(-r_x, 5), o_cos'length);
                        o_sin <= resize(shift_right(-r_y, 5), o_sin'length);
                    else
                        o_cos <= resize(shift_right(r_x, 5), o_cos'length);
                        o_sin <= resize(shift_right(r_y, 5), o_sin'length);
                    end if;
                end if;
            end if;
        end if;
    end process datapath;
end architecture rtl;
