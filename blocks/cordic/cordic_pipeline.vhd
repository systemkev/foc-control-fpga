library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

library work;
use work.math_pkg.all;

entity cordic_pipeline is
    port (
        i_clk           : in std_logic;
        i_rst           : in std_logic;
        
        -- 16-bit angle: -32768 = -180 deg, +32767 = +179.99 deg.
        i_angle         : in signed(15 downto 0);             
        i_vld           : in std_logic;                         
        
        o_vld           : out std_logic;                        
        -- Q1.14
        o_cos           : out signed(15 downto 0);             -- Q1.14 cos output
        -- Q1.14
        o_sin           : out signed(15 downto 0)              -- Q1.14 sin output    
    );
end entity cordic_pipeline;

architecture rtl of cordic_pipeline is 
    constant C_CORDIC_GAIN : real := 0.607253;
    
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

    -- Q1.19
    type t_fixed_pipe   is array(0 to C_CORDIC_NUM_OF_ITERS) of signed(20 downto 0);
    type t_signed_pipe  is array(0 to C_CORDIC_NUM_OF_ITERS) of signed(15 downto 0);
    type t_vld_pipe     is array(0 to C_CORDIC_NUM_OF_ITERS) of std_logic;

    signal r_x_pipe         : t_fixed_pipe;
    signal r_y_pipe         : t_fixed_pipe;
    signal r_z_pipe         : t_signed_pipe;
    signal r_inv_pipe       : t_vld_pipe;
    signal r_vld_pipe       : t_vld_pipe;

    signal w_angl_err_start : signed(15 downto 0);
    signal w_inv_start      : std_logic;
begin 
    -- Combinational preprocessing 
    w_inv_start         <= i_angle(15) xor i_angle(14);
    w_angl_err_start    <= resize(i_angle(14 downto 0), 16) when w_inv_start = '1' else i_angle;

    pipe : process(i_clk)
        -- Q1.19
        variable v_x_shift  : signed(20 downto 0);
        -- Q1.19
        variable v_y_shift  : signed(20 downto 0);
    begin
        if i_rst = '1' then 
            r_vld_pipe      <= (others => '0');
        elsif rising_edge(i_clk) then
            -- Q1.19
            r_x_pipe(0)     <= to_signed(integer(round(C_CORDIC_GAIN * (2.0 ** 19))), r_x_pipe(0)'length);
            -- Q1.19
            r_y_pipe(0)     <= (others => '0');
            r_z_pipe(0)     <= w_angl_err_start;
            r_inv_pipe(0)   <= w_inv_start;
            r_vld_pipe(0)   <= i_vld;

            for i in 0 to C_CORDIC_NUM_OF_ITERS-1 loop 
                v_x_shift   := shift_right(r_x_pipe(i), i);
                v_y_shift   := shift_right(r_y_pipe(i), i);

                if r_z_pipe(i) >= 0 then 
                    r_x_pipe(i+1)   <= resize(r_x_pipe(i) - v_y_shift, r_x_pipe(i+1)'length);
                    r_y_pipe(i+1)   <= resize(r_y_pipe(i) + v_x_shift, r_y_pipe(i+1)'length);
                    r_z_pipe(i+1)   <= resize(r_z_pipe(i) - C_ARCTAN_TABLE(i), r_z_pipe(i+1)'length);
                else 
                    r_x_pipe(i+1)   <= resize(r_x_pipe(i) + v_y_shift, r_x_pipe(i+1)'length);
                    r_y_pipe(i+1)   <= resize(r_y_pipe(i) - v_x_shift, r_y_pipe(i+1)'length);
                    r_z_pipe(i+1)   <= resize(r_z_pipe(i) + C_ARCTAN_TABLE(i), r_z_pipe(i+1)'length);
                end if;

                r_inv_pipe(i+1) <= r_inv_pipe(i);
                r_vld_pipe(i+1) <= r_vld_pipe(i);
            end loop;
        end if;
    end process pipe;

    drive_output : process(all)
    begin
        o_vld <= r_vld_pipe(C_CORDIC_NUM_OF_ITERS);

        if r_inv_pipe(C_CORDIC_NUM_OF_ITERS) = '1' then
            -- Q1.14
            o_cos <= resize(shift_right(-r_x_pipe(C_CORDIC_NUM_OF_ITERS), 5), o_cos'length);
            -- Q1.14
            o_sin <= resize(shift_right(-r_y_pipe(C_CORDIC_NUM_OF_ITERS), 5), o_sin'length);
        else
            -- Q1.14
            o_cos <= resize(shift_right(r_x_pipe(C_CORDIC_NUM_OF_ITERS), 5), o_cos'length);
            -- Q1.14
            o_sin <= resize(shift_right(r_y_pipe(C_CORDIC_NUM_OF_ITERS), 5), o_sin'length);
        end if;
    end process drive_output;
end architecture rtl;
