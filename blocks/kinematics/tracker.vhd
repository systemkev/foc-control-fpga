library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;
use work.common_pkg.all;

entity tracker is
    port (
        i_clk           : in std_logic;
        i_rst           : in std_logic;
        i_vld           : in std_logic;
        i_actual_pos    : in t_angl_raw_unwr;
        o_vld           : out std_logic;
        o_actual_pos    : out t_angl_position;
        o_actual_vel    : out t_angl_velocity
    );
end entity tracker;

architecture rtl of tracker is

    -- Q0.20
    constant C_ALPHA_IIR : signed(20 downto 0) :=
        to_signed(integer(round(2.0e-4 * (2.0 ** 20))), 21);
    constant C_POS_HALF  : signed(14 downto 0) := to_signed(2**13, 15);
    constant C_NEG_HALF  : signed(14 downto 0) := to_signed(-(2**13), 15);

    signal r_last_angle  : t_angl_raw_unwr;
    signal r_num_wraps   : integer;
    signal r_actual_pos  : t_angl_position;
    signal r_last_pos    : t_angl_position;
    signal r_omega_curr  : t_angl_velocity;
    signal r_wrap_done   : std_logic;
    signal r_initialized : std_logic;

    -- Velocity filter timing pipeline.
    -- The system launches one encoder transaction per PWM period, so there are
    -- thousands of 100 MHz clocks between samples.  These registers therefore
    -- preserve system behavior while removing the multiply/add feedback chain
    -- from a single timing path.
    subtype t_filter_product is signed(49 downto 0); -- 21 x 29 bits

    signal r_filter_error    : t_angl_velocity := (others => '0');
    signal r_filter_omega    : t_angl_velocity := (others => '0');
    signal r_filter_pos      : t_angl_position := (others => '0');
    signal r_filter_vld      : std_logic := '0';

    signal r_filter_product  : t_filter_product := (others => '0');
    signal r_filter_omega_d  : t_angl_velocity := (others => '0');
    signal r_filter_pos_d    : t_angl_position := (others => '0');
    signal r_product_vld     : std_logic := '0';

begin

    ---------------------------------------------------------------------------
    -- Position unwrap.
    ---------------------------------------------------------------------------
    wrap : process(i_clk)
        variable v_num_wraps : integer;
        variable v_ang_diff  : signed(14 downto 0);
        variable v_seed_pos  : t_angl_position;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_num_wraps   <= 0;
                r_last_angle  <= (others => '0');
                r_actual_pos  <= (others => '0');
                r_last_pos    <= (others => '0');
                r_wrap_done   <= '0';
                r_initialized <= '0';
            elsif i_vld = '1' then
                if r_initialized = '0' then
                    v_seed_pos := resize(signed('0' & i_actual_pos), v_seed_pos'length);
                    r_num_wraps   <= 0;
                    r_last_angle  <= i_actual_pos;
                    r_actual_pos  <= v_seed_pos;
                    r_last_pos    <= v_seed_pos;
                    r_wrap_done   <= '1';
                    r_initialized <= '1';
                else
                    v_ang_diff  := resize(
                        signed('0' & i_actual_pos) - signed('0' & r_last_angle),
                        v_ang_diff'length
                    );
                    v_num_wraps := r_num_wraps;

                    if v_ang_diff >= C_POS_HALF then
                        v_num_wraps := v_num_wraps - 1;
                    elsif v_ang_diff <= C_NEG_HALF then
                        v_num_wraps := v_num_wraps + 1;
                    end if;

                    r_num_wraps   <= v_num_wraps;
                    r_last_angle  <= i_actual_pos;
                    r_last_pos    <= r_actual_pos;
                    r_actual_pos  <= resize(
                        to_signed(v_num_wraps * 16384, 32) + signed('0' & i_actual_pos),
                        r_actual_pos'length
                    );
                    r_wrap_done <= '1';
                end if;
            else
                r_wrap_done <= '0';
            end if;
        end if;
    end process wrap;

    ---------------------------------------------------------------------------
    -- Stage 1: build the IIR correction error and capture the corresponding
    -- position plus the current filter state.
    ---------------------------------------------------------------------------
    filter_error_stage : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_filter_error <= (others => '0');
                r_filter_omega <= (others => '0');
                r_filter_pos   <= (others => '0');
                r_filter_vld   <= '0';
            else
                r_filter_vld <= r_wrap_done;
                if r_wrap_done = '1' then
                    r_filter_error <=
                        shift_left(
                            resize(r_actual_pos - r_last_pos, r_omega_curr'length),
                            20
                        ) - r_omega_curr;
                    r_filter_omega <= r_omega_curr;
                    r_filter_pos   <= r_actual_pos;
                end if;
            end if;
        end if;
    end process filter_error_stage;

    ---------------------------------------------------------------------------
    -- Stage 2: IIR multiply.  This is now isolated between registers so Vivado
    -- can map it cleanly into a DSP48 path.
    ---------------------------------------------------------------------------
    filter_multiply_stage : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_filter_product <= (others => '0');
                r_filter_omega_d <= (others => '0');
                r_filter_pos_d   <= (others => '0');
                r_product_vld    <= '0';
            else
                r_product_vld <= r_filter_vld;
                if r_filter_vld = '1' then
                    r_filter_product <= C_ALPHA_IIR * r_filter_error;
                    r_filter_omega_d <= r_filter_omega;
                    r_filter_pos_d   <= r_filter_pos;
                end if;
            end if;
        end if;
    end process filter_multiply_stage;

    ---------------------------------------------------------------------------
    -- Stage 3: rescale the correction, update the feedback state, and publish.
    ---------------------------------------------------------------------------
    filter_output_stage : process(i_clk)
        variable v_correction : t_angl_velocity;
        variable v_vel_cur    : t_angl_velocity;
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_omega_curr <= (others => '0');
                o_actual_vel <= (others => '0');
                o_actual_pos <= (others => '0');
                o_vld        <= '0';
            else
                o_vld <= r_product_vld;
                if r_product_vld = '1' then
                    v_correction := resize(
                        shift_right(r_filter_product, 20),
                        v_correction'length
                    );
                    v_vel_cur := resize(
                        r_filter_omega_d + v_correction,
                        v_vel_cur'length
                    );

                    r_omega_curr <= v_vel_cur;
                    o_actual_vel <= v_vel_cur;
                    o_actual_pos <= r_filter_pos_d;
                end if;
            end if;
        end if;
    end process filter_output_stage;

end architecture rtl;
