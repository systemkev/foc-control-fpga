library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.common_pkg.all;

entity motor_current_reader is
    port (
        i_clk                : in  std_logic;
        i_reset              : in  std_logic;
        i_pwm_center_trigger : in  std_logic;
        i_calibrate          : in  std_logic;

        i_top_pin_vauxp4     : in  std_logic;
        i_top_pin_vauxn4     : in  std_logic;
        i_top_pin_vauxp6     : in  std_logic;
        i_top_pin_vauxn6     : in  std_logic;

        o_vld                : out std_logic;
        o_phases_A           : out t_phase_current;
        o_phases_B           : out t_phase_current;
        o_phases_C           : out t_phase_current;
        o_calibrating        : out std_logic;
        o_cal_off_a          : out unsigned(15 downto 0);
        o_cal_off_b          : out unsigned(15 downto 0)
    );
end motor_current_reader;

architecture behavioral of motor_current_reader is

    component xadc_wiz_0 is
        port (
            daddr_in        : in  std_logic_vector(6 downto 0);
            den_in          : in  std_logic;
            di_in           : in  std_logic_vector(15 downto 0);
            dwe_in          : in  std_logic;
            do_out          : out std_logic_vector(15 downto 0);
            drdy_out        : out std_logic;
            dclk_in         : in  std_logic;
            reset_in        : in  std_logic;
            convst_in       : in  std_logic;
            vauxp4          : in  std_logic;
            vauxn4          : in  std_logic;
            vauxp6          : in  std_logic;
            vauxn6          : in  std_logic;
            busy_out        : out std_logic;
            channel_out     : out std_logic_vector(4 downto 0);
            eoc_out         : out std_logic;
            eos_out         : out std_logic;
            alarm_out       : out std_logic;
            vp_in           : in  std_logic;
            vn_in           : in  std_logic
        );
    end component;

    constant C_ADC_SCALE              : unsigned(15 downto 0) := to_unsigned(27034, 16);
    constant C_ADC_OFFSET_DEFAULT     : unsigned(15 downto 0) := to_unsigned(13517, 16);
    constant C_VAUX4_ADDR             : std_logic_vector(6 downto 0) := "0010100";
    constant C_VAUX6_ADDR             : std_logic_vector(6 downto 0) := "0010110";
    constant C_INTER_CONV_WAIT_CLKS   : natural := 100;
    constant C_CAL_SAMPLES            : natural := 256;

    subtype t_current_q12 is signed(15 downto 0);
    subtype t_current_sum is signed(16 downto 0);
    subtype t_cal_sum is unsigned(23 downto 0);

    function sat_to_current_q12(x : t_current_sum) return t_current_q12 is
        constant C_MAX : t_current_q12 := to_signed(2**15 - 1, 16);
        constant C_MIN : t_current_q12 := to_signed(-(2**15), 16);
    begin
        if x > resize(C_MAX, x'length) then
            return C_MAX;
        elsif x < resize(C_MIN, x'length) then
            return C_MIN;
        else
            return resize(x, 16);
        end if;
    end function;

    type t_acq_state is (
        ST_IDLE,
        ST_WAIT_DATA,
        ST_INTER_WAIT,
        ST_SCALE,
        ST_CONVERT_AB,
        ST_CONVERT_C
    );

    signal daddr_signal       : std_logic_vector(6 downto 0);
    signal den_signal         : std_logic;
    signal data_out_signal    : std_logic_vector(15 downto 0);
    signal drdy_signal        : std_logic;
    signal channel_signal     : std_logic_vector(4 downto 0);
    signal eoc_signal         : std_logic;
    signal r_daddr_pending    : std_logic_vector(6 downto 0) := (others => '0');
    signal r_convst           : std_logic := '0';

    signal r_state            : t_acq_state := ST_IDLE;
    signal r_wait_cnt         : natural range 0 to C_INTER_CONV_WAIT_CLKS := 0;
    signal r_have_a           : std_logic := '0';
    signal r_have_b           : std_logic := '0';

    signal adc_val_a          : unsigned(15 downto 0) := (others => '0');
    signal adc_val_b          : unsigned(15 downto 0) := (others => '0');
    signal calc_a             : unsigned(31 downto 0) := (others => '0');
    signal calc_b             : unsigned(31 downto 0) := (others => '0');

    signal current_a          : t_current_q12 := (others => '0');
    signal current_b          : t_current_q12 := (others => '0');
    signal current_c          : t_current_q12 := (others => '0');

    -- Timing-closure pipeline registers.  A/B are computed and saturated first,
    -- then C = -(A+B) is reconstructed on the following clock.  The externally
    -- visible phase outputs are committed together in ST_CONVERT_C so the sample
    -- remains coherent when o_vld is asserted.
    signal r_current_a_pending : t_current_q12 := (others => '0');
    signal r_current_b_pending : t_current_q12 := (others => '0');

    signal r_offset_a         : unsigned(15 downto 0) := C_ADC_OFFSET_DEFAULT;
    signal r_offset_b         : unsigned(15 downto 0) := C_ADC_OFFSET_DEFAULT;
    signal r_cal_active       : std_logic := '0';
    signal r_cal_count        : natural range 0 to C_CAL_SAMPLES - 1 := 0;
    signal r_cal_sum_a        : t_cal_sum := (others => '0');
    signal r_cal_sum_b        : t_cal_sum := (others => '0');

begin

    u_xadc : xadc_wiz_0
    port map (
        daddr_in        => daddr_signal,
        den_in          => den_signal,
        di_in           => (others => '0'),
        dwe_in          => '0',
        do_out          => data_out_signal,
        drdy_out        => drdy_signal,
        dclk_in         => i_clk,
        reset_in        => i_reset,
        convst_in       => r_convst,
        vauxp4          => i_top_pin_vauxp4,
        vauxn4          => i_top_pin_vauxn4,
        vauxp6          => i_top_pin_vauxp6,
        vauxn6          => i_top_pin_vauxn6,
        busy_out        => open,
        channel_out     => channel_signal,
        eoc_out         => eoc_signal,
        eos_out         => open,
        alarm_out       => open,
        vp_in           => '0',
        vn_in           => '0'
    );

    daddr_signal <= "00" & channel_signal;
    den_signal   <= eoc_signal;

    -- Q3.12
    o_phases_A   <= current_a;
    -- Q3.12
    o_phases_B   <= current_b;
    -- Q3.12
    o_phases_C   <= current_c;
    o_calibrating <= r_cal_active;
    o_cal_off_a   <= r_offset_a;
    o_cal_off_b   <= r_offset_b;

    process(i_clk)
        variable v_have_a    : std_logic;
        variable v_have_b    : std_logic;
        variable v_diff_a    : t_current_sum;
        variable v_diff_b    : t_current_sum;
        variable v_current_a : t_current_q12;
        variable v_current_b : t_current_q12;
        variable v_current_c : t_current_sum;
        variable v_sum_a     : t_cal_sum;
        variable v_sum_b     : t_cal_sum;
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                r_daddr_pending <= (others => '0');
                r_convst        <= '0';
                r_state         <= ST_IDLE;
                r_wait_cnt      <= 0;
                r_have_a        <= '0';
                r_have_b        <= '0';
                adc_val_a       <= (others => '0');
                adc_val_b       <= (others => '0');
                calc_a          <= (others => '0');
                calc_b          <= (others => '0');
                current_a          <= (others => '0');
                current_b          <= (others => '0');
                current_c          <= (others => '0');
                r_current_a_pending <= (others => '0');
                r_current_b_pending <= (others => '0');
                r_offset_a      <= C_ADC_OFFSET_DEFAULT;
                r_offset_b      <= C_ADC_OFFSET_DEFAULT;
                r_cal_active    <= '1';
                r_cal_count     <= 0;
                r_cal_sum_a     <= (others => '0');
                r_cal_sum_b     <= (others => '0');
                o_vld           <= '0';
            else
                r_convst <= '0';
                o_vld    <= '0';

                if eoc_signal = '1' then
                    r_daddr_pending <= daddr_signal;
                end if;

                if i_calibrate = '1' then
                    r_cal_active <= '1';
                    r_cal_count  <= 0;
                    r_cal_sum_a  <= (others => '0');
                    r_cal_sum_b  <= (others => '0');
                end if;

                case r_state is
                    when ST_IDLE =>
                        if i_pwm_center_trigger = '1' then
                            r_have_a <= '0';
                            r_have_b <= '0';
                            r_convst <= '1';
                            r_state  <= ST_WAIT_DATA;
                        end if;

                    when ST_WAIT_DATA =>
                        if drdy_signal = '1' then
                            v_have_a := r_have_a;
                            v_have_b := r_have_b;

                            if r_daddr_pending = C_VAUX4_ADDR then
                                adc_val_a <= unsigned(data_out_signal);
                                v_have_a := '1';
                            elsif r_daddr_pending = C_VAUX6_ADDR then
                                adc_val_b <= unsigned(data_out_signal);
                                v_have_b := '1';
                            end if;

                            r_have_a <= v_have_a;
                            r_have_b <= v_have_b;

                            if v_have_a = '1' and v_have_b = '1' then
                                r_state <= ST_SCALE;
                            else
                                r_wait_cnt <= C_INTER_CONV_WAIT_CLKS;
                                r_state    <= ST_INTER_WAIT;
                            end if;
                        end if;

                    when ST_INTER_WAIT =>
                        if r_wait_cnt = 0 then
                            r_convst <= '1';
                            r_state  <= ST_WAIT_DATA;
                        else
                            r_wait_cnt <= r_wait_cnt - 1;
                        end if;

                    when ST_SCALE =>
                        calc_a  <= adc_val_a * C_ADC_SCALE;
                        calc_b  <= adc_val_b * C_ADC_SCALE;
                        r_state <= ST_CONVERT_AB;

                    when ST_CONVERT_AB =>
                        -- Stage 1 of current conversion: scale-domain subtraction
                        -- and per-phase saturation.  Keeping C reconstruction out
                        -- of this cycle removes the old subtract/saturate/add/
                        -- saturate chain from the critical path.
                        v_diff_a := signed(resize(unsigned(calc_a(31 downto 16)), 17))
                                    - signed(resize(r_offset_a, 17));
                        v_diff_b := signed(resize(r_offset_b, 17))
                                    - signed(resize(unsigned(calc_b(31 downto 16)), 17));

                        r_current_a_pending <= sat_to_current_q12(v_diff_a);
                        r_current_b_pending <= sat_to_current_q12(v_diff_b);
                        r_state             <= ST_CONVERT_C;

                    when ST_CONVERT_C =>
                        -- Stage 2: reconstruct phase C from the already registered
                        -- A/B values, then commit the complete sample atomically.
                        v_current_c := -resize(r_current_a_pending, 17)
                                       - resize(r_current_b_pending, 17);

                        current_a <= r_current_a_pending;
                        current_b <= r_current_b_pending;
                        current_c <= sat_to_current_q12(v_current_c);
                        o_vld     <= '1';

                        -- Calibration still uses the scaled ADC values associated
                        -- with this exact sample.  The extra pipeline cycle does
                        -- not change the averaging arithmetic or sample count.
                        if r_cal_active = '1' then
                            v_sum_a := r_cal_sum_a + resize(unsigned(calc_a(31 downto 16)), r_cal_sum_a'length);
                            v_sum_b := r_cal_sum_b + resize(unsigned(calc_b(31 downto 16)), r_cal_sum_b'length);

                            if r_cal_count = C_CAL_SAMPLES - 1 then
                                r_offset_a   <= resize(shift_right(v_sum_a, 8), r_offset_a'length);
                                r_offset_b   <= resize(shift_right(v_sum_b, 8), r_offset_b'length);
                                r_cal_active <= '0';
                                r_cal_count  <= 0;
                                r_cal_sum_a  <= (others => '0');
                                r_cal_sum_b  <= (others => '0');
                            else
                                r_cal_sum_a <= v_sum_a;
                                r_cal_sum_b <= v_sum_b;
                                r_cal_count <= r_cal_count + 1;
                            end if;
                        end if;

                        r_state <= ST_IDLE;
                end case;
            end if;
        end if;
    end process;

end architecture behavioral;

