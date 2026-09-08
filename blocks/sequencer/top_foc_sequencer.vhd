library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library work;
use work.common_pkg.all;
use work.reg_pkg.all;

entity motor_controller_top is
    port (
        -- Arty A7 100 MHz system clock / reset
        i_clk               : in  std_logic;
        i_rst               : in  std_logic;

        -- Host UART
        i_uart_rx           : in  std_logic;
        o_uart_tx           : out std_logic;

        -- AS5048A SPI
        i_enc_miso          : in  std_logic;
        o_enc_mosi          : out std_logic;
        o_enc_n_cs          : out std_logic;
        o_enc_sclk          : out std_logic;

        -- XADC current-sense inputs
        i_vauxp4            : in  std_logic;
        i_vauxn4            : in  std_logic;
        i_vauxp6            : in  std_logic;
        i_vauxn6            : in  std_logic;

        -- SimpleFOC Shield / L6234 inputs
        o_pwm_a             : out std_logic;
        o_pwm_b             : out std_logic;
        o_pwm_c             : out std_logic;
        o_motor_en          : out std_logic
    );
end entity motor_controller_top;

architecture rtl of motor_controller_top is

    subtype t_current_q12 is signed(15 downto 0);

    function current_limit_q12(x : std_logic_vector(31 downto 0)) return t_current_q12 is
        variable v_ext : signed(32 downto 0);
        variable v_mag : signed(32 downto 0);
        variable v_q12 : signed(32 downto 0);
        constant C_MAX : t_current_q12 := to_signed(2**15 - 1, 16);
    begin
        v_ext := resize(signed(x), v_ext'length);
        if v_ext < 0 then
            v_mag := -v_ext;
        else
            v_mag := v_ext;
        end if;
        v_q12 := shift_right(v_mag, 10);
        if v_q12 > resize(C_MAX, v_q12'length) then
            return C_MAX;
        else
            return resize(v_q12, 16);
        end if;
    end function;

    function clamp_current_q12(x : t_current_q12; lim : t_current_q12) return t_current_q12 is
    begin
        if x > lim then
            return lim;
        elsif x < -lim then
            return -lim;
        else
            return x;
        end if;
    end function;

    ---------------------------------------------------------------------------
    -- Minimal operating-mode convention for CONTROL[2:1]
    ---------------------------------------------------------------------------
    constant C_MODE_CURRENT  : std_logic_vector(1 downto 0) := "00";
    constant C_MODE_VELOCITY : std_logic_vector(1 downto 0) := "01";
    constant C_MODE_POSITION : std_logic_vector(1 downto 0) := "10";
    constant C_MODE_DISABLED : std_logic_vector(1 downto 0) := "11";

    -- A missing encoder or current sample for three complete PWM periods
    -- latches a fault and removes motor enable.
    constant C_SENSOR_TIMEOUT_CLKS : natural := 3 * C_CLKS_IN_PWM_PRD;


    ---------------------------------------------------------------------------
    -- UART / register-file interconnect
    ---------------------------------------------------------------------------
    signal w_uart_rx_vld       : std_logic;
    signal w_uart_rx_byte      : std_logic_vector(7 downto 0);

    signal w_req_vld           : std_logic;
    signal w_req_rw            : std_logic;
    signal w_req_data          : std_logic_vector(31 downto 0);
    signal w_req_addr          : std_logic_vector(7 downto 0);

    signal w_rsp_vld           : std_logic;
    signal w_rsp_data          : std_logic_vector(31 downto 0);
    signal w_rsp_addr          : std_logic_vector(7 downto 0);

    signal w_ser_vld           : std_logic;
    signal w_ser_byte          : std_logic_vector(7 downto 0);
    signal w_uart_tx_rdy       : std_logic;


    ---------------------------------------------------------------------------
    -- Register file
    ---------------------------------------------------------------------------
    signal REG_FOC_KP_GAIN      : std_logic_vector(31 downto 0);
    signal REG_FOC_KI_GAIN      : std_logic_vector(31 downto 0);
    signal REG_FOC_WINDUP_MAX   : std_logic_vector(31 downto 0);
    signal REG_FOC_VOLTAGE_MAX  : std_logic_vector(31 downto 0);
    signal REG_VEL_KP_GAIN      : std_logic_vector(31 downto 0);
    signal REG_VEL_KI_GAIN      : std_logic_vector(31 downto 0);
    signal REG_VEL_WINDUP_MAX   : std_logic_vector(31 downto 0);
    signal REG_VEL_CURRENT_MAX  : std_logic_vector(31 downto 0);
    signal REG_POS_KP_GAIN      : std_logic_vector(31 downto 0);
    signal REG_POS_MAX_VELOCITY : std_logic_vector(31 downto 0);
    signal REG_POS_DEADBAND_TCK : std_logic_vector(31 downto 0);
    signal REG_CONTROL          : std_logic_vector(31 downto 0);
    signal REG_TARGET_DQ        : std_logic_vector(31 downto 0);
    signal REG_TARGET_VEL       : std_logic_vector(31 downto 0);
    signal REG_TARGET_POS       : std_logic_vector(31 downto 0);
    signal REG_MANUAL_ELEC      : std_logic_vector(31 downto 0);

    signal w_clear_fault_pulse  : std_logic;
    signal w_calibrate_pulse    : std_logic;


    ---------------------------------------------------------------------------
    -- Controller state / safety
    ---------------------------------------------------------------------------
    -- Registered copy of the host control decode.  REG_CONTROL is written by
    -- the UART register file and changes very infrequently, so one clock of
    -- command latency is harmless and removes the register-file flops from
    -- timing-critical high-fanout control paths.
    signal w_mode               : std_logic_vector(1 downto 0);
    signal r_enable_cmd         : std_logic;
    signal r_manual_elec_mode   : std_logic;
    signal r_sensor_inv         : std_logic;

    signal w_mode_valid         : std_logic;
    signal w_drive_en           : std_logic;

    signal r_have_encoder       : std_logic;
    signal r_have_current       : std_logic;
    signal r_sensor_fault       : std_logic;
    signal r_encoder_timeout    : natural range 0 to C_SENSOR_TIMEOUT_CLKS;
    signal r_current_timeout    : natural range 0 to C_SENSOR_TIMEOUT_CLKS;

    -- Registered synchronous resets for the control loops.  These are local
    -- clocked sources rather than high-fanout combinational functions of
    -- REG_CONTROL and the safety state.
    signal r_current_rst        : std_logic := '1';
    signal r_velocity_rst       : std_logic := '1';
    signal r_position_rst       : std_logic := '1';


    ---------------------------------------------------------------------------
    -- PWM timing / current acquisition
    ---------------------------------------------------------------------------
    signal w_pwm_sync           : std_logic;
    signal w_pwm_count_a        : integer range 0 to C_CLKS_IN_PWM_PRD;
    signal w_pwm_count_b        : integer range 0 to C_CLKS_IN_PWM_PRD;
    signal w_pwm_count_c        : integer range 0 to C_CLKS_IN_PWM_PRD;

    signal w_phase_current_a    : t_phase_current;
    signal w_phase_current_b    : t_phase_current;
    signal w_phase_current_c    : t_phase_current;
    signal w_current_vld        : std_logic;
    signal w_calibrating        : std_logic;
    signal w_cal_off_a          : unsigned(15 downto 0);
    signal w_cal_off_b          : unsigned(15 downto 0);

    -- Hard transaction boundary between the ADC reader and Clarke.  The reader
    -- may be physically distant from the transform DSPs, so capturing the
    -- complete sample here prevents that route from being part of Clarke's
    -- arithmetic timing path.
    signal r_phase_sample_a     : t_phase_current := (others => '0');
    signal r_phase_sample_b     : t_phase_current := (others => '0');
    signal r_phase_sample_c     : t_phase_current := (others => '0');

    signal r_current_fresh      : std_logic;
    signal r_trig_fresh         : std_logic;
    signal w_foc_fire           : std_logic;


    ---------------------------------------------------------------------------
    -- Encoder / kinematics
    ---------------------------------------------------------------------------
    signal w_encoder_vld        : std_logic;
    signal w_encoder_raw        : t_angl_raw_unwr;
    signal w_mech_raw           : t_angl_raw_unwr;

    signal w_tracker_vld        : std_logic;
    signal w_actual_pos         : t_angl_position;
    signal w_actual_vel         : t_angl_velocity;

    signal w_elec_vld           : std_logic;
    signal w_elec_pos           : t_angl_raw_unwr;
    signal w_elec_cordic        : t_cordic_inputs;
    signal w_manual_cordic      : t_cordic_inputs;


    ---------------------------------------------------------------------------
    -- CORDIC / electrical angle
    ---------------------------------------------------------------------------
    signal w_cordic_i_vld       : std_logic;
    signal w_cordic_i_angle     : t_cordic_inputs;
    signal w_cordic_rdy         : std_logic;
    signal w_cordic_vld         : std_logic;
    signal w_cordic_cos         : t_sin_cos_outpt;
    signal w_cordic_sin         : t_sin_cos_outpt;

    signal r_cos_q14            : signed(15 downto 0);
    signal r_sin_q14            : signed(15 downto 0);


    ---------------------------------------------------------------------------
    -- Clarke / Park current path
    ---------------------------------------------------------------------------
    signal w_phase_a_q12        : signed(15 downto 0);
    signal w_phase_b_q12        : signed(15 downto 0);
    signal w_phase_c_q12        : signed(15 downto 0);

    signal w_clarke_vld         : std_logic;
    signal w_curr_alpha         : signed(15 downto 0);
    signal w_curr_beta          : signed(15 downto 0);

    signal w_park_vld           : std_logic;
    signal w_curr_d             : signed(15 downto 0);
    signal w_curr_q             : signed(15 downto 0);


    ---------------------------------------------------------------------------
    -- Outer control loops
    ---------------------------------------------------------------------------
    signal w_pos_vld            : std_logic;
    signal w_pos_omega          : t_angl_velocity;

    signal w_vel_loop_vld       : std_logic;
    signal w_vel_target         : t_angl_velocity;
    signal w_target_vel_reg     : t_angl_velocity;
    signal w_vel_vld            : std_logic;
    signal w_vel_current_d      : t_phase_current;
    signal w_vel_current_q      : t_phase_current;

    signal w_target_curr_d      : signed(15 downto 0);
    signal w_target_curr_q      : signed(15 downto 0);
    signal r_vel_current_max_cfg : std_logic_vector(31 downto 0) := (others => '0');
    signal r_current_limit_q12    : t_current_q12 := (others => '0');


    ---------------------------------------------------------------------------
    -- Inner current loop / voltage path
    ---------------------------------------------------------------------------
    signal w_foc_vld            : std_logic;
    signal w_voltage_d          : signed(17 downto 0);
    signal w_voltage_q          : signed(17 downto 0);

    signal w_inv_park_vld       : std_logic;
    signal w_voltage_alpha      : signed(17 downto 0);
    signal w_voltage_beta       : signed(17 downto 0);

    signal w_inv_clarke_vld     : std_logic;
    signal w_voltage_a          : signed(17 downto 0);
    signal w_voltage_b          : signed(17 downto 0);
    signal w_voltage_c          : signed(17 downto 0);
    signal w_abc_voltage_a      : t_phase_voltage;
    signal w_abc_voltage_b      : t_phase_voltage;
    signal w_abc_voltage_c      : t_phase_voltage;


    ---------------------------------------------------------------------------
    -- Telemetry register words
    ---------------------------------------------------------------------------
    signal w_status_data        : std_logic_vector(31 downto 0);
    signal w_actual_pos_data    : std_logic_vector(31 downto 0);
    signal w_actual_vel_data    : std_logic_vector(31 downto 0);
    signal w_phase_ab_data      : std_logic_vector(31 downto 0);
    signal w_phase_c_id_data    : std_logic_vector(31 downto 0);
    signal w_iq_elec_data       : std_logic_vector(31 downto 0);

begin

    ---------------------------------------------------------------------------
    -- Registered control decode / timing boundary
    ---------------------------------------------------------------------------
    control_decode_regs : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                w_mode             <= C_MODE_DISABLED;
                r_enable_cmd       <= '0';
                r_manual_elec_mode <= '0';
                r_sensor_inv       <= '0';
                r_vel_current_max_cfg <= (others => '0');
            else
                w_mode             <= REG_CONTROL(C_CTRL_MODE_MSB downto C_CTRL_MODE_LSB);
                r_enable_cmd       <= REG_CONTROL(C_CTRL_ENABLE_BIT);
                r_manual_elec_mode <= REG_CONTROL(C_CTRL_MANUAL_ELEC_BIT);
                r_sensor_inv       <= REG_CONTROL(C_CTRL_SENSOR_INV_BIT);

                -- First configuration stage: remove REG_VEL_CURRENT_MAX itself
                -- from the high-fanout/current-loop timing cone.
                r_vel_current_max_cfg <= REG_VEL_CURRENT_MAX;
            end if;
        end if;
    end process control_decode_regs;

    -- Second configuration stage: perform the relatively wide abs/shift/
    -- saturation conversion between two dedicated registers.  Host writes are
    -- rare, so two clocks of configuration latency is insignificant.
    current_limit_reg : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_current_limit_q12 <= (others => '0');
            else
                r_current_limit_q12 <= current_limit_q12(r_vel_current_max_cfg);
            end if;
        end if;
    end process current_limit_reg;

    w_mode_valid <= '1' when w_mode = C_MODE_CURRENT or
                             w_mode = C_MODE_VELOCITY or
                             w_mode = C_MODE_POSITION
                    else '0';

    w_drive_en <= '1' when r_enable_cmd = '1' and
                           w_mode_valid = '1' and
                           r_have_encoder = '1' and
                           r_have_current = '1' and
                           r_sensor_fault = '0' and
                           w_calibrating = '0'
                  else '0';

    ---------------------------------------------------------------------------
    -- Registered loop resets
    --
    -- All three downstream loops use synchronous reset, so registering these
    -- reset conditions removes the high-fanout REG_CONTROL -> loop paths while
    -- retaining deterministic state clearing on enable/mode changes.
    ---------------------------------------------------------------------------
    loop_reset_regs : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_current_rst  <= '1';
                r_velocity_rst <= '1';
                r_position_rst <= '1';
            else
                if w_drive_en = '0' then
                    r_current_rst <= '1';
                else
                    r_current_rst <= '0';
                end if;

                if w_drive_en = '0' or
                   (w_mode /= C_MODE_VELOCITY and w_mode /= C_MODE_POSITION) then
                    r_velocity_rst <= '1';
                else
                    r_velocity_rst <= '0';
                end if;

                if w_drive_en = '0' or w_mode /= C_MODE_POSITION then
                    r_position_rst <= '1';
                else
                    r_position_rst <= '0';
                end if;
            end if;
        end if;
    end process loop_reset_regs;


    ---------------------------------------------------------------------------
    -- UART receive -> parser -> register file -> serializer -> UART transmit
    ---------------------------------------------------------------------------
    u_uart_rx : entity work.uart_rx
        port map (
            i_clk  => i_clk,
            i_rst  => i_rst,
            i_rx   => i_uart_rx,
            o_data => w_uart_rx_byte,
            o_vld  => w_uart_rx_vld,
            o_baud => open,
            o_bit  => open
        );

    u_parser : entity work.parser
        port map (
            i_clk  => i_clk,
            i_rst  => i_rst,
            i_vld  => w_uart_rx_vld,
            i_byte => w_uart_rx_byte,
            o_vld  => w_req_vld,
            o_rw   => w_req_rw,
            o_data => w_req_data,
            o_addr => w_req_addr
        );

    u_register_file : entity work.register_file
        port map (
            i_clk                  => i_clk,
            i_rst                  => i_rst,
            i_req_vld              => w_req_vld,
            i_req_rw               => w_req_rw,
            i_req_data             => w_req_data,
            i_req_addr             => w_req_addr,
            o_rsp_vld              => w_rsp_vld,
            o_rsp_data             => w_rsp_data,
            o_rsp_addr             => w_rsp_addr,

            REG_FOC_KP_GAIN        => REG_FOC_KP_GAIN,
            REG_FOC_KI_GAIN        => REG_FOC_KI_GAIN,
            REG_FOC_WINDUP_MAX     => REG_FOC_WINDUP_MAX,
            REG_FOC_VOLTAGE_MAX    => REG_FOC_VOLTAGE_MAX,
            REG_VEL_KP_GAIN        => REG_VEL_KP_GAIN,
            REG_VEL_KI_GAIN        => REG_VEL_KI_GAIN,
            REG_VEL_WINDUP_MAX     => REG_VEL_WINDUP_MAX,
            REG_VEL_CURRENT_MAX    => REG_VEL_CURRENT_MAX,
            REG_POS_KP_GAIN        => REG_POS_KP_GAIN,
            REG_POS_MAX_VELOCITY   => REG_POS_MAX_VELOCITY,
            REG_POS_DEADBAND_TCK   => REG_POS_DEADBAND_TCK,
            REG_CONTROL            => REG_CONTROL,
            REG_TARGET_DQ          => REG_TARGET_DQ,
            REG_TARGET_VEL         => REG_TARGET_VEL,
            REG_TARGET_POS         => REG_TARGET_POS,
            REG_MANUAL_ELEC        => REG_MANUAL_ELEC,

            o_clear_fault_pulse    => w_clear_fault_pulse,
            o_calibrate_pulse      => w_calibrate_pulse,

            i_status               => w_status_data,
            i_actual_pos           => w_actual_pos_data,
            i_actual_vel           => w_actual_vel_data,
            i_phase_ab             => w_phase_ab_data,
            i_phase_c_id           => w_phase_c_id_data,
            i_iq_elec              => w_iq_elec_data,
            i_cal_off_ab           => std_logic_vector(w_cal_off_a) & std_logic_vector(w_cal_off_b),
            i_cal_off_c            => (others => '0')
        );

    u_serializer : entity work.serializer
        port map (
            i_clk  => i_clk,
            i_rst  => i_rst,
            i_vld  => w_rsp_vld,
            i_data => w_rsp_data,
            i_addr => w_rsp_addr,
            i_rdy  => w_uart_tx_rdy,
            o_vld  => w_ser_vld,
            o_byte => w_ser_byte
        );

    u_uart_tx : entity work.uart_tx
        port map (
            i_clk  => i_clk,
            i_rst  => i_rst,
            i_vld  => w_ser_vld,
            i_byte => w_ser_byte,
            o_tx   => o_uart_tx,
            o_done => open,
            o_rdy  => w_uart_tx_rdy
        );


    ---------------------------------------------------------------------------
    -- 3-phase PWM.  The generator free-runs even while disabled so its sync
    -- pulse remains the single timing reference for sensor acquisition.
    ---------------------------------------------------------------------------
    u_pwm_driver : entity work.pwm_3ph_driver
        port map (
            i_clk       => i_clk,
            i_rst       => i_rst,
            i_en        => w_drive_en,
            i_abc_cnts_A => w_pwm_count_a,
            i_abc_cnts_B => w_pwm_count_b,
            i_abc_cnts_C => w_pwm_count_c,
            o_pwm_a     => o_pwm_a,
            o_pwm_b     => o_pwm_b,
            o_pwm_c     => o_pwm_c,
            o_en        => o_motor_en,
            o_sync_tick => w_pwm_sync
        );


    ---------------------------------------------------------------------------
    -- Current acquisition
    ---------------------------------------------------------------------------
    u_current_reader : entity work.motor_current_reader
        port map (
            i_clk                => i_clk,
            i_reset              => i_rst,
            i_pwm_center_trigger => w_pwm_sync,
            i_calibrate          => w_calibrate_pulse,
            i_top_pin_vauxp4     => i_vauxp4,
            i_top_pin_vauxn4     => i_vauxn4,
            i_top_pin_vauxp6     => i_vauxp6,
            i_top_pin_vauxn6     => i_vauxn6,
            o_phases_A           => w_phase_current_a,
            o_phases_B           => w_phase_current_b,
            o_phases_C           => w_phase_current_c,
            o_vld                => w_current_vld,
            o_calibrating        => w_calibrating,
            o_cal_off_a          => w_cal_off_a,
            o_cal_off_b          => w_cal_off_b
        );

    -- Register a complete current sample at the reader-valid boundary.  The
    -- freshness flag below guarantees this snapshot is the one consumed by the
    -- next w_foc_fire transaction.
    current_sample_regs : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_phase_sample_a <= (others => '0');
                r_phase_sample_b <= (others => '0');
                r_phase_sample_c <= (others => '0');
            elsif w_current_vld = '1' then
                r_phase_sample_a <= w_phase_current_a;
                r_phase_sample_b <= w_phase_current_b;
                r_phase_sample_c <= w_phase_current_c;
            end if;
        end if;
    end process current_sample_regs;

    -- Q3.12 registered snapshot used by Clarke.
    w_phase_a_q12 <= r_phase_sample_a;
    w_phase_b_q12 <= r_phase_sample_b;
    w_phase_c_q12 <= r_phase_sample_c;


    ---------------------------------------------------------------------------
    -- AS5048A read.  One read sequence is launched per PWM period.
    ---------------------------------------------------------------------------
    u_encoder : entity work.as5048a_spi_master
        port map (
            i_clk       => i_clk,
            i_rst       => i_rst,
            i_start     => w_pwm_sync,
            i_miso      => i_enc_miso,
            o_mosi      => o_enc_mosi,
            o_n_cs      => o_enc_n_cs,
            o_sclk      => o_enc_sclk,
            o_vld       => w_encoder_vld,
            o_raw_angle => w_encoder_raw
        );

    -- Optional encoder direction inversion, modulo 2^14.
    encoder_direction : process(all)
    begin
        if r_sensor_inv = '1' then
            w_mech_raw <= (not w_encoder_raw) + 1;
        else
            w_mech_raw <= w_encoder_raw;
        end if;
    end process encoder_direction;


    ---------------------------------------------------------------------------
    -- Mechanical position / velocity tracking
    ---------------------------------------------------------------------------
    u_tracker : entity work.tracker
        port map (
            i_clk        => i_clk,
            i_rst        => i_rst,
            i_vld        => w_encoder_vld,
            i_actual_pos => w_mech_raw,
            o_vld        => w_tracker_vld,
            o_actual_pos => w_actual_pos,
            o_actual_vel => w_actual_vel
        );


    ---------------------------------------------------------------------------
    -- Mechanical -> electrical angle
    --
    -- Minimal register convention:
    --   REG_MANUAL_ELEC[13:0], MANUAL_ELEC=0 -> electrical encoder offset
    --   REG_MANUAL_ELEC[13:0], MANUAL_ELEC=1 -> absolute electrical test angle
    --
    -- This avoids adding another configuration register solely for alignment.
    ---------------------------------------------------------------------------
    u_mech_to_elec : entity work.mech_to_elec
        port map (
            i_clk         => i_clk,
            i_rst         => i_rst,
            i_vld         => w_encoder_vld,
            i_mech_pos    => w_mech_raw,
            i_offset      => unsigned(REG_MANUAL_ELEC(13 downto 0)),
            o_vld         => w_elec_vld,
            o_elec_pos    => w_elec_pos,
            o_elec_cordic => w_elec_cordic
        );

    w_manual_cordic <= signed((not REG_MANUAL_ELEC(13)) &
                              REG_MANUAL_ELEC(12 downto 0) & "00");

    w_cordic_i_vld   <= (w_encoder_vld and w_cordic_rdy)
                        when r_manual_elec_mode = '1'
                        else (w_elec_vld and w_cordic_rdy);

    w_cordic_i_angle <= w_manual_cordic when r_manual_elec_mode = '1'
                        else w_elec_cordic;


    ---------------------------------------------------------------------------
    -- Electrical sin/cos
    ---------------------------------------------------------------------------
    u_cordic : entity work.cordic
        port map (
            i_clk   => i_clk,
            i_rst   => i_rst,
            i_vld   => w_cordic_i_vld,
            o_rdy   => w_cordic_rdy,
            i_angle => w_cordic_i_angle,
            o_vld   => w_cordic_vld,
            o_cos   => w_cordic_cos,
            o_sin   => w_cordic_sin
        );


    ---------------------------------------------------------------------------
    -- Pair the newest electrical angle and newest current conversion belonging
    -- to the current PWM period.  Either sensor may finish first.
    ---------------------------------------------------------------------------
    current_fresh : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' or w_drive_en = '0' or w_pwm_sync = '1' then
                r_current_fresh <= '0';
            elsif w_current_vld = '1' then
                r_current_fresh <= '1';
            elsif w_foc_fire = '1' then
                r_current_fresh <= '0';
            end if;
        end if;
    end process current_fresh;

    trig_fresh : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_cos_q14    <= (others => '0');
                r_sin_q14    <= (others => '0');
                r_trig_fresh <= '0';
            elsif w_drive_en = '0' or w_pwm_sync = '1' then
                r_trig_fresh <= '0';
            elsif w_cordic_vld = '1' then
                -- Q1.14
                r_cos_q14    <= w_cordic_cos;
                -- Q1.14
                r_sin_q14    <= w_cordic_sin;
                r_trig_fresh <= '1';
            elsif w_foc_fire = '1' then
                r_trig_fresh <= '0';
            end if;
        end if;
    end process trig_fresh;

    w_foc_fire <= '1' when r_current_fresh = '1' and
                           r_trig_fresh = '1' and
                           w_drive_en = '1'
                  else '0';


    ---------------------------------------------------------------------------
    -- Current Clarke / Park transforms
    ---------------------------------------------------------------------------
    u_clarke : entity work.clarke
        port map (
            i_clk          => i_clk,
            i_rst          => i_rst,
            i_vld          => w_foc_fire,
            i_phases_A     => w_phase_a_q12,
            i_phases_B     => w_phase_b_q12,
            i_phases_C     => w_phase_c_q12,
            o_vld          => w_clarke_vld,
            o_clarke_alpha => w_curr_alpha,
            o_clarke_beta  => w_curr_beta
        );

    u_park : entity work.park
        port map (
            i_clk   => i_clk,
            i_rst   => i_rst,
            i_vld   => w_clarke_vld,
            i_alpha => w_curr_alpha,
            i_beta  => w_curr_beta,
            i_cos   => r_cos_q14,
            i_sin   => r_sin_q14,
            o_vld   => w_park_vld,
            o_d     => w_curr_d,
            o_q     => w_curr_q
        );


    ---------------------------------------------------------------------------
    -- Position loop
    ---------------------------------------------------------------------------
    u_position_cloop : entity work.position_cloop
        port map (
            i_clk                => i_clk,
            i_rst                => r_position_rst,
            i_vld                => w_tracker_vld,
            i_target_pos         => signed(REG_TARGET_POS),
            i_actual_pos         => w_actual_pos,
            o_vld                => w_pos_vld,
            o_omega              => w_pos_omega,
            -- Q7.24
            REG_POS_KP_GAIN      => signed(REG_POS_KP_GAIN),
            -- Q15.16
            REG_POS_MAX_VELOCITY => signed(REG_POS_MAX_VELOCITY),
            REG_POS_DEADBAND_TCK => unsigned(REG_POS_DEADBAND_TCK)
        );


    ---------------------------------------------------------------------------
    -- Velocity loop.  In position mode its target comes from the position loop;
    -- in velocity mode it comes directly from REG_TARGET_VEL[28:0].
    ---------------------------------------------------------------------------
    -- Q8.20
    w_target_vel_reg <= signed(REG_TARGET_VEL(28 downto 0));

    w_vel_loop_vld <= w_pos_vld when w_mode = C_MODE_POSITION
                      else w_tracker_vld when w_mode = C_MODE_VELOCITY
                      else '0';

    w_vel_target <= w_pos_omega when w_mode = C_MODE_POSITION
                    else w_target_vel_reg;

    u_velocity_cloop : entity work.velocity_cloop
        port map (
            i_clk               => i_clk,
            i_rst               => r_velocity_rst,
            i_vld               => w_vel_loop_vld,
            i_target_vel        => w_vel_target,
            i_actual_vel        => w_actual_vel,
            o_vld               => w_vel_vld,
            o_current_d         => w_vel_current_d,
            o_current_q         => w_vel_current_q,
            -- Q11.20
            REG_VEL_KP_GAIN     => signed(REG_VEL_KP_GAIN),
            -- Q7.24
            REG_VEL_KI_GAIN     => signed(REG_VEL_KI_GAIN),
            -- Q9.22
            REG_VEL_WINDUP_MAX  => signed(REG_VEL_WINDUP_MAX),
            -- Q9.22
            REG_VEL_CURRENT_MAX => signed(REG_VEL_CURRENT_MAX)
        );


    ---------------------------------------------------------------------------
    -- Current target mux
    --
    -- REG_TARGET_DQ[31:16] = Id target, signed Q3.12
    -- REG_TARGET_DQ[15:0]  = Iq target, signed Q3.12
    ---------------------------------------------------------------------------
    target_current_mux : process(all)
        variable v_target_d : t_current_q12;
        variable v_target_q : t_current_q12;
        variable v_limit    : t_current_q12;
    begin
        v_target_d := (others => '0');
        v_target_q := (others => '0');
        v_limit    := r_current_limit_q12;

        if w_mode = C_MODE_CURRENT then
            v_target_d := signed(REG_TARGET_DQ(31 downto 16));
            v_target_q := signed(REG_TARGET_DQ(15 downto 0));
        elsif w_mode = C_MODE_VELOCITY or w_mode = C_MODE_POSITION then
            -- Q3.12
            v_target_d := w_vel_current_d;
            -- Q3.12
            v_target_q := w_vel_current_q;
        end if;

        w_target_curr_d <= clamp_current_q12(v_target_d, v_limit);
        w_target_curr_q <= clamp_current_q12(v_target_q, v_limit);
    end process target_current_mux;


    ---------------------------------------------------------------------------
    -- Inner d/q current loop
    ---------------------------------------------------------------------------
    u_current_cloop : entity work.current_cloop
        port map (
            i_clk               => i_clk,
            i_rst               => r_current_rst,
            i_vld               => w_park_vld,
            i_target_curr_d     => w_target_curr_d,
            i_target_curr_q     => w_target_curr_q,
            i_actual_curr_d     => w_curr_d,
            i_actual_curr_q     => w_curr_q,
            o_vld               => w_foc_vld,
            o_ph_voltage_d      => w_voltage_d,
            o_ph_voltage_q      => w_voltage_q,
            REG_FOC_KP_GAIN     => signed(REG_FOC_KP_GAIN),
            REG_FOC_KI_GAIN     => signed(REG_FOC_KI_GAIN),
            REG_FOC_WINDUP_MAX  => signed(REG_FOC_WINDUP_MAX),
            REG_FOC_VOLTAGE_MAX => signed(REG_FOC_VOLTAGE_MAX)
        );


    ---------------------------------------------------------------------------
    -- Inverse Park / Clarke -> SVPWM
    ---------------------------------------------------------------------------
    u_park_inverse : entity work.park_inverse
        port map (
            i_clk   => i_clk,
            i_rst   => i_rst,
            i_vld   => w_foc_vld,
            i_d     => w_voltage_d,
            i_q     => w_voltage_q,
            i_cos   => r_cos_q14,
            i_sin   => r_sin_q14,
            o_vld   => w_inv_park_vld,
            o_alpha => w_voltage_alpha,
            o_beta  => w_voltage_beta
        );

    u_clarke_inverse : entity work.clarke_inverse
        port map (
            i_clk      => i_clk,
            i_rst      => i_rst,
            i_vld      => w_inv_park_vld,
            i_alpha    => w_voltage_alpha,
            i_beta     => w_voltage_beta,
            o_vld      => w_inv_clarke_vld,
            o_phases_A => w_voltage_a,
            o_phases_B => w_voltage_b,
            o_phases_C => w_voltage_c
        );

    -- Q5.12
    w_abc_voltage_a <= w_voltage_a;
    -- Q5.12
    w_abc_voltage_b <= w_voltage_b;
    -- Q5.12
    w_abc_voltage_c <= w_voltage_c;

    u_svpwm : entity work.svpwm
        port map (
            i_clk      => i_clk,
            i_rst      => i_rst,
            i_vld      => w_inv_clarke_vld,
            i_abc_volt_A => w_abc_voltage_a,
            i_abc_volt_B => w_abc_voltage_b,
            i_abc_volt_C => w_abc_voltage_c,
            o_vld      => open,
            o_abc_cnts_A => w_pwm_count_a,
            o_abc_cnts_B => w_pwm_count_b,
            o_abc_cnts_C => w_pwm_count_c
        );


    ---------------------------------------------------------------------------
    -- Sensor-presence / timeout fault
    ---------------------------------------------------------------------------
    sensor_watchdog : process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_have_encoder    <= '0';
                r_have_current    <= '0';
                r_sensor_fault    <= '0';
                r_encoder_timeout <= 0;
                r_current_timeout <= 0;

            else
                if w_clear_fault_pulse = '1' then
                    r_sensor_fault <= '0';
                end if;

                if w_encoder_vld = '1' then
                    r_have_encoder    <= '1';
                    r_encoder_timeout <= 0;
                else
                    if r_encoder_timeout = C_SENSOR_TIMEOUT_CLKS then
                        r_have_encoder <= '0';
                        r_sensor_fault <= '1';
                    else
                        r_encoder_timeout <= r_encoder_timeout + 1;
                    end if;
                end if;

                if w_current_vld = '1' then
                    r_have_current    <= '1';
                    r_current_timeout <= 0;
                else
                    if r_current_timeout = C_SENSOR_TIMEOUT_CLKS then
                        r_have_current <= '0';
                        r_sensor_fault <= '1';
                    else
                        r_current_timeout <= r_current_timeout + 1;
                    end if;
                end if;
            end if;
        end if;
    end process sensor_watchdog;


    ---------------------------------------------------------------------------
    -- Telemetry packing
    --
    -- STATUS:
    --   bit 0     motor drive enabled
    --   bit 1     at least one encoder sample received
    --   bit 2     at least one current sample received
    --   bit 3     latched sensor timeout fault
    --   bit 4     manual electrical angle mode
    --   bit 5     encoder direction invert
    --   bits 7:6  operating mode
    --   bit 8     current calibration active
    ---------------------------------------------------------------------------
    telemetry : process(all)
        variable v_status : std_logic_vector(31 downto 0);
    begin
        v_status := (others => '0');
        v_status(0) := w_drive_en;
        v_status(1) := r_have_encoder;
        v_status(2) := r_have_current;
        v_status(3) := r_sensor_fault;
        v_status(4) := r_manual_elec_mode;
        v_status(5) := r_sensor_inv;
        v_status(7 downto 6) := w_mode;
        v_status(8) := w_calibrating;
        w_status_data <= v_status;
    end process telemetry;

    w_actual_pos_data <= std_logic_vector(w_actual_pos);

    -- Q8.20
    w_actual_vel_data <= std_logic_vector(
        resize(w_actual_vel, 32)
    );

    -- Q3.12
    w_phase_ab_data <= std_logic_vector(w_phase_current_a) &
                       std_logic_vector(w_phase_current_b);

    -- Q3.12
    w_phase_c_id_data <= std_logic_vector(w_phase_current_c) &
                         std_logic_vector(w_curr_d);

    w_iq_elec_data <= std_logic_vector(w_curr_q) &
                      "00" &
                      std_logic_vector(w_elec_pos);

end architecture rtl;
