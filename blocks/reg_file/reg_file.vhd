library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

library work;
use work.reg_pkg.all;

entity register_file is
    port (
        -- System
        i_clk                  : in  std_logic;
        i_rst                  : in  std_logic;

        -- Request from UART parser
        -- i_req_rw = '0' -> read
        -- i_req_rw = '1' -> write
        i_req_vld              : in  std_logic;
        i_req_rw               : in  std_logic;
        i_req_data             : in  std_logic_vector(31 downto 0);
        i_req_addr             : in  std_logic_vector(7 downto 0);

        -- Read response to UART serializer
        o_rsp_vld              : out std_logic;
        o_rsp_data             : out std_logic_vector(31 downto 0);
        o_rsp_addr             : out std_logic_vector(7 downto 0);

        -- Writable controller registers
        REG_FOC_KP_GAIN        : out std_logic_vector(31 downto 0);
        REG_FOC_KI_GAIN        : out std_logic_vector(31 downto 0);
        REG_FOC_WINDUP_MAX     : out std_logic_vector(31 downto 0);
        REG_FOC_VOLTAGE_MAX    : out std_logic_vector(31 downto 0);
        REG_VEL_KP_GAIN        : out std_logic_vector(31 downto 0);
        REG_VEL_KI_GAIN        : out std_logic_vector(31 downto 0);
        REG_VEL_WINDUP_MAX     : out std_logic_vector(31 downto 0);
        REG_VEL_CURRENT_MAX    : out std_logic_vector(31 downto 0);
        REG_POS_KP_GAIN        : out std_logic_vector(31 downto 0);
        REG_POS_MAX_VELOCITY   : out std_logic_vector(31 downto 0);
        REG_POS_DEADBAND_TCK   : out std_logic_vector(31 downto 0);
        REG_CONTROL            : out std_logic_vector(31 downto 0);
        REG_TARGET_DQ          : out std_logic_vector(31 downto 0);
        REG_TARGET_VEL         : out std_logic_vector(31 downto 0);
        REG_TARGET_POS         : out std_logic_vector(31 downto 0);
        REG_MANUAL_ELEC        : out std_logic_vector(31 downto 0);

        -- One-clock command pulses
        o_clear_fault_pulse    : out std_logic;
        o_calibrate_pulse      : out std_logic;

        -- Read-only telemetry/status inputs
        i_status               : in  std_logic_vector(31 downto 0);
        i_actual_pos           : in  std_logic_vector(31 downto 0);
        i_actual_vel           : in  std_logic_vector(31 downto 0);
        i_phase_ab             : in  std_logic_vector(31 downto 0);
        i_phase_c_id           : in  std_logic_vector(31 downto 0);
        i_iq_elec              : in  std_logic_vector(31 downto 0);
        i_cal_off_ab           : in  std_logic_vector(31 downto 0);
        i_cal_off_c            : in  std_logic_vector(31 downto 0)
    );
end entity register_file;

architecture rtl of register_file is

    signal r_foc_kp_gain       : std_logic_vector(31 downto 0);
    signal r_foc_ki_gain       : std_logic_vector(31 downto 0);
    signal r_foc_windup_max    : std_logic_vector(31 downto 0);
    signal r_foc_voltage_max   : std_logic_vector(31 downto 0);
    signal r_vel_kp_gain       : std_logic_vector(31 downto 0);
    signal r_vel_ki_gain       : std_logic_vector(31 downto 0);
    signal r_vel_windup_max    : std_logic_vector(31 downto 0);
    signal r_vel_current_max   : std_logic_vector(31 downto 0);
    signal r_pos_kp_gain       : std_logic_vector(31 downto 0);
    signal r_pos_max_velocity  : std_logic_vector(31 downto 0);
    signal r_pos_deadband_tck  : std_logic_vector(31 downto 0);
    signal r_control           : std_logic_vector(31 downto 0);
    signal r_target_dq         : std_logic_vector(31 downto 0);
    signal r_target_vel        : std_logic_vector(31 downto 0);
    signal r_target_pos        : std_logic_vector(31 downto 0);
    signal r_manual_elec       : std_logic_vector(31 downto 0);

    signal w_read_data         : std_logic_vector(31 downto 0);

begin

    ---------------------------------------------------------------------------
    -- Register outputs
    ---------------------------------------------------------------------------
    REG_FOC_KP_GAIN      <= r_foc_kp_gain;
    REG_FOC_KI_GAIN      <= r_foc_ki_gain;
    REG_FOC_WINDUP_MAX   <= r_foc_windup_max;
    REG_FOC_VOLTAGE_MAX  <= r_foc_voltage_max;
    REG_VEL_KP_GAIN      <= r_vel_kp_gain;
    REG_VEL_KI_GAIN      <= r_vel_ki_gain;
    REG_VEL_WINDUP_MAX   <= r_vel_windup_max;
    REG_VEL_CURRENT_MAX  <= r_vel_current_max;
    REG_POS_KP_GAIN      <= r_pos_kp_gain;
    REG_POS_MAX_VELOCITY <= r_pos_max_velocity;
    REG_POS_DEADBAND_TCK <= r_pos_deadband_tck;
    REG_CONTROL          <= r_control;
    REG_TARGET_DQ        <= r_target_dq;
    REG_TARGET_VEL       <= r_target_vel;
    REG_TARGET_POS       <= r_target_pos;
    REG_MANUAL_ELEC      <= r_manual_elec;


    ---------------------------------------------------------------------------
    -- Combinational read mux
    ---------------------------------------------------------------------------
    read_mux : process(all)
    begin
        w_read_data <= (others => '0');

        case i_req_addr is
            when C_FOC_KP_GAIN_ADDR =>
                w_read_data <= r_foc_kp_gain;

            when C_FOC_KI_GAIN_ADDR =>
                w_read_data <= r_foc_ki_gain;

            when C_FOC_WINDUP_MAX =>
                w_read_data <= r_foc_windup_max;

            when C_FOC_VOLTAGE_MAX =>
                w_read_data <= r_foc_voltage_max;

            when C_VEL_KP_GAIN_ADDR =>
                w_read_data <= r_vel_kp_gain;

            when C_VEL_KI_GAIN_ADDR =>
                w_read_data <= r_vel_ki_gain;

            when C_VEL_WINDUP_MAX =>
                w_read_data <= r_vel_windup_max;

            when C_VEL_CURRENT_MAX =>
                w_read_data <= r_vel_current_max;

            when C_POS_KP_GAIN_ADDR =>
                w_read_data <= r_pos_kp_gain;

            when C_POS_MAX_VELOCITY =>
                w_read_data <= r_pos_max_velocity;

            when C_POS_DEADBAND_TCK =>
                w_read_data <= r_pos_deadband_tck;

            when C_CONTROL_ADDR =>
                w_read_data <= r_control;

            when C_TARGET_DQ_ADDR =>
                w_read_data <= r_target_dq;

            when C_TARGET_VEL_ADDR =>
                w_read_data <= r_target_vel;

            when C_TARGET_POS_ADDR =>
                w_read_data <= r_target_pos;

            when C_MANUAL_ELEC_ADDR =>
                w_read_data <= r_manual_elec;

            when C_STATUS_ADDR =>
                w_read_data <= i_status;

            when C_ACTUAL_POS_ADDR =>
                w_read_data <= i_actual_pos;

            when C_ACTUAL_VEL_ADDR =>
                w_read_data <= i_actual_vel;

            when C_PHASE_AB_ADDR =>
                w_read_data <= i_phase_ab;

            when C_PHASE_C_ID_ADDR =>
                w_read_data <= i_phase_c_id;

            when C_IQ_ELEC_ADDR =>
                w_read_data <= i_iq_elec;

            when C_CAL_OFF_AB_ADDR =>
                w_read_data <= i_cal_off_ab;

            when C_CAL_OFF_C_ADDR =>
                w_read_data <= i_cal_off_c;

            when others =>
                w_read_data <= (others => '0');
        end case;
    end process read_mux;


    ---------------------------------------------------------------------------
    -- Writable registers and read response
    --
    -- All registers reset to zero intentionally.  That makes the controller
    -- disabled after reset and requires the host to explicitly provide gains,
    -- limits, targets, and then set CONTROL.ENABLE.
    ---------------------------------------------------------------------------
    registers : process(i_clk)
        variable v_control : std_logic_vector(31 downto 0);
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_foc_kp_gain       <= (others => '0');
                r_foc_ki_gain       <= (others => '0');
                r_foc_windup_max    <= (others => '0');
                r_foc_voltage_max   <= (others => '0');
                r_vel_kp_gain       <= (others => '0');
                r_vel_ki_gain       <= (others => '0');
                r_vel_windup_max    <= (others => '0');
                r_vel_current_max   <= (others => '0');
                r_pos_kp_gain       <= (others => '0');
                r_pos_max_velocity  <= (others => '0');
                r_pos_deadband_tck  <= (others => '0');
                r_control           <= (others => '0');
                r_target_dq         <= (others => '0');
                r_target_vel        <= (others => '0');
                r_target_pos        <= (others => '0');
                r_manual_elec       <= (others => '0');

                o_rsp_vld           <= '0';
                o_rsp_data          <= (others => '0');
                o_rsp_addr          <= (others => '0');
                o_clear_fault_pulse <= '0';
                o_calibrate_pulse   <= '0';

            else
                o_rsp_vld           <= '0';
                o_clear_fault_pulse <= '0';
                o_calibrate_pulse   <= '0';

                if i_req_vld = '1' then
                    if i_req_rw = '1' then
                        case i_req_addr is
                            when C_FOC_KP_GAIN_ADDR =>
                                r_foc_kp_gain <= i_req_data;

                            when C_FOC_KI_GAIN_ADDR =>
                                r_foc_ki_gain <= i_req_data;

                            when C_FOC_WINDUP_MAX =>
                                r_foc_windup_max <= i_req_data;

                            when C_FOC_VOLTAGE_MAX =>
                                r_foc_voltage_max <= i_req_data;

                            when C_VEL_KP_GAIN_ADDR =>
                                r_vel_kp_gain <= i_req_data;

                            when C_VEL_KI_GAIN_ADDR =>
                                r_vel_ki_gain <= i_req_data;

                            when C_VEL_WINDUP_MAX =>
                                r_vel_windup_max <= i_req_data;

                            when C_VEL_CURRENT_MAX =>
                                r_vel_current_max <= i_req_data;

                            when C_POS_KP_GAIN_ADDR =>
                                r_pos_kp_gain <= i_req_data;

                            when C_POS_MAX_VELOCITY =>
                                r_pos_max_velocity <= i_req_data;

                            when C_POS_DEADBAND_TCK =>
                                r_pos_deadband_tck <= i_req_data;

                            when C_CONTROL_ADDR =>
                                v_control := i_req_data;

                                if i_req_data(C_CTRL_CALIBRATE_BIT) = '1' then
                                    o_calibrate_pulse <= '1';
                                    v_control(C_CTRL_ENABLE_BIT) := '0';
                                end if;

                                if i_req_data(C_CTRL_CLEAR_FAULT_BIT) = '1' then
                                    o_clear_fault_pulse <= '1';
                                end if;

                                v_control(C_CTRL_CALIBRATE_BIT)   := '0';
                                v_control(C_CTRL_CLEAR_FAULT_BIT) := '0';
                                r_control <= v_control;

                            when C_TARGET_DQ_ADDR =>
                                r_target_dq <= i_req_data;

                            when C_TARGET_VEL_ADDR =>
                                r_target_vel <= i_req_data;

                            when C_TARGET_POS_ADDR =>
                                r_target_pos <= i_req_data;

                            when C_MANUAL_ELEC_ADDR =>
                                r_manual_elec <= i_req_data;

                            -- Read-only / invalid addresses ignore writes.
                            when others =>
                                null;
                        end case;

                    else
                        o_rsp_vld  <= '1';
                        o_rsp_data <= w_read_data;
                        o_rsp_addr <= i_req_addr;
                    end if;
                end if;
            end if;
        end if;
    end process registers;

end architecture rtl;
