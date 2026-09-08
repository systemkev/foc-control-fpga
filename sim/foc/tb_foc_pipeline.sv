`timescale 1ns/1ps

// =============================================================================
// Self-checking regression for:
//   motor_current_reader -> Clarke -> Park -> current_cloop
//   -> inverse Park -> inverse Clarke -> SVPWM
//
// Intended for a mixed-language simulator (Questa/ModelSim, Xcelium, Riviera,
// or Vivado xsim mixed-language flow) with the VHDL RTL compiled into WORK.
//
// The xadc_wiz_0 module below is a simulation replacement for the Xilinx XADC
// wizard instantiated as a VHDL component by motor_current_reader.
// =============================================================================

package xadc_model_pkg;
    // Programmable raw 16-bit XADC conversion values.
    int unsigned adc_raw_a = 16'h8000;
    int unsigned adc_raw_b = 16'h8000;

    // Fault/ordering controls used by directed tests.
    bit start_with_b          = 1'b0;
    bit inject_bad_chan_once  = 1'b0;
    bit drop_next_drdy        = 1'b0;
endpackage


// =============================================================================
// XADC wizard simulation model
// =============================================================================
module xadc_wiz_0 (
    input  logic [6:0]  daddr_in,
    input  logic        den_in,
    input  logic [15:0] di_in,
    input  logic        dwe_in,
    output logic [15:0] do_out,
    output logic        drdy_out,
    input  logic        dclk_in,
    input  logic        reset_in,
    input  logic        convst_in,
    input  logic        vauxp4,
    input  logic        vauxn4,
    input  logic        vauxp6,
    input  logic        vauxn6,
    output logic        busy_out,
    output logic [4:0]  channel_out,
    output logic        eoc_out,
    output logic        eos_out,
    output logic        alarm_out,
    input  logic        vp_in,
    input  logic        vn_in
);
    import xadc_model_pkg::*;

    localparam logic [4:0] CH_VAUX4 = 5'h14;
    localparam logic [4:0] CH_VAUX6 = 5'h16;

    int  conv_countdown;
    bit  active;
    bit  next_is_b;
    bit  pending_is_b;
    bit  pending_bad_chan;
    int unsigned pending_raw;

    always @(posedge dclk_in) begin
        // One-cycle pulses by default.
        eoc_out   <= 1'b0;
        drdy_out  <= 1'b0;
        eos_out   <= 1'b0;
        alarm_out <= 1'b0;

        if (reset_in) begin
            do_out          <= '0;
            busy_out        <= 1'b0;
            channel_out     <= CH_VAUX4;
            conv_countdown  <= 0;
            active          <= 1'b0;
            next_is_b       <= start_with_b;
            pending_is_b    <= 1'b0;
            pending_bad_chan<= 1'b0;
            pending_raw     <= 0;
        end
        else begin
            if (!active && convst_in) begin
                active           <= 1'b1;
                busy_out         <= 1'b1;
                conv_countdown   <= 2;
                pending_is_b     <= next_is_b;
                pending_raw      <= next_is_b ? adc_raw_b : adc_raw_a;
                pending_bad_chan <= inject_bad_chan_once;
                next_is_b        <= ~next_is_b;

                if (inject_bad_chan_once)
                    inject_bad_chan_once = 1'b0;
            end
            else if (active) begin
                case (conv_countdown)
                    2: begin
                        // Present channel and EOC. motor_current_reader captures
                        // daddr on the following clock edge.
                        channel_out <= pending_bad_chan ? 5'h00 :
                                       (pending_is_b ? CH_VAUX6 : CH_VAUX4);
                        eoc_out        <= 1'b1;
                        conv_countdown <= 1;
                    end

                    1: begin
                        // Present data first, then pulse DRDY for a full cycle.
                        do_out <= pending_raw[15:0];
                        if (drop_next_drdy) begin
                            drop_next_drdy = 1'b0;
                        end
                        else begin
                            drdy_out <= 1'b1;
                        end

                        busy_out       <= 1'b0;
                        active         <= 1'b0;
                        conv_countdown <= 0;
                    end

                    default: begin
                        active         <= 1'b0;
                        busy_out       <= 1'b0;
                        conv_countdown <= 0;
                    end
                endcase
            end
        end
    end

    // These ports are deliberately unused by the behavioral model.
    wire _unused = &{1'b0, daddr_in, den_in, di_in, dwe_in,
                     vauxp4, vauxn4, vauxp6, vauxn6, vp_in, vn_in};
endmodule


// =============================================================================
// Main testbench
// =============================================================================
module tb_foc_pipeline;
    import xadc_model_pkg::*;

    parameter bit RUN_LONG_READER_TESTS = 1'b1;
    parameter bit RUN_PROTOCOL_STRESS   = 1'b0;
    parameter bit RUN_ANGLE_TAG_STRESS  = 1'b0;
    parameter int RANDOM_ITERS          = 100;

    localparam time CLK_PERIOD = 20ns; // common_pkg says 50 MHz

    localparam int ADC_SCALE          = 27034;
    localparam int ADC_OFFSET_DEFAULT = 13517;
    localparam int CAL_SAMPLES        = 256;
    localparam int PWM_PERIOD_COUNTS  = 2500; // 50 MHz / 20 kHz
    localparam int PWM_HALF_COUNTS    = 1250;
    localparam int V_TO_CNT_Q12       = 853333; // integer((2500/12)*2^12)

    logic clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    logic rst;

    // -------------------------------------------------------------------------
    // motor_current_reader interface
    // -------------------------------------------------------------------------
    logic pwm_center_trigger;
    logic calibrate;
    logic reader_vld;
    logic signed [15:0] reader_a, reader_b, reader_c;
    logic reader_calibrating;
    logic [15:0] reader_off_a, reader_off_b;

    // Physical XADC pins are not used by the behavioral xadc_wiz_0 model.
    logic vauxp4, vauxn4, vauxp6, vauxn6;

    motor_current_reader u_reader (
        .i_clk                (clk),
        .i_reset              (rst),
        .i_pwm_center_trigger (pwm_center_trigger),
        .i_calibrate          (calibrate),
        .i_top_pin_vauxp4     (vauxp4),
        .i_top_pin_vauxn4     (vauxn4),
        .i_top_pin_vauxp6     (vauxp6),
        .i_top_pin_vauxn6     (vauxn6),
        .o_vld                (reader_vld),
        .o_phases_A           (reader_a),
        .o_phases_B           (reader_b),
        .o_phases_C           (reader_c),
        .o_calibrating        (reader_calibrating),
        .o_cal_off_a          (reader_off_a),
        .o_cal_off_b          (reader_off_b)
    );

    // -------------------------------------------------------------------------
    // Source mux: reader-driven full-chain testing or direct current injection.
    // Direct mode is intentionally included to stress valid throughput and
    // mathematical corner cases that the serial XADC path cannot generate.
    // -------------------------------------------------------------------------
    logic use_direct;
    logic direct_vld;
    logic signed [15:0] direct_a, direct_b, direct_c;

    wire src_vld = use_direct ? direct_vld : reader_vld;
    wire signed [15:0] src_a = use_direct ? direct_a : reader_a;
    wire signed [15:0] src_b = use_direct ? direct_b : reader_b;
    wire signed [15:0] src_c = use_direct ? direct_c : reader_c;

    // Trig is provided directly because CORDIC is outside the requested chain.
    logic signed [15:0] trig_cos_q14, trig_sin_q14;

    // Current-loop inputs/configuration.
    logic signed [15:0] target_d_q12, target_q_q12;
    logic signed [31:0] reg_kp_q24;
    logic signed [31:0] reg_ki_q28;
    logic signed [31:0] reg_windup_q20;
    logic signed [31:0] reg_vmax_q20;

    // -------------------------------------------------------------------------
    // Clarke
    // -------------------------------------------------------------------------
    logic clarke_vld;
    logic signed [15:0] clarke_alpha, clarke_beta;

    clarke u_clarke (
        .i_clk          (clk),
        .i_rst          (rst),
        .i_vld          (src_vld),
        .i_phases_A     (src_a),
        .i_phases_B     (src_b),
        .i_phases_C     (src_c),
        .o_vld          (clarke_vld),
        .o_clarke_alpha (clarke_alpha),
        .o_clarke_beta  (clarke_beta)
    );

    // -------------------------------------------------------------------------
    // Park
    // -------------------------------------------------------------------------
    logic park_vld;
    logic signed [15:0] park_d, park_q;

    park u_park (
        .i_clk   (clk),
        .i_rst   (rst),
        .i_vld   (clarke_vld),
        .i_alpha (clarke_alpha),
        .i_beta  (clarke_beta),
        .i_cos   (trig_cos_q14),
        .i_sin   (trig_sin_q14),
        .o_vld   (park_vld),
        .o_d     (park_d),
        .o_q     (park_q)
    );

    // -------------------------------------------------------------------------
    // Current PI loop
    // -------------------------------------------------------------------------
    logic cloop_vld;
    logic signed [17:0] voltage_d, voltage_q;

    current_cloop u_current_cloop (
        .i_clk               (clk),
        .i_rst               (rst),
        .i_vld               (park_vld),
        .i_target_curr_d     (target_d_q12),
        .i_target_curr_q     (target_q_q12),
        .i_actual_curr_d     (park_d),
        .i_actual_curr_q     (park_q),
        .o_vld               (cloop_vld),
        .o_ph_voltage_d      (voltage_d),
        .o_ph_voltage_q      (voltage_q),
        .REG_FOC_KP_GAIN     (reg_kp_q24),
        .REG_FOC_KI_GAIN     (reg_ki_q28),
        .REG_FOC_WINDUP_MAX  (reg_windup_q20),
        .REG_FOC_VOLTAGE_MAX (reg_vmax_q20)
    );

    // -------------------------------------------------------------------------
    // Inverse Park
    // -------------------------------------------------------------------------
    logic ipark_vld;
    logic signed [17:0] voltage_alpha, voltage_beta;

    park_inverse u_park_inverse (
        .i_clk   (clk),
        .i_rst   (rst),
        .i_vld   (cloop_vld),
        .i_d     (voltage_d),
        .i_q     (voltage_q),
        .i_cos   (trig_cos_q14),
        .i_sin   (trig_sin_q14),
        .o_vld   (ipark_vld),
        .o_alpha (voltage_alpha),
        .o_beta  (voltage_beta)
    );

    // -------------------------------------------------------------------------
    // Inverse Clarke
    // -------------------------------------------------------------------------
    logic iclarke_vld;
    logic signed [17:0] voltage_a, voltage_b, voltage_c;

    clarke_inverse u_clarke_inverse (
        .i_clk      (clk),
        .i_rst      (rst),
        .i_vld      (ipark_vld),
        .i_alpha    (voltage_alpha),
        .i_beta     (voltage_beta),
        .o_vld      (iclarke_vld),
        .o_phases_A (voltage_a),
        .o_phases_B (voltage_b),
        .o_phases_C (voltage_c)
    );

    // -------------------------------------------------------------------------
    // SVPWM
    // -------------------------------------------------------------------------
    logic svpwm_vld;
    integer pwm_a, pwm_b, pwm_c;

    svpwm u_svpwm (
        .i_clk        (clk),
        .i_rst        (rst),
        .i_vld        (iclarke_vld),
        .i_abc_volt_A (voltage_a),
        .i_abc_volt_B (voltage_b),
        .i_abc_volt_C (voltage_c),
        .o_vld        (svpwm_vld),
        .o_abc_cnts_A (pwm_a),
        .o_abc_cnts_B (pwm_b),
        .o_abc_cnts_C (pwm_c)
    );

    // =========================================================================
    // Scoreboard data structures
    // =========================================================================
    typedef struct {
        int id;
        int due_cycle;

        int signed a, b, c;
        int signed alpha, beta;
        int signed d, q;
        int signed vd, vq;
        int signed valpha, vbeta;
        int signed va, vb, vc;
        int signed pwm_a, pwm_b, pwm_c;

        int signed cos_q14, sin_q14;
        int signed target_d, target_q;
        int signed kp, ki, windup, vmax;
    } txn_t;

    typedef struct {
        int id;
        int signed a, b, c;
    } reader_txn_t;

    txn_t q_clarke[$];
    txn_t q_park[$];
    txn_t q_cloop[$];
    txn_t q_ipark[$];
    txn_t q_iclarke[$];
    txn_t q_svpwm[$];
    reader_txn_t q_reader[$];

    int cycle_count;
    int next_txn_id;
    int next_reader_id;
    int checks;
    int errors;
    int warnings;
    int tests_run;
    int tests_passed;
    int test_error_base;
    string current_test;

    // Bit-exact PI reference state.
    longint signed ref_int_d;
    longint signed ref_int_q;

    // Bit-exact current-reader calibration state.
    int unsigned ref_off_a, ref_off_b;
    longint unsigned ref_cal_sum_a, ref_cal_sum_b;
    int ref_cal_count;
    bit ref_cal_active;

    bit reader_vld_prev;

    // =========================================================================
    // Utility / fixed-point reference helpers
    // =========================================================================
    function automatic longint signed wrap_s(
        input longint signed x,
        input int bits
    );
        longint unsigned mask;
        longint unsigned u;
        longint unsigned sign_bit;
        begin
            mask     = (64'h1 << bits) - 1;
            sign_bit = 64'h1 << (bits-1);
            u        = $unsigned(x) & mask;
            if ((u & sign_bit) != 0)
                wrap_s = $signed(u | ~mask);
            else
                wrap_s = $signed(u);
        end
    endfunction

    function automatic longint signed sat_s(
        input longint signed x,
        input longint signed lo,
        input longint signed hi
    );
        if (x > hi)      sat_s = hi;
        else if (x < lo) sat_s = lo;
        else             sat_s = x;
    endfunction

    function automatic int signed q12(input real x);
        real scaled;
        begin
            scaled = x * 4096.0;
            q12 = (scaled >= 0.0) ? $rtoi(scaled + 0.5) : $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic int signed q14(input real x);
        real scaled;
        begin
            scaled = x * 16384.0;
            q14 = (scaled >= 0.0) ? $rtoi(scaled + 0.5) : $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic int signed q7_24(input real x);
        real scaled;
        begin
            scaled = x * 16777216.0;
            q7_24 = (scaled >= 0.0) ? $rtoi(scaled + 0.5) : $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic int signed q3_28(input real x);
        real scaled;
        begin
            scaled = x * 268435456.0;
            q3_28 = (scaled >= 0.0) ? $rtoi(scaled + 0.5) : $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic int signed q11_20(input real x);
        real scaled;
        begin
            scaled = x * 1048576.0;
            q11_20 = (scaled >= 0.0) ? $rtoi(scaled + 0.5) : $rtoi(scaled - 0.5);
        end
    endfunction

    function automatic int unsigned adc_scaled(input int unsigned raw);
        longint unsigned p;
        begin
            p = longint'(raw) * ADC_SCALE;
            adc_scaled = int'(p >> 16);
        end
    endfunction

    function automatic int unsigned raw_for_scaled(input int signed scaled_q12);
        // Finds a raw XADC code whose hardware scale result is close to the
        // requested post-scale unsigned code. Used only for human-readable
        // stimulus construction; the scoreboard always models the raw value.
        longint signed guess;
        begin
            guess = (longint'(scaled_q12) << 16) / ADC_SCALE;
            if (guess < 0) guess = 0;
            if (guess > 65535) guess = 65535;
            raw_for_scaled = int'(guess);
        end
    endfunction

    task automatic set_angle_deg(input real deg);
        real rad;
        int signed c, s;
        begin
            rad = deg * 3.14159265358979323846 / 180.0;
            c = q14($cos(rad));
            s = q14($sin(rad));
            if (c >  32767) c =  32767;
            if (c < -32768) c = -32768;
            if (s >  32767) s =  32767;
            if (s < -32768) s = -32768;
            @(negedge clk);
            trig_cos_q14 = c;
            trig_sin_q14 = s;
        end
    endtask

    task automatic set_gains(
        input real kp,
        input real ki,
        input real windup_v,
        input real vmax_v
    );
        begin
            @(negedge clk);
            reg_kp_q24      = q7_24(kp);
            reg_ki_q28      = q3_28(ki);
            reg_windup_q20  = q11_20(windup_v);
            reg_vmax_q20    = q11_20(vmax_v);
        end
    endtask

    task automatic set_targets(input real d_a, input real q_a);
        begin
            @(negedge clk);
            target_d_q12 = q12(d_a);
            target_q_q12 = q12(q_a);
        end
    endtask

    // =========================================================================
    // Reference models for each pipeline stage
    // =========================================================================
    task automatic model_clarke(inout txn_t t);
        longint signed alpha_acc, beta_acc;
        begin
            alpha_acc = longint'(t.a) *  699051
                      + longint'(t.b) * -349525
                      + longint'(t.c) * -349525;
            beta_acc  = longint'(t.b) *  605396
                      + longint'(t.c) * -605396;

            t.alpha = int'(sat_s(alpha_acc >>> 20, -20480, 20480));
            t.beta  = int'(sat_s(beta_acc  >>> 20, -20480, 20480));
        end
    endtask

    task automatic model_park(inout txn_t t);
        longint signed d_acc, q_acc;
        begin
            d_acc = longint'(t.alpha) * t.cos_q14
                  + longint'(t.beta)  * t.sin_q14;
            q_acc = longint'(t.beta)  * t.cos_q14
                  - longint'(t.alpha) * t.sin_q14;

            t.d = int'(sat_s(d_acc >>> 14, -32768, 32767));
            t.q = int'(sat_s(q_acc >>> 14, -32768, 32767));
        end
    endtask

    task automatic model_current_cloop(inout txn_t t);
        longint signed err_d, err_q;
        longint signed prop_d, prop_q;
        longint signed igain_d, igain_q;
        longint signed wind;
        longint signed vmax;
        longint signed sum_d, sum_q;
        begin
            // 17-bit error range exactly covers all 16-bit target-actual cases.
            err_d = longint'(t.target_d) - longint'(t.d);
            err_q = longint'(t.target_q) - longint'(t.q);

            prop_d  = sat_s((err_d * longint'(t.kp)) >>> 24, -131072, 131071);
            prop_q  = sat_s((err_q * longint'(t.kp)) >>> 24, -131072, 131071);
            igain_d = sat_s((err_d * longint'(t.ki)) >>> 28, -131072, 131071);
            igain_q = sat_s((err_q * longint'(t.ki)) >>> 28, -131072, 131071);

            wind = sat_s(longint'(t.windup) >>> 8, -131072, 131071);

            sum_d = ref_int_d + igain_d;
            sum_q = ref_int_q + igain_q;

            if (sum_d > wind)       ref_int_d = wind;
            else if (sum_d < -wind) ref_int_d = -wind;
            else                    ref_int_d = sum_d;

            if (sum_q > wind)       ref_int_q = wind;
            else if (sum_q < -wind) ref_int_q = -wind;
            else                    ref_int_q = sum_q;

            vmax = sat_s(longint'(t.vmax) >>> 8, -131072, 131071);

            t.vd = int'(sat_s(ref_int_d + prop_d, -vmax, vmax));
            t.vq = int'(sat_s(ref_int_q + prop_q, -vmax, vmax));
        end
    endtask

    task automatic model_inverse_park(inout txn_t t);
        longint signed alpha_acc, beta_acc;
        begin
            alpha_acc = longint'(t.vd) * t.cos_q14
                      - longint'(t.vq) * t.sin_q14;
            beta_acc  = longint'(t.vd) * t.sin_q14
                      + longint'(t.vq) * t.cos_q14;

            t.valpha = int'(sat_s(alpha_acc >>> 14, -131072, 131071));
            t.vbeta  = int'(sat_s(beta_acc  >>> 14, -131072, 131071));
        end
    endtask

    task automatic model_inverse_clarke(inout txn_t t);
        longint signed b_acc, c_acc;
        begin
            // A is an exact pass-through in the RTL.
            t.va = t.valpha;

            b_acc = longint'(t.valpha) * -524288
                  + longint'(t.vbeta)  *  908093;
            c_acc = longint'(t.valpha) * -524288
                  + longint'(t.vbeta)  * -908093;

            t.vb = int'(sat_s(b_acc >>> 20, -131072, 131071));
            t.vc = int'(sat_s(c_acc >>> 20, -131072, 131071));
        end
    endtask

    task automatic model_svpwm(inout txn_t t);
        longint signed vmax, vmin;
        longint signed sum18, neg18, offset18;
        longint signed a_star, b_star, c_star;
        longint signed cnt_a, cnt_b, cnt_c;
        begin
            vmax = t.va;
            if (t.vb > vmax) vmax = t.vb;
            if (t.vc > vmax) vmax = t.vc;

            vmin = t.va;
            if (t.vb < vmin) vmin = t.vb;
            if (t.vc < vmin) vmin = t.vc;

            // Important: numeric_std +/- on equal-width SIGNED operands keeps
            // the operand width. Model the 18-bit intermediate wrap exactly.
            sum18    = wrap_s(vmax + vmin, 18);
            neg18    = wrap_s(-sum18, 18);
            offset18 = neg18 >>> 1;

            a_star = wrap_s(longint'(t.va) + offset18, 18);
            b_star = wrap_s(longint'(t.vb) + offset18, 18);
            c_star = wrap_s(longint'(t.vc) + offset18, 18);

            cnt_a = ((a_star * V_TO_CNT_Q12) >>> 24) + PWM_HALF_COUNTS;
            cnt_b = ((b_star * V_TO_CNT_Q12) >>> 24) + PWM_HALF_COUNTS;
            cnt_c = ((c_star * V_TO_CNT_Q12) >>> 24) + PWM_HALF_COUNTS;

            t.pwm_a = int'(sat_s(cnt_a, 0, PWM_PERIOD_COUNTS));
            t.pwm_b = int'(sat_s(cnt_b, 0, PWM_PERIOD_COUNTS));
            t.pwm_c = int'(sat_s(cnt_c, 0, PWM_PERIOD_COUNTS));
        end
    endtask

    // =========================================================================
    // Current-reader bit-exact reference model
    // =========================================================================
    task automatic reader_ref_reset;
        begin
            ref_off_a      = ADC_OFFSET_DEFAULT;
            ref_off_b      = ADC_OFFSET_DEFAULT;
            ref_cal_sum_a  = 0;
            ref_cal_sum_b  = 0;
            ref_cal_count  = 0;
            ref_cal_active = 1'b1;
        end
    endtask

    task automatic reader_ref_restart_cal;
        begin
            ref_cal_sum_a  = 0;
            ref_cal_sum_b  = 0;
            ref_cal_count  = 0;
            ref_cal_active = 1'b1;
        end
    endtask

    task automatic reader_queue_expected(
        input int unsigned raw_a,
        input int unsigned raw_b
    );
        reader_txn_t r;
        longint signed sa, sb;
        longint signed ia, ib, ic17;
        longint unsigned suma, sumb;
        begin
            sa = adc_scaled(raw_a);
            sb = adc_scaled(raw_b);

            ia = sat_s(sa - ref_off_a, -32768, 32767);
            ib = sat_s(ref_off_b - sb, -32768, 32767);

            // The RTL computes C in a 17-bit signed temporary before saturation.
            ic17 = wrap_s(-ia - ib, 17);

            r.id = next_reader_id++;
            r.a  = int'(ia);
            r.b  = int'(ib);
            r.c  = int'(sat_s(ic17, -32768, 32767));
            q_reader.push_back(r);

            if (ref_cal_active) begin
                suma = ref_cal_sum_a + sa;
                sumb = ref_cal_sum_b + sb;

                if (ref_cal_count == CAL_SAMPLES-1) begin
                    ref_off_a      = int'(suma >> 8);
                    ref_off_b      = int'(sumb >> 8);
                    ref_cal_sum_a  = 0;
                    ref_cal_sum_b  = 0;
                    ref_cal_count  = 0;
                    ref_cal_active = 1'b0;
                end
                else begin
                    ref_cal_sum_a = suma;
                    ref_cal_sum_b = sumb;
                    ref_cal_count++;
                end
            end
        end
    endtask

    // =========================================================================
    // Scoreboard comparison helpers
    // =========================================================================
    task automatic fail(input string what, input string detail);
        begin
            errors++;
            $error("[FAIL][%s][cycle=%0d] %s: %s", current_test, cycle_count, what, detail);
        end
    endtask

    task automatic warn(input string what, input string detail);
        begin
            warnings++;
            $warning("[WARN][%s][cycle=%0d] %s: %s", current_test, cycle_count, what, detail);
        end
    endtask

    task automatic cmp_i(
        input string what,
        input int signed got,
        input int signed exp,
        input int id
    );
        begin
            checks++;
            if (got !== exp)
                fail(what, $sformatf("txn=%0d got=%0d (0x%0h), expected=%0d (0x%0h)",
                                    id, got, got, exp, exp));
        end
    endtask

    task automatic check_due(input string stage, input int got_cycle, input txn_t t);
        begin
            checks++;
            if (got_cycle != t.due_cycle)
                fail($sformatf("%s.latency", stage),
                     $sformatf("txn=%0d output cycle=%0d expected cycle=%0d",
                               t.id, got_cycle, t.due_cycle));
        end
    endtask

    task automatic check_clarke_stage;
        txn_t t;
        begin
            if ($isunknown(clarke_vld)) begin
                fail("clarke.o_vld", "X/Z on valid");
            end
            else if (clarke_vld) begin
                if (q_clarke.size() == 0) begin
                    fail("clarke.o_vld", "unexpected valid with empty expected queue");
                end
                else begin
                    t = q_clarke.pop_front();
                    check_due("clarke", cycle_count, t);
                    cmp_i("clarke.alpha", $signed(clarke_alpha), t.alpha, t.id);
                    cmp_i("clarke.beta",  $signed(clarke_beta),  t.beta,  t.id);

                    model_park(t);
                    t.due_cycle = cycle_count + 2;
                    q_park.push_back(t);
                end
            end
        end
    endtask

    task automatic check_park_stage;
        txn_t t;
        begin
            if ($isunknown(park_vld)) begin
                fail("park.o_vld", "X/Z on valid");
            end
            else if (park_vld) begin
                if (q_park.size() == 0) begin
                    fail("park.o_vld", "unexpected valid with empty expected queue");
                end
                else begin
                    t = q_park.pop_front();
                    check_due("park", cycle_count, t);
                    cmp_i("park.d", $signed(park_d), t.d, t.id);
                    cmp_i("park.q", $signed(park_q), t.q, t.id);

                    model_current_cloop(t);
                    t.due_cycle = cycle_count + 5;
                    q_cloop.push_back(t);
                end
            end
        end
    endtask

    task automatic check_cloop_stage;
        txn_t t;
        begin
            if ($isunknown(cloop_vld)) begin
                fail("current_cloop.o_vld", "X/Z on valid");
            end
            else if (cloop_vld) begin
                if (q_cloop.size() == 0) begin
                    fail("current_cloop.o_vld", "unexpected valid with empty expected queue");
                end
                else begin
                    t = q_cloop.pop_front();
                    check_due("current_cloop", cycle_count, t);
                    cmp_i("current_cloop.vd", $signed(voltage_d), t.vd, t.id);
                    cmp_i("current_cloop.vq", $signed(voltage_q), t.vq, t.id);

                    model_inverse_park(t);
                    t.due_cycle = cycle_count + 2;
                    q_ipark.push_back(t);
                end
            end
        end
    endtask

    task automatic check_ipark_stage;
        txn_t t;
        begin
            if ($isunknown(ipark_vld)) begin
                fail("park_inverse.o_vld", "X/Z on valid");
            end
            else if (ipark_vld) begin
                if (q_ipark.size() == 0) begin
                    fail("park_inverse.o_vld", "unexpected valid with empty expected queue");
                end
                else begin
                    t = q_ipark.pop_front();
                    check_due("park_inverse", cycle_count, t);
                    cmp_i("park_inverse.alpha", $signed(voltage_alpha), t.valpha, t.id);
                    cmp_i("park_inverse.beta",  $signed(voltage_beta),  t.vbeta,  t.id);

                    model_inverse_clarke(t);
                    t.due_cycle = cycle_count + 2;
                    q_iclarke.push_back(t);
                end
            end
        end
    endtask

    task automatic check_iclarke_stage;
        txn_t t;
        begin
            if ($isunknown(iclarke_vld)) begin
                fail("clarke_inverse.o_vld", "X/Z on valid");
            end
            else if (iclarke_vld) begin
                if (q_iclarke.size() == 0) begin
                    fail("clarke_inverse.o_vld", "unexpected valid with empty expected queue");
                end
                else begin
                    t = q_iclarke.pop_front();
                    check_due("clarke_inverse", cycle_count, t);
                    cmp_i("clarke_inverse.A", $signed(voltage_a), t.va, t.id);
                    cmp_i("clarke_inverse.B", $signed(voltage_b), t.vb, t.id);
                    cmp_i("clarke_inverse.C", $signed(voltage_c), t.vc, t.id);

                    model_svpwm(t);
                    t.due_cycle = cycle_count + 4;
                    q_svpwm.push_back(t);
                end
            end
        end
    endtask

    task automatic check_svpwm_stage;
        txn_t t;
        begin
            if ($isunknown(svpwm_vld)) begin
                fail("svpwm.o_vld", "X/Z on valid");
            end
            else if (svpwm_vld) begin
                if (q_svpwm.size() == 0) begin
                    fail("svpwm.o_vld", "unexpected valid with empty expected queue");
                end
                else begin
                    t = q_svpwm.pop_front();
                    check_due("svpwm", cycle_count, t);
                    cmp_i("svpwm.countA", pwm_a, t.pwm_a, t.id);
                    cmp_i("svpwm.countB", pwm_b, t.pwm_b, t.id);
                    cmp_i("svpwm.countC", pwm_c, t.pwm_c, t.id);

                    checks += 3;
                    if ((pwm_a < 0) || (pwm_a > PWM_PERIOD_COUNTS))
                        fail("svpwm.rangeA", $sformatf("count=%0d", pwm_a));
                    if ((pwm_b < 0) || (pwm_b > PWM_PERIOD_COUNTS))
                        fail("svpwm.rangeB", $sformatf("count=%0d", pwm_b));
                    if ((pwm_c < 0) || (pwm_c > PWM_PERIOD_COUNTS))
                        fail("svpwm.rangeC", $sformatf("count=%0d", pwm_c));
                end
            end
        end
    endtask

    task automatic check_reader_stage;
        reader_txn_t r;
        longint signed sumabc;
        begin
            if ($isunknown(reader_vld)) begin
                fail("reader.o_vld", "X/Z on valid");
            end

            if (reader_vld && reader_vld_prev)
                fail("reader.o_vld", "valid wider than one cycle");

            if (reader_vld) begin
                if (q_reader.size() == 0) begin
                    fail("reader.o_vld", "unexpected valid with empty expected queue");
                end
                else begin
                    r = q_reader.pop_front();
                    cmp_i("reader.A", $signed(reader_a), r.a, r.id);
                    cmp_i("reader.B", $signed(reader_b), r.b, r.id);
                    cmp_i("reader.C", $signed(reader_c), r.c, r.id);

                    // When C itself is not saturated, A+B+C must be exactly zero.
                    sumabc = longint'($signed(reader_a))
                           + longint'($signed(reader_b))
                           + longint'($signed(reader_c));
                    if ((r.c != 32767) && (r.c != -32768)) begin
                        checks++;
                        if (sumabc != 0)
                            fail("reader.KCL", $sformatf("A+B+C=%0d", sumabc));
                    end
                end
            end

            reader_vld_prev = reader_vld;
        end
    endtask

    task automatic scoreboard_clear;
        begin
            q_clarke.delete();
            q_park.delete();
            q_cloop.delete();
            q_ipark.delete();
            q_iclarke.delete();
            q_svpwm.delete();
            q_reader.delete();
            ref_int_d = 0;
            ref_int_q = 0;
            reader_vld_prev = 1'b0;
            reader_ref_reset();
        end
    endtask

    function automatic bit pipeline_empty;
        pipeline_empty = (q_clarke.size()  == 0) &&
                         (q_park.size()    == 0) &&
                         (q_cloop.size()   == 0) &&
                         (q_ipark.size()   == 0) &&
                         (q_iclarke.size() == 0) &&
                         (q_svpwm.size()   == 0) &&
                         !src_vld && !clarke_vld && !park_vld && !cloop_vld &&
                         !ipark_vld && !iclarke_vld && !svpwm_vld;
    endfunction

    // =========================================================================
    // Main monitor: sample source BEFORE sequential updates, then outputs after
    // one delta/timeprecision step so VHDL registered outputs are settled.
    // =========================================================================
    always @(posedge clk) begin : p_scoreboard
        txn_t t;
        bit source_accept;

        cycle_count++;
        source_accept = src_vld;

        if (rst) begin
            scoreboard_clear();
        end
        else if (source_accept) begin
            t.id         = next_txn_id++;
            t.a          = $signed(src_a);
            t.b          = $signed(src_b);
            t.c          = $signed(src_c);
            t.cos_q14    = $signed(trig_cos_q14);
            t.sin_q14    = $signed(trig_sin_q14);
            t.target_d   = $signed(target_d_q12);
            t.target_q   = $signed(target_q_q12);
            t.kp         = $signed(reg_kp_q24);
            t.ki         = $signed(reg_ki_q28);
            t.windup     = $signed(reg_windup_q20);
            t.vmax       = $signed(reg_vmax_q20);

            model_clarke(t);
            t.due_cycle = cycle_count + 1;
            q_clarke.push_back(t);
        end

        #1ps;

        if (rst) begin
            checks += 7;
            if (reader_vld  !== 1'b0) fail("reset.reader_vld",  "not low");
            if (clarke_vld  !== 1'b0) fail("reset.clarke_vld",  "not low");
            if (park_vld    !== 1'b0) fail("reset.park_vld",    "not low");
            if (cloop_vld   !== 1'b0) fail("reset.cloop_vld",   "not low");
            if (ipark_vld   !== 1'b0) fail("reset.ipark_vld",   "not low");
            if (iclarke_vld !== 1'b0) fail("reset.iclarke_vld", "not low");
            if (svpwm_vld   !== 1'b0) fail("reset.svpwm_vld",   "not low");

            checks += 3;
            if (pwm_a != PWM_HALF_COUNTS) fail("reset.pwmA", $sformatf("got=%0d", pwm_a));
            if (pwm_b != PWM_HALF_COUNTS) fail("reset.pwmB", $sformatf("got=%0d", pwm_b));
            if (pwm_c != PWM_HALF_COUNTS) fail("reset.pwmC", $sformatf("got=%0d", pwm_c));
        end
        else begin
            check_reader_stage();
            check_clarke_stage();
            check_park_stage();
            check_cloop_stage();
            check_ipark_stage();
            check_iclarke_stage();
            check_svpwm_stage();
        end
    end

    // =========================================================================
    // Driver / test-control tasks
    // =========================================================================
    task automatic reset_dut;
        begin
            @(negedge clk);
            rst                = 1'b1;
            pwm_center_trigger = 1'b0;
            calibrate          = 1'b0;
            direct_vld         = 1'b0;
            repeat (5) @(negedge clk);
            rst = 1'b0;
            repeat (2) @(negedge clk);
        end
    endtask

    task automatic begin_test(input string name);
        begin
            current_test    = name;
            test_error_base = errors;
            tests_run++;
            $display("\n[TEST] %s", name);
        end
    endtask

    task automatic end_test(input int timeout_cycles = 500);
        int n;
        begin
            n = 0;
            while (!pipeline_empty() && (n < timeout_cycles)) begin
                @(posedge clk); #1ps;
                n++;
            end
            if (!pipeline_empty())
                fail("pipeline.drain", $sformatf("did not drain within %0d cycles", timeout_cycles));

            if (q_reader.size() != 0)
                fail("reader.queue", $sformatf("%0d expected reader transactions remain", q_reader.size()));

            if (errors == test_error_base) begin
                tests_passed++;
                $display("[PASS] %s", current_test);
            end
            else begin
                $display("[FAIL] %s (%0d new errors)", current_test, errors-test_error_base);
            end
        end
    endtask

    task automatic send_direct_raw(
        input int signed a,
        input int signed b,
        input int signed c
    );
        begin
            @(negedge clk);
            direct_a   = a;
            direct_b   = b;
            direct_c   = c;
            direct_vld = 1'b1;
            @(negedge clk);
            direct_vld = 1'b0;
        end
    endtask

    task automatic send_direct_abc(
        input real a,
        input real b,
        input real c
    );
        send_direct_raw(q12(a), q12(b), q12(c));
    endtask

    task automatic send_constant_burst(
        input int n,
        input int signed a,
        input int signed b,
        input int signed c
    );
        begin
            @(negedge clk);
            direct_a   = a;
            direct_b   = b;
            direct_c   = c;
            direct_vld = 1'b1;
            repeat (n) @(negedge clk);
            direct_vld = 1'b0;
        end
    endtask

    task automatic send_varying_burst(input int n);
        int i;
        int signed a, b, c;
        begin
            for (i = 0; i < n; i++) begin
                a = q12(-1.0 + (0.25*i));
                b = q12( 0.5 - (0.10*i));
                c = -a - b;
                @(negedge clk);
                direct_a   = a;
                direct_b   = b;
                direct_c   = c;
                direct_vld = 1'b1;
            end
            @(negedge clk);
            direct_vld = 1'b0;
        end
    endtask

    task automatic pulse_calibrate;
        begin
            // Only call while the reader is idle; this mirrors the write-one
            // register pulse in the real top level.
            reader_ref_restart_cal();
            @(negedge clk);
            calibrate = 1'b1;
            @(negedge clk);
            calibrate = 1'b0;
        end
    endtask

    task automatic acquire_reader(
        input int unsigned raw_a,
        input int unsigned raw_b,
        input int timeout_cycles = 1000
    );
        int n;
        begin
            adc_raw_a = raw_a;
            adc_raw_b = raw_b;
            reader_queue_expected(raw_a, raw_b);

            @(negedge clk);
            pwm_center_trigger = 1'b1;
            @(negedge clk);
            pwm_center_trigger = 1'b0;

            n = 0;
            while ((reader_vld !== 1'b1) && (n < timeout_cycles)) begin
                @(posedge clk); #1ps;
                n++;
            end
            if (n >= timeout_cycles)
                fail("reader.timeout", $sformatf("no o_vld within %0d clocks", timeout_cycles));

            // Let the one-cycle valid pulse clear and downstream consume it.
            @(posedge clk); #1ps;
        end
    endtask

    task automatic pulse_extra_pwm_trigger;
        begin
            @(negedge clk);
            pwm_center_trigger = 1'b1;
            @(negedge clk);
            pwm_center_trigger = 1'b0;
        end
    endtask

    task automatic check_reader_offsets;
        begin
            checks += 2;
            if (reader_off_a !== ref_off_a[15:0])
                fail("reader.offsetA", $sformatf("got=%0d expected=%0d", reader_off_a, ref_off_a));
            if (reader_off_b !== ref_off_b[15:0])
                fail("reader.offsetB", $sformatf("got=%0d expected=%0d", reader_off_b, ref_off_b));
        end
    endtask

    task automatic idle_cycles(input int n);
        repeat (n) @(negedge clk);
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    initial begin : p_tests
        int i;
        int signed ra, rb, rc;
        real ang;

        // Deterministic startup values.
        rst                = 1'b1;
        pwm_center_trigger = 1'b0;
        calibrate          = 1'b0;
        vauxp4             = 1'b0;
        vauxn4             = 1'b0;
        vauxp6             = 1'b0;
        vauxn6             = 1'b0;
        use_direct         = 1'b0;
        direct_vld         = 1'b0;
        direct_a           = '0;
        direct_b           = '0;
        direct_c           = '0;
        trig_cos_q14       = 16'sd16384;
        trig_sin_q14       = 16'sd0;
        target_d_q12       = '0;
        target_q_q12       = '0;
        reg_kp_q24         = q7_24(1.0);
        reg_ki_q28         = q3_28(0.0);
        reg_windup_q20     = q11_20(8.0);
        reg_vmax_q20       = q11_20(12.0);

        cycle_count     = 0;
        next_txn_id     = 1;
        next_reader_id  = 1;
        checks           = 0;
        errors           = 0;
        warnings         = 0;
        tests_run        = 0;
        tests_passed     = 0;
        current_test     = "startup";
        scoreboard_clear();

        start_with_b         = 1'b0;
        inject_bad_chan_once = 1'b0;
        drop_next_drdy       = 1'b0;

        reset_dut();

        // ---------------------------------------------------------------------
        // 1) Reader: power-up calibration, scale/polarity, C reconstruction.
        // ---------------------------------------------------------------------
        begin_test("reader_powerup_calibration_midscale");
        use_direct = 1'b0;
        for (i = 0; i < CAL_SAMPLES; i++)
            acquire_reader(16'h8000, 16'h8000);

        checks++;
        if (reader_calibrating !== 1'b0)
            fail("reader.calibrating", "did not clear after 256 samples");
        check_reader_offsets();
        end_test(1000);

        begin_test("reader_polarity_endpoints_and_kcl");
        acquire_reader(16'd36000, 16'd30000);
        acquire_reader(16'd30000, 16'd36000);
        acquire_reader(16'd65535, 16'd0);
        acquire_reader(16'd0,     16'd65535);
        acquire_reader(16'd40000, 16'd40000);
        end_test(1000);

        begin_test("reader_bad_channel_is_ignored_then_recovers");
        inject_bad_chan_once = 1'b1;
        acquire_reader(16'd35000, 16'd31000, 1500);
        end_test(1000);

        begin_test("reader_ignores_pwm_trigger_while_busy");
        adc_raw_a = 16'd34500;
        adc_raw_b = 16'd31500;
        reader_queue_expected(adc_raw_a, adc_raw_b);
        @(negedge clk);
        pwm_center_trigger = 1'b1;
        @(negedge clk);
        pwm_center_trigger = 1'b0;
        repeat (8) @(negedge clk);
        pulse_extra_pwm_trigger();
        // There must still be exactly one reader transaction.
        i = 0;
        while ((reader_vld !== 1'b1) && i < 1500) begin
            @(posedge clk); #1ps; i++;
        end
        if (i >= 1500)
            fail("reader.timeout", "busy-trigger test never completed");
        repeat (250) @(posedge clk);
        end_test(1000);

        if (RUN_LONG_READER_TESTS) begin
            // ---------------------------------------------------------------
            // 2) Reader: calibration restart and channel-order independence.
            // ---------------------------------------------------------------
            begin_test("reader_recalibration_restart_and_b_first_order");
            start_with_b = 1'b1;
            reset_dut();
            use_direct = 1'b0;

            // Deliberately contaminate the first partial calibration.
            for (i = 0; i < 23; i++)
                acquire_reader(16'd30000, 16'd36000);

            pulse_calibrate();

            // Restart must discard all 23 old samples. B-first conversion order
            // must not matter because the reader classifies by channel address.
            for (i = 0; i < CAL_SAMPLES; i++)
                acquire_reader(16'd33000, 16'd31000);

            check_reader_offsets();
            checks++;
            if (reader_calibrating !== 1'b0)
                fail("reader.calibrating", "stuck active after restarted calibration");
            end_test(1000);

            // The ADC scale limits A/B to about +/-6.6 A around extreme
            // calibration offsets, but reconstructed C can exceed 16-bit Q3.12.
            // Exercise both C saturation rails with asymmetric calibrations.
            begin_test("reader_reconstructed_c_positive_and_negative_saturation");
            start_with_b = 1'b0;
            reset_dut();
            use_direct = 1'b0;

            for (i = 0; i < CAL_SAMPLES; i++)
                acquire_reader(16'd65535, 16'd0);
            acquire_reader(16'd0, 16'd65535); // A<0, B<0 => C saturates positive

            pulse_calibrate();
            for (i = 0; i < CAL_SAMPLES; i++)
                acquire_reader(16'd0, 16'd65535);
            acquire_reader(16'd65535, 16'd0); // A>0, B>0 => C saturates negative
            end_test(1000);
        end

        // ---------------------------------------------------------------
        // 3) Fault injection: document the current-reader no-DRDY behavior.
        // This is an architectural weakness test, not a normal operating case.
        // ---------------------------------------------------------------
        begin_test("reader_missing_drdy_requires_external_reset");
        drop_next_drdy = 1'b1;
        adc_raw_a = 16'h8000;
        adc_raw_b = 16'h8000;
        @(negedge clk);
        pwm_center_trigger = 1'b1;
        @(negedge clk);
        pwm_center_trigger = 1'b0;

        for (i = 0; i < 300; i++) begin
            @(posedge clk); #1ps;
            if (reader_vld)
                fail("reader.drop_drdy", "unexpected completion after intentionally dropped DRDY");
        end
        warn("reader.drop_drdy",
             "reader remains in ST_WAIT_DATA with no local timeout; top-level watchdog/reset must recover it");
        reset_dut();
        end_test(1000);

        // ---------------------------------------------------------------------
        // Switch to direct source for dense mathematical/control testing.
        // ---------------------------------------------------------------------
        use_direct = 1'b1;
        set_angle_deg(0.0);
        set_gains(1.0, 0.0, 8.0, 12.0);
        set_targets(0.0, 0.0);

        // ---------------------------------------------------------------------
        // 4) Zero / canonical transform cases.
        // ---------------------------------------------------------------------
        begin_test("zero_vector_full_pipeline");
        send_direct_abc(0.0, 0.0, 0.0);
        end_test();

        begin_test("clarke_known_alpha_beta_and_zero_sequence_cases");
        // alpha = +1, beta = 0
        send_direct_abc(1.0, -0.5, -0.5);
        // alpha = 0, beta ~= +1
        send_direct_abc(0.0, 0.8660254038, -0.8660254038);
        // pure zero-sequence should be rejected by Clarke
        send_direct_abc(1.0, 1.0, 1.0);
        end_test();

        begin_test("clarke_saturation_positive_negative_beta");
        send_direct_raw( 32767, -32768, -32768);
        send_direct_raw(-32768,  32767,  32767);
        send_direct_raw(     0,  32767, -32768);
        send_direct_raw(     0, -32768,  32767);
        end_test();

        // ---------------------------------------------------------------------
        // 5) Park quadrant/sign convention checks.
        // ---------------------------------------------------------------------
        begin_test("park_cardinal_angles");
        set_angle_deg(0.0);
        send_direct_abc(1.0, -0.5, -0.5);
        end_test();

        begin_test("park_plus_90_deg");
        set_angle_deg(90.0);
        send_direct_abc(1.0, -0.5, -0.5);
        end_test();

        begin_test("park_minus_90_deg");
        set_angle_deg(-90.0);
        send_direct_abc(1.0, -0.5, -0.5);
        end_test();

        begin_test("park_180_deg");
        set_angle_deg(180.0);
        send_direct_abc(1.0, -0.5, -0.5);
        end_test();

        // ---------------------------------------------------------------------
        // 6) Current-loop proportional behavior and voltage clamps.
        // ---------------------------------------------------------------------
        begin_test("current_loop_p_only_dq_signs");
        reset_dut();
        use_direct = 1'b1;
        set_angle_deg(0.0);
        set_gains(1.0, 0.0, 8.0, 12.0);
        set_targets(1.0, -0.75);
        send_direct_abc(0.0, 0.0, 0.0);
        end_test();

        begin_test("current_loop_feedback_subtraction");
        set_targets(0.0, 0.0);
        // Actual alpha = +1 at theta=0 -> Id=+1, therefore vd should be -1.
        send_direct_abc(1.0, -0.5, -0.5);
        end_test();

        begin_test("current_loop_voltage_clamp_both_polarities");
        set_gains(8.0, 0.0, 8.0, 2.0);
        set_targets(2.0, -2.0);
        send_direct_abc(0.0, 0.0, 0.0);
        end_test();

        begin_test("current_loop_internal_gain_saturation_extreme_error");
        reset_dut();
        use_direct = 1'b1;
        set_angle_deg(0.0);
        set_gains(100.0, 0.0, 31.0, 100.0);
        set_targets(7.5, -7.5);
        send_direct_abc(0.0, 0.0, 0.0);
        end_test();

        // ---------------------------------------------------------------------
        // 7) Integrator accumulation, anti-windup, and unwind.
        // ---------------------------------------------------------------------
        begin_test("current_loop_integrator_windup_and_unwind");
        reset_dut();
        use_direct = 1'b1;
        set_angle_deg(0.0);
        set_gains(0.0, 1.0, 1.5, 10.0);
        set_targets(1.0, 0.0);

        // Sparse cadence avoids any throughput assumptions while checking state.
        for (i = 0; i < 4; i++) begin
            send_direct_abc(0.0, 0.0, 0.0);
            while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        end

        // Reverse the error to prove the accumulator can unwind from the clamp.
        set_targets(-1.0, 0.0);
        send_direct_abc(0.0, 0.0, 0.0);
        end_test();

        // ---------------------------------------------------------------------
        // 8) Inverse transforms + SVPWM sector boundaries.
        // With zero measured current, P gain maps target d/q directly to voltage.
        // ---------------------------------------------------------------------
        begin_test("svpwm_sector_boundary_sweep");
        reset_dut();
        use_direct = 1'b1;
        set_gains(6.0, 0.0, 12.0, 31.0);
        set_targets(0.0, 1.0);

        // Around every 60-degree ordering transition plus cardinal points.
        for (i = 0; i <= 12; i++) begin
            ang = i * 30.0;
            set_angle_deg(ang);
            send_direct_abc(0.0, 0.0, 0.0);
            while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        end
        set_angle_deg(59.9);  send_direct_abc(0.0,0.0,0.0); while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        set_angle_deg(60.0);  send_direct_abc(0.0,0.0,0.0); while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        set_angle_deg(60.1);  send_direct_abc(0.0,0.0,0.0); while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        set_angle_deg(119.9); send_direct_abc(0.0,0.0,0.0); while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        set_angle_deg(120.0); send_direct_abc(0.0,0.0,0.0); while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        set_angle_deg(120.1); send_direct_abc(0.0,0.0,0.0);
        end_test();

        begin_test("svpwm_overmodulation_clamps_to_legal_counts");
        set_angle_deg(30.0);
        set_gains(8.0, 0.0, 31.0, 31.0);
        set_targets(0.0, 2.0); // ~16 V q-axis request, beyond a 12 V bus duty range
        send_direct_abc(0.0, 0.0, 0.0);
        end_test();

        // ---------------------------------------------------------------------
        // 9) Valid bubbles and dense valid traffic with invariant data.
        // ---------------------------------------------------------------------
        begin_test("valid_bubbles_and_output_hold");
        reset_dut();
        use_direct = 1'b1;
        set_angle_deg(25.0);
        set_gains(1.25, 0.0, 8.0, 12.0);
        set_targets(0.5, -0.25);
        send_direct_abc(0.25, -0.10, -0.15);
        idle_cycles(3);
        send_direct_abc(-0.50, 0.20, 0.30);
        idle_cycles(7);
        send_direct_abc(0.10, -0.40, 0.30);
        end_test();

        begin_test("back_to_back_identical_samples_pipeline_throughput");
        reset_dut();
        use_direct = 1'b1;
        set_angle_deg(15.0);
        set_gains(1.0, 0.0, 8.0, 12.0);
        set_targets(0.5, 0.25);
        send_constant_burst(8, q12(0.2), q12(-0.1), q12(-0.1));
        end_test(1000);

        // ---------------------------------------------------------------------
        // 10) Randomized bit-exact full-chain regression.
        // Angle is held until each transaction drains, matching the top-level
        // one-control-update-per-PWM-period timing contract.
        // ---------------------------------------------------------------------
        begin_test("randomized_balanced_full_chain");
        reset_dut();
        use_direct = 1'b1;
        set_gains(0.75, 0.0, 8.0, 12.0);

        for (i = 0; i < RANDOM_ITERS; i++) begin
            ang = real'($urandom_range(0, 35999)) / 100.0;
            set_angle_deg(ang);

            target_d_q12 = $signed($urandom_range(0, q12(3.0))) - q12(1.5);
            target_q_q12 = $signed($urandom_range(0, q12(3.0))) - q12(1.5);

            ra = $signed($urandom_range(0, q12(2.0))) - q12(1.0);
            rb = $signed($urandom_range(0, q12(2.0))) - q12(1.0);
            rc = -ra - rb;
            if (rc >  32767) rc =  32767;
            if (rc < -32768) rc = -32768;

            send_direct_raw(ra, rb, rc);
            while (!pipeline_empty()) begin @(posedge clk); #1ps; end
        end
        end_test();

        // ---------------------------------------------------------------------
        // 11) Reset mid-flight: all valids must flush and SVPWM reset to center.
        // ---------------------------------------------------------------------
        begin_test("reset_midflight_flushes_pipeline");
        set_angle_deg(37.0);
        set_gains(1.0, 0.0, 8.0, 12.0);
        set_targets(1.0, 0.5);
        send_direct_abc(0.2, -0.1, -0.1);
        repeat (3) @(negedge clk);
        reset_dut();
        checks += 3;
        if (pwm_a != PWM_HALF_COUNTS) fail("reset_midflight.pwmA", $sformatf("got=%0d",pwm_a));
        if (pwm_b != PWM_HALF_COUNTS) fail("reset_midflight.pwmB", $sformatf("got=%0d",pwm_b));
        if (pwm_c != PWM_HALF_COUNTS) fail("reset_midflight.pwmC", $sformatf("got=%0d",pwm_c));
        end_test();

        // ---------------------------------------------------------------------
        // 12) Optional protocol stress: these intentionally exceed the cadence
        // used by motor_controller_top and are valuable for exposing sample-tag
        // alignment weaknesses if future requirements demand dense traffic.
        // ---------------------------------------------------------------------
        if (RUN_PROTOCOL_STRESS) begin
            begin_test("STRESS_back_to_back_varying_samples");
            reset_dut();
            use_direct = 1'b1;
            set_angle_deg(10.0);
            set_gains(1.0, 0.0, 8.0, 12.0);
            set_targets(0.75, -0.5);
            send_varying_burst(10);
            end_test(1500);
        end
        else begin
            $display("[SKIP] STRESS_back_to_back_varying_samples (set RUN_PROTOCOL_STRESS=1 to enable)");
        end

        if (RUN_ANGLE_TAG_STRESS) begin
            begin_test("STRESS_angle_changes_every_source_sample");
            reset_dut();
            use_direct = 1'b1;
            set_gains(1.0, 0.0, 8.0, 12.0);
            set_targets(0.5, 0.25);
            for (i = 0; i < 8; i++) begin
                @(negedge clk);
                ang = i * 22.5;
                trig_cos_q14 = q14($cos(ang * 3.14159265358979323846 / 180.0));
                trig_sin_q14 = q14($sin(ang * 3.14159265358979323846 / 180.0));
                direct_a   = q12(0.25 + i*0.05);
                direct_b   = q12(-0.10);
                direct_c   = -direct_a - direct_b;
                direct_vld = 1'b1;
            end
            @(negedge clk);
            direct_vld = 1'b0;
            end_test(1500);
        end
        else begin
            $display("[SKIP] STRESS_angle_changes_every_source_sample (set RUN_ANGLE_TAG_STRESS=1 to enable)");
        end

        // ---------------------------------------------------------------------
        // Final report
        // ---------------------------------------------------------------------
        $display("\n============================================================");
        $display("FOC PIPELINE REGRESSION SUMMARY");
        $display("  tests run    : %0d", tests_run);
        $display("  tests passed : %0d", tests_passed);
        $display("  checks       : %0d", checks);
        $display("  warnings     : %0d", warnings);
        $display("  errors       : %0d", errors);
        $display("============================================================\n");

        if (errors == 0) begin
            $display("PASS: all enabled self-checks passed.");
            $finish;
        end
        else begin
            $fatal(1, "FAIL: %0d scoreboard/assertion errors detected", errors);
        end
    end

    // Global simulation watchdog.
    initial begin
        #20ms;
        $fatal(1, "GLOBAL TIMEOUT: testbench exceeded 20 ms simulated time");
    end

endmodule
