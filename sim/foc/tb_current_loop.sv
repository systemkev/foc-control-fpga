module tb_current_cloop;
    timeunit 1ns;
    timeprecision 1ps;

    logic clk = 0;
    logic rst = 1;

    logic i_vld = 0;
    logic o_vld;

    typedef struct packed {
        logic signed [15:0] d;
        logic signed [15:0] q;
    } t_dq_phase;

    typedef struct packed {
        logic signed [17:0] d;
        logic signed [17:0] q;
    } t_dq_volt;

    t_dq_phase i_target_curr = '{default:0};
    t_dq_phase i_actual_curr = '{default:0};
    t_dq_volt o_ph_voltage;

    logic signed [31:0] REG_FOC_KP_GAIN = 0;
    logic signed [31:0] REG_FOC_KI_GAIN = 0;
    logic signed [31:0] REG_FOC_WINDUP_MAX = 0;
    logic signed [31:0] REG_FOC_VOLTAGE_MAX = 0;

    current_cloop uut (
        .i_clk(clk),
        .i_rst(rst),
        .i_vld(i_vld),
        .i_target_curr(i_target_curr),
        .i_actual_curr(i_actual_curr),
        .o_vld(o_vld),
        .o_ph_voltage(o_ph_voltage),
        .REG_FOC_KP_GAIN(REG_FOC_KP_GAIN),
        .REG_FOC_KI_GAIN(REG_FOC_KI_GAIN),
        .REG_FOC_WINDUP_MAX(REG_FOC_WINDUP_MAX),
        .REG_FOC_VOLTAGE_MAX(REG_FOC_VOLTAGE_MAX)
    );

    always #5 clk = ~clk;

    task automatic check_val(real act, real exp_val, real tol, string msg);
        real diff;
        diff = act - exp_val;
        if (diff < 0.0) diff = -diff;
        if (diff > tol) begin
            $display("ERROR: %s | Expected: %f, Got: %f", msg, exp_val, act);
            $stop;
        end
    endtask

    task automatic wait_cycles(int n);
        repeat(n) @(posedge clk);
    endtask

    initial begin
        REG_FOC_KP_GAIN = $rtoi(2.0 * 16777216.0);
        REG_FOC_KI_GAIN = $rtoi(0.5 * 268435456.0);
        REG_FOC_WINDUP_MAX = $rtoi(5.0 * 1048576.0);
        REG_FOC_VOLTAGE_MAX = $rtoi(15.0 * 1048576.0);

        wait_cycles(3);
        rst = 0;
        wait_cycles(2);

        if (o_vld !== 0) begin
            $display("ERROR: Reset failure");
            $stop;
        end

        REG_FOC_KI_GAIN = 0;
        i_target_curr.d = $rtoi(2.0 * 4096.0);
        i_target_curr.q = $rtoi(3.0 * 4096.0);
        i_actual_curr.d = $rtoi(1.0 * 4096.0);
        i_actual_curr.q = $rtoi(4.0 * 4096.0);

        i_vld = 1;
        wait_cycles(1);
        i_vld = 0;

        wait_cycles(2);
        if (o_vld !== 0) begin
            $display("ERROR: Pipeline fired early");
            $stop;
        end

        wait_cycles(1);
        if (o_vld !== 1) begin
            $display("ERROR: Pipeline failed to fire");
            $stop;
        end
        
        check_val(real'(o_ph_voltage.d) / 4096.0, 2.0, 0.001, "T2: D-Axis Proportional Failure");
        check_val(real'(o_ph_voltage.q) / 4096.0, -2.0, 0.001, "T2: Q-Axis Proportional Failure");

        wait_cycles(1);
        if (o_vld !== 0) begin
            $display("ERROR: o_vld stayed high");
            $stop;
        end

        REG_FOC_KP_GAIN = 0;
        REG_FOC_KI_GAIN = $rtoi(1.0 * 268435456.0);
        i_target_curr.d = $rtoi(1.0 * 4096.0);
        i_actual_curr.d = 0;

        i_vld = 1;
        wait_cycles(3);
        i_vld = 0;

        wait_cycles(1);
        if (o_vld !== 1) $stop;
        check_val(real'(o_ph_voltage.d) / 4096.0, 1.0, 0.001, "T3: I-Accumulation Cycle 1");
        
        wait_cycles(1);
        check_val(real'(o_ph_voltage.d) / 4096.0, 2.0, 0.001, "T3: I-Accumulation Cycle 2");
        
        wait_cycles(1);
        check_val(real'(o_ph_voltage.d) / 4096.0, 3.0, 0.001, "T3: I-Accumulation Cycle 3");
        
        wait_cycles(1);
        if (o_vld !== 0) $stop;

        REG_FOC_KI_GAIN = $rtoi(2.0 * 268435456.0);
        i_vld = 1;
        i_target_curr.d = $rtoi(2.0 * 4096.0);
        wait_cycles(3);
        i_vld = 0;

        wait_cycles(4);
        check_val(real'(o_ph_voltage.d) / 4096.0, 5.0, 0.001, "T4: Positive Windup Clamping");

        i_vld = 1;
        i_target_curr.d = $rtoi(-4.0 * 4096.0);
        wait_cycles(4);
        i_vld = 0;

        wait_cycles(3);
        check_val(real'(o_ph_voltage.d) / 4096.0, -5.0, 0.001, "T4: Negative Windup Clamping");

        rst = 1;
        wait_cycles(1);
        rst = 0;

        REG_FOC_KP_GAIN = $rtoi(10.0 * 16777216.0);
        REG_FOC_KI_GAIN = 0;
        REG_FOC_VOLTAGE_MAX = $rtoi(15.0 * 1048576.0);

        i_target_curr.d = $rtoi(2.0 * 4096.0);
        i_target_curr.q = $rtoi(-3.0 * 4096.0);
        i_actual_curr.d = 0;
        i_actual_curr.q = 0;

        i_vld = 1;
        wait_cycles(1);
        i_vld = 0;

        wait_cycles(4);
        check_val(real'(o_ph_voltage.d) / 4096.0, 15.0, 0.001, "T5: Positive Voltage Saturation");
        check_val(real'(o_ph_voltage.q) / 4096.0, -15.0, 0.001, "T5: Negative Voltage Saturation");

        $display("All tests completed successfully.");
        $finish;
    end
endmodule