`timescale 1ns/1ps

module tb_clarke_inverse;

    import tb_foc_pkg::*;

    localparam time CLK_PERIOD = 10ns;
    localparam real SQRT3 = 1.7320508075688772935;
    localparam real TOL   = 4.0 * VOLTAGE_LSB;
    localparam int TIMEOUT_CYCLES = 6;

    logic clk = 1'b0;
    logic rst = 1'b1;

    logic i_vld;

    logic signed [17:0] i_alpha;
    logic signed [17:0] i_beta;

    logic o_vld;

    logic signed [17:0] o_phases_A;
    logic signed [17:0] o_phases_B;
    logic signed [17:0] o_phases_C;

    int tests  = 0;
    int passes = 0;
    int fails  = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    clarke_inverse dut (
        .i_clk      (clk),
        .i_rst      (rst),
        .i_vld      (i_vld),
        .i_alpha    (i_alpha),
        .i_beta     (i_beta),
        .o_vld      (o_vld),
        .o_phases_A (o_phases_A),
        .o_phases_B (o_phases_B),
        .o_phases_C (o_phases_C)
    );

    task automatic check_true(
        input string name,
        input bit condition
    );
        tests++;

        if (condition) begin
            passes++;
            $display("[PASS] %s", name);
        end else begin
            fails++;
            $error("[FAIL] %s", name);
        end
    endtask

    task automatic check_near(
        input string name,
        input real actual,
        input real expected
    );
        real err;

        tests++;
        err = rabs(actual - expected);

        if (err <= TOL) begin
            passes++;

            $display(
                "[PASS] %-42s actual=%0.6f expected=%0.6f err=%0.6f",
                name, actual, expected, err
            );
        end else begin
            fails++;

            $error(
                "[FAIL] %-42s actual=%0.6f expected=%0.6f err=%0.6f tol=%0.6f",
                name, actual, expected, err, TOL
            );
        end
    endtask

    task automatic golden_inverse(
        input ab_volt_t ab,
        output real exp_a,
        output real exp_b,
        output real exp_c
    );
        real alpha;
        real beta;

        alpha = q5_12_to_real(ab.alpha);
        beta  = q5_12_to_real(ab.beta);

        exp_a = alpha;
        exp_b = -0.5 * alpha + (SQRT3/2.0) * beta;
        exp_c = -0.5 * alpha - (SQRT3/2.0) * beta;

        exp_a = clamp_q5_12(exp_a);
        exp_b = clamp_q5_12(exp_b);
        exp_c = clamp_q5_12(exp_c);
    endtask

    task automatic wait_for_valid(
        input string name,
        output bit seen
    );
        seen = 1'b0;

        for (int k = 0; k < TIMEOUT_CYCLES; k++) begin
            @(posedge clk);
            #1ps;

            if (o_vld === 1'b1) begin
                seen = 1'b1;
                return;
            end
        end

        tests++;
        fails++;

        $error("[FAIL] %s: timed out waiting for o_vld", name);
    endtask

    task automatic run_case(
        input string name,
        input real alpha,
        input real beta
    );
        ab_volt_t applied;

        real exp_a;
        real exp_b;
        real exp_c;

        bit seen;

        applied.alpha = q5_12(alpha);
        applied.beta  = q5_12(beta);

        golden_inverse(applied, exp_a, exp_b, exp_c);

        @(negedge clk);

        i_alpha = applied.alpha;
        i_beta  = applied.beta;
        i_vld   = 1'b1;

        @(negedge clk);

        i_vld = 1'b0;

        wait_for_valid(name, seen);

        if (seen) begin
            check_near(
                $sformatf("%s phase A", name),
                q5_12_to_real(o_phases_A),
                exp_a
            );

            check_near(
                $sformatf("%s phase B", name),
                q5_12_to_real(o_phases_B),
                exp_b
            );

            check_near(
                $sformatf("%s phase C", name),
                q5_12_to_real(o_phases_C),
                exp_c
            );

            @(posedge clk);
            #1ps;

            check_true(
                $sformatf("%s -- valid pulse width", name),
                o_vld === 1'b0
            );
        end
    endtask

    task automatic test_invalid_gating;
        abc_volt_t held;
        bit any_valid;

        $display("\n--- i_vld gating test ---");

        run_case("gating seed", 1.5, -0.75);

        held.A = o_phases_A;
        held.B = o_phases_B;
        held.C = o_phases_C;

        any_valid = 1'b0;

        repeat (3) begin
            @(negedge clk);

            i_vld   = 1'b0;
            i_alpha = q5_12(-5.0);
            i_beta  = q5_12(5.0);

            @(posedge clk);
            #1ps;

            if (o_vld === 1'b1)
                any_valid = 1'b1;
        end

        check_true(
            "no inverse-Clarke valid without i_vld",
            !any_valid
        );

        check_true(
            "phase A holds without valid input",
            o_phases_A === held.A
        );

        check_true(
            "phase B holds without valid input",
            o_phases_B === held.B
        );

        check_true(
            "phase C holds without valid input",
            o_phases_C === held.C
        );
    endtask

    task automatic test_back_to_back;
        ab_volt_t vec [0:4];

        real exp_a [0:4];
        real exp_b [0:4];
        real exp_c [0:4];

        bit seen;

        $display("\n--- Back-to-back throughput test ---");

        vec[0] = '{q5_12(0.0),  q5_12(0.0)};
        vec[1] = '{q5_12(1.0),  q5_12(0.0)};
        vec[2] = '{q5_12(0.0),  q5_12(1.0)};
        vec[3] = '{q5_12(10.0), q5_12(-7.5)};
        vec[4] = '{q5_12(-8.0), q5_12(4.0)};

        for (int n = 0; n < 5; n++)
            golden_inverse(vec[n], exp_a[n], exp_b[n], exp_c[n]);

        fork
            begin : driver
                @(negedge clk);

                for (int n = 0; n < 5; n++) begin
                    i_alpha = vec[n].alpha;
                    i_beta  = vec[n].beta;
                    i_vld   = 1'b1;

                    @(negedge clk);
                end

                i_vld = 1'b0;
            end

            begin : monitor
                for (int n = 0; n < 5; n++) begin
                    wait_for_valid(
                        $sformatf("back-to-back[%0d]", n),
                        seen
                    );

                    if (seen) begin
                        check_near(
                            $sformatf("back-to-back[%0d] A", n),
                            q5_12_to_real(o_phases_A),
                            exp_a[n]
                        );

                        check_near(
                            $sformatf("back-to-back[%0d] B", n),
                            q5_12_to_real(o_phases_B),
                            exp_b[n]
                        );

                        check_near(
                            $sformatf("back-to-back[%0d] C", n),
                            q5_12_to_real(o_phases_C),
                            exp_c[n]
                        );
                    end
                end
            end
        join
    endtask

    task automatic test_reset_mid_pipeline;

        $display("\n--- Reset during pipeline test ---");

        @(negedge clk);

        i_alpha = q5_12(3.0);
        i_beta  = q5_12(-2.0);
        i_vld   = 1'b1;

        @(posedge clk);

        @(negedge clk);

        i_vld = 1'b0;
        rst   = 1'b1;

        @(posedge clk);
        #1ps;

        check_true(
            "reset cancels pending inverse-Clarke valid",
            o_vld === 1'b0
        );

        check_true(
            "reset zeros phase A",
            o_phases_A === 18'sd0
        );

        check_true(
            "reset zeros phase B",
            o_phases_B === 18'sd0
        );

        check_true(
            "reset zeros phase C",
            o_phases_C === 18'sd0
        );

        @(negedge clk);

        rst = 1'b0;
    endtask

    initial begin

        i_vld   = 1'b0;
        i_alpha = '0;
        i_beta  = '0;

        $display("============================================================");
        $display(" tb_clarke_inverse: self-checking inverse Clarke verification");
        $display(
            " tolerance = %0.7f (%0.1f Q5.12 LSBs)",
            TOL, TOL/VOLTAGE_LSB
        );
        $display("============================================================");

        repeat (4) @(posedge clk);

        @(negedge clk);
        rst = 1'b0;

        @(posedge clk);
        #1ps;

        check_true(
            "inverse Clarke valid low after reset",
            o_vld === 1'b0
        );

        run_case("zero vector",                 0.0,   0.0);
        run_case("pure alpha +1",               1.0,   0.0);
        run_case("pure alpha -1",              -1.0,   0.0);
        run_case("pure beta +1",                0.0,   1.0);
        run_case("pure beta -1",                0.0,  -1.0);
        run_case("general quadrant I",          2.25,  1.75);
        run_case("general quadrant III",       -2.25, -1.75);
        run_case("small alpha LSB",             VOLTAGE_LSB, 0.0);
        run_case("small beta LSB",              0.0, VOLTAGE_LSB);

        run_case("positive max alpha",          31.0,   0.0);
        run_case("negative max alpha",         -31.0,   0.0);
        run_case("phase B positive saturation",-31.0,  31.0);
        run_case("phase B negative saturation", 31.0, -31.0);
        run_case("phase C positive saturation",-31.0, -31.0);
        run_case("phase C negative saturation", 31.0,  31.0);

        test_invalid_gating();
        test_back_to_back();
        test_reset_mid_pipeline();

        $display("\n============================================================");
        $display(
            " tb_clarke_inverse summary: tests=%0d pass=%0d fail=%0d",
            tests, passes, fails
        );
        $display("============================================================");

        if (fails == 0) begin
            $display("RESULT: PASS");
            $finish;
        end else begin
            $fatal(
                1,
                "RESULT: FAIL -- tb_clarke_inverse had %0d failing checks",
                fails
            );
        end
    end

endmodule