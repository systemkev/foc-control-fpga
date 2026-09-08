`timescale 1ns/1ps

module tb_clarke;

    import tb_foc_pkg::*;

    localparam time CLK_PERIOD = 10ns;
    localparam real SQRT3 = 1.7320508075688772935;
    localparam real TOL   = 4.0 * PHASE_LSB;
    localparam int TIMEOUT_CYCLES = 6;

    logic clk = 1'b0;
    logic rst = 1'b1;

    logic i_vld;

    logic signed [15:0] i_phases_A;
    logic signed [15:0] i_phases_B;
    logic signed [15:0] i_phases_C;

    logic o_vld;

    logic signed [15:0] o_clarke_alpha;
    logic signed [15:0] o_clarke_beta;

    int tests  = 0;
    int passes = 0;
    int fails  = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    clarke dut (
        .i_clk           (clk),
        .i_rst           (rst),
        .i_vld           (i_vld),
        .i_phases_A      (i_phases_A),
        .i_phases_B      (i_phases_B),
        .i_phases_C      (i_phases_C),
        .o_vld           (o_vld),
        .o_clarke_alpha  (o_clarke_alpha),
        .o_clarke_beta   (o_clarke_beta)
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

    task automatic golden_clarke(
        input abc_phase_t ph,
        output real exp_alpha,
        output real exp_beta
    );
        real a;
        real b;
        real c;

        a = q3_12_to_real(ph.A);
        b = q3_12_to_real(ph.B);
        c = q3_12_to_real(ph.C);

        exp_alpha = (2.0/3.0) * (a - 0.5*b - 0.5*c);
        exp_beta  = (2.0/3.0) *
                    ((SQRT3/2.0)*b - (SQRT3/2.0)*c);

        exp_alpha = clamp5(exp_alpha);
        exp_beta  = clamp5(exp_beta);
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
        input real a,
        input real b,
        input real c
    );
        abc_phase_t applied;

        real exp_alpha;
        real exp_beta;

        bit seen;

        applied.A = q3_12(a);
        applied.B = q3_12(b);
        applied.C = q3_12(c);

        golden_clarke(applied, exp_alpha, exp_beta);

        @(negedge clk);

        i_phases_A = applied.A;
        i_phases_B = applied.B;
        i_phases_C = applied.C;
        i_vld      = 1'b1;

        @(negedge clk);

        i_vld = 1'b0;

        wait_for_valid(name, seen);

        if (seen) begin
            check_near(
                $sformatf("%s alpha", name),
                q3_12_to_real(o_clarke_alpha),
                exp_alpha
            );

            check_near(
                $sformatf("%s beta", name),
                q3_12_to_real(o_clarke_beta),
                exp_beta
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
        ab_phase_t held;
        bit any_valid;

        $display("\n--- i_vld gating test ---");

        run_case("gating seed", 1.25, -0.5, -0.75);

        held.alpha = o_clarke_alpha;
        held.beta  = o_clarke_beta;

        any_valid = 1'b0;

        repeat (3) begin
            @(negedge clk);

            i_vld      = 1'b0;
            i_phases_A = q3_12(5.0);
            i_phases_B = q3_12(-5.0);
            i_phases_C = q3_12(3.0);

            @(posedge clk);
            #1ps;

            if (o_vld === 1'b1)
                any_valid = 1'b1;
        end

        check_true(
            "no output valid when i_vld remains low",
            !any_valid
        );

        check_true(
            "alpha holds without a valid sample",
            o_clarke_alpha === held.alpha
        );

        check_true(
            "beta holds without a valid sample",
            o_clarke_beta === held.beta
        );
    endtask

    task automatic test_back_to_back;
        abc_phase_t vec [0:4];

        real exp_a [0:4];
        real exp_b [0:4];

        bit seen;

        $display("\n--- Back-to-back throughput test ---");

        vec[0] = '{q3_12(1.0),  q3_12(-0.5), q3_12(-0.5)};
        vec[1] = '{q3_12(0.0),  q3_12(1.0),  q3_12(-1.0)};
        vec[2] = '{q3_12(-2.0), q3_12(0.25), q3_12(1.75)};
        vec[3] = '{q3_12(5.0),  q3_12(-5.0), q3_12(-5.0)};
        vec[4] = '{q3_12(-5.0), q3_12(5.0),  q3_12(5.0)};

        for (int n = 0; n < 5; n++)
            golden_clarke(vec[n], exp_a[n], exp_b[n]);

        fork
            begin : driver
                @(negedge clk);

                for (int n = 0; n < 5; n++) begin
                    i_phases_A = vec[n].A;
                    i_phases_B = vec[n].B;
                    i_phases_C = vec[n].C;
                    i_vld      = 1'b1;

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
                            $sformatf("back-to-back[%0d] alpha", n),
                            q3_12_to_real(o_clarke_alpha),
                            exp_a[n]
                        );

                        check_near(
                            $sformatf("back-to-back[%0d] beta", n),
                            q3_12_to_real(o_clarke_beta),
                            exp_b[n]
                        );
                    end
                end
            end
        join
    endtask

    task automatic test_reset_mid_pipeline;

        $display("\n--- Reset during pipeline test ---");

        @(negedge clk);

        i_phases_A = q3_12(3.0);
        i_phases_B = q3_12(-1.0);
        i_phases_C = q3_12(-2.0);
        i_vld      = 1'b1;

        @(posedge clk);

        @(negedge clk);

        i_vld = 1'b0;
        rst   = 1'b1;

        @(posedge clk);
        #1ps;

        check_true(
            "reset cancels pending Clarke valid",
            o_vld === 1'b0
        );

        check_true(
            "reset zeros Clarke alpha",
            o_clarke_alpha === 16'sd0
        );

        check_true(
            "reset zeros Clarke beta",
            o_clarke_beta === 16'sd0
        );

        @(negedge clk);

        rst = 1'b0;
    endtask

    initial begin

        i_vld      = 1'b0;
        i_phases_A = '0;
        i_phases_B = '0;
        i_phases_C = '0;

        $display("============================================================");
        $display(" tb_clarke: self-checking Clarke verification");
        $display(
            " tolerance = %0.7f (%0.1f Q3.12 LSBs)",
            TOL, TOL/PHASE_LSB
        );
        $display("============================================================");

        repeat (4) @(posedge clk);

        @(negedge clk);
        rst = 1'b0;

        @(posedge clk);
        #1ps;

        check_true(
            "Clarke valid low after reset",
            o_vld === 1'b0
        );

        run_case("zero vector",                0.0,  0.0,  0.0);
        run_case("balanced phase A axis",      1.0, -0.5, -0.5);
        run_case("balanced negative A axis",  -1.0,  0.5,  0.5);
        run_case("pure beta direction",        0.0,  0.8660254, -0.8660254);
        run_case("common mode +1",             1.0,  1.0,  1.0);
        run_case("common mode -5",            -5.0, -5.0, -5.0);
        run_case("general unbalanced",         2.25, -1.5, 0.75);
        run_case("small positive LSB",         PHASE_LSB, 0.0, 0.0);
        run_case("small negative LSB",        -PHASE_LSB, 0.0, 0.0);

        run_case("alpha positive saturation",  5.0, -5.0, -5.0);
        run_case("alpha negative saturation", -5.0,  5.0,  5.0);
        run_case("beta positive saturation",   0.0,  5.0, -5.0);
        run_case("beta negative saturation",   0.0, -5.0,  5.0);

        test_invalid_gating();
        test_back_to_back();
        test_reset_mid_pipeline();

        $display("\n============================================================");
        $display(
            " tb_clarke summary: tests=%0d pass=%0d fail=%0d",
            tests, passes, fails
        );
        $display("============================================================");

        if (fails == 0) begin
            $display("RESULT: PASS");
            $finish;
        end else begin
            $fatal(
                1,
                "RESULT: FAIL -- tb_clarke had %0d failing checks",
                fails
            );
        end
    end

endmodule