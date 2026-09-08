`timescale 1ns/1ps

module tb_park_inverse;

    import tb_foc_pkg::*;

    localparam time CLK_PERIOD = 10ns;
    localparam real TOL = 4.0 * VOLTAGE_LSB;
    localparam int TIMEOUT_CYCLES = 6;

    typedef struct packed {
        logic signed [17:0] d;
        logic signed [17:0] q;
        logic signed [15:0] cos_v;
        logic signed [15:0] sin_v;
    } inv_park_vec_t;

    logic clk = 1'b0;
    logic rst = 1'b1;

    logic i_vld;

    logic signed [17:0] i_d;
    logic signed [17:0] i_q;

    logic signed [15:0] i_cos;
    logic signed [15:0] i_sin;

    logic o_vld;

    logic signed [17:0] o_alpha;
    logic signed [17:0] o_beta;

    int tests  = 0;
    int passes = 0;
    int fails  = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    park_inverse dut (
        .i_clk   (clk),
        .i_rst   (rst),
        .i_vld   (i_vld),
        .i_d     (i_d),
        .i_q     (i_q),
        .i_cos   (i_cos),
        .i_sin   (i_sin),
        .o_vld   (o_vld),
        .o_alpha (o_alpha),
        .o_beta  (o_beta)
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

    task automatic golden_inverse_park(
        input inv_park_vec_t v,
        output real exp_alpha,
        output real exp_beta
    );
        real d;
        real q;
        real cos_v;
        real sin_v;

        d     = q5_12_to_real(v.d);
        q     = q5_12_to_real(v.q);
        cos_v = q1_14_to_real(v.cos_v);
        sin_v = q1_14_to_real(v.sin_v);

        exp_alpha = d*cos_v - q*sin_v;
        exp_beta  = d*sin_v + q*cos_v;

        exp_alpha = clamp_q5_12(exp_alpha);
        exp_beta  = clamp_q5_12(exp_beta);
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
        input real d,
        input real q,
        input real cos_v,
        input real sin_v
    );
        inv_park_vec_t applied;

        real exp_alpha;
        real exp_beta;

        bit seen;

        applied.d     = q5_12(d);
        applied.q     = q5_12(q);
        applied.cos_v = q1_14(cos_v);
        applied.sin_v = q1_14(sin_v);

        golden_inverse_park(
            applied,
            exp_alpha,
            exp_beta
        );

        @(negedge clk);

        i_d   = applied.d;
        i_q   = applied.q;
        i_cos = applied.cos_v;
        i_sin = applied.sin_v;
        i_vld = 1'b1;

        @(negedge clk);

        i_vld = 1'b0;

        wait_for_valid(name, seen);

        if (seen) begin
            check_near(
                $sformatf("%s alpha", name),
                q5_12_to_real(o_alpha),
                exp_alpha
            );

            check_near(
                $sformatf("%s beta", name),
                q5_12_to_real(o_beta),
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
        ab_volt_t held;
        bit any_valid;

        $display("\n--- i_vld gating test ---");

        run_case(
            "gating seed",
            3.0,
            -2.0,
            0.8660254,
            0.5
        );

        held.alpha = o_alpha;
        held.beta  = o_beta;

        any_valid = 1'b0;

        repeat (3) begin
            @(negedge clk);

            i_vld = 1'b0;
            i_d   = q5_12(10.0);
            i_q   = q5_12(-5.0);
            i_cos = q1_14(1.0);
            i_sin = q1_14(0.0);

            @(posedge clk);
            #1ps;

            if (o_vld === 1'b1)
                any_valid = 1'b1;
        end

        check_true(
            "no inverse-Park valid without i_vld",
            !any_valid
        );

        check_true(
            "alpha holds without valid input",
            o_alpha === held.alpha
        );

        check_true(
            "beta holds without valid input",
            o_beta === held.beta
        );
    endtask

    task automatic test_back_to_back;
        inv_park_vec_t vec [0:4];

        real exp_alpha [0:4];
        real exp_beta [0:4];

        bit seen;

        $display("\n--- Back-to-back throughput test ---");

        vec[0] = '{
            q5_12(1.0),
            q5_12(0.0),
            q1_14(1.0),
            q1_14(0.0)
        };

        vec[1] = '{
            q5_12(0.0),
            q5_12(1.0),
            q1_14(0.0),
            q1_14(1.0)
        };

        vec[2] = '{
            q5_12(4.0),
            q5_12(-2.0),
            q1_14(0.70710678),
            q1_14(0.70710678)
        };

        vec[3] = '{
            q5_12(-8.0),
            q5_12(3.0),
            q1_14(0.8660254),
            q1_14(-0.5)
        };

        vec[4] = '{
            q5_12(15.0),
            q5_12(10.0),
            q1_14(0.70710678),
            q1_14(0.70710678)
        };

        for (int n = 0; n < 5; n++)
            golden_inverse_park(
                vec[n],
                exp_alpha[n],
                exp_beta[n]
            );

        fork
            begin : driver
                @(negedge clk);

                for (int n = 0; n < 5; n++) begin
                    i_d   = vec[n].d;
                    i_q   = vec[n].q;
                    i_cos = vec[n].cos_v;
                    i_sin = vec[n].sin_v;
                    i_vld = 1'b1;

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
                            q5_12_to_real(o_alpha),
                            exp_alpha[n]
                        );

                        check_near(
                            $sformatf("back-to-back[%0d] beta", n),
                            q5_12_to_real(o_beta),
                            exp_beta[n]
                        );
                    end
                end
            end
        join
    endtask

    task automatic test_reset_mid_pipeline;

        $display("\n--- Reset during pipeline test ---");

        @(negedge clk);

        i_d   = q5_12(5.0);
        i_q   = q5_12(-3.0);
        i_cos = q1_14(0.70710678);
        i_sin = q1_14(0.70710678);
        i_vld = 1'b1;

        @(posedge clk);

        @(negedge clk);

        i_vld = 1'b0;
        rst   = 1'b1;

        @(posedge clk);
        #1ps;

        check_true(
            "reset cancels pending inverse-Park valid",
            o_vld === 1'b0
        );

        check_true(
            "reset zeros alpha",
            o_alpha === 18'sd0
        );

        check_true(
            "reset zeros beta",
            o_beta === 18'sd0
        );

        @(negedge clk);

        rst = 1'b0;
    endtask

    initial begin

        i_vld = 1'b0;
        i_d   = '0;
        i_q   = '0;
        i_cos = '0;
        i_sin = '0;

        $display("============================================================");
        $display(" tb_park_inverse: self-checking inverse Park verification");
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
            "inverse Park valid low after reset",
            o_vld === 1'b0
        );

        run_case("zero",          0.0,  0.0, 1.0, 0.0);
        run_case("zero degrees",  5.0, -2.0, 1.0, 0.0);
        run_case("+90 degrees",   5.0, -2.0, 0.0, 1.0);
        run_case("-90 degrees",   5.0, -2.0, 0.0,-1.0);

        run_case(
            "+45 degrees",
            5.0,
            -2.0,
            0.70710678,
            0.70710678
        );

        run_case(
            "+30 degrees",
            6.0,
            -3.0,
            0.8660254,
            0.5
        );

        run_case(
            "-30 degrees",
            -6.0,
            3.0,
            0.8660254,
            -0.5
        );

        run_case(
            "small d LSB",
            VOLTAGE_LSB,
            0.0,
            1.0,
            0.0
        );

        run_case(
            "positive alpha saturation",
            31.0,
            -31.0,
            0.70710678,
            -0.70710678
        );

        run_case(
            "negative alpha saturation",
            -31.0,
            31.0,
            0.70710678,
            -0.70710678
        );

        run_case(
            "positive beta saturation",
            31.0,
            31.0,
            0.70710678,
            0.70710678
        );

        run_case(
            "negative beta saturation",
            -31.0,
            -31.0,
            0.70710678,
            0.70710678
        );

        test_invalid_gating();
        test_back_to_back();
        test_reset_mid_pipeline();

        $display("\n============================================================");
        $display(
            " tb_park_inverse summary: tests=%0d pass=%0d fail=%0d",
            tests, passes, fails
        );
        $display("============================================================");

        if (fails == 0) begin
            $display("RESULT: PASS");
            $finish;
        end else begin
            $fatal(
                1,
                "RESULT: FAIL -- tb_park_inverse had %0d failing checks",
                fails
            );
        end
    end

endmodule