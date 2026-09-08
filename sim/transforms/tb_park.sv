`timescale 1ns/1ps

module tb_park;

    import tb_foc_pkg::*;

    localparam time CLK_PERIOD = 10ns;
    localparam real TOL = 4.0 * PHASE_LSB;
    localparam int TIMEOUT_CYCLES = 6;

    typedef struct packed {
        logic signed [15:0] alpha;
        logic signed [15:0] beta;
        logic signed [15:0] cos_v;
        logic signed [15:0] sin_v;
    } park_vec_t;

    logic clk = 1'b0;
    logic rst = 1'b1;

    logic i_vld;

    logic signed [15:0] i_alpha;
    logic signed [15:0] i_beta;
    logic signed [15:0] i_cos;
    logic signed [15:0] i_sin;

    logic o_vld;

    logic signed [15:0] o_d;
    logic signed [15:0] o_q;

    int tests  = 0;
    int passes = 0;
    int fails  = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    park dut (
        .i_clk   (clk),
        .i_rst   (rst),
        .i_vld   (i_vld),
        .i_alpha (i_alpha),
        .i_beta  (i_beta),
        .i_cos   (i_cos),
        .i_sin   (i_sin),
        .o_vld   (o_vld),
        .o_d     (o_d),
        .o_q     (o_q)
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

    task automatic golden_park(
        input park_vec_t v,
        output real exp_d,
        output real exp_q
    );
        real alpha;
        real beta;
        real cos_v;
        real sin_v;

        alpha = q3_12_to_real(v.alpha);
        beta  = q3_12_to_real(v.beta);
        cos_v = q1_14_to_real(v.cos_v);
        sin_v = q1_14_to_real(v.sin_v);

        exp_d = alpha*cos_v + beta*sin_v;
        exp_q = beta*cos_v - alpha*sin_v;

        exp_d = clamp_q3_12(exp_d);
        exp_q = clamp_q3_12(exp_q);
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
        input real beta,
        input real cos_v,
        input real sin_v
    );
        park_vec_t applied;

        real exp_d;
        real exp_q;

        bit seen;

        applied.alpha = q3_12(alpha);
        applied.beta  = q3_12(beta);
        applied.cos_v = q1_14(cos_v);
        applied.sin_v = q1_14(sin_v);

        golden_park(applied, exp_d, exp_q);

        @(negedge clk);

        i_alpha = applied.alpha;
        i_beta  = applied.beta;
        i_cos   = applied.cos_v;
        i_sin   = applied.sin_v;
        i_vld   = 1'b1;

        @(negedge clk);

        i_vld = 1'b0;

        wait_for_valid(name, seen);

        if (seen) begin
            check_near(
                $sformatf("%s d", name),
                q3_12_to_real(o_d),
                exp_d
            );

            check_near(
                $sformatf("%s q", name),
                q3_12_to_real(o_q),
                exp_q
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
        dq_phase_t held;
        bit any_valid;

        $display("\n--- i_vld gating test ---");

        run_case(
            "gating seed",
            1.25,
            -0.75,
            0.8660254,
            0.5
        );

        held.d = o_d;
        held.q = o_q;

        any_valid = 1'b0;

        repeat (3) begin
            @(negedge clk);

            i_vld   = 1'b0;
            i_alpha = q3_12(4.0);
            i_beta  = q3_12(-3.0);
            i_cos   = q1_14(1.0);
            i_sin   = q1_14(0.0);

            @(posedge clk);
            #1ps;

            if (o_vld === 1'b1)
                any_valid = 1'b1;
        end

        check_true(
            "no Park valid without i_vld",
            !any_valid
        );

        check_true(
            "d holds without valid input",
            o_d === held.d
        );

        check_true(
            "q holds without valid input",
            o_q === held.q
        );
    endtask

    task automatic test_back_to_back;
        park_vec_t vec [0:4];

        real exp_d [0:4];
        real exp_q [0:4];

        bit seen;

        $display("\n--- Back-to-back throughput test ---");

        vec[0] = '{
            q3_12(1.0),
            q3_12(0.0),
            q1_14(1.0),
            q1_14(0.0)
        };

        vec[1] = '{
            q3_12(0.0),
            q3_12(1.0),
            q1_14(0.0),
            q1_14(1.0)
        };

        vec[2] = '{
            q3_12(2.0),
            q3_12(-1.0),
            q1_14(0.70710678),
            q1_14(0.70710678)
        };

        vec[3] = '{
            q3_12(-3.0),
            q3_12(2.0),
            q1_14(0.8660254),
            q1_14(-0.5)
        };

        vec[4] = '{
            q3_12(5.0),
            q3_12(5.0),
            q1_14(0.70710678),
            q1_14(0.70710678)
        };

        for (int n = 0; n < 5; n++)
            golden_park(vec[n], exp_d[n], exp_q[n]);

        fork
            begin : driver
                @(negedge clk);

                for (int n = 0; n < 5; n++) begin
                    i_alpha = vec[n].alpha;
                    i_beta  = vec[n].beta;
                    i_cos   = vec[n].cos_v;
                    i_sin   = vec[n].sin_v;
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
                            $sformatf("back-to-back[%0d] d", n),
                            q3_12_to_real(o_d),
                            exp_d[n]
                        );

                        check_near(
                            $sformatf("back-to-back[%0d] q", n),
                            q3_12_to_real(o_q),
                            exp_q[n]
                        );
                    end
                end
            end
        join
    endtask

    task automatic test_reset_mid_pipeline;

        $display("\n--- Reset during pipeline test ---");

        @(negedge clk);

        i_alpha = q3_12(3.0);
        i_beta  = q3_12(-2.0);
        i_cos   = q1_14(0.70710678);
        i_sin   = q1_14(0.70710678);
        i_vld   = 1'b1;

        @(posedge clk);

        @(negedge clk);

        i_vld = 1'b0;
        rst   = 1'b1;

        @(posedge clk);
        #1ps;

        check_true(
            "reset cancels pending Park valid",
            o_vld === 1'b0
        );

        check_true(
            "reset zeros d",
            o_d === 16'sd0
        );

        check_true(
            "reset zeros q",
            o_q === 16'sd0
        );

        @(negedge clk);

        rst = 1'b0;
    endtask

    initial begin

        i_vld   = 1'b0;
        i_alpha = '0;
        i_beta  = '0;
        i_cos   = '0;
        i_sin   = '0;

        $display("============================================================");
        $display(" tb_park: self-checking Park verification");
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
            "Park valid low after reset",
            o_vld === 1'b0
        );

        run_case("zero",           0.0,  0.0,  1.0, 0.0);
        run_case("zero degrees",   2.0, -1.0,  1.0, 0.0);
        run_case("+90 degrees",    2.0, -1.0,  0.0, 1.0);
        run_case("-90 degrees",    2.0, -1.0,  0.0,-1.0);

        run_case(
            "+45 degrees",
            2.0,
            -1.0,
            0.70710678,
            0.70710678
        );

        run_case(
            "+30 degrees",
            2.25,
            -1.5,
            0.8660254,
            0.5
        );

        run_case(
            "-30 degrees",
            -2.25,
            1.5,
            0.8660254,
            -0.5
        );

        run_case(
            "small alpha LSB",
            PHASE_LSB,
            0.0,
            1.0,
            0.0
        );

        run_case(
            "positive saturation",
            5.0,
            5.0,
            0.70710678,
            0.70710678
        );

        run_case(
            "negative saturation",
            -5.0,
            -5.0,
            0.70710678,
            0.70710678
        );

        test_invalid_gating();
        test_back_to_back();
        test_reset_mid_pipeline();

        $display("\n============================================================");
        $display(
            " tb_park summary: tests=%0d pass=%0d fail=%0d",
            tests, passes, fails
        );
        $display("============================================================");

        if (fails == 0) begin
            $display("RESULT: PASS");
            $finish;
        end else begin
            $fatal(
                1,
                "RESULT: FAIL -- tb_park had %0d failing checks",
                fails
            );
        end
    end

endmodule