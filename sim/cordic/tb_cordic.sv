`timescale 1ns/1ps

module tb_cordic;

    import tb_foc_pkg::*;

    localparam time CLK_PERIOD = 10ns;
    localparam real TOL = 8.0 * CORDIC_LSB;
    localparam int  TIMEOUT_CYCLES = 40;

    logic clk = 1'b0;
    logic rst = 1'b1;

    logic                  i_vld;
    logic signed [15:0]    i_angle;
    logic                  o_rdy;
    logic                  o_vld;
    logic signed [15:0]    o_cos;
    logic signed [15:0]    o_sin;

    int tests  = 0;
    int passes = 0;
    int fails  = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    // Direct mixed-language instantiation of cordic.vhd.
    cordic dut (
        .i_clk   (clk),
        .i_rst   (rst),
        .i_vld   (i_vld),
        .o_rdy   (o_rdy),
        .i_angle (i_angle),
        .o_vld   (o_vld),
        .o_cos   (o_cos),
        .o_sin   (o_sin)
    );

    task automatic check_true(input string name, input bit condition);
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
        input real expected,
        input real tol
    );
        real err;
        tests++;
        err = rabs(actual - expected);

        if (err <= tol) begin
            passes++;
            $display("[PASS] %-42s actual=% .7f expected=% .7f err=%0.7f",
                     name, actual, expected, err);
        end else begin
            fails++;
            $error("[FAIL] %-42s actual=% .7f expected=% .7f err=%0.7f tol=%0.7f",
                   name, actual, expected, err, tol);
        end
    endtask

    task automatic wait_for_valid(input string name, output bit seen);
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

    task automatic run_raw(input string name, input int raw_angle);
        logic signed [15:0] angle16;
        real theta;
        real exp_cos, exp_sin;
        real act_cos, act_sin;
        bit seen;

        angle16 = $signed(raw_angle);
        theta   = angle_raw_to_radians(angle16);
        exp_cos = $cos(theta);
        exp_sin = $sin(theta);

        // Wait until the CORDIC advertises readiness.
        while (o_rdy !== 1'b1)
            @(negedge clk);

        @(negedge clk);
        i_angle = angle16;
        i_vld   = 1'b1;

        @(negedge clk);
        i_vld   = 1'b0;

        #1ps;
        check_true($sformatf("%s -- o_rdy drops while busy", name), o_rdy === 1'b0);

        wait_for_valid(name, seen);

        if (seen) begin
            act_cos = q1_14_to_real(o_cos);
            act_sin = q1_14_to_real(o_sin);

            check_near($sformatf("%s cos", name), act_cos, exp_cos, TOL);
            check_near($sformatf("%s sin", name), act_sin, exp_sin, TOL);

            @(posedge clk);
            #1ps;
            check_true($sformatf("%s -- o_vld is one cycle", name), o_vld === 1'b0);
            check_true($sformatf("%s -- ready returns", name),      o_rdy === 1'b1);
        end
    endtask

    task automatic test_busy_request_is_ignored;
        logic signed [15:0] first_angle;
        logic signed [15:0] ignored_angle;
        real theta;
        bit seen;
        bit extra_seen;

        $display("\n--- Busy-request rejection test ---");

        first_angle   = angle_from_degrees(30.0);
        ignored_angle = angle_from_degrees(-60.0);

        while (o_rdy !== 1'b1)
            @(negedge clk);

        @(negedge clk);
        i_angle = first_angle;
        i_vld   = 1'b1;
        @(negedge clk);
        i_vld   = 1'b0;

        // Pulse a second request while the DUT is definitely busy.
        repeat (3) @(posedge clk);
        @(negedge clk);
        check_true("CORDIC busy before ignored request", o_rdy === 1'b0);
        i_angle = ignored_angle;
        i_vld   = 1'b1;
        @(negedge clk);
        i_vld   = 1'b0;

        wait_for_valid("CORDIC original request after busy pulse", seen);

        if (seen) begin
            theta = angle_raw_to_radians(first_angle);
            check_near("busy test preserves original cosine",
                       q1_14_to_real(o_cos), $cos(theta), TOL);
            check_near("busy test preserves original sine",
                       q1_14_to_real(o_sin), $sin(theta), TOL);
        end

        // The ignored pulse must not create a second result.
        extra_seen = 1'b0;
        repeat (8) begin
            @(posedge clk);
            #1ps;
            if (o_vld === 1'b1)
                extra_seen = 1'b1;
        end
        check_true("request pulsed while busy is ignored", !extra_seen);
    endtask

    task automatic test_reset_mid_operation;
        bit stray_valid;

        $display("\n--- Reset-during-operation test ---");

        while (o_rdy !== 1'b1)
            @(negedge clk);

        @(negedge clk);
        i_angle = angle_from_degrees(67.5);
        i_vld   = 1'b1;
        @(negedge clk);
        i_vld   = 1'b0;

        repeat (5) @(posedge clk);

        @(negedge clk);
        rst = 1'b1;
        repeat (2) @(posedge clk);
        #1ps;
        check_true("reset forces o_vld low", o_vld === 1'b0);

        @(negedge clk);
        rst = 1'b0;

        @(posedge clk);
        #1ps;
        check_true("CORDIC returns ready after reset", o_rdy === 1'b1);

        stray_valid = 1'b0;
        repeat (8) begin
            @(posedge clk);
            #1ps;
            if (o_vld === 1'b1)
                stray_valid = 1'b1;
        end
        check_true("aborted CORDIC transaction produces no result", !stray_valid);
    endtask

    initial begin
        i_vld   = 1'b0;
        i_angle = '0;

        $display("============================================================");
        $display(" tb_cordic: self-checking CORDIC verification");
        $display(" tolerance = %0.7f (%0.1f Q1.14 LSBs)", TOL, TOL/CORDIC_LSB);
        $display("============================================================");

        repeat (4) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;

        @(posedge clk);
        #1ps;
        check_true("CORDIC ready after reset", o_rdy === 1'b1);
        check_true("CORDIC valid low after reset", o_vld === 1'b0);

        // Exact / near quadrant boundaries and numeric extrema.
        run_raw("raw -32768 (-180 deg)", -32768);
        run_raw("raw -32767 (-180 + 1 LSB)", -32767);
        run_raw("raw -24576 (-135 deg)", -24576);
        run_raw("raw -16385 (just below -90)", -16385);
        run_raw("raw -16384 (-90 deg)", -16384);
        run_raw("raw -16383 (just above -90)", -16383);
        run_raw("raw -8192 (-45 deg)", -8192);
        run_raw("raw -1 (negative angle LSB)", -1);
        run_raw("raw 0 (0 deg)", 0);
        run_raw("raw +1 (positive angle LSB)", 1);
        run_raw("raw 8192 (+45 deg)", 8192);
        run_raw("raw 16383 (just below +90)", 16383);
        run_raw("raw 16384 (+90 deg)", 16384);
        run_raw("raw 16385 (just above +90)", 16385);
        run_raw("raw 24576 (+135 deg)", 24576);
        run_raw("raw 32766 (near +180)", 32766);
        run_raw("raw 32767 (max positive)", 32767);

        // Important non-axis angles.
        run_raw("-60 deg", $signed(angle_from_degrees(-60.0)));
        run_raw("-30 deg", $signed(angle_from_degrees(-30.0)));
        run_raw("+30 deg", $signed(angle_from_degrees(30.0)));
        run_raw("+60 deg", $signed(angle_from_degrees(60.0)));
        run_raw("+22.5 deg", $signed(angle_from_degrees(22.5)));
        run_raw("+67.5 deg", $signed(angle_from_degrees(67.5)));

        test_busy_request_is_ignored();
        test_reset_mid_operation();

        $display("\n============================================================");
        $display(" tb_cordic summary: tests=%0d pass=%0d fail=%0d",
                 tests, passes, fails);
        $display("============================================================");

        if (fails == 0) begin
            $display("RESULT: PASS");
            $finish;
        end else begin
            $fatal(1, "RESULT: FAIL -- tb_cordic had %0d failing checks", fails);
        end
    end

endmodule
