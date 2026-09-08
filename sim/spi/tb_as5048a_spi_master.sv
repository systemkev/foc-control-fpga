`timescale 1ns/1ps

// ============================================================================
// Self-checking, scoreboard-based testbench for as5048a_spi_master
//
// AS5048A SPI requirements checked here (datasheet rev v1-11, pp. 10-19):
//   * 16 clocks per CSn-low transmission
//   * MSB first
//   * AS5048A samples MOSI on CLK falling edges
//   * AS5048A updates MISO on CLK rising edges
//   * tL     : CSn falling -> first CLK rising       >= 350 ns
//   * TCLK   : serial clock period                   >= 100 ns
//   * tCLKL  : CLK low period                        >=  50 ns
//   * tCLKH  : CLK high period                       >=  50 ns
//   * tH     : last CLK falling -> CSn rising        >=  50 ns
//   * TCSnH  : CSn high between transmissions        >= 350 ns
//   * tMOSI  : MOSI valid before sampling edge       >=  20 ns
//   * tMISO  : CLK rising -> MISO valid              <=  20 ns
//
// The slave model deliberately applies MISO at the worst-case tMISO = 20 ns.
// Therefore a correct master must still receive every response correctly.
//
// The testbench does NOT rely on C_WAIT_MAX or C_SCLK_SWTCH_CNT. It measures
// actual pin timing in physical simulation time. Set CLK_PERIOD to the clock
// period for which spi_pkg was configured.
// ============================================================================

module tb_as5048a_spi_master;

  timeunit      1ns;
  timeprecision 1ps;

  // Change this to match the clock frequency assumed by work.spi_pkg.
  parameter time CLK_PERIOD = 20ns;   // 50 MHz default
  parameter int  RANDOM_ITERS = 50;

  // AS5048A datasheet SPI timing limits.
  localparam time DS_TL_MIN       = 350ns;
  localparam time DS_TCLK_MIN     = 100ns;
  localparam time DS_TCLKL_MIN    =  50ns;
  localparam time DS_TCLKH_MIN    =  50ns;
  localparam time DS_TH_MIN       =  50ns;
  localparam time DS_TCSNH_MIN    = 350ns;
  localparam time DS_TMOSI_MIN    =  20ns;
  localparam time DS_TMISO_MAX    =  20ns;

  localparam logic [13:0] ADDR_CLEAR_ERROR = 14'h0001;
  localparam logic [13:0] ADDR_ANGLE       = 14'h3FFF;

  // --------------------------------------------------------------------------
  // DUT signals
  // --------------------------------------------------------------------------
  logic        i_clk;
  logic        i_rst;
  logic        i_start;
  logic        i_miso;

  logic        o_mosi;
  logic        o_n_cs;
  logic        o_sclk;
  logic        o_vld;
  logic [13:0] o_raw_angle;

  // Mixed-language simulators normally allow direct instantiation of the
  // VHDL entity by its entity name after the VHDL has been compiled into work.
  as5048a_spi_master dut (
    .i_clk       (i_clk),
    .i_rst       (i_rst),
    .i_start     (i_start),
    .i_miso      (i_miso),
    .o_mosi      (o_mosi),
    .o_n_cs      (o_n_cs),
    .o_sclk      (o_sclk),
    .o_vld       (o_vld),
    .o_raw_angle (o_raw_angle)
  );

  // --------------------------------------------------------------------------
  // Utility functions
  // --------------------------------------------------------------------------

  // Build an AS5048A READ command:
  //   bit 15    = even parity
  //   bit 14    = 1 (read)
  //   bits13:0  = address
  function automatic logic [15:0] make_read_cmd(input logic [13:0] addr);
    logic [15:0] f;
    begin
      f         = '0;
      f[14]     = 1'b1;
      f[13:0]   = addr;
      f[15]     = ^f[14:0];  // makes XOR of all 16 bits equal zero
      return f;
    end
  endfunction

  // Build a sensor READ response:
  //   bit 15    = even parity
  //   bit 14    = EF
  //   bits13:0  = data
  //
  // bad_parity=1 intentionally flips bit 15 for negative testing.
  function automatic logic [15:0] make_read_rsp(
    input logic [13:0] data,
    input logic        ef,
    input logic        bad_parity
  );
    logic [15:0] f;
    begin
      f         = '0;
      f[14]     = ef;
      f[13:0]   = data;
      f[15]     = ^f[14:0];
      if (bad_parity)
        f[15] = ~f[15];
      return f;
    end
  endfunction

  function automatic bit even_parity(input logic [15:0] f);
    return (^f) === 1'b0;
  endfunction

  localparam logic [15:0] CMD_READ_ANGLE  = make_read_cmd(ADDR_ANGLE);
  localparam logic [15:0] CMD_CLEAR_ERROR = make_read_cmd(ADDR_CLEAR_ERROR);

  // --------------------------------------------------------------------------
  // Self-reporting infrastructure
  // --------------------------------------------------------------------------
  int unsigned check_count      = 0;
  int unsigned pass_count       = 0;
  int unsigned fail_count       = 0;
  int unsigned transaction_count = 0;
  int unsigned aborted_count      = 0;
  int unsigned vld_count          = 0;

  string current_test = "startup";

  task automatic check(input bit condition, input string message);
    begin
      check_count++;
      if (condition) begin
        pass_count++;
      end
      else begin
        fail_count++;
        $display("[%0t] FAIL [%s] %s", $time, current_test, message);
      end
    end
  endtask

  task automatic info(input string message);
    $display("[%0t] INFO [%s] %s", $time, current_test, message);
  endtask

  task automatic begin_test(input string name);
    begin
      current_test = name;
      $display("\n============================================================");
      $display("TEST: %s", current_test);
      $display("============================================================");
    end
  endtask

  task automatic end_test;
    $display("[%0t] END TEST: %s", $time, current_test);
  endtask

  // --------------------------------------------------------------------------
  // Transaction scoreboard queues
  //
  // For every expected CSn-low transaction:
  //   exp_mosi_q[n] says what the DUT must transmit
  //   rsp_miso_q[n] says what the sensor model will return
  //
  // exp_angle_q holds the expected o_raw_angle for each o_vld pulse.
  // --------------------------------------------------------------------------
  logic [15:0] exp_mosi_q[$];
  logic [15:0] rsp_miso_q[$];
  logic [13:0] exp_angle_q[$];

  task automatic expect_spi_transaction(
    input logic [15:0] expected_mosi,
    input logic [15:0] sensor_miso
  );
    begin
      exp_mosi_q.push_back(expected_mosi);
      rsp_miso_q.push_back(sensor_miso);
    end
  endtask

  task automatic expect_successful_read(input logic [13:0] angle);
    begin
      // Transmission 1 response is the response to the command that preceded
      // this read sequence, so the DUT must ignore it.
      expect_spi_transaction(
        CMD_READ_ANGLE,
        make_read_rsp(14'h1234, 1'b0, 1'b0)
      );

      // Transmission 2 returns the requested angle.
      expect_spi_transaction(
        CMD_READ_ANGLE,
        make_read_rsp(angle, 1'b0, 1'b0)
      );

      exp_angle_q.push_back(angle);
    end
  endtask

  task automatic clear_expected_queues;
    begin
      exp_mosi_q.delete();
      rsp_miso_q.delete();
      exp_angle_q.delete();
    end
  endtask

  task automatic check_queues_empty;
    begin
      check(exp_mosi_q.size() == 0,
            $sformatf("MOSI scoreboard not empty: %0d transaction(s) remain",
                      exp_mosi_q.size()));
      check(rsp_miso_q.size() == 0,
            $sformatf("MISO response queue not empty: %0d transaction(s) remain",
                      rsp_miso_q.size()));
      check(exp_angle_q.size() == 0,
            $sformatf("Angle scoreboard not empty: %0d value(s) remain",
                      exp_angle_q.size()));
    end
  endtask

  // --------------------------------------------------------------------------
  // Clock / stimulus helpers
  // --------------------------------------------------------------------------
  initial i_clk = 1'b0;
  always #(CLK_PERIOD/2) i_clk = ~i_clk;

  // Hold i_start across TWO rising edges.
  //
  // Why two? o_vld is a combinational decode of S_DONE. If a caller sees
  // o_vld rise and immediately requests the next conversion, the first
  // following i_clk rising edge only moves the DUT from S_DONE -> S_IDLE.
  // A one-clock start pulse can therefore be completely missed. Holding start
  // across two rising edges guarantees that one rising edge occurs while the
  // FSM is in S_IDLE, while still being short enough for the intentional
  // "start while busy" tests below.
  task automatic pulse_start;
    begin
      @(negedge i_clk);
      i_start = 1'b1;
      repeat (2) @(posedge i_clk);
      @(negedge i_clk);
      i_start = 1'b0;
    end
  endtask

  task automatic wait_for_vld_count(
    input int unsigned target,
    input int unsigned timeout_cycles
  );
    int unsigned k;
    bit reached;
    begin
      reached = 1'b0;
      for (k = 0; k < timeout_cycles; k++) begin
        @(posedge i_clk);
        #1ps;
        if (vld_count >= target) begin
          reached = 1'b1;
          break;
        end
      end
      if (!reached) begin
        check(1'b0,
              $sformatf("Timeout waiting for o_vld count %0d; current count=%0d",
                        target, vld_count));

        // A functional timeout means the transaction/response queues are no
        // longer trustworthy for later tests. Stop here instead of producing
        // hundreds of misleading cascade failures.
        $display("[%0t] FATAL [%s] Functional timeout; stopping regression to avoid scoreboard cascade.",
                 $time, current_test);
        $display("         pending MOSI transactions = %0d", exp_mosi_q.size());
        $display("         pending MISO responses    = %0d", rsp_miso_q.size());
        $display("         pending expected angles   = %0d", exp_angle_q.size());
        $fatal(1, "Functional timeout in %s", current_test);
      end
    end
  endtask

  task automatic ensure_quiet(input int unsigned cycles);
    int unsigned start_transactions;
    begin
      start_transactions = transaction_count;
      repeat (cycles) @(posedge i_clk);
      check(transaction_count == start_transactions,
            $sformatf("Unexpected SPI transaction while idle; before=%0d after=%0d",
                      start_transactions, transaction_count));
    end
  endtask

  // --------------------------------------------------------------------------
  // AS5048A behavioral slave model
  //
  // It supplies one queued response per CSn-low transaction.
  // MISO is intentionally updated DS_TMISO_MAX after each rising SCLK edge,
  // i.e. at the datasheet's worst-case output-valid delay.
  // --------------------------------------------------------------------------
  bit          monitor_enable = 1'b0;
  bit          txn_active     = 1'b0;
  bit          active_expected_valid = 1'b0;
  logic [15:0] active_expected_mosi;
  logic [15:0] active_sensor_rsp;
  logic [15:0] observed_mosi;
  int unsigned sclk_rise_count;
  int unsigned sclk_fall_count;

  time sensor_miso_delay = DS_TMISO_MAX;

  // Timing state.
  realtime last_cs_fall;
  realtime last_cs_rise;
  realtime last_sclk_rise;
  realtime last_sclk_fall;
  realtime last_mosi_change;
  realtime prev_mosi_change;

  bit last_cs_rise_valid   = 1'b0;
  bit last_sclk_rise_valid = 1'b0;
  bit last_sclk_fall_valid = 1'b0;

  // Measured minima for the final report.
  realtime min_tl          = 1.0e30;
  realtime min_tclk        = 1.0e30;
  realtime min_tclkl       = 1.0e30;
  realtime min_tclkh       = 1.0e30;
  realtime min_th          = 1.0e30;
  realtime min_tcsnh       = 1.0e30;
  realtime min_tmosi_setup = 1.0e30;
  realtime min_miso_sample = 1.0e30;

  // Track MOSI changes. Because RTL can change MOSI in the same simulation
  // time slot as a falling SCLK edge, both the current and previous timestamps
  // are retained. The setup checker uses the last value that was already valid
  // before the sampling edge, not a zero-delay "next bit" update.
  always @(o_mosi) begin
    prev_mosi_change = last_mosi_change;
    last_mosi_change = $realtime;
  end

  // Start of SPI frame / CSn falling edge.
  always @(negedge o_n_cs) begin
    realtime dt;

    if (monitor_enable && i_rst !== 1'b1) begin
      check(!txn_active, "CSn fell while a transaction was already active");
      txn_active = 1'b1;

      if (last_cs_rise_valid) begin
        dt = $realtime - last_cs_rise;
        if (dt < min_tcsnh) min_tcsnh = dt;
        check(dt >= DS_TCSNH_MIN,
              $sformatf("TCSnH violation: CSn high time=%0.3f ns, min=%0.3f ns",
                        dt, realtime'(DS_TCSNH_MIN)));
      end

      last_cs_fall       = $realtime;
      sclk_rise_count    = 0;
      sclk_fall_count    = 0;
      observed_mosi      = '0;
      last_sclk_rise_valid = 1'b0;
      last_sclk_fall_valid = 1'b0;

      if (exp_mosi_q.size() == 0 || rsp_miso_q.size() == 0) begin
        active_expected_valid = 1'b0;
        active_expected_mosi  = 16'hXXXX;
        active_sensor_rsp     = 16'h0000;
        check(1'b0, "Unexpected SPI transaction: scoreboard queue is empty");
      end
      else begin
        active_expected_valid = 1'b1;
        active_expected_mosi  = exp_mosi_q[0];
        active_sensor_rsp     = rsp_miso_q[0];
      end
    end
  end

  // Rising SCLK:
  //   - AS5048A is allowed up to 20 ns to make MISO valid.
  //   - MOSI is captured here only for scoreboard reconstruction because it is
  //     stable through the complete high phase before the sensor's falling-edge
  //     sample. This avoids zero-delay RTL race artifacts at the falling edge.
  always @(posedge o_sclk) begin
    realtime dt;

    if (monitor_enable && i_rst !== 1'b1) begin
      check(o_n_cs === 1'b0, "SCLK rose while CSn was high");
      check(txn_active, "SCLK rose without an active CSn-low transaction");

      if (sclk_rise_count == 0) begin
        dt = $realtime - last_cs_fall;
        if (dt < min_tl) min_tl = dt;
        check(dt >= DS_TL_MIN,
              $sformatf("tL violation: CSn falling -> first SCLK rising=%0.3f ns, min=%0.3f ns",
                        dt, realtime'(DS_TL_MIN)));
      end

      if (last_sclk_fall_valid) begin
        dt = $realtime - last_sclk_fall;
        if (dt < min_tclkl) min_tclkl = dt;
        check(dt >= DS_TCLKL_MIN,
              $sformatf("tCLKL violation: SCLK low=%0.3f ns, min=%0.3f ns",
                        dt, realtime'(DS_TCLKL_MIN)));
      end

      if (last_sclk_rise_valid) begin
        dt = $realtime - last_sclk_rise;
        if (dt < min_tclk) min_tclk = dt;
        check(dt >= DS_TCLK_MIN,
              $sformatf("TCLK violation: period=%0.3f ns, min=%0.3f ns",
                        dt, realtime'(DS_TCLK_MIN)));
      end

      last_sclk_rise       = $realtime;
      last_sclk_rise_valid = 1'b1;

      if (sclk_rise_count < 16) begin
        observed_mosi = {observed_mosi[14:0], o_mosi};

        // Worst-case AS5048A MISO propagation delay.
        i_miso <= #(sensor_miso_delay)
                  active_sensor_rsp[15-sclk_rise_count];
      end
      else begin
        check(1'b0,
              $sformatf("Too many SCLK rising edges in frame: %0d",
                        sclk_rise_count + 1));
      end

      sclk_rise_count++;
    end
  end

  // Falling SCLK:
  //   - AS5048A samples MOSI here.
  //   - DUT samples MISO here.
  always @(negedge o_sclk) begin
    realtime dt;
    realtime valid_since;
    realtime setup_dt;
    realtime miso_sample_dt;

    if (monitor_enable && i_rst !== 1'b1 && o_n_cs === 1'b0) begin
      check(txn_active, "SCLK fell without an active transaction");

      if (last_sclk_rise_valid) begin
        dt = $realtime - last_sclk_rise;
        if (dt < min_tclkh) min_tclkh = dt;
        check(dt >= DS_TCLKH_MIN,
              $sformatf("tCLKH violation: SCLK high=%0.3f ns, min=%0.3f ns",
                        dt, realtime'(DS_TCLKH_MIN)));

        // This is the time the master gives the sensor from the rising edge
        // (where the AS5048A starts updating MISO) to the falling edge where
        // this DUT samples it. With a 20 ns max tMISO, this must be safely >20ns.
        miso_sample_dt = dt;
        if (miso_sample_dt < min_miso_sample)
          min_miso_sample = miso_sample_dt;
        check(miso_sample_dt >= DS_TMISO_MAX,
              $sformatf("Insufficient MISO sampling margin: rising->falling=%0.3f ns, sensor may need %0.3f ns",
                        miso_sample_dt, realtime'(DS_TMISO_MAX)));
      end

      // Ignore a same-timestamp RTL update of the *next* MOSI bit.
      if (last_mosi_change == $realtime)
        valid_since = prev_mosi_change;
      else
        valid_since = last_mosi_change;

      setup_dt = $realtime - valid_since;
      if (setup_dt < min_tmosi_setup) min_tmosi_setup = setup_dt;
      check(setup_dt >= DS_TMOSI_MIN,
            $sformatf("tMOSI violation: data valid only %0.3f ns before falling edge, min=%0.3f ns",
                      setup_dt, realtime'(DS_TMOSI_MIN)));

      last_sclk_fall       = $realtime;
      last_sclk_fall_valid = 1'b1;
      sclk_fall_count++;

      if (sclk_fall_count > 16)
        check(1'b0,
              $sformatf("Too many SCLK falling edges in frame: %0d",
                        sclk_fall_count));
    end
  end

  // End of SPI frame / CSn rising edge.
  always @(posedge o_n_cs) begin
    realtime dt;
    logic [15:0] q_dummy;

    // MISO is don't-care when the sensor is deselected.
    i_miso <= 1'b0;

    if (monitor_enable) begin
      last_cs_rise       = $realtime;
      last_cs_rise_valid = 1'b1;

      if (txn_active) begin
        if (i_rst === 1'b1) begin
          // Reset is intentionally allowed to abort an in-progress frame.
          aborted_count++;
          txn_active             = 1'b0;
          active_expected_valid  = 1'b0;
        end
        else begin
          check(o_sclk === 1'b0, "CSn rose while SCLK was not low");
          check(sclk_rise_count == 16,
                $sformatf("Expected 16 SCLK rising edges, observed %0d",
                          sclk_rise_count));
          check(sclk_fall_count == 16,
                $sformatf("Expected 16 SCLK falling edges, observed %0d",
                          sclk_fall_count));

          if (last_sclk_fall_valid) begin
            dt = $realtime - last_sclk_fall;
            if (dt < min_th) min_th = dt;
            check(dt >= DS_TH_MIN,
                  $sformatf("tH violation: last SCLK falling -> CSn rising=%0.3f ns, min=%0.3f ns",
                            dt, realtime'(DS_TH_MIN)));
          end

          if (active_expected_valid) begin
            check(observed_mosi === active_expected_mosi,
                  $sformatf("MOSI frame mismatch: expected 0x%04h observed 0x%04h",
                            active_expected_mosi, observed_mosi));

            check(even_parity(observed_mosi),
                  $sformatf("DUT transmitted command with bad parity: 0x%04h",
                            observed_mosi));

            if (exp_mosi_q.size() != 0)
              q_dummy = exp_mosi_q.pop_front();
            if (rsp_miso_q.size() != 0)
              q_dummy = rsp_miso_q.pop_front();
          end

          transaction_count++;
          txn_active            = 1'b0;
          active_expected_valid = 1'b0;
        end
      end
    end
  end

  // --------------------------------------------------------------------------
  // Output scoreboard
  // --------------------------------------------------------------------------
  realtime vld_rise_time;
  bit      vld_rise_valid = 1'b0;

  always @(posedge o_vld) begin
    logic [13:0] expected_angle;

    if (monitor_enable && i_rst !== 1'b1) begin
      vld_count++;
      vld_rise_time  = $realtime;
      vld_rise_valid = 1'b1;

      check(o_n_cs === 1'b1, "o_vld asserted while CSn was active");
      check(o_sclk === 1'b0, "o_vld asserted while SCLK was not idle low");

      if (exp_angle_q.size() == 0) begin
        check(1'b0,
              $sformatf("Unexpected o_vld pulse; angle=0x%04h",
                        o_raw_angle));
      end
      else begin
        expected_angle = exp_angle_q.pop_front();
        check(o_raw_angle === expected_angle,
              $sformatf("Angle mismatch: expected 0x%04h observed 0x%04h",
                        expected_angle, o_raw_angle));
      end
    end
  end

  always @(negedge o_vld) begin
    realtime dt;
    if (monitor_enable && vld_rise_valid && i_rst !== 1'b1) begin
      dt = $realtime - vld_rise_time;
      check((dt >= (CLK_PERIOD-1ps)) && (dt <= (CLK_PERIOD+1ps)),
            $sformatf("o_vld pulse width=%0.3f ns, expected one i_clk period=%0.3f ns",
                      dt, realtime'(CLK_PERIOD)));
      vld_rise_valid = 1'b0;
    end
  end

  // The angle register is only allowed to change on a successful response.
  // Wait one precision tick so the combinational o_vld decode of S_DONE has
  // settled in mixed-language simulation.
  always @(o_raw_angle) begin
    if (monitor_enable && i_rst !== 1'b1) begin
      #1ps;
      check(o_vld === 1'b1,
            $sformatf("o_raw_angle changed to 0x%04h without o_vld",
                      o_raw_angle));
    end
  end

  // Global pin-level sanity checks.
  always @(posedge o_sclk or negedge o_sclk) begin
    if (monitor_enable && i_rst !== 1'b1)
      check(o_n_cs === 1'b0, "SCLK toggled while CSn was high");
  end

  // --------------------------------------------------------------------------
  // Main test sequence
  // --------------------------------------------------------------------------
  initial begin : main_test
    int unsigned target_vld;
    int unsigned start_txns;
    int unsigned start_vlds;
    int unsigned seed;
    logic [13:0] rand_angle;
    logic [13:0] directed_angles [0:5];

    // Deterministic stimulus seed.
    seed = 32'hA5048A01;

    i_rst   = 1'b1;
    i_start = 1'b0;
    i_miso  = 1'b0;

    directed_angles[0] = 14'h0000;
    directed_angles[1] = 14'h3FFF;
    directed_angles[2] = 14'h0001;
    directed_angles[3] = 14'h2000;
    directed_angles[4] = 14'h1555;
    directed_angles[5] = 14'h2AAA;

    // Let VHDL initialization and synchronous reset settle.
    repeat (5) @(posedge i_clk);
    #1ps;

    begin_test("reset_and_command_encoding");
    check(o_n_cs      === 1'b1, "CSn must be high during reset");
    check(o_sclk      === 1'b0, "SCLK must be low during reset");
    check(o_vld       === 1'b0, "o_vld must be low during reset");
    check(o_raw_angle === 14'h0000, "Angle must reset to zero");

    // Datasheet-derived command encodings:
    // READ angle address 0x3FFF -> 0xFFFF
    // READ clear-error address 0x0001 -> 0x4001
    check(CMD_READ_ANGLE  == 16'hFFFF,
          $sformatf("READ ANGLE command encoding wrong: 0x%04h",
                    CMD_READ_ANGLE));
    check(CMD_CLEAR_ERROR == 16'h4001,
          $sformatf("CLEAR ERROR command encoding wrong: 0x%04h",
                    CMD_CLEAR_ERROR));

    // Reset must dominate start.
    @(negedge i_clk);
    i_start = 1'b1;
    repeat (2) begin
      @(posedge i_clk);
      #1ps;
      check(o_n_cs === 1'b1, "i_start during reset pulled CSn low");
      check(o_sclk === 1'b0, "i_start during reset toggled SCLK");
      check(o_vld  === 1'b0, "i_start during reset asserted o_vld");
    end
    @(negedge i_clk);
    i_start = 1'b0;
    end_test();

    @(negedge i_clk);
    i_rst = 1'b0;
    monitor_enable = 1'b1;

    // ----------------------------------------------------------------------
    begin_test("directed_valid_reads_and_bit_order");
    foreach (directed_angles[idx]) begin
      target_vld = vld_count + 1;
      expect_successful_read(directed_angles[idx]);
      pulse_start();
      wait_for_vld_count(target_vld, 20000);
      #1ps;
      check_queues_empty();
    end
    end_test();

    // ----------------------------------------------------------------------
    // The first response belongs to the transaction before the current READ.
    // Deliberately make it both EF=1 and parity-bad. A correct implementation
    // must ignore it while in CMD_PHASE and accept the valid second response.
    begin_test("ignore_pipeline_response_during_command_phase");
    target_vld = vld_count + 1;

    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h3ABC, 1'b1, 1'b1)
    );
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h0ACE, 1'b0, 1'b0)
    );
    exp_angle_q.push_back(14'h0ACE);

    start_txns = transaction_count;
    pulse_start();
    wait_for_vld_count(target_vld, 20000);
    #1ps;
    check(transaction_count == start_txns + 2,
          "Command-phase response incorrectly caused retry/error handling");
    check_queues_empty();
    end_test();

    // ----------------------------------------------------------------------
    begin_test("bad_parity_response_retries_full_read");
    target_vld = vld_count + 1;

    // Initial command transmission.
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h0123, 1'b0, 1'b0)
    );
    // First read response: parity error.
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h1555, 1'b0, 1'b1)
    );
    // Retry command phase.
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h0777, 1'b0, 1'b0)
    );
    // Retry read response: valid.
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h2D3A, 1'b0, 1'b0)
    );
    exp_angle_q.push_back(14'h2D3A);

    start_txns = transaction_count;
    pulse_start();
    wait_for_vld_count(target_vld, 30000);
    #1ps;
    check(transaction_count == start_txns + 4,
          $sformatf("Parity retry should take 4 transmissions, observed %0d",
                    transaction_count - start_txns));
    check_queues_empty();
    end_test();

    // ----------------------------------------------------------------------
    begin_test("repeated_bad_parity_then_recovery");
    target_vld = vld_count + 1;

    // Attempt 1
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0000, 1'b0, 1'b0));
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0001, 1'b0, 1'b1));
    // Attempt 2
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0000, 1'b0, 1'b0));
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h3FFE, 1'b0, 1'b1));
    // Attempt 3 succeeds
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0000, 1'b0, 1'b0));
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h1EAD, 1'b0, 1'b0));
    exp_angle_q.push_back(14'h1EAD);

    start_txns = transaction_count;
    pulse_start();
    wait_for_vld_count(target_vld, 50000);
    #1ps;
    check(transaction_count == start_txns + 6,
          $sformatf("Two parity retries should take 6 transmissions, observed %0d",
                    transaction_count - start_txns));
    check_queues_empty();
    end_test();

    // ----------------------------------------------------------------------
    begin_test("error_flag_clear_sequence");
    target_vld = vld_count + 1;

    // 1) READ ANGLE command
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h0000, 1'b0, 1'b0)
    );
    // 2) READ ANGLE response reports EF=1
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h2222, 1'b1, 1'b0)
    );
    // 3) DUT must issue CLEAR ERROR FLAG (read address 0x0001)
    expect_spi_transaction(
      CMD_CLEAR_ERROR,
      make_read_rsp(14'h1111, 1'b1, 1'b0)
    );
    // 4) DUT resumes with READ ANGLE command; response here is the clear-error
    //    register result and is intentionally ignored by CMD_PHASE.
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h0004, 1'b1, 1'b0)
    );
    // 5) Following READ ANGLE returns a new valid angle with EF clear.
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h3141, 1'b0, 1'b0)
    );
    exp_angle_q.push_back(14'h3141);

    start_txns = transaction_count;
    pulse_start();
    wait_for_vld_count(target_vld, 50000);
    #1ps;
    check(transaction_count == start_txns + 5,
          $sformatf("EF recovery should take 5 transmissions, observed %0d",
                    transaction_count - start_txns));
    check_queues_empty();
    end_test();

    // ----------------------------------------------------------------------
    // In S_CHECK the RTL tests parity before EF. A frame with both faults must
    // trigger parity retry, NOT the clear-error command yet.
    begin_test("parity_has_priority_over_error_flag");
    target_vld = vld_count + 1;

    // Initial attempt
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0000, 1'b0, 1'b0));
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0101, 1'b1, 1'b1)); // both faults

    // Parity retry: command phase must still be READ ANGLE, not CLEAR ERROR.
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0000, 1'b0, 1'b0));
    // Now return valid parity with EF=1.
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0101, 1'b1, 1'b0));

    // Clear-error recovery.
    expect_spi_transaction(CMD_CLEAR_ERROR,
                           make_read_rsp(14'h0000, 1'b1, 1'b0));
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h0004, 1'b1, 1'b0));
    expect_spi_transaction(CMD_READ_ANGLE,
                           make_read_rsp(14'h2BEE, 1'b0, 1'b0));
    exp_angle_q.push_back(14'h2BEE);

    start_txns = transaction_count;
    pulse_start();
    wait_for_vld_count(target_vld, 70000);
    #1ps;
    check(transaction_count == start_txns + 7,
          $sformatf("Combined parity/EF scenario expected 7 transmissions, observed %0d",
                    transaction_count - start_txns));
    check_queues_empty();
    end_test();

    // ----------------------------------------------------------------------
    begin_test("start_pulse_while_busy_is_ignored");
    target_vld = vld_count + 1;
    start_txns  = transaction_count;

    expect_successful_read(14'h0BAD);
    pulse_start();

    // Inject another start while the first SPI frame is in progress.
    @(negedge o_n_cs);
    repeat (4) @(posedge o_sclk);
    pulse_start();

    wait_for_vld_count(target_vld, 30000);
    #1ps;
    check(transaction_count == start_txns + 2,
          $sformatf("Busy start corrupted/created a transfer; expected 2 transactions, got %0d",
                    transaction_count - start_txns));
    check_queues_empty();

    // No latent request should remain from the ignored busy pulse.
    ensure_quiet(100);
    end_test();

    // ----------------------------------------------------------------------
    // CSn being high does not necessarily mean the FSM is in S_IDLE; between
    // the two pipeline frames it is in S_CHECK/S_CS_HIGH. A start pulse there
    // must not disturb or queue another operation.
    begin_test("start_during_interframe_cs_high_is_ignored");
    target_vld = vld_count + 1;
    start_txns  = transaction_count;

    expect_successful_read(14'h1A2B);
    pulse_start();

    // First frame ends here; inject start during the mandatory CSn-high gap.
    @(posedge o_n_cs);
    pulse_start();

    wait_for_vld_count(target_vld, 30000);
    #1ps;
    check(transaction_count == start_txns + 2,
          $sformatf("Inter-frame start pulse created/corrupted traffic; expected 2 transactions, got %0d",
                    transaction_count - start_txns));
    check_queues_empty();
    ensure_quiet(100);
    end_test();

    // ----------------------------------------------------------------------
    begin_test("synchronous_reset_mid_transfer_aborts_and_recovers");
    start_vlds = vld_count;

    // Queue one transaction only; it will be deliberately aborted.
    expect_spi_transaction(
      CMD_READ_ANGLE,
      make_read_rsp(14'h2AAA, 1'b0, 1'b0)
    );

    pulse_start();
    @(negedge o_n_cs);
    repeat (4) @(posedge o_sclk);

    // Remove pending expectations before intentionally aborting the frame.
    exp_mosi_q.delete();
    rsp_miso_q.delete();

    @(negedge i_clk);
    i_rst   = 1'b1;
    i_start = 1'b0;

    @(posedge i_clk);
    #1ps;
    check(o_n_cs === 1'b1, "Reset did not immediately return CSn high");
    check(o_sclk === 1'b0, "Reset did not return SCLK low");
    check(o_vld  === 1'b0, "Reset unexpectedly asserted o_vld");
    check(o_raw_angle === 14'h0000, "Reset did not clear o_raw_angle");

    repeat (3) @(posedge i_clk);
    check(vld_count == start_vlds,
          "Aborted transaction produced an o_vld pulse");

    @(negedge i_clk);
    i_rst = 1'b0;

    // Prove the FSM/shift counters recover cleanly.
    target_vld = vld_count + 1;
    expect_successful_read(14'h1234);
    pulse_start();
    wait_for_vld_count(target_vld, 30000);
    #1ps;
    check_queues_empty();
    end_test();

    // ----------------------------------------------------------------------
    // i_start is level-sensitive in S_IDLE. Holding it high should therefore
    // launch another complete read after the first operation returns to IDLE.
    begin_test("held_start_retriggers_after_return_to_idle");
    target_vld = vld_count + 2;
    start_txns  = transaction_count;

    expect_successful_read(14'h0111);
    expect_successful_read(14'h0222);

    @(negedge i_clk);
    i_start = 1'b1;

    wait_for_vld_count(target_vld, 60000);

    // Drop start while the second operation is in DONE/return-to-IDLE path so
    // that a third operation is not launched.
    @(negedge i_clk);
    i_start = 1'b0;

    #1ps;
    check(transaction_count == start_txns + 4,
          $sformatf("Held start should cause exactly two 2-frame reads here; got %0d transactions",
                    transaction_count - start_txns));
    check_queues_empty();
    ensure_quiet(100);
    end_test();

    // ----------------------------------------------------------------------
    begin_test("random_valid_angle_sweep");
    for (int i = 0; i < RANDOM_ITERS; i++) begin
      rand_angle = $urandom(seed) & 14'h3FFF;
      target_vld = vld_count + 1;
      expect_successful_read(rand_angle);
      pulse_start();
      wait_for_vld_count(target_vld, 20000);
      #1ps;
      check_queues_empty();
    end
    end_test();

    // ----------------------------------------------------------------------
    begin_test("final_idle_and_timing_summary");
    ensure_quiet(100);

    check(o_n_cs === 1'b1, "Final CSn is not idle high");
    check(o_sclk === 1'b0, "Final SCLK is not idle low");
    check(o_vld  === 1'b0, "Final o_vld is unexpectedly high");

    $display("\n---------------- AS5048A TIMING OBSERVED ----------------");
    $display("min tL                  = %0.3f ns   (required >= 350 ns)", min_tl);
    $display("min TCLK                = %0.3f ns   (required >= 100 ns)", min_tclk);
    $display("min tCLKL               = %0.3f ns   (required >=  50 ns)", min_tclkl);
    $display("min tCLKH               = %0.3f ns   (required >=  50 ns)", min_tclkh);
    $display("min tH                  = %0.3f ns   (required >=  50 ns)", min_th);
    $display("min TCSnH               = %0.3f ns   (required >= 350 ns)", min_tcsnh);
    $display("min MOSI setup          = %0.3f ns   (required >=  20 ns)", min_tmosi_setup);
    $display("min MISO rise->sample   = %0.3f ns   (sensor tMISO max = 20 ns)", min_miso_sample);
    $display("sensor model tMISO      = %0t (worst-case datasheet delay)", sensor_miso_delay);
    end_test();

    // Final global scoreboard checks.
    check_queues_empty();

    $display("\n===================== TESTBENCH SUMMARY =====================");
    $display("Checks              : %0d", check_count);
    $display("Passed              : %0d", pass_count);
    $display("Failed              : %0d", fail_count);
    $display("Completed SPI frames: %0d", transaction_count);
    $display("Reset-aborted frames: %0d", aborted_count);
    $display("o_vld pulses        : %0d", vld_count);
    $display("=============================================================");

    if (fail_count == 0) begin
      $display("RESULT: PASS");
      $finish;
    end
    else begin
      $fatal(1, "RESULT: FAIL -- %0d check(s) failed", fail_count);
    end
  end

  // Hard watchdog so a broken FSM cannot hang regression forever.
  initial begin
    #5ms;
    $fatal(1, "TESTBENCH WATCHDOG TIMEOUT");
  end

endmodule
