`timescale 1ns/1ps

// ============================================================================
// uart_pipeline_scoreboard_tb_v2.sv
//
// End-to-end, self-checking, self-reporting mixed-language testbench for:
//
//   UART RX pin -> uart_rx -> parser -> register_file
//   register_file -> serializer -> uart_tx -> UART TX pin
//
// The DUT blocks are VHDL entities instantiated directly from SystemVerilog.
// This style is supported by mixed-language simulators such as Questa/ModelSim
// and Xcelium. Compile the VHDL packages/entities first, then compile this file.
//
// REGISTER MAP / CONTROL DEFINITION
// ---------------------------------
// The constants below are copied exactly from the supplied reg_pkg.vhd:
//   writable  : 0x00..0x0F
//   read-only : 0x10..0x17
//   LAST_ADDR : 0x17
//   CONTROL   : ENABLE=0, MODE=[2:1], CALIBRATE=3, CLEAR_FAULT=4,
//               MANUAL_ELEC=5, SENSOR_INV=6, WATCHDOG=7
// The scoreboard therefore has no register-map assumptions or address plusargs.
//
// Optional strict checks:
//   +STRICT_BURST_READS    : dropped read responses during overlapping traffic
//                            are errors instead of warnings.
//   +STRICT_RESYNC         : inability to recover in-band from a truncated frame
//                            is an error instead of a warning.
//   +WARN_RX_CONTIGUOUS     : downgrade the standards-compliant no-extra-idle
//                            UART byte-stream throughput check to a warning.
//
// IMPORTANT RX THROUGHPUT NOTE
// ----------------------------
// Normal functional tests intentionally add a small idle guard between UART bytes
// so one RX throughput defect cannot desynchronize every later scoreboard queue.
// A separate isolated test sends truly contiguous 8N1 bytes (only the required stop
// bit, no extra idle) and reports whether uart_rx can sustain full line rate.
//
// Optional random length:
//   +RANDOM_ITERS=<N>      : default 24
//
// Clock:
//   CLK_HZ defaults to the supplied common_pkg.C_CLK_FREQ value: 50 MHz.
//   uart_pkg.C_STEP_SIZE/C_STEP_SIZE_STD are therefore exercised at the same
//   system-clock frequency as the RTL.
// ============================================================================

module uart_pipeline_scoreboard_tb;

  timeunit 1ns;
  timeprecision 1ps;

  // --------------------------------------------------------------------------
  // Protocol constants copied from uart_pkg.vhd
  // --------------------------------------------------------------------------
  localparam int unsigned BAUD_RATE = 921_600;
  localparam logic [7:0] SYNC_BYTE  = 8'hAA;
  localparam logic [7:0] READ_CMD   = 8'h01;
  localparam logic [7:0] WRITE_CMD  = 8'h02;

  parameter int unsigned CLK_HZ = 50_000_000;
  localparam time CLK_PERIOD = 1s / CLK_HZ;
  localparam time UART_BIT    = 1s / BAUD_RATE;
  // Guard used by ordinary regression traffic. This is intentionally NOT part of
  // the UART protocol requirement; it only isolates end-to-end functional tests
  // from a receiver throughput failure. The dedicated contiguous-byte test below
  // uses zero guard and is the authoritative full-rate UART check.
  localparam realtime NORMAL_RX_GUARD_BITS = 0.50;

  localparam int N_WR = 16;
  localparam int N_RO = 8;

  // --------------------------------------------------------------------------
  // Clock/reset and UART pins
  // --------------------------------------------------------------------------
  logic clk = 1'b0;
  logic rst = 1'b1;
  logic uart_rx_pin = 1'b1;
  logic uart_tx_pin;

  always #(CLK_PERIOD/2) clk = ~clk;

  // --------------------------------------------------------------------------
  // Interconnect: uart_rx -> parser -> register file -> serializer -> uart_tx
  // --------------------------------------------------------------------------
  logic [7:0]  rx_data;
  logic        rx_vld;
  logic        rx_baud_dbg;
  integer      rx_bit_dbg;

  logic        req_vld;
  logic        req_rw;
  logic [31:0] req_data;
  logic [7:0]  req_addr;

  logic        rf_rsp_vld;
  logic [31:0] rf_rsp_data;
  logic [7:0]  rf_rsp_addr;

  logic        ser_vld;
  logic [7:0]  ser_byte;

  logic        tx_done;
  logic        tx_rdy;

  logic        clear_fault_pulse;
  logic        calibrate_pulse;

  // Writable register outputs in the same order as register_file.vhd.
  logic [31:0] hw_wr [0:N_WR-1];

  // Read-only inputs in the same order as register_file.vhd.
  logic [31:0] ro_in [0:N_RO-1];

  // --------------------------------------------------------------------------
  // Direct mixed-language DUT instantiation
  // --------------------------------------------------------------------------
  uart_rx u_uart_rx (
    .i_clk  (clk),
    .i_rst  (rst),
    .i_rx   (uart_rx_pin),
    .o_data (rx_data),
    .o_vld  (rx_vld),
    .o_baud (rx_baud_dbg),
    .o_bit  (rx_bit_dbg)
  );

  parser u_parser (
    .i_clk  (clk),
    .i_rst  (rst),
    .i_vld  (rx_vld),
    .i_byte (rx_data),
    .o_vld  (req_vld),
    .o_rw   (req_rw),
    .o_data (req_data),
    .o_addr (req_addr)
  );

  register_file u_register_file (
    .i_clk                (clk),
    .i_rst                (rst),
    .i_req_vld            (req_vld),
    .i_req_rw             (req_rw),
    .i_req_data           (req_data),
    .i_req_addr           (req_addr),
    .o_rsp_vld            (rf_rsp_vld),
    .o_rsp_data           (rf_rsp_data),
    .o_rsp_addr           (rf_rsp_addr),

    .REG_FOC_KP_GAIN      (hw_wr[0]),
    .REG_FOC_KI_GAIN      (hw_wr[1]),
    .REG_FOC_WINDUP_MAX   (hw_wr[2]),
    .REG_FOC_VOLTAGE_MAX  (hw_wr[3]),
    .REG_VEL_KP_GAIN      (hw_wr[4]),
    .REG_VEL_KI_GAIN      (hw_wr[5]),
    .REG_VEL_WINDUP_MAX   (hw_wr[6]),
    .REG_VEL_CURRENT_MAX  (hw_wr[7]),
    .REG_POS_KP_GAIN      (hw_wr[8]),
    .REG_POS_MAX_VELOCITY (hw_wr[9]),
    .REG_POS_DEADBAND_TCK (hw_wr[10]),
    .REG_CONTROL          (hw_wr[11]),
    .REG_TARGET_DQ        (hw_wr[12]),
    .REG_TARGET_VEL       (hw_wr[13]),
    .REG_TARGET_POS       (hw_wr[14]),
    .REG_MANUAL_ELEC      (hw_wr[15]),

    .o_clear_fault_pulse  (clear_fault_pulse),
    .o_calibrate_pulse    (calibrate_pulse),

    .i_status             (ro_in[0]),
    .i_actual_pos         (ro_in[1]),
    .i_actual_vel         (ro_in[2]),
    .i_phase_ab           (ro_in[3]),
    .i_phase_c_id         (ro_in[4]),
    .i_iq_elec            (ro_in[5]),
    .i_cal_off_ab         (ro_in[6]),
    .i_cal_off_c          (ro_in[7])
  );

  serializer u_serializer (
    .i_clk  (clk),
    .i_rst  (rst),
    .i_vld  (rf_rsp_vld),
    .i_data (rf_rsp_data),
    .i_addr (rf_rsp_addr),
    .i_rdy  (tx_rdy),
    .o_vld  (ser_vld),
    .o_byte (ser_byte)
  );

  uart_tx u_uart_tx (
    .i_clk  (clk),
    .i_rst  (rst),
    .i_vld  (ser_vld),
    .i_byte (ser_byte),
    .o_tx   (uart_tx_pin),
    .o_done (tx_done),
    .o_rdy  (tx_rdy)
  );

  // --------------------------------------------------------------------------
  // Exact register map copied from reg_pkg.vhd
  // --------------------------------------------------------------------------
  byte unsigned wr_addr [0:N_WR-1];
  byte unsigned ro_addr [0:N_RO-1];
  string wr_name [0:N_WR-1];
  string ro_name [0:N_RO-1];

  byte unsigned last_addr;
  int ctrl_enable_bit;
  int ctrl_mode_lsb;
  int ctrl_mode_msb;
  int ctrl_calibrate_bit;
  int ctrl_clear_fault_bit;
  int ctrl_manual_elec_bit;
  int ctrl_sensor_inv_bit;
  int ctrl_watchdog_bit;

  bit strict_burst_reads;
  bit strict_resync;
  bit warn_rx_contiguous;
  bit suppress_rx_scoreboard;
  bit suppress_req_scoreboard;
  int random_iters;

  task automatic plusarg_int(input string key, ref int dst);
    int tmp;
    if ($value$plusargs({key,"=%d"}, tmp)) dst = tmp;
  endtask

  task automatic init_config;
    // Exact values from reg_pkg.vhd.
    wr_addr[0]  = 8'h00; // C_FOC_KP_GAIN_ADDR
    wr_addr[1]  = 8'h01; // C_FOC_KI_GAIN_ADDR
    wr_addr[2]  = 8'h02; // C_FOC_WINDUP_MAX
    wr_addr[3]  = 8'h03; // C_FOC_VOLTAGE_MAX
    wr_addr[4]  = 8'h04; // C_VEL_KP_GAIN_ADDR
    wr_addr[5]  = 8'h05; // C_VEL_KI_GAIN_ADDR
    wr_addr[6]  = 8'h06; // C_VEL_WINDUP_MAX
    wr_addr[7]  = 8'h07; // C_VEL_CURRENT_MAX
    wr_addr[8]  = 8'h08; // C_POS_KP_GAIN_ADDR
    wr_addr[9]  = 8'h09; // C_POS_MAX_VELOCITY
    wr_addr[10] = 8'h0A; // C_POS_DEADBAND_TCK
    wr_addr[11] = 8'h0B; // C_CONTROL_ADDR
    wr_addr[12] = 8'h0C; // C_TARGET_DQ_ADDR
    wr_addr[13] = 8'h0D; // C_TARGET_VEL_ADDR
    wr_addr[14] = 8'h0E; // C_TARGET_POS_ADDR
    wr_addr[15] = 8'h0F; // C_MANUAL_ELEC_ADDR

    ro_addr[0] = 8'h10; // C_STATUS_ADDR
    ro_addr[1] = 8'h11; // C_ACTUAL_POS_ADDR
    ro_addr[2] = 8'h12; // C_ACTUAL_VEL_ADDR
    ro_addr[3] = 8'h13; // C_PHASE_AB_ADDR
    ro_addr[4] = 8'h14; // C_PHASE_C_ID_ADDR
    ro_addr[5] = 8'h15; // C_IQ_ELEC_ADDR
    ro_addr[6] = 8'h16; // C_CAL_OFF_AB_ADDR
    ro_addr[7] = 8'h17; // C_CAL_OFF_C_ADDR

    wr_name[0]  = "FOC_KP_GAIN";
    wr_name[1]  = "FOC_KI_GAIN";
    wr_name[2]  = "FOC_WINDUP_MAX";
    wr_name[3]  = "FOC_VOLTAGE_MAX";
    wr_name[4]  = "VEL_KP_GAIN";
    wr_name[5]  = "VEL_KI_GAIN";
    wr_name[6]  = "VEL_WINDUP_MAX";
    wr_name[7]  = "VEL_CURRENT_MAX";
    wr_name[8]  = "POS_KP_GAIN";
    wr_name[9]  = "POS_MAX_VELOCITY";
    wr_name[10] = "POS_DEADBAND_TCK";
    wr_name[11] = "CONTROL";
    wr_name[12] = "TARGET_DQ";
    wr_name[13] = "TARGET_VEL";
    wr_name[14] = "TARGET_POS";
    wr_name[15] = "MANUAL_ELEC";

    ro_name[0] = "STATUS";
    ro_name[1] = "ACTUAL_POS";
    ro_name[2] = "ACTUAL_VEL";
    ro_name[3] = "PHASE_AB";
    ro_name[4] = "PHASE_C_ID";
    ro_name[5] = "IQ_ELEC";
    ro_name[6] = "CAL_OFF_AB";
    ro_name[7] = "CAL_OFF_C";

    last_addr            = 8'h17;
    ctrl_enable_bit      = 0;
    ctrl_mode_lsb        = 1;
    ctrl_mode_msb        = 2;
    ctrl_calibrate_bit   = 3;
    ctrl_clear_fault_bit = 4;
    ctrl_manual_elec_bit = 5;
    ctrl_sensor_inv_bit  = 6;
    ctrl_watchdog_bit    = 7;

    strict_burst_reads = $test$plusargs("STRICT_BURST_READS");
    strict_resync      = $test$plusargs("STRICT_RESYNC");
    warn_rx_contiguous = $test$plusargs("WARN_RX_CONTIGUOUS");
    suppress_rx_scoreboard  = 1'b0;
    suppress_req_scoreboard = 1'b0;
    random_iters       = 24;
    plusarg_int("RANDOM_ITERS", random_iters);
  endtask

  task automatic print_config;
    $display("\n================ UART PIPELINE TB CONFIG ================");
    $display("CLK_HZ=%0d, BAUD=%0d, CLK_PERIOD=%0t, UART_BIT=%0t",
             CLK_HZ, BAUD_RATE, CLK_PERIOD, UART_BIT);
    $display("Register map and CONTROL bits are exact copies of supplied reg_pkg.vhd.");
    for (int i=0; i<N_WR; i++)
      $display("  WR[%02d] %-20s addr=0x%02h", i, wr_name[i], wr_addr[i]);
    for (int i=0; i<N_RO; i++)
      $display("  RO[%02d] %-20s addr=0x%02h", i, ro_name[i], ro_addr[i]);
    $display("  LAST_ADDR=0x%02h", last_addr);
    $display("  CONTROL bits: enable=%0d mode=[%0d:%0d] calibrate=%0d clear_fault=%0d manual_elec=%0d sensor_inv=%0d watchdog=%0d",
             ctrl_enable_bit, ctrl_mode_msb, ctrl_mode_lsb, ctrl_calibrate_bit,
             ctrl_clear_fault_bit, ctrl_manual_elec_bit, ctrl_sensor_inv_bit, ctrl_watchdog_bit);
    $display("  normal RX inter-byte guard = %0.2f bit-times", NORMAL_RX_GUARD_BITS);
    $display("  strict_burst_reads=%0b strict_resync=%0b warn_rx_contiguous=%0b random_iters=%0d",
             strict_burst_reads, strict_resync, warn_rx_contiguous, random_iters);
    $display("=========================================================\n");
  endtask

  task automatic validate_config;
    // Configuration mistakes should fail immediately instead of masquerading as
    // DUT protocol failures later in the regression.
    for (int i=0; i<N_WR; i++) begin
      pass_check(wr_addr[i] <= last_addr,
        $sformatf("configured writable address %s=0x%02h exceeds LAST_ADDR=0x%02h",
                  wr_name[i], wr_addr[i], last_addr));
      for (int j=i+1; j<N_WR; j++)
        pass_check(wr_addr[i] != wr_addr[j],
          $sformatf("duplicate writable addresses: %s and %s both 0x%02h",
                    wr_name[i], wr_name[j], wr_addr[i]));
      for (int j=0; j<N_RO; j++)
        pass_check(wr_addr[i] != ro_addr[j],
          $sformatf("address collision: writable %s and read-only %s both 0x%02h",
                    wr_name[i], ro_name[j], wr_addr[i]));
    end
    for (int i=0; i<N_RO; i++) begin
      pass_check(ro_addr[i] <= last_addr,
        $sformatf("configured read-only address %s=0x%02h exceeds LAST_ADDR=0x%02h",
                  ro_name[i], ro_addr[i], last_addr));
      for (int j=i+1; j<N_RO; j++)
        pass_check(ro_addr[i] != ro_addr[j],
          $sformatf("duplicate read-only addresses: %s and %s both 0x%02h",
                    ro_name[i], ro_name[j], ro_addr[i]));
    end
    pass_check((ctrl_enable_bit >= 0) && (ctrl_enable_bit < 32),
               "CTRL_ENABLE_BIT must be in [0,31]");
    pass_check((ctrl_calibrate_bit >= 0) && (ctrl_calibrate_bit < 32),
               "CTRL_CALIBRATE_BIT must be in [0,31]");
    pass_check((ctrl_clear_fault_bit >= 0) && (ctrl_clear_fault_bit < 32),
               "CTRL_CLEAR_FAULT_BIT must be in [0,31]");
    pass_check(ctrl_mode_lsb == 1 && ctrl_mode_msb == 2,
               "CONTROL MODE field must be exactly [2:1] per reg_pkg");
    pass_check(ctrl_manual_elec_bit == 5, "CONTROL MANUAL_ELEC bit must be 5 per reg_pkg");
    pass_check(ctrl_sensor_inv_bit  == 6, "CONTROL SENSOR_INV bit must be 6 per reg_pkg");
    pass_check(ctrl_watchdog_bit    == 7, "CONTROL WATCHDOG bit must be 7 per reg_pkg");
    pass_check(ctrl_enable_bit != ctrl_calibrate_bit,
               "CONTROL enable and calibrate bit indices collide");
    pass_check(ctrl_enable_bit != ctrl_clear_fault_bit,
               "CONTROL enable and clear-fault bit indices collide");
    pass_check(ctrl_calibrate_bit != ctrl_clear_fault_bit,
               "CONTROL calibrate and clear-fault bit indices collide");
  endtask

  // --------------------------------------------------------------------------
  // Reference model, expected transactions, coverage counters
  // --------------------------------------------------------------------------
  typedef struct packed {
    logic        rw;
    logic [7:0]  addr;
    logic [31:0] data;
  } req_t;

  typedef struct packed {
    logic [7:0]  addr;
    logic [31:0] data;
  } rsp_t;

  req_t exp_req_q[$];
  rsp_t exp_rf_rsp_q[$];
  rsp_t exp_tx_rsp_q[$];
  byte unsigned exp_rx_byte_q[$];
  byte unsigned exp_ser_byte_q[$];

  logic [31:0] ref_wr [0:N_WR-1];

  int wr_write_hits [0:N_WR-1];
  int wr_read_hits  [0:N_WR-1];
  int ro_write_hits [0:N_RO-1];
  int ro_read_hits  [0:N_RO-1];

  int checks = 0;
  int errors = 0;
  int warnings = 0;
  int tests_run = 0;
  int tests_passed = 0;
  int tests_failed = 0;
  int errors_at_test_start;

  int rx_bytes_seen = 0;
  int parser_reqs_seen = 0;
  int rf_rsps_seen = 0;
  int serializer_bytes_seen = 0;
  int tx_bytes_seen = 0;
  int tx_frames_seen = 0;
  int backpressure_cycles = 0;

  int expected_cal_pulses = 0;
  int expected_clear_pulses = 0;
  int actual_cal_pulses = 0;
  int actual_clear_pulses = 0;

  logic prev_req_vld;
  logic prev_rsp_vld;
  logic prev_cal_pulse;
  logic prev_clear_pulse;

  function automatic int wr_index(input logic [7:0] a);
    for (int i=0; i<N_WR; i++) if (a == wr_addr[i]) return i;
    return -1;
  endfunction

  function automatic int ro_index(input logic [7:0] a);
    for (int i=0; i<N_RO; i++) if (a == ro_addr[i]) return i;
    return -1;
  endfunction

  function automatic logic [31:0] model_read(input logic [7:0] a);
    int wi;
    int ri;
    wi = wr_index(a);
    ri = ro_index(a);
    if (wi >= 0) return ref_wr[wi];
    if (ri >= 0) return ro_in[ri];
    return 32'h0000_0000;
  endfunction

  task automatic model_write(input logic [7:0] a, input logic [31:0] d);
    int wi;
    logic [31:0] v;
    wi = wr_index(a);
    if (wi < 0) return; // RO or unmapped writes are ignored.

    if (wi == 11) begin
      v = d;
      if ((ctrl_calibrate_bit >= 0) && (ctrl_calibrate_bit < 32) && d[ctrl_calibrate_bit]) begin
        expected_cal_pulses++;
        if ((ctrl_enable_bit >= 0) && (ctrl_enable_bit < 32)) v[ctrl_enable_bit] = 1'b0;
      end
      if ((ctrl_clear_fault_bit >= 0) && (ctrl_clear_fault_bit < 32) && d[ctrl_clear_fault_bit])
        expected_clear_pulses++;
      if ((ctrl_calibrate_bit >= 0) && (ctrl_calibrate_bit < 32))
        v[ctrl_calibrate_bit] = 1'b0;
      if ((ctrl_clear_fault_bit >= 0) && (ctrl_clear_fault_bit < 32))
        v[ctrl_clear_fault_bit] = 1'b0;
      ref_wr[wi] = v;
    end else begin
      ref_wr[wi] = d;
    end
  endtask

  task automatic clear_scoreboard_queues;
    exp_req_q.delete();
    exp_rf_rsp_q.delete();
    exp_tx_rsp_q.delete();
    exp_rx_byte_q.delete();
    exp_ser_byte_q.delete();
  endtask

  task automatic reset_reference_model;
    for (int i=0; i<N_WR; i++) ref_wr[i] = 32'h0;
  endtask

  task automatic pass_check(input bit condition, input string msg);
    checks++;
    if (!condition) begin
      errors++;
      $error("[TB][CHECK FAIL][%0t] %s", $time, msg);
    end
  endtask

  task automatic warn_msg(input string msg);
    warnings++;
    $display("[TB][WARNING][%0t] %s", $time, msg);
  endtask

  task automatic test_begin(input string name);
    tests_run++;
    errors_at_test_start = errors;
    $display("\n[TB][TEST %0d START] %s", tests_run, name);
  endtask

  task automatic test_end(input string name);
    if (errors == errors_at_test_start) begin
      tests_passed++;
      $display("[TB][TEST PASS] %s", name);
    end else begin
      tests_failed++;
      $display("[TB][TEST FAIL] %s  (+%0d errors)", name, errors-errors_at_test_start);
    end
  endtask

  // --------------------------------------------------------------------------
  // CRC-8: polynomial 0x07, init 0x00, MSB first over 56 bits
  // Matches uart_pkg.calc_crc8_56bit().
  // --------------------------------------------------------------------------
  function automatic logic [7:0] crc8_56(input logic [55:0] din);
    logic [7:0] crc;
    logic inv;
    crc = 8'h00;
    for (int i=55; i>=0; i--) begin
      inv = crc[7] ^ din[i];
      crc = {crc[6:0], 1'b0};
      if (inv) crc ^= 8'h07;
    end
    return crc;
  endfunction

  function automatic logic [7:0] request_crc(
    input logic [7:0] cmd,
    input logic [7:0] addr,
    input logic [31:0] data
  );
    return crc8_56({SYNC_BYTE, cmd, addr, data});
  endfunction

  function automatic logic [7:0] response_crc(
    input logic [7:0] addr,
    input logic [31:0] data
  );
    return crc8_56({SYNC_BYTE, 8'h00, addr, data});
  endfunction

  task automatic push_serializer_frame(input logic [7:0] a, input logic [31:0] d);
    exp_ser_byte_q.push_back(SYNC_BYTE);
    exp_ser_byte_q.push_back(8'h00);
    exp_ser_byte_q.push_back(a);
    exp_ser_byte_q.push_back(d[31:24]);
    exp_ser_byte_q.push_back(d[23:16]);
    exp_ser_byte_q.push_back(d[15:8]);
    exp_ser_byte_q.push_back(d[7:0]);
    exp_ser_byte_q.push_back(response_crc(a,d));
  endtask

  // --------------------------------------------------------------------------
  // UART RX pin driver
  // --------------------------------------------------------------------------
  task automatic drive_for(input logic level, input realtime duration);
    uart_rx_pin = level;
    #(duration);
  endtask

  task automatic uart_send_byte_core(
    input logic [7:0] b,
    input realtime baud_scale,
    input bit bad_stop,
    input bit expect_rx,
    input realtime post_guard_bits
  );
    realtime bt;
    bt = realtime'(UART_BIT) * baud_scale;
    if (expect_rx) exp_rx_byte_q.push_back(b);

    drive_for(1'b0, bt); // start
    for (int i=0; i<8; i++) drive_for(b[i], bt); // LSB first
    drive_for(bad_stop ? 1'b0 : 1'b1, bt);
    uart_rx_pin = 1'b1;
    if (post_guard_bits > 0.0)
      drive_for(1'b1, bt * post_guard_bits);
  endtask

  // Ordinary regression byte: valid 8N1 plus a deliberate idle guard.
  task automatic uart_send_byte(
    input logic [7:0] b,
    input realtime baud_scale = 1.0
  );
    uart_send_byte_core(b, baud_scale, 1'b0, 1'b1, NORMAL_RX_GUARD_BITS);
  endtask

  // Standards-throughput byte: exactly 1 start + 8 data + 1 stop, no extra idle.
  // Used only by the isolated contiguous-byte diagnostic.
  task automatic uart_send_byte_no_guard(
    input logic [7:0] b,
    input bit expect_rx = 1'b0
  );
    uart_send_byte_core(b, 1.0, 1'b0, expect_rx, 0.0);
  endtask

  // Inject a very short disturbance around the center of one data bit. It is
  // intentionally shorter than one 16x oversample interval so the 3-sample
  // majority detector should reject it unless phase alignment is pathological.
  task automatic uart_send_byte_one_sample_glitch(
    input logic [7:0] b,
    input int glitch_data_bit
  );
    realtime bt;
    bt = realtime'(UART_BIT);
    exp_rx_byte_q.push_back(b);

    drive_for(1'b0, bt);
    for (int i=0; i<8; i++) begin
      if (i != glitch_data_bit) begin
        drive_for(b[i], bt);
      end else begin
        drive_for(b[i], bt*0.48);
        drive_for(~b[i], bt*0.04);
        drive_for(b[i], bt*0.48);
      end
    end
    drive_for(1'b1, bt);
    uart_rx_pin = 1'b1;
    drive_for(1'b1, bt * NORMAL_RX_GUARD_BITS);
  endtask

  task automatic uart_false_start;
    realtime bt;
    bt = realtime'(UART_BIT);
    drive_for(1'b0, bt*0.25);
    drive_for(1'b1, bt*2.0);
  endtask

  task automatic uart_bad_stop_byte(input logic [7:0] b);
    uart_send_byte_core(b, 1.0, 1'b1, 1'b0, 0.0);
    drive_for(1'b1, realtime'(UART_BIT)*2.0);
  endtask

  task automatic queue_expected_request(
    input logic [7:0] cmd,
    input logic [7:0] a,
    input logic [31:0] d
  );
    req_t e;
    e.rw   = (cmd == WRITE_CMD);
    e.addr = a;
    e.data = d;
    exp_req_q.push_back(e);
  endtask

  task automatic uart_send_request(
    input logic [7:0] cmd,
    input logic [7:0] a,
    input logic [31:0] d,
    input bit expect_accept = 1'b1,
    input logic [7:0] crc_xor = 8'h00,
    input int interbyte_gap_bits = 0,
    input realtime baud_scale = 1.0
  );
    logic [7:0] crc;
    crc = request_crc(cmd,a,d) ^ crc_xor;
    if (expect_accept) queue_expected_request(cmd,a,d);

    uart_send_byte(SYNC_BYTE, baud_scale);
    if (interbyte_gap_bits) drive_for(1'b1, realtime'(UART_BIT)*baud_scale*interbyte_gap_bits);
    uart_send_byte(cmd, baud_scale);
    if (interbyte_gap_bits) drive_for(1'b1, realtime'(UART_BIT)*baud_scale*interbyte_gap_bits);
    uart_send_byte(a, baud_scale);
    if (interbyte_gap_bits) drive_for(1'b1, realtime'(UART_BIT)*baud_scale*interbyte_gap_bits);
    uart_send_byte(d[31:24], baud_scale);
    if (interbyte_gap_bits) drive_for(1'b1, realtime'(UART_BIT)*baud_scale*interbyte_gap_bits);
    uart_send_byte(d[23:16], baud_scale);
    if (interbyte_gap_bits) drive_for(1'b1, realtime'(UART_BIT)*baud_scale*interbyte_gap_bits);
    uart_send_byte(d[15:8], baud_scale);
    if (interbyte_gap_bits) drive_for(1'b1, realtime'(UART_BIT)*baud_scale*interbyte_gap_bits);
    uart_send_byte(d[7:0], baud_scale);
    if (interbyte_gap_bits) drive_for(1'b1, realtime'(UART_BIT)*baud_scale*interbyte_gap_bits);
    uart_send_byte(crc, baud_scale);
  endtask

  task automatic uart_send_request_forced_crc(
    input logic [7:0] cmd,
    input logic [7:0] a,
    input logic [31:0] d,
    input logic [7:0] forced_crc,
    input bit expect_accept = 1'b0
  );
    if (expect_accept) queue_expected_request(cmd,a,d);
    uart_send_byte(SYNC_BYTE);
    uart_send_byte(cmd);
    uart_send_byte(a);
    uart_send_byte(d[31:24]);
    uart_send_byte(d[23:16]);
    uart_send_byte(d[15:8]);
    uart_send_byte(d[7:0]);
    uart_send_byte(forced_crc);
  endtask


  // Raw request with no extra idle between bytes. No scoreboard expectations are
  // queued here; callers must isolate/suppress boundary scoreboards explicitly.
  task automatic uart_send_request_no_guard_raw(
    input logic [7:0] cmd,
    input logic [7:0] a,
    input logic [31:0] d
  );
    logic [7:0] crc;
    crc = request_crc(cmd,a,d);
    uart_send_byte_no_guard(SYNC_BYTE);
    uart_send_byte_no_guard(cmd);
    uart_send_byte_no_guard(a);
    uart_send_byte_no_guard(d[31:24]);
    uart_send_byte_no_guard(d[23:16]);
    uart_send_byte_no_guard(d[15:8]);
    uart_send_byte_no_guard(d[7:0]);
    uart_send_byte_no_guard(crc);
  endtask

  task automatic uart_send_request_with_glitch(
    input logic [7:0] cmd,
    input logic [7:0] a,
    input logic [31:0] d
  );
    logic [7:0] crc;
    crc = request_crc(cmd,a,d);
    queue_expected_request(cmd,a,d);
    uart_send_byte(SYNC_BYTE);
    uart_send_byte(cmd);
    uart_send_byte(a);
    uart_send_byte(d[31:24]);
    uart_send_byte_one_sample_glitch(d[23:16], 3);
    uart_send_byte(d[15:8]);
    uart_send_byte(d[7:0]);
    uart_send_byte(crc);
  endtask

  // --------------------------------------------------------------------------
  // UART TX pin decoder
  // --------------------------------------------------------------------------
  task automatic capture_tx_uart_byte(output logic [7:0] b, output bit ok);
    realtime bt;
    bt = realtime'(UART_BIT);
    b = '0;
    ok = 1'b0;

    @(negedge uart_tx_pin);
    if (rst) return;

    #(bt*0.50);
    if (rst) return;
    if (uart_tx_pin !== 1'b0) begin
      pass_check(1'b0, "TX start bit was not low at center sample");
      return;
    end

    for (int i=0; i<8; i++) begin
      #(bt);
      if (rst) return;
      b[i] = uart_tx_pin;
    end

    #(bt);
    if (rst) return;
    pass_check(uart_tx_pin === 1'b1, "TX stop bit was not high at center sample");
    ok = 1'b1;
  endtask

  // --------------------------------------------------------------------------
  // Boundary monitors / scoreboards
  // --------------------------------------------------------------------------

  // uart_rx output monitor: verifies every physically valid input byte.
  always @(posedge clk) begin
    #1ps;
    if (!rst && rx_vld) begin
      logic [7:0] e;
      rx_bytes_seen++;
      if (suppress_rx_scoreboard) begin
        // Count only. The isolated raw-throughput diagnostic intentionally does
        // not participate in the normal ordered RX expectation queue.
      end else if (exp_rx_byte_q.size() == 0) begin
        pass_check(1'b0, $sformatf("uart_rx emitted unexpected byte 0x%02h", rx_data));
      end else begin
        e = exp_rx_byte_q.pop_front();
        pass_check(rx_data === e,
          $sformatf("uart_rx byte mismatch exp=0x%02h got=0x%02h", e, rx_data));
      end
    end
  end

  // Parser request monitor + reference-model update + expected response queues.
  always @(posedge clk) begin
    #1ps;
    if (!rst && req_vld) begin
      req_t e;
      rsp_t r;
      int wi;
      int ri;
      parser_reqs_seen++;

      if (suppress_req_scoreboard) begin
        // Count only. Diagnostic traffic is reset away before normal checking
        // resumes, so it must not update the architectural reference model.
      end else if (exp_req_q.size() == 0) begin
        pass_check(1'b0,
          $sformatf("parser emitted unexpected request rw=%0b addr=0x%02h data=0x%08h",
                    req_rw, req_addr, req_data));
      end else begin
        e = exp_req_q.pop_front();
        pass_check(req_rw   === e.rw,   $sformatf("parser rw mismatch exp=%0b got=%0b", e.rw, req_rw));
        pass_check(req_addr === e.addr, $sformatf("parser addr mismatch exp=0x%02h got=0x%02h", e.addr, req_addr));
        pass_check(req_data === e.data, $sformatf("parser data mismatch exp=0x%08h got=0x%08h", e.data, req_data));

        wi = wr_index(req_addr);
        ri = ro_index(req_addr);
        if (req_rw) begin
          if (wi >= 0) wr_write_hits[wi]++;
          else if (ri >= 0) ro_write_hits[ri]++;
          model_write(req_addr, req_data);
        end else begin
          if (wi >= 0) wr_read_hits[wi]++;
          else if (ri >= 0) ro_read_hits[ri]++;
          r.addr = req_addr;
          r.data = model_read(req_addr);
          exp_rf_rsp_q.push_back(r);
          exp_tx_rsp_q.push_back(r);
        end
      end
    end
  end

  // Register-file read response monitor.
  always @(posedge clk) begin
    #1ps;
    if (!rst && rf_rsp_vld) begin
      rsp_t e;
      rf_rsps_seen++;
      if (exp_rf_rsp_q.size() == 0) begin
        pass_check(1'b0,
          $sformatf("register_file emitted unexpected response addr=0x%02h data=0x%08h",
                    rf_rsp_addr, rf_rsp_data));
      end else begin
        e = exp_rf_rsp_q.pop_front();
        pass_check(rf_rsp_addr === e.addr,
          $sformatf("regfile rsp addr mismatch exp=0x%02h got=0x%02h", e.addr, rf_rsp_addr));
        pass_check(rf_rsp_data === e.data,
          $sformatf("regfile rsp data mismatch addr=0x%02h exp=0x%08h got=0x%08h",
                    e.addr, e.data, rf_rsp_data));
        push_serializer_frame(e.addr, e.data);
      end
    end
  end

  // Serializer byte-level monitor. This pinpoints framing/endian/CRC errors before
  // they are obscured by uart_tx timing.
  always @(posedge clk) begin
    #1ps;
    if (!rst) begin
      if ((exp_ser_byte_q.size() != 0) && !tx_rdy) backpressure_cycles++;
      if (ser_vld) begin
        logic [7:0] e;
        serializer_bytes_seen++;
        if (exp_ser_byte_q.size() == 0) begin
          pass_check(1'b0, $sformatf("serializer emitted unexpected byte 0x%02h", ser_byte));
        end else begin
          e = exp_ser_byte_q.pop_front();
          pass_check(ser_byte === e,
            $sformatf("serializer byte mismatch exp=0x%02h got=0x%02h", e, ser_byte));
        end
      end
    end
  end

  // UART TX pin monitor, groups every 8 UART bytes into one response frame.
  initial begin : tx_wire_monitor
    logic [7:0] b;
    logic [7:0] frame [0:7];
    int n;
    bit ok;
    rsp_t e;
    n = 0;
    wait (rst == 1'b0);
    forever begin
      capture_tx_uart_byte(b, ok);
      if (!ok) begin
        if (rst) n = 0;
      end else begin
        tx_bytes_seen++;
        frame[n] = b;
        n++;
        if (n == 8) begin
          logic [31:0] got_data;
          logic [7:0] got_crc;
          tx_frames_seen++;
          n = 0;
          got_data = {frame[3],frame[4],frame[5],frame[6]};
          got_crc  = crc8_56({frame[0],frame[1],frame[2],got_data});

          if (exp_tx_rsp_q.size() == 0) begin
            pass_check(1'b0,
              $sformatf("unexpected TX frame %02h %02h %02h %02h %02h %02h %02h %02h",
                        frame[0],frame[1],frame[2],frame[3],frame[4],frame[5],frame[6],frame[7]));
          end else begin
            e = exp_tx_rsp_q.pop_front();
            pass_check(frame[0] === SYNC_BYTE,
              $sformatf("TX sync mismatch exp=0x%02h got=0x%02h", SYNC_BYTE, frame[0]));
            pass_check(frame[1] === 8'h00,
              $sformatf("TX response command mismatch exp=0x00 got=0x%02h", frame[1]));
            pass_check(frame[2] === e.addr,
              $sformatf("TX response addr mismatch exp=0x%02h got=0x%02h", e.addr, frame[2]));
            pass_check(got_data === e.data,
              $sformatf("TX response data mismatch addr=0x%02h exp=0x%08h got=0x%08h",
                        e.addr, e.data, got_data));
            pass_check(frame[7] === got_crc,
              $sformatf("TX response CRC mismatch calculated=0x%02h got=0x%02h", got_crc, frame[7]));
          end
        end
      end
    end
  end

  // Pulse and one-cycle-valid sanity checks.
  always @(posedge clk) begin
    #1ps;
    if (rst) begin
      prev_req_vld     <= 1'b0;
      prev_rsp_vld     <= 1'b0;
      prev_cal_pulse   <= 1'b0;
      prev_clear_pulse <= 1'b0;
    end else begin
      if (prev_req_vld && req_vld)
        pass_check(1'b0, "parser o_vld stayed high for more than one clock");
      if (prev_rsp_vld && rf_rsp_vld)
        pass_check(1'b0, "register_file o_rsp_vld stayed high for more than one clock");
      if (prev_cal_pulse && calibrate_pulse)
        pass_check(1'b0, "calibrate pulse stayed high for more than one clock");
      if (prev_clear_pulse && clear_fault_pulse)
        pass_check(1'b0, "clear-fault pulse stayed high for more than one clock");

      if (calibrate_pulse) begin actual_cal_pulses++; end
      if (clear_fault_pulse) begin actual_clear_pulses++; end

      prev_req_vld     <= req_vld;
      prev_rsp_vld     <= rf_rsp_vld;
      prev_cal_pulse   <= calibrate_pulse;
      prev_clear_pulse <= clear_fault_pulse;

      pass_check(!$isunknown({rx_vld,req_vld,rf_rsp_vld,ser_vld,tx_rdy,uart_tx_pin}),
                 "X/Z detected on always-valid critical control path after reset");
      if (req_vld)
        pass_check(!$isunknown({req_rw,req_addr,req_data}),
                   "X/Z detected on parser request payload while req_vld=1");
      if (rf_rsp_vld)
        pass_check(!$isunknown({rf_rsp_addr,rf_rsp_data}),
                   "X/Z detected on register-file response while rsp_vld=1");
      if (ser_vld)
        pass_check(!$isunknown(ser_byte),
                   "X/Z detected on serializer byte while ser_vld=1");
    end
  end

  // --------------------------------------------------------------------------
  // Wait/check helpers
  // --------------------------------------------------------------------------
  task automatic wait_for_parser_req_after(input int baseline, input string what);
    time deadline;
    deadline = $time + 25*UART_BIT;
    while ((parser_reqs_seen <= baseline) && ($time < deadline)) @(posedge clk);
    pass_check(parser_reqs_seen > baseline, {"timeout waiting for parser request: ", what});
  endtask

  task automatic wait_for_tx_frame_after(input int baseline, input string what);
    time deadline;
    deadline = $time + 120*UART_BIT;
    while ((tx_frames_seen <= baseline) && ($time < deadline)) @(posedge clk);
    pass_check(tx_frames_seen > baseline, {"timeout waiting for TX response frame: ", what});
  endtask

  task automatic expect_no_parser_request(input int baseline, input time quiet_time, input string what);
    #(quiet_time);
    pass_check(parser_reqs_seen == baseline, {"unexpected parser request after ", what});
  endtask

  task automatic wait_scoreboards_empty(input time timeout, input string what);
    time deadline;
    deadline = $time + timeout;
    while (($time < deadline) &&
           ((exp_req_q.size()!=0) || (exp_rf_rsp_q.size()!=0) ||
            (exp_tx_rsp_q.size()!=0) || (exp_rx_byte_q.size()!=0) ||
            (exp_ser_byte_q.size()!=0))) begin
      @(posedge clk);
    end
    pass_check(exp_req_q.size()==0,      {what, ": expected parser-request queue not empty"});
    pass_check(exp_rf_rsp_q.size()==0,   {what, ": expected regfile-response queue not empty"});
    pass_check(exp_tx_rsp_q.size()==0,   {what, ": expected TX-response queue not empty"});
    pass_check(exp_rx_byte_q.size()==0,  {what, ": expected RX-byte queue not empty"});
    pass_check(exp_ser_byte_q.size()==0, {what, ": expected serializer-byte queue not empty"});
  endtask

  task automatic send_write_and_wait(
    input logic [7:0] a,
    input logic [31:0] d,
    input realtime baud_scale = 1.0,
    input int gap_bits = 0
  );
    int before_req;
    before_req = parser_reqs_seen;
    uart_send_request(WRITE_CMD,a,d,1'b1,8'h00,gap_bits,baud_scale);
    wait_for_parser_req_after(before_req, $sformatf("write addr=0x%02h",a));
    repeat (4) @(posedge clk);
  endtask

  task automatic send_read_and_wait(
    input logic [7:0] a,
    input logic [31:0] payload = 32'h0000_0000,
    input realtime baud_scale = 1.0,
    input int gap_bits = 0
  );
    int before_req;
    int before_tx;
    before_req = parser_reqs_seen;
    before_tx  = tx_frames_seen;
    uart_send_request(READ_CMD,a,payload,1'b1,8'h00,gap_bits,baud_scale);
    wait_for_parser_req_after(before_req, $sformatf("read addr=0x%02h",a));
    wait_for_tx_frame_after(before_tx, $sformatf("read addr=0x%02h",a));
  endtask

  task automatic apply_reset(input string reason);
    $display("[TB] Applying reset: %s", reason);
    rst = 1'b1;
    uart_rx_pin = 1'b1;
    clear_scoreboard_queues();
    reset_reference_model();
    repeat (6) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;
    repeat (6) @(posedge clk);
  endtask

  // Keep one root-cause failure from shifting every ordered queue for the rest of
  // the regression. Call after a failed named test. Hit counters and error counts
  // are intentionally preserved; DUT/reference state and pending queues are reset.
  task automatic recover_after_failed_test(input string test_name);
    if (errors != errors_at_test_start) begin
      $display("[TB][RECOVERY] isolating failure from %s before next test", test_name);
      apply_reset({"automatic recovery after ", test_name});
    end
  endtask

  task automatic check_hw_registers(input string ctx);
    for (int i=0; i<N_WR; i++) begin
      pass_check(hw_wr[i] === ref_wr[i],
        $sformatf("%s: %s mismatch exp=0x%08h got=0x%08h",
                  ctx, wr_name[i], ref_wr[i], hw_wr[i]));
    end
  endtask

  // --------------------------------------------------------------------------
  // Directed tests
  // --------------------------------------------------------------------------
  task automatic test_reset_sanity;
    string n = "reset_sanity";
    test_begin(n);
    check_hw_registers("after reset");
    pass_check(uart_tx_pin === 1'b1, "uart_tx must idle high after reset");
    pass_check(clear_fault_pulse === 1'b0, "clear-fault pulse must be low after reset");
    pass_check(calibrate_pulse === 1'b0, "calibrate pulse must be low after reset");
    pass_check(rf_rsp_vld === 1'b0, "regfile response valid must be low after reset");
    test_end(n);
  endtask

  task automatic test_all_writable_registers;
    string n = "all_writable_registers_write_and_readback";
    logic [31:0] d;
    test_begin(n);
    for (int i=0; i<N_WR; i++) begin
      if (i == 11) continue; // CONTROL has special semantics; tested separately.
      d = 32'hA500_0000 ^ (i * 32'h0101_1021) ^ {24'h0,wr_addr[i]};
      send_write_and_wait(wr_addr[i], d);
      pass_check(hw_wr[i] === ref_wr[i],
        $sformatf("write did not update %s addr=0x%02h exp=0x%08h got=0x%08h",
                  wr_name[i], wr_addr[i], ref_wr[i], hw_wr[i]));
      send_read_and_wait(wr_addr[i], 32'hDEAD_BEEF); // read payload should be ignored by regfile
    end
    test_end(n);
  endtask

  task automatic test_data_patterns_endianness;
    string n = "data_patterns_and_endianness";
    logic [31:0] patterns [0:7];
    test_begin(n);
    patterns[0] = 32'h0000_0000;
    patterns[1] = 32'hFFFF_FFFF;
    patterns[2] = 32'hAAAA_AAAA;
    patterns[3] = 32'h5555_5555;
    patterns[4] = 32'h0123_4567;
    patterns[5] = 32'h89AB_CDEF;
    patterns[6] = 32'hAA01_02FF; // embeds sync/read/write-like values in payload
    patterns[7] = 32'h8000_0001;
    for (int p=0; p<8; p++) begin
      send_write_and_wait(wr_addr[p % 11], patterns[p]);
      send_read_and_wait(wr_addr[p % 11], ~patterns[p]);
    end
    test_end(n);
  endtask

  task automatic test_read_only_registers;
    string n = "read_only_telemetry_and_write_ignore";
    test_begin(n);
    for (int i=0; i<N_RO; i++) begin
      send_read_and_wait(ro_addr[i]);
      send_write_and_wait(ro_addr[i], 32'hDEAD_0000 | i);
      send_read_and_wait(ro_addr[i], 32'hFFFF_FFFF);
      pass_check(ro_in[i] !== (32'hDEAD_0000 | i),
        $sformatf("testbench telemetry sentinel accidentally equals attempted RO write for %s",ro_name[i]));
    end
    test_end(n);
  endtask

  task automatic test_unmapped_in_range_address;
    string n = "unmapped_in_range_address_zero_read_ignore_write";
    bit found;
    byte unsigned a;
    test_begin(n);
    found = 1'b0;
    a = 8'h00;
    for (int unsigned x=0; x<=last_addr; x++) begin
      if (!found && (wr_index(x[7:0]) < 0) && (ro_index(x[7:0]) < 0)) begin
        found = 1'b1;
        a = x[7:0];
      end
    end
    if (found) begin
      send_write_and_wait(a, 32'hD15C_A11E);
      send_read_and_wait(a, 32'h1234_5678);
      // model_read() returns zero for unmapped addresses; TX/regfile scoreboards
      // therefore verify both ignored write and zero-valued read behavior.
    end else begin
      warn_msg("no unmapped address exists at or below LAST_ADDR; in-range-hole test skipped");
    end
    test_end(n);
  endtask

  task automatic test_control_semantics;
    string n = "control_self_clear_and_command_pulses";
    logic [31:0] d;
    int cal_before;
    int clr_before;
    test_begin(n);

    // Enable only.
    d = 32'h0;
    d[ctrl_enable_bit] = 1'b1;
    send_write_and_wait(wr_addr[11], d);
    pass_check(hw_wr[11][ctrl_enable_bit] === 1'b1,
               "CONTROL.ENABLE did not latch high");

    // All non-command CONTROL fields must latch without being mistaken for pulses.
    // This is particularly important because MODE occupies bits [2:1], immediately
    // below the actual CALIBRATE/CLEAR_FAULT command bits [3]/[4].
    d = 32'hA5C3_0000;
    d[ctrl_enable_bit]      = 1'b1;
    d[2:1] = 2'b10;
    d[ctrl_manual_elec_bit] = 1'b1;
    d[ctrl_sensor_inv_bit]  = 1'b1;
    d[ctrl_watchdog_bit]    = 1'b1;
    d[ctrl_calibrate_bit]   = 1'b0;
    d[ctrl_clear_fault_bit] = 1'b0;
    cal_before = actual_cal_pulses;
    clr_before = actual_clear_pulses;
    send_write_and_wait(wr_addr[11], d);
    pass_check(hw_wr[11] === d,
               $sformatf("CONTROL ordinary fields not preserved exp=0x%08h got=0x%08h", d, hw_wr[11]));
    pass_check(actual_cal_pulses == cal_before,
               "CONTROL MODE/config write incorrectly generated CALIBRATE pulse");
    pass_check(actual_clear_pulses == clr_before,
               "CONTROL MODE/config write incorrectly generated CLEAR_FAULT pulse");
    send_read_and_wait(wr_addr[11], 32'h0000_0000);

    // CALIBRATE must pulse, self-clear, and force ENABLE low.
    cal_before = actual_cal_pulses;
    d = 32'h5A3C_0000;
    d[ctrl_enable_bit] = 1'b1;
    d[2:1] = 2'b01;
    d[ctrl_manual_elec_bit] = 1'b1;
    d[ctrl_sensor_inv_bit]  = 1'b0;
    d[ctrl_watchdog_bit]    = 1'b1;
    d[ctrl_calibrate_bit] = 1'b1;
    d[ctrl_clear_fault_bit] = 1'b0;
    send_write_and_wait(wr_addr[11], d);
    repeat (3) @(posedge clk);
    pass_check(actual_cal_pulses == cal_before+1, "CALIBRATE did not generate exactly one pulse");
    pass_check(hw_wr[11][ctrl_calibrate_bit] === 1'b0, "CONTROL.CALIBRATE did not self-clear");
    pass_check(hw_wr[11][ctrl_enable_bit] === 1'b0, "CALIBRATE did not force CONTROL.ENABLE low");
    pass_check(hw_wr[11][2:1] === 2'b01, "CALIBRATE corrupted CONTROL.MODE");
    pass_check(hw_wr[11][ctrl_manual_elec_bit] === 1'b1, "CALIBRATE corrupted CONTROL.MANUAL_ELEC");
    pass_check(hw_wr[11][ctrl_sensor_inv_bit]  === 1'b0, "CALIBRATE corrupted CONTROL.SENSOR_INV");
    pass_check(hw_wr[11][ctrl_watchdog_bit]    === 1'b1, "CALIBRATE corrupted CONTROL.WATCHDOG");

    // CLEAR_FAULT must pulse and self-clear.
    clr_before = actual_clear_pulses;
    d = 32'h0;
    d[ctrl_clear_fault_bit] = 1'b1;
    send_write_and_wait(wr_addr[11], d);
    repeat (3) @(posedge clk);
    pass_check(actual_clear_pulses == clr_before+1, "CLEAR_FAULT did not generate exactly one pulse");
    pass_check(hw_wr[11][ctrl_clear_fault_bit] === 1'b0, "CONTROL.CLEAR_FAULT did not self-clear");

    // Both command bits together.
    cal_before = actual_cal_pulses;
    clr_before = actual_clear_pulses;
    d = 32'h0;
    d[ctrl_calibrate_bit] = 1'b1;
    d[ctrl_clear_fault_bit] = 1'b1;
    send_write_and_wait(wr_addr[11], d);
    repeat (3) @(posedge clk);
    pass_check(actual_cal_pulses == cal_before+1, "combined CONTROL write missed CALIBRATE pulse");
    pass_check(actual_clear_pulses == clr_before+1, "combined CONTROL write missed CLEAR_FAULT pulse");
    send_read_and_wait(wr_addr[11]);
    test_end(n);
  endtask

  task automatic test_parser_rejection_and_resync;
    string n = "parser_rejection_bad_sync_cmd_addr_crc_and_resync";
    int before_req;
    logic [7:0] bad_addr;
    test_begin(n);

    // Wrong sync byte: must not leave IDLE.
    before_req = parser_reqs_seen;
    uart_send_byte(8'h55);
    expect_no_parser_request(before_req, 4*UART_BIT, "wrong sync byte");

    // Valid sync followed by illegal command: parser must return to IDLE.
    before_req = parser_reqs_seen;
    uart_send_byte(SYNC_BYTE);
    uart_send_byte(8'h7E);
    expect_no_parser_request(before_req, 4*UART_BIT, "illegal command");

    // The inclusive upper boundary itself must still be accepted.
    send_read_and_wait(last_addr, 32'h0BAD_F00D);

    // Address above C_LAST_ADDR should be rejected, if a value exists.
    if (last_addr != 8'hFF) begin
      bad_addr = last_addr + 1'b1;
      before_req = parser_reqs_seen;
      uart_send_byte(SYNC_BYTE);
      uart_send_byte(WRITE_CMD);
      uart_send_byte(bad_addr);
      expect_no_parser_request(before_req, 4*UART_BIT, "address above LAST_ADDR");
    end else begin
      warn_msg("LAST_ADDR is 0xFF, so there is no 8-bit out-of-range address to test.");
    end

    // Corrupt CRC; full UART frame is valid but parser must not execute it.
    before_req = parser_reqs_seen;
    uart_send_request(WRITE_CMD, wr_addr[0], 32'h1357_9BDF, 1'b0, 8'h01);
    expect_no_parser_request(before_req, 8*UART_BIT, "CRC xor 0x01");

    before_req = parser_reqs_seen;
    uart_send_request(WRITE_CMD, wr_addr[0], 32'h2468_ACE0, 1'b0, 8'h80);
    expect_no_parser_request(before_req, 8*UART_BIT, "CRC xor 0x80");

    // Immediate good frame after bad CRC proves parser resynchronization.
    send_write_and_wait(wr_addr[0], 32'hC001_C0DE);
    send_read_and_wait(wr_addr[0]);
    test_end(n);
  endtask

  task automatic test_crc_detects_data_bit_errors;
    string n = "crc_detects_single_bit_data_errors";
    logic [31:0] original;
    logic [31:0] corrupted;
    logic [7:0] stale_crc;
    int bit_list [0:7];
    int before_req;
    test_begin(n);
    original = 32'h6D3A_C5F0;
    stale_crc = request_crc(WRITE_CMD, wr_addr[9], original);
    bit_list[0]=0; bit_list[1]=1; bit_list[2]=7; bit_list[3]=8;
    bit_list[4]=15; bit_list[5]=16; bit_list[6]=23; bit_list[7]=31;
    for (int k=0; k<8; k++) begin
      corrupted = original ^ (32'h1 << bit_list[k]);
      before_req = parser_reqs_seen;
      uart_send_request_forced_crc(WRITE_CMD, wr_addr[9], corrupted, stale_crc, 1'b0);
      expect_no_parser_request(before_req, 8*UART_BIT,
        $sformatf("single-bit data corruption at bit %0d with stale CRC", bit_list[k]));
    end
    test_end(n);
  endtask

  task automatic test_interbyte_stalls;
    string n = "parser_interbyte_stalls";
    test_begin(n);
    // Parser has no inter-byte timeout, so long legal gaps must preserve state.
    send_write_and_wait(wr_addr[1], 32'h1122_3344, 1.0, 5);
    send_read_and_wait(wr_addr[1], 32'h5566_7788, 1.0, 3);
    test_end(n);
  endtask

  task automatic test_uart_rx_weak_areas;
    string n = "uart_rx_false_start_bad_stop_glitch_and_baud_tolerance";
    int rx_before;
    int req_before;
    test_begin(n);

    // False-start rejection.
    rx_before = rx_bytes_seen;
    uart_false_start();
    #(2*UART_BIT);
    pass_check(rx_bytes_seen == rx_before, "uart_rx accepted a short false-start pulse");

    // Bad stop bit rejection.
    rx_before = rx_bytes_seen;
    uart_bad_stop_byte(8'hA5);
    #(2*UART_BIT);
    pass_check(rx_bytes_seen == rx_before, "uart_rx accepted a byte with a low stop bit");

    // Majority-vote glitch tolerance in one payload byte.
    req_before = parser_reqs_seen;
    uart_send_request_with_glitch(WRITE_CMD, wr_addr[2], 32'h5A3C_C3A5);
    wait_for_parser_req_after(req_before, "glitch-tolerant write");
    repeat (4) @(posedge clk);
    pass_check(hw_wr[2] === ref_wr[2], "glitch-tolerant UART frame did not write expected data");

    // Moderate baud mismatch in both directions.
    send_write_and_wait(wr_addr[3], 32'hCAFE_0200, 1.02, 0);
    send_read_and_wait (wr_addr[3], 32'h0,       0.98, 0);
    test_end(n);
  endtask

  task automatic test_serializer_latches_read_response;
    string n = "serializer_latches_response_against_live_telemetry_change";
    int before_req;
    int before_rsp;
    int before_tx;
    logic [31:0] old_value;
    logic [31:0] new_value;
    test_begin(n);
    old_value = ro_in[0];
    new_value = old_value ^ 32'h00FF_FF00;
    before_req = parser_reqs_seen;
    before_rsp = rf_rsps_seen;
    before_tx  = tx_frames_seen;

    uart_send_request(READ_CMD, ro_addr[0], 32'hCAF0_0001);
    wait_for_parser_req_after(before_req, "serializer latch test read");
    begin
      time deadline = $time + 10*UART_BIT;
      while ((rf_rsps_seen <= before_rsp) && ($time < deadline)) @(posedge clk);
    end
    pass_check(rf_rsps_seen > before_rsp, "timeout waiting for regfile response in serializer latch test");

    // Change the live telemetry after rf_rsp_vld has delivered the old value.
    // The serializer/TX response must remain the already-captured old_value.
    ro_in[0] = new_value;
    wait_for_tx_frame_after(before_tx, "serializer latch test TX frame");
    ro_in[0] = old_value;
    test_end(n);
  endtask

  task automatic test_guarded_back_to_back_writes;
    string n = "back_to_back_frames_with_regression_guard";
    int req_count_before;
    test_begin(n);
    req_count_before = parser_reqs_seen;
    uart_send_request(WRITE_CMD, wr_addr[4], 32'h1111_2222);
    uart_send_request(WRITE_CMD, wr_addr[5], 32'h3333_4444);
    uart_send_request(WRITE_CMD, wr_addr[6], 32'h5555_6666);
    begin
      time deadline = $time + 40*UART_BIT;
      while ((parser_reqs_seen < req_count_before+3) && ($time < deadline)) @(posedge clk);
    end
    pass_check(parser_reqs_seen == req_count_before+3,
               "guarded back-to-back write frames were not all accepted");
    repeat (5) @(posedge clk);
    pass_check(hw_wr[4] === ref_wr[4], "guarded write #1 mismatch");
    pass_check(hw_wr[5] === ref_wr[5], "guarded write #2 mismatch");
    pass_check(hw_wr[6] === ref_wr[6], "guarded write #3 mismatch");
    test_end(n);
  endtask

  // This is the standards-compliant throughput test that exposed the failure in
  // the supplied run: a UART is permitted to begin the next start bit immediately
  // after the required stop bit. The current uart_rx waits until the END of ST_STOP,
  // then spends ST_DONE/IDLE/synchronizer latency reacquiring the next start. That
  // latency accumulates across contiguous bytes and can corrupt/drop later bytes.
  task automatic test_uart_rx_contiguous_bytes;
    string n = "uart_rx_contiguous_8n1_no_extra_idle";
    int req_before;
    int rx_before;
    int err_before_diag;
    bit accepted;
    test_begin(n);
    req_before = parser_reqs_seen;
    rx_before  = rx_bytes_seen;
    err_before_diag = errors;

    suppress_rx_scoreboard  = 1'b1;
    suppress_req_scoreboard = 1'b1;
    uart_send_request_no_guard_raw(WRITE_CMD, wr_addr[4], 32'h1357_9BDF);

    begin
      time deadline = $time + 30*UART_BIT;
      while ((parser_reqs_seen <= req_before) && ($time < deadline)) @(posedge clk);
    end
    accepted = (parser_reqs_seen > req_before);

    // Reset while suppression is still active so any partial frame cannot leak
    // into the next test's ordered scoreboards/reference model.
    apply_reset("after contiguous-byte UART throughput diagnostic");
    suppress_rx_scoreboard  = 1'b0;
    suppress_req_scoreboard = 1'b0;

    if (!accepted) begin
      string msg;
      msg = $sformatf("uart_rx did not sustain one contiguous 8-byte 8N1 request at 921600 baud (saw %0d RX bytes before reset). This is a DUT throughput limitation, not an ordered-scoreboard desynchronization.",
                      rx_bytes_seen-rx_before);
      if (warn_rx_contiguous) warn_msg(msg);
      else pass_check(1'b0, msg);
    end
    test_end(n);
  endtask

  task automatic test_burst_reads_diagnostic;
    string n = "burst_reads_while_serializer_tx_busy";
    int before_req;
    int before_tx;
    int err_before_local;
    test_begin(n);
    err_before_local = errors;
    before_req = parser_reqs_seen;
    before_tx  = tx_frames_seen;

    // This targets the architectural weak point: register_file can produce a
    // one-cycle response while serializer only latches i_vld in ST_IDLE.
    // A protocol that permits overlapped reads requires both responses.
    uart_send_request(READ_CMD, ro_addr[0], 32'h0);
    uart_send_request(READ_CMD, ro_addr[1], 32'h0);

    begin
      time deadline = $time + 240*UART_BIT;
      while ((tx_frames_seen < before_tx+2) && ($time < deadline)) @(posedge clk);
    end

    if (parser_reqs_seen != before_req+2) begin
      pass_check(1'b0, "burst-read diagnostic: parser did not accept both requests");
    end

    if (tx_frames_seen != before_tx+2) begin
      string msg;
      msg = $sformatf("burst-read diagnostic: accepted %0d reads but produced only %0d complete responses; serializer has no input queue and only captures rf_rsp_vld in IDLE.",
                      parser_reqs_seen-before_req, tx_frames_seen-before_tx);
      if (strict_burst_reads) pass_check(1'b0, msg);
      else begin
        // Prevent diagnostic leftovers from contaminating subsequent tests.
        warn_msg(msg);
        exp_rf_rsp_q.delete();
        exp_tx_rsp_q.delete();
        exp_ser_byte_q.delete();
      end
    end

    // If strict mode is off, do not let expected known-risk behavior turn the
    // whole named diagnostic into a false FAIL solely because of queue timeouts.
    if (!strict_burst_reads && errors > err_before_local) begin
      warn_msg("burst diagnostic also produced scoreboard errors; inspect timestamps above.");
    end
    test_end(n);
  endtask

  task automatic test_reset_mid_frame_recovery;
    string n = "reset_mid_rx_frame_and_recovery";
    realtime bt;
    test_begin(n);
    bt = realtime'(UART_BIT);

    // Begin a frame, then assert synchronous reset before it can complete.
    // Do not queue RX/parser expectations for this intentionally aborted traffic.
    drive_for(1'b0, bt);
    for (int i=0; i<4; i++) drive_for(SYNC_BYTE[i], bt);
    rst = 1'b1;
    uart_rx_pin = 1'b1;
    clear_scoreboard_queues();
    reset_reference_model();
    repeat (6) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;
    repeat (6) @(posedge clk);

    check_hw_registers("after mid-frame reset");
    send_write_and_wait(wr_addr[7], 32'hABCD_1234);
    send_read_and_wait(wr_addr[7]);
    test_end(n);
  endtask

  task automatic test_truncated_frame_resync_diagnostic;
    string n = "truncated_frame_in_band_resync_diagnostic";
    int before_req;
    int before_tx;
    logic [7:0] crc;
    test_begin(n);
    before_req = parser_reqs_seen;

    // Send sync/cmd/addr and only one data byte, then a long idle. The current
    // parser has no timeout in ST_DATA. We then send a fresh valid frame WITHOUT
    // reset. A robust self-resynchronizing protocol may require the new sync to
    // recover; the current RTL may instead consume it as remaining data.
    uart_send_byte(SYNC_BYTE);
    uart_send_byte(WRITE_CMD);
    uart_send_byte(wr_addr[8]);
    uart_send_byte(8'h12);
    drive_for(1'b1, realtime'(UART_BIT)*20.0);

    // Fresh frame, queued as expected only for the diagnostic.
    before_tx = tx_frames_seen;
    uart_send_request(READ_CMD, ro_addr[0], 32'h0);

    begin
      time deadline = $time + 180*UART_BIT;
      while ((tx_frames_seen <= before_tx) && ($time < deadline)) @(posedge clk);
    end

    if (tx_frames_seen <= before_tx) begin
      string msg = "parser did not recover in-band from a truncated frame; RTL has no ST_DATA timeout/sync escape";
      if (strict_resync) pass_check(1'b0, msg);
      else begin
        warn_msg(msg);
        clear_scoreboard_queues();
        apply_reset("recover after truncated-frame diagnostic");
      end
    end
    test_end(n);
  endtask

  task automatic test_random_regression;
    string n = "randomized_end_to_end_regression";
    int idx;
    logic [31:0] d;
    test_begin(n);
    for (int k=0; k<random_iters; k++) begin
      d = {$urandom(), $urandom()};
      case ($urandom_range(0,3))
        0,1: begin
          idx = $urandom_range(0,N_WR-1);
          if (idx == 11) idx = 10; // keep random writes away from command register
          send_write_and_wait(wr_addr[idx], d, 1.0, $urandom_range(0,2));
          if ($urandom_range(0,1)) send_read_and_wait(wr_addr[idx], ~d);
        end
        2: begin
          idx = $urandom_range(0,N_RO-1);
          send_read_and_wait(ro_addr[idx], d);
        end
        default: begin
          idx = $urandom_range(0,10);
          send_read_and_wait(wr_addr[idx], d);
        end
      endcase
    end
    test_end(n);
  endtask

  // --------------------------------------------------------------------------
  // Final report
  // --------------------------------------------------------------------------
  task automatic report_coverage;
    int uncovered;
    uncovered = 0;
    $display("\n================ FUNCTIONAL HIT REPORT ===================");
    for (int i=0; i<N_WR; i++) begin
      $display("  WR %-20s addr=%02h writes=%0d reads=%0d",
               wr_name[i], wr_addr[i], wr_write_hits[i], wr_read_hits[i]);
      if (wr_write_hits[i] == 0) uncovered++;
      if (wr_read_hits[i]  == 0) uncovered++;
    end
    for (int i=0; i<N_RO; i++) begin
      $display("  RO %-20s addr=%02h write_attempts=%0d reads=%0d",
               ro_name[i], ro_addr[i], ro_write_hits[i], ro_read_hits[i]);
      if (ro_write_hits[i] == 0) uncovered++;
      if (ro_read_hits[i]  == 0) uncovered++;
    end
    $display("  calibrate pulses expected/actual = %0d/%0d", expected_cal_pulses, actual_cal_pulses);
    $display("  clear pulses     expected/actual = %0d/%0d", expected_clear_pulses, actual_clear_pulses);
    $display("  observed serializer backpressure cycles = %0d", backpressure_cycles);
    $display("  uncovered required register access bins = %0d", uncovered);
    $display("=========================================================\n");
    pass_check(uncovered == 0, "one or more required register access coverage bins were not hit");
  endtask

  task automatic final_report;
    $display("\n================ UART PIPELINE TB SUMMARY ================");
    $display("Tests run/pass/fail       : %0d / %0d / %0d", tests_run, tests_passed, tests_failed);
    $display("Checks / errors / warnings: %0d / %0d / %0d", checks, errors, warnings);
    $display("RX bytes seen             : %0d", rx_bytes_seen);
    $display("Parser requests seen      : %0d", parser_reqs_seen);
    $display("Regfile responses seen    : %0d", rf_rsps_seen);
    $display("Serializer bytes seen     : %0d", serializer_bytes_seen);
    $display("TX bytes / frames seen    : %0d / %0d", tx_bytes_seen, tx_frames_seen);
    $display("Remaining expected queues : req=%0d rf=%0d ser=%0d tx=%0d rx=%0d",
             exp_req_q.size(), exp_rf_rsp_q.size(), exp_ser_byte_q.size(),
             exp_tx_rsp_q.size(), exp_rx_byte_q.size());
    $display("==========================================================");

    if (errors == 0) begin
      $display("[TB][PASS] UART pipeline regression completed with zero errors.");
    end else begin
      $display("[TB][FAIL] UART pipeline regression completed with %0d errors.", errors);
    end
  endtask

  // --------------------------------------------------------------------------
  // Main sequence
  // --------------------------------------------------------------------------
  initial begin : main
    $timeformat(-6, 3, " us", 12);
    init_config();
    print_config();
    validate_config();

    // Unique telemetry sentinels make address/data mixups obvious.
    ro_in[0] = 32'h5100_0001;
    ro_in[1] = 32'h5200_1002;
    ro_in[2] = 32'h5300_2003;
    ro_in[3] = 32'h5400_3004;
    ro_in[4] = 32'h5500_4005;
    ro_in[5] = 32'h5600_5006;
    ro_in[6] = 32'h5700_6007;
    ro_in[7] = 32'h5800_7008;

    for (int i=0; i<N_WR; i++) begin
      wr_write_hits[i] = 0;
      wr_read_hits[i]  = 0;
    end
    for (int i=0; i<N_RO; i++) begin
      ro_write_hits[i] = 0;
      ro_read_hits[i]  = 0;
    end

    reset_reference_model();
    clear_scoreboard_queues();
    apply_reset("initial reset");

    test_reset_sanity();
    recover_after_failed_test("reset_sanity");
    test_all_writable_registers();
    recover_after_failed_test("all_writable_registers_write_and_readback");
    test_data_patterns_endianness();
    recover_after_failed_test("data_patterns_and_endianness");
    test_read_only_registers();
    recover_after_failed_test("read_only_telemetry_and_write_ignore");
    test_unmapped_in_range_address();
    recover_after_failed_test("unmapped_in_range_address_zero_read_ignore_write");
    test_control_semantics();
    recover_after_failed_test("control_self_clear_and_command_pulses");
    test_parser_rejection_and_resync();
    recover_after_failed_test("parser_rejection_bad_sync_cmd_addr_crc_and_resync");
    test_crc_detects_data_bit_errors();
    recover_after_failed_test("crc_detects_single_bit_data_errors");
    test_interbyte_stalls();
    recover_after_failed_test("parser_interbyte_stalls");
    test_uart_rx_weak_areas();
    recover_after_failed_test("uart_rx_false_start_bad_stop_glitch_and_baud_tolerance");
    test_serializer_latches_read_response();
    recover_after_failed_test("serializer_latches_response_against_live_telemetry_change");
    test_guarded_back_to_back_writes();
    recover_after_failed_test("back_to_back_frames_with_regression_guard");

    // Full-rate UART byte-stream test is isolated so a known RX throughput defect
    // becomes one precise failure instead of shifting every later expected queue.
    test_uart_rx_contiguous_bytes();
    recover_after_failed_test("uart_rx_contiguous_8n1_no_extra_idle");

    // Diagnostic architectural weak areas are run before a reset so they cannot
    // contaminate the deterministic regression that follows.
    test_burst_reads_diagnostic();
    recover_after_failed_test("burst_reads_while_serializer_tx_busy");
    apply_reset("after burst-read diagnostic");
    test_truncated_frame_resync_diagnostic();
    recover_after_failed_test("truncated_frame_in_band_resync_diagnostic");

    test_reset_mid_frame_recovery();
    recover_after_failed_test("reset_mid_rx_frame_and_recovery");
    test_random_regression();
    recover_after_failed_test("randomized_end_to_end_regression");

    wait_scoreboards_empty(150*UART_BIT, "end of regression");
    repeat (10) @(posedge clk);

    pass_check(expected_cal_pulses == actual_cal_pulses,
      $sformatf("calibrate pulse count mismatch exp=%0d got=%0d", expected_cal_pulses, actual_cal_pulses));
    pass_check(expected_clear_pulses == actual_clear_pulses,
      $sformatf("clear-fault pulse count mismatch exp=%0d got=%0d", expected_clear_pulses, actual_clear_pulses));

    report_coverage();
    final_report();

    if (errors != 0) $fatal(1, "UART pipeline testbench FAILED");
    $finish;
  end

  // Global deadman. Increase if RANDOM_ITERS is made very large.
  initial begin : watchdog
    #(50ms);
    $fatal(1, "[TB][WATCHDOG] simulation exceeded 50 ms; likely deadlock or wrong CLK_HZ/BAUD configuration");
  end

endmodule
