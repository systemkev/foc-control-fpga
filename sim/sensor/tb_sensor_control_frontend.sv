`timescale 1ns/1ps

module tb_sensor_control_frontend;

  // ---------------------------------------------------------------------------
  // Constants mirrored from the supplied VHDL packages.
  // Keep these synchronized with common_pkg.vhd, math_pkg.vhd, and spi_pkg.vhd.
  // ---------------------------------------------------------------------------
  localparam time CLK_PERIOD            = 20ns;       // 50 MHz
  localparam int  RAW_BITS              = 14;
  localparam int  RAW_MOD               = 1 << RAW_BITS;
  localparam int  RAW_MASK              = RAW_MOD - 1;
  localparam int  NUM_POLE_PAIRS        = 7;
  localparam int  CORDIC_ITERS          = 20;
  localparam int  CORDIC_X0_Q19         = 318375;     // round(0.607253 * 2^19)
  localparam int  TRACKER_ALPHA_Q20     = 210;        // round(2.0e-4 * 2^20)
  localparam int  TRACKER_HALF_TURN     = 8192;
  localparam int  CORDIC_IDEAL_TOL_LSB  = 7;
  localparam logic [15:0] READ_ANGLE_CMD = 16'hFFFF;
  localparam logic [15:0] CLEAR_ERR_CMD  = 16'h4001;

  // VHDL C_ARCTAN_TABLE values after real->integer rounding.
  localparam int signed CORDIC_ATAN [0:CORDIC_ITERS-1] = '{
    8192, 4836, 2555, 1297, 651, 326, 163, 81, 41, 20,
    10, 5, 3, 1, 1, 0, 0, 0, 0, 0
  };

  localparam real PI = 3.1415926535897932384626433832795;

  // ---------------------------------------------------------------------------
  // Clock/reset
  // ---------------------------------------------------------------------------
  logic clk = 1'b0;
  logic rst = 1'b1;
  always #(CLK_PERIOD/2) clk = ~clk;

  // ---------------------------------------------------------------------------
  // End-to-end front-end chain:
  //   AS5048A SPI raw angle -> tracker
  //                         -> mech_to_elec -> cordic
  //
  // This is intentionally a branch, not tracker->mech_to_elec, because the
  // supplied RTL declares tracker.o_actual_pos as signed[31:0] but
  // mech_to_elec.i_mech_pos as unsigned[13:0].
  // ---------------------------------------------------------------------------
  logic        spi_start;
  logic        spi_miso;
  logic        spi_mosi;
  logic        spi_n_cs;
  logic        spi_sclk;
  logic        spi_vld;
  logic [13:0] spi_raw_angle;

  logic               p_tr_vld;
  logic signed [31:0] p_tr_pos;
  logic signed [28:0] p_tr_vel;

  logic [13:0]        pipe_offset;
  logic               p_mech_vld;
  logic [13:0]        p_elec_pos;
  logic signed [15:0] p_elec_cordic;

  logic               p_cordic_rdy;
  logic               p_cordic_vld;
  logic signed [15:0] p_cos;
  logic signed [15:0] p_sin;

  as5048a_spi_master u_spi (
    .i_clk       (clk),
    .i_rst       (rst),
    .i_start     (spi_start),
    .i_miso      (spi_miso),
    .o_mosi      (spi_mosi),
    .o_n_cs      (spi_n_cs),
    .o_sclk      (spi_sclk),
    .o_vld       (spi_vld),
    .o_raw_angle (spi_raw_angle)
  );

  tracker u_tracker_pipe (
    .i_clk        (clk),
    .i_rst        (rst),
    .i_vld        (spi_vld),
    .i_actual_pos (spi_raw_angle),
    .o_vld        (p_tr_vld),
    .o_actual_pos (p_tr_pos),
    .o_actual_vel (p_tr_vel)
  );

  mech_to_elec u_mech_pipe (
    .i_clk         (clk),
    .i_rst         (rst),
    .i_vld         (spi_vld),
    .i_mech_pos    (spi_raw_angle),
    .i_offset      (pipe_offset),
    .o_vld         (p_mech_vld),
    .o_elec_pos    (p_elec_pos),
    .o_elec_cordic (p_elec_cordic)
  );

  cordic u_cordic_pipe (
    .i_clk   (clk),
    .i_rst   (rst),
    .i_vld   (p_mech_vld),
    .o_rdy   (p_cordic_rdy),
    .i_angle (p_elec_cordic),
    .o_vld   (p_cordic_vld),
    .o_cos   (p_cos),
    .o_sin   (p_sin)
  );

  // ---------------------------------------------------------------------------
  // Focused tracker instance.  This allows high-rate wrap/filter stress without
  // waiting for an SPI transaction for every sample.
  // ---------------------------------------------------------------------------
  logic               tr_i_vld;
  logic [13:0]        tr_i_raw;
  logic               tr_o_vld;
  logic signed [31:0] tr_o_pos;
  logic signed [28:0] tr_o_vel;

  tracker u_tracker_direct (
    .i_clk        (clk),
    .i_rst        (rst),
    .i_vld        (tr_i_vld),
    .i_actual_pos (tr_i_raw),
    .o_vld        (tr_o_vld),
    .o_actual_pos (tr_o_pos),
    .o_actual_vel (tr_o_vel)
  );

  // ---------------------------------------------------------------------------
  // Focused mech_to_elec instance.
  // ---------------------------------------------------------------------------
  logic               m_i_vld;
  logic [13:0]        m_i_pos;
  logic [13:0]        m_i_offset;
  logic               m_o_vld;
  logic [13:0]        m_o_elec;
  logic signed [15:0] m_o_cordic;

  mech_to_elec u_mech_direct (
    .i_clk         (clk),
    .i_rst         (rst),
    .i_vld         (m_i_vld),
    .i_mech_pos    (m_i_pos),
    .i_offset      (m_i_offset),
    .o_vld         (m_o_vld),
    .o_elec_pos    (m_o_elec),
    .o_elec_cordic (m_o_cordic)
  );

  // ---------------------------------------------------------------------------
  // Focused CORDIC instance.  This reaches every signed 16-bit input, including
  // values not reachable through mech_to_elec (which always produces multiples
  // of four in its CORDIC encoding).
  // ---------------------------------------------------------------------------
  logic               c_i_vld;
  logic signed [15:0] c_i_angle;
  logic               c_o_rdy;
  logic               c_o_vld;
  logic signed [15:0] c_o_cos;
  logic signed [15:0] c_o_sin;

  cordic u_cordic_direct (
    .i_clk   (clk),
    .i_rst   (rst),
    .i_vld   (c_i_vld),
    .o_rdy   (c_o_rdy),
    .i_angle (c_i_angle),
    .o_vld   (c_o_vld),
    .o_cos   (c_o_cos),
    .o_sin   (c_o_sin)
  );

  // ---------------------------------------------------------------------------
  // Self-reporting scoreboard counters
  // ---------------------------------------------------------------------------
  int unsigned n_checks = 0;
  int unsigned n_errors = 0;
  int unsigned n_tests  = 0;
  int unsigned n_warnings = 0;
  int unsigned spi_total_vld_count = 0;
  bit allow_known_cordic_reset_glitch = 0;

  task automatic sb_check(input bit condition, input string message);
    n_checks++;
    if (!condition) begin
      n_errors++;
      $error("[%0t] CHECK FAILED: %s", $time, message);
    end
  endtask

  task automatic sb_warn(input bit condition, input string message);
    if (condition) begin
      n_warnings++;
      $display("[%0t] WARNING: %s", $time, message);
    end
  endtask

  task automatic start_test(input string name);
    n_tests++;
    $display("\n[%0t] TEST %0d: %s", $time, n_tests, name);
  endtask

  function automatic int abs_i(input int v);
    return (v < 0) ? -v : v;
  endfunction


  // Deterministic stateful PRNG.  Avoid passing the same seed to
  // $urandom(seed) on every call: some simulators treat that as a re-seed,
  // collapsing a supposedly-random regression to the same value repeatedly.
  function automatic int unsigned prng_next(inout int unsigned state);
    int unsigned x;
    begin
      x = state;
      if (x == 0)
        x = 32'h1;
      x ^= (x << 13);
      x ^= (x >> 17);
      x ^= (x << 5);
      state = x;
      return x;
    end
  endfunction

  function automatic longint signed wrap_signed(
    input longint signed value,
    input int width
  );
    longint signed modulus;
    longint signed half;
    longint signed t;
    begin
      modulus = 64'sd1 <<< width;
      half    = 64'sd1 <<< (width-1);
      t       = value % modulus;
      if (t < 0)
        t = t + modulus;
      if (t >= half)
        t = t - modulus;
      return t;
    end
  endfunction

  function automatic int round_nearest(input real x);
    if (x >= 0.0)
      return $rtoi(x + 0.5);
    else
      return $rtoi(x - 0.5);
  endfunction

  // ---------------------------------------------------------------------------
  // AS5048A response helper.  Bit 15 is chosen so XOR of all 16 bits is zero.
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] as5048_frame(
    input logic [13:0] angle,
    input bit error_flag,
    input bit corrupt_parity
  );
    logic [15:0] f;
    begin
      f         = '0;
      f[13:0]   = angle;
      f[14]     = error_flag;
      f[15]     = ^f[14:0];
      if (corrupt_parity)
        f[15] = ~f[15];
      return f;
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Exact fixed-point CORDIC reference model.
  // This mirrors the widths, arithmetic shifts, quadrant folding, rounded
  // atan table, and final Q1.19->Q1.14 shift in the supplied RTL.
  // ---------------------------------------------------------------------------
  task automatic calc_cordic_fixed(
    input  logic signed [15:0] angle,
    output logic signed [15:0] cos_exact,
    output logic signed [15:0] sin_exact
  );
    longint signed x, y, z;
    longint signed x_shift, y_shift;
    longint signed nx, ny, nz;
    int unsigned bits;
    bit inv;
    begin
      bits = $unsigned(angle);
      inv  = bits[15] ^ bits[14];

      if (inv) begin
        // signed resize of i_angle(14 downto 0) from 15 bits to 16 bits
        z = wrap_signed(bits & 16'h7FFF, 15);
        z = wrap_signed(z, 16);
      end else begin
        z = wrap_signed(bits, 16);
      end

      x = wrap_signed(CORDIC_X0_Q19, 21);
      y = 0;

      for (int i = 0; i < CORDIC_ITERS; i++) begin
        x_shift = x >>> i;
        y_shift = y >>> i;

        if (z >= 0) begin
          nx = wrap_signed(x - y_shift, 21);
          ny = wrap_signed(y + x_shift, 21);
          nz = wrap_signed(z - CORDIC_ATAN[i], 16);
        end else begin
          nx = wrap_signed(x + y_shift, 21);
          ny = wrap_signed(y - x_shift, 21);
          nz = wrap_signed(z + CORDIC_ATAN[i], 16);
        end

        x = nx;
        y = ny;
        z = nz;
      end

      if (inv) begin
        cos_exact = wrap_signed((-x) >>> 5, 16);
        sin_exact = wrap_signed((-y) >>> 5, 16);
      end else begin
        cos_exact = wrap_signed(x >>> 5, 16);
        sin_exact = wrap_signed(y >>> 5, 16);
      end
    end
  endtask

  task automatic calc_cordic_ideal(
    input  logic signed [15:0] angle,
    output int signed ideal_cos,
    output int signed ideal_sin
  );
    int signed a;
    real rad;
    begin
      a = $signed(angle);
      rad = PI * $itor(a) / 32768.0;
      ideal_cos = round_nearest($cos(rad) * 16384.0);
      ideal_sin = round_nearest($sin(rad) * 16384.0);
    end
  endtask

  function automatic logic [13:0] expected_elec_pos(
    input logic [13:0] mech,
    input logic [13:0] offset
  );
    int unsigned t;
    begin
      t = NUM_POLE_PAIRS * $unsigned(mech) + $unsigned(offset);
      return t[13:0]; // modulo 2^14, matching resize to 14 bits
    end
  endfunction

  function automatic logic signed [15:0] expected_mech_cordic(
    input logic [13:0] elec
  );
    logic [15:0] bits;
    begin
      bits = {~elec[13], elec[12:0], 2'b00};
      return $signed(bits);
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Scoreboard transaction types / queues
  // ---------------------------------------------------------------------------
  typedef struct {
    logic signed [31:0] pos;
    logic signed [28:0] vel;
    realtime accepted_at;
  } tracker_exp_t;

  typedef struct {
    logic [13:0]        elec;
    logic signed [15:0] cordic_angle;
    realtime accepted_at;
  } mech_exp_t;

  typedef struct {
    logic signed [15:0] angle;
    logic signed [15:0] cos_exact;
    logic signed [15:0] sin_exact;
    int signed          cos_ideal;
    int signed          sin_ideal;
    realtime accepted_at;
  } cordic_exp_t;

  tracker_exp_t tr_q[$];
  tracker_exp_t p_tr_q[$];
  mech_exp_t    m_q[$];
  mech_exp_t    p_m_q[$];
  cordic_exp_t  c_q[$];
  cordic_exp_t  p_c_q[$];

  // ---------------------------------------------------------------------------
  // Tracker reference state: direct instance
  // ---------------------------------------------------------------------------
  bit            tr_ref_initialized;
  int signed     tr_ref_wraps;
  int signed     tr_ref_last_raw;
  longint signed tr_ref_pos;
  longint signed tr_ref_last_pos;
  longint signed tr_ref_omega;

  // Tracker reference state: pipeline instance
  bit            p_tr_ref_initialized;
  int signed     p_tr_ref_wraps;
  int signed     p_tr_ref_last_raw;
  longint signed p_tr_ref_pos;
  longint signed p_tr_ref_last_pos;
  longint signed p_tr_ref_omega;

  int unsigned cov_tracker_fwd_wrap;
  int unsigned cov_tracker_rev_wrap;
  int unsigned cov_tracker_pos_half_exact;
  int unsigned cov_tracker_neg_half_exact;
  int unsigned cov_mech_mod_wrap;
  int unsigned cov_cordic_quad[0:3];
  int unsigned cov_cordic_axis_or_edge;

  task automatic tracker_ref_reset_direct;
    tr_ref_initialized = 0;
    tr_ref_wraps       = 0;
    tr_ref_last_raw    = 0;
    tr_ref_pos         = 0;
    tr_ref_last_pos    = 0;
    tr_ref_omega       = 0;
  endtask

  task automatic tracker_ref_reset_pipe;
    p_tr_ref_initialized = 0;
    p_tr_ref_wraps       = 0;
    p_tr_ref_last_raw    = 0;
    p_tr_ref_pos         = 0;
    p_tr_ref_last_pos    = 0;
    p_tr_ref_omega       = 0;
  endtask

  // Bit-accurate tracker model.  Note the deliberate 29-bit wrap around the
  // Q8.20 left shift, because shift_left on a signed(28 downto 0) retains width.
  task automatic tracker_ref_accept_direct(input logic [13:0] raw);
    tracker_exp_t e;
    int signed diff;
    longint signed delta29;
    longint signed delta_q20_29;
    longint signed err29;
    longint signed correction29;
    begin
      if (!tr_ref_initialized) begin
        tr_ref_initialized = 1;
        tr_ref_wraps       = 0;
        tr_ref_last_raw    = $unsigned(raw);
        tr_ref_pos         = $unsigned(raw);
        tr_ref_last_pos    = $unsigned(raw);
        tr_ref_omega       = 0;
      end else begin
        diff = $unsigned(raw) - tr_ref_last_raw;

        if (diff >= TRACKER_HALF_TURN) begin
          tr_ref_wraps--;
          cov_tracker_rev_wrap++;
          if (diff == TRACKER_HALF_TURN)
            cov_tracker_pos_half_exact++;
        end else if (diff <= -TRACKER_HALF_TURN) begin
          tr_ref_wraps++;
          cov_tracker_fwd_wrap++;
          if (diff == -TRACKER_HALF_TURN)
            cov_tracker_neg_half_exact++;
        end

        tr_ref_last_raw = $unsigned(raw);
        tr_ref_last_pos = tr_ref_pos;
        tr_ref_pos      = longint'(tr_ref_wraps) * RAW_MOD + $unsigned(raw);

        delta29       = wrap_signed(tr_ref_pos - tr_ref_last_pos, 29);
        delta_q20_29  = wrap_signed(delta29 <<< 20, 29);
        err29          = wrap_signed(delta_q20_29 - tr_ref_omega, 29);
        correction29   = wrap_signed((TRACKER_ALPHA_Q20 * err29) >>> 20, 29);
        tr_ref_omega   = wrap_signed(tr_ref_omega + correction29, 29);
      end

      e.pos         = tr_ref_pos[31:0];
      e.vel         = tr_ref_omega[28:0];
      e.accepted_at = $realtime;
      tr_q.push_back(e);
    end
  endtask

  task automatic tracker_ref_accept_pipe(input logic [13:0] raw);
    tracker_exp_t e;
    int signed diff;
    longint signed delta29;
    longint signed delta_q20_29;
    longint signed err29;
    longint signed correction29;
    begin
      if (!p_tr_ref_initialized) begin
        p_tr_ref_initialized = 1;
        p_tr_ref_wraps       = 0;
        p_tr_ref_last_raw    = $unsigned(raw);
        p_tr_ref_pos         = $unsigned(raw);
        p_tr_ref_last_pos    = $unsigned(raw);
        p_tr_ref_omega       = 0;
      end else begin
        diff = $unsigned(raw) - p_tr_ref_last_raw;

        if (diff >= TRACKER_HALF_TURN)
          p_tr_ref_wraps--;
        else if (diff <= -TRACKER_HALF_TURN)
          p_tr_ref_wraps++;

        p_tr_ref_last_raw = $unsigned(raw);
        p_tr_ref_last_pos = p_tr_ref_pos;
        p_tr_ref_pos      = longint'(p_tr_ref_wraps) * RAW_MOD + $unsigned(raw);

        delta29       = wrap_signed(p_tr_ref_pos - p_tr_ref_last_pos, 29);
        delta_q20_29  = wrap_signed(delta29 <<< 20, 29);
        err29          = wrap_signed(delta_q20_29 - p_tr_ref_omega, 29);
        correction29   = wrap_signed((TRACKER_ALPHA_Q20 * err29) >>> 20, 29);
        p_tr_ref_omega = wrap_signed(p_tr_ref_omega + correction29, 29);
      end

      e.pos         = p_tr_ref_pos[31:0];
      e.vel         = p_tr_ref_omega[28:0];
      e.accepted_at = $realtime;
      p_tr_q.push_back(e);
    end
  endtask

  task automatic mech_ref_accept_direct(
    input logic [13:0] mech,
    input logic [13:0] offset
  );
    mech_exp_t e;
    int unsigned full_sum;
    begin
      full_sum = NUM_POLE_PAIRS * $unsigned(mech) + $unsigned(offset);
      if (full_sum >= RAW_MOD)
        cov_mech_mod_wrap++;
      e.elec          = expected_elec_pos(mech, offset);
      e.cordic_angle  = expected_mech_cordic(e.elec);
      e.accepted_at   = $realtime;
      m_q.push_back(e);
    end
  endtask

  task automatic cordic_ref_accept_direct(input logic signed [15:0] angle);
    cordic_exp_t e;
    int signed a;
    begin
      e.angle = angle;
      calc_cordic_fixed(angle, e.cos_exact, e.sin_exact);
      calc_cordic_ideal(angle, e.cos_ideal, e.sin_ideal);
      e.accepted_at = $realtime;
      c_q.push_back(e);

      a = $signed(angle);
      if (a < -16384)      cov_cordic_quad[2]++;
      else if (a < 0)      cov_cordic_quad[3]++;
      else if (a < 16384)  cov_cordic_quad[0]++;
      else                 cov_cordic_quad[1]++;

      if ((a == -32768) || (a == -16384) || (a == -1) ||
          (a == 0) || (a == 1) || (a == 16384) || (a == 32767))
        cov_cordic_axis_or_edge++;
    end
  endtask

  task automatic pipeline_ref_accept(
    input logic [13:0] raw,
    input logic [13:0] offset
  );
    mech_exp_t m;
    cordic_exp_t c;
    begin
      tracker_ref_accept_pipe(raw);

      m.elec         = expected_elec_pos(raw, offset);
      m.cordic_angle = expected_mech_cordic(m.elec);
      m.accepted_at  = $realtime;
      p_m_q.push_back(m);

      c.angle = m.cordic_angle;
      calc_cordic_fixed(c.angle, c.cos_exact, c.sin_exact);
      calc_cordic_ideal(c.angle, c.cos_ideal, c.sin_ideal);
      c.accepted_at = 0.0; // overwritten when the CORDIC handshake occurs
      p_c_q.push_back(c);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Central scoreboard sampling.
  // Inputs are sampled at the active edge before VHDL signal updates. Outputs
  // are sampled 1 ps later so mixed-language delta-cycle updates have settled.
  // ---------------------------------------------------------------------------
  bit prev_spi_vld;
  bit prev_c_vld;
  bit prev_p_c_vld;

  always @(posedge clk) begin : SCOREBOARD
    tracker_exp_t te;
    mech_exp_t    me;
    cordic_exp_t  ce;
    realtime      latency;

    // Acceptance / reference model side
    if (!rst) begin
      if (tr_i_vld)
        tracker_ref_accept_direct(tr_i_raw);

      if (m_i_vld)
        mech_ref_accept_direct(m_i_pos, m_i_offset);

      if (c_i_vld && c_o_rdy)
        cordic_ref_accept_direct(c_i_angle);

      // This is exactly the edge on which tracker/mech consume the SPI angle.
      if (spi_vld)
        pipeline_ref_accept(spi_raw_angle, pipe_offset);

      // End-to-end contract: every mech pulse must meet a ready CORDIC.
      if (p_mech_vld)
        sb_check(p_cordic_rdy,
          "pipeline CORDIC was not ready when mech_to_elec asserted o_vld");

      // Record the true pipeline CORDIC acceptance time on its pending head.
      if (p_mech_vld && p_cordic_rdy) begin
        sb_check(p_c_q.size() != 0,
          "pipeline CORDIC accepted data but expected queue was empty");
        if (p_c_q.size() != 0) begin
          ce = p_c_q.pop_front();
          sb_check(p_elec_cordic === ce.angle,
            $sformatf("pipeline CORDIC input mismatch exp=%0d got=%0d",
                      $signed(ce.angle), $signed(p_elec_cordic)));
          ce.accepted_at = $realtime;
          p_c_q.push_front(ce);
        end
      end
    end

    #1ps;

    if (rst) begin
      sb_check(spi_n_cs === 1'b1, "SPI CS must be high in reset");
      sb_check(spi_sclk === 1'b0, "SPI SCLK must be low in reset");
      sb_check(spi_vld  === 1'b0, "SPI valid must be low in reset");
      sb_check(tr_o_vld === 1'b0, "direct tracker valid must be low in reset");
      sb_check(m_o_vld  === 1'b0, "direct mech valid must be low in reset");
      sb_check(p_tr_vld === 1'b0, "pipeline tracker valid must be low in reset");
      sb_check(p_mech_vld === 1'b0, "pipeline mech valid must be low in reset");
      sb_check(p_cordic_vld === 1'b0, "pipeline CORDIC valid must be low in reset");
      if (c_o_vld !== 1'b0) begin
        if (allow_known_cordic_reset_glitch)
          sb_warn(1'b1, "KNOWN RTL ISSUE: direct CORDIC o_vld asserted during reset (datapath has no explicit reset guard)");
        else
          sb_check(1'b0, "direct CORDIC valid must be low in reset");
      end else begin
        sb_check(1'b1, "direct CORDIC valid low in reset");
      end
    end else begin
      // SPI valid must be exactly one system clock wide.
      sb_check(!(spi_vld && prev_spi_vld), "SPI o_vld wider than one clock");
      if (spi_vld)
        spi_total_vld_count++;

      // Direct tracker scoreboard
      if (tr_o_vld) begin
        sb_check(tr_q.size() != 0, "direct tracker produced unexpected o_vld");
        if (tr_q.size() != 0) begin
          te = tr_q.pop_front();
          sb_check(tr_o_pos === te.pos,
            $sformatf("tracker pos mismatch exp=%0d got=%0d",
                      $signed(te.pos), $signed(tr_o_pos)));
          sb_check(tr_o_vel === te.vel,
            $sformatf("tracker vel mismatch exp=%0d got=%0d",
                      $signed(te.vel), $signed(tr_o_vel)));
          latency = $realtime - te.accepted_at;
          sb_check((latency > 19.999) && (latency < 20.002),
            $sformatf("tracker latency expected 20ns got %.3fns", latency));
        end
      end

      // Pipeline tracker scoreboard
      if (p_tr_vld) begin
        sb_check(p_tr_q.size() != 0, "pipeline tracker produced unexpected o_vld");
        if (p_tr_q.size() != 0) begin
          te = p_tr_q.pop_front();
          sb_check(p_tr_pos === te.pos,
            $sformatf("pipeline tracker pos mismatch exp=%0d got=%0d",
                      $signed(te.pos), $signed(p_tr_pos)));
          sb_check(p_tr_vel === te.vel,
            $sformatf("pipeline tracker vel mismatch exp=%0d got=%0d",
                      $signed(te.vel), $signed(p_tr_vel)));
          latency = $realtime - te.accepted_at;
          sb_check((latency > 19.999) && (latency < 20.002),
            $sformatf("pipeline tracker latency expected 20ns got %.3fns", latency));
        end
      end

      // Direct mech scoreboard
      if (m_o_vld) begin
        sb_check(m_q.size() != 0, "direct mech_to_elec produced unexpected o_vld");
        if (m_q.size() != 0) begin
          me = m_q.pop_front();
          sb_check(m_o_elec === me.elec,
            $sformatf("mech elec mismatch exp=%0d got=%0d", me.elec, m_o_elec));
          sb_check(m_o_cordic === me.cordic_angle,
            $sformatf("mech CORDIC map mismatch exp=%0d got=%0d",
                      $signed(me.cordic_angle), $signed(m_o_cordic)));
          latency = $realtime - me.accepted_at;
          sb_check((latency > 19.999) && (latency < 20.002),
            $sformatf("mech latency expected 20ns got %.3fns", latency));
        end
      end

      // Pipeline mech scoreboard
      if (p_mech_vld) begin
        sb_check(p_m_q.size() != 0, "pipeline mech_to_elec produced unexpected o_vld");
        if (p_m_q.size() != 0) begin
          me = p_m_q.pop_front();
          sb_check(p_elec_pos === me.elec,
            $sformatf("pipeline elec mismatch exp=%0d got=%0d", me.elec, p_elec_pos));
          sb_check(p_elec_cordic === me.cordic_angle,
            $sformatf("pipeline CORDIC angle mismatch exp=%0d got=%0d",
                      $signed(me.cordic_angle), $signed(p_elec_cordic)));
          latency = $realtime - me.accepted_at;
          sb_check((latency > 19.999) && (latency < 20.002),
            $sformatf("pipeline mech latency expected 20ns got %.3fns", latency));
        end
      end

      // Direct CORDIC scoreboard: exact implementation oracle + numerical oracle.
      if (c_o_vld) begin
        sb_check(c_q.size() != 0, "direct CORDIC produced unexpected o_vld");
        if (c_q.size() != 0) begin
          ce = c_q.pop_front();
          sb_check(c_o_cos === ce.cos_exact,
            $sformatf("CORDIC cos bit-exact mismatch angle=%0d exp=%0d got=%0d",
                      $signed(ce.angle), $signed(ce.cos_exact), $signed(c_o_cos)));
          sb_check(c_o_sin === ce.sin_exact,
            $sformatf("CORDIC sin bit-exact mismatch angle=%0d exp=%0d got=%0d",
                      $signed(ce.angle), $signed(ce.sin_exact), $signed(c_o_sin)));
          sb_check(abs_i($signed(c_o_cos) - ce.cos_ideal) <= CORDIC_IDEAL_TOL_LSB,
            $sformatf("CORDIC cos numerical error > %0d LSB angle=%0d ideal=%0d got=%0d",
                      CORDIC_IDEAL_TOL_LSB, $signed(ce.angle), ce.cos_ideal, $signed(c_o_cos)));
          sb_check(abs_i($signed(c_o_sin) - ce.sin_ideal) <= CORDIC_IDEAL_TOL_LSB,
            $sformatf("CORDIC sin numerical error > %0d LSB angle=%0d ideal=%0d got=%0d",
                      CORDIC_IDEAL_TOL_LSB, $signed(ce.angle), ce.sin_ideal, $signed(c_o_sin)));
          latency = $realtime - ce.accepted_at;
          sb_check((latency > 419.999) && (latency < 420.002),
            $sformatf("CORDIC latency expected 420ns/21 clocks got %.3fns", latency));
        end
      end
      sb_check(!(c_o_vld && prev_c_vld), "direct CORDIC o_vld wider than one clock");

      // Pipeline CORDIC scoreboard.  Queue head remains in place between
      // handshake and output; accepted_at is stamped at handshake above.
      if (p_cordic_vld) begin
        sb_check(p_c_q.size() != 0, "pipeline CORDIC produced unexpected o_vld");
        if (p_c_q.size() != 0) begin
          ce = p_c_q.pop_front();
          sb_check(p_cos === ce.cos_exact,
            $sformatf("pipeline cos mismatch angle=%0d exp=%0d got=%0d",
                      $signed(ce.angle), $signed(ce.cos_exact), $signed(p_cos)));
          sb_check(p_sin === ce.sin_exact,
            $sformatf("pipeline sin mismatch angle=%0d exp=%0d got=%0d",
                      $signed(ce.angle), $signed(ce.sin_exact), $signed(p_sin)));
          sb_check(abs_i($signed(p_cos) - ce.cos_ideal) <= CORDIC_IDEAL_TOL_LSB,
            "pipeline cos exceeded ideal-error tolerance");
          sb_check(abs_i($signed(p_sin) - ce.sin_ideal) <= CORDIC_IDEAL_TOL_LSB,
            "pipeline sin exceeded ideal-error tolerance");
          latency = $realtime - ce.accepted_at;
          sb_check((latency > 419.999) && (latency < 420.002),
            $sformatf("pipeline CORDIC latency expected 420ns got %.3fns", latency));
        end
      end
      sb_check(!(p_cordic_vld && prev_p_c_vld), "pipeline CORDIC o_vld wider than one clock");
    end

    prev_spi_vld <= spi_vld;
    prev_c_vld   <= c_o_vld;
    prev_p_c_vld <= p_cordic_vld;
  end

  // ---------------------------------------------------------------------------
  // Behavioral AS5048A SPI slave / protocol checker
  //
  // The test supplies an exact sequence of expected MOSI command frames and
  // response words. MISO is changed only after each falling SCLK edge, i.e. just
  // after the master samples it. MOSI is sampled on rising SCLK where it is
  // stable in this master implementation.
  // ---------------------------------------------------------------------------
  logic [15:0] spi_cmd_plan [0:31];
  logic [15:0] spi_rsp_plan [0:31];
  int          spi_plan_len;
  int          spi_frame_idx;
  bit          spi_plan_active;
  bit          spi_frame_active;
  int          spi_bit_idx;
  int          spi_rise_count;
  int          spi_fall_count;
  logic [15:0] spi_cmd_capture;
  logic [15:0] spi_rsp_current;
  realtime     spi_cs_fall_time;
  realtime     spi_last_cs_rise_time;
  realtime     spi_last_sclk_edge_time;
  realtime     spi_last_fall_time;
  bit          spi_have_last_cs_rise;
  bit          spi_have_sclk_edge;

  int unsigned cov_spi_normal;
  int unsigned cov_spi_parity_retry;
  int unsigned cov_spi_error_clear;
  int unsigned cov_spi_parity_over_error;
  int unsigned cov_spi_busy_start;
  int unsigned cov_spi_reset_abort;

  task automatic spi_plan_clear;
    spi_plan_len = 0;
    spi_frame_idx = 0;
  endtask

  task automatic spi_plan_add(
    input logic [15:0] expected_cmd,
    input logic [15:0] response
  );
    sb_check(spi_plan_len < 32, "SPI plan overflow");
    if (spi_plan_len < 32) begin
      spi_cmd_plan[spi_plan_len] = expected_cmd;
      spi_rsp_plan[spi_plan_len] = response;
      spi_plan_len++;
    end
  endtask

  // CS asserted: load the next response MSB immediately.
  always @(negedge spi_n_cs) begin
    if (!rst) begin
      sb_check(spi_plan_active, "SPI transaction started without an active sensor plan");
      sb_check(!spi_frame_active, "nested SPI frame detected");
      sb_check(spi_frame_idx < spi_plan_len, "SPI master emitted more frames than planned");

      spi_frame_active = 1;
      spi_bit_idx       = 15;
      spi_rise_count    = 0;
      spi_fall_count    = 0;
      spi_cmd_capture   = '0;
      spi_cs_fall_time  = $realtime;
      spi_have_sclk_edge = 0;

      if (spi_have_last_cs_rise) begin
        sb_check(($realtime - spi_last_cs_rise_time) >= 349.999,
          $sformatf("CS high gap below 350ns: %.3fns",
                    $realtime - spi_last_cs_rise_time));
      end

      if (spi_frame_idx < spi_plan_len)
        spi_rsp_current = spi_rsp_plan[spi_frame_idx];
      else
        spi_rsp_current = 16'h0000;

      spi_miso = spi_rsp_current[15];
    end
  end

  // Capture command bits on rising SCLK.
  always @(posedge spi_sclk) begin
    if (!rst) begin
      sb_check(spi_n_cs === 1'b0, "SCLK rose while CS was high");
      sb_check(spi_frame_active, "SCLK rose without active SPI frame");
      if (spi_frame_active) begin
        if (!spi_have_sclk_edge) begin
          sb_check(($realtime - spi_cs_fall_time) >= 349.999,
            $sformatf("CS setup below 350ns: %.3fns",
                      $realtime - spi_cs_fall_time));
        end else begin
          sb_check((($realtime - spi_last_sclk_edge_time) > 99.999) &&
                   (($realtime - spi_last_sclk_edge_time) < 100.002),
            $sformatf("SCLK half-period not 100ns: %.3fns",
                      $realtime - spi_last_sclk_edge_time));
        end
        spi_have_sclk_edge      = 1;
        spi_last_sclk_edge_time = $realtime;

        sb_check((spi_bit_idx >= 0) && (spi_bit_idx <= 15), "SPI bit index out of range");
        if ((spi_bit_idx >= 0) && (spi_bit_idx <= 15))
          spi_cmd_capture[spi_bit_idx] = spi_mosi;
        spi_rise_count++;
      end
    end
  end

  // Master samples MISO on falling SCLK.  Update to the next bit 1 ps later.
  always @(negedge spi_sclk) begin
    if (!rst) begin
      sb_check(spi_n_cs === 1'b0, "SCLK fell while CS was high");
      sb_check(spi_frame_active, "SCLK fell without active SPI frame");
      if (spi_frame_active) begin
        sb_check((($realtime - spi_last_sclk_edge_time) > 99.999) &&
                 (($realtime - spi_last_sclk_edge_time) < 100.002),
          $sformatf("SCLK half-period not 100ns: %.3fns",
                    $realtime - spi_last_sclk_edge_time));
        spi_last_sclk_edge_time = $realtime;
        spi_last_fall_time      = $realtime;
        spi_fall_count++;

        if (spi_bit_idx > 0) begin
          spi_bit_idx--;
          #1ps spi_miso = spi_rsp_current[spi_bit_idx];
        end
      end
    end
  end

  // CS deasserted: a normal frame must have exactly 16 rising and falling edges.
  always @(posedge spi_n_cs) begin
    if (spi_frame_active) begin
      if (rst) begin
        // Synchronous reset can abort a frame. Do not consume the plan entry.
        cov_spi_reset_abort++;
      end else begin
        sb_check(spi_rise_count == 16,
          $sformatf("SPI frame had %0d rising edges instead of 16", spi_rise_count));
        sb_check(spi_fall_count == 16,
          $sformatf("SPI frame had %0d falling edges instead of 16", spi_fall_count));
        sb_check(($realtime - spi_last_fall_time) >= 99.999,
          $sformatf("CS hold after last falling edge below 100ns: %.3fns",
                    $realtime - spi_last_fall_time));

        if (spi_frame_idx < spi_plan_len) begin
          sb_check(spi_cmd_capture === spi_cmd_plan[spi_frame_idx],
            $sformatf("SPI MOSI command mismatch frame=%0d exp=%h got=%h",
                      spi_frame_idx, spi_cmd_plan[spi_frame_idx], spi_cmd_capture));
        end
        spi_frame_idx++;
      end
      spi_frame_active = 0;
      spi_miso         = 1'b0;
    end

    spi_last_cs_rise_time = $realtime;
    spi_have_last_cs_rise = 1;
  end

  // ---------------------------------------------------------------------------
  // Stimulus helpers
  // ---------------------------------------------------------------------------
  task automatic clear_all_models_and_queues;
    tracker_ref_reset_direct();
    tracker_ref_reset_pipe();
    tr_q.delete();
    p_tr_q.delete();
    m_q.delete();
    p_m_q.delete();
    c_q.delete();
    p_c_q.delete();
    prev_spi_vld = 0;
    prev_c_vld   = 0;
    prev_p_c_vld = 0;
  endtask

  task automatic apply_reset(input string reason);
    $display("[%0t] reset: %s", $time, reason);
    @(negedge clk);
    rst        = 1'b1;
    spi_start  = 1'b0;
    tr_i_vld   = 1'b0;
    m_i_vld    = 1'b0;
    c_i_vld    = 1'b0;
    spi_miso   = 1'b0;
    spi_plan_active = 0;
    clear_all_models_and_queues();

    repeat (3) @(posedge clk);
    #2ps;
    sb_check(spi_n_cs === 1'b1, "SPI CS not high after reset");
    sb_check(spi_sclk === 1'b0, "SPI SCLK not low after reset");
    sb_check(c_o_rdy  === 1'b1, "direct CORDIC not ready after reset");
    sb_check(p_cordic_rdy === 1'b1, "pipeline CORDIC not ready after reset");

    @(negedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);
  endtask

  task automatic wait_direct_tracker_empty;
    int timeout;
    timeout = 0;
    while ((tr_q.size() != 0 || tr_o_vld) && timeout < 1000) begin
      @(posedge clk);
      timeout++;
    end
    sb_check(timeout < 1000, "timeout draining direct tracker scoreboard");
  endtask

  task automatic wait_direct_mech_empty;
    int timeout;
    timeout = 0;
    while ((m_q.size() != 0 || m_o_vld) && timeout < 1000) begin
      @(posedge clk);
      timeout++;
    end
    sb_check(timeout < 1000, "timeout draining direct mech scoreboard");
  endtask

  task automatic wait_direct_cordic_empty;
    int timeout;
    timeout = 0;
    while ((c_q.size() != 0 || c_o_vld || !c_o_rdy) && timeout < 10000) begin
      @(posedge clk);
      timeout++;
    end
    sb_check(timeout < 10000, "timeout draining direct CORDIC scoreboard");
  endtask

  task automatic wait_pipeline_empty;
    int timeout;
    timeout = 0;
    while ((p_tr_q.size() != 0 || p_m_q.size() != 0 || p_c_q.size() != 0 ||
            p_tr_vld || p_mech_vld || p_cordic_vld || !p_cordic_rdy) &&
           timeout < 10000) begin
      @(posedge clk);
      timeout++;
    end
    sb_check(timeout < 10000, "timeout draining end-to-end pipeline scoreboards");
  endtask

  task automatic tracker_sample(input int unsigned raw);
    @(negedge clk);
    tr_i_raw = raw[13:0];
    tr_i_vld = 1'b1;
    @(negedge clk);
    tr_i_vld = 1'b0;
  endtask

  task automatic tracker_idle_cycles(input int n);
    @(negedge clk);
    tr_i_vld = 1'b0;
    repeat (n) @(negedge clk);
  endtask

  task automatic mech_sample(
    input int unsigned raw,
    input int unsigned offset
  );
    @(negedge clk);
    m_i_pos    = raw[13:0];
    m_i_offset = offset[13:0];
    m_i_vld    = 1'b1;
    @(negedge clk);
    m_i_vld    = 1'b0;
  endtask

  task automatic cordic_send(input int signed angle);
    // Drive only when ready.  Sampling at negedge gives a clean half-cycle setup.
    @(negedge clk);
    while (!c_o_rdy)
      @(negedge clk);
    c_i_angle = angle[15:0];
    c_i_vld   = 1'b1;
    @(negedge clk);
    c_i_vld   = 1'b0;
  endtask

  task automatic execute_spi_plan(
    input string       name,
    input logic [13:0] expected_angle,
    input bit          inject_busy_start
  );
    bit got_vld;
    int timeout;
    int unsigned vld_count_before;
    begin
      $display("[%0t]   SPI case: %s", $time, name);
      sb_check(spi_plan_len > 0, "execute_spi_plan called with empty plan");
      sb_check(spi_n_cs === 1'b1, "SPI must be idle/high before starting plan");
      spi_frame_idx  = 0;
      spi_plan_active = 1;
      vld_count_before = spi_total_vld_count;

      @(negedge clk);
      spi_start = 1'b1;
      @(negedge clk);
      spi_start = 1'b0;

      if (inject_busy_start) begin
        fork
          begin
            wait (spi_n_cs === 1'b0);
            repeat (3) @(posedge spi_sclk);
            @(negedge clk);
            spi_start = 1'b1;
            @(negedge clk);
            spi_start = 1'b0;
            cov_spi_busy_start++;
          end
        join_none
      end

      got_vld = 0;
      timeout = 0;
      while (!got_vld && timeout < 5000) begin
        @(posedge clk);
        #2ps;
        if (spi_vld)
          got_vld = 1;
        timeout++;
      end

      sb_check(got_vld, $sformatf("SPI case '%s' timed out waiting for o_vld", name));
      if (got_vld)
        sb_check(spi_raw_angle === expected_angle,
          $sformatf("SPI case '%s' angle mismatch exp=%0d got=%0d",
                    name, expected_angle, spi_raw_angle));
      sb_check(spi_frame_idx == spi_plan_len,
        $sformatf("SPI case '%s' frame count mismatch exp=%0d got=%0d",
                  name, spi_plan_len, spi_frame_idx));
      sb_check(spi_total_vld_count == (vld_count_before + 1),
        $sformatf("SPI case '%s' expected exactly one o_vld pulse; before=%0d after=%0d",
                  name, vld_count_before, spi_total_vld_count));

      // Allow any injected-start helper to finish before the next plan is loaded.
      repeat (2) @(posedge clk);
      spi_plan_active = 0;
    end
  endtask

  // Convenience SPI plans matching the master's retry/clear behavior.
  task automatic spi_case_normal(
    input string name,
    input logic [13:0] angle,
    input bit inject_busy_start
  );
    spi_plan_clear();
    spi_plan_add(READ_ANGLE_CMD, 16'hA55A); // command-phase response is ignored
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(angle, 0, 0));
    cov_spi_normal++;
    execute_spi_plan(name, angle, inject_busy_start);
  endtask

  task automatic spi_case_parity_retry(
    input string name,
    input logic [13:0] bad_angle,
    input logic [13:0] good_angle
  );
    spi_plan_clear();
    spi_plan_add(READ_ANGLE_CMD, 16'hDEAD);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(bad_angle, 0, 1));
    spi_plan_add(READ_ANGLE_CMD, 16'h1234); // retry command phase
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(good_angle, 0, 0));
    cov_spi_parity_retry++;
    execute_spi_plan(name, good_angle, 0);
  endtask

  task automatic spi_case_error_clear(
    input string name,
    input logic [13:0] error_angle,
    input logic [13:0] good_angle
  );
    spi_plan_clear();
    spi_plan_add(READ_ANGLE_CMD, 16'hBEEF);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(error_angle, 1, 0));
    spi_plan_add(CLEAR_ERR_CMD,  16'hCAFE); // clear response is ignored
    spi_plan_add(READ_ANGLE_CMD, 16'h0F0F);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(good_angle, 0, 0));
    cov_spi_error_clear++;
    execute_spi_plan(name, good_angle, 0);
  endtask

  task automatic spi_case_parity_over_error(
    input string name,
    input logic [13:0] bad_angle,
    input logic [13:0] good_angle
  );
    spi_plan_clear();
    spi_plan_add(READ_ANGLE_CMD, 16'h3333);
    // Both error flag and bad parity: RTL checks parity first, so no clear command.
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(bad_angle, 1, 1));
    spi_plan_add(READ_ANGLE_CMD, 16'h7777);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(good_angle, 0, 0));
    cov_spi_parity_over_error++;
    execute_spi_plan(name, good_angle, 0);
  endtask

  task automatic spi_case_repeated_error_clear(
    input string name,
    input logic [13:0] good_angle
  );
    spi_plan_clear();
    spi_plan_add(READ_ANGLE_CMD, 16'h1111);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(14'h010, 1, 0));
    spi_plan_add(CLEAR_ERR_CMD,  16'h2222);
    spi_plan_add(READ_ANGLE_CMD, 16'h3333);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(14'h020, 1, 0));
    spi_plan_add(CLEAR_ERR_CMD,  16'h4444);
    spi_plan_add(READ_ANGLE_CMD, 16'h5555);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(good_angle, 0, 0));
    cov_spi_error_clear++;
    execute_spi_plan(name, good_angle, 0);
  endtask

  // ---------------------------------------------------------------------------
  // Test sequence
  // ---------------------------------------------------------------------------
  initial begin : TEST_SEQUENCE
    int unsigned seed;
    int signed angle;
    int signed logical_pos;
    int signed step;
    int unsigned raw;
    int unsigned off;
    int unsigned rand_word;
    int random_count;
    bit exhaustive;

    // Deterministic defaults
    spi_start       = 0;
    spi_miso        = 0;
    pipe_offset     = 0;
    tr_i_vld        = 0;
    tr_i_raw        = 0;
    m_i_vld         = 0;
    m_i_pos         = 0;
    m_i_offset      = 0;
    c_i_vld         = 0;
    c_i_angle       = 0;
    spi_plan_active = 0;
    spi_frame_active = 0;
    spi_plan_len    = 0;
    spi_frame_idx   = 0;
    spi_have_last_cs_rise = 0;
    seed = 32'h51A7_C0DE;
    exhaustive = $test$plusargs("EXHAUSTIVE");
    allow_known_cordic_reset_glitch = $test$plusargs("ALLOW_KNOWN_CORDIC_RESET_GLITCH");

    cov_tracker_fwd_wrap = 0;
    cov_tracker_rev_wrap = 0;
    cov_tracker_pos_half_exact = 0;
    cov_tracker_neg_half_exact = 0;
    cov_mech_mod_wrap = 0;
    cov_spi_normal = 0;
    cov_spi_parity_retry = 0;
    cov_spi_error_clear = 0;
    cov_spi_parity_over_error = 0;
    cov_spi_busy_start = 0;
    cov_spi_reset_abort = 0;
    cov_cordic_axis_or_edge = 0;
    foreach (cov_cordic_quad[i]) cov_cordic_quad[i] = 0;

    clear_all_models_and_queues();

    // -----------------------------------------------------------------------
    start_test("Synchronous reset / idle contracts");
    apply_reset("initial power-on");

    // -----------------------------------------------------------------------
    start_test("SPI nominal reads, endpoint angles, and start-while-busy ignore");
    pipe_offset = 14'd0;
    spi_case_normal("angle zero",       14'd0,     0);
    spi_case_normal("angle max",        14'h3FFF,  1);
    spi_case_normal("midscale",         14'd8192,  0);
    wait_pipeline_empty();

    // -----------------------------------------------------------------------
    start_test("SPI parity retry, sensor error clear, parity-vs-error priority");
    pipe_offset = 14'd137;
    spi_case_parity_retry("bad parity then recover", 14'd1234, 14'd4321);
    spi_case_error_clear("error flag -> clear -> recover", 14'd77, 14'd9000);
    spi_case_parity_over_error("bad parity dominates error flag", 14'd321, 14'd7654);
    spi_case_repeated_error_clear("two error-clear cycles before good data", 14'd2222);
    wait_pipeline_empty();

    // -----------------------------------------------------------------------
    start_test("SPI reset abort in the middle of a frame, then clean recovery");
    spi_plan_clear();
    spi_plan_add(READ_ANGLE_CMD, 16'hA5A5);
    spi_plan_add(READ_ANGLE_CMD, as5048_frame(14'd2468, 0, 0));
    spi_plan_active = 1;
    @(negedge clk);
    spi_start = 1;
    @(negedge clk);
    spi_start = 0;
    wait (spi_n_cs === 1'b0);
    repeat (5) @(posedge spi_sclk);
    @(negedge clk);
    rst = 1;
    clear_all_models_and_queues();
    repeat (3) @(posedge clk);
    #2ps;
    sb_check(spi_n_cs === 1'b1, "reset did not immediately restore CS high on clock edge");
    sb_check(spi_sclk === 1'b0, "reset did not restore SCLK low");
    @(negedge clk);
    rst = 0;
    spi_plan_active = 0;
    repeat (2) @(posedge clk);
    spi_case_normal("post-abort recovery", 14'd2468, 0);
    wait_pipeline_empty();

    // -----------------------------------------------------------------------
    start_test("Tracker initialization: first sample seeds position, velocity zero");
    apply_reset("tracker initialization");
    tracker_sample(14'd12345);
    wait_direct_tracker_empty();

    // -----------------------------------------------------------------------
    start_test("Tracker forward wrap and reverse wrap");
    apply_reset("forward wrap scenario");
    tracker_sample(14'd16380);
    tracker_sample(14'd2);       // forward crossing -> unwrapped 16386
    tracker_sample(14'd10);
    wait_direct_tracker_empty();

    apply_reset("reverse wrap scenario");
    tracker_sample(14'd3);
    tracker_sample(14'd16380);   // reverse crossing -> unwrapped -4
    tracker_sample(14'd16370);
    wait_direct_tracker_empty();

    // -----------------------------------------------------------------------
    start_test("Tracker exact +/- half-turn ambiguity boundaries");
    apply_reset("+8192 exact threshold");
    tracker_sample(14'd0);
    tracker_sample(14'd8192);    // >= +8192 decrements wrap count
    wait_direct_tracker_empty();

    apply_reset("-8192 exact threshold");
    tracker_sample(14'd8192);
    tracker_sample(14'd0);       // <= -8192 increments wrap count
    wait_direct_tracker_empty();

    apply_reset("just inside threshold");
    tracker_sample(14'd0);
    tracker_sample(14'd8191);    // must not wrap
    wait_direct_tracker_empty();

    // -----------------------------------------------------------------------
    start_test("Tracker sparse valid, consecutive throughput, velocity sign/filter arithmetic");
    apply_reset("tracker filter stress");
    tracker_sample(14'd5000);
    tracker_idle_cycles(17);     // gaps must not create outputs or alter state

    // 120 consecutive +100-count samples. This stresses one-sample-per-clock
    // throughput and the exact 29-bit Q8.20 arithmetic.
    @(negedge clk);
    tr_i_vld = 1;
    logical_pos = 5000;
    for (int i = 0; i < 120; i++) begin
      logical_pos += 100;
      tr_i_raw = logical_pos & RAW_MASK;
      @(negedge clk);
    end
    tr_i_vld = 0;
    wait_direct_tracker_empty();

    // Then reverse direction.
    @(negedge clk);
    tr_i_vld = 1;
    for (int i = 0; i < 80; i++) begin
      logical_pos -= 75;
      tr_i_raw = logical_pos & RAW_MASK;
      @(negedge clk);
    end
    tr_i_vld = 0;
    wait_direct_tracker_empty();

    // Q8.20 representability boundary.  +255 counts/sample is representable;
    // +256 places a 1 in bit 28 after <<20 and therefore aliases negative in
    // the current 29-bit implementation.  The exact scoreboard checks RTL
    // behavior; the warning highlights the design-intent risk without assuming
    // that the architecture was supposed to saturate.
    apply_reset("tracker Q8.20 +255 boundary");
    tracker_sample(14'd1000);
    tracker_sample(14'd1255);
    wait_direct_tracker_empty();
    sb_check($signed(tr_o_vel) > 0, "+255 tracker step should produce positive filtered velocity");

    apply_reset("tracker Q8.20 -255 boundary");
    tracker_sample(14'd1000);
    tracker_sample(14'd745);
    wait_direct_tracker_empty();
    sb_check($signed(tr_o_vel) < 0, "-255 tracker step should produce negative filtered velocity");

    apply_reset("tracker Q8.20 +256 alias boundary");
    tracker_sample(14'd1000);
    tracker_sample(14'd1256);
    wait_direct_tracker_empty();
    sb_warn($signed(tr_o_vel) < 0,
      "+256-count positive step aliases to a negative Q8.20 instantaneous term; verify this is acceptable for the maximum encoder sample rate");

    // -----------------------------------------------------------------------
    start_test("Tracker randomized bounded walk across many wraps");
    apply_reset("random tracker walk");
    logical_pos = 1000;
    @(negedge clk);
    tr_i_vld = 1;
    for (int i = 0; i < 1000; i++) begin
      step = int'(prng_next(seed) % 401) - 200; // unambiguous |step| < 8192
      logical_pos += step;
      tr_i_raw = logical_pos & RAW_MASK;
      @(negedge clk);
    end
    tr_i_vld = 0;
    wait_direct_tracker_empty();

    // -----------------------------------------------------------------------
    start_test("mech_to_elec cardinal mappings, modulo overflow, and throughput");
    apply_reset("mech_to_elec directed vectors");
    mech_sample(14'd0,     14'd0);       // elec 0 -> CORDIC -32768
    mech_sample(14'd0,     14'd4096);    // -90 deg CORDIC code
    mech_sample(14'd0,     14'd8192);    // 0 deg CORDIC code
    mech_sample(14'd0,     14'd12288);   // +90 deg CORDIC code
    mech_sample(14'h3FFF,  14'h3FFF);    // strong modulo-overflow case
    mech_sample(14'd2341,  14'd1);       // arbitrary multiply+offset
    wait_direct_mech_empty();

    // Full-rate randomized pipeline through the two registered stages.
    @(negedge clk);
    m_i_vld = 1;
    for (int i = 0; i < 2000; i++) begin
      m_i_pos    = prng_next(seed) & RAW_MASK;
      m_i_offset = prng_next(seed) & RAW_MASK;
      @(negedge clk);
    end
    m_i_vld = 0;
    wait_direct_mech_empty();

    if (exhaustive) begin
      $display("[%0t]   +EXHAUSTIVE: sweeping all 16384 mechanical raw inputs", $time);
      @(negedge clk);
      m_i_vld = 1;
      m_i_offset = 14'd0;
      for (int i = 0; i < RAW_MOD; i++) begin
        m_i_pos = i[13:0];
        @(negedge clk);
      end
      m_i_vld = 0;
      wait_direct_mech_empty();
    end

    // -----------------------------------------------------------------------
    start_test("CORDIC axes, quadrant boundaries, extreme signed codes");
    apply_reset("CORDIC directed vectors");
    // Around -180, -90, 0, +90, +180 and fold boundaries.
    cordic_send(-32768);
    cordic_send(-32767);
    cordic_send(-24576);
    cordic_send(-16385);
    cordic_send(-16384);
    cordic_send(-16383);
    cordic_send(-8192);
    cordic_send(-1);
    cordic_send(0);
    cordic_send(1);
    cordic_send(8192);
    cordic_send(16383);
    cordic_send(16384);
    cordic_send(16385);
    cordic_send(24576);
    cordic_send(32766);
    cordic_send(32767);
    wait_direct_cordic_empty();

    // -----------------------------------------------------------------------
    start_test("CORDIC ignores i_vld while busy, then accepts when ready");
    cordic_send(16'sd4096);
    // Immediately pulse a different value while busy; it must not produce a
    // second result because the handshake is not accepted.
    @(negedge clk);
    sb_check(!c_o_rdy, "CORDIC unexpectedly ready immediately after accepting input");
    c_i_angle = -16'sd12000;
    c_i_vld   = 1;
    @(negedge clk);
    c_i_vld   = 0;
    wait_direct_cordic_empty();
    // Now send that value legally and require its result.
    cordic_send(-16'sd12000);
    wait_direct_cordic_empty();

    // -----------------------------------------------------------------------
    start_test("CORDIC reset exactly while FSM is in ST_DONE");
    apply_reset("prepare CORDIC ST_DONE reset test");
    cordic_send(16'sd7000);
    // cordic_send returns on the negedge immediately after acceptance.
    // Twenty following rising edges execute iterations 0..19; after the 20th
    // edge the FSM is in ST_DONE. Assert reset before the next rising edge.
    repeat (20) @(posedge clk);
    @(negedge clk);
    rst = 1'b1;
    c_i_vld = 1'b0;
    c_q.delete(); // this transaction is intentionally aborted by reset
    repeat (2) @(posedge clk);
    @(negedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);
    // With the supplied RTL this strict test exposes that datapath has no
    // i_rst guard: ST_DONE can still drive o_vld='1' on the reset edge.
    // Use +ALLOW_KNOWN_CORDIC_RESET_GLITCH only while tracking that bug.

    // -----------------------------------------------------------------------
    start_test("CORDIC randomized full-range numerical and bit-exact regression");
    random_count = exhaustive ? 0 : 512;
    // Stratify the random codes across the four signed-angle quadrants.
    // The normal regression therefore has exactly 128 random vectors in each
    // quadrant, while the lower 14 bits remain pseudo-random.
    for (int i = 0; i < random_count; i++) begin
      rand_word = prng_next(seed);
      angle = wrap_signed(((i & 3) << 14) | (rand_word & 14'h3FFF), 16);
      cordic_send(angle);
    end
    wait_direct_cordic_empty();

    if (exhaustive) begin
      $display("[%0t]   +EXHAUSTIVE: sweeping all 65536 CORDIC input codes", $time);
      for (int i = -32768; i <= 32767; i++)
        cordic_send(i);
      wait_direct_cordic_empty();
    end

    // -----------------------------------------------------------------------
    start_test("End-to-end SPI -> tracker + mech_to_elec -> CORDIC wrap sequence");
    apply_reset("end-to-end directed sequence");
    pipe_offset = 14'd3072;
    spi_case_normal("E2E seed near top",       14'd16370, 0);
    spi_case_normal("E2E forward wrap",        14'd4,     0);
    spi_case_normal("E2E continue forward",    14'd120,   0);
    spi_case_normal("E2E reverse wrap",        14'd16360, 0);
    spi_case_parity_retry("E2E retry does not leak bad sample", 14'd999, 14'd2500);
    spi_case_error_clear("E2E error sample does not leak",      14'd333, 14'd7000);
    wait_pipeline_empty();

    // -----------------------------------------------------------------------
    start_test("End-to-end randomized valid samples with changing offsets");
    for (int i = 0; i < 24; i++) begin
      raw = prng_next(seed) & RAW_MASK;
      off = prng_next(seed) & RAW_MASK;
      pipe_offset = off[13:0];
      spi_case_normal($sformatf("random E2E %0d", i), raw[13:0], 0);
    end
    wait_pipeline_empty();

    // -----------------------------------------------------------------------
    // Coverage / final self-report
    // -----------------------------------------------------------------------
    sb_check(cov_spi_normal > 0, "coverage miss: nominal SPI");
    sb_check(cov_spi_parity_retry > 0, "coverage miss: SPI parity retry");
    sb_check(cov_spi_error_clear > 0, "coverage miss: SPI error clear");
    sb_check(cov_spi_parity_over_error > 0, "coverage miss: parity-over-error priority");
    sb_check(cov_spi_busy_start > 0, "coverage miss: start while SPI busy");
    sb_check(cov_spi_reset_abort > 0, "coverage miss: SPI mid-frame reset abort");
    sb_check(cov_tracker_fwd_wrap > 0, "coverage miss: tracker forward wrap");
    sb_check(cov_tracker_rev_wrap > 0, "coverage miss: tracker reverse wrap");
    sb_check(cov_tracker_pos_half_exact > 0, "coverage miss: tracker +8192 boundary");
    sb_check(cov_tracker_neg_half_exact > 0, "coverage miss: tracker -8192 boundary");
    sb_check(cov_mech_mod_wrap > 0, "coverage miss: mech modulo overflow");
    foreach (cov_cordic_quad[i]) begin
      sb_check(cov_cordic_quad[i] > 0,
        $sformatf("coverage miss: CORDIC quadrant bin %0d", i));
      if (!exhaustive)
        sb_check(cov_cordic_quad[i] >= 128,
          $sformatf("random CORDIC coverage too weak in quadrant %0d: only %0d hits",
                    i, cov_cordic_quad[i]));
    end
    sb_check(cov_cordic_axis_or_edge >= 7, "coverage miss: CORDIC axis/edge set");

    $display("\n============================================================");
    $display(" SENSOR / CONTROL FRONT-END TESTBENCH SUMMARY");
    $display("============================================================");
    $display(" Tests executed : %0d", n_tests);
    $display(" Checks executed: %0d", n_checks);
    $display(" Errors         : %0d", n_errors);
    $display(" Warnings       : %0d", n_warnings);
    $display(" SPI coverage   : normal=%0d parity_retry=%0d err_clear=%0d parity>err=%0d busy_start=%0d reset_abort=%0d",
             cov_spi_normal, cov_spi_parity_retry, cov_spi_error_clear,
             cov_spi_parity_over_error, cov_spi_busy_start, cov_spi_reset_abort);
    $display(" Tracker cover  : fwd_wrap=%0d rev_wrap=%0d +8192=%0d -8192=%0d",
             cov_tracker_fwd_wrap, cov_tracker_rev_wrap,
             cov_tracker_pos_half_exact, cov_tracker_neg_half_exact);
    $display(" Mech cover     : modulo_wrap=%0d", cov_mech_mod_wrap);
    $display(" CORDIC cover   : Q0=%0d Q1=%0d Q2=%0d Q3=%0d axis/edge=%0d",
             cov_cordic_quad[0], cov_cordic_quad[1],
             cov_cordic_quad[2], cov_cordic_quad[3], cov_cordic_axis_or_edge);
    $display(" Exhaustive mode: %s", exhaustive ? "YES" : "NO");
    $display(" Allow known CORDIC reset glitch: %s", allow_known_cordic_reset_glitch ? "YES" : "NO");
    $display("============================================================");

    if (n_errors == 0) begin
      $display("RESULT: PASS");
      $finish;
    end else begin
      $display("RESULT: FAIL");
      $fatal(1, "Front-end regression failed with %0d errors", n_errors);
    end
  end

  // Global deadlock guard.  +EXHAUSTIVE CORDIC sweep is ~30 ms at 50 MHz.
  initial begin : WATCHDOG
    #100ms;
    $fatal(1, "Global watchdog expired; likely deadlock or missing valid/ready transition");
  end

endmodule
