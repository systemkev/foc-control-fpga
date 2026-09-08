`timescale 1ns/1ps

package top_foc_tb_shared_pkg;
    integer xadc_raw_a = 16'h8000;
    integer xadc_raw_b = 16'h8000;
    integer xadc_eoc_delay = 2;
    integer xadc_drdy_delay = 2;
    bit xadc_enable = 1'b1;
    bit xadc_junk_once = 1'b0;
endpackage

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
    import top_foc_tb_shared_pkg::*;

    bit phase_b;
    bit eoc_pending;
    bit drdy_pending;
    bit wait_den;
    bit advance_phase;
    integer eoc_count;
    integer drdy_count;
    logic [6:0] selected_addr;
    logic [6:0] read_addr;

    assign busy_out = eoc_pending | drdy_pending | wait_den;
    assign eos_out = 1'b0;
    assign alarm_out = 1'b0;

    always @(posedge dclk_in) begin
        if (reset_in) begin
            do_out <= 16'h0000;
            drdy_out <= 1'b0;
            channel_out <= 5'h14;
            eoc_out <= 1'b0;
            phase_b <= 1'b0;
            eoc_pending <= 1'b0;
            drdy_pending <= 1'b0;
            wait_den <= 1'b0;
            advance_phase <= 1'b0;
            eoc_count <= 0;
            drdy_count <= 0;
            selected_addr <= 7'h14;
            read_addr <= 7'h14;
        end else begin
            eoc_out <= 1'b0;
            drdy_out <= 1'b0;

            if (convst_in && !eoc_pending && !drdy_pending && !wait_den) begin
                if (xadc_junk_once) begin
                    selected_addr <= 7'h15;
                    advance_phase <= 1'b0;
                    xadc_junk_once = 1'b0;
                end else begin
                    selected_addr <= phase_b ? 7'h16 : 7'h14;
                    advance_phase <= 1'b1;
                end
                eoc_pending <= 1'b1;
                eoc_count <= xadc_eoc_delay;
            end

            if (eoc_pending) begin
                if (eoc_count <= 0) begin
                    channel_out <= selected_addr[4:0];
                    eoc_out <= 1'b1;
                    eoc_pending <= 1'b0;
                    wait_den <= 1'b1;
                end else begin
                    eoc_count <= eoc_count - 1;
                end
            end

            if (wait_den && den_in) begin
                read_addr <= daddr_in;
                wait_den <= 1'b0;
                drdy_pending <= 1'b1;
                drdy_count <= xadc_drdy_delay;
            end

            if (drdy_pending) begin
                if (drdy_count <= 0) begin
                    if (xadc_enable) begin
                        if (read_addr == 7'h14)
                            do_out <= xadc_raw_a[15:0];
                        else if (read_addr == 7'h16)
                            do_out <= xadc_raw_b[15:0];
                        else
                            do_out <= 16'h55AA;
                        drdy_out <= 1'b1;
                        drdy_pending <= 1'b0;
                        if (advance_phase)
                            phase_b <= ~phase_b;
                    end
                end else begin
                    drdy_count <= drdy_count - 1;
                end
            end
        end
    end
endmodule

module top_foc_sequencer_tb;
    import top_foc_tb_shared_pkg::*;

    localparam integer CLK_NS = 20;
    localparam integer PWM_CLKS = 2500;
    localparam realtime UART_BIT_NS = 1085.069444;
    localparam realtime PI = 3.14159265358979323846;

    localparam byte A_FOC_KP = 8'h00;
    localparam byte A_FOC_KI = 8'h01;
    localparam byte A_FOC_WINDUP = 8'h02;
    localparam byte A_FOC_VMAX = 8'h03;
    localparam byte A_VEL_KP = 8'h04;
    localparam byte A_VEL_KI = 8'h05;
    localparam byte A_VEL_WINDUP = 8'h06;
    localparam byte A_VEL_IMAX = 8'h07;
    localparam byte A_POS_KP = 8'h08;
    localparam byte A_POS_VMAX = 8'h09;
    localparam byte A_POS_DEADBAND = 8'h0A;
    localparam byte A_CONTROL = 8'h0B;
    localparam byte A_TARGET_DQ = 8'h0C;
    localparam byte A_TARGET_VEL = 8'h0D;
    localparam byte A_TARGET_POS = 8'h0E;
    localparam byte A_MANUAL_ELEC = 8'h0F;
    localparam byte A_STATUS = 8'h10;
    localparam byte A_ACTUAL_POS = 8'h11;
    localparam byte A_ACTUAL_VEL = 8'h12;
    localparam byte A_PHASE_AB = 8'h13;
    localparam byte A_PHASE_C_ID = 8'h14;
    localparam byte A_IQ_ELEC = 8'h15;
    localparam byte A_CAL_OFF_AB = 8'h16;
    localparam byte A_CAL_OFF_C = 8'h17;

    localparam logic [31:0] CTRL_CURRENT_MANUAL = 32'h00000021;
    localparam logic [31:0] CTRL_VELOCITY_MANUAL = 32'h00000023;
    localparam logic [31:0] CTRL_POSITION_MANUAL = 32'h00000025;
    localparam logic [31:0] CTRL_DISABLED_MANUAL = 32'h00000027;

    logic i_clk;
    logic i_rst;
    logic i_uart_rx;
    logic o_uart_tx;
    logic i_enc_miso;
    logic o_enc_mosi;
    logic o_enc_n_cs;
    logic o_enc_sclk;
    logic i_vauxp4;
    logic i_vauxn4;
    logic i_vauxp6;
    logic i_vauxn6;
    logic o_pwm_a;
    logic o_pwm_b;
    logic o_pwm_c;
    logic o_motor_en;

    logic cp_vld;
    logic signed [15:0] cp_angle;
    logic cp_o_vld;
    logic signed [15:0] cp_cos;
    logic signed [15:0] cp_sin;

    integer checks;
    integer errors;
    integer expected_off_a;
    integer expected_off_b;

    integer enc_angle;
    integer enc_state;
    integer enc_clear_seen;
    bit enc_bad_once;
    bit enc_error_once;
    bit enc_hold_bad;
    logic [15:0] spi_response;
    logic [15:0] spi_tx_word;
    integer spi_bit_idx;
    integer spi_rise_count;
    bit spi_read_bad_latched;
    bit spi_read_error_latched;

    localparam integer ENC_CMD = 0;
    localparam integer ENC_READ = 1;
    localparam integer ENC_CLEAR = 2;

    mailbox #(byte unsigned) tx_mb = new();

    motor_controller_top dut (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_uart_rx(i_uart_rx),
        .o_uart_tx(o_uart_tx),
        .i_enc_miso(i_enc_miso),
        .o_enc_mosi(o_enc_mosi),
        .o_enc_n_cs(o_enc_n_cs),
        .o_enc_sclk(o_enc_sclk),
        .i_vauxp4(i_vauxp4),
        .i_vauxn4(i_vauxn4),
        .i_vauxp6(i_vauxp6),
        .i_vauxn6(i_vauxn6),
        .o_pwm_a(o_pwm_a),
        .o_pwm_b(o_pwm_b),
        .o_pwm_c(o_pwm_c),
        .o_motor_en(o_motor_en)
    );

    always #(CLK_NS/2) i_clk = ~i_clk;

    function automatic logic [7:0] crc8_56(input logic [55:0] data_in);
        logic [7:0] c;
        logic inv;
        integer i;
        begin
            c = 8'h00;
            for (i = 55; i >= 0; i = i - 1) begin
                inv = c[7] ^ data_in[i];
                c = {c[6:0], 1'b0};
                if (inv)
                    c = c ^ 8'h07;
            end
            crc8_56 = c;
        end
    endfunction

    function automatic logic [15:0] make_encoder_frame(input integer angle, input bit err, input bit bad);
        logic [15:0] f;
        begin
            f = 16'h0000;
            f[13:0] = angle[13:0];
            f[14] = err;
            f[15] = ^f[14:0];
            if (bad)
                f[15] = ~f[15];
            make_encoder_frame = f;
        end
    endfunction

    function automatic integer adc_scaled(input integer raw);
        longint signed p;
        begin
            p = longint'(raw) * 27034;
            adc_scaled = p >>> 16;
        end
    endfunction

    function automatic integer clamp16(input integer x);
        begin
            if (x > 32767)
                clamp16 = 32767;
            else if (x < -32768)
                clamp16 = -32768;
            else
                clamp16 = x;
        end
    endfunction

    function automatic integer abs_i(input integer x);
        begin
            abs_i = x < 0 ? -x : x;
        end
    endfunction

    function automatic integer raw_for_a(input integer curr_q12);
        longint signed num;
        integer scaled;
        integer r;
        begin
            scaled = expected_off_a + curr_q12;
            if (scaled < 0)
                scaled = 0;
            if (scaled > 27033)
                scaled = 27033;
            num = longint'(scaled) * 65536 + 27033;
            r = num / 27034;
            if (r < 0)
                r = 0;
            if (r > 65535)
                r = 65535;
            raw_for_a = r;
        end
    endfunction

    function automatic integer raw_for_b(input integer curr_q12);
        longint signed num;
        integer scaled;
        integer r;
        begin
            scaled = expected_off_b - curr_q12;
            if (scaled < 0)
                scaled = 0;
            if (scaled > 27033)
                scaled = 27033;
            num = longint'(scaled) * 65536 + 27033;
            r = num / 27034;
            if (r < 0)
                r = 0;
            if (r > 65535)
                r = 65535;
            raw_for_b = r;
        end
    endfunction

    function automatic integer signed16(input logic [15:0] v);
        begin
            signed16 = $signed(v);
        end
    endfunction

    function automatic integer round_real(input real x);
        begin
            if (x >= 0.0)
                round_real = $rtoi(x + 0.5);
            else
                round_real = $rtoi(x - 0.5);
        end
    endfunction

    task automatic check(input bit cond, input string msg);
        begin
            checks = checks + 1;
            if (!cond) begin
                errors = errors + 1;
                $error("CHECK_FAIL %s", msg);
            end
        end
    endtask

    task automatic wait_clks(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                @(posedge i_clk);
        end
    endtask

    task automatic wait_pwm_periods(input integer n);
        begin
            wait_clks(n * PWM_CLKS);
        end
    endtask

    task automatic uart_send_byte(input byte unsigned b);
        integer i;
        begin
            i_uart_rx = 1'b0;
            #(UART_BIT_NS);
            for (i = 0; i < 8; i = i + 1) begin
                i_uart_rx = b[i];
                #(UART_BIT_NS);
            end
            i_uart_rx = 1'b1;
            #(UART_BIT_NS);
        end
    endtask

    task automatic uart_send_packet(input byte unsigned cmd, input byte unsigned addr, input logic [31:0] data, input bit corrupt_crc);
        logic [55:0] d;
        byte unsigned c;
        begin
            d = {8'hAA, cmd, addr, data};
            c = crc8_56(d);
            if (corrupt_crc)
                c = c ^ 8'h5A;
            uart_send_byte(8'hAA);
            uart_send_byte(cmd);
            uart_send_byte(addr);
            uart_send_byte(data[31:24]);
            uart_send_byte(data[23:16]);
            uart_send_byte(data[15:8]);
            uart_send_byte(data[7:0]);
            uart_send_byte(c);
        end
    endtask

    task automatic get_tx_byte(output byte unsigned b);
        bit got;
        begin
            got = 1'b0;
            fork : gtb
                begin
                    tx_mb.get(b);
                    got = 1'b1;
                end
                begin
                    #(UART_BIT_NS * 20.0);
                end
            join_any
            disable gtb;
            if (!got)
                $fatal(1, "UART response timeout");
        end
    endtask

    task automatic drain_tx;
        byte unsigned b;
        begin
            while (tx_mb.try_get(b)) begin
            end
        end
    endtask

    task automatic write_reg(input byte unsigned addr, input logic [31:0] data);
        begin
            uart_send_packet(8'h02, addr, data, 1'b0);
            wait_clks(20);
        end
    endtask

    task automatic read_reg(input byte unsigned addr, output logic [31:0] data);
        byte unsigned f[0:7];
        logic [55:0] d;
        integer i;
        begin
            drain_tx();
            uart_send_packet(8'h01, addr, 32'h00000000, 1'b0);
            for (i = 0; i < 8; i = i + 1)
                get_tx_byte(f[i]);
            check(f[0] == 8'hAA, $sformatf("UART response sync addr=%02x", addr));
            check(f[1] == 8'h00, $sformatf("UART response command addr=%02x", addr));
            check(f[2] == addr, $sformatf("UART response address addr=%02x got=%02x", addr, f[2]));
            data = {f[3], f[4], f[5], f[6]};
            d = {f[0], f[1], f[2], data};
            check(f[7] == crc8_56(d), $sformatf("UART response CRC addr=%02x", addr));
        end
    endtask

    task automatic set_currents(input integer ia_q12, input integer ib_q12);
        begin
            xadc_raw_a = raw_for_a(ia_q12);
            xadc_raw_b = raw_for_b(ib_q12);
        end
    endtask

    task automatic get_model_currents(output integer ia, output integer ib, output integer ic);
        begin
            ia = adc_scaled(xadc_raw_a) - expected_off_a;
            ib = expected_off_b - adc_scaled(xadc_raw_b);
            ic = clamp16(-ia - ib);
        end
    endtask

    task automatic wait_motor(input bit state, input integer max_clks);
        integer i;
        begin
            for (i = 0; i < max_clks; i = i + 1) begin
                @(posedge i_clk);
                if (o_motor_en === state)
                    return;
            end
            $fatal(1, "motor enable timeout expected=%0d got=%b", state, o_motor_en);
        end
    endtask

    task automatic wait_calibration_done;
        logic [31:0] s;
        integer i;
        begin
            for (i = 0; i < 40; i = i + 1) begin
                read_reg(A_STATUS, s);
                if (s[8] == 1'b0)
                    return;
                wait_clks(25000);
            end
            $fatal(1, "calibration timeout");
        end
    endtask

    task automatic measure_pwm(output integer a, output integer b, output integer c);
        realtime ta;
        realtime tb;
        realtime tc;
        bit finished;
        begin
            a = -1;
            b = -1;
            c = -1;
            finished = 1'b0;
            fork : mpwm_outer
                begin
                    fork
                        begin
                            @(posedge o_pwm_a);
                            ta = $realtime;
                            @(negedge o_pwm_a);
                            a = round_real(($realtime - ta) / real'(CLK_NS));
                        end
                        begin
                            @(posedge o_pwm_b);
                            tb = $realtime;
                            @(negedge o_pwm_b);
                            b = round_real(($realtime - tb) / real'(CLK_NS));
                        end
                        begin
                            @(posedge o_pwm_c);
                            tc = $realtime;
                            @(negedge o_pwm_c);
                            c = round_real(($realtime - tc) / real'(CLK_NS));
                        end
                    join
                    finished = 1'b1;
                end
                begin
                    #(200000.0);
                end
            join_any
            disable mpwm_outer;
            if (!finished)
                $fatal(1, "PWM measurement timeout A=%0d B=%0d C=%0d en=%b", a, b, c, o_motor_en);
        end
    endtask

    task automatic calc_pwm_ideal(input real vd, input real vq, input real theta, output integer a, output integer b, output integer c);
        real ca;
        real sa;
        real alpha;
        real beta;
        real va;
        real vb;
        real vc;
        real vmax;
        real vmin;
        real voff;
        real xa;
        real xb;
        real xc;
        begin
            ca = $cos(theta);
            sa = $sin(theta);
            alpha = vd * ca - vq * sa;
            beta = vd * sa + vq * ca;
            va = alpha;
            vb = -0.5 * alpha + 0.8660254037844386 * beta;
            vc = -0.5 * alpha - 0.8660254037844386 * beta;
            vmax = va;
            if (vb > vmax)
                vmax = vb;
            if (vc > vmax)
                vmax = vc;
            vmin = va;
            if (vb < vmin)
                vmin = vb;
            if (vc < vmin)
                vmin = vc;
            voff = -0.5 * (vmax + vmin);
            xa = (va + voff) * 2500.0 / 12.0 + 1250.0;
            xb = (vb + voff) * 2500.0 / 12.0 + 1250.0;
            xc = (vc + voff) * 2500.0 / 12.0 + 1250.0;
            a = $floor(xa);
            b = $floor(xb);
            c = $floor(xc);
            if (a < 0) a = 0;
            if (b < 0) b = 0;
            if (c < 0) c = 0;
            if (a > PWM_CLKS) a = PWM_CLKS;
            if (b > PWM_CLKS) b = PWM_CLKS;
            if (c > PWM_CLKS) c = PWM_CLKS;
        end
    endtask

    task automatic check_pwm_ideal(input real vd, input real vq, input real theta, input integer tol, input string tag);
        integer ma;
        integer mb;
        integer mc;
        integer ea;
        integer eb;
        integer ec;
        begin
            wait_pwm_periods(3);
            measure_pwm(ma, mb, mc);
            calc_pwm_ideal(vd, vq, theta, ea, eb, ec);
            check(abs_i(ma - ea) <= tol, $sformatf("%s PWM A measured=%0d expected=%0d", tag, ma, ea));
            check(abs_i(mb - eb) <= tol, $sformatf("%s PWM B measured=%0d expected=%0d", tag, mb, eb));
            check(abs_i(mc - ec) <= tol, $sformatf("%s PWM C measured=%0d expected=%0d", tag, mc, ec));
        end
    endtask

    task automatic check_transform(input integer manual_raw, input integer tol, input string tag);
        logic [31:0] c_id;
        logic [31:0] iq_el;
        integer ia;
        integer ib;
        integer ic;
        integer md;
        integer mq;
        integer ed;
        integer eq;
        real alpha;
        real beta;
        real theta;
        real d;
        real q;
        begin
            write_reg(A_MANUAL_ELEC, manual_raw[13:0]);
            wait_pwm_periods(2);
            read_reg(A_PHASE_C_ID, c_id);
            read_reg(A_IQ_ELEC, iq_el);
            get_model_currents(ia, ib, ic);
            alpha = (2.0 / 3.0) * (real'(ia) - 0.5 * real'(ib) - 0.5 * real'(ic));
            beta = (2.0 / 3.0) * (0.8660254037844386 * real'(ib) - 0.8660254037844386 * real'(ic));
            theta = (real'(manual_raw) - 8192.0) * 2.0 * PI / 16384.0;
            d = alpha * $cos(theta) + beta * $sin(theta);
            q = beta * $cos(theta) - alpha * $sin(theta);
            ed = round_real(d);
            eq = round_real(q);
            md = signed16(c_id[15:0]);
            mq = signed16(iq_el[31:16]);
            check(abs_i(md - ed) <= tol, $sformatf("%s d measured=%0d expected=%0d", tag, md, ed));
            check(abs_i(mq - eq) <= tol, $sformatf("%s q measured=%0d expected=%0d", tag, mq, eq));
        end
    endtask

    initial begin
        byte unsigned b;
        logic [31:0] r;
        logic [31:0] pattern;
        integer addr;
        integer ia;
        integer ib;
        integer ic;
        integer i;
        integer raw_angle_cases[0:12];
        integer rand_a;
        integer rand_b;
        integer old_clear_seen;
        integer high_seen;
        integer offa;
        integer offb;

        i_clk = 1'b0;
        i_rst = 1'b1;
        i_uart_rx = 1'b1;
        i_enc_miso = 1'b0;
        i_vauxp4 = 1'b0;
        i_vauxn4 = 1'b0;
        i_vauxp6 = 1'b0;
        i_vauxn6 = 1'b0;
        cp_vld = 1'b0;
        cp_angle = '0;
        checks = 0;
        errors = 0;
        expected_off_a = 13517;
        expected_off_b = 13517;
        enc_angle = 1000;
        enc_state = ENC_CMD;
        enc_clear_seen = 0;
        enc_bad_once = 1'b0;
        enc_error_once = 1'b0;
        enc_hold_bad = 1'b0;
        spi_response = 16'h0000;
        spi_tx_word = 16'h0000;
        spi_bit_idx = 15;
        spi_rise_count = 0;
        spi_read_bad_latched = 1'b0;
        spi_read_error_latched = 1'b0;
        xadc_raw_a = 16'h8000;
        xadc_raw_b = 16'h8000;
        xadc_eoc_delay = 2;
        xadc_drdy_delay = 2;
        xadc_enable = 1'b1;
        xadc_junk_once = 1'b0;

        wait_clks(20);
        i_rst = 1'b0;
        wait_clks(20);

        $display("TEST reset register defaults");
        for (addr = 0; addr <= 15; addr = addr + 1) begin
            read_reg(addr[7:0], r);
            check(r == 32'h00000000, $sformatf("reset register %02x value=%08x", addr, r));
        end

        $display("TEST parser CRC rejection and resynchronization");
        uart_send_packet(8'h02, A_FOC_KP, 32'hDEADBEEF, 1'b1);
        wait_clks(100);
        read_reg(A_FOC_KP, r);
        check(r == 32'h00000000, $sformatf("bad CRC rejected got=%08x", r));
        drain_tx();
        uart_send_packet(8'h55, A_FOC_KP, 32'h00000000, 1'b0);
        #(UART_BIT_NS * 14.0);
        check(tx_mb.num() == 0, "invalid command generated no response");
        uart_send_packet(8'h01, 8'h18, 32'h00000000, 1'b0);
        #(UART_BIT_NS * 14.0);
        check(tx_mb.num() == 0, "invalid address generated no response");
        read_reg(A_FOC_KP, r);
        check(r == 32'h00000000, "parser recovered after malformed frames");

        $display("TEST writable register round trips");
        for (addr = 0; addr <= 10; addr = addr + 1) begin
            pattern = 32'h13579BDF ^ (32'h01010101 * addr);
            write_reg(addr[7:0], pattern);
            read_reg(addr[7:0], r);
            check(r == pattern, $sformatf("register roundtrip %02x got=%08x exp=%08x", addr, r, pattern));
        end
        for (addr = 12; addr <= 15; addr = addr + 1) begin
            pattern = 32'h2468ACE0 ^ (32'h00112233 * addr);
            write_reg(addr[7:0], pattern);
            read_reg(addr[7:0], r);
            check(r == pattern, $sformatf("register roundtrip %02x got=%08x exp=%08x", addr, r, pattern));
        end
        read_reg(A_CAL_OFF_C, r);
        check(r == 32'h00000000, "cal offset C read-only zero");

        $display("TEST disabled PWM gating during startup calibration");
        write_reg(A_CONTROL, 32'h00000001);
        wait_clks(100);
        check(o_motor_en == 1'b0, "invalid startup conditions block motor enable");
        high_seen = 0;
        for (i = 0; i < 3000; i = i + 1) begin
            @(posedge i_clk);
            if (o_pwm_a || o_pwm_b || o_pwm_c)
                high_seen = 1;
        end
        check(high_seen == 0, "PWM outputs low while motor disabled");
        write_reg(A_CONTROL, 32'h00000000);

        $display("TEST automatic current calibration and sensor presence");
        wait_calibration_done();
        read_reg(A_CAL_OFF_AB, r);
        check(r[31:16] == 16'd13517, $sformatf("cal offset A got=%0d", r[31:16]));
        check(r[15:0] == 16'd13517, $sformatf("cal offset B got=%0d", r[15:0]));
        read_reg(A_STATUS, r);
        check(r[1] == 1'b1, "encoder presence latched");
        check(r[2] == 1'b1, "current presence latched");
        check(r[3] == 1'b0, "no startup sensor fault");
        check(r[8] == 1'b0, "calibration inactive after 256 samples");

        $display("TEST functional configuration");
        write_reg(A_FOC_KP, 32'h00000000);
        write_reg(A_FOC_KI, 32'h00000000);
        write_reg(A_FOC_WINDUP, 32'h00400000);
        write_reg(A_FOC_VMAX, 32'h00400000);
        write_reg(A_VEL_KP, 32'h00100000);
        write_reg(A_VEL_KI, 32'h00000000);
        write_reg(A_VEL_WINDUP, 32'h00800000);
        write_reg(A_VEL_IMAX, 32'h00800000);
        write_reg(A_POS_KP, 32'h01000000);
        write_reg(A_POS_VMAX, 32'h00010000);
        write_reg(A_POS_DEADBAND, 32'h00000000);
        write_reg(A_TARGET_DQ, 32'h00000000);
        write_reg(A_MANUAL_ELEC, 32'd8192);
        set_currents(0, 0);
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL);
        wait_motor(1'b1, 10000);
        read_reg(A_STATUS, r);
        check(r[0] == 1'b1, "status drive enabled");
        check(r[4] == 1'b1, "status manual electrical mode");
        check(r[7:6] == 2'b00, "status current mode");

        $display("TEST phase-current acquisition signs and sum");
        set_currents(4096, -2048);
        wait_pwm_periods(3);
        read_reg(A_PHASE_AB, r);
        get_model_currents(ia, ib, ic);
        check(abs_i(signed16(r[31:16]) - ia) <= 1, $sformatf("phase A telemetry got=%0d exp=%0d", signed16(r[31:16]), ia));
        check(abs_i(signed16(r[15:0]) - ib) <= 1, $sformatf("phase B telemetry got=%0d exp=%0d", signed16(r[15:0]), ib));
        read_reg(A_PHASE_C_ID, r);
        check(abs_i(signed16(r[31:16]) - ic) <= 1, $sformatf("phase C telemetry got=%0d exp=%0d", signed16(r[31:16]), ic));

        $display("TEST CORDIC quadrants plus Clarke and Park canonical vectors");
        check_transform(8192, 28, "theta0");
        check_transform(12288, 28, "theta90");
        check_transform(4096, 28, "theta_minus90");
        check_transform(0, 28, "theta_minus180");

        raw_angle_cases[0] = 0;
        raw_angle_cases[1] = 1;
        raw_angle_cases[2] = 4095;
        raw_angle_cases[3] = 4096;
        raw_angle_cases[4] = 4097;
        raw_angle_cases[5] = 8191;
        raw_angle_cases[6] = 8192;
        raw_angle_cases[7] = 8193;
        raw_angle_cases[8] = 12287;
        raw_angle_cases[9] = 12288;
        raw_angle_cases[10] = 12289;
        raw_angle_cases[11] = 16382;
        raw_angle_cases[12] = 16383;
        for (i = 0; i < 13; i = i + 1)
            check_transform(raw_angle_cases[i], 36, $sformatf("boundary_raw_%0d", raw_angle_cases[i]));

        $display("TEST randomized transform pipeline");
        for (i = 0; i < 24; i = i + 1) begin
            rand_a = $urandom_range(9000, 0) - 4500;
            rand_b = $urandom_range(9000, 0) - 4500;
            if (rand_a + rand_b > 7000)
                rand_b = 7000 - rand_a;
            if (rand_a + rand_b < -7000)
                rand_b = -7000 - rand_a;
            set_currents(rand_a, rand_b);
            check_transform($urandom_range(16383, 0), 48, $sformatf("random_transform_%0d", i));
        end

        $display("TEST encoder electrical conversion and offset");
        set_currents(0, 0);
        enc_angle = 1234;
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL & ~32'h00000020);
        write_reg(A_MANUAL_ELEC, 32'd321);
        wait_pwm_periods(3);
        read_reg(A_IQ_ELEC, r);
        check(r[13:0] == ((1234 * 7 + 321) & 14'h3FFF), $sformatf("mech to elec got=%0d exp=%0d", r[13:0], ((1234 * 7 + 321) & 14'h3FFF)));

        $display("TEST encoder direction inversion");
        write_reg(A_CONTROL, 32'h00000041);
        wait_pwm_periods(3);
        read_reg(A_IQ_ELEC, r);
        check(r[13:0] == ((((16384 - 1234) & 14'h3FFF) * 7 + 321) & 14'h3FFF), $sformatf("sensor invert electrical got=%0d", r[13:0]));
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL);
        write_reg(A_MANUAL_ELEC, 32'd8192);

        $display("TEST current-loop proportional path through inverse transforms, SVPWM, and PWM");
        set_currents(0, 0);
        write_reg(A_FOC_KP, 32'h01000000);
        write_reg(A_FOC_KI, 32'h00000000);
        write_reg(A_FOC_VMAX, 32'h00400000);
        write_reg(A_VEL_IMAX, 32'h01000000);
        write_reg(A_TARGET_DQ, {16'sd4096,16'sd0});
        check_pwm_ideal(1.0, 0.0, 0.0, 9, "current_d_1A");
        write_reg(A_TARGET_DQ, {16'sd0,16'sd4096});
        check_pwm_ideal(0.0, 1.0, 0.0, 9, "current_q_1A");
        write_reg(A_TARGET_DQ, {(-16'sd4096),16'sd0});
        check_pwm_ideal(-1.0, 0.0, 0.0, 9, "current_d_minus1A");

        $display("TEST current target clamp");
        write_reg(A_VEL_IMAX, 32'h00200000);
        write_reg(A_TARGET_DQ, {16'sd8192,16'sd0});
        check_pwm_ideal(0.5, 0.0, 0.0, 10, "current_target_clamp");

        $display("TEST current-loop voltage saturation positive and negative");
        write_reg(A_VEL_IMAX, 32'h01000000);
        write_reg(A_FOC_KP, 32'h04000000);
        write_reg(A_FOC_VMAX, 32'h00100000);
        write_reg(A_TARGET_DQ, {16'sd4096,16'sd0});
        check_pwm_ideal(1.0, 0.0, 0.0, 10, "voltage_sat_pos");
        write_reg(A_TARGET_DQ, {(-16'sd4096),16'sd0});
        check_pwm_ideal(-1.0, 0.0, 0.0, 10, "voltage_sat_neg");

        $display("TEST integrator windup clamp and loop reset on disable");
        write_reg(A_FOC_KP, 32'h00000000);
        write_reg(A_FOC_KI, 32'h10000000);
        write_reg(A_FOC_WINDUP, 32'h00080000);
        write_reg(A_FOC_VMAX, 32'h00200000);
        write_reg(A_TARGET_DQ, {16'sd4096,16'sd0});
        check_pwm_ideal(0.5, 0.0, 0.0, 12, "integrator_windup");
        write_reg(A_TARGET_DQ, 32'h00000000);
        check_pwm_ideal(0.5, 0.0, 0.0, 12, "integrator_hold");
        write_reg(A_CONTROL, CTRL_DISABLED_MANUAL);
        wait_motor(1'b0, 1000);
        wait_clks(50);
        check(o_pwm_a == 1'b0 && o_pwm_b == 1'b0 && o_pwm_c == 1'b0, "disabled mode gates PWM");
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL);
        wait_motor(1'b1, 1000);
        check_pwm_ideal(0.0, 0.0, 0.0, 10, "integrator_reset");

        $display("TEST velocity mode and current saturation");
        write_reg(A_FOC_KP, 32'h01000000);
        write_reg(A_FOC_KI, 32'h00000000);
        write_reg(A_FOC_VMAX, 32'h00400000);
        write_reg(A_VEL_KP, 32'h00100000);
        write_reg(A_VEL_KI, 32'h00000000);
        write_reg(A_VEL_IMAX, 32'h00800000);
        write_reg(A_TARGET_VEL, 32'd262144);
        write_reg(A_CONTROL, CTRL_VELOCITY_MANUAL);
        check_pwm_ideal(0.0, 0.25, 0.0, 12, "velocity_0p25");
        write_reg(A_VEL_IMAX, 32'h00080000);
        write_reg(A_TARGET_VEL, 32'd1048576);
        check_pwm_ideal(0.0, 0.125, 0.0, 12, "velocity_current_clamp");

        $display("TEST position mode deadband, velocity clamp, and cascade");
        write_reg(A_VEL_IMAX, 32'h00800000);
        write_reg(A_POS_KP, 32'h01000000);
        write_reg(A_POS_VMAX, 32'h00004000);
        write_reg(A_POS_DEADBAND, 32'd2);
        write_reg(A_CONTROL, CTRL_POSITION_MANUAL);
        wait_pwm_periods(2);
        read_reg(A_ACTUAL_POS, r);
        write_reg(A_TARGET_POS, $signed(r) + 1);
        check_pwm_ideal(0.0, 0.0, 0.0, 12, "position_deadband");
        read_reg(A_ACTUAL_POS, r);
        write_reg(A_TARGET_POS, $signed(r) + 20);
        check_pwm_ideal(0.0, 0.25, 0.0, 14, "position_velocity_clamp");
        read_reg(A_ACTUAL_POS, r);
        write_reg(A_TARGET_POS, $signed(r) - 20);
        check_pwm_ideal(0.0, -0.25, 0.0, 14, "position_negative");

        $display("TEST tracker wrap continuity");
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL);
        write_reg(A_FOC_KP, 32'h00000000);
        write_reg(A_TARGET_DQ, 32'h00000000);
        enc_angle = 16380;
        wait_pwm_periods(3);
        read_reg(A_ACTUAL_POS, r);
        pattern = r;
        enc_angle = 4;
        wait_pwm_periods(3);
        read_reg(A_ACTUAL_POS, r);
        check($signed(r) > $signed(pattern), $sformatf("forward wrap continuity before=%0d after=%0d", $signed(pattern), $signed(r)));
        check(abs_i($signed(r) - $signed(pattern)) < 64, "forward wrap delta bounded");
        enc_angle = 16380;
        wait_pwm_periods(3);
        read_reg(A_ACTUAL_POS, r);
        check(abs_i($signed(r) - $signed(pattern)) < 64, "reverse wrap delta bounded");

        $display("TEST SPI parity retry");
        enc_angle = 2222;
        enc_bad_once = 1'b1;
        wait_pwm_periods(3);
        read_reg(A_ACTUAL_POS, r);
        check(r[13:0] == 14'd2222 || abs_i(($signed(r) & 14'h3FFF) - 2222) < 8, $sformatf("parity retry recovered actual_pos=%0d", $signed(r)));
        read_reg(A_STATUS, r);
        check(r[3] == 1'b0, "single parity retry does not fault");

        $display("TEST SPI error flag clear transaction and recovery");
        old_clear_seen = enc_clear_seen;
        enc_angle = 3333;
        enc_error_once = 1'b1;
        wait_pwm_periods(4);
        check(enc_clear_seen > old_clear_seen, "encoder clear-error command observed");
        read_reg(A_STATUS, r);
        check(r[3] == 1'b0, "single encoder error recovery does not fault");

        $display("TEST fresh-pair ordering with current conversion before angle");
        enc_angle = 1000;
        xadc_eoc_delay = 1;
        xadc_drdy_delay = 1;
        set_currents(0, 0);
        write_reg(A_MANUAL_ELEC, 32'd8192);
        write_reg(A_FOC_KP, 32'h01000000);
        write_reg(A_FOC_KI, 32'h00000000);
        write_reg(A_FOC_VMAX, 32'h00400000);
        write_reg(A_VEL_IMAX, 32'h00800000);
        write_reg(A_TARGET_DQ, {16'sd4096,16'sd0});
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL);
        check_pwm_ideal(1.0, 0.0, 0.0, 12, "fresh_current_first");

        $display("TEST fresh-pair ordering with angle before current plus ignored XADC channel");
        xadc_eoc_delay = 120;
        xadc_drdy_delay = 420;
        xadc_junk_once = 1'b1;
        check_pwm_ideal(1.0, 0.0, 0.0, 14, "fresh_angle_first");
        read_reg(A_STATUS, r);
        check(r[3] == 1'b0, "slow current conversion remains inside watchdog window");
        xadc_eoc_delay = 2;
        xadc_drdy_delay = 2;

        $display("TEST encoder watchdog fault latch and clear");
        enc_hold_bad = 1'b1;
        wait_clks(8000);
        wait_motor(1'b0, 1000);
        read_reg(A_STATUS, r);
        check(r[3] == 1'b1, "encoder timeout fault latched");
        check(r[1] == 1'b0, "encoder presence cleared on timeout");
        enc_hold_bad = 1'b0;
        wait_pwm_periods(3);
        read_reg(A_STATUS, r);
        check(r[1] == 1'b1, "encoder presence recovers before fault clear");
        check(r[3] == 1'b1, "fault remains latched after sensor recovery");
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL | 32'h00000010);
        wait_pwm_periods(1);
        read_reg(A_CONTROL, r);
        check(r[4] == 1'b0, "clear-fault command bit self clears");
        wait_motor(1'b1, 5000);
        read_reg(A_STATUS, r);
        check(r[3] == 1'b0, "encoder fault cleared");

        $display("TEST current-sensor watchdog fault latch and clear");
        xadc_enable = 1'b0;
        wait_clks(8000);
        wait_motor(1'b0, 1000);
        read_reg(A_STATUS, r);
        check(r[3] == 1'b1, "current timeout fault latched");
        check(r[2] == 1'b0, "current presence cleared on timeout");
        xadc_enable = 1'b1;
        wait_pwm_periods(3);
        read_reg(A_STATUS, r);
        check(r[2] == 1'b1, "current presence recovers before fault clear");
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL | 32'h00000010);
        wait_motor(1'b1, 5000);
        read_reg(A_STATUS, r);
        check(r[3] == 1'b0, "current fault cleared");

        $display("TEST explicit recalibration command, forced disable, offset update, and current-C saturation");
        xadc_raw_a = 16'h4000;
        xadc_raw_b = 16'hC000;
        offa = adc_scaled(16'h4000);
        offb = adc_scaled(16'hC000);
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL | 32'h00000008);
        wait_motor(1'b0, 1000);
        read_reg(A_CONTROL, r);
        check(r[0] == 1'b0, "calibrate command forces stored enable low");
        check(r[3] == 1'b0, "calibrate command bit self clears");
        read_reg(A_STATUS, r);
        check(r[8] == 1'b1, "explicit calibration active");
        wait_calibration_done();
        expected_off_a = offa;
        expected_off_b = offb;
        read_reg(A_CAL_OFF_AB, r);
        check(r[31:16] == offa[15:0], $sformatf("recal offset A got=%0d exp=%0d", r[31:16], offa));
        check(r[15:0] == offb[15:0], $sformatf("recal offset B got=%0d exp=%0d", r[15:0], offb));
        xadc_raw_a = 16'hFFFF;
        xadc_raw_b = 16'h0000;
        write_reg(A_FOC_KP, 32'h00000000);
        write_reg(A_FOC_KI, 32'h00000000);
        write_reg(A_TARGET_DQ, 32'h00000000);
        write_reg(A_CONTROL, CTRL_CURRENT_MANUAL);
        wait_motor(1'b1, 5000);
        wait_pwm_periods(3);
        read_reg(A_PHASE_C_ID, r);
        get_model_currents(ia, ib, ic);
        check(ia > 19000 && ib > 19000, $sformatf("asymmetric offsets create large same-sign currents A=%0d B=%0d", ia, ib));
        check(ic == -32768, $sformatf("model current C saturation exp=-32768 got=%0d", ic));
        check(signed16(r[31:16]) == -32768, $sformatf("DUT current C saturation got=%0d", signed16(r[31:16])));

        $display("TEST reset interruption while pipeline active");
        write_reg(A_FOC_KP, 32'h01000000);
        write_reg(A_TARGET_DQ, {16'sd4096,16'sd4096});
        wait_clks(300);
        i_rst = 1'b1;
        wait_clks(4);
        check(o_motor_en == 1'b0, "reset drops motor enable");
        check(o_pwm_a == 1'b0 && o_pwm_b == 1'b0 && o_pwm_c == 1'b0, "reset drives all PWM low");
        i_rst = 1'b0;
        expected_off_a = 13517;
        expected_off_b = 13517;
        xadc_raw_a = 16'h8000;
        xadc_raw_b = 16'h8000;
        wait_clks(50);
        read_reg(A_CONTROL, r);
        check(r == 32'h00000000, "reset clears control register");
        read_reg(A_FOC_KP, r);
        check(r == 32'h00000000, "reset clears gains");
        read_reg(A_STATUS, r);
        check(r[0] == 1'b0, "reset status drive disabled");
        check(r[8] == 1'b1, "reset restarts current calibration");

        if (errors == 0) begin
            $display("TOP_FOC_SEQUENCER_TB_PASS checks=%0d errors=%0d", checks, errors);
            $finish;
        end else begin
            $fatal(1, "TOP_FOC_SEQUENCER_TB_FAIL checks=%0d errors=%0d", checks, errors);
        end
    end

    initial begin
        byte unsigned rb;
        integer i;
        forever begin
            @(negedge o_uart_tx);
            #(UART_BIT_NS * 1.5);
            rb = 8'h00;
            for (i = 0; i < 8; i = i + 1) begin
                rb[i] = o_uart_tx;
                #(UART_BIT_NS);
            end
            check(o_uart_tx === 1'b1, "UART TX stop bit");
            tx_mb.put(rb);
            #(UART_BIT_NS * 0.4);
        end
    end

    always @(negedge o_enc_n_cs) begin
        spi_tx_word = 16'h0000;
        spi_rise_count = 0;
        spi_bit_idx = 15;
        spi_read_bad_latched = 1'b0;
        spi_read_error_latched = 1'b0;
        if (enc_state == ENC_READ) begin
            spi_read_bad_latched = enc_hold_bad | enc_bad_once;
            spi_read_error_latched = enc_error_once;
            spi_response = make_encoder_frame(enc_angle, spi_read_error_latched, spi_read_bad_latched);
            if (enc_bad_once)
                enc_bad_once = 1'b0;
            if (enc_error_once)
                enc_error_once = 1'b0;
        end else begin
            spi_response = 16'h0000;
        end
        i_enc_miso = spi_response[15];
    end

    always @(posedge o_enc_sclk) begin
        if (!o_enc_n_cs) begin
            spi_tx_word = {spi_tx_word[14:0], o_enc_mosi};
            spi_rise_count = spi_rise_count + 1;
        end
    end

    always @(negedge o_enc_sclk) begin
        if (!o_enc_n_cs && spi_bit_idx > 0) begin
            spi_bit_idx = spi_bit_idx - 1;
            i_enc_miso = spi_response[spi_bit_idx];
        end
    end

    always @(posedge o_enc_n_cs) begin
        if (!i_rst && spi_rise_count >= 16) begin
            if (enc_state == ENC_CMD) begin
                enc_state = ENC_READ;
            end else if (enc_state == ENC_READ) begin
                if (spi_read_bad_latched)
                    enc_state = ENC_CMD;
                else if (spi_read_error_latched)
                    enc_state = ENC_CLEAR;
                else
                    enc_state = ENC_CMD;
            end else begin
                if (spi_tx_word == 16'h4001)
                    enc_clear_seen = enc_clear_seen + 1;
                enc_state = ENC_CMD;
            end
        end
    end

    always @(posedge i_rst) begin
        enc_state = ENC_CMD;
        i_enc_miso = 1'b0;
        spi_response = 16'h0000;
        spi_tx_word = 16'h0000;
        spi_bit_idx = 15;
        spi_rise_count = 0;
        spi_read_bad_latched = 1'b0;
        spi_read_error_latched = 1'b0;
    end

    initial begin
        #(100000000.0);
        $fatal(1, "global simulation timeout");
    end
endmodule
