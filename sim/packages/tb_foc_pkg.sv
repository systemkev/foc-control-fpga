package tb_foc_pkg;

    localparam real PI = 3.1415926535897932384626433832795;

    localparam int PHASE_FRAC_BITS   = 12;
    localparam int VOLTAGE_FRAC_BITS = 12;
    localparam int CORDIC_FRAC_BITS  = 14;

    localparam real PHASE_LSB   = 1.0 / 4096.0;
    localparam real VOLTAGE_LSB = 1.0 / 4096.0;
    localparam real CORDIC_LSB  = 1.0 / 16384.0;

    typedef struct packed {
        logic signed [15:0] A;
        logic signed [15:0] B;
        logic signed [15:0] C;
    } abc_phase_t;

    typedef struct packed {
        logic signed [15:0] alpha;
        logic signed [15:0] beta;
    } ab_phase_t;

    typedef struct packed {
        logic signed [15:0] d;
        logic signed [15:0] q;
    } dq_phase_t;

    typedef struct packed {
        logic signed [17:0] A;
        logic signed [17:0] B;
        logic signed [17:0] C;
    } abc_volt_t;

    typedef struct packed {
        logic signed [17:0] alpha;
        logic signed [17:0] beta;
    } ab_volt_t;

    typedef struct packed {
        logic signed [17:0] d;
        logic signed [17:0] q;
    } dq_volt_t;

    function automatic int round_nearest(input real x);
        if (x >= 0.0)
            return $rtoi(x + 0.5);
        else
            return $rtoi(x - 0.5);
    endfunction

    function automatic logic signed [15:0] q3_12(input real x);
        int raw;

        raw = round_nearest(x * 4096.0);

        if (raw > 32767)
            raw = 32767;
        else if (raw < -32768)
            raw = -32768;

        return $signed(raw);
    endfunction

    function automatic real q3_12_to_real(
        input logic signed [15:0] x
    );
        return $itor($signed(x)) / 4096.0;
    endfunction

    function automatic logic signed [17:0] q5_12(input real x);
        int raw;

        raw = round_nearest(x * 4096.0);

        if (raw > 131071)
            raw = 131071;
        else if (raw < -131072)
            raw = -131072;

        return $signed(raw);
    endfunction

    function automatic real q5_12_to_real(
        input logic signed [17:0] x
    );
        return $itor($signed(x)) / 4096.0;
    endfunction

    function automatic logic signed [15:0] q1_14(input real x);
        int raw;

        raw = round_nearest(x * 16384.0);

        if (raw > 32767)
            raw = 32767;
        else if (raw < -32768)
            raw = -32768;

        return $signed(raw);
    endfunction

    function automatic real q1_14_to_real(
        input logic signed [15:0] x
    );
        return $itor($signed(x)) / 16384.0;
    endfunction

    function automatic logic signed [15:0] angle_from_degrees(
        input real deg
    );
        int raw;

        raw = round_nearest(deg * 32768.0 / 180.0);

        if (raw > 32767)
            raw = 32767;
        else if (raw < -32768)
            raw = -32768;

        return $signed(raw);
    endfunction

    function automatic real angle_raw_to_radians(
        input logic signed [15:0] raw
    );
        return $itor($signed(raw)) * PI / 32768.0;
    endfunction

    function automatic real angle_raw_to_degrees(
        input logic signed [15:0] raw
    );
        return $itor($signed(raw)) * 180.0 / 32768.0;
    endfunction

    function automatic real clamp5(input real x);
        if (x > 5.0)
            return 5.0;
        else if (x < -5.0)
            return -5.0;
        else
            return x;
    endfunction

    function automatic real clamp_q3_12(input real x);
        if (x > 7.999755859375)
            return 7.999755859375;
        else if (x < -8.0)
            return -8.0;
        else
            return x;
    endfunction

    function automatic real clamp_q5_12(input real x);
        if (x > 31.999755859375)
            return 31.999755859375;
        else if (x < -32.0)
            return -32.0;
        else
            return x;
    endfunction

    function automatic real rabs(input real x);
        return (x < 0.0) ? -x : x;
    endfunction

endpackage