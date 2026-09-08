`timescale 1ns/1ns;
module pwm_gen_TB;

localparam real C_CLK_FREQ      = 50.0e6;
localparam real C_CLK_PRD_NS    = 1.0e9 / C_CLK_FREQ;
localparam real C_PWM_FREQ      = 20.0e3;
localparam int  C_CLKS_IN_PWM   = int'(C_CLK_FREQ / C_PWM_FREQ);
localparam real C_PWM_PRD_NS    = C_CLKS_IN_PWM * C_CLK_PRD_NS;
localparam int  C_DC_BITS       = $clog2(C_CLKS_IN_PWM);

logic clk = 0;
logic rst;
logic [C_DC_BITS-1 : 0] duty;
logic pwm;
logic pwm_start;

pwm_gen u_pwm_gen (
    .i_clk              (clk),
    .i_rst              (rst),
    .i_duty             (duty),
    .o_pwm              (pwm),
    .o_pwm_is_first_cycle (pwm_start)
);

always begin 
    clk = ~clk;
    #(C_CLK_PRD_NS / 2.0);
end 

task automatic run_test;
    input real duty_cycle;      // 0 to 100 (%)
    
    duty = int'(duty_cycle * C_CLKS_IN_PWM / 100.0);
endtask 

initial begin
    rst   = 1;
    #(C_CLK_PRD_NS * 3);
    
    rst   = 0;
    #(C_CLK_PRD_NS * 3);

    run_test(25.0);

    #(C_PWM_PRD_NS * 10);

    $finish;
end

endmodule 