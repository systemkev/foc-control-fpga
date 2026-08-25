`timescale 1ns/1ps

module uart_TB;

	localparam time CLK_PERIOD = 10ns;
	localparam time BIT_PERIOD = 1s / 921600;

	logic       clk;
	logic       rst;
	logic       rx;
	logic [7:0] data;
	logic       vld;
    logic       baud;
    int         cnt;

	uart_rx u_uart_tx (
		.i_clk  (clk),
		.i_rst  (rst),
		.i_rx   (rx),
		.o_data (data),
		.o_vld  (vld),
        .o_baud (baud),
        .o_bit  (cnt)
	);

    parser u_parser (
        
    );

	always begin
		#(CLK_PERIOD / 2) clk = ~clk;
	end

    task automatic send_byte;
        input logic[7:0] byt;
        int i = 0;

        rx = 1'b0;
        #(BIT_PERIOD);

        for(int i = 0; i < 8; i++) begin 
            rx = byt[i];
            #(BIT_PERIOD);
        end

        rx = 1'b1;
        #(BIT_PERIOD);

    endtask

	initial begin
        clk = 1'b0;
		rst = 1'b1;
		rx  = 1'b1;

		repeat (10) @(posedge clk);
		rst = 1'b0;
        repeat (5) @(posedge clk);
		
        send_byte(8'hCE);

        repeat(5) #(BIT_PERIOD);

        send_byte(8'hAB);

        repeat(5) #(BIT_PERIOD);

        send_byte(8'h23);

        repeat(5) #(BIT_PERIOD);

        send_byte(8'h69);

		repeat (10000) @(posedge clk);


		$finish;
	end

endmodule
