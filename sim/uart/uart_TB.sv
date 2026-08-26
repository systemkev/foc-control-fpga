`timescale 1ns/1ps

module uart_TB;

	localparam time CLK_PERIOD = 10ns;
	localparam time BIT_PERIOD = 1s / 921600;
    localparam logic [7:0] SYNC_BYTE  = 8'hAA;
    localparam logic [7:0] READ_CMD   = 8'h01;
    localparam logic [7:0] WRITE_CMD  = 8'h02;

    logic clk;
    logic rst;
    logic rx;

    logic tx;
    logic tx_done;
    logic tx_rdy;

    logic [7:0] rx_data;
    logic       rx_vld;
    logic       rx_baud;
    integer     rx_bit;

    logic       parser_vld;
    logic       parser_rw;
    logic [31:0] parser_data;
    logic [7:0]  parser_addr;

    logic        rf_vld;
    logic [31:0] rf_data;
    logic [7:0]  rf_addr;

    logic       ser_vld;
    logic [7:0] ser_byte;

    function automatic logic [7:0] crc8_56bit(input logic [55:0] data_in);
        logic [7:0] crc;
        logic [7:0] poly;
        logic inv;
        begin
            crc = 8'h00;
            poly = 8'h07;

            for (int i = 55; i >= 0; i--) begin
                inv = crc[7] ^ data_in[i];
                crc = {crc[6:0], 1'b0};
                if (inv) begin
                    crc = crc ^ poly;
                end
            end

            return crc;
        end
    endfunction

    task automatic send_frame;
        input logic rw;
        input logic [7:0] addr;
        input logic [31:0] data;
        logic [7:0] cmd;
        logic [7:0] crc;
        begin
            cmd = rw ? WRITE_CMD : READ_CMD;
            crc = crc8_56bit({SYNC_BYTE, cmd, addr, data});

            send_byte(SYNC_BYTE);
            send_byte(cmd);
            send_byte(addr);
            send_byte(data[31:24]);
            send_byte(data[23:16]);
            send_byte(data[15:8]);
            send_byte(data[7:0]);
            send_byte(crc);
        end
    endtask

	always begin
		#(CLK_PERIOD / 2) clk = ~clk;
	end

    uart_rx u_uart_rx (
        .i_clk  (clk),
        .i_rst  (rst),
        .i_rx   (rx),
        .o_data (rx_data),
        .o_vld  (rx_vld),
        .o_baud (rx_baud),
        .o_bit  (rx_bit)
    );

    parser u_parser (
        .i_clk  (clk),
        .i_rst  (rst),
        .i_vld  (rx_vld),
        .i_byte (rx_data),
        .o_vld  (parser_vld),
        .o_rw   (parser_rw),
        .o_data (parser_data),
        .o_addr (parser_addr)
    );

    reg_file u_reg_file (
        .i_clk  (clk),
        .i_rst  (rst),
        .i_vld  (parser_vld),
        .i_rw   (parser_rw),
        .i_data (parser_data),
        .i_addr (parser_addr),
        .o_vld  (rf_vld),
        .o_data (rf_data),
        .o_addr (rf_addr)
    );

    serializer u_serializer (
        .i_clk  (clk),
        .i_rst  (rst),
        .i_vld  (rf_vld),
        .i_data (rf_data),
        .i_addr (rf_addr),
        .i_rdy  (tx_rdy),
        .o_vld  (ser_vld),
        .o_byte (ser_byte)
    );

    uart_tx u_uart_tx (
        .i_clk  (clk),
        .i_rst  (rst),
        .i_vld  (ser_vld),
        .i_byte (ser_byte),
        .o_tx   (tx),
        .o_done (tx_done),
        .o_rdy  (tx_rdy)
    );

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

        for (int addr = 0; addr <= 8'h0A; addr++) begin
            automatic logic [7:0] a;
            automatic logic [31:0] d;

            a = addr[7:0];
            d = 32'h1000_0000 + (addr << 8) + addr;

            // Write each address with a unique, deterministic payload.
            send_frame(1'b1, a, d);
            repeat (2) #(BIT_PERIOD);

            // Read the same address back through the reg_file -> serializer -> uart_tx path.
            send_frame(1'b0, a, 32'h0000_0000);
            repeat (3) #(BIT_PERIOD);
        end

        repeat (10000) @(posedge clk);
		$finish;
	end

endmodule
