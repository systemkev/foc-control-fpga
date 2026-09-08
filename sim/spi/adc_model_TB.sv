`timescale 1ns/1ns

module adc_model_TB;

localparam CLK_FREQ     = 50.0e6;
localparam CLK_PRD_NS   = 1.0e9 / CLK_FREQ;

logic i_clk; 
logic i_rst;
logic i_start;
logic i_miso;
logic o_mosi;
logic o_n_cs;
logic o_sclk;
logic o_vld;

logic [13:0] o_raw_angle;

int bit_counter;

logic [15:0] mock_miso_payload = 16'h0CCC; 
    
adc_spi_master u_spi_master (
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

always begin
    #(CLK_PRD_NS / 2.0);
    i_clk = ~i_clk;
end 

assign bit_counter = u_spi_master.r_bit_cnt;

always_comb begin
    if (bit_counter >= 0 && bit_counter <= 15) begin
        i_miso = mock_miso_payload[15 - bit_counter];
    end else begin
        i_miso = 1'b0;
    end
end

initial begin 
    i_clk   = 1'b0;
    i_rst   = 1'b1;
    i_start = 1'b0;

    #(CLK_PRD_NS * 5.0);
    i_rst   = 1'b0;
    #(CLK_PRD_NS * 15.0);

    @(posedge i_clk);
    i_start = 1'b1;

    @(posedge i_clk);
    i_start = 1'b0;

    #(CLK_PRD_NS * 1000);
    $finish;
end 

endmodule