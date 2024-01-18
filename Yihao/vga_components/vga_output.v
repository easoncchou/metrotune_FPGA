module vga_output(
    input clk, 
    input reset,
    input [7:0] stateNum,
	 
	 output VGA_CLK,
	 output VGA_HS,
	 output VGA_VS,
	 output VGA_BLANK_N,
	 output VGA_SYNC_N,
	 output [7:0] VGA_R,
	 output [7:0] VGA_G,
	 output [7:0] VGA_B
	 
);

wire [$clog2(160):0] oX;
wire [$clog2(120):0] oY;
wire [2:0] oColour;
wire oPlot;

read_mif(.clk(clk), .reset(reset), .stateNum(stateNum), .oX(oX), .oY(oY), .oColour(oColour), .oPlot(oPlot));

vga_adapter VGA(
        .resetn(reset),
        .clock(clk),
        .colour(oColour),
        .x(oX),
        .y(oY),
        .plot(oPlot),
        /* Signals for the DAC to drive the monitor. */
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK(VGA_BLANK_N),
        .VGA_SYNC(VGA_SYNC_N),
        .VGA_CLK(VGA_CLK));
    defparam VGA.RESOLUTION = "160x120";
    defparam VGA.MONOCHROME = "FALSE";
    defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
    defparam VGA.BACKGROUND_IMAGE = "black.mif";

endmodule

