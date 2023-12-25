module keyboard_input(
    input CLOCK_50, 
    inout PS2_CLK, PS2_DAT,
    input reset,
    output [7:0] received_data,
	 output data_en
);

always @ (posedge CLOCK_50) begin
    dataOutput = 8'b0;
end

PS2_Controller ps2_keyboard(.CLOCK_50(CLOCK_50), .reset(reset),
                             .PS2_CLK(PS2_CLK), .PS2_DAT(PS2_DAT),
                             .received_data(received_data), .received_data_en(data_en));
endmodule

