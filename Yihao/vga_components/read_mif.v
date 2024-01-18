module read_mif(
    input clk,
    input reset,
    input [7:0] stateNum,
    output reg [$clog2(160):0] oX,
    output reg [$clog2(120):0] oY,
    output wire [2:0] oColour,
    output reg oPlot
);

//reg [$clog2(160):0] x_coord;
//reg [$clog2(120):0] y_coord;
wire [15:0] address;
wire [14:0] mem_address;

assign address = ({1'b0, oY , 7'd0} + {1'b0, oY, 5'd0} + {1'b0, oX});
assign mem_address = address[14:0];

memory_controller mc0 (.clk(clk), .address(mem_address), .chipSelect(stateNum), .oQ(oColour));

always @(posedge clk) begin
    if (!reset) begin
//        x_coord <= 0;
//        y_coord <= 0;
			oY <=0;
			oX <= 0;
        oPlot <= 0;
    end

    else begin
        if (oX == 8'd159 && oY == 7'd119) begin
            oX <= 0;
            oY <= 0;

        end 

        else if (oX == 8'd159) begin
            oX <= 0;
            oY <= oY +1;
        end

        else begin
            oX <= oX +1;
        end
    end
    oPlot <= 1;
	 
end

endmodule