module memory_controller(
    input clk,
    input [14:0] address,
    input [7:0] chipSelect,
    output reg [2:0] oQ
);

wire [2:0] q_black, q_E2, q_A2, q_D3, q_G3, q_B3, q_E4, q_60, q_120, q_180, q_240;

e2 (.clock(clk), .address(address), .q(q_E2));
a2 (.clock(clk), .address(address), .q(q_A2));
d3 (.clock(clk), .address(address), .q(q_D3));
g3 (.clock(clk), .address(address), .q(q_G3));
b3 (.clock(clk), .address(address), .q(q_B3));
e4 (.clock(clk), .address(address), .q(q_E4));

bpm60 (.clock(clk), .address(address), .q(q_60));
bpm120 (.clock(clk), .address(address), .q(q_120));
bpm180 (.clock(clk), .address(address), .q(q_180));
bpm240 (.clock(clk), .address(address), .q(q_240));

black (.clock(clk), .address(address), .q(q_black));


always @(*) begin
    case (chipSelect)
        8'd0: oQ <= q_black;
        8'd1: oQ <= q_E2;
        8'd3: oQ <= q_A2;
        8'd5: oQ <= q_D3;
        8'd7: oQ <= q_G3;
        8'd9: oQ <= q_B3;
        8'd11: oQ <= q_E4;
        8'd13: oQ <= q_60;
        8'd15: oQ <= q_120;
        8'd17: oQ <= q_180;
        8'd19: oQ <= q_240;
    endcase

end

endmodule