module board_display(
    input clk,
    input reset,
    input [7:0] stateNum,
    output [6:0] HEX0, HEX1, HEX2,
    output reg [5:0] LEDR
);

reg [3:0] hex0_display, hex1_display, hex2_display;

hex_decoder h0 (hex0_display, HEX0);
hex_decoder h1 (hex1_display, HEX1);
hex_decoder h2 (hex2_display, HEX2);

parameter E2 = 8'd1; // E2 note
parameter A2 = 8'd3; // A2 note
parameter D3 = 8'd5; // D3 note
parameter G3 = 8'd7; // G3 note
parameter B3 = 8'd9; // B3 note
parameter E4 = 8'd11; // E4 note

parameter bpm60 = 8'd13; 
parameter bpm120 = 8'd15;
parameter bpm180 = 8'd17;
parameter bpm240 = 8'd19;


always @(posedge clk or posedge reset) begin
    //reset
    if (reset) begin
        LEDR <= 0;
        hex0_display <= 0;
        hex1_display <= 0;
        hex2_display <= 0;
    end 
    
    //LEDR output
    case (stateNum)
        E2: begin
            LEDR[0] <= 0;
            LEDR[1] <= 0;
            LEDR[2] <= 0;
            LEDR[3] <= 0;
            LEDR[4] <= 0;
            LEDR[5] <= 1;
        end
        A2: begin
            LEDR[0] <= 0;
            LEDR[1] <= 0;
            LEDR[2] <= 0;
            LEDR[3] <= 0;
            LEDR[4] <= 1;
            LEDR[5] <= 0;
        end
        D3: begin
            LEDR[0] <= 0;
            LEDR[1] <= 0;
            LEDR[2] <= 0;
            LEDR[3] <= 1;
            LEDR[4] <= 0;
            LEDR[5] <= 0;
        end
        G3: begin
            LEDR[0] <= 0;
            LEDR[1] <= 0;
            LEDR[2] <= 1;
            LEDR[3] <= 0;
            LEDR[4] <= 0;
            LEDR[5] <= 0;
        end
        B3: begin
            LEDR[0] <= 0;
            LEDR[1] <= 1;
            LEDR[2] <= 0;
            LEDR[3] <= 0;
            LEDR[4] <= 0;
            LEDR[5] <= 0;
        end
        E4: begin
            LEDR[0] <= 1;
            LEDR[1] <= 0;
            LEDR[2] <= 0;
            LEDR[3] <= 0;
            LEDR[4] <= 0;
            LEDR[5] <= 0;
        end

        default: begin
            LEDR[0] <= 0;
            LEDR[1] <= 0;
            LEDR[2] <= 0;
            LEDR[3] <= 0;
            LEDR[4] <= 0;
            LEDR[5] <= 0;
        end
    endcase

    //hex output
    case (stateNum)
        E2: begin
            hex0_display <= 4'b0010; 
            hex1_display <= 4'b1110;
            hex2_display <= 4'b0;
        end
        A2: begin
            hex0_display <= 4'b0010;
            hex1_display <= 4'b1010;
            hex2_display <= 4'b0;
        end
        D3: begin
            hex0_display <= 4'b0011;
            hex1_display <= 4'b1101;
            hex2_display <= 4'b0;
        end
        G3: begin
            hex0_display <= 4'b0011;
            hex1_display <= 4'b1001;
            hex2_display <= 4'b0;
        end
        B3: begin
            hex0_display <= 4'b0011;
            hex1_display <= 4'b1011;
            hex2_display <= 4'b0;
        end
        E4: begin
            hex0_display <= 4'b0100;
            hex1_display <= 4'b1110;
            hex2_display <= 4'b0;
        end
        bpm60: begin
            hex0_display <= 4'b0;
            hex1_display <= 4'b0110;
            hex2_display <= 4'b0;
        end

        bpm120: begin
            hex0_display <= 4'b0;
            hex1_display <= 4'b0010;
            hex2_display <= 4'b1;
        end

        bpm180: begin
            hex0_display <= 4'b0;
            hex1_display <= 4'b1000;
            hex2_display <= 4'b1;
        end

        bpm240: begin
            hex0_display <= 4'b0;
            hex1_display <= 4'b0100;
            hex2_display <= 4'b0010;
        end

        default: begin
            hex0_display <= 0;
            hex1_display <= 0;
            hex2_display <= 0;
        end
    endcase
end

endmodule


