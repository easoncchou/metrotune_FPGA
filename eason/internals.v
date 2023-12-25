// top level module "topdog"
module internals #(parameter CLOCK_FREQUENCY = 50000000)(KEY, SW, CLOCK_50, HEX0, HEX1, HEX2, LEDR, 
                AUD_ADCDAT, AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, FPGA_I2C_SDAT, AUD_XCK, AUD_DACDAT,
                FPGA_I2C_SCLK, PS2_CLK, PS2_DAT,
					 VGA_CLK, VGA_HS,VGA_VS, VGA_BLANK_N,VGA_SYNC_N, VGA_R, VGA_G, VGA_B);
					 
    input [2:0]KEY; // 0 = Reset, 1 = Tuner, 2 = Metronome
    input [9:0]SW; 
    input CLOCK_50; // fpga clock runs at 50 MHz
	 
	 //VGA IOs
	 output VGA_CLK;
	 output VGA_HS;
	 output VGA_VS;
	 output VGA_BLANK_N;
	 output VGA_SYNC_N;
	 output [7:0] VGA_R;
	 output [7:0] VGA_G;
	 output [7:0] VGA_B;

    // input-inout-outputs for the audio
    input AUD_ADCDAT;

    inout AUD_BCLK;
    inout AUD_ADCLRCK;
    inout AUD_DACLRCK;
	 
	 inout PS2_CLK;
	 inout PS2_DAT;

    inout FPGA_I2C_SDAT;
    
    output AUD_XCK;
    output AUD_DACDAT;
    output FPGA_I2C_SCLK;

    // other outputs
    output [6:0]HEX0;
    output [6:0]HEX1;
	 output [6:0]HEX2; 
    output [9:0]LEDR; // LEDs will light up when a sound is being played;

    // top level internal data transfer
    wire [7:0]State_num; // a 4-bit wide register to transfer data between the FSM and hex decoder modules
    wire [2:0]Pitch;
    wire [1:0]Speed;
    wire Pulse;
	 assign LEDR[9] = Pulse;
    wire Count;
    wire [7:0]KeyboardData;

    // internal wires and register declarations for the audio controller
    wire				audio_in_available;
    wire		[31:0]	left_channel_audio_in;
    wire		[31:0]	right_channel_audio_in;
    wire				read_audio_in;

    wire				audio_out_allowed;
    wire		[31:0]	left_channel_audio_out;
    wire		[31:0]	right_channel_audio_out;
    wire				write_audio_out;

    reg [19:0] delay_cnt;
    reg [19:0] delay;
    reg snd;

    localparam // state encoding
        START = 8'd0,
        START_WAIT = 8'd21,
        NOTE_E_LOW = 8'd1,
        NOTE_E_LOW_WAIT = 8'd2,
        NOTE_A = 8'd3,
        NOTE_A_WAIT = 8'd4,
        NOTE_D = 8'd5,
        NOTE_D_WAIT = 8'd6,
        NOTE_G = 8'd7,
        NOTE_G_WAIT = 8'd8,
        NOTE_B = 8'd9,
        NOTE_B_WAIT = 8'd10,
        NOTE_E_HIGH = 8'd11,
        NOTE_E_HIGH_WAIT = 8'd12,
        BPM_60 = 8'd13,
        BPM_60_WAIT = 8'd14,
        BPM_120 = 8'd15,
        BPM_120_WAIT = 8'd16,
        BPM_180 = 8'd17,
        BPM_180_WAIT = 8'd18,
        BPM_240 = 8'd19,
        BPM_240_WAIT = 8'd20;

    // output logic for the audio controller (sequential logic)
    always @(posedge CLOCK_50)
	if(delay_cnt == delay) begin
		delay_cnt <= 20'd0;
		snd <= !snd;
	end else delay_cnt <= delay_cnt + 1;

    // output logic for the audio controller (combinational logic)
    always @ * begin
        case (State_num)
            NOTE_E_LOW: begin
                delay = 20'd303878;
            end
            NOTE_A: begin
                delay = 20'd227272;
            end
            NOTE_D: begin
                delay = 20'd170068;
            end
            NOTE_G: begin
                delay = 20'd127551;
            end
            NOTE_B: begin
                delay = 20'd101215;
            end
            NOTE_E_HIGH: begin
                delay = 20'd75757;
            end
            default: begin
                delay = 20'd170068; // frequency for the metronomes
            end
        endcase
    end

    wire [31:0] sound = (State_num == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;

    
    assign read_audio_in			= audio_in_available & audio_out_allowed;

    assign left_channel_audio_out	= left_channel_audio_in+sound;
    assign right_channel_audio_out	= right_channel_audio_in+sound;
    assign write_audio_out			= audio_in_available & audio_out_allowed & Pulse;

    // module instantiations
    control_path cp0(CLOCK_50, KEY[0], KEY[1], KEY[2], KeyboardData, Pitch, Speed, State_num);
    rate_divider #(.CLOCK_FREQUENCY(CLOCK_FREQUENCY)) rd0(CLOCK_50, KEY[0], Speed, Pulse, Count);
	 
    // hex_decoder decoder0(State_num[3:0], HEX0);
    // hex_decoder decoder1(State_num[7:4], HEX1);
	 
	 // hex_decoder decoder2(KeyboardData[7:4], HEX5);
	 // hex_decoder decoder3(KeyboardData[3:0], HEX4);
	 board_display bd0(CLOCK_50, ~KEY[0], State_num, HEX0, HEX1, HEX2, LEDR[5:0]);
	 
    keyboard_input ki0(CLOCK_50, PS2_CLK, PS2_DAT, ~KEY[0], KeyboardData, LEDR[6]);
	 
	 vga_output vga (CLOCK_50, KEY[0], State_num, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_R, VGA_G, VGA_B);

    // audio controller
    Audio_Controller audio_controller_1 (
	// Inputs
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)
    );

    // automatic audio-video configuration module
    avconf #(.USE_MIC_INPUT(1)) avc (
        .FPGA_I2C_SCLK(FPGA_I2C_SCLK),
        .FPGA_I2C_SDAT(FPGA_I2C_SDAT),
        .CLOCK_50(CLOCK_50),
        .reset(~KEY[0])
    );

endmodule

module control_path(clock, resetn, tuner, metronome, keyboardData, pitch, speed, state);
    input clock;
    input resetn;
    input tuner;
    input metronome;
    input [7:0] keyboardData;
    output reg [2:0]pitch;
    output reg [1:0]speed;
    output reg [7:0]state;

    reg [7:0]current_state, next_state;

    localparam // state encoding
        START = 8'd0,
        TUNER_WAIT = 8'd21,
		METRONOME_WAIT = 8'd22,
        NOTE_E_LOW = 8'd1,
        NOTE_E_LOW_WAIT = 8'd2,
        NOTE_A = 8'd3,
        NOTE_A_WAIT = 8'd4,
        NOTE_D = 8'd5,
        NOTE_D_WAIT = 8'd6,
        NOTE_G = 8'd7,
        NOTE_G_WAIT = 8'd8,
        NOTE_B = 8'd9,
        NOTE_B_WAIT = 8'd10,
        NOTE_E_HIGH = 8'd11,
        NOTE_E_HIGH_WAIT = 8'd12,
        BPM_60 = 8'd13,
        BPM_60_WAIT = 8'd14,
        BPM_120 = 8'd15,
        BPM_120_WAIT = 8'd16,
        BPM_180 = 8'd17,
        BPM_180_WAIT = 8'd18,
        BPM_240 = 8'd19,
        BPM_240_WAIT = 8'd20;

    always @(posedge clock or negedge resetn) begin // state transition code
        if (!resetn)
            current_state <= START;
        else if (keyboardData == 8'h24) begin
            current_state <= NOTE_E_LOW;
		  end 
        else if (keyboardData == 8'h1C) begin
            current_state <= NOTE_A;
		  end
        else if (keyboardData == 8'h23) begin
            current_state <= NOTE_D;
		  end
        else if (keyboardData == 8'h34) begin
            current_state <= NOTE_G;
		  end
        else if (keyboardData == 8'h32) begin
            current_state <= NOTE_B;
		  end
        else if ((keyboardData == 8'h59 || keyboardData == 8'h12) && current_state == NOTE_E_LOW) begin
            current_state <= NOTE_E_HIGH;
		  end
        else if (keyboardData == 8'h16) begin
            current_state <= BPM_60;
		  end
        else if (keyboardData == 8'h1E) begin
            current_state <= BPM_120;
		  end
        else if (keyboardData == 8'h26) begin
            current_state <= BPM_180;
		  end
        else if (keyboardData == 8'h25) begin
            current_state <= BPM_240;
		  end
        else 
            current_state <= next_state;
    end
    // ***
    // NOTE THAT STATE TRANSITIONS BETWEEN NOTES WILL HAVE MORE ADDED TO IT ONCE KEYBOARD INPUT IS FINISHED
    // ***
    always @* begin // state transition logic
        case (current_state)
            START: begin
                if (!tuner)
                    next_state = TUNER_WAIT;
                else if (!metronome)
                    next_state = METRONOME_WAIT;
                else 
                    next_state = START; 
            end
            TUNER_WAIT: next_state = tuner ? NOTE_E_LOW : TUNER_WAIT;
				
			METRONOME_WAIT: next_state = metronome ? BPM_60 : METRONOME_WAIT;
				
            NOTE_E_LOW: next_state = !tuner ? NOTE_E_LOW_WAIT : NOTE_E_LOW;
				
            NOTE_E_LOW_WAIT: next_state = tuner ? NOTE_A : NOTE_E_LOW_WAIT;
				
            NOTE_A: next_state = !tuner ? NOTE_A_WAIT : NOTE_A;
				
            NOTE_A_WAIT: next_state = tuner ? NOTE_D : NOTE_A_WAIT;
				
            NOTE_D: next_state = !tuner ? NOTE_D_WAIT : NOTE_D;
				
            NOTE_D_WAIT: next_state = tuner ? NOTE_G : NOTE_D_WAIT;
				
            NOTE_G: next_state = !tuner ? NOTE_G_WAIT : NOTE_G;
				
            NOTE_G_WAIT: next_state = tuner ? NOTE_B : NOTE_G_WAIT;
				
            NOTE_B: next_state = !tuner ? NOTE_B_WAIT : NOTE_B;
				
            NOTE_B_WAIT: next_state = tuner ? NOTE_E_HIGH : NOTE_B_WAIT;
				
            NOTE_E_HIGH: next_state = !tuner ? NOTE_E_HIGH_WAIT : NOTE_E_HIGH;
				
            NOTE_E_HIGH_WAIT: next_state = tuner ? NOTE_E_LOW : NOTE_E_HIGH_WAIT;
				
            BPM_60: next_state = !metronome ? BPM_60_WAIT : BPM_60;
				
            BPM_60_WAIT: next_state = metronome ? BPM_120 : BPM_60_WAIT;
				
            BPM_120: next_state = !metronome ? BPM_120_WAIT : BPM_120;
				
            BPM_120_WAIT: next_state = metronome ? BPM_180 : BPM_120_WAIT;
				
            BPM_180: next_state = !metronome ? BPM_180_WAIT : BPM_180;
				
            BPM_180_WAIT: next_state = metronome ? BPM_240 : BPM_180_WAIT;
				
            BPM_240: next_state = !metronome ? BPM_240_WAIT : BPM_240;
				
            BPM_240_WAIT: next_state = metronome ? BPM_60 : BPM_240_WAIT;
            default: next_state = START;
        endcase
    end

    always @(posedge clock) begin // output logic
        case (current_state)
            START: begin
                speed <= 2'b00;
                pitch <= 3'b000;
                state <= 8'b00000000; // state #0
            end
            NOTE_E_LOW: begin
                speed <= 2'b01;
                pitch <= 3'b001;
                state <= 8'b00000001; // state #1
            end
            NOTE_A: begin
                speed <= 2'b01;
                pitch <= 3'b010;
                state <= 8'b00000011; // state #3
            end
            NOTE_D: begin
                speed <= 2'b01;
                pitch <= 3'b011;
                state <= 8'b00000101; // state #5
            end
            NOTE_G: begin
                speed <= 2'b01;
                pitch <= 3'b100;
                state <= 8'b00000111; // state #7
            end
            NOTE_B: begin
                speed <= 2'b01;
                pitch <= 3'b101;
                state <= 8'b00001001; // state #9
            end
            NOTE_E_HIGH: begin
                speed <= 2'b01;
                pitch <= 3'b110;
                state <= 8'b00001011; // state #11
            end
            BPM_60: begin
                speed <= 2'b00;
                pitch <= 3'b010;
                state <= 8'b00001101; // state #13
            end
            BPM_120: begin
                speed <= 2'b01;
                pitch <= 3'b010;
                state <= 8'b00001111; // state #15
            end
            BPM_180: begin
                speed <= 2'b10;
                pitch <= 3'b010;
                state <= 8'b00010001; // state #17
            end
            BPM_240: begin
                speed <= 2'b11;
                pitch <= 3'b010;
                state <= 8'b00010011; // state #19
            end
				default:  begin
					speed <= 2'b0;
					pitch <= 3'b0;
					state <= 8'b00000000;
				end
        endcase
    end
endmodule

module rate_divider
    #(parameter CLOCK_FREQUENCY = 50000000) (
    input ClockIn,
    input Reset,
    input [1:0] Speed,
    output reg pulse,
    output reg [$clog2(4*CLOCK_FREQUENCY) : 0] count
    );

    always @ (posedge ClockIn) begin
        if (!Reset)
            count <= 0;
        else if (count != 0)
            count <= (count-1);
        else if (count == 0)
            case (Speed)
                2'b00: count = (CLOCK_FREQUENCY-1);    // 60 BPM
                2'b01: count = (0.5*CLOCK_FREQUENCY-1); // 120
                2'b10: count = (0.33*CLOCK_FREQUENCY-1); // 180
                2'b11: count = (0.25*CLOCK_FREQUENCY-1); // 240
                default: count = 7'b1000101; //funny number
            endcase
    end

    always @* begin
        case (Speed)
            2'b00: begin
               if (count < (CLOCK_FREQUENCY-1) / 10)
                    pulse = 1;
					else pulse = 0;
            end
            2'b01: begin
               if (count < (2*CLOCK_FREQUENCY-1) / 20)
                    pulse = 1;
					else pulse = 0;
            end
            2'b10: begin
               if (count < (3*CLOCK_FREQUENCY-1) / 30)
                    pulse = 1;
					else pulse = 0;
            end
            2'b11: begin
               if (count < (4*CLOCK_FREQUENCY-1) / 40)
                    pulse = 1;
					else pulse = 0;
            end
            default: pulse = 0;
        endcase
    end
endmodule

module hex_decoder(c, display);
    input [3:0] c;
    output [6:0] display;

    // top middle
    assign display[0] = (c[0] & ~c[1] & ~c[2] & ~c[3]|
                        ~c[0] & ~c[1] & c[2] & ~c[3]|
                        c[0] & c[1] & ~c[2] & c[3]|
                        c[0] & ~c[1] & c[2] & c[3]);

    // top right
    assign display[1] = (c[0] & ~c[1] & c[2] & ~c[3]|
                        ~c[0] & c[1] & c[2] & ~c[3]|
                        c[0] & c[1] & ~c[2] & c[3]|
                        ~c[0] & ~c[1] & c[2] & c[3]|
                        ~c[0] & c[1] & c[2] & c[3]|
                        c[0] & c[1] & c[2] & c[3]);

    // bottom right
    assign display[2] = (~c[0] & c[1] & ~c[2] & ~c[3]|
                        ~c[0] & ~c[1] & c[2] & c[3]|
                        ~c[0] & c[1] & c[2] & c[3]|
                        c[0] & c[1] & c[2] & c[3]);

    // bottom middle
    assign display[3] = (c[0] & ~c[1] & ~c[2] & ~c[3]|
                        ~c[0] & ~c[1] & c[2] & ~c[3]|
                        c[0] & c[1] & c[2] & ~c[3]|
                        ~c[0] & c[1] & ~c[2] & c[3]|
                        c[0] & c[1] & c[2] & c[3]);

    // bottom left
    assign display[4] = (c[0] & ~c[1] & ~c[2] & ~c[3]|
                        c[0] & c[1] & ~c[2] & ~c[3]|
                        ~c[0] & ~c[1] & c[2] & ~c[3]|
                        c[0] & ~c[1] & c[2] & ~c[3]|
                        c[0] & c[1] & c[2] & ~c[3]|
                        c[0] & ~c[1] & ~c[2] & c[3]);

    // top left
    assign display[5] = (c[0] & ~c[1] & ~c[2] & ~c[3]|
                        ~c[0] & c[1] & ~c[2] & ~c[3]|
                        c[0] & c[1] & ~c[2] & ~c[3]|
                        c[0] & c[1] & c[2] & ~c[3]|
                        c[0] & ~c[1] & c[2] & c[3]);

    // center
    assign display[6] = (~c[0] & ~c[1] & ~c[2] & ~c[3]|
                        c[0] & ~c[1] & ~c[2] & ~c[3]|
                        c[0] & c[1] & c[2] & ~c[3]|
                        ~c[0] & ~c[1] & c[2] & c[3]);
endmodule

