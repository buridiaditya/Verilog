module testbench();
	reg[15:0] IR[0:4];
	reg clock,reset;
	integer i = 0;
	reg status;
	reg[15:0] IRout;
	initial begin
		IR[0] = 16'b0000000000000000;
		IR[1] = 16'b0000000000000000;
		IR[2] = 16'b0000000000000000;
		IR[3] = 16'b0000000000000000;
		IR[4] = 16'b0000000000000000;
		status = 0;
		clock = 0;
	end
	always begin
		#5 clock = ~clock;
	end 
	// always @(posedge ldIR) begin
	// 	IRout = IR[i];
	// 	i = i+1;
	// end
	
endmodule

module controller(
	IR, status, MFC, reset, clock,
	ldMAR, ldMDR, ldIR, 
	ldPC, ldReg, ldYBuff, ldSP,
	TPC, TSP, TMAR, TMDR, TDBUS, TReg, TALU, TIR,
	funcSelect, regSelect, read, write
	);

	input[15:0] IR;
	input status, MFC, reset, clock;
	output reg read, write;
	output reg ldMAR, ldMDR, ldIR, ldPC, ldReg, ldYBuff, ldSP;
	output reg TPC, TSP, TMAR, TMDR, TDBUS, TReg, TALU, TIR;
	output reg[2:0] funcSelect;
	output reg[2:0] regSelect;

	reg[3:0] state;

	always @(posedge clock or negedge clock) begin
		if (reset == 1) begin
			state = 5'b00000;
		end
		else begin
			case(state)
				/////////////////////////// FETCH //////////////////////////////
				5'b00000: begin
					TPC = 1;	// Closing switch TPC
					funcSelect = 3'b000;	// Use transfer ALU operation
					state = 5'b00001;
				end
				5'b00001: begin
					ldMAR = 1;	// Load in MAR 
					read = 1;	// Read mode in memory
					ldMAR = 0;	// Unsetting ld control wire
					TPC = 0;	// Opening switch TPC
					TMAR = 1;	// Close switch TMAR
					state = 5'b00010;	// State transition
				end
				5'b00010: begin
					if(MFC == 1) begin
						ldIR = 1;
						TMAR = 0;
						read = 0;
						TPC = 1;
						funcSelect = 3'b001;
						state = 5'b00011;
					end
				end
				5'b00011: begin
					ldPC = 1;
					ldPC = 0;
					TPC = 0;
					if( IR[15:12] == 4'b1010 ) begin
						// IDLE
						state = 5'b00000;
					end
					else if( (IR[15:12] == 4'b1111 && IR[11:9] == 3'b000) || (IR[15:12] == 4'b1001) ) begin
						// PUSH / CALL
						state = 5'b01010;
					end
					else if(IR[15:12] == 4'b1111) begin
						// POP / RETURN / ALU
						state = 5'b00100;
					end
					else begin
						// BRANCH
						state = 5'b01111;
					end
				end
				/////////////////////////////////////////////////////////////////////////
				////////////////////////////// MEMORY READ //////////////////////////////
				5'b00100: begin
					TSP = 1;
					funcSelect = 3'b000;
					state = 5'b00101;
				end
				5'b00101: begin
					ldMAR = 1;
					read = 1;
					ldMAR = 0;
					TSP = 0;
					TMAR = 1;
					state = 5'b00110;
				end
				5'b00110: begin
					if(MFC == 1) begin
						TDBUS = 1;
						ldMDR = 1;
						TSP = 1;
						funcSelect = 3'b001;
						state = 5'b00111;
					end
				end
				5'b00111: begin
					TDBUS = 0;
					ldMDR = 0;
					ldSP = 1;
					ldSP = 0;
					state = 5'b01000;
				end
				/////////////////////////////////////////////////////////////////////////
				////////////////////////////////// ALU //////////////////////////////////
				5'b01000: begin
					TMDR = 1;
					if(IR[15:12] == 4'b1111 && IR[11:9] == 3'b110) begin
						// RETURN
						funcSelect = 3'b000;
					end
					else if(IR[15:12] == 4'b1111 && IR[11:9] == 3'b001) begin
						// POP
						funcSelect = 3'b000;
						TMDR = 0;
						regSelect = IR[8:6];
					end
					else if(IR[15:12] == 4'b1111 && (IR[11:9] == 3'b011 || IR[11:9] == 3'b101) ) begin
						// NEG or NOT
						funcSelect = IR[11:9];
						TMDR = 0;
						regSelect = IR[8:6];
					end
					else if(IR[15:12] == 4'b1111 && (IR[11:9] == 3'b100 || IR[11:9] == 3'b010) ) begin
						// ADD or OR
						ldYBuff = 1;
						ldYBuff = 0;
						regSelect = IR[8:6];
						TMDR = 0;
						TReg = 1;
						funcSelect = IR[11:9];
					end
					state = 5'b01001;
				end
				5'b01001: begin
					if(IR[15:12] == 4'b1111 && IR[11:9] == 3'b110) begin
						// RETURN
						ldPC = 1;
						ldPC = 0;
					end
					else begin
						// POP / NEG / NOT / ADD / OR
						ldReg = 1;
						ldReg = 0;
						TReg = 0;
					end
					state = 5'b00000;
				end
				/////////////////////////////////////////////////////////////////////////
				////////////////////////////// MEMORY WRITE /////////////////////////////
				5'b01010: begin
					TSP = 1;
					funcSelect = 3'b110;
					state = 5'b01011;
				end
				5'b01011: begin
					ldSP = 1;
					ldSP = 0;
					funcSelect = 3'b000;
					state = 5'b01100;
				end
				5'b01100: begin
					ldMAR = 1;
					ldMAR = 0;
					TSP = 0;
					TMAR = 1;
					if(IR[15:12] == 4'b1111 && IR[11:9] == 3'b000) begin
						// PUSH 
						regSelect = IR[8:6]; 
						TReg = 1;
					end 
					else if( IR[15:12] == 4'b1001 ) begin
						// CALL
						TPC = 1;
					end
					funcSelect = 3'b000;
					TALU = 1;
					state = 5'b01101;						
				end
				5'b01101: begin
					ldMDR = 1;
					ldMDR = 0;
					TALU = 0;
					write = 1;
					TDBUS = 1;
					state = 5'b01110;
				end
				5'b01110: begin
					if(MFC == 1) begin
						TDBUS = 0;
						write = 0;
						if(IR[15:12] == 4'b1111 && IR[11:9] == 3'b000) begin
							// PUSH 
							state = 5'b00000;
						end 
						else if(IR[15:12] == 4'b1001) begin
							// CALL
							state = 5'b01111;
						end
					end
				end
				/////////////////////////////////////////////////////////////////////////
				//////////////////////////////// PC UPDATE //////////////////////////////
				5'b01111: begin
					TIR = 1;
					ldYBuff = 1;
					TIR = 0;
					TPC = 1;
					funcSelect = 3'b010;
					state = 5'b10000;
				end
				5'b10000: begin
					ldPC = 1;
					ldPC = 0;
					TPC = 0;
					state = 5'b00000;
				end
			endcase
		end
	end
endmodule
