module controller(
	status,clock, reset,IR[15:0], read, write,
	ldReg,ldPC,ldSP,
	S1,	// SP Update {sp+1,sp-1}
	S2,	// M - Input DBus {Reg, PC}
	S3, // M - Address Bus {sp, sp-updated}
	S4, // ALU - YBus {MemOut, PC + 4}
	S5,	// ALU - XBus {RegOut, Label}
	S6, S7 // PC - Update {PC + 4, PC + 4 + Label, M[SP]} 
	); 
	input status, clock, reset;
	input[15:0] IR;
	output wire ldReg, ldPC, ldSP, read, write;
	output wire S1,S2,S3,S4,S5,S6,S7;
	reg state;

	assign ldReg = ( (state) && ( ~(|(IR[15:11] ^ 5'b00010)) || ~(|(IR[15:11] ^ 5'b00011)) || ~(|(IR[15:11] ^ 5'b00100)) || ~(|(IR[15:11] ^ 5'b00101)) ) );
	assign ldSP = ( (state) && ( (|(IR[15:11] ^ 5'b00000)) );
	assign ldPC = (state);
	assign write = (state) && ( ~(|(IR[15:11] ^ 5'b00000)) || ~(|(IR[15:11] ^ 5'b00110)));
	assign read = ~write;
	assign S1 = (~state) && ;
	always @( (posedge clock) || (negedge clock) ) begin
		if(reset == 1) begin
			state = 0;
		end
		else begin
			if(state == 1)begin
				state = 0;
			end
			else begin
				state == 1;
			end
		end
	end
endmodule