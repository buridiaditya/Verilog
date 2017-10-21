module testbench();
	reg[15:0] IR[0:4];
	integer i = 0;
	reg[15:0] IRout;

	reg status, MFC, reset, clock;
	wire read, write;
	wire ldMAR, ldMDR, ldIR, ldPC, ldReg, ldYBuff, ldSP;
	wire TPC, TSP, TMAR, TMDR, TDBUS, TReg, TALU, TIR;
	wire[2:0] funcSelect;
	wire[2:0] regSelect;

	initial begin
		$dumpfile ("shifter.vcd");
		$dumpvars;
		IR[0] = 16'b1111010001000000;
		IR[1] = 16'b0000000000000000;
		IR[2] = 16'b0000000000000000;
		IR[3] = 16'b0000000000000000;
		IR[4] = 16'b0000000000000000;
		status = 0;
		clock = 0;
		#5 reset = 1;
		#10 reset = 0;
		#9 MFC = 1;
		#3 MFC = 0;
		#20 MFC = 1;
		#27 MFC = 0;
	end
	always begin
		#5 clock = ~clock;
	end 
	always @(posedge ldIR) begin
		IRout = IR[i];
		i = i+1;
	end
	controller a(
	IRout, status, MFC, reset, clock,
	ldMAR, ldMDR, ldIR, 
	ldPC, ldReg, ldYBuff, ldSP,
	TPC, TSP, TMAR, TMDR, TDBUS, TReg, TALU, TIR,
	funcSelect, regSelect, read, write
	);
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
			read = 0;
			write = 0;
			ldMAR = 0;
			ldMDR = 0;
			ldIR = 0;
			ldPC = 0;
			ldReg = 0;
			ldYBuff = 0;
			ldSP = 0;
			TPC = 0;
			TSP = 0;
			TMAR = 0;
			TMDR = 0;
			TDBUS = 0;
			TReg = 0;
			TALU = 0;
			TIR = 0;
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

module register16bit(out, in, e, reset, clock);
    output [15:0] out;
    input [15:0] in;
    input e;
    input reset;
    input clock;
     (*keep = "true"*) reg [15:0] mem;
    
    always @(posedge clock) 
    begin 
      if (reset) 
        mem <= 16'b0; 
      else if (e) 
        mem <= in; 
    end 
    
    assign out = mem;
endmodule

module demultiplexer1_8 (w,x ,y ,z, e1,e2,e3,e4,e5,e6,e7,e8 );
output e1 ;
output e2 ;
output e3 ;
output e4 ;
output e5 ;
output e6 ;
output e7 ;
output e8 ;

input w;
input x ;
input y ;
input z;

assign e1 = w & (~x) & (~y) & (~z);
assign e2 = w & (~x) & (~y) & z ;
assign e3 = w & (~x) & (y) & (~z);
assign e4 = w & (~x) & (y) & (z);
assign e5 = w & (x) & (~y) & (~z);
assign e6 = w & (x) & (~y) & (z);
assign e7 = w & (x) & (y) & (~z);
assign e8 = w & (x) & (y) & (z);
endmodule

module regBank(dataIn, dataOut, regselect, write, read,reset, clock);
    output [15:0] dataOut;
    input [2:0] regselect;    
    input write;
    input read;
    input [15:0] dataIn;
    input reset, clock;
    
    wire e1,e2,e3,e4,e5,e6,e7,e8;
    wire[15:0] out1;
    wire[15:0] out2;
    wire[15:0] out3;
    wire[15:0] out4;
    wire[15:0] out5;
    wire[15:0] out6;
    wire[15:0] out7;
    wire[15:0] out8;
    
    demultiplexer1_8 demuxwr(write,regselect[2],regselect[1],regselect[0],e1,e2,e3,e4,e5,e6,e7,e8);
    
    register16bit r1(out1, dataIn, e1, reset, clock);
    register16bit r2(out2, dataIn, e2, reset, clock);
    register16bit r3(out3, dataIn, e3, reset, clock);
    register16bit r4(out4, dataIn, e4, reset, clock);
    register16bit r5(out5, dataIn, e5, reset, clock);
    register16bit r6(out6, dataIn, e6, reset, clock);
    register16bit r7(out7, dataIn, e7, reset, clock);
    register16bit r8(out8, dataIn, e8, reset, clock);
   
    bit16_mux_8to1(dataOut,out1,out2,out3,out4,out5,out6,out7,out8,{regselect[2],regselect[1],regselect[0]}); 
    
endmodule

module mux2to1(inp1,inp2,enable,out);
	input inp1,inp2;
    input enable;
    output out;
    assign out = enable?inp2:inp1;
endmodule

module mux4to1(a,sel,out);
	input [3:0] a;
	input [1:0] sel;
	output out;

	wire mux_1,mux_2;

	mux2to1 m1 (a[3],a[2],sel[0],mux_1),
	        m2 (a[1],a[4],sel[0],mux_2),
	        m3 (mux_1,mux_2,sel[1],out);
endmodule

module mux8to1(a,sel,out);
  input [7:0] a;
  input [2:0] sel;
  output out;
  wire mux[2:0];
 
  mux4to1 m1 (a[7:4],sel[1:0],mux[1]),
  m2 (a[3:0],sel[1:0],mux[0]);
  mux2to1 m3 (mux[1],mux[0],sel[2],out);
endmodule
 
// 16 bit mux 8 to 1
module bit16_mux_8to1(out,x0,x1,x2,x3,x4,x5,x6,x7,sel);
  input [15:0] x0,x1,x2,x3,x4,x5,x6,x7;
  input [2:0] sel;
  output [15:0] out;
  generate
      genvar k;
      for(k=0;k<16;k=k+1)
        begin : wer
          mux8to1 m({x7[k],x6[k],x5[k],x4[k],x3[k],x2[k],x1[k],x0[k]},sel,out[k]);
        end
    endgenerate
endmodule

module ALU(x,y,z,f,cy,cym1);
input [15:0] x;
input [15:0] y;
output [15:0] z;
input [2:0] f;
output cy;
output cym1;

wire [15:0] incr;
wire [15:0] orop;
wire [15:0] add;
wire [15:0] neg;
wire [15:0] decr;

assign {cy,incr} = x + 16'b000000001;
assign orop = x | y;
assign {cy,add} = x + y;
assign neg = -x;
assign decr = x - 16'b000000001;


assign z[0] = ((~f[2])&(~f[1])&(~f[0])&x[0]) + ((~f[2])&(~f[1])&(f[0])& incr[0])+ ((f[2])&(~f[1])&(~f[0])& orop[0]) + ((f[2])&(~f[1])&(f[0])& (~x[0])) + ((~f[2])&(f[1])&(~f[0])& add[0]) + ((~f[2])&(f[1])&(f[0])& neg[0]) +((f[2])&(f[1])&(~f[0])& decr[0]);
assign z[1] = ((~f[2])&(~f[1])&(~f[0])&x[1]) + ((~f[2])&(~f[1])&(f[0])& incr[1])+ ((f[2])&(~f[1])&(~f[0])& orop[1]) + ((f[2])&(~f[1])&(f[0])& (~x[1])) + ((~f[2])&(f[1])&(~f[0])& add[1]) + ((~f[2])&(f[1])&(f[0])& neg[1]) +((f[2])&(f[1])&(~f[0])& decr[1]);
assign z[2] = ((~f[2])&(~f[1])&(~f[0])&x[2]) + ((~f[2])&(~f[1])&(f[0])& incr[2])+ ((f[2])&(~f[1])&(~f[0])& orop[2]) + ((f[2])&(~f[1])&(f[0])& (~x[2])) + ((~f[2])&(f[1])&(~f[0])& add[2]) + ((~f[2])&(f[1])&(f[0])& neg[2]) +((f[2])&(f[1])&(~f[0])& decr[2]);
assign z[3] = ((~f[2])&(~f[1])&(~f[0])&x[3]) + ((~f[2])&(~f[1])&(f[0])& incr[3])+ ((f[2])&(~f[1])&(~f[0])& orop[3]) + ((f[2])&(~f[1])&(f[0])& (~x[3])) + ((~f[2])&(f[1])&(~f[0])& add[3]) + ((~f[2])&(f[1])&(f[0])& neg[3]) +((f[2])&(f[1])&(~f[0])& decr[3]);
assign z[4] = ((~f[2])&(~f[1])&(~f[0])&x[4]) + ((~f[2])&(~f[1])&(f[0])& incr[4])+ ((f[2])&(~f[1])&(~f[0])& orop[4]) + ((f[2])&(~f[1])&(f[0])& (~x[4])) + ((~f[2])&(f[1])&(~f[0])& add[4]) + ((~f[2])&(f[1])&(f[0])& neg[4]) +((f[2])&(f[1])&(~f[0])& decr[4]);
assign z[5] = ((~f[2])&(~f[1])&(~f[0])&x[5]) + ((~f[2])&(~f[1])&(f[0])& incr[5])+ ((f[2])&(~f[1])&(~f[0])& orop[5]) + ((f[2])&(~f[1])&(f[0])& (~x[5])) + ((~f[2])&(f[1])&(~f[0])& add[5]) + ((~f[2])&(f[1])&(f[0])& neg[5]) +((f[2])&(f[1])&(~f[0])& decr[5]);
assign z[6] = ((~f[2])&(~f[1])&(~f[0])&x[6]) + ((~f[2])&(~f[1])&(f[0])& incr[6])+ ((f[2])&(~f[1])&(~f[0])& orop[6]) + ((f[2])&(~f[1])&(f[0])& (~x[6])) + ((~f[2])&(f[1])&(~f[0])& add[6]) + ((~f[2])&(f[1])&(f[0])& neg[6]) +((f[2])&(f[1])&(~f[0])& decr[6]);
assign z[7] = ((~f[2])&(~f[1])&(~f[0])&x[7]) + ((~f[2])&(~f[1])&(f[0])& incr[7])+ ((f[2])&(~f[1])&(~f[0])& orop[7]) + ((f[2])&(~f[1])&(f[0])& (~x[7])) + ((~f[2])&(f[1])&(~f[0])& add[7]) + ((~f[2])&(f[1])&(f[0])& neg[7]) +((f[2])&(f[1])&(~f[0])& decr[7]);
assign z[8] = ((~f[2])&(~f[1])&(~f[0])&x[8]) + ((~f[2])&(~f[1])&(f[0])& incr[8])+ ((f[2])&(~f[1])&(~f[0])& orop[8]) + ((f[2])&(~f[1])&(f[0])& (~x[8])) + ((~f[2])&(f[1])&(~f[0])& add[8]) + ((~f[2])&(f[1])&(f[0])& neg[8]) +((f[2])&(f[1])&(~f[0])& decr[8]);
assign z[9] = ((~f[2])&(~f[1])&(~f[0])&x[9]) + ((~f[2])&(~f[1])&(f[0])& incr[9])+ ((f[2])&(~f[1])&(~f[0])& orop[9]) + ((f[2])&(~f[1])&(f[0])& (~x[9])) + ((~f[2])&(f[1])&(~f[0])& add[9]) + ((~f[2])&(f[1])&(f[0])& neg[9]) +((f[2])&(f[1])&(~f[0])& decr[9]);
assign z[10] = ((~f[2])&(~f[1])&(~f[0])&x[10]) + ((~f[2])&(~f[1])&(f[0])& incr[10])+ ((f[2])&(~f[1])&(~f[0])& orop[10]) + ((f[2])&(~f[1])&(f[0])& (~x[10])) + ((~f[2])&(f[1])&(~f[0])& add[10]) + ((~f[2])&(f[1])&(f[0])& neg[10]) +((f[2])&(f[1])&(~f[0])& decr[10]);
assign z[11] = ((~f[2])&(~f[1])&(~f[0])&x[11]) + ((~f[2])&(~f[1])&(f[0])& incr[11])+ ((f[2])&(~f[1])&(~f[0])& orop[11]) + ((f[2])&(~f[1])&(f[0])& (~x[11])) + ((~f[2])&(f[1])&(~f[0])& add[11]) + ((~f[2])&(f[1])&(f[0])& neg[11]) +((f[2])&(f[1])&(~f[0])& decr[11]);
assign z[12] = ((~f[2])&(~f[1])&(~f[0])&x[12]) + ((~f[2])&(~f[1])&(f[0])& incr[12])+ ((f[2])&(~f[1])&(~f[0])& orop[12]) + ((f[2])&(~f[1])&(f[0])& (~x[12])) + ((~f[2])&(f[1])&(~f[0])& add[12]) + ((~f[2])&(f[1])&(f[0])& neg[12]) +((f[2])&(f[1])&(~f[0])& decr[12]);
assign z[13] = ((~f[2])&(~f[1])&(~f[0])&x[13]) + ((~f[2])&(~f[1])&(f[0])& incr[13])+ ((f[2])&(~f[1])&(~f[0])& orop[13]) + ((f[2])&(~f[1])&(f[0])& (~x[13])) + ((~f[2])&(f[1])&(~f[0])& add[13]) + ((~f[2])&(f[1])&(f[0])& neg[13]) +((f[2])&(f[1])&(~f[0])& decr[13]);
assign z[14] = ((~f[2])&(~f[1])&(~f[0])&x[14]) + ((~f[2])&(~f[1])&(f[0])& incr[14])+ ((f[2])&(~f[1])&(~f[0])& orop[14]) + ((f[2])&(~f[1])&(f[0])& (~x[14])) + ((~f[2])&(f[1])&(~f[0])& add[14]) + ((~f[2])&(f[1])&(f[0])& neg[14]) +((f[2])&(f[1])&(~f[0])& decr[14]);
assign z[15] = ((~f[2])&(~f[1])&(~f[0])&x[15]) + ((~f[2])&(~f[1])&(f[0])& incr[15])+ ((f[2])&(~f[1])&(~f[0])& orop[15]) + ((f[2])&(~f[1])&(f[0])& (~x[15])) + ((~f[2])&(f[1])&(~f[0])& add[15]) + ((~f[2])&(f[1])&(f[0])& neg[15]) +((f[2])&(f[1])&(~f[0])& decr[15]);

endmodule

module DFF(out, in, e, reset, clock);
    output out;
    input in;
    input e;
    input reset;
    input clock;
     (*keep = "true"*) reg mem;
    
    always @(posedge clock) 
    begin 
      if (reset) 
        mem <= 1'b0; 
      else if (e) 
        mem <= in; 
    end 
    
    assign out = mem;
endmodule

//module statusDetect(cy,cym1)
