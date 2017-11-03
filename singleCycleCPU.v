module testbench();
	reg[15:0] mem[0:19];
	reg clock, status, reset;
	integer i = 0;
	wire ldReg, ldPC, ldSP, read, write;
	wire S1,S2,S3,S4,S5,S6,S7;
	wire[2:0] functionSelect;

	initial begin
		$dumpfile ("shifter.vcd");
		$dumpvars;
		clock = 0;
		status = 0;
		i = -1;
		mem[0] = 16'b0000000100000000; // PUSH
		mem[1] = 16'b0000100100000000;	// POP
		mem[2] = 16'b0001000100000000;	// ADD
		mem[3] = 16'b0001100100000000;	// NEG
		mem[4] = 16'b0010000100000000;	// OR
		mem[5] = 16'b0010100100000000;	// NOT
		mem[6] = 16'b0011000000000000;	// CALL
		mem[7] = 16'b0011100000000000;	// RETURN
		mem[17]= 16'b0000000000000010;
		mem[18]= 16'b0000000000000100;
		mem[19]= 16'b0000000000000001;  
		#10 reset = 1;
		#23 reset = 0;
	end

	always begin
		#5 clock = ~clock;
	end 
	
	always @(posedge ldPC) begin
		i = i+1;
		if(i > 7) begin
			i = 7;
		end
	end

	controller m1(
	status,clock, reset,mem[i], read, write,
	ldReg,ldPC,ldSP,S1,	S2,	S3, S4, S5,	S6, S7, functionSelect ); 

endmodule

module controller(
	status,clock, reset,IR[15:0], read, write,
	ldReg,ldPC,ldSP,
	S1,	// SP Update {sp+1,sp-1}
	S2,	// M - Input DBus {Reg, PC}
	S3, // M - Address Bus {sp, sp-updated}
	S4, // ALU - YBus {MemOut, PC + 4}
	S5,	// ALU - XBus {RegOut, Label}
	S6, S7, // PC - Update {PC + 4, PC + 4 + Label, M[SP]}
	functionSelect 
	); 
	input status, clock, reset;
	input[15:0] IR;
	output wire ldReg, ldPC, ldSP, read, write;
	output wire S1,S2,S3,S4,S5,S6,S7;
	output wire[2:0] functionSelect;
	reg state;

	assign ldReg = ( (state) && ( ~(|(IR[15:11] ^ 5'b00010)) || ~(|(IR[15:11] ^ 5'b00011)) || ~(|(IR[15:11] ^ 5'b00100)) || ~(|(IR[15:11] ^ 5'b00101)) ) );
	assign ldSP =  (state) &&  (|(IR[15:11] ^ 5'b00000)) ;
	assign ldPC = (state);
	assign write = (state) && ( ~(|(IR[15:11] ^ 5'b00000)) || ~(|(IR[15:11] ^ 5'b00110)) );
	assign read = ~write;
	assign S1 = ( |(IR[15:11] ^ 5'b00000) && |(IR[15:11] ^ 5'b00110) );
	assign S2 = ( |(IR[15:11] ^ 5'b00110) );
	assign S3 = ( |(IR[15:11] ^ 5'b00000) && |(IR[15:11] ^ 5'b00110) );
	assign S4 = ( |(IR[15:14] ^ 2'b01) && |(IR[15:14] ^ 2'b10) && |(IR[15:11] ^ 5'b00110) );
	assign S5 = S4;
	assign S6 = ( |(IR[15:14] ^ 2'b01) && |(IR[15:14] ^ 2'b10) && |(IR[15:11] ^ 5'b00110) && |(IR[15:11] ^ 5'b00111) );
	assign S7 = ( ~(|(IR[15:14] ^ 2'b01)) || ~(|(IR[15:14] ^ 2'b10)) || ~(|(IR[15:11] ^ 5'b00110)) );
	assign functionSelect[2] = ~(|(IR[15:11] ^ 5'b00001)) || ~(|(IR[15:11] ^ 5'b00111));
	assign functionSelect[1] = ~(|(IR[15:11] ^ 5'b00100)) || ~(|(IR[15:11] ^ 5'b00101));
	assign functionSelect[0] = ~(|(IR[15:11] ^ 5'b00011)) || ~(|(IR[15:11] ^ 5'b00101));

	always @(posedge clock or negedge clock) begin
		if(reset == 1) begin
			state = 0;
		end
		else begin
			if(state == 1)begin
				state = 0;
			end
			else begin
				state = 1;
			end
		end
	end
endmodule

	/*	     
module datapath(ldPC,ldFlg,ldSp,ldRegBank,funcSel,s1,s2,s3,s4,s5,s6,s7,IR,condition,reset);
    input ldPC,ldFlg,ldSp,ldRegBank,s1,s2,s3,s4,s5,s6,s7,reset;
    input [2:0] funcSel;
    output [15:0] IR;
    output condition;
    
    wire [15:0] inPC;
    wire [15:0] outPC;
    
    register16bitPC PC(outPC, inPC, 1'b1, reset, ldPC);
    //instructionMemory 

endmodule

module triStateBuffer(enable,inp,out);
	input[15:0] inp;
	input enable;
	output[15:0] out;
	assign out = enable?inp:'bz;
endmodule

module adder(inp1,inp2,outp);
    input [15:0] inp1;
    input [15:0] inp2;
    output [15:0] outp;
    assign outp = inp1 + inp2;
endmodule

module subtractor(inp1,inp2,outp);
    input [15:0] inp1;
    input [15:0] inp2;
    output [15:0] outp;
    assign outp = inp1 - inp2;
endmodule

module register16bitPC(out, in, e, reset, ldSig);
	output [15:0] out;
	input [15:0] in;
	input e;
	input reset;
	input ldSig;
	(*keep = "true"*) reg [15:0] mem;
	always @(posedge ldSig) begin 
		if (reset == 1) 
			mem <= 16'b0; 
		else if (e == 1) 
			mem <= in; 
	end 
	assign out = mem;
endmodule

module register16bitSP(out, in, e, reset, ldSig);
	output [15:0] out;
	input [15:0] in;
	input e;
	input reset;
	input ldSig;
	(*keep = "true"*) reg [15:0] mem;
	always @(posedge ldSig) begin 
		if (reset == 1) 
			mem <= 16'b0; 
		else if (e == 1) 
			mem <= in; 
	end 
	assign out = mem;
endmodule

module demultiplexer1_8 (w,x ,y ,z, e1,e2,e3,e4,e5,e6,e7,e8 );
	output e1, e2 ;
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

module regBank(dataIn, dataOut, regselect, reset, ldRegBank);
	output [15:0] dataOut;
	input [2:0] regselect;    
	input [15:0] dataIn;
	input reset, ldRegBank;

	wire e1,e2,e3,e4,e5,e6,e7,e8;
	wire[15:0] out1;
	wire[15:0] out2;
	wire[15:0] out3;
	wire[15:0] out4;
	wire[15:0] out5;
	wire[15:0] out6;
	wire[15:0] out7;
	wire[15:0] out8;

	demultiplexer1_8 demuxwr(1'b1,regselect[2],regselect[1],regselect[0],e1,e2,e3,e4,e5,e6,e7,e8);

	register16bit r1(out1, dataIn, e1, reset, ldRegBank);
	register16bit r2(out2, dataIn, e2, reset, ldRegBank);
	register16bit r3(out3, dataIn, e3, reset, ldRegBank);
	register16bit r4(out4, dataIn, e4, reset, ldRegBank);
	register16bit r5(out5, dataIn, e5, reset, ldRegBank);
	register16bit r6(out6, dataIn, e6, reset, ldRegBank);
	register16bit r7(out7, dataIn, e7, reset, ldRegBank);
	register16bit r8(out8, dataIn, e8, reset, ldRegBank);

	bit16_mux_8to1 mu(dataOut,out8,out7,out6,out5,out4,out3,out2,out1,regselect); 
endmodule

module mux2to1(inp1,inp2,enable,out);
	input inp1,inp2;
	input enable;
	output out;
	assign out = enable?inp2:inp1;
endmodule

module mux3to1( select, d, q );

    input[1:0] select;
    input[3:0] d;
    output q; 
    reg q;
    wire[1:0] select;
    wire[3:0] d;
    
    always @( select or d )
    begin
       if( select == 0)
          q = d[0];
    
       if( select == 1)
          q = d[1];
    
       if( select == 2)
          q = d[2];
    end
endmodule

module bit16_mux_3to1(out,x0,x1,x2,sel);
	input [15:0] x0,x1,x2;
	input [1:0] sel;
	output [15:0] out;
	generate
		genvar k;
		for(k=0;k<16;k=k+1) begin : wer
			mux3to1 m(sel,{x2[k],x1[k],x0[k]},out[k]);
		end
	endgenerate
endmodule



module mux4to1(a,sel,out);
	input [3:0] a;
	input [1:0] sel;
	output out;
	wire mux_1,mux_2;
	mux2to1 m1 (a[3],a[2],sel[0],mux_1), m2 (a[1],a[4],sel[0],mux_2), m3 (mux_1,mux_2,sel[1],out);
endmodule

module mux8to1(a,sel,out);
	input [7:0] a;
	input [2:0] sel;
	output out;
	wire [2:0] mux;

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
		for(k=0;k<16;k=k+1) begin : wer
			mux8to1 m({x7[k],x6[k],x5[k],x4[k],x3[k],x2[k],x1[k],x0[k]},sel,out[k]);
		end
	endgenerate
endmodule

module ALU(x,y,z,f,cy,cym1);
	input [15:0] x;
	input [15:0] y;
	output [15:0] z;
	input [2:0] f;
	output reg cy;
	output reg cym1;

	wire [15:0] incr;
	wire [15:0] orop;
	wire [15:0] add;
	wire [15:0] neg;
	wire [15:0] decr;

	reg [15:0] temp;
	reg [14:0] temp1;

	assign incr = x + 16'b000000001;
	assign orop = x | y;
	assign add = x + y;
	assign neg = -x;
	assign decr = x - 16'b000000001;

	always @(*) begin
		if(f == 3'b001) begin
			{cy,temp} = x + 16'b000000001;
			{cym1,temp1} = x[14:0] + 15'b00000001;
		end

		else if(f == 3'b010) begin
			{cy,temp} = x + y;
			{cym1,temp1} = x[14:0] + y[14:0];
		end

		else if(f == 3'b110) begin
			{cy,temp} = x - 16'b000000001;
			{cym1,temp1} = x[14:0] - 15'b00000001;
		end

		else begin
			{cy,temp} = x - 16'b000000001;
			{cym1,temp1} = x[14:0] - 15'b00000001;
		end
	end



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

	always @(posedge clock) begin 
		if (reset == 1) 
			mem <= 1'b0; 
		else if (e == 1) 
			mem <= in; 
	end 
	assign out = mem;
endmodule

module statusDetect(cy,cym1,out,z,nz,v,nv,c,nc,s,ns,ldflg,reset);
	input cy;
	input cym1;
	input [15:0] out;
	input reset;
	input ldflg;
	output z,nz,v,nv,c,nc,s,ns;

	wire zeroout,vout;
	zero_detector zd(out,zeroout);
	xor(vout,cy,cym1);

	DFF dz(z,zeroout,1'b1,reset,ldflg);
	DFF dv(v,vout,1'b1,reset,ldflg);
	DFF dc(c,cy,1'b1,reset,ldflg);
	DFF ds(s,out[15],1'b1,reset,ldflg);

	not(nz,z);
	not(nc,c);
	not(ns,s);
	not(nv,v); 
endmodule

module zero_detector(z,status);
	input [15:0] z;
	output status;
	wire t12,t34,t56,t78,t910,t1112,t1314,t1516;
	wire t1234,t5678,t9101112,t13141516;
	wire ta,tb,tc;
	or (t12,z[0],z[1]);
	or (t34,z[2],z[3]);
	or (t56,z[4],z[5]);
	or (t78,z[6],z[7]);
	or (t910,z[8],z[9]);
	or (t1112,z[10],z[11]);
	or (t1314,z[12],z[13]);
	or (t1516,z[14],z[15]);
	or (t1234,t12,t34);
	or (t5678,t56,t78);
	or (t9101112, t910,t1112);
	or (t13141516, t1314, t1516);
	or (ta, t1234,t5678);
	or (tb,t9101112,t13141516);
	or (tc,ta,tb);
	not(status,tc);
endmodule

module mux9to1(a,sel,out);
	input [8:0] a;
	input [3:0] sel;
	output reg out;

	always @(*) begin
		if(sel == 4'b0011) begin out = a[8]; end
		else if(sel == 4'b0100) begin out = a[7]; end
		else if(sel == 4'b0101) begin out = a[6]; end
		else if(sel == 4'b0110) begin out = a[5]; end
		else if(sel == 4'b0001) begin out = a[4]; end
		else if(sel == 4'b0010) begin out = a[3]; end
		else if(sel == 4'b0111) begin out = a[2]; end
		else if(sel == 4'b1000) begin out = a[1]; end
		else if(sel == 4'b0000) begin out = 1; end
		else begin out = 0; end
	end
endmodule

module statusConditionSelection(z,nz,v,nv,c,nc,s,ns,condition,status);
	input z,nz,v,nv,c,nc,s,ns;
	input [3:0] condition;
	output status;

	mux9to1 m({z,nz,v,nv,c,nc,s,ns,1'b1},condition,status);
endmodule 
*/