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
    
    else
    {cy,temp} = x - 16'b000000001;
    {cym1,temp1} = x[14:0] - 15'b00000001;
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
