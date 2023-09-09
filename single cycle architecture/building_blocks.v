
`timescale 1ns/10ps

//==============================================================================

module async_mem(
   input clk,
   input write,
   input [31:0] address,
   input [31:0] write_data,
   output [31:0] read_data
);

	reg [31:0] mem_data [0:(1<<16)-1];

// assign #7 read_data = mem_data[ address[17:2] ];  // address to read data delay of 7ns
   assign    read_data = mem_data[ address[17:2] ];  // zero delay, address to read data

   always @( posedge clk )
      if ( write )
         mem_data[ address[17:2] ] <= write_data;

endmodule


//==============================================================================
`define DEBUG	// comment this line to disable register content writing below
//==============================================================================

module reg_file(
	input  clk,
	input  write,
	input  [ 4:0] WR,
	input  [31:0] WD,
	input  [ 4:0] RR1,
	input  [ 4:0] RR2,
	output [31:0] RD1,
	output [31:0] RD2
	);

	reg [31:0] reg_data [0:31];

	assign RD1 = reg_data[RR1];
	assign RD2 = reg_data[RR2];

// Data Forwarding in Register File for future use
//	assign RD1 = (RR1 == WR && write && RR1) ? WD : reg_data[RR1];
//	assign RD2 = (RR2 == WR && write && RR2) ? WD : reg_data[RR2];
	
	always @(posedge clk) begin
		if(write) begin
			reg_data[WR] <= WD;

			`ifdef DEBUG
			if(WR)
				$display("$%0d = %x", WR, WD);
			`endif
		end
		reg_data[0] <= 32'h00000000;
	end

endmodule

`define ADD 4'b0000
`define SUB 4'b0001
`define SLT 4'b0010
`define SLTU 4'b0011
`define AND 4'b0100
`define OR 4'b0101
`define NOR 4'b0110
`define XOR 4'b0111
`define LUI 4'b1000



module my_alu(
   input  [3:0] Op,
   input  [31:0] A,
   input  [31:0] B,
   output [31:0] javab,
   output        Z
   );

   wire sub = Op != `ADD;
   wire [31:0] bb = sub ? ~B : B;
   wire [32:0] sum = A + bb + sub;
   wire [31:0] sum1 = A + bb + sub;
   wire sltu = ! sum[32];
   
   wire v = sub ?
            ( A[31] != B[31] && A[31] != sum[31] )
          : ( A[31] == B[31] && A[31] != sum[31] );

   wire slt = v ^ sum[31];

   //reg [31:0] javab;

  /* always @( * )
      case( Op )
         `ADD : javab = sum;
         `SUB : javab = sum;
         `SLT : javab = slt;
         `SLTU: javab = sltu;
         `AND : javab = A & B;
         `OR  : javab = A | B;
         `NOR : javab = ~(A | B);
         `XOR : javab = A ^ B;
         `LUI : javab = B << 16;	   
         default : javab = 32'h77777777;
      endcase*/
	assign javab = (Op == `ADD) ? sum1 : (Op == `SUB) ? sum1 :
			(Op == `SLT) ? slt : (Op == `SLTU) ? sltu : (Op == `AND) ? A & B :
			(Op == `OR) ? A | B : (Op == `NOR) ? ~(A | B) : (Op == `XOR) ? A ^ B :
			(Op == `LUI) ? (B << 16) : 32'h77777777;
	

   //assign Javab = javab;
   assign Z = javab == 32'h00000000;

endmodule

module Control_Unit(
	input [5:0] Op,
	input [5:0] Funct,
	output reg RegWrite,
	output reg AluSrc,
	output reg BranchEq,
	output reg BranchNe,	
	output reg MemWrite,
	output reg MemtoReg,
	output reg RegDes,
	output reg [3:0] AluOp,
	output reg SignEx
	);
	always @ (*)
	begin
		case(Op)
			6'b000000 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b0;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b1;
				case(Funct) 
					6'b100000 : AluOp = `ADD ;
					6'b100001 : AluOp = `ADD ;
					6'b100010 : AluOp = `SUB ;
					6'b100011 : AluOp = `SUB ;
					6'b100100 : AluOp = `AND ;
					6'b100101 : AluOp = `OR ;
					6'b100110 : AluOp = `XOR ;
					6'b100111 : AluOp = `NOR ;
					6'b101010 : AluOp = `SLT ;
					6'b101011 : AluOp = `SLTU;
					default : AluOp = 4'bxxxx;
				endcase
				SignEx = 1'bx;
			end
			6'b001000 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `ADD;
				SignEx = 1'b1;
			end
			6'b001001 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `ADD;
				SignEx = 1'b1;
			end
			6'b001010 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `SLT;
				SignEx = 1'b1;
			end
			6'b001011 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `SLTU;
				SignEx = 1'b1;
			end
			6'b001100 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `AND;
				SignEx = 1'b0;
			end	
			6'b001101 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `OR;
				SignEx = 1'b0;
			end
			6'b001110 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `XOR;
				SignEx = 1'b0;
			end
			6'b001111 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b0;
				RegDes = 1'b0;
				AluOp = `LUI;
				SignEx = 1'bx;
			end
			6'b000100 : begin
				RegWrite = 1'b0;
				AluSrc = 1'b0;
				BranchEq = 1'b1;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'bx;
				RegDes = 1'bx;
				AluOp = `XOR;
				SignEx = 1'b1;
			end
			6'b000101 : begin
				RegWrite = 1'b0;
				AluSrc = 1'b0;
				BranchEq = 1'b0;
				BranchNe = 1'b1;
				MemWrite = 1'b0;
				MemtoReg = 1'bx;
				RegDes = 1'bx;
				AluOp = `XOR;
				SignEx = 1'b1;
			end
			6'b100011 : begin
				RegWrite = 1'b1;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b0;
				MemtoReg = 1'b1;
				RegDes = 1'b0;
				AluOp = `ADD;
				SignEx = 1'b1;
			end
			6'b101011 : begin
				RegWrite = 1'b0;
				AluSrc = 1'b1;
				BranchEq = 1'b0;
				BranchNe = 1'b0;
				MemWrite = 1'b1;
				MemtoReg = 1'bx;
				RegDes = 1'bx;
				AluOp = `ADD;
				SignEx = 1'b1;
			end
			default : begin
				RegWrite = 1'bx;
				AluSrc = 1'bx;
				BranchEq = 1'bx;
				BranchNe = 1'bx;
				MemWrite = 1'bx;
				MemtoReg = 1'bx;
				RegDes = 1'bx;
				AluOp = 4'bxxxx;
				SignEx = 1'bx;
			end
		endcase
	end
endmodule
