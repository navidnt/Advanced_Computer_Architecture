//===========================================================
//
//			Navid Nadertehrani & 98102465
//
//			Implemented Instructions are:
//			R format:  add(u), sub(u), and, or, xor, nor, slt, sltu;
//			I format:  beq, bne, lw, sw, addiu, slti, sltiu, andi, ori, xori, lui.
//
//===========================================================

`timescale 1ns/1ns

`define ADD 4'b0000
`define SUB 4'b0001
`define SLT 4'b0010
`define SLTU 4'b0011
`define AND 4'b0100
`define OR 4'b0101
`define NOR 4'b0110
`define XOR 4'b0111
`define LUI 4'b1000

	

module single_cycle_mips 
(
	input clk,
	input reset
);
 	
	initial begin
		$display("Single Cycle ACA-MIPS Implemention");
		$display("Navid Nadertehrani & 98102465");
	end

	reg [31:0] PC;          // Keep PC as it is, its name is used in higher level test bench

	
	// YOUR DESIGN COMES HERE
	
	wire [31:0] nxt_PC;
	wire [31:0] PC_Copy;
	assign PC_Copy = PC;

	wire [31:0] Instr;
	wire [31:0] Imm;


	reg memtoreg;
	reg memwrite;
	reg brancheq;
	reg branchne;
	reg [3:0] alucontrol;
	reg alusrc;
	reg regdes;
	reg regwrite;
	reg signex;
	
	

	wire [31:0] PCBranch;
	wire [31:0] PCPlus4;
	wire [4:0] WriteReg;
	wire PCSrc;
	wire zero;
	wire [31:0] srcA;
	wire [31:0] srcB;
	wire [31:0] WriteData;
	wire [31:0] ALUResult;
	wire [31:0] ReadData;
	wire [31:0] Result;	


	assign Imm = (signex) ? {Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15:0]} : ((Instr[15:0] << 16) >>16);
	assign PCPlus4 = PC + 3'b100;
	assign PCBranch = (Imm << 2) + PCPlus4;
	assign WriteReg = (regdes) ? Instr [15:11] : Instr [20:16];
	assign PCSrc = (brancheq & zero) || (branchne & (~zero));
	assign srcB = (alusrc) ? Imm : WriteData;
	assign Result = (memtoreg) ? ReadData : ALUResult ;


	assign nxt_PC = (PCSrc) ? PCBranch :  PCPlus4;

	always @ (posedge clk)
	begin
		if(reset)
			PC <= 32'h00000000;
		else
			PC <= nxt_PC;
	end
	
	
	always @ (*)
	begin
		case(Instr [31:26])
			6'b000000 : begin
				regwrite = 1'b1;
				alusrc = 1'b0;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b1;
				case(Instr [5:0]) 
					6'b100000 : alucontrol = 4'b0000 ;
					6'b100001 : alucontrol = 4'b0000 ;
					6'b100010 : alucontrol = 4'b0001 ;
					6'b100011 : alucontrol = 4'b0001 ;
					6'b100100 : alucontrol = 4'b0100 ;
					6'b100101 : alucontrol = 4'b0101 ;
					6'b100110 : alucontrol = 4'b0111 ;
					6'b100111 : alucontrol = 4'b0110 ;
					6'b101010 : alucontrol = 4'b0010 ;
					6'b101011 : alucontrol = 4'b0011;
					default : alucontrol = 4'bxxxx;
				endcase
				signex = 1'bx;
			end
			6'b001000 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b0000;
				signex = 1'b1;
			end
			6'b001001 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b0000;
				signex = 1'b1;
			end
			6'b001010 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b0010;
				signex = 1'b1;
			end
			6'b001011 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b0011;
				signex = 1'b1;
			end
			6'b001100 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b0100;
				signex = 1'b0;
			end	
			6'b001101 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b0101;
				signex = 1'b0;
			end
			6'b001110 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b0111;
				signex = 1'b0;
			end
			6'b001111 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b0;
				regdes = 1'b0;
				alucontrol = 4'b1000;
				signex = 1'bx;
			end
			6'b000100 : begin
				regwrite = 1'b0;
				alusrc = 1'b0;
				brancheq = 1'b1;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'bx;
				regdes = 1'bx;
				alucontrol = 4'b0111;
				signex = 1'b1;
			end
			6'b000101 : begin
				regwrite = 1'b0;
				alusrc = 1'b0;
				brancheq = 1'b0;
				branchne = 1'b1;
				memwrite = 1'b0;
				memtoreg = 1'bx;
				regdes = 1'bx;
				alucontrol = 4'b0111;
				signex = 1'b1;
			end
			6'b100011 : begin
				regwrite = 1'b1;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b0;
				memtoreg = 1'b1;
				regdes = 1'b0;
				alucontrol = 4'b0000;
				signex = 1'b1;
			end
			6'b101011 : begin
				regwrite = 1'b0;
				alusrc = 1'b1;
				brancheq = 1'b0;
				branchne = 1'b0;
				memwrite = 1'b1;
				memtoreg = 1'bx;
				regdes = 1'bx;
				alucontrol = 4'b0000;
				signex = 1'b1;
			end
			default : begin
				regwrite = 1'bx;
				alusrc = 1'bx;
				brancheq = 1'bx;
				branchne = 1'bx;
				memwrite = 1'bx;
				memtoreg = 1'bx;
				regdes = 1'bx;
				alucontrol = 4'bxxxx;
				signex = 1'bx;
			end
		endcase
	end
	
//========================================================== 
//	instantiated modules
//========================================================== 

//	Instruction Memory
	async_mem imem			// keep the exact instance name
	(
		.clk		   (1'b0),
		.write		(1'b0),		// no write for instruction memory
		.address	   (PC_Copy),		   // address instruction memory with pc
		.write_data	('bx),
		.read_data	( Instr )
	);
	
// Data Memory
	async_mem dmem			// keep the exact instance name
	(
		.clk		   ( clk ),
		.write		( memwrite ),
		.address	   ( ALUResult ),
		.write_data	( WriteData ),
		.read_data	( ReadData )
	);

	reg_file regfile(
		.clk			( clk ),
		.write			( regwrite ),
		.WR				( WriteReg ),
		.WD				( Result ),
		.RR1			( Instr [25:21] ),
		.RR2			( Instr [20:16] ),
		.RD1			( srcA ),
		.RD2			( WriteData )
	);
	
	my_alu alu
	(
		.Op			( alucontrol ),
   		.A			( srcA ),
   		.B			( srcB ),
   		.javab			( ALUResult ),
   		.Z			( zero )
   	);
	
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

	   
		assign javab = (Op == `ADD) ? sum1 : (Op == `SUB) ? sum1 :
				(Op == `SLT) ? slt : (Op == `SLTU) ? sltu : (Op == `AND) ? A & B :
				(Op == `OR) ? A | B : (Op == `NOR) ? ~(A | B) : (Op == `XOR) ? A ^ B :
				(Op == `LUI) ? (B << 16) : 32'h77777777;
		

		assign Z = javab == 32'h00000000;

	endmodule

endmodule

	
