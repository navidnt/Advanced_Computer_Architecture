//===========================================================
//
//			Navid Nadertehrani & 98102465
//
//			Implemented Instructions are:
//			R format:  add(u), sub(u), and, or, xor, nor, slt, sltu;
//			I format:  beq, bne, lw, sw, addi(u), slti, sltiu, andi, ori, xori, lui.
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

module pipelined_mips 
(
	input clk,
	input reset
);
 
	initial begin
		$display("Pipelined ACA-MIPS Implemention");
		$display("Navid Nadertehrani & 98102465");
	end

	reg [31:0] PC;          // Keep PC as it is, its name is used in higher level test bench
	
	

	// YOUR DESIGN COMES HERE
	
	// Hazard Unit variables
	wire [1:0] ForwardAE;
	wire [1:0] ForwardBE;
	wire lwstall;
	
	
	// Single cycle variables
	
	wire [31:0] dmemRd;
	wire [31:0] regfileRd1;
	wire [31:0] regfileRd2;
	
	
	wire [31:0] nxt_PC;
	wire [31:0] PC_Copy;
	assign PC_Copy = PC;

	wire [31:0] Instr;
	wire [31:0] Imm;


	wire memtoreg;
	wire memwrite;
	wire brancheq;
	wire branchne;
	wire [3:0] alucontrol;
	wire alusrc;
	wire regdes;
	wire regwrite;
	wire signex;
	


	wire [31:0] PCBranch;
	wire [31:0] PCPlus4;
	wire [4:0] WriteReg;
	wire PCSrc;
	wire zero;
	wire [31:0] srcA;
	wire [31:0] srcB;
	wire [31:0] WriteData;
	wire [31:0] ALUResult;
	wire [31:0] Result;	
	
	// Pipeline Registers
	
	reg [31:0] lv1Instr;
	reg [31:0] lv1PCPlus4;
	
	//reg [10:0] lv2Control;
	reg [31:0] lv2rd1;
	reg [31:0] lv2rd2;
	reg [4:0] lv2rs;
	reg [4:0] lv2rt;
	reg [4:0] lv2rd;
	reg [31:0] lv2signImm;
	reg [31:0] lv2PCPlus4;
	reg lv2RegWrite;
	reg lv2MemtoReg;
	reg lv2MemWrite;
	reg [3:0] lv2ALUcontrol;
	reg lv2ALUSrc;
	reg lv2RegDes;
	reg lv2Branchne;
	reg lv2Brancheq;
	
	
	//reg [2:0] lv3Control;
	reg [31:0] lv3ALUOut;
	reg [31:0] lv3wd;
	reg [4:0] lv3wr;
	reg lv3RegWrite;
	reg lv3MemtoReg;
	reg lv3MemWrite;
	
	//reg [1:0] lv4Control;
	reg [31:0] lv4rd;
	reg [31:0] lv4ALUOut;
	reg [4:0] lv4wr;
	reg lv4RegWrite;
	reg lv4MemtoReg;

	
	//DESIGN
	
	assign Imm = (signex) ? {lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15],lv1Instr[15:0]} : ((lv1Instr[15:0] << 16) >>16);
	assign PCPlus4 = PC + 3'b100;
	assign PCBranch = (lv2signImm << 2) + lv2PCPlus4;
	assign WriteReg = (lv2RegDes) ? lv2rd : lv2rt;
	assign PCSrc = (lv2Brancheq & zero) || (lv2Branchne & (~zero));
	assign WriteData = (ForwardBE [1]) ? lv3ALUOut : (ForwardBE [0]) ? Result : lv2rd2;
	assign srcA = (ForwardAE [1]) ? lv3ALUOut : (ForwardAE [0]) ? Result : lv2rd1;
	assign srcB = (lv2ALUSrc) ? lv2signImm : WriteData; //TODO WriteData
	assign Result = (lv4MemtoReg) ? lv4rd : lv4ALUOut ;
	assign nxt_PC = (PCSrc) ? PCBranch :  PCPlus4;

	
	always @ (posedge clk)
	begin
		if(reset)
			begin
				PC <= 32'h00000000;
				lv1Instr <= 32'h00000000;
				lv2RegWrite <= 1'b0;
				lv2MemtoReg <= 1'b0;
				lv2MemWrite <= 1'b0;
				lv2ALUcontrol <= 4'h0;
				lv2ALUSrc <= 1'b0;
				lv2RegDes <= 1'b0;
				lv2Brancheq <= 1'b0;
				lv2Branchne <= 1'b0;
			end
		else
			begin
				PC <= (lwstall) ? PC : nxt_PC;
			
				lv1Instr <= (lwstall) ? lv1Instr :(PCSrc) ? 32'h00000000 : Instr;
				lv1PCPlus4 <= (lwstall) ? lv1PCPlus4 :(PCSrc) ? 32'h00000000 : PCPlus4;
				
				lv2rd1 <= regfileRd1;	
				lv2rd2 <= regfileRd2;
				lv2rs <= lv1Instr [25:21];
				lv2rt <= lv1Instr [20:16];
				lv2rd <= lv1Instr [15:11];
				lv2signImm <= Imm;
				lv2PCPlus4 <= lv1PCPlus4;
				lv2RegWrite <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : regwrite;
				lv2MemtoReg <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : memtoreg;
				lv2MemWrite <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : memwrite;
				lv2ALUcontrol <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : alucontrol;
				lv2ALUSrc <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : alusrc;
				lv2RegDes <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : regdes;
				lv2Brancheq <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : brancheq;
				lv2Branchne <= (lwstall) ? 1'b0 :(PCSrc) ? 1'b0 : branchne;
				
				lv3ALUOut <= ALUResult;
				lv3wd <= WriteData;
				lv3wr <= WriteReg;
				lv3RegWrite <= lv2RegWrite;
				lv3MemtoReg <= lv2MemtoReg;
				lv3MemWrite <= lv2MemWrite;
				
				lv4rd <= dmemRd;
				lv4ALUOut <= lv3ALUOut;
				lv4wr <= lv3wr;
				lv4RegWrite <= lv3RegWrite;
				lv4MemtoReg <= lv3MemtoReg;
			end
		
		/*if(regwrite)
				$display("$%0d = %x , %x , %x , %x , %x , %x", WriteReg, Result, alu.Op , alu.bb , ALUResult , alu.javab , alucontrol);*/
	end
	
//========================================================== 
//	instantiated modules
//========================================================== 

//	Instruction Memory
	async_mem #(0) imem			// keep the exact instance name
	(
		.clk		   (1'b0),
		.write		(1'b0),		// no write for instruction memory
		.address	   ( PC_Copy ),		   // address instruction memory with pc
		.write_data	(32'bx),
		.read_data	( Instr )
	);
	
// Data Memory
	async_mem #(0) dmem			// keep the exact instance name
	(
		.clk		   ( clk ),
		.write		( lv3MemWrite ),
		.address	   ( lv3ALUOut ),
		.write_data	( lv3wd ),
		.read_data	( dmemRd )
	);
	
// ALU
	my_alu alu
	(
		.Op			( lv2ALUcontrol ),
   		.A			( srcA ),
   		.B			( srcB ),
   		.javab			( ALUResult ),
   		.Z			( zero )
	);

// Control Unit
	Control_Unit ctrlunit(
		.Op				( lv1Instr [31:26] ),
		.Funct			( lv1Instr [5:0] ),
		.RegWrite		( regwrite ),
		.AluSrc			( alusrc ),
		.BranchEq		( brancheq ),
		.BranchNe		( branchne ),	
		.MemWrite		( memwrite ),
		.MemtoReg		( memtoreg ),
		.RegDes			( regdes ),
		.AluOp			( alucontrol ),
		.SignEx			( signex )
	);

// Register File
	reg_file regfile(
		.clk			( clk ),
		.write			( lv4RegWrite ),
		.WR				( lv4wr ),
		.WD				( Result ),
		.RR1			( lv1Instr [25:21] ),
		.RR2			( lv1Instr [20:16] ),
		.RD1			( regfileRd1 ),
		.RD2			( regfileRd2 )
	);
	
	Hazard_Unit hazard_unit(
		.lv4RegWrite			( lv4RegWrite ),
		.lv3RegWrite			( lv3RegWrite ),
		.lv2MemtoReg			( lv2MemtoReg ),
		.lv4WriteReg			( lv4wr ),
		.lv3WriteReg			( lv3wr ),
		.lv2rs					( lv2rs ),
		.lv2rt					( lv2rt ),
		.lv1rs					( lv1Instr [25:21] ),
		.lv1rt					( lv1Instr [20:16] ),
		.ForwardAE				( ForwardAE ),
		.ForwardBE				( ForwardBE ),
		.lwstall				( lwstall )
	);
	
endmodule

module Hazard_Unit(
	input lv4RegWrite,
	input lv3RegWrite,
	input lv2MemtoReg,
	input [4:0] lv4WriteReg,
	input [4:0] lv3WriteReg,
	input [4:0] lv2rs,
	input [4:0] lv2rt,
	input [4:0] lv1rs,
	input [4:0] lv1rt,
	output reg [1:0] ForwardAE,
	output reg [1:0] ForwardBE,
	output reg lwstall
	);
	
	
	/*assign StallD = lwstall;
	assign StallF = lwstall;
	assign FlushE = lwstall;
	*/
	
	always @ (*)
	begin
		if(lv2rs != 5'b00000 && lv2rs == lv3WriteReg && lv3RegWrite)
			ForwardAE = 2'b10;
		else if (lv2rs != 5'b00000 && lv2rs == lv4WriteReg && lv4RegWrite)
			ForwardAE = 2'b01;
		else
			ForwardAE = 2'b00;
			
		if(lv2rt != 5'b00000 && lv2rt == lv3WriteReg && lv3RegWrite)
			ForwardBE = 2'b10;
		else if (lv2rt != 5'b00000 && lv2rt == lv4WriteReg && lv4RegWrite)
			ForwardBE = 2'b01;
		else
			ForwardBE = 2'b00;
		if( ((lv1rs == lv2rt) || (lv1rt == lv2rt)) && lv2MemtoReg )
			lwstall = 1'b1;
		else
			lwstall = 1'b0;
	end
	
	
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


   //assign Javab = javab;
   assign Z = javab == 32'h00000000;

endmodule
