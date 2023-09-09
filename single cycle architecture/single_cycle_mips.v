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

module single_cycle_mips 
(
	input clk,
	input reset
);
 
	initial begin
		$display("Single Cycle MIPS Implemention");
		$display("Navid Nadertehrani & 98102465");
	end

	reg [31:0] PC;          // Keep PC as it is, its name is used in higher level test bench


	// YOUR DESIGN COMES HERE

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
	wire [31:0] ReadData;
	wire [31:0] Result;	


	assign Imm = (signex) ? {Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15],Instr[15:0]} : ((Instr[15:0] << 16) >>16);
	assign PCPlus4 = PC + 3'b100;
	assign PCBranch = (Imm << 2) + PCPlus4;
	assign WriteReg = (regdes) ? Instr [15:11] : Instr [20:16];
	assign PCSrc = (brancheq & zero) || (branchne & (~zero));
	assign srcB = (alusrc) ? Imm : WriteData;
	//assign ALUResult = srcA + srcB;
	assign Result = (memtoreg) ? ReadData : ALUResult ;


	assign nxt_PC = (PCSrc) ? PCBranch :  PCPlus4;

	always @ (posedge clk)
	begin
		if(reset)
			PC <= 32'h00000000;
		else
			PC <= nxt_PC;
		/*if(regwrite)
				$display("$%0d = %x , %x , %x , %x , %x , %x", WriteReg, Result, alu.Op , alu.bb , ALUResult , alu.javab , alucontrol);*/
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

	Control_Unit ctrlunit(
		.Op				( Instr [31:26] ),
		.Funct			( Instr [5:0] ),
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

endmodule

