
`timescale 1ns/100ps

   `define ADD  4'b0000
   `define SUB  4'b0001
   `define SLT  4'b0010
   `define SLTU 4'b0011
   `define AND  4'b0100
   `define XOR  4'b0101
   `define OR   4'b0110
   `define NOR  4'b0111
   `define LUI  4'b1000

module multi_cycle_mips(

   input clk,
   input reset,

   // Memory Ports
   output  [31:0] mem_addr,
   input   [31:0] mem_read_data,
   output  [31:0] mem_write_data,
   output         mem_read,
   output         mem_write
);

   // Data Path Registers
   reg MRE, MWE;
   reg [31:0] A, B, PC, IR, MDR, MAR;
   
   
   //mflo & mfhi
   reg [31:0] lo,hi;
   
   

   // Data Path Control Lines, donot forget, regs are not always regs !!
   reg setMRE, clrMRE, setMWE, clrMWE;
   reg Awrt, Bwrt, RFwrt, PCwrt, IRwrt, MDRwrt, MARwrt;
   
   reg mltuwrt;

   // Memory Ports Binding
   assign mem_addr = MAR;
   assign mem_read = MRE;
   assign mem_write = MWE;
   assign mem_write_data = B;

   // Mux & ALU Control Lines
   reg [3:0] aluOp;
   reg [1:0] aluSelB;
   reg SgnExt, aluSelA, MemtoReg, RegDst, IorD;
   reg st, mfhi, mflo, link;
   reg [1:0] PCC;

   // Wiring
   wire aluZero;
   wire [31:0] aluResult, rfRD1, rfRD2;

   // Clocked Registers
   always @( posedge clk ) begin
      if( reset )
         PC <= #0.1 32'h00000000;
      else if( PCwrt ) begin
		case( PCC )
			2'b00 : PC <= #0.1 aluResult;
			2'b01 : PC <= #0.1 rfRD1;
			2'b10 : PC <= #0.1 {PC[31:28], IR[25:0], 2'b00};
			default: ;
		endcase
	  end


	  
      if( Awrt ) A <= #0.1 rfRD1;
      if( Bwrt ) B <= #0.1 rfRD2;
	  if(mltuwrt) {hi,lo} <= #0.1 mltu.Product;

	  
      if( MARwrt ) 
		case( PCC )
			2'b00 : MAR <= #0.1 IorD ? aluResult : PC;
			2'b01 : MAR <= #0.1 rfRD1;
			2'b10 : MAR <= #0.1 {PC[31:28], IR[25:0], 2'b00};
			default: ;
		endcase

      if( IRwrt ) IR <= #0.1 mem_read_data;
      if( MDRwrt ) MDR <= #0.1 mem_read_data;

      if( reset | clrMRE ) MRE <= #0.1 1'b0;
          else if( setMRE ) MRE <= #0.1 1'b1;

      if( reset | clrMWE ) MWE <= #0.1 1'b0;
          else if( setMWE) MWE <= #0.1 1'b1;
		  
		  
	
	  //$display("%x %x ", MWE, MAR[11:2]);
	//$display("%x ", Bwrt);
   end

   // Register File
   reg_file rf(
      .clk( clk ),
      .write( RFwrt ),

      .RR1( IR[25:21] ),
      .RR2( IR[20:16] ),
      .RD1( rfRD1 ),
      .RD2( rfRD2 ),
	  
      .WR( link ? 5'h1f : RegDst ? IR[15:11] : IR[20:16] ),
      .WD( link ? PC : mflo ? lo : mfhi ? hi : MemtoReg ? MDR : aluResult )
   );

   // Sign/Zero Extension
   wire [31:0] SZout = SgnExt ? {{16{IR[15]}}, IR[15:0]} : {16'h0000, IR[15:0]};


   // NEW PC for j , jal
   //wire [31:0] new_j_PC = {PC[
   // ALU-A Mux
   //wire [31:0] aluA = aluSelA ? A : PC;
   //reg [31:0] aluA;
   /*always @(*)
   case (aluSelA)
      3'b000: aluA = PC;
      3'b001: aluA = A;
      3'b010: aluA = (PC >> 28) << 28;
	  3'b011: aluA = 32'h00000000;
	  3'b100: aluA = SZout << 16;
   endcase
 

   // ALU-B Mux
   reg [31:0] aluB;
   always @(*)
   case (aluSelB)
      3'b000: aluB = B;
      3'b001: aluB = 32'h4;
      3'b010: aluB = SZout;
      3'b011: aluB = SZout << 2;
	  3'b100: aluB = IR[25:0] << 2;
	  3'b101: aluB = 32'h00000000;
	  3'b110: aluB = hi;
	  3'b111: aluB = lo;
   endcase
   */
   // ALU-A Mux
   wire [31:0] aluA = aluSelA ? A : PC;

   // ALU-B Mux
   reg [31:0] aluB;
   always @(*)
   case (aluSelB)
      2'b00: aluB = B;
      2'b01: aluB = 32'h4;
      2'b10: aluB = SZout;
      2'b11: aluB = SZout << 2;
   endcase
   
   
   multiplier2 mltu(
    .clk    (clk),  
    .start  (st),
    .A    (A),
    .B    (B), 
    .Product( ),
    .ready  ( )
    );

   my_alu alu(
      .A( aluA ),
      .B( aluB ),
      .Op( aluOp ),

      .X( aluResult ),
      .Z( aluZero )
   );


   // Controller Starts Here

   // Controller State Registers
   reg [4:0] state, nxt_state;

   // State Names & Numbers
   localparam
      RESET = 0, FETCH1 = 1, FETCH2 = 2, FETCH3 = 3, DECODE = 4,
      EX_ALU_R = 7, EX_ALU_I = 8, EX_SLT_R = 9,
      EX_LW_1 = 11, EX_LW_2 = 12, EX_LW_3 = 13, EX_LW_4 = 14, EX_LW_5 = 15, EX_MLTU_1 =18,EX_MLTU_2 = 19,
      EX_SW_1 = 21, EX_SW_2 = 22, EX_SW_3 = 23, EX_MF_R = 24,
      EX_BRA_1 = 25, EX_BRA_2 = 26;

   // State Clocked Register 
   always @(posedge clk)
      if(reset)
         state <= #0.1 RESET;
      else
         state <= #0.1 nxt_state;

   task PrepareFetch;
      begin
         IorD = 0;
         setMRE = 1;
         MARwrt = 1;
         nxt_state = FETCH1;
      end
   endtask

   // State Machine Body Starts Here
   always @( * ) begin

      nxt_state = 'bx;

      SgnExt = 'bx; IorD = 'bx;
      MemtoReg = 'bx; RegDst = 'bx;
      aluSelA = 'bx; aluSelB = 'bx; aluOp = 'bx;
	  
	  
      PCwrt = 0;
      Awrt = 0; Bwrt = 0;
      RFwrt = 0; IRwrt = 0;
      MDRwrt = 0; MARwrt = 0;
	  link = 0; mflo = 0; mfhi	 = 0; PCC = 0;
      setMRE = 0; clrMRE = 0;
      setMWE = 0; clrMWE = 0;
	  st = 0; mltuwrt = 0;
	  

      case(state)

         RESET:
            PrepareFetch;

         FETCH1:
            nxt_state = FETCH2;

         FETCH2:
            nxt_state = FETCH3;

         FETCH3: begin
			PCC = 0;
            IRwrt = 1;
            PCwrt = 1;
            clrMRE = 1;
            aluSelA = 0;
            aluSelB = 2'b01;
            aluOp = `ADD;
            nxt_state = DECODE;
         end

         DECODE: begin
            Awrt = 1;
            Bwrt = 1;
            case( IR[31:26] )
               6'b000_000: begin            // R-format
                  case( IR[5:3] )
                     3'b000: ;
                     3'b001: begin//nxt_state = EX_JR_1;
						PCC = 1;
						PCwrt = 1;
						MARwrt = 1;
						setMRE = 1;
						nxt_state = FETCH1;
						if(IR[2:0]) begin
							link = 1;
							RFwrt = 1;
						end
					end
							
						
                     3'b010: nxt_state = EX_MF_R;
                     3'b011: nxt_state = EX_MLTU_1;
                     3'b100: nxt_state = EX_ALU_R;
                     3'b101: nxt_state = EX_SLT_R;
                     3'b110: ;
                     3'b111: ;
                  endcase
				  //RFwrt = 1; 
			   end

               6'b001_000,             // addi
               6'b001_001,             // addiu
               6'b001_010,             // slti
               6'b001_011,             // sltiu
               6'b001_100,             // andi
               6'b001_101,             // ori
               6'b001_110: begin            // xori
                  nxt_state = EX_ALU_I;
				  //RFwrt = 1; 
			   end
			   6'b001_111: begin
				  nxt_state = EX_ALU_I;
			   end

               6'b100_011: begin
                  nxt_state = EX_LW_1;
			  	  //RFwrt = 1; 
			   end

               6'b101_011: begin
                  nxt_state = EX_SW_1;
				  //RFwrt = 0;
			   end

               6'b000_100,
               6'b000_101: begin
				  //RFwrt = 0;
                  nxt_state = EX_BRA_1;
               end 
                  

               // rest of instructiones should be decoded here
			   6'b000_011,
			   6'b000_010: begin
				  MARwrt = 1;
				  setMRE = 1;
				  PCwrt = 1;
				  PCC = 2;
				  nxt_state = FETCH1;
				  if(IR[26]) begin
					RFwrt = 1;
					link = 1;
				  end
			   end
			   
			   
            endcase
         end

         EX_ALU_R: begin
			RFwrt = 1;
            RegDst = 1;
			MemtoReg = 0;
			aluSelA = 1;
            aluSelB = 2'b00;
			case ( IR[2:0] )
				3'b000: aluOp = `ADD;
                3'b001: aluOp = `ADD;
				3'b010: aluOp = `SUB;
				3'b011: aluOp = `SUB;
				3'b100: aluOp = `AND;
				3'b101:	aluOp = `OR;
				3'b110: aluOp = `XOR;
				3'b111: aluOp = `NOR;
            endcase
			PrepareFetch;
         end
		 EX_MF_R: begin
			RFwrt = 1;
            RegDst = 1;
			MemtoReg = 0;
			aluSelA = 1;
            aluSelB = 2'b00;
			case ( IR[2:0] )
				3'b000: mfhi = 1;
				3'b010:	mflo = 1;
				default: ;
            endcase
			PrepareFetch;
         end

		 EX_SLT_R: begin
			RFwrt = 1;
			RegDst = 1;
			MemtoReg = 0;
			aluSelA = 1;
            aluSelB = 2'b00;
			case ( IR[2:0] )
				3'b010: aluOp = `SLT;
				3'b011: aluOp = `SLTU;
			endcase
			PrepareFetch;
		 end
		 
		 
         EX_ALU_I: begin
			RFwrt = 1;
			MemtoReg = 0;
            RegDst = 0;
			aluSelA = 1;
            aluSelB = 2'b10;
			case ( IR[28:26] )
				3'b000: aluOp = `ADD;
                3'b001: aluOp = `ADD;
				3'b010: aluOp = `SLT;
				3'b011: aluOp = `SLTU;
				3'b100: aluOp = `AND;
				3'b101:	aluOp = `OR;
				3'b110: aluOp = `XOR;
				3'b111: aluOp = `LUI;
            endcase
			case ( IR[28:26] )
				3'b000: SgnExt = 1;
                3'b001: SgnExt = 1;
				3'b010: SgnExt = 1;
				3'b011: SgnExt = 0;
				3'b100: SgnExt = 0;
				3'b101:	SgnExt = 0;
				3'b110: SgnExt = 0;	
				3'b111: SgnExt = 0;
			endcase
			PrepareFetch;
         end

         EX_LW_1: begin
            IorD = 1;
			setMRE = 1;
			SgnExt = 1;
			aluSelA = 1;
            aluSelB = 2'b10;
			MARwrt = 1;
            aluOp = `ADD;
			nxt_state = EX_LW_2;
			
         end
		 
		 EX_LW_2: begin
			nxt_state = EX_LW_3;
		 end
         EX_LW_3: begin
			nxt_state = EX_LW_4;
		 end
		 EX_LW_4: begin
			clrMRE = 1;
			MDRwrt = 1;
			nxt_state = EX_LW_5;
			
		 end
		 EX_LW_5: begin
			RegDst = 0;
			MemtoReg = 1;
			RFwrt = 1;
			PrepareFetch;
		 end
		 
		 
		 EX_SW_1: begin
			//RFwrt = 0;
            IorD = 1;
			setMWE = 1;
			SgnExt = 1;
			aluSelA = 1;
            aluSelB = 2'b10;
            aluOp = `ADD;
			MARwrt = 1;
			nxt_state = EX_SW_2;
         end
		 
		 EX_SW_2: begin
			clrMWE = 1;
			nxt_state = EX_SW_3;
		 end
		 
		 EX_SW_3: begin
			PrepareFetch;
		 end
			

         EX_BRA_1: begin
            aluSelA = 1;
            aluSelB = 2'b00;
			aluOp = `XOR;
			SgnExt = 1;
			if( (aluZero && IR[28:26] == 3'b100) || (aluZero == 0 && IR[28:26] == 3'b101))
				nxt_state = EX_BRA_2;
			else
				PrepareFetch;
         end
		 
		 EX_BRA_2: begin
			PCC = 0;
			SgnExt = 1;
			aluSelA = 0;
            aluSelB = 2'b11;
			aluOp = `ADD;
			IorD = 1;
			setMRE = 1;
			PCwrt = 1;
			MARwrt = 1;
			nxt_state = FETCH1;
		 end

            //. . . . .
            //. . . . .
            //. . . . .
		 /*EX_LUI_I: begin
			RFwrt = 1;
			MemtoReg = 0;
            RegDst = 0;
			aluSelA = 3'b100;
            aluSelB = 3'b101;
		 end*/
		
		 EX_MLTU_1: begin
			st = 1;
			nxt_state = EX_MLTU_2;
		 end
		 EX_MLTU_2: begin
			if(mltu.ready) begin
				mltuwrt =1;
				PrepareFetch;
			end
			else
				nxt_state = EX_MLTU_2;
		end
		
      endcase

   end

endmodule

//==============================================================================

module my_alu(
   input [3:0] Op,
   input [31:0] A,
   input [31:0] B,

   output [31:0] X,
   output        Z
);

   wire sub = Op != `ADD;

   wire [31:0] bb = sub ? ~B : B;

   wire [32:0] sum = A + bb + sub;

   wire sltu = ! sum[32];

   wire v = sub ? 
        ( A[31] != B[31] && A[31] != sum[31] )
      : ( A[31] == B[31] && A[31] != sum[31] );

   wire slt = v ^ sum[31];

   reg [31:0] x;

   always @( * )
      case( Op )
         `ADD : x = sum;
         `SUB : x = sum;
         `SLT : x = slt;
         `SLTU: x = sltu;
         `AND : x =   A & B;
         `OR  : x =   A | B;
         `NOR : x = ~(A | B);
         `XOR : x =   A ^ B;
		 `LUI : x = B << 16;
         default : x = 32'hxxxxxxxx;
      endcase

   assign #2 X = x;
   assign #2 Z = x == 32'h00000000;

endmodule

//==============================================================================

module reg_file(
   input clk,
   input write,
   input [4:0] WR,
   input [31:0] WD,
   input [4:0] RR1,
   input [4:0] RR2,
   output [31:0] RD1,
   output [31:0] RD2
);

   reg [31:0] rf_data [0:31];

   assign #2 RD1 = rf_data[ RR1 ];
   assign #2 RD2 = rf_data[ RR2 ];   

   always @( posedge clk ) begin
      if ( write )
         rf_data[ WR ] <= WD;
	
      rf_data[0] <= 32'h00000000;
	  
   end

endmodule

//==============================================================================