
`timescale 1ns/10ps

//==============================================================================

module async_mem #(parameter integer A2R)(
   input clk,
   input write,
   input [31:0] address,
   input [31:0] write_data,
   output [31:0] read_data
);

	reg [31:0] mem_data [0:(1<<16)-1];

   assign #A2R read_data = mem_data[ address[17:2] ];  // address to read data delay of A2R-ns

   always @( posedge clk )
      if ( write )
         mem_data[ address[17:2] ] <= write_data;

endmodule


module my_cache(
	input clk,
	input reset,
	input write,
	input [15:0] address,
	input [31:0] write_data,
	input copy,
	output [31:0] read_data,
	output hit
	);
	
	
	wire [31:0] mem_read_data;
	
	reg [44:0] cache_data [0:15];
	
	assign read_data = copy ? mem_read_data : cache_data[ address[3:0] ] [31:0];
	assign hit = (cache_data[ address[3:0] ] [44] == 1'b0) ? 1'b0 : (cache_data [ address[3:0] ] [43:32] == address [15:4]) ? 1'b1 : 1'b0;
	
	integer i;
	
	always @( posedge clk ) begin
		if ( write ) begin
			cache_data[ address[3:0] ] <= {1'b1, address[15:4], write_data};
		end
		if ( copy ) begin
			cache_data [ address[3:0] ] <= {1'b1, address[15:4], mem_read_data};
		end
		if ( reset ) begin
			for (i = 0; i < 16; i = i+1) begin
				cache_data [i] <= 0;
			end
		end
	end
	
	async_mem #(22) dmem			// keep the exact instance name
	(
		.clk		   ( clk ),
		.write		( write ),
		.address	   ( {14'h0000, address, 2'b00} ),
		.write_data	( write_data ),
		.read_data	( mem_read_data )
	);
	
endmodule

//==============================================================================
//`define DEBUG	// comment this line to disable register content writing below
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

// Data Forwarding in Register File for Pipelined Implementation
	assign RD1 = (RR1 == WR && write && RR1) ? WD : reg_data[RR1];
	assign RD2 = (RR2 == WR && write && RR2) ? WD : reg_data[RR2];
	
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

