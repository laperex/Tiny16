

module alu16 #(parameter WIDTH = 16) (clk, reset, A, B, E, L, LS, Cn, PI, PS, psr);
	input [WIDTH - 1:0] A, B;
	input [3:0] L;
	input Cn;
	input clk;
	input reset;
	input LS;
	input PI, PS;

	output [WIDTH - 1:0] E;
	output reg [3:0] psr;

	wire [WIDTH - 1:0] lA;
	wire [WIDTH - 1:0] lB;
	wire [WIDTH:0] tE;

	genvar i;

	for (i = 0; i < WIDTH; i = i + 1)
		assign lB[i] = A[i] ? (B[i] ? L[3] : L[2]) : (B[i] ? L[1] : L[0]);

	assign lA = LS ? 0: A;
	assign tE = { 1'b0, lA } + { 1'b0, lB } + (1 & Cn);
	assign E = tE[WIDTH - 1:0];

	// assign psr_wire[3] = (E[WIDTH - 1] & ~A[WIDTH - 1] & (lB[WIDTH - 1] == 0)) | (~E[WIDTH - 1] & A[WIDTH - 1] & lB[WIDTH - 1]);

	always @(posedge clk or negedge reset) begin
		if (reset == 1) begin
			psr <= 0;
		end
		else
		if (PI == 1) begin
			psr[0] <= tE[WIDTH];
			psr[1] <= E[WIDTH - 1];
			psr[2] <= (E[WIDTH - 1: 0] == 0) ? 1: 0;
			psr[3] <= 0;

			// if (PS == 1) begin
			// end
			// else begin
			//     psr <= psr_wire;
			// end
		end
	end
endmodule


module sync_mux_2x1 #(parameter WIDTH = 16) (clk, reset, sel, enable, in0, in1, out);
	input sel, enable, clk, reset;
	input [WIDTH - 1:0] in0, in1;

	output reg [WIDTH - 1:0] out;

	always @(posedge clk or posedge reset) begin
		if (reset == 1)
			out = 0;
		else
		if (enable) begin
			if (sel == 1)
				out = in1;
			else
				out = in0;
		end
	end
endmodule


module memory #(parameter WIDTH = 16) (clk, reset, MI, RI, write, read, mar);
	input clk, MI, RI;
	input reset;
	
	output [WIDTH - 1:0] mar;
	input [WIDTH - 1:0] write;
	output [WIDTH - 1:0] read;
	
	reg [WIDTH - 1:0] mem_addr_reg;
	reg [WIDTH - 1:0] ram [2**WIDTH - 1:0];
	
	integer i;

	assign mar = mem_addr_reg;
	
	parameter imm = 0 << 6;
	parameter abs = 1 << 6;
	parameter off = 2 << 6;

	parameter loadi	= 	5'b00001;
	parameter loada =	loadi | abs;

	parameter store = 	5'b00010 | abs;

	parameter jump 	= 	5'b00100 | abs;
	parameter jumpz =	5'b00101 | abs;
	parameter jumpc =	5'b00110 | abs;
	parameter jumpn =	5'b00111 | abs;

	parameter add 	= 	6'b100000;
	parameter adc 	= 	6'b100001;
	parameter sbb 	= 	6'b100010;
	parameter sub 	= 	6'b100011;

	parameter adda 	= 	add | abs;
	parameter adca 	= 	adc | abs;
	parameter sbba 	= 	sbb | abs;
	parameter suba 	= 	sub | abs;
		
	parameter tmp = 10;
	parameter first = 11;
	parameter second = 12;
	parameter count = 13;

	integer org, start, finish;

	wire [WIDTH - 1:0] display;
	

	always @(posedge clk or posedge reset) begin
		if (reset == 1) begin
			start = 100;
			finish = 200;
			
			mem_addr_reg <= 0;

			for (i = 0; i < 2**(WIDTH - 1); i++)
				ram[i] = 0;
			
			org = start;

			ram[0] = org - 1;

			ram[first] = 0;
			ram[second] = 1;
			ram[count] = 5;

			start = org++;

			ram[start] = loada; ram[org++] = count;
			ram[org++] = sbb; ram[org++] = 0;
			ram[org++] = store; ram[org++] = count;

			ram[org++] = jumpz; ram[org++] = finish - 1;
			
			ram[org++] = loada; ram[org++] = second;
			ram[org++] = store; ram[org++] = tmp;

			ram[org++] = adda; ram[org++] = first;
			ram[org++] = store; ram[org++] = second;

			ram[org++] = loada; ram[org++] = tmp;
			ram[org++] = store; ram[org++] = first;
			
			ram[org++] = jump; ram[org++] = start - 1;

			ram[finish] = 8'hff;
		end else begin
			if (MI == 1)
				mem_addr_reg <= write;

			if (RI == 1)
				ram[mem_addr_reg] <= write;
		end
	end

	assign read = ram[mem_addr_reg];
	assign display = ram[first];
endmodule


module controlunit(clk, reset, inst, psr, cAI, cAS, cPI, cPS, cMI, cRI, cES, cCn, cL, cLS, cII, decode, counter);
	// Enable Control Lines

	parameter AI = 1 << 0;
	parameter AS = 1 << 1;
	parameter PI = 1 << 2;
	parameter PS = 1 << 3;
	parameter MI = 1 << 4;
	parameter RI = 1 << 5;
	parameter ES = 1 << 6;
	parameter Cn = 1 << 7;
	parameter LS = 1 << 8;
	parameter II = 1 << 9;

	// ALU Control Lines

	parameter GEN_0 = 14'b0000_0000000000 | LS;
	parameter GEN_1 = 14'b1111_0000000000 | LS;
	parameter BUF_A = 14'b1100_0000000000;
	parameter NEG_A = 14'b0011_0000000000;
	parameter BUF_B = 14'b1010_0000000000;
	parameter NEG_B = 14'b0101_0000000000;

	parameter load 	= 	6'b000001;
	parameter loadi = 	6'b000001;

	parameter store = 	6'b000010;

	parameter jump 	= 	6'b000100;
	parameter jumpz =	6'b000101;
	parameter jumpc =	6'b000110;
	parameter jumpn =	6'b000111;

	parameter add 	= 	6'b100000;
	parameter addc 	= 	6'b100001;
	parameter subb 	= 	6'b100010;
	parameter sub 	= 	6'b100011;

	input clk;
	input reset;
	input [7:0] inst;
	input [3:0] psr;

	output wire cAI, cAS;
	output wire cPI, cPS;
	output wire cMI, cRI;
	output wire cES;
	output wire cCn;
	output wire cLS;
	output wire cII;
	output wire [3:0] cL;

	output [15:0] decode;
	output reg [2:0] counter;

	wire TYPE_REG, TYPE_IMM, TYPE_ABS, TYPE_OFF;
	wire [5:0] opcode;

	assign TYPE_IMM = ~inst[7] & ~inst[6];	// 0
	assign TYPE_ABS = ~inst[7] & inst[6];	// 1
	assign TYPE_OFF = inst[7] & ~inst[6];	// 2
	assign TYPE_REG = inst[7] & inst[6];	// 3

	assign opcode = inst[5:0];

	always @(negedge clk or posedge reset) begin
		if (reset == 1) begin
			counter <= 0;
		end else begin
			counter <= counter + 1;
		end
	end
	
	assign decode =
		counter == 0 ? GEN_0 | MI:
		counter == 1 ? BUF_B | Cn | MI | RI | LS:
		counter == 2 ? II | GEN_0 | MI:
		counter == 3 ? BUF_B | Cn | MI | RI | LS:
		counter == 4 ?
			TYPE_IMM ?
				opcode == load ? AI:

				opcode == add ? BUF_B | PI | AS | AI:
				opcode == addc ? BUF_B | PI | AS | AI | Cn:
				opcode == subb ? NEG_B | PI | AS | AI:
				opcode == sub ? NEG_B | PI | AS | AI | Cn:
				0:
			TYPE_ABS ?
				opcode == load ? BUF_B | MI | LS:

				opcode == store ? BUF_B | MI | LS:

				((opcode == jump) |
				((opcode == jumpz) & (psr[2] == 1)) |
				((opcode == jumpc) & (psr[0] == 1)) |
				((opcode == jumpn) & (psr[1] == 1))) ? GEN_0 | MI | AI:

				opcode == add ? BUF_B | MI | LS:
				opcode == addc ? BUF_B | MI | LS:
				opcode == sub ? BUF_B | MI | LS:
				opcode == subb ? BUF_B | MI | LS:
				0:
			0:
		counter == 5 ?
			TYPE_ABS ?
				opcode == load ? AI:

				opcode == store ? BUF_A | RI | LS:

				((opcode == jump) |
				((opcode == jumpz) & (psr[2] == 1)) |
				((opcode == jumpc) & (psr[0] == 1)) |
				((opcode == jumpn) & (psr[1] == 1))) ? BUF_A | MI | RI | LS:
				
				opcode == add ? BUF_B | PI | AS | AI:
				opcode == addc ? BUF_B | PI | AS | AI | Cn:
				opcode == subb ? NEG_B | PI | AS | AI:
				opcode == sub ? NEG_B | PI | AS | AI | Cn:
				0:
			0:
		0;
	
	always @(decode == 0) begin
		counter <= 0;
	end

	assign cAI = decode[0];
	assign cAS = decode[1];
	assign cPI = decode[2];
	assign cPS = decode[3];
	assign cMI = decode[4];
	assign cRI = decode[5];
	assign cES = decode[6];
	assign cCn = decode[7];
	assign cLS = decode[8];
	assign cII = decode[9];
	assign cL = decode[14:10];
endmodule

module processor #(parameter WIDTH = 16)(clk, reset, halt);
	wire [WIDTH - 1:0] mem_input_bus;
	wire [WIDTH - 1:0] accumulator_bus;
	wire [WIDTH - 1:0] ram_read_bus;
	wire [WIDTH - 1:0] ram_write_bus;
	wire [WIDTH - 1:0] alu_out_bus;
	wire [3:0] psr_bus;

	input clk;
	input reset;
	output reg halt;

	reg [7:0] inst;

	wire AI, AS;
	wire PI, PS;
	wire MI, RI;
	wire ES;
	wire C_in;
	wire LS;
	wire II;
	wire [3:0] L;

	alu16 #(WIDTH) alu (clk, reset, accumulator_bus, ram_read_bus, alu_out_bus, L, LS, C_in, PI, PS, psr_bus);

	// ES == 0 -> ALU
	// ES == 1 -> PSR
	assign ram_write_bus = ES ? psr_bus: alu_out_bus;

	sync_mux_2x1 #(WIDTH) rega_mux(clk, reset, AS, AI, ram_read_bus, ram_write_bus, accumulator_bus);

	wire [WIDTH - 1:0] mar;
	memory #(WIDTH) ram(clk, reset, MI, RI, ram_write_bus, ram_read_bus, mar);

	wire [15:0] decode;
	wire [2:0] counter;
	controlunit cu(clk, reset, inst, psr_bus, AI, AS, PI, PS, MI, RI, ES, C_in, L, LS, II, decode, counter);

	always @(posedge clk or posedge reset) begin
		if (reset == 1)
			inst <= 0;
		else if (II == 1) begin
			inst <= ram_read_bus;
			halt <= 0;
		end
	end

	always @(posedge clk & inst == 2**WIDTH - 1) begin
		halt <= 1;
	end
endmodule


module testbench;
	reg clk;
	reg reset;
	wire halt;

	processor #(8) tiny(clk, reset, halt);
	
	initial begin
		clk = 0;
		reset = 1;

		forever begin
			#1 clk = ~clk;
			if (reset == 1 & clk)
				reset = 0;
			else if (halt)
				$finish;
		end
	end

	initial begin
		$dumpfile ("testbench.vcd");
		$dumpvars (0, testbench);

		$monitor ($time, ":\tclk=%B | reset=%B | halt=%B", clk, reset, halt);

		#2000
		$finish;
	end
endmodule