

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

    always @(posedge clk or posedge reset) begin
        if (reset == 1) begin
            psr <= 0;
        end
        else
        if (PI == 1) begin
            if (PS == 1) begin
                psr <= tE[3:0];
            end
            else begin
                psr <= {
					tE[16],
					tE[WIDTH - 1],
					tE,
					(E[WIDTH - 1] & ~A[WIDTH - 1] & ~lB[WIDTH - 1]) | (~E[WIDTH - 1] & A[WIDTH - 1] & lB[WIDTH - 1])
				};
            end	
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
    reg [WIDTH - 1:0] ram [2**16-1:0];
    
    integer i;

    assign mar = mem_addr_reg;
	
	// assign TYPE_IMM = ~inst[7] & ~inst[6];	// 0
	// assign TYPE_ABS = ~inst[7] & inst[6];	// 1
	// assign TYPE_OFF = inst[7] & ~inst[6];	// 2
	// assign TYPE_REG = inst[7] & inst[6];	// 3
	
	parameter imm = 0 << 6;
	parameter abs = 1 << 6;
	parameter off = 2 << 6;

	parameter loadi	= 	5'b00001;
	parameter loada =	loadi | abs;

	parameter store = 	5'b00010 | abs;

	parameter jump 	= 	5'b00100;
	parameter jumpz =	5'b00101;
	parameter jumpc =	5'b00110;
	parameter jumpn =	5'b00111;

	parameter add 	= 	5'b01000;
	parameter adc 	= 	5'b01001;
	parameter sbb 	= 	5'b01010;
	parameter sub 	= 	5'b01011;
	
	parameter adda 	= 	add | abs;
	parameter adca 	= 	adc | abs;
	parameter sbba 	= 	sbb | abs;
	parameter suba 	= 	sub | abs;
		
	parameter tmp = 10;
	parameter first = 11;
	parameter second = 12;
	parameter count = 13;

	integer org;
	
	integer start, finish;

    always @(posedge clk or posedge reset) begin
        if (reset == 1) begin
            mem_addr_reg <= 0;

            for (i = 0; i < 2**(WIDTH - 1); i++)
                ram[i] = 0;
			
			org = 100;

			ram[0] = org - 1;

			ram[first] = 0;
			ram[second] = 1;
			ram[count] = 10;

			start = org++;
			ram[start] = loada;
			ram[org++] = count;
			ram[org++] = sub;
			ram[org++] = 10;

			finish = org++;
			ram[finish] = 8'hff;
			ram[org++] = 4;
        end
        else
        begin
            if (MI == 1)
                mem_addr_reg <= write;

            if (RI == 1)
                ram[mem_addr_reg] <= write;
        end
    end

    assign read = ram[mem_addr_reg];
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

module controlunit(clk, reset, inst, cAI, cAS, cPI, cPS, cMI, cRI, cES, cCn, cL, cLS, cII, decode, counter);
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

	// ALU Control Liines

    parameter GEN_0 = 14'b0000_0000000000 | LS;
    parameter GEN_1 = 14'b1111_0000000000 | LS;
    parameter BUF_A = 14'b1100_0000000000;
    parameter NEG_A = 14'b0011_0000000000;
    parameter BUF_B = 14'b1010_0000000000;
    parameter NEG_B = 14'b0101_0000000000;

    // parameter NAND = 14'b0111_0000000000 | LS;
    // parameter AND = 14'b1000_0000000000 | LS;
    // parameter OR = 14'b1110_0000000000 | LS;
    // parameter NOR = 14'b0001_0000000000 | LS;
    // parameter XOR = 14'b0110_0000000000 | LS;
    // parameter XNOR = 14'b1001_0000000000 | LS;

	// parameter INC_A = BUF_A | Cn | LS;
	// parameter DEC_A = NEG_A | LS;

	// parameter INC_B = BUF_B | Cn | LS;
	// parameter DEC_B = NEG_B | LS;

    // parameter ADD = BUF_B;
	// parameter ADC = BUF_B | Cn;
	// parameter SUB = NEG_B | Cn;
	// parameter SBB = NEG_B;
	
	// parameter INST_IMM = 1;
	// parameter INST_ABS = 2;
	// parameter INST_OFF = 3;
	
	// // Instructions
	// parameter NOP = 8'b0;
	
	// parameter INST_TYPE_REG = 0;
	// parameter INST_TYPE_IMM = 1;
	// parameter INST_TYPE_ABS = 2;
	// parameter INST_TYPE_OFF = 3;

	// ALU INT - ADDRESSING_TYPE[2] - INST_TYPE[]

	parameter load 	= 	6'b000001;
	parameter loadi 	= 	6'b000001;

	parameter store = 	6'b000010;

	parameter jump 	= 	6'b000100;
	parameter jumpz =	6'b000101;
	parameter jumpc =	6'b000110;
	parameter jumpn =	6'b000111;

	parameter add 	= 	6'b001000;
	parameter addc 	= 	6'b001001;
	parameter subb 	= 	6'b001010;
	parameter sub 	= 	6'b001011;

	input clk;
    input reset;
    input [7:0] inst;

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

	reg reset_counter;

	assign TYPE_IMM = ~inst[7] & ~inst[6];	// 0
	assign TYPE_ABS = ~inst[7] & inst[6];	// 1
	assign TYPE_OFF = inst[7] & ~inst[6];	// 2
	assign TYPE_REG = inst[7] & inst[6];	// 3

	assign opcode = inst[5:0];

    always @(negedge clk or posedge reset) begin
        if (reset == 1) begin
            counter <= 0;
        end else begin
			// case (counter)
			// 	4'h0:
			// 		decode <= GEN_0 | MI;

			// 	4'h1:
			// 		decode <= BUF_B | Cn | MI | RI | LS;

			// 	4'h2:
			// 		decode <= II | GEN_0 | MI;

			// 	4'h3:
			// 		decode <= BUF_B | Cn | MI | RI | LS;

			// 	4'h4:
			// 		decode <=
			// 			TYPE_IMM ? 
			// 				(opcode == load) ? AI : ((opcode == add) ? BUF_B | AS | AI : 0)
			// 				// (opcode == add) ? 
			// 			: 0;
			// endcase

			// if (counter == 5)
			// 	counter <= 0;
			// else

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
				opcode == add ? BUF_B | PI | AS | AI | AS:
				opcode == addc ? BUF_B | PI | AS | AI | AS | Cn:
				opcode == subb ? NEG_B | PI | AS | AI | AS:
				opcode == sub ? NEG_B | PI | AS | AI | AS | Cn:
				0:
			TYPE_ABS ?
				opcode == load ? BUF_B | MI | LS:
				opcode == add ? BUF_B | MI | LS:
				opcode == addc ? BUF_B | MI | LS:
				opcode == sub ? BUF_B | MI | LS:
				opcode == subb ? BUF_B | MI | LS:
				0:
			0:
		counter == 5 ?
			TYPE_ABS ?
				opcode == load ? AI:
				opcode == add ? BUF_B | PI | AS | AI:
				opcode == addc ? BUF_B | PI | AS | AI | Cn:
				opcode == subb ? NEG_B | PI | AS | AI:
				opcode == sub ? NEG_B | PI | AS | AI | Cn:
				0:
			0:
		0;

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


module testbench;
	parameter WIDTH = 8;

    wire [WIDTH - 1:0] mem_input_bus;
    wire [WIDTH - 1:0] accumulator_bus;
    wire [WIDTH - 1:0] ram_read_bus;
    wire [WIDTH - 1:0] ram_write_bus;
    wire [WIDTH - 1:0] alu_out_bus;
    wire [3:0] psr_bus;

    reg clk;
    reg reset;
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
    controlunit cu(clk, reset, inst, AI, AS, PI, PS, MI, RI, ES, C_in, L, LS, II, decode, counter);

	always @(posedge clk or negedge reset) begin
		if (reset == 1)
			inst <= 0;
		else if (II == 1)
			inst <= ram_read_bus;
	end

    always @(posedge reset) begin
        inst <= 0;
    end
	
    initial begin
		inst = 0;
        clk = 0;
        reset = 1;
		
        while (~inst) begin
            #10 clk = ~clk;
            if (reset == 1 & clk)
                reset = 0;
        end
    end

    initial begin
        $dumpfile ("testbench.vcd");
        $dumpvars (0, testbench);
        $monitor ($time, ":\tclk=%B | reset=%B | step=%D | RAM_WRITE=%B:%D | RAM_READ=%B:%D | MAR=%B:%D | REG=%B:%D | decode=%B | Inst=%B", clk, reset, counter, ram_write_bus, ram_write_bus, ram_read_bus, ram_read_bus, mar, mar, accumulator_bus, accumulator_bus, decode, inst);
        // $monitor ($time, ":\tclk=%B | A=%B | B=%B | C=%B", clk, a, b, c);
        // #10 MI = 1; RI = 0; ram_write_bus = 1;
        // #20 MI = 1; RI = 0; ram_write_bus = 2;
        // #20 MI = 0; RI = 1; ram_write_bus = 3;
        // #20 MI = 1; ram_write_bus = 10;
        // #20 RI = 1; ram_write_bus = 1;
        // #20 MI = 1; ram_write_bus = 2;

        // #5 s = 1; a = 100; b = 0; e = 1;
        // #5 s = 0; b = 20;
        // #5 s = 1; b = 1;
        // #5 s = 1; b = 2;
        // #5 s = 1; b = 3;
        
        // ram_read_bus = 0; ram

        // #5 B = 8'd255; A = 0; L = 15;
        // #5 A = 10; B = 0; L = 4'b1100;	// Buff A
        // #5 A = 10; B = 0; L = 4'b0011;	// Neg A
        // #5 A = 10; B = 0; L = 4'b1010;	// Buff B
        // #5 A = 10; B = 0; L = 4'b0101;	// Neg B

        #2000
        $finish;
    end
endmodule