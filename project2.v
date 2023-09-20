module mux2X1(select, I0, I1, out);
  input select;
  input [31:0] I0, I1;
  output [31:0] out;

  assign out = (select) ? I1 : I0; // Selects I1 if select is true (1), otherwise selects I0.
endmodule

module mux2X1_tb();
  reg select;
  reg [31:0] I0, I1;
  wire [31:0] out;
  mux2X1 m21(select, I0, I1, out);

  initial begin
    I0 <= 32'h12345678; 
    I1 <= 32'h789ABCDE;
    select <= 0;        // Initialize select with 0 (false)
    #5ns select <= 1;   // After a delay of 5 nanoseconds, change select to 1 (true)
  end
endmodule

module mux4X1(select, I0, I1, I2, I3, out);
  input [31:0] select;
  input [31:0] I0, I1, I2, I3;
  output [31:0] out;

  assign out = (select[1]) ? ((select[0]) ? I3 : I2) : ((select[0]) ? I1 : I0);
  // Selects I3 if select[1] is true (1), otherwise selects I2.
  // If select[1] is false (0), selects I1 if select[0] is true (1), otherwise selects I0.
endmodule

module mux4X1_tb();
  reg [1:0] select;
  reg [31:0] I0, I1, I2, I3;
  wire [31:0] out;
  mux4X1 m41(select, I0, I1, I2, I3, out);

  initial begin
    I0 <= 32'h11122233; 
    I1 <= 32'h33344455;
    I2 <= 32'h55566677; 
    I3 <= 32'h77788899; 
    select <= 0;        // Initialize select with 0 (false)
    repeat (3) #5ns select <= select + 1; // After a delay of 5 nanoseconds, increment select by 1 (0, 1, 2)
  end
endmodule

module adder(
  input [31:0] A, B,
  output [31:0] out
);
  assign out = A + B; // Adds inputs A and B and assigns the result to out
endmodule

module adder_tb();
  reg [31:0] A, B;
  wire [31:0] out;
  adder adder1(A, B, out);

  initial begin
    A = 32'h12345678;   
    B = 32'h12345678;   
    #5ns;
    A = 32'h12345678;   
    B = 32'hFF133456;   
    #5ns;
    $finish;            // Finish the simulation
  end
endmodule


module registers_file(
  input [4:0] RA, RB, RW,       // Input registers for read addresses (RA, RB) and write address (RW)
  input [31:0] BusW,            // Input bus for write data
  input enableW,                // Input enable signal for write operation
  output [31:0] BusA, BusB      // Output buses for read data
);
  reg [31:0] file[0:31];        // Register file with 32 registers
  
  initial begin
    for (int i = 0; i <= 31; i = i + 1) begin
      file[i] = 0;              // Initialize all registers in the file to 0
    end
    file[2] = 5;                // Assign a value of 5 to register 2
    file[3] = 5;                // Assign a value of 5 to register 3
    file[4] = 5;                // Assign a value of 5 to register 4
  end
  
  assign BusA = file[RA];       // Assign the value of the register specified by RA to BusA
  assign BusB = file[RB];       // Assign the value of the register specified by RB to BusB
  
  always @* begin
    if (enableW && RW != 0) begin
      file[RW] <= BusW;         // If write operation is enabled and RW is not 0, assign the value of BusW to the register specified by RW
    end
  end
endmodule

module rf_tb();
  reg [4:0] RA, RB, RW;          // Input registers for read addresses (RA, RB) and write address (RW)
  reg [31:0] BusW;              // Input bus for write data
  reg enableW;                  // Input enable signal for write operation
  wire [31:0] BusA, BusB;       // Output buses for read data
  registers_file rf(
    .RA(RA),
    .RB(RB),
    .RW(RW),
    .enableW(enableW),
    .BusW(BusW),
    .BusA(BusA),
    .BusB(BusB)
  );

  initial begin
    RA = 1;                     // Initialize RA with value 1
    RB = 0;                     // Initialize RB with value 0
    RW = 1;                     // Initialize RW with value 1
    enableW = 1;                // Set enableW to 1 (true)
    BusW = 32'h123456;          // Initialize BusW with hexadecimal value 123456
    repeat (32) begin           // Repeat the following statements 32 times
      #5ns RA <= RA + 1;        // After a delay of 5 nanoseconds, increment RA by 1
      RB <= RB + 1;             // Increment RB by 1
      RW <= RW + 1;             // Increment RW by 1
      BusW <= BusW + 1;         // Increment BusW by 1
    end
  end
endmodule


module extender(
  input [1:0] ExtendSel,
  input [23:0] Immediate,
  output reg [31:0] out
);
  always @* begin
    if (ExtendSel == 0) begin
      out[13:0] = Immediate[13:0];
      for (int i = 14; i < 32; i++) begin
          out[i] = Immediate[13];
      end
    end
    else if (ExtendSel == 1) begin
      out[23:0] = Immediate[23:0];
      for (int i = 24; i < 32; i++) begin
        out[i] = Immediate[23];
      end
    end
    else begin
      out[4:0] = Immediate[8:4];
      for (int i = 5; i < 32; i++) begin
        out[i] = Immediate[8];
      end
    end
  end
endmodule

module extender_tb;
  
  reg [1:0] ExtendSel;
  reg [23:0] Immediate;
  wire [31:0] out;
  
  extender dut (
    .ExtendSel(ExtendSel),
    .Immediate(Immediate),
    .out(out)
  );
  
  initial begin
    // Test case 1
    ExtendSel = 0;
    Immediate = 24'hABCDE; // Example immediate value
    
    #10; // Allow some time for computation
    
    $display("Test Case 1:");
    $display("ExtendSel = %b", ExtendSel);
    $display("Immediate = %h", Immediate);
    $display("out = %h", out);
    
    // Test case 2
    ExtendSel = 1;
    Immediate = 24'h12345; // Example immediate value
    
    #10; // Allow some time for computation
    
    $display("Test Case 2:");
    $display("ExtendSel = %b", ExtendSel);
    $display("Immediate = %h", Immediate);
    $display("out = %h", out);
    
    // Test case 3
    ExtendSel = 2;
    Immediate = 24'hFEDCB; // Example immediate value
    
    #10; // Allow some time for computation
    
    $display("Test Case 3:");
    $display("ExtendSel = %b", ExtendSel);
    $display("Immediate = %h", Immediate);
    $display("out = %h", out);
    
    $finish;
  end
endmodule


module PCregister (
    input wire PCWr,                     // Input wire for PC write enable signal
    input wire [31:0] input_address,      // Input wire for the new PC address
    output reg [31:0] address = 0         // Output register for the current PC address, initialized to 0
);

always @(posedge PCWr) begin
    address <= input_address;             // Assign the value of input_address to address on the positive edge of PCWr
end

endmodule


`timescale 1ns/1ps

module test_PCregister;

    reg PCWr;
    reg [31:0] input_address;
    wire [31:0] address;

    PCregister uut (
        .PCWr(PCWr),
        .input_address(input_address),
        .address(address)
    );

    initial begin

        PCWr = 0;
        input_address = 0;

        #10;

        input_address = 32'h00000001;
        PCWr = 1;
        #10;

        input_address = 32'h00000002;
        PCWr = 0;
        #10;

        PCWr = 1;
        #10;

        input_address = 32'h00000003;
        #10;

        PCWr = 0;
        input_address = 32'h00000004;
        #10;

        $finish;
    end

    always @(address) begin
        $display("Time: %t, Address: %h", $time, address);
    end

endmodule 

module InstructionMemory(
    input [31:0] address,
    output reg [31:0] instruction
);
    reg [31:0] instructions [0:255];  
    					   

	
    parameter AND = 5'b00000, ADD = 5'b00001, SUB = 5'b00010, CMP = 5'b00011;
    parameter ANDI = 5'b00000, ADDI = 5'b00001, LW = 5'b00010, SW = 5'b00011, BEQ = 5'b00100;
    parameter J = 5'b00000, JAL = 5'b00001;
    parameter SLL = 5'b00000, SLR = 5'b00001, SLLV = 5'b00010, SLRV = 5'b00011;
    
    parameter R0 = 5'b00000, R1 = 5'b00001, R2 = 5'b00010, R3 = 5'b00011, R4 = 5'b00100, 
              R5 = 5'b00101, R6 = 5'b00110, R7 = 5'b00111, R8 = 5'b01000, R9 = 5'b01001,
              R10 = 5'b01010, R11 = 5'b01011, R12 = 5'b01100, R13 = 5'b01101, R14 = 5'b01110, 
              R15 = 5'b01111, R16 = 5'b10000, R17 = 5'b10001, R18 = 5'b10010, R19 = 5'b10011,
              R20 = 5'b10100, R21 = 5'b10101, R22 = 5'b10110, R23 = 5'b10111, R24 = 5'b11000, 
              R25 = 5'b11001, R26 = 5'b11010, R27 = 5'b11011, R28 = 5'b11100, R29 = 5'b11101,
              R30 = 5'b11110, R31 = 5'b11111;
              
    initial begin 
		
		instructions[0] = {ADDI, R2, R3,14'b11, 2'b10, 1'b0};
		//instructions[1] = {JAL,24'b000000000000000000000011, 2'b01, 1'b0}; 
		//instructions[1] = {BEQ, R3, R2,14'b11, 2'b10, 1'b0};
		instructions[2] = {ANDI, R3, R2,14'b11, 2'b10, 1'b0};
		instructions[3] = {AND, R2, R1, R3, 9'b0, 2'b00, 1'b0};
		instructions[4] = {SUB, R1, R1, R2, 9'b0, 2'b00, 1'b0};
		instructions[5] = {SW, R2, R3,14'b110, 2'b10, 1'b1};
		instructions[6] = {SLRV, R2, R3,R4, 5'b11,4'b0, 2'b11, 1'b0};
		instructions[7] = {SLLV, R3, R7,R4, 5'b11,4'b0, 2'b11, 1'b0}; 
		instructions[8] = {SLR, R7, R8,5'b11, 5'b11,4'b0, 2'b11, 1'b0};
		instructions[9] = {LW, R2, R3,14'b110, 2'b10, 1'b0};
		instructions[10] = {CMP, R8, R1, R7, 9'b0, 2'b00, 1'b0};

    end

    assign instruction = instructions[address];
endmodule
`timescale 1ns/1ps

module test_InstructionMemory;
    reg [31:0] address;
    wire [31:0] instruction;

    InstructionMemory uut (
        .address(address),
        .instruction(instruction)
    );

    initial begin
      
        address = 0;
        
        #10;

        address = 5;
        #10;

        address = 8;
        #10;

        address = 10;
        #10;
        
        address = 13;
        #10;

        $finish;
    end

    initial begin
        $monitor("At time %dns, the address is %d, and the instruction is %b", $time, address, instruction);
    end

endmodule	

module Stack (
    input wire [31:0] pc_in,     // Input wire for the program counter value to be stored in the stack (pushed)
    input wire jal,              // Input wire indicating a jump and store operation
    input wire stop,             // Input wire indicating a return and pop operation
    output wire [31:0] pc_out    // Output wire for the program counter value retrieved from the stack (popped)
);

    parameter STACK_DEPTH = 256;  // Parameter defining the depth of the stack

    reg [31:0] stack_mem [0:STACK_DEPTH-1];   // Register array representing the stack memory

    reg [7:0] SP = 0;             // Register representing the stack pointer (SP)

    reg [31:0] next_pc;           // Register for storing the next program counter value

    assign pc_out = next_pc;      // Assign the value of next_pc to pc_out

    always @(*) begin
        if (jal) begin
            stack_mem[SP] = pc_in;   // Store the value of pc_in in the stack at the current SP position
            SP = SP + 1;             // Increment the stack pointer (SP)
        end else if (stop) begin
            SP = SP - 1;             // Decrement the stack pointer (SP)
            next_pc = stack_mem[SP]; // Retrieve the value from the stack at the updated SP position and assign it to next_pc
        end 
    end
endmodule

  

module tb_Stack;

    reg [31:0] pc_in;
    reg jal;
    reg stop;
    wire [31:0] pc_out;

    Stack stack0 (
        .pc_in(pc_in),
        .jal(jal),
        .stop(stop),
        .pc_out(pc_out)
    );

    initial begin
        pc_in = 0;
        jal = 0;
        stop = 0;
        #10;
        pc_in = 32'hABCDE;
        jal = 1;
        #10;
     
        jal = 0;
        #10;

        stop = 1;
        #10;
        
        stop = 0;
        #10;
       
        pc_in = 32'h12345;
        jal = 1;
        #10;
        
        jal = 0;
        #10;
       
        stop = 1;
        #10;
       
        stop = 0;
        #10;
        
        pc_in = 32'h54321;
        jal = 1;
        #10;
        
        jal = 0;
        #10;
      
        stop = 1;
        #10;
        
        $finish;
    end
endmodule




module ALU (operand1, operand2, ALU_OP, res, z_flag);
    input [31:0] operand1;
    input [31:0] operand2;
    input [2:0] ALU_OP;
    output reg z_flag;
    output reg [31:0] res;

    reg [31:0] result;

    initial begin
        z_flag = 1'b0;
    end

    always @(*) begin
        case (ALU_OP)
            3'b000: result = operand1 & operand2;  // and + andi
            3'b001: result = operand1 << operand2; // shift left
            3'b010: result = operand1 + operand2;  // add + addi
            3'b011: result = operand1 - operand2;  // sub + subi
            3'b100: result = operand1 >> operand2; // shift right 
            default: result = 0;
        endcase
        z_flag = (result == 0);
    end

    assign res = result;
endmodule
	 

module ALU_tb;
    reg [31:0] operand1, operand2;
    reg [2:0] ALU_OP;
    wire [31:0] res;
    wire z_flag;
    ALU uut (
        .operand1(operand1),
        .operand2(operand2),
        .ALU_OP(ALU_OP),
        .res(res),
        .z_flag(z_flag)
    );
    initial begin
        operand2 = 32'h012121;
        operand1 = 32'h987654;
        ALU_OP = 0;
		repeat(6)
			#5ns ALU_OP = ALU_OP + 1;
    end
endmodule 

module data_memory (
    input [31:0] address,
    input MemWr,
    input MemRd,
    input [31:0] data_in,
    output reg [31:0] data_out
);

reg [31:0] memory [0:511];

//If MemRd is true (non-zero or high), which means a memory read operation is requested, the value of memory[address] 
//is assigned to data_out. In other words, the value read from the memory at the specified address is assigned to data_out.
//If MemRd is false (zero or low), which means a memory read operation is not requested, 
//24'h000000 (hexadecimal representation of all zeroes) is assigned to data_out. In other words, data_out is assigned zero.

always @* begin
    if (MemWr) begin
        memory[address] <= data_in;
    end
end

assign data_out = MemRd ? memory[address] : 32'h00000000;

endmodule	 


module data_memory_tb;

reg [31:0] address;
reg MemWr, MemRd;
reg [31:0] data_in;
wire [31:0] data_out;

data_memory mem (
    .address(address),
    .MemWr(MemWr),
    .MemRd(MemRd),
    .data_in(data_in),
    .data_out(data_out)
);

initial begin
    // write data to address 0 and 1
    address = 32'h00000000;
    MemWr = 1;
    data_in = 32'h01234567;
    #10;
    address = 32'h00000001;
    data_in = 32'h89ABCDEF;
    #10;

    // read data from address 0 and 1
    address = 32'h00000000;
    MemWr = 0;
    MemRd = 1;
    #10;
    address = 32'h00000001;
    #10;
end

endmodule



module control_unit(CLK, Type, Function_value, Zero_flag,state, PcWr, JalSig, StopSig, RegDst, ExtSel, RegWr, PcSrc, ALUSrc, ALUOp, MemWr, MemRd, WbData, stop_flag, jal_flag);
    input CLK;
    input [1:0] Type;
    input [4:0] Function_value;
    input Zero_flag, stop_flag, jal_flag;
    output reg [2:0] ALUOp;
    output reg [1:0] PcSrc;
    output reg [1:0] ExtSel;
	output reg[4:0] state;
    output reg RegWr, JalSig, StopSig, MemWr, MemRd, PcWr,ALUSrc, RegDst, WbData;
	wire [6:0] Instr;
	assign Instr = {Function_value, Type};
    parameter AND = 7'b0000000, ADD = 7'b0000100, SUB = 7'b0001000, CMP = 7'b0001100;
    parameter ANDI = 7'b0000010, ADDI = 7'b0000110, LW = 7'b0001010, SW = 7'b0001110, BEQ = 7'b0010010;
    parameter J = 7'b0000001, JAL = 7'b0000101;
    parameter SLL = 7'b0000011, SLR = 7'b0000111, SLLV = 7'b0001011, SLRV = 7'b0001111;
    parameter R_Type = 2'b00, J_Type = 2'b01, I_Type = 2'b10, S_Type = 2'b11;
    parameter and_op = 3'b000, sl_op = 3'b001, add_op = 3'b010, sub_op = 3'b011, sr_op = 3'b100;

    initial begin 
		
        {ExtSel, RegDst, WbData, state, RegWr,ALUOp,ALUSrc, PcSrc, MemWr, MemRd, JalSig, StopSig, PcWr} = 0;
	
    end

    always @(posedge CLK) 
		#3ns
		begin 
			
			case(state) 
				0: 
				begin
				{RegWr, PcSrc, MemWr ,JalSig, StopSig, PcWr} = 0;
				
				if (Instr == AND || Instr == ADD || Instr == SUB || Instr == SLRV || Instr == SLLV ) begin
					
					state=1;
				end
				
				else if ( Instr == CMP) begin
					state = 2;
				end
				
				else if ( Instr == ADDI || Instr == ANDI || Instr == LW) begin
					state = 3;
				end
				
				else if ( Instr == SW) begin
					state = 4;
				end
				
				else if ( Instr == BEQ) begin
					state = 5;
				end
				
				else if ( Instr == JAL) begin
					state = 6;
				end	
				
				else if ( Instr == SLL || Instr == SLR) begin
					state = 7;
				end
				
				else begin
					state = 8;
				end
				

				end
				
				1: 
				begin
				
				RegWr=1;
				RegDst=0;
				
				if ( Instr == ADD) begin
					state = 9;
				end
				else if ( Instr == AND) begin
					state = 10;
				end
				
				else if ( Instr == SUB) begin
					state = 11;
				end
				else if ( Instr == SLRV) begin
					state = 12;
				end
				else begin
					state = 13;
				end
				
				end
				
				2: 
				begin
				RegWr=0;
				RegDst=0;
				state=14;
				end
				
				3: 
				begin
				RegWr=1;
				ExtSel=2'b00;
				
				if ( Instr == ADDI || Instr == LW) begin
					state = 15;
				end	
				
				else begin
					state=16;
				end
				
				end
				
				4: 
				begin
				RegWr=0;
				RegDst=1;
				ExtSel=2'b00;
				state=17;
				end
				
				5: 
				begin
				RegWr=0;
				RegDst=1;
				state=18;	
				end
				
				6: 
				begin
				RegWr=0;
				PcSrc=1;
				StopSig=0;
				JalSig=1;
				PcWr = 1;
				#1ns PcWr = 0;
				state=0;		
				end
				
				7: 
				begin
				RegWr=0;
				ExtSel=2'b10;
				
			    if ( Instr == SLL) begin
					state = 19;
				end
				
				else begin
					state=20;	
				end
				end
				
				8: 
				begin
				RegWr=0;
				ExtSel=2'b01;
				PcSrc=1;
				PcWr = 1;
				#1ns PcWr = 0;
				state=0;
				end
				
				9: 
				begin
				
				ALUSrc= 0;
				ALUOp=add_op;
				state=23;
				
				end
				
				10: 
				begin
				ALUSrc= 0;
				ALUOp=and_op;
				state=23;	
				end
				
				11: 
				begin
				ALUSrc= 0;
				ALUOp=sub_op;
				state=23;	
				end
				
				12: 
				begin
				ALUSrc= 0;
				ALUOp=sr_op;
				state=23;	
				end
				
				13: 
				begin
				ALUSrc= 0;
				ALUOp=sl_op;
				state=23;	
				end
				
				14: 
				begin
				ALUSrc=0;
				ALUOp=sub_op;
				if (stop_flag == 1 && jal_flag == 0) begin 
					    JalSig = 0;
					    StopSig = 1;
						#1ns StopSig = 0;
					    PcSrc = 2'b11;
				end 
				else if (stop_flag == 0 && jal_flag == 1) begin 
					    JalSig = 1;
					    StopSig = 0;
						#1ns JalSig = 0;
					    PcSrc = 2'b01;
					end 
				else begin
					    PcSrc = 2'b00;
				end
				PcWr = 1;
				#1ns PcWr = 0;
				state=0;
				end
				
				15: 
				begin
				ALUSrc= 1;
				ALUOp=add_op;
				
				if (Instr ==ADDI) begin
					state=23;	
				end
				
				else begin 
					state=21;
				end
				
				end
				
				16: 
				begin
				ALUSrc= 1;
				ALUOp=and_op;
				state=23;	
				end
				
				17: 
				begin
				ALUSrc= 1;
				ALUOp=add_op;
				state=22;	
				end
				
				18: 
				begin
				ALUSrc= 1;
				ALUOp=sub_op;
				
				if(Zero_flag == 1) begin 
						PcSrc = 2'b10;
					end
					else begin
					   PcSrc = 2'b00;	
					end
				PcWr = 1;
				#1ns PcWr = 0;
				state=0;
				end
				
				19: 
				begin
				ALUSrc= 1;
				ALUOp=sl_op;
				state=23;	
				end
				
				20: 
				begin
				ALUSrc= 1;
				ALUOp=sr_op;
				state=23;	
				end
				
				21: 
				begin
				MemRd=1;
				#1ns MemRd=0;
				MemWr=0;
				state=24;
				end
				
				22: 
				begin
				MemRd=0;
				MemWr=1;
				#1ns MemWr=0;
				if (stop_flag == 1 && jal_flag == 0) begin 
					    JalSig = 0;
					    StopSig = 1; 
						#1ns StopSig = 0;
					    PcSrc = 2'b11;
					end else if (stop_flag == 0 && jal_flag == 1) begin 
					    JalSig = 1;
					    StopSig = 0; 
						#1ns JalSig = 0;
					    PcSrc = 2'b01;
					end
				
				else begin
					    PcSrc = 2'b00;
				end
				PcWr = 1;
				#1ns PcWr = 0;
				state=0;	
				end
				
				23: 
				begin
				WbData=1;
				if (stop_flag == 1 && jal_flag == 0) begin 
					    JalSig = 0;
					    StopSig = 1;
						#1ns StopSig = 0;
					    PcSrc = 2'b11;
				end 
				else if (stop_flag == 0 && jal_flag == 1) begin 
					    JalSig = 1;
					    StopSig = 0; 
						#1ns JalSig = 0;
					    PcSrc = 2'b01;
					end 
				else begin
					    PcSrc = 2'b00;
				end	
				PcWr = 1;
				#1ns PcWr = 0;
				state=0;
				#2ns  ;
				end
				
				24: 
				begin
					
				WbData=0;
				if (stop_flag == 1 && jal_flag == 0) begin 
					    JalSig = 0;
					    StopSig = 1;
						#1ns StopSig = 0;
					    PcSrc = 2'b11;
				end 
				else if (stop_flag == 0 && jal_flag == 1) begin 
					    JalSig = 1;
					    StopSig = 0; 
						#1ns JalSig = 0;
					    PcSrc = 2'b01;
					end 
				else begin
					    PcSrc = 2'b00;
				end
				PcWr = 1;
				#1ns PcWr = 0;
				state=0;
				end
				endcase
				end
       
endmodule



module control_unit_tb;

  // Declare local variables
  reg CLK;
  reg [1:0] Type;
  reg [4:0] Function_value;
  reg Zero_flag, stop_flag, jal_flag;
  wire [2:0] ALUOp;
  wire [1:0] PcSrc;
  wire [1:0] ExtSel;
  wire [4:0] state;
  wire RegWr, JalSig, StopSig, MemWr, MemRd, PcWr,ALUSrc, RegDst, WbData;  
	parameter AND = 5'b00000, ADD = 5'b00001, SUB = 5'b00010, CMP = 5'b00011;
	parameter ANDI = 5'b00000, ADDI = 5'b00001, LW = 5'b00010, SW = 5'b00011, BEQ = 5'b00100;
	parameter J = 5'b00000, JAL = 5'b00001;
	parameter SLL = 5'b00000, SLR = 5'b00001, SLLV = 5'b00010, SLRV = 5'b00011;
	parameter R_Type = 2'b00, J_Type = 2'b01, I_Type = 2'b10, S_Type = 2'b11;

  // Instantiate the device under test (DUT)
  control_unit DUT (
    .CLK(CLK),
    .Type(Type),
    .Function_value(Function_value),
    .Zero_flag(Zero_flag),
    .stop_flag(stop_flag),
    .jal_flag(jal_flag),
    .ALUOp(ALUOp),
    .PcSrc(PcSrc),
    .ExtSel(ExtSel),
    .state(state),
    .RegWr(RegWr),
    .JalSig(JalSig),
    .StopSig(StopSig),
    .MemWr(MemWr),
    .MemRd(MemRd),
    .PcWr(PcWr),
    .ALUSrc(ALUSrc),
    .RegDst(RegDst),
    .WbData(WbData)
  );

  // Generate a clock with a 10ns period
  always begin
    #5 CLK = ~CLK;
  end

  // Apply input stimulus
  initial begin
    // Initialize inputs
    CLK = 0;
    Type = S_Type;
    Function_value = SLL;
    Zero_flag = 1'b1;
    stop_flag = 1'b0;
    jal_flag = 1'b0;

    // Hold for a while
    #50;

    // Finish the simulation
    $finish;
  end

endmodule

module DFlipFlop(CLK,D,Q,Qb);
	input D,CLK;                  // Input wires for D and CLK
	output reg Q,Qb;              // Output registers for Q and Qb
	assign Qb=~Q;                 // Assign the complement of Q to Qb
	always @(posedge CLK)         // On the positive edge of CLK
		begin
		Q<=D;                     // Assign the value of D to Q
		end
	
endmodule

module RegisternBit(CLK, D, Q);
	parameter n=32;               // Parameter defining the number of bits in the register
	input [(n-1):0] D;            // Input bus for data D
	input CLK;                    // Input wire for clock signal
	output [(n-1):0] Q;           // Output bus for registered data Q
	wire [(n-1):0] te;            // Wire bus for intermediate signals
	genvar i;
	
	generate
	for(i=0;i<n;i=i+1)
		begin:register
			DFlipFlop dff(CLK,D[i],Q[i],te[i]);  // Instantiate a D flip-flop for each bit in the register
		end
	endgenerate	 	
endmodule

module JalFlag(
  input [4:0] Function,            // Input bus for function code
  input [1:0] Type,                // Input bus for instruction type
  output reg jal_flag              // Output register for jal_flag
);

  always @(*) begin
    if (Function == 5'b00001 && Type == 2'b01)   // Check if the function is 00001 (jal) and the type is 01 (J-type)
      jal_flag = 1'b1;                          // Set jal_flag to 1 if the conditions are met
    else
      jal_flag = 1'b0;                          // Set jal_flag to 0 otherwise
  end

endmodule




module ExtendModule(
  input [1:0] ExtendSel,
  input [13:0] immediate14,
  input [23:0] immediate24,
  input [4:0] sa,
  output reg [31:0] Extend_out
);

  always @*
  case (ExtendSel)
    2'b00: Extend_out = {18'b0, immediate14};
    2'b01: Extend_out = {8'b0, immediate24};
    2'b10: Extend_out = {27'b0, sa};
    default: Extend_out = 32'b0; // Default value if ExtendSel is invalid
  endcase

endmodule


module full_system();
reg CLK;                                                 // Clock signal
wire [31:0] PCnext, PCout, PCplus;                       // Program counter signals
wire [4:0] state, RB, rs1, rs2, rd, sa, Function;         // Control signals
wire RegWr, JalSig, StopSig, MemWr, MemRd, PcWr, ALUSrc, RegDst, WbData, z_flag, jal_flag, stop_flag;  // Control signals
wire [31:0] BusBReg, BusAReg, op1, op2, Stack_pop, Stack_pop_reg, BEQOP_out, BEQOP_out_reg, JUMP_OUT, JUMP_OUT_reg;   // Data signals
wire [31:0] Extend_out, Extend_out_reg, InstMemOut_reg, InstMem_out, PCPlusTemp, busA, busB, busW, res, res_reg, d_out, d_out_reg;  // Data signals
wire [13:0] immediate14;                                  // Immediate signals
wire [23:0] immediate24;                                  // Immediate signals
wire [1:0] ExtenSel, PcSrc, Type;                         // Control signals
wire [2:0] ALU_OP;                                        // ALU control signals

assign Function = InstMemOut_reg[31:27];                  // Assign bits 31-27 of InstMemOut_reg to Function
assign rs1 = InstMemOut_reg[26:22];                       // Assign bits 26-22 of InstMemOut_reg to rs1
assign rd = InstMemOut_reg[21:17];                        // Assign bits 21-17 of InstMemOut_reg to rd
assign rs2 = InstMemOut_reg[16:12];                       // Assign bits 16-12 of InstMemOut_reg to rs2
assign unused_r = InstMemOut_reg[11:3];                   // Assign bits 11-3 of InstMemOut_reg to unused_r
assign Type = InstMemOut_reg[2:1];                        // Assign bits 2-1 of InstMemOut_reg to Type
assign stop_flag = InstMemOut_reg[0];                     // Assign bit 0 of InstMemOut_reg to stop_flag
assign immediate14 = InstMemOut_reg[16:3];                // Assign bits 16-3 of InstMemOut_reg to immediate14
assign immediate24 = InstMemOut_reg[26:3];                // Assign bits 26-3 of InstMemOut_reg to immediate24
assign sa = InstMemOut_reg[11:7];                         // Assign bits 11-7 of InstMemOut_reg to sa
assign unused_s = InstMemOut_reg[6:3];                    // Assign bits 6-3 of InstMemOut_reg to unused_s
assign PCPlusTemp = PCout + 1;                            // Assign PCout + 1 to PCPlusTemp
//assign JUMP_OUT = Extend_out_reg + PCout; 


PCregister PCMod(PcWr, PCnext, PCout);                    // Program counter module
InstructionMemory InstructMemory(PCout, InstMem_out);      // Instruction memory module
RegisternBit InstructMemory_reg(CLK, InstMem_out, InstMemOut_reg);  // Register for storing the instruction memory output
mux2X1 RegDstMux (RegDst, rs2, rd, RB);                   // Multiplexer for selecting the destination register
registers_file rf(rs1, RB, rd, RegWr, busW, busA, busB);   // Register file module
RegisternBit BusA_OUT(CLK, busA, BusAReg);                 // Register for storing the value of busA
RegisternBit BusB_OUT(CLK, busB, BusBReg);                 // Register for storing the value of busB
extender ext(ExtenSel, immediate24, Extend_out);           // Extender module
RegisternBit extend(CLK, Extend_out, Extend_out_reg);      // Register for storing the extended immediate value
mux2X1 alu_src(ALUSrc, BusBReg, Extend_out_reg, op2);      // Multiplexer for selecting the ALU second operand
ALU alu_unit(BusAReg, op2, ALU_OP, res, z_flag);           // ALU module
RegisternBit alu_res(CLK, res, res_reg);                   // Register for storing the ALU result
data_memory data_mem(res_reg, MemWr, MemRd, BusBReg, d_out);  // Data memory module
RegisternBit mem_reg(clk, d_out, d_out_reg);               // Register for storing the data memory output
mux2X1 WbDatamux(WbData, d_out_reg, res_reg, busW);        // Multiplexer for selecting the data to be written back
JalFlag jal_sig(Function, Type, jal_flag);                 // JalFlag module
RegisternBit PCplus_reg(CLK, PCPlusTemp, PCplus);          // Register for storing the PCplus value
Stack stack(PCout, JalSig, StopSig, Stack_pop);           // Stack module
RegisternBit stackOut_reg(CLK, Stack_pop, Stack_pop_reg);  // Register for storing the stack output
adder BEQ_op (PCplus, Extend_out_reg, BEQOP_out);          // Adder for BEQ operation
RegisternBit BEQop_reg(CLK, BEQOP_out, BEQOP_out_reg);     // Register for storing the BEQ operation output


adder JUMP_out(PCout, Extend_out_reg, JUMP_OUT);           // Adder for JUMP operation
	
	
RegisternBit JUMP_out_reg(CLK, JUMP_OUT, JUMP_OUT_reg);    // Register for storing the JUMP operation output
mux4X1 pc_selection(PcSrc, PCplus, JUMP_OUT_reg, BEQOP_out_reg, Stack_pop_reg, PCnext);  // Multiplexer for selecting the next PC value
control_unit cu(CLK, Type, Function, z_flag, state, PcWr, JalSig, StopSig, RegDst, ExtenSel, RegWr, PcSrc, ALUSrc, ALU_OP, MemWr, MemRd, WbData, stop_flag, jal_flag);  // Control unit module

initial begin
    CLK = 0;
    repeat(100)
        #5ns CLK = !CLK;
    $finish;
end

endmodule

 
