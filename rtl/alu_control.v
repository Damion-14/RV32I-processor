module alu_ctl(
  input wire [ 1:0 ] alu_op,
  input wire [ 3:0 ] func37,

  output wire [ 2:0 ] i_opsel,
  output wire i_sub, 
  output wire i_unsigned, 
  output wire i_arith,

);
//inner wire that will carry the alu control signal that determines 
wire [3:0] alu_control

//derive the alu control signal from Func3, Func7 from the instructions, and ALU Op from control
assign alu_control     = (alu_op == 2'b00) ? // R-type
                        (func37[3] == 1'b0) ? // Check funct7[5]
                            (func37[2:0] == 3'b000) ? 4'b0000 : // ADD
                            (func37[2:0] == 3'b111) ? 4'b0010 : // AND
                            (func37[2:0] == 3'b110) ? 4'b0011 : // OR
                            (func37[2:0] == 3'b100) ? 4'b0100 : // XOR
                            (func37[2:0] == 3'b001) ? 4'b0101 : // SLL
                            (func37[2:0] == 3'b101) ? 4'b0110 : // SRL
                            (func37[2:0] == 3'b010) ? 4'b1000 : // SLT
                            (func37[2:0] == 3'b011) ? 4'b1001 : // SLTU
                            4'b1111 // Invalid
                        :
                            (func37[2:0] == 3'b000) ? 4'b0001 : // SUB
                            (func37[2:0] == 3'b101) ? 4'b0111 : // SRA
                            4'b1111 // Invalid
                        :
                        (alu_op == 2'b01) ? // I-type ALU
                            (func37[2:0] == 3'b000) ? 4'b0000 : // ADDI
                            (func37[2:0] == 3'b111) ? 4'b0010 : // ANDI
                            (func37[2:0] == 3'b110) ? 4'b0011 : // ORI
                            (func37[2:0] == 3'b100) ? 4'b0100 : // XORI
                            (func37[2:0] == 3'b001) ? 4'b0101 : // SLLI
                            (func37[2:0] == 3'b101) ?
                            (func37[3] == 1'b0) ? 4'b0110 : // SRLI
                            4'b0111 : // SRA
                            (func37[2:0] == 3'b010) ? 4'b1000 : // SLTI
                            (func37[2:0] == 3'b011) ? 4'b1001 : // SLTIU
                            4'b1111 : // Invalid
                        (alu_op == 2'b10) ? 4'b1010 : // AUIPC - PASS_B
                            4'b1111;        // Invalid
  
//Use the ALU control signal to determine the option select for the ALU based on following tables
// ADD: Addition 						           (ALU Op = 4’b0000)
// SUB: Subtraction 					         (ALU Op = 4’b0001)
// AND: Bitwise AND 					         (ALU Op = 4’b0010)
// OR: Bitwise OR		 				           (ALU Op = 4’b0011)
// XOR: Bitwise XOR 				           (ALU Op = 4’b0100)
// SLL: Shift left logical 				     (ALU Op = 4’b0101)
// SRL: Shift right logical 				   (ALU Op = 4’b0110)
// SRA: Shift right arithmetic 			   (ALU Op = 4’b0111)
// SLT: Set less than (signed) 			   (ALU Op = 4’b1000)
// SLTU: Set less than (unsigned) 		 (ALU Op = 4’b1001)
// PASS_B: Pass 2nd operand (LUI) 	   (ALU Op = 4’b1010) <- not used in schematic
// SUB/CMP: Sub for comp (set flags)   (ALU Op = 4’b1011) <- not sure if this is needed as ALU always checks the flags?

// 3'b000: addition/subtraction if `i_sub` asserted
// 3'b001: shift left logical
// 3'b010,
// 3'b011: set less than/unsigned if `i_unsigned` asserted
// 3'b100: exclusive or
// 3'b101: shift right logical/arithmetic if `i_arith` asse
// 3'b110: or
// 3'b111: and
  assign i_opsel = (alu_op = 4'b0000) ? 3'b000 : //ADD
                    (alu_op = 4'b0001) ? 3'b000 : //SUB
                    (alu_op = 4'b0010) ? 3'b111 : //AND
                    (alu_op = 4'b0011) ? 3'b110 : //OR
                    (alu_op = 4'b0100) ? 3'b100 : //XOR
                    (alu_op = 4'b0101) ? 3'b001 : //SLL
                    (alu_op = 4'b0110) ? 3'b101 : //SRL
                    (alu_op = 4'b0111) ? 3'b101 : //SRA
                    (alu_op = 4'b1000) ? 3'b011 : //SLT
                    (alu_op = 4'b1001) ? 3'b011 : //SLTU
                    (alu_op = 4'b1010) ? 3'b010 : //PASS_B 
// not sure how to implment PASS_B currently using the unused op_sel 
//but that means we'd have to change the ALU
                      4'b1111; // invalid

  assign i_sub = (alu_op = 4'b0001 | alu_op = 4'b1011) ? 1 : 0; // set 1 if SUB or SUB/CMP
  
  assign i_unsigned = (alu_op = 4'b1001) ? 1 : 0; // set 1 if SLTU
  
  assign i_arith = (alu_op = 4'b0111) ? 1 : 0; // set 1 if SRA

endmodule