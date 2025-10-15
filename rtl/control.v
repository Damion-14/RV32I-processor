
// =============================================================================
// Control Signal Truth Table
// =============================================================================
// Instruction Type       | Opcode  | Branch | MemRead | MemtoReg | MemWrite | ALUSrc | RegWrite
// -----------------------|---------|--------|---------|----------|----------|--------|----------
// R-type (add, sub, etc.)| 0110011 |   0    |    0    |    0     |    0     |   0    |    1
// I-type (addi, etc.)    | 0010011 |   0    |    0    |    0     |    0     |   1    |    1
// Load (lw, lh, lb, etc.)| 0000011 |   0    |    1    |    1     |    0     |   1    |    1
// Store (sw, sh, sb)     | 0100011 |   0    |    0    |    X     |    1     |   1    |    0
// Branch (beq, bne, etc.)| 1100011 |   1    |    0    |    X     |    0     |   0    |    0
// =============================================================================

module ctl (
    input wire [ 6:0 ] instruction,

    output wire [ 1:0 ] U_sel,
    output wire [ 5:0 ] i_format,
    output wire [ 2:0 ] bj_type,
    output wire [ 3:0 ] alu_op,
    output wire mem_read,
    output wire mem_to_reg,
    output wire mem_write,
    output wire alu_src,
    output wire reg_write 
);

    // Extract opcode from instruction
    wire [ 6:0 ] opcode = instruction[6:0];

    // Control signals based on opcode
    
    assign U_sel      = (opcode == 7'b0110111) ? 2'b01 : // LUI
                        (opcode == 7'b0010111) ? 2'b10 : // AUIPC
                        2'b00;                          // None
// Instruction format, determined by the instruction decoder based on the
    // opcode. This is one-hot encoded according to the following format:
    // [0] R-type (don't-care, see below)
    // [1] I-type
    // [2] S-type
    // [3] B-type
    // [4] U-type
    // [5] J-type
    assign i_format   = (opcode == 7'b0110011) ? 6'b000001 : // R-type
                        (opcode == 7'b0010011) ? 6'b000010 : // I-type
                        (opcode == 7'b0100011) ? 6'b000100 : // S-type
                        (opcode == 7'b1100011) ? 6'b001000 : // B-type
                        (opcode == 7'b0010111) ? 6'b010000 : // U-type (AUIPC)
                        (opcode == 7'b1101111) ? 6'b100000 : // J-type (JAL)
                        6'b000000;                          // None

    //bj-type: Branch type for branch and jump instructions
        // 000: BEQ
        // 001: BNE
        // 100: BLT
        // 101: BGE
        // 110: BLTU
        // 111: BGEU
        // 011: JALR
        // 011: JAL
    assign bj_type   = (opcode == 7'b1100011) ? instruction[14:12] : // Branch type from funct3
                        (opcode == 7'b1101111) ? 3'b011 : // JAL
                        (opcode == 7'b1100111) ? 3'b011 : // JALR
                        3'b010;                           // None


// ALU operation encoding (4 bits):
//ADD: Addition (ALU Op = 4’b0000)
//•SUB: Subtraction (ALU Op = 4’b0001)
//•AND: Bitwise AND (ALU Op = 4’b0010)
//•OR: Bitwise OR (ALU Op = 4’b0011)
//•XOR: Bitwise XOR (ALU Op = 4’b0100)
//•SLL: Shift left logical (ALU Op = 4’b0101)
//•SRL: Shift right logical (ALU Op = 4’b0110)
//•SRA: Shift right arithmetic (ALU Op = 4’b0111)
//•SLT: Set less than (signed) (ALU Op = 4’b1000)
//•SLTU: Set less than (unsigned) (ALU Op = 4’b1001)
//•PASS_B: Pass 2nd operand (LUI) (ALU Op = 4’b1010)
//•SUB/CMP: Sub for comp (set flags) (ALU Op = 4’b1011)
    assign alu_op     = (opcode == 7'b0110011) ? // R-type
                        (instruction[30] == 1'b0) ? // Check funct7[5]
                            (instruction[14:12] == 3'b000) ? 4'b0000 : // ADD
                            (instruction[14:12] == 3'b111) ? 4'b0010 : // AND
                            (instruction[14:12] == 3'b110) ? 4'b0011 : // OR
                            (instruction[14:12] == 3'b100) ? 4'b0100 : // XOR
                            (instruction[14:12] == 3'b001) ? 4'b0101 : // SLL
                            (instruction[14:12] == 3'b101) ? 4'b0110 : // SRL
                            (instruction[14:12] == 3'b010) ? 4'b1000 : // SLT
                            (instruction[14:12] == 3'b011) ? 4'b1001 : // SLTU
                            4'b1111 // Invalid
                        :
                            (instruction[14:12] == 3'b000) ? 4'b0001 : // SUB
                            (instruction[14:12] == 3'b101) ? 4'b0111 : // SRA
                            4'b1111 // Invalid
                        :
                        (opcode == 7'b0010011) ? // I-type ALU
                            (instruction[14:12] == 3'b000) ? 4'b0000 : // ADDI
                            (instruction[14:12] == 3'b111) ? 4'b0010 : // ANDI
                            (instruction[14:12] == 3'b110) ? 4'b0011 : // ORI
                            (instruction[14:12] == 3'b100) ? 4'b0100 : // XORI
                            (instruction[14:12] == 3'b001) ? 4'b0101 : // SLLI
                            (instruction[14:12] == 3'b101) ?
                            (instruction[30] == 1'b0) ? 4'b0110 : // SRLI
                            4'b0111 : // SRA
                            (instruction[14:12] == 3'b010) ? 4'b1000 : // SLTI
                            (instruction[14:12] == 3'b011) ? 4'b1001 : // SLTIU
                            4'b1111 : // Invalid
                            (opcode == 7'b0000011) ? 4'b0000 : // Load (LW, LH, LB, etc.) - ADD
                            (opcode == 7'b0100011) ? 4'b0000 : // Store (SW, SH, SB) - ADD
                            (opcode == 7'b0010111) ? 4'b1010 : // AUIPC - PASS_B
                            (opcode == 7'b1101111) ? 4'b0000 : // JAL - ADD
                            (opcode == 7'b1100111) ? 4'b0000 : // JALR - ADD
                            4'b1111;                          // Invalid
     

    assign mem_read   = (opcode == 7'b0000011) ? 1'b1 : 1'b0; // Load

    assign mem_to_reg = (opcode == 7'b0000011) ? 1'b1 : 1'b0; // Load

    assign mem_write  = (opcode == 7'b0100011) ? 1'b1 : 1'b0; // Store

    assign reg_write  = (opcode == 7'b0110011) ? 1'b1 : // R-type ALU
                        (opcode == 7'b0010011) ? 1'b1 : // I-type ALU
                        (opcode == 7'b0000011) ? 1'b1 : // Load
                        (opcode == 7'b0010111) ? 1'b1 : // AUIPC
                        (opcode == 7'b1101111) ? 1'b1 : // JAL
                        (opcode == 7'b1100111) ? 1'b1 : // JALR
                        1'b0;                          // Store and Branch

    assign alu_src    = (opcode == 7'b0010011) ? 1'b1 : // I-type ALU
                        (opcode == 7'b0000011) ? 1'b1 : // Load
                        (opcode == 7'b0100011) ? 1'b1 : // Store
                        (opcode == 7'b0010111) ? 1'b1 : // AUIPC
                        (opcode == 7'b1101111) ? 1'b1 : // JAL
                        (opcode == 7'b1100111) ? 1'b1 : // JALR
                        1'b0;                          // R-type ALU and Branch

endmodule