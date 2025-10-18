
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
    output wire [ 1:0 ] alu_op,
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



// I Assigned 00 to be R-type instructions or instructions that use add in the ALU
//            01 to be I-type instructions
//            10 to be Pass_B which is U-type
//            11 is invalid
// The ALU control will use the 2 bits from here, Func 3 and Func 7 from the instruction
// to determine what to tell the ALU to do                        

    assign alu_op     = (opcode == 7'b0110011) ? 2'b00 : // R-type 
                        (opcode == 7'b0010011) ? 2'b01 :// I-type ALU
                        (opcode == 7'b0000011) ? 2'b00 : // Load (LW, LH, LB, etc.) - ADD
                        (opcode == 7'b0100011) ? 2'b00 : // Store (SW, SH, SB) - ADD
                        (opcode == 7'b0010111) ? 4'b10 : // AUIPC - PASS_B
                        (opcode == 7'b1101111) ? 4'b00 : // JAL - ADD
                        (opcode == 7'b1100111) ? 4'b00 : // JALR - ADD
                            2'b11;                          // Invalid
     

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