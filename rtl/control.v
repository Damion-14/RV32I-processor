
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
    output wire [ 5:0 ] alu_op,
    output wire mem_read,
    output wire mem_to_reg,
    output wire mem_write,
    output wire alu_src,
    output wire reg_write 
);
// TODO

endmodule