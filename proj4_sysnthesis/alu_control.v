//=============================================================================
// ALU Control Unit - RISC-V RV32I Processor
//=============================================================================
// This module generates detailed control signals for the Arithmetic Logic Unit
// (ALU) based on the instruction type and function codes from the instruction.
// It serves as a secondary decoder that translates high-level ALU operation
// types from the main control unit into specific ALU control signals.
//
// The ALU control unit implements a two-level decoding scheme:
// 1. Main control unit generates 2-bit alu_op based on instruction opcode
// 2. This unit combines alu_op with function codes to generate detailed controls
//
// Supported Operations:
// - Arithmetic: ADD, SUB, SLT, SLTU
// - Logical: AND, OR, XOR
// - Shift: SLL, SRL, SRA
// - Comparison: All branch conditions via SLT/SLTU + equality
//=============================================================================

`default_nettype none

module alu_ctl (
    //=========================================================================
    // INPUT SIGNALS
    //=========================================================================
    input wire [1:0] alu_op,           // High-level ALU operation type from main control
                                       // 2'b00: ADD (R-type, Load, Store, Branch, JAL, JALR)
                                       // 2'b01: Function-specific (I-type ALU instructions)
                                       // 2'b10: PASS_B (AUIPC, LUI - currently unused)
                                       // 2'b11: Invalid

    input wire [3:0] func37,           // Combined function codes: {funct7[5], funct3[2:0]}
                                       // Used to distinguish between similar operations
                                       // e.g., ADD vs SUB, SRL vs SRA

    //=========================================================================
    // OUTPUT SIGNALS - ALU Control Interface
    //=========================================================================
    output wire [2:0] i_opsel,         // ALU operation select (maps to ALU's operation mux)
    output wire       i_sub,           // Subtract enable (vs addition)
    output wire       i_unsigned,      // Unsigned comparison enable
    output wire       i_arith          // Arithmetic right shift enable (vs logical)
);

    //=========================================================================
    // INTERNAL ALU CONTROL ENCODING
    //=========================================================================
    // Internal 4-bit control signal that encodes all possible ALU operations
    // This intermediate encoding simplifies the complex decoding logic
    wire [3:0] alu_control;

    // ALU Control Encoding Table:
    // 4'b0000: ADD    - Addition
    // 4'b0001: SUB    - Subtraction
    // 4'b0010: AND    - Bitwise AND
    // 4'b0011: OR     - Bitwise OR
    // 4'b0100: XOR    - Bitwise XOR
    // 4'b0101: SLL    - Shift Left Logical
    // 4'b0110: SRL    - Shift Right Logical
    // 4'b0111: SRA    - Shift Right Arithmetic
    // 4'b1000: SLT    - Set Less Than (signed)
    // 4'b1001: SLTU   - Set Less Than Unsigned
    // 4'b1010: PASS_B - Pass second operand (for LUI/AUIPC, currently unused)
    // 4'b1011: SUB_CMP - Subtract for comparison (reserved)
    // 4'b1111: INVALID - Invalid operation

    //=========================================================================
    // PRIMARY ALU CONTROL DECODER
    //=========================================================================
    // This decoder implements a hierarchical approach:
    // 1. First level: Decode based on alu_op (instruction class)
    // 2. Second level: Decode based on func37 (specific operation within class)

    assign alu_control =
        //---------------------------------------------------------------------
        // Class 1: ADD-type operations (alu_op = 2'b00)
        // Used by: R-type, Load, Store, Branch, JAL, JALR
        //---------------------------------------------------------------------
        (alu_op == 2'b00) ? (
            // For R-type instructions, func37[3] = funct7[5] distinguishes ADD/SUB
            (func37[3] == 1'b0) ? (  // funct7[5] = 0 (normal operations)
                (func37[2:0] == 3'b000) ? 4'b0000 : // ADD
                (func37[2:0] == 3'b111) ? 4'b0010 : // AND
                (func37[2:0] == 3'b110) ? 4'b0011 : // OR
                (func37[2:0] == 3'b100) ? 4'b0100 : // XOR
                (func37[2:0] == 3'b001) ? 4'b0101 : // SLL (Shift Left Logical)
                (func37[2:0] == 3'b101) ? 4'b0110 : // SRL (Shift Right Logical)
                (func37[2:0] == 3'b010) ? 4'b1000 : // SLT (Set Less Than)
                (func37[2:0] == 3'b011) ? 4'b1001 : // SLTU (Set Less Than Unsigned)
                4'b1111 // Invalid combination
            ) : (  // funct7[5] = 1 (alternate operations)
                (func37[2:0] == 3'b000) ? 4'b0001 : // SUB (Subtract)
                (func37[2:0] == 3'b101) ? 4'b0111 : // SRA (Shift Right Arithmetic)
                4'b1111 // Invalid combination
            )
        ) :
        //---------------------------------------------------------------------
        // Class 2: I-type ALU operations (alu_op = 2'b01)
        // Used by: ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
        //---------------------------------------------------------------------
        (alu_op == 2'b01) ? (
            (func37[2:0] == 3'b000) ? 4'b0000 : // ADDI (Add Immediate)
            (func37[2:0] == 3'b111) ? 4'b0010 : // ANDI (AND Immediate)
            (func37[2:0] == 3'b110) ? 4'b0011 : // ORI (OR Immediate)
            (func37[2:0] == 3'b100) ? 4'b0100 : // XORI (XOR Immediate)
            (func37[2:0] == 3'b001) ? 4'b0101 : // SLLI (Shift Left Logical Immediate)
            (func37[2:0] == 3'b101) ? (
                // For immediate shifts, func37[3] = funct7[5] distinguishes logical/arithmetic
                (func37[3] == 1'b0) ? 4'b0110 : // SRLI (Shift Right Logical Immediate)
                4'b0111                         // SRAI (Shift Right Arithmetic Immediate)
            ) :
            (func37[2:0] == 3'b010) ? 4'b1000 : // SLTI (Set Less Than Immediate)
            (func37[2:0] == 3'b011) ? 4'b1001 : // SLTIU (Set Less Than Immediate Unsigned)
            4'b1111 // Invalid combination
        ) :
        //---------------------------------------------------------------------
        // Class 3: Pass-through operations (alu_op = 2'b10)
        // Used by: AUIPC, LUI (currently handled in hart.v, not used here)
        //---------------------------------------------------------------------
        (alu_op == 2'b10) ? 4'b1010 : // PASS_B (pass second operand through)
        //---------------------------------------------------------------------
        // Invalid: All other alu_op values
        //---------------------------------------------------------------------
        4'b1111; // Invalid operation

    //=========================================================================
    // ALU CONTROL SIGNAL GENERATION
    //=========================================================================
    // Convert the internal 4-bit alu_control to the specific control signals
    // required by the ALU module. The ALU uses a different encoding scheme
    // optimized for its internal operation selection multiplexer.

    //-------------------------------------------------------------------------
    // Operation Select (i_opsel) - 3-bit ALU operation multiplexer control
    //-------------------------------------------------------------------------
    // Maps internal operations to ALU's operation selection encoding:
    // 3'b000: Addition/Subtraction (controlled by i_sub)
    // 3'b001: Shift Left Logical
    // 3'b010: Set Less Than (used by PASS_B, currently unused)
    // 3'b011: Set Less Than (signed/unsigned controlled by i_unsigned)
    // 3'b100: Exclusive OR
    // 3'b101: Shift Right (logical/arithmetic controlled by i_arith)
    // 3'b110: Bitwise OR
    // 3'b111: Bitwise AND

    assign i_opsel = (alu_control == 4'b0000) ? 3'b000 : // ADD -> Addition
                     (alu_control == 4'b0001) ? 3'b000 : // SUB -> Addition (with i_sub)
                     (alu_control == 4'b0010) ? 3'b111 : // AND -> Bitwise AND
                     (alu_control == 4'b0011) ? 3'b110 : // OR  -> Bitwise OR
                     (alu_control == 4'b0100) ? 3'b100 : // XOR -> Exclusive OR
                     (alu_control == 4'b0101) ? 3'b001 : // SLL -> Shift Left Logical
                     (alu_control == 4'b0110) ? 3'b101 : // SRL -> Shift Right (with i_arith=0)
                     (alu_control == 4'b0111) ? 3'b101 : // SRA -> Shift Right (with i_arith=1)
                     (alu_control == 4'b1000) ? 3'b011 : // SLT -> Set Less Than (with i_unsigned=0)
                     (alu_control == 4'b1001) ? 3'b011 : // SLTU-> Set Less Than (with i_unsigned=1)
                     (alu_control == 4'b1010) ? 3'b010 : // PASS_B -> Special encoding (unused)
                     3'b111; // Default to AND for invalid operations

    //-------------------------------------------------------------------------
    // Subtract Enable (i_sub) - Controls addition vs subtraction
    //-------------------------------------------------------------------------
    // When asserted, the ALU performs subtraction instead of addition
    // Used by: SUB instruction and branch comparisons
    assign i_sub = (alu_control == 4'b0001) ? 1'b1 : // SUB instruction
                   1'b0; // All other operations use addition

    //-------------------------------------------------------------------------
    // Unsigned Comparison Enable (i_unsigned) - Controls signed vs unsigned comparison
    //-------------------------------------------------------------------------
    // When asserted, the ALU performs unsigned comparison for set-less-than operations
    // Used by: SLTU, SLTIU, BLTU, BGEU branch instructions
    assign i_unsigned = (alu_control == 4'b1001) ? 1'b1 : // SLTU/SLTIU operations
                        1'b0; // All other operations use signed comparison

    //-------------------------------------------------------------------------
    // Arithmetic Shift Enable (i_arith) - Controls logical vs arithmetic right shift
    //-------------------------------------------------------------------------
    // When asserted, the ALU performs arithmetic right shift (sign extension)
    // When deasserted, the ALU performs logical right shift (zero fill)
    // Used by: SRA, SRAI instructions
    assign i_arith = (alu_control == 4'b0111) ? 1'b1 : // SRA/SRAI operations
                     1'b0; // All other operations use logical shift

endmodule

`default_nettype wire

//=============================================================================
// ALU CONTROL OPERATION SUMMARY
//=============================================================================
// Input Combinations -> Output Signals:
//
// R-Type Instructions (alu_op = 2'b00):
// ADD  (func37=4'b0000) -> i_opsel=3'b000, i_sub=0, i_unsigned=0, i_arith=0
// SUB  (func37=4'b1000) -> i_opsel=3'b000, i_sub=1, i_unsigned=0, i_arith=0
// AND  (func37=4'b0111) -> i_opsel=3'b111, i_sub=0, i_unsigned=0, i_arith=0
// OR   (func37=4'b0110) -> i_opsel=3'b110, i_sub=0, i_unsigned=0, i_arith=0
// XOR  (func37=4'b0100) -> i_opsel=3'b100, i_sub=0, i_unsigned=0, i_arith=0
// SLL  (func37=4'b0001) -> i_opsel=3'b001, i_sub=0, i_unsigned=0, i_arith=0
// SRL  (func37=4'b0101) -> i_opsel=3'b101, i_sub=0, i_unsigned=0, i_arith=0
// SRA  (func37=4'b1101) -> i_opsel=3'b101, i_sub=0, i_unsigned=0, i_arith=1
// SLT  (func37=4'b0010) -> i_opsel=3'b011, i_sub=0, i_unsigned=0, i_arith=0
// SLTU (func37=4'b0011) -> i_opsel=3'b011, i_sub=0, i_unsigned=1, i_arith=0
//
// I-Type ALU Instructions (alu_op = 2'b01):
// Similar mappings as R-type but with immediate operands
//
// Other Instructions (alu_op = 2'b00):
// Load/Store/Branch/JAL/JALR -> ADD operation for address calculation
//=============================================================================