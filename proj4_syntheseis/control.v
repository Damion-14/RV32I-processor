//=============================================================================
// Main Control Unit - RISC-V RV32I Processor
//=============================================================================
// This module serves as the primary control unit for the RISC-V processor,
// generating all major control signals based on the instruction opcode.
// It implements the control path of the processor, determining how data
// flows through the datapath for each instruction type.
//
// The control unit uses a single-level decoding scheme where the 7-bit
// instruction opcode directly determines all control signals. This approach
// provides fast decoding suitable for a single-cycle processor implementation.
//
// Supported Instruction Types:
// - R-type: Register-to-register operations (ADD, SUB, AND, OR, etc.)
// - I-type: Immediate operations (ADDI, ANDI, Load instructions, JALR)
// - S-type: Store operations (SW, SH, SB)
// - B-type: Branch operations (BEQ, BNE, BLT, BGE, BLTU, BGEU)
// - U-type: Upper immediate operations (LUI, AUIPC)
// - J-type: Jump operations (JAL)
//=============================================================================

`default_nettype none

module ctl (
    //=========================================================================
    // INPUT SIGNALS
    //=========================================================================
    input wire [31:0] instruction,     // Complete 32-bit instruction word
                                        // Only bits [6:0] (opcode) and [14:12] (funct3)
                                        // are used by this control unit

    //=========================================================================
    // OUTPUT SIGNALS - Control Path Interface
    //=========================================================================

    //-------------------------------------------------------------------------
    // Instruction Format and Type Identification
    //-------------------------------------------------------------------------
    output wire [1:0] U_sel,            // Upper immediate select (currently unused)
                                         // 2'b00: No upper immediate
                                         // 2'b01: LUI operation
                                         // 2'b10: AUIPC operation
                                         // Note: LUI/AUIPC are handled directly in hart.v

    output wire [5:0] i_format,         // Instruction format (one-hot encoded)
                                         // Bit [0]: R-type (register-register operations)
                                         // Bit [1]: I-type (immediate operations, loads, JALR)
                                         // Bit [2]: S-type (store operations)
                                         // Bit [3]: B-type (branch operations)
                                         // Bit [4]: U-type (upper immediate operations)
                                         // Bit [5]: J-type (jump operations)

    output wire [2:0] bj_type,          // Branch/Jump type specification
                                         // For branches (B-type): encodes funct3 directly
                                         // 3'b000: BEQ  (branch if equal)
                                         // 3'b001: BNE  (branch if not equal)
                                         // 3'b100: BLT  (branch if less than, signed)
                                         // 3'b101: BGE  (branch if greater/equal, signed)
                                         // 3'b110: BLTU (branch if less than, unsigned)
                                         // 3'b111: BGEU (branch if greater/equal, unsigned)
                                         // For jumps: 3'b011 (JAL, JALR)

    //-------------------------------------------------------------------------
    // ALU Control Interface
    //-------------------------------------------------------------------------
    output wire [1:0] alu_op,           // High-level ALU operation class
                                         // 2'b00: ADD (R-type, Load, Store, Branch, JAL, JALR)
                                         // 2'b01: Function-specific (I-type ALU instructions)
                                         // 2'b10: PASS_B (AUIPC, LUI - currently unused)
                                         // 2'b11: Invalid

    //-------------------------------------------------------------------------
    // Memory Interface Control
    //-------------------------------------------------------------------------
    output wire mem_read,               // Enable data memory read (Load instructions)
    output wire mem_to_reg,             // Select memory data for register writeback
                                        // 1: Load instruction (write memory data to register)
                                        // 0: Other instructions (write ALU result or other)

    output wire mem_write,              // Enable data memory write (Store instructions)

    //-------------------------------------------------------------------------
    // Register File Control
    //-------------------------------------------------------------------------
    output wire alu_src,                // ALU second operand source select
                                        // 1: Use immediate value (I, S, U, J-type)
                                        // 0: Use rs2 register data (R-type, Branches)

    output wire reg_write               // Register file write enable
                                        // 1: Write result to destination register
                                        // 0: No register write (Store, Branch instructions)
);

    //=========================================================================
    // INSTRUCTION OPCODE EXTRACTION
    //=========================================================================
    // Extract the 7-bit opcode from the instruction word
    // This is the primary field used for instruction decode
    wire [6:0] opcode;
    assign opcode = instruction[6:0];

    //=========================================================================
    // CONTROL SIGNAL GENERATION
    //=========================================================================
    // All control signals are generated combinationally based on the opcode
    // The control unit implements a truth table approach for maximum clarity

    //-------------------------------------------------------------------------
    // Upper Immediate Select (U_sel) - Currently Unused
    //-------------------------------------------------------------------------
    // This signal was intended to control upper immediate handling but is
    // currently unused since LUI and AUIPC are handled directly in hart.v
    assign U_sel = (opcode == 7'b0110111) ? 2'b01 : // LUI
                   (opcode == 7'b0010111) ? 2'b10 : // AUIPC
                   2'b00;                           // None

    //-------------------------------------------------------------------------
    // Instruction Format Identification (i_format)
    //-------------------------------------------------------------------------
    // One-hot encoding of instruction formats for the immediate generator
    // This allows the immediate generator to select the correct immediate format
    assign i_format = (opcode == 7'b0110011) ? 6'b000001 : // R-type
                      (opcode == 7'b0010011 || opcode == 7'b0000011 || opcode == 7'b1100111) ? 6'b000010 : // I-type (ALU-I, Load, JALR)
                      (opcode == 7'b0100011) ? 6'b000100 : // S-type (Store)
                      (opcode == 7'b1100011) ? 6'b001000 : // B-type (Branch)
                      (opcode == 7'b0110111 || opcode == 7'b0010111) ? 6'b010000 : // U-type (LUI, AUIPC)
                      (opcode == 7'b1101111) ? 6'b100000 : // J-type (JAL)
                      6'b000000;                           // Invalid/None

    //-------------------------------------------------------------------------
    // Branch/Jump Type Specification (bj_type)
    //-------------------------------------------------------------------------
    // For branch instructions, this directly passes through funct3 to specify
    // the branch condition. For jump instructions, uses a fixed encoding.
    assign bj_type = (opcode == 7'b1100011) ? instruction[14:12] : // Branch: use funct3
                     (opcode == 7'b1101111) ? 3'b011 :             // JAL
                     (opcode == 7'b1100111) ? 3'b011 :             // JALR
                     3'b010;                                        // None/Default

    //-------------------------------------------------------------------------
    // ALU Operation Class (alu_op)
    //-------------------------------------------------------------------------
    // High-level classification of ALU operations for the ALU control unit
    // This implements a two-level control scheme: main control -> ALU control -> ALU
    assign alu_op = (opcode == 7'b0010011) ? 2'b01 : // I-type ALU (function-specific)
                    (opcode == 7'b0110011 ||
                     opcode == 7'b1100011 || opcode == 7'b0110111 || opcode == 7'b0010111 ||
                     opcode == 7'b1101111 || opcode == 7'b1100111) ? 2'b00 : // ADD or don't care
                    (opcode == 7'b0000011 || opcode == 7'b0100011) ? 2'b11 : // Load/Store (always ADD)
                    2'b10;                                                    // Invalid

    //-------------------------------------------------------------------------
    // Memory Control Signals
    //-------------------------------------------------------------------------
    // These signals control the data memory interface for load and store operations

    // Memory Read Enable - Asserted for all load instructions
    assign mem_read = (opcode == 7'b0000011) ? 1'b1 : 1'b0; // Load instructions

    // Memory-to-Register Mux Control - Selects memory data for register writeback
    assign mem_to_reg = (opcode == 7'b0000011) ? 1'b1 : 1'b0; // Load instructions

    // Memory Write Enable - Asserted for all store instructions
    assign mem_write = (opcode == 7'b0100011) ? 1'b1 : 1'b0; // Store instructions

    //-------------------------------------------------------------------------
    // Register File Control Signals
    //-------------------------------------------------------------------------
    // These signals control register file access and data flow

    // Register Write Enable - Asserted for instructions that write to rd
    // All instructions except stores and branches write to a register
    assign reg_write = (opcode == 7'b0110011 || opcode == 7'b0010011 || opcode == 7'b0000011 ||
                        opcode == 7'b0110111 || opcode == 7'b0010111 || opcode == 7'b1101111 ||
                        opcode == 7'b1100111) ? 1'b1 : // Instructions that write to rd
                       1'b0;                           // Store and Branch (no register write)

    // ALU Source Select - Controls second ALU operand (rs2 vs immediate)
    // Instructions using immediate values assert this signal
    assign alu_src = (opcode == 7'b0010011 || opcode == 7'b0000011 || opcode == 7'b0100011 ||
                      opcode == 7'b0110111 || opcode == 7'b0010111 || opcode == 7'b1101111 ||
                      opcode == 7'b1100111) ? 1'b1 : // Instructions using immediate
                     1'b0;                           // R-type ALU and Branch (use rs2)

endmodule

`default_nettype wire

//=============================================================================
// CONTROL SIGNAL TRUTH TABLE
//=============================================================================
// Complete truth table showing all control signals for each instruction type:
//
// Instruction Type    | Opcode  |U_sel|i_format|bj_type|alu_op|mem_rd|mem2reg|mem_wr|alu_src|reg_wr
// --------------------|---------|-----|--------|-------|------|------|-------|------|-------|------
// R-type ALU          |0110011  | 00  |000001  | 010   | 00   |  0   |   0   |  0   |   0   |  1
// I-type ALU          |0010011  | 00  |000010  | 010   | 01   |  0   |   0   |  0   |   1   |  1
// Load (LW,LH,LB...)  |0000011  | 00  |000010  | 010   | 00   |  1   |   1   |  0   |   1   |  1
// Store (SW,SH,SB)    |0100011  | 00  |000100  | 010   | 00   |  0   |   X   |  1   |   1   |  0
// Branch (BEQ,BNE...) |1100011  | 00  |001000  |funct3 | 00   |  0   |   X   |  0   |   0   |  0
// JAL                 |1101111  | 00  |100000  | 011   | 00   |  0   |   0   |  0   |   1   |  1
// JALR                |1100111  | 00  |000010  | 011   | 00   |  0   |   0   |  0   |   1   |  1
// LUI                 |0110111  | 01  |010000  | 010   | 00   |  0   |   0   |  0   |   1   |  1
// AUIPC               |0010111  | 10  |010000  | 010   | 00   |  0   |   0   |  0   |   1   |  1
//
// Legend:
// X = Don't care (signal not used for this instruction type)
// funct3 = Branch type extracted from instruction[14:12]
// U_sel currently unused (LUI/AUIPC handled in hart.v)
//=============================================================================

//=============================================================================
// INSTRUCTION TYPE BREAKDOWN
//=============================================================================
//
// R-TYPE INSTRUCTIONS (opcode = 7'b0110011):
// - Register-to-register operations
// - Examples: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA, SLT, SLTU
// - Use both rs1 and rs2, write to rd
// - ALU operation determined by funct3 and funct7
//
// I-TYPE INSTRUCTIONS:
// - Immediate ALU (opcode = 7'b0010011): ADDI, ANDI, ORI, XORI, SLLI, SRLI, SRAI, SLTI, SLTIU
// - Load instructions (opcode = 7'b0000011): LB, LH, LW, LBU, LHU
// - JALR (opcode = 7'b1100111): Jump and Link Register
// - Use rs1 and immediate, write to rd (except stores)
//
// S-TYPE INSTRUCTIONS (opcode = 7'b0100011):
// - Store operations: SB, SH, SW
// - Use rs1 (base) and rs2 (data) with immediate offset
// - No register write (rd field used for immediate bits)
//
// B-TYPE INSTRUCTIONS (opcode = 7'b1100011):
// - Branch operations: BEQ, BNE, BLT, BGE, BLTU, BGEU
// - Compare rs1 and rs2, branch based on condition
// - No register write
//
// U-TYPE INSTRUCTIONS:
// - LUI (opcode = 7'b0110111): Load Upper Immediate
// - AUIPC (opcode = 7'b0010111): Add Upper Immediate to PC
// - Load 20-bit immediate into upper bits of rd
//
// J-TYPE INSTRUCTIONS (opcode = 7'b1101111):
// - JAL: Jump and Link
// - Store return address (PC+4) in rd, jump to PC+immediate
//=============================================================================