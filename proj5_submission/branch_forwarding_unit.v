//=============================================================================
// Branch Forwarding Unit
//=============================================================================
// This module detects data hazards for branch operands in the ID stage
// and generates forwarding control signals to bypass data from EX, MEM, or
// WB stages back to the ID stage for branch comparisons.
//
// Three types of forwarding:
// 1. EX-ID Forwarding (ID/EX -> ID): Forward from EX stage to ID stage
// 2. MEM-ID Forwarding (EX/MEM -> ID): Forward from MEM stage to ID stage
// 3. WB-ID Forwarding (MEM/WB -> ID): Forward from WB stage to ID stage
//
// Priority: EX-ID > MEM-ID > WB-ID (the most recent instruction's result)
//
// Forwarding control signals (2-bit):
// 00 = No forwarding (use register file data)
// 01 = Forward from ID/EX stage (EX stage result)
// 10 = Forward from EX/MEM stage (MEM stage result)
// 11 = Forward from MEM/WB stage (WB stage result)
//=============================================================================

`default_nettype none

module branch_forwarding_unit (
    // Source registers from ID stage (instruction currently being decoded)
    input  wire [4:0]  i_id_rs1,
    input  wire [4:0]  i_id_rs2,

    // Destination register and control from EX stage (ID/EX pipeline)
    input  wire [4:0]  i_ex_rd,
    input  wire        i_ex_reg_write,

    // Destination register and control from MEM stage (EX/MEM pipeline)
    input  wire [4:0]  i_mem_rd,
    input  wire        i_mem_reg_write,

    // Destination register and control from WB stage (MEM/WB pipeline)
    input  wire [4:0]  i_wb_rd,
    input  wire        i_wb_reg_write,

    // Forwarding control outputs
    output wire [1:0]  o_forward_a,        // Forward control for branch operand A (rs1)
    output wire [1:0]  o_forward_b         // Forward control for branch operand B (rs2)
);

    //-------------------------------------------------------------------------
    // EX-ID Forwarding (from EX stage)
    //-------------------------------------------------------------------------
    // Forward from EX stage if:
    // 1. EX stage will write to a register (reg_write = 1)
    // 2. EX stage destination is not x0
    // 3. EX stage destination matches ID stage source register

    wire forward_from_ex_to_rs1;
    wire forward_from_ex_to_rs2;

    assign forward_from_ex_to_rs1 = i_ex_reg_write &&        // EX stage writes
                                    (i_ex_rd != 5'b0) &&     // Not writing to x0
                                    (i_ex_rd == i_id_rs1);   // Destination matches rs1

    assign forward_from_ex_to_rs2 = i_ex_reg_write &&        // EX stage writes
                                    (i_ex_rd != 5'b0) &&     // Not writing to x0
                                    (i_ex_rd == i_id_rs2);   // Destination matches rs2

    //-------------------------------------------------------------------------
    // MEM-ID Forwarding (from MEM stage)
    //-------------------------------------------------------------------------
    // Forward from MEM stage if:
    // 1. MEM stage will write to a register (reg_write = 1)
    // 2. MEM stage destination is not x0
    // 3. MEM stage destination matches ID stage source register
    // 4. NOT already forwarding from EX stage (EX has priority)

    wire forward_from_mem_to_rs1;
    wire forward_from_mem_to_rs2;

    assign forward_from_mem_to_rs1 = i_mem_reg_write &&       // MEM stage writes
                                     (i_mem_rd != 5'b0) &&    // Not writing to x0
                                     (i_mem_rd == i_id_rs1) && // Destination matches rs1
                                     !forward_from_ex_to_rs1; // EX not already forwarding

    assign forward_from_mem_to_rs2 = i_mem_reg_write &&       // MEM stage writes
                                     (i_mem_rd != 5'b0) &&    // Not writing to x0
                                     (i_mem_rd == i_id_rs2) && // Destination matches rs2
                                     !forward_from_ex_to_rs2; // EX not already forwarding

    //-------------------------------------------------------------------------
    // WB-ID Forwarding (from WB stage)
    //-------------------------------------------------------------------------
    // Forward from WB stage if:
    // 1. WB stage will write to a register (reg_write = 1)
    // 2. WB stage destination is not x0
    // 3. WB stage destination matches ID stage source register
    // 4. NOT already forwarding from EX or MEM stages

    wire forward_from_wb_to_rs1;
    wire forward_from_wb_to_rs2;

    assign forward_from_wb_to_rs1 = i_wb_reg_write &&         // WB stage writes
                                    (i_wb_rd != 5'b0) &&      // Not writing to x0
                                    (i_wb_rd == i_id_rs1) &&  // Destination matches rs1
                                    !forward_from_ex_to_rs1 && // EX not already forwarding
                                    !forward_from_mem_to_rs1; // MEM not already forwarding

    assign forward_from_wb_to_rs2 = i_wb_reg_write &&         // WB stage writes
                                    (i_wb_rd != 5'b0) &&      // Not writing to x0
                                    (i_wb_rd == i_id_rs2) &&  // Destination matches rs2
                                    !forward_from_ex_to_rs2 && // EX not already forwarding
                                    !forward_from_mem_to_rs2; // MEM not already forwarding

    //-------------------------------------------------------------------------
    // Generate Forwarding Control Signals
    //-------------------------------------------------------------------------
    // 2-bit control for each branch operand:
    // 00 = No forwarding (use data from register file)
    // 01 = Forward from ID/EX (EX stage)
    // 10 = Forward from EX/MEM (MEM stage)
    // 11 = Forward from MEM/WB (WB stage)

    assign o_forward_a = forward_from_ex_to_rs1  ? 2'b01 :   // EX-ID forwarding (highest priority)
                         forward_from_wb_to_rs1  ? 2'b11 :   // WB-ID forwarding
                         forward_from_mem_to_rs1 ? 2'b10 :   // MEM-ID forwarding
                         2'b00;                              // No forwarding

    assign o_forward_b = forward_from_ex_to_rs2  ? 2'b01 :   // EX-ID forwarding (highest priority)
                         forward_from_wb_to_rs2  ? 2'b11 :   // WB-ID forwarding
                         forward_from_mem_to_rs2 ? 2'b10 :   // MEM-ID forwarding
                         2'b00;                              // No forwarding

endmodule

`default_nettype wire