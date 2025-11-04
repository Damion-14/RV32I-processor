//=============================================================================
// Forwarding Unit (Data Bypass Unit)
//=============================================================================
// This module detects data hazards and generates forwarding control signals
// to bypass data from later pipeline stages back to the EX stage ALU inputs.
//
// Two types of forwarding:
// 1. EX-EX Forwarding (EX/MEM -> EX): Forward from MEM stage to EX stage
// 2. MEM-EX Forwarding (MEM/WB -> EX): Forward from WB stage to EX stage
//
// Priority: EX-EX forwarding has higher priority than MEM-EX forwarding
// (the most recent instruction's result should be forwarded)
//
// Forwarding control signals (2-bit):
// 00 = No forwarding (use register file data)
// 01 = Forward from EX/MEM stage (MEM stage result)
// 10 = Forward from MEM/WB stage (WB stage result)
//=============================================================================

`default_nettype none

module forwarding_unit (
    // Source registers from EX stage (instruction currently executing)
    input  wire [4:0]  i_ex_rs1,
    input  wire [4:0]  i_ex_rs2,

    // Destination register and control from MEM stage (EX/MEM pipeline)
    input  wire [4:0]  i_mem_rd,
    input  wire        i_mem_reg_write,

    // Destination register and control from WB stage (MEM/WB pipeline)
    input  wire [4:0]  i_wb_rd,
    input  wire        i_wb_reg_write,

    // Forwarding control outputs
    output wire [1:0]  o_forward_a,        // Forward control for ALU operand A (rs1)
    output wire [1:0]  o_forward_b         // Forward control for ALU operand B (rs2)
);

    //-------------------------------------------------------------------------
    // EX-EX Forwarding (from MEM stage)
    //-------------------------------------------------------------------------
    // Forward from MEM stage if:
    // 1. MEM stage will write to a register (reg_write = 1)
    // 2. MEM stage destination is not x0
    // 3. MEM stage destination matches EX stage source register

    wire forward_from_mem_to_rs1;
    wire forward_from_mem_to_rs2;

    assign forward_from_mem_to_rs1 = i_mem_reg_write &&       // MEM stage writes
                                     (i_mem_rd != 5'b0) &&    // Not writing to x0
                                     (i_mem_rd == i_ex_rs1);  // Destination matches rs1

    assign forward_from_mem_to_rs2 = i_mem_reg_write &&       // MEM stage writes
                                     (i_mem_rd != 5'b0) &&    // Not writing to x0
                                     (i_mem_rd == i_ex_rs2);  // Destination matches rs2

    //-------------------------------------------------------------------------
    // MEM-EX Forwarding (from WB stage)
    //-------------------------------------------------------------------------
    // Forward from WB stage if:
    // 1. WB stage will write to a register (reg_write = 1)
    // 2. WB stage destination is not x0
    // 3. WB stage destination matches EX stage source register
    // 4. NOT already forwarding from MEM stage (MEM has priority)

    wire forward_from_wb_to_rs1;
    wire forward_from_wb_to_rs2;

    assign forward_from_wb_to_rs1 = i_wb_reg_write &&         // WB stage writes
                                    (i_wb_rd != 5'b0) &&      // Not writing to x0
                                    (i_wb_rd == i_ex_rs1) &&  // Destination matches rs1
                                    !forward_from_mem_to_rs1; // MEM not already forwarding

    assign forward_from_wb_to_rs2 = i_wb_reg_write &&         // WB stage writes
                                    (i_wb_rd != 5'b0) &&      // Not writing to x0
                                    (i_wb_rd == i_ex_rs2) &&  // Destination matches rs2
                                    !forward_from_mem_to_rs2; // MEM not already forwarding

    //-------------------------------------------------------------------------
    // Generate Forwarding Control Signals
    //-------------------------------------------------------------------------
    // 2-bit control for each ALU input:
    // 00 = No forwarding (use data from ID/EX pipeline register)
    // 01 = Forward from EX/MEM (MEM stage)
    // 10 = Forward from MEM/WB (WB stage)

    assign o_forward_a = forward_from_mem_to_rs1 ? 2'b01 :   // EX-EX forwarding
                         forward_from_wb_to_rs1  ? 2'b10 :   // MEM-EX forwarding
                         2'b00;                              // No forwarding

    assign o_forward_b = forward_from_mem_to_rs2 ? 2'b01 :   // EX-EX forwarding
                         forward_from_wb_to_rs2  ? 2'b10 :   // MEM-EX forwarding
                         2'b00;                              // No forwarding

endmodule

`default_nettype wire
