//=============================================================================
// Hazard Detection Unit
//=============================================================================
// This module detects data hazards (RAW dependencies) in the pipeline and
// generates control signals to stall the pipeline when necessary.
//
// Stall conditions:
// 1. Load-use hazard: When an instruction in EX stage is a load and the
//    instruction in ID stage uses the loaded value (requires 1 cycle stall)
// 2. With forwarding disabled: Any RAW dependency requires stalling
//
// When a stall is needed:
// - IF/ID pipeline register holds its current value (stall_if_id = 1)
// - PC holds its current value (stall_pc = 1)
// - ID/EX pipeline register is loaded with a bubble/NOP (bubble_id_ex = 1)
//=============================================================================

`default_nettype none

module hazard_unit (
    // Source registers from ID stage (current instruction being decoded)
    input  wire [4:0]  i_id_rs1,
    input  wire [4:0]  i_id_rs2,

    // Destination register and control signals from EX stage
    input  wire [4:0]  i_ex_rd,
    input  wire        i_ex_reg_write,
    input  wire        i_ex_mem_read,      // Is EX stage instruction a load?

    // Destination register and control signals from MEM stage
    input  wire [4:0]  i_mem_rd,
    input  wire        i_mem_reg_write,

    // Control outputs
    output wire        o_stall_pc,         // Stall program counter
    output wire        o_stall_if_id,      // Stall IF/ID pipeline register
    output wire        o_bubble_id_ex      // Insert bubble in ID/EX (convert to NOP)
);

    //-------------------------------------------------------------------------
    // Load-Use Hazard Detection
    //-------------------------------------------------------------------------
    // A load-use hazard occurs when:
    // 1. The instruction in EX stage is a load (mem_read = 1)
    // 2. The instruction in ID stage reads the register being loaded
    // 3. The register is not x0 (which is always 0)
    //
    // Even with forwarding, load-use hazards require a 1-cycle stall because
    // the load data is not available until after the MEM stage completes.

    wire load_use_hazard;

    assign load_use_hazard = i_ex_mem_read &&              // EX stage is a load
                             i_ex_reg_write &&             // EX stage will write a register
                             (i_ex_rd != 5'b0) &&          // Not writing to x0
                             ((i_ex_rd == i_id_rs1) ||     // ID stage reads rs1 from load
                              (i_ex_rd == i_id_rs2));      // ID stage reads rs2 from load

    //-------------------------------------------------------------------------
    // Stall and Bubble Control Signals
    //-------------------------------------------------------------------------
    // When a load-use hazard is detected:
    // - Stall IF and ID stages (hold PC and IF/ID register)
    // - Insert a bubble (NOP) into the ID/EX register

    assign o_stall_pc    = load_use_hazard;
    assign o_stall_if_id = load_use_hazard;
    assign o_bubble_id_ex = load_use_hazard;

endmodule

`default_nettype wire
