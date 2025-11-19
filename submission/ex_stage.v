//=============================================================================
// RISC-V RV32I Pipelined Processor - Execute (EX) Stage
//=============================================================================
// This module implements the Execute (EX) stage of the 5-stage pipelined
// RISC-V processor. The EX stage is responsible for:
// - Performing ALU operations
// - Data forwarding from MEM and WB stages
// - Computing branch/jump targets (already resolved in ID stage)
//
// Note: The EX/MEM pipeline registers are located in the MEM stage module.
//=============================================================================

`default_nettype none

module ex_stage (
    //=========================================================================
    // GLOBAL SIGNALS
    //=========================================================================
    input  wire        i_clk,              // Global clock
    input  wire        i_rst,              // Synchronous active-high reset

    //=========================================================================
    // INPUTS FROM ID STAGE (combinational)
    //=========================================================================
    input  wire [31:0] i_pc,               // Program counter
    input  wire [31:0] i_rs1_data,         // Register rs1 data
    input  wire [31:0] i_rs2_data,         // Register rs2 data
    input  wire [31:0] i_imm,              // Immediate value
    input  wire [4:0]  i_rs1,              // rs1 address
    input  wire [4:0]  i_rs2,              // rs2 address
    input  wire [4:0]  i_rd,               // rd address
    input  wire [1:0]  i_alu_op,           // ALU operation type
    input  wire [2:0]  i_bj_type,          // Branch/jump type
    input  wire        i_alu_src,          // ALU source select
    input  wire        i_mem_read,         // Memory read enable
    input  wire        i_mem_write,        // Memory write enable
    input  wire        i_mem_to_reg,       // Memory to register select
    input  wire        i_reg_write,        // Register write enable
    input  wire [6:0]  i_opcode,           // Opcode
    input  wire [31:0] i_pc_plus_4,        // PC + 4
    input  wire [2:0]  i_funct3,           // Function code 3
    input  wire [6:0]  i_funct7,           // Function code 7
    input  wire [31:0] i_inst,             // Instruction word
    input  wire        i_valid,            // Pipeline valid bit
    input  wire        i_is_jal,           // Is JAL instruction
    input  wire        i_is_jalr,          // Is JALR instruction
    input  wire        i_is_branch,        // Is branch instruction
    input  wire [31:0] i_branch_target,    // Branch/jump target address

    //=========================================================================
    // FORWARDING INPUTS
    //=========================================================================
    input  wire [1:0]  i_forward_a,        // Forward control for rs1
    input  wire [1:0]  i_forward_b,        // Forward control for rs2
    input  wire [31:0] i_mem_fwd_data,     // Forwarding data from MEM stage
    input  wire [31:0] i_wb_fwd_data,      // Forwarding data from WB stage

    //=========================================================================
    // OUTPUTS TO MEM STAGE (combinational, will be registered in MEM stage)
    //=========================================================================
    output wire [31:0] o_alu_result,       // ALU result
    output wire [31:0] o_rs2_data_fwd,     // Forwarded rs2 data (for stores)
    output wire [31:0] o_rs1_data_fwd,     // Forwarded rs1 data (for retire)
    output wire [31:0] o_pc,               // Program counter (passthrough)
    output wire [4:0]  o_rs1,              // rs1 address (passthrough)
    output wire [4:0]  o_rs2,              // rs2 address (passthrough)
    output wire [4:0]  o_rd,               // rd address (passthrough)
    output wire        o_mem_read,         // Memory read enable (passthrough)
    output wire        o_mem_write,        // Memory write enable (passthrough)
    output wire        o_mem_to_reg,       // Memory to register select (passthrough)
    output wire        o_reg_write,        // Register write enable (passthrough)
    output wire [6:0]  o_opcode,           // Opcode (passthrough)
    output wire [31:0] o_pc_plus_4,        // PC + 4 (passthrough)
    output wire [2:0]  o_funct3,           // Function code 3 (passthrough)
    output wire [6:0]  o_funct7,           // Function code 7 (passthrough)
    output wire        o_is_jal,           // Is JAL (passthrough)
    output wire        o_is_jalr,          // Is JALR (passthrough)
    output wire        o_is_branch,        // Is branch (passthrough)
    output wire        o_is_store,         // Is store instruction
    output wire [31:0] o_inst,             // Instruction (passthrough)
    output wire        o_valid,            // Pipeline valid bit (passthrough)
    output wire [31:0] o_next_pc,          // Next PC value
    output wire [31:0] o_branch_target,    // Branch target (passthrough)
    output wire [31:0] o_imm               // Immediate (passthrough)
);

    //=========================================================================
    // ALU CONTROL UNIT
    //=========================================================================
    wire [2:0] i_opsel;                    // ALU operation select
    wire i_sub;                            // Subtract (vs add)
    wire i_unsigned;                       // Unsigned comparison
    wire i_arith;                          // Arithmetic (vs logical) right shift
    wire [3:0] func37;                     // Combined function codes

    assign func37 = {i_funct7[5], i_funct3};   // Concatenate funct7[5] and funct3

    alu_ctl alu_control_unit (
        .alu_op(i_alu_op),
        .func37(func37),
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith)
    );

    //=========================================================================
    // FORWARDING MUXES
    //=========================================================================
    wire [31:0] forwarded_rs1_data;        // rs1 data after forwarding
    wire [31:0] forwarded_rs2_data;        // rs2 data after forwarding

    // Forwarding Mux for rs1 (ALU operand A)
    // 00 = use data from ID/EX, 01 = forward from EX/MEM, 10 = forward from MEM/WB
    assign forwarded_rs1_data = (i_forward_a == 2'b01) ? i_mem_fwd_data :
                                (i_forward_a == 2'b10) ? i_wb_fwd_data :
                                i_rs1_data;

    // Forwarding Mux for rs2 (ALU operand B or store data)
    // 00 = use data from ID/EX, 01 = forward from EX/MEM, 10 = forward from MEM/WB
    assign forwarded_rs2_data = (i_forward_b == 2'b01) ? i_mem_fwd_data :
                                (i_forward_b == 2'b10) ? i_wb_fwd_data :
                                i_rs2_data;

    //=========================================================================
    // ALU (ARITHMETIC LOGIC UNIT)
    //=========================================================================
    wire [31:0] alu_op1, alu_op2;          // ALU input operands
    wire alu_eq;                           // Operands are equal
    wire alu_slt;                          // Operand 1 < Operand 2

    assign alu_op1 = forwarded_rs1_data;                            // First operand with forwarding
    assign alu_op2 = i_alu_src ? i_imm : forwarded_rs2_data;       // Second operand: immediate or rs2

    alu alu_unit (
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith),
        .i_op1(alu_op1),
        .i_op2(alu_op2),
        .o_result(o_alu_result),
        .o_eq(alu_eq),
        .o_slt(alu_slt)
    );

    //=========================================================================
    // OUTPUTS (COMBINATIONAL PASSTHROUGHS)
    //=========================================================================
    // These signals are passed through to the MEM stage where they will be
    // registered in the EX/MEM pipeline registers

    assign o_rs2_data_fwd   = forwarded_rs2_data;
    assign o_rs1_data_fwd   = forwarded_rs1_data;
    assign o_pc             = i_pc;
    assign o_rs1            = i_rs1;
    assign o_rs2            = i_rs2;
    assign o_rd             = i_rd;
    assign o_mem_read       = i_mem_read;
    assign o_mem_write      = i_mem_write;
    assign o_mem_to_reg     = i_mem_to_reg;
    assign o_reg_write      = i_reg_write;
    assign o_opcode         = i_opcode;
    assign o_pc_plus_4      = i_pc_plus_4;
    assign o_funct3         = i_funct3;
    assign o_funct7         = i_funct7;
    assign o_is_jal         = i_is_jal;
    assign o_is_jalr        = i_is_jalr;
    assign o_is_branch      = i_is_branch;
    assign o_is_store       = (i_opcode == 7'b0100011);
    assign o_inst           = i_inst;
    assign o_valid          = i_valid;
    assign o_next_pc        = (i_is_jal | i_is_jalr | i_is_branch) ? i_branch_target : i_pc_plus_4;
    assign o_branch_target  = i_branch_target;
    assign o_imm            = i_imm;

endmodule

`default_nettype wire

//=============================================================================
// END OF EX STAGE MODULE
//=============================================================================
