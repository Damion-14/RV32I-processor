//=============================================================================
// RISC-V RV32I Pipelined Processor (Hart) - Top-Level Wrapper
//=============================================================================
// This module is the top-level wrapper for the 5-stage pipelined RISC-V
// processor. It instantiates and connects the following modules:
//
// Pipeline Stages:
//   1. IF Stage  - Instruction Fetch (stages/if_stage.v)
//   2. ID Stage  - Instruction Decode (stages/id_stage.v)
//   3. EX Stage  - Execute (stages/ex_stage.v)
//   4. MEM Stage - Memory Access (stages/mem_stage.v)
//   5. WB Stage  - Write Back (stages/wb_stage.v)
//
// Control Units:
//   - Hazard Unit         - Detects pipeline hazards (pipeline_control/hazard_unit.v)
//   - Forwarding Unit     - Data forwarding logic (pipeline_control/forwarding_unit.v)
//   - Branch Forwarding   - Branch operand forwarding (pipeline_control/branch_forwarding_unit.v)
//
// Architecture: 5-stage pipeline with separate instruction/data memories
// ISA Support: RV32I base integer instruction set
// Features: Zero-cycle branch penalty, full data forwarding, load-use hazard detection
//=============================================================================

`default_nettype none

module hart #(
    parameter RESET_ADDR = 32'h00000000    // Reset PC address
) (
    //=========================================================================
    // GLOBAL SIGNALS
    //=========================================================================
    input  wire        i_clk,              // Global clock
    input  wire        i_rst,              // Synchronous active-high reset

    //=========================================================================
    // INSTRUCTION MEMORY INTERFACE
    //=========================================================================
    output wire [31:0] o_imem_raddr,       // Instruction memory read address
    output wire        o_imem_ren,         // Instruction memory read enable
    input  wire [31:0] i_imem_rdata,       // Instruction word from memory
    input  wire        i_imem_ready,       // Instruction memory ready (legacy)
    input  wire        i_imem_valid,       // Instruction memory data valid (legacy)

    //=========================================================================
    // DATA MEMORY INTERFACE
    //=========================================================================
    output wire [31:0] o_dmem_addr,        // Data memory address (4-byte aligned)
    output wire        o_dmem_ren,         // Data memory read enable
    output wire        o_dmem_wen,         // Data memory write enable
    output wire [31:0] o_dmem_wdata,       // Data to write to memory
    output wire [ 3:0] o_dmem_mask,        // Byte mask for sub-word accesses
    input  wire [31:0] i_dmem_rdata,       // Data read from memory
    input  wire        i_dmem_ready,       // Data memory ready (legacy)
    input  wire        i_dmem_valid,       // Data memory data valid (legacy)

    //=========================================================================
    // INSTRUCTION RETIRE INTERFACE (Verification/Debug)
    //=========================================================================
    output wire        o_retire_valid,     // Instruction retired this cycle
    output wire [31:0] o_retire_inst,      // Retired instruction word
    output wire        o_retire_trap,      // Trap occurred
    output wire        o_retire_halt,      // EBREAK instruction (halt processor)
    output wire [ 4:0] o_retire_rs1_raddr, // Source register 1 address
    output wire [ 4:0] o_retire_rs2_raddr, // Source register 2 address
    output wire [31:0] o_retire_rs1_rdata, // Source register 1 data
    output wire [31:0] o_retire_rs2_rdata, // Source register 2 data
    output wire [ 4:0] o_retire_rd_waddr,  // Destination register address
    output wire [31:0] o_retire_rd_wdata,  // Destination register data
    output wire [31:0] o_retire_pc,        // Program counter of retired instruction
    output wire [31:0] o_retire_next_pc,   // Next program counter value
    output wire [31:0] o_retire_dmem_addr, // Data memory address for retired instruction
    output wire        o_retire_dmem_ren,  // Data memory read enable for retired instruction
    output wire        o_retire_dmem_wen,  // Data memory write enable for retired instruction
    output wire [ 3:0] o_retire_dmem_mask, // Data memory byte mask for retired instruction
    output wire [31:0] o_retire_dmem_wdata,// Data memory write data for retired instruction
    output wire [31:0] o_retire_dmem_rdata // Data memory read data for retired instruction

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS
`endif
);

    //=========================================================================
    // INTER-STAGE SIGNALS
    //=========================================================================

    // IF -> ID signals
    wire [31:0] if_id_inst;
    wire [31:0] if_id_fetch_pc;
    wire [31:0] if_id_pc_plus_4;
    wire [4:0]  if_id_rs1_addr;
    wire [4:0]  if_id_rs2_addr;
    wire        if_id_is_branch;
    wire        if_id_is_jalr;
    wire        if_id_valid;
    wire        if_inst_valid;
    wire        if_cache_busy;
    wire        mem_cache_busy;

    // ID -> EX signals (ID/EX pipeline register outputs)
    wire [31:0] id_ex_pc;
    wire [31:0] id_ex_rs1_data;
    wire [31:0] id_ex_rs2_data;
    wire [31:0] id_ex_imm;
    wire [4:0]  id_ex_rs1;
    wire [4:0]  id_ex_rs2;
    wire [4:0]  id_ex_rd;
    wire [1:0]  id_ex_alu_op;
    wire [2:0]  id_ex_bj_type;
    wire        id_ex_alu_src;
    wire        id_ex_mem_read;
    wire        id_ex_mem_write;
    wire        id_ex_mem_to_reg;
    wire        id_ex_reg_write;
    wire [6:0]  id_ex_opcode;
    wire [31:0] id_ex_pc_plus_4;
    wire [2:0]  id_ex_funct3;
    wire [6:0]  id_ex_funct7;
    wire [31:0] id_ex_inst;
    wire        id_ex_valid;
    wire        id_ex_is_jal;
    wire        id_ex_is_jalr;
    wire        id_ex_is_branch;
    wire [31:0] id_ex_branch_target;

    // EX -> MEM signals (combinational, outputs from EX stage)
    wire [31:0] ex_alu_result;         // Combinational ALU output from EX stage
    wire [31:0] ex_rs2_data;
    wire [31:0] ex_rs1_data;
    wire [31:0] ex_pc;
    wire [4:0]  ex_rs1;
    wire [4:0]  ex_rs2;
    wire [4:0]  ex_rd;
    wire        ex_mem_read;
    wire        ex_mem_write;
    wire        ex_mem_to_reg;
    wire        ex_reg_write;
    wire [6:0]  ex_opcode;
    wire [31:0] ex_pc_plus_4;
    wire [2:0]  ex_funct3;
    wire [6:0]  ex_funct7;
    wire        ex_is_jal;
    wire        ex_is_jalr;
    wire        ex_is_branch;
    wire        ex_is_store;
    wire [31:0] ex_inst;
    wire        ex_valid;
    wire [31:0] ex_next_pc;
    wire [31:0] ex_branch_target;
    wire [31:0] ex_imm;

    // Registered EX/MEM signals (outputs from MEM stage pipeline registers, for forwarding)
    wire [31:0] ex_mem_alu_result_reg;
    wire [4:0]  ex_mem_rd_reg;
    wire        ex_mem_reg_write_reg;
    wire        ex_mem_valid_reg;
    wire        ex_mem_mem_to_reg_reg;

    // MEM -> WB signals (combinational)
    wire [31:0] mem_wb_mem_read_data;
    wire [31:0] mem_wb_mem_read_data_raw;
    wire [31:0] mem_wb_alu_result;
    wire [4:0]  mem_wb_rd;
    wire        mem_wb_mem_to_reg;
    wire        mem_wb_reg_write;
    wire [31:0] mem_wb_pc_plus_4;
    wire [6:0]  mem_wb_opcode;
    wire [31:0] mem_wb_imm;
    wire        mem_wb_is_jal;
    wire        mem_wb_is_jalr;
    wire        mem_wb_is_branch;
    wire        mem_wb_mem_read;
    wire        mem_wb_mem_write;
    wire [2:0]  mem_wb_funct3;
    wire [4:0]  mem_wb_rs1;
    wire [4:0]  mem_wb_rs2;
    wire [31:0] mem_wb_rs1_data;
    wire [31:0] mem_wb_rs2_data;
    wire [31:0] mem_wb_pc;
    wire [31:0] mem_wb_inst;
    wire        mem_wb_is_store;
    wire        mem_wb_unaligned_pc;
    wire        mem_wb_unaligned_mem;
    wire        mem_wb_valid;
    wire [31:0] mem_wb_dmem_addr;
    wire [ 1:0] mem_wb_byte_offset;
    wire [ 3:0] mem_wb_dmem_mask;
    wire [31:0] mem_wb_dmem_wdata;
    wire [31:0] mem_wb_next_pc;

    // WB -> ID signals (register file write)
    wire [4:0]  wb_rd;
    wire [31:0] wb_rd_data;
    wire        wb_reg_write;

    // Control signals
    wire        next_pc_sel;
    wire [31:0] next_pc;
    wire        flush_if_id;
    wire        hazard_stall_pc, hazard_stall_if_id, hazard_bubble_id_ex;
    wire        stall_pc, stall_if_id, bubble_id_ex;
    wire        stall_id_ex;
    wire [1:0]  forward_a, forward_b;

    

    //=========================================================================
    // HAZARD DETECTION UNIT
    //=========================================================================
    // Detects load-use hazards and generates stall/bubble signals
    hazard_unit hazard_detector (
        .i_id_rs1(if_id_rs1_addr),
        .i_id_rs2(if_id_rs2_addr),
        .i_id_valid(if_id_valid),
        .i_id_is_branch(if_id_is_branch),
        .i_id_is_jalr(if_id_is_jalr),
        .i_ex_rd(id_ex_rd),
        .i_ex_reg_write(id_ex_reg_write),
        .i_ex_mem_read(id_ex_mem_read),
        .i_mem_rd(ex_mem_rd_reg),                // Use registered EX/MEM value
        .i_mem_reg_write(ex_mem_reg_write_reg),  // Use registered EX/MEM value
        .i_mem_mem_read(ex_mem_mem_to_reg_reg),  // Use registered EX/MEM value
        .i_icache_busy(if_cache_busy),
        .o_stall_pc(hazard_stall_pc),
        .o_stall_if_id(hazard_stall_if_id),
        .o_bubble_id_ex(hazard_bubble_id_ex)
    );

    //=========================================================================
    // DATA FORWARDING UNIT (for EX stage)
    //=========================================================================
    // Detects when to forward data from MEM or WB stages to EX stage
    forwarding_unit data_forwarder (
        .i_ex_rs1(id_ex_rs1),
        .i_ex_rs2(id_ex_rs2),
        .i_mem_rd(ex_mem_rd_reg),                                    // Use registered value
        .i_mem_reg_write(ex_mem_reg_write_reg && ex_valid), // Use registered value
        .i_wb_rd(wb_rd),
        .i_wb_reg_write(wb_reg_write && mem_wb_valid),
        .o_forward_a(forward_a),
        .o_forward_b(forward_b)
    );

    wire frontend_stall;
    assign frontend_stall = if_cache_busy | mem_cache_busy;
    assign stall_pc       = hazard_stall_pc | frontend_stall;
    assign stall_if_id    = hazard_stall_if_id | frontend_stall;
    assign bubble_id_ex   = hazard_bubble_id_ex | frontend_stall;
    assign stall_id_ex    = mem_cache_busy;

   
    //=========================================================================
    // PIPELINE STAGE INSTANTIATIONS
    //=========================================================================

    //-------------------------------------------------------------------------
    // IF STAGE - Instruction Fetch
    //-------------------------------------------------------------------------
    if_stage #(
        .RESET_ADDR(RESET_ADDR)
    ) if_stage_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_stall_pc(stall_pc),
        .i_stall_if_id(stall_if_id),
        .i_pc_redirect(next_pc_sel),
        .i_pc_redirect_target(next_pc),
        .o_inst_valid(if_inst_valid),
        .o_imem_raddr(o_imem_raddr),
        .o_imem_ren(o_imem_ren),
        .i_imem_rdata(i_imem_rdata),
        .i_imem_ready(i_imem_ready),
        .i_imem_valid(i_imem_valid),
        .o_inst(if_id_inst),
        .o_fetch_pc(if_id_fetch_pc),
        .o_pc_plus_4(if_id_pc_plus_4),
        .o_cache_busy(if_cache_busy)
    );

    //-------------------------------------------------------------------------
    // ID STAGE - Instruction Decode
    //-------------------------------------------------------------------------
    id_stage #(
        .RESET_ADDR(RESET_ADDR)
    ) id_stage_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_inst(if_id_inst),
        .i_fetch_pc(if_id_fetch_pc),
        .i_pc_plus_4(if_id_pc_plus_4),
        .i_if_inst_valid(if_inst_valid),
        .i_stall_if_id(stall_if_id),
        .i_flush_if_id(flush_if_id),
        .i_rst_stall(bubble_id_ex),          // Use bubble signal for ID/EX stall
        .i_stall_id_ex(stall_id_ex),
        .i_wb_rd(wb_rd),
        .i_wb_rd_data(wb_rd_data),
        .i_wb_reg_write(wb_reg_write),
        // Branch forwarding inputs
        .i_ex_rd(id_ex_rd),
        .i_ex_reg_write(id_ex_reg_write),
        .i_ex_valid(id_ex_valid),
        .i_ex_alu_result(ex_alu_result),              // Combinational EX ALU result
        .i_mem_rd(ex_mem_rd_reg),                     // Registered EX/MEM rd
        .i_mem_reg_write(ex_mem_reg_write_reg),       // Registered EX/MEM reg_write
        .i_mem_valid(ex_mem_valid_reg),               // Registered EX/MEM valid
        .i_mem_result(ex_mem_alu_result_reg),         // Registered EX/MEM ALU result
        .i_mem_to_reg(ex_mem_mem_to_reg_reg),         // Registered EX/MEM mem_to_reg
        .i_mem_read_data(mem_wb_mem_read_data),
        // Outputs
        .o_next_pc(next_pc),
        .o_flush_if_id(flush_if_id),
        .o_if_id_rs1_addr(if_id_rs1_addr),
        .o_if_id_rs2_addr(if_id_rs2_addr),
        .o_if_id_is_branch(if_id_is_branch),
        .o_if_id_is_jalr(if_id_is_jalr),
        .o_if_id_valid(if_id_valid),
        // ID/EX pipeline register outputs
        .o_pc(id_ex_pc),
        .o_rs1_data(id_ex_rs1_data),
        .o_rs2_data(id_ex_rs2_data),
        .o_imm(id_ex_imm),
        .o_rs1(id_ex_rs1),
        .o_rs2(id_ex_rs2),
        .o_rd(id_ex_rd),
        .o_alu_op(id_ex_alu_op),
        .o_bj_type(id_ex_bj_type),
        .o_alu_src(id_ex_alu_src),
        .o_mem_read(id_ex_mem_read),
        .o_mem_write(id_ex_mem_write),
        .o_mem_to_reg(id_ex_mem_to_reg),
        .o_reg_write(id_ex_reg_write),
        .o_opcode(id_ex_opcode),
        .o_pc_plus_4(id_ex_pc_plus_4),
        .o_funct3(id_ex_funct3),
        .o_funct7(id_ex_funct7),
        .o_inst(id_ex_inst),
        .o_valid(id_ex_valid),
        .o_is_jal(id_ex_is_jal),
        .o_is_jalr(id_ex_is_jalr),
        .o_is_branch(id_ex_is_branch),
        .o_branch_target(id_ex_branch_target)
    );

    assign next_pc_sel = flush_if_id;

    //-------------------------------------------------------------------------
    // EX STAGE - Execute
    //-------------------------------------------------------------------------
    ex_stage ex_stage_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        // Inputs from ID/EX pipeline registers
        .i_pc(id_ex_pc),
        .i_rs1_data(id_ex_rs1_data),
        .i_rs2_data(id_ex_rs2_data),
        .i_imm(id_ex_imm),
        .i_rs1(id_ex_rs1),
        .i_rs2(id_ex_rs2),
        .i_rd(id_ex_rd),
        .i_alu_op(id_ex_alu_op),
        .i_bj_type(id_ex_bj_type),
        .i_alu_src(id_ex_alu_src),
        .i_mem_read(id_ex_mem_read),
        .i_mem_write(id_ex_mem_write),
        .i_mem_to_reg(id_ex_mem_to_reg),
        .i_reg_write(id_ex_reg_write),
        .i_opcode(id_ex_opcode),
        .i_pc_plus_4(id_ex_pc_plus_4),
        .i_funct3(id_ex_funct3),
        .i_funct7(id_ex_funct7),
        .i_inst(id_ex_inst),
        .i_valid(id_ex_valid),
        .i_is_jal(id_ex_is_jal),
        .i_is_jalr(id_ex_is_jalr),
        .i_is_branch(id_ex_is_branch),
        .i_branch_target(id_ex_branch_target),
        // Forwarding inputs
        .i_forward_a(forward_a),
        .i_forward_b(forward_b),
        .i_mem_fwd_data(ex_mem_alu_result_reg),  // Use registered EX/MEM value (breaks loop!)
        .i_wb_fwd_data(wb_rd_data),
        // Outputs (combinational)
        .o_alu_result(ex_alu_result),
        .o_rs2_data_fwd(ex_rs2_data),
        .o_rs1_data_fwd(ex_rs1_data),
        .o_pc(ex_pc),
        .o_rs1(ex_rs1),
        .o_rs2(ex_rs2),
        .o_rd(ex_rd),
        .o_mem_read(ex_mem_read),
        .o_mem_write(ex_mem_write),
        .o_mem_to_reg(ex_mem_to_reg),
        .o_reg_write(ex_reg_write),
        .o_opcode(ex_opcode),
        .o_pc_plus_4(ex_pc_plus_4),
        .o_funct3(ex_funct3),
        .o_funct7(ex_funct7),
        .o_is_jal(ex_is_jal),
        .o_is_jalr(ex_is_jalr),
        .o_is_branch(ex_is_branch),
        .o_is_store(ex_is_store),
        .o_inst(ex_inst),
        .o_valid(ex_valid),
        .o_next_pc(ex_next_pc),
        .o_branch_target(ex_branch_target),
        .o_imm(ex_imm)
    );

    //-------------------------------------------------------------------------
    // MEM STAGE - Memory Access
    //-------------------------------------------------------------------------
    mem_stage mem_stage_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        // Inputs from EX stage (combinational)
        .i_alu_result(ex_alu_result),
        .i_rs2_data(ex_rs2_data),
        .i_rs1_data(ex_rs1_data),
        .i_pc(ex_pc),
        .i_rs1(ex_rs1),
        .i_rs2(ex_rs2),
        .i_rd(ex_rd),
        .i_mem_read(ex_mem_read),
        .i_mem_write(ex_mem_write),
        .i_mem_to_reg(ex_mem_to_reg),
        .i_reg_write(ex_reg_write),
        .i_opcode(ex_opcode),
        .i_pc_plus_4(ex_pc_plus_4),
        .i_funct3(ex_funct3),
        .i_funct7(ex_funct7),
        .i_is_jal(ex_is_jal),
        .i_is_jalr(ex_is_jalr),
        .i_is_branch(ex_is_branch),
        .i_is_store(ex_is_store),
        .i_inst(ex_inst),
        .i_valid(ex_valid),
        .i_next_pc(ex_next_pc),
        .i_branch_target(ex_branch_target),
        .i_imm(ex_imm),
        // Data memory interface
        .o_dmem_addr(o_dmem_addr),
        .o_dmem_ren(o_dmem_ren),
        .o_dmem_wen(o_dmem_wen),
        .o_dmem_wdata(o_dmem_wdata),
        .o_dmem_mask(o_dmem_mask),
        .i_dmem_rdata(i_dmem_rdata),
        .i_dmem_ready(i_dmem_ready),
        .i_dmem_valid(i_dmem_valid),
        .o_dcache_busy(mem_cache_busy),
        // Forwarding outputs (registered EX/MEM values)
        .o_ex_mem_alu_result_reg(ex_mem_alu_result_reg),
        .o_ex_mem_rd_reg(ex_mem_rd_reg),
        .o_ex_mem_reg_write_reg(ex_mem_reg_write_reg),
        .o_ex_mem_valid_reg(ex_mem_valid_reg),
        .o_ex_mem_mem_to_reg_reg(ex_mem_mem_to_reg_reg),
        // Outputs to WB stage
        .o_mem_read_data(mem_wb_mem_read_data),
        .o_mem_read_data_raw(mem_wb_mem_read_data_raw),
        .o_alu_result(mem_wb_alu_result),
        .o_rd(mem_wb_rd),
        .o_mem_to_reg(mem_wb_mem_to_reg),
        .o_reg_write(mem_wb_reg_write),
        .o_pc_plus_4(mem_wb_pc_plus_4),
        .o_opcode(mem_wb_opcode),
        .o_imm(mem_wb_imm),
        .o_is_jal(mem_wb_is_jal),
        .o_is_jalr(mem_wb_is_jalr),
        .o_is_branch(mem_wb_is_branch),
        .o_mem_read(mem_wb_mem_read),
        .o_mem_write(mem_wb_mem_write),
        .o_funct3(mem_wb_funct3),
        .o_rs1(mem_wb_rs1),
        .o_rs2(mem_wb_rs2),
        .o_rs1_data(mem_wb_rs1_data),
        .o_rs2_data(mem_wb_rs2_data),
        .o_pc(mem_wb_pc),
        .o_inst(mem_wb_inst),
        .o_is_store(mem_wb_is_store),
        .o_unaligned_pc(mem_wb_unaligned_pc),
        .o_unaligned_mem(mem_wb_unaligned_mem),
        .o_valid(mem_wb_valid),
        .o_dmem_addr_out(mem_wb_dmem_addr),
        .o_byte_offset(mem_wb_byte_offset),
        .o_dmem_mask_out(mem_wb_dmem_mask),
        .o_dmem_wdata_out(mem_wb_dmem_wdata),
        .o_next_pc(mem_wb_next_pc)
    );

    //-------------------------------------------------------------------------
    // WB STAGE - Write Back
    //-------------------------------------------------------------------------
    wb_stage wb_stage_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        // Inputs from MEM stage
        .i_mem_read_data(mem_wb_mem_read_data),
        .i_mem_read_data_raw(mem_wb_mem_read_data_raw),
        .i_alu_result(mem_wb_alu_result),
        .i_rd(mem_wb_rd),
        .i_mem_to_reg(mem_wb_mem_to_reg),
        .i_reg_write(mem_wb_reg_write),
        .i_pc_plus_4(mem_wb_pc_plus_4),
        .i_opcode(mem_wb_opcode),
        .i_imm(mem_wb_imm),
        .i_is_jal(mem_wb_is_jal),
        .i_is_jalr(mem_wb_is_jalr),
        .i_is_branch(mem_wb_is_branch),
        .i_mem_read(mem_wb_mem_read),
        .i_mem_write(mem_wb_mem_write),
        .i_funct3(mem_wb_funct3),
        .i_rs1(mem_wb_rs1),
        .i_rs2(mem_wb_rs2),
        .i_rs1_data(mem_wb_rs1_data),
        .i_rs2_data(mem_wb_rs2_data),
        .i_pc(mem_wb_pc),
        .i_inst(mem_wb_inst),
        .i_is_store(mem_wb_is_store),
        .i_unaligned_pc(mem_wb_unaligned_pc),
        .i_unaligned_mem(mem_wb_unaligned_mem),
        .i_valid(mem_wb_valid),
        .i_dmem_addr(mem_wb_dmem_addr),
        .i_byte_offset(mem_wb_byte_offset),
        .i_dmem_mask(mem_wb_dmem_mask),
        .i_dmem_wdata(mem_wb_dmem_wdata),
        .i_next_pc(mem_wb_next_pc),
        // Outputs to ID stage (register file write)
        .o_wb_rd(wb_rd),
        .o_wb_rd_data(wb_rd_data),
        .o_wb_reg_write(wb_reg_write),
        // Retire interface
        .o_retire_valid(o_retire_valid),
        .o_retire_inst(o_retire_inst),
        .o_retire_trap(o_retire_trap),
        .o_retire_halt(o_retire_halt),
        .o_retire_rs1_raddr(o_retire_rs1_raddr),
        .o_retire_rs2_raddr(o_retire_rs2_raddr),
        .o_retire_rs1_rdata(o_retire_rs1_rdata),
        .o_retire_rs2_rdata(o_retire_rs2_rdata),
        .o_retire_rd_waddr(o_retire_rd_waddr),
        .o_retire_rd_wdata(o_retire_rd_wdata),
        .o_retire_pc(o_retire_pc),
        .o_retire_next_pc(o_retire_next_pc),
        .o_retire_dmem_addr(o_retire_dmem_addr),
        .o_retire_dmem_ren(o_retire_dmem_ren),
        .o_retire_dmem_wen(o_retire_dmem_wen),
        .o_retire_dmem_mask(o_retire_dmem_mask),
        .o_retire_dmem_wdata(o_retire_dmem_wdata),
        .o_retire_dmem_rdata(o_retire_dmem_rdata)
    );

endmodule

`default_nettype wire

//=============================================================================
// END OF HART MODULE
//=============================================================================
