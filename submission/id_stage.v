//=============================================================================
// RISC-V RV32I Pipelined Processor - Instruction Decode (ID) Stage
//=============================================================================
// This module implements the Instruction Decode (ID) stage of the 5-stage
// pipelined RISC-V processor. The ID stage is responsible for:
// - IF/ID pipeline registers (at the start of the stage)
// - Decoding instruction fields
// - Generating control signals
// - Reading from the register file
// - Generating immediate values
// - Early branch resolution for zero-cycle branch penalty
// - Branch forwarding logic
//
// Note: Branches and jumps are resolved in this stage for zero-cycle penalty
//=============================================================================

`default_nettype none

module id_stage #(
    parameter RESET_ADDR = 32'h00000000    // Reset PC address
) (
    //=========================================================================
    // GLOBAL SIGNALS
    //=========================================================================
    input  wire        i_clk,              // Global clock
    input  wire        i_rst,              // Synchronous active-high reset

    //=========================================================================
    // INPUTS FROM IF STAGE
    //=========================================================================
    input  wire [31:0] i_inst,             // Instruction from IF stage
    input  wire [31:0] i_fetch_pc,         // PC from IF stage
    input  wire [31:0] i_pc_plus_4,        // PC+4 from IF stage

    //=========================================================================
    // PIPELINE CONTROL SIGNALS
    //=========================================================================
    input  wire        i_stall_if_id,      // Stall IF/ID pipeline register
    input  wire        i_flush_if_id,      // Flush IF/ID pipeline register
    input  wire        i_rst_stall,        // Reset stall signal
    input  wire        i_dmem_ready,       // Data memory ready signal
    input  wire        i_imem_valid,       // Instruction memory ready signal
    input  wire        i_dmem_valid,       // Data memory valid signal

    //=========================================================================
    // REGISTER FILE WRITE-BACK (from WB stage)
    //=========================================================================
    input  wire [4:0]  i_wb_rd,            // WB destination register address
    input  wire [31:0] i_wb_rd_data,       // WB register write data
    input  wire        i_wb_reg_write,     // WB register write enable

    //=========================================================================
    // FORWARDING INPUTS (for branch forwarding)
    //=========================================================================
    input  wire [4:0]  i_ex_rd,            // EX stage destination register
    input  wire        i_ex_reg_write,     // EX stage register write enable
    input  wire        i_ex_valid,         // EX stage valid bit
    input  wire [31:0] i_ex_alu_result,    // EX stage ALU result (for forwarding)

    input  wire [4:0]  i_mem_rd,           // MEM stage destination register
    input  wire        i_mem_reg_write,    // MEM stage register write enable
    input  wire        i_mem_valid,        // MEM stage valid bit
    input  wire [31:0] i_mem_result,       // MEM stage result (for forwarding)
    input  wire        i_mem_to_reg,       // MEM stage mem_to_reg signal
    input  wire [31:0] i_mem_read_data,    // MEM stage memory read data

    //=========================================================================
    // OUTPUTS TO IF STAGE
    //=========================================================================
    output wire [31:0] o_next_pc,          // Next PC value
    output wire        o_flush_if_id,      // Flush signal for IF stage

    //=========================================================================
    // OUTPUTS FOR HAZARD UNIT (ID stage view)
    //=========================================================================
    output wire [4:0]  o_if_id_rs1_addr,   // rs1 from IF/ID register
    output wire [4:0]  o_if_id_rs2_addr,   // rs2 from IF/ID register
    output wire        o_if_id_is_branch,  // IF/ID indicates branch
    output wire        o_if_id_is_jalr,    // IF/ID indicates JALR
    output wire        o_if_id_valid,      // IF/ID valid bit

    //=========================================================================
    // OUTPUTS TO EX STAGE (ID/EX pipeline register outputs)
    //=========================================================================
    output reg  [31:0] o_pc,               // Program counter
    output reg  [31:0] o_rs1_data,         // Register rs1 data
    output reg  [31:0] o_rs2_data,         // Register rs2 data
    output reg  [31:0] o_imm,              // Immediate value
    output reg  [4:0]  o_rs1,              // rs1 address
    output reg  [4:0]  o_rs2,              // rs2 address
    output reg  [4:0]  o_rd,               // rd address
    output reg  [1:0]  o_alu_op,           // ALU operation type
    output reg  [2:0]  o_bj_type,          // Branch/jump type
    output reg         o_alu_src,          // ALU source select (imm vs rs2)
    output reg         o_mem_read,         // Memory read enable
    output reg         o_mem_write,        // Memory write enable
    output reg         o_mem_to_reg,       // Memory to register select
    output reg         o_reg_write,        // Register write enable
    output reg  [6:0]  o_opcode,           // Opcode
    output reg  [31:0] o_pc_plus_4,        // PC + 4
    output reg  [2:0]  o_funct3,           // Function code 3
    output reg  [6:0]  o_funct7,           // Function code 7
    output reg  [31:0] o_inst,             // Instruction word
    output reg         o_valid,            // Pipeline valid bit
    output reg         o_is_jal,           // Is JAL instruction
    output reg         o_is_jalr,          // Is JALR instruction
    output reg         o_is_branch,        // Is branch instruction
    output reg  [31:0] o_branch_target     // Branch/jump target address
);

    //=========================================================================
    // IF/ID PIPELINE REGISTERS
    //=========================================================================
    reg  [31:0] if_id_inst;                // Instruction
    reg  [31:0] if_id_pc;                  // Program counter
    reg  [31:0] if_id_next_pc;             // Next PC (PC+4)
    reg         if_id_valid;               // Valid bit
    reg         flush_if_id_d;             // Delayed flush signal

    // IF/ID Pipeline Register
    always @(posedge i_clk) begin
        if (i_rst) begin
            if_id_inst    <= 32'b0;
            if_id_pc      <= 32'b0;
            if_id_next_pc <= 32'b0;
            if_id_valid   <= 1'b0;
        end else if (i_flush_if_id | flush_if_id_d) begin
            // Flush pipeline: insert bubble (NOP)
            if_id_inst    <= 32'h00000013;  // NOP instruction
            if_id_pc      <= 32'b0;
            if_id_next_pc <= 32'b0;
            if_id_valid   <= 1'b0;
        end else if (i_stall_if_id) begin
            // Hold current values during stall
            if_id_inst    <= if_id_inst;
            if_id_pc      <= if_id_pc;
            if_id_next_pc <= if_id_next_pc;
            if_id_valid   <= if_id_valid;
        end else begin
            // Align IF/ID PC with the instruction returned by synchronous imem
            if_id_inst    <= i_inst;
            if_id_pc      <= i_fetch_pc;
            if_id_next_pc <= i_pc_plus_4;
            if_id_valid   <= 1'b1;
        end
    end

    //=========================================================================
    // INSTRUCTION FIELD EXTRACTION
    //=========================================================================
    wire [6:0] opcode;                     // Operation code [6:0]
    wire [4:0] rd, rs1, rs2;               // Register addresses
    wire [2:0] funct3;                     // Function code 3 [14:12]
    wire [6:0] funct7;                     // Function code 7 [31:25]

    assign opcode = if_id_inst[6:0];
    assign rd     = if_id_inst[11:7];
    assign funct3 = if_id_inst[14:12];
    assign rs1    = if_id_inst[19:15];
    assign rs2    = if_id_inst[24:20];
    assign funct7 = if_id_inst[31:25];

    //=========================================================================
    // CONTROL UNIT
    //=========================================================================
    wire [1:0] U_sel;                      // Upper immediate select
    wire [5:0] i_format;                   // Instruction format
    wire [2:0] bj_type;                    // Branch/jump type
    wire [1:0] alu_op;                     // ALU operation class
    wire mem_read;                         // Memory read enable
    wire mem_to_reg;                       // Memory to register
    wire mem_write;                        // Memory write enable
    wire alu_src;                          // ALU source select
    wire reg_write;                        // Register write enable

    ctl control_unit (
        .instruction(if_id_inst),
        .U_sel(U_sel),
        .i_format(i_format),
        .bj_type(bj_type),
        .alu_op(alu_op),
        .mem_read(mem_read),
        .mem_to_reg(mem_to_reg),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .reg_write(reg_write)
    );

    //=========================================================================
    // REGISTER FILE
    //=========================================================================
    wire [31:0] rs1_data, rs2_data;        // Register read data

    rf #(.BYPASS_EN(1)) register_file (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_rs1_raddr(rs1),                 // Read address 1 (ID stage)
        .i_rs2_raddr(rs2),                 // Read address 2 (ID stage)
        .i_rd_waddr(i_wb_rd),              // Write address (WB stage)
        .i_rd_wdata(i_wb_rd_data),         // Write data (WB stage)
        .i_rd_wen(i_wb_reg_write),         // Write enable (WB stage)
        .o_rs1_rdata(rs1_data),            // Read data 1
        .o_rs2_rdata(rs2_data)             // Read data 2
    );

    //=========================================================================
    // IMMEDIATE GENERATION
    //=========================================================================
    wire [31:0] imm;                       // Generated immediate value

    imm immediate_generator (
        .i_inst(if_id_inst),
        .i_format(i_format),
        .o_immediate(imm)
    );

    //=========================================================================
    // BRANCH AND JUMP LOGIC (ID STAGE RESOLUTION)
    //=========================================================================
    wire is_branch_id;                     // Current instruction is a branch
    wire is_jal_id;                        // Current instruction is JAL
    wire is_jalr_id;                       // Current instruction is JALR

    assign is_branch_id = (opcode == 7'b1100011); // Branch instructions
    assign is_jal_id    = (opcode == 7'b1101111); // Jump and Link
    assign is_jalr_id   = (opcode == 7'b1100111); // Jump and Link Register

    assign o_if_id_rs1_addr  = rs1;
    assign o_if_id_rs2_addr  = rs2;
    assign o_if_id_is_branch = is_branch_id;
    assign o_if_id_is_jalr   = is_jalr_id;
    assign o_if_id_valid     = if_id_valid;

    //-------------------------------------------------------------------------
    // Branch Forwarding Unit
    //-------------------------------------------------------------------------
    wire [1:0] forward_branch_a;           // Forward control for branch rs1
    wire [1:0] forward_branch_b;           // Forward control for branch rs2

    branch_forwarding_unit branch_forwarder (
        .i_id_rs1(rs1),
        .i_id_rs2(rs2),
        .i_ex_rd(i_ex_rd),
        .i_ex_reg_write(i_ex_reg_write && i_ex_valid),
        .i_mem_rd(i_mem_rd),
        .i_mem_reg_write(i_mem_reg_write && i_mem_valid),
        .i_wb_rd(i_wb_rd),
        .i_wb_reg_write(i_wb_reg_write),
        .o_forward_a(forward_branch_a),
        .o_forward_b(forward_branch_b)
    );

    //-------------------------------------------------------------------------
    // Forwarding Muxes for Branch Operands
    //-------------------------------------------------------------------------
    wire [31:0] branch_rs1_data;           // rs1 data with forwarding
    wire [31:0] branch_rs2_data;           // rs2 data with forwarding

    // Data to forward from EX stage: use ALU result
    wire [31:0] ex_forward_data;
    assign ex_forward_data = i_ex_alu_result;

    // Data to forward from MEM stage: use memory read data for loads, ALU result otherwise
    wire [31:0] mem_forward_data;
    assign mem_forward_data = i_mem_to_reg ? i_mem_read_data : i_mem_result;

    // Forward from EX, MEM, or WB stage to ID stage for branch operands
    // 00 = No forwarding (use register file data)
    // 01 = Forward from EX stage
    // 10 = Forward from MEM stage
    // 11 = Forward from WB stage
    assign branch_rs1_data = (forward_branch_a == 2'b01) ? ex_forward_data :
                             (forward_branch_a == 2'b10) ? mem_forward_data :
                             (forward_branch_a == 2'b11) ? i_wb_rd_data :
                             rs1_data;

    assign branch_rs2_data = (forward_branch_b == 2'b01) ? ex_forward_data :
                             (forward_branch_b == 2'b10) ? mem_forward_data :
                             (forward_branch_b == 2'b11) ? i_wb_rd_data :
                             rs2_data;

    //-------------------------------------------------------------------------
    // Branch Condition Evaluation
    //-------------------------------------------------------------------------
    wire branch_eq;                        // Branch operands are equal
    wire branch_lt_signed;                 // rs1 < rs2 (signed)
    wire branch_lt_unsigned;               // rs1 < rs2 (unsigned)

    assign branch_eq = (branch_rs1_data == branch_rs2_data);
    assign branch_lt_signed = ($signed(branch_rs1_data) < $signed(branch_rs2_data));
    assign branch_lt_unsigned = (branch_rs1_data < branch_rs2_data);

    // Branch condition based on branch type
    wire branch_condition_id;
    assign branch_condition_id = (bj_type == 3'b000) ? branch_eq :          // BEQ
                                 (bj_type == 3'b001) ? ~branch_eq :         // BNE
                                 (bj_type == 3'b100) ? branch_lt_signed :   // BLT
                                 (bj_type == 3'b101) ? ~branch_lt_signed :  // BGE
                                 (bj_type == 3'b110) ? branch_lt_unsigned : // BLTU
                                 (bj_type == 3'b111) ? ~branch_lt_unsigned :// BGEU
                                 1'b0;

    wire branch_taken_id;                  // Branch is taken
    assign branch_taken_id = is_branch_id & branch_condition_id;

    // With synchronous imem, carry a 1-cycle delayed flush to squash
    // the wrong-path instruction
    always @(posedge i_clk) begin
        if (i_rst) begin
            flush_if_id_d <= 1'b0;
        end else begin
            flush_if_id_d <= i_flush_if_id;
        end
    end

    // Flush signal - asserted when control flow change taken in ID stage
    wire flush_if_id_internal;
    assign flush_if_id_internal = !i_stall_if_id && (is_jalr_id | is_jal_id | branch_taken_id);
    assign o_flush_if_id = flush_if_id_internal;

    //-------------------------------------------------------------------------
    // Next PC Calculation
    //-------------------------------------------------------------------------
    wire [31:0] branch_target_id;          // Branch/JAL target address
    wire [31:0] jalr_target_id;            // JALR target address
    wire [31:0] pc_plus_4_internal;

    assign branch_target_id = if_id_pc + imm;                        // PC-relative for branches/JAL
    assign jalr_target_id   = (branch_rs1_data + imm) & ~32'd1;     // Register+immediate, clear LSB
    assign pc_plus_4_internal = if_id_pc + 32'd4;

    // Next PC Selection (controlled by ID stage branch/jump decisions)
    wire [31:0] next_pc_internal;
    assign next_pc_internal = is_jalr_id ? jalr_target_id :                       // JALR: rs1 + imm
                              (is_jal_id | branch_taken_id) ? branch_target_id :  // JAL/taken branch: PC + imm
                              pc_plus_4_internal;                                 // Default: PC + 4

    assign o_next_pc = next_pc_internal;

    //=========================================================================
    // ID/EX PIPELINE REGISTER
    //=========================================================================
    // Pipeline registers between Instruction Decode and Execute stages
    always @(posedge i_clk) begin
        if (i_rst) begin
            o_pc            <= 32'b0;
            o_rs1_data      <= 32'b0;
            o_rs2_data      <= 32'b0;
            o_imm           <= 32'b0;
            o_rs1           <= 5'b0;
            o_rs2           <= 5'b0;
            o_rd            <= 5'b0;
            o_alu_op        <= 2'b0;
            o_bj_type       <= 3'b0;
            o_alu_src       <= 1'b0;
            o_mem_read      <= 1'b0;
            o_mem_write     <= 1'b0;
            o_mem_to_reg    <= 1'b0;
            o_reg_write     <= 1'b0;
            o_opcode        <= 7'b0010011;  // I-type for NOP
            o_pc_plus_4     <= 32'b0;
            o_funct3        <= 3'b0;
            o_funct7        <= 7'b0;
            o_inst          <= 32'h00000013;  // NOP
            o_valid         <= 1'b0;
            o_is_jal        <= 1'b0;
            o_is_jalr       <= 1'b0;
            o_is_branch     <= 1'b0;
            o_branch_target <= 32'b0;
        end else if (i_rst_stall) begin
            // Insert bubble: set all control signals to create a NOP
            o_pc            <= if_id_pc;
            o_rs1_data      <= 32'b0;
            o_rs2_data      <= 32'b0;
            o_imm           <= 32'b0;
            o_rs1           <= 5'b0;
            o_rs2           <= 5'b0;
            o_rd            <= 5'b0;          // No destination register
            o_alu_op        <= 2'b00;
            o_bj_type       <= 3'b000;
            o_alu_src       <= 1'b0;
            o_mem_read      <= 1'b0;          // No memory read
            o_mem_write     <= 1'b0;          // No memory write
            o_mem_to_reg    <= 1'b0;
            o_reg_write     <= 1'b0;          // No register write (NOP)
            o_opcode        <= 7'b0010011;    // I-type opcode (addi x0, x0, 0)
            o_pc_plus_4     <= if_id_next_pc;
            o_funct3        <= 3'b000;
            o_funct7        <= 7'b0;
            o_inst          <= 32'h00000013;  // NOP instruction encoding
            o_valid         <= 1'b0;          // Mark as invalid
            o_is_jal        <= 1'b0;
            o_is_jalr       <= 1'b0;
            o_is_branch     <= 1'b0;
            o_branch_target <= 32'b0;
        end else if ((~i_dmem_ready) & (i_dmem_valid)) begin
            // Stall if memory not ready, but only if memory has signaled validity
            o_pc            <= o_pc;            
            o_rs1_data      <= o_rs1_data;      
            o_rs2_data      <= o_rs2_data;      
            o_imm           <= o_imm;           
            o_rs1           <= o_rs1;           
            o_rs2           <= o_rs2;           
            o_rd            <= o_rd;             
            o_alu_op        <= o_alu_op;        
            o_bj_type       <= o_bj_type;       
            o_alu_src       <= o_alu_src;       
            o_mem_read      <= o_mem_read;      
            o_mem_write     <= o_mem_write;     
            o_mem_to_reg    <= o_mem_to_reg;    
            o_reg_write     <= o_reg_write;      
            o_opcode        <= o_opcode;         
            o_pc_plus_4     <= o_pc_plus_4;     
            o_funct3        <= o_funct3;        
            o_funct7        <= o_funct7;        
            o_inst          <= o_inst;           
            o_valid         <= o_valid;          
            o_is_jal        <= o_is_jal;        
            o_is_jalr       <= o_is_jalr;       
            o_is_branch     <= o_is_branch;     
            o_branch_target <= o_branch_target; 
        end else begin
            o_pc            <= if_id_pc;
            o_rs1_data      <= rs1_data;
            o_rs2_data      <= rs2_data;
            o_imm           <= imm;
            o_rs1           <= rs1;
            o_rs2           <= rs2;
            o_rd            <= rd;
            o_alu_op        <= alu_op;
            o_bj_type       <= bj_type;
            o_alu_src       <= alu_src;
            o_mem_read      <= mem_read;
            o_mem_write     <= mem_write;
            o_mem_to_reg    <= mem_to_reg;
            o_reg_write     <= reg_write;
            o_opcode        <= opcode;
            o_pc_plus_4     <= if_id_next_pc;
            o_funct3        <= funct3;
            o_funct7        <= funct7;
            o_inst          <= if_id_inst;
            o_valid         <= if_id_valid;
            o_is_jal        <= is_jal_id;
            o_is_jalr       <= is_jalr_id;
            o_is_branch     <= is_branch_id;
            o_branch_target <= is_jalr_id ? jalr_target_id : branch_target_id;
        end
    end

endmodule

`default_nettype wire

//=============================================================================
// END OF ID STAGE MODULE
//=============================================================================
