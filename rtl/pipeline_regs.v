//=============================================================================
// Pipeline Register Modules
//=============================================================================
// This file contains the pipeline register modules for the 5-stage pipeline.
// Each module encapsulates the registers and update logic for one pipeline
// stage boundary, reducing code duplication in the main hart module.
//=============================================================================

`default_nettype none

//=============================================================================
// IF/ID Pipeline Registers
//=============================================================================
module if_id_regs (
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire        i_flush,
    input  wire        i_stall,        // Stall signal from hazard detection

    // Inputs from IF stage
    input  wire [31:0] i_inst,
    input  wire [31:0] i_pc,
    input  wire [31:0] i_next_pc,
    input  wire        i_valid,

    // Outputs to ID stage
    output reg  [31:0] o_inst,
    output reg  [31:0] o_pc,
    output reg  [31:0] o_next_pc,
    output reg         o_valid
);

    always @(posedge i_clk) begin
        if (i_rst || i_flush) begin
            o_inst    <= 32'b0;
            o_pc      <= 32'b0;
            o_next_pc <= 32'b0;
            o_valid   <= 1'b0;
        end else if (i_stall) begin
            // Hold current values during stall
            o_inst    <= o_inst;
            o_pc      <= o_pc;
            o_next_pc <= o_next_pc;
            o_valid   <= o_valid;
        end else begin
            o_inst    <= i_inst;
            o_pc      <= i_pc;
            o_next_pc <= i_next_pc;
            o_valid   <= i_valid;
        end
    end

endmodule

//=============================================================================
// ID/EX Pipeline Registers
//=============================================================================
module id_ex_regs (
    input  wire        i_clk,
    input  wire        i_rst,
    input  wire        i_bubble,       // Insert bubble (NOP) signal

    // Inputs from ID stage
    input  wire [31:0] i_pc,
    input  wire [31:0] i_rs1_data,
    input  wire [31:0] i_rs2_data,
    input  wire [31:0] i_imm,
    input  wire [4:0]  i_rs1,
    input  wire [4:0]  i_rs2,
    input  wire [4:0]  i_rd,
    input  wire [1:0]  i_alu_op,
    input  wire [2:0]  i_bj_type,
    input  wire        i_alu_src,
    input  wire        i_mem_read,
    input  wire        i_mem_write,
    input  wire        i_mem_to_reg,
    input  wire        i_reg_write,
    input  wire [6:0]  i_opcode,
    input  wire [31:0] i_pc_plus_4,
    input  wire [2:0]  i_funct3,
    input  wire [6:0]  i_funct7,
    input  wire [31:0] i_inst,
    input  wire        i_valid,

    // Outputs to EX stage
    output reg  [31:0] o_pc,
    output reg  [31:0] o_rs1_data,
    output reg  [31:0] o_rs2_data,
    output reg  [31:0] o_imm,
    output reg  [4:0]  o_rs1,
    output reg  [4:0]  o_rs2,
    output reg  [4:0]  o_rd,
    output reg  [1:0]  o_alu_op,
    output reg  [2:0]  o_bj_type,
    output reg         o_alu_src,
    output reg         o_mem_read,
    output reg         o_mem_write,
    output reg         o_mem_to_reg,
    output reg         o_reg_write,
    output reg  [6:0]  o_opcode,
    output reg  [31:0] o_pc_plus_4,
    output reg  [2:0]  o_funct3,
    output reg  [6:0]  o_funct7,
    output reg  [31:0] o_inst,
    output reg         o_valid
);

    always @(posedge i_clk) begin
        if (i_rst) begin
            o_pc         <= 32'b0;
            o_rs1_data   <= 32'b0;
            o_rs2_data   <= 32'b0;
            o_imm        <= 32'b0;
            o_rs1        <= 5'b0;
            o_rs2        <= 5'b0;
            o_rd         <= 5'b0;
            o_alu_op     <= 2'b0;
            o_bj_type    <= 3'b0;
            o_alu_src    <= 1'b0;
            o_mem_read   <= 1'b0;
            o_mem_write  <= 1'b0;
            o_mem_to_reg <= 1'b0;
            o_reg_write  <= 1'b0;
            o_opcode     <= 7'b0;
            o_pc_plus_4  <= 32'b0;
            o_funct3     <= 3'b0;
            o_funct7     <= 7'b0;
            o_inst       <= 32'b0;
            o_valid      <= 1'b0;
        end else if (i_bubble) begin
            // Insert bubble: preserve data but deassert all control signals
            // This creates a NOP in the pipeline
            o_pc         <= i_pc;
            o_rs1_data   <= i_rs1_data;
            o_rs2_data   <= i_rs2_data;
            o_imm        <= i_imm;
            o_rs1        <= i_rs1;
            o_rs2        <= i_rs2;
            o_rd         <= 5'b0;          // No destination register
            o_alu_op     <= i_alu_op;
            o_bj_type    <= i_bj_type;
            o_alu_src    <= i_alu_src;
            o_mem_read   <= 1'b0;          // No memory read
            o_mem_write  <= 1'b0;          // No memory write
            o_mem_to_reg <= 1'b0;
            o_reg_write  <= 1'b0;          // No register write (NOP)
            o_opcode     <= 7'b0010011;    // I-type opcode (addi x0, x0, 0)
            o_pc_plus_4  <= i_pc_plus_4;
            o_funct3     <= 3'b000;
            o_funct7     <= 7'b0;
            o_inst       <= 32'h00000013;  // NOP instruction encoding
            o_valid      <= 1'b0;          // Mark as invalid
        end else begin
            o_pc         <= i_pc;
            o_rs1_data   <= i_rs1_data;
            o_rs2_data   <= i_rs2_data;
            o_imm        <= i_imm;
            o_rs1        <= i_rs1;
            o_rs2        <= i_rs2;
            o_rd         <= i_rd;
            o_alu_op     <= i_alu_op;
            o_bj_type    <= i_bj_type;
            o_alu_src    <= i_alu_src;
            o_mem_read   <= i_mem_read;
            o_mem_write  <= i_mem_write;
            o_mem_to_reg <= i_mem_to_reg;
            o_reg_write  <= i_reg_write;
            o_opcode     <= i_opcode;
            o_pc_plus_4  <= i_pc_plus_4;
            o_funct3     <= i_funct3;
            o_funct7     <= i_funct7;
            o_inst       <= i_inst;
            o_valid      <= i_valid;
        end
    end

endmodule

//=============================================================================
// EX/MEM Pipeline Registers
//=============================================================================
module ex_mem_regs (
    input  wire        i_clk,
    input  wire        i_rst,

    // Inputs from EX stage
    input  wire [31:0] i_pc,
    input  wire [31:0] i_rs1_data,
    input  wire [31:0] i_rs2_data,
    input  wire [31:0] i_imm,
    input  wire [4:0]  i_rs1,
    input  wire [4:0]  i_rs2,
    input  wire [4:0]  i_rd,
    input  wire        i_mem_read,
    input  wire        i_mem_write,
    input  wire        i_mem_to_reg,
    input  wire        i_reg_write,
    input  wire [6:0]  i_opcode,
    input  wire [31:0] i_pc_plus_4,
    input  wire [31:0] i_alu_result,
    input  wire [2:0]  i_funct3,
    input  wire [6:0]  i_funct7,
    input  wire        i_is_jal,
    input  wire        i_is_jalr,
    input  wire        i_is_branch,
    input  wire        i_is_store,
    input  wire [31:0] i_inst,
    input  wire        i_unaligned_pc,
    input  wire        i_valid,

    // Outputs to MEM stage
    output reg  [31:0] o_pc,
    output reg  [31:0] o_rs1_data,
    output reg  [31:0] o_rs2_data,
    output reg  [31:0] o_imm,
    output reg  [4:0]  o_rs1,
    output reg  [4:0]  o_rs2,
    output reg  [4:0]  o_rd,
    output reg         o_mem_read,
    output reg         o_mem_write,
    output reg         o_mem_to_reg,
    output reg         o_reg_write,
    output reg  [6:0]  o_opcode,
    output reg  [31:0] o_pc_plus_4,
    output reg  [31:0] o_alu_result,
    output reg  [2:0]  o_funct3,
    output reg  [6:0]  o_funct7,
    output reg         o_is_jal,
    output reg         o_is_jalr,
    output reg         o_is_branch,
    output reg         o_is_store,
    output reg  [31:0] o_inst,
    output reg         o_unaligned_pc,
    output reg         o_valid
);

    always @(posedge i_clk) begin
        if (i_rst) begin
            o_alu_result    <= 32'b0;
            o_rs2_data      <= 32'b0;
            o_opcode        <= 7'b0;
            o_pc_plus_4     <= 32'b0;
            o_mem_read      <= 1'b0;
            o_mem_write     <= 1'b0;
            o_mem_to_reg    <= 1'b0;
            o_reg_write     <= 1'b0;
            o_rd            <= 5'b0;
            o_pc            <= 32'b0;
            o_rs1           <= 5'b0;
            o_rs2           <= 5'b0;
            o_imm           <= 32'b0;
            o_funct3        <= 3'b0;
            o_funct7        <= 7'b0;
            o_is_jal        <= 1'b0;
            o_is_jalr       <= 1'b0;
            o_is_branch     <= 1'b0;
            o_is_store      <= 1'b0;
            o_inst          <= 32'b0;
            o_rs1_data      <= 32'b0;
            o_unaligned_pc  <= 1'b0;
            o_valid         <= 1'b0;
        end else begin
            o_alu_result    <= i_alu_result;
            o_rs2_data      <= i_rs2_data;
            o_opcode        <= i_opcode;
            o_pc_plus_4     <= i_pc_plus_4;
            o_mem_read      <= i_mem_read;
            o_mem_write     <= i_mem_write;
            o_mem_to_reg    <= i_mem_to_reg;
            o_reg_write     <= i_reg_write;
            o_rd            <= i_rd;
            o_pc            <= i_pc;
            o_rs1           <= i_rs1;
            o_rs2           <= i_rs2;
            o_imm           <= i_imm;
            o_funct3        <= i_funct3;
            o_funct7        <= i_funct7;
            o_is_jal        <= i_is_jal;
            o_is_jalr       <= i_is_jalr;
            o_is_branch     <= i_is_branch;
            o_is_store      <= i_is_store;
            o_inst          <= i_inst;
            o_rs1_data      <= i_rs1_data;
            o_unaligned_pc  <= i_unaligned_pc;
            o_valid         <= i_valid;
        end
    end

endmodule

//=============================================================================
// MEM/WB Pipeline Registers
//=============================================================================
module mem_wb_regs (
    input  wire        i_clk,
    input  wire        i_rst,

    // Inputs from MEM stage
    input  wire [31:0] i_mem_read_data,
    input  wire [31:0] i_alu_result,
    input  wire [4:0]  i_rd,
    input  wire        i_mem_to_reg,
    input  wire        i_reg_write,
    input  wire [31:0] i_pc_plus_4,
    input  wire [6:0]  i_opcode,
    input  wire [31:0] i_imm,
    input  wire        i_is_jal,
    input  wire        i_is_jalr,
    input  wire        i_is_branch,
    input  wire        i_mem_read,
    input  wire        i_mem_write,
    input  wire [2:0]  i_funct3,
    input  wire [4:0]  i_rs1,
    input  wire [4:0]  i_rs2,
    input  wire [31:0] i_rs1_data,
    input  wire [31:0] i_rs2_data,
    input  wire [31:0] i_pc,
    input  wire [31:0] i_inst,
    input  wire        i_is_store,
    input  wire        i_unaligned_pc,
    input  wire        i_unaligned_mem,
    input  wire        i_valid,
    // Memory interface signals for retire interface
    input  wire [31:0] i_dmem_addr,
    input  wire [ 3:0] i_dmem_mask,
    input  wire [31:0] i_dmem_wdata,

    // Outputs to WB stage
    output reg  [31:0] o_mem_read_data,
    output reg  [31:0] o_alu_result,
    output reg  [4:0]  o_rd,
    output reg         o_mem_to_reg,
    output reg         o_reg_write,
    output reg  [31:0] o_pc_plus_4,
    output reg  [6:0]  o_opcode,
    output reg  [31:0] o_imm,
    output reg         o_is_jal,
    output reg         o_is_jalr,
    output reg         o_is_branch,
    output reg         o_mem_read,
    output reg         o_mem_write,
    output reg  [2:0]  o_funct3,
    output reg  [4:0]  o_rs1,
    output reg  [4:0]  o_rs2,
    output reg  [31:0] o_rs1_data,
    output reg  [31:0] o_rs2_data,
    output reg  [31:0] o_pc,
    output reg  [31:0] o_inst,
    output reg         o_is_store,
    output reg         o_unaligned_pc,
    output reg         o_unaligned_mem,
    output reg         o_valid,
    // Memory interface signals for retire interface
    output reg  [31:0] o_dmem_addr,
    output reg  [ 3:0] o_dmem_mask,
    output reg  [31:0] o_dmem_wdata
);

    always @(posedge i_clk) begin
        if (i_rst) begin
            o_mem_read_data  <= 32'b0;
            o_alu_result     <= 32'b0;
            o_rd             <= 5'b0;
            o_mem_to_reg     <= 1'b0;
            o_reg_write      <= 1'b0;
            o_pc_plus_4      <= 32'b0;
            o_opcode         <= 7'b0;
            o_imm            <= 32'b0;
            o_is_jal         <= 1'b0;
            o_is_jalr        <= 1'b0;
            o_is_branch      <= 1'b0;
            o_mem_read       <= 1'b0;
            o_mem_write      <= 1'b0;
            o_funct3         <= 3'b0;
            o_rs1            <= 5'b0;
            o_rs2            <= 5'b0;
            o_rs1_data       <= 32'b0;
            o_rs2_data       <= 32'b0;
            o_pc             <= 32'b0;
            o_inst           <= 32'b0;
            o_is_store       <= 1'b0;
            o_unaligned_pc   <= 1'b0;
            o_unaligned_mem  <= 1'b0;
            o_valid          <= 1'b0;
            o_dmem_addr      <= 32'b0;
            o_dmem_mask      <= 4'b0;
            o_dmem_wdata     <= 32'b0;
        end else begin
            o_mem_read_data  <= i_mem_read_data;
            o_alu_result     <= i_alu_result;
            o_rd             <= i_rd;
            o_mem_to_reg     <= i_mem_to_reg;
            o_reg_write      <= i_reg_write;
            o_pc_plus_4      <= i_pc_plus_4;
            o_opcode         <= i_opcode;
            o_imm            <= i_imm;
            o_is_jal         <= i_is_jal;
            o_is_jalr        <= i_is_jalr;
            o_is_branch      <= i_is_branch;
            o_mem_read       <= i_mem_read;
            o_mem_write      <= i_mem_write;
            o_funct3         <= i_funct3;
            o_rs1            <= i_rs1;
            o_rs2            <= i_rs2;
            o_rs1_data       <= i_rs1_data;
            o_rs2_data       <= i_rs2_data;
            o_pc             <= i_pc;
            o_inst           <= i_inst;
            o_is_store       <= i_is_store;
            o_unaligned_pc   <= i_unaligned_pc;
            o_unaligned_mem  <= i_unaligned_mem;
            o_valid          <= i_valid;
            o_dmem_addr      <= i_dmem_addr;
            o_dmem_mask      <= i_dmem_mask;
            o_dmem_wdata     <= i_dmem_wdata;
        end
    end

endmodule

`default_nettype wire
