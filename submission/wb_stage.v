//=============================================================================
// RISC-V RV32I Pipelined Processor - Write Back (WB) Stage
//=============================================================================
// This module implements the Write Back (WB) stage of the 5-stage pipelined
// RISC-V processor. The WB stage is responsible for:
// - MEM/WB pipeline registers (at the start of the stage)
// - Selecting the appropriate data to write back to the register file
// - Trap detection (illegal instructions, unaligned accesses)
// - Providing retire interface signals for verification
//
//=============================================================================

`default_nettype none

module wb_stage (
    //=========================================================================
    // GLOBAL SIGNALS
    //=========================================================================
    input  wire        i_clk,              // Global clock
    input  wire        i_rst,              // Synchronous active-high reset

    //=========================================================================
    // INPUTS FROM MEM STAGE (combinational)
    //=========================================================================
    input  wire [31:0] i_mem_read_data,    // Processed memory read data
    input  wire [31:0] i_mem_read_data_raw,// Raw memory read data
    input  wire [31:0] i_alu_result,       // ALU result
    input  wire [4:0]  i_rd,               // rd address
    input  wire        i_mem_to_reg,       // Memory to register select
    input  wire        i_reg_write,        // Register write enable
    input  wire [31:0] i_pc_plus_4,        // PC + 4
    input  wire [6:0]  i_opcode,           // Opcode
    input  wire [31:0] i_imm,              // Immediate value
    input  wire        i_is_jal,           // Is JAL instruction
    input  wire        i_is_jalr,          // Is JALR instruction
    input  wire        i_is_branch,        // Is branch instruction
    input  wire        i_mem_read,         // Memory read enable
    input  wire        i_mem_write,        // Memory write enable
    input  wire [2:0]  i_funct3,           // Function code 3
    input  wire [4:0]  i_rs1,              // rs1 address
    input  wire [4:0]  i_rs2,              // rs2 address
    input  wire [31:0] i_rs1_data,         // rs1 data
    input  wire [31:0] i_rs2_data,         // rs2 data
    input  wire [31:0] i_pc,               // PC
    input  wire [31:0] i_inst,             // Instruction word
    input  wire        i_is_store,         // Is store instruction
    input  wire        i_unaligned_pc,     // Unaligned PC trap flag
    input  wire        i_unaligned_mem,    // Unaligned memory trap flag
    input  wire        i_valid,            // Pipeline valid bit
    input  wire [31:0] i_dmem_addr,        // Memory address
    input  wire [ 1:0] i_byte_offset,      // Byte offset
    input  wire [ 3:0] i_dmem_mask,        // Memory mask
    input  wire [31:0] i_dmem_wdata,       // Memory write data
    input  wire [31:0] i_next_pc,          // Next PC

    //=========================================================================
    // DATA MEMORY INTERFACE (for current cycle read data)
    //=========================================================================
    input  wire [31:0] i_dmem_rdata,       // Current cycle memory read data

    //=========================================================================
    // OUTPUTS TO ID STAGE (Register File Write)
    //=========================================================================
    output wire [4:0]  o_wb_rd,            // Destination register address
    output wire [31:0] o_wb_rd_data,       // Destination register data
    output wire        o_wb_reg_write,     // Register write enable

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
);

    //=========================================================================
    // MEM/WB PIPELINE REGISTERS
    //=========================================================================
    reg  [31:0] mem_wb_mem_read_data;      // Processed memory read data
    reg  [31:0] mem_wb_mem_read_data_raw;  // Raw memory data
    reg  [31:0] mem_wb_alu_result;
    reg  [4:0]  mem_wb_rd;
    reg         mem_wb_mem_to_reg;
    reg         mem_wb_reg_write;
    reg  [31:0] mem_wb_pc_plus_4;
    reg  [6:0]  mem_wb_opcode;
    reg  [31:0] mem_wb_imm;
    reg         mem_wb_is_jal;
    reg         mem_wb_is_jalr;
    reg         mem_wb_is_branch;
    reg         mem_wb_mem_read;
    reg         mem_wb_mem_write;
    reg  [2:0]  mem_wb_funct3;
    reg  [4:0]  mem_wb_rs1;
    reg  [4:0]  mem_wb_rs2;
    reg  [31:0] mem_wb_rs1_data;
    reg  [31:0] mem_wb_rs2_data;
    reg  [31:0] mem_wb_pc;
    reg  [31:0] mem_wb_inst;
    reg         mem_wb_is_store;
    reg         mem_wb_unaligned_pc;
    reg         mem_wb_unaligned_mem;
    reg         mem_wb_valid;
    reg  [31:0] mem_wb_dmem_addr;
    reg  [ 1:0] mem_wb_byte_offset;
    reg  [ 3:0] mem_wb_dmem_mask;
    reg  [31:0] mem_wb_dmem_wdata;
    reg  [31:0] mem_wb_next_pc;

    // MEM/WB Pipeline Register
    always @(posedge i_clk) begin
        if (i_rst) begin
            mem_wb_mem_read_data      <= 32'b0;
            mem_wb_mem_read_data_raw  <= 32'b0;
            mem_wb_alu_result         <= 32'b0;
            mem_wb_rd                 <= 5'b0;
            mem_wb_mem_to_reg         <= 1'b0;
            mem_wb_reg_write          <= 1'b0;
            mem_wb_pc_plus_4          <= 32'b0;
            mem_wb_opcode             <= 7'b0010011;  // I-type for NOP
            mem_wb_imm                <= 32'b0;
            mem_wb_is_jal             <= 1'b0;
            mem_wb_is_jalr            <= 1'b0;
            mem_wb_is_branch          <= 1'b0;
            mem_wb_mem_read           <= 1'b0;
            mem_wb_mem_write          <= 1'b0;
            mem_wb_funct3             <= 3'b0;
            mem_wb_rs1                <= 5'b0;
            mem_wb_rs2                <= 5'b0;
            mem_wb_rs1_data           <= 32'b0;
            mem_wb_rs2_data           <= 32'b0;
            mem_wb_pc                 <= 32'b0;
            mem_wb_inst               <= 32'h00000013;  // NOP
            mem_wb_is_store           <= 1'b0;
            mem_wb_unaligned_pc       <= 1'b0;
            mem_wb_unaligned_mem      <= 1'b0;
            mem_wb_valid              <= 1'b0;
            mem_wb_dmem_addr          <= 32'b0;
            mem_wb_byte_offset        <= 2'b00;
            mem_wb_dmem_mask          <= 4'b0;
            mem_wb_dmem_wdata         <= 32'b0;
            mem_wb_next_pc            <= 32'b0;
        end else begin
            mem_wb_mem_read_data      <= i_mem_read_data;
            mem_wb_mem_read_data_raw  <= i_mem_read_data_raw;
            mem_wb_alu_result         <= i_alu_result;
            mem_wb_rd                 <= i_rd;
            mem_wb_mem_to_reg         <= i_mem_to_reg;
            mem_wb_reg_write          <= i_reg_write;
            mem_wb_pc_plus_4          <= i_pc_plus_4;
            mem_wb_opcode             <= i_opcode;
            mem_wb_imm                <= i_imm;
            mem_wb_is_jal             <= i_is_jal;
            mem_wb_is_jalr            <= i_is_jalr;
            mem_wb_is_branch          <= i_is_branch;
            mem_wb_mem_read           <= i_mem_read;
            mem_wb_mem_write          <= i_mem_write;
            mem_wb_funct3             <= i_funct3;
            mem_wb_rs1                <= i_rs1;
            mem_wb_rs2                <= i_rs2;
            mem_wb_rs1_data           <= i_rs1_data;
            mem_wb_rs2_data           <= i_rs2_data;
            mem_wb_pc                 <= i_pc;
            mem_wb_inst               <= i_inst;
            mem_wb_is_store           <= i_is_store;
            mem_wb_unaligned_pc       <= i_unaligned_pc;
            mem_wb_unaligned_mem      <= i_unaligned_mem;
            mem_wb_valid              <= i_valid;
            mem_wb_dmem_addr          <= i_dmem_addr;
            mem_wb_byte_offset        <= i_byte_offset;
            mem_wb_dmem_mask          <= i_dmem_mask;
            mem_wb_dmem_wdata         <= i_dmem_wdata;
            mem_wb_next_pc            <= i_next_pc;

        end
    end

    //=========================================================================
    // WRITE-BACK DATA SELECTION
    //=========================================================================
    wire is_lui;
    wire is_auipc;

    assign is_lui   = (mem_wb_opcode == 7'b0110111);  // Load Upper Immediate
    assign is_auipc = (mem_wb_opcode == 7'b0010111);  // Add Upper Immediate to PC

    // Reprocess current memory data for retiring load instructions
    wire [1:0] wb_byte_offset;
    assign wb_byte_offset = mem_wb_byte_offset;

    reg [31:0] wb_load_data_processed;
    always @(*) begin
        case (mem_wb_funct3)
            // LB: Load Byte (sign-extended)
            3'b000: begin
                case (wb_byte_offset)
                    2'b00: wb_load_data_processed = {{24{i_dmem_rdata[7]}},  i_dmem_rdata[7:0]};
                    2'b01: wb_load_data_processed = {{24{i_dmem_rdata[15]}}, i_dmem_rdata[15:8]};
                    2'b10: wb_load_data_processed = {{24{i_dmem_rdata[23]}}, i_dmem_rdata[23:16]};
                    default: wb_load_data_processed = {{24{i_dmem_rdata[31]}}, i_dmem_rdata[31:24]};
                endcase
            end

            // LH: Load Half-word (sign-extended)
            3'b001: begin
                case (wb_byte_offset[1])
                    1'b0: wb_load_data_processed = {{16{i_dmem_rdata[15]}}, i_dmem_rdata[15:0]};
                    default: wb_load_data_processed = {{16{i_dmem_rdata[31]}}, i_dmem_rdata[31:16]};
                endcase
            end

            // LW: Load Word (no extension needed)
            3'b010: wb_load_data_processed = i_dmem_rdata;

            // LBU: Load Byte Unsigned (zero-extended)
            3'b100: begin
                case (wb_byte_offset)
                    2'b00: wb_load_data_processed = {24'b0, i_dmem_rdata[7:0]};
                    2'b01: wb_load_data_processed = {24'b0, i_dmem_rdata[15:8]};
                    2'b10: wb_load_data_processed = {24'b0, i_dmem_rdata[23:16]};
                    default: wb_load_data_processed = {24'b0, i_dmem_rdata[31:24]};
                endcase
            end

            // LHU: Load Half-word Unsigned (zero-extended)
            3'b101: begin
                case (wb_byte_offset[1])
                    1'b0: wb_load_data_processed = {16'b0, i_dmem_rdata[15:0]};
                    default: wb_load_data_processed = {16'b0, i_dmem_rdata[31:16]};
                endcase
            end

            default: wb_load_data_processed = i_dmem_rdata;    // Default to word load
        endcase
    end

    // Register Write Data Selection
    wire [31:0] rd_data;
    assign rd_data = mem_wb_mem_to_reg ? wb_load_data_processed :                    // Load instructions
                     (mem_wb_is_jal | mem_wb_is_jalr) ? mem_wb_pc_plus_4 :          // JAL/JALR: return address
                     is_lui ? mem_wb_imm :                                           // LUI: immediate value
                     is_auipc ? (mem_wb_pc + mem_wb_imm) :                          // AUIPC: PC + immediate
                     mem_wb_alu_result;                                              // Default: ALU result

    //=========================================================================
    // TRAP DETECTION
    //=========================================================================
    wire illegal_inst;
    wire unaligned_pc;
    wire unaligned_mem;

    // Illegal Instruction Detection
    wire unsupported_opcode;
    assign unsupported_opcode = ~(mem_wb_opcode == 7'b0110011 ||    // R-type
                                  mem_wb_opcode == 7'b0010011 ||    // I-type ALU
                                  mem_wb_opcode == 7'b0000011 ||    // Load
                                  mem_wb_opcode == 7'b0100011 ||    // Store
                                  mem_wb_opcode == 7'b1100011 ||    // Branch
                                  mem_wb_opcode == 7'b1101111 ||    // JAL
                                  mem_wb_opcode == 7'b1100111 ||    // JALR
                                  mem_wb_opcode == 7'b0110111 ||    // LUI
                                  mem_wb_opcode == 7'b0010111 ||    // AUIPC
                                  mem_wb_opcode == 7'b1110011);     // EBREAK/ECALL

    assign illegal_inst  = mem_wb_valid && unsupported_opcode;
    assign unaligned_pc  = mem_wb_valid && mem_wb_unaligned_pc;
    assign unaligned_mem = mem_wb_valid && mem_wb_unaligned_mem;

    //=========================================================================
    // OUTPUTS TO ID STAGE (Register File Write)
    //=========================================================================
    assign o_wb_rd        = mem_wb_rd;
    assign o_wb_rd_data   = rd_data;
    assign o_wb_reg_write = mem_wb_reg_write && mem_wb_valid;

    //=========================================================================
    // INSTRUCTION RETIRE INTERFACE
    //=========================================================================
    // Retire whenever MEM/WB holds a valid instruction
    assign o_retire_valid = mem_wb_valid;
    assign o_retire_inst       = mem_wb_inst;
    assign o_retire_trap       = illegal_inst | unaligned_pc | unaligned_mem;
    assign o_retire_halt       = o_retire_trap |
                                 (mem_wb_valid && (mem_wb_opcode == 7'b1110011) &&
                                  (mem_wb_funct3 == 3'b000) && (mem_wb_inst[31:20] == 12'h001));
    assign o_retire_rs1_raddr  = mem_wb_rs1;
    assign o_retire_rs2_raddr  = mem_wb_rs2;
    assign o_retire_rs1_rdata  = mem_wb_rs1_data;
    assign o_retire_rs2_rdata  = mem_wb_rs2_data;
    assign o_retire_rd_waddr   = (mem_wb_is_branch || mem_wb_is_store) ? 5'b00000 : mem_wb_rd;
    assign o_retire_rd_wdata   = rd_data;
    assign o_retire_pc         = mem_wb_pc;
    assign o_retire_next_pc    = mem_wb_next_pc;
    assign o_retire_dmem_addr  = mem_wb_dmem_addr;
    assign o_retire_dmem_ren   = mem_wb_mem_read;
    assign o_retire_dmem_wen   = mem_wb_mem_write;
    assign o_retire_dmem_mask  = mem_wb_dmem_mask;
    assign o_retire_dmem_wdata = mem_wb_dmem_wdata;
    assign o_retire_dmem_rdata = i_dmem_rdata;

endmodule

`default_nettype wire

//=============================================================================
// END OF WB STAGE MODULE
//=============================================================================
