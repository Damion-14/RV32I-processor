//=============================================================================
// RISC-V RV32I Pipelined Processor - Memory Access (MEM) Stage
//=============================================================================
// This module implements the Memory Access (MEM) stage of the 5-stage
// pipelined RISC-V processor. The MEM stage is responsible for:
// - EX/MEM pipeline registers (at the start of the stage)
// - Load and store operations
// - Memory address alignment and byte masking
// - Store-to-load forwarding for back-to-back memory operations
// - Data shifting for unaligned accesses
//
// Note: The MEM/WB pipeline registers are located in the WB stage module.
//=============================================================================

`default_nettype none

module mem_stage (
    //=========================================================================
    // GLOBAL SIGNALS
    //=========================================================================
    input  wire        i_clk,              // Global clock
    input  wire        i_rst,              // Synchronous active-high reset

    //=========================================================================
    // INPUTS FROM EX STAGE (combinational)
    //=========================================================================
    input  wire [31:0] i_alu_result,       // ALU result
    input  wire [31:0] i_rs2_data,         // Forwarded rs2 data (post-forwarding from EX)
    input  wire [31:0] i_rs1_data,         // Forwarded rs1 data (post-forwarding from EX)
    input  wire [31:0] i_pc,               // Program counter
    input  wire [4:0]  i_rs1,              // rs1 address
    input  wire [4:0]  i_rs2,              // rs2 address
    input  wire [4:0]  i_rd,               // rd address
    input  wire        i_mem_read,         // Memory read enable
    input  wire        i_mem_write,        // Memory write enable
    input  wire        i_mem_to_reg,       // Memory to register select
    input  wire        i_reg_write,        // Register write enable
    input  wire [6:0]  i_opcode,           // Opcode
    input  wire [31:0] i_pc_plus_4,        // PC + 4
    input  wire [2:0]  i_funct3,           // Function code 3
    input  wire [6:0]  i_funct7,           // Function code 7
    input  wire        i_is_jal,           // Is JAL instruction
    input  wire        i_is_jalr,          // Is JALR instruction
    input  wire        i_is_branch,        // Is branch instruction
    input  wire        i_is_store,         // Is store instruction
    input  wire [31:0] i_inst,             // Instruction word
    input  wire        i_valid,            // Pipeline valid bit
    input  wire [31:0] i_next_pc,          // Next PC value
    input  wire [31:0] i_branch_target,    // Branch target
    input  wire [31:0] i_imm,              // Immediate value

    //=========================================================================
    // DATA MEMORY INTERFACE
    //=========================================================================
    output wire [31:0] o_dmem_addr,        // Data memory address (4-byte aligned)
    output wire        o_dmem_ren,         // Data memory read enable
    output wire        o_dmem_wen,         // Data memory write enable
    output wire [31:0] o_dmem_wdata,       // Data to write to memory
    output wire [ 3:0] o_dmem_mask,        // Byte mask for sub-word accesses
    input  wire [31:0] i_dmem_rdata,       // Data read from memory
    input  wire        i_dmem_ready,       // Data memory ready
    input  wire        i_dmem_valid,       // Data memory data valid
    output wire        o_dcache_busy,      // Data cache is servicing a miss

    //=========================================================================
    // OUTPUTS FOR FORWARDING (registered EX/MEM values)
    //=========================================================================
    output wire [31:0] o_ex_mem_alu_result_reg, // Registered ALU result for forwarding
    output wire [4:0]  o_ex_mem_rd_reg,         // Registered rd for forwarding
    output wire        o_ex_mem_reg_write_reg,  // Registered reg_write for forwarding
    output wire        o_ex_mem_valid_reg,      // Registered valid for forwarding
    output wire        o_ex_mem_mem_to_reg_reg, // Registered mem_to_reg for forwarding

    //=========================================================================
    // OUTPUTS TO WB STAGE (combinational, will be registered in WB stage)
    //=========================================================================
    output wire [31:0] o_mem_read_data,    // Processed memory read data
    output wire [31:0] o_mem_read_data_raw,// Raw memory read data (for retire)
    output wire [31:0] o_alu_result,       // ALU result (passthrough)
    output wire [4:0]  o_rd,               // rd address (passthrough)
    output wire        o_mem_to_reg,       // Memory to register select (passthrough)
    output wire        o_reg_write,        // Register write enable (passthrough)
    output wire [31:0] o_pc_plus_4,        // PC + 4 (passthrough)
    output wire [6:0]  o_opcode,           // Opcode (passthrough)
    output wire [31:0] o_imm,              // Immediate (passthrough)
    output wire        o_is_jal,           // Is JAL (passthrough)
    output wire        o_is_jalr,          // Is JALR (passthrough)
    output wire        o_is_branch,        // Is branch (passthrough)
    output wire        o_mem_read,         // Memory read enable (passthrough)
    output wire        o_mem_write,        // Memory write enable (passthrough)
    output wire [2:0]  o_funct3,           // Function code 3 (passthrough)
    output wire [4:0]  o_rs1,              // rs1 address (passthrough)
    output wire [4:0]  o_rs2,              // rs2 address (passthrough)
    output wire [31:0] o_rs1_data,         // rs1 data (passthrough)
    output wire [31:0] o_rs2_data,         // rs2 data (passthrough)
    output wire [31:0] o_pc,               // PC (passthrough)
    output wire [31:0] o_inst,             // Instruction (passthrough)
    output wire        o_is_store,         // Is store (passthrough)
    output wire        o_unaligned_pc,     // Unaligned PC trap flag
    output wire        o_unaligned_mem,    // Unaligned memory trap flag
    output wire        o_valid,            // Pipeline valid bit (passthrough)
    output wire [31:0] o_dmem_addr_out,    // Memory address for retire
    output wire [ 1:0] o_byte_offset,      // Byte offset for retire
    output wire [ 3:0] o_dmem_mask_out,    // Memory mask for retire
    output wire [31:0] o_dmem_wdata_out,   // Memory write data for retire
    output wire [31:0] o_next_pc           // Next PC (passthrough)
);

    //=========================================================================
    // EX/MEM PIPELINE REGISTERS
    //=========================================================================
    reg  [31:0] ex_mem_alu_result;
    reg  [31:0] ex_mem_rs2_data;
    reg  [31:0] ex_mem_rs1_data;
    reg  [31:0] ex_mem_pc;
    reg  [4:0]  ex_mem_rs1;
    reg  [4:0]  ex_mem_rs2;
    reg  [4:0]  ex_mem_rd;
    reg         ex_mem_mem_read;
    reg         ex_mem_mem_write;
    reg         ex_mem_mem_to_reg;
    reg         ex_mem_reg_write;
    reg  [6:0]  ex_mem_opcode;
    reg  [31:0] ex_mem_pc_plus_4;
    reg  [2:0]  ex_mem_funct3;
    reg  [6:0]  ex_mem_funct7;
    reg         ex_mem_is_jal;
    reg         ex_mem_is_jalr;
    reg         ex_mem_is_branch;
    reg         ex_mem_is_store;
    reg  [31:0] ex_mem_inst;
    reg         ex_mem_valid;
    reg  [31:0] ex_mem_next_pc;
    reg  [31:0] ex_mem_branch_target;
    reg  [31:0] ex_mem_imm;

    // Data cache interface
    wire [31:0] dcache_mem_addr;
    wire        dcache_mem_ren;
    wire        dcache_mem_wen;
    wire [31:0] dcache_mem_wdata;
    wire [31:0] dcache_rdata;
    wire        dcache_busy;
    reg         dcache_waiting;
    reg  [31:0] dcache_req_addr_q;

    // EX/MEM Pipeline Register
    always @(posedge i_clk) begin
        if (i_rst) begin
            ex_mem_alu_result    <= 32'b0;
            ex_mem_rs2_data      <= 32'b0;
            ex_mem_rs1_data      <= 32'b0;
            ex_mem_opcode        <= 7'b0010011;  // I-type for NOP
            ex_mem_pc_plus_4     <= 32'b0;
            ex_mem_mem_read      <= 1'b0;
            ex_mem_mem_write     <= 1'b0;
            ex_mem_mem_to_reg    <= 1'b0;
            ex_mem_reg_write     <= 1'b0;
            ex_mem_rd            <= 5'b0;
            ex_mem_pc            <= 32'b0;
            ex_mem_rs1           <= 5'b0;
            ex_mem_rs2           <= 5'b0;
            ex_mem_funct3        <= 3'b0;
            ex_mem_funct7        <= 7'b0;
            ex_mem_is_jal        <= 1'b0;
            ex_mem_is_jalr       <= 1'b0;
            ex_mem_is_branch     <= 1'b0;
            ex_mem_is_store      <= 1'b0;
            ex_mem_inst          <= 32'h00000013;  // NOP
            ex_mem_valid         <= 1'b0;
            ex_mem_next_pc       <= 32'b0;
            ex_mem_branch_target <= 32'b0;
            ex_mem_imm           <= 32'b0;
        end else if (dcache_busy) begin
            // Hold EX/MEM pipeline registers while data cache services a miss
        end else begin
            ex_mem_alu_result    <= i_alu_result;
            ex_mem_rs2_data      <= i_rs2_data;
            ex_mem_rs1_data      <= i_rs1_data;
            ex_mem_opcode        <= i_opcode;
            ex_mem_pc_plus_4     <= i_pc_plus_4;
            ex_mem_mem_read      <= i_mem_read;
            ex_mem_mem_write     <= i_mem_write;
            ex_mem_mem_to_reg    <= i_mem_to_reg;
            ex_mem_reg_write     <= i_reg_write;
            ex_mem_rd            <= i_rd;
            ex_mem_pc            <= i_pc;
            ex_mem_rs1           <= i_rs1;
            ex_mem_rs2           <= i_rs2;
            ex_mem_funct3        <= i_funct3;
            ex_mem_funct7        <= i_funct7;
            ex_mem_is_jal        <= i_is_jal;
            ex_mem_is_jalr       <= i_is_jalr;
            ex_mem_is_branch     <= i_is_branch;
            ex_mem_is_store      <= i_is_store;
            ex_mem_inst          <= i_inst;
            ex_mem_valid         <= i_valid;
            ex_mem_next_pc       <= i_next_pc;
            ex_mem_branch_target <= i_branch_target;
            ex_mem_imm           <= i_imm;
        end
    end

    //==========================================================================
    // MEMORY ADDRESS CALCULATION AND ALIGNMENT
    //==========================================================================
    wire [31:0] dmem_addr_unaligned;       // Unaligned memory address from ALU
    wire [1:0]  byte_offset;               // Byte offset within 4-byte word
    wire [31:0] cpu_dmem_addr_aligned;

    assign dmem_addr_unaligned = ex_mem_alu_result;           // Address from ALU (rs1 + imm)
    assign byte_offset         = dmem_addr_unaligned[1:0];    // Extract byte offset
    assign cpu_dmem_addr_aligned = {dmem_addr_unaligned[31:2], 2'b00};

    //=========================================================================
    // STORE OPERATIONS (MEMORY WRITES)
    //==========================================================================
    // Generate byte mask based on access size and byte offset
    wire [3:0] dmem_mask;
    assign dmem_mask = (ex_mem_funct3[1:0] == 2'b00) ? (4'b0001 << byte_offset) :  // SB: single byte
                       (ex_mem_funct3[1:0] == 2'b01) ? (4'b0011 << byte_offset) :  // SH: half-word
                       4'b1111;                                                     // SW: full word

    // Shift store data to correct byte lanes based on byte offset
    wire [31:0] store_data_shifted;
    assign store_data_shifted = (byte_offset == 2'b00) ? ex_mem_rs2_data :          // No shift
                                (byte_offset == 2'b01) ? (ex_mem_rs2_data << 8) :   // Shift left 8 bits
                                (byte_offset == 2'b10) ? (ex_mem_rs2_data << 16) :  // Shift left 16 bits
                                (ex_mem_rs2_data << 24);                            // Shift left 24 bits

    //==========================================================================
    // STORE-TO-LOAD FORWARDING (MEM-to-MEM)
    //==========================================================================
    // Previous cycle's store information
    reg [31:0] prev_store_addr;
    reg [31:0] prev_store_data_raw;        // Original rs2 data before shifting
    reg [ 3:0] prev_store_mask;
    reg        prev_store_valid;
    reg [ 2:0] prev_store_funct3;          // Store size (SB/SH/SW)

    always @(posedge i_clk) begin
        if (i_rst) begin
            prev_store_addr     <= 32'b0;
            prev_store_data_raw <= 32'b0;
            prev_store_mask     <= 4'b0;
            prev_store_valid    <= 1'b0;
            prev_store_funct3   <= 3'b0;
        end else begin
            prev_store_addr     <= cpu_dmem_addr_aligned;
            prev_store_data_raw <= store_data_shifted;
            prev_store_mask     <= dmem_mask;
            prev_store_valid    <= ex_mem_mem_write && ex_mem_valid;
            prev_store_funct3   <= ex_mem_funct3;
        end
    end

    // Detect store-to-load forwarding condition
    wire mem_to_mem_forward;
    assign mem_to_mem_forward = prev_store_valid &&              // Previous instruction was a store
                                ex_mem_mem_read &&               // Current instruction is a load
                                (prev_store_addr == cpu_dmem_addr_aligned); // Same aligned address

    // Reconstruct the memory word with forwarded store data
    wire [31:0] dmem_rdata_or_forwarded;
    wire [31:0] forwarded_store_word;

    assign forwarded_store_word = {
        prev_store_mask[3] ? prev_store_data_raw[31:24] : dcache_rdata[31:24],
        prev_store_mask[2] ? prev_store_data_raw[23:16] : dcache_rdata[23:16],
        prev_store_mask[1] ? prev_store_data_raw[15:8]  : dcache_rdata[15:8],
        prev_store_mask[0] ? prev_store_data_raw[7:0]   : dcache_rdata[7:0]
    };

    assign dmem_rdata_or_forwarded = mem_to_mem_forward ? forwarded_store_word : dcache_rdata;

    //=========================================================================
    // LOAD OPERATIONS (MEMORY READS)
    //=========================================================================
    reg [31:0] load_data_processed;

    always @(*) begin
        case (ex_mem_funct3)
            // LB: Load Byte (sign-extended)
            3'b000: begin
                case (byte_offset)
                    2'b00: load_data_processed = {{24{dmem_rdata_or_forwarded[7]}},  dmem_rdata_or_forwarded[7:0]};
                    2'b01: load_data_processed = {{24{dmem_rdata_or_forwarded[15]}}, dmem_rdata_or_forwarded[15:8]};
                    2'b10: load_data_processed = {{24{dmem_rdata_or_forwarded[23]}}, dmem_rdata_or_forwarded[23:16]};
                    default: load_data_processed = {{24{dmem_rdata_or_forwarded[31]}}, dmem_rdata_or_forwarded[31:24]};
                endcase
            end

            // LH: Load Half-word (sign-extended)
            3'b001: begin
                case (byte_offset[1])
                    1'b0: load_data_processed = {{16{dmem_rdata_or_forwarded[15]}}, dmem_rdata_or_forwarded[15:0]};
                    default: load_data_processed = {{16{dmem_rdata_or_forwarded[31]}}, dmem_rdata_or_forwarded[31:16]};
                endcase
            end

            // LW: Load Word (no extension needed)
            3'b010: load_data_processed = dmem_rdata_or_forwarded;

            // LBU: Load Byte Unsigned (zero-extended)
            3'b100: begin
                case (byte_offset)
                    2'b00: load_data_processed = {24'b0, dmem_rdata_or_forwarded[7:0]};
                    2'b01: load_data_processed = {24'b0, dmem_rdata_or_forwarded[15:8]};
                    2'b10: load_data_processed = {24'b0, dmem_rdata_or_forwarded[23:16]};
                    default: load_data_processed = {24'b0, dmem_rdata_or_forwarded[31:24]};
                endcase
            end

            // LHU: Load Half-word Unsigned (zero-extended)
            3'b101: begin
                case (byte_offset[1])
                    1'b0: load_data_processed = {16'b0, dmem_rdata_or_forwarded[15:0]};
                    default: load_data_processed = {16'b0, dmem_rdata_or_forwarded[31:16]};
                endcase
            end

            default: load_data_processed = dmem_rdata_or_forwarded;    // Default to word load
        endcase
    end

        //=========================================================================
        // DATA CACHE AND EXTERNAL MEMORY INTERFACE
        //=========================================================================
        wire mem_access_valid;
        wire dcache_req_fire;
        wire [31:0] dcache_req_addr;
        wire mem_stage_waiting;          // Data cache currently servicing this instruction

        assign mem_access_valid = ex_mem_valid && (ex_mem_mem_read | ex_mem_mem_write);
        assign mem_stage_waiting = mem_access_valid && dcache_busy;
        assign dcache_req_fire  = mem_access_valid && !dcache_waiting;
        assign dcache_req_addr  = dcache_waiting ? dcache_req_addr_q : cpu_dmem_addr_aligned;

        always @(posedge i_clk) begin
            if (i_rst) begin
                dcache_waiting <= 1'b0;
                dcache_req_addr_q <= 32'b0;
            end else if (!dcache_busy) begin
                dcache_waiting <= 1'b0;
            end else if (dcache_req_fire) begin
                dcache_waiting <= 1'b1;
            end

            if (dcache_req_fire) begin
                dcache_req_addr_q <= cpu_dmem_addr_aligned;
            end
        end

        cache dcache (
            .i_clk       (i_clk),
            .i_rst       (i_rst),
            .i_mem_ready (i_dmem_ready),
            .o_mem_addr  (dcache_mem_addr),
            .o_mem_ren   (dcache_mem_ren),
            .o_mem_wen   (dcache_mem_wen),
            .o_mem_wdata (dcache_mem_wdata),
            .i_mem_rdata (i_dmem_rdata),
            .i_mem_valid (i_dmem_valid),
            .o_busy      (dcache_busy),
            .i_req_addr  (dcache_req_addr),
            .i_req_ren   (dcache_req_fire && ex_mem_mem_read),
            .i_req_wen   (dcache_req_fire && ex_mem_mem_write),
            .i_req_mask  (dmem_mask),
            .i_req_wdata (store_data_shifted),
            .o_res_rdata (dcache_rdata)
        );

        assign o_dmem_addr   = dcache_mem_addr;
        assign o_dmem_ren    = dcache_mem_ren;
        assign o_dmem_wen    = dcache_mem_wen;
        assign o_dmem_wdata  = dcache_mem_wdata;
        assign o_dmem_mask   = dcache_mem_wen ? dmem_mask : 4'b0000;
        assign o_dcache_busy = mem_stage_waiting;

    //=========================================================================
    // TRAP DETECTION
    //=========================================================================
    wire unaligned_pc_trap;
    wire unaligned_mem_trap;

    // Check for unaligned branch/jump target
    assign unaligned_pc_trap = (ex_mem_is_jal | ex_mem_is_jalr | ex_mem_is_branch) &&
                               (ex_mem_branch_target[1:0] != 2'b00);

    // Check for unaligned memory access
    assign unaligned_mem_trap = (ex_mem_mem_read | ex_mem_mem_write) &&
                                ((ex_mem_funct3[1:0] == 2'b10 && dmem_addr_unaligned[1:0] != 2'b00) ||
                                 (ex_mem_funct3[1:0] == 2'b01 && dmem_addr_unaligned[0] != 1'b0));

    //=========================================================================
    // OUTPUTS FOR FORWARDING (REGISTERED VALUES)
    //=========================================================================
    assign o_ex_mem_alu_result_reg = ex_mem_alu_result;
    assign o_ex_mem_rd_reg         = ex_mem_rd;
    assign o_ex_mem_reg_write_reg  = ex_mem_reg_write;
    assign o_ex_mem_valid_reg      = ex_mem_valid;
    assign o_ex_mem_mem_to_reg_reg = ex_mem_mem_to_reg;

    //=========================================================================
    // OUTPUTS (COMBINATIONAL PASSTHROUGHS)
    //=========================================================================
    assign o_mem_read_data     = load_data_processed;
    assign o_mem_read_data_raw = dmem_rdata_or_forwarded;
    assign o_alu_result        = ex_mem_alu_result;
    assign o_rd                = ex_mem_rd;
    assign o_mem_to_reg        = ex_mem_mem_to_reg;
    assign o_reg_write         = ex_mem_reg_write;
    assign o_pc_plus_4         = ex_mem_pc_plus_4;
    assign o_opcode            = ex_mem_opcode;
    assign o_imm               = ex_mem_imm;
    assign o_is_jal            = ex_mem_is_jal;
    assign o_is_jalr           = ex_mem_is_jalr;
    assign o_is_branch         = ex_mem_is_branch;
    assign o_mem_read          = ex_mem_mem_read;
    assign o_mem_write         = ex_mem_mem_write;
    assign o_funct3            = ex_mem_funct3;
    assign o_rs1               = ex_mem_rs1;
    assign o_rs2               = ex_mem_rs2;
    assign o_rs1_data          = ex_mem_rs1_data;  // Forwarded operand from EX (registered)
    assign o_rs2_data          = ex_mem_rs2_data;  // Forwarded operand from EX (registered)
    assign o_pc                = ex_mem_pc;
    assign o_inst              = ex_mem_inst;
    assign o_is_store          = ex_mem_is_store;
    assign o_unaligned_pc      = unaligned_pc_trap;
    assign o_unaligned_mem     = unaligned_mem_trap;
    assign o_valid             = ex_mem_valid && !mem_stage_waiting;
    assign o_dmem_addr_out     = cpu_dmem_addr_aligned;
    assign o_byte_offset       = byte_offset;
    assign o_dmem_mask_out     = dmem_mask;
    assign o_dmem_wdata_out    = store_data_shifted;
    assign o_next_pc           = ex_mem_next_pc;

endmodule

`default_nettype wire

//=============================================================================
// END OF MEM STAGE MODULE
//=============================================================================
