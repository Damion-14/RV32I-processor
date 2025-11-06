//=============================================================================
// RISC-V RV32I Pipelined Processor (Hart)
//=============================================================================
// This module implements a 5-stage pipelined RISC-V processor supporting
// the RV32I base instruction set. The processor uses the following pipeline stages:
//
// 1. INSTRUCTION FETCH (IF)  - Fetch instruction from memory
// 2. INSTRUCTION DECODE (ID) - Decode instruction, generate control signals, and resolve branches
// 3. EXECUTE (EX)           - ALU operations
// 4. MEMORY ACCESS (MEM)    - Load/store operations with alignment handling
// 5. WRITE BACK (WB)        - Write results back to register file
//
// Branch Resolution: Branches are resolved in ID stage (stage 2) for zero-cycle penalty
// Forwarding: Data forwarding from EX/MEM and MEM/WB to both EX and ID stages
// Hazard Detection: Load-use hazards detected and stalled appropriately
// Architecture: 5-stage pipeline with separate instruction/data memories
// ISA Support: RV32I base integer instruction set
// Features: Unaligned memory access, trap detection, comprehensive retire interface
//=============================================================================

`default_nettype none

module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    //=========================================================================
    // GLOBAL SIGNALS
    //=========================================================================
    input  wire        i_clk,              // Global clock
    input  wire        i_rst,              // Synchronous active-high reset

    //=========================================================================
    // INSTRUCTION MEMORY INTERFACE
    //=========================================================================
    // Instruction fetch goes through a read-only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word.
    output wire [31:0] o_imem_raddr,       // Instruction memory read address (4-byte aligned)
    input  wire [31:0] i_imem_rdata,       // Instruction word fetched from memory

    //=========================================================================
    // DATA MEMORY INTERFACE (Load/Store Operations)
    //=========================================================================
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that supports byte, half-word, and word accesses with proper alignment
    // and masking. Reads are combinational, writes occur on the next clock edge.
    output wire [31:0] o_dmem_addr,        // Data memory address (4-byte aligned)
    output wire        o_dmem_ren,         // Data memory read enable
    output wire        o_dmem_wen,         // Data memory write enable
    output wire [31:0] o_dmem_wdata,       // Data to write to memory
    output wire [ 3:0] o_dmem_mask,        // Byte mask for sub-word accesses
    input  wire [31:0] i_dmem_rdata,       // Data read from memory

    //=========================================================================
    // INSTRUCTION RETIRE INTERFACE (Verification/Debug)
    //=========================================================================
    // The retire interface signals to the testbench that the CPU has completed
    // and retired an instruction. In a single-cycle implementation, this is
    // asserted every cycle unless the processor is stalled or trapped.
    output wire        o_retire_valid,     // Instruction retired this cycle
    output wire [31:0] o_retire_inst,      // Retired instruction word
    output wire        o_retire_trap,      // Trap occurred (illegal inst, misalignment)
    output wire        o_retire_halt,      // EBREAK instruction (halt processor)
    output wire [ 4:0] o_retire_rs1_raddr, // Source register 1 address
    output wire [ 4:0] o_retire_rs2_raddr, // Source register 2 address
    output wire [31:0] o_retire_rs1_rdata, // Source register 1 data
    output wire [31:0] o_retire_rs2_rdata, // Source register 2 data
    output wire [ 4:0] o_retire_rd_waddr,  // Destination register address
    output wire [31:0] o_retire_rd_wdata,  // Destination register data
    output wire [31:0] o_retire_pc,        // Program counter of retired instruction
    output wire [31:0] o_retire_next_pc,   // Next program counter value
    // Data memory interface for retired instruction
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
    // FORWARD DECLARATIONS FOR PIPELINE SIGNALS
    //=========================================================================
    wire stall_pc, stall_if_id, bubble_id_ex;
    wire [31:0] alu_result;
    reg         id_ex_valid;
    reg  [4:0]  id_ex_rd;
    reg         id_ex_reg_write;
    reg  [31:0] ex_mem_alu_result;
    reg  [4:0]  ex_mem_rd;
    reg         ex_mem_reg_write;
    reg         ex_mem_valid;
    reg         ex_mem_mem_to_reg;
    wire [31:0] rd_data;
    wire [31:0] mem_forward_data;
    wire [4:0]  mem_wb_rd;
    wire        mem_wb_reg_write;
    wire        mem_wb_valid;
    wire        flush_if_id;

    // IF/ID pipeline wires driven by dedicated fetch stage
    wire [31:0] if_id_inst;
    wire [31:0] if_id_pc;
    wire [31:0] if_id_next_pc;
    wire        if_id_valid;
    wire [31:0] pc_plus_4;
    wire        rst_stall;

    //=========================================================================
    // STAGE 1: INSTRUCTION FETCH (IF)
    //=========================================================================
    wire [31:0] next_pc;

    hart_if_stage #(
        .RESET_ADDR(RESET_ADDR)
    ) if_stage (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_stall_pc(stall_pc),
        .i_stall_if_id(stall_if_id),
        .i_flush(flush_if_id),
        .i_next_pc(next_pc),
        .i_imem_rdata(i_imem_rdata),
        .o_imem_raddr(o_imem_raddr),
        .o_if_id_inst(if_id_inst),
        .o_if_id_pc(if_id_pc),
        .o_if_id_next_pc(if_id_next_pc),
        .o_if_id_valid(if_id_valid),
        .o_pc_plus_4(pc_plus_4),
        .o_rst_stall(rst_stall)
    );

    // The IF stage is encapsulated inside hart_if_stage.v
    //=========================================================================
    // STAGE 2: INSTRUCTION DECODE (ID)
    //=========================================================================
    // The ID stage decodes the instruction word into its constituent fields,
    // generates control signals, reads from the register file, and generates
    // immediate values. All operations in this stage are combinational.

    //-------------------------------------------------------------------------
    // 2.1: Instruction Field Extraction
    //-------------------------------------------------------------------------
    wire [6:0] opcode;                     // Operation code [6:0]
    wire [4:0] rd, rs1, rs2;               // Register addresses [11:7], [19:15], [24:20]
    wire [2:0] funct3;                     // Function code 3 [14:12]
    wire [6:0] funct7;                     // Function code 7 [31:25]

    assign opcode = if_id_inst[6:0];
    assign rd     = if_id_inst[11:7];
    assign funct3 = if_id_inst[14:12];
    assign rs1    = if_id_inst[19:15];
    assign rs2    = if_id_inst[24:20];
    assign funct7 = if_id_inst[31:25];

    //-------------------------------------------------------------------------
    // 2.2: Control Unit (Main Control Decoder)
    //-------------------------------------------------------------------------
    // Generates primary control signals based on the instruction opcode
    wire [1:0] U_sel;                      // Upper immediate select (unused in current implementation)
    wire [5:0] i_format;                   // Instruction format (R, I, S, B, U, J)
    wire [2:0] bj_type;                    // Branch/jump type for condition evaluation
    wire [1:0] alu_op;                     // ALU operation class
    wire mem_read;                         // Enable data memory read
    wire mem_to_reg;                       // Select memory data for register write
    wire mem_write;                        // Enable data memory write
    wire alu_src;                          // Select immediate vs register for ALU input 2
    wire reg_write;                        // Enable register file write

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

    //-------------------------------------------------------------------------
    // 2.3: Register File Access
    //-------------------------------------------------------------------------
    // The register file provides two read ports and one write port.
    // Register x0 is hardwired to zero. Bypass is disabled for single-cycle.
    wire [31:0] rs1_data, rs2_data;        // Register read data
    wire [31:0] rd_data;                   // Register write data (from WB stage)

    // Register writeback must occur in WB stage; use mem_wb_* signals
    // BYPASS_EN=1 allows reading a register in the same cycle it's being written
    rf #(.BYPASS_EN(1)) rf (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_rs1_raddr(rs1),                 // Read address 1 (ID stage)
        .i_rs2_raddr(rs2),                 // Read address 2 (ID stage)
        .i_rd_waddr(mem_wb_rd),            // Write address (WB stage)
        .i_rd_wdata(rd_data),              // Write data (WB stage mux)
        .i_rd_wen(mem_wb_reg_write),       // Write enable (WB stage)
        .o_rs1_rdata(rs1_data),            // Read data 1
        .o_rs2_rdata(rs2_data)             // Read data 2
    );

    //-------------------------------------------------------------------------
    // 2.4: Immediate Generation
    //-------------------------------------------------------------------------
    // Generates sign-extended immediate values for different instruction formats
    wire [31:0] imm;                       // Generated immediate value

    imm imm_gen (
        .i_inst(if_id_inst),
        .i_format(i_format),
        .o_immediate(imm)
    );

    //-------------------------------------------------------------------------
    // 2.5: Branch and Jump Logic (Moved from EX to ID Stage)
    //-------------------------------------------------------------------------
    // Branch/jump decisions are made in ID stage for zero-cycle branch penalty

    wire is_branch_id;                     // Current instruction is a branch
    wire is_jal_id;                        // Current instruction is JAL
    wire is_jalr_id;                       // Current instruction is JALR

    assign is_branch_id = (opcode == 7'b1100011); // Branch instructions
    assign is_jal_id    = (opcode == 7'b1101111); // Jump and Link
    assign is_jalr_id   = (opcode == 7'b1100111); // Jump and Link Register

    // Branch forwarding unit instantiation
    wire [1:0] forward_branch_a;           // Forward control for branch rs1
    wire [1:0] forward_branch_b;           // Forward control for branch rs2

    branch_forwarding_unit branch_forwarder (
        .i_id_rs1(rs1),
        .i_id_rs2(rs2),
        .i_ex_rd(id_ex_rd),
        .i_ex_reg_write(id_ex_reg_write && id_ex_valid),
        .i_mem_rd(ex_mem_rd),
        .i_mem_reg_write(ex_mem_reg_write && ex_mem_valid),
        .i_wb_rd(mem_wb_rd),
        .i_wb_reg_write(mem_wb_reg_write && mem_wb_valid),
        .o_forward_a(forward_branch_a),
        .o_forward_b(forward_branch_b)
    );

    // Forwarding muxes for branch operands in ID stage
    wire [31:0] branch_rs1_data;           // rs1 data with forwarding for branches
    wire [31:0] branch_rs2_data;           // rs2 data with forwarding for branches

    // Data to forward from EX stage: use ALU result (computed in this cycle)
    wire [31:0] ex_forward_data;
    assign ex_forward_data = alu_result;

    // Data to forward from MEM stage: provided by memory/writeback module
    wire [31:0] mem_forward_data;

    // Forward from ID/EX, EX/MEM, or MEM/WB stage to ID stage for branch operands
    // 00 = No forwarding (use register file data)
    // 01 = Forward from ID/EX (EX stage)
    // 10 = Forward from EX/MEM (MEM stage)
    // 11 = Forward from MEM/WB (WB stage)
    assign branch_rs1_data = (forward_branch_a == 2'b01) ? ex_forward_data :
                             (forward_branch_a == 2'b10) ? mem_forward_data :
                             (forward_branch_a == 2'b11) ? rd_data :
                             rs1_data;

    assign branch_rs2_data = (forward_branch_b == 2'b01) ? ex_forward_data :
                             (forward_branch_b == 2'b10) ? mem_forward_data :
                             (forward_branch_b == 2'b11) ? rd_data :
                             rs2_data;

    // Branch condition evaluation (using rs1 and rs2 with forwarding)
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

    // Flush signal - asserted when control flow change taken in ID stage
    assign flush_if_id = (is_jalr_id | is_jal_id | branch_taken_id);

    // With synchronous imem, the wrong-path instruction (from the address
    // presented this cycle) arrives next cycle. Carry a 1-cycle delayed flush
    // to squash that instruction as well.
    always @(posedge i_clk) begin
        if (i_rst) begin
            flush_if_id_d <= 1'b0;
        end else begin
            flush_if_id_d <= flush_if_id;
        end
    end

    // Target address calculation
    wire [31:0] branch_target_id;          // Branch/JAL target address
    wire [31:0] jalr_target_id;            // JALR target address

    assign branch_target_id = if_id_pc + imm;                        // PC-relative for branches/JAL
    assign jalr_target_id = (branch_rs1_data + imm) & ~32'd1;       // Register+immediate, clear LSB

    // Next PC Selection (controlled by ID stage branch/jump decisions)
    assign next_pc = is_jalr_id ? jalr_target_id :                 // JALR: rs1 + imm
                     (is_jal_id | branch_taken_id) ? branch_target_id : // JAL/taken branch: PC + imm
                     pc_plus_4;                                     // Default: PC + 4

    //-------------------------------------------------------------------------
    // 2.6: ID/EX Pipeline Register
    //-------------------------------------------------------------------------
    // Pipeline registers between Instruction Decode and Execute stages
    reg  [31:0] id_ex_pc;               // ID/EX pipeline register for PC
    reg  [31:0] id_ex_rs1_data;         // ID/EX pipeline register for rs1 data
    reg  [31:0] id_ex_rs2_data;         // ID/EX pipeline register for rs2 data
    reg  [31:0] id_ex_imm;              // ID/EX pipeline register for immediate
    reg  [4:0]  id_ex_rs1;              // ID/EX pipeline register for rs1 address
    reg  [4:0]  id_ex_rs2;              // ID/EX pipeline register for rs2 address
    reg  [1:0]  id_ex_alu_op;           // ID/EX pipeline register for ALU operation
    reg  [2:0]  id_ex_bj_type;          // ID/EX pipeline register for branch/jump type
    reg         id_ex_alu_src;          // ID/EX pipeline register for ALU source select
    reg         id_ex_mem_read;         // ID/EX pipeline register for memory read enable
    reg         id_ex_mem_write;        // ID/EX pipeline register for memory write enable
    reg         id_ex_mem_to_reg;       // ID/EX pipeline register for memory to register select
    reg  [6:0]  id_ex_opcode;           // ID/EX pipeline register for opcode
    reg  [31:0] id_ex_pc_plus_4;        // ID/EX pipeline register for PC + 4
    reg  [2:0]  id_ex_funct3;          // ID/EX pipeline register for funct3
    reg  [6:0]  id_ex_funct7;          // ID/EX pipeline register for funct7
    reg  [31:0] id_ex_inst;            // ID/EX pipeline register for instruction
    reg         id_ex_is_jal;          // ID/EX pipeline register for is_jal
    reg         id_ex_is_jalr;         // ID/EX pipeline register for is_jalr
    reg         id_ex_is_branch;       // ID/EX pipeline register for is_branch
    reg  [31:0] id_ex_branch_target;   // ID/EX pipeline register for branch/jump target

    // ID/EX Pipeline Register
    always @(posedge i_clk) begin
        if (i_rst) begin
            id_ex_pc            <= 32'b0;
            id_ex_rs1_data      <= 32'b0;
            id_ex_rs2_data      <= 32'b0;
            id_ex_imm           <= 32'b0;
            id_ex_rs1           <= 5'b0;
            id_ex_rs2           <= 5'b0;
            id_ex_rd            <= 5'b0;
            id_ex_alu_op        <= 2'b0;
            id_ex_bj_type       <= 3'b0;
            id_ex_alu_src       <= 1'b0;
            id_ex_mem_read      <= 1'b0;
            id_ex_mem_write     <= 1'b0;
            id_ex_mem_to_reg    <= 1'b0;
            id_ex_reg_write     <= 1'b0;
            id_ex_opcode        <= 7'b0010011;  // I-type for NOP
            id_ex_pc_plus_4     <= 32'b0;
            id_ex_funct3        <= 3'b0;
            id_ex_funct7        <= 7'b0;
            id_ex_inst          <= 32'h00000013;  // NOP
            id_ex_valid         <= 1'b0;
            id_ex_is_jal        <= 1'b0;
            id_ex_is_jalr       <= 1'b0;
            id_ex_is_branch     <= 1'b0;
            id_ex_branch_target <= 32'b0;
        end else if (bubble_id_ex) begin
            // Insert bubble: preserve data but deassert all control signals
            // This creates a NOP in the pipeline
            id_ex_pc            <= if_id_pc;
            id_ex_rs1_data      <= id_ex_rs1_data;
            id_ex_rs2_data      <= id_ex_rs2_data;
            id_ex_imm           <= imm;
            id_ex_rs1           <= id_ex_rs1;
            id_ex_rs2           <= id_ex_rs2;
            id_ex_rd            <= 5'b0;          // No destination register
            id_ex_alu_op        <= alu_op;
            id_ex_bj_type       <= bj_type;
            id_ex_alu_src       <= alu_src;
            id_ex_mem_read      <= 1'b0;          // No memory read
            id_ex_mem_write     <= 1'b0;          // No memory write
            id_ex_mem_to_reg    <= 1'b0;
            id_ex_reg_write     <= 1'b0;          // No register write (NOP)
            id_ex_opcode        <= 7'b0010011;    // I-type opcode (addi x0, x0, 0)
            id_ex_pc_plus_4     <= if_id_next_pc;
            id_ex_funct3        <= 3'b000;
            id_ex_funct7        <= 7'b0;
            id_ex_inst          <= 32'h00000013;  // NOP instruction encoding
            id_ex_valid         <= 1'b0;          // Mark as invalid
            id_ex_is_jal        <= 1'b0;
            id_ex_is_jalr       <= 1'b0;
            id_ex_is_branch     <= 1'b0;
            id_ex_branch_target <= 32'b0;
        end else begin
            id_ex_pc            <= if_id_pc;
            id_ex_rs1_data      <= rs1_data;
            id_ex_rs2_data      <= rs2_data;
            id_ex_imm           <= imm;
            id_ex_rs1           <= rs1;
            id_ex_rs2           <= rs2;
            id_ex_rd            <= rd;
            id_ex_alu_op        <= alu_op;
            id_ex_bj_type       <= bj_type;
            id_ex_alu_src       <= alu_src;
            id_ex_mem_read      <= mem_read;
            id_ex_mem_write     <= mem_write;
            id_ex_mem_to_reg    <= mem_to_reg;
            id_ex_reg_write     <= reg_write;
            id_ex_opcode        <= opcode;
            id_ex_pc_plus_4     <= if_id_next_pc;
            id_ex_funct3        <= funct3;
            id_ex_funct7        <= funct7;
            id_ex_inst          <= if_id_inst;
            id_ex_valid         <= if_id_valid;
            id_ex_is_jal        <= is_jal_id;
            id_ex_is_jalr       <= is_jalr_id;
            id_ex_is_branch     <= is_branch_id;
            id_ex_branch_target <= is_jalr_id ? jalr_target_id : branch_target_id;
        end
    end

    //=========================================================================
    // HAZARD DETECTION AND FORWARDING UNITS
    //=========================================================================
    // These units handle pipeline hazards to ensure correct execution
    // without requiring manual insertion of NOPs.

    //-------------------------------------------------------------------------
    // Hazard Detection Unit
    //-------------------------------------------------------------------------
    // Detects RAW hazards and generates stall signals for load-use hazards

    hazard_unit hazard_detector (
        .i_id_rs1(rs1),
        .i_id_rs2(rs2),
        .i_id_is_branch(is_branch_id),
        .i_id_is_jalr(is_jalr_id),
        .i_ex_rd(id_ex_rd),
        .i_ex_reg_write(id_ex_reg_write),
        .i_ex_mem_read(id_ex_mem_read),
        .i_mem_rd(ex_mem_rd),
        .i_mem_reg_write(ex_mem_reg_write),
        .i_rst_stall(rst_stall),
        .o_stall_pc(stall_pc),
        .o_stall_if_id(stall_if_id),
        .o_bubble_id_ex(bubble_id_ex)
    );

    //-------------------------------------------------------------------------
    // Forwarding Unit
    //-------------------------------------------------------------------------
    // Detects when to forward data from MEM or WB stages to EX stage
    wire [1:0] forward_a, forward_b;

    forwarding_unit data_forwarder (
        .i_ex_rs1(id_ex_rs1),
        .i_ex_rs2(id_ex_rs2),
        .i_mem_rd(ex_mem_rd),
        .i_mem_reg_write(ex_mem_reg_write && ex_mem_valid),
        .i_wb_rd(mem_wb_rd),
        .i_wb_reg_write(mem_wb_reg_write && mem_wb_valid),
        .o_forward_a(forward_a),
        .o_forward_b(forward_b)
    );

    //=========================================================================
    // STAGE 3: EXECUTE (EX)
    //=========================================================================
    // The execute stage performs arithmetic operations using the ALU.
    // Branch/jump logic has been moved to ID stage for zero-cycle penalty.

    //-------------------------------------------------------------------------
    // 3.1: ALU Control Unit
    //-------------------------------------------------------------------------
    // Generates detailed ALU control signals based on instruction type and function codes
    wire [2:0] i_opsel;                    // ALU operation select
    wire i_sub;                            // Subtract (vs add)
    wire i_unsigned;                       // Unsigned comparison
    wire i_arith;                          // Arithmetic (vs logical) right shift
    wire [3:0] func37;                     // Combined function codes

    assign func37 = {id_ex_funct7[5], id_ex_funct3};   // Concatenate funct7[5] and funct3

    alu_ctl alu_control_unit (
        .alu_op(id_ex_alu_op),
        .func37(func37),
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith)
    );

    //-------------------------------------------------------------------------
    // 3.2: Forwarding Muxes
    //-------------------------------------------------------------------------
    // Select ALU operands with forwarding from MEM or WB stages
    wire [31:0] forwarded_rs1_data;        // rs1 data after forwarding
    wire [31:0] forwarded_rs2_data;        // rs2 data after forwarding

    // Forwarding Mux for rs1 (ALU operand A)
    // 00 = use data from ID/EX, 01 = forward from EX/MEM, 10 = forward from MEM/WB
    assign forwarded_rs1_data = (forward_a == 2'b01) ? ex_mem_alu_result :
                                (forward_a == 2'b10) ? rd_data :
                                id_ex_rs1_data;

    // Forwarding Mux for rs2 (ALU operand B or store data)
    // 00 = use data from ID/EX, 01 = forward from EX/MEM, 10 = forward from MEM/WB
    assign forwarded_rs2_data = (forward_b == 2'b01) ? ex_mem_alu_result :
                                (forward_b == 2'b10) ? rd_data :
                                id_ex_rs2_data;

    //-------------------------------------------------------------------------
    // 3.3: ALU (Arithmetic Logic Unit)
    //-------------------------------------------------------------------------
    // Performs arithmetic and logical operations on two 32-bit operands
    wire [31:0] alu_op1, alu_op2;          // ALU input operands
    wire alu_eq;                           // Operands are equal
    wire alu_slt;                          // Operand 1 < Operand 2 (signed/unsigned)

    assign alu_op1 = forwarded_rs1_data;                                    // First operand with forwarding
    assign alu_op2 = id_ex_alu_src ? id_ex_imm : forwarded_rs2_data;       // Second operand: immediate or rs2 with forwarding

    alu alu_unit (
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith),
        .i_op1(alu_op1),
        .i_op2(alu_op2),
        .o_result(alu_result),
        .o_eq(alu_eq),
        .o_slt(alu_slt)
    );


    //Pipeline registers between EX and MEM stages
    reg  [31:0] ex_mem_pc;               // EX/MEM pipeline register for PC
    reg  [31:0] ex_mem_rs1_data;         // EX/MEM pipeline register for rs1 data
    reg  [31:0] ex_mem_rs2_data;         // EX/MEM pipeline register for rs2 data
    reg  [31:0] ex_mem_imm;              // EX/MEM pipeline register for immediate
    reg  [4:0]  ex_mem_rs1;              // EX/MEM pipeline register for rs1 address
    reg  [4:0]  ex_mem_rs2;              // EX/MEM pipeline register for rs2 address
    // ex_mem_rd already declared at top
    reg         ex_mem_mem_read;         // EX/MEM pipeline register for memory read enable
    reg         ex_mem_mem_write;        // EX/MEM pipeline register for memory write enable
    // ex_mem_reg_write already declared at top
    reg  [6:0]  ex_mem_opcode;           // EX/MEM pipeline register for opcode
    reg  [31:0] ex_mem_pc_plus_4;        // EX/MEM pipeline register for PC + 4
    // ex_mem_alu_result already declared at top
    reg  [2:0]  ex_mem_funct3;          // EX/MEM pipeline register for funct3
    reg  [6:0]  ex_mem_funct7;          // EX/MEM pipeline register for funct7
    reg         ex_mem_is_jal;           // EX/MEM pipeline register for is_jal
    reg         ex_mem_is_jalr;          // EX/MEM pipeline register for is_jalr
    reg         ex_mem_is_branch;        // EX/MEM pipeline register for is_branch
    reg         ex_mem_is_store;         // EX/MEM pipeline register for is_store
    reg  [31:0] ex_mem_inst;           // EX/MEM pipeline register for instruction
    reg  [31:0] ex_mem_next_pc;          // EX/MEM pipeline register for actual next PC
    reg  [31:0] ex_mem_branch_target;    // EX/MEM pipeline register for branch target (for unaligned check)

    // EX/MEM Pipeline Register
    always @(posedge i_clk) begin
        if (i_rst) begin
            ex_mem_alu_result    <= 32'b0;
            ex_mem_rs2_data      <= 32'b0;
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
            ex_mem_imm           <= 32'b0;
            ex_mem_funct3        <= 3'b0;
            ex_mem_funct7        <= 7'b0;
            ex_mem_is_jal        <= 1'b0;
            ex_mem_is_jalr       <= 1'b0;
            ex_mem_is_branch     <= 1'b0;
            ex_mem_is_store      <= 1'b0;
            ex_mem_inst          <= 32'h00000013;  // NOP
            ex_mem_rs1_data      <= 32'b0;
            ex_mem_valid         <= 1'b0;
            ex_mem_next_pc       <= 32'b0;
            ex_mem_branch_target <= 32'b0;
        end else begin
            ex_mem_alu_result    <= alu_result;
            ex_mem_rs2_data      <= forwarded_rs2_data;  // Original register value for retire interface
            ex_mem_opcode        <= id_ex_opcode;
            ex_mem_pc_plus_4     <= id_ex_pc_plus_4;
            ex_mem_mem_read      <= id_ex_mem_read;
            ex_mem_mem_write     <= id_ex_mem_write;
            ex_mem_mem_to_reg    <= id_ex_mem_to_reg;
            ex_mem_reg_write     <= id_ex_reg_write;
            ex_mem_rd            <= id_ex_rd;
            ex_mem_pc            <= id_ex_pc;
            ex_mem_rs1           <= id_ex_rs1;
            ex_mem_rs2           <= id_ex_rs2;
            ex_mem_imm           <= id_ex_imm;
            ex_mem_funct3        <= id_ex_funct3;
            ex_mem_funct7        <= id_ex_funct7;
            ex_mem_is_jal        <= id_ex_is_jal;
            ex_mem_is_jalr       <= id_ex_is_jalr;
            ex_mem_is_branch     <= id_ex_is_branch;
            ex_mem_is_store      <= (id_ex_opcode == 7'b0100011);
            ex_mem_inst          <= id_ex_inst;
            ex_mem_rs1_data      <= forwarded_rs1_data;  // Original register value for retire interface
            ex_mem_valid         <= id_ex_valid;
            ex_mem_next_pc       <= (id_ex_is_jal | id_ex_is_jalr | id_ex_is_branch) ? id_ex_branch_target : id_ex_pc_plus_4;
            ex_mem_branch_target <= id_ex_branch_target;
        end
    end
    //=========================================================================
    // STAGE 4/5: MEMORY ACCESS AND WRITE BACK
    //=========================================================================
    hart_mem_wb_stage mem_wb_stage (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_dmem_rdata(i_dmem_rdata),
        .i_ex_mem_pc(ex_mem_pc),
        .i_ex_mem_pc_plus_4(ex_mem_pc_plus_4),
        .i_ex_mem_next_pc(ex_mem_next_pc),
        .i_ex_mem_alu_result(ex_mem_alu_result),
        .i_ex_mem_rs1_data(ex_mem_rs1_data),
        .i_ex_mem_rs2_data(ex_mem_rs2_data),
        .i_ex_mem_imm(ex_mem_imm),
        .i_ex_mem_inst(ex_mem_inst),
        .i_ex_mem_branch_target(ex_mem_branch_target),
        .i_ex_mem_rd(ex_mem_rd),
        .i_ex_mem_rs1(ex_mem_rs1),
        .i_ex_mem_rs2(ex_mem_rs2),
        .i_ex_mem_opcode(ex_mem_opcode),
        .i_ex_mem_funct3(ex_mem_funct3),
        .i_ex_mem_mem_read(ex_mem_mem_read),
        .i_ex_mem_mem_write(ex_mem_mem_write),
        .i_ex_mem_mem_to_reg(ex_mem_mem_to_reg),
        .i_ex_mem_reg_write(ex_mem_reg_write),
        .i_ex_mem_is_store(ex_mem_is_store),
        .i_ex_mem_is_jal(ex_mem_is_jal),
        .i_ex_mem_is_jalr(ex_mem_is_jalr),
        .i_ex_mem_is_branch(ex_mem_is_branch),
        .i_ex_mem_valid(ex_mem_valid),
        .o_dmem_addr(o_dmem_addr),
        .o_dmem_ren(o_dmem_ren),
        .o_dmem_wen(o_dmem_wen),
        .o_dmem_wdata(o_dmem_wdata),
        .o_dmem_mask(o_dmem_mask),
        .o_mem_forward_data(mem_forward_data),
        .o_rd_data(rd_data),
        .o_mem_wb_rd(mem_wb_rd),
        .o_mem_wb_reg_write(mem_wb_reg_write),
        .o_mem_wb_valid(mem_wb_valid),
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
