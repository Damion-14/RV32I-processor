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
    // These signals are used across multiple pipeline stages and need to be
    // declared early to avoid "undefined variable" errors
    
    // Hazard detection and control signals
    wire stall_pc, stall_if_id, bubble_id_ex;
    
    // EX/MEM pipeline registers (needed for forwarding in EX stage)
    reg  [31:0] ex_mem_alu_result;
    reg  [4:0]  ex_mem_rd;
    reg         ex_mem_reg_write;
    reg         ex_mem_valid;            // EX/MEM pipeline valid bit

    
    // MEM/WB pipeline registers (needed for RF write and forwarding)
    reg  [4:0]  mem_wb_rd;
    reg         mem_wb_reg_write;
    reg         mem_wb_valid;            // MEM/WB pipeline valid bit

    reg         ex_mem_mem_to_reg;       // EX/MEM pipeline register for memory to register select
    wire [31:0] mem_read_data;

    //=========================================================================
    // STAGE 1: INSTRUCTION FETCH (IF)
    //=========================================================================
    // The IF stage manages the program counter and fetches instructions from
    // instruction memory. The PC is updated based on control flow decisions
    // made in the decode stage (branches, jumps, or sequential execution).

    // Program Counter (PC) Register and Logic
    reg  [31:0] pc;                        // Current program counter
    reg         rst_store;
    wire [31:0] pc_plus_4;                 // PC + 4 for sequential execution
    wire [31:0] next_pc;                   // Next PC value (from ID stage)
    wire        rst_stall;

    assign pc_plus_4 = pc + 32'd4;         // Calculate next sequential PC
        assign o_imem_raddr = pc;              // Send current PC to instruction memory

    // PC Update (Synchronous)
    // PC is updated every clock cycle unless stalled by hazard detection
    always @(posedge i_clk) begin
        if (i_rst) begin
            pc <= RESET_ADDR;            // Reset PC to specified address
        end else if (!stall_pc) begin
            pc <= next_pc;  
        end             // else: PC holds its current value during stall
        rst_store <= i_rst;
    end
    assign rst_stall = rst_store;

    // Instruction Fetch from Memory
    wire [31:0] inst;                      // Current instruction word
    wire flush_if_id;
    reg flush_if_id_d;
    assign inst = i_imem_rdata;            // Instruction returned from (synchronous) imem

    // The testbench models imem as synchronous (1-cycle latency). The instruction
    // observed on 'inst' corresponds to the address presented on 'o_imem_raddr'
    // in the previous cycle. Track that address so we can align IF/ID.pc with
    // the arriving instruction correctly.
    reg [31:0] fetch_pc;                   // PC used for the instruction arriving this cycle
    always @(posedge i_clk) begin
        if (i_rst) begin
            fetch_pc <= RESET_ADDR;
        end else if (!stall_pc) begin
            fetch_pc <= pc;                // Address driven to imem this cycle
        end else begin
            fetch_pc <= fetch_pc;          // Hold during stall
        end
    end

    reg  [31:0] if_id_inst;             // IF/ID pipeline register for instruction
    reg  [31:0] if_id_pc;               // IF/ID pipeline register for PC
    reg  [31:0] if_id_next_pc;          // IF/ID pipeline register for next PC
    reg         if_id_valid;            // IF/ID pipeline valid bi

    // IF/ID Pipeline Register
    always @(posedge i_clk) begin
        if (i_rst) begin
            if_id_inst    <= 32'b0;
            if_id_pc      <= 32'b0;
            if_id_next_pc <= 32'b0;
            if_id_valid   <= 1'b0;
        end else if (flush_if_id | flush_if_id_d) begin
            // Flush pipeline: insert bubble (NOP)
            if_id_inst    <= 32'h00000013;  // NOP instruction
            if_id_pc      <= 32'b0;
            if_id_next_pc <= 32'b0;
            if_id_valid   <= 1'b0;
        end else if (stall_if_id) begin
            // Hold current values during stall; preserve valid to avoid dropping the instruction
            if_id_inst    <= if_id_inst;
            if_id_pc      <= if_id_pc;
            if_id_next_pc <= if_id_next_pc;
            if_id_valid   <= if_id_valid;
        end else begin
            // Align IF/ID PC with the instruction returned by synchronous imem
            if_id_inst    <= inst;
            if_id_pc      <= fetch_pc;
            if_id_next_pc <= fetch_pc + 32'd4;
            if_id_valid   <= 1'b1;
        end
    end
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

    // Data to forward from MEM stage: use memory read data for loads, ALU result otherwise
    wire [31:0] mem_forward_data;
    assign mem_forward_data = ex_mem_mem_to_reg ? mem_read_data : ex_mem_alu_result;

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
    reg  [4:0]  id_ex_rd;               // ID/EX pipeline register for rd address
    reg  [1:0]  id_ex_alu_op;           // ID/EX pipeline register for ALU operation
    reg  [2:0]  id_ex_bj_type;          // ID/EX pipeline register for branch/jump type
    reg         id_ex_alu_src;          // ID/EX pipeline register for ALU source select
    reg         id_ex_mem_read;         // ID/EX pipeline register for memory read enable
    reg         id_ex_mem_write;        // ID/EX pipeline register for memory write enable
    reg         id_ex_mem_to_reg;       // ID/EX pipeline register for memory to register select
    reg         id_ex_reg_write;        // ID/EX pipeline register for register write enable
    reg  [6:0]  id_ex_opcode;           // ID/EX pipeline register for opcode
    reg  [31:0] id_ex_pc_plus_4;        // ID/EX pipeline register for PC + 4
    reg  [2:0]  id_ex_funct3;          // ID/EX pipeline register for funct3
    reg  [6:0]  id_ex_funct7;          // ID/EX pipeline register for funct7
    reg  [31:0] id_ex_inst;            // ID/EX pipeline register for instruction
    reg         id_ex_valid;           // ID/EX pipeline valid bit
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
    wire [31:0] alu_result;                // ALU output result
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
    // STAGE 4: MEMORY ACCESS (MEM)
    //=========================================================================
    // The memory stage handles load and store operations with support for
    // byte, half-word, and word accesses at aligned and unaligned addresses.
    // It also handles the necessary data shifting and masking operations.

    //-------------------------------------------------------------------------
    // 4.0: Store-to-Load Forwarding (MEM-to-MEM)
    //-------------------------------------------------------------------------
    // Detect when a load reads from the same address a store is writing to.
    // Since memory is synchronous (writes take effect on clock edge), we need
    // to forward store data when a load follows a store to the same address.

    // Previous cycle's store information
    reg [31:0] prev_store_addr;
    reg [31:0] prev_store_data_raw;  // Original rs2 data before shifting
    reg [ 3:0] prev_store_mask;
    reg        prev_store_valid;
    reg [ 2:0] prev_store_funct3;    // Store size (SB/SH/SW)

    always @(posedge i_clk) begin
        if (i_rst) begin
            prev_store_addr     <= 32'b0;
            prev_store_data_raw <= 32'b0;
            prev_store_mask     <= 4'b0;
            prev_store_valid    <= 1'b0;
            prev_store_funct3   <= 3'b0;
        end else begin
            prev_store_addr     <= o_dmem_addr;
            prev_store_data_raw <= ex_mem_rs2_data;  // Store the original unshifted data
            prev_store_mask     <= o_dmem_mask;
            prev_store_valid    <= o_dmem_wen;
            prev_store_funct3   <= ex_mem_funct3;
        end
    end

    // Detect store-to-load forwarding condition
    wire mem_to_mem_forward;
    assign mem_to_mem_forward = prev_store_valid &&           // Previous instruction was a store
                                ex_mem_mem_read &&            // Current instruction is a load
                                (prev_store_addr == o_dmem_addr); // Same aligned address

    // Reconstruct the memory word with forwarded store data
    // We need to merge the stored bytes into the memory word based on the store mask
    wire [31:0] dmem_rdata_or_forwarded;
    wire [31:0] forwarded_store_word;

    // Reconstruct what the memory word looks like after the store
    // based on which bytes were written (indicated by prev_store_mask)
    // Each byte of the memory word comes from either the stored data or existing memory
    assign forwarded_store_word = {
        prev_store_mask[3] ? prev_store_data_raw[31:24] : i_dmem_rdata[31:24],
        prev_store_mask[2] ? prev_store_data_raw[23:16] : i_dmem_rdata[23:16],
        prev_store_mask[1] ? prev_store_data_raw[15:8]  : i_dmem_rdata[15:8],
        prev_store_mask[0] ? prev_store_data_raw[7:0]   : i_dmem_rdata[7:0]
    };

    assign dmem_rdata_or_forwarded = mem_to_mem_forward ? forwarded_store_word : i_dmem_rdata;

    //-------------------------------------------------------------------------
    // 4.1: Memory Address Calculation and Alignment
    //-------------------------------------------------------------------------
    wire [31:0] dmem_addr_unaligned;       // Unaligned memory address from ALU
    wire [1:0]  byte_offset;               // Byte offset within 4-byte word

    assign dmem_addr_unaligned = ex_mem_alu_result;           // Address from ALU (rs1 + imm)
    assign byte_offset = dmem_addr_unaligned[1:0];     // Extract byte offset
    assign o_dmem_addr = {dmem_addr_unaligned[31:2], 2'b00}; // Align to 4-byte boundary

    // Memory Control Signals
    assign o_dmem_ren = ex_mem_mem_read;          // Read enable from control unit
    assign o_dmem_wen = ex_mem_mem_write;         // Write enable from control unit

    //-------------------------------------------------------------------------
    // 4.2: Store Operations (Memory Writes)
    //-------------------------------------------------------------------------
    // For store operations, data must be shifted to the correct byte lanes
    // and the appropriate mask must be generated based on access size.

    // Generate byte mask based on access size and byte offset
    wire [3:0] dmem_mask;
    assign dmem_mask = (ex_mem_funct3[1:0] == 2'b00) ? (4'b0001 << byte_offset) :  // SB: single byte
                       (ex_mem_funct3[1:0] == 2'b01) ? (4'b0011 << byte_offset) :  // SH: half-word (2 bytes)
                       4'b1111;                                             // SW: full word (4 bytes) / default
    assign o_dmem_mask = dmem_mask;

    // Shift store data to correct byte lanes based on byte offset
    wire [31:0] store_data_shifted;
    assign store_data_shifted = (byte_offset == 2'b00) ? ex_mem_rs2_data :          // No shift needed
                                (byte_offset == 2'b01) ? (ex_mem_rs2_data << 8) :   // Shift left 8 bits
                                (byte_offset == 2'b10) ? (ex_mem_rs2_data << 16) :  // Shift left 16 bits
                                (ex_mem_rs2_data << 24);                            // Shift left 24 bits
    assign o_dmem_wdata = store_data_shifted;

    //-------------------------------------------------------------------------
    // 4.3: Load Operations (Memory Reads)
    //-------------------------------------------------------------------------
    // For load operations, data must be extracted from the correct byte lanes,
    // shifted to the LSBs, and sign/zero extended based on the instruction type.

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

    assign mem_read_data = load_data_processed;

    //-------------------------------------------------------------------------
    // 4.4: MEM/WB Pipeline Register
    //-------------------------------------------------------------------------
    reg  [31:0] mem_wb_mem_read_data;     // MEM/WB pipeline register for memory read data (processed)
    reg  [31:0] mem_wb_mem_read_data_raw; // MEM/WB pipeline register for raw memory data (for retire interface)
    reg  [31:0] mem_wb_alu_result;        // MEM/WB pipeline register for ALU result
    // mem_wb_rd already declared at top
    reg         mem_wb_mem_to_reg;        // MEM/WB pipeline register for memory to register select
    // mem_wb_reg_write already declared at top (reg for RF)
    reg  [31:0] mem_wb_pc_plus_4;        // MEM/WB pipeline register for PC + 4
    reg  [6:0]  mem_wb_opcode;           // MEM/WB pipeline register for opcode
    reg  [31:0] mem_wb_imm;              // MEM/WB pipeline register for immediate value
    reg         mem_wb_is_jal;           // MEM/WB pipeline register for is_jal
    reg         mem_wb_is_jalr;          // MEM/WB pipeline register for is_jalr
    reg         mem_wb_is_branch;        // MEM/WB pipeline register for is_branch
    reg         mem_wb_mem_read;         // MEM/WB pipeline register for memory read enable
    reg         mem_wb_mem_write;        // MEM/WB pipeline register for memory write enable
    reg  [2:0]  mem_wb_funct3;           // MEM/WB pipeline register for funct3
    reg  [4:0]  mem_wb_rs1;              // MEM/WB pipeline register for rs1 address
    reg  [4:0]  mem_wb_rs2;              // MEM/WB pipeline register for rs2 address
    reg  [31:0] mem_wb_rs1_data;         // MEM/WB pipeline register for rs1 data
    reg  [31:0] mem_wb_rs2_data;         // MEM/WB pipeline register for rs2 data
    reg  [31:0] mem_wb_pc;               // MEM/WB pipeline register for PC
    reg  [31:0] mem_wb_inst;             // MEM/WB pipeline register for instruction
    reg         mem_wb_is_store;         // MEM/WB pipeline register for is_store
    reg         mem_wb_unaligned_pc;     // MEM/WB pipeline register for unaligned PC trap flag
    reg         mem_wb_unaligned_mem;    // MEM/WB pipeline register for unaligned MEM trap flag
    reg  [31:0] mem_wb_dmem_addr;        // MEM/WB pipeline register for memory address
    reg  [ 3:0] mem_wb_dmem_mask;        // MEM/WB pipeline register for memory mask
    reg  [31:0] mem_wb_dmem_wdata;       // MEM/WB pipeline register for memory write data
    reg  [31:0] mem_wb_next_pc;          // MEM/WB pipeline register for actual next PC

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
            mem_wb_dmem_mask          <= 4'b0;
            mem_wb_dmem_wdata         <= 32'b0;
            mem_wb_next_pc            <= 32'b0;
        end else begin
            mem_wb_mem_read_data      <= mem_read_data;
            mem_wb_mem_read_data_raw  <= dmem_rdata_or_forwarded;  // Raw memory word for retire interface
            mem_wb_alu_result         <= ex_mem_alu_result;
            mem_wb_rd                 <= ex_mem_rd;
            mem_wb_mem_to_reg         <= ex_mem_mem_to_reg;
            mem_wb_reg_write          <= ex_mem_reg_write;
            mem_wb_pc_plus_4          <= ex_mem_pc_plus_4;
            mem_wb_opcode             <= ex_mem_opcode;
            mem_wb_imm                <= ex_mem_imm;
            mem_wb_is_jal             <= ex_mem_is_jal;
            mem_wb_is_jalr            <= ex_mem_is_jalr;
            mem_wb_is_branch          <= ex_mem_is_branch;
            mem_wb_mem_read           <= ex_mem_mem_read;
            mem_wb_mem_write          <= ex_mem_mem_write;
            mem_wb_funct3             <= ex_mem_funct3;
            mem_wb_rs1                <= ex_mem_rs1;
            mem_wb_rs2                <= ex_mem_rs2;
            mem_wb_rs1_data           <= ex_mem_rs1_data;
            mem_wb_rs2_data           <= ex_mem_rs2_data;
            mem_wb_pc                 <= ex_mem_pc;
            mem_wb_inst               <= ex_mem_inst;
            mem_wb_is_store           <= ex_mem_is_store;
            // Check for unaligned branch/jump target in MEM stage
            mem_wb_unaligned_pc       <= (ex_mem_is_jal | ex_mem_is_jalr | ex_mem_is_branch) &&
                                         (ex_mem_branch_target[1:0] != 2'b00);
            mem_wb_unaligned_mem      <= (ex_mem_mem_read | ex_mem_mem_write) &&
                                         ((ex_mem_funct3[1:0] == 2'b10 && dmem_addr_unaligned[1:0] != 2'b00) ||
                                          (ex_mem_funct3[1:0] == 2'b01 && dmem_addr_unaligned[0] != 1'b0));
            mem_wb_valid              <= ex_mem_valid;
            mem_wb_dmem_addr          <= o_dmem_addr;
            mem_wb_dmem_mask          <= o_dmem_mask;
            mem_wb_dmem_wdata         <= o_dmem_wdata;
            mem_wb_next_pc            <= ex_mem_next_pc;
        end
    end

    //=========================================================================
    // STAGE 5: WRITE BACK (WB)
    //=========================================================================
    // The write back stage selects the appropriate data to write to the
    // destination register based on the instruction type. Different instruction
    // types require different data sources for the register write.

    wire is_lui = (mem_wb_opcode == 7'b0110111);              // Load Upper Immediate
    wire is_auipc = (mem_wb_opcode == 7'b0010111);            // Add Upper Immediate to PC
    wire is_store = (mem_wb_opcode == 7'b0100011);            // Store instructions

    // Register Write Data Selection
    assign rd_data = mem_wb_mem_to_reg ? mem_wb_mem_read_data :       // Load instructions: memory data
                     (mem_wb_is_jal | mem_wb_is_jalr) ? mem_wb_pc_plus_4 :   // JAL/JALR: return address (PC+4)
                     is_lui ? mem_wb_imm :                     // LUI: immediate value
                     is_auipc ? (mem_wb_pc + mem_wb_imm) :            // AUIPC: PC + immediate
                     mem_wb_alu_result;                        // Default: ALU result

    //=========================================================================
    // TRAP DETECTION AND ERROR HANDLING
    //=========================================================================
    // The processor detects various error conditions and illegal operations
    // that should generate traps. In a full implementation, these would
    // trigger exception handlers.

    wire illegal_inst;                     // Illegal instruction detected (gated by pipeline valid)
    wire unaligned_pc;                     // Unaligned PC on control flow change (gated by pipeline valid)
    wire unaligned_mem;                    // Unaligned memory access (gated by pipeline valid)

    // Illegal Instruction Detection
    // Check if the opcode corresponds to a supported instruction
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
    assign illegal_inst = mem_wb_valid && unsupported_opcode;
    // PC Alignment Check
    // RISC-V requires PC to be 4-byte aligned; check this on control flow changes
    assign unaligned_pc = mem_wb_valid && mem_wb_unaligned_pc;

    // Memory Alignment Check
    // Word accesses must be 4-byte aligned, half-word accesses must be 2-byte aligned
    assign unaligned_mem = mem_wb_valid && mem_wb_unaligned_mem;

    //=========================================================================
    // INSTRUCTION RETIRE INTERFACE
    //=========================================================================
    // This interface provides visibility into instruction execution for
    // verification and debugging. In a single-cycle processor, instructions
    // retire every cycle unless there's a trap or halt condition.

    // Retire is valid only when a real instruction reaches WB (even on trap)
    assign o_retire_valid = mem_wb_valid;
    assign o_retire_inst = mem_wb_inst;                        // Current instruction word
    assign o_retire_trap = illegal_inst | unaligned_pc | unaligned_mem; // Any trap condition (gated by valid)
    assign o_retire_halt = o_retire_trap | (mem_wb_valid && (mem_wb_opcode == 7'b1110011) && (mem_wb_funct3 == 3'b000) && (mem_wb_inst[31:20] == 12'h001)); // Trap conditions or EBREAK (gated by valid)
    assign o_retire_rs1_raddr = mem_wb_rs1;                    // Source register 1 address
    assign o_retire_rs2_raddr = mem_wb_rs2;                    // Source register 2 address
    assign o_retire_rs1_rdata = mem_wb_rs1_data;               // Source register 1 data
    assign o_retire_rs2_rdata = mem_wb_rs2_data;               // Source register 2 data
    assign o_retire_rd_waddr = (mem_wb_is_branch || mem_wb_is_store) ? 5'b00000 : mem_wb_rd;         // Destination register address
    assign o_retire_rd_wdata = rd_data;                 // Destination register data
    assign o_retire_pc = mem_wb_pc;                            // Current PC
    assign o_retire_next_pc = mem_wb_next_pc;                  // Actual next PC (branch target or PC+4)

    // Memory interface signals for retired instruction
    assign o_retire_dmem_addr = mem_wb_dmem_addr;              // Data memory address
    assign o_retire_dmem_ren = mem_wb_mem_read;                // Data memory read enable
    assign o_retire_dmem_wen = mem_wb_mem_write;               // Data memory write enable
    assign o_retire_dmem_mask = mem_wb_dmem_mask;              // Data memory byte mask
    assign o_retire_dmem_wdata = mem_wb_dmem_wdata;            // Data memory write data
    assign o_retire_dmem_rdata = mem_wb_mem_read_data_raw;     // Data memory read data (raw, before processing)

endmodule

`default_nettype wire

//=============================================================================
// END OF HART MODULE
//=============================================================================