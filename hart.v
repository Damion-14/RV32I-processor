//=============================================================================
// RISC-V RV32I Single-Cycle Processor (Hart)
//=============================================================================
// This module implements a complete single-cycle RISC-V processor supporting
// the RV32I base instruction set. The processor executes one instruction per
// clock cycle through the following pipeline stages executed combinationally:
//
// 1. INSTRUCTION FETCH (IF)  - Fetch instruction from memory
// 2. INSTRUCTION DECODE (ID) - Decode instruction and generate control signals
// 3. EXECUTE (EX)           - ALU operations and branch/jump logic
// 4. MEMORY ACCESS (MEM)    - Load/store operations with alignment handling
// 5. WRITE BACK (WB)        - Write results back to register file
//
// Architecture: Single-cycle datapath with separate instruction/data memories
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
    // STAGE 1: INSTRUCTION FETCH (IF)
    //=========================================================================
    // The IF stage manages the program counter and fetches instructions from
    // instruction memory. The PC is updated based on control flow decisions
    // made in the execute stage (branches, jumps, or sequential execution).

    // Program Counter (PC) Register and Logic
    reg  [31:0] pc;                        // Current program counter
    wire [31:0] pc_plus_4;                 // PC + 4 for sequential execution
    wire [31:0] next_pc;                   // Next PC value (from EX stage)

    assign pc_plus_4 = pc + 32'd4;         // Calculate next sequential PC
    assign o_imem_raddr = pc;              // Send current PC to instruction memory

    // PC Update (Synchronous)
    // PC is updated on every clock cycle unless stalled by hazard detection
    // Hold PC for one cycle after reset to ensure instruction at reset address is fetched
    always @(posedge i_clk) begin
        if (i_rst) begin
            pc <= RESET_ADDR;              // Reset PC to specified address
        end else if (pipeline_started && !stall_pc) begin
            pc <= next_pc;                 // Update PC from execute stage (unless stalled)
        end
        // else: PC holds its current value during stall or first cycle after reset
    end

    // Instruction Fetch from Memory
    wire [31:0] inst;                      // Current instruction word
    assign inst = i_imem_rdata;            // Direct connection to memory data

    wire [31:0] if_id_inst;             // IF/ID pipeline register for instruction
    wire [31:0] if_id_pc;               // IF/ID pipeline register for PC
    wire [31:0] if_id_next_pc;          // IF/ID pipeline register for next PC
    wire        if_id_valid;            // IF/ID pipeline valid bit

    // Flush signal asserted when a control-flow change (branch/jump) is taken in EX
    wire flush;

    // Track when pipeline should start (one cycle after reset)
    reg pipeline_started;
    always @(posedge i_clk) begin
        if (i_rst)
            pipeline_started <= 1'b0;
        else
            pipeline_started <= 1'b1;
    end

    // IF/ID Pipeline Register Module
    if_id_regs if_id_stage (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_flush(flush),
        .i_stall(stall_if_id),         // Stall signal from hazard detection
        .i_inst(inst),
        .i_pc(pc),
        .i_next_pc(pc_plus_4),
        .i_valid(pipeline_started),    // Only valid after reset cycle completes
        .o_inst(if_id_inst),
        .o_pc(if_id_pc),
        .o_next_pc(if_id_next_pc),
        .o_valid(if_id_valid)
    );
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
    // 2.5: ID/EX Pipeline Register
    //-------------------------------------------------------------------------
    // Pipeline registers between Instruction Decode and Execute stages
    wire [31:0] id_ex_pc;               // ID/EX pipeline register for PC
    wire [31:0] id_ex_rs1_data;         // ID/EX pipeline register for rs1 data
    wire [31:0] id_ex_rs2_data;         // ID/EX pipeline register for rs2 data
    wire [31:0] id_ex_imm;              // ID/EX pipeline register for immediate
    wire [4:0]  id_ex_rs1;              // ID/EX pipeline register for rs1 address
    wire [4:0]  id_ex_rs2;              // ID/EX pipeline register for rs2 address
    wire [4:0]  id_ex_rd;               // ID/EX pipeline register for rd address
    wire [1:0]  id_ex_alu_op;           // ID/EX pipeline register for ALU operation
    wire [2:0]  id_ex_bj_type;          // ID/EX pipeline register for branch/jump type
    wire        id_ex_alu_src;          // ID/EX pipeline register for ALU source select
    wire        id_ex_mem_read;         // ID/EX pipeline register for memory read enable
    wire        id_ex_mem_write;        // ID/EX pipeline register for memory write enable
    wire        id_ex_mem_to_reg;       // ID/EX pipeline register for memory to register select
    wire        id_ex_reg_write;        // ID/EX pipeline register for register write enable
    wire [6:0]  id_ex_opcode;           // ID/EX pipeline register for opcode
    wire [31:0] id_ex_pc_plus_4;        // ID/EX pipeline register for PC + 4
    wire [2:0]  id_ex_funct3;          // ID/EX pipeline register for funct3
    wire [6:0]  id_ex_funct7;          // ID/EX pipeline register for funct7
    wire [31:0] id_ex_inst;            // ID/EX pipeline register for instruction
    wire        id_ex_valid;           // ID/EX pipeline valid bit

    // ID/EX Pipeline Register Module
    id_ex_regs id_ex_stage (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_bubble(bubble_id_ex),       // Bubble signal from hazard detection
        .i_pc(if_id_pc),
        .i_rs1_data(rs1_data),
        .i_rs2_data(rs2_data),
        .i_imm(imm),
        .i_rs1(rs1),
        .i_rs2(rs2),
        .i_rd(rd),
        .i_alu_op(alu_op),
        .i_bj_type(bj_type),
        .i_alu_src(alu_src),
        .i_mem_read(mem_read),
        .i_mem_write(mem_write),
        .i_mem_to_reg(mem_to_reg),
        .i_reg_write(reg_write),
        .i_opcode(opcode),
        .i_pc_plus_4(if_id_next_pc),
        .i_funct3(funct3),
        .i_funct7(funct7),
        .i_inst(if_id_inst),
        .i_valid(if_id_valid),
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
        .o_valid(id_ex_valid)
    );

    //=========================================================================
    // HAZARD DETECTION AND FORWARDING UNITS
    //=========================================================================
    // These units handle pipeline hazards to ensure correct execution
    // without requiring manual insertion of NOPs.

    //-------------------------------------------------------------------------
    // Hazard Detection Unit
    //-------------------------------------------------------------------------
    // Detects RAW hazards and generates stall signals for load-use hazards
    wire stall_pc, stall_if_id, bubble_id_ex;

    hazard_unit hazard_detector (
        .i_id_rs1(rs1),
        .i_id_rs2(rs2),
        .i_ex_rd(id_ex_rd),
        .i_ex_reg_write(id_ex_reg_write),
        .i_ex_mem_read(id_ex_mem_read),
        .i_mem_rd(ex_mem_rd),
        .i_mem_reg_write(ex_mem_reg_write),
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
        .i_mem_reg_write(ex_mem_reg_write),
        .i_wb_rd(mem_wb_rd),
        .i_wb_reg_write(mem_wb_reg_write_wire),
        .o_forward_a(forward_a),
        .o_forward_b(forward_b)
    );

    //=========================================================================
    // STAGE 3: EXECUTE (EX)
    //=========================================================================
    // The execute stage performs arithmetic operations, evaluates branch
    // conditions, and calculates target addresses for control flow instructions.

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
    assign forwarded_rs1_data = id_ex_rs1_data;

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

    //-------------------------------------------------------------------------
    // 3.3: Branch and Jump Logic
    //-------------------------------------------------------------------------
    // Evaluates branch conditions and calculates target addresses for control flow
    wire is_branch;                        // Current instruction is a branch
    wire is_jal;                           // Current instruction is JAL
    wire is_jalr;                          // Current instruction is JALR
    wire branch_taken;                     // Branch condition is satisfied

    assign is_branch = (id_ex_opcode == 7'b1100011); // Branch instructions
    assign is_jal    = (id_ex_opcode == 7'b1101111); // Jump and Link
    assign is_jalr   = (id_ex_opcode == 7'b1100111); // Jump and Link Register

    // Branch Condition Evaluation
    // Determines if branch should be taken based on branch type and ALU flags
    wire branch_condition;
    assign branch_condition = (bj_type == 3'b000) ? alu_eq :        // BEQ: branch if equal
                              (bj_type == 3'b001) ? ~alu_eq :       // BNE: branch if not equal
                              (bj_type == 3'b100) ? alu_slt :       // BLT: branch if less than (signed)
                              (bj_type == 3'b101) ? ~alu_slt :      // BGE: branch if greater/equal (signed)
                              (bj_type == 3'b110) ? alu_slt :       // BLTU: branch if less than (unsigned)
                              (bj_type == 3'b111) ? ~alu_slt :      // BGEU: branch if greater/equal (unsigned)
                              1'b0;                                  // Default: No branch

    assign branch_taken = is_branch & branch_condition;

    // Target Address Calculation
    wire [31:0] branch_target;             // Branch/JAL target address
    wire [31:0] jalr_target;               // JALR target address

    assign branch_target = id_ex_pc + id_ex_imm;                       // PC-relative for branches/JAL
    assign jalr_target = (forwarded_rs1_data + id_ex_imm) & ~32'd1;   // Register+immediate (with forwarding), clear LSB

    // Next PC Selection (feeds back to IF stage)
    // Default sequential PC should use current PC+4; redirect only when EX decides a control flow change
    assign next_pc = is_jalr ? jalr_target :                 // JALR: rs1 + imm
                     (is_jal | branch_taken) ? branch_target : // JAL/taken branch: PC + imm
                     pc_plus_4;                              // Default: current PC + 4

    // Flush asserted when a control-flow change is taken in EX
    assign flush = (is_jalr | is_jal | branch_taken);

    //Pipeline registers between EX and MEM stages
    wire [31:0] ex_mem_pc;               // EX/MEM pipeline register for PC
    wire [31:0] ex_mem_rs1_data;         // EX/MEM pipeline register for rs1 data
    wire [31:0] ex_mem_rs2_data;         // EX/MEM pipeline register for rs2 data
    wire [31:0] ex_mem_imm;              // EX/MEM pipeline register for immediate
    wire [4:0]  ex_mem_rs1;              // EX/MEM pipeline register for rs1 address
    wire [4:0]  ex_mem_rs2;              // EX/MEM pipeline register for rs2 address
    wire [4:0]  ex_mem_rd;               // EX/MEM pipeline register for rd address
    wire        ex_mem_mem_read;         // EX/MEM pipeline register for memory read enable
    wire        ex_mem_mem_write;        // EX/MEM pipeline register for memory write enable
    wire        ex_mem_mem_to_reg;       // EX/MEM pipeline register for memory to register select
    wire        ex_mem_reg_write;        // EX/MEM pipeline register for register write enable
    wire [6:0]  ex_mem_opcode;           // EX/MEM pipeline register for opcode
    wire [31:0] ex_mem_pc_plus_4;        // EX/MEM pipeline register for PC + 4
    wire [31:0] ex_mem_alu_result;       // EX/MEM pipeline register for ALU result
    wire [2:0]  ex_mem_funct3;          // EX/MEM pipeline register for funct3
    wire [6:0]  ex_mem_funct7;          // EX/MEM pipeline register for funct7
    wire        ex_mem_is_jal;           // EX/MEM pipeline register for is_jal
    wire        ex_mem_is_jalr;          // EX/MEM pipeline register for is_jalr
    wire        ex_mem_is_branch;        // EX/MEM pipeline register for is_branch
    wire        ex_mem_is_store;         // EX/MEM pipeline register for is_store
    wire [31:0] ex_mem_inst;           // EX/MEM pipeline register for instruction
    wire        ex_mem_unaligned_pc;     // EX/MEM pipeline register for unaligned PC flag
    wire        ex_mem_valid;            // EX/MEM pipeline valid bit

    // EX/MEM Pipeline Register Module
    ex_mem_regs ex_mem_stage (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_pc(id_ex_pc),
        .i_rs1_data(forwarded_rs1_data),     // Use forwarded data for branch/jump target calculation
        .i_rs2_data(forwarded_rs2_data),     // Use forwarded data for store operations
        .i_imm(id_ex_imm),
        .i_rs1(id_ex_rs1),
        .i_rs2(id_ex_rs2),
        .i_rd(id_ex_rd),
        .i_mem_read(id_ex_mem_read),
        .i_mem_write(id_ex_mem_write),
        .i_mem_to_reg(id_ex_mem_to_reg),
        .i_reg_write(id_ex_reg_write),
        .i_opcode(id_ex_opcode),
        .i_pc_plus_4(id_ex_pc_plus_4),
        .i_alu_result(alu_result),
        .i_funct3(id_ex_funct3),
        .i_funct7(id_ex_funct7),
        .i_is_jal(is_jal),
        .i_is_jalr(is_jalr),
        .i_is_branch(is_branch),
        .i_is_store(id_ex_opcode == 7'b0100011),
        .i_inst(id_ex_inst),
        .i_unaligned_pc((is_jal | branch_taken) ? (branch_target[1:0] != 2'b00) :
                        (is_jalr ? (jalr_target[1:0] != 2'b00) : 1'b0)),
        .i_valid(id_ex_valid),
        .o_pc(ex_mem_pc),
        .o_rs1_data(ex_mem_rs1_data),
        .o_rs2_data(ex_mem_rs2_data),
        .o_imm(ex_mem_imm),
        .o_rs1(ex_mem_rs1),
        .o_rs2(ex_mem_rs2),
        .o_rd(ex_mem_rd),
        .o_mem_read(ex_mem_mem_read),
        .o_mem_write(ex_mem_mem_write),
        .o_mem_to_reg(ex_mem_mem_to_reg),
        .o_reg_write(ex_mem_reg_write),
        .o_opcode(ex_mem_opcode),
        .o_pc_plus_4(ex_mem_pc_plus_4),
        .o_alu_result(ex_mem_alu_result),
        .o_funct3(ex_mem_funct3),
        .o_funct7(ex_mem_funct7),
        .o_is_jal(ex_mem_is_jal),
        .o_is_jalr(ex_mem_is_jalr),
        .o_is_branch(ex_mem_is_branch),
        .o_is_store(ex_mem_is_store),
        .o_inst(ex_mem_inst),
        .o_unaligned_pc(ex_mem_unaligned_pc),
        .o_valid(ex_mem_valid)
    );
    //=========================================================================
    // STAGE 4: MEMORY ACCESS (MEM)
    //=========================================================================
    // The memory stage handles load and store operations with support for
    // byte, half-word, and word accesses at aligned and unaligned addresses.
    // It also handles the necessary data shifting and masking operations.

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
                    2'b00: load_data_processed = {{24{i_dmem_rdata[7]}},  i_dmem_rdata[7:0]};
                    2'b01: load_data_processed = {{24{i_dmem_rdata[15]}}, i_dmem_rdata[15:8]};
                    2'b10: load_data_processed = {{24{i_dmem_rdata[23]}}, i_dmem_rdata[23:16]};
                    default: load_data_processed = {{24{i_dmem_rdata[31]}}, i_dmem_rdata[31:24]};
                endcase
            end

            // LH: Load Half-word (sign-extended)
            3'b001: begin
                case (byte_offset[1])
                    1'b0: load_data_processed = {{16{i_dmem_rdata[15]}}, i_dmem_rdata[15:0]};
                    default: load_data_processed = {{16{i_dmem_rdata[31]}}, i_dmem_rdata[31:16]};
                endcase
            end

            // LW: Load Word (no extension needed)
            3'b010: load_data_processed = i_dmem_rdata;

            // LBU: Load Byte Unsigned (zero-extended)
            3'b100: begin
                case (byte_offset)
                    2'b00: load_data_processed = {24'b0, i_dmem_rdata[7:0]};
                    2'b01: load_data_processed = {24'b0, i_dmem_rdata[15:8]};
                    2'b10: load_data_processed = {24'b0, i_dmem_rdata[23:16]};
                    default: load_data_processed = {24'b0, i_dmem_rdata[31:24]};
                endcase
            end

            // LHU: Load Half-word Unsigned (zero-extended)
            3'b101: begin
                case (byte_offset[1])
                    1'b0: load_data_processed = {16'b0, i_dmem_rdata[15:0]};
                    default: load_data_processed = {16'b0, i_dmem_rdata[31:16]};
                endcase
            end

            default: load_data_processed = i_dmem_rdata;    // Default to word load
        endcase
    end

    wire [31:0] mem_read_data;
    assign mem_read_data = load_data_processed;

    //-------------------------------------------------------------------------
    // 4.4: MEM/WB Pipeline Register
    //-------------------------------------------------------------------------
    wire [31:0] mem_wb_mem_read_data;     // MEM/WB pipeline register for memory read data
    wire [31:0] mem_wb_alu_result;        // MEM/WB pipeline register for ALU result
    wire [4:0]  mem_wb_rd;                // MEM/WB pipeline register for rd address
    wire        mem_wb_mem_to_reg;        // MEM/WB pipeline register for memory to register select
    reg         mem_wb_reg_write;         // MEM/WB pipeline register for register write enable (reg for RF)
    wire [31:0] mem_wb_pc_plus_4;        // MEM/WB pipeline register for PC + 4
    wire [6:0]  mem_wb_opcode;           // MEM/WB pipeline register for opcode
    wire [31:0] mem_wb_imm;             // MEM/WB pipeline register for immediate value
    wire        mem_wb_is_jal;           // MEM/WB pipeline register for is_jal
    wire        mem_wb_is_jalr;          // MEM/WB pipeline register for is_jalr
    wire        mem_wb_is_branch;        // MEM/WB pipeline register for is_branch
    wire        mem_wb_mem_read;         // MEM/WB pipeline register for memory read enable
    wire        mem_wb_mem_write;        // MEM/WB pipeline register for memory write enable
    wire [2:0]  mem_wb_funct3;           // MEM/WB pipeline register for funct3
    wire [4:0]  mem_wb_rs1;              // MEM/WB pipeline register for rs1 address
    wire [4:0]  mem_wb_rs2;              // MEM/WB pipeline register for rs2 address
    wire [31:0] mem_wb_rs1_data;         // MEM/WB pipeline register for rs1 data
    wire [31:0] mem_wb_rs2_data;         // MEM/WB pipeline register for rs2 data
    wire [31:0] mem_wb_pc;               // MEM/WB pipeline register for PC
    wire [31:0] mem_wb_inst;             // MEM/WB pipeline register for instruction
    wire        mem_wb_is_store;         // MEM/WB pipeline register for is_store
    wire        mem_wb_unaligned_pc;     // MEM/WB pipeline register for unaligned PC trap flag
    wire        mem_wb_unaligned_mem;    // MEM/WB pipeline register for unaligned MEM trap flag
    wire        mem_wb_valid;            // MEM/WB pipeline valid bit
    wire [31:0] mem_wb_dmem_addr;        // MEM/WB pipeline register for memory address
    wire [ 3:0] mem_wb_dmem_mask;        // MEM/WB pipeline register for memory mask
    wire [31:0] mem_wb_dmem_wdata;       // MEM/WB pipeline register for memory write data

    // Wire for reg_write output from MEM/WB module
    wire mem_wb_reg_write_wire;

    // MEM/WB Pipeline Register Module
    mem_wb_regs mem_wb_stage (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_mem_read_data(mem_read_data),
        .i_alu_result(ex_mem_alu_result),
        .i_rd(ex_mem_rd),
        .i_mem_to_reg(ex_mem_mem_to_reg),
        .i_reg_write(ex_mem_reg_write),
        .i_pc_plus_4(ex_mem_pc_plus_4),
        .i_opcode(ex_mem_opcode),
        .i_imm(ex_mem_imm),
        .i_is_jal(ex_mem_is_jal),
        .i_is_jalr(ex_mem_is_jalr),
        .i_is_branch(ex_mem_is_branch),
        .i_mem_read(ex_mem_mem_read),
        .i_mem_write(ex_mem_mem_write),
        .i_funct3(ex_mem_funct3),
        .i_rs1(ex_mem_rs1),
        .i_rs2(ex_mem_rs2),
        .i_rs1_data(ex_mem_rs1_data),
        .i_rs2_data(ex_mem_rs2_data),
        .i_pc(ex_mem_pc),
        .i_inst(ex_mem_inst),
        .i_is_store(ex_mem_is_store),
        .i_unaligned_pc(ex_mem_unaligned_pc),
        .i_unaligned_mem((ex_mem_mem_read | ex_mem_mem_write) &&
                         ((ex_mem_funct3[1:0] == 2'b10 && dmem_addr_unaligned[1:0] != 2'b00) ||
                          (ex_mem_funct3[1:0] == 2'b01 && dmem_addr_unaligned[0] != 1'b0))),
        .i_valid(ex_mem_valid),
        .i_dmem_addr(o_dmem_addr),
        .i_dmem_mask(o_dmem_mask),
        .i_dmem_wdata(o_dmem_wdata),
        .o_mem_read_data(mem_wb_mem_read_data),
        .o_alu_result(mem_wb_alu_result),
        .o_rd(mem_wb_rd),
        .o_mem_to_reg(mem_wb_mem_to_reg),
        .o_reg_write(mem_wb_reg_write_wire),
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
        .o_dmem_addr(mem_wb_dmem_addr),
        .o_dmem_mask(mem_wb_dmem_mask),
        .o_dmem_wdata(mem_wb_dmem_wdata)
    );

    // Register file requires a registered write enable signal
    always @(posedge i_clk) begin
        if (i_rst)
            mem_wb_reg_write <= 1'b0;
        else
            mem_wb_reg_write <= mem_wb_reg_write_wire;
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
    assign o_retire_next_pc = mem_wb_pc + 32'd4;               // Next PC (always PC+4 for retired instruction)

    // Memory interface signals for retired instruction
    assign o_retire_dmem_addr = mem_wb_dmem_addr;              // Data memory address
    assign o_retire_dmem_ren = mem_wb_mem_read;                // Data memory read enable
    assign o_retire_dmem_wen = mem_wb_mem_write;               // Data memory write enable
    assign o_retire_dmem_mask = mem_wb_dmem_mask;              // Data memory byte mask
    assign o_retire_dmem_wdata = mem_wb_dmem_wdata;            // Data memory write data
    assign o_retire_dmem_rdata = mem_wb_mem_read_data;         // Data memory read data

endmodule

`default_nettype wire

//=============================================================================
// END OF HART MODULE
//=============================================================================