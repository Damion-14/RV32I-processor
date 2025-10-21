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
    output wire [31:0] o_retire_next_pc    // Next program counter value

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
    always @(posedge i_clk) begin
        if (i_rst) begin
            pc <= RESET_ADDR;              // Reset PC to specified address
        end else begin
            pc <= next_pc;                 // Update PC from execute stage
        end
    end

    // Instruction Fetch from Memory
    wire [31:0] inst;                      // Current instruction word
    assign inst = i_imem_rdata;            // Direct connection to memory data

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

    assign opcode = inst[6:0];
    assign rd     = inst[11:7];
    assign funct3 = inst[14:12];
    assign rs1    = inst[19:15];
    assign rs2    = inst[24:20];
    assign funct7 = inst[31:25];

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
        .instruction(inst),
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

    rf #(.BYPASS_EN(0)) rf ( // BYPASS = 0
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_rs1_raddr(rs1),                 // Read address 1
        .i_rs2_raddr(rs2),                 // Read address 2
        .i_rd_waddr(rd),                   // Write address
        .i_rd_wdata(rd_data),              // Write data
        .i_rd_wen(reg_write),              // Write enable
        .o_rs1_rdata(rs1_data),            // Read data 1
        .o_rs2_rdata(rs2_data)             // Read data 2
    );

    //-------------------------------------------------------------------------
    // 2.4: Immediate Generation
    //-------------------------------------------------------------------------
    // Generates sign-extended immediate values for different instruction formats
    wire [31:0] imm;                       // Generated immediate value

    imm imm_gen (
        .i_inst(inst),
        .i_format(i_format),
        .o_immediate(imm)
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

    assign func37 = {funct7[5], funct3};   // Concatenate funct7[5] and funct3

    alu_ctl alu_control_unit (
        .alu_op(alu_op),
        .func37(func37),
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith)
    );

    //-------------------------------------------------------------------------
    // 3.2: ALU (Arithmetic Logic Unit)
    //-------------------------------------------------------------------------
    // Performs arithmetic and logical operations on two 32-bit operands
    wire [31:0] alu_op1, alu_op2;          // ALU input operands
    wire [31:0] alu_result;                // ALU output result
    wire alu_eq;                           // Operands are equal
    wire alu_slt;                          // Operand 1 < Operand 2 (signed/unsigned)

    assign alu_op1 = rs1_data;             // First operand always from rs1
    assign alu_op2 = alu_src ? imm : rs2_data; // Second operand: immediate or rs2

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

    assign is_branch = (opcode == 7'b1100011); // Branch instructions
    assign is_jal    = (opcode == 7'b1101111); // Jump and Link
    assign is_jalr   = (opcode == 7'b1100111); // Jump and Link Register

    // Branch Condition Evaluation
    // Determines if branch should be taken based on branch type and ALU flags
    reg branch_condition;
    always @(posedge i_clk) begin
        case (bj_type)
            3'b000: branch_condition = alu_eq;      // BEQ: branch if equal
            3'b001: branch_condition = ~alu_eq;     // BNE: branch if not equal
            3'b100: branch_condition = alu_slt;     // BLT: branch if less than (signed)
            3'b101: branch_condition = ~alu_slt;    // BGE: branch if greater/equal (signed)
            3'b110: branch_condition = alu_slt;     // BLTU: branch if less than (unsigned)
            3'b111: branch_condition = ~alu_slt;    // BGEU: branch if greater/equal (unsigned)
            default: branch_condition = 1'b0;       // No branch
        endcase
    end

    assign branch_taken = is_branch & branch_condition;

    // Target Address Calculation
    wire [31:0] branch_target;             // Branch/JAL target address
    wire [31:0] jalr_target;               // JALR target address

    assign branch_target = pc + imm;                    // PC-relative for branches/JAL
    assign jalr_target = (rs1_data + imm) & ~32'd1;    // Register+immediate, clear LSB

    // Next PC Selection (feeds back to IF stage)
    assign next_pc = is_jalr ? jalr_target :           // JALR: rs1 + imm
                     (is_jal | branch_taken) ? branch_target : // JAL/taken branch: PC + imm
                     pc_plus_4;                        // Default: PC + 4

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

    assign dmem_addr_unaligned = alu_result;           // Address from ALU (rs1 + imm)
    assign byte_offset = dmem_addr_unaligned[1:0];     // Extract byte offset
    assign o_dmem_addr = {dmem_addr_unaligned[31:2], 2'b00}; // Align to 4-byte boundary

    // Memory Control Signals
    assign o_dmem_ren = mem_read;          // Read enable from control unit
    assign o_dmem_wen = mem_write;         // Write enable from control unit

    //-------------------------------------------------------------------------
    // 4.2: Store Operations (Memory Writes)
    //-------------------------------------------------------------------------
    // For store operations, data must be shifted to the correct byte lanes
    // and the appropriate mask must be generated based on access size.

    // Generate byte mask based on access size and byte offset
    reg [3:0] dmem_mask;
    always @(posedge i_clk) begin
        case (funct3[1:0])
            2'b00: dmem_mask = 4'b0001 << byte_offset;  // SB: single byte
            2'b01: dmem_mask = 4'b0011 << byte_offset;  // SH: half-word (2 bytes)
            2'b10: dmem_mask = 4'b1111;                 // SW: full word (4 bytes)
            default: dmem_mask = 4'b1111;               // Default to word
        endcase
    end
    assign o_dmem_mask = dmem_mask;

    // Shift store data to correct byte lanes based on byte offset
    reg [31:0] store_data_shifted;
    always @(posedge i_clk) begin
        case (byte_offset)
            2'b00: store_data_shifted = rs2_data;           // No shift needed
            2'b01: store_data_shifted = rs2_data << 8;      // Shift left 8 bits
            2'b10: store_data_shifted = rs2_data << 16;     // Shift left 16 bits
            2'b11: store_data_shifted = rs2_data << 24;     // Shift left 24 bits
        endcase
    end
    assign o_dmem_wdata = store_data_shifted;

    //-------------------------------------------------------------------------
    // 4.3: Load Operations (Memory Reads)
    //-------------------------------------------------------------------------
    // For load operations, data must be extracted from the correct byte lanes,
    // shifted to the LSBs, and sign/zero extended based on the instruction type.

    reg [31:0] load_data_processed;
    always @(posedge i_clk) begin
        case (funct3)
            // LB: Load Byte (sign-extended)
            3'b000: begin
                case (byte_offset)
                    2'b00: load_data_processed = {{24{i_dmem_rdata[7]}},  i_dmem_rdata[7:0]};
                    2'b01: load_data_processed = {{24{i_dmem_rdata[15]}}, i_dmem_rdata[15:8]};
                    2'b10: load_data_processed = {{24{i_dmem_rdata[23]}}, i_dmem_rdata[23:16]};
                    2'b11: load_data_processed = {{24{i_dmem_rdata[31]}}, i_dmem_rdata[31:24]};
                endcase
            end

            // LH: Load Half-word (sign-extended)
            3'b001: begin
                case (byte_offset[1])
                    1'b0: load_data_processed = {{16{i_dmem_rdata[15]}}, i_dmem_rdata[15:0]};
                    1'b1: load_data_processed = {{16{i_dmem_rdata[31]}}, i_dmem_rdata[31:16]};
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
                    2'b11: load_data_processed = {24'b0, i_dmem_rdata[31:24]};
                endcase
            end

            // LHU: Load Half-word Unsigned (zero-extended)
            3'b101: begin
                case (byte_offset[1])
                    1'b0: load_data_processed = {16'b0, i_dmem_rdata[15:0]};
                    1'b1: load_data_processed = {16'b0, i_dmem_rdata[31:16]};
                endcase
            end

            default: load_data_processed = i_dmem_rdata;    // Default to word load
        endcase
    end

    wire [31:0] mem_read_data;
    assign mem_read_data = load_data_processed;

    //=========================================================================
    // STAGE 5: WRITE BACK (WB)
    //=========================================================================
    // The write back stage selects the appropriate data to write to the
    // destination register based on the instruction type. Different instruction
    // types require different data sources for the register write.

    wire is_lui = (opcode == 7'b0110111);              // Load Upper Immediate
    wire is_auipc = (opcode == 7'b0010111);            // Add Upper Immediate to PC

    // Register Write Data Selection
    assign rd_data = mem_to_reg ? mem_read_data :       // Load instructions: memory data
                     (is_jal | is_jalr) ? pc_plus_4 :   // JAL/JALR: return address (PC+4)
                     is_lui ? imm :                     // LUI: immediate value
                     is_auipc ? (pc + imm) :            // AUIPC: PC + immediate
                     alu_result;                        // Default: ALU result

    //=========================================================================
    // TRAP DETECTION AND ERROR HANDLING
    //=========================================================================
    // The processor detects various error conditions and illegal operations
    // that should generate traps. In a full implementation, these would
    // trigger exception handlers.

    wire illegal_inst;                     // Illegal instruction detected
    wire unaligned_pc;                     // Unaligned PC on control flow change
    wire unaligned_mem;                    // Unaligned memory access

    // Illegal Instruction Detection
    // Check if the opcode corresponds to a supported instruction
    assign illegal_inst = ~(opcode == 7'b0110011 ||    // R-type
                           opcode == 7'b0010011 ||      // I-type ALU
                           opcode == 7'b0000011 ||      // Load
                           opcode == 7'b0100011 ||      // Store
                           opcode == 7'b1100011 ||      // Branch
                           opcode == 7'b1101111 ||      // JAL
                           opcode == 7'b1100111 ||      // JALR
                           opcode == 7'b0110111 ||      // LUI
                           opcode == 7'b0010111 ||      // AUIPC
                           opcode == 7'b1110011);       // EBREAK/ECALL

    // PC Alignment Check
    // RISC-V requires PC to be 4-byte aligned; check this on control flow changes
    assign unaligned_pc = (is_jal | is_jalr | branch_taken) && (next_pc[1:0] != 2'b00);

    // Memory Alignment Check
    // Word accesses must be 4-byte aligned, half-word accesses must be 2-byte aligned
    assign unaligned_mem = (mem_read | mem_write) && (
                          (funct3[1:0] == 2'b10 && dmem_addr_unaligned[1:0] != 2'b00) ||  // Word misaligned
                          (funct3[1:0] == 2'b01 && dmem_addr_unaligned[0] != 1'b0));      // Half-word misaligned

    //=========================================================================
    // INSTRUCTION RETIRE INTERFACE
    //=========================================================================
    // This interface provides visibility into instruction execution for
    // verification and debugging. In a single-cycle processor, instructions
    // retire every cycle unless there's a trap or halt condition.

    assign o_retire_valid = 1'b1;                       // Always valid in single-cycle
    assign o_retire_inst = inst;                        // Current instruction word
    assign o_retire_trap = illegal_inst | unaligned_pc | unaligned_mem; // Any trap condition
    assign o_retire_halt = o_retire_trap | ((opcode == 7'b1110011) && (funct3 == 3'b000) && (inst[31:20] == 12'h001)); // Trap conditions or EBREAK
    assign o_retire_rs1_raddr = rs1;                    // Source register 1 address
    assign o_retire_rs2_raddr = rs2;                    // Source register 2 address
    assign o_retire_rs1_rdata = rs1_data;               // Source register 1 data
    assign o_retire_rs2_rdata = rs2_data;               // Source register 2 data
    assign o_retire_rd_waddr = (is_branch) ? 4'b0000 : rd;                      // Destination register address
    assign o_retire_rd_wdata = rd_data;                 // Destination register data
    assign o_retire_pc = pc;                            // Current PC
    assign o_retire_next_pc = next_pc;                  // Next PC

endmodule

`default_nettype wire

//=============================================================================
// END OF HART MODULE
//=============================================================================