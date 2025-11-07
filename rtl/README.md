# RTL Implementation Directory (rtl/)

This directory contains all the Verilog RTL (Register Transfer Level) implementation files for the RISC-V RV32I 5-stage pipelined processor.

## Purpose

The RTL directory houses the complete hardware description of the processor, organized into a modular 5-stage pipeline architecture with separate directories for different functional units.

## Directory Structure

```
rtl/
├── README.md                    # This file
├── hart.v                       # Top-level wrapper (instantiates all stages)
├── hart.v.bak                   # Backup of original monolithic hart.v
├── stages/                      # Pipeline stage modules
│   ├── if_stage.v              # Instruction Fetch stage
│   ├── id_stage.v              # Instruction Decode stage (with IF/ID registers)
│   ├── ex_stage.v              # Execute stage (with ID/EX registers)
│   ├── mem_stage.v             # Memory Access stage (with EX/MEM registers)
│   └── wb_stage.v              # Write Back stage (with MEM/WB registers)
├── components/                  # Reusable datapath components
│   ├── alu.v                   # Arithmetic Logic Unit
│   ├── alu_control.v           # ALU operation decoder
│   ├── control.v               # Main instruction decoder
│   ├── imm.v                   # Immediate value generator
│   └── rf.v                    # Register file (32 x 32-bit registers)
└── pipeline_control/            # Hazard detection and data forwarding
    ├── hazard_unit.v           # Load-use hazard detection
    ├── forwarding_unit.v       # EX stage data forwarding logic
    └── branch_forwarding_unit.v # ID stage branch forwarding logic
```

## Design Architecture

### 5-Stage Pipeline

The processor uses a classic 5-stage RISC pipeline with the following characteristics:

**Stage 1: Instruction Fetch (IF)**
- Manages the Program Counter (PC)
- Fetches instructions from instruction memory
- Handles PC stalls and control flow changes
- **No pipeline registers** (output directly feeds ID stage)

**Stage 2: Instruction Decode (ID)**
- **Contains IF/ID pipeline registers** (at start of stage)
- Decodes instruction fields (opcode, rs1, rs2, rd, funct3, funct7)
- Generates control signals via control unit
- Reads from register file
- Generates immediate values
- **Resolves branches and jumps** (zero-cycle branch penalty)
- Performs branch forwarding for early resolution
- **Outputs ID/EX pipeline registers** (at end of stage)

**Stage 3: Execute (EX)**
- **Contains ID/EX pipeline registers** (at start of stage)
- Performs ALU operations
- Handles data forwarding from MEM and WB stages
- Computes effective addresses for memory operations
- **Outputs to MEM stage** (combinational, registered in MEM stage)

**Stage 4: Memory Access (MEM)**
- **Contains EX/MEM pipeline registers** (at start of stage)
- Handles load and store operations
- Performs memory address alignment and byte masking
- Implements store-to-load forwarding (MEM-to-MEM)
- Supports unaligned memory accesses
- **Outputs to WB stage** (combinational, registered in WB stage)

**Stage 5: Write Back (WB)**
- **Contains MEM/WB pipeline registers** (at start of stage)
- Selects write-back data source (ALU, memory, PC+4, immediate)
- Writes result to register file
- Performs trap detection (illegal instructions, misalignments)
- Provides retire interface for verification

### Pipeline Register Organization

Pipeline registers are located at the **start of each stage** (except IF):
- IF stage: No input pipeline registers
- ID stage: IF/ID registers (receive data from IF stage)
- EX stage: ID/EX registers (receive data from ID stage)
- MEM stage: EX/MEM registers (receive data from EX stage)
- WB stage: MEM/WB registers (receive data from MEM stage)

This organization makes each stage self-contained and easier to understand.

### Key Features

1. **Zero-Cycle Branch Penalty**
   - Branches resolved in ID stage (stage 2)
   - Branch forwarding enables early operand availability
   - No pipeline bubbles for taken branches

2. **Full Data Forwarding**
   - EX stage: forwards from MEM and WB stages
   - ID stage: forwards for branch operands
   - MEM stage: store-to-load forwarding

3. **Hazard Detection**
   - Load-use hazards detected and stalled automatically
   - Bubble insertion for unavoidable stalls
   - Reset stall handling

4. **Memory Access**
   - Unaligned access support with byte masking
   - Separate instruction and data memory interfaces
   - Store-to-load forwarding for back-to-back operations

## Module Hierarchy

```
hart (top-level wrapper)
├── hazard_unit                  # Hazard detection
├── forwarding_unit              # Data forwarding control
├── if_stage                     # Stage 1
├── id_stage                     # Stage 2
│   ├── control (ctl)           # Instruction decoder
│   ├── rf                      # Register file
│   ├── imm                     # Immediate generator
│   └── branch_forwarding_unit  # Branch forwarding
├── ex_stage                     # Stage 3
│   ├── alu_control (alu_ctl)  # ALU operation control
│   └── alu                    # Arithmetic logic unit
├── mem_stage                    # Stage 4
└── wb_stage                     # Stage 5
```

## ISA Support

**RV32I Base Integer Instruction Set**
- Arithmetic: ADD, ADDI, SUB
- Logical: AND, ANDI, OR, ORI, XOR, XORI
- Shifts: SLL, SLLI, SRL, SRLI, SRA, SRAI
- Comparisons: SLT, SLTI, SLTU, SLTIU
- Loads: LB, LH, LW, LBU, LHU
- Stores: SB, SH, SW
- Branches: BEQ, BNE, BLT, BGE, BLTU, BGEU
- Jumps: JAL, JALR
- Upper Immediate: LUI, AUIPC
- System: EBREAK, ECALL (detection only)

## Compilation Notes

When compiling this design with Verilog simulators or synthesis tools, include all files in the proper order:

### Recommended Compilation Order

1. **Components** (no dependencies)
   - alu.v
   - alu_control.v
   - control.v
   - imm.v
   - rf.v

2. **Pipeline Control** (no dependencies)
   - hazard_unit.v
   - forwarding_unit.v
   - branch_forwarding_unit.v

3. **Pipeline Stages** (depend on components)
   - if_stage.v
   - id_stage.v (depends on: control, rf, imm, branch_forwarding_unit)
   - ex_stage.v (depends on: alu_control, alu)
   - mem_stage.v
   - wb_stage.v

4. **Top-level wrapper** (depends on all above)
   - hart.v

### Example iverilog Command

```bash
iverilog -g2012 -o hart_sim \
    rtl/components/*.v \
    rtl/pipeline_control/*.v \
    rtl/stages/*.v \
    rtl/hart.v \
    tb/tb.v
```

### Example Verilator Command

```bash
verilator --cc --exe --build -j 0 \
    -Wall -Wno-fatal \
    rtl/components/*.v \
    rtl/pipeline_control/*.v \
    rtl/stages/*.v \
    rtl/hart.v \
    sim_main.cpp
```

## Implementation Notes

- All modules follow consistent naming conventions
  - Inputs: `i_` prefix
  - Outputs: `o_` prefix
  - Internal signals: no prefix
  - Pipeline registers: `stage1_stage2_signal` format

- Proper Verilog coding practices for synthesis
  - All sequential logic uses synchronous reset
  - No latches (all combinational blocks fully specified)
  - Synthesizable constructs only

- Pipeline register valid bits track instruction flow
  - Bubbles inserted have valid=0
  - Used to gate forwarding and hazard detection

- Extensive comments explain design decisions
  - Each stage file documents its functionality
  - Inter-stage interfaces clearly defined

## Testing

The testbench (tb/tb.v) remains unchanged and works with the refactored design:
- Instantiates hart module by its interface
- Provides synchronous instruction and data memories
- Monitors retire interface for verification
- Compatible with all test programs in tb/ directory

## Performance Characteristics

- **CPI**: ~1.0 (cycles per instruction) for most programs
  - Zero-cycle branch penalty
  - Minimal stalls with full forwarding
  - Only load-use hazards cause 1-cycle stalls

- **Critical Path**: Typically in EX stage (ALU operation)
  - Can be optimized for higher clock frequencies
  - Pipeline registers break combinational paths

## Migration from Original Design

The original monolithic hart.v (1,082 lines) has been refactored into:
- 5 stage files (~150-350 lines each)
- 5 component files (45-245 lines each)
- 3 pipeline control files (98-131 lines each)
- 1 top-level wrapper (~500 lines)

**Benefits**:
- Much easier to understand and modify individual stages
- Clear separation of concerns
- Easier debugging (can focus on one stage)
- Better for team development
- Simpler to extend (e.g., add new instructions to ID/EX stages)

**Original file backed up as**: `hart.v.bak`
