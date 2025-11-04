# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a RISC-V RV32I processor implementation in Verilog, currently implementing a 5-stage pipeline with hazard detection, data forwarding, and branch prediction. The processor supports the complete RV32I base instruction set.

## Build and Test Commands

### Assembly Test Programs

Assembly test programs are located in `tests/asm/` and must be assembled using a Docker container:

```bash
cd tests/asm/
make PROGRAM=<test_name>  # Assembles <test_name>.asm
```

This generates `program.mem` in the `tb/` directory, which is loaded by the testbench.

Docker image required: `coderkalyan/ece552-tools:latest`

### Simulation

The testbench file is `tb/tb.v`. Run simulation using your Verilog simulator (e.g., Icarus Verilog, ModelSim, VCS) pointing to the testbench and RTL files. The testbench:
- Loads `program.mem` from the `tb/` directory
- Executes until `ebreak` instruction (halt condition)
- Generates trace output showing PC, instruction, register values, and memory operations
- Tracks cycle count for performance analysis

## Architecture Overview

### Pipeline Structure

The processor uses a 5-stage pipeline implemented in `rtl/hart.v`:

1. **IF (Instruction Fetch)** - Fetches instruction from memory using PC
2. **ID (Instruction Decode)** - Decodes instruction, reads registers, generates immediates
3. **EX (Execute)** - ALU operations, branch condition evaluation, target calculation
4. **MEM (Memory Access)** - Load/store operations with alignment handling
5. **WB (Write Back)** - Write results to register file

Pipeline registers between stages are modularized in `rtl/pipeline_regs.v`:
- `if_id_regs` - IF/ID pipeline register
- `id_ex_regs` - ID/EX pipeline register
- `ex_mem_regs` - EX/MEM pipeline register
- `mem_wb_regs` - MEM/WB pipeline register

### Hazard Handling

**Hazard Detection Unit** (`rtl/hazard_unit.v`):
- Detects load-use hazards (when instruction immediately after load needs loaded data)
- Generates stall signals: `o_stall_pc`, `o_stall_if_id`, `o_bubble_id_ex`
- Creates 1-cycle pipeline bubble for load-use hazards

**Forwarding Unit** (`rtl/forwarding_unit.v`):
- Implements EX-EX forwarding (from EX/MEM stage to EX stage)
- Implements MEM-EX forwarding (from MEM/WB stage to EX stage)
- Generates `forward_a` and `forward_b` control signals (2-bit each)
- Priority: EX-EX > MEM-EX (most recent value forwarded)

### Branch Prediction

Uses an always-not-taken static predictor:
- Always fetches PC+4 (next sequential instruction)
- If branch is taken (resolved in EX stage): flush IF/ID register and redirect to target
- 1-cycle penalty for taken branches/jumps
- 0-cycle penalty for not-taken branches

### Module Breakdown

**Top-Level (`rtl/hart.v`):**
- Instantiates all pipeline stages and control units
- Manages PC logic and control flow
- Connects pipeline registers, hazard unit, forwarding unit
- Implements trap detection and retire interface

**Supporting Modules:**
- `rtl/control.v` - Main instruction decoder (generates control signals from opcode)
- `rtl/alu_control.v` - ALU operation decoder (generates ALU control from funct3/funct7)
- `rtl/alu.v` - Arithmetic Logic Unit (arithmetic, logic, comparison operations)
- `rtl/imm.v` - Immediate generator (extracts and sign-extends immediates)
- `rtl/rf.v` - Register file (2 read ports, 1 write port, x0 hardwired to 0, BYPASS_EN=1)

### Memory Interface

**Instruction Memory:**
- Read-only interface
- 4-byte aligned addresses
- Combinational read

**Data Memory:**
- Separate read/write interface
- Supports byte (B/BU), half-word (H/HU), and word (W) accesses
- Handles unaligned accesses via byte offset calculation and masking
- Synchronous writes, combinational reads

### Pipeline Valid Bits

To prevent spurious traps during reset and pipeline bubbles, each stage has a `valid` bit:
- `if_id_valid`, `id_ex_valid`, `ex_mem_valid`, `mem_wb_valid`
- Cleared on reset and flush, propagate forward each cycle
- Traps (illegal opcode, misalignment) are gated by `mem_wb_valid`
- Retire interface (`o_retire_valid`) only asserts when real instruction reaches WB

## Important Implementation Details

### Register File Bypass

The register file has `BYPASS_EN=1`, allowing same-cycle write-then-read. This is critical for the forwarding unit to work correctly - a register written in WB stage can be read in ID stage in the same cycle.

### Branch Resolution

Branches are resolved in the EX stage (not ID), requiring:
- Forwarding for branch operands (rs1, rs2)
- 1-cycle penalty for taken branches due to already-fetched instruction in IF/ID

### JALR Target Calculation

JALR target is `(rs1 + imm) & ~1` (LSB cleared per RISC-V spec), and requires forwarding for rs1 data.

### Store Data Forwarding

Store instructions need forwarded rs2 data for the value to store. This is handled by the forwarding muxes before the EX/MEM pipeline register.

### Trap Detection

Three trap conditions detected in WB stage (gated by `mem_wb_valid`):
- Illegal instruction (unsupported opcode)
- Unaligned PC (control flow target not 4-byte aligned)
- Unaligned memory access (word access not 4-byte aligned, half-word not 2-byte aligned)

### EBREAK Halt

Processor halts when EBREAK instruction (`opcode=7'b1110011`, `funct3=3'b000`, `inst[31:20]=12'h001`) reaches WB stage and `mem_wb_valid` is asserted.

## Performance Characteristics

Expected CPI (Cycles Per Instruction):
- **Without forwarding**: ~3.0+ (2 NOPs needed after each RAW dependency)
- **With hazard detection + forwarding**: ~1.0-1.3
  - Most RAW hazards eliminated via forwarding
  - Only stalls for load-use hazards (1 cycle)
  - Branch penalty: 1 cycle for taken, 0 for not-taken

## File Organization

All RTL files must be in `rtl/` (flat structure, no subdirectories).

Assembly test files in `tests/asm/` should end with `ebreak` instruction.

Generated `program.mem` is placed in `tb/` by the Makefile.

Trace outputs go in `traces/` directory.

## Common Development Patterns

When modifying the pipeline:
1. Update pipeline register module in `rtl/pipeline_regs.v` if adding new signals
2. Thread new signals through all pipeline stages in `rtl/hart.v`
3. Update hazard/forwarding units if new hazards introduced
4. Verify with assembly tests covering the modification

When adding instructions:
1. Update `rtl/control.v` for new opcode/format
2. Update `rtl/alu_control.v` if new ALU operation needed
3. Update `rtl/alu.v` if new ALU operation needed
4. Update trap detection logic in `rtl/hart.v` if applicable
5. Add assembly test cases

When debugging:
- Check retire interface signals in simulation
- Verify pipeline valid bits propagate correctly
- Ensure forwarding paths are correct for new operations
- Check for unintended pipeline flushes or stalls
