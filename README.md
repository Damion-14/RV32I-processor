# RISC-V RV32I Single-Cycle Processor

This project implements a single-cycle processor that supports the RISC-V RV32I ISA (Instruction Set Architecture).

## Project Description

The processor implements the complete RV32I instruction set in a single-cycle design. Key features include:
- Support for all RV32I instructions
- Single-cycle execution model
- Memory interface for instruction and data
- Halt functionality using the `ebreak` instruction

## Directory Structure

```
RV32I-processor/
├── tb/          # Testbench directory - contains simulation and testing files
├── rtl/         # RTL implementation - all Verilog processor implementation files
├── tests/asm/   # Assembly tests - RISC-V assembly test programs (.asm files)
├── traces/      # Trace output - execution traces and simulation results
└── README.md    # This file
```

## Testing Workflow

1. **Assembly**: Use the provided Docker container to assemble RISC-V programs
   ```bash
   docker pull coderkalyan/ece552-tools:latest
   cd tests/asm/
   make PROGRAM=01add
   ```

2. **Simulation**: Run the testbench with your processor implementation
   - The testbench reads `program.mem` from the `tb/` directory
   - Trace output is generated in the `traces/` directory

3. **Verification**: Analyze the trace files to verify correct processor operation

## Key Implementation Notes

- All RTL files should be placed in the `rtl/` directory (no subdirectories)
- The processor must implement the `ebreak` instruction to halt execution
- Memory initialization uses `program.mem` files generated from assembly programs
- Trace files help debug and verify processor functionality

## Resources

- WISC-F25 specification document (primary reference)
- RISC-V cheat sheet for quick instruction reference
- Online RISC-V simulator for verification
- RISC-V instruction decoder for debugging
