# RTL Implementation Directory (rtl/)

This directory contains all the Verilog RTL (Register Transfer Level) implementation files for the RISC-V RV32I single-cycle processor.

## Purpose

The RTL directory houses the complete hardware description of the processor, including:
- Main processor module and datapath
- Control unit for instruction decode and control signal generation
- Memory interfaces (instruction and data memory)
- ALU (Arithmetic Logic Unit) implementation
- Register file implementation
- All supporting modules and components

## Design Requirements

- **Single-cycle design**: Each instruction completes in exactly one clock cycle
- **RV32I ISA support**: Must implement all required RISC-V 32-bit integer instructions
- **Flat structure**: All Verilog files must be in this directory (no subdirectories)

## Key Modules (typical structure)

- **hart.v** - top-level interface for the CPU

## Implementation Notes

- All modules should follow consistent naming conventions
- Use proper Verilog coding practices for synthesis
- Include proper reset behavior for all sequential elements
- Ensure timing closure for single-cycle operation
- Test each module individually before integration