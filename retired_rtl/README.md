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

- **hart.v** - top-level interface for the CPU (connect all other componets in this folder and hanlde some simple logic connection)
- **alu.v** - arithmetic logic unit
- **imm.v** - immediate generator
- **rf.v** - register file
- **alu_control.v** - feeds the operation to the ALU
- **control.v** - instruction decoder

```text 
hart.v (Top-level datapath)
├── PC logic & PC+4 adder
├── Branch target calculation & branch decision
├── Memory interface connections
├── Datapath muxes (ALU src, write-back, PC source)
└── Module instantiations:
    ├── control.v (main decoder)
    ├── alu_control.v (ALU op decoder)
    ├── imm.v (immediate generator)
    ├── alu.v (arithmetic/logic unit)
    └── rf.v (register file)
```

## Implementation Notes

- All modules should follow consistent naming conventions
- Use proper Verilog coding practices for synthesis
- Include proper reset behavior for all sequential elements
- Ensure timing closure for single-cycle operation
- Test each module individually before integration