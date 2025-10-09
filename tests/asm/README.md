# Assembly Tests Directory (tests/asm/)

This directory contains RISC-V assembly test programs used to verify the functionality of the processor implementation.

## Purpose

The assembly tests directory provides:
- Test programs covering different RV32I instruction types
- Comprehensive test coverage for processor verification
- Examples of RISC-V assembly programming
- Validation cases for debugging and development

## File Structure

- **\*.asm** - RISC-V assembly source files
- **Makefile** - Build configuration for assembling programs
- Individual test programs targeting specific functionality

## Assembly Workflow

1. **Write/Edit Tests**: Create `.asm` files with RISC-V assembly code
2. **Assemble**: Use the Docker toolchain to convert assembly to machine code
   ```bash
   make PROGRAM=test_name
   ```
3. **Output**: Generated `program.mem` file is placed in `tb/` directory
4. **Test**: Run simulation with the generated memory file

## Docker Toolchain

Use the provided Docker container for assembly:
```bash
docker pull coderkalyan/ece552-tools:latest
```

## Test Program Requirements

- **Halt instruction**: All test programs must end with `ebreak` to halt the processor
- **Proper format**: Follow RISC-V assembly syntax
- **Clear objectives**: Each test should target specific instruction types or scenarios

## Makefile Configuration

The Makefile should be configured to:
- Remove `.txt` extension from output files
- Generate `program.mem` in the correct format
- Support the `PROGRAM=name` parameter for specifying which test to assemble

## Example Test Categories

- **Arithmetic**: ADD, SUB, AND, OR, XOR operations
- **Immediate**: ADDI, ANDI, ORI, XORI operations  
- **Memory**: LW, SW, LB, SB operations
- **Branch**: BEQ, BNE, BLT, BGE operations
- **Jump**: JAL, JALR operations
- **System**: EBREAK for halt functionality