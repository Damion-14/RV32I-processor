# Testbench Directory (tb/)

This directory contains the testbench files used to simulate and test the RISC-V processor implementation.

## Purpose

The testbench directory serves as the simulation environment where:
- Test programs are loaded into processor memory
- The processor is instantiated and connected to test infrastructure
- Simulation control and monitoring occurs
- Program memory files (`program.mem`) are read from

## Key Files

- **tb.v** - Main testbench module that instantiates the processor
- **program.mem** - Memory initialization file containing assembled RISC-V instructions
- Other testbench utilities and simulation control files

## Usage

1. Place assembled program memory files (`program.mem`) in this directory
2. Run your Verilog simulator with the testbench files
3. The testbench will load the program and execute it on your processor
4. Monitor processor signals and generate trace output

## Memory Format

The `program.mem` file contains hexadecimal instruction words, one per line, that get loaded into the processor's instruction memory during simulation initialization.