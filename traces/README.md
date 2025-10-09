# Traces Directory (traces/)

This directory contains execution traces and simulation output files generated during processor testing and verification.

## Purpose

The traces directory serves as the output location for:
- Processor execution traces showing instruction-by-instruction execution
- Signal dumps and waveform data
- Performance analysis output
- Debugging information and simulation logs

## Generated Files

- **trace.txt** - Main execution trace showing PC, instructions, and register states
- **\*.vcd** - Value Change Dump files for waveform viewing
- **\*.log** - Simulation log files with detailed execution information
- Performance metrics and cycle counts

## Trace File Format

Typical trace file contents include:
- Cycle number
- Program Counter (PC) value
- Instruction being executed (in hex)
- Register file state changes
- Memory access information
- Control signal states

## Usage for Debugging

1. **Execution Flow**: Trace the sequence of instructions executed
2. **Register Tracking**: Monitor register value changes throughout execution
3. **Memory Access**: Verify correct load/store operations
4. **Control Logic**: Check control signal generation
5. **Performance**: Analyze cycle counts and execution time

## Trace Analysis

Use trace files to:
- Verify correct instruction execution sequence
- Debug incorrect processor behavior
- Compare against expected execution patterns
- Validate against reference implementations
- Identify performance bottlenecks

## File Management

- Trace files are automatically generated during simulation
- Old traces may be overwritten on new simulation runs
- Save important traces with descriptive names for later reference
- Clean up old trace files periodically to manage disk space