# RV32I Processor Testing Guide

This document explains how to test the refactored RV32I pipelined processor design.

## Table of Contents
- [Quick Start](#quick-start)
- [Prerequisites](#prerequisites)
- [Running Tests](#running-tests)
- [Test Programs](#test-programs)
- [Understanding Test Output](#understanding-test-output)
- [Debugging](#debugging)
- [Advanced Testing](#advanced-testing)

---

## Quick Start

The fastest way to run all tests:

```bash
# Run all tests
make test

# Or run a specific test
make test-add
make test-branch
```

---

## Prerequisites

### Required Tools

1. **Icarus Verilog** (for simulation)
   ```bash
   brew install icarus-verilog
   ```

2. **Verilator** (for linting - optional but recommended)
   ```bash
   brew install verilator
   ```

### Verify Installation

```bash
# Check iverilog
iverilog -v

# Check verilator
verilator --version
```

---

## Running Tests

### Using the Makefile (Recommended)

The Makefile provides convenient targets for all testing operations.

#### Compile the Design
```bash
make compile
```
This compiles all RTL files and creates the `hart_sim` executable.

#### Run All Tests
```bash
make test
```
Runs all test programs in the `tb/` directory and reports pass/fail for each.

#### Run a Specific Test
```bash
make test-add      # Run ADD instruction test
make test-branch   # Run branch instruction test
make test-memory   # Run memory operation test
make test-shift    # Run shift instruction test
```

Available test targets:
- `test-add`, `test-addi`
- `test-and`, `test-andi`
- `test-or`, `test-ori`
- `test-xor`, `test-xori`
- `test-sll`, `test-slli`
- `test-srl`, `test-srli`
- `test-sra`, `test-srai`
- `test-slt`, `test-slti`, `test-sltu`, `test-sltiu`
- `test-lui`, `test-auipc`
- `test-memory`
- `test-branch`, `test-jal`, `test-jalr`

#### Verification (Lint Checking)
```bash
make verify
```
Runs the verification script to check for:
- Syntax errors
- Wiring issues
- Width mismatches
- Unconnected signals

#### Clean Build Artifacts
```bash
make clean
```
Removes compiled simulators and temporary files.

### Manual Testing

If you prefer to run tests manually:

#### 1. Compile the Design
```bash
iverilog -g2012 -o hart_sim \
    rtl/components/*.v \
    rtl/pipeline_control/*.v \
    rtl/stages/*.v \
    rtl/hart.v \
    tb/tb.v
```

#### 2. Run a Test
```bash
# Edit tb/tb.v to load the desired test file, or:
./hart_sim
```

The testbench currently loads `tb/01add.mem` by default (line 102 in `tb/tb.v`).

---

## Test Programs

All test programs are located in the `tb/` directory as `.mem` files.

### Available Tests

| Test File | Instruction Type | Description |
|-----------|------------------|-------------|
| `01add.mem` | Arithmetic | ADD instruction tests |
| `02addi.mem` | Arithmetic | ADDI (add immediate) tests |
| `03and.mem` | Logical | AND instruction tests |
| `04andi.mem` | Logical | ANDI (and immediate) tests |
| `05lui.mem` | Upper Immediate | LUI (load upper immediate) tests |
| `06memory.mem` | Memory | Load/store operations |
| `07or.mem` | Logical | OR instruction tests |
| `08ori.mem` | Logical | ORI (or immediate) tests |
| `09sll.mem` | Shift | SLL (shift left logical) tests |
| `10slli.mem` | Shift | SLLI (shift left logical immediate) tests |
| `11slt.mem` | Comparison | SLT (set less than) tests |
| `12slti.mem` | Comparison | SLTI (set less than immediate) tests |
| `13sltiu.mem` | Comparison | SLTIU (set less than immediate unsigned) tests |
| `14sltu.mem` | Comparison | SLTU (set less than unsigned) tests |
| `15sra.mem` | Shift | SRA (shift right arithmetic) tests |
| `16srai.mem` | Shift | SRAI (shift right arithmetic immediate) tests |
| `17srl.mem` | Shift | SRL (shift right logical) tests |
| `18srli.mem` | Shift | SRLI (shift right logical immediate) tests |
| `19sub.mem` | Arithmetic | SUB (subtract) tests |
| `20xor.mem` | Logical | XOR instruction tests |
| `21xori.mem` | Logical | XORI (xor immediate) tests |
| `22auipc.mem` | Upper Immediate | AUIPC (add upper immediate to PC) tests |
| `23beq.mem` | Branch | BEQ (branch if equal) tests |
| `24bge.mem` | Branch | BGE (branch if greater or equal) tests |
| `25bgeu.mem` | Branch | BGEU (branch if greater or equal unsigned) tests |
| `26blt.mem` | Branch | BLT (branch if less than) tests |
| `27bltu.mem` | Branch | BLTU (branch if less than unsigned) tests |
| `28bne.mem` | Branch | BNE (branch if not equal) tests |
| `29jal.mem` | Jump | JAL (jump and link) tests |
| `30jalr.mem` | Jump | JALR (jump and link register) tests |

---

## Understanding Test Output

### Successful Test Output

When a test passes, you'll see output like:

```
VCD info: dumpfile hart.vcd opened for output.
Loading program.
Resetting hart.
Cycle  PC        Inst     rs1            rs2            [rd, load, store]
[00000000] 00000093 r[ 0]=00000000 r[xx]=xxxxxxxx w[ 1]=00000000
[00000004] 00000113 r[ 0]=00000000 r[xx]=xxxxxxxx w[ 2]=00000000
...
Program halted after         316 cycles.
Total instructions retired:         309
CPI: 1.022654
```

**Key Metrics:**
- **CPI (Cycles Per Instruction)**: Should be close to 1.0 for the pipelined design
  - CPI < 1.1: Excellent (minimal stalls)
  - CPI 1.1-1.3: Good (some load-use hazards)
  - CPI > 1.3: Check for excessive stalls

### Output Format

Each line shows:
```
[PC] INSTRUCTION r[rs1]=VALUE r[rs2]=VALUE w[rd]=RESULT
```

- `PC`: Program counter (instruction address)
- `INSTRUCTION`: 32-bit instruction word in hex
- `r[rs1]=VALUE`: Source register 1 and its value
- `r[rs2]=VALUE`: Source register 2 and its value
- `w[rd]=RESULT`: Destination register and written value

**Memory operations** also show:
- `l[ADDRESS,MASK]=DATA`: Load operation
- `s[ADDRESS,MASK]=DATA`: Store operation

**Repeated lines** are normal - each instruction appears multiple times as it flows through the 5 pipeline stages.

### Test Failure

If a test fails, look for:
- **TRAP** indicator: Illegal instruction or misalignment
- Program doesn't halt: Infinite loop or incorrect branch
- Wrong register values: Logic error in the implementation

---

## Debugging

### View Waveforms

After running a test, examine the waveform file:

```bash
# On macOS with GTKWave
open hart.vcd

# Or if you have gtkwave installed
gtkwave hart.vcd
```

The VCD file contains all signal transitions for debugging timing issues.

### Enable Verbose Output

Edit `tb/tb.v` to add more `$display` statements for debugging specific scenarios.

### Check Specific Instructions

To debug a particular instruction:

1. Find the test file (e.g., `tb/01add.mem` for ADD)
2. Examine the hex values (4 bytes per instruction, little-endian)
3. Decode using RISC-V ISA reference

Example:
```
93 00 00 00  →  0x00000093  →  ADDI x1, x0, 0
```

### Common Issues

**Issue: Simulation hangs**
- Likely cause: Infinite loop or PC not incrementing
- Check: Branch conditions, PC update logic
- Solution: View waveform to see where PC gets stuck

**Issue: Wrong results**
- Likely cause: Incorrect forwarding or hazard detection
- Check: Data forwarding paths in `rtl/hart.v`
- Solution: Compare register values with expected results

**Issue: High CPI**
- Likely cause: Excessive stalls
- Check: Hazard detection unit, load-use dependencies
- Solution: Review pipeline stall signals in waveform

---

## Advanced Testing

### Running Custom Test Programs

1. **Write RISC-V assembly:**
   ```assembly
   # test.s
   addi x1, x0, 10
   addi x2, x0, 20
   add  x3, x1, x2
   ebreak
   ```

2. **Assemble to hex:**
   ```bash
   cd tests/asm
   make test.hex
   ```

3. **Copy to tb directory:**
   ```bash
   cp test.hex ../../tb/custom.mem
   ```

4. **Update testbench:**
   Edit `tb/tb.v` line 102 to load `tb/custom.mem`

5. **Run:**
   ```bash
   make compile
   ./hart_sim
   ```

### Performance Analysis

Monitor performance metrics:

```bash
# Run all tests and extract CPI
make test | grep "CPI:"

# Average CPI across all tests
make test | grep "CPI:" | awk '{sum+=$2; count++} END {print "Average CPI:", sum/count}'
```

### Regression Testing

Create a baseline:
```bash
make test > baseline_results.txt
```

After making changes:
```bash
make test > new_results.txt
diff baseline_results.txt new_results.txt
```

### Synthesis Testing (Optional)

If you have Yosys installed:

```bash
# Check if design is synthesizable
yosys -p "read_verilog -sv rtl/**/*.v rtl/hart.v; synth; stat"
```

---

## Makefile Reference

### Common Targets

| Target | Description |
|--------|-------------|
| `make` or `make all` | Compile and run verification |
| `make compile` | Compile the design to create simulator |
| `make test` | Run all test programs |
| `make test-<name>` | Run specific test (e.g., `test-add`) |
| `make verify` | Run lint check with Verilator |
| `make clean` | Remove build artifacts |
| `make help` | Show all available targets |

### Variables

You can override Makefile variables:

```bash
# Use different compiler
make VERILOG=verilator compile

# Change test directory
make TESTDIR=my_tests test

# Add compiler flags
make VFLAGS="-Wall -Wno-fatal" compile
```

---

## Continuous Integration

For automated testing in CI/CD:

```bash
# Install tools
brew install icarus-verilog verilator

# Run verification
make verify || exit 1

# Run all tests
make test || exit 1

# Check CPI is reasonable
make test | grep "CPI:" | awk '{if ($2 > 1.5) exit 1}'
```

---

## Getting Help

If tests fail or you encounter issues:

1. **Check this documentation** for common issues
2. **Run verification** with `make verify` to catch wiring errors
3. **Examine waveforms** in `hart.vcd` using GTKWave
4. **Review RTL** in the `rtl/` directory (now organized by stage!)
5. **Check the README** in `rtl/README.md` for architecture details

---

## Additional Resources

- **RV32I ISA Specification**: https://riscv.org/specifications/
- **Icarus Verilog Manual**: http://iverilog.icarus.com/
- **Verilator Manual**: https://verilator.org/guide/latest/
- **GTKWave**: http://gtkwave.sourceforge.net/

---

**Last Updated**: November 2025
**Processor**: RV32I 5-Stage Pipelined Implementation
**Test Suite**: 30 instruction tests covering full RV32I ISA
