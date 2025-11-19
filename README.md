# RISC-V RV32I Pipelined Processor

This project implements a **5-stage pipelined processor** that supports the RISC-V RV32I ISA (Instruction Set Architecture).

## 🎯 Project Overview

A fully functional, modular RV32I processor with:
- ✅ **5-stage pipeline** (IF → ID → EX → MEM → WB)
- ✅ **Zero-cycle branch penalty** (branches resolved in ID stage)
- ✅ **Full data forwarding** (minimal stalls)
- ✅ **Complete RV32I ISA support** (all 40+ instructions)
- ✅ **Modular design** (easy to understand and modify)
- ✅ **~1.0 CPI** (near-perfect pipeline efficiency)

## 📁 Directory Structure

```
RV32I-processor/
├── rtl/                    # RTL implementation (REFACTORED!)
│   ├── hart.v             # Top-level wrapper (523 lines)
│   ├── stages/            # 5 pipeline stages
│   │   ├── if_stage.v    # Instruction Fetch
│   │   ├── id_stage.v    # Instruction Decode
│   │   ├── ex_stage.v    # Execute
│   │   ├── mem_stage.v   # Memory Access
│   │   └── wb_stage.v    # Write Back
│   ├── components/        # ALU, control, register file, etc.
│   └── pipeline_control/  # Hazard detection, forwarding
├── tb/                     # Testbench and test programs
├── tests/asm/             # Assembly test source files
├── Makefile               # Easy build and test automation
├── QUICKSTART.md          # 30-second start guide
├── TESTING.md             # Comprehensive testing guide
└── README.md              # This file
```

## 🚀 Quick Start

### 1. Install Tools (One-Time Setup)
```bash
brew install icarus-verilog verilator
```

### 2. Run Tests
```bash
# Run all tests
make test

# Or run specific tests
make test-add
make test-branch
make test-memory
```

### 3. Check Results
```bash
# View statistics
make stats

# Analyze performance
make cpi-analysis
```

**That's it!** ✅ See `QUICKSTART.md` for more examples.

## 📚 Documentation

| Document | Description |
|----------|-------------|
| **[QUICKSTART.md](QUICKSTART.md)** | Get started in 30 seconds |
| **[TESTING.md](TESTING.md)** | Complete testing guide |
| **[rtl/README.md](rtl/README.md)** | Architecture documentation |
| **Makefile** | Run `make help` for all commands |

## 🏗️ Architecture Highlights

### Refactored Design (November 2025)

The processor was recently refactored from a monolithic 1,082-line file into a clean, modular design:

**Before:**
- ❌ Single 1,082-line `hart.v` file
- ❌ Hard to understand and modify
- ❌ Difficult to debug

**After:**
- ✅ 5 separate stage files (95-412 lines each)
- ✅ Clear separation of concerns
- ✅ Easy to understand and modify
- ✅ **Original backed up as `rtl/hart.v.bak`**

### Pipeline Features

1. **Zero-Cycle Branch Penalty**
   - Branches resolved in ID stage
   - No pipeline bubbles for taken branches

2. **Full Data Forwarding**
   - EX stage: forwards from MEM/WB
   - ID stage: forwards for branch operands
   - MEM stage: store-to-load forwarding

3. **Hazard Detection**
   - Automatic load-use hazard detection
   - Minimal stalls with smart forwarding

## 🧪 Testing

### Quick Test Commands

```bash
# All tests
make test

# Arithmetic
make test-01add test-02addi test-19sub

# Logical operations
make test-03and test-07or test-20xor

# Branches
make test-23beq test-28bne test-26blt

# Memory
make test-06memory

# Jumps
make test-29jal test-30jalr
```

### Available Test Programs (30 total)

The `tb/` directory contains 30 test programs covering the complete RV32I ISA:
- Arithmetic: ADD, ADDI, SUB
- Logical: AND, OR, XOR (and immediate variants)
- Shifts: SLL, SRL, SRA (and immediate variants)
- Comparisons: SLT, SLTU (and immediate variants)
- Loads/Stores: LB, LH, LW, LBU, LHU, SB, SH, SW
- Branches: BEQ, BNE, BLT, BGE, BLTU, BGEU
- Jumps: JAL, JALR
- Upper Immediate: LUI, AUIPC

## 📊 Performance

Expected performance metrics:
- **CPI**: ~1.0 (cycles per instruction)
- **Pipeline Efficiency**: >95%
- **Branch Penalty**: 0 cycles (resolved in ID)
- **Load-Use Penalty**: 1 cycle (when unavoidable)

Run `make cpi-analysis` to see detailed performance across all tests.

## 🔧 Development Workflow

### Making Changes

1. **Edit the appropriate stage file:**
   ```bash
   # Example: Modify ALU operation
   vim rtl/components/alu.v
   ```

2. **Verify your changes:**
   ```bash
   make verify
   ```

3. **Run tests:**
   ```bash
   make test
   ```

4. **Check waveforms if needed:**
   ```bash
   make wave
   ```

### Adding New Instructions

1. Update control unit (`rtl/components/control.v`)
2. Update ALU if needed (`rtl/components/alu.v`)
3. Create a test program in `tb/`
4. Run `make test-<yourtest>`

## 🛠️ Makefile Commands

```bash
make              # Compile and verify
make compile      # Build the simulator
make test         # Run all tests
make test-<name>  # Run specific test
make verify       # Check for errors
make stats        # Show code statistics
make wave         # View waveforms
make clean        # Clean build files
make help         # Show all commands
```

## 📈 Design Statistics

- **Original hart.v**: 1,082 lines
- **New hart.v (wrapper)**: 523 lines
- **Total RTL**: ~2,900 lines (across all modules)
- **Pipeline Stages**: 5 files (95-412 lines each)
- **Components**: 5 files (45-245 lines each)
- **Pipeline Control**: 3 files (98-131 lines each)

Run `make stats` for detailed breakdown.

## 🔍 Debugging

### View Execution Trace
```bash
make test-01add
# Output shows each instruction execution
```

### View Waveforms
```bash
make wave
# Opens GTKWave with hart.vcd
```

### Check Specific Signals
See `TESTING.md` for detailed debugging guide.

## 📖 Resources

### Documentation
- **RISC-V ISA Spec**: https://riscv.org/specifications/
- **Icarus Verilog**: http://iverilog.icarus.com/
- **Verilator**: https://verilator.org/

### Project-Specific
- `rtl/README.md` - Detailed architecture documentation
- `TESTING.md` - Complete testing guide
- `QUICKSTART.md` - Quick reference

## 🤝 Contributing

When making changes:
1. Run `make verify` to check syntax
2. Run `make test` to ensure all tests pass
3. Check CPI hasn't degraded: `make cpi-analysis`
4. Update documentation if needed

## 📝 Version History

- **November 2025**: Major refactoring
  - Split monolithic hart.v into 5 pipeline stages
  - Added comprehensive Makefile
  - Created testing documentation
  - Verified all tests pass with CPI ~1.0

- **Earlier**: Original pipelined implementation
  - 5-stage pipeline in single file
  - Zero-cycle branch penalty
  - Full RV32I ISA support

## 🎓 Educational Value

This processor is excellent for learning:
- **Pipelined processor design**
- **Hazard detection and forwarding**
- **RISC-V ISA implementation**
- **Verilog best practices**
- **Modular hardware design**

## 🏆 Key Achievements

✅ Complete RV32I ISA support
✅ Near-optimal CPI (~1.0)
✅ Zero-cycle branch penalty
✅ Clean, modular codebase
✅ Comprehensive test suite (30 tests)
✅ Full verification passing

---

**Happy Hacking! 🚀**

For questions or issues, see the documentation files listed above.
