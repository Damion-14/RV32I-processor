# Quick Start Guide - RV32I Processor Testing

## 🚀 Getting Started in 30 Seconds

```bash
# 1. Install tools (one-time setup)
brew install icarus-verilog verilator

# 2. Run all tests
make test

# 3. That's it! ✅
```

---

## 📋 Common Commands

### Testing
```bash
make test              # Run ALL tests
make test-add          # Test ADD instructions
make test-branch       # Test all branches
make test-memory       # Test loads/stores
make test-01add        # Run specific test file
```

### Building
```bash
make compile           # Compile the design
make verify            # Check for errors
make clean             # Clean up
```

### Analysis
```bash
make stats             # Show code statistics
make cpi-analysis      # Performance analysis
make wave              # View waveforms (GTKWave)
```

---

## 📊 What to Expect

**Successful Test Output:**
```
✓ Test 01add PASSED (CPI: 1.022654)
```

**Good CPI Values:**
- CPI ≈ 1.0: Excellent! (minimal stalls)
- CPI 1.0-1.2: Good (some hazards)
- CPI > 1.3: Investigate stalls

---

## 🔧 Debugging Failed Tests

```bash
# 1. Run the specific test
make test-<name>

# 2. Check the log
cat test_<name>.log

# 3. View waveforms
make wave
```

---

## 📁 Project Structure

```
rtl/
├── hart.v               ← Top-level wrapper (523 lines)
├── stages/              ← 5 pipeline stages
│   ├── if_stage.v      ← Instruction Fetch
│   ├── id_stage.v      ← Decode + Branch resolution
│   ├── ex_stage.v      ← ALU operations
│   ├── mem_stage.v     ← Memory access
│   └── wb_stage.v      ← Write back
├── components/          ← ALU, control, register file, etc.
└── pipeline_control/    ← Hazard detection, forwarding
```

---

## 💡 Tips

- **First time?** Run `make help` to see all commands
- **Test failed?** Check `TESTING.md` for debugging tips
- **Want details?** See `rtl/README.md` for architecture
- **Need info?** Run `make info` for configuration

---

## 🎯 Quick Test Examples

```bash
# Test arithmetic instructions
make test-01add test-02addi test-19sub

# Test logical operations
make test-03and test-07or test-20xor

# Test branches
make test-23beq test-28bne test-26blt

# Test jumps
make test-29jal test-30jalr
```

---

## 📈 Performance Check

```bash
# See CPI for all tests
make cpi-analysis

# Expected output:
#   01add                CPI: 1.022654
#   02addi               CPI: 1.018692
#   ...
#   Average CPI: 1.0X
```

---

## 🆘 Help

| Command | What It Does |
|---------|-------------|
| `make help` | Show all available commands |
| `make list-tests` | List all test programs |
| `make info` | Show build configuration |
| `make stats` | Show design statistics |

---

**For detailed documentation, see:**
- `TESTING.md` - Complete testing guide
- `rtl/README.md` - Architecture documentation
- `Makefile` - All available targets

**Happy Testing! 🎉**
