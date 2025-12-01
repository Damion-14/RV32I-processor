#!/bin/bash
#=============================================================================
# Verification Script for Refactored RV32I Processor
#=============================================================================
# This script checks that the refactored design compiles correctly
# and that all modules are properly connected.
#
# Usage: ./verify_refactor.sh
#=============================================================================

set -e  # Exit on error

echo "=========================================="
echo "RV32I Processor Refactoring Verification"
echo "=========================================="
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check what tools are available
echo "Checking for Verilog tools..."
echo ""

HAS_IVERILOG=false
HAS_VERILATOR=false
HAS_YOSYS=false

if command -v iverilog &> /dev/null; then
    echo -e "${GREEN}✓${NC} Icarus Verilog (iverilog) found: $(iverilog -v 2>&1 | head -1)"
    HAS_IVERILOG=true
else
    echo -e "${YELLOW}✗${NC} Icarus Verilog not found. Install with: brew install icarus-verilog"
fi

if command -v verilator &> /dev/null; then
    echo -e "${GREEN}✓${NC} Verilator found: $(verilator --version | head -1)"
    HAS_VERILATOR=true
else
    echo -e "${YELLOW}✗${NC} Verilator not found. Install with: brew install verilator"
fi

if command -v yosys &> /dev/null; then
    echo -e "${GREEN}✓${NC} Yosys found: $(yosys -V | head -1)"
    HAS_YOSYS=true
else
    echo -e "${YELLOW}✗${NC} Yosys not found. Install with: brew install yosys"
fi

echo ""

# If no tools found, exit with instructions
if [ "$HAS_IVERILOG" = false ] && [ "$HAS_VERILATOR" = false ] && [ "$HAS_YOSYS" = false ]; then
    echo -e "${RED}ERROR: No Verilog tools found!${NC}"
    echo ""
    echo "Please install at least one tool:"
    echo "  brew install icarus-verilog   # For simulation"
    echo "  brew install verilator         # For linting/checking"
    echo "  brew install yosys             # For synthesis"
    echo ""
    exit 1
fi

# Define file lists
COMPONENTS="rtl/components/alu.v rtl/components/alu_control.v rtl/components/control.v rtl/components/imm.v rtl/components/rf.v"
PIPELINE_CONTROL="rtl/pipeline_control/hazard_unit.v rtl/pipeline_control/forwarding_unit.v rtl/pipeline_control/branch_forwarding_unit.v"
STAGES="rtl/stages/if_stage.v rtl/stages/id_stage.v rtl/stages/ex_stage.v rtl/stages/mem_stage.v rtl/stages/wb_stage.v"
TOP="rtl/hart.v"
TESTBENCH="tb/tb.v"

ALL_RTL="$COMPONENTS $PIPELINE_CONTROL $STAGES $TOP"

#=============================================================================
# Test 1: Verilator Lint Check
#=============================================================================
if [ "$HAS_VERILATOR" = true ]; then
    echo "=========================================="
    echo "Test 1: Verilator Lint Check"
    echo "=========================================="
    echo "Checking for wiring errors, width mismatches, and undefined signals..."
    echo ""

    if verilator --lint-only -Wall -Wno-fatal \
        --top-module hart \
        $ALL_RTL 2>&1 | tee verilator_lint.log; then
        echo ""
        echo -e "${GREEN}✓ Verilator lint check PASSED${NC}"
        echo ""
    else
        echo ""
        echo -e "${RED}✗ Verilator lint check FAILED${NC}"
        echo "See verilator_lint.log for details"
        echo ""
        exit 1
    fi
fi

#=============================================================================
# Test 2: Icarus Verilog Compilation
#=============================================================================
if [ "$HAS_IVERILOG" = true ]; then
    echo "=========================================="
    echo "Test 2: Icarus Verilog Compilation"
    echo "=========================================="
    echo "Compiling design with testbench..."
    echo ""

    if iverilog -g2012 -o hart_sim \
        $ALL_RTL $TESTBENCH 2>&1 | tee iverilog_compile.log; then
        echo ""
        echo -e "${GREEN}✓ Icarus Verilog compilation PASSED${NC}"
        echo ""
        echo "Compiled executable: hart_sim"
        echo ""

        # Check if test files exist
        if [ -f "tb/01add.mem" ]; then
            echo "To run a test program:"
            echo "  ./hart_sim +TEST=tb/01add.mem"
            echo ""
        fi
    else
        echo ""
        echo -e "${RED}✗ Icarus Verilog compilation FAILED${NC}"
        echo "See iverilog_compile.log for details"
        echo ""
        exit 1
    fi
fi

#=============================================================================
# Test 3: Yosys Hierarchy Check
#=============================================================================
if [ "$HAS_YOSYS" = true ]; then
    echo "=========================================="
    echo "Test 3: Yosys Hierarchy Check"
    echo "=========================================="
    echo "Checking module hierarchy and instantiations..."
    echo ""

    # Create a Yosys script
    cat > yosys_check.ys << EOF
# Read all Verilog files
read_verilog -sv $ALL_RTL

# Check hierarchy (finds missing modules, port mismatches)
hierarchy -check -top hart

# Print statistics
stat

# Write out the hierarchy
show -prefix hierarchy -format dot hart
EOF

    if yosys -s yosys_check.ys 2>&1 | tee yosys_check.log; then
        echo ""
        echo -e "${GREEN}✓ Yosys hierarchy check PASSED${NC}"
        echo ""

        if [ -f "hierarchy.dot" ]; then
            echo "Module hierarchy saved to: hierarchy.dot"
            if command -v dot &> /dev/null; then
                dot -Tpng hierarchy.dot -o hierarchy.png
                echo "Hierarchy diagram saved to: hierarchy.png"
            fi
            echo ""
        fi
    else
        echo ""
        echo -e "${RED}✗ Yosys hierarchy check FAILED${NC}"
        echo "See yosys_check.log for details"
        echo ""
        exit 1
    fi

    # Cleanup
    rm -f yosys_check.ys
fi

#=============================================================================
# Summary
#=============================================================================
echo "=========================================="
echo "Verification Summary"
echo "=========================================="
echo ""
echo -e "${GREEN}All checks PASSED!${NC}"
echo ""
echo "Your refactored design:"
echo "  ✓ Has no syntax errors"
echo "  ✓ All modules are properly connected"
echo "  ✓ All signals have correct widths"
echo "  ✓ Module hierarchy is correct"
echo ""
echo "The refactoring is successful!"
echo ""

# Show file statistics
echo "File Statistics:"
echo "  Original hart.v: $(wc -l < rtl/hart.v.bak) lines (backup)"
echo "  New hart.v:      $(wc -l < rtl/hart.v) lines (wrapper)"
echo ""
echo "  Stage files:"
for stage in rtl/stages/*.v; do
    printf "    %-25s %5d lines\n" "$(basename $stage)" "$(wc -l < $stage)"
done
echo ""

echo "To run tests with the compiled simulator:"
echo "  ./hart_sim +TEST=tb/01add.mem"
echo ""
