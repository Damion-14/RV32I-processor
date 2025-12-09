#=============================================================================
# Makefile for RV32I Pipelined Processor
#=============================================================================
# This Makefile provides convenient targets for compiling, testing, and
# verifying the refactored RV32I processor design.
#
# Quick Start:
#   make          - Compile and verify
#   make test     - Run all tests
#   make clean    - Clean build artifacts
#   make help     - Show all available targets
#=============================================================================

# Tool Configuration
IVERILOG := iverilog
VERILATOR := verilator
VVP := vvp
WAVEFORM_VIEWER := surfer

# Verilog Standard
VERILOG_STD := -g2012

# Directories
RTL_DIR := rtl
TB_DIR := tb
BUILD_DIR := build
BASELINE_DIR := baselines
BASELINE_DIFF_DIR := $(BASELINE_DIR)/diffs
TRACES_DIR := traces

# RTL Source Files (in dependency order)
COMPONENTS := $(wildcard $(RTL_DIR)/components/*.v)
PIPELINE_CONTROL := $(wildcard $(RTL_DIR)/pipeline_control/*.v)
STAGES := $(wildcard $(RTL_DIR)/stages/*.v)
TOP := $(RTL_DIR)/hart.v
CACHE_RTL := $(RTL_DIR)/cache.v
TESTBENCH := $(TB_DIR)/tb.v

ALL_RTL := $(COMPONENTS) $(PIPELINE_CONTROL) $(STAGES) $(TOP) $(CACHE_RTL)

# Simulator Executable
SIM := hart_sim
CACHE_SIM := cache_sim

# Cache Testbench Files
CACHE_TB := $(TB_DIR)/cache_tb.sv
CACHE_RTL := $(RTL_DIR)/cache.v
MEMORY_RTL := $(RTL_DIR)/tb_memory.v

# Test Files
TEST_FILES := $(sort $(wildcard $(TB_DIR)/*.mem))
TEST_NAMES := $(basename $(notdir $(TEST_FILES)))

# Color output
RED := \033[0;31m
GREEN := \033[0;32m
YELLOW := \033[1;33m
BLUE := \033[0;34m
NC := \033[0m # No Color

#=============================================================================
# Default Target
#=============================================================================

.DEFAULT_GOAL := all

.PHONY: all
all: compile
	@echo "$(GREEN)✓ Build complete!$(NC)"

#=============================================================================
# Compilation Targets
#=============================================================================

.PHONY: compile
compile: $(SIM)
	@echo "$(GREEN)✓ Compilation successful!$(NC)"
	@echo "  Simulator: $(SIM)"
	@echo "  To run: ./$(SIM)"

$(SIM): $(ALL_RTL) $(TESTBENCH)
	@echo "$(BLUE)Compiling RV32I processor...$(NC)"
	@$(IVERILOG) $(VERILOG_STD) -o $@ $(ALL_RTL) $(TESTBENCH)

#=============================================================================
# Cache Testbench Targets
#=============================================================================

.PHONY: cache
cache: $(CACHE_SIM)
	@echo "$(BLUE)Running cache testbench...$(NC)"
	@./$(CACHE_SIM)
	@echo "$(GREEN)✓ Cache simulation complete!$(NC)"
	@echo "  Waveform: cache.vcd"
	@echo "  To view: make cache-wave"

$(CACHE_SIM): $(CACHE_RTL) $(MEMORY_RTL) $(CACHE_TB)
	@echo "$(BLUE)Compiling cache testbench...$(NC)"
	@if [ ! -f $(MEMORY_RTL) ]; then \
		echo "$(RED)Error: Memory module not found at $(MEMORY_RTL)$(NC)"; \
		echo "$(YELLOW)Please ensure memory.v exists in rtl/ directory$(NC)"; \
		exit 1; \
	fi
	@$(IVERILOG) $(VERILOG_STD) -o $@ $(CACHE_RTL) $(MEMORY_RTL) $(CACHE_TB)
	@echo "$(GREEN)✓ Cache compilation successful!$(NC)"

.PHONY: cache-wave
cache-wave:
	@if [ -f cache.vcd ]; then \
		echo "$(BLUE)Opening cache waveform viewer...$(NC)"; \
		$(WAVEFORM_VIEWER) cache.vcd & \
	else \
		echo "$(RED)Error: No cache waveform file found. Run 'make cache' first.$(NC)"; \
		exit 1; \
	fi

#=============================================================================
# Verification Targets
#=============================================================================

.PHONY: verify
verify:
	@echo "$(BLUE)Running verification checks...$(NC)"
	@./verify_refactor.sh

.PHONY: lint
lint:
	@echo "$(BLUE)Running Verilator lint check...$(NC)"
	@$(VERILATOR) --lint-only -Wall -Wno-fatal \
		--top-module hart \
		$(ALL_RTL)
	@echo "$(GREEN)✓ Lint check passed!$(NC)"

#=============================================================================
# Test Targets
#=============================================================================

.PHONY: test
test:
	@echo "$(BLUE)=======================================$(NC)"
	@echo "$(BLUE)Running All Tests$(NC)"
	@echo "$(BLUE)=======================================$(NC)"
	@echo ""
	@passed=0; failed=0; \
	for test in $(TEST_NAMES); do \
		echo "$(YELLOW)Running test: $$test$(NC)"; \
		sed -i.bak "s|tb/[^\"]*\.mem|tb/$$test.mem|g" $(TB_DIR)/tb.v; \
		$(IVERILOG) $(VERILOG_STD) -o $(SIM) $(ALL_RTL) $(TESTBENCH) 2>/dev/null; \
		if ./$(SIM) 2>&1 | grep -q "Program halted"; then \
			cpi=$$(./$(SIM) 2>&1 | grep "CPI:" | awk '{print $$2}'); \
			echo "  $(GREEN)✓ PASSED$(NC) (CPI: $$cpi)"; \
			passed=$$((passed + 1)); \
		else \
			echo "  $(RED)✗ FAILED$(NC)"; \
			failed=$$((failed + 1)); \
		fi; \
		echo ""; \
	done; \
	mv $(TB_DIR)/tb.v.bak $(TB_DIR)/tb.v 2>/dev/null || true; \
	echo "$(BLUE)========================================$(NC)"; \
	echo "$(BLUE)Test Summary$(NC)"; \
	echo "$(BLUE)========================================$(NC)"; \
	echo "  $(GREEN)Passed: $$passed$(NC)"; \
	echo "  $(RED)Failed: $$failed$(NC)"; \
	echo "  Total:  $$((passed + failed))"; \
	if [ $$failed -eq 0 ]; then \
		echo ""; \
		echo "$(GREEN)✓ All tests passed!$(NC)"; \
	else \
		echo ""; \
		echo "$(RED)✗ Some tests failed!$(NC)"; \
		exit 1; \
	fi

# Individual test targets
.PHONY: test-%
test-%:
	@echo "$(BLUE)Running test: $*$(NC)"
	@test_file="$(TB_DIR)/$*.mem"; \
	if [ ! -f "$$test_file" ]; then \
		echo "$(RED)Error: Test file $$test_file not found!$(NC)"; \
		echo "Available tests:"; \
		for t in $(TEST_NAMES); do echo "  - $$t"; done; \
		exit 1; \
	fi
	@mkdir -p $(TRACES_DIR)
	@sed -i.bak "s|tb/[^\"]*\.mem|tb/$*.mem|g" $(TB_DIR)/tb.v
	@$(IVERILOG) $(VERILOG_STD) -o $(SIM) $(ALL_RTL) $(TESTBENCH) 2>/dev/null
	@./$(SIM) 2>&1 | tee $(TRACES_DIR)/test_$*.log
	@mv $(TB_DIR)/tb.v.bak $(TB_DIR)/tb.v 2>/dev/null || true

#=============================================================================
# Baseline Recording & Comparison
#=============================================================================

.PHONY: baseline-record
baseline-record:
	@echo "$(BLUE)Recording baseline logs for all tests...$(NC)"
	@mkdir -p $(BASELINE_DIR)
	@for test in $(TEST_NAMES); do \
		echo "$(YELLOW)[Baseline] Running $$test...$(NC)"; \
		$(MAKE) --no-print-directory test-$$test >/dev/null || exit 1; \
		cp $(TRACES_DIR)/test_$$test.log $(BASELINE_DIR)/$$test.log; \
	done
	@echo "$(GREEN)✓ Baseline logs stored in $(BASELINE_DIR)/$(NC)"

.PHONY: baseline-compare
baseline-compare:
	@echo "$(BLUE)Comparing current test outputs against baseline...$(NC)"
	@if [ ! -d $(BASELINE_DIR) ]; then \
		echo "$(RED)Error: Baseline directory $(BASELINE_DIR) not found. Run 'make baseline-record' first.$(NC)"; \
		exit 1; \
	fi
	@mkdir -p $(BASELINE_DIFF_DIR)
	@failed=0; \
	for test in $(TEST_NAMES); do \
		if [ ! -f $(BASELINE_DIR)/$$test.log ]; then \
			echo "$(YELLOW)Baseline for $$test missing. Skipping.$(NC)"; \
			continue; \
		fi; \
		echo "$(YELLOW)[Compare] Running $$test...$(NC)"; \
		$(MAKE) --no-print-directory test-$$test >/dev/null || exit 1; \
		if diff -q $(BASELINE_DIR)/$$test.log $(TRACES_DIR)/test_$$test.log >/dev/null; then \
			echo "  $(GREEN)Match$(NC)"; \
			rm -f $(BASELINE_DIFF_DIR)/$$test.diff; \
		else \
			echo "  $(RED)Mismatch — differences saved to $(BASELINE_DIFF_DIR)/$$test.diff$(NC)"; \
			diff -u $(BASELINE_DIR)/$$test.log $(TRACES_DIR)/test_$$test.log > $(BASELINE_DIFF_DIR)/$$test.diff || true; \
			failed=$$((failed + 1)); \
		fi; \
	done; \
	if [ $$failed -ne 0 ]; then \
		echo "$(RED)✗ Baseline comparison failed for $$failed test(s). See $(BASELINE_DIFF_DIR)/ for details.$(NC)"; \
		exit 1; \
	else \
		echo "$(GREEN)✓ All tests match baseline outputs.$(NC)"; \
	fi

# Convenient aliases for common tests
.PHONY: test-add test-branch test-memory test-shift
test-add: test-01add
test-branch: test-23beq test-24bge test-25bgeu test-26blt test-27bltu test-28bne
test-memory: test-06memory
test-shift: test-09sll test-10slli test-15sra test-16srai test-17srl test-18srli

#=============================================================================
# Interactive Testing
#=============================================================================

.PHONY: run
run: $(SIM)
	@echo "$(BLUE)Running simulator (default test)...$(NC)"
	@./$(SIM)

.PHONY: wave
wave:
	@if [ -f hart.vcd ]; then \
		echo "$(BLUE)Opening waveform viewer...$(NC)"; \
		$(WAVEFORM_VIEWER) hart.vcd & \
	else \
		echo "$(RED)Error: No waveform file found. Run a test first.$(NC)"; \
		exit 1; \
	fi

#=============================================================================
# Statistics and Analysis
#=============================================================================

.PHONY: stats
stats:
	@echo "$(BLUE)========================================$(NC)"
	@echo "$(BLUE)Design Statistics$(NC)"
	@echo "$(BLUE)========================================$(NC)"
	@echo ""
	@echo "RTL Files:"
	@echo "  Original hart.v:     $$(wc -l < $(RTL_DIR)/hart.v.bak) lines"
	@echo "  New hart.v (wrapper): $$(wc -l < $(RTL_DIR)/hart.v) lines"
	@echo ""
	@echo "Pipeline Stages:"
	@for stage in $(STAGES); do \
		printf "  %-30s %5d lines\n" "$$(basename $$stage)" "$$(wc -l < $$stage)"; \
	done
	@echo ""
	@echo "Components:"
	@for comp in $(COMPONENTS); do \
		printf "  %-30s %5d lines\n" "$$(basename $$comp)" "$$(wc -l < $$comp)"; \
	done
	@echo ""
	@echo "Pipeline Control:"
	@for ctrl in $(PIPELINE_CONTROL); do \
		printf "  %-30s %5d lines\n" "$$(basename $$ctrl)" "$$(wc -l < $$ctrl)"; \
	done
	@echo ""
	@total_lines=0; \
	for f in $(ALL_RTL); do \
		total_lines=$$((total_lines + $$(wc -l < $$f))); \
	done; \
	echo "Total RTL Lines: $$total_lines"

.PHONY: cpi-analysis
cpi-analysis:
	@echo "$(BLUE)========================================$(NC)"
	@echo "$(BLUE)CPI Analysis Across All Tests$(NC)"
	@echo "$(BLUE)========================================$(NC)"
	@echo ""
	@total_cpi=0; count=0; \
	for test in $(TEST_NAMES); do \
		sed -i.bak "s|tb/[^\"]*\.mem|tb/$$test.mem|g" $(TB_DIR)/tb.v; \
		$(IVERILOG) $(VERILOG_STD) -o $(SIM) $(ALL_RTL) $(TESTBENCH) 2>/dev/null; \
		cpi=$$(./$(SIM) 2>&1 | grep "CPI:" | awk '{print $$2}'); \
		if [ -n "$$cpi" ]; then \
			printf "  %-20s CPI: %s\n" "$$test" "$$cpi"; \
			total_cpi=$$(echo "$$total_cpi + $$cpi" | bc); \
			count=$$((count + 1)); \
		fi; \
	done; \
	mv $(TB_DIR)/tb.v.bak $(TB_DIR)/tb.v 2>/dev/null || true; \
	if [ $$count -gt 0 ]; then \
		avg_cpi=$$(echo "scale=6; $$total_cpi / $$count" | bc); \
		echo ""; \
		echo "$(GREEN)Average CPI: $$avg_cpi$(NC)"; \
	fi

#=============================================================================
# Submission Target
#=============================================================================

.PHONY: submit
submit:
	@echo "$(BLUE)Creating submission package...$(NC)"
	@mkdir -p submission
	@rm -f submission/*.v
	@echo "$(BLUE)Copying RTL files to submission folder...$(NC)"
	@for file in $(ALL_RTL); do \
		cp $$file submission/; \
		echo "  Copied $$(basename $$file)"; \
	done
	@echo ""
	@echo "$(GREEN)✓ Submission package created!$(NC)"
	@echo "  Location: ./submission/"
	@echo "  Files copied: $$(ls -1 submission/*.v | wc -l)"
	@echo ""
	@echo "Submission contents:"
	@ls -1 submission/

#=============================================================================
# Cleanup Targets
#=============================================================================

.PHONY: clean
clean:
	@echo "$(BLUE)Cleaning build artifacts...$(NC)"
	@rm -f $(SIM)
	@rm -f $(CACHE_SIM)
	@rm -f hart.vcd
	@rm -f cache.vcd
	@rm -f *.log
	@rm -f *.vcd
	@rm -f verilator_lint.log
	@rm -f iverilog_compile.log
	@rm -f yosys_check.log
	@rm -f yosys_check.ys
	@rm -f hierarchy.dot hierarchy.png
	@rm -f $(TB_DIR)/tb.v.bak
	@echo "$(GREEN)✓ Clean complete!$(NC)"

.PHONY: distclean
distclean: clean
	@echo "$(BLUE)Removing all generated files...$(NC)"
	@rm -rf $(BUILD_DIR)
	@rm -rf submission
	@echo "$(GREEN)✓ Distclean complete!$(NC)"

#=============================================================================
# Documentation and Help
#=============================================================================

.PHONY: help
help:
	@echo "$(BLUE)========================================$(NC)"
	@echo "$(BLUE)RV32I Processor Makefile Help$(NC)"
	@echo "$(BLUE)========================================$(NC)"
	@echo ""
	@echo "$(YELLOW)Build Targets:$(NC)"
	@echo "  make               - Compile and verify (default)"
	@echo "  make compile       - Compile the processor design"
	@echo "  make verify        - Run full verification suite"
	@echo "  make lint          - Run Verilator lint check"
	@echo ""
	@echo "$(YELLOW)Test Targets:$(NC)"
	@echo "  make test          - Run all test programs"
	@echo "  make test-<name>   - Run specific test (e.g., test-add)"
	@echo "  make test-add      - Run arithmetic tests"
	@echo "  make test-branch   - Run all branch tests"
	@echo "  make test-memory   - Run memory operation tests"
	@echo "  make test-shift    - Run shift instruction tests"
	@echo ""
	@echo "$(YELLOW)Cache Targets:$(NC)"
	@echo "  make cache         - Compile and run cache testbench"
	@echo "  make cache-wave    - Open cache waveform viewer (GTKWave)"
	@echo ""
	@echo "$(YELLOW)Available Individual Tests:$(NC)"
	@for test in $(TEST_NAMES); do \
		echo "  - $$test"; \
	done | head -10
	@echo "  ... and $$(echo $(TEST_NAMES) | wc -w) more (run 'make list-tests' to see all)"
	@echo ""
	@echo "$(YELLOW)Analysis Targets:$(NC)"
	@echo "  make stats         - Show design statistics"
	@echo "  make cpi-analysis  - Analyze CPI across all tests"
	@echo ""
	@echo "$(YELLOW)Interactive Targets:$(NC)"
	@echo "  make run           - Run simulator with default test"
	@echo "  make wave          - Open waveform viewer (GTKWave)"
	@echo ""
	@echo "$(YELLOW)Submission:$(NC)"
	@echo "  make submit        - Create flat submission package in ./submission/"
	@echo ""
	@echo "$(YELLOW)Cleanup Targets:$(NC)"
	@echo "  make clean         - Remove build artifacts"
	@echo "  make distclean     - Remove all generated files"
	@echo ""
	@echo "$(YELLOW)Documentation:$(NC)"
	@echo "  See TESTING.md for detailed testing guide"
	@echo "  See rtl/README.md for architecture documentation"
	@echo ""

.PHONY: list-tests
list-tests:
	@echo "$(BLUE)Available Tests:$(NC)"
	@for test in $(TEST_NAMES); do \
		echo "  - $$test"; \
	done

.PHONY: info
info:
	@echo "$(BLUE)========================================$(NC)"
	@echo "$(BLUE)Build Configuration$(NC)"
	@echo "$(BLUE)========================================$(NC)"
	@echo ""
	@echo "Tools:"
	@echo "  Verilog Compiler: $(IVERILOG)"
	@echo "  Verilog Standard: $(VERILOG_STD)"
	@echo "  Verilator:        $(VERILATOR)"
	@echo ""
	@echo "Directories:"
	@echo "  RTL Source:       $(RTL_DIR)"
	@echo "  Testbench:        $(TB_DIR)"
	@echo "  Build:            $(BUILD_DIR)"
	@echo ""
	@echo "Files:"
	@echo "  Components:       $$(echo $(COMPONENTS) | wc -w) files"
	@echo "  Pipeline Control: $$(echo $(PIPELINE_CONTROL) | wc -w) files"
	@echo "  Pipeline Stages:  $$(echo $(STAGES) | wc -w) files"
	@echo "  Test Programs:    $$(echo $(TEST_FILES) | wc -w) files"
	@echo ""

#=============================================================================
# Special Targets
#=============================================================================

# Prevent deletion of intermediate files
.PRECIOUS: $(SIM)

# Disable built-in rules
.SUFFIXES:

# Ensure these targets always run
.PHONY: all compile verify lint test run wave stats cpi-analysis clean distclean help list-tests info cache cache-wave
