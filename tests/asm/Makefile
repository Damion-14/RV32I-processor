.ONESHELL:
PROGRAM ?= 01add

# Find all .asm files and generate the base names
ASM_FILES := $(wildcard *.asm)
PROGRAMS := $(basename $(ASM_FILES))
MEM_FILES := $(patsubst %,../../tb/%.mem,$(PROGRAMS))

# Default target: compile single program (for backward compatibility)
assemble:
	docker run --rm -v $(PWD):/src coderkalyan/ece552-tools:latest bash -c "\
		cd /src && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM).asm -o $(PROGRAM).o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM).o $(PROGRAM).hex \
	"
	xxd -g 1 -c 4 -p $(PROGRAM).hex | sed 's/../& /g' > ../../tb/$(PROGRAM).mem
	@echo "Compiled $(PROGRAM).asm -> ../../tb/$(PROGRAM).mem"

# Compile all assembly files
all: $(MEM_FILES)
	@echo "Successfully compiled all test programs!"
	@echo "Generated files:"
	@ls -1 ../../tb/*.mem

# Pattern rule to compile individual programs
../../tb/%.mem: %.asm
	@echo "Compiling $<..."
	docker run --rm -v $(PWD):/src coderkalyan/ece552-tools:latest bash -c "\
		cd /src && \
		riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $*.asm -o $*.o && \
		riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $*.o $*.hex \
	"
	xxd -g 1 -c 4 -p $*.hex | sed 's/../& /g' > ../../tb/$*.mem
	@echo "  -> Generated ../../tb/$*.mem"

clean:
	rm -f *.o *.hex ../../tb/*.mem ../../tb/a.out

.PHONY: assemble all clean