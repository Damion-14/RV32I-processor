.ONESHELL:
PROGRAM ?= 01add
assemble:
	riscv64-unknown-elf-as -march=rv32i -mabi=ilp32 $(PROGRAM).asm -o $(PROGRAM).o
	riscv64-unknown-elf-objcopy -O binary -j .text --pad-to=0x400 $(PROGRAM).o $(PROGRAM).hex
	xxd -g 1 -c 4 -p $(PROGRAM).hex | sed 's/../& /g' > ../../tb/program.mem


clean:
	rm -f *.o *.hex ../../tb/program.mem ../../tb/a.out