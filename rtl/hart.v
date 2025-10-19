module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available on the same cycle.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002003`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
	// The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);
    // Fill in your implementation here.
	

    // PROGRAM COUNTER
    reg [31:0] pc;
    wire [31:0] pc_plus_4;
    wire [31:0] next_pc;
 
    assign pc_plus_4 = pc + 32'd4;
    assign o_imem_raddr = pc;


    // INSTRUCTION FETCH
    wire [31:0] inst;
    assign inst = i_imem_rdata;
    
 
    // INSTRUCTION DECODE
    wire [6:0] opcode;
    wire [4:0] rd, rs1, rs2;
    wire [2:0] funct3;
    wire [6:0] funct7;
    
    assign opcode = inst[6:0];
    assign rd     = inst[11:7];
    assign funct3 = inst[14:12];
    assign rs1    = inst[19:15];
    assign rs2    = inst[24:20];
    assign funct7 = inst[31:25];

  
    // CONTROL UNIT
    wire [1:0] U_sel;
    wire [5:0] i_format; 
    wire [2:0] bj_type;     
    wire [1:0] alu_op;    
    wire mem_read;         
    wire mem_to_reg;       
    wire mem_write;        
    wire alu_src;           
    wire reg_write;        
    
    ctl control_unit (
        .instruction(inst),
        .U_sel(U_sel),
        .i_format(i_format),
        .bj_type(bj_type),
        .alu_op(alu_op),
        .mem_read(mem_read),
        .mem_to_reg(mem_to_reg),
        .mem_write(mem_write),
        .alu_src(alu_src),
        .reg_write(reg_write)
    );


    // REGISTER FILE  
    wire [31:0] rs1_data, rs2_data;  
    wire [31:0] rd_data;              
    
    rf #(.BYPASS_EN(0)) register_file (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_rs1_raddr(rs1),              
        .i_rs2_raddr(rs2),              
        .i_rd_waddr(rd),                
        .i_rd_wdata(rd_data),          
        .i_rd_wen(reg_write),           
        .o_rs1_rdata(rs1_data),         
        .o_rs2_rdata(rs2_data)          
    );


    // IMMEDIATE GENERATOR
    wire [31:0] imm;  
    
    imm imm_gen (
        .i_inst(inst),
        .i_format(i_format), 
        .o_immediate(imm)
    );

 
    // ALU CONTROL
    wire [2:0] i_opsel;     
    wire i_sub;            
    wire i_unsigned;        
    wire i_arith;           
    wire [3:0] func37;
    
    assign func37 = {funct7[5], funct3};
    
    alu_ctl alu_control_unit (
        .alu_op(alu_op),       
        .func37(func37),       
        .i_opsel(i_opsel),     
        .i_sub(i_sub),          
        .i_unsigned(i_unsigned),
        .i_arith(i_arith)      
    );

   
    // ALU
	wire [31:0] alu_op1, alu_op2;
    
    assign alu_op1 = rs1_data;  
    assign alu_op2 = alu_src ? imm : rs2_data;     
    
    wire [31:0] alu_result;  
    wire alu_eq;             
    wire alu_slt;           
    
    alu alu_unit (
        .i_opsel(i_opsel),
        .i_sub(i_sub),
        .i_unsigned(i_unsigned),
        .i_arith(i_arith),
        .i_op1(alu_op1),        
        .i_op2(alu_op2),        
        .o_result(alu_result),  
        .o_eq(alu_eq),         
        .o_slt(alu_slt)         
    );


    // BRANCH/JUMP LOGIC
    wire is_branch;
    wire is_jal;
    wire is_jalr;
    wire branch_taken;
    
    assign is_branch = (opcode == 7'b1100011);
    assign is_jal = (opcode == 7'b1101111);
    assign is_jalr = (opcode == 7'b1100111);
    
    // Branch condition based on bj_type
    reg branch_condition;
    always @(*) begin
        case (bj_type)
            3'b000: branch_condition = alu_eq;         
            3'b001: branch_condition = ~alu_eq;         
            3'b100: branch_condition = alu_slt;          
            3'b101: branch_condition = ~alu_slt;         
            3'b110: branch_condition = alu_slt;         
            3'b111: branch_condition = ~alu_slt;         
            default: branch_condition = 1'b0;
        endcase
    end
    
    assign branch_taken = is_branch & branch_condition;


    // PC Control
    wire [31:0] branch_target;
    wire [31:0] jalr_target;
    
    assign branch_target = pc + imm;
    assign jalr_target = (rs1_data + imm) & ~32'd1;
    
    assign next_pc = is_jalr ? jalr_target :
                     (is_jal | branch_taken) ? branch_target :
                     pc_plus_4;
    
    // PC Update
    always @(posedge i_clk) begin
        if (i_rst) begin
            pc <= RESET_ADDR;
        end else begin
            pc <= next_pc;
        end
    end
	

    // DATA MEMORY INTERFACE
	//TODO
	
	
    // WRITE BACK
	//TODO





    // RETIRE INTERFACE   
    assign o_retire_valid = 1'b1;  // Always valid in single-cycle processor
    assign o_retire_inst = inst;
    assign o_retire_trap = illegal_inst | unaligned_pc | unaligned_mem;
    assign o_retire_halt = (opcode == 7'b1110011) && (funct3 == 3'b000) && (inst[31:20] == 12'h001); // EBREAK detection
    assign o_retire_rs1_raddr = rs1;
    assign o_retire_rs2_raddr = rs2;
    assign o_retire_rs1_rdata = rs1_data;
    assign o_retire_rs2_rdata = rs2_data;
    assign o_retire_rd_waddr = rd;
    assign o_retire_rd_wdata = rd_data;
    assign o_retire_pc = pc;
    assign o_retire_next_pc = next_pc;
endmodule

`default_nettype wire
