onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /hart_tb/dut/i_clk
add wave -noupdate /hart_tb/dut/i_rst
add wave -noupdate /hart_tb/dut/inst
add wave -noupdate /hart_tb/dut/o_imem_raddr
add wave -noupdate /hart_tb/dut/i_imem_rdata
add wave -noupdate /hart_tb/dut/o_dmem_addr
add wave -noupdate /hart_tb/dut/o_dmem_ren
add wave -noupdate /hart_tb/dut/o_dmem_wen
add wave -noupdate /hart_tb/dut/o_dmem_wdata
add wave -noupdate /hart_tb/dut/o_dmem_mask
add wave -noupdate /hart_tb/dut/i_dmem_rdata
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_valid
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_inst
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_trap
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_halt
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_rs1_raddr
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_rs2_raddr
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_rs1_rdata
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_rs2_rdata
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_rd_waddr
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_rd_wdata
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_pc
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_next_pc
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_dmem_addr
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_dmem_ren
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_dmem_wen
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_dmem_mask
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_dmem_wdata
add wave -noupdate -expand -group retire /hart_tb/dut/o_retire_dmem_rdata
add wave -noupdate -divider bottom
add wave -noupdate -expand -group {IF/ID Pipe} /hart_tb/dut/if_id_inst
add wave -noupdate -expand -group {IF/ID Pipe} /hart_tb/dut/if_id_pc
add wave -noupdate -expand -group {IF/ID Pipe} /hart_tb/dut/if_id_next_pc
add wave -noupdate -expand -group {IF/ID Pipe} /hart_tb/dut/if_id_valid
add wave -noupdate -divider bottom
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_pc
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_rs1_data
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_rs2_data
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_imm
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_rs1
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_rs2
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_rd
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_alu_op
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_bj_type
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_alu_src
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_mem_read
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_mem_write
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_mem_to_reg
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_reg_write
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_opcode
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_pc_plus_4
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_funct3
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_funct7
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_inst
add wave -noupdate -expand -group {ID/EX Pipe} /hart_tb/dut/id_ex_valid
add wave -noupdate -divider bottom
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_pc
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_rs1_data
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_rs2_data
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_imm
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_rs1
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_rs2
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_mem_read
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_mem_write
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_mem_to_reg
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_opcode
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_pc_plus_4
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_funct3
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_funct7
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_is_jal
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_is_jalr
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_is_branch
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_is_store
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_inst
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_unaligned_pc
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_valid
add wave -noupdate -expand -group {EX/MEM Pipe} /hart_tb/dut/ex_mem_next_pc
add wave -noupdate -divider bottom
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_mem_read_data
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_alu_result
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_mem_to_reg
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_pc_plus_4
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_opcode
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_imm
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_is_jal
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_is_jalr
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_is_branch
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_mem_read
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_mem_write
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_funct3
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_rs1
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_rs2
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_rs1_data
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_rs2_data
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_pc
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_inst
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_is_store
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_unaligned_pc
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_unaligned_mem
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_valid
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_dmem_addr
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_dmem_mask
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_dmem_wdata
add wave -noupdate -expand -group {MEM/WB Pipe} /hart_tb/dut/mem_wb_next_pc
add wave -noupdate -divider bottom
add wave -noupdate /hart_tb/dut/stall_pc
add wave -noupdate /hart_tb/dut/stall_if_id
add wave -noupdate /hart_tb/dut/bubble_id_ex
add wave -noupdate /hart_tb/dut/flush
add wave -noupdate /hart_tb/dut/ex_mem_alu_result
add wave -noupdate /hart_tb/dut/ex_mem_rd
add wave -noupdate /hart_tb/dut/ex_mem_reg_write
add wave -noupdate /hart_tb/dut/mem_wb_rd
add wave -noupdate /hart_tb/dut/mem_wb_reg_write
add wave -noupdate /hart_tb/dut/pc
add wave -noupdate /hart_tb/dut/pc_plus_4
add wave -noupdate /hart_tb/dut/next_pc
add wave -noupdate /hart_tb/dut/opcode
add wave -noupdate /hart_tb/dut/rd
add wave -noupdate /hart_tb/dut/rs1
add wave -noupdate /hart_tb/dut/rs2
add wave -noupdate /hart_tb/dut/funct3
add wave -noupdate /hart_tb/dut/funct7
add wave -noupdate /hart_tb/dut/U_sel
add wave -noupdate /hart_tb/dut/i_format
add wave -noupdate /hart_tb/dut/bj_type
add wave -noupdate /hart_tb/dut/alu_op
add wave -noupdate /hart_tb/dut/mem_read
add wave -noupdate /hart_tb/dut/mem_to_reg
add wave -noupdate /hart_tb/dut/mem_write
add wave -noupdate /hart_tb/dut/alu_src
add wave -noupdate /hart_tb/dut/reg_write
add wave -noupdate /hart_tb/dut/rs1_data
add wave -noupdate /hart_tb/dut/rs2_data
add wave -noupdate /hart_tb/dut/rd_data
add wave -noupdate /hart_tb/dut/imm
add wave -noupdate /hart_tb/dut/forward_a
add wave -noupdate /hart_tb/dut/forward_b
add wave -noupdate /hart_tb/dut/i_opsel
add wave -noupdate /hart_tb/dut/i_sub
add wave -noupdate /hart_tb/dut/i_unsigned
add wave -noupdate /hart_tb/dut/i_arith
add wave -noupdate /hart_tb/dut/func37
add wave -noupdate /hart_tb/dut/forwarded_rs1_data
add wave -noupdate /hart_tb/dut/forwarded_rs2_data
add wave -noupdate /hart_tb/dut/alu_op1
add wave -noupdate /hart_tb/dut/alu_op2
add wave -noupdate /hart_tb/dut/alu_result
add wave -noupdate /hart_tb/dut/alu_eq
add wave -noupdate /hart_tb/dut/alu_slt
add wave -noupdate /hart_tb/dut/is_branch
add wave -noupdate /hart_tb/dut/is_jal
add wave -noupdate /hart_tb/dut/is_jalr
add wave -noupdate /hart_tb/dut/branch_taken
add wave -noupdate /hart_tb/dut/branch_condition
add wave -noupdate /hart_tb/dut/branch_target
add wave -noupdate /hart_tb/dut/jalr_target
add wave -noupdate /hart_tb/dut/dmem_addr_unaligned
add wave -noupdate /hart_tb/dut/byte_offset
add wave -noupdate /hart_tb/dut/dmem_mask
add wave -noupdate /hart_tb/dut/store_data_shifted
add wave -noupdate /hart_tb/dut/load_data_processed
add wave -noupdate /hart_tb/dut/mem_read_data
add wave -noupdate /hart_tb/dut/is_lui
add wave -noupdate /hart_tb/dut/is_auipc
add wave -noupdate /hart_tb/dut/is_store
add wave -noupdate /hart_tb/dut/illegal_inst
add wave -noupdate /hart_tb/dut/unaligned_pc
add wave -noupdate /hart_tb/dut/unaligned_mem
add wave -noupdate /hart_tb/dut/unsupported_opcode
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/i_id_rs1
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/i_id_rs2
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/i_ex_rd
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/i_ex_reg_write
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/i_ex_mem_read
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/i_mem_rd
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/i_mem_reg_write
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/o_stall_pc
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/o_stall_if_id
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/o_bubble_id_ex
add wave -noupdate -expand -group {HAZZARD UNIT} /hart_tb/dut/hazard_detector/load_use_hazard
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {15 ns} 0}
quietly wave cursor active 1
configure wave -namecolwidth 278
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ns} {105 ns}
