
module ctl (
    input wire [ 6:0 ] instruction,

    output wire [ 1:0 ] U_sel,
    output wire [ 2:0 ] Imm_sel,
    output wire [ 2:0 ] bj_type,
    output wire [ 5:0 ] alu_op,
    output wire mem_read,
    output wire mem_write,
    output wire mem_to_reg,
    output wire alu_src,
    output wire reg_write 
);
// TODO

endmodule