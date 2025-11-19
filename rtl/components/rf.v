`default_nettype none

// The register file is effectively a single cycle memory with 32-bit words
// and depth 32. It has two asynchronous read ports, allowing two independent
// registers to be read at the same time combinationally, and one synchronous
// write port, allowing a register to be written to on the next clock edge.
//
// The register `x0` is hardwired to zero.
// NOTE: This can be implemented either by silently discarding writesto
// address 5'd0, or by muxing the output to zero when reading from that
// address.
module rf #(
    // When this parameter is set to 1, "RF bypass" mode is enabled. This
    // allows data at the write port to be observed at the read ports
    // immediately without having to wait for the next clock edge. This is
    // a common forwarding optimization in a pipelined core (phase 5), but will
    // cause a single-cycle processor to behave incorrectly. You are required
    // to implement and test both modes. In phase 4, you will disable this
    // parameter, before enabling it in phase 6.
    parameter BYPASS_EN = 0
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Both read register ports are asynchronous (zero-cycle). That is, read
    // data is visible combinationally without having to wait for a clock.
    //
    // The read ports are *independent* and can read two different registers
    // (but of course, also the same register if needed).
    //
    // Register `x0` is hardwired to zero, so reading from address 5'd0
    // should always return 32'd0 on either port regardless of any writes.
    //
    // Register read port 1, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs1_raddr,
    output wire [31:0] o_rs1_rdata,
    // Register read port 2, with input address [0, 31] and output data.
    input  wire [ 4:0] i_rs2_raddr,
    output wire [31:0] o_rs2_rdata,
    // The register write port is synchronous. When write is enabled, the
    // data at the write port will be written to the specified register
    // at the next clock edge. When the writen enable is low, the register
    // file should remain unchanged at the clock edge.
    //
    // Write register enable, address [0, 31] and input data.
    input  wire        i_rd_wen,
    input  wire [ 4:0] i_rd_waddr,
    input  wire [31:0] i_rd_wdata
);
    // Fill in your implementation here.
    reg [31:0] mem [31:0];

    always @(posedge i_clk) begin
        if (i_rst) begin
            // set registers to 0
            mem[0] <= 32'd0; mem[1] <= 32'd0; mem[2] <= 32'd0; mem[3] <= 32'd0;
            mem[4] <= 32'd0; mem[5] <= 32'd0; mem[6] <= 32'd0; mem[7] <= 32'd0;
            mem[8] <= 32'd0; mem[9] <= 32'd0; mem[10] <= 32'd0; mem[11] <= 32'd0;
            mem[12] <= 32'd0; mem[13] <= 32'd0; mem[14] <= 32'd0; mem[15] <= 32'd0;
            mem[16] <= 32'd0; mem[17] <= 32'd0; mem[18] <= 32'd0; mem[19] <= 32'd0;
            mem[20] <= 32'd0; mem[21] <= 32'd0; mem[22] <= 32'd0; mem[23] <= 32'd0;
            mem[24] <= 32'd0; mem[25] <= 32'd0; mem[26] <= 32'd0; mem[27] <= 32'd0;
            mem[28] <= 32'd0; mem[29] <= 32'd0; mem[30] <= 32'd0; mem[31] <= 32'd0;
        end else begin
            // normal register function
            if (i_rd_wen && i_rd_waddr != 5'd0) begin
                mem[i_rd_waddr] <= i_rd_wdata;
            end
        end
    end

    assign o_rs1_rdata = (i_rs1_raddr == 5'd0) ? 32'd0 :
                         (BYPASS_EN && i_rd_wen && (i_rd_waddr == i_rs1_raddr) && (i_rd_waddr != 5'd0)) ? i_rd_wdata :
                         mem[i_rs1_raddr];

    assign o_rs2_rdata = (i_rs2_raddr == 5'd0) ? 32'd0 :
                         (BYPASS_EN && i_rd_wen && (i_rd_waddr == i_rs2_raddr) && (i_rd_waddr != 5'd0)) ? i_rd_wdata :
                         mem[i_rs2_raddr];

endmodule

`default_nettype wire
