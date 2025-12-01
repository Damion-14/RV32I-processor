//=============================================================================
// RISC-V RV32I Pipelined Processor - Instruction Fetch (IF) Stage
//=============================================================================
// This module implements the Instruction Fetch (IF) stage of the 5-stage
// pipelined RISC-V processor. The IF stage is responsible for:
// - Managing the Program Counter (PC)
// - Fetching instructions from instruction memory
// - Computing PC+4 for sequential execution
// - Handling PC stalls and control flow changes
//
// Note: The IF/ID pipeline registers are located in the ID stage module.
//=============================================================================

`default_nettype none

module if_stage #(
    parameter RESET_ADDR = 32'h00000000    // Reset PC address
) (
    //=========================================================================
    // GLOBAL SIGNALS
    //=========================================================================
    input  wire        i_clk,              // Global clock
    input  wire        i_rst,              // Synchronous active-high reset

    //=========================================================================
    // CONTROL SIGNALS
    //=========================================================================
    input  wire        i_stall_pc,         // Stall PC update (from hazard unit)
    input  wire        i_pc_redirect,      // Override PC with branch/jump target
    input  wire [31:0] i_pc_redirect_target, // Branch/jump target from ID stage

    //=========================================================================
    // INSTRUCTION MEMORY INTERFACE
    //=========================================================================
    output wire [31:0] o_imem_raddr,       // Instruction memory read address
    input  wire [31:0] i_imem_rdata,       // Instruction word from memory

    //=========================================================================
    // OUTPUTS TO ID STAGE
    //=========================================================================
    output wire [31:0] o_inst,             // Current instruction word
    output wire [31:0] o_fetch_pc,         // PC of instruction arriving this cycle
    output wire [31:0] o_pc_plus_4         // PC + 4 for sequential execution
);

    //=========================================================================
    // PROGRAM COUNTER LOGIC
    //=========================================================================
    // Current program counter register
    reg  [31:0] pc;
    wire [31:0] pc_plus_4;

    // The testbench models imem as synchronous (1-cycle latency). The instruction
    // observed on 'inst' corresponds to the address presented on 'o_imem_raddr'
    // in the previous cycle. Track that address so we can align IF/ID.pc with
    // the arriving instruction correctly.
    reg [31:0] fetch_pc;                   // PC used for the instruction arriving this cycle

    always @(posedge i_clk) begin
        if (i_rst) begin
            fetch_pc <= RESET_ADDR;
        end else if (!i_stall_pc) begin
            fetch_pc <= pc;                // Address driven to imem this cycle
        end else begin
            fetch_pc <= fetch_pc;          // Hold during stall
        end
    end

    assign pc_plus_4 = pc + 32'd4;         // Calculate next sequential PC
    assign o_imem_raddr = (i_stall_pc) ? fetch_pc : pc;  // Send current PC to instruction memory

    // PC Update (Synchronous)
    // PC is updated every clock cycle unless stalled by hazard detection
    always @(posedge i_clk) begin
        if (i_rst) begin
            pc <= RESET_ADDR;              // Reset PC to specified address
        end else if (!i_stall_pc) begin
            pc <= i_pc_redirect ? i_pc_redirect_target
                                 : pc_plus_4;         // Default sequential PC+4
        end
        // else: PC holds its current value during stall
    end

    //=========================================================================
    // OUTPUTS
    //=========================================================================
    assign o_inst      = i_imem_rdata;     // Pass through instruction from memory
    assign o_fetch_pc  = fetch_pc;         // PC corresponding to current instruction
    assign o_pc_plus_4 = fetch_pc + 32'd4; // Next sequential PC

endmodule

`default_nettype wire

//=============================================================================
// END OF IF STAGE MODULE
//=============================================================================
