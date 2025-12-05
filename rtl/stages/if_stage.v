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
    output wire        o_imem_ren,         // Instruction memory read enable
    input  wire [31:0] i_imem_rdata,       // Instruction word from memory
    input  wire        i_imem_ready,       // Instruction memory ready
    input  wire        i_imem_valid,       // Instruction memory data valid

    //=========================================================================
    // OUTPUTS TO ID STAGE
    //=========================================================================
    output wire [31:0] o_inst,             // Current instruction word
    output wire [31:0] o_fetch_pc,         // PC of instruction arriving this cycle
    output wire [31:0] o_pc_plus_4,         // PC + 4 for sequential execution
    output wire        o_cache_busy         // Instruction cache busy signal
);

    //=========================================================================
    // INSTRUCTION CACHE INTERFACE SIGNALS
    //=========================================================================
    wire        cache_busy;
    wire [31:0] cache_rdata;
    wire [31:0] cache_mem_addr;
    wire        cache_mem_ren;

    // Track when a miss is in-flight so we only assert i_req_ren once
    reg         icache_waiting;
    reg  [31:0] icache_req_addr_q;
    wire        cache_req_fire;
    wire [31:0] cache_req_addr;

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
    // INSTRUCTION CACHE REQUEST CONTROL
    //=========================================================================
    // Delay the stall signal by one cycle for the cache request fire logic to
    // avoid a combinational loop between the cache busy flag -> stall_pc ->
    // cache_req_fire path. The immediate (combinational) stall signal still
    // feeds the PC update logic directly above.
    reg stall_pc_q;

    always @(posedge i_clk) begin
        if (i_rst) begin
            stall_pc_q <= 1'b0;
        end else begin
            stall_pc_q <= i_stall_pc;
        end
    end

    assign cache_req_fire = !stall_pc_q && !icache_waiting;
    assign cache_req_addr = icache_waiting ? icache_req_addr_q : pc;

    always @(posedge i_clk) begin
        if (i_rst) begin
            icache_waiting     <= 1'b0;
            icache_req_addr_q  <= RESET_ADDR;
        end else begin
            if (!cache_busy) begin
                icache_waiting <= 1'b0;
            end else if (cache_req_fire) begin
                icache_waiting <= 1'b1;
            end

            if (cache_req_fire) begin
                icache_req_addr_q <= pc;
            end
        end
    end

    //=========================================================================
    // INSTRUCTION CACHE
    //=========================================================================
    cache icache (
        .i_clk         (i_clk),
        .i_rst         (i_rst),
        .i_mem_ready   (i_imem_ready),
        .o_mem_addr    (cache_mem_addr),
        .o_mem_ren     (cache_mem_ren),
        .o_mem_wen     (),
        .o_mem_wdata   (),
        .i_mem_rdata   (i_imem_rdata),
        .i_mem_valid   (i_imem_valid),
        .o_busy        (cache_busy),
        .i_req_addr    (cache_req_addr),
        .i_req_ren     (cache_req_fire),
        .i_req_wen     (1'b0),
        .i_req_mask    (4'b1111),
        .i_req_wdata   (32'b0),
        .o_res_rdata   (cache_rdata)
    );

    assign o_imem_raddr = cache_mem_addr;
    assign o_imem_ren   = cache_mem_ren;

    //=========================================================================
    // OUTPUTS
    //=========================================================================
    assign o_inst        = cache_rdata;      // Instruction from cache (hit or filled miss)
    assign o_fetch_pc    = fetch_pc;         // PC corresponding to current instruction
    assign o_pc_plus_4   = fetch_pc + 32'd4; // Next sequential PC
    assign o_cache_busy  = cache_busy;

endmodule

`default_nettype wire

//=============================================================================
// END OF IF STAGE MODULE
//=============================================================================
