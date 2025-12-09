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
    output wire        o_inst_valid,        // Instruction valid signal
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

    reg         cache_waiting;            // Outstanding miss in flight
    reg  [31:0] cache_req_addr_q;         // Address associated with request
    reg         drop_resp;                // Drop next cache response (redirect issued)
    reg         inst_valid_q;             // Buffered instruction valid flag

    wire        cache_req_fire;           // Asserted when a new request should issue
    wire [31:0] cache_req_addr;           // Address presented to the cache
    wire        cache_resp_valid;         // Cache returned a word this cycle
    wire [31:0] resp_pc;                  // PC associated with cache response
    wire        pc_hold;                  // Local hold signal for the PC

    //=========================================================================
    // PROGRAM COUNTER LOGIC
    //=========================================================================
    // Current program counter register
    reg  [31:0] pc;
    wire [31:0] pc_plus_4;

    // Track the PC associated with the instruction currently being presented
    // to the ID stage so we can keep the frontend frozen while the cache
    // services a miss (the cache contract requires the CPU to hold request
    // signals steady while `o_busy` is asserted).
    reg [31:0] fetch_pc;
    reg [31:0] inst_q;
    reg valid_blanking;
    reg cache_waiting_1;

    always @(posedge i_clk) begin
        if(i_rst) begin
            cache_waiting_1 <= 1'b0;
        end else
            cache_waiting_1 <= cache_waiting;
    end

    always @(negedge cache_waiting or posedge i_clk) begin
        if (i_rst) begin
            valid_blanking <= 1'b0;
        end else begin
            if (cache_waiting_1 && !cache_waiting) begin
                valid_blanking <= 1'b1;
        end else begin
                valid_blanking <= 1'b0;
        end
    end
    end


    always @(posedge i_clk) begin
        if (i_rst) begin
            fetch_pc     <= RESET_ADDR;
            inst_q       <= 32'h00000013;      // Treat reset like a NOP bubble
            inst_valid_q <= 1'b0;
        end else begin
            if (cache_resp_valid && !drop_resp && !valid_blanking) begin
                fetch_pc     <= resp_pc;
                inst_q       <= cache_rdata;
                inst_valid_q <= 1'b1;
            end else if (!i_stall_pc && inst_valid_q) begin
                inst_valid_q <= 1'b0;          // ID consumed buffered instruction
            end

            if (i_pc_redirect) begin
                inst_valid_q <= 1'b0;          // Flush buffered instruction on redirect
            end
        end
        // Hold the previous instruction/PC whenever no new word is ready.
    end

    assign pc_plus_4 = pc + 32'd4;         // Calculate next sequential PC
    assign pc_hold   = i_stall_pc || cache_waiting || inst_valid_q;

    // PC Update (Synchronous)
    // PC is updated every clock cycle unless stalled by hazard detection
    always @(posedge i_clk) begin
        if (i_rst) begin
            pc <= RESET_ADDR;              // Reset PC to specified address
        end else if (i_pc_redirect) begin
            pc <= i_pc_redirect_target;    // Immediate redirect has priority
        end else if (!pc_hold) begin
            pc <= pc_plus_4;               // Default sequential PC+4
        end
        // else: PC holds its current value during stall or while buffer busy
    end



    //=========================================================================
    // INSTRUCTION CACHE REQUEST CONTROL
    //=========================================================================

    assign cache_req_fire   = !inst_valid_q && !cache_waiting && !valid_blanking && !i_rst;
    assign cache_req_addr   = cache_waiting ? cache_req_addr_q : pc;
    assign cache_resp_valid = !cache_busy && (cache_waiting || cache_req_fire);
    assign resp_pc          = cache_waiting ? cache_req_addr_q : pc;

    assign o_inst_valid = inst_valid_q;

    // Track outstanding miss state, request address, and redirect drops
    always @(posedge i_clk) begin
        if (i_rst) begin
            cache_waiting    <= 1'b0;
            cache_req_addr_q <= RESET_ADDR;
            drop_resp        <= 1'b0;
        end else begin
            if (cache_req_fire) begin
                cache_req_addr_q <= pc;       // Remember address of newest request
            end

            if (cache_resp_valid) begin
                cache_waiting <= 1'b0;
                drop_resp     <= 1'b0;
            end else if (cache_req_fire && cache_busy) begin
                cache_waiting <= 1'b1;        // Miss detected, wait for fill
            end

            if (i_pc_redirect && (cache_waiting || cache_req_fire)) begin
                drop_resp <= 1'b1;            // Discard in-flight response on redirect
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
    assign o_inst        = inst_q;           // Registered instruction from cache
    assign o_fetch_pc    = fetch_pc;         // PC corresponding to current instruction
    assign o_pc_plus_4   = fetch_pc + 32'd4; // Next sequential PC
    assign o_cache_busy  = cache_busy;

endmodule

`default_nettype wire

//=============================================================================
// END OF IF STAGE MODULE
//=============================================================================
