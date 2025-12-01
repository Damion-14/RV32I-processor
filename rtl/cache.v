`default_nettype none

module cache (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // External memory interface. See hart interface for details. This
    // interface is nearly identical to the phase 5 memory interface, with the
    // exception that the byte mask (`o_mem_mask`) has been removed. This is
    // no longer needed as the cache will only access the memory at word
    // granularity, and implement masking internally.
    input  wire        i_mem_ready,
    output wire [31:0] o_mem_addr,
    output wire        o_mem_ren,
    output wire        o_mem_wen,
    output wire [31:0] o_mem_wdata,
    input  wire [31:0] i_mem_rdata,
    input  wire        i_mem_valid,
    // Interface to CPU hart. This is nearly identical to the phase 5 hart memory
    // interface, but includes a stall signal (`o_busy`), and the input/output
    // polarities are swapped for obvious reasons.
    //
    // The CPU should use this as a stall signal for both instruction fetch
    // (IF) and memory (MEM) stages, from the instruction or data cache
    // respectively. If a memory request is made (`i_req_ren` for instruction
    // cache, or either `i_req_ren` or `i_req_wen` for data cache), this
    // should be asserted *combinationally* if the request results in a cache
    // miss.
    //
    // In case of a cache miss, the CPU must stall the respective pipeline
    // stage and deassert ren/wen on subsequent cycles, until the cache
    // deasserts `o_busy` to indicate it has serviced the cache miss. However,
    // the CPU must keep the other request lines constant. For example, the
    // CPU should not change the request address while stalling.
    output wire        o_busy,
    // 32-bit read/write address to access from the cache. This should be
    // 32-bit aligned (i.e. the two LSBs should be zero). See `i_req_mask` for
    // how to perform half-word and byte accesses to unaligned addresses.
    input  wire [31:0] i_req_addr,
    // When asserted, the cache should perform a read at the aligned address
    // specified by `i_req_addr` and return the 32-bit word at that address,
    // either immediately (i.e. combinationally) on a cache hit, or
    // synchronously on a cache miss. It is illegal to assert this and
    // `i_dmem_wen` on the same cycle.
    input  wire        i_req_ren,
    // When asserted, the cache should perform a write at the aligned address
    // specified by `i_req_addr` with the 32-bit word provided in
    // `o_req_wdata` (specified by the mask). This is necessarily synchronous,
    // but may either happen on the next clock edge (on a cache hit) or after
    // multiple cycles of latency (cache miss). As the cache is write-through
    // and write-allocate, writes must be applied to both the cache and
    // underlying memory.
    // It is illegal to assert this and `i_dmem_ren` on the same cycle.
    input  wire        i_req_wen,
    // The memory interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    input  wire [ 3:0] i_req_mask,
    // The 32-bit word to write to memory, if the request is a write
    // (i_req_wen is asserted). Only the bytes corresponding to set bits in
    // the mask should be written into the cache (and to backing memory).
    input  wire [31:0] i_req_wdata,
    // THe 32-bit data word read from memory on a read request.
    output wire [31:0] o_res_rdata
);
    // These parameters are equivalent to those provided in the project
    // 6 specification. Feel free to use them, but hardcoding these numbers
    // rather than using the localparams is also permitted, as long as the
    // same values are used (and consistent with the project specification).
    //
    // 32 sets * 2 ways per set * 16 bytes per way = 1K cache
    localparam O = 4;            // 4 bit offset => 16 byte cache line
    localparam S = 5;            // 5 bit set index => 32 sets
    localparam DEPTH = 2 ** S;   // 32 sets
    localparam W = 2;            // 2 way set associative, NMRU
    localparam T = 32 - O - S;   // 23 bit tag
    localparam D = 2 ** O / 4;   // 16 bytes per line / 4 bytes per word = 4 words per line

    // The following memory arrays model the cache structure. As this is
    // an internal implementation detail, you are *free* to modify these
    // arrays as you please.

    // Backing memory, modeled as two separate ways.
    reg [   31:0] datas0 [DEPTH - 1:0][D - 1:0];
    reg [   31:0] datas1 [DEPTH - 1:0][D - 1:0];
    reg [T - 1:0] tags0  [DEPTH - 1:0];
    reg [T - 1:0] tags1  [DEPTH - 1:0];
    reg [1:0]     valid  [DEPTH - 1:0];
    reg           lru    [DEPTH - 1:0];

    // Address decomposition
    wire [T - 1:0] req_tag;
    wire [S - 1:0] req_index;
    wire [O - 1:0] req_offset;
    wire [1:0] req_word_offset;

    assign req_tag = i_req_addr[31:O+S];
    assign req_index = i_req_addr[O+S-1:O];
    assign req_offset = i_req_addr[O-1:0];
    assign req_word_offset = i_req_addr[O-1:2];

    // Hit detection
    wire hit0, hit1, cache_hit;
    wire way_select;  // 0 for way0, 1 for way1

    assign hit0 = valid[req_index][0] && (tags0[req_index] == req_tag);
    assign hit1 = valid[req_index][1] && (tags1[req_index] == req_tag);
    assign cache_hit = hit0 || hit1;
    assign way_select = hit1;  // If both hit (shouldn't happen), prefer way1

    // State machine
    localparam STATE_IDLE = 2'd0;
    localparam STATE_READ_LINE = 2'd1;
    localparam STATE_WRITE_MEM = 2'd2;

    reg [1:0] state, next_state;
    reg [1:0] word_counter;
    reg write_way;  // Which way to write to on miss

    // Output busy signal
    reg busy_reg;
    assign o_busy = busy_reg;

    // Read data output
    reg [31:0] rdata_reg;
    assign o_res_rdata = rdata_reg;

    // Memory interface outputs
    reg mem_ren_reg, mem_wen_reg;
    reg [31:0] mem_addr_reg, mem_wdata_reg;

    assign o_mem_ren = mem_ren_reg;
    assign o_mem_wen = mem_wen_reg;
    assign o_mem_addr = mem_addr_reg;
    assign o_mem_wdata = mem_wdata_reg;

    // FSM state transitions
    always @(posedge i_clk) begin
        if (i_rst) begin
            state <= STATE_IDLE;
        end else begin
            state <= next_state;
        end
    end

    // Next state logic
    always @(*) begin
        next_state = state;
        case (state)
            STATE_IDLE: begin
                if ((i_req_ren || i_req_wen) && !cache_hit) begin
                    if (i_req_wen) begin
                        next_state = STATE_WRITE_MEM;
                    end else begin
                        next_state = STATE_READ_LINE;
                    end
                end
            end

            STATE_READ_LINE: begin
                if (i_mem_valid && word_counter == 2'd3) begin
                    next_state = STATE_IDLE;
                end
            end

            STATE_WRITE_MEM: begin
                if (i_mem_valid) begin
                    next_state = STATE_READ_LINE;
                end
            end
        endcase
    end

    // Busy signal generation
    always @(*) begin
        busy_reg = 1'b0;
        if (state != STATE_IDLE) begin
            busy_reg = 1'b1;
        end else if ((i_req_ren || i_req_wen) && !cache_hit) begin
            busy_reg = 1'b1;
        end
    end

    // Word counter for filling cache lines
    // MOVED LOGIC: We only increment if we successfully handled the step to stay in sync
    always @(posedge i_clk) begin
        if (i_rst) begin
            word_counter <= 2'd0;
        end else if (state == STATE_READ_LINE && i_mem_valid) begin
            word_counter <= word_counter + 1;
        end else if (state == STATE_IDLE) begin
            word_counter <= 2'd0;
        end
    end

    // Memory interface control
    always @(posedge i_clk) begin
        if (i_rst) begin
            mem_ren_reg <= 1'b0;
            mem_wen_reg <= 1'b0;
            mem_addr_reg <= 32'd0;
            mem_wdata_reg <= 32'd0;
        end else begin
            mem_ren_reg <= 1'b0;
            mem_wen_reg <= 1'b0;

            case (state)
                STATE_IDLE: begin
                    if ((i_req_ren || i_req_wen) && !cache_hit) begin
                        if (i_req_wen) begin
                            // Write-allocate: write to memory first
                            if (i_mem_ready) begin
                                mem_wen_reg <= 1'b1;
                                mem_addr_reg <= i_req_addr;
                                mem_wdata_reg <= i_req_wdata;
                            end
                        end else begin
                            // Read miss: start reading cache line (Request Word 0)
                            if (i_mem_ready) begin
                                mem_ren_reg <= 1'b1;
                                mem_addr_reg <= {i_req_addr[31:O], {O{1'b0}}};
                            end
                        end
                    end else if (cache_hit && i_req_wen) begin
                        // Write-through on cache hit
                        if (i_mem_ready) begin
                            mem_wen_reg <= 1'b1;
                            mem_addr_reg <= i_req_addr;
                            mem_wdata_reg <= i_req_wdata;
                        end
                    end
                end

                STATE_READ_LINE: begin
                    // We have received data (valid is high), so the counter is about to increment.
                    // We must fire the request for the NEXT word (counter + 1).
                    // We request words 1, 2, and 3 here.
                    if (i_mem_ready && i_mem_valid && word_counter < 2'd3) begin
                        mem_ren_reg <= 1'b1;
                        // Use the current address tag, but update the word offset
                        mem_addr_reg <= {mem_addr_reg[31:O], word_counter + 1'b1, 2'b00};
                    end 
                end

                STATE_WRITE_MEM: begin
                    if (i_mem_valid) begin
                        // Write is done. 
                        // The state machine will transition to READ_LINE automatically.
                        // We must trigger the Read for Word 0 immediately here.
                        if (i_mem_ready) begin
                            mem_ren_reg <= 1'b1; 
                            mem_addr_reg <= {i_req_addr[31:O], {O{1'b0}}};
                        end
                    end
                end
            endcase
        end
    end

    // Determine which way to replace on miss (NMRU - Not Most Recently Used)
    always @(*) begin
        if (!valid[req_index][0]) begin
            write_way = 1'b0;  // Way 0 is invalid, use it
        end else if (!valid[req_index][1]) begin
            write_way = 1'b1;  // Way 1 is invalid, use it
        end else begin
            write_way = ~lru[req_index];  // Replace the not-most-recently-used way
        end
    end

    // Cache data update
    integer i;
    always @(posedge i_clk) begin
        if (i_rst) begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                valid[i] <= 2'b00;
                lru[i] <= 1'b0;
            end
        end else begin
            // Handle cache hits - update LRU
            if (state == STATE_IDLE && cache_hit && (i_req_ren || i_req_wen)) begin
                lru[req_index] <= way_select;

                // Handle writes on cache hit (write-through)
                if (i_req_wen) begin
                    if (hit0) begin
                        // Write to way 0 with masking
                        if (i_req_mask[0]) datas0[req_index][req_word_offset][7:0] <= i_req_wdata[7:0];
                        if (i_req_mask[1]) datas0[req_index][req_word_offset][15:8] <= i_req_wdata[15:8];
                        if (i_req_mask[2]) datas0[req_index][req_word_offset][23:16] <= i_req_wdata[23:16];
                        if (i_req_mask[3]) datas0[req_index][req_word_offset][31:24] <= i_req_wdata[31:24];
                    end else begin
                        // Write to way 1 with masking
                        if (i_req_mask[0]) datas1[req_index][req_word_offset][7:0] <= i_req_wdata[7:0];
                        if (i_req_mask[1]) datas1[req_index][req_word_offset][15:8] <= i_req_wdata[15:8];
                        if (i_req_mask[2]) datas1[req_index][req_word_offset][23:16] <= i_req_wdata[23:16];
                        if (i_req_mask[3]) datas1[req_index][req_word_offset][31:24] <= i_req_wdata[31:24];
                    end
                end
            end

            // Handle cache line fills on miss
            if (state == STATE_READ_LINE && i_mem_valid) begin
                if (write_way == 1'b0) begin
                    datas0[req_index][word_counter] <= i_mem_rdata;
                    if (word_counter == 2'd3) begin
                        tags0[req_index] <= req_tag;
                        valid[req_index][0] <= 1'b1;
                        lru[req_index] <= 1'b0;
                    end
                end else begin
                    datas1[req_index][word_counter] <= i_mem_rdata;
                    if (word_counter == 2'd3) begin
                        tags1[req_index] <= req_tag;
                        valid[req_index][1] <= 1'b1;
                        lru[req_index] <= 1'b1;
                    end
                end
            end
        end
    end

    // Read data output (combinational for cache hits)
    always @(*) begin
        if (cache_hit) begin
            if (hit0) begin
                rdata_reg = datas0[req_index][req_word_offset];
            end else begin
                rdata_reg = datas1[req_index][req_word_offset];
            end
        end else if (state == STATE_READ_LINE && i_mem_valid && word_counter == 2'd3) begin
            // Return data at the end of cache fill
            if (write_way == 1'b0) begin
                rdata_reg = datas0[req_index][req_word_offset];
            end else begin
                rdata_reg = datas1[req_index][req_word_offset];
            end
        end else begin
            rdata_reg = 32'hxxxxxxxx;
        end
    end

endmodule

`default_nettype wire
