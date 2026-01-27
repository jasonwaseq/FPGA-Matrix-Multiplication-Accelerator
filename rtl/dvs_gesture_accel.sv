// DVS Gesture Accelerator with Temporal Voxel Binning
// Implements the full processing pipeline:
//   1. Spatial compression: 320x320 -> 16x16 grid (20x20 pooling)
//   2. Temporal binning: 400µs window divided into 8 bins (50µs each)
//   3. Double ring buffer: one accumulating, one for classification
//   4. Voxel counters: 16x16 spatial × 8 temporal × 2 polarity
//   5. Matrix multiplication classifier for UP, DOWN, LEFT, RIGHT
//
// Memory organization:
//   - Two ring buffers (A and B), each with 8 temporal bins
//   - Each bin: 16×16×2 = 512 counters (4-bit each)
//   - Total: 2 × 8 × 512 × 4 = 32,768 bits = 4KB
//
// Timing:
//   - 50µs per bin at 12MHz = 600 clock cycles per bin
//   - 400µs full window = 4800 cycles
//   - Ring swap every 400µs

module dvs_gesture_accel #(
    parameter CLK_FREQ_HZ      = 12_000_000,  // 12 MHz clock
    parameter WINDOW_US        = 400,          // Total window in microseconds
    parameter NUM_BINS         = 8,            // Temporal bins per window
    parameter GRID_SIZE        = 16,           // Compressed spatial resolution
    parameter SENSOR_RES       = 320,          // DVS sensor resolution
    parameter COUNTER_BITS     = 4,            // Bits per voxel counter (max 15 events)
    parameter FIFO_DEPTH       = 8             // Async input FIFO depth
)(
    input  logic clk,
    input  logic rst,
    
    // DVS Event Input (with flow control)
    input  logic event_valid,
    input  logic [8:0] event_x,        // 0-319
    input  logic [8:0] event_y,        // 0-319
    input  logic event_polarity,       // 1=ON, 0=OFF
    input  logic [15:0] event_ts,      // Timestamp in microseconds (unused - internal timing)
    output logic event_ready,          // Backpressure signal
    
    // Gesture Output
    output logic [1:0] gesture,        // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    output logic gesture_valid,        // Single cycle pulse
    
    // Debug outputs
    output logic ring_swap,            // Pulse on ring buffer swap
    output logic [2:0] current_bin     // Current active bin (0-7)
);

    // =========================================================================
    // Local Parameters
    // =========================================================================
    
    // Timing: Calculate cycles per bin
    // bin_period_us = WINDOW_US / NUM_BINS = 400 / 8 = 50µs
    // cycles_per_bin = CLK_FREQ_HZ * bin_period_us / 1_000_000
    localparam CYCLES_PER_BIN = (CLK_FREQ_HZ / 1_000_000) * (WINDOW_US / NUM_BINS);
    localparam BIN_TIMER_BITS = $clog2(CYCLES_PER_BIN + 1);
    
    // Spatial compression factor
    localparam POOL_SIZE = SENSOR_RES / GRID_SIZE;  // 320/16 = 20
    
    // Memory addressing
    localparam GRID_BITS = $clog2(GRID_SIZE);       // 4 bits for 0-15
    localparam BIN_BITS = $clog2(NUM_BINS);         // 3 bits for 0-7
    
    // =========================================================================
    // Input FIFO (Small sync buffer for burst handling)
    // =========================================================================
    
    // FIFO entry: x[8:0], y[8:0], polarity[0] = 19 bits
    localparam FIFO_WIDTH = 19;
    localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);
    
    logic [FIFO_WIDTH-1:0] fifo_mem [0:FIFO_DEPTH-1];
    logic [FIFO_PTR_BITS:0] fifo_wr_ptr, fifo_rd_ptr;
    logic fifo_empty, fifo_full;
    
    logic [8:0] fifo_out_x, fifo_out_y;
    logic fifo_out_pol;
    logic fifo_rd_en;
    
    // FIFO status
    assign fifo_empty = (fifo_wr_ptr == fifo_rd_ptr);
    assign fifo_full = (fifo_wr_ptr[FIFO_PTR_BITS] != fifo_rd_ptr[FIFO_PTR_BITS]) &&
                       (fifo_wr_ptr[FIFO_PTR_BITS-1:0] == fifo_rd_ptr[FIFO_PTR_BITS-1:0]);
    assign event_ready = ~fifo_full;
    
    // FIFO write
    always_ff @(posedge clk) begin
        if (rst) begin
            fifo_wr_ptr <= '0;
        end else if (event_valid && ~fifo_full) begin
            fifo_mem[fifo_wr_ptr[FIFO_PTR_BITS-1:0]] <= {event_x, event_y, event_polarity};
            fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
        end
    end
    
    // FIFO read data
    wire [FIFO_WIDTH-1:0] fifo_rd_data = fifo_mem[fifo_rd_ptr[FIFO_PTR_BITS-1:0]];
    assign fifo_out_x = fifo_rd_data[18:10];
    assign fifo_out_y = fifo_rd_data[9:1];
    assign fifo_out_pol = fifo_rd_data[0];
    
    always_ff @(posedge clk) begin
        if (rst) begin
            fifo_rd_ptr <= '0;
        end else if (fifo_rd_en && ~fifo_empty) begin
            fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
        end
    end
    
    // =========================================================================
    // Temporal Bin Control & Double Ring Buffer Management
    // =========================================================================
    
    logic [BIN_TIMER_BITS-1:0] bin_timer;
    logic [BIN_BITS-1:0] active_bin;
    logic active_ring;  // 0 = ring A is accumulating, 1 = ring B is accumulating
    logic bin_advance;  // Pulse when moving to next bin
    logic [BIN_BITS-1:0] bins_since_swap;
    logic ring_swap_r;
    
    assign current_bin = active_bin;
    assign ring_swap = ring_swap_r;
    
    // Bin timer and advancement
    always_ff @(posedge clk) begin
        if (rst) begin
            bin_timer <= '0;
            active_bin <= '0;
            active_ring <= 1'b0;
            bin_advance <= 1'b0;
            bins_since_swap <= '0;
            ring_swap_r <= 1'b0;
        end else begin
            bin_advance <= 1'b0;
            ring_swap_r <= 1'b0;
            
            if (bin_timer >= CYCLES_PER_BIN - 1) begin
                bin_timer <= '0;
                bin_advance <= 1'b1;
                bins_since_swap <= bins_since_swap + 1'b1;
                
                // After 8 bins (full window), swap rings
                if (bins_since_swap == NUM_BINS - 1) begin
                    active_ring <= ~active_ring;
                    bins_since_swap <= '0;
                    ring_swap_r <= 1'b1;
                    active_bin <= '0;
                end else begin
                    active_bin <= active_bin + 1'b1;
                end
            end else begin
                bin_timer <= bin_timer + 1'b1;
            end
        end
    end
    
    // =========================================================================
    // Voxel Memory (Double-buffered Ring Buffers)
    // =========================================================================
    // 
    // Address: {ring, bin[2:0], y[3:0], x[3:0], pol} = 1+3+4+4+1 = 13 bits
    // Total entries: 2^13 = 8192, each 4 bits = 32Kbit
    
    localparam VOXEL_ADDR_BITS = 1 + BIN_BITS + GRID_BITS + GRID_BITS + 1;
    localparam VOXEL_MEM_SIZE = 1 << VOXEL_ADDR_BITS;
    
    logic [COUNTER_BITS-1:0] voxel_mem [0:VOXEL_MEM_SIZE-1];
    
    // Memory initialization
    integer init_i;
    initial begin
        for (init_i = 0; init_i < VOXEL_MEM_SIZE; init_i = init_i + 1) begin
            voxel_mem[init_i] = '0;
        end
    end
    
    // Address generation function
    function automatic [VOXEL_ADDR_BITS-1:0] voxel_addr;
        input logic ring;
        input logic [BIN_BITS-1:0] bin;
        input logic [GRID_BITS-1:0] y;
        input logic [GRID_BITS-1:0] x;
        input logic pol;
        begin
            voxel_addr = {ring, bin, y, x, pol};
        end
    endfunction
    
    // =========================================================================
    // Event Processing Pipeline
    // =========================================================================
    
    typedef enum logic [2:0] {
        EVT_IDLE,
        EVT_FETCH,
        EVT_COMPRESS,
        EVT_READ_VOXEL,
        EVT_WRITE_VOXEL,
        EVT_CLEAR_BIN
    } evt_state_t;
    
    evt_state_t evt_state;
    
    // Pipeline registers
    logic [8:0] raw_x, raw_y;
    logic raw_pol;
    logic [GRID_BITS-1:0] comp_x, comp_y;
    logic [VOXEL_ADDR_BITS-1:0] pipe_addr;
    logic [COUNTER_BITS-1:0] pipe_counter;
    
    // Bin clearing state
    logic [VOXEL_ADDR_BITS-1:0] clear_addr;
    logic clearing_bin;
    logic [BIN_BITS-1:0] next_bin;
    logic need_clear;
    
    // Spatial compression using multiplication: (x * 13) >> 8 ≈ x/20
    wire [13:0] mult_x = fifo_out_x * 5'd13;
    wire [13:0] mult_y = fifo_out_y * 5'd13;
    wire [GRID_BITS-1:0] compressed_x = mult_x[12:8];
    wire [GRID_BITS-1:0] compressed_y = mult_y[12:8];
    
    // Clamp to valid range
    wire [GRID_BITS-1:0] clamped_x = (compressed_x >= GRID_SIZE) ? GRID_SIZE[GRID_BITS-1:0] - 1'b1 : compressed_x;
    wire [GRID_BITS-1:0] clamped_y = (compressed_y >= GRID_SIZE) ? GRID_SIZE[GRID_BITS-1:0] - 1'b1 : compressed_y;
    
    // Track pending bin clear
    always_ff @(posedge clk) begin
        if (rst) begin
            need_clear <= 1'b0;
            next_bin <= '0;
        end else if (bin_advance && !ring_swap_r) begin
            need_clear <= 1'b1;
            next_bin <= active_bin + 1'b1;
        end else if (evt_state == EVT_CLEAR_BIN && !clearing_bin) begin
            need_clear <= 1'b0;
        end
    end
    
    // FIFO read control
    assign fifo_rd_en = (evt_state == EVT_FETCH) && !fifo_empty && !need_clear && !clearing_bin;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            evt_state <= EVT_IDLE;
            raw_x <= '0;
            raw_y <= '0;
            raw_pol <= 1'b0;
            comp_x <= '0;
            comp_y <= '0;
            pipe_addr <= '0;
            pipe_counter <= '0;
            clearing_bin <= 1'b0;
            clear_addr <= '0;
        end else begin
            case (evt_state)
                EVT_IDLE: begin
                    if (need_clear) begin
                        // Start clearing the next bin in the active ring
                        clearing_bin <= 1'b1;
                        clear_addr <= voxel_addr(active_ring, next_bin, 4'd0, 4'd0, 1'b0);
                        evt_state <= EVT_CLEAR_BIN;
                    end else if (!fifo_empty) begin
                        evt_state <= EVT_FETCH;
                    end
                end
                
                EVT_FETCH: begin
                    if (!fifo_empty && !need_clear) begin
                        // Latch raw coordinates from FIFO
                        raw_x <= fifo_out_x;
                        raw_y <= fifo_out_y;
                        raw_pol <= fifo_out_pol;
                        evt_state <= EVT_COMPRESS;
                    end else begin
                        evt_state <= EVT_IDLE;
                    end
                end
                
                EVT_COMPRESS: begin
                    // Apply spatial compression
                    comp_x <= clamped_x;
                    comp_y <= clamped_y;
                    evt_state <= EVT_READ_VOXEL;
                end
                
                EVT_READ_VOXEL: begin
                    // Generate address and read counter
                    pipe_addr <= voxel_addr(active_ring, active_bin, comp_y, comp_x, raw_pol);
                    pipe_counter <= voxel_mem[voxel_addr(active_ring, active_bin, comp_y, comp_x, raw_pol)];
                    evt_state <= EVT_WRITE_VOXEL;
                end
                
                EVT_WRITE_VOXEL: begin
                    // Increment and write back (saturating add)
                    if (pipe_counter < {COUNTER_BITS{1'b1}}) begin
                        voxel_mem[pipe_addr] <= pipe_counter + 1'b1;
                    end
                    evt_state <= EVT_IDLE;
                end
                
                EVT_CLEAR_BIN: begin
                    // Clear one location per cycle
                    voxel_mem[clear_addr] <= '0;
                    
                    // Check if done clearing this bin (512 locations = 16×16×2)
                    // Low 9 bits = {y[3:0], x[3:0], pol} = 9 bits = 512 locations
                    if (clear_addr[GRID_BITS+GRID_BITS:0] == {(2*GRID_BITS+1){1'b1}}) begin
                        clearing_bin <= 1'b0;
                        evt_state <= EVT_IDLE;
                    end else begin
                        clear_addr[GRID_BITS+GRID_BITS:0] <= clear_addr[GRID_BITS+GRID_BITS:0] + 1'b1;
                    end
                end
                
                default: evt_state <= EVT_IDLE;
            endcase
        end
    end
    
    // =========================================================================
    // Feature Extraction & Classification (runs on ring swap)
    // =========================================================================
    //
    // When rings swap, the read-only ring has stable 8 bins of data.
    // We compute motion by comparing centroids of early vs late bins.
    
    typedef enum logic [2:0] {
        CLS_IDLE,
        CLS_START,
        CLS_READ_VOXELS,
        CLS_COMPUTE,
        CLS_OUTPUT
    } cls_state_t;
    
    cls_state_t cls_state;
    
    // Classification memory read
    logic [GRID_BITS-1:0] cls_x, cls_y;
    logic [BIN_BITS-1:0] cls_bin;
    logic cls_pol;
    logic cls_ring;  // Ring to read (opposite of active_ring after swap)
    
    // Accumulators for centroid computation
    // Sum of (position * count) for early and late bins
    logic signed [23:0] early_sum_x, early_sum_y;
    logic signed [23:0] late_sum_x, late_sum_y;
    logic [19:0] early_count, late_count;
    
    // Final motion vectors
    logic signed [23:0] delta_x, delta_y;
    
    // Classification trigger (delayed ring swap)
    logic start_classification;
    logic [1:0] start_delay;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            start_classification <= 1'b0;
            start_delay <= '0;
        end else begin
            start_classification <= 1'b0;
            if (ring_swap_r) begin
                start_delay <= 2'd2;
            end else if (start_delay > 0) begin
                start_delay <= start_delay - 1'b1;
                if (start_delay == 2'd1) begin
                    start_classification <= 1'b1;
                end
            end
        end
    end
    
    // Read voxel for classification (from stable ring)
    logic [COUNTER_BITS-1:0] cls_cnt;
    always_ff @(posedge clk) begin
        cls_cnt <= voxel_mem[voxel_addr(cls_ring, cls_bin, cls_y, cls_x, cls_pol)];
    end
    
    // Intermediate calculation signals (moved outside always block for Icarus compatibility)
    logic signed [4:0] cls_cx, cls_cy;
    logic signed [8:0] cls_weighted_x, cls_weighted_y;
    logic signed [23:0] cls_abs_dx, cls_abs_dy;
    
    // Combinational calculations
    always_comb begin
        cls_cx = $signed({1'b0, cls_x}) - $signed(5'd8);
        cls_cy = $signed({1'b0, cls_y}) - $signed(5'd8);
        cls_weighted_x = cls_cx * $signed({1'b0, cls_cnt});
        cls_weighted_y = cls_cy * $signed({1'b0, cls_cnt});
        cls_abs_dx = (delta_x < 0) ? -delta_x : delta_x;
        cls_abs_dy = (delta_y < 0) ? -delta_y : delta_y;
    end
    
    // Classification state machine
    always_ff @(posedge clk) begin
        if (rst) begin
            cls_state <= CLS_IDLE;
            cls_x <= '0;
            cls_y <= '0;
            cls_bin <= '0;
            cls_pol <= 1'b0;
            cls_ring <= 1'b0;
            early_sum_x <= '0;
            early_sum_y <= '0;
            late_sum_x <= '0;
            late_sum_y <= '0;
            early_count <= '0;
            late_count <= '0;
            delta_x <= '0;
            delta_y <= '0;
            gesture <= 2'b00;
            gesture_valid <= 1'b0;
        end else begin
            gesture_valid <= 1'b0;
            
            case (cls_state)
                CLS_IDLE: begin
                    if (start_classification) begin
                        // Start reading from the ring that just became stable
                        cls_ring <= ~active_ring;
                        cls_x <= '0;
                        cls_y <= '0;
                        cls_bin <= '0;
                        cls_pol <= 1'b0;
                        early_sum_x <= '0;
                        early_sum_y <= '0;
                        late_sum_x <= '0;
                        late_sum_y <= '0;
                        early_count <= '0;
                        late_count <= '0;
                        cls_state <= CLS_START;
                    end
                end
                
                CLS_START: begin
                    // First cycle: set up read address
                    cls_state <= CLS_READ_VOXELS;
                end
                
                CLS_READ_VOXELS: begin
                    // Accumulate using cls_cnt (available this cycle)
                    // Only use ON events (polarity=1) for motion estimation
                    if (cls_cnt > 0 && cls_pol == 1'b1) begin
                        if (cls_bin < NUM_BINS/2) begin
                            // Early bins (0-3)
                            early_sum_x <= early_sum_x + {{15{cls_weighted_x[8]}}, cls_weighted_x};
                            early_sum_y <= early_sum_y + {{15{cls_weighted_y[8]}}, cls_weighted_y};
                            early_count <= early_count + {16'b0, cls_cnt};
                        end else begin
                            // Late bins (4-7)
                            late_sum_x <= late_sum_x + {{15{cls_weighted_x[8]}}, cls_weighted_x};
                            late_sum_y <= late_sum_y + {{15{cls_weighted_y[8]}}, cls_weighted_y};
                            late_count <= late_count + {16'b0, cls_cnt};
                        end
                    end
                    
                    // Advance through all voxels: pol -> x -> y -> bin
                    if (cls_pol == 1'b0) begin
                        cls_pol <= 1'b1;
                    end else begin
                        cls_pol <= 1'b0;
                        if (cls_x == GRID_SIZE - 1) begin
                            cls_x <= '0;
                            if (cls_y == GRID_SIZE - 1) begin
                                cls_y <= '0;
                                if (cls_bin == NUM_BINS - 1) begin
                                    // Done reading all voxels
                                    cls_state <= CLS_COMPUTE;
                                end else begin
                                    cls_bin <= cls_bin + 1'b1;
                                end
                            end else begin
                                cls_y <= cls_y + 1'b1;
                            end
                        end else begin
                            cls_x <= cls_x + 1'b1;
                        end
                    end
                end
                
                CLS_COMPUTE: begin
                    // Motion = late_centroid - early_centroid
                    // We use sum difference (unnormalized) as it preserves sign and magnitude
                    delta_x <= late_sum_x - early_sum_x;
                    delta_y <= late_sum_y - early_sum_y;
                    cls_state <= CLS_OUTPUT;
                end
                
                CLS_OUTPUT: begin
                    // Classify based on dominant motion direction
                    // Equivalent to matrix multiply with:
                    //   UP:    [0, +1] · [dx, dy] = dy
                    //   DOWN:  [0, -1] · [dx, dy] = -dy
                    //   LEFT:  [-1, 0] · [dx, dy] = -dx
                    //   RIGHT: [+1, 0] · [dx, dy] = dx
                    
                    // Only output if there's meaningful activity
                    if (early_count > 4 || late_count > 4) begin
                        if (cls_abs_dy > cls_abs_dx) begin
                            // Vertical motion dominant
                            if (delta_y > 0)
                                gesture <= 2'b00;  // UP (increasing Y = moving down in image coords)
                            else
                                gesture <= 2'b01;  // DOWN
                        end else if (cls_abs_dx > 0) begin
                            // Horizontal motion dominant
                            if (delta_x > 0)
                                gesture <= 2'b11;  // RIGHT
                            else
                                gesture <= 2'b10;  // LEFT
                        end
                        gesture_valid <= 1'b1;
                    end
                    
                    cls_state <= CLS_IDLE;
                end
                
                default: cls_state <= CLS_IDLE;
            endcase
        end
    end

endmodule
