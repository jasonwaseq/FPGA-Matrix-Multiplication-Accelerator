// DVS Gesture Accelerator - Lightweight Version for ice40 UP5K
//
// Architecture:
//   - Uses centroid accumulators instead of full voxel memory
//   - Tracks weighted X/Y positions for early and late time periods
//   - Minimal memory footprint (fits in LUTs, no BRAM needed)
//
// Gesture Detection:
//   - Accumulate event positions weighted by time period
//   - Compare early vs late centroids to detect motion direction
//   - Output gesture when bin timer completes a full window

module dvs_gesture_accel #(
    parameter CLK_FREQ_HZ    = 12_000_000,
    parameter CYCLES_PER_BIN = 600_000,     // Cycles per temporal bin (50ms default)
    parameter NUM_BINS       = 8,           // Number of temporal bins in window
    parameter GRID_SIZE      = 16,          // Spatial grid dimension
    parameter SENSOR_RES     = 320          // DVS sensor resolution
)(
    input  logic clk,
    input  logic rst,
    
    // DVS Event Input
    input  logic        event_valid,
    input  logic [8:0]  event_x,            // 0-319
    input  logic [8:0]  event_y,            // 0-319
    input  logic        event_polarity,     // 1=ON, 0=OFF
    input  logic [15:0] event_ts,           // Timestamp (unused)
    output logic        event_ready,        // Always ready
    
    // Gesture Output
    output logic [1:0]  gesture,            // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
    output logic        gesture_valid,      // Single cycle pulse
    
    // Debug
    output logic [2:0]  current_bin         // Current bin in window (0-7)
);

    // =========================================================================
    // Parameters
    // =========================================================================
    
    localparam BIN_TIMER_BITS = $clog2(CYCLES_PER_BIN + 1);
    localparam BIN_BITS = $clog2(NUM_BINS);
    localparam GRID_BITS = $clog2(GRID_SIZE);
    
    // =========================================================================
    // Spatial Compression: 320 -> 16 using (x * 13) >> 8
    // =========================================================================
    
    wire [12:0] mult_x = event_x * 5'd13;
    wire [12:0] mult_y = event_y * 5'd13;
    
    wire [GRID_BITS-1:0] grid_x = (mult_x[12:8] >= GRID_SIZE) ? 
                                   (GRID_SIZE - 1) : mult_x[11:8];
    wire [GRID_BITS-1:0] grid_y = (mult_y[12:8] >= GRID_SIZE) ? 
                                   (GRID_SIZE - 1) : mult_y[11:8];
    
    // =========================================================================
    // Bin Timer
    // =========================================================================
    
    logic [BIN_TIMER_BITS-1:0] bin_timer;
    logic [BIN_BITS-1:0]       bin_idx;
    logic                      bin_advance;
    logic                      window_complete;
    
    assign current_bin = bin_idx;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            bin_timer       <= '0;
            bin_idx         <= '0;
            bin_advance     <= 1'b0;
            window_complete <= 1'b0;
        end else begin
            bin_advance     <= 1'b0;
            window_complete <= 1'b0;
            
            if (bin_timer >= CYCLES_PER_BIN - 1) begin
                bin_timer   <= '0;
                bin_advance <= 1'b1;
                
                if (bin_idx == NUM_BINS - 1) begin
                    bin_idx         <= '0;
                    window_complete <= 1'b1;
                end else begin
                    bin_idx <= bin_idx + 1'b1;
                end
            end else begin
                bin_timer <= bin_timer + 1'b1;
            end
        end
    end
    
    // =========================================================================
    // Centroid Accumulators
    // =========================================================================
    //
    // Track sum of X, sum of Y, and count for early/late halves of window
    // Early = bins 0-3, Late = bins 4-7
    
    wire is_early_half = (bin_idx < NUM_BINS/2);
    
    // Signed position relative to center (center = 8 for 16x16 grid)
    wire signed [4:0] pos_x = $signed({1'b0, grid_x}) - $signed(5'd8);
    wire signed [4:0] pos_y = $signed({1'b0, grid_y}) - $signed(5'd8);
    
    // Accumulators (enough bits for ~1000 events)
    logic signed [15:0] early_sum_x, early_sum_y;
    logic signed [15:0] late_sum_x, late_sum_y;
    logic [11:0] early_count, late_count;
    
    // Always ready to accept events
    assign event_ready = 1'b1;
    
    always_ff @(posedge clk) begin
        if (rst) begin
            early_sum_x <= '0;
            early_sum_y <= '0;
            late_sum_x  <= '0;
            late_sum_y  <= '0;
            early_count <= '0;
            late_count  <= '0;
        end else begin
            // Reset accumulators on window complete
            if (window_complete) begin
                early_sum_x <= '0;
                early_sum_y <= '0;
                late_sum_x  <= '0;
                late_sum_y  <= '0;
                early_count <= '0;
                late_count  <= '0;
            end
            // Accumulate ON events (polarity = 1)
            else if (event_valid && event_polarity) begin
                if (is_early_half) begin
                    early_sum_x <= early_sum_x + {{11{pos_x[4]}}, pos_x};
                    early_sum_y <= early_sum_y + {{11{pos_y[4]}}, pos_y};
                    early_count <= early_count + 1'b1;
                end else begin
                    late_sum_x <= late_sum_x + {{11{pos_x[4]}}, pos_x};
                    late_sum_y <= late_sum_y + {{11{pos_y[4]}}, pos_y};
                    late_count <= late_count + 1'b1;
                end
            end
        end
    end
    
    // =========================================================================
    // Gesture Classification
    // =========================================================================
    //
    // Compare centroids: motion = late - early
    // Classify based on dominant axis
    
    logic signed [15:0] delta_x, delta_y;
    logic signed [15:0] abs_dx, abs_dy;
    
    // Compute deltas (use counts to normalize if needed)
    // Simple version: just use sums (assumes similar event rates)
    always_comb begin
        delta_x = late_sum_x - early_sum_x;
        delta_y = late_sum_y - early_sum_y;
        abs_dx = (delta_x < 0) ? -delta_x : delta_x;
        abs_dy = (delta_y < 0) ? -delta_y : delta_y;
    end
    
    // Output gesture on window complete
    always_ff @(posedge clk) begin
        if (rst) begin
            gesture       <= 2'b00;
            gesture_valid <= 1'b0;
        end else begin
            gesture_valid <= 1'b0;
            
            if (window_complete) begin
                // Require minimum activity
                if ((early_count > 5) || (late_count > 5)) begin
                    if (abs_dy > abs_dx) begin
                        // Vertical motion dominant
                        if (delta_y > 0)
                            gesture <= 2'b01;  // DOWN (positive delta_y = moved down)
                        else
                            gesture <= 2'b00;  // UP
                    end else if (abs_dx > 0) begin
                        // Horizontal motion dominant  
                        if (delta_x > 0)
                            gesture <= 2'b10;  // LEFT (positive delta_x = moved left)
                        else
                            gesture <= 2'b11;  // RIGHT
                    end
                    gesture_valid <= 1'b1;
                end
            end
        end
    end

endmodule
