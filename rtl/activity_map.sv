`timescale 1ns/1ps
// 2D activity map with event-driven temporal decay
module activity_map
  #(parameter int WIDTH_P = 8
   ,parameter int HEIGHT_P = 8
   ,parameter int COUNTER_WIDTH_P = 8
   ,parameter int TIMESTAMP_WIDTH_P = 16
   ,parameter int DECAY_THRESHOLD_P = 1000)  // time units between decays
  (input  logic clk_i
  ,input  logic reset_i

  // Event input with timestamp
  ,input  logic event_valid_i
  ,input  logic [$clog2(WIDTH_P)-1:0] event_x_i
  ,input  logic [$clog2(HEIGHT_P)-1:0] event_y_i
  ,input  logic event_polarity_i  // 1=increment, 0=decrement
  ,input  logic [TIMESTAMP_WIDTH_P-1:0] event_timestamp_i
  ,output logic event_ready_o
  ,output logic decay_triggered_o  // Pulse when decay starts

  // Read interface for classifier
  ,input  logic read_valid_i
  ,input  logic [$clog2(WIDTH_P*HEIGHT_P)-1:0] read_addr_i
  ,output logic signed [COUNTER_WIDTH_P-1:0] read_data_o);

  localparam int MAP_SIZE = WIDTH_P * HEIGHT_P;

  // Single-port RAM for activity map
  logic mem_wr_valid;
  logic [$clog2(MAP_SIZE)-1:0] mem_addr;
  logic signed [COUNTER_WIDTH_P-1:0] mem_wr_data;
  logic signed [COUNTER_WIDTH_P-1:0] mem_rd_data;

  ram_1r1w_sync #(
    .width_p(COUNTER_WIDTH_P),
    .depth_p(MAP_SIZE),
    .filename_p("")
  ) activity_mem (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .wr_valid_i(mem_wr_valid),
    .wr_data_i(mem_wr_data),
    .wr_addr_i(mem_addr),
    .rd_valid_i(1'b1),
    .rd_addr_i(mem_addr),
    .rd_data_o(mem_rd_data)
  );

  typedef enum logic [1:0] {S_IDLE, S_UPDATE, S_DECAY} state_e;
  state_e state, state_n;

  logic [$clog2(MAP_SIZE):0] decay_idx, decay_idx_n;
  logic [TIMESTAMP_WIDTH_P-1:0] last_decay_timestamp, last_decay_timestamp_n;
  logic [$clog2(WIDTH_P)-1:0] event_x_reg;
  logic [$clog2(HEIGHT_P)-1:0] event_y_reg;
  logic event_polarity_reg;
  logic decay_needed;

  // Check if decay is needed based on timestamp
  logic [TIMESTAMP_WIDTH_P:0] time_elapsed;
  assign time_elapsed = event_timestamp_i - last_decay_timestamp;
  assign decay_needed = (time_elapsed >= DECAY_THRESHOLD_P);

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      state <= S_IDLE;
      decay_idx <= '0;
      last_decay_timestamp <= '0;
      event_x_reg <= '0;
      event_y_reg <= '0;
      event_polarity_reg <= '0;
      decay_triggered_o <= 1'b0;
    end else begin
      state <= state_n;
      decay_idx <= decay_idx_n;
      last_decay_timestamp <= last_decay_timestamp_n;
      if (event_valid_i && event_ready_o) begin
        event_x_reg <= event_x_i;
        event_y_reg <= event_y_i;
        event_polarity_reg <= event_polarity_i;
      end
      decay_triggered_o <= (state == S_IDLE && state_n == S_DECAY) ? 1'b1 : 1'b0;
    end
  end

  always_comb begin
    state_n = state;
    decay_idx_n = decay_idx;
    last_decay_timestamp_n = last_decay_timestamp;
    event_ready_o = 1'b0;
    mem_wr_valid = 1'b0;
    mem_addr = '0;
    mem_wr_data = '0;
    read_data_o = '0;

    unique case (state)
      S_IDLE: begin
        if (read_valid_i) begin
          // Read request from classifier (has priority)
          mem_addr = read_addr_i;
          read_data_o = mem_rd_data;
        end else if (event_valid_i && decay_needed) begin
          // Event arrived AND decay threshold exceeded
          // Start decay first, then process event after
          state_n = S_DECAY;
          decay_idx_n = '0;
          last_decay_timestamp_n = event_timestamp_i;
          mem_addr = '0;
        end else if (event_valid_i) begin
          // Accept event and update (no decay needed)
          event_ready_o = 1'b1;
          state_n = S_UPDATE;
          mem_addr = event_y_i * WIDTH_P + event_x_i;
        end
      end

      S_UPDATE: begin
        // Write updated value back
        mem_addr = event_y_reg * WIDTH_P + event_x_reg;
        mem_wr_valid = 1'b1;
        if (event_polarity_reg) begin
          // Increment (saturate at max)
          if (mem_rd_data < $signed({1'b0, {(COUNTER_WIDTH_P-1){1'b1}}}))
            mem_wr_data = mem_rd_data + 1;
          else
            mem_wr_data = mem_rd_data;
        end else begin
          // Decrement (saturate at min)
          if (mem_rd_data > $signed({1'b1, {(COUNTER_WIDTH_P-1){1'b0}}}))
            mem_wr_data = mem_rd_data - 1;
          else
            mem_wr_data = mem_rd_data;
        end
        state_n = S_IDLE;
      end

      S_DECAY: begin
        // Global decay: right-shift all values (divide by 2, moving toward zero)
        if (decay_idx < MAP_SIZE) begin
          mem_addr = decay_idx[$clog2(MAP_SIZE)-1:0];
          mem_wr_valid = 1'b1;
          mem_wr_data = mem_rd_data >>> 1;  // arithmetic shift
          decay_idx_n = decay_idx + 1;
        end else begin
          state_n = S_IDLE;
        end
      end
    endcase
  end

endmodule
