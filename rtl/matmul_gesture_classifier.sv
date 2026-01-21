`timescale 1ns/1ps
// Matrix multiplication based gesture classifier
// Uses learned weights to classify activity map into 4 gestures
module matmul_gesture_classifier
  #(parameter int WIDTH_P = 8
   ,parameter int HEIGHT_P = 8
   ,parameter int COUNTER_WIDTH_P = 8
   ,parameter int NUM_GESTURES_P = 4
   ,parameter int ACC_WIDTH_P = 32)
  (input  logic clk_i
  ,input  logic reset_i

  // Trigger classification
  ,input  logic classify_trigger_i
  ,output logic classify_busy_o

  // Read interface to activity map
  ,output logic map_read_valid_o
  ,output logic [$clog2(WIDTH_P*HEIGHT_P)-1:0] map_read_addr_o
  ,input  logic signed [COUNTER_WIDTH_P-1:0] map_read_data_i

  // Gesture output
  ,output logic gesture_valid_o
  ,output logic [1:0] gesture_idx_o  // 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
  ,output logic signed [ACC_WIDTH_P-1:0] confidence_score_o);

  localparam int MAP_SIZE = WIDTH_P * HEIGHT_P;


  typedef enum logic [2:0] {S_IDLE, S_COMPUTE, S_DECIDE} state_e;
  state_e state, state_n;

  logic [$clog2(MAP_SIZE):0] pixel_idx, pixel_idx_n;
  logic [1:0] gesture_idx, gesture_idx_n;
  logic signed [ACC_WIDTH_P-1:0] scores [NUM_GESTURES_P];
  logic signed [ACC_WIDTH_P-1:0] scores_n [NUM_GESTURES_P];

  // Matmul cores for each gesture
  logic [NUM_GESTURES_P-1:0] matmul_valid;
  logic [NUM_GESTURES_P-1:0] matmul_acc_clear;
  logic signed [ACC_WIDTH_P-1:0] matmul_results [NUM_GESTURES_P];
  logic signed [COUNTER_WIDTH_P-1:0] weight_values [NUM_GESTURES_P];

  genvar g;
  generate
    for (g = 0; g < NUM_GESTURES_P; g++) begin : gen_matmul
      matmul_weights #(
        .WIDTH_P(WIDTH_P),
        .HEIGHT_P(HEIGHT_P),
        .COUNTER_WIDTH_P(COUNTER_WIDTH_P)
      ) weight_rom (
        .gesture_i(g[1:0]),
        .pixel_addr_i(pixel_idx[$clog2(MAP_SIZE)-1:0]),
        .weight_o(weight_values[g])
      );

      matmul_core #(
        .WIDTH_P(COUNTER_WIDTH_P),
        .MULT_WIDTH_P(COUNTER_WIDTH_P * 2),
        .ACC_WIDTH_P(ACC_WIDTH_P)
      ) matmul_inst (
        .clk_i(clk_i),
        .reset_i(reset_i),
        .valid_i(matmul_valid[g]),
        .a_i(map_read_data_i),
        .b_i(weight_values[g]),
        .ready_o(),
        .acc_clear_i(matmul_acc_clear[g]),
        .valid_o(),
        .result_o(matmul_results[g])
      );
    end
  endgenerate

  always_ff @(posedge clk_i) begin
    if (reset_i) begin
      state <= S_IDLE;
      pixel_idx <= '0;
      gesture_idx <= '0;
      for (int i = 0; i < NUM_GESTURES_P; i++) scores[i] <= '0;
    end else begin
      state <= state_n;
      pixel_idx <= pixel_idx_n;
      gesture_idx <= gesture_idx_n;
      for (int i = 0; i < NUM_GESTURES_P; i++) scores[i] <= scores_n[i];
    end
  end

  logic signed [ACC_WIDTH_P-1:0] max_score;
  logic [1:0] max_idx;

  always_comb begin
    state_n = state;
    pixel_idx_n = pixel_idx;
    gesture_idx_n = gesture_idx;
    for (int i = 0; i < NUM_GESTURES_P; i++) scores_n[i] = scores[i];

    classify_busy_o = 1'b1;
    map_read_valid_o = 1'b0;
    map_read_addr_o = '0;
    gesture_valid_o = 1'b0;
    gesture_idx_o = '0;
    confidence_score_o = '0;

    for (int i = 0; i < NUM_GESTURES_P; i++) begin
      matmul_valid[i] = 1'b0;
      matmul_acc_clear[i] = 1'b0;
    end

    unique case (state)
      S_IDLE: begin
        classify_busy_o = 1'b0;
        if (classify_trigger_i) begin
          state_n = S_COMPUTE;
          pixel_idx_n = '0;
          // Clear accumulators
          for (int i = 0; i < NUM_GESTURES_P; i++) matmul_acc_clear[i] = 1'b1;
        end
      end

      S_COMPUTE: begin
        if (pixel_idx < MAP_SIZE) begin
          map_read_valid_o = 1'b1;
          map_read_addr_o = pixel_idx[$clog2(MAP_SIZE)-1:0];
          
          // Feed to all matmul cores
          for (int i = 0; i < NUM_GESTURES_P; i++) begin
            matmul_valid[i] = 1'b1;
          end

          pixel_idx_n = pixel_idx + 1;
        end else begin
          // Capture final scores
          for (int i = 0; i < NUM_GESTURES_P; i++) begin
            scores_n[i] = matmul_results[i];
          end
          state_n = S_DECIDE;
        end
      end

      S_DECIDE: begin
        gesture_valid_o = 1'b1;
        
        // Find maximum score
        max_score = scores[0];
        max_idx = 2'd0;
        
        for (int i = 1; i < NUM_GESTURES_P; i++) begin
          if (scores[i] > max_score) begin
            max_score = scores[i];
            max_idx = i[1:0];
          end
        end

        gesture_idx_o = max_idx;
        confidence_score_o = max_score;
        state_n = S_IDLE;
      end
    endcase
  end

endmodule
