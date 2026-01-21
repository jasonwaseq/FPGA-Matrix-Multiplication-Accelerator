`timescale 1ns/1ps
module dvs_gesture_detector_top
  #(parameter int WIDTH_P = 8
   ,parameter int HEIGHT_P = 8
   ,parameter int COUNTER_WIDTH_P = 8
   ,parameter int ACC_WIDTH_P = 16
   ,parameter int EVENT_BUFFER_DEPTH_LOG2 = 8
   ,parameter int CLASSIFY_INTERVAL_P = 1000)
  (input  logic clk_i
  ,input  logic reset_i

  ,input  logic dvs_event_valid_i
  ,input  logic [$clog2(WIDTH_P)-1:0] dvs_x_i
  ,input  logic [$clog2(HEIGHT_P)-1:0] dvs_y_i
  ,input  logic dvs_polarity_i
  ,input  logic [15:0] dvs_timestamp_i
  ,output logic dvs_ready_o

  ,output logic gesture_valid_o
  ,output logic [1:0] gesture_idx_o
  ,output logic signed [ACC_WIDTH_P-1:0] confidence_score_o);

  // Event buffer
  wire event_buf_valid, event_buf_ready;
  wire [$clog2(WIDTH_P)-1:0] event_buf_x;
  wire [$clog2(HEIGHT_P)-1:0] event_buf_y;
  wire event_buf_polarity;
  wire [15:0] event_buf_timestamp;

  dvs_event_buffer #(
    .WIDTH_P(WIDTH_P),
    .HEIGHT_P(HEIGHT_P),
    .DEPTH_LOG2_P(EVENT_BUFFER_DEPTH_LOG2)
  ) event_buffer (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .valid_i(dvs_event_valid_i),
    .x_i(dvs_x_i),
    .y_i(dvs_y_i),
    .polarity_i(dvs_polarity_i),
    .timestamp_i(dvs_timestamp_i),
    .ready_o(dvs_ready_o),
    .valid_o(event_buf_valid),
    .x_o(event_buf_x),
    .y_o(event_buf_y),
    .polarity_o(event_buf_polarity),
    .timestamp_o(event_buf_timestamp),
    .ready_i(event_buf_ready)
  );

  // Activity map with event-driven temporal decay
  wire map_event_ready;
  wire map_decay_triggered;
  wire map_read_valid;
  wire [$clog2(WIDTH_P*HEIGHT_P)-1:0] map_read_addr;
  wire signed [COUNTER_WIDTH_P-1:0] map_read_data;

  activity_map #(
    .WIDTH_P(WIDTH_P),
    .HEIGHT_P(HEIGHT_P),
    .COUNTER_WIDTH_P(COUNTER_WIDTH_P),
    .TIMESTAMP_WIDTH_P(16),
    .DECAY_THRESHOLD_P(CLASSIFY_INTERVAL_P)
  ) activity_map_inst (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .event_valid_i(event_buf_valid),
    .event_x_i(event_buf_x),
    .event_y_i(event_buf_y),
    .event_polarity_i(event_buf_polarity),
    .event_timestamp_i(event_buf_timestamp),
    .event_ready_o(map_event_ready),
    .decay_triggered_o(map_decay_triggered),
    .read_valid_i(map_read_valid),
    .read_addr_i(map_read_addr),
    .read_data_o(map_read_data)
  );

  assign event_buf_ready = map_event_ready;

  // Matmul-based Classifier
  matmul_gesture_classifier #(
    .WIDTH_P(WIDTH_P),
    .HEIGHT_P(HEIGHT_P),
    .COUNTER_WIDTH_P(COUNTER_WIDTH_P),
    .NUM_GESTURES_P(4),
    .ACC_WIDTH_P(ACC_WIDTH_P)
  ) classifier (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .classify_trigger_i(map_decay_triggered),
    .classify_busy_o(),
    .map_read_valid_o(map_read_valid),
    .map_read_addr_o(map_read_addr),
    .map_read_data_i(map_read_data),
    .gesture_valid_o(gesture_valid_o),
    .gesture_idx_o(gesture_idx_o),
    .confidence_score_o(confidence_score_o)
  );

endmodule
