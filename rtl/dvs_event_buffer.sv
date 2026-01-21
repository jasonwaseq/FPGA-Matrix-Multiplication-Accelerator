`timescale 1ns/1ps
module dvs_event_buffer
  #(parameter int WIDTH_P = 8
   ,parameter int HEIGHT_P = 8
   ,parameter int DEPTH_LOG2_P = 8)
  (input  logic clk_i
  ,input  logic reset_i

  ,input  logic valid_i
  ,input  logic [$clog2(WIDTH_P)-1:0] x_i
  ,input  logic [$clog2(HEIGHT_P)-1:0] y_i
  ,input  logic polarity_i
  ,input  logic [15:0] timestamp_i
  ,output logic ready_o

  ,output logic valid_o
  ,output logic [$clog2(WIDTH_P)-1:0] x_o
  ,output logic [$clog2(HEIGHT_P)-1:0] y_o
  ,output logic polarity_o
  ,output logic [15:0] timestamp_o
  ,input  logic ready_i);

  localparam int EVENT_WIDTH = $clog2(WIDTH_P) + $clog2(HEIGHT_P) + 1 + 16;
  localparam int X_WIDTH = $clog2(WIDTH_P);
  localparam int Y_WIDTH = $clog2(HEIGHT_P);

  logic [EVENT_WIDTH-1:0] data_packed_in;
  logic [EVENT_WIDTH-1:0] data_packed_out;

  // Pack input signals
  assign data_packed_in = {timestamp_i, polarity_i, y_i, x_i};

  // Unpack output signals
  assign x_o = data_packed_out[X_WIDTH-1:0];
  assign y_o = data_packed_out[X_WIDTH + Y_WIDTH - 1:X_WIDTH];
  assign polarity_o = data_packed_out[X_WIDTH + Y_WIDTH];
  assign timestamp_o = data_packed_out[EVENT_WIDTH-1:X_WIDTH + Y_WIDTH + 1];

  fifo_1r1w #(
    .width_p(EVENT_WIDTH),
    .depth_log2_p(DEPTH_LOG2_P)
  ) event_fifo (
    .clk_i(clk_i),
    .reset_i(reset_i),
    .data_i(data_packed_in),
    .valid_i(valid_i),
    .ready_o(ready_o),
    .valid_o(valid_o),
    .data_o(data_packed_out),
    .ready_i(ready_i)
  );

endmodule
