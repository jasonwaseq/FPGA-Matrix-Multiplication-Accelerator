`timescale 1ns/1ps
// Parameterized multiply-accumulate core for matrix multiplication
module matmul_core
  #(parameter int WIDTH_P = 8
   ,parameter int MULT_WIDTH_P = 16
   ,parameter int ACC_WIDTH_P = 32)
  (input  logic clk_i
  ,input  logic reset_i

  // Input operands
  ,input  logic valid_i
  ,input  logic signed [WIDTH_P-1:0] a_i
  ,input  logic signed [WIDTH_P-1:0] b_i
  ,output logic ready_o

  // Accumulator control
  ,input  logic acc_clear_i

  // Result output
  ,output logic valid_o
  ,output logic signed [ACC_WIDTH_P-1:0] result_o);

  logic signed [MULT_WIDTH_P-1:0] mult_result;
  logic signed [ACC_WIDTH_P-1:0] accumulator;

  assign mult_result = a_i * b_i;
  assign ready_o = 1'b1;  // Always ready for new inputs

  always_ff @(posedge clk_i) begin
    if (reset_i || acc_clear_i) begin
      accumulator <= '0;
      valid_o <= 1'b0;
    end else if (valid_i) begin
      accumulator <= accumulator + $signed(mult_result);
      valid_o <= 1'b1;
    end else begin
      valid_o <= 1'b0;
    end
  end

  assign result_o = accumulator;

endmodule
