// Weight ROM for gesture classification
module matmul_weights
  #(parameter int WIDTH_P = 8
   ,parameter int HEIGHT_P = 8
   ,parameter int COUNTER_WIDTH_P = 8)
  (input  logic [1:0] gesture_i
  ,input  logic [5:0] pixel_addr_i
  ,output logic signed [COUNTER_WIDTH_P-1:0] weight_o);

  logic [2:0] y;
  logic [2:0] x;
  assign y = pixel_addr_i[5:3];  // Upper 3 bits (row)
  assign x = pixel_addr_i[2:0];  // Lower 3 bits (col)

  always_comb begin
    case (gesture_i)
      2'd0: weight_o = (y < 3'd4) ? 8'sd2 : -8'sd2;  // UP
      2'd1: weight_o = (y >= 3'd4) ? 8'sd2 : -8'sd2; // DOWN
      2'd2: weight_o = (x < 3'd4) ? 8'sd2 : -8'sd2;  // LEFT
      2'd3: weight_o = (x >= 3'd4) ? 8'sd2 : -8'sd2; // RIGHT
    endcase
  end

endmodule
