module uart_rx (
	clk,
	rst,
	rx,
	data,
	valid
);
	parameter CLKS_PER_BIT = 104;
	input wire clk;
	input wire rst;
	input wire rx;
	output reg [7:0] data;
	output reg valid;
	localparam IDLE = 2'd0;
	localparam START = 2'd1;
	localparam DATA = 2'd2;
	localparam STOP = 2'd3;
	reg [1:0] state;
	reg [7:0] clk_cnt;
	reg [2:0] bit_idx;
	reg [7:0] rx_data;
	reg rx_sync;
	reg rx_d;
	always @(posedge clk)
		if (rst) begin
			rx_sync <= 1'b1;
			rx_d <= 1'b1;
		end
		else begin
			rx_sync <= rx;
			rx_d <= rx_sync;
		end
	always @(posedge clk)
		if (rst) begin
			state <= IDLE;
			clk_cnt <= 8'd0;
			bit_idx <= 3'd0;
			rx_data <= 8'd0;
			data <= 8'd0;
			valid <= 1'b0;
		end
		else begin
			valid <= 1'b0;
			case (state)
				IDLE: begin
					clk_cnt <= 8'd0;
					bit_idx <= 3'd0;
					if (rx_d == 1'b0)
						state <= START;
				end
				START:
					if (clk_cnt == ((CLKS_PER_BIT - 1) / 2)) begin
						if (rx_d == 1'b0) begin
							clk_cnt <= 8'd0;
							state <= DATA;
						end
						else
							state <= IDLE;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				DATA:
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						rx_data[bit_idx] <= rx_d;
						if (bit_idx == 3'd7) begin
							bit_idx <= 3'd0;
							state <= STOP;
						end
						else
							bit_idx <= bit_idx + 1'b1;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				STOP:
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						state <= IDLE;
						if (rx_d == 1'b1) begin
							data <= rx_data;
							valid <= 1'b1;
						end
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				default: state <= IDLE;
			endcase
		end
endmodule
module uart_tx (
	clk,
	rst,
	data,
	valid,
	tx,
	busy
);
	parameter CLKS_PER_BIT = 104;
	input wire clk;
	input wire rst;
	input wire [7:0] data;
	input wire valid;
	output reg tx;
	output reg busy;
	localparam IDLE = 2'd0;
	localparam START = 2'd1;
	localparam DATA = 2'd2;
	localparam STOP = 2'd3;
	reg [1:0] state;
	reg [7:0] clk_cnt;
	reg [2:0] bit_idx;
	reg [7:0] tx_data;
	always @(posedge clk)
		if (rst) begin
			state <= IDLE;
			clk_cnt <= 8'd0;
			bit_idx <= 3'd0;
			tx_data <= 8'd0;
			tx <= 1'b1;
			busy <= 1'b0;
		end
		else
			case (state)
				IDLE: begin
					tx <= 1'b1;
					clk_cnt <= 8'd0;
					bit_idx <= 3'd0;
					busy <= 1'b0;
					if (valid) begin
						tx_data <= data;
						busy <= 1'b1;
						state <= START;
					end
				end
				START: begin
					tx <= 1'b0;
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						state <= DATA;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				end
				DATA: begin
					tx <= tx_data[bit_idx];
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						if (bit_idx == 3'd7) begin
							bit_idx <= 3'd0;
							state <= STOP;
						end
						else
							bit_idx <= bit_idx + 1'b1;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				end
				STOP: begin
					tx <= 1'b1;
					if (clk_cnt == (CLKS_PER_BIT - 1)) begin
						clk_cnt <= 8'd0;
						state <= IDLE;
						busy <= 1'b0;
					end
					else
						clk_cnt <= clk_cnt + 1'b1;
				end
				default: state <= IDLE;
			endcase
endmodule
module dvs_gesture_accel (
	clk,
	rst,
	event_valid,
	event_x,
	event_y,
	event_polarity,
	event_ts,
	event_ready,
	gesture,
	gesture_valid,
	current_bin
);
	reg _sv2v_0;
	parameter CLK_FREQ_HZ = 12000000;
	parameter CYCLES_PER_BIN = 600000;
	parameter NUM_BINS = 8;
	parameter GRID_SIZE = 16;
	parameter SENSOR_RES = 320;
	input wire clk;
	input wire rst;
	input wire event_valid;
	input wire [8:0] event_x;
	input wire [8:0] event_y;
	input wire event_polarity;
	input wire [15:0] event_ts;
	output wire event_ready;
	output reg [1:0] gesture;
	output reg gesture_valid;
	output wire [2:0] current_bin;
	localparam BIN_TIMER_BITS = $clog2(CYCLES_PER_BIN + 1);
	localparam BIN_BITS = $clog2(NUM_BINS);
	localparam GRID_BITS = $clog2(GRID_SIZE);
	wire [12:0] mult_x = event_x * 5'd13;
	wire [12:0] mult_y = event_y * 5'd13;
	wire [GRID_BITS - 1:0] grid_x = (mult_x[12:8] >= GRID_SIZE ? GRID_SIZE - 1 : mult_x[11:8]);
	wire [GRID_BITS - 1:0] grid_y = (mult_y[12:8] >= GRID_SIZE ? GRID_SIZE - 1 : mult_y[11:8]);
	reg [BIN_TIMER_BITS - 1:0] bin_timer;
	reg [BIN_BITS - 1:0] bin_idx;
	reg bin_advance;
	reg window_complete;
	assign current_bin = bin_idx;
	always @(posedge clk)
		if (rst) begin
			bin_timer <= 1'sb0;
			bin_idx <= 1'sb0;
			bin_advance <= 1'b0;
			window_complete <= 1'b0;
		end
		else begin
			bin_advance <= 1'b0;
			window_complete <= 1'b0;
			if (bin_timer >= (CYCLES_PER_BIN - 1)) begin
				bin_timer <= 1'sb0;
				bin_advance <= 1'b1;
				if (bin_idx == (NUM_BINS - 1)) begin
					bin_idx <= 1'sb0;
					window_complete <= 1'b1;
				end
				else
					bin_idx <= bin_idx + 1'b1;
			end
			else
				bin_timer <= bin_timer + 1'b1;
		end
	wire is_early_half = bin_idx < (NUM_BINS / 2);
	wire signed [4:0] pos_x = $signed({1'b0, grid_x}) - $signed(5'd8);
	wire signed [4:0] pos_y = $signed({1'b0, grid_y}) - $signed(5'd8);
	reg signed [15:0] early_sum_x;
	reg signed [15:0] early_sum_y;
	reg signed [15:0] late_sum_x;
	reg signed [15:0] late_sum_y;
	reg [11:0] early_count;
	reg [11:0] late_count;
	assign event_ready = 1'b1;
	always @(posedge clk)
		if (rst) begin
			early_sum_x <= 1'sb0;
			early_sum_y <= 1'sb0;
			late_sum_x <= 1'sb0;
			late_sum_y <= 1'sb0;
			early_count <= 1'sb0;
			late_count <= 1'sb0;
		end
		else if (window_complete) begin
			early_sum_x <= 1'sb0;
			early_sum_y <= 1'sb0;
			late_sum_x <= 1'sb0;
			late_sum_y <= 1'sb0;
			early_count <= 1'sb0;
			late_count <= 1'sb0;
		end
		else if (event_valid && event_polarity) begin
			if (is_early_half) begin
				early_sum_x <= early_sum_x + {{11 {pos_x[4]}}, pos_x};
				early_sum_y <= early_sum_y + {{11 {pos_y[4]}}, pos_y};
				early_count <= early_count + 1'b1;
			end
			else begin
				late_sum_x <= late_sum_x + {{11 {pos_x[4]}}, pos_x};
				late_sum_y <= late_sum_y + {{11 {pos_y[4]}}, pos_y};
				late_count <= late_count + 1'b1;
			end
		end
	reg signed [15:0] delta_x;
	reg signed [15:0] delta_y;
	reg signed [15:0] abs_dx;
	reg signed [15:0] abs_dy;
	always @(*) begin
		if (_sv2v_0)
			;
		delta_x = late_sum_x - early_sum_x;
		delta_y = late_sum_y - early_sum_y;
		abs_dx = (delta_x < 0 ? -delta_x : delta_x);
		abs_dy = (delta_y < 0 ? -delta_y : delta_y);
	end
	always @(posedge clk)
		if (rst) begin
			gesture <= 2'b00;
			gesture_valid <= 1'b0;
		end
		else begin
			gesture_valid <= 1'b0;
			if (window_complete) begin
				if ((early_count > 5) || (late_count > 5)) begin
					if (abs_dy > abs_dx) begin
						if (delta_y > 0)
							gesture <= 2'b01;
						else
							gesture <= 2'b00;
					end
					else if (abs_dx > 0) begin
						if (delta_x > 0)
							gesture <= 2'b10;
						else
							gesture <= 2'b11;
					end
					gesture_valid <= 1'b1;
				end
			end
		end
	initial _sv2v_0 = 0;
endmodule
module uart_gesture_top (
	clk,
	uart_rx,
	uart_tx,
	led_heartbeat,
	led_gesture_valid
);
	parameter CLK_FREQ = 12000000;
	parameter BAUD_RATE = 115200;
	parameter CYCLES_PER_BIN = 600000;
	parameter NUM_BINS = 8;
	parameter GRID_SIZE = 16;
	parameter SENSOR_RES = 320;
	input wire clk;
	input wire uart_rx;
	output wire uart_tx;
	output wire led_heartbeat;
	output wire led_gesture_valid;
	localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
	reg [4:0] por_cnt = 5'd0;
	wire rst = ~&por_cnt;
	always @(posedge clk)
		if (!(&por_cnt))
			por_cnt <= por_cnt + 1'b1;
	wire [7:0] rx_data;
	wire rx_valid;
	reg [7:0] tx_data;
	reg tx_valid;
	wire tx_busy;
	uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx(
		.clk(clk),
		.rst(rst),
		.rx(uart_rx),
		.data(rx_data),
		.valid(rx_valid)
	);
	uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx(
		.clk(clk),
		.rst(rst),
		.data(tx_data),
		.valid(tx_valid),
		.tx(uart_tx),
		.busy(tx_busy)
	);
	wire [1:0] gesture;
	wire gesture_valid;
	wire [2:0] current_bin;
	wire event_ready;
	reg event_valid;
	reg [8:0] event_x;
	reg [8:0] event_y;
	reg event_polarity;
	dvs_gesture_accel #(
		.CLK_FREQ_HZ(CLK_FREQ),
		.CYCLES_PER_BIN(CYCLES_PER_BIN),
		.NUM_BINS(NUM_BINS),
		.GRID_SIZE(GRID_SIZE),
		.SENSOR_RES(SENSOR_RES)
	) u_accel(
		.clk(clk),
		.rst(rst),
		.event_valid(event_valid),
		.event_x(event_x),
		.event_y(event_y),
		.event_polarity(event_polarity),
		.event_ts(16'd0),
		.event_ready(event_ready),
		.gesture(gesture),
		.gesture_valid(gesture_valid),
		.current_bin(current_bin)
	);
	reg [23:0] heartbeat_cnt;
	always @(posedge clk) heartbeat_cnt <= (rst ? 24'd0 : heartbeat_cnt + 1'b1);
	assign led_heartbeat = ~heartbeat_cnt[22];
	reg [19:0] gesture_led_cnt;
	always @(posedge clk)
		if (rst)
			gesture_led_cnt <= 1'sb0;
		else if (gesture_valid)
			gesture_led_cnt <= {20 {1'b1}};
		else if (gesture_led_cnt > 0)
			gesture_led_cnt <= gesture_led_cnt - 1'b1;
	assign led_gesture_valid = ~(gesture_led_cnt > 0);
	reg [2:0] pkt_state;
	reg [8:0] pkt_x;
	reg [8:0] pkt_y;
	reg [1:0] tx_state;
	reg [1:0] pending_gesture;
	reg [2:0] pending_bin;
	always @(posedge clk)
		if (rst) begin
			pkt_state <= 3'd0;
			pkt_x <= 9'd0;
			pkt_y <= 9'd0;
			event_valid <= 1'b0;
			event_x <= 9'd0;
			event_y <= 9'd0;
			event_polarity <= 1'b0;
			tx_state <= 2'd0;
			tx_data <= 8'd0;
			tx_valid <= 1'b0;
			pending_gesture <= 2'd0;
			pending_bin <= 3'd0;
		end
		else begin
			event_valid <= 1'b0;
			tx_valid <= 1'b0;
			if (rx_valid)
				case (pkt_state)
					3'd0:
						if (rx_data == 8'hff) begin
							if (tx_state == 2'd0)
								tx_state <= 2'd1;
						end
						else if (rx_data == 8'hfe) begin
							if (tx_state == 2'd0) begin
								pending_bin <= current_bin;
								tx_state <= 2'd2;
							end
						end
						else begin
							pkt_x[8] <= rx_data[0];
							pkt_state <= 3'd1;
						end
					3'd1: begin
						pkt_x[7:0] <= rx_data;
						pkt_state <= 3'd2;
					end
					3'd2: begin
						pkt_y[8] <= rx_data[0];
						pkt_state <= 3'd3;
					end
					3'd3: begin
						pkt_y[7:0] <= rx_data;
						pkt_state <= 3'd4;
					end
					3'd4: begin
						if (event_ready) begin
							event_x <= pkt_x;
							event_y <= pkt_y;
							event_polarity <= rx_data[0];
							event_valid <= 1'b1;
						end
						pkt_state <= 3'd0;
					end
					default: pkt_state <= 3'd0;
				endcase
			if (gesture_valid && (tx_state == 2'd0)) begin
				pending_gesture <= gesture;
				tx_state <= 2'd3;
			end
			case (tx_state)
				2'd0:
					;
				2'd1:
					if (!tx_busy) begin
						tx_data <= 8'h55;
						tx_valid <= 1'b1;
						tx_state <= 2'd0;
					end
				2'd2:
					if (!tx_busy) begin
						tx_data <= {5'h16, pending_bin};
						tx_valid <= 1'b1;
						tx_state <= 2'd0;
					end
				2'd3:
					if (!tx_busy) begin
						tx_data <= {6'h28, pending_gesture};
						tx_valid <= 1'b1;
						tx_state <= 2'd0;
					end
				default: tx_state <= 2'd0;
			endcase
		end
endmodule