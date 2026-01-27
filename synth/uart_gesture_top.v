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
	ring_swap,
	current_bin
);
	reg _sv2v_0;
	parameter CLK_FREQ_HZ = 12000000;
	parameter WINDOW_US = 400;
	parameter NUM_BINS = 8;
	parameter GRID_SIZE = 16;
	parameter SENSOR_RES = 320;
	parameter COUNTER_BITS = 4;
	parameter FIFO_DEPTH = 8;
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
	output wire ring_swap;
	output wire [2:0] current_bin;
	localparam CYCLES_PER_BIN = (CLK_FREQ_HZ / 1000000) * (WINDOW_US / NUM_BINS);
	localparam BIN_TIMER_BITS = $clog2(CYCLES_PER_BIN + 1);
	localparam POOL_SIZE = SENSOR_RES / GRID_SIZE;
	localparam GRID_BITS = $clog2(GRID_SIZE);
	localparam BIN_BITS = $clog2(NUM_BINS);
	localparam FIFO_WIDTH = 19;
	localparam FIFO_PTR_BITS = $clog2(FIFO_DEPTH);
	reg [18:0] fifo_mem [0:FIFO_DEPTH - 1];
	reg [FIFO_PTR_BITS:0] fifo_wr_ptr;
	reg [FIFO_PTR_BITS:0] fifo_rd_ptr;
	wire fifo_empty;
	wire fifo_full;
	wire [8:0] fifo_out_x;
	wire [8:0] fifo_out_y;
	wire fifo_out_pol;
	wire fifo_rd_en;
	assign fifo_empty = fifo_wr_ptr == fifo_rd_ptr;
	assign fifo_full = (fifo_wr_ptr[FIFO_PTR_BITS] != fifo_rd_ptr[FIFO_PTR_BITS]) && (fifo_wr_ptr[FIFO_PTR_BITS - 1:0] == fifo_rd_ptr[FIFO_PTR_BITS - 1:0]);
	assign event_ready = ~fifo_full;
	always @(posedge clk)
		if (rst)
			fifo_wr_ptr <= 1'sb0;
		else if (event_valid && ~fifo_full) begin
			fifo_mem[fifo_wr_ptr[FIFO_PTR_BITS - 1:0]] <= {event_x, event_y, event_polarity};
			fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
		end
	wire [18:0] fifo_rd_data = fifo_mem[fifo_rd_ptr[FIFO_PTR_BITS - 1:0]];
	assign fifo_out_x = fifo_rd_data[18:10];
	assign fifo_out_y = fifo_rd_data[9:1];
	assign fifo_out_pol = fifo_rd_data[0];
	always @(posedge clk)
		if (rst)
			fifo_rd_ptr <= 1'sb0;
		else if (fifo_rd_en && ~fifo_empty)
			fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
	reg [BIN_TIMER_BITS - 1:0] bin_timer;
	reg [BIN_BITS - 1:0] active_bin;
	reg active_ring;
	reg bin_advance;
	reg [BIN_BITS - 1:0] bins_since_swap;
	reg ring_swap_r;
	assign current_bin = active_bin;
	assign ring_swap = ring_swap_r;
	always @(posedge clk)
		if (rst) begin
			bin_timer <= 1'sb0;
			active_bin <= 1'sb0;
			active_ring <= 1'b0;
			bin_advance <= 1'b0;
			bins_since_swap <= 1'sb0;
			ring_swap_r <= 1'b0;
		end
		else begin
			bin_advance <= 1'b0;
			ring_swap_r <= 1'b0;
			if (bin_timer >= (CYCLES_PER_BIN - 1)) begin
				bin_timer <= 1'sb0;
				bin_advance <= 1'b1;
				bins_since_swap <= bins_since_swap + 1'b1;
				if (bins_since_swap == (NUM_BINS - 1)) begin
					active_ring <= ~active_ring;
					bins_since_swap <= 1'sb0;
					ring_swap_r <= 1'b1;
					active_bin <= 1'sb0;
				end
				else
					active_bin <= active_bin + 1'b1;
			end
			else
				bin_timer <= bin_timer + 1'b1;
		end
	localparam VOXEL_ADDR_BITS = (((1 + BIN_BITS) + GRID_BITS) + GRID_BITS) + 1;
	localparam VOXEL_MEM_SIZE = 1 << VOXEL_ADDR_BITS;
	reg [COUNTER_BITS - 1:0] voxel_mem [0:VOXEL_MEM_SIZE - 1];
	integer init_i;
	initial for (init_i = 0; init_i < VOXEL_MEM_SIZE; init_i = init_i + 1)
		voxel_mem[init_i] = 1'sb0;
	function automatic [VOXEL_ADDR_BITS - 1:0] voxel_addr;
		input reg ring;
		input reg [BIN_BITS - 1:0] bin;
		input reg [GRID_BITS - 1:0] y;
		input reg [GRID_BITS - 1:0] x;
		input reg pol;
		voxel_addr = {ring, bin, y, x, pol};
	endfunction
	reg [2:0] evt_state;
	reg [8:0] raw_x;
	reg [8:0] raw_y;
	reg raw_pol;
	reg [GRID_BITS - 1:0] comp_x;
	reg [GRID_BITS - 1:0] comp_y;
	reg [VOXEL_ADDR_BITS - 1:0] pipe_addr;
	reg [COUNTER_BITS - 1:0] pipe_counter;
	reg [VOXEL_ADDR_BITS - 1:0] clear_addr;
	reg clearing_bin;
	reg [BIN_BITS - 1:0] next_bin;
	reg need_clear;
	wire [13:0] mult_x = fifo_out_x * 5'd13;
	wire [13:0] mult_y = fifo_out_y * 5'd13;
	wire [GRID_BITS - 1:0] compressed_x = mult_x[12:8];
	wire [GRID_BITS - 1:0] compressed_y = mult_y[12:8];
	wire [GRID_BITS - 1:0] clamped_x = (compressed_x >= GRID_SIZE ? GRID_SIZE[GRID_BITS - 1:0] - 1'b1 : compressed_x);
	wire [GRID_BITS - 1:0] clamped_y = (compressed_y >= GRID_SIZE ? GRID_SIZE[GRID_BITS - 1:0] - 1'b1 : compressed_y);
	always @(posedge clk)
		if (rst) begin
			need_clear <= 1'b0;
			next_bin <= 1'sb0;
		end
		else if (bin_advance && !ring_swap_r) begin
			need_clear <= 1'b1;
			next_bin <= active_bin + 1'b1;
		end
		else if ((evt_state == 3'd5) && !clearing_bin)
			need_clear <= 1'b0;
	assign fifo_rd_en = (((evt_state == 3'd1) && !fifo_empty) && !need_clear) && !clearing_bin;
	always @(posedge clk)
		if (rst) begin
			evt_state <= 3'd0;
			raw_x <= 1'sb0;
			raw_y <= 1'sb0;
			raw_pol <= 1'b0;
			comp_x <= 1'sb0;
			comp_y <= 1'sb0;
			pipe_addr <= 1'sb0;
			pipe_counter <= 1'sb0;
			clearing_bin <= 1'b0;
			clear_addr <= 1'sb0;
		end
		else
			case (evt_state)
				3'd0:
					if (need_clear) begin
						clearing_bin <= 1'b1;
						clear_addr <= voxel_addr(active_ring, next_bin, 4'd0, 4'd0, 1'b0);
						evt_state <= 3'd5;
					end
					else if (!fifo_empty)
						evt_state <= 3'd1;
				3'd1:
					if (!fifo_empty && !need_clear) begin
						raw_x <= fifo_out_x;
						raw_y <= fifo_out_y;
						raw_pol <= fifo_out_pol;
						evt_state <= 3'd2;
					end
					else
						evt_state <= 3'd0;
				3'd2: begin
					comp_x <= clamped_x;
					comp_y <= clamped_y;
					evt_state <= 3'd3;
				end
				3'd3: begin
					pipe_addr <= voxel_addr(active_ring, active_bin, comp_y, comp_x, raw_pol);
					pipe_counter <= voxel_mem[voxel_addr(active_ring, active_bin, comp_y, comp_x, raw_pol)];
					evt_state <= 3'd4;
				end
				3'd4: begin
					if (pipe_counter < {COUNTER_BITS {1'b1}})
						voxel_mem[pipe_addr] <= pipe_counter + 1'b1;
					evt_state <= 3'd0;
				end
				3'd5: begin
					voxel_mem[clear_addr] <= 1'sb0;
					if (clear_addr[GRID_BITS + GRID_BITS:0] == {(2 * GRID_BITS) + 1 {1'b1}}) begin
						clearing_bin <= 1'b0;
						evt_state <= 3'd0;
					end
					else
						clear_addr[GRID_BITS + GRID_BITS:0] <= clear_addr[GRID_BITS + GRID_BITS:0] + 1'b1;
				end
				default: evt_state <= 3'd0;
			endcase
	reg [2:0] cls_state;
	reg [GRID_BITS - 1:0] cls_x;
	reg [GRID_BITS - 1:0] cls_y;
	reg [BIN_BITS - 1:0] cls_bin;
	reg cls_pol;
	reg cls_ring;
	reg signed [23:0] early_sum_x;
	reg signed [23:0] early_sum_y;
	reg signed [23:0] late_sum_x;
	reg signed [23:0] late_sum_y;
	reg [19:0] early_count;
	reg [19:0] late_count;
	reg signed [23:0] delta_x;
	reg signed [23:0] delta_y;
	reg start_classification;
	reg [1:0] start_delay;
	always @(posedge clk)
		if (rst) begin
			start_classification <= 1'b0;
			start_delay <= 1'sb0;
		end
		else begin
			start_classification <= 1'b0;
			if (ring_swap_r)
				start_delay <= 2'd2;
			else if (start_delay > 0) begin
				start_delay <= start_delay - 1'b1;
				if (start_delay == 2'd1)
					start_classification <= 1'b1;
			end
		end
	reg [COUNTER_BITS - 1:0] cls_cnt;
	always @(posedge clk) cls_cnt <= voxel_mem[voxel_addr(cls_ring, cls_bin, cls_y, cls_x, cls_pol)];
	reg signed [4:0] cls_cx;
	reg signed [4:0] cls_cy;
	reg signed [8:0] cls_weighted_x;
	reg signed [8:0] cls_weighted_y;
	reg signed [23:0] cls_abs_dx;
	reg signed [23:0] cls_abs_dy;
	always @(*) begin
		if (_sv2v_0)
			;
		cls_cx = $signed({1'b0, cls_x}) - $signed(5'd8);
		cls_cy = $signed({1'b0, cls_y}) - $signed(5'd8);
		cls_weighted_x = cls_cx * $signed({1'b0, cls_cnt});
		cls_weighted_y = cls_cy * $signed({1'b0, cls_cnt});
		cls_abs_dx = (delta_x < 0 ? -delta_x : delta_x);
		cls_abs_dy = (delta_y < 0 ? -delta_y : delta_y);
	end
	always @(posedge clk)
		if (rst) begin
			cls_state <= 3'd0;
			cls_x <= 1'sb0;
			cls_y <= 1'sb0;
			cls_bin <= 1'sb0;
			cls_pol <= 1'b0;
			cls_ring <= 1'b0;
			early_sum_x <= 1'sb0;
			early_sum_y <= 1'sb0;
			late_sum_x <= 1'sb0;
			late_sum_y <= 1'sb0;
			early_count <= 1'sb0;
			late_count <= 1'sb0;
			delta_x <= 1'sb0;
			delta_y <= 1'sb0;
			gesture <= 2'b00;
			gesture_valid <= 1'b0;
		end
		else begin
			gesture_valid <= 1'b0;
			case (cls_state)
				3'd0:
					if (start_classification) begin
						cls_ring <= ~active_ring;
						cls_x <= 1'sb0;
						cls_y <= 1'sb0;
						cls_bin <= 1'sb0;
						cls_pol <= 1'b0;
						early_sum_x <= 1'sb0;
						early_sum_y <= 1'sb0;
						late_sum_x <= 1'sb0;
						late_sum_y <= 1'sb0;
						early_count <= 1'sb0;
						late_count <= 1'sb0;
						cls_state <= 3'd1;
					end
				3'd1: cls_state <= 3'd2;
				3'd2: begin
					if ((cls_cnt > 0) && (cls_pol == 1'b1)) begin
						if (cls_bin < (NUM_BINS / 2)) begin
							early_sum_x <= early_sum_x + {{15 {cls_weighted_x[8]}}, cls_weighted_x};
							early_sum_y <= early_sum_y + {{15 {cls_weighted_y[8]}}, cls_weighted_y};
							early_count <= early_count + {16'b0000000000000000, cls_cnt};
						end
						else begin
							late_sum_x <= late_sum_x + {{15 {cls_weighted_x[8]}}, cls_weighted_x};
							late_sum_y <= late_sum_y + {{15 {cls_weighted_y[8]}}, cls_weighted_y};
							late_count <= late_count + {16'b0000000000000000, cls_cnt};
						end
					end
					if (cls_pol == 1'b0)
						cls_pol <= 1'b1;
					else begin
						cls_pol <= 1'b0;
						if (cls_x == (GRID_SIZE - 1)) begin
							cls_x <= 1'sb0;
							if (cls_y == (GRID_SIZE - 1)) begin
								cls_y <= 1'sb0;
								if (cls_bin == (NUM_BINS - 1))
									cls_state <= 3'd3;
								else
									cls_bin <= cls_bin + 1'b1;
							end
							else
								cls_y <= cls_y + 1'b1;
						end
						else
							cls_x <= cls_x + 1'b1;
					end
				end
				3'd3: begin
					delta_x <= late_sum_x - early_sum_x;
					delta_y <= late_sum_y - early_sum_y;
					cls_state <= 3'd4;
				end
				3'd4: begin
					if ((early_count > 4) || (late_count > 4)) begin
						if (cls_abs_dy > cls_abs_dx) begin
							if (delta_y > 0)
								gesture <= 2'b00;
							else
								gesture <= 2'b01;
						end
						else if (cls_abs_dx > 0) begin
							if (delta_x > 0)
								gesture <= 2'b11;
							else
								gesture <= 2'b10;
						end
						gesture_valid <= 1'b1;
					end
					cls_state <= 3'd0;
				end
				default: cls_state <= 3'd0;
			endcase
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
	parameter WINDOW_US = 400;
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
	wire rst;
	always @(posedge clk)
		if (&por_cnt)
			por_cnt <= por_cnt;
		else
			por_cnt <= por_cnt + 1'b1;
	assign rst = ~&por_cnt;
	wire [7:0] rx_data;
	wire rx_valid;
	reg [7:0] tx_data;
	reg tx_valid;
	wire tx_busy;
	uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_rx(
		.clk(clk),
		.rst(rst),
		.rx(uart_rx),
		.data(rx_data),
		.valid(rx_valid)
	);
	uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_tx(
		.clk(clk),
		.rst(rst),
		.data(tx_data),
		.valid(tx_valid),
		.tx(uart_tx),
		.busy(tx_busy)
	);
	wire [1:0] gesture;
	wire gesture_valid;
	wire event_ready;
	wire ring_swap;
	wire [2:0] current_bin;
	reg event_valid;
	reg [8:0] event_x;
	reg [8:0] event_y;
	reg event_polarity;
	dvs_gesture_accel #(
		.CLK_FREQ_HZ(CLK_FREQ),
		.WINDOW_US(WINDOW_US),
		.NUM_BINS(NUM_BINS),
		.GRID_SIZE(GRID_SIZE),
		.SENSOR_RES(SENSOR_RES),
		.COUNTER_BITS(4),
		.FIFO_DEPTH(8)
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
		.ring_swap(ring_swap),
		.current_bin(current_bin)
	);
	reg [23:0] heartbeat_cnt;
	always @(posedge clk)
		if (rst)
			heartbeat_cnt <= 24'd0;
		else
			heartbeat_cnt <= heartbeat_cnt + 1'b1;
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
	reg [1:0] tx_state;
	reg [1:0] pending_gesture;
	reg [2:0] pending_bin;
	reg [2:0] pkt_state;
	reg [8:0] saved_x;
	reg [8:0] saved_y;
	reg saved_pol;
	always @(posedge clk)
		if (rst) begin
			pkt_state <= 3'd0;
			saved_x <= 9'd0;
			saved_y <= 9'd0;
			saved_pol <= 1'b0;
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
			if (rx_valid) begin
				if (pkt_state == 3'd0) begin
					if (rx_data == 8'hff) begin
						if (tx_state == 2'd0)
							tx_state <= 2'd1;
					end
					else if (rx_data == 8'hfe) begin
						if (tx_state == 2'd0) begin
							pending_bin <= current_bin;
							tx_state <= 2'd3;
						end
					end
					else begin
						saved_x[8] <= rx_data[0];
						pkt_state <= 3'd1;
					end
				end
				else
					case (pkt_state)
						3'd1: begin
							saved_x[7:0] <= rx_data;
							pkt_state <= 3'd2;
						end
						3'd2: begin
							saved_y[8] <= rx_data[0];
							pkt_state <= 3'd3;
						end
						3'd3: begin
							saved_y[7:0] <= rx_data;
							pkt_state <= 3'd4;
						end
						3'd4: begin
							if (event_ready) begin
								event_x <= saved_x;
								event_y <= saved_y;
								event_polarity <= rx_data[0];
								event_valid <= 1'b1;
							end
							pkt_state <= 3'd0;
						end
						default: pkt_state <= 3'd0;
					endcase
			end
			if (gesture_valid && (tx_state == 2'd0)) begin
				pending_gesture <= gesture;
				tx_state <= 2'd2;
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
						tx_data <= {6'h28, pending_gesture};
						tx_valid <= 1'b1;
						tx_state <= 2'd0;
					end
				2'd3:
					if (!tx_busy) begin
						tx_data <= {5'h16, pending_bin};
						tx_valid <= 1'b1;
						tx_state <= 2'd0;
					end
				default: tx_state <= 2'd0;
			endcase
		end
endmodule