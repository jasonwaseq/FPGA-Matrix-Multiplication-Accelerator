// UART Top Module for DVS Gesture Accelerator
// Single Sliding-Window Ring Buffer Architecture
//
// UART Protocol (115200 baud, 8N1):
//   RX: 5 bytes per event [X_HI, X_LO, Y_HI, Y_LO, POL]
//       X_HI[0] = X[8], X_LO = X[7:0] (9-bit coordinate, 0-319)
//       Y_HI[0] = Y[8], Y_LO = Y[7:0] (9-bit coordinate, 0-319)
//       POL[0] = polarity (1=ON, 0=OFF)
//   TX: Gesture detected: [0xA0 | gesture] (gesture: 0=UP,1=DOWN,2=LEFT,3=RIGHT)
//       Echo test: Send 0xFF -> Receive 0x55
//       Status query: Send 0xFE -> Receive [0xB0 | bin]

module uart_gesture_top #(
    parameter CLK_FREQ       = 12_000_000,
    parameter BAUD_RATE      = 115200,
    parameter CYCLES_PER_BIN = 600_000,    // 50ms at 12MHz (use 600 for sim)
    parameter NUM_BINS       = 8,
    parameter GRID_SIZE      = 16,
    parameter SENSOR_RES     = 320
)(
    input  logic clk,
    input  logic uart_rx,
    output logic uart_tx,
    output logic led_heartbeat,
    output logic led_gesture_valid
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // =========================================================================
    // Power-On Reset
    // =========================================================================
    
    reg [4:0] por_cnt = 5'd0;
    wire rst = ~&por_cnt;
    
    always @(posedge clk) begin
        if (!(&por_cnt))
            por_cnt <= por_cnt + 1'b1;
    end

    // =========================================================================
    // UART RX/TX
    // =========================================================================
    
    logic [7:0] rx_data;
    logic rx_valid;
    logic [7:0] tx_data;
    logic tx_valid;
    logic tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst(rst), .rx(uart_rx),
        .data(rx_data), .valid(rx_valid)
    );
    
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst(rst), .data(tx_data), .valid(tx_valid),
        .tx(uart_tx), .busy(tx_busy)
    );

    // =========================================================================
    // DVS Gesture Accelerator
    // =========================================================================
    
    logic [1:0] gesture;
    logic gesture_valid;
    logic [2:0] current_bin;
    logic event_ready;
    
    logic event_valid;
    logic [8:0] event_x, event_y;
    logic event_polarity;

    dvs_gesture_accel #(
        .CLK_FREQ_HZ(CLK_FREQ),
        .CYCLES_PER_BIN(CYCLES_PER_BIN),
        .NUM_BINS(NUM_BINS),
        .GRID_SIZE(GRID_SIZE),
        .SENSOR_RES(SENSOR_RES)
    ) u_accel (
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

    // =========================================================================
    // Heartbeat LED (~3Hz blink)
    // =========================================================================
    
    logic [23:0] heartbeat_cnt;
    always @(posedge clk)
        heartbeat_cnt <= rst ? 24'd0 : heartbeat_cnt + 1'b1;
    assign led_heartbeat = ~heartbeat_cnt[22];

    // Gesture indicator (pulse stretch for visibility)
    logic [19:0] gesture_led_cnt;
    always @(posedge clk) begin
        if (rst)
            gesture_led_cnt <= '0;
        else if (gesture_valid)
            gesture_led_cnt <= {20{1'b1}};
        else if (gesture_led_cnt > 0)
            gesture_led_cnt <= gesture_led_cnt - 1'b1;
    end
    assign led_gesture_valid = ~(gesture_led_cnt > 0);

    // =========================================================================
    // Packet RX State Machine (5-byte packets)
    // =========================================================================
    
    typedef enum logic [2:0] {
        PKT_X_HI, PKT_X_LO, PKT_Y_HI, PKT_Y_LO, PKT_POL
    } pkt_state_t;
    
    pkt_state_t pkt_state;
    logic [8:0] pkt_x, pkt_y;

    // =========================================================================
    // TX State Machine
    // =========================================================================
    
    typedef enum logic [1:0] {
        TX_IDLE, TX_ECHO, TX_STATUS, TX_GESTURE
    } tx_state_t;
    
    tx_state_t tx_state;
    logic [1:0] pending_gesture;
    logic [2:0] pending_bin;

    // =========================================================================
    // Main Control Logic
    // =========================================================================
    
    always @(posedge clk) begin
        if (rst) begin
            pkt_state       <= PKT_X_HI;
            pkt_x           <= 9'd0;
            pkt_y           <= 9'd0;
            event_valid     <= 1'b0;
            event_x         <= 9'd0;
            event_y         <= 9'd0;
            event_polarity  <= 1'b0;
            tx_state        <= TX_IDLE;
            tx_data         <= 8'd0;
            tx_valid        <= 1'b0;
            pending_gesture <= 2'd0;
            pending_bin     <= 3'd0;
        end else begin
            // Defaults
            event_valid <= 1'b0;
            tx_valid    <= 1'b0;
            
            // Handle RX bytes
            if (rx_valid) begin
                case (pkt_state)
                    PKT_X_HI: begin
                        if (rx_data == 8'hFF) begin
                            // Echo command
                            if (tx_state == TX_IDLE)
                                tx_state <= TX_ECHO;
                        end else if (rx_data == 8'hFE) begin
                            // Status query
                            if (tx_state == TX_IDLE) begin
                                pending_bin <= current_bin;
                                tx_state <= TX_STATUS;
                            end
                        end else begin
                            // Normal packet
                            pkt_x[8] <= rx_data[0];
                            pkt_state <= PKT_X_LO;
                        end
                    end
                    PKT_X_LO: begin
                        pkt_x[7:0] <= rx_data;
                        pkt_state <= PKT_Y_HI;
                    end
                    PKT_Y_HI: begin
                        pkt_y[8] <= rx_data[0];
                        pkt_state <= PKT_Y_LO;
                    end
                    PKT_Y_LO: begin
                        pkt_y[7:0] <= rx_data;
                        pkt_state <= PKT_POL;
                    end
                    PKT_POL: begin
                        // Complete packet - send to accelerator
                        if (event_ready) begin
                            event_x <= pkt_x;
                            event_y <= pkt_y;
                            event_polarity <= rx_data[0];
                            event_valid <= 1'b1;
                        end
                        pkt_state <= PKT_X_HI;
                    end
                    default: pkt_state <= PKT_X_HI;
                endcase
            end
            
            // Capture gesture for TX
            if (gesture_valid && tx_state == TX_IDLE) begin
                pending_gesture <= gesture;
                tx_state <= TX_GESTURE;
            end
            
            // TX state machine
            case (tx_state)
                TX_IDLE: ;  // Wait
                
                TX_ECHO: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'h55;
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_STATUS: begin
                    if (!tx_busy) begin
                        tx_data  <= {4'hB, 1'b0, pending_bin};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_GESTURE: begin
                    if (!tx_busy) begin
                        tx_data  <= {4'hA, 2'b00, pending_gesture};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
